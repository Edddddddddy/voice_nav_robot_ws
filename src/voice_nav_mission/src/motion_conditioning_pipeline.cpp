// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "voice_nav_mission/motion_conditioning_pipeline.hpp"

#include <rmw/qos_profiles.h>

#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <condition_variable>
#include <exception>
#include <future>
#include <iomanip>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <composition_interfaces/srv/list_nodes.hpp>
#include <composition_interfaces/srv/load_node.hpp>
#include <composition_interfaces/srv/unload_node.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "voice_nav_mission/motion_authority_ros_adapter.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;
using LoadNode = composition_interfaces::srv::LoadNode;
using UnloadNode = composition_interfaces::srv::UnloadNode;
using ListNodes = composition_interfaces::srv::ListNodes;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using CollisionState = nav2_msgs::msg::CollisionMonitorState;
using LaserScan = sensor_msgs::msg::LaserScan;
using Odometry = nav_msgs::msg::Odometry;
using Clock = rosgraph_msgs::msg::Clock;

constexpr char kCollisionMonitorNode[] = "/collision_monitor";
constexpr char kVelocitySmootherNode[] = "/velocity_smoother";
constexpr char kCollisionMonitorPlugin[] =
  "nav2_collision_monitor::CollisionMonitor";
constexpr char kVelocitySmootherPlugin[] =
  "nav2_velocity_smoother::VelocitySmoother";
constexpr char kCollisionMonitorFqn[] = "/collision_monitor";
constexpr char kVelocitySmootherFqn[] = "/velocity_smoother";

std::string random_identifier()
{
  std::array<std::uint8_t, 16> bytes{};
  std::random_device random;
  for (auto & byte : bytes) {
    byte = static_cast<std::uint8_t>(random());
  }
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : bytes) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

template<typename ValueT>
rcl_interfaces::msg::Parameter parameter(
  const std::string & name,
  const ValueT & value)
{
  return rclcpp::Parameter(name, value).to_parameter_msg();
}

MotionConditioningResult make_result(
  MotionConditioningState state,
  MotionConditioningFailure failure,
  bool ok,
  bool zero_proven,
  bool collision_stop,
  std::string lease_id,
  std::string candidate_topic,
  std::string detail)
{
  return MotionConditioningResult{
    ok,
    state,
    failure,
    zero_proven,
    collision_stop,
    std::move(lease_id),
    std::move(candidate_topic),
    std::move(detail)};
}

bool gate_snapshot_proves_zero(const GateSnapshot & snapshot)
{
  return !snapshot.gate_instance_id.empty() &&
         snapshot.endpoint_available &&
         snapshot.state == GateState::Inhibited &&
         snapshot.motion_inhibited &&
         snapshot.zero_selected &&
         snapshot.zero_published;
}

}  // namespace

class MotionConditioningPipeline::Impl
{
public:
  struct RenewCallbackGuard
  {
    explicit RenewCallbackGuard(Impl & owner)
    : owner_(owner), active_(owner.begin_renew_callback()) {}

    ~RenewCallbackGuard()
    {
      if (active_) {
        owner_.end_renew_callback();
      }
    }

    Impl & owner_;
    bool active_{false};
  };

  Impl(
    rclcpp::Node & node,
    std::shared_ptr<MotionAuthorityPort> authority,
    std::shared_ptr<MotionProducerPort> producer,
    MotionConditioningConfig config)
  : node_(node),
    authority_(std::move(authority)),
    producer_(std::move(producer)),
    config_(std::move(config)),
    request_id_generator_(config_.request_id_generator ?
      std::move(config_.request_id_generator) :
      std::function<std::string()>(random_identifier))
  {
    if (!authority_) {
      throw std::invalid_argument("MotionConditioningPipeline requires a Gate port");
    }
    component_callback_group_ = node_.create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    renew_callback_group_ = node_.create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    load_client_ = node_.create_client<LoadNode>(
      config_.container_fqn + "/_container/load_node",
      rmw_qos_profile_services_default,
      component_callback_group_);
    unload_client_ = node_.create_client<UnloadNode>(
      config_.container_fqn + "/_container/unload_node",
      rmw_qos_profile_services_default,
      component_callback_group_);
    list_nodes_client_ = node_.create_client<ListNodes>(
      config_.container_fqn + "/_container/list_nodes",
      rmw_qos_profile_services_default,
      component_callback_group_);
    controller_client_ = node_.create_client<ListControllers>(
      config_.controller_manager_service,
      rmw_qos_profile_services_default,
      component_callback_group_);
    rclcpp::SubscriptionOptions health_options;
    health_options.callback_group = component_callback_group_;
    scan_subscription_ = node_.create_subscription<LaserScan>(
      config_.scan_topic,
      rclcpp::SensorDataQoS(),
      [this](const LaserScan::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(health_mutex_);
        last_scan_receipt_ = std::chrono::steady_clock::now();
      },
      health_options);
    odom_subscription_ = node_.create_subscription<Odometry>(
      config_.odom_topic,
      rclcpp::SensorDataQoS(),
      [this](const Odometry::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(health_mutex_);
        last_odom_receipt_ = std::chrono::steady_clock::now();
      },
      health_options);
    clock_subscription_ = node_.create_subscription<Clock>(
      config_.clock_topic,
      rclcpp::ClockQoS(),
      [this](const Clock::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(health_mutex_);
        const auto stamp = static_cast<std::int64_t>(message->clock.sec) * 1000000000LL +
        static_cast<std::int64_t>(message->clock.nanosec);
        if (clock_seen_ && stamp > last_clock_stamp_) {
          clock_progressed_ = true;
        }
        clock_seen_ = true;
        last_clock_stamp_ = stamp;
        last_clock_receipt_ = std::chrono::steady_clock::now();
      },
      health_options);
    rclcpp::SubscriptionOptions collision_options;
    collision_options.callback_group = component_callback_group_;
    collision_subscription_ = node_.create_subscription<CollisionState>(
      config_.collision_state_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      [this](const CollisionState::ConstSharedPtr message) {
        if (
          message->action_type == CollisionState::STOP &&
          message->polygon_name == "stop_zone")
        {
          std::lock_guard<std::recursive_mutex> lock(mutex_);
          collision_stop_ = true;
          last_result_.collision_stop = true;
        }
      },
      collision_options);
  }

  ~Impl()
  {
    (void)stop();
    wait_for_renew_callbacks();
  }

  MotionConditioningResult prepare()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (state_ == MotionConditioningState::Prepared ||
      state_ == MotionConditioningState::Running)
    {
      return remember(make_result(
          state_,
          MotionConditioningFailure::SafetyFault,
          false,
          false,
          collision_stop_,
          lease_id_,
          candidate_topic_,
          "MotionConditioningPipeline already owns an active generation"));
    }

    if (!cleanup_generation("prepare handover cleanup")) {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "old conditioning generation could not be cleaned up"));
    }
    collision_stop_ = false;

    AuthorityResult gate_prepare;
    try {
      gate_prepare = authority_->prepare(make_operation());
    } catch (const std::exception & error) {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          std::string{"MotionGate PREPARE raised: "} + error.what()));
    } catch (...) {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "MotionGate PREPARE raised an unknown exception"));
    }
    lease_id_ = !gate_prepare.lease_id.empty() ?
      gate_prepare.lease_id : gate_prepare.snapshot.lease_id;
    candidate_topic_ = gate_prepare.snapshot.candidate_topic;
    if (!gate_prepare.applied || lease_id_.empty() ||
      gate_prepare.snapshot.state != GateState::Prepared ||
      !gate_prepare.snapshot.motion_inhibited ||
      !gate_prepare.snapshot.zero_selected ||
      !gate_prepare.zero_proven ||
      !gate_prepare.snapshot.zero_published ||
      gate_prepare.snapshot.gate_instance_id.empty())
    {
      return remember(fail_result(
        MotionConditioningFailure::SafetyFault,
        "MotionGate PREPARE did not prove an inhibited zero state: " +
          gate_prepare.detail));
    }
    prepare_open_deadline_ =
      std::chrono::steady_clock::now() + config_.prepare_open_deadline;

    if (!load_and_configure_components()) {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "Nav2 component load/configure failed"));
    }
    if (std::chrono::steady_clock::now() >= prepare_open_deadline_) {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "PREPARE to OPEN deadline expired during component setup"));
    }
    state_ = MotionConditioningState::Prepared;
    return remember(make_result(
        state_,
        MotionConditioningFailure::None,
        gate_prepare.zero_proven && gate_prepare.snapshot.zero_published,
        true,
        false,
        lease_id_,
        candidate_topic_,
        "conditioning generation prepared"));
  }

  MotionConditioningResult start()
  {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if (state_ != MotionConditioningState::Prepared) {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "MotionConditioningPipeline start requires PREPARED state"));
    }
    if (std::chrono::steady_clock::now() >= prepare_open_deadline_) {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "PREPARE to OPEN deadline expired"));
    }

    const auto handover_deadline = prepare_open_deadline_;
    AuthorityResult gate_open;
    lock.unlock();
    if (!controller_is_active(handover_deadline)) {
      lock.lock();
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "diff_drive_controller is not active before MotionGate OPEN"));
    }
    try {
      const auto open_operation = make_operation(lease_id_);
      gate_open = authority_->open(open_operation);
      if (gate_open.snapshot.gate_instance_id != open_operation.gate_instance_id) {
        gate_open.applied = false;
        gate_open.detail = "MotionGate OPEN returned a stale gate identity";
      }
    } catch (const std::exception & error) {
      lock.lock();
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          std::string{"MotionGate OPEN raised: "} + error.what()));
    } catch (...) {
      lock.lock();
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "MotionGate OPEN raised an unknown exception"));
    }
    if (!gate_open.applied || !gate_open.snapshot.authority_live ||
      gate_open.snapshot.motion_inhibited ||
      !gate_open.snapshot.writer_bound ||
      gate_open.snapshot.candidate_topic != candidate_topic_)
    {
      lock.lock();
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "MotionGate OPEN did not bind the current candidate writer: " +
          gate_open.detail));
    }

    lock.lock();
    enable_renew_callbacks();
    renew_timer_ = node_.create_wall_timer(
      config_.renew_period,
      [this]() {on_renew();},
      renew_callback_group_);
    activation_in_progress_.store(true);
    activation_failed_.store(false);
    {
      std::lock_guard<std::mutex> activation_lock(activation_mutex_);
      activation_failure_detail_.clear();
    }
    lock.unlock();

    if (!renew_for_activation()) {
      lock.lock();
      activation_in_progress_.store(false);
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          activation_failure_detail()));
    }
    if (!change_state(
        kCollisionMonitorFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        handover_deadline) ||
      activation_failed_.load() ||
      std::chrono::steady_clock::now() >= handover_deadline)
    {
      lock.lock();
      activation_in_progress_.store(false);
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "Nav2 component activation failed"));
    }
    if (!change_state(
        kVelocitySmootherFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        handover_deadline) ||
      activation_failed_.load() ||
      std::chrono::steady_clock::now() >= handover_deadline)
    {
      lock.lock();
      activation_in_progress_.store(false);
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "Nav2 component activation failed"));
    }
    if (!renew_for_activation() ||
      !runtime_graph_is_healthy(handover_deadline) ||
      activation_failed_.load())
    {
      lock.lock();
      activation_in_progress_.store(false);
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "conditioning authority or dependency health failed during activation"));
    }
    lock.lock();
    if (activation_failed_.load() ||
      std::chrono::steady_clock::now() >= handover_deadline)
    {
      activation_in_progress_.store(false);
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "PREPARE to OPEN deadline expired during activation"));
    }
    if (!producer_ || !producer_->start(config_.raw_topic)) {
      activation_in_progress_.store(false);
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "conditioning producer could not start"));
    }
    activation_in_progress_.store(false);
    state_ = MotionConditioningState::Running;
    return remember(make_result(
        state_,
        MotionConditioningFailure::None,
        true,
        false,
        false,
        lease_id_,
        candidate_topic_,
        "conditioning generation running"));
  }

  MotionConditioningResult stop()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (state_ == MotionConditioningState::Stopped) {
      return remember(make_result(
          state_, MotionConditioningFailure::None, true, true, collision_stop_,
                 {}, {}, "conditioning pipeline already stopped"));
    }
    if (producer_) {
      producer_->stop();
    }
    disable_renew_callbacks();
    const bool zero_proven = inhibit_gate();
    const bool components_clean = cleanup_components();
    if (!zero_proven || !components_clean) {
      state_ = MotionConditioningState::Failed;
      return remember(make_result(
          state_, MotionConditioningFailure::SafetyFault, false, zero_proven,
          collision_stop_, lease_id_, candidate_topic_,
          "conditioning stop could not prove zero and cleanup"));
    }
    reset_generation();
    state_ = MotionConditioningState::Stopped;
    return remember(make_result(
        state_, MotionConditioningFailure::None, true, true, collision_stop_,
               {}, {}, "conditioning generation stopped"));
  }

  MotionConditioningResult fail(
    MotionConditioningFailure failure,
    std::string detail)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (producer_) {
      producer_->stop();
    }
    disable_renew_callbacks();
    const bool zero_proven = inhibit_gate();
    const bool components_clean = cleanup_components();
    state_ = MotionConditioningState::Failed;
    if (failure == MotionConditioningFailure::None) {
      failure = MotionConditioningFailure::InternalError;
    }
    if (!components_clean) {
      detail += "; component cleanup failed";
    }
    return remember(make_result(
        state_, failure, false, zero_proven, collision_stop_, lease_id_,
        candidate_topic_, std::move(detail)));
  }

  MotionConditioningState state() const noexcept
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return state_;
  }

  MotionConditioningResult last_result() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return last_result_;
  }

private:
  static std::chrono::milliseconds remaining_until(
    std::chrono::steady_clock::time_point deadline)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return 0ms;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
  }

  struct Component
  {
    std::uint64_t unique_id{0U};
    std::string node_fqn;
  };

  struct PendingLoad
  {
    std::string expected_fqn;
    std::shared_future<std::shared_ptr<LoadNode::Response>> future;
  };

  [[nodiscard]] MotionConditioningResult & remember(
    MotionConditioningResult result)
  {
    last_result_ = std::move(result);
    return last_result_;
  }

  [[nodiscard]] MotionConditioningResult fail_result(
    MotionConditioningFailure failure,
    std::string detail)
  {
    const bool zero_proven = inhibit_gate();
    const bool clean = cleanup_components();
    state_ = MotionConditioningState::Failed;
    if (!clean) {
      detail += "; component cleanup failed";
    }
    return make_result(
      state_, failure, false, zero_proven, collision_stop_, lease_id_,
      candidate_topic_, std::move(detail));
  }

  [[nodiscard]] bool begin_renew_callback()
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (!renew_callbacks_enabled_) {
      return false;
    }
    ++active_renew_callbacks_;
    return true;
  }

  void end_renew_callback()
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (--active_renew_callbacks_ == 0U) {
      callback_cv_.notify_all();
    }
  }

  void enable_renew_callbacks()
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    renew_callbacks_enabled_ = true;
  }

  void disable_renew_callbacks()
  {
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      renew_callbacks_enabled_ = false;
    }
    renew_timer_.reset();
  }

  void wait_for_renew_callbacks()
  {
    std::unique_lock<std::mutex> lock(callback_mutex_);
    callback_cv_.wait(lock, [this]() {return active_renew_callbacks_ == 0U;});
  }

  [[nodiscard]] AuthorityOperation make_operation(
    const std::string & lease = {})
  {
    const auto snapshot = authority_->snapshot();
    const auto request_id = request_id_generator_();
    if (request_id.size() != 32U) {
      throw std::runtime_error("conditioning request ID must be 32 hex characters");
    }
    return AuthorityOperation{
      request_id,
      snapshot.gate_instance_id,
      snapshot.control_seq,
      lease};
  }

  [[nodiscard]] bool call_load(
    const std::string & package_name,
    const std::string & plugin_name,
    const std::string & node_name,
    const std::vector<rcl_interfaces::msg::Parameter> & parameters,
    const std::vector<std::string> & remaps,
    Component & component,
    std::chrono::steady_clock::time_point overall_deadline)
  {
    const auto rpc_deadline = std::min(
      overall_deadline,
      std::chrono::steady_clock::now() + config_.component_rpc_timeout);
    if (!load_client_->wait_for_service(remaining_until(rpc_deadline)) ||
      std::chrono::steady_clock::now() >= rpc_deadline)
    {
      return false;
    }
    auto request = std::make_shared<LoadNode::Request>();
    request->package_name = package_name;
    request->plugin_name = plugin_name;
    request->node_name = node_name;
    request->node_namespace = "/";
    request->parameters = parameters;
    request->remap_rules = remaps;
    request->extra_arguments.push_back(
      parameter("use_intra_process_comms", false));
    auto future = load_client_->async_send_request(request);
    pending_loads_.push_back(PendingLoad{"/" + node_name, std::move(future)});
    auto & pending_future = pending_loads_.back().future;
    if (pending_future.wait_for(remaining_until(rpc_deadline)) !=
      std::future_status::ready)
    {
      return false;
    }
    const auto response = pending_future.get();
    pending_loads_.pop_back();
    if (!response->success) {
      return false;
    }
    component.unique_id = response->unique_id;
    component.node_fqn = response->full_node_name;
    if (component.unique_id != 0U) {
      components_loaded_ = true;
    }
    return std::chrono::steady_clock::now() < rpc_deadline &&
           component.unique_id != 0U && component.node_fqn == "/" + node_name;
  }

  [[nodiscard]] bool call_unload(
    std::uint64_t unique_id,
    std::chrono::steady_clock::time_point overall_deadline)
  {
    const auto rpc_deadline = std::min(
      overall_deadline,
      std::chrono::steady_clock::now() + config_.component_rpc_timeout);
    if (!unload_client_->wait_for_service(remaining_until(rpc_deadline)) ||
      std::chrono::steady_clock::now() >= rpc_deadline)
    {
      return false;
    }
    auto request = std::make_shared<UnloadNode::Request>();
    request->unique_id = unique_id;
    auto future = unload_client_->async_send_request(request);
    if (future.wait_for(remaining_until(rpc_deadline)) !=
      std::future_status::ready)
    {
      return false;
    }
    if (std::chrono::steady_clock::now() >= rpc_deadline) {
      return false;
    }
    return future.get()->success;
  }

  [[nodiscard]] bool change_state(
    const std::string & node_fqn,
    std::uint8_t transition_id,
    std::chrono::steady_clock::time_point overall_deadline =
    std::chrono::steady_clock::time_point::max())
  {
    auto client = node_.create_client<ChangeState>(
      node_fqn + "/change_state",
      rmw_qos_profile_services_default,
      component_callback_group_);
    const auto rpc_deadline = std::min(
      overall_deadline,
      std::chrono::steady_clock::now() + config_.component_rpc_timeout);
    if (!client->wait_for_service(remaining_until(rpc_deadline)) ||
      std::chrono::steady_clock::now() >= rpc_deadline)
    {
      return false;
    }
    auto request = std::make_shared<ChangeState::Request>();
    request->transition.id = transition_id;
    auto future = client->async_send_request(request);
    if (future.wait_for(remaining_until(rpc_deadline)) !=
      std::future_status::ready)
    {
      return false;
    }
    if (std::chrono::steady_clock::now() >= rpc_deadline) {
      return false;
    }
    return future.get()->success;
  }

  [[nodiscard]] std::uint8_t component_state(
    const std::string & node_fqn,
    std::chrono::steady_clock::time_point overall_deadline =
    std::chrono::steady_clock::time_point::max())
  {
    auto client = node_.create_client<GetState>(
      node_fqn + "/get_state",
      rmw_qos_profile_services_default,
      component_callback_group_);
    const auto rpc_deadline = std::min(
      overall_deadline,
      std::chrono::steady_clock::now() + config_.component_rpc_timeout);
    if (!client->wait_for_service(remaining_until(rpc_deadline)) ||
      std::chrono::steady_clock::now() >= rpc_deadline)
    {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    auto request = std::make_shared<GetState::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(remaining_until(rpc_deadline)) !=
      std::future_status::ready)
    {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    if (std::chrono::steady_clock::now() >= rpc_deadline) {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    return future.get()->current_state.id;
  }

  [[nodiscard]] bool load_and_configure_components()
  {
    const auto common = std::vector<rcl_interfaces::msg::Parameter>{
      parameter("use_sim_time", true),
      parameter("enable_stamped_cmd_vel", true),
    };
    auto collision_parameters = common;
    collision_parameters.insert(
      collision_parameters.end(),
      {
        parameter("base_frame_id", std::string{"base_footprint"}),
        parameter("odom_frame_id", std::string{"odom"}),
        parameter("cmd_vel_in_topic", config_.smoothed_topic),
        parameter("cmd_vel_out_topic", candidate_topic_),
        parameter("state_topic", config_.collision_state_topic),
        parameter("transform_tolerance", 0.10),
        parameter("source_timeout", 0.20),
        parameter("stop_pub_timeout", 0.50),
        parameter("polygons", std::vector<std::string>{"stop_zone"}),
        parameter("stop_zone.type", std::string{"circle"}),
        parameter("stop_zone.radius", 0.24),
        parameter("stop_zone.action_type", std::string{"stop"}),
        parameter("stop_zone.min_points", 3),
        parameter("stop_zone.visualize", false),
        parameter("stop_zone.enabled", true),
        parameter("observation_sources", std::vector<std::string>{"scan"}),
        parameter("scan.type", std::string{"scan"}),
        parameter("scan.topic", std::string{"/scan"}),
        parameter("scan.enabled", true),
      });
    if (!call_load(
        "nav2_collision_monitor",
        kCollisionMonitorPlugin,
        "collision_monitor",
        collision_parameters,
        {},
        collision_component_,
        prepare_open_deadline_))
    {
      return false;
    }
    components_loaded_ = true;

    auto smoother_parameters = common;
    smoother_parameters.insert(
      smoother_parameters.end(),
      {
        parameter("smoothing_frequency", 50.0),
        parameter("feedback", std::string{"CLOSED_LOOP"}),
        parameter("scale_velocities", false),
        parameter("odom_topic", std::string{"/odom"}),
        parameter("velocity_timeout", 0.20),
        parameter("max_velocity", std::vector<double>{0.40, 0.0, 1.20}),
        parameter("min_velocity", std::vector<double>{-0.20, 0.0, -1.20}),
        parameter("max_accel", std::vector<double>{0.50, 0.0, 1.50}),
        parameter("max_decel", std::vector<double>{-0.70, 0.0, -2.00}),
        parameter("deadband_velocity", std::vector<double>{0.0, 0.0, 0.0}),
        parameter("stamp_smoothed_velocity_with_smoothing_time", true),
      });
    if (!call_load(
        "nav2_velocity_smoother",
        kVelocitySmootherPlugin,
        "velocity_smoother",
        smoother_parameters,
      {
        "cmd_vel:=" + config_.raw_topic,
        "cmd_vel_smoothed:=" + config_.smoothed_topic,
        },
        smoother_component_,
        prepare_open_deadline_))
    {
      return false;
    }
    if (!change_state(
      kCollisionMonitorFqn,
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
      prepare_open_deadline_) ||
      !change_state(
        kVelocitySmootherFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
        prepare_open_deadline_))
    {
      return false;
    }
    return component_state(kCollisionMonitorFqn, prepare_open_deadline_) ==
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
           component_state(kVelocitySmootherFqn, prepare_open_deadline_) ==
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  }

  [[nodiscard]] bool wait_for_writer_to_disappear(
    const std::string & topic)
  {
    if (topic.empty()) {
      return true;
    }
    const auto deadline =
      std::chrono::steady_clock::now() + config_.writer_graph_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (node_.get_publishers_info_by_topic(topic).empty()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return node_.get_publishers_info_by_topic(topic).empty();
  }

  [[nodiscard]] bool list_nodes(
    std::vector<Component> & nodes,
    std::chrono::steady_clock::time_point overall_deadline)
  {
    const auto rpc_deadline = std::min(
      overall_deadline,
      std::chrono::steady_clock::now() + config_.component_rpc_timeout);
    if (!list_nodes_client_->wait_for_service(remaining_until(rpc_deadline))) {
      return false;
    }
    auto future = list_nodes_client_->async_send_request(
      std::make_shared<ListNodes::Request>());
    if (future.wait_for(remaining_until(rpc_deadline)) !=
      std::future_status::ready)
    {
      return false;
    }
    const auto response = future.get();
    nodes.clear();
    const auto count = std::min(
      response->full_node_names.size(), response->unique_ids.size());
    for (std::size_t index = 0U; index < count; ++index) {
      nodes.push_back(Component{
          response->unique_ids[index], response->full_node_names[index]});
    }
    return std::chrono::steady_clock::now() <= overall_deadline;
  }

  [[nodiscard]] bool reconcile_pending_loads(
    std::chrono::steady_clock::time_point overall_deadline)
  {
    for (auto iterator = pending_loads_.begin();
      iterator != pending_loads_.end(); )
    {
      if (iterator->future.wait_for(remaining_until(overall_deadline)) !=
        std::future_status::ready)
      {
        return false;
      }
      const auto response = iterator->future.get();
      if (response->success && response->unique_id != 0U) {
        residual_components_.push_back(Component{
            response->unique_id, response->full_node_name});
        components_loaded_ = true;
      }
      iterator = pending_loads_.erase(iterator);
    }
    return true;
  }

  [[nodiscard]] bool reconcile_residual_nodes(
    std::chrono::steady_clock::time_point overall_deadline)
  {
    std::size_t consecutive_absent = 0U;
    while (std::chrono::steady_clock::now() < overall_deadline) {
      for (auto iterator = residual_components_.begin();
        iterator != residual_components_.end(); )
      {
        if (call_unload(iterator->unique_id, overall_deadline)) {
          iterator = residual_components_.erase(iterator);
        } else {
          ++iterator;
        }
      }
      std::vector<Component> listed;
      if (!list_nodes(listed, overall_deadline)) {
        return false;
      }
      bool target_present = false;
      for (const auto & node : listed) {
        if (node.node_fqn == kCollisionMonitorFqn ||
          node.node_fqn == kVelocitySmootherFqn)
        {
          target_present = true;
          (void)call_unload(node.unique_id, overall_deadline);
        }
      }
      if (residual_components_.empty() && !target_present) {
        ++consecutive_absent;
        if (consecutive_absent >= 2U) {
          return true;
        }
      } else {
        consecutive_absent = 0U;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  [[nodiscard]] bool cleanup_components()
  {
    if (!components_loaded_ && pending_loads_.empty() &&
      residual_components_.empty())
    {
      return true;
    }
    const auto cleanup_deadline = std::chrono::steady_clock::now() +
      std::max(config_.component_rpc_timeout, config_.writer_graph_timeout);
    bool success = true;
    const auto smoother_state = component_state(
      kVelocitySmootherFqn, cleanup_deadline);
    if (smoother_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      success = change_state(
        kVelocitySmootherFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN,
        cleanup_deadline) && success;
    }
    if (smoother_state ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      success = change_state(
        kVelocitySmootherFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN,
        cleanup_deadline) && success;
    }
    const auto collision_state = component_state(
      kCollisionMonitorFqn, cleanup_deadline);
    if (collision_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      success = change_state(
        kCollisionMonitorFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN,
        cleanup_deadline) && success;
    }
    if (collision_state ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      success = change_state(
        kCollisionMonitorFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN,
        cleanup_deadline) && success;
    }
    if (smoother_component_.unique_id != 0U) {
      if (call_unload(smoother_component_.unique_id, cleanup_deadline)) {
        smoother_component_ = {};
      } else {
        residual_components_.push_back(smoother_component_);
        success = false;
      }
    }
    if (collision_component_.unique_id != 0U) {
      if (call_unload(collision_component_.unique_id, cleanup_deadline)) {
        collision_component_ = {};
      } else {
        residual_components_.push_back(collision_component_);
        success = false;
      }
    }
    success = reconcile_pending_loads(cleanup_deadline) && success;
    success = reconcile_residual_nodes(cleanup_deadline) && success;
    success = wait_for_writer_to_disappear(candidate_topic_) && success;
    if (success && pending_loads_.empty() && residual_components_.empty()) {
      components_loaded_ = false;
      collision_component_ = {};
      smoother_component_ = {};
    } else {
      success = false;
    }
    if (!success) {
      cleanup_blocked_ = true;
    }
    return success;
  }

  [[nodiscard]] bool inhibit_gate()
  {
    if (lease_id_.empty()) {
      return gate_snapshot_proves_zero(authority_->snapshot());
    }
    try {
      std::lock_guard<std::mutex> authority_lock(authority_call_mutex_);
      const auto operation = make_operation(lease_id_);
      const auto result = authority_->inhibit(operation);
      if (!result.applied &&
        result.snapshot.gate_instance_id == operation.gate_instance_id &&
        gate_snapshot_proves_zero(result.snapshot))
      {
        return true;
      }
      return result.applied && result.zero_proven &&
             result.snapshot.gate_instance_id == operation.gate_instance_id &&
             gate_snapshot_proves_zero(result.snapshot);
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool cleanup_generation(const std::string &)
  {
    if (cleanup_blocked_) {
      return false;
    }
    if (producer_) {
      producer_->stop();
    }
    disable_renew_callbacks();
    const bool zero_proven = inhibit_gate();
    const bool clean = cleanup_components();
    if (!zero_proven || !clean) {
      cleanup_blocked_ = true;
      return false;
    }
    if (!candidate_topic_.empty()) {
      const bool writer_gone = wait_for_writer_to_disappear(candidate_topic_);
      cleanup_blocked_ = !writer_gone;
      return writer_gone;
    }
    return true;
  }

  void reset_generation()
  {
    components_loaded_ = false;
    collision_component_ = {};
    smoother_component_ = {};
    pending_loads_.clear();
    residual_components_.clear();
    lease_id_.clear();
    candidate_topic_.clear();
    prepare_open_deadline_ = {};
  }

  void set_activation_failure(std::string detail)
  {
    {
      std::lock_guard<std::mutex> lock(activation_mutex_);
      activation_failure_detail_ = std::move(detail);
    }
    activation_failed_.store(true);
  }

  [[nodiscard]] std::string activation_failure_detail() const
  {
    std::lock_guard<std::mutex> lock(activation_mutex_);
    return activation_failure_detail_;
  }

  [[nodiscard]] bool renew_for_activation()
  {
    if (activation_failed_.load()) {
      return false;
    }
    try {
      if (!controller_is_active(prepare_open_deadline_)) {
        set_activation_failure(
          "diff_drive_controller is not active during activation");
        (void)inhibit_gate();
        return false;
      }
      AuthorityResult result;
      {
        std::lock_guard<std::mutex> authority_lock(authority_call_mutex_);
        result = authority_->renew(make_operation(lease_id_));
      }
      if (!result.applied || !result.snapshot.authority_live) {
        set_activation_failure("MotionGate RENEW failed during activation");
        (void)inhibit_gate();
        return false;
      }
      return true;
    } catch (const std::exception & error) {
      set_activation_failure(
        std::string{"MotionGate RENEW raised during activation: "} + error.what());
      (void)inhibit_gate();
    } catch (...) {
      set_activation_failure("MotionGate RENEW raised during activation");
      (void)inhibit_gate();
    }
    return false;
  }

  void on_renew()
  {
    RenewCallbackGuard callback_guard(*this);
    if (!callback_guard.active_) {
      return;
    }
    if (activation_in_progress_.load()) {
      (void)renew_for_activation();
      return;
    }
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (state_ != MotionConditioningState::Running) {
      return;
    }
    if (collision_stop_) {
      (void)fail(
        MotionConditioningFailure::ExecutionFailed,
        "Collision Monitor reported STOP for stop_zone");
      return;
    }
    if (!runtime_graph_is_healthy(
        std::chrono::steady_clock::now() + config_.health_rpc_timeout))
    {
      (void)fail(
        MotionConditioningFailure::SafetyFault,
        "conditioning component or dependency graph is unhealthy");
      return;
    }
    try {
      AuthorityResult result;
      {
        std::lock_guard<std::mutex> authority_lock(authority_call_mutex_);
        result = authority_->renew(make_operation(lease_id_));
      }
      if (!result.applied || !result.snapshot.authority_live) {
        (void)fail(
          MotionConditioningFailure::SafetyFault,
          "MotionGate RENEW failed closed");
      }
    } catch (const std::exception & error) {
      (void)fail(
        MotionConditioningFailure::SafetyFault,
        std::string{"MotionGate RENEW raised: "} + error.what());
    } catch (...) {
      (void)fail(
        MotionConditioningFailure::SafetyFault,
        "MotionGate RENEW raised an unknown exception");
    }
  }

  [[nodiscard]] bool controller_is_active(
    std::chrono::steady_clock::time_point overall_deadline)
  {
    const auto rpc_deadline = std::min(
      overall_deadline,
      std::chrono::steady_clock::now() + config_.health_rpc_timeout);
    if (!controller_client_->wait_for_service(remaining_until(rpc_deadline)) ||
      std::chrono::steady_clock::now() >= rpc_deadline)
    {
      return false;
    }
    auto future = controller_client_->async_send_request(
      std::make_shared<ListControllers::Request>());
    if (future.wait_for(remaining_until(rpc_deadline)) !=
      std::future_status::ready || std::chrono::steady_clock::now() >= rpc_deadline)
    {
      return false;
    }
    const auto response = future.get();
    return std::any_of(
      response->controller.cbegin(), response->controller.cend(),
      [this](const auto & controller) {
        return controller.name == config_.controller_name &&
               controller.state == "active";
      });
  }

  [[nodiscard]] bool runtime_graph_is_healthy(
    std::chrono::steady_clock::time_point overall_deadline)
  {
    if (!components_loaded_ ||
      component_state(kCollisionMonitorFqn, overall_deadline) !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
      component_state(kVelocitySmootherFqn, overall_deadline) !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      return false;
    }
    if (std::chrono::steady_clock::now() >= overall_deadline ||
      !node_.get_clock()->ros_time_is_active() ||
      node_.get_clock()->now().nanoseconds() <= 0)
    {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(health_mutex_);
      const auto now = std::chrono::steady_clock::now();
      const auto max_age = config_.dependency_liveness_timeout;
      if (!clock_seen_ || !clock_progressed_ ||
        now - last_scan_receipt_ > max_age ||
        now - last_odom_receipt_ > max_age ||
        now - last_clock_receipt_ > max_age)
      {
        return false;
      }
      clock_progressed_ = false;
    }
    const auto candidate_publishers =
      node_.get_publishers_info_by_topic(candidate_topic_);
    const bool candidate_writer = std::any_of(
      candidate_publishers.cbegin(), candidate_publishers.cend(),
      [](const rclcpp::TopicEndpointInfo & endpoint) {
        return endpoint.node_name() == "collision_monitor" &&
               endpoint.topic_type() == "geometry_msgs/msg/TwistStamped";
      });
    return candidate_writer && controller_is_active(overall_deadline);
  }

  rclcpp::Node & node_;
  std::shared_ptr<MotionAuthorityPort> authority_;
  std::shared_ptr<MotionProducerPort> producer_;
  MotionConditioningConfig config_;
  std::function<std::string()> request_id_generator_;
  mutable std::recursive_mutex mutex_;
  std::mutex authority_call_mutex_;
  MotionConditioningState state_{MotionConditioningState::Stopped};
  MotionConditioningResult last_result_{};
  bool collision_stop_{false};
  bool components_loaded_{false};
  bool cleanup_blocked_{false};
  Component collision_component_;
  Component smoother_component_;
  std::vector<PendingLoad> pending_loads_;
  std::vector<Component> residual_components_;
  std::string lease_id_;
  std::string candidate_topic_;
  std::chrono::steady_clock::time_point prepare_open_deadline_{};
  std::atomic<bool> activation_in_progress_{false};
  std::atomic<bool> activation_failed_{false};
  mutable std::mutex activation_mutex_;
  std::string activation_failure_detail_;
  std::mutex callback_mutex_;
  std::condition_variable callback_cv_;
  std::size_t active_renew_callbacks_{0U};
  bool renew_callbacks_enabled_{true};
  rclcpp::CallbackGroup::SharedPtr component_callback_group_;
  rclcpp::CallbackGroup::SharedPtr renew_callback_group_;
  rclcpp::Client<LoadNode>::SharedPtr load_client_;
  rclcpp::Client<UnloadNode>::SharedPtr unload_client_;
  rclcpp::Client<ListNodes>::SharedPtr list_nodes_client_;
  rclcpp::Client<ListControllers>::SharedPtr controller_client_;
  rclcpp::Subscription<CollisionState>::SharedPtr collision_subscription_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<Clock>::SharedPtr clock_subscription_;
  std::mutex health_mutex_;
  std::chrono::steady_clock::time_point last_scan_receipt_{};
  std::chrono::steady_clock::time_point last_odom_receipt_{};
  std::chrono::steady_clock::time_point last_clock_receipt_{};
  std::int64_t last_clock_stamp_{0};
  bool clock_seen_{false};
  bool clock_progressed_{false};
  rclcpp::TimerBase::SharedPtr renew_timer_;
};

MotionConditioningPipeline::MotionConditioningPipeline(
  rclcpp::Node & node,
  std::shared_ptr<MotionAuthorityPort> authority,
  std::shared_ptr<MotionProducerPort> producer,
  MotionConditioningConfig config)
: impl_(std::make_unique<Impl>(
    node, std::move(authority), std::move(producer), std::move(config)))
{
}

MotionConditioningPipeline::~MotionConditioningPipeline() = default;

MotionConditioningResult MotionConditioningPipeline::prepare()
{
  return impl_->prepare();
}

MotionConditioningResult MotionConditioningPipeline::start()
{
  return impl_->start();
}

MotionConditioningResult MotionConditioningPipeline::stop()
{
  return impl_->stop();
}

MotionConditioningResult MotionConditioningPipeline::fail(
  MotionConditioningFailure failure,
  std::string detail)
{
  return impl_->fail(failure, std::move(detail));
}

MotionConditioningState MotionConditioningPipeline::state() const noexcept
{
  return impl_->state();
}

MotionConditioningResult MotionConditioningPipeline::last_result() const
{
  return impl_->last_result();
}

}  // namespace voice_nav_mission
