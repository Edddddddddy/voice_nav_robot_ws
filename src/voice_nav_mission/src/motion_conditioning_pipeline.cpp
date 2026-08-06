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

#include <composition_interfaces/srv/load_node.hpp>
#include <composition_interfaces/srv/unload_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <rmw/qos_profiles.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
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

#include "voice_nav_mission/motion_authority_ros_adapter.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;
using LoadNode = composition_interfaces::srv::LoadNode;
using UnloadNode = composition_interfaces::srv::UnloadNode;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using CollisionState = nav2_msgs::msg::CollisionMonitorState;

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

}  // namespace

class MotionConditioningPipeline::Impl
{
public:
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
    if (!gate_prepare.applied || gate_prepare.lease_id.empty() ||
      !gate_prepare.snapshot.motion_inhibited ||
      !gate_prepare.snapshot.zero_selected)
    {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "MotionGate PREPARE did not prove an inhibited zero state: " +
          gate_prepare.detail));
    }
    lease_id_ = gate_prepare.lease_id;
    candidate_topic_ = gate_prepare.snapshot.candidate_topic;
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
        true,
        true,
        false,
        lease_id_,
        candidate_topic_,
        "conditioning generation prepared"));
  }

  MotionConditioningResult start()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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

    AuthorityResult gate_open;
    try {
      gate_open = authority_->open(make_operation(lease_id_));
    } catch (const std::exception & error) {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          std::string{"MotionGate OPEN raised: "} + error.what()));
    } catch (...) {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "MotionGate OPEN raised an unknown exception"));
    }
    if (!gate_open.applied || gate_open.snapshot.motion_inhibited ||
      !gate_open.snapshot.writer_bound ||
      gate_open.snapshot.candidate_topic != candidate_topic_)
    {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "MotionGate OPEN did not bind the current candidate writer: " +
          gate_open.detail));
    }

    if (!change_state(kCollisionMonitorFqn, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) ||
      !change_state(kVelocitySmootherFqn, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "Nav2 component activation failed"));
    }
    if (!producer_ || !producer_->start(config_.raw_topic)) {
      return remember(fail_result(
          MotionConditioningFailure::SafetyFault,
          "conditioning producer could not start"));
    }
    renew_timer_ = node_.create_wall_timer(
      config_.renew_period,
      [this]() {on_renew();},
      renew_callback_group_);
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
    renew_timer_.reset();
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
    renew_timer_.reset();
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
  struct Component
  {
    std::uint64_t unique_id{0U};
    std::string node_fqn;
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
    Component & component)
  {
    if (!load_client_->wait_for_service(config_.component_rpc_timeout)) {
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
    if (future.wait_for(config_.component_rpc_timeout) !=
      std::future_status::ready)
    {
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      return false;
    }
    component.unique_id = response->unique_id;
    component.node_fqn = response->full_node_name;
    return component.unique_id != 0U && component.node_fqn == "/" + node_name;
  }

  [[nodiscard]] bool call_unload(std::uint64_t unique_id)
  {
    if (!unload_client_->wait_for_service(config_.component_rpc_timeout)) {
      return false;
    }
    auto request = std::make_shared<UnloadNode::Request>();
    request->unique_id = unique_id;
    auto future = unload_client_->async_send_request(request);
    if (future.wait_for(config_.component_rpc_timeout) !=
      std::future_status::ready)
    {
      return false;
    }
    return future.get()->success;
  }

  [[nodiscard]] bool change_state(
    const std::string & node_fqn,
    std::uint8_t transition_id)
  {
    auto client = node_.create_client<ChangeState>(
      node_fqn + "/change_state",
      rmw_qos_profile_services_default,
      component_callback_group_);
    if (!client->wait_for_service(config_.component_rpc_timeout)) {
      return false;
    }
    auto request = std::make_shared<ChangeState::Request>();
    request->transition.id = transition_id;
    auto future = client->async_send_request(request);
    if (future.wait_for(config_.component_rpc_timeout) !=
      std::future_status::ready)
    {
      return false;
    }
    return future.get()->success;
  }

  [[nodiscard]] std::uint8_t component_state(
    const std::string & node_fqn) const
  {
    auto client = node_.create_client<GetState>(
      node_fqn + "/get_state",
      rmw_qos_profile_services_default,
      component_callback_group_);
    if (!client->wait_for_service(config_.component_rpc_timeout)) {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    auto request = std::make_shared<GetState::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(config_.component_rpc_timeout) !=
      std::future_status::ready)
    {
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
        collision_component_))
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
        smoother_component_))
    {
      return false;
    }
    if (!change_state(
        kCollisionMonitorFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE) ||
      !change_state(
        kVelocitySmootherFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
    {
      return false;
    }
    return component_state(kCollisionMonitorFqn) ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
      component_state(kVelocitySmootherFqn) ==
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

  [[nodiscard]] bool cleanup_components()
  {
    if (!components_loaded_) {
      return true;
    }
    bool success = true;
    const auto smoother_state = component_state(kVelocitySmootherFqn);
    if (smoother_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      success = change_state(
        kVelocitySmootherFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN) && success;
    } else if (
      smoother_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      success = change_state(
        kVelocitySmootherFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN) && success;
    }
    const auto collision_state = component_state(kCollisionMonitorFqn);
    if (collision_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      success = change_state(
        kCollisionMonitorFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN) && success;
    } else if (
      collision_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      success = change_state(
        kCollisionMonitorFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN) && success;
    }
    if (smoother_component_.unique_id != 0U) {
      success = call_unload(smoother_component_.unique_id) && success;
    }
    if (collision_component_.unique_id != 0U) {
      success = call_unload(collision_component_.unique_id) && success;
    }
    success = wait_for_writer_to_disappear(candidate_topic_) && success;
    if (success) {
      components_loaded_ = false;
      collision_component_ = {};
      smoother_component_ = {};
    }
    return success;
  }

  [[nodiscard]] bool inhibit_gate()
  {
    if (lease_id_.empty()) {
      return true;
    }
    try {
      const auto result = authority_->inhibit(make_operation(lease_id_));
      return result.applied && result.zero_proven;
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool cleanup_generation(const std::string &)
  {
    if (producer_) {
      producer_->stop();
    }
    renew_timer_.reset();
    const bool zero_proven = inhibit_gate();
    const bool clean = cleanup_components();
    if (!candidate_topic_.empty()) {
      return zero_proven && clean &&
             wait_for_writer_to_disappear(candidate_topic_);
    }
    return zero_proven && clean;
  }

  void reset_generation()
  {
    components_loaded_ = false;
    collision_component_ = {};
    smoother_component_ = {};
    lease_id_.clear();
    candidate_topic_.clear();
    prepare_open_deadline_ = {};
  }

  void on_renew()
  {
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
    if (!runtime_graph_is_healthy()) {
      (void)fail(
        MotionConditioningFailure::SafetyFault,
        "conditioning component or dependency graph is unhealthy");
      return;
    }
    try {
      const auto result = authority_->renew(make_operation(lease_id_));
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

  [[nodiscard]] bool runtime_graph_is_healthy() const
  {
    if (!components_loaded_ ||
      component_state(kCollisionMonitorFqn) !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
      component_state(kVelocitySmootherFqn) !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      return false;
    }
    const auto candidate_publishers =
      node_.get_publishers_info_by_topic(candidate_topic_);
    const bool candidate_writer = std::any_of(
      candidate_publishers.cbegin(), candidate_publishers.cend(),
      [](const rclcpp::TopicEndpointInfo & endpoint) {
        return endpoint.node_name() == "collision_monitor" &&
               endpoint.topic_type() == "geometry_msgs/msg/TwistStamped";
      });
    return candidate_writer &&
           node_.count_publishers(config_.scan_topic) > 0U &&
           node_.count_publishers(config_.odom_topic) > 0U &&
           node_.count_publishers(config_.collision_state_topic) > 0U;
  }

  rclcpp::Node & node_;
  std::shared_ptr<MotionAuthorityPort> authority_;
  std::shared_ptr<MotionProducerPort> producer_;
  MotionConditioningConfig config_;
  std::function<std::string()> request_id_generator_;
  mutable std::recursive_mutex mutex_;
  MotionConditioningState state_{MotionConditioningState::Stopped};
  MotionConditioningResult last_result_{};
  bool collision_stop_{false};
  bool components_loaded_{false};
  Component collision_component_;
  Component smoother_component_;
  std::string lease_id_;
  std::string candidate_topic_;
  std::chrono::steady_clock::time_point prepare_open_deadline_{};
  rclcpp::CallbackGroup::SharedPtr component_callback_group_;
  rclcpp::CallbackGroup::SharedPtr renew_callback_group_;
  rclcpp::Client<LoadNode>::SharedPtr load_client_;
  rclcpp::Client<UnloadNode>::SharedPtr unload_client_;
  rclcpp::Subscription<CollisionState>::SharedPtr collision_subscription_;
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
