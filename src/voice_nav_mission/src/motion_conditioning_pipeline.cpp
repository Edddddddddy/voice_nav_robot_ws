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
#include <rmw/types.h>

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
#include <unordered_map>
#include <optional>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_set>
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
#include "voice_nav_mission/motion_source_freshness.hpp"

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
using WriterGid = std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>;

rclcpp::QoS latest_sensor_qos()
{
  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  return qos;
}

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

WriterGid endpoint_writer_gid(const rclcpp::TopicEndpointInfo & endpoint)
{
  WriterGid gid{};
  const auto & endpoint_gid = endpoint.endpoint_gid();
  std::copy(endpoint_gid.cbegin(), endpoint_gid.cend(), gid.begin());
  return gid;
}

WriterGid message_writer_gid(const rclcpp::MessageInfo & message_info)
{
  WriterGid gid{};
  const auto & raw_gid = message_info.get_rmw_message_info().publisher_gid;
  std::copy_n(raw_gid.data, RMW_GID_STORAGE_SIZE, gid.begin());
  return gid;
}

bool gid_is_zero(const WriterGid & gid)
{
  return std::all_of(gid.cbegin(), gid.cend(), [](const auto byte) {
             return byte == 0U;
  });
}

bool same_gid(const WriterGid & left, const WriterGid & right)
{
  return left == right;
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

  struct StartOperationGuard
  {
    StartOperationGuard(Impl & owner, const std::uint64_t generation)
    : owner_(owner), generation_(generation),
      active_(owner.begin_start_operation(generation)) {}

    ~StartOperationGuard()
    {
      if (active_) {
        owner_.end_start_operation(generation_);
      }
    }

    Impl & owner_;
    std::uint64_t generation_{0U};
    bool active_{false};
  };

  struct IngressCallbackState
  {
    std::function<void(const LaserScan::ConstSharedPtr)> scan_handler;
    std::function<void(const Odometry::ConstSharedPtr)> odom_handler;
    std::function<void(const Clock::ConstSharedPtr)> clock_handler;
    std::function<void(
        const CollisionState::ConstSharedPtr &, const rclcpp::MessageInfo &)>
    collision_handler;
    std::function<void()> renew_handler;

    bool enter()
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!accepting) {
        return false;
      }
      ++active;
      ++active_by_thread[std::this_thread::get_id()];
      return true;
    }

    void leave()
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (active > 0U) {
        --active;
      }
      const auto thread_iterator = active_by_thread.find(
        std::this_thread::get_id());
      if (thread_iterator != active_by_thread.end()) {
        if (thread_iterator->second > 1U) {
          --thread_iterator->second;
        } else {
          active_by_thread.erase(thread_iterator);
        }
      }
      condition.notify_all();
    }

    void disable()
    {
      std::lock_guard<std::mutex> lock(mutex);
      accepting = false;
    }

    void wait()
    {
      std::unique_lock<std::mutex> lock(mutex);
      const auto current_thread = std::this_thread::get_id();
      condition.wait(lock, [this, current_thread]() {
          const auto current_iterator = active_by_thread.find(current_thread);
          const auto current_active = current_iterator == active_by_thread.end() ?
          0U : current_iterator->second;
          return active <= current_active;
        });
    }

    std::size_t active_count() const
    {
      std::lock_guard<std::mutex> lock(mutex);
      return active;
    }

    std::function<void(const LaserScan::ConstSharedPtr)> copy_scan_handler() const
    {
      std::lock_guard<std::mutex> lock(mutex);
      return scan_handler;
    }

    std::function<void(const Odometry::ConstSharedPtr)> copy_odom_handler() const
    {
      std::lock_guard<std::mutex> lock(mutex);
      return odom_handler;
    }

    std::function<void(const Clock::ConstSharedPtr)> copy_clock_handler() const
    {
      std::lock_guard<std::mutex> lock(mutex);
      return clock_handler;
    }

    std::function<void(
        const CollisionState::ConstSharedPtr &, const rclcpp::MessageInfo & )>
    copy_collision_handler() const
    {
      std::lock_guard<std::mutex> lock(mutex);
      return collision_handler;
    }

    std::function<void()> copy_renew_handler() const
    {
      std::lock_guard<std::mutex> lock(mutex);
      return renew_handler;
    }

private:
    mutable std::mutex mutex;
    std::condition_variable condition;
    bool accepting{true};
    std::size_t active{0U};
    std::unordered_map<std::thread::id, std::size_t> active_by_thread;
  };

  struct IngressCallbackGuard
  {
    explicit IngressCallbackGuard(std::shared_ptr<IngressCallbackState> state)
    : state_(std::move(state)), active_(state_ && state_->enter()) {}

    ~IngressCallbackGuard()
    {
      if (active_) {
        state_->leave();
      }
    }

    bool is_active() const
    {
      return active_;
    }

    std::shared_ptr<IngressCallbackState> state_;
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
    local_transaction_plane_(config_.transaction_plane ?
      config_.transaction_plane : std::make_shared<RuntimeTransactionPlane>(0U)),
    scan_freshness_(config_.dependency_liveness_timeout),
    odom_freshness_(config_.dependency_liveness_timeout),
    clock_freshness_(config_.dependency_liveness_timeout),
    clock_progress_freshness_(config_.dependency_liveness_timeout),
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
    callback_state_ = std::make_shared<IngressCallbackState>();
    callback_state_->scan_handler = [this](const LaserScan::ConstSharedPtr) {
        if (config_.before_health_callback) {
          config_.before_health_callback();
        }
        {
          std::lock_guard<std::mutex> lock(health_mutex_);
          scan_freshness_.observe(std::chrono::steady_clock::now());
        }
        if (config_.after_health_callback) {
          config_.after_health_callback();
        }
      };
    callback_state_->odom_handler = [this](const Odometry::ConstSharedPtr) {
        if (config_.before_health_callback) {
          config_.before_health_callback();
        }
        {
          std::lock_guard<std::mutex> lock(health_mutex_);
          odom_freshness_.observe(std::chrono::steady_clock::now());
        }
        if (config_.after_health_callback) {
          config_.after_health_callback();
        }
      };
    callback_state_->clock_handler = [this](const Clock::ConstSharedPtr message) {
        if (config_.before_health_callback) {
          config_.before_health_callback();
        }
        {
          std::lock_guard<std::mutex> lock(health_mutex_);
          const auto stamp = static_cast<std::int64_t>(message->clock.sec) * 1000000000LL +
            static_cast<std::int64_t>(message->clock.nanosec);
          const auto receipt = std::chrono::steady_clock::now();
          if (clock_seen_ && stamp > last_clock_stamp_) {
            clock_progress_freshness_.observe(receipt);
          }
          clock_seen_ = true;
          last_clock_stamp_ = stamp;
          clock_freshness_.observe(receipt);
        }
        if (config_.after_health_callback) {
          config_.after_health_callback();
        }
      };
    callback_state_->collision_handler = [this](
      const CollisionState::ConstSharedPtr message,
      const rclcpp::MessageInfo & message_info) {
        const auto publisher_gid = message_writer_gid(message_info);
        if (
          message->action_type == CollisionState::STOP &&
          message->polygon_name == "stop_zone")
        {
          std::lock_guard<std::recursive_mutex> lock(mutex_);
          if (
            !destroying_.load() &&
            state_ == MotionConditioningState::Running &&
            collision_writer_bound_ &&
            collision_writer_generation_ == generation_ &&
            same_gid(publisher_gid, collision_writer_gid_) &&
            !collision_writer_gid_is_retired(publisher_gid))
          {
            collision_stop_ = true;
            collision_token_ = correlation_token_;
          }
        }
      };
    callback_state_->renew_handler = [this]() {on_renew();};
    rclcpp::SubscriptionOptions health_options;
    health_options.callback_group = component_callback_group_;
    const auto weak_callback_state = std::weak_ptr<IngressCallbackState>(callback_state_);
    scan_subscription_ = node_.create_subscription<LaserScan>(
      config_.scan_topic,
      latest_sensor_qos(),
      [weak_callback_state](const LaserScan::ConstSharedPtr message) {
        auto state = weak_callback_state.lock();
        IngressCallbackGuard guard(state);
        if (!guard.is_active()) {
          return;
        }
        const auto handler = state->copy_scan_handler();
        if (handler) {
          handler(message);
        }
      },
      health_options);
    odom_subscription_ = node_.create_subscription<Odometry>(
      config_.odom_topic,
      rclcpp::SensorDataQoS(),
      [weak_callback_state](const Odometry::ConstSharedPtr message) {
        auto state = weak_callback_state.lock();
        IngressCallbackGuard guard(state);
        if (!guard.is_active()) {
          return;
        }
        const auto handler = state->copy_odom_handler();
        if (handler) {
          handler(message);
        }
      },
      health_options);
    clock_subscription_ = node_.create_subscription<Clock>(
      config_.clock_topic,
      rclcpp::ClockQoS(),
      [weak_callback_state](const Clock::ConstSharedPtr message) {
        auto state = weak_callback_state.lock();
        IngressCallbackGuard guard(state);
        if (!guard.is_active()) {
          return;
        }
        const auto handler = state->copy_clock_handler();
        if (handler) {
          handler(message);
        }
      },
      health_options);
    rclcpp::SubscriptionOptions collision_options;
    collision_options.callback_group = component_callback_group_;
    collision_subscription_ = node_.create_subscription<CollisionState>(
      config_.collision_state_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      [weak_callback_state](
        const CollisionState::ConstSharedPtr message,
        const rclcpp::MessageInfo & message_info) {
        auto state = weak_callback_state.lock();
        IngressCallbackGuard guard(state);
        if (!guard.is_active()) {
          return;
        }
        const auto handler = state->copy_collision_handler();
        if (handler) {
          handler(message, message_info);
        }
      },
      collision_options);
  }

  ~Impl()
  {
    destroying_.store(true);
    shutdown_ingress_requested_.store(true, std::memory_order_release);
    disable_ingress_callbacks();
    (void)stop();
    drain_start_operations();
    wait_for_renew_callbacks();
    join_cleanup_continuation();
    finalize_destruction_cleanup();
    wait_for_ingress_callbacks();
    callback_state_.reset();
  }

  MotionConditioningResult prepare()
  {
    if (config_.startup_reconciliation_on_prepare) {
      const auto startup = reconcile_startup();
      if (!startup.ok) {
        return startup;
      }
    }
    MotionConditioningResult existing;
    bool cleanup_needed = false;
    if (!begin_prepare(existing, cleanup_needed)) {
      return existing;
    }

    if (cleanup_needed && !cleanup_generation("prepare handover cleanup")) {
      MotionConditioningResult result;
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        state_ = MotionConditioningState::Failed;
        result = remember(make_result(
            state_, MotionConditioningFailure::SafetyFault, false, false,
            collision_stop_, lease_id_, candidate_topic_,
            with_cleanup_failure(
              "old conditioning generation could not be cleaned up")));
      }
      finish_teardown(result, TeardownIntent::Failure);
      return result;
    }

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      reset_generation(true);
      collision_stop_ = false;
    }

    bool current_zero_proven = false;
    try {
      std::lock_guard<std::mutex> authority_lock(authority_call_mutex_);
      current_zero_proven = gate_snapshot_proves_zero(authority_->snapshot());
    } catch (...) {
      current_zero_proven = false;
    }
    if (!current_zero_proven) {
      MotionConditioningResult result;
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        state_ = MotionConditioningState::Failed;
        result = remember(make_result(
            state_, MotionConditioningFailure::SafetyFault, false, false,
            collision_stop_, lease_id_, candidate_topic_,
            "current MotionGate snapshot did not prove an inhibited zero"));
      }
      finish_teardown(result, TeardownIntent::Failure);
      return result;
    }
    if (prepare_cancel_requested_.load()) {
      return finish_cancelled_prepare(
        "conditioning PREPARE was cancelled before the Gate request");
    }

    std::optional<TransactionLease> prepare_lease;
    std::optional<AuthorityResult> gate_prepare_result;
    try {
      prepare_lease = begin_side_effect(RuntimeTransactionSideEffect::Prepare);
      if (prepare_lease.has_value()) {
        gate_prepare_result = prepare_lease->invoke(
          RuntimeTransactionSideEffect::Prepare,
          [this]() {return authority_->prepare(make_operation());});
      }
    } catch (const std::exception & error) {
      if (prepare_cancel_requested_.load()) {
        return finish_cancelled_prepare(
          "conditioning PREPARE was cancelled while the Gate request drained");
      }
      const auto result = fail_owned(
        MotionConditioningFailure::SafetyFault,
        std::string{"MotionGate PREPARE raised: "} + error.what(), true);
      finish_teardown(result, TeardownIntent::Failure);
      return result;
    } catch (...) {
      if (prepare_cancel_requested_.load()) {
        return finish_cancelled_prepare(
          "conditioning PREPARE was cancelled while the Gate request drained");
      }
      const auto result = fail_owned(
        MotionConditioningFailure::SafetyFault,
        "MotionGate PREPARE raised an unknown exception", true);
      finish_teardown(result, TeardownIntent::Failure);
      return result;
    }

    if (!gate_prepare_result.has_value()) {
      if (prepare_cancel_requested_.load()) {
        return finish_cancelled_prepare(
          "conditioning PREPARE was cancelled at its transaction commit");
      }
      const auto result = fail_owned(
        MotionConditioningFailure::SafetyFault,
        "generation permit was revoked before MotionGate PREPARE", true);
      finish_teardown(result, TeardownIntent::Failure);
      return result;
    }
    const auto gate_prepare = std::move(*gate_prepare_result);

    bool gate_prepare_valid = false;
    std::string gate_prepare_detail;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      lease_id_ = !gate_prepare.lease_id.empty() ?
        gate_prepare.lease_id : gate_prepare.snapshot.lease_id;
      candidate_topic_ = gate_prepare.snapshot.candidate_topic;
      gate_prepare_valid =
        gate_prepare.applied && !lease_id_.empty() &&
        gate_prepare.snapshot.state == GateState::Prepared &&
        gate_prepare.snapshot.motion_inhibited &&
        gate_prepare.snapshot.zero_selected && gate_prepare.zero_proven &&
        gate_prepare.snapshot.zero_published &&
        !gate_prepare.snapshot.gate_instance_id.empty();
      gate_prepare_detail = gate_prepare.detail;
      if (gate_prepare_valid) {
        generation_ = ++generation_counter_;
        prepare_open_deadline_ =
          std::chrono::steady_clock::now() + config_.prepare_open_deadline;
      }
    }
    if (prepare_cancel_requested_.load()) {
      return finish_cancelled_prepare(
        "conditioning PREPARE was cancelled after the Gate request");
    }
    if (!gate_prepare_valid) {
      const auto result = fail_owned(
        MotionConditioningFailure::SafetyFault,
        "MotionGate PREPARE did not prove an inhibited zero state: " +
        gate_prepare_detail, true);
      finish_teardown(result, TeardownIntent::Failure);
      return result;
    }
    std::uint64_t prepared_generation = 0U;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      prepared_generation = generation_;
    }
    set_teardown_generation(prepared_generation);

    if (prepare_cancel_requested_.load()) {
      return finish_cancelled_prepare(
        "conditioning PREPARE was cancelled before component setup");
    }
    if (!prepare_lease->current()) {
      const auto failed = fail_owned(
        MotionConditioningFailure::SafetyFault,
        "generation permit was revoked before component setup", true);
      finish_teardown(failed, TeardownIntent::Failure);
      return failed;
    }

    setup_failure_detail_.clear();
    if (!load_and_configure_components()) {
      if (prepare_cancel_requested_.load()) {
        return finish_cancelled_prepare(
          "conditioning PREPARE was cancelled while component setup drained");
      }
      const auto result = fail_owned(
        MotionConditioningFailure::SafetyFault,
        "Nav2 component load/configure failed: " + setup_failure_detail_, true);
      finish_teardown(result, TeardownIntent::Failure);
      return result;
    }
    if (prepare_cancel_requested_.load()) {
      return finish_cancelled_prepare(
        "conditioning PREPARE was cancelled after component setup");
    }
    if (!prepare_lease->current()) {
      const auto failed = fail_owned(
        MotionConditioningFailure::SafetyFault,
        "generation permit was revoked after component setup", true);
      finish_teardown(failed, TeardownIntent::Failure);
      return failed;
    }
    if (std::chrono::steady_clock::now() >= prepare_open_deadline_) {
      const auto result = fail_owned(
        MotionConditioningFailure::SafetyFault,
        "PREPARE to OPEN deadline expired during component setup", true);
      finish_teardown(result, TeardownIntent::Failure);
      return result;
    }
    if (prepare_cancel_requested_.load()) {
      return finish_cancelled_prepare(
        "conditioning PREPARE was cancelled before final transaction commit");
    }
    MotionConditioningResult result;
    const bool prepare_committed = prepare_lease.has_value() &&
      prepare_lease->commit([&]() {
          std::lock_guard<std::recursive_mutex> lock(mutex_);
          if (prepare_cancel_requested_.load() || generation_ != prepared_generation ||
          (lease_id_ != gate_prepare.lease_id &&
          lease_id_ != gate_prepare.snapshot.lease_id))
          {
            return false;
          }
          state_ = MotionConditioningState::Prepared;
          result = remember(make_result(
            state_, MotionConditioningFailure::None,
            gate_prepare.zero_proven && gate_prepare.snapshot.zero_published,
            true, false, lease_id_, candidate_topic_,
            "conditioning generation prepared"));
          if (result.zero_proven) {
            result.zero_proven_at = std::chrono::steady_clock::now();
            last_result_.zero_proven_at = result.zero_proven_at;
          }
          return true;
      });
    if (!prepare_committed) {
      if (prepare_cancel_requested_.load()) {
        return finish_cancelled_prepare(
          "conditioning PREPARE was cancelled at its final transaction commit");
      }
      const auto failed = fail_owned(
        MotionConditioningFailure::SafetyFault,
        "generation or identity permit was revoked before PREPARE commit", true);
      finish_teardown(failed, TeardownIntent::Failure);
      return failed;
    }
    finish_prepare(result);
    return result;
  }

  MotionConditioningResult reconcile_startup()
  {
    std::lock_guard<std::mutex> startup_lock(startup_mutex_);
    if (startup_reconciled_ || startup_reconcile_failed_) {
      return startup_result_;
    }
    return reconcile_startup_owned();
  }

  MotionConditioningResult start()
  {
    std::chrono::steady_clock::time_point handover_deadline;
    std::uint64_t generation = 0U;
    std::string expected_lease;
    std::string expected_candidate;
    std::string invalid_detail;
    bool prepare_open_deadline_expired = false;
    bool active_generation_rejected = false;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (state_ != MotionConditioningState::Prepared) {
        active_generation_rejected =
          state_ == MotionConditioningState::Running ||
          activation_in_progress_.load();
        if (!active_generation_rejected) {
          invalid_detail =
            "MotionConditioningPipeline start requires PREPARED state";
        }
      } else if (std::chrono::steady_clock::now() >= prepare_open_deadline_) {
        invalid_detail = "PREPARE to OPEN deadline expired";
        prepare_open_deadline_expired = true;
      } else {
        handover_deadline = prepare_open_deadline_;
        generation = generation_;
        expected_lease = lease_id_;
        expected_candidate = candidate_topic_;
      }
    }
    if (active_generation_rejected) {
      return start_busy_result();
    }
    if (!invalid_detail.empty()) {
      if (prepare_open_deadline_expired) {
        return fail_synchronously(
          MotionConditioningFailure::SafetyFault, std::move(invalid_detail));
      }
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      auto result = make_result(
        state_, MotionConditioningFailure::SafetyFault, false,
        last_result_.zero_proven, collision_stop_, lease_id_, candidate_topic_,
        std::move(invalid_detail));
      result.zero_proven_at = last_result_.zero_proven_at;
      return result;
    }

    StartOperationGuard start_operation(*this, generation);
    if (!start_operation.active_) {
      return start_busy_result();
    }

    std::optional<TransactionLease> open_lease;
    std::optional<AuthorityResult> gate_open_result;
    AuthorityOperation open_operation;
    bool activation_ready = false;
    try {
      open_operation = make_operation(expected_lease);
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        activation_ready =
          generation_ == generation &&
          state_ == MotionConditioningState::Prepared &&
          lease_id_ == expected_lease && candidate_topic_ == expected_candidate &&
          std::chrono::steady_clock::now() < handover_deadline;
        if (activation_ready) {
          generation_request_id_ = open_operation.request_id;
          correlation_token_ = MotionConditioningCorrelationToken{
            generation, expected_lease, open_operation.request_id};
          activation_in_progress_.store(true);
          activation_failed_.store(false);
          std::lock_guard<std::mutex> activation_lock(activation_mutex_);
          activation_failure_detail_.clear();
        }
      }
    } catch (const std::exception & error) {
      return fail_synchronously(
        MotionConditioningFailure::SafetyFault,
        std::string{"MotionGate OPEN operation could not be created: "} + error.what());
    } catch (...) {
      return fail_synchronously(
        MotionConditioningFailure::SafetyFault,
        "MotionGate OPEN operation could not be created");
    }
    if (!activation_ready) {
      if (std::chrono::steady_clock::now() >= handover_deadline) {
        return fail_synchronously(
          MotionConditioningFailure::SafetyFault,
          "PREPARE to OPEN deadline expired before MotionGate OPEN");
      }
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "activation was cancelled before MotionGate OPEN");
    }
    if (!activation_token_current(generation)) {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        activation_failure_detail());
    }
    if (!controller_is_active(handover_deadline)) {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "diff_drive_controller is not active before MotionGate OPEN");
    }
    if (!activation_token_current(generation)) {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        activation_failure_detail());
    }
    if (!candidate_writer_is_visible(expected_candidate, handover_deadline, generation)) {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "candidate writer was not visible before MotionGate OPEN");
    }

    if (config_.before_open_callback) {
      try {
        config_.before_open_callback();
      } catch (...) {
        return abort_activation(
          generation,
          MotionConditioningFailure::SafetyFault,
          "OPEN cancellation fence raised an exception");
      }
    }
    if (!activation_token_current(generation)) {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "activation was cancelled before MotionGate OPEN");
    }

    bool producer_started = false;
    std::unique_lock<std::mutex> producer_lock;
    try {
      open_lease = begin_side_effect(RuntimeTransactionSideEffect::Open);
      if (open_lease.has_value()) {
        gate_open_result = open_lease->invoke(
          RuntimeTransactionSideEffect::Open,
          [this, &open_operation]() {
            std::lock_guard<std::mutex> authority_lock(authority_call_mutex_);
            auto result = authority_->open(open_operation);
            if (result.snapshot.gate_instance_id != open_operation.gate_instance_id) {
              result.applied = false;
              result.detail = "MotionGate OPEN returned a stale gate identity";
            }
            return result;
          });
      }
    } catch (const std::exception & error) {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        std::string{"MotionGate OPEN raised: "} + error.what());
    } catch (...) {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "MotionGate OPEN raised an unknown exception");
    }
    if (!gate_open_result.has_value()) {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "generation permit was revoked before MotionGate OPEN");
    }
    const auto gate_open = std::move(*gate_open_result);
    if (!open_lease->current() || !activation_token_current(generation) ||
      !gate_open.applied || !gate_open.snapshot.authority_live ||
      gate_open.snapshot.motion_inhibited ||
      !gate_open.snapshot.writer_bound ||
      gate_open.snapshot.lease_id != expected_lease ||
      gate_open.snapshot.candidate_topic != expected_candidate)
    {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "MotionGate OPEN did not bind the current candidate writer: " +
        gate_open.detail);
    }

    std::string timer_error;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!activation_current_locked(generation, expected_lease, expected_candidate)) {
        timer_error = "activation was cancelled before renew timer startup";
      } else {
        try {
          const auto weak_callback_state =
            std::weak_ptr<IngressCallbackState>(callback_state_);
          renew_timer_ = node_.create_wall_timer(
            config_.renew_period,
            [weak_callback_state]() {
              auto state = weak_callback_state.lock();
              IngressCallbackGuard guard(state);
              if (!guard.is_active()) {
                return;
              }
              const auto handler = state->copy_renew_handler();
              if (handler) {
                handler();
              }
            },
            renew_callback_group_,
            false);
        } catch (const std::exception & error) {
          timer_error =
            std::string{"MotionGate renew timer could not start: "} + error.what();
        } catch (...) {
          timer_error = "MotionGate renew timer could not start";
        }
      }
    }
    if (!timer_error.empty()) {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        std::move(timer_error));
    }

    if (!renew_for_activation(
        generation, expected_lease, open_operation.gate_instance_id, &*open_lease))
    {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        activation_failure_detail());
    }
    if (!open_lease->current() || !activation_token_current(generation)) {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        activation_failure_detail());
    }
    if (!change_state(
        kCollisionMonitorFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        handover_deadline) ||
      !open_lease->current() || !activation_token_current(generation) ||
      std::chrono::steady_clock::now() >= handover_deadline)
    {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "Collision Monitor activation failed after MotionGate OPEN");
    }
    if (!change_state(
        kVelocitySmootherFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        handover_deadline) ||
      !open_lease->current() || !activation_token_current(generation) ||
      std::chrono::steady_clock::now() >= handover_deadline)
    {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "Velocity Smoother activation failed after MotionGate OPEN");
    }
    const auto second_renew_ok = renew_for_activation(
      generation, expected_lease, open_operation.gate_instance_id, &*open_lease);
    const auto graph_health = second_renew_ok ?
      runtime_graph_health(handover_deadline, expected_candidate) :
      RuntimeHealthAssessment{
      RuntimeHealthReason::ComponentUnavailable,
      "MotionGate RENEW failed during activation"};
    if (!open_lease->current() || !graph_health.healthy() ||
      !activation_token_current(generation))
    {
      return abort_activation(
        generation,
        failure_for_health(graph_health.reason),
        graph_health.detail.empty() ?
        "conditioning authority or dependency health failed during activation" :
        graph_health.detail);
    }

    if (std::chrono::steady_clock::now() >= handover_deadline) {
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "PREPARE to OPEN deadline expired during activation");
    }

    producer_lock = std::unique_lock<std::mutex>(producer_mutex_);
    const auto producer_health = runtime_graph_health(
      handover_deadline, expected_candidate);
    if (!producer_health.healthy()) {
      producer_lock.unlock();
      return abort_activation(
        generation,
        failure_for_health(producer_health.reason),
        producer_health.detail.empty() ?
        "conditioning dependency graph changed before producer start" :
        producer_health.detail);
    }
    bool activation_fence_valid = false;
    const bool open_lease_current = open_lease->current();
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      activation_fence_valid =
        open_lease_current &&
        activation_current_locked(generation, expected_lease, expected_candidate) &&
        std::chrono::steady_clock::now() < handover_deadline;
    }
    if (!activation_fence_valid) {
      producer_lock.unlock();
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "activation fence was cancelled before producer start");
    }

    GateSnapshot final_snapshot;
    try {
      std::lock_guard<std::mutex> authority_lock(authority_call_mutex_);
      final_snapshot = authority_->snapshot();
    } catch (...) {
      producer_lock.unlock();
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "MotionGate snapshot raised before producer start");
    }
    if (final_snapshot.gate_instance_id != open_operation.gate_instance_id ||
      final_snapshot.lease_id != expected_lease ||
      !final_snapshot.authority_live || final_snapshot.motion_inhibited ||
      !final_snapshot.writer_bound ||
      final_snapshot.candidate_topic != expected_candidate ||
      activation_failed_.load() ||
      !open_lease->current())
    {
      producer_lock.unlock();
      return abort_activation(
        generation,
        MotionConditioningFailure::SafetyFault,
        "activation fence failed before producer start");
    }

    if (!producer_started) {
      try {
        const auto producer_result = open_lease->invoke(
          RuntimeTransactionSideEffect::ControllerStart,
          [this]() {return producer_ && producer_->start(config_.raw_topic);});
        if (!producer_result.has_value()) {
          producer_lock.unlock();
          return abort_activation(
            generation,
            MotionConditioningFailure::SafetyFault,
            "generation permit was revoked before controller start");
        }
        producer_started = *producer_result;
      } catch (const std::exception & error) {
        producer_lock.unlock();
        return abort_activation(
          generation,
          MotionConditioningFailure::SafetyFault,
          std::string{"conditioning producer raised: "} + error.what());
      } catch (...) {
        producer_lock.unlock();
        return abort_activation(
          generation,
          MotionConditioningFailure::SafetyFault,
          "conditioning producer raised an unknown exception");
      }
      if (!producer_started) {
        producer_lock.unlock();
        return abort_activation(
          generation,
          MotionConditioningFailure::SafetyFault,
          "conditioning producer could not start");
      }
    }
    MotionConditioningResult result;
    const bool activation_committed = open_lease->commit([&]() {
          std::lock_guard<std::recursive_mutex> lock(mutex_);
          if (!activation_current_locked(generation, expected_lease, expected_candidate) ||
          std::chrono::steady_clock::now() >= handover_deadline ||
          activation_failed_.load() || !renew_timer_)
          {
            return false;
          }
          try {
            // The timer is created stopped.  Its first possible callback is
            // admitted only after this lease commits the Running state.
            renew_timer_->reset();
          } catch (...) {
            return false;
          }
          activation_in_progress_.store(false);
          state_ = MotionConditioningState::Running;
          enable_renew_callbacks();
          result = remember(make_result(
            state_, MotionConditioningFailure::None, true, false, false,
            lease_id_, candidate_topic_, "conditioning generation running"));
          return true;
      });
    if (activation_committed) {
      producer_lock.unlock();
      return result;
    }
    try {
      producer_->stop();
      producer_stop_proven_.store(true);
    } catch (...) {
    }
    producer_lock.unlock();
    return abort_activation(
      generation,
      MotionConditioningFailure::SafetyFault,
      "activation fence failed at producer commit");
  }

  MotionConditioningResult stop()
  {
    MotionConditioningResult existing;
    if (!begin_teardown(existing, true)) {
      return existing;
    }
    MotionConditioningResult result;
    try {
      result = stop_owned();
    } catch (...) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      state_ = MotionConditioningState::Failed;
      result = remember(make_result(
          state_, MotionConditioningFailure::SafetyFault, false,
          last_zero_proven_at_ != std::chrono::steady_clock::time_point{},
          collision_stop_, lease_id_, candidate_topic_,
          "conditioning stop raised during teardown"));
      result.zero_proven_at = last_zero_proven_at_;
    }
    finish_teardown(result, TeardownIntent::Stop);
    return result;
  }

  void begin_shutdown_ingress() noexcept
  {
    shutdown_ingress_requested_.store(true, std::memory_order_release);
    disable_ingress_callbacks();
  }

  MotionConditioningResult fail(
    MotionConditioningCorrelationToken token,
    MotionConditioningFailure failure,
    std::string detail)
  {
    if (config_.before_token_claim) {
      config_.before_token_claim(token);
    }
    MotionConditioningResult existing;
    if (!begin_token_teardown(token, existing, true)) {
      return existing;
    }
    MotionConditioningResult result;
    try {
      result = fail_owned(failure, std::move(detail), true);
    } catch (...) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      state_ = MotionConditioningState::Failed;
      result = remember(make_result(
          state_, MotionConditioningFailure::SafetyFault, false,
          last_zero_proven_at_ != std::chrono::steady_clock::time_point{},
          collision_stop_, lease_id_, candidate_topic_,
          "conditioning failure raised during teardown"));
      result.zero_proven_at = last_zero_proven_at_;
    }
    finish_teardown(result, TeardownIntent::Failure);
    return result;
  }

  MotionConditioningCorrelationToken correlation_token() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return correlation_token_;
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
  using TransactionLease = RuntimeTransactionPlane::Lease;

  [[nodiscard]] std::shared_ptr<RuntimeTransactionPlane> transaction_plane() const
  {
    return config_.transaction_plane ? config_.transaction_plane :
           local_transaction_plane_;
  }

  [[nodiscard]] std::optional<TransactionLease> begin_side_effect(
    const RuntimeTransactionSideEffect side_effect)
  {
    const auto plane = transaction_plane();
    const auto generation = config_.transaction_generation_provider ?
      config_.transaction_generation_provider() : plane->generation();
    return plane->begin(generation, side_effect);
  }

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
    std::uint64_t generation{0U};
    std::string expected_fqn;
    std::shared_future<std::shared_ptr<LoadNode::Response>> future;
  };

  struct CleanupFailureContext
  {
    std::string phase;
    std::string fqn;
    std::uint64_t unique_id{0U};
    std::string detail;
  };

  enum class TeardownIntent : std::uint8_t
  {
    Stop,
    Failure,
  };

  enum class TeardownOwner : std::uint8_t
  {
    None,
    Prepare,
    Cleanup,
    CleanupContinuation,
  };

  struct TerminalRecord
  {
    std::uint64_t generation{0U};
    TeardownIntent intent{TeardownIntent::Failure};
    MotionConditioningResult result;
  };

  [[nodiscard]] MotionConditioningResult & remember(
    MotionConditioningResult result)
  {
    last_result_ = std::move(result);
    return last_result_;
  }

  void record_cleanup_failure(
    std::string phase,
    std::string fqn,
    std::uint64_t unique_id,
    std::string detail)
  {
    if (!cleanup_failure_) {
      cleanup_failure_ = CleanupFailureContext{
        std::move(phase), std::move(fqn), unique_id, std::move(detail)};
    }
  }

  [[nodiscard]] std::string cleanup_failure_detail() const
  {
    if (!cleanup_failure_) {
      return {};
    }
    return "cleanup phase=" + cleanup_failure_->phase +
           " fqn=" + cleanup_failure_->fqn +
           " unique_id=" + std::to_string(cleanup_failure_->unique_id) +
           " detail=" + cleanup_failure_->detail;
  }

  [[nodiscard]] std::string with_cleanup_failure(std::string detail) const
  {
    const auto cleanup_detail = cleanup_failure_detail();
    if (!cleanup_detail.empty()) {
      detail += "; " + cleanup_detail;
    }
    return detail;
  }

  [[nodiscard]] MotionConditioningResult start_busy_result() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto result = make_result(
      state_, MotionConditioningFailure::SafetyFault, false,
      last_result_.zero_proven, collision_stop_, lease_id_, candidate_topic_,
      "another start already owns this conditioning generation");
    result.zero_proven_at = last_result_.zero_proven_at;
    return result;
  }

  [[nodiscard]] bool wait_for_existing_teardown(
    MotionConditioningResult & result)
  {
    std::unique_lock<std::mutex> teardown_lock(teardown_mutex_);
    if (!teardown_in_progress_.load() && !terminal_record_) {
      return false;
    }
    if (teardown_in_progress_.load()) {
      teardown_cv_.wait(teardown_lock, [this]() {
          return !teardown_in_progress_.load();
        });
    }
    result = terminal_record_ ? terminal_record_->result : teardown_result_;
    return true;
  }

  [[nodiscard]] MotionConditioningResult fail_synchronously(
    MotionConditioningFailure failure,
    std::string detail)
  {
    MotionConditioningResult existing;
    if (!begin_teardown(existing, true)) {
      return existing;
    }
    MotionConditioningResult result;
    try {
      result = fail_owned(failure, std::move(detail), true);
    } catch (...) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      state_ = MotionConditioningState::Failed;
      result = remember(make_result(
          state_, MotionConditioningFailure::SafetyFault, false,
          last_zero_proven_at_ != std::chrono::steady_clock::time_point{},
          collision_stop_, lease_id_, candidate_topic_,
          "synchronous conditioning failure raised during teardown"));
      result.zero_proven_at = last_zero_proven_at_;
    }
    finish_teardown(result, TeardownIntent::Failure);
    return result;
  }

  [[nodiscard]] bool begin_teardown(
    MotionConditioningResult & existing,
    bool wait_for_existing)
  {
    for (;; ) {
      bool prepare_takeover = false;
      {
        std::unique_lock<std::mutex> teardown_lock(teardown_mutex_);
        if (teardown_in_progress_.load()) {
          if (!wait_for_existing) {
            existing = teardown_result_;
            return false;
          }
          if (teardown_owner_ == TeardownOwner::Prepare) {
            // Fence PREPARE immediately, but leave component ownership to the
            // one cleanup transaction that will claim the owner after
            // PREPARE's in-flight RPC has returned.
            prepare_cancel_requested_.store(true);
            {
              std::lock_guard<std::recursive_mutex> state_lock(mutex_);
              teardown_generation_ = generation_;
              invalidate_activation_locked();
            }
            prepare_takeover = true;
          } else {
            teardown_cv_.wait(teardown_lock, [this]() {
                return !teardown_in_progress_.load();
              });
            existing = teardown_result_;
            return false;
          }
        } else if (terminal_record_) {
          existing = terminal_record_->result;
          return false;
        } else {
          {
            std::lock_guard<std::recursive_mutex> state_lock(mutex_);
            teardown_generation_ = generation_;
            invalidate_activation_locked();
          }
          teardown_owner_ = TeardownOwner::Cleanup;
          teardown_in_progress_.store(true);
          return true;
        }
      }

      if (prepare_takeover) {
        // This call deliberately bypasses the normal authority-call mutex so
        // emergency zero proof does not wait behind PREPARE/OPEN.
        (void)inhibit_gate(true);
        std::unique_lock<std::mutex> teardown_lock(teardown_mutex_);
        teardown_cv_.wait(teardown_lock, [this]() {
            return !teardown_in_progress_.load();
          });
      }
    }
  }

  [[nodiscard]] bool begin_token_teardown(
    const MotionConditioningCorrelationToken & token,
    MotionConditioningResult & existing,
    bool wait_for_existing)
  {
    std::unique_lock<std::mutex> teardown_lock(teardown_mutex_);
    if (cleanup_continuation_running_) {
      teardown_cv_.wait(teardown_lock, [this]() {
          return !cleanup_continuation_running_;
        });
    }
    if (teardown_in_progress_.load()) {
      if (!wait_for_existing) {
        existing = teardown_result_;
        return false;
      }
      teardown_cv_.wait(teardown_lock, [this]() {
          return !teardown_in_progress_.load();
        });
      existing = teardown_result_;
      return false;
    }
    {
      std::lock_guard<std::recursive_mutex> state_lock(mutex_);
      const bool token_matches =
        !destroying_.load() && token.generation != 0U &&
        token.generation == correlation_token_.generation &&
        token.lease_id == correlation_token_.lease_id &&
        token.request_id == correlation_token_.request_id &&
        (state_ == MotionConditioningState::Prepared ||
        state_ == MotionConditioningState::Running);
      if (!token_matches) {
        existing = last_result_;
        return false;
      }
      if (terminal_record_) {
        existing = terminal_record_->result;
        return false;
      }
      teardown_generation_ = generation_;
      invalidate_activation_locked();
    }
    teardown_owner_ = TeardownOwner::Cleanup;
    teardown_in_progress_.store(true);
    return true;
  }

  [[nodiscard]] bool begin_start_operation(std::uint64_t generation)
  {
    std::lock_guard<std::mutex> teardown_lock(teardown_mutex_);
    if (destroying_.load() || teardown_in_progress_.load()) {
      return false;
    }
    if (active_start_operations_ != 0U) {
      return false;
    }
    std::lock_guard<std::recursive_mutex> state_lock(mutex_);
    if (generation_ != generation || state_ != MotionConditioningState::Prepared) {
      return false;
    }
    ++active_start_operations_;
    ++start_operation_threads_[std::this_thread::get_id()];
    return true;
  }

  void end_start_operation(const std::uint64_t generation)
  {
    std::lock_guard<std::mutex> teardown_lock(teardown_mutex_);
    if (active_start_operations_ > 0U) {
      --active_start_operations_;
    }
    const auto thread_id = std::this_thread::get_id();
    const auto thread_iterator = start_operation_threads_.find(thread_id);
    if (thread_iterator != start_operation_threads_.end()) {
      if (thread_iterator->second > 1U) {
        --thread_iterator->second;
      } else {
        start_operation_threads_.erase(thread_iterator);
      }
    }
    if (active_start_operations_ == 0U) {
      terminal_records_.erase(generation);
      start_operation_cv_.notify_all();
    }
  }

  void join_cleanup_continuation()
  {
    std::thread continuation;
    {
      std::lock_guard<std::mutex> lock(teardown_mutex_);
      if (cleanup_continuation_thread_.joinable()) {
        continuation = std::move(cleanup_continuation_thread_);
      }
    }
    if (continuation.joinable()) {
      continuation.join();
    }
  }

  void wait_for_cleanup_continuation()
  {
    std::unique_lock<std::mutex> lock(teardown_mutex_);
    teardown_cv_.wait(lock, [this]() {
        return !cleanup_continuation_running_;
      });
  }

  void schedule_cleanup_continuation()
  {
    std::lock_guard<std::mutex> lock(teardown_mutex_);
    if (cleanup_continuation_running_ || cleanup_continuation_thread_.joinable()) {
      return;
    }
    cleanup_continuation_running_ = true;
    teardown_owner_ = TeardownOwner::CleanupContinuation;
    cleanup_complete_.store(false);
    try {
      cleanup_continuation_thread_ = std::thread([this]() {
            run_cleanup_continuation();
        });
    } catch (...) {
      cleanup_continuation_running_ = false;
      teardown_owner_ = TeardownOwner::Cleanup;
      cleanup_blocked_ = true;
      record_cleanup_failure(
        "cleanup_continuation", config_.container_fqn, 0U,
        "cleanup continuation could not be started");
      teardown_cv_.notify_all();
    }
  }

  void run_cleanup_continuation() noexcept
  {
    try {
      drain_start_operations();
      const bool producer_stopped = safe_producer_stop();
      bool components_clean = false;
      try {
        components_clean = cleanup_components();
      } catch (...) {
        record_cleanup_failure(
          "cleanup", config_.container_fqn, 0U,
          "cleanup continuation raised an unknown exception");
      }
      const bool clean = producer_stopped && components_clean;
      cleanup_complete_.store(clean);
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (clean) {
          cleanup_blocked_ = false;
          reset_generation();
        } else {
          cleanup_blocked_ = true;
        }
      }
    } catch (...) {
      cleanup_blocked_ = true;
      cleanup_complete_.store(false);
      try {
        record_cleanup_failure(
          "cleanup_continuation", config_.container_fqn, 0U,
          "cleanup continuation failed unexpectedly");
      } catch (...) {
      }
    }
    {
      std::lock_guard<std::mutex> lock(teardown_mutex_);
      cleanup_continuation_running_ = false;
      if (teardown_owner_ == TeardownOwner::CleanupContinuation) {
        teardown_owner_ = TeardownOwner::None;
      }
    }
    teardown_cv_.notify_all();
  }

  [[nodiscard]] bool wait_for_start_operations()
  {
    std::unique_lock<std::mutex> teardown_lock(teardown_mutex_);
    const auto owner_iterator = start_operation_threads_.find(
      std::this_thread::get_id());
    const auto own_operations = owner_iterator == start_operation_threads_.cend() ?
      0U : owner_iterator->second;
    const auto deadline = std::chrono::steady_clock::now() + config_.stop_barrier;
    return start_operation_cv_.wait_until(
      teardown_lock, deadline, [this, own_operations]() {
        return active_start_operations_ <= own_operations;
      });
  }

  void drain_start_operations()
  {
    std::unique_lock<std::mutex> teardown_lock(teardown_mutex_);
    const auto owner_iterator = start_operation_threads_.find(
      std::this_thread::get_id());
    const auto own_operations = owner_iterator == start_operation_threads_.cend() ?
      0U : owner_iterator->second;
    start_operation_cv_.wait(teardown_lock, [this, own_operations]() {
        return active_start_operations_ <= own_operations;
      });
  }

  [[nodiscard]] bool begin_prepare(
    MotionConditioningResult & existing,
    bool & cleanup_needed)
  {
    std::unique_lock<std::mutex> teardown_lock(teardown_mutex_);
    const auto terminal_failure_reusable = [this]() {
        return terminal_record_ &&
               terminal_record_->result.zero_proven &&
               terminal_record_->result.zero_proven_at !=
               std::chrono::steady_clock::time_point{} &&
               (terminal_record_->result.failure ==
               MotionConditioningFailure::DependencyUnavailable ||
               terminal_record_->result.failure ==
               MotionConditioningFailure::ExecutionFailed ||
               terminal_record_->result.failure ==
               MotionConditioningFailure::Timeout) &&
               !cleanup_blocked_ && !cleanup_identity_fault_ &&
               !cleanup_failure_.has_value();
      };
    if (cleanup_continuation_running_) {
      teardown_cv_.wait(teardown_lock, [this]() {
          return !cleanup_continuation_running_;
        });
    }
    if (teardown_in_progress_.load()) {
      teardown_cv_.wait(teardown_lock, [this]() {
          return !teardown_in_progress_.load();
        });
      if (terminal_record_ && !terminal_record_->result.ok &&
        !terminal_failure_reusable())
      {
        existing = terminal_record_->result;
        return false;
      }
    }
    {
      std::lock_guard<std::recursive_mutex> state_lock(mutex_);
      if (state_ == MotionConditioningState::Prepared ||
        state_ == MotionConditioningState::Running)
      {
        existing = remember(make_result(
            state_, MotionConditioningFailure::SafetyFault, false, false,
            collision_stop_, lease_id_, candidate_topic_,
            "MotionConditioningPipeline already owns an active generation"));
        return false;
      }
      if (terminal_record_ && !terminal_record_->result.ok &&
        !terminal_failure_reusable())
      {
        existing = terminal_record_->result;
        return false;
      }
      cleanup_needed =
        components_loaded_ || !pending_loads_.empty() ||
        !residual_components_.empty() || !lease_id_.empty() ||
        activation_in_progress_.load();
      teardown_generation_ = generation_;
      invalidate_activation_locked();
      terminal_record_.reset();
      prepare_cancel_requested_.store(false);
      teardown_owner_ = TeardownOwner::Prepare;
      teardown_in_progress_.store(true);
    }
    return true;
  }

  void finish_prepare(const MotionConditioningResult & result)
  {
    {
      std::lock_guard<std::mutex> lock(teardown_mutex_);
      teardown_result_ = result;
      teardown_owner_ = TeardownOwner::None;
      teardown_in_progress_.store(false);
    }
    teardown_cv_.notify_all();
  }

  [[nodiscard]] MotionConditioningResult finish_cancelled_prepare(
    std::string detail)
  {
    MotionConditioningResult result;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      state_ = MotionConditioningState::Stopped;
      result = remember(make_result(
          state_, MotionConditioningFailure::ExecutionFailed, false, false,
          collision_stop_, lease_id_, candidate_topic_, std::move(detail)));
    }
    finish_prepare(result);
    return result;
  }

  void set_teardown_generation(std::uint64_t generation)
  {
    std::lock_guard<std::mutex> lock(teardown_mutex_);
    teardown_generation_ = generation;
  }

  void fail_from_renew_callback(
    const MotionConditioningCorrelationToken & token,
    MotionConditioningFailure failure,
    std::string detail)
  {
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      const bool token_matches =
        !destroying_.load() && token.generation != 0U &&
        token.generation == correlation_token_.generation &&
        token.lease_id == correlation_token_.lease_id &&
        token.request_id == correlation_token_.request_id &&
        (state_ == MotionConditioningState::Prepared ||
        state_ == MotionConditioningState::Running);
      if (!token_matches) {
        return;
      }
    }
    MotionConditioningResult existing;
    // The transaction lease may already be non-current because the
    // generation fence closed after authority_->renew() returned.  The
    // pipeline token is still the identity fence for this Running generation;
    // use the normal cleanup owner so the same lease remains InFlight until
    // zero/producer/component cleanup is complete.
    if (!begin_teardown(existing, false)) {
      return;
    }
    MotionConditioningResult result;
    try {
      result = fail_owned(failure, std::move(detail), false);
    } catch (...) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      state_ = MotionConditioningState::Failed;
      result = remember(make_result(
          state_, MotionConditioningFailure::SafetyFault, false,
          last_zero_proven_at_ != std::chrono::steady_clock::time_point{},
          collision_stop_, lease_id_, candidate_topic_,
          "conditioning callback failure raised during teardown"));
      result.zero_proven_at = last_zero_proven_at_;
    }
    finish_teardown(result, TeardownIntent::Failure);
  }

  void finish_teardown(
    const MotionConditioningResult & result,
    TeardownIntent intent)
  {
    {
      std::lock_guard<std::mutex> lock(teardown_mutex_);
      teardown_result_ = result;
      if (!terminal_record_) {
        terminal_record_ = TerminalRecord{teardown_generation_, intent, result};
        terminal_records_.emplace(teardown_generation_, result);
        prune_terminal_records_locked();
      }
      if (!cleanup_continuation_running_) {
        teardown_owner_ = TeardownOwner::None;
      }
      teardown_in_progress_.store(false);
    }
    teardown_cv_.notify_all();
  }

  void prune_terminal_records_locked()
  {
    constexpr std::size_t kMaxTerminalRecords = 8U;
    while (terminal_records_.size() > kMaxTerminalRecords) {
      auto candidate = terminal_records_.end();
      for (auto iterator = terminal_records_.begin();
        iterator != terminal_records_.end(); ++iterator)
      {
        if (iterator->first == teardown_generation_) {
          continue;
        }
        if (candidate == terminal_records_.end() ||
          iterator->first < candidate->first)
        {
          candidate = iterator;
        }
      }
      if (candidate == terminal_records_.end()) {
        break;
      }
      terminal_records_.erase(candidate);
    }
  }

  [[nodiscard]] bool terminal_result_for_generation(
    std::uint64_t generation,
    MotionConditioningResult & result) const
  {
    std::lock_guard<std::mutex> lock(teardown_mutex_);
    const auto found = terminal_records_.find(generation);
    if (found == terminal_records_.cend()) {
      return false;
    }
    result = found->second;
    return true;
  }

  [[nodiscard]] MotionConditioningResult stop_owned()
  {
    // A normal generation handover closes only this generation's renew and
    // producer side effects.  Health/collision subscriptions stay alive so a
    // subsequent prepare() on the same Adapter can prove fresh dependencies.
    // Explicit shutdown/destroy sets shutdown_ingress_requested_ and closes
    // all source ingress through begin_shutdown_ingress().
    disable_renew_callbacks();
    bool zero_proven = false;
    std::chrono::steady_clock::time_point zero_proven_at{};
    // Invalidation in begin_teardown() is the cancellation fence.  Prove
    // Gate zero before disabling/waiting for callbacks or an in-flight
    // component RPC so a concurrent STOP/Cancel cannot leave the producer
    // commanded while a 2s/4s start operation drains.
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      zero_proven = last_zero_proven_at_ !=
        std::chrono::steady_clock::time_point{} &&
      last_zero_proven_lease_id_ == lease_id_;
      if (zero_proven) {
        zero_proven_at = last_zero_proven_at_;
      }
    }
    if (!zero_proven) {
      zero_proven = inhibit_gate(true, &zero_proven_at);
    }
    if (shutdown_ingress_requested_.load(std::memory_order_acquire)) {
      wait_for_ingress_callbacks();
    }
    wait_for_renew_callbacks();
    // The emergency zero proof above is independent and immediate.  The
    // unique cleanup owner waits on the start-operation CV until every
    // in-flight lifecycle/RPC boundary has returned, then owns producer and
    // component cleanup.  A normal STOP has a bounded response barrier: if a
    // start operation is still inside an uncancellable external call, the
    // emergency zero is retained and destruction resumes the same cleanup
    // responsibility after that operation drains.
    if (!destroying_.load() && !wait_for_start_operations()) {
      cleanup_blocked_ = true;
      schedule_cleanup_continuation();
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      state_ = MotionConditioningState::Failed;
      auto result = make_result(
          state_, MotionConditioningFailure::SafetyFault, false,
          zero_proven,
          collision_stop_, lease_id_, candidate_topic_,
          "active start operation did not drain before teardown deadline");
      result.zero_proven_at = zero_proven_at;
      return remember(std::move(result));
    }
    drain_start_operations();
    const bool producer_stopped = safe_producer_stop();
    bool components_clean = false;
    try {
      if (!destroying_.load() && !zero_proven) {
        zero_proven = inhibit_gate(false, &zero_proven_at);
      }
      components_clean = cleanup_components();
    } catch (...) {
      components_clean = false;
      record_cleanup_failure(
        "cleanup", config_.container_fqn, 0U,
        "cleanup raised an unknown exception");
    }
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!producer_stopped || !zero_proven || !components_clean) {
      state_ = MotionConditioningState::Failed;
      auto result = make_result(
          state_, MotionConditioningFailure::SafetyFault, false,
          zero_proven && producer_stopped,
          collision_stop_, lease_id_, candidate_topic_,
          with_cleanup_failure(
            "conditioning stop could not prove zero and cleanup"));
      result.zero_proven_at = zero_proven_at;
      return remember(std::move(result));
    }
    reset_generation();
    state_ = MotionConditioningState::Stopped;
    auto result = make_result(
        state_, MotionConditioningFailure::None, true, true, collision_stop_,
      {}, {}, "conditioning generation stopped");
    result.zero_proven_at = zero_proven_at;
    return remember(std::move(result));
  }

  void finalize_destruction_cleanup()
  {
    drain_start_operations();
    if (cleanup_complete_.load() && producer_stop_proven_.load()) {
      return;
    }
    bool expected = false;
    if (!destruction_cleanup_claimed_.compare_exchange_strong(expected, true)) {
      return;
    }
    const bool producer_stopped = safe_producer_stop();
    bool components_clean = false;
    try {
      components_clean = cleanup_components();
    } catch (...) {
      components_clean = false;
      record_cleanup_failure(
        "cleanup", config_.container_fqn, 0U,
        "destruction cleanup raised an unknown exception");
    }
    cleanup_complete_.store(producer_stopped && components_clean);
  }

  [[nodiscard]] MotionConditioningResult fail_owned(
    MotionConditioningFailure failure,
    std::string detail,
    bool wait_for_callbacks)
  {
    // A failed generation cannot keep renewing or producing output while its
    // immutable zero/terminal cleanup is in flight.  Shared health/collision
    // ingress remains available for the next generation unless shutdown has
    // explicitly closed it.
    disable_renew_callbacks();
    if (wait_for_callbacks) {
      wait_for_renew_callbacks();
    }
    if (failure == MotionConditioningFailure::None) {
      failure = MotionConditioningFailure::InternalError;
    }
    bool zero_proven = false;
    std::chrono::steady_clock::time_point zero_proven_at{};
    zero_proven = inhibit_gate(true, &zero_proven_at);
    if (shutdown_ingress_requested_.load(std::memory_order_acquire)) {
      wait_for_ingress_callbacks();
    }
    if (!destroying_.load() && !wait_for_start_operations()) {
      cleanup_blocked_ = true;
      schedule_cleanup_continuation();
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      state_ = MotionConditioningState::Failed;
      auto result = make_result(
          state_,
          MotionConditioningFailure::SafetyFault,
          false, zero_proven,
          collision_stop_, lease_id_, candidate_topic_,
          zero_proven ?
          detail + "; active start operation did not drain before failure teardown" :
          detail + "; active start operation did not drain before failure teardown; "
          "Gate zero proof was unavailable");
      result.zero_proven_at = zero_proven_at;
      return remember(std::move(result));
    }
    drain_start_operations();
    const bool producer_stopped = safe_producer_stop();
    bool components_clean = false;
    try {
      if (!zero_proven) {
        zero_proven = inhibit_gate(false, &zero_proven_at);
      }
      components_clean = cleanup_components();
    } catch (...) {
      components_clean = false;
      record_cleanup_failure(
        "cleanup", config_.container_fqn, 0U,
        "cleanup raised an unknown exception");
    }
    if (!producer_stopped || !zero_proven) {
      failure = MotionConditioningFailure::SafetyFault;
      if (!producer_stopped) {
        detail += "; producer stop could not be proven";
      }
      if (!zero_proven) {
        detail += "; Gate zero proof was unavailable";
      }
    }
    const bool cleanup_residual =
      !components_clean || cleanup_failure_.has_value() ||
      cleanup_identity_fault_ || !pending_loads_.empty() ||
      !residual_components_.empty();
    if (cleanup_residual) {
      failure = MotionConditioningFailure::SafetyFault;
      detail += "; component/container/writer cleanup could not be proven";
    }
    detail = with_cleanup_failure(std::move(detail));
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_ = MotionConditioningState::Failed;
    auto result = make_result(
        state_, failure, false, zero_proven && producer_stopped,
        collision_stop_, lease_id_, candidate_topic_, std::move(detail));
    result.zero_proven_at = zero_proven_at;
    return remember(std::move(result));
  }

  void invalidate_activation_locked()
  {
    generation_ = ++generation_counter_;
    correlation_token_ = {};
    activation_in_progress_.store(false);
    activation_failed_.store(true);
    try {
      node_.get_node_graph_interface()->notify_graph_change();
    } catch (...) {
    }
  }

  [[nodiscard]] bool activation_current_locked(
    std::uint64_t generation,
    const std::string & expected_lease,
    const std::string & expected_candidate) const
  {
    return generation_ == generation &&
           state_ == MotionConditioningState::Prepared &&
           activation_in_progress_.load() && !activation_failed_.load() &&
           lease_id_ == expected_lease && candidate_topic_ == expected_candidate;
  }

  [[nodiscard]] bool activation_token_current(std::uint64_t generation) const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return activation_generation_current_locked(generation) &&
           !activation_failed_.load();
  }

  [[nodiscard]] bool activation_generation_current_locked(
    std::uint64_t generation) const
  {
    return generation_ == generation &&
           state_ == MotionConditioningState::Prepared &&
           activation_in_progress_.load();
  }

  [[nodiscard]] MotionConditioningResult stale_activation_result(
    std::uint64_t generation)
  {
    MotionConditioningResult terminal;
    if (terminal_result_for_generation(generation, terminal)) {
      auto result = make_result(
        terminal.state,
        terminal.failure == MotionConditioningFailure::None ?
        MotionConditioningFailure::SafetyFault : terminal.failure,
        false,
        terminal.zero_proven,
        terminal.collision_stop,
        terminal.lease_id,
        terminal.candidate_topic,
        "activation generation was cancelled before producer start");
      result.zero_proven_at = terminal.zero_proven_at;
      return result;
    }
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto result = last_result_;
    result.ok = false;
    result.failure = result.failure == MotionConditioningFailure::None ?
      MotionConditioningFailure::SafetyFault : result.failure;
    result.detail = "activation generation was cancelled before producer start";
    return result;
  }

  [[nodiscard]] MotionConditioningResult abort_activation(
    std::uint64_t generation,
    MotionConditioningFailure failure,
    std::string detail)
  {
    if (!activation_generation_current(generation)) {
      if (!teardown_in_progress_.load()) {
        wait_for_failure_completion();
      }
      return stale_activation_result(generation);
    }
    return fail_synchronously(failure, std::move(detail));
  }

  [[nodiscard]] bool activation_generation_current(std::uint64_t generation) const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return activation_generation_current_locked(generation);
  }

  [[nodiscard]] bool running_generation_current(std::uint64_t generation) const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return generation_ == generation &&
           state_ == MotionConditioningState::Running &&
           timer_enabled_.load();
  }

  [[nodiscard]] bool safe_producer_stop()
  {
    if (!producer_) {
      return true;
    }
    std::lock_guard<std::mutex> lock(producer_mutex_);
    if (producer_stop_proven_.load()) {
      return true;
    }
    try {
      producer_->stop();
      producer_stop_proven_.store(true);
      return true;
    } catch (...) {
      return false;
    }
  }

  void disable_ingress_callbacks()
  {
    if (callback_state_) {
      callback_state_->disable();
    }
    scan_subscription_.reset();
    odom_subscription_.reset();
    clock_subscription_.reset();
    collision_subscription_.reset();
    disable_renew_callbacks();
  }

  void wait_for_ingress_callbacks()
  {
    if (!callback_state_) {
      return;
    }
    if (callback_state_->active_count() != 0U && config_.before_callback_wait) {
      config_.before_callback_wait();
    }
    callback_state_->wait();
  }

  void wait_for_failure_completion()
  {
    std::unique_lock<std::mutex> lock(teardown_mutex_);
    teardown_cv_.wait(lock, [this]() {return !teardown_in_progress_.load();});
  }

  [[nodiscard]] bool begin_renew_callback()
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (!renew_callbacks_enabled_ || destroying_.load()) {
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
    timer_enabled_.store(true);
  }

  void disable_renew_callbacks()
  {
    timer_enabled_.store(false);
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      renew_callbacks_enabled_ = false;
    }
    renew_timer_.reset();
  }

  void wait_for_renew_callbacks()
  {
    std::function<void()> wait_hook;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      if (active_renew_callbacks_ != 0U) {
        wait_hook = config_.before_renew_wait;
      }
    }
    if (wait_hook) {
      wait_hook();
    }
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
    const auto expected_fqn = "/" + node_name;
    try {
      const auto rpc_deadline = std::min(
        overall_deadline,
        std::chrono::steady_clock::now() + config_.component_rpc_timeout);
      if (!load_client_->wait_for_service(remaining_until(rpc_deadline)) ||
        std::chrono::steady_clock::now() >= rpc_deadline)
      {
        record_cleanup_failure(
          "load", expected_fqn, 0U, "load service unavailable or deadline expired");
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
      cleanup_complete_.store(false);
      auto future = load_client_->async_send_request(request);
      pending_loads_.push_back(
        PendingLoad{generation_, expected_fqn, std::move(future)});
      auto & pending_future = pending_loads_.back().future;
      if (pending_future.wait_for(remaining_until(rpc_deadline)) !=
        std::future_status::ready)
      {
        record_cleanup_failure(
          "pending_load", expected_fqn, 0U, "load response remained unresolved");
        return false;
      }
      const auto response = pending_future.get();
      pending_loads_.pop_back();
      if (!response || !response->success) {
        record_cleanup_failure(
          "load_response", expected_fqn, 0U, "load response was unsuccessful");
        return false;
      }
      component.unique_id = response->unique_id;
      component.node_fqn = response->full_node_name;
      if (component.unique_id != 0U) {
        components_loaded_ = true;
      }
      if (component.unique_id == 0U) {
        record_cleanup_failure(
          "load_response", expected_fqn, 0U, "load response had no unique_id");
        return false;
      }
      if (component.node_fqn != expected_fqn) {
        cleanup_identity_fault_ = true;
        record_cleanup_failure(
          "load_fqn", component.node_fqn, component.unique_id,
          "expected " + expected_fqn);
        return false;
      }
      if (std::chrono::steady_clock::now() >= rpc_deadline) {
        record_cleanup_failure(
          "load_response", component.node_fqn, component.unique_id,
          "load response arrived after its deadline");
        return false;
      }
      return true;
    } catch (...) {
      record_cleanup_failure(
        "load", expected_fqn, 0U, "load request raised an unknown exception");
      return false;
    }
  }

  [[nodiscard]] bool call_unload(
    std::uint64_t unique_id,
    std::chrono::steady_clock::time_point overall_deadline)
  {
    try {
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
      const auto response = future.get();
      return response && response->success;
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool change_state(
    const std::string & node_fqn,
    std::uint8_t transition_id,
    std::chrono::steady_clock::time_point overall_deadline =
    std::chrono::steady_clock::time_point::max())
  {
    try {
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
        RCLCPP_ERROR(
          node_.get_logger(),
          "conditioning lifecycle transition service unavailable: node=%s transition=%u",
          node_fqn.c_str(), static_cast<unsigned int>(transition_id));
        return false;
      }
      auto request = std::make_shared<ChangeState::Request>();
      request->transition.id = transition_id;
      auto future = client->async_send_request(request);
      if (future.wait_for(remaining_until(rpc_deadline)) !=
        std::future_status::ready)
      {
        RCLCPP_ERROR(
          node_.get_logger(),
          "conditioning lifecycle transition response timed out: node=%s transition=%u",
          node_fqn.c_str(), static_cast<unsigned int>(transition_id));
        return false;
      }
      if (std::chrono::steady_clock::now() >= rpc_deadline) {
        return false;
      }
      const auto response = future.get();
      if (!response || !response->success) {
        RCLCPP_ERROR(
          node_.get_logger(),
          "conditioning lifecycle transition rejected: node=%s transition=%u",
          node_fqn.c_str(), static_cast<unsigned int>(transition_id));
      }
      return response && response->success;
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] std::uint8_t component_state(
    const std::string & node_fqn,
    std::chrono::steady_clock::time_point overall_deadline =
    std::chrono::steady_clock::time_point::max())
  {
    try {
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
      const auto response = future.get();
      return response ? response->current_state.id :
             lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    } catch (...) {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  [[nodiscard]] bool load_and_configure_components()
  {
    const auto cancelled = [this]() {
        return prepare_cancel_requested_.load();
      };
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
        parameter(
          "source_timeout",
          std::chrono::duration<double>(config_.collision_source_timeout).count()),
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
        parameter("scan.topic", config_.scan_topic),
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
      setup_failure_detail_ = "Collision Monitor load RPC failed";
      return false;
    }
    if (cancelled()) {
      setup_failure_detail_ = "conditioning PREPARE was cancelled after Collision Monitor load";
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
      setup_failure_detail_ = "Velocity Smoother load RPC failed";
      return false;
    }
    if (cancelled()) {
      setup_failure_detail_ = "conditioning PREPARE was cancelled after Velocity Smoother load";
      return false;
    }
    if (!change_state(
      kCollisionMonitorFqn,
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
      prepare_open_deadline_) ||
      cancelled() ||
      !change_state(
        kVelocitySmootherFqn,
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
        prepare_open_deadline_))
    {
      if (cancelled()) {
        setup_failure_detail_ = "conditioning PREPARE was cancelled during component configure";
        return false;
      }
      setup_failure_detail_ = "component configure RPC failed";
      return false;
    }
    if (cancelled()) {
      setup_failure_detail_ = "conditioning PREPARE was cancelled after component configure";
      return false;
    }
    if (component_state(kCollisionMonitorFqn, prepare_open_deadline_) !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
      cancelled() ||
      component_state(kVelocitySmootherFqn, prepare_open_deadline_) !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      if (cancelled()) {
        setup_failure_detail_ = "conditioning PREPARE was cancelled while checking component state";
        return false;
      }
      setup_failure_detail_ = "component did not reach inactive state after configure";
      return false;
    }
    if (cancelled()) {
      setup_failure_detail_ = "conditioning PREPARE was cancelled before writer binding";
      return false;
    }
    if (!pin_collision_writer(prepare_open_deadline_)) {
      setup_failure_detail_ = "collision state writer was not visible";
      return false;
    }
    if (cancelled()) {
      setup_failure_detail_ = "conditioning PREPARE was cancelled after writer binding";
      return false;
    }
    return true;
  }

  [[nodiscard]] bool collision_writer_gid_is_retired(const WriterGid & gid) const
  {
    return std::any_of(
      retired_collision_writer_gids_.cbegin(),
      retired_collision_writer_gids_.cend(),
      [&gid](const auto & retired_gid) {return same_gid(gid, retired_gid);});
  }

  [[nodiscard]] bool pin_collision_writer(
    std::chrono::steady_clock::time_point overall_deadline)
  {
    const auto graph_deadline = std::min(
      overall_deadline,
      std::chrono::steady_clock::now() + config_.writer_graph_timeout);
    rclcpp::Event::SharedPtr graph_event;
    try {
      graph_event = node_.get_graph_event();
    } catch (...) {
      return false;
    }
    while (std::chrono::steady_clock::now() < graph_deadline) {
      const auto publishers = node_.get_publishers_info_by_topic(
        config_.collision_state_topic);
      const auto writer = std::find_if(
        publishers.cbegin(), publishers.cend(), [this](const auto & endpoint) {
          const auto gid = endpoint_writer_gid(endpoint);
          return static_cast<rmw_endpoint_type_t>(endpoint.endpoint_type()) ==
                 RMW_ENDPOINT_PUBLISHER &&
                 endpoint.topic_type() == "nav2_msgs/msg/CollisionMonitorState" &&
                 endpoint.node_name() == "collision_monitor" &&
                 endpoint.node_namespace() == "/" &&
                 !gid_is_zero(gid) &&
                 !collision_writer_gid_is_retired(gid);
        });
      if (writer != publishers.cend()) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        collision_writer_gid_ = endpoint_writer_gid(*writer);
        collision_writer_generation_ = generation_;
        collision_writer_bound_ = true;
        return true;
      }
      if (prepare_cancel_requested_.load()) {
        return false;
      }
      try {
        node_.wait_for_graph_change(
          graph_event,
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            graph_deadline - std::chrono::steady_clock::now()));
        graph_event->check_and_clear();
      } catch (...) {
        return false;
      }
    }
    return false;
  }

  [[nodiscard]] bool candidate_writer_is_exact(
    const rclcpp::TopicEndpointInfo & endpoint) const
  {
    const auto gid = endpoint_writer_gid(endpoint);
    return static_cast<rmw_endpoint_type_t>(endpoint.endpoint_type()) ==
           RMW_ENDPOINT_PUBLISHER &&
           endpoint.topic_type() == "geometry_msgs/msg/TwistStamped" &&
           endpoint.node_name() == "collision_monitor" &&
           endpoint.node_namespace() == "/" && !gid_is_zero(gid);
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
    std::chrono::steady_clock::time_point overall_deadline,
    const bool strict_identity = false)
  {
    const auto list_fqn = config_.container_fqn + "/_container/list_nodes";
    try {
      const auto rpc_deadline = std::min(
        overall_deadline,
        std::chrono::steady_clock::now() + config_.component_rpc_timeout);
      if (!list_nodes_client_->wait_for_service(remaining_until(rpc_deadline))) {
        record_cleanup_failure(
          "list_nodes", list_fqn, 0U, "ListNodes service unavailable");
        return false;
      }
      auto future = list_nodes_client_->async_send_request(
        std::make_shared<ListNodes::Request>());
      if (future.wait_for(remaining_until(rpc_deadline)) !=
        std::future_status::ready)
      {
        record_cleanup_failure(
          "list_nodes", list_fqn, 0U, "ListNodes response timed out");
        return false;
      }
      const auto response = future.get();
      if (!response) {
        record_cleanup_failure(
          "list_nodes", list_fqn, 0U, "ListNodes returned no response");
        return false;
      }
      nodes.clear();
      if (strict_identity &&
        response->full_node_names.size() != response->unique_ids.size())
      {
        record_cleanup_failure(
          "list_nodes_identity", list_fqn, 0U,
          "ListNodes returned mismatched FQN and unique_id arrays");
        return false;
      }
      const auto count = std::min(
        response->full_node_names.size(), response->unique_ids.size());
      std::unordered_set<std::uint64_t> unique_ids;
      std::unordered_set<std::string> node_fqns;
      for (std::size_t index = 0U; index < count; ++index) {
        const auto unique_id = response->unique_ids[index];
        const auto & node_fqn = response->full_node_names[index];
        const bool allowed_fqn = node_fqn == kCollisionMonitorFqn ||
          node_fqn == kVelocitySmootherFqn;
        const bool unique_id_inserted = unique_ids.insert(unique_id).second;
        const bool node_fqn_inserted = node_fqns.insert(node_fqn).second;
        if (strict_identity &&
          (unique_id == 0U || !allowed_fqn ||
          !unique_id_inserted || !node_fqn_inserted))
        {
          record_cleanup_failure(
            "list_nodes_identity", node_fqn, unique_id,
            unique_id == 0U ? "ListNodes returned zero unique_id" :
            !allowed_fqn ? "ListNodes returned an unknown component FQN" :
            !unique_id_inserted ?
            "ListNodes returned a duplicate unique_id" :
            "ListNodes returned a duplicate component FQN");
          return false;
        }
        nodes.push_back(Component{unique_id, node_fqn});
      }
      if (std::chrono::steady_clock::now() > overall_deadline) {
        record_cleanup_failure(
          "list_nodes", list_fqn, 0U, "ListNodes response exceeded deadline");
        return false;
      }
      return true;
    } catch (...) {
      record_cleanup_failure(
        "list_nodes", list_fqn, 0U, "ListNodes request raised an exception");
      return false;
    }
  }

  [[nodiscard]] bool cleanup_startup_components(
    const std::vector<Component> & listed,
    const std::chrono::steady_clock::time_point overall_deadline)
  {
    if (!startup_gate_zero_proven()) {
      record_cleanup_failure(
        "startup_gate", config_.container_fqn, 0U,
        "MotionGate stopped proving inhibited zero during startup cleanup");
      return false;
    }
    std::vector<Component> ordered = listed;
    std::sort(
      ordered.begin(), ordered.end(), [](const Component & left, const Component & right) {
        return left.node_fqn < right.node_fqn;
      });
    for (const auto & component : ordered) {
      const auto state = component_state(component.node_fqn, overall_deadline);
      if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN) {
        record_cleanup_failure(
          "startup_lifecycle_state", component.node_fqn, component.unique_id,
          "GetState did not return a usable lifecycle state");
        return false;
      }
      if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        if (!change_state(
            component.node_fqn,
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
            overall_deadline))
        {
          record_cleanup_failure(
            "startup_lifecycle_deactivate", component.node_fqn,
            component.unique_id, "active component could not be deactivated");
          return false;
        }
        if (!change_state(
            component.node_fqn,
            lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
            overall_deadline))
        {
          record_cleanup_failure(
            "startup_lifecycle_cleanup", component.node_fqn,
            component.unique_id, "inactive component could not be cleaned up");
          return false;
        }
      } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        if (!change_state(
            component.node_fqn,
            lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
            overall_deadline))
        {
          record_cleanup_failure(
            "startup_lifecycle_cleanup", component.node_fqn,
            component.unique_id, "inactive component could not be cleaned up");
          return false;
        }
      } else if (
        state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
        state != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
      {
        record_cleanup_failure(
          "startup_lifecycle_state", component.node_fqn, component.unique_id,
          "component lifecycle state was not cleanup-safe");
        return false;
      }
    }

    if (!startup_gate_zero_proven()) {
      record_cleanup_failure(
        "startup_gate", config_.container_fqn, 0U,
        "MotionGate stopped proving inhibited zero before startup UnloadNode");
      return false;
    }
    for (const auto & component : ordered) {
      if (!call_unload(component.unique_id, overall_deadline)) {
        record_cleanup_failure(
          "startup_unload", component.node_fqn, component.unique_id,
          "UnloadNode failed for the discovered component");
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] bool startup_candidate_writer_visible() const
  {
    try {
      for (const auto & topic : node_.get_topic_names_and_types()) {
        const auto has_twist_stamped = std::find(
          topic.second.cbegin(), topic.second.cend(),
          "geometry_msgs/msg/TwistStamped") != topic.second.cend();
        if (!has_twist_stamped) {
          continue;
        }
        const auto publishers = node_.get_publishers_info_by_topic(topic.first);
        if (std::any_of(
            publishers.cbegin(), publishers.cend(),
            [](const auto & endpoint) {
              return static_cast<rmw_endpoint_type_t>(endpoint.endpoint_type()) ==
                     RMW_ENDPOINT_PUBLISHER &&
                     endpoint.topic_type() == "geometry_msgs/msg/TwistStamped" &&
                     endpoint.node_name() == "collision_monitor" &&
                     endpoint.node_namespace() == "/";
            }))
        {
          return true;
        }
      }
    } catch (...) {
      return true;
    }
    return false;
  }

  [[nodiscard]] bool startup_gate_zero_proven() const
  {
    try {
      return gate_snapshot_proves_zero(authority_->snapshot());
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool wait_for_startup_graph_change(
    const rclcpp::Event::SharedPtr & graph_event,
    const std::chrono::steady_clock::time_point deadline) const
  {
    const auto remaining = remaining_until(deadline);
    if (remaining.count() == 0) {
      return false;
    }
    try {
      node_.wait_for_graph_change(
        graph_event,
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::min(remaining, 10ms)));
      graph_event->check_and_clear();
      return true;
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool confirm_startup_graph_clean(
    const std::chrono::steady_clock::time_point deadline)
  {
    rclcpp::Event::SharedPtr graph_event;
    try {
      graph_event = node_.get_graph_event();
    } catch (...) {
      record_cleanup_failure(
        "startup_graph", config_.container_fqn, 0U,
        "could not create a graph event for residual confirmation");
      return false;
    }

    std::size_t consecutive_empty = 0U;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!startup_gate_zero_proven()) {
        record_cleanup_failure(
          "startup_gate", config_.container_fqn, 0U,
          "MotionGate stopped proving inhibited zero during residual confirmation");
        return false;
      }
      std::vector<Component> listed;
      if (!list_nodes(listed, deadline, true)) {
        return false;
      }
      const bool empty = listed.empty() && !startup_candidate_writer_visible();
      if (empty) {
        ++consecutive_empty;
        if (consecutive_empty >= 2U) {
          return true;
        }
      } else {
        consecutive_empty = 0U;
        if (!listed.empty() && !cleanup_startup_components(listed, deadline)) {
          return false;
        }
      }
      if (!wait_for_startup_graph_change(graph_event, deadline)) {
        break;
      }
    }
    record_cleanup_failure(
      "startup_graph", config_.container_fqn, 0U,
      "two consecutive empty ListNodes and writer-graph observations were not proven");
    return false;
  }

  [[nodiscard]] MotionConditioningResult reconcile_startup_owned()
  {
    cleanup_failure_.reset();
    cleanup_identity_fault_ = false;
    cleanup_blocked_ = false;
    const auto deadline = std::chrono::steady_clock::now() +
      config_.startup_reconciliation_timeout;
    std::chrono::steady_clock::time_point zero_proven_at{};
    const auto fail_startup = [this, &zero_proven_at](std::string detail) {
        cleanup_blocked_ = true;
        const auto zero_proven = zero_proven_at != std::chrono::steady_clock::time_point{};
        MotionConditioningResult result;
        {
          std::lock_guard<std::recursive_mutex> lock(mutex_);
          state_ = MotionConditioningState::Failed;
          result = remember(make_result(
              state_, MotionConditioningFailure::SafetyFault, false,
              zero_proven, false, {}, {}, with_cleanup_failure(std::move(detail))));
          result.zero_proven_at = zero_proven_at;
        }
        startup_reconcile_failed_ = true;
        startup_result_ = result;
        return result;
      };

    if (!inhibit_gate(false, &zero_proven_at)) {
      return fail_startup(
        "startup reconciliation could not prove an inhibited zero Gate state");
    }
    std::vector<Component> listed;
    if (!list_nodes(listed, deadline, true)) {
      return fail_startup("startup ListNodes identity discovery failed");
    }
    if (!listed.empty() && !cleanup_startup_components(listed, deadline)) {
      return fail_startup("startup component lifecycle or unload cleanup failed");
    }
    if (!confirm_startup_graph_clean(deadline)) {
      return fail_startup("startup component residual or candidate writer cleanup failed");
    }

    MotionConditioningResult result;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      result = remember(make_result(
          MotionConditioningState::Stopped, MotionConditioningFailure::None, true,
          true, false, {}, {}, "startup component reconciliation complete"));
      result.zero_proven_at = zero_proven_at;
      last_result_.zero_proven_at = zero_proven_at;
    }
    startup_reconciled_ = true;
    startup_result_ = result;
    return result;
  }

  [[nodiscard]] bool reconcile_pending_loads(
    std::chrono::steady_clock::time_point overall_deadline)
  {
    bool success = true;
    for (auto iterator = pending_loads_.begin();
      iterator != pending_loads_.end(); )
    {
      const auto remaining = remaining_until(overall_deadline);
      if (remaining.count() == 0 ||
        iterator->future.wait_for(remaining) != std::future_status::ready)
      {
        record_cleanup_failure(
          "pending_load", iterator->expected_fqn, 0U,
          "load response remained unresolved during reconciliation");
        success = false;
        ++iterator;
        continue;
      }
      try {
        const auto response = iterator->future.get();
        if (response && response->success && response->unique_id != 0U) {
          residual_components_.push_back(Component{
              response->unique_id, response->full_node_name});
          components_loaded_ = true;
          if (response->full_node_name != iterator->expected_fqn) {
            cleanup_identity_fault_ = true;
            record_cleanup_failure(
              "pending_fqn", response->full_node_name, response->unique_id,
              "expected " + iterator->expected_fqn);
            success = false;
          }
        } else {
          record_cleanup_failure(
            "pending_load", iterator->expected_fqn, 0U,
            "late load response was unsuccessful");
          success = false;
        }
      } catch (...) {
        record_cleanup_failure(
          "pending_load", iterator->expected_fqn, 0U,
          "late load response raised an exception");
        success = false;
      }
      iterator = pending_loads_.erase(iterator);
    }
    return success;
  }

  [[nodiscard]] bool reconcile_residual_nodes(
    std::chrono::steady_clock::time_point confirmation_deadline,
    std::unordered_set<std::uint64_t> & unloaded_ids)
  {
    std::size_t consecutive_absent = 0U;
    while (std::chrono::steady_clock::now() < confirmation_deadline) {
      (void)reconcile_pending_loads(confirmation_deadline);
      std::unordered_set<std::uint64_t> attempted_ids;
      for (auto iterator = residual_components_.begin();
        iterator != residual_components_.end(); )
      {
        if (unloaded_ids.find(iterator->unique_id) != unloaded_ids.cend()) {
          iterator = residual_components_.erase(iterator);
          continue;
        }
        if (!attempted_ids.insert(iterator->unique_id).second) {
          iterator = residual_components_.erase(iterator);
          continue;
        }
        const auto unload_deadline = std::chrono::steady_clock::now() +
          config_.component_rpc_timeout;
        if (call_unload(iterator->unique_id, unload_deadline)) {
          unloaded_ids.insert(iterator->unique_id);
          iterator = residual_components_.erase(iterator);
        } else {
          record_cleanup_failure(
            "unload", iterator->node_fqn, iterator->unique_id,
            "UnloadNode failed during residual reconciliation");
          ++iterator;
        }
      }
      std::vector<Component> listed;
      if (!list_nodes(listed, confirmation_deadline, true)) {
        return false;
      }
      bool target_present = false;
      for (const auto & node : listed) {
        if (node.unique_id == 0U) {
          target_present = true;
          continue;
        }
        if (unloaded_ids.find(node.unique_id) != unloaded_ids.cend()) {
          continue;
        }
        target_present = true;
        const auto known_residual = std::find_if(
          residual_components_.cbegin(), residual_components_.cend(),
          [&node](const auto & component) {
            return component.unique_id == node.unique_id;
          });
        if (known_residual == residual_components_.cend()) {
          residual_components_.push_back(node);
        }
        if (!attempted_ids.insert(node.unique_id).second) {
          continue;
        }
        const auto unload_deadline = std::chrono::steady_clock::now() +
          config_.component_rpc_timeout;
        if (call_unload(node.unique_id, unload_deadline)) {
          unloaded_ids.insert(node.unique_id);
          residual_components_.erase(
            std::remove_if(
              residual_components_.begin(), residual_components_.end(),
              [&node](const auto & component) {
                return component.unique_id == node.unique_id;
              }),
            residual_components_.end());
        } else {
          record_cleanup_failure(
            "unload", node.node_fqn, node.unique_id,
            "UnloadNode failed during graph reconciliation");
        }
      }
      if (pending_loads_.empty() && residual_components_.empty() &&
        !target_present)
      {
        ++consecutive_absent;
        if (consecutive_absent >= 2U) {
          return true;
        }
      } else {
        consecutive_absent = 0U;
      }
      const auto remaining = confirmation_deadline -
        std::chrono::steady_clock::now();
      const auto sleep_duration = std::min(
        10ms,
        std::chrono::duration_cast<std::chrono::milliseconds>(remaining));
      std::this_thread::sleep_for(sleep_duration);
    }
    return false;
  }

  [[nodiscard]] bool cleanup_components()
  {
    if (!components_loaded_ && pending_loads_.empty() &&
      residual_components_.empty())
    {
      cleanup_complete_.store(true);
      return true;
    }
    bool success = true;
    std::unordered_set<std::uint64_t> unloaded_ids;
    const auto pending_deadline = std::chrono::steady_clock::now() +
      config_.component_rpc_timeout;
    success = reconcile_pending_loads(pending_deadline) && success;

    std::vector<Component> listed;
    const auto list_deadline = std::chrono::steady_clock::now() +
      config_.component_rpc_timeout;
    const bool listed_ok = list_nodes(listed, list_deadline, true);
    success = listed_ok && success;

    std::vector<Component> actual_components;
    const auto add_unique = [&actual_components](const Component & component) {
        if (component.unique_id == 0U) {
          return;
        }
        const auto found = std::find_if(
          actual_components.cbegin(), actual_components.cend(),
          [&component](const Component & existing) {
            return existing.unique_id == component.unique_id;
          });
        if (found == actual_components.cend()) {
          actual_components.push_back(component);
        }
      };
    add_unique(collision_component_);
    add_unique(smoother_component_);
    for (const auto & component : residual_components_) {
      add_unique(component);
    }
    if (listed_ok) {
      for (const auto & component : listed) {
        add_unique(component);
      }
    }

    if (listed_ok) {
      for (const auto & component : listed) {
        if (component.node_fqn != kCollisionMonitorFqn &&
          component.node_fqn != kVelocitySmootherFqn)
        {
          continue;
        }
        const auto current_state = component_state(
          component.node_fqn,
          std::chrono::steady_clock::now() + config_.component_rpc_timeout);
        if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          const bool deactivate_ok = change_state(
            component.node_fqn,
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
            std::chrono::steady_clock::now() + config_.component_rpc_timeout);
          if (!deactivate_ok) {
            record_cleanup_failure(
              "lifecycle_deactivate", component.node_fqn, component.unique_id,
              "active lifecycle deactivate failed");
          }
          success = deactivate_ok && success;
          if (deactivate_ok) {
            const bool cleanup_ok = change_state(
              component.node_fqn,
              lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
              std::chrono::steady_clock::now() + config_.component_rpc_timeout);
            if (!cleanup_ok) {
              record_cleanup_failure(
                "lifecycle_cleanup", component.node_fqn, component.unique_id,
                "inactive lifecycle cleanup failed after deactivate");
            }
            success = cleanup_ok && success;
          }
          continue;
        }
        if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          const bool cleanup_ok = change_state(
            component.node_fqn,
            lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
            std::chrono::steady_clock::now() + config_.component_rpc_timeout);
          if (!cleanup_ok) {
            record_cleanup_failure(
              "lifecycle_cleanup", component.node_fqn, component.unique_id,
              "inactive lifecycle cleanup failed");
          }
          success = cleanup_ok && success;
          continue;
        }
        if (
          current_state != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED &&
          current_state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN)
        {
          record_cleanup_failure(
            "lifecycle_state", component.node_fqn, component.unique_id,
            "component did not reach a cleanup-safe lifecycle state");
          success = false;
        }
      }
    }

    for (const auto & component : actual_components) {
      const auto unload_deadline = std::chrono::steady_clock::now() +
        config_.component_rpc_timeout;
      if (!call_unload(component.unique_id, unload_deadline)) {
        record_cleanup_failure(
          "unload", component.node_fqn, component.unique_id,
          "UnloadNode failed");
        const auto found = std::find_if(
          residual_components_.cbegin(), residual_components_.cend(),
          [&component](const Component & existing) {
            return existing.unique_id == component.unique_id;
          });
        if (found == residual_components_.cend()) {
          residual_components_.push_back(component);
        }
        success = false;
      } else {
        unloaded_ids.insert(component.unique_id);
        residual_components_.erase(
          std::remove_if(
            residual_components_.begin(), residual_components_.end(),
            [&component](const Component & existing) {
              return existing.unique_id == component.unique_id;
            }),
          residual_components_.end());
        if (collision_component_.unique_id == component.unique_id) {
          collision_component_ = {};
        }
        if (smoother_component_.unique_id == component.unique_id) {
          smoother_component_ = {};
        }
      }
    }

    const auto confirmation_deadline = std::chrono::steady_clock::now() +
      config_.writer_graph_timeout;
    success = reconcile_residual_nodes(
      confirmation_deadline, unloaded_ids) && success;
    success = !cleanup_identity_fault_ && success;
    const bool writer_gone = wait_for_writer_to_disappear(candidate_topic_);
    if (!writer_gone) {
      record_cleanup_failure(
        "writer_disappearance", candidate_topic_, 0U,
        "candidate writer remained visible after cleanup");
    }
    success = writer_gone && success;
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
    cleanup_complete_.store(success);
    return success;
  }

  [[nodiscard]] bool inhibit_gate(
    const bool emergency = false,
    std::chrono::steady_clock::time_point * zero_proven_at = nullptr)
  {
    try {
      std::string lease_id;
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        lease_id = lease_id_;
      }
      if (lease_id.empty()) {
        const bool proven = gate_snapshot_proves_zero(authority_->snapshot());
        if (proven) {
          const auto proven_at = std::chrono::steady_clock::now();
          if (zero_proven_at != nullptr) {
            *zero_proven_at = proven_at;
          }
          remember_zero_proof(proven_at, {});
        }
        return proven;
      }
      std::unique_lock<std::mutex> authority_lock(
        authority_call_mutex_, std::defer_lock);
      if (!emergency) {
        authority_lock.lock();
      } else {
        // A concurrent OPEN/RENEW may be inside the normal conditioning
        // serialization mutex.  The #35 MotionGate control service is itself
        // request-sequence linearized; emergency INHIBIT must be allowed to
        // race that bounded call so STOP/Cancel can fence the generation and
        // prove zero without waiting for the RPC to return.
        (void)authority_lock.try_lock();
      }
      const auto operation = make_operation(lease_id);
      const auto result = authority_->inhibit(operation);
      const bool proven = (!result.applied &&
        result.snapshot.gate_instance_id == operation.gate_instance_id &&
        gate_snapshot_proves_zero(result.snapshot)) ||
        (result.applied && result.zero_proven &&
        result.snapshot.gate_instance_id == operation.gate_instance_id &&
        gate_snapshot_proves_zero(result.snapshot));
      if (proven) {
        const auto proven_at = std::chrono::steady_clock::now();
        if (zero_proven_at != nullptr) {
          *zero_proven_at = proven_at;
        }
        remember_zero_proof(proven_at, operation.lease_id);
      }
      return proven;
    } catch (...) {
      return false;
    }
  }

  void remember_zero_proof(
    const std::chrono::steady_clock::time_point proven_at,
    std::string lease_id)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    last_zero_proven_at_ = proven_at;
    last_zero_proven_lease_id_ = std::move(lease_id);
  }

  [[nodiscard]] bool cleanup_generation(const std::string &)
  {
    if (cleanup_blocked_) {
      return false;
    }
    disable_renew_callbacks();
    wait_for_renew_callbacks();
    if (!wait_for_start_operations()) {
      cleanup_blocked_ = true;
      return false;
    }
    const bool producer_stopped = safe_producer_stop();
    const bool zero_proven = inhibit_gate();
    const bool clean = cleanup_components();
    if (!producer_stopped || !zero_proven || !clean) {
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

  void reset_generation(bool ready_for_new_generation = false)
  {
    if (collision_writer_bound_ && !gid_is_zero(collision_writer_gid_)) {
      retired_collision_writer_gids_.push_back(collision_writer_gid_);
    }
    collision_writer_bound_ = false;
    collision_writer_generation_ = 0U;
    collision_writer_gid_ = {};
    collision_token_.reset();
    components_loaded_ = false;
    collision_component_ = {};
    smoother_component_ = {};
    pending_loads_.clear();
    residual_components_.clear();
    cleanup_complete_.store(true);
    cleanup_identity_fault_ = false;
    cleanup_failure_.reset();
    last_zero_proven_at_ = {};
    last_zero_proven_lease_id_.clear();
    lease_id_.clear();
    candidate_topic_.clear();
    generation_request_id_.clear();
    prepare_open_deadline_ = {};
    if (ready_for_new_generation) {
      producer_stop_proven_.store(false);
    }
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

  [[nodiscard]] bool renew_for_activation(
    std::uint64_t generation,
    const std::string & expected_lease,
    const std::string & expected_gate_instance,
    TransactionLease * transaction_lease = nullptr)
  {
    std::optional<TransactionLease> owned_renew_lease;
    if (transaction_lease == nullptr) {
      owned_renew_lease = begin_side_effect(RuntimeTransactionSideEffect::Renew);
      if (!owned_renew_lease.has_value()) {
        return false;
      }
      transaction_lease = &*owned_renew_lease;
    }
    if (!transaction_lease->current() || !activation_token_current(generation)) {
      return false;
    }
    try {
      const auto controller_active = controller_is_active(prepare_open_deadline_);
      if (!controller_active) {
        set_activation_failure(
          "diff_drive_controller is not active during activation");
        return false;
      }
      if (!transaction_lease->current() || !activation_token_current(generation)) {
        return false;
      }
      AuthorityResult result;
      const auto renewed = transaction_lease->invoke(
        RuntimeTransactionSideEffect::Renew,
        [this, &expected_lease]() {
          std::lock_guard<std::mutex> authority_lock(authority_call_mutex_);
          return authority_->renew(make_operation(expected_lease));
        });
      if (!renewed.has_value()) {
        set_activation_failure("generation permit was revoked during activation RENEW");
        return false;
      }
      result = std::move(*renewed);
      if (!transaction_lease->current() || !activation_token_current(generation) ||
        !result.applied || !result.snapshot.authority_live ||
        result.snapshot.gate_instance_id != expected_gate_instance ||
        result.snapshot.lease_id != expected_lease ||
        result.snapshot.motion_inhibited)
      {
        set_activation_failure("MotionGate RENEW failed during activation");
        return false;
      }
      if (owned_renew_lease.has_value() &&
        !owned_renew_lease->commit([]() {return true;}))
      {
        set_activation_failure("generation permit was revoked after activation RENEW");
        return false;
      }
      return true;
    } catch (const std::exception & error) {
      set_activation_failure(
        std::string{"MotionGate RENEW raised during activation: "} + error.what());
    } catch (...) {
      set_activation_failure("MotionGate RENEW raised during activation");
    }
    return false;
  }

  void on_renew()
  {
    MotionConditioningCorrelationToken callback_token;
    std::optional<TransactionLease> renew_lease;
    try {
      RenewCallbackGuard callback_guard(*this);
      if (!callback_guard.active_) {
        return;
      }
      if (config_.before_renew_callback) {
        config_.before_renew_callback();
      }
      bool activation = false;
      bool collision_stop = false;
      bool running = false;
      std::optional<MotionConditioningCorrelationToken> collision_token;
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        activation = activation_in_progress_.load();
        callback_token = correlation_token_;
        collision_stop = collision_stop_;
        collision_token = collision_token_;
        running = state_ == MotionConditioningState::Running;
      }
      if (!activation && !running) {
        return;
      }
      // The synchronous activation transaction owns the bounded Gate
      // handover.  A renew callback must not race it with a second control
      // sequence; the first running callback starts once the generation is
      // committed.
      if (activation) {
        return;
      }
      renew_lease = begin_side_effect(RuntimeTransactionSideEffect::Renew);
      if (!renew_lease.has_value() || !renew_lease->current() ||
        !running_generation_current(callback_token.generation))
      {
        return;
      }
      if (collision_stop) {
        if (!renew_lease->current() ||
          !running_generation_current(callback_token.generation))
        {
          return;
        }
        fail_from_renew_callback(
          collision_token.value_or(callback_token),
          MotionConditioningFailure::ExecutionFailed,
          "Collision Monitor reported STOP for stop_zone");
        renew_lease->reject();
        return;
      }
      const auto health = runtime_graph_health(
        std::chrono::steady_clock::now() + config_.health_rpc_timeout,
        {}, false);
      if (!health.healthy()) {
        if (!renew_lease->current() ||
          !running_generation_current(callback_token.generation))
        {
          return;
        }
        fail_from_renew_callback(
          callback_token,
          failure_for_health(health.reason),
          health.detail.empty() ?
          "conditioning component or dependency graph is unhealthy" :
          health.detail);
        renew_lease->reject();
        return;
      }
      if (!renew_lease->current() ||
        !running_generation_current(callback_token.generation))
      {
        return;
      }
      AuthorityResult result;
      AuthorityOperation renew_operation;
      const auto renewed = renew_lease->invoke(
        RuntimeTransactionSideEffect::Renew,
        [this, &callback_token, &renew_operation]() {
          std::lock_guard<std::mutex> authority_lock(authority_call_mutex_);
          renew_operation = make_operation(callback_token.lease_id);
          return authority_->renew(renew_operation);
        });
      if (!renewed.has_value()) {
        if (renew_lease->side_effect_executed()) {
          fail_from_renew_callback(
            callback_token,
            MotionConditioningFailure::SafetyFault,
            "MotionGate RENEW returned without a guarded outcome");
          renew_lease->reject();
        }
        return;
      }
      result = std::move(*renewed);
      if (!result.applied || !result.snapshot.authority_live ||
        result.snapshot.gate_instance_id != renew_operation.gate_instance_id ||
        result.snapshot.lease_id != callback_token.lease_id ||
        result.snapshot.motion_inhibited)
      {
        if (renew_lease->side_effect_executed()) {
          fail_from_renew_callback(
            callback_token,
            MotionConditioningFailure::SafetyFault,
            "MotionGate RENEW failed closed");
          renew_lease->reject();
        }
        return;
      }
      const bool committed = renew_lease->commit([]() {return true;});
      if (!committed && renew_lease->side_effect_executed()) {
        fail_from_renew_callback(
          callback_token,
          MotionConditioningFailure::SafetyFault,
          "MotionGate RENEW was rejected by the generation fence");
        renew_lease->reject();
      }
    } catch (const std::exception & error) {
      if (renew_lease.has_value() && renew_lease->side_effect_executed()) {
        fail_from_renew_callback(
          callback_token,
          MotionConditioningFailure::SafetyFault,
          std::string{"MotionGate RENEW raised: "} + error.what());
        renew_lease->reject();
      }
    } catch (...) {
      if (renew_lease.has_value() && renew_lease->side_effect_executed()) {
        fail_from_renew_callback(
          callback_token,
          MotionConditioningFailure::SafetyFault,
          "MotionGate RENEW raised an unknown exception");
        renew_lease->reject();
      }
    }
  }

  [[nodiscard]] bool controller_is_active(
    std::chrono::steady_clock::time_point overall_deadline)
  {
    try {
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
      return response && std::any_of(
        response->controller.cbegin(), response->controller.cend(),
        [this](const auto & controller) {
          return controller.name == config_.controller_name &&
                 controller.state == "active";
        });
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool candidate_writer_is_visible(
    const std::string & candidate_topic,
    std::chrono::steady_clock::time_point overall_deadline,
    std::uint64_t generation)
  {
    const auto graph_deadline = std::min(
      overall_deadline,
      std::chrono::steady_clock::now() + config_.writer_graph_timeout);
    rclcpp::Event::SharedPtr graph_event;
    try {
      graph_event = node_.get_graph_event();
    } catch (...) {
      return false;
    }
    while (std::chrono::steady_clock::now() < graph_deadline) {
      if (!activation_token_current(generation)) {
        return false;
      }
      const auto publishers = node_.get_publishers_info_by_topic(candidate_topic);
      const auto writer = std::find_if(
        publishers.cbegin(), publishers.cend(),
        [this](const auto & endpoint) {return candidate_writer_is_exact(endpoint);});
      if (writer != publishers.cend()) {
        return true;
      }
      try {
        node_.wait_for_graph_change(
          graph_event,
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            graph_deadline - std::chrono::steady_clock::now()));
        graph_event->check_and_clear();
      } catch (...) {
        return false;
      }
    }
    return false;
  }

  enum class RuntimeHealthReason : std::uint8_t
  {
    Healthy,
    ScanSourceLost,
    OdomSourceLost,
    ClockSourceLost,
    ClockNotAdvancing,
    Deadline,
    ComponentUnavailable,
    CandidateWriterUnavailable,
    ControllerUnavailable,
  };

  struct RuntimeHealthAssessment
  {
    RuntimeHealthReason reason{RuntimeHealthReason::Healthy};
    std::string detail;

    [[nodiscard]] bool healthy() const
    {
      return reason == RuntimeHealthReason::Healthy;
    }
  };

  [[nodiscard]] static MotionConditioningFailure failure_for_health(
    RuntimeHealthReason reason)
  {
    switch (reason) {
      case RuntimeHealthReason::ScanSourceLost:
      case RuntimeHealthReason::OdomSourceLost:
      case RuntimeHealthReason::ClockSourceLost:
      case RuntimeHealthReason::ClockNotAdvancing:
        return MotionConditioningFailure::DependencyUnavailable;
      case RuntimeHealthReason::Deadline:
        return MotionConditioningFailure::SafetyFault;
      case RuntimeHealthReason::Healthy:
      case RuntimeHealthReason::ComponentUnavailable:
      case RuntimeHealthReason::CandidateWriterUnavailable:
      case RuntimeHealthReason::ControllerUnavailable:
      default:
        return MotionConditioningFailure::SafetyFault;
    }
  }

  [[nodiscard]] RuntimeHealthAssessment runtime_graph_health(
    std::chrono::steady_clock::time_point overall_deadline,
    const std::string & expected_candidate = {},
    bool check_component_states = true)
  {
    bool components_loaded = false;
    std::string candidate_topic = expected_candidate;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      components_loaded = components_loaded_;
      if (candidate_topic.empty()) {
        candidate_topic = candidate_topic_;
      }
    }
    if (!components_loaded) {
      return {
        RuntimeHealthReason::ComponentUnavailable,
        "conditioning components are not loaded"};
    }
    // OPEN performs the full lifecycle graph check.  During the running
    // renew path, the 100 ms health budget must also leave time for the
    // authority RPC; the candidate writer, dependency freshness, and
    // controller state below are the bounded live indicators.  A second
    // sequential lifecycle-state RPC here could consume the whole lease
    // renewal window before Gate RENEW is sent.
    if (check_component_states &&
      (component_state(kCollisionMonitorFqn, overall_deadline) !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
      component_state(kVelocitySmootherFqn, overall_deadline) !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE))
    {
      return {
        RuntimeHealthReason::ComponentUnavailable,
        "conditioning lifecycle component is not active"};
    }
    if (std::chrono::steady_clock::now() >= overall_deadline) {
      return {
        RuntimeHealthReason::Deadline,
        "conditioning health step exceeded its deadline"};
    }
    if (!node_.get_clock()->ros_time_is_active() ||
      node_.get_clock()->now().nanoseconds() <= 0)
    {
      return {
        RuntimeHealthReason::ClockSourceLost,
        "ROS clock is not active or has no positive sample"};
    }
    {
      std::lock_guard<std::mutex> lock(health_mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (!clock_seen_ || !clock_freshness_.fresh_at(now)) {
        return {
          RuntimeHealthReason::ClockSourceLost,
          "clock source is not fresh"};
      }
      if (!scan_freshness_.fresh_at(now)) {
        return {
          RuntimeHealthReason::ScanSourceLost,
          "scan source is not fresh"};
      }
      if (!odom_freshness_.fresh_at(now)) {
        return {
          RuntimeHealthReason::OdomSourceLost,
          "odom source is not fresh"};
      }
      if (!clock_progress_freshness_.fresh_at(now)) {
        return {
          RuntimeHealthReason::ClockNotAdvancing,
          "clock source has not advanced"};
      }
    }
    const auto candidate_publishers =
      node_.get_publishers_info_by_topic(candidate_topic);
    const bool candidate_writer = std::any_of(
      candidate_publishers.cbegin(), candidate_publishers.cend(),
      [this](const rclcpp::TopicEndpointInfo & endpoint) {
        return candidate_writer_is_exact(endpoint);
      });
    if (!candidate_writer) {
      return {
        RuntimeHealthReason::CandidateWriterUnavailable,
        "candidate writer is not visible"};
    }
    if (std::chrono::steady_clock::now() >= overall_deadline) {
      return {
        RuntimeHealthReason::Deadline,
        "conditioning health step exceeded its deadline"};
    }
    if (!controller_is_active(overall_deadline)) {
      if (std::chrono::steady_clock::now() >= overall_deadline) {
        return {
          RuntimeHealthReason::Deadline,
          "controller health step exceeded its deadline"};
      }
      return {
        RuntimeHealthReason::ControllerUnavailable,
        "diff_drive_controller is not active"};
    }
    return {RuntimeHealthReason::Healthy, {}};
  }

  rclcpp::Node & node_;
  std::shared_ptr<MotionAuthorityPort> authority_;
  std::shared_ptr<MotionProducerPort> producer_;
  MotionConditioningConfig config_;
  std::shared_ptr<RuntimeTransactionPlane> local_transaction_plane_;
  SteadySourceFreshness scan_freshness_;
  SteadySourceFreshness odom_freshness_;
  SteadySourceFreshness clock_freshness_;
  SteadySourceFreshness clock_progress_freshness_;
  std::function<std::string()> request_id_generator_;
  mutable std::recursive_mutex mutex_;
  std::mutex authority_call_mutex_;
  MotionConditioningState state_{MotionConditioningState::Stopped};
  MotionConditioningResult last_result_{};
  std::chrono::steady_clock::time_point last_zero_proven_at_{};
  std::string last_zero_proven_lease_id_;
  bool collision_stop_{false};
  std::optional<MotionConditioningCorrelationToken> collision_token_;
  bool components_loaded_{false};
  bool cleanup_blocked_{false};
  bool cleanup_identity_fault_{false};
  std::optional<CleanupFailureContext> cleanup_failure_;
  std::string setup_failure_detail_;
  bool collision_writer_bound_{false};
  std::uint64_t collision_writer_generation_{0U};
  WriterGid collision_writer_gid_{};
  std::vector<WriterGid> retired_collision_writer_gids_;
  Component collision_component_;
  Component smoother_component_;
  std::vector<PendingLoad> pending_loads_;
  std::vector<Component> residual_components_;
  std::string lease_id_;
  std::string candidate_topic_;
  std::chrono::steady_clock::time_point prepare_open_deadline_{};
  std::atomic<bool> activation_in_progress_{false};
  std::atomic<bool> activation_failed_{false};
  std::atomic<bool> destroying_{false};
  std::atomic<bool> shutdown_ingress_requested_{false};
  std::atomic<bool> cleanup_complete_{true};
  std::atomic<bool> destruction_cleanup_claimed_{false};
  std::atomic<bool> teardown_in_progress_{false};
  mutable std::mutex teardown_mutex_;
  std::condition_variable teardown_cv_;
  std::condition_variable start_operation_cv_;
  std::size_t active_start_operations_{0U};
  std::unordered_map<std::thread::id, std::size_t> start_operation_threads_;
  std::thread cleanup_continuation_thread_;
  bool cleanup_continuation_running_{false};
  MotionConditioningResult teardown_result_{};
  TeardownOwner teardown_owner_{TeardownOwner::None};
  std::atomic<bool> prepare_cancel_requested_{false};
  std::optional<TerminalRecord> terminal_record_;
  std::unordered_map<std::uint64_t, MotionConditioningResult> terminal_records_;
  std::uint64_t teardown_generation_{0U};
  mutable std::mutex activation_mutex_;
  std::string activation_failure_detail_;
  std::mutex callback_mutex_;
  std::condition_variable callback_cv_;
  std::size_t active_renew_callbacks_{0U};
  bool renew_callbacks_enabled_{false};
  std::shared_ptr<IngressCallbackState> callback_state_;
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
  std::int64_t last_clock_stamp_{0};
  bool clock_seen_{false};
  std::uint64_t generation_counter_{0U};
  std::uint64_t generation_{0U};
  std::string generation_request_id_;
  MotionConditioningCorrelationToken correlation_token_;
  std::mutex startup_mutex_;
  bool startup_reconciled_{false};
  bool startup_reconcile_failed_{false};
  MotionConditioningResult startup_result_{};
  std::mutex producer_mutex_;
  std::atomic<bool> producer_stop_proven_{false};
  std::atomic<bool> timer_enabled_{false};
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
  MotionConditioningCorrelationToken token,
  MotionConditioningFailure failure,
  std::string detail)
{
  return impl_->fail(std::move(token), failure, std::move(detail));
}

MotionConditioningCorrelationToken MotionConditioningPipeline::correlation_token() const
{
  return impl_->correlation_token();
}

MotionConditioningState MotionConditioningPipeline::state() const noexcept
{
  return impl_->state();
}

MotionConditioningResult MotionConditioningPipeline::last_result() const
{
  return impl_->last_result();
}

void MotionConditioningPipeline::begin_shutdown_ingress() noexcept
{
  if (impl_) {
    impl_->begin_shutdown_ingress();
  }
}

void detail::begin_motion_conditioning_shutdown(
  MotionConditioningPipeline & pipeline) noexcept
{
  pipeline.begin_shutdown_ingress();
}

MotionConditioningResult detail::reconcile_motion_conditioning_startup(
  MotionConditioningPipeline & pipeline)
{
  return pipeline.impl_->reconcile_startup();
}

}  // namespace voice_nav_mission
