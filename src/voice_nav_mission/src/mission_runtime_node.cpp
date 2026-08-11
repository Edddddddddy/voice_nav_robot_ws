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

#include "voice_nav_mission/mission_runtime_core.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "voice_nav_interfaces/action/execute_mission.hpp"
#include "voice_nav_interfaces/msg/mission_state.hpp"
#include "voice_nav_interfaces/msg/mission_step.hpp"
#include "voice_nav_interfaces/srv/stop_mission.hpp"
#include "voice_nav_mission/action_admission_tracker.hpp"
#include "voice_nav_mission/mission_action_result_router.hpp"
#include "voice_nav_mission/motion_authority_ros_adapter.hpp"
#include "voice_nav_mission/rapid_mission_ros_adapters.hpp"
#include "voice_nav_mission/relative_motion_ros_adapter.hpp"
#include "voice_nav_mission/runtime_admission_gate.hpp"
#include "voice_nav_mission/runtime_emergency_fence.hpp"
#include "voice_nav_mission/runtime_execution_plane.hpp"
#include "voice_nav_mission/runtime_event_ingress.hpp"
#include "voice_nav_mission/runtime_event_queue.hpp"
#include "voice_nav_mission/runtime_shutdown_coordinator.hpp"

namespace voice_nav_mission
{
namespace
{

using ExecuteMission = voice_nav_interfaces::action::ExecuteMission;
using StopMission = voice_nav_interfaces::srv::StopMission;
using MissionStateMessage = voice_nav_interfaces::msg::MissionState;
using MissionStepMessage = voice_nav_interfaces::msg::MissionStep;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMission>;
using GoalUUID = rclcpp_action::GoalUUID;

// Shared by the rclcpp_action server callbacks and the shutdown coordinator.
// Each handler runs while the lifetime mutex is held, so shutdown cannot
// destroy Node-owned state while an already-entered callback still uses it.
class ActionCallbackLifetime final
{
public:
  using GoalHandler = std::function<rclcpp_action::GoalResponse(
        const GoalUUID &, std::shared_ptr<const ExecuteMission::Goal>)>;
  using CancelHandler = std::function<rclcpp_action::CancelResponse(
        const std::shared_ptr<GoalHandle> &)>;
  using AcceptedHandler = std::function<void(const std::shared_ptr<GoalHandle> &)>;

  ActionCallbackLifetime(
    GoalHandler goal_handler,
    CancelHandler cancel_handler,
    AcceptedHandler accepted_handler)
  : goal_handler_(std::move(goal_handler)),
    cancel_handler_(std::move(cancel_handler)),
    accepted_handler_(std::move(accepted_handler))
  {
  }

  [[nodiscard]] rclcpp_action::GoalResponse on_goal(
    const GoalUUID & uuid,
    std::shared_ptr<const ExecuteMission::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || !goal_handler_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return goal_handler_(uuid, std::move(goal));
  }

  [[nodiscard]] rclcpp_action::CancelResponse on_cancel(
    const std::shared_ptr<GoalHandle> & goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || !cancel_handler_) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    return cancel_handler_(goal_handle);
  }

  void on_accepted(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_ && accepted_handler_) {
      accepted_handler_(goal_handle);
      return;
    }
    // The provisional ticket is revoked by ActionAdmissionTracker during the
    // bounded shutdown drain.  Jazzy does not expose a transport handoff
    // guarantee for a callback that has not entered production on_accepted;
    // do not fabricate a Result for that no-handle path.
  }

  void deactivate() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    goal_handler_ = {};
    cancel_handler_ = {};
    accepted_handler_ = {};
  }

  [[nodiscard]] bool active() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
  }

private:
  mutable std::mutex mutex_;
  GoalHandler goal_handler_;
  CancelHandler cancel_handler_;
  AcceptedHandler accepted_handler_;
  bool active_{true};
};

constexpr char kExecuteAction[] = "/mission/execute";
constexpr char kStopService[] = "/mission/stop";
constexpr char kStateTopic[] = "/mission/state";
constexpr std::int64_t kTrustedMissionDeadlineMs = 30000;
constexpr std::int64_t kTrustedGateDiscoveryDeadlineMs = 2000;
constexpr std::int64_t kTrustedControlResponseDeadlineMs = 100;
constexpr std::int64_t kTrustedStopBarrierMs = 250;
constexpr std::int64_t kTrustedCancelGraceMs = 250;
constexpr std::int64_t kTrustedStationarityDeadlineMs = 1200;
constexpr std::int64_t kTrustedCollisionSourceTimeoutMs = 300;
constexpr float kTrustedMoveDistanceMinM = 0.05F;
constexpr float kTrustedMoveDistanceMaxM = 2.0F;
constexpr float kTrustedRotateAngleMinRad = 0.05F;
constexpr float kTrustedRotateAngleMaxRad = 6.283185F;

struct StopWaiter
{
  std::mutex mutex;
  std::condition_variable condition;
  bool completed{false};
  StopResponse response{};
};

struct AdmitEvent
{
  MissionGoal goal;
  std::shared_ptr<GoalHandle> goal_handle;
};

struct CancelEvent
{
  std::uint64_t mission_id{0U};
};

struct StopEvent
{
  StopRequest request;
  std::shared_ptr<StopWaiter> waiter;
};

struct TickEvent
{
  SteadyClockPort::TimePoint now{};
};

struct GateSnapshotEvent
{
  GateSnapshot snapshot;
};

struct ChildFeedbackEvent
{
  MotionToken token;
  double progress{0.0};
};

struct ChildResultEvent
{
  MotionToken token;
};

struct QueueFaultEvent
{
  std::string detail;
};

using RuntimeEventPayload = std::variant<
  AdmitEvent,
  CancelEvent,
  StopEvent,
  TickEvent,
  GateSnapshotEvent,
  ChildFeedbackEvent,
  ChildResultEvent,
  QueueFaultEvent>;

struct RuntimeEvent
{
  std::uint64_t generation{0U};
  RuntimeEventPayload payload;
};

using RuntimeEventQueueType = RuntimeEventQueue<RuntimeEvent>;

[[nodiscard]] RuntimeEventQueueType::Lane runtime_event_lane(
  const RuntimeEvent & event) noexcept
{
  return std::holds_alternative<CancelEvent>(event.payload) ||
         std::holds_alternative<StopEvent>(event.payload) ||
         std::holds_alternative<ChildResultEvent>(event.payload) ||
         std::holds_alternative<QueueFaultEvent>(event.payload) ?
         RuntimeEventQueueType::Lane::Control : RuntimeEventQueueType::Lane::Normal;
}

class RosSteadyClock final : public SteadyClockPort
{
public:
  [[nodiscard]] TimePoint now() const override
  {
    return std::chrono::steady_clock::now();
  }
};

// Explicitly non-safe authority used only when rapid_delegate_action is set.
// It preserves Runtime sequencing without creating a second velocity writer;
// the delegated rapid bridge remains the sole simulation command source.
class RapidUnsafeMotionAuthority final : public MotionAuthorityPort
{
public:
  RapidUnsafeMotionAuthority()
  : snapshot_{
      "rapid-unsafe-authority", 1U, "", GateState::Inhibited,
      true, true, true, true, "", true, true}
  {
  }

  [[nodiscard]] GateSnapshot snapshot() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

  [[nodiscard]] AuthorityResult prepare(
    const AuthorityOperation &) override
  {
    return transition(GateState::Prepared, true, std::string(32U, 'r'));
  }

  [[nodiscard]] AuthorityResult open(const AuthorityOperation &) override
  {
    return transition(GateState::Armed, false, snapshot().lease_id);
  }

  [[nodiscard]] AuthorityResult renew(const AuthorityOperation &) override
  {
    const auto current = snapshot();
    return AuthorityResult{
      current.state == GateState::Armed,
      false,
      false,
      current,
      current.lease_id,
      "rapid unsafe authority renewed",
      false};
  }

  [[nodiscard]] AuthorityResult inhibit(const AuthorityOperation &) override
  {
    return transition(GateState::Inhibited, true, "");
  }

private:
  [[nodiscard]] AuthorityResult transition(
    const GateState state, const bool inhibited, std::string lease)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++snapshot_.control_seq;
    snapshot_.state = state;
    snapshot_.lease_id = std::move(lease);
    snapshot_.motion_inhibited = inhibited;
    snapshot_.zero_selected = inhibited;
    snapshot_.zero_published = inhibited;
    return AuthorityResult{
      true,
      inhibited,
      false,
      snapshot_,
      snapshot_.lease_id,
      "rapid unsafe authority applied",
      false};
  }

  mutable std::mutex mutex_;
  GateSnapshot snapshot_;
};

}  // namespace

class MissionRuntimeNode final : public rclcpp::Node
{
public:
  explicit MissionRuntimeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mission_runtime_node", options),
    clock_(std::make_shared<RosSteadyClock>()),
    action_admission_tracker_([this]() {return clock_->now();}),
    admission_gate_(),
    event_queue_([]() {
        return RuntimeEvent{0U, QueueFaultEvent{"Runtime event queue overflow"}};
      }),
    emergency_fence_(1U),
    event_ingress_(
      event_queue_,
      emergency_fence_,
      runtime_event_lane,
      [this]() {request_independent_emergency();},
      [this](const RuntimeEmergencyFenceSnapshot & snapshot) {
        if (execution_plane_ && execution_plane_->core()) {
          std::lock_guard<std::recursive_mutex> lock(
            execution_plane_->core_serial_mutex());
          execution_plane_->core()->fail_closed_at_epoch(
            snapshot.admission_epoch, snapshot.detail);
        }
        // Keep completion cleanup independently wakeable even after this
        // one-shot fence has been consumed by the runtime worker.
        if (execution_plane_) {
          execution_plane_->completion_mailbox().request_reap();
        }
      },
      [](const RuntimeEvent & event) {
        return std::holds_alternative<CancelEvent>(event.payload) ||
               std::holds_alternative<StopEvent>(event.payload);
      })
  {
    if (std::string(get_fully_qualified_name()) != "/mission_runtime_node") {
      throw std::runtime_error("mission_runtime_node must run at /mission_runtime_node");
    }
    config_ = load_config();
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    state_qos.reliable().transient_local();
    state_publisher_ = create_publisher<MissionStateMessage>(kStateTopic, state_qos);
    const auto rapid_delegate_action = declare_parameter<std::string>(
      "rapid_delegate_action", "");
    RuntimePorts runtime_ports;
    if (!rapid_delegate_action.empty()) {
      authority_ = std::make_shared<RapidUnsafeMotionAuthority>();
      auto delegate = std::make_shared<RapidMissionDelegate>(
        *this, rapid_delegate_action);
      runtime_ports.relative_motion =
        std::make_shared<RapidRelativeMotionPort>(delegate);
      if (config_.operating_mode == OperatingMode::Navigation) {
        runtime_ports.navigation =
          std::make_shared<RapidNavigationPort>(delegate);
      } else {
        runtime_ports.map_store =
          std::make_shared<RapidMapStorePort>(delegate);
      }
    } else {
      ros_authority_ = std::make_shared<RosMotionAuthorityPort>(
        *this,
        config_.control_response_deadline,
        config_.stop_barrier,
        [this](const GateSnapshot & snapshot) {
          (void)enqueue_internal_event(
            RuntimeEvent{0U, GateSnapshotEvent{snapshot}});
        });
      authority_ = ros_authority_;
      RelativeMotionPolicy motion_policy;
      motion_policy.stationarity_deadline = config_.stationarity_deadline;
      MotionConditioningConfig conditioning_config;
      conditioning_config.stop_barrier = config_.stop_barrier;
      conditioning_config.collision_source_timeout = std::chrono::milliseconds(
        kTrustedCollisionSourceTimeoutMs);
      conditioning_config.transaction_plane = admission_gate_.transaction_plane();
      conditioning_config.admission_fence_check =
        [this](const std::uint64_t epoch) {
          return admission_gate_.admission_allowed(
            epoch,
            [this](const std::uint64_t value) {
              return event_ingress_.admission_allowed(value);
            });
        };
      conditioning_config.completion_relay =
        [this](RelativeMotionCompletionRecordPtr record) {
          return execution_plane_ &&
                 execution_plane_->completion_mailbox().relay(std::move(record));
        };
      relative_motion_ = std::make_shared<RelativeMotionRosAdapter>(
        *this, authority_, motion_policy, conditioning_config);
      runtime_ports.relative_motion = relative_motion_;
    }
    const auto initial_gate_snapshot = authority_->snapshot();
    execution_plane_ = std::make_unique<RuntimeExecutionPlane>(
      config_,
      clock_,
      authority_,
      std::move(runtime_ports),
      [this](const RuntimeState & state) {publish_state(state);},
      [this](std::uint64_t mission_id, const MissionFeedback & feedback) {
        publish_feedback(mission_id, feedback);
      },
      [this](std::uint64_t mission_id, const MissionResult & result) {
        action_adapter_.finish(mission_id, result);
      },
      [this](const MotionToken & token, const double progress) {
        return enqueue_internal_event(RuntimeEvent{
          token.mission_generation, ChildFeedbackEvent{token, progress}});
      },
      [this](const std::uint64_t epoch) {
        return admission_gate_.admission_allowed(
          epoch,
          [this](const std::uint64_t value) {
            return event_ingress_.admission_allowed(value);
          });
      },
      [this](const MotionToken & token) {
        return enqueue_internal_event(RuntimeEvent{
          token.mission_generation, ChildResultEvent{token}});
      },
      [this](std::string detail) {
        request_emergency_fence(std::move(detail));
      });
    shutdown_coordinator_ = std::make_unique<RuntimeShutdownCoordinator>(
      [this]() {
        return admission_gate_.close_generation(action_admission_tracker_);
      },
      [this](const RuntimeShutdownCoordinator::TimePoint deadline) {
        timer_.reset();
        stop_service_.reset();
        if (relative_motion_) {
          detail::begin_relative_motion_shutdown(*relative_motion_, deadline);
        }
      },
      [this](const RuntimeShutdownCoordinator::TimePoint deadline) {
        return wait_for_shutdown_conditions(deadline);
      },
      [this]() {request_independent_emergency();},
      [this](std::string detail) {request_emergency_fence(std::move(detail));},
      [this](std::string detail) {
        if (execution_plane_ && execution_plane_->core()) {
          std::lock_guard<std::recursive_mutex> lock(
            execution_plane_->core_serial_mutex());
          execution_plane_->core()->fail_closed_at_epoch(
            emergency_fence_.admission_epoch(), std::move(detail));
        }
      });
    runtime_worker_ = std::thread([this]() {run_runtime_events();});
    (void)enqueue_event(RuntimeEvent{
        0U, GateSnapshotEvent{initial_gate_snapshot}});

    action_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    action_callback_lifetime_ = std::make_shared<ActionCallbackLifetime>(
      [this](const GoalUUID & uuid, std::shared_ptr<const ExecuteMission::Goal> goal) {
        return on_goal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandle> & goal_handle) {
        return on_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandle> & goal_handle) {
        on_accepted(goal_handle);
      });
    action_server_ = rclcpp_action::create_server<ExecuteMission>(
      this,
      kExecuteAction,
      [lifetime = action_callback_lifetime_](
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteMission::Goal> goal) {
        return lifetime->on_goal(uuid, std::move(goal));
      },
      [lifetime = action_callback_lifetime_](
        const std::shared_ptr<GoalHandle> goal_handle) {
        return lifetime->on_cancel(goal_handle);
      },
      [lifetime = action_callback_lifetime_](
        const std::shared_ptr<GoalHandle> goal_handle) {
        lifetime->on_accepted(goal_handle);
      },
      rcl_action_server_get_default_options(),
      action_callback_group_);
    stop_service_ = create_service<StopMission>(
      kStopService,
      [this](
        const std::shared_ptr<StopMission::Request> request,
        std::shared_ptr<StopMission::Response> response) {
        on_stop(*request, *response);
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {
        const auto now = clock_->now();
        (void)action_admission_tracker_.revoke_expired(now);
        (void)enqueue_event(RuntimeEvent{0U, TickEvent{now}});
      });
    shutdown_context_ = get_node_base_interface()->get_context();
    shutdown_callback_handle_ = shutdown_context_->add_pre_shutdown_callback(
      [this]() {shutdown_barrier();});
  }

  ~MissionRuntimeNode() override
  {
    shutdown_barrier();
    if (shutdown_context_) {
      (void)shutdown_context_->remove_pre_shutdown_callback(
        shutdown_callback_handle_);
      shutdown_context_.reset();
    }
  }

private:
  [[nodiscard]] static std::string goal_uuid_key(const GoalUUID & uuid)
  {
    return std::string(
      reinterpret_cast<const char *>(uuid.data()), uuid.size());
  }

  [[nodiscard]] bool wait_for_action_admission_drain() noexcept
  {
    (void)action_admission_tracker_.revoke_expired(clock_->now());
    const auto handoff_deadline = std::chrono::steady_clock::now() +
      ActionAdmissionTracker::kDefaultHandoffDeadline;
    if (action_admission_tracker_.wait_for_drain_until(handoff_deadline)) {
      return true;
    }
    // A response may have been sent without an on_accepted callback.  Revoke
    // those provisional tickets at the fixed bound, then give an already
    // entered callback one separate short drain window.
    (void)action_admission_tracker_.revoke_all_provisional(clock_->now());
    const auto callback_deadline = std::chrono::steady_clock::now() +
      ActionAdmissionTracker::kDefaultHandoffDeadline;
    return action_admission_tracker_.wait_for_drain_until(callback_deadline);
  }

  void shutdown_barrier() noexcept
  {
    {
      std::unique_lock<std::mutex> lock(shutdown_mutex_);
      if (shutdown_barrier_complete_) {
        return;
      }
      if (shutdown_barrier_started_) {
        if (shutdown_barrier_owner_ != std::this_thread::get_id()) {
          shutdown_condition_.wait(lock, [this]() {
              return shutdown_barrier_complete_;
            });
        }
        return;
      }
      shutdown_barrier_started_ = true;
      shutdown_barrier_owner_ = std::this_thread::get_id();
    }

    const auto shutdown_deadline = std::chrono::steady_clock::now() +
      config_.stop_barrier + config_.stationarity_deadline;
    const auto shutdown_outcome = shutdown_coordinator_ ?
      shutdown_coordinator_->run(shutdown_deadline) :
      RuntimeShutdownCoordinator::Outcome{false, true};
    if (!shutdown_outcome.transaction_drained) {
      action_shutdown_fail_closed_ = true;
    }
    if (action_callback_lifetime_) {
      action_callback_lifetime_->deactivate();
    }
    // The lifetime mutex and admission lease cover callbacks that have already
    // entered production on_accepted.  A provisional ticket without that
    // handoff is revoked at the fixed bound and has no fabricated Result.
    const auto action_admission_drained = wait_for_action_admission_drain();
    if (!action_admission_drained) {
      action_shutdown_fail_closed_ = true;
      request_independent_emergency();
      request_emergency_fence(
        "Action admission drain deadline expired; no-handle Result is not "
        "guaranteed; SAFETY_FAULT");
      RCLCPP_ERROR(
        get_logger(),
        "Action callback drain deadline expired; closing without fabricating a "
        "no-handle Result and retaining fail-closed state");
    }
    event_queue_.close();
    if (runtime_worker_.joinable()) {
      runtime_worker_.join();
    }
    // An accepted production callback has already completed its handoff before
    // this point; the runtime worker has also delivered active GoalHandles.
    // If only a provisional/no-handle tombstone missed the bound, close the
    // shared transport state without claiming that a client Result can still
    // be delivered after context shutdown.
    action_server_.reset();
    action_admission_tracker_.clear();
    action_callback_lifetime_.reset();
    if (relative_motion_) {
      relative_motion_->finalize_shutdown();
    }
    if (execution_plane_) {
      execution_plane_->shutdown();
      execution_plane_.reset();
    }
    relative_motion_.reset();
    ros_authority_.reset();
    authority_.reset();

    {
      std::lock_guard<std::mutex> lock(shutdown_mutex_);
      shutdown_barrier_owner_ = std::thread::id{};
      shutdown_barrier_complete_ = true;
    }
    shutdown_condition_.notify_all();
  }

  [[nodiscard]] bool wait_for_shutdown_conditions(
    const RuntimeShutdownCoordinator::TimePoint deadline) noexcept
  {
    bool internal_complete = relative_motion_ == nullptr;
    try {
      if (relative_motion_) {
        internal_complete = detail::wait_for_relative_motion_internal_completion(
          *relative_motion_, deadline);
      }
    } catch (...) {
      internal_complete = false;
    }

    const auto transaction_drained = admission_gate_.wait_for_transaction_drain(
      deadline);
    bool gate_safe = false;
    try {
      const auto snapshot = authority_ ? authority_->snapshot() : GateSnapshot{};
      gate_safe = (!relative_motion_ || relative_motion_->zero_proven()) &&
        snapshot.state == GateState::Inhibited && snapshot.motion_inhibited &&
        snapshot.zero_selected && snapshot.zero_published;
    } catch (...) {
      gate_safe = false;
    }
    return internal_complete && transaction_drained && gate_safe &&
           wait_for_runtime_terminal_until(deadline);
  }

  [[nodiscard]] bool wait_for_runtime_terminal_until(
    const RuntimeShutdownCoordinator::TimePoint deadline) noexcept
  {
    std::unique_lock<std::mutex> lock(shutdown_completion_mutex_);
    return runtime_terminal_condition_.wait_until(lock, deadline, [this]() {
               bool goals_empty = false;
               {
                 std::lock_guard<std::recursive_mutex> goal_lock(mutex_);
                 goals_empty = goals_.empty();
               }
               bool core_idle = true;
               if (execution_plane_ && execution_plane_->core()) {
                 std::lock_guard<std::recursive_mutex> core_lock(
          execution_plane_->core_serial_mutex());
                 core_idle = !execution_plane_->core()->has_active_mission();
               }
               return goals_empty && core_idle;
    });
  }

  RuntimeConfig load_config()
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Trusted Mission Runtime policy; immutable after startup";
    descriptor.read_only = true;
    const auto mode = declare_parameter<std::string>(
      "operating_mode", "mapping", descriptor);
    const auto mission_deadline_ms = declare_parameter<std::int64_t>(
      "mission_deadline_ms", kTrustedMissionDeadlineMs, descriptor);
    const auto gate_discovery_deadline_ms = declare_parameter<std::int64_t>(
      "gate_discovery_deadline_ms", kTrustedGateDiscoveryDeadlineMs, descriptor);
    const auto control_response_deadline_ms = declare_parameter<std::int64_t>(
      "control_response_deadline_ms", kTrustedControlResponseDeadlineMs, descriptor);
    const auto stop_barrier_ms = declare_parameter<std::int64_t>(
      "stop_barrier_ms", kTrustedStopBarrierMs, descriptor);
    const auto cancel_grace_ms = declare_parameter<std::int64_t>(
      "cancel_grace_ms", kTrustedCancelGraceMs, descriptor);
    const auto stationarity_deadline_ms = declare_parameter<std::int64_t>(
      "stationarity_deadline_ms", kTrustedStationarityDeadlineMs, descriptor);
    const auto source_cache_size = declare_parameter<std::int64_t>(
      "source_cache_size", 64, descriptor);
    const auto stop_cache_size = declare_parameter<std::int64_t>(
      "stop_cache_size", 64, descriptor);
    const auto max_steps = declare_parameter<std::int64_t>(
      "max_steps", 3, descriptor);
    const auto move_distance_min_m = declare_parameter<double>(
      "move_distance_min_m", kTrustedMoveDistanceMinM, descriptor);
    const auto move_distance_max_m = declare_parameter<double>(
      "move_distance_max_m", kTrustedMoveDistanceMaxM, descriptor);
    const auto rotate_angle_min_rad = declare_parameter<double>(
      "rotate_angle_min_rad", kTrustedRotateAngleMinRad, descriptor);
    const auto rotate_angle_max_rad = declare_parameter<double>(
      "rotate_angle_max_rad", kTrustedRotateAngleMaxRad, descriptor);
    const auto named_place_ids = declare_parameter<std::vector<std::string>>(
      "named_place_ids", {}, descriptor);
    if (mode != "mapping" && mode != "navigation") {
      throw std::invalid_argument("operating_mode must be mapping or navigation");
    }
    if (
      mission_deadline_ms != kTrustedMissionDeadlineMs ||
      gate_discovery_deadline_ms != kTrustedGateDiscoveryDeadlineMs ||
      control_response_deadline_ms != kTrustedControlResponseDeadlineMs ||
      stop_barrier_ms != kTrustedStopBarrierMs ||
      cancel_grace_ms != kTrustedCancelGraceMs ||
      stationarity_deadline_ms != kTrustedStationarityDeadlineMs ||
      source_cache_size != 64 ||
      stop_cache_size != 64 || max_steps != 3 ||
      !std::isfinite(move_distance_min_m) ||
      !std::isfinite(move_distance_max_m) ||
      !std::isfinite(rotate_angle_min_rad) ||
      !std::isfinite(rotate_angle_max_rad) ||
      static_cast<float>(move_distance_min_m) != kTrustedMoveDistanceMinM ||
      static_cast<float>(move_distance_max_m) != kTrustedMoveDistanceMaxM ||
      static_cast<float>(rotate_angle_min_rad) != kTrustedRotateAngleMinRad ||
      static_cast<float>(rotate_angle_max_rad) != kTrustedRotateAngleMaxRad ||
      named_place_ids.size() > 32U)
    {
      throw std::invalid_argument("Mission Runtime trusted YAML is not frozen");
    }
    for (const auto & named_place : named_place_ids) {
      if (named_place.empty() || named_place.size() > 64U) {
        throw std::invalid_argument("named place ID is outside the trusted bound");
      }
    }
    RuntimeConfig config;
    config.operating_mode = mode == "mapping" ?
      OperatingMode::Mapping : OperatingMode::Navigation;
    config.mission_deadline = std::chrono::milliseconds(mission_deadline_ms);
    config.gate_discovery_deadline = std::chrono::milliseconds(gate_discovery_deadline_ms);
    config.control_response_deadline = std::chrono::milliseconds(control_response_deadline_ms);
    config.stop_barrier = std::chrono::milliseconds(stop_barrier_ms);
    config.cancel_grace = std::chrono::milliseconds(cancel_grace_ms);
    config.stationarity_deadline =
      std::chrono::milliseconds(stationarity_deadline_ms);
    config.move_distance_min_m = static_cast<float>(move_distance_min_m);
    config.move_distance_max_m = static_cast<float>(move_distance_max_m);
    config.rotate_angle_min_rad = static_cast<float>(rotate_angle_min_rad);
    config.rotate_angle_max_rad = static_cast<float>(rotate_angle_max_rad);
    config.named_place_ids = named_place_ids;
    return config;
  }

  rclcpp_action::GoalResponse on_goal(
    const GoalUUID & uuid,
    const std::shared_ptr<const ExecuteMission::Goal> & goal)
  {
    const auto uuid_key = goal_uuid_key(uuid);
    if (!rclcpp::ok() || !admission_gate_.try_provision(
        action_admission_tracker_, uuid_key, goal->admission_epoch,
        [this](const std::uint64_t epoch) {
          return !event_ingress_.blocked() &&
                 event_ingress_.admission_allowed(epoch);
        }))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Every wire-valid Goal reaches Core. Business rejection is returned as
    // a structured Result with an ABORTED outer Action status.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(
    const std::shared_ptr<GoalHandle> & goal_handle)
  {
    std::optional<std::uint64_t> mission_id;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      for (const auto & entry : goals_) {
        if (entry.second == goal_handle) {
          mission_id = entry.first;
          break;
        }
      }
      if (!mission_id.has_value()) {
        pending_goal_cancels_.insert(goal_handle.get());
      }
    }
    if (mission_id.has_value()) {
      (void)enqueue_event(RuntimeEvent{*mission_id, CancelEvent{*mission_id}});
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accepted(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    const auto uuid_key = goal_uuid_key(goal_handle->get_goal_id());
    auto admission_lease = action_admission_tracker_.enter_accepted(uuid_key);
    const auto goal_message = goal_handle->get_goal();
    MissionGoal goal;
    goal.source_instance_id = goal_message->source_instance_id;
    goal.source_seq = goal_message->source_seq;
    goal.runtime_instance_id = goal_message->runtime_instance_id;
    goal.admission_epoch = goal_message->admission_epoch;
    goal.steps.reserve(goal_message->steps.size());
    for (const auto & step : goal_message->steps) {
      goal.steps.push_back(MissionStep{
          step.kind, step.distance_m, step.angle_rad, step.target_id});
    }
    const auto queued = admission_lease.has_ticket() &&
      !admission_lease.was_revoked() && admission_gate_.submit([this, &goal,
        goal_handle, goal_message]() {
          if (event_ingress_.blocked() ||
          !event_ingress_.admission_allowed(goal_message->admission_epoch))
          {
            return false;
          }
          return enqueue_internal_event(RuntimeEvent{
          0U, AdmitEvent{std::move(goal), goal_handle}});
      });
    if (!queued) {
      abort_goal(goal_handle, MissionResult{
          MissionResultCode::SafetyFault, -1,
          admission_gate_.quiescing() ? "Runtime is quiescing" :
          "Runtime admission queue could not accept Mission admission"});
    }
  }

  [[nodiscard]] bool enqueue_event(RuntimeEvent event) noexcept
  {
    return admission_gate_.submit([this, &event]() {
               return enqueue_internal_event(std::move(event));
    });
  }

  [[nodiscard]] bool enqueue_internal_event(RuntimeEvent event) noexcept
  {
    return event_ingress_.enqueue(std::move(event));
  }

  void request_independent_emergency() noexcept
  {
    if (!relative_motion_) {
      return;
    }
    relative_motion_->request_emergency_stop();
  }

  void request_emergency_fence(std::string detail) noexcept
  {
    event_ingress_.request_emergency(std::move(detail));
  }

  void run_runtime_events()
  {
    event_ingress_.run(
      [this](RuntimeEvent & event) {
        std::lock_guard<std::recursive_mutex> lock(
          execution_plane_->core_serial_mutex());
        std::visit(
          [this](auto & typed_event) {process_event(typed_event);},
          event.payload);
      },
      [this](std::string detail) {
        request_emergency_fence(std::move(detail));
      });
  }

  void process_event(AdmitEvent & event)
  {
    if (!execution_plane_ || !execution_plane_->core()) {
      abort_goal(event.goal_handle, MissionResult{
          MissionResultCode::SafetyFault, -1,
          "Runtime execution plane is unavailable"});
      return;
    }
    const auto start_permit = admission_gate_.claim_start(
      event.goal.admission_epoch);
    if (!start_permit.issued) {
      abort_goal(event.goal_handle, MissionResult{
          MissionResultCode::SafetyFault, -1,
          "Runtime admission was quiesced before dispatch"});
      return;
    }
    std::uint64_t admitted_id = 0U;
    bool cancel_after_admission = false;
    action_adapter_.on_accepted(
      event.goal,
      [this, start_permit](const MissionGoal & value) {
        return execution_plane_->core()->admit(
          value,
          [this, start_permit]() {
            return admission_gate_.start_allowed(start_permit) &&
                   event_ingress_.admission_allowed(start_permit.admission_epoch);
          },
          start_permit.generation);
      },
      [this, &event, &admitted_id, &cancel_after_admission](
        const std::uint64_t mission_id) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        goals_.emplace(mission_id, event.goal_handle);
        admitted_id = mission_id;
        const auto found = pending_goal_cancels_.find(event.goal_handle.get());
        if (found != pending_goal_cancels_.end()) {
          pending_goal_cancels_.erase(found);
          cancel_after_admission = true;
        }
      },
      [this](const std::uint64_t mission_id, const ActionResultDelivery & delivery) {
        finish_goal(mission_id, delivery);
      },
      [this, &event](const MissionResult & result) {
        {
          std::lock_guard<std::recursive_mutex> lock(mutex_);
          pending_goal_cancels_.erase(event.goal_handle.get());
        }
        abort_goal(event.goal_handle, result);
      });
    if (cancel_after_admission && admitted_id != 0U) {
      execution_plane_->core()->cancel(admitted_id);
    }
  }

  void process_event(CancelEvent & event)
  {
    execution_plane_->core()->cancel(event.mission_id);
  }

  void process_event(StopEvent & event)
  {
    StopResponse response;
    try {
      response = execution_plane_->core()->stop(event.request);
    } catch (const std::exception & error) {
      request_independent_emergency();
      response = StopResponse{
        2U, {}, 0U, false,
        std::string{"STOP worker raised: "} + error.what()};
      execution_plane_->core()->fail_closed(response.detail);
    } catch (...) {
      request_independent_emergency();
      response = StopResponse{
        2U, {}, 0U, false, "STOP worker raised an unknown exception"};
      execution_plane_->core()->fail_closed(response.detail);
    }
    {
      std::lock_guard<std::mutex> lock(event.waiter->mutex);
      event.waiter->response = std::move(response);
      event.waiter->completed = true;
    }
    event.waiter->condition.notify_one();
  }

  void process_event(TickEvent & event)
  {
    (void)action_admission_tracker_.revoke_expired(event.now);
    if (ros_authority_) {
      ros_authority_->refresh_endpoint();
    }
    execution_plane_->core()->on_tick();
  }

  void process_event(GateSnapshotEvent & event)
  {
    execution_plane_->core()->observe_gate(event.snapshot);
  }

  void process_event(ChildFeedbackEvent & event)
  {
    execution_plane_->core()->on_child_feedback(event.token, event.progress);
  }

  void process_event(ChildResultEvent & event)
  {
    const auto dispatch = execution_plane_->completion_mailbox().take(event.token);
    if (!dispatch.has_value()) {
      return;
    }
    try {
      if (dispatch->delivery) {
        dispatch->delivery(dispatch->record->token, dispatch->record->result);
      }
    } catch (...) {
      request_emergency_fence("Node completion delivery raised");
    }
  }

  void process_event(QueueFaultEvent & event)
  {
    request_emergency_fence(event.detail);
  }

  static void abort_goal(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const MissionResult & result)
  {
    auto action_result = std::make_shared<ExecuteMission::Result>();
    fill_result(result, *action_result);
    goal_handle->abort(action_result);
  }

  void on_stop(
    const StopMission::Request & request,
    StopMission::Response & response)
  {
    auto waiter = std::make_shared<StopWaiter>();
    const auto accepted = enqueue_event(RuntimeEvent{
        0U,
        StopEvent{
          StopRequest{
            request.request_id,
            request.source_instance_id,
            request.source_seq,
            request.reason},
          waiter}});
    if (!accepted) {
      const auto emergency_deadline = std::chrono::steady_clock::now() +
        config_.stationarity_deadline + config_.stop_barrier;
      bool zero_proven = false;
      if (relative_motion_) {
        try {
          zero_proven = relative_motion_->emergency_stop(emergency_deadline);
        } catch (...) {
          zero_proven = false;
        }
      }
      const auto state = state_snapshot();
      response.code = 2U;
      response.runtime_instance_id = state.runtime_instance_id;
      response.admission_epoch = state.admission_epoch;
      response.motion_inhibited = zero_proven;
      response.detail = "Runtime event queue could not accept STOP";
      return;
    }
    std::unique_lock<std::mutex> lock(waiter->mutex);
    const auto wait_deadline = std::chrono::steady_clock::now() +
      config_.stationarity_deadline + config_.stop_barrier;
    if (!waiter->condition.wait_until(
        lock, wait_deadline, [waiter]() {return waiter->completed;}))
    {
      request_independent_emergency();
      const auto state = state_snapshot();
      response.code = 2U;
      response.runtime_instance_id = state.runtime_instance_id;
      response.admission_epoch = state.admission_epoch;
      response.motion_inhibited = relative_motion_ && relative_motion_->zero_proven();
      response.detail = "STOP response deadline expired before zero proof";
      return;
    }
    response.code = waiter->response.code;
    response.runtime_instance_id = waiter->response.runtime_instance_id;
    response.admission_epoch = waiter->response.admission_epoch;
    response.motion_inhibited = waiter->response.motion_inhibited;
    response.detail = waiter->response.detail;
  }

  void finish_goal(
    std::uint64_t mission_id,
    const ActionResultDelivery & delivery)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto found = goals_.find(mission_id);
    if (found == goals_.end()) {
      return;
    }
    auto action_result = std::make_shared<ExecuteMission::Result>();
    fill_result(delivery.result, *action_result);
    if (delivery.status == OuterActionStatus::Succeeded) {
      found->second->succeed(action_result);
    } else if (delivery.status == OuterActionStatus::Canceled) {
      found->second->canceled(action_result);
    } else {
      found->second->abort(action_result);
    }
    goals_.erase(found);
    runtime_terminal_condition_.notify_all();
  }

  static void fill_result(
    const MissionResult & source,
    ExecuteMission::Result & target)
  {
    target.code = static_cast<std::uint16_t>(source.code);
    target.failed_step = source.failed_step;
    target.detail = source.detail;
  }

  void publish_feedback(
    std::uint64_t mission_id,
    const MissionFeedback & feedback)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto found = goals_.find(mission_id);
    if (found == goals_.end()) {
      return;
    }
    auto message = std::make_shared<ExecuteMission::Feedback>();
    message->phase = static_cast<std::uint8_t>(feedback.phase);
    message->step_index = feedback.step_index;
    message->progress = feedback.progress;
    found->second->publish_feedback(message);
  }

  void publish_state(const RuntimeState & state)
  {
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      cached_state_ = state;
      cached_state_valid_ = true;
    }
    if (!state_publisher_) {
      return;
    }
    MissionStateMessage message;
    message.runtime_instance_id = state.runtime_instance_id;
    message.admission_epoch = state.admission_epoch;
    message.operating_mode = static_cast<std::uint8_t>(state.operating_mode);
    message.availability = static_cast<std::uint8_t>(state.availability);
    message.gate_state = state.gate_state == GateState::Faulted ?
      MissionStateMessage::GATE_FAULTED :
      (state.gate_state == GateState::Armed ?
      MissionStateMessage::GATE_ARMED : MissionStateMessage::GATE_INHIBITED);
    message.active_step = state.active_step;
    message.supported_step_mask = state.supported_step_mask;
    message.max_steps = state.max_steps;
    message.named_place_ids.assign(
      state.named_place_ids.begin(), state.named_place_ids.end());
    state_publisher_->publish(message);
  }

  [[nodiscard]] RuntimeState state_snapshot()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return cached_state_;
  }

  RuntimeConfig config_;
  std::shared_ptr<RosSteadyClock> clock_;
  ActionAdmissionTracker action_admission_tracker_;
  RuntimeAdmissionGate admission_gate_;
  std::shared_ptr<MotionAuthorityPort> authority_;
  std::shared_ptr<RosMotionAuthorityPort> ros_authority_;
  std::shared_ptr<RelativeMotionRosAdapter> relative_motion_;
  std::unique_ptr<RuntimeExecutionPlane> execution_plane_;
  RuntimeEventQueueType event_queue_;
  RuntimeEmergencyFence emergency_fence_{1U};
  RuntimeEventIngress<RuntimeEvent> event_ingress_;
  std::thread runtime_worker_;
  std::unique_ptr<RuntimeShutdownCoordinator> shutdown_coordinator_;
  std::recursive_mutex mutex_;
  std::unordered_set<const GoalHandle *> pending_goal_cancels_;
  RuntimeState cached_state_{};
  bool cached_state_valid_{false};
  std::shared_ptr<rclcpp::Context> shutdown_context_;
  rclcpp::PreShutdownCallbackHandle shutdown_callback_handle_;
  std::mutex shutdown_mutex_;
  std::mutex shutdown_completion_mutex_;
  std::condition_variable shutdown_condition_;
  std::condition_variable runtime_terminal_condition_;
  std::thread::id shutdown_barrier_owner_{};
  bool shutdown_barrier_started_{false};
  bool shutdown_barrier_complete_{false};
  bool action_shutdown_fail_closed_{false};
  rclcpp::Publisher<MissionStateMessage>::SharedPtr state_publisher_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  std::shared_ptr<ActionCallbackLifetime> action_callback_lifetime_;
  rclcpp_action::Server<ExecuteMission>::SharedPtr action_server_;
  rclcpp::Service<StopMission>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  MissionActionAdapterBoundary action_adapter_;
  std::unordered_map<std::uint64_t, std::shared_ptr<GoalHandle>> goals_;
};

}  // namespace voice_nav_mission

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<voice_nav_mission::MissionRuntimeNode>();
    // The conditioning Module keeps the public Core synchronous while its ROS
    // ports wait on Gate and component-service responses.  Keep spare workers
    // available for those responses and for the conditioning health streams
    // while another worker is in the Runtime timer or Action callback.
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions{}, 32U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("mission_runtime_node"),
      "Mission Runtime startup failed: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
