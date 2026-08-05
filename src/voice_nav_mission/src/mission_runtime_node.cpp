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

#include <rmw/qos_profiles.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "voice_nav_interfaces/action/execute_mission.hpp"
#include "voice_nav_interfaces/msg/mission_state.hpp"
#include "voice_nav_interfaces/msg/mission_step.hpp"
#include "voice_nav_interfaces/srv/stop_mission.hpp"
#include "voice_nav_mission/msg/internal_motion_gate_state.hpp"
#include "voice_nav_mission/srv/internal_motion_gate_control.hpp"

namespace voice_nav_mission
{
namespace
{

using ExecuteMission = voice_nav_interfaces::action::ExecuteMission;
using StopMission = voice_nav_interfaces::srv::StopMission;
using MissionStateMessage = voice_nav_interfaces::msg::MissionState;
using MissionStepMessage = voice_nav_interfaces::msg::MissionStep;
using GateStateMessage = voice_nav_mission::msg::InternalMotionGateState;
using GateControl = voice_nav_mission::srv::InternalMotionGateControl;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMission>;

constexpr char kExecuteAction[] = "/mission/execute";
constexpr char kStopService[] = "/mission/stop";
constexpr char kStateTopic[] = "/mission/state";
constexpr char kGateControlService[] = "/motion_gate/internal/control";
constexpr char kGateStateTopic[] = "/motion_gate/internal/state";
constexpr std::int64_t kTrustedMissionDeadlineMs = 30000;
constexpr std::int64_t kTrustedGateDiscoveryDeadlineMs = 2000;
constexpr std::int64_t kTrustedControlResponseDeadlineMs = 100;
constexpr std::int64_t kTrustedStopBarrierMs = 250;
constexpr std::int64_t kTrustedCancelGraceMs = 250;
constexpr float kTrustedMoveDistanceMinM = 0.05F;
constexpr float kTrustedMoveDistanceMaxM = 2.0F;
constexpr float kTrustedRotateAngleMinRad = 0.05F;
constexpr float kTrustedRotateAngleMaxRad = 6.283185F;

class RosSteadyClock final : public SteadyClockPort
{
public:
  [[nodiscard]] TimePoint now() const override
  {
    return std::chrono::steady_clock::now();
  }
};

GateState gate_state_from_message(std::uint8_t state)
{
  switch (state) {
    case GateStateMessage::INHIBITED:
      return GateState::Inhibited;
    case GateStateMessage::PREPARED:
      return GateState::Prepared;
    case GateStateMessage::ARMED:
      return GateState::Armed;
    default:
      return GateState::Faulted;
  }
}

class RosMotionAuthorityPort final : public MotionAuthorityPort
{
public:
  using GateChangedCallback = std::function<void(const GateSnapshot &)>;

  RosMotionAuthorityPort(
    rclcpp::Node & node,
    std::chrono::milliseconds control_response_deadline,
    std::chrono::milliseconds stop_barrier,
    GateChangedCallback callback)
  : node_(node),
    control_response_deadline_(control_response_deadline),
    stop_barrier_(stop_barrier),
    callback_(std::move(callback))
  {
    client_callback_group_ = node_.create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    client_ = node_.create_client<GateControl>(
      kGateControlService,
      rmw_qos_profile_services_default,
      client_callback_group_);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.reliable().transient_local();
    subscription_ = node_.create_subscription<GateStateMessage>(
      kGateStateTopic,
      qos,
      [this](const GateStateMessage::ConstSharedPtr message) {
        const bool graph_available = graph_endpoint_available();
        snapshot_ = GateSnapshot{
          message->gate_instance_id,
          message->control_seq,
          message->lease_id,
          gate_state_from_message(message->state),
          graph_available,
          message->motion_inhibited,
          message->zero_selected,
          graph_available && message->zero_publish_seq != 0U &&
          message->zero_publish_seq >= message->output_publish_seq};
        state_sample_available_ = graph_available;
        if (callback_) {
          callback_(snapshot_);
        }
      });
  }

  [[nodiscard]] GateSnapshot snapshot() const override
  {
    return snapshot_;
  }

  [[nodiscard]] AuthorityResult prepare(
    const AuthorityOperation & operation) override
  {
    return converge(
      operation, GateControl::Request::PREPARE, control_response_deadline_);
  }

  [[nodiscard]] AuthorityResult open(
    const AuthorityOperation & operation) override
  {
    return converge(
      operation, GateControl::Request::OPEN, control_response_deadline_);
  }

  [[nodiscard]] AuthorityResult renew(
    const AuthorityOperation & operation) override
  {
    return converge(
      operation, GateControl::Request::RENEW, control_response_deadline_);
  }

  [[nodiscard]] AuthorityResult inhibit(
    const AuthorityOperation & operation) override
  {
    return converge(operation, GateControl::Request::INHIBIT, stop_barrier_);
  }

  void refresh_endpoint()
  {
    const bool available = graph_endpoint_available();
    if (!available) {
      if (snapshot_.endpoint_available || state_sample_available_) {
        snapshot_.endpoint_available = false;
        snapshot_.zero_published = false;
        state_sample_available_ = false;
        if (callback_) {
          callback_(snapshot_);
        }
      }
      return;
    }
    if (state_sample_available_ && !snapshot_.endpoint_available) {
      snapshot_.endpoint_available = true;
      if (callback_) {
        callback_(snapshot_);
      }
    }
  }

private:
  [[nodiscard]] bool graph_endpoint_available() const
  {
    return client_->service_is_ready() &&
           node_.count_publishers(kGateStateTopic) > 0U;
  }

  [[nodiscard]] AuthorityResult converge(
    const AuthorityOperation & operation,
    std::uint8_t operation_code,
    std::chrono::milliseconds budget)
  {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    auto current = operation;
    AuthorityResult last = unavailable(
      "MotionGate control operation did not complete before its steady deadline",
      true);
    while (std::chrono::steady_clock::now() < deadline) {
      if (operation_code == GateControl::Request::INHIBIT) {
        refresh_endpoint();
      }
      last = send_once(current, operation_code, deadline);
      if (
        last.applied &&
        (operation_code != GateControl::Request::INHIBIT || last.zero_proven))
      {
        return last;
      }
      if (!last.retryable) {
        return last;
      }
      if (last.snapshot.endpoint_available) {
        current.gate_instance_id = last.snapshot.gate_instance_id;
        current.expected_control_seq = last.snapshot.control_seq;
        current.lease_id = operation_code == GateControl::Request::PREPARE ?
          std::string{} : last.snapshot.lease_id;
      }
    }
    return last;
  }

  [[nodiscard]] AuthorityResult send_once(
    const AuthorityOperation & operation,
    std::uint8_t operation_code,
    std::chrono::steady_clock::time_point deadline)
  {
    const auto remaining = [&deadline]() {
        const auto now = std::chrono::steady_clock::now();
        return now >= deadline ? std::chrono::milliseconds(0) :
               std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      };
    if (!client_->wait_for_service(remaining())) {
      return unavailable("MotionGate control service is unavailable", false);
    }
    auto request = std::make_shared<GateControl::Request>();
    request->operation = operation_code;
    request->request_id = operation.request_id;
    request->gate_instance_id = operation.gate_instance_id;
    request->expected_control_seq = operation.expected_control_seq;
    request->lease_id = operation.lease_id;
    auto future = client_->async_send_request(request);
    if (future.wait_for(remaining()) !=
      std::future_status::ready)
    {
      return unavailable(
        "MotionGate control response exceeded the steady deadline", true);
    }
    const auto response = future.get();
    snapshot_ = GateSnapshot{
      response->gate_instance_id,
      response->control_seq,
      response->lease_id,
      gate_state_from_message(response->state),
      graph_endpoint_available(),
      response->motion_inhibited,
      response->zero_selected,
      response->zero_published && graph_endpoint_available()};
    state_sample_available_ = snapshot_.endpoint_available;
    if (callback_) {
      callback_(snapshot_);
    }
    const bool applied =
      response->code == GateControl::Response::APPLIED ||
      response->code == GateControl::Response::DUPLICATE;
    const bool zero = response->motion_inhibited && response->zero_selected &&
      response->zero_published;
    const bool retryable =
      response->reason == GateControl::Response::STALE_GATE ||
      response->reason == GateControl::Response::STALE_SEQUENCE ||
      response->reason == GateControl::Response::STALE_LEASE;
    return AuthorityResult{
      applied,
      zero,
      retryable,
      snapshot_,
      response->lease_id,
      response->detail};
  }

  [[nodiscard]] AuthorityResult unavailable(
    std::string detail, bool retryable) const
  {
    return AuthorityResult{
      false, false, retryable, snapshot_, {}, std::move(detail)};
  }

  rclcpp::Node & node_;
  std::chrono::milliseconds control_response_deadline_;
  std::chrono::milliseconds stop_barrier_;
  GateChangedCallback callback_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Client<GateControl>::SharedPtr client_;
  rclcpp::Subscription<GateStateMessage>::SharedPtr subscription_;
  GateSnapshot snapshot_{};
  bool state_sample_available_{false};
};

class UnavailableRelativeMotionPort final : public RelativeMotionPort
{
public:
  [[nodiscard]] bool healthy() const override {return false;}
  void start(
    const MotionToken &, const MissionStep &, FeedbackCallback, ResultCallback) override
  {
    throw std::logic_error("production relative motion adapter is unavailable");
  }
  [[nodiscard]] bool cancel(
    const MotionToken &, SteadyClockPort::TimePoint) override {return true;}
  void tick(SteadyClockPort::TimePoint) override {}
};

}  // namespace

class MissionRuntimeNode final : public rclcpp::Node
{
public:
  explicit MissionRuntimeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mission_runtime_node", options),
    clock_(std::make_shared<RosSteadyClock>()),
    relative_motion_(std::make_shared<UnavailableRelativeMotionPort>())
  {
    if (std::string(get_fully_qualified_name()) != "/mission_runtime_node") {
      throw std::runtime_error("mission_runtime_node must run at /mission_runtime_node");
    }
    config_ = load_config();
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    state_qos.reliable().transient_local();
    state_publisher_ = create_publisher<MissionStateMessage>(kStateTopic, state_qos);
    authority_ = std::make_shared<RosMotionAuthorityPort>(
      *this,
      config_.control_response_deadline,
      config_.stop_barrier,
      [this](const GateSnapshot & snapshot) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (core_) {
          core_->observe_gate(snapshot);
        } else {
          pending_gate_snapshot_ = snapshot;
        }
      });
    core_ = std::make_unique<RuntimeCore>(
      config_,
      clock_,
      authority_,
      relative_motion_,
      [this](const RuntimeState & state) {publish_state(state);},
      [this](std::uint64_t mission_id, const MissionFeedback & feedback) {
        publish_feedback(mission_id, feedback);
      },
      [this](std::uint64_t mission_id, const MissionResult & result) {
        finish_goal(mission_id, result);
      });
    core_->observe_gate(pending_gate_snapshot_.value_or(authority_->snapshot()));

    action_server_ = rclcpp_action::create_server<ExecuteMission>(
      this,
      kExecuteAction,
      [this](const rclcpp_action::GoalUUID &,
      std::shared_ptr<const ExecuteMission::Goal> goal) {
        return on_goal(goal);
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        return on_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        on_accepted(goal_handle);
      });
    stop_service_ = create_service<StopMission>(
      kStopService,
      [this](
        const std::shared_ptr<StopMission::Request> request,
        std::shared_ptr<StopMission::Response> response) {
        on_stop(*request, *response);
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        authority_->refresh_endpoint();
        core_->on_tick();
      });
    publish_state(core_->state());
  }

private:
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
      cancel_grace_ms != kTrustedCancelGraceMs || source_cache_size != 64 ||
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
    config.move_distance_min_m = static_cast<float>(move_distance_min_m);
    config.move_distance_max_m = static_cast<float>(move_distance_max_m);
    config.rotate_angle_min_rad = static_cast<float>(rotate_angle_min_rad);
    config.rotate_angle_max_rad = static_cast<float>(rotate_angle_max_rad);
    config.named_place_ids = named_place_ids;
    return config;
  }

  rclcpp_action::GoalResponse on_goal(
    const std::shared_ptr<const ExecuteMission::Goal> &) const
  {
    if (!rclcpp::ok()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Every wire-valid Goal reaches Core. Business rejection is returned as
    // a structured Result with an ABORTED outer Action status.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(
    const std::shared_ptr<GoalHandle> & goal_handle)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (const auto & entry : goals_) {
      if (entry.second == goal_handle) {
        core_->cancel(entry.first);
        return rclcpp_action::CancelResponse::ACCEPT;
      }
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accepted(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    pending_goal_ = goal_handle;
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
    AdmissionResult admission;
    try {
      admission = core_->admit(goal);
    } catch (const std::exception & error) {
      pending_goal_.reset();
      auto result = std::make_shared<ExecuteMission::Result>();
      fill_result(
        MissionResult{
          MissionResultCode::InternalError, -1,
          std::string{"Mission admission threw: "} + error.what()},
        *result);
      goal_handle->abort(result);
      return;
    } catch (...) {
      pending_goal_.reset();
      auto result = std::make_shared<ExecuteMission::Result>();
      fill_result(
        MissionResult{
          MissionResultCode::InternalError, -1,
          "Mission admission threw an unknown exception"},
        *result);
      goal_handle->abort(result);
      return;
    }
    if (!admission.accepted) {
      pending_goal_.reset();
      auto result = std::make_shared<ExecuteMission::Result>();
      fill_result(admission.result, *result);
      goal_handle->abort(result);
      return;
    }
    goals_.emplace(
      admission.mission_id, std::exchange(pending_goal_, std::nullopt).value());
    const auto early = early_results_.find(admission.mission_id);
    if (early != early_results_.end()) {
      const auto result = early->second;
      early_results_.erase(early);
      finish_goal(admission.mission_id, result);
    }
  }

  void on_stop(
    const StopMission::Request & request,
    StopMission::Response & response)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto stop = core_->stop(StopRequest{
        request.request_id,
        request.source_instance_id,
        request.source_seq,
        request.reason});
    response.code = stop.code;
    response.runtime_instance_id = stop.runtime_instance_id;
    response.admission_epoch = stop.admission_epoch;
    response.motion_inhibited = stop.motion_inhibited;
    response.detail = stop.detail;
  }

  void finish_goal(std::uint64_t mission_id, const MissionResult & result)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto found = goals_.find(mission_id);
    if (found == goals_.end()) {
      if (pending_goal_.has_value()) {
        early_results_[mission_id] = result;
      }
      return;
    }
    auto action_result = std::make_shared<ExecuteMission::Result>();
    fill_result(result, *action_result);
    if (result.code == MissionResultCode::Succeeded) {
      found->second->succeed(action_result);
    } else if (result.code == MissionResultCode::Canceled) {
      found->second->canceled(action_result);
    } else {
      found->second->abort(action_result);
    }
    goals_.erase(found);
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

  RuntimeConfig config_;
  std::shared_ptr<RosSteadyClock> clock_;
  std::shared_ptr<RosMotionAuthorityPort> authority_;
  std::shared_ptr<UnavailableRelativeMotionPort> relative_motion_;
  std::unique_ptr<RuntimeCore> core_;
  std::recursive_mutex mutex_;
  std::optional<GateSnapshot> pending_gate_snapshot_;
  rclcpp::Publisher<MissionStateMessage>::SharedPtr state_publisher_;
  rclcpp_action::Server<ExecuteMission>::SharedPtr action_server_;
  rclcpp::Service<StopMission>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<std::uint64_t, std::shared_ptr<GoalHandle>> goals_;
  std::optional<std::shared_ptr<GoalHandle>> pending_goal_;
  std::unordered_map<std::uint64_t, MissionResult> early_results_;
};

}  // namespace voice_nav_mission

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<voice_nav_mission::MissionRuntimeNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions{}, 2U);
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
