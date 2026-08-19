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
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
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
#include "voice_nav_mission/map_package.hpp"
#include "voice_nav_mission/map_store_ros_adapter.hpp"
#include "voice_nav_mission/motion_authority_ros_adapter.hpp"
#include "voice_nav_mission/relative_motion_ros_adapter.hpp"
#include "../src/runtime_engine.hpp"

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

// Shared by Gate, timer, STOP, and action callbacks.  A callback holds a
// shared Engine handle while it is in flight; shutdown closes this ingress and
// drains every entered callback before releasing the Engine.
class RuntimeIngressLifetime final
{
public:
  void bind(const std::shared_ptr<RuntimeEngine> & runtime)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    runtime_ = runtime;
  }

  [[nodiscard]] std::shared_ptr<RuntimeEngine> acquire() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      return {};
    }
    auto runtime = runtime_.lock();
    if (runtime) {
      ++inflight_;
    }
    return runtime;
  }

  void release() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (inflight_ > 0U) {
      --inflight_;
      if (inflight_ == 0U) {
        condition_.notify_all();
      }
    }
  }

  void deactivate_and_drain() noexcept
  {
    std::unique_lock<std::mutex> lock(mutex_);
    active_ = false;
    runtime_.reset();
    condition_.wait(lock, [this]() {return inflight_ == 0U;});
  }

private:
  std::weak_ptr<RuntimeEngine> runtime_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::size_t inflight_{0U};
  bool active_{true};
};

// Action callbacks use the same ingress lifetime as Gate/timer/STOP.  Handler
// function objects may retain Node state, but the shared ingress guarantees
// they finish before shutdown releases that state.
class ActionCallbackLifetime final
{
public:
  using GoalHandler = std::function<rclcpp_action::GoalResponse(
        const GoalUUID &, std::shared_ptr<const ExecuteMission::Goal>)>;
  using CancelHandler = std::function<rclcpp_action::CancelResponse(
        const std::shared_ptr<GoalHandle> &)>;
  using AcceptedHandler = std::function<void(const std::shared_ptr<GoalHandle> &)>;

  ActionCallbackLifetime(
    std::shared_ptr<RuntimeIngressLifetime> ingress,
    GoalHandler goal_handler,
    CancelHandler cancel_handler,
    AcceptedHandler accepted_handler)
  : ingress_(std::move(ingress)),
    goal_handler_(std::move(goal_handler)),
    cancel_handler_(std::move(cancel_handler)),
    accepted_handler_(std::move(accepted_handler))
  {
  }

  [[nodiscard]] rclcpp_action::GoalResponse on_goal(
    const GoalUUID & uuid,
    std::shared_ptr<const ExecuteMission::Goal> goal)
  {
    const auto runtime = ingress_ ? ingress_->acquire() : nullptr;
    GoalHandler handler;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_) {
        handler = goal_handler_;
      }
    }
    if (!runtime || !handler) {
      if (runtime) {
        ingress_->release();
      }
      return rclcpp_action::GoalResponse::REJECT;
    }
    try {
      const auto response = handler(uuid, std::move(goal));
      ingress_->release();
      return response;
    } catch (...) {
      ingress_->release();
      throw;
    }
  }

  [[nodiscard]] rclcpp_action::CancelResponse on_cancel(
    const std::shared_ptr<GoalHandle> & goal_handle)
  {
    const auto runtime = ingress_ ? ingress_->acquire() : nullptr;
    CancelHandler handler;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_) {
        handler = cancel_handler_;
      }
    }
    if (!runtime || !handler) {
      if (runtime) {
        ingress_->release();
      }
      return rclcpp_action::CancelResponse::REJECT;
    }
    try {
      const auto response = handler(goal_handle);
      ingress_->release();
      return response;
    } catch (...) {
      ingress_->release();
      throw;
    }
  }

  void on_accepted(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    const auto runtime = ingress_ ? ingress_->acquire() : nullptr;
    AcceptedHandler handler;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_) {
        handler = accepted_handler_;
      }
    }
    if (runtime && handler) {
      try {
        handler(goal_handle);
        ingress_->release();
      } catch (...) {
        ingress_->release();
        throw;
      }
      return;
    }
    if (runtime) {
      ingress_->release();
    }
    // RuntimeEngine owns the provisional admission fence during the bounded
    // shutdown drain.  Jazzy does not expose a transport handoff guarantee
    // for a callback that has not entered production on_accepted; do not
    // fabricate a Result for that no-handle path.
  }

  void deactivate() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_ = false;
      goal_handler_ = {};
      cancel_handler_ = {};
      accepted_handler_ = {};
    }
    if (ingress_) {
      ingress_->deactivate_and_drain();
    }
  }

  [[nodiscard]] bool active() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
  }

private:
  std::shared_ptr<RuntimeIngressLifetime> ingress_;
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

class RosSteadyClock final : public SteadyClockPort
{
public:
  [[nodiscard]] TimePoint now() const override
  {
    return std::chrono::steady_clock::now();
  }
};

class RosGoalSink final : public RuntimeGoalSink
{
public:
  explicit RosGoalSink(std::shared_ptr<GoalHandle> goal_handle)
  : goal_handle_(std::move(goal_handle))
  {
  }

  [[nodiscard]] const void * identity() const noexcept override
  {
    return goal_handle_.get();
  }

  void deliver(const ActionResultDelivery & delivery) override
  {
    auto result = std::make_shared<ExecuteMission::Result>();
    result->code = static_cast<std::uint16_t>(delivery.result.code);
    result->failed_step = delivery.result.failed_step;
    result->detail = delivery.result.detail;
    if (delivery.status == OuterActionStatus::Succeeded) {
      goal_handle_->succeed(result);
    } else if (delivery.status == OuterActionStatus::Canceled) {
      goal_handle_->canceled(result);
    } else {
      goal_handle_->abort(result);
    }
  }

  void feedback(const MissionFeedback & value) override
  {
    auto message = std::make_shared<ExecuteMission::Feedback>();
    message->phase = static_cast<std::uint8_t>(value.phase);
    message->step_index = value.step_index;
    message->progress = value.progress;
    goal_handle_->publish_feedback(message);
  }

private:
  std::shared_ptr<GoalHandle> goal_handle_;
};

class RosStateSink final : public RuntimeStateSink
{
public:
  explicit RosStateSink(
    rclcpp::Publisher<MissionStateMessage>::SharedPtr publisher)
  : publisher_(std::move(publisher))
  {
  }

  void publish(const RuntimeState & state) override
  {
    if (!publisher_) {
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
    publisher_->publish(message);
  }

private:
  rclcpp::Publisher<MissionStateMessage>::SharedPtr publisher_;
};

}  // namespace

class MissionRuntimeNode final : public rclcpp::Node
{
public:
  explicit MissionRuntimeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mission_runtime_node", options),
    clock_(std::make_shared<RosSteadyClock>())
  {
    if (std::string(get_fully_qualified_name()) != "/mission_runtime_node") {
      throw std::runtime_error("mission_runtime_node must run at /mission_runtime_node");
    }
    config_ = load_config();
    map_reader_ = std::make_unique<MapPackageReader>(default_map_root());
    if (config_.operating_mode == OperatingMode::Mapping) {
      map_upstream_ = std::make_shared<RosMapStoreUpstream>(*this);
      map_store_ = std::make_shared<ProductionMapStore>(
        default_map_root(),
        [this](const std::filesystem::path & staging_directory) {
          return map_upstream_ ? map_upstream_->capture(staging_directory) :
                 ChildResult{ChildResultCode::DependencyUnavailable,
                 "Map upstream is unavailable"};
        },
        trusted_named_places_file_);
    }
    const NamedPlaceResolver named_place_resolver =
      [this](const std::string & place_id) -> std::optional<NavigationPlace> {
        if (!map_reader_) {
          return std::nullopt;
        }
        MapPackage package;
        if (map_reader_->load(map_id_, &package).code != ChildResultCode::Succeeded) {
          return std::nullopt;
        }
        NamedPlace place;
        if (map_reader_->read_named_place(package, place_id, &place).code !=
          ChildResultCode::Succeeded)
        {
          return std::nullopt;
        }
        return NavigationPlace{place.x, place.y, place.yaw};
      };
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    state_qos.reliable().transient_local();
    state_publisher_ = create_publisher<MissionStateMessage>(kStateTopic, state_qos);
    auto ingress_lifetime = std::make_shared<RuntimeIngressLifetime>();
    auto gate_observer =
      std::make_shared<std::function<void(const GateSnapshot &)>>();
    authority_ = std::make_shared<RosMotionAuthorityPort>(
      *this,
      config_.control_response_deadline,
      config_.stop_barrier,
      [gate_observer](const GateSnapshot & snapshot) {
        if (*gate_observer) {
          (*gate_observer)(snapshot);
        }
      });
    const auto initial_gate_snapshot = authority_->snapshot();
    auto relative_slot =
      std::make_shared<std::shared_ptr<RelativeMotionRosAdapter>>();
    runtime_ = std::make_shared<RuntimeEngine>(
      config_,
      clock_,
      authority_,
      initial_gate_snapshot,
      [this, named_place_resolver, relative_slot](
        const RuntimeEngine::MotionConditioningBindings & bindings) {
        RelativeMotionPolicy motion_policy;
        motion_policy.stationarity_deadline = config_.stationarity_deadline;
        MotionConditioningConfig conditioning_config;
        conditioning_config.stop_barrier = config_.stop_barrier;
        conditioning_config.collision_source_timeout = std::chrono::milliseconds(
          kTrustedCollisionSourceTimeoutMs);
        conditioning_config.transaction_plane = bindings.transaction_plane;
        conditioning_config.admission_fence_check = bindings.admission_fence_check;
        conditioning_config.completion_relay = bindings.completion_relay;
        auto relative = std::make_shared<RelativeMotionRosAdapter>(
          *this, authority_, motion_policy, conditioning_config, named_place_resolver);
        *relative_slot = relative;
        return RuntimeEngine::ChildDependencies{
        relative,
        std::static_pointer_cast<NavigationPort>(relative),
        map_store_};
      },
      [relative_slot]() {
        if (*relative_slot) {
          (*relative_slot)->request_emergency_stop();
        }
      },
      [relative_slot](const RuntimeEngine::TimePoint deadline) {
        return *relative_slot && (*relative_slot)->emergency_stop(deadline);
      },
      [authority = authority_]() {authority->refresh_endpoint();},
      std::make_shared<RosStateSink>(state_publisher_));
    relative_motion_ = *relative_slot;
    ingress_lifetime->bind(runtime_);
    *gate_observer = [ingress_lifetime](const GateSnapshot & snapshot) {
        const auto runtime = ingress_lifetime->acquire();
        if (runtime) {
          runtime->post(RuntimeEngine::GateSnapshotInput{snapshot});
          ingress_lifetime->release();
        }
      };

    action_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    action_callback_lifetime_ = std::make_shared<ActionCallbackLifetime>(
      ingress_lifetime,
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
      [this, ingress_lifetime](
        const std::shared_ptr<StopMission::Request> request,
        std::shared_ptr<StopMission::Response> response) {
        const auto runtime = ingress_lifetime->acquire();
        if (runtime) {
          try {
            on_stop(*runtime, *request, *response);
          } catch (...) {
            response->detail = "Runtime STOP callback raised";
          }
          ingress_lifetime->release();
        } else {
          response->detail = "Runtime is unavailable";
        }
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [ingress_lifetime, clock = clock_]() {
        const auto runtime = ingress_lifetime->acquire();
        if (runtime) {
          runtime->post(RuntimeEngine::TickInput{clock->now()});
          ingress_lifetime->release();
        }
      });
    ingress_lifetime_ = std::move(ingress_lifetime);
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
    if (action_callback_lifetime_) {
      action_callback_lifetime_->deactivate();
    } else if (ingress_lifetime_) {
      ingress_lifetime_->deactivate_and_drain();
    }
    (void)runtime_->shutdown(
      shutdown_deadline,
      RuntimeEngine::ShutdownHooks{
        [this](const RuntimeEngine::TimePoint deadline) {
          timer_.reset();
          stop_service_.reset();
          if (relative_motion_) {
            detail::begin_relative_motion_shutdown(*relative_motion_, deadline);
          }
        },
        [this](const RuntimeEngine::TimePoint deadline) {
          return relative_motion_ &&
                 detail::wait_for_relative_motion_internal_completion(
                   *relative_motion_, deadline);
        },
        [this]() {
          if (relative_motion_) {
            relative_motion_->finalize_shutdown();
          }
        }});
    action_server_.reset();
    action_callback_lifetime_.reset();
    runtime_.reset();
    ingress_lifetime_.reset();
    relative_motion_.reset();
    authority_.reset();

    {
      std::lock_guard<std::mutex> lock(shutdown_mutex_);
      shutdown_barrier_owner_ = std::thread::id{};
      shutdown_barrier_complete_ = true;
    }
    shutdown_condition_.notify_all();
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
    const auto map_id = declare_parameter<std::string>(
      "map_id", "voice_mvp", descriptor);
    const auto trusted_named_places_file = declare_parameter<std::string>(
      "trusted_named_places_file", "", descriptor);
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
      named_place_ids.size() > 32U || !valid_map_id(map_id) ||
      (!trusted_named_places_file.empty() &&
      !std::filesystem::path(trusted_named_places_file).is_absolute()))
    {
      throw std::invalid_argument("Mission Runtime trusted YAML is not frozen");
    }
    for (const auto & named_place : named_place_ids) {
      if (named_place.empty() || named_place.size() > 64U) {
        throw std::invalid_argument("named place ID is outside the trusted bound");
      }
    }
    map_id_ = map_id;
    trusted_named_places_file_ = trusted_named_places_file;
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
    if (!rclcpp::ok() || !runtime_ || !goal ||
      !runtime_->authorize_admission(goal_uuid_key(uuid), goal->admission_epoch))
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
    if (runtime_ && goal_handle) {
      runtime_->submit_cancel(goal_handle.get());
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accepted(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    if (!runtime_ || !goal_handle) {
      return;
    }
    const auto uuid_key = goal_uuid_key(goal_handle->get_goal_id());
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
    runtime_->submit_admission(
      uuid_key, std::move(goal), std::make_shared<RosGoalSink>(goal_handle));
  }

  void on_stop(
    RuntimeEngine & runtime,
    const StopMission::Request & request,
    StopMission::Response & response)
  {
    StopResponse stop_response;
    runtime.submit_stop(
      StopRequest{
        request.request_id,
        request.source_instance_id,
        request.source_seq,
        request.reason},
      stop_response);
    response.code = stop_response.code;
    response.runtime_instance_id = stop_response.runtime_instance_id;
    response.admission_epoch = stop_response.admission_epoch;
    response.motion_inhibited = stop_response.motion_inhibited;
    response.detail = stop_response.detail;
  }

  RuntimeConfig config_;
  std::shared_ptr<RosSteadyClock> clock_;
  std::shared_ptr<RosMotionAuthorityPort> authority_;
  std::unique_ptr<MapPackageReader> map_reader_;
  std::shared_ptr<RosMapStoreUpstream> map_upstream_;
  std::shared_ptr<MapStorePort> map_store_;
  std::string map_id_{"voice_mvp"};
  std::filesystem::path trusted_named_places_file_;
  std::shared_ptr<RelativeMotionRosAdapter> relative_motion_;
  std::shared_ptr<RuntimeEngine> runtime_;
  std::shared_ptr<rclcpp::Context> shutdown_context_;
  rclcpp::PreShutdownCallbackHandle shutdown_callback_handle_;
  std::mutex shutdown_mutex_;
  std::condition_variable shutdown_condition_;
  std::thread::id shutdown_barrier_owner_{};
  bool shutdown_barrier_started_{false};
  bool shutdown_barrier_complete_{false};
  rclcpp::Publisher<MissionStateMessage>::SharedPtr state_publisher_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  std::shared_ptr<ActionCallbackLifetime> action_callback_lifetime_;
  std::shared_ptr<RuntimeIngressLifetime> ingress_lifetime_;
  rclcpp_action::Server<ExecuteMission>::SharedPtr action_server_;
  rclcpp::Service<StopMission>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
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
