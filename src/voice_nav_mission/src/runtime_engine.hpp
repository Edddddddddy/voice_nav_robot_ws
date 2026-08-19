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

#ifndef RUNTIME_ENGINE_HPP_
#define RUNTIME_ENGINE_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <utility>

#include "voice_nav_mission/action_admission_tracker.hpp"
#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/mission_action_result_router.hpp"
#include "voice_nav_mission/runtime_admission_gate.hpp"
#include "voice_nav_mission/runtime_emergency_fence.hpp"
#include "voice_nav_mission/runtime_event_ingress.hpp"
#include "voice_nav_mission/runtime_event_queue.hpp"
#include "voice_nav_mission/runtime_execution_plane.hpp"
#include "voice_nav_mission/runtime_shutdown_coordinator.hpp"

namespace voice_nav_mission
{

struct MotionConditioningConfig;

// Package-private Goal adapter.  RuntimeEngine invokes it only from its
// serialized worker; ROS callbacks only submit immutable events to the engine.
class RuntimeGoalSink
{
public:
  virtual ~RuntimeGoalSink() = default;
  [[nodiscard]] virtual const void * identity() const noexcept = 0;
  virtual void deliver(const ActionResultDelivery & delivery) = 0;
  virtual void feedback(const MissionFeedback & feedback) = 0;
};

class RuntimeStateSink
{
public:
  virtual ~RuntimeStateSink() = default;
  virtual void publish(const RuntimeState & state) = 0;
};

// A single bounded handoff worker keeps external GoalHandle calls out of the
// Core lock and out of the RuntimeEngine worker.  The worker owns only
// immutable delivery tasks and shared sink references; it never captures the
// Engine, Core, or Node.
class RuntimeDeliveryState final
  : public std::enable_shared_from_this<RuntimeDeliveryState>
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  RuntimeDeliveryState() = default;
  ~RuntimeDeliveryState();

  // Called once after the object is owned by shared_ptr so a deadline
  // timeout can safely detach the worker while it retains shared state.
  void start();

  RuntimeDeliveryState(const RuntimeDeliveryState &) = delete;
  RuntimeDeliveryState & operator=(const RuntimeDeliveryState &) = delete;

  [[nodiscard]] bool submit_result(
    std::shared_ptr<RuntimeGoalSink> sink,
    ActionResultDelivery delivery) noexcept;
  [[nodiscard]] bool submit_feedback(
    std::shared_ptr<RuntimeGoalSink> sink,
    MissionFeedback feedback) noexcept;
  [[nodiscard]] bool submit_state(
    std::shared_ptr<RuntimeStateSink> sink,
    RuntimeState state) noexcept;
  [[nodiscard]] bool wait_for_terminal_capacity(TimePoint deadline) noexcept;
  [[nodiscard]] bool close(TimePoint deadline) noexcept;

private:
  static constexpr std::size_t kTerminalCapacity = ActionAdmissionTracker::kCapacity;
  enum class Kind : std::uint8_t { Result, Feedback, State };

  struct Task
  {
    Kind kind{Kind::Result};
    std::shared_ptr<RuntimeGoalSink> sink;
    ActionResultDelivery result{};
    std::shared_ptr<RuntimeStateSink> state_sink;
    RuntimeState state{};
    MissionFeedback feedback{};
  };

  [[nodiscard]] bool submit(Task task) noexcept;
  void run() noexcept;
  void record_failure(const Task & task) noexcept;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Task> terminal_tasks_;
  std::optional<Task> latest_feedback_;
  std::optional<Task> latest_state_;
  std::thread worker_;
  bool closed_{false};
  bool detached_{false};
  bool terminal_inflight_{false};
  bool external_inflight_{false};
  bool state_disabled_{false};
  std::size_t failed_deliveries_{0U};
};

class RuntimeEngine final
{
public:
  using TimePoint = SteadyClockPort::TimePoint;
  using Emergency = std::function<void()>;
  using EmergencyStop = std::function<bool(TimePoint)>;
  using RefreshEndpoint = std::function<void()>;

  struct ChildDependencies
  {
    std::shared_ptr<RelativeMotionPort> relative_motion;
    std::shared_ptr<NavigationPort> navigation;
    std::shared_ptr<MapStorePort> map_store;
  };

  struct MotionConditioningBindings
  {
    std::shared_ptr<RuntimeTransactionPlane> transaction_plane;
    std::function<bool(std::uint64_t)> admission_fence_check;
    std::function<bool(RelativeMotionCompletionRecordPtr)> completion_relay;
  };

  using ChildFactory = std::function<ChildDependencies(
        const MotionConditioningBindings &)>;

  struct GateSnapshotInput { GateSnapshot snapshot; };
  struct TickInput { TimePoint now{}; };
  struct ChildFeedbackInput { MotionToken token; double progress{0.0}; };
  struct ChildResultInput { MotionToken token; };
  using Input = std::variant<
    GateSnapshotInput, TickInput, ChildFeedbackInput, ChildResultInput>;

  struct ShutdownHooks
  {
    std::function<void(TimePoint)> begin_external_shutdown;
    std::function<bool(TimePoint)> wait_external_completion;
    std::function<void()> finalize_external_shutdown;
  };

  RuntimeEngine(
    RuntimeConfig config,
    std::shared_ptr<SteadyClockPort> clock,
    std::shared_ptr<MotionAuthorityPort> authority,
    GateSnapshot initial_gate_snapshot,
    ChildFactory child_factory,
    Emergency emergency = {},
    EmergencyStop emergency_stop = {},
    RefreshEndpoint refresh_endpoint = {},
    std::shared_ptr<RuntimeStateSink> state_sink = {});

  ~RuntimeEngine();

  RuntimeEngine(const RuntimeEngine &) = delete;
  RuntimeEngine & operator=(const RuntimeEngine &) = delete;

  [[nodiscard]] bool authorize_admission(
    const std::string & uuid,
    std::uint64_t admission_epoch);

  void submit_admission(
    const std::string & uuid,
    MissionGoal goal,
    std::shared_ptr<RuntimeGoalSink> sink) noexcept;

  void submit_cancel(const void * identity) noexcept;
  void post(Input input) noexcept;
  void submit_stop(const StopRequest & request, StopResponse & response) noexcept;
  [[nodiscard]] RuntimeState snapshot() const noexcept;

  [[nodiscard]] bool shutdown(
    TimePoint deadline,
    ShutdownHooks hooks = {}) noexcept;

private:
  struct AdmitEvent
  {
    MissionGoal goal;
    std::shared_ptr<RuntimeGoalSink> sink;
  };

  struct CancelEvent
  {
    const void * identity{nullptr};
  };

  struct StopWaiter
  {
    std::mutex mutex;
    std::condition_variable condition;
    bool completed{false};
    StopResponse response{};
  };

  struct StopEvent
  {
    StopRequest request;
    std::shared_ptr<StopWaiter> waiter;
  };

  struct TickEvent
  {
    TimePoint now{};
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

  struct ChildTerminalEvent
  {
    MotionToken token;
    ChildResult result;
  };

  struct QueueFaultEvent
  {
    std::string detail;
  };

  struct ShutdownFaultEvent
  {
    std::string detail;
  };

  struct ShutdownWaiter
  {
    std::mutex mutex;
    std::condition_variable condition;
    bool completed{false};
  };

  struct ShutdownEvent
  {
    std::shared_ptr<ShutdownWaiter> waiter;
  };

  using EventPayload = std::variant<
    AdmitEvent,
    CancelEvent,
    StopEvent,
    TickEvent,
    GateSnapshotEvent,
    ChildFeedbackEvent,
    ChildResultEvent,
    ChildTerminalEvent,
    QueueFaultEvent,
    ShutdownFaultEvent,
    ShutdownEvent>;

  struct Event
  {
    std::uint64_t generation{0U};
    EventPayload payload;
  };

  using Queue = RuntimeEventQueue<Event>;

  [[nodiscard]] bool relay_completion(
    RelativeMotionCompletionRecordPtr record) noexcept;
  [[nodiscard]] bool close_admission() noexcept;
  [[nodiscard]] bool wait_for_action_admission_drain() noexcept;
  [[nodiscard]] static Queue::Lane event_lane(const Event & event) noexcept;
  [[nodiscard]] bool enqueue(Event event) noexcept;
  [[nodiscard]] bool enqueue_internal(Event event) noexcept;
  void request_emergency_fence(std::string detail) noexcept;
  void process(Event & event);
  void process(AdmitEvent & event);
  void process(CancelEvent & event);
  void process(StopEvent & event);
  void process(TickEvent & event);
  void process(GateSnapshotEvent & event);
  void process(ChildFeedbackEvent & event);
  void process(ChildResultEvent & event);
  void process(ChildTerminalEvent & event);
  void process(QueueFaultEvent & event);
  void process(ShutdownFaultEvent & event);
  void process(ShutdownEvent & event);
  void reject_queued_admissions() noexcept;
  void process_pending_shutdown() noexcept;
  void run_worker() noexcept;
  void finish_goal(
    std::uint64_t mission_id,
    const ActionResultDelivery & delivery);
  void publish_feedback(
    std::uint64_t mission_id,
    const MissionFeedback & feedback);
  void publish_state(const RuntimeState & state);
  bool flush_external_deliveries() noexcept;
  void update_runtime_terminal() noexcept;
  [[nodiscard]] bool queue_delivery(
    std::shared_ptr<RuntimeGoalSink> sink,
    ActionResultDelivery delivery) noexcept;
  void commit_terminal(std::uint64_t mission_id) noexcept;
  void enqueue_shutdown_fault(std::string detail) noexcept;
  [[nodiscard]] bool wait_for_runtime_terminal_until(TimePoint deadline) noexcept;
  [[nodiscard]] bool wait_for_shutdown_conditions(
    TimePoint deadline,
    const ShutdownHooks & hooks) noexcept;
  void fail_closed(std::string detail) noexcept;

  RuntimeConfig config_;
  std::shared_ptr<SteadyClockPort> clock_;
  std::shared_ptr<MotionAuthorityPort> authority_;
  std::shared_ptr<RelativeMotionPort> relative_motion_;
  std::shared_ptr<NavigationPort> navigation_;
  std::shared_ptr<MapStorePort> map_store_;
  GateSnapshot initial_gate_snapshot_;
  Emergency emergency_;
  EmergencyStop emergency_stop_;
  RefreshEndpoint refresh_endpoint_;
  std::shared_ptr<RuntimeStateSink> state_sink_;
  std::shared_ptr<RuntimeDeliveryState> delivery_state_;

  RuntimeAdmissionGate admission_gate_;
  ActionAdmissionTracker admission_tracker_;
  Queue event_queue_;
  RuntimeEmergencyFence emergency_fence_;
  RuntimeEventIngress<Event> event_ingress_;
  std::unique_ptr<RuntimeExecutionPlane> execution_plane_;
  std::thread worker_;
  MissionActionAdapterBoundary action_adapter_;

  std::unordered_map<std::uint64_t, std::shared_ptr<RuntimeGoalSink>> goals_;
  std::unordered_map<const void *, std::uint64_t> goal_ids_;
  std::unordered_set<const void *> pending_cancels_;
  mutable std::mutex state_mutex_;
  RuntimeState cached_state_{};
  std::condition_variable terminal_condition_;
  mutable std::mutex terminal_mutex_;
  bool runtime_idle_{true};
  struct PendingDelivery
  {
    std::uint64_t mission_id{0U};
    std::shared_ptr<RuntimeGoalSink> sink;
    ActionResultDelivery delivery;
  };
  std::deque<PendingDelivery> pending_deliveries_;
  std::unordered_set<std::uint64_t> pending_terminals_;
  bool worker_shutdown_complete_{false};
  mutable std::mutex shutdown_mutex_;
  bool shutdown_started_{false};
  bool shutdown_complete_{false};
  bool shutdown_requested_{false};
  std::shared_ptr<ShutdownWaiter> shutdown_waiter_;
};

}  // namespace voice_nav_mission

#endif  // RUNTIME_ENGINE_HPP_
