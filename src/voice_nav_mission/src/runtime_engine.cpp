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

#include "runtime_engine.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "voice_nav_mission/motion_conditioning_pipeline.hpp"

namespace voice_nav_mission
{

namespace
{

[[nodiscard]] ActionResultDelivery aborted(const MissionResult & result)
{
  return ActionResultDelivery{OuterActionStatus::Aborted, result};
}

}  // namespace

RuntimeDeliveryState::~RuntimeDeliveryState()
{
  (void)close(std::chrono::steady_clock::now());
}

void RuntimeDeliveryState::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (worker_.joinable() || detached_) {
    return;
  }
  const auto self = shared_from_this();
  worker_ = std::thread([self]() {self->run();});
}

bool RuntimeDeliveryState::submit_result(
  std::shared_ptr<RuntimeGoalSink> sink,
  ActionResultDelivery delivery) noexcept
{
  return submit(
    Task{Kind::Result, std::move(sink), std::move(delivery), {}, {}, {}});
}

bool RuntimeDeliveryState::submit_feedback(
  std::shared_ptr<RuntimeGoalSink> sink,
  MissionFeedback feedback) noexcept
{
  return submit(
    Task{Kind::Feedback, std::move(sink), {}, {}, {}, std::move(feedback)});
}

bool RuntimeDeliveryState::submit_state(
  std::shared_ptr<RuntimeStateSink> sink,
  RuntimeState state) noexcept
{
  return submit(
    Task{Kind::State, {}, {}, std::move(sink), std::move(state), {}});
}

bool RuntimeDeliveryState::submit(Task task) noexcept
{
  try {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) {
        return false;
      }
      if (task.kind == Kind::Result) {
        if (!task.sink || terminal_tasks_.size() +
          (terminal_inflight_ ? 1U : 0U) >= kTerminalCapacity)
        {
          return false;
        }
        terminal_tasks_.push_back(std::move(task));
      } else if (task.kind == Kind::Feedback) {
        if (!task.sink) {
          return false;
        }
        latest_feedback_.emplace(std::move(task));
      } else {
        if (!task.state_sink) {
          return false;
        }
        if (state_disabled_) {
          return true;
        }
        latest_state_.emplace(std::move(task));
      }
    }
    condition_.notify_one();
    return true;
  } catch (...) {
    return false;
  }
}

bool RuntimeDeliveryState::wait_for_terminal_capacity(
  const TimePoint deadline) noexcept
{
  std::unique_lock<std::mutex> lock(mutex_);
  return condition_.wait_until(lock, deadline, [this]() {
             return detached_ || closed_ || terminal_tasks_.size() +
                    (terminal_inflight_ ? 1U : 0U) < kTerminalCapacity;
         });
}

bool RuntimeDeliveryState::close(const TimePoint deadline) noexcept
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (detached_) {
      return false;
    }
    closed_ = true;
  }
  condition_.notify_all();
  if (!worker_.joinable()) {
    return true;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  const auto drained = condition_.wait_until(lock, deadline, [this]() {
        return terminal_tasks_.empty() && !latest_feedback_.has_value() &&
               !latest_state_.has_value() && !terminal_inflight_ &&
               !external_inflight_;
    });
  if (drained) {
    lock.unlock();
    worker_.join();
    return true;
  }
  // The worker may still be inside an arbitrary GoalHandle call. The shared
  // worker retains only this state and GoalSink references.
  detached_ = true;
  lock.unlock();
  worker_.detach();
  return false;
}

void RuntimeDeliveryState::record_failure(const Task & task) noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (task.kind == Kind::State) {
    state_disabled_ = true;
    latest_state_.reset();
  } else {
    ++failed_deliveries_;
  }
  condition_.notify_all();
}

void RuntimeDeliveryState::run() noexcept
{
  for (;; ) {
    Task task;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]() {
          return closed_ || !terminal_tasks_.empty() || latest_feedback_.has_value() ||
                 latest_state_.has_value();
        });
      if (terminal_tasks_.empty() && !latest_feedback_.has_value() &&
        !latest_state_.has_value())
      {
        return;
      }
      if (!terminal_tasks_.empty()) {
        task = std::move(terminal_tasks_.front());
        terminal_tasks_.pop_front();
        terminal_inflight_ = true;
      } else if (latest_feedback_.has_value()) {
        task = std::move(*latest_feedback_);
        latest_feedback_.reset();
        external_inflight_ = true;
      } else {
        task = std::move(*latest_state_);
        latest_state_.reset();
        external_inflight_ = true;
      }
    }
    try {
      if (task.kind == Kind::Result) {
        if (!task.sink) {
          throw std::runtime_error("Runtime delivery has no sink");
        }
        task.sink->deliver(task.result);
      } else if (task.kind == Kind::Feedback) {
        if (!task.sink) {
          throw std::runtime_error("Runtime feedback delivery has no sink");
        }
        task.sink->feedback(task.feedback);
      } else {
        if (!task.state_sink) {
          throw std::runtime_error("Runtime state delivery has no sink");
        }
        task.state_sink->publish(task.state);
      }
    } catch (const std::exception &) {
      record_failure(task);
    } catch (...) {
      record_failure(task);
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (task.kind == Kind::Result) {
        terminal_inflight_ = false;
      } else {
        external_inflight_ = false;
      }
    }
    condition_.notify_all();
  }
}

RuntimeEngine::RuntimeEngine(
  RuntimeConfig config,
  std::shared_ptr<SteadyClockPort> clock,
  std::shared_ptr<MotionAuthorityPort> authority,
  GateSnapshot initial_gate_snapshot,
  ChildFactory child_factory,
  Emergency emergency,
  EmergencyStop emergency_stop,
  RefreshEndpoint refresh_endpoint,
  std::shared_ptr<RuntimeStateSink> state_sink)
: config_(std::move(config)),
  clock_(std::move(clock)),
  authority_(std::move(authority)),
  initial_gate_snapshot_(std::move(initial_gate_snapshot)),
  emergency_(std::move(emergency)),
  emergency_stop_(std::move(emergency_stop)),
  refresh_endpoint_(std::move(refresh_endpoint)),
  state_sink_(std::move(state_sink)),
  delivery_state_(std::make_shared<RuntimeDeliveryState>()),
  admission_gate_(),
  admission_tracker_([this]() {return clock_->now();}),
  event_queue_([]() {
      return Event{0U, QueueFaultEvent{"Runtime event queue overflow"}};
    }),
  emergency_fence_(config.initial_admission_epoch),
  event_ingress_(
    event_queue_,
    emergency_fence_,
    [](const Event & event) {return RuntimeEngine::event_lane(event);},
    [this]() {
      try {
        if (emergency_) {
          emergency_();
        }
      } catch (...) {
      }
    },
    [this](const RuntimeEmergencyFenceSnapshot & snapshot) {
      fail_closed(snapshot.detail);
      flush_external_deliveries();
      if (execution_plane_) {
        execution_plane_->completion_mailbox().request_reap();
      }
    },
    [](const Event & event) {
      return std::holds_alternative<CancelEvent>(event.payload) ||
             std::holds_alternative<StopEvent>(event.payload) ||
             std::holds_alternative<ChildResultEvent>(event.payload) ||
             std::holds_alternative<ChildTerminalEvent>(event.payload) ||
             std::holds_alternative<QueueFaultEvent>(event.payload) ||
             std::holds_alternative<ShutdownFaultEvent>(event.payload) ||
             std::holds_alternative<ShutdownEvent>(event.payload);
    },
    {},
    [this]() {process_pending_shutdown();})
{
  if (!clock_ || !authority_) {
    throw std::invalid_argument("RuntimeEngine requires clock and authority");
  }
  config_.admission_epoch_advance =
    [this](const std::uint64_t current, const std::uint64_t next) {
      return emergency_fence_.advance_epoch(current, next);
    };
  if (!child_factory) {
    throw std::invalid_argument("RuntimeEngine requires child dependencies");
  }
  const MotionConditioningBindings bindings{
    admission_gate_.transaction_plane(),
    [this](const std::uint64_t epoch) {
      return admission_gate_.admission_allowed(
        epoch,
        [this](const std::uint64_t value) {
          return event_ingress_.admission_allowed(value);
        });
    },
    [this](RelativeMotionCompletionRecordPtr record) {
      return relay_completion(std::move(record));
    }};
  const auto children = child_factory(bindings);
  if (!children.relative_motion) {
    throw std::invalid_argument("RuntimeEngine child factory returned no motion port");
  }
  relative_motion_ = children.relative_motion;
  navigation_ = children.navigation;
  map_store_ = children.map_store;
  execution_plane_ = std::make_unique<RuntimeExecutionPlane>(
    config_,
    clock_,
    authority_,
    relative_motion_,
    [this](const RuntimeState & state) {publish_state(state);},
    [this](std::uint64_t mission_id, const MissionFeedback & feedback) {
      publish_feedback(mission_id, feedback);
    },
    [this](std::uint64_t mission_id, const MissionResult & result) {
      action_adapter_.finish(mission_id, result);
    },
    [this](const MotionToken & token, const double progress) {
      return enqueue_internal(Event{
        token.mission_generation, ChildFeedbackEvent{token, progress}});
    },
    bindings.admission_fence_check,
    [this](const MotionToken & token) {
      return enqueue_internal(Event{token.mission_generation, ChildResultEvent{token}});
    },
    [this](std::string detail) {request_emergency_fence(std::move(detail));},
    RuntimeExecutionPlane::ChildResultDeliveryDecorator{},
    navigation_,
    map_store_,
    [this](const MotionToken & token, const ChildResult & result) {
      return enqueue_internal(Event{
        token.mission_generation, ChildTerminalEvent{token, result}});
    },
    [this](const MotionToken & token, const ChildResult & result) {
      return enqueue_internal(Event{
        token.mission_generation, ChildTerminalEvent{token, result}});
    });
  delivery_state_->start();
  worker_ = std::thread([this]() {run_worker();});
  (void)enqueue_internal(Event{0U, GateSnapshotEvent{initial_gate_snapshot_}});
}

RuntimeEngine::~RuntimeEngine()
{
  const auto deadline = std::chrono::steady_clock::now() +
    config_.stop_barrier + config_.stationarity_deadline;
  (void)shutdown(deadline, {});
}

RuntimeEventQueue<RuntimeEngine::Event>::Lane RuntimeEngine::event_lane(
  const Event & event) noexcept
{
  return std::holds_alternative<CancelEvent>(event.payload) ||
         std::holds_alternative<StopEvent>(event.payload) ||
         std::holds_alternative<ChildResultEvent>(event.payload) ||
         std::holds_alternative<ChildTerminalEvent>(event.payload) ||
         std::holds_alternative<QueueFaultEvent>(event.payload) ||
         std::holds_alternative<ShutdownFaultEvent>(event.payload) ||
         std::holds_alternative<ShutdownEvent>(event.payload) ?
         Queue::Lane::Control : Queue::Lane::Normal;
}

bool RuntimeEngine::authorize_admission(
  const std::string & uuid,
  const std::uint64_t admission_epoch)
{
  return admission_gate_.try_provision(
    admission_tracker_,
    uuid,
    admission_epoch,
    [this](const std::uint64_t epoch) {
      return !event_ingress_.blocked() && event_ingress_.admission_allowed(epoch);
    });
}

void RuntimeEngine::submit_admission(
  const std::string & uuid,
  MissionGoal goal,
  std::shared_ptr<RuntimeGoalSink> sink) noexcept
{
  if (!sink) {
    return;
  }
  const auto reject = [this, &sink](std::string detail) noexcept {
      (void)queue_delivery(sink, aborted(MissionResult{
        MissionResultCode::SafetyFault, -1, std::move(detail)}));
      flush_external_deliveries();
    };
  auto lease = admission_tracker_.enter_accepted(uuid);
  if (!lease.has_ticket() || lease.was_revoked()) {
    reject("Runtime admission was revoked before dispatch");
    return;
  }
  auto admission_sink = sink;
  const auto queued = admission_gate_.submit(
    [this, goal = std::move(goal), sink = std::move(admission_sink)]() mutable {
      if (event_ingress_.blocked() ||
      !event_ingress_.admission_allowed(goal.admission_epoch))
      {
        return false;
      }
      return enqueue_internal(Event{0U, AdmitEvent{std::move(goal), std::move(sink)}});
    });
  if (!queued) {
    reject(admission_gate_.quiescing() ?
      "Runtime is quiescing" :
      "Runtime admission queue could not accept Mission admission");
  }
}

void RuntimeEngine::submit_cancel(const void * identity) noexcept
{
  (void)enqueue(Event{0U, CancelEvent{identity}});
}

void RuntimeEngine::post(Input input) noexcept
{
  std::visit(
    [this](auto & value) {
      using Value = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<Value, GateSnapshotInput>) {
        (void)enqueue_internal(Event{0U, GateSnapshotEvent{std::move(value.snapshot)}});
      } else if constexpr (std::is_same_v<Value, TickInput>) {
        (void)enqueue(Event{0U, TickEvent{value.now}});
      } else if constexpr (std::is_same_v<Value, ChildFeedbackInput>) {
        (void)enqueue_internal(Event{
          value.token.mission_generation,
          ChildFeedbackEvent{value.token, value.progress}});
      } else {
        (void)enqueue_internal(Event{
          value.token.mission_generation, ChildResultEvent{value.token}});
      }
    },
    input);
}

bool RuntimeEngine::relay_completion(
  RelativeMotionCompletionRecordPtr record) noexcept
{
  return execution_plane_ && execution_plane_->completion_mailbox().relay(
    std::move(record));
}

void RuntimeEngine::submit_stop(
  const StopRequest & request,
  StopResponse & response) noexcept
{
  auto waiter = std::make_shared<StopWaiter>();
  if (!enqueue(Event{0U, StopEvent{request, waiter}})) {
    bool zero_proven = false;
    try {
      if (emergency_stop_) {
        zero_proven = emergency_stop_(
          std::chrono::steady_clock::now() +
          config_.stationarity_deadline + config_.stop_barrier);
      }
    } catch (...) {
    }
    const auto state = snapshot();
    response = StopResponse{
      2U, state.runtime_instance_id, state.admission_epoch, zero_proven,
      "Runtime event queue could not accept STOP"};
    return;
  }
  std::unique_lock<std::mutex> lock(waiter->mutex);
  const auto deadline = std::chrono::steady_clock::now() +
    config_.stationarity_deadline + config_.stop_barrier;
  if (!waiter->condition.wait_until(
      lock, deadline, [waiter]() {return waiter->completed;}))
  {
    try {
      if (emergency_) {
        emergency_();
      }
    } catch (...) {
    }
    const auto state = snapshot();
    response = StopResponse{
      2U, state.runtime_instance_id, state.admission_epoch,
      relative_motion_ && relative_motion_->zero_proven(),
      "STOP response deadline expired before zero proof"};
    return;
  }
  response = waiter->response;
}

RuntimeState RuntimeEngine::snapshot() const noexcept
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cached_state_;
}

bool RuntimeEngine::close_admission() noexcept
{
  return admission_gate_.close_generation(admission_tracker_);
}

bool RuntimeEngine::wait_for_action_admission_drain() noexcept
{
  (void)admission_tracker_.revoke_expired(clock_->now());
  auto deadline = std::chrono::steady_clock::now() +
    ActionAdmissionTracker::kDefaultHandoffDeadline;
  if (admission_tracker_.wait_for_drain_until(deadline)) {
    return true;
  }
  (void)admission_tracker_.revoke_all_provisional(clock_->now());
  deadline = std::chrono::steady_clock::now() +
    ActionAdmissionTracker::kDefaultHandoffDeadline;
  return admission_tracker_.wait_for_drain_until(deadline);
}

bool RuntimeEngine::shutdown(
  const TimePoint deadline,
  ShutdownHooks hooks) noexcept
{
  {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    if (shutdown_complete_) {
      return true;
    }
    if (shutdown_started_) {
      return false;
    }
    shutdown_started_ = true;
  }

  bool drained = true;
  if (execution_plane_) {
    RuntimeShutdownCoordinator coordinator(
      [this]() {return close_admission();},
      [&hooks](const TimePoint value) {
        if (hooks.begin_external_shutdown) {
          hooks.begin_external_shutdown(value);
        }
      },
      [this, &hooks](const TimePoint value) {
        return wait_for_shutdown_conditions(value, hooks);
      },
      [this]() {
        try {
          if (emergency_) {
            emergency_();
          }
        } catch (...) {
        }
      },
      [this](std::string detail) {request_emergency_fence(std::move(detail));},
      [this](std::string detail) {enqueue_shutdown_fault(std::move(detail));});
    const auto outcome = coordinator.run(deadline);
    drained = outcome.transaction_drained;
  } else {
    (void)close_admission();
  }

  if (!wait_for_action_admission_drain()) {
    drained = false;
    try {
      if (emergency_) {
        emergency_();
      }
    } catch (...) {
    }
    request_emergency_fence("Action admission drain deadline expired");
  }

  if (execution_plane_) {
    auto waiter = std::make_shared<ShutdownWaiter>();
    {
      std::lock_guard<std::mutex> lock(shutdown_mutex_);
      shutdown_requested_ = true;
      shutdown_waiter_ = waiter;
    }
    if (!enqueue_internal(Event{0U, ShutdownEvent{waiter}})) {
      drained = false;
      request_emergency_fence("Runtime shutdown event queue rejected shutdown");
    }
    event_queue_.wake();
    {
      std::unique_lock<std::mutex> lock(waiter->mutex);
      if (!waiter->condition.wait_until(
          lock, deadline, [waiter]() {return waiter->completed;}))
      {
        drained = false;
      }
    }
  }
  if (delivery_state_) {
    while (!flush_external_deliveries()) {
      if (!delivery_state_->wait_for_terminal_capacity(deadline)) {
        drained = false;
        break;
      }
    }
  }
  if (delivery_state_ && !delivery_state_->close(deadline)) {
    drained = false;
  }
  event_queue_.close();
  if (worker_.joinable()) {
    worker_.join();
  }
  if (hooks.finalize_external_shutdown) {
    try {
      hooks.finalize_external_shutdown();
    } catch (...) {
      drained = false;
    }
  }
  execution_plane_.reset();
  relative_motion_.reset();
  authority_.reset();
  admission_tracker_.clear();
  {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    shutdown_complete_ = true;
  }
  return drained;
}

bool RuntimeEngine::enqueue(Event event) noexcept
{
  return admission_gate_.submit([this, event = std::move(event)]() mutable {
             return enqueue_internal(std::move(event));
    });
}

bool RuntimeEngine::enqueue_internal(Event event) noexcept
{
  return event_ingress_.enqueue(std::move(event));
}

void RuntimeEngine::request_emergency_fence(std::string detail) noexcept
{
  event_ingress_.request_emergency(std::move(detail));
}

void RuntimeEngine::run_worker() noexcept
{
  event_ingress_.run(
    [this](Event & event) {process(event);},
    [this](std::string detail) {request_emergency_fence(std::move(detail));});
}

void RuntimeEngine::process(Event & event)
{
  if (std::holds_alternative<ShutdownEvent>(event.payload)) {
    std::visit([this](auto & value) {process(value);}, event.payload);
    return;
  }
  if (!execution_plane_ || !execution_plane_->core()) {
    return;
  }
  {
    std::lock_guard<std::recursive_mutex> lock(
      execution_plane_->core_serial_mutex());
    std::visit([this](auto & value) {process(value);}, event.payload);
    update_runtime_terminal();
  }
  flush_external_deliveries();
  update_runtime_terminal();
}

void RuntimeEngine::process(AdmitEvent & event)
{
  if (!execution_plane_ || !execution_plane_->core()) {
    (void)queue_delivery(event.sink, aborted(MissionResult{
        MissionResultCode::SafetyFault, -1,
        "Runtime execution plane is unavailable"}));
    return;
  }
  const auto permit = admission_gate_.claim_start(event.goal.admission_epoch);
  if (!permit.issued) {
    (void)queue_delivery(event.sink, aborted(MissionResult{
        MissionResultCode::SafetyFault, -1,
        "Runtime admission was quiesced before dispatch"}));
    return;
  }
  std::uint64_t mission_id = 0U;
  bool cancel_after_admission = false;
  action_adapter_.on_accepted(
    event.goal,
    [this, permit](const MissionGoal & value) {
      return execution_plane_->core()->admit(
        value,
        [this, permit]() {
          return admission_gate_.start_allowed(permit) &&
                 admission_gate_.admission_allowed(
                   permit.admission_epoch,
            [this](const std::uint64_t epoch) {
              return event_ingress_.admission_allowed(epoch);
                   });
        },
        permit.generation);
    },
    [this, &event, &mission_id, &cancel_after_admission](
      const std::uint64_t value) {
      const auto identity = event.sink ? event.sink->identity() : nullptr;
      {
        std::lock_guard<std::mutex> lock(terminal_mutex_);
        goals_.emplace(value, event.sink);
        if (identity != nullptr) {
          goal_ids_.emplace(identity, value);
        }
        mission_id = value;
      }
      if (identity != nullptr) {
        const auto found = pending_cancels_.find(identity);
        if (found != pending_cancels_.end()) {
          pending_cancels_.erase(found);
          cancel_after_admission = true;
        }
      }
    },
    [this](const std::uint64_t value, const ActionResultDelivery & delivery) {
      finish_goal(value, delivery);
    },
    [this, &event](const MissionResult & result) {
      (void)queue_delivery(event.sink, aborted(result));
    });
  if (cancel_after_admission && mission_id != 0U) {
    execution_plane_->core()->cancel(mission_id);
  }
}

void RuntimeEngine::process(CancelEvent & event)
{
  std::uint64_t mission_id = 0U;
  {
    std::lock_guard<std::mutex> lock(terminal_mutex_);
    const auto found = goal_ids_.find(event.identity);
    if (found == goal_ids_.end()) {
      pending_cancels_.insert(event.identity);
      return;
    }
    mission_id = found->second;
  }
  execution_plane_->core()->cancel(mission_id);
}

void RuntimeEngine::process(StopEvent & event)
{
  StopResponse result{
    2U, snapshot().runtime_instance_id,
    snapshot().admission_epoch, false, "Runtime execution plane unavailable"};
  try {
    if (execution_plane_ && execution_plane_->core()) {
      result = execution_plane_->core()->stop(event.request);
    }
  } catch (const std::exception & error) {
    try {
      if (emergency_) {
        emergency_();
      }
    } catch (...) {
    }
    result.detail = std::string{"STOP worker raised: "} + error.what();
    fail_closed(result.detail);
  } catch (...) {
    try {
      if (emergency_) {
        emergency_();
      }
    } catch (...) {
    }
    result.detail = "STOP worker raised an unknown exception";
    fail_closed(result.detail);
  }
  {
    std::lock_guard<std::mutex> lock(event.waiter->mutex);
    event.waiter->response = std::move(result);
    event.waiter->completed = true;
  }
  event.waiter->condition.notify_one();
}

void RuntimeEngine::process(TickEvent & event)
{
  (void)admission_tracker_.revoke_expired(event.now);
  if (refresh_endpoint_) {
    refresh_endpoint_();
  }
  execution_plane_->core()->on_tick();
}

void RuntimeEngine::process(GateSnapshotEvent & event)
{
  execution_plane_->core()->observe_gate(event.snapshot);
}

void RuntimeEngine::process(ChildFeedbackEvent & event)
{
  execution_plane_->core()->on_child_feedback(event.token, event.progress);
}

void RuntimeEngine::process(ChildResultEvent & event)
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

void RuntimeEngine::process(ChildTerminalEvent & event)
{
  execution_plane_->core()->on_child_result(event.token, event.result);
}

void RuntimeEngine::process(QueueFaultEvent & event)
{
  request_emergency_fence(event.detail);
}

void RuntimeEngine::process(ShutdownFaultEvent & event)
{
  const auto detail = event.detail;
  fail_closed(std::move(event.detail));
  request_emergency_fence(detail);
}

void RuntimeEngine::process(ShutdownEvent & event)
{
  reject_queued_admissions();
  flush_external_deliveries();
  if (execution_plane_) {
    execution_plane_->shutdown();
  }
  {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    shutdown_requested_ = false;
    worker_shutdown_complete_ = true;
  }
  if (event.waiter) {
    {
      std::lock_guard<std::mutex> lock(event.waiter->mutex);
      event.waiter->completed = true;
    }
    event.waiter->condition.notify_one();
  }
}

void RuntimeEngine::reject_queued_admissions() noexcept
{
  auto pending = event_queue_.drain_normal();
  for (auto & event : pending) {
    if (auto * admit = std::get_if<AdmitEvent>(&event.payload)) {
      (void)queue_delivery(admit->sink, aborted(MissionResult{
          MissionResultCode::SafetyFault, -1,
          "Runtime admission was quiesced before dispatch"}));
    }
  }
}

void RuntimeEngine::process_pending_shutdown() noexcept
{
  std::shared_ptr<ShutdownWaiter> waiter;
  {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    if (!shutdown_requested_) {
      return;
    }
    waiter = shutdown_waiter_;
  }
  ShutdownEvent event{std::move(waiter)};
  process(event);
}

void RuntimeEngine::finish_goal(
  const std::uint64_t mission_id,
  const ActionResultDelivery & delivery)
{
  {
    std::lock_guard<std::mutex> lock(terminal_mutex_);
    const auto found = goals_.find(mission_id);
    if (found == goals_.end()) {
      return;
    }
    if (found->second && pending_terminals_.insert(mission_id).second) {
      pending_deliveries_.push_back(
        PendingDelivery{mission_id, found->second, delivery});
    }
  }
}

void RuntimeEngine::publish_feedback(
  const std::uint64_t mission_id,
  const MissionFeedback & feedback)
{
  std::shared_ptr<RuntimeGoalSink> sink;
  {
    std::lock_guard<std::mutex> lock(terminal_mutex_);
    const auto found = goals_.find(mission_id);
    if (found != goals_.end() && pending_terminals_.count(mission_id) == 0U) {
      sink = found->second;
    }
  }
  if (sink && !delivery_state_->submit_feedback(std::move(sink), feedback)) {
    request_emergency_fence("Runtime feedback delivery queue is full");
  }
}

void RuntimeEngine::publish_state(const RuntimeState & state)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cached_state_ = state;
  }
  if (state_sink_) {
    (void)delivery_state_->submit_state(state_sink_, state);
  }
}

bool RuntimeEngine::queue_delivery(
  std::shared_ptr<RuntimeGoalSink> sink,
  ActionResultDelivery delivery) noexcept
{
  if (!sink) {
    return false;
  }
  try {
    std::lock_guard<std::mutex> lock(terminal_mutex_);
    pending_deliveries_.push_back(
      PendingDelivery{0U, std::move(sink), std::move(delivery)});
    return true;
  } catch (...) {
    return false;
  }
}

void RuntimeEngine::commit_terminal(const std::uint64_t mission_id) noexcept
{
  if (mission_id == 0U) {
    return;
  }
  std::lock_guard<std::mutex> lock(terminal_mutex_);
  pending_terminals_.erase(mission_id);
  const auto found = goals_.find(mission_id);
  if (found != goals_.end()) {
    if (found->second) {
      goal_ids_.erase(found->second->identity());
    }
    goals_.erase(found);
  }
}

bool RuntimeEngine::flush_external_deliveries() noexcept
{
  std::deque<PendingDelivery> deliveries;
  {
    std::lock_guard<std::mutex> lock(terminal_mutex_);
    deliveries.swap(pending_deliveries_);
  }
  for (auto & item : deliveries) {
    if (!delivery_state_->submit_result(item.sink, item.delivery)) {
      {
        std::lock_guard<std::mutex> lock(terminal_mutex_);
        pending_deliveries_.push_front(std::move(item));
      }
      request_emergency_fence("Runtime result delivery queue is full");
    } else {
      commit_terminal(item.mission_id);
    }
  }
  std::lock_guard<std::mutex> lock(terminal_mutex_);
  return pending_deliveries_.empty();
}

void RuntimeEngine::update_runtime_terminal() noexcept
{
  bool idle = true;
  {
    std::lock_guard<std::mutex> lock(terminal_mutex_);
    idle = goals_.empty();
  }
  if (execution_plane_ && execution_plane_->core()) {
    idle = idle && !execution_plane_->core()->has_active_mission();
  }
  {
    std::lock_guard<std::mutex> lock(terminal_mutex_);
    runtime_idle_ = idle;
  }
  if (idle) {
    terminal_condition_.notify_all();
  }
}

void RuntimeEngine::enqueue_shutdown_fault(std::string detail) noexcept
{
  if (!enqueue_internal(Event{0U, ShutdownFaultEvent{std::move(detail)}})) {
    request_emergency_fence("Runtime shutdown fault event was rejected");
  }
}

bool RuntimeEngine::wait_for_runtime_terminal_until(const TimePoint deadline) noexcept
{
  std::unique_lock<std::mutex> lock(terminal_mutex_);
  return terminal_condition_.wait_until(lock, deadline, [this]() {
             return runtime_idle_;
    });
}

bool RuntimeEngine::wait_for_shutdown_conditions(
  const TimePoint deadline,
  const ShutdownHooks & hooks) noexcept
{
  bool external_complete = true;
  try {
    external_complete = !hooks.wait_external_completion ||
      hooks.wait_external_completion(deadline);
  } catch (...) {
    external_complete = false;
  }
  bool gate_safe = false;
  try {
    const auto snapshot = authority_->snapshot();
    gate_safe = relative_motion_ && relative_motion_->zero_proven() &&
      snapshot.state == GateState::Inhibited && snapshot.motion_inhibited &&
      snapshot.zero_selected && snapshot.zero_published;
  } catch (...) {
    gate_safe = false;
  }
  const auto transaction_drained = admission_gate_.wait_for_transaction_drain(deadline);
  return external_complete && gate_safe && transaction_drained &&
         wait_for_runtime_terminal_until(deadline);
}

void RuntimeEngine::fail_closed(std::string detail) noexcept
{
  if (!execution_plane_ || !execution_plane_->core()) {
    return;
  }
  try {
    std::lock_guard<std::recursive_mutex> lock(
      execution_plane_->core_serial_mutex());
    execution_plane_->core()->fail_closed_at_epoch(
      emergency_fence_.admission_epoch(), std::move(detail));
  } catch (...) {
  }
}

}  // namespace voice_nav_mission
