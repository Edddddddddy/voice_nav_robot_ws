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

#include "../src/runtime_engine.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;

RuntimeConfig test_config()
{
  RuntimeConfig config;
  config.runtime_instance_id = "0123456789abcdef0123456789abcdef";
  config.identifier_generator = []() {return std::string(32U, 'a');};
  config.mission_deadline = 1s;
  config.stop_barrier = 20ms;
  config.cancel_grace = 20ms;
  config.stationarity_deadline = 50ms;
  return config;
}

MissionGoal make_goal(
  const RuntimeConfig & config,
  std::string source,
  const std::uint64_t sequence,
  const std::uint64_t epoch = 1U)
{
  return MissionGoal{
    std::move(source), sequence, config.runtime_instance_id, epoch,
    {MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.5F, 0.0F,
        ""}}};
}

class StateRecorder final
{
public:
  void record(const RuntimeState & state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_ = state;
    condition_.notify_all();
  }

  bool wait_for(
    const std::function<bool(const RuntimeState &)> & predicate,
    const std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this, &predicate]() {
               return predicate(last_);
      });
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  RuntimeState last_{};
};

class RecordingStateSink final : public RuntimeStateSink
{
public:
  explicit RecordingStateSink(StateRecorder & recorder)
  : recorder_(recorder)
  {
  }

  void publish(const RuntimeState & state) override
  {
    recorder_.record(state);
  }

private:
  StateRecorder & recorder_;
};

class RecordingSink final : public RuntimeGoalSink
{
public:
  [[nodiscard]] const void * identity() const noexcept override {return this;}

  void deliver(const ActionResultDelivery & delivery) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    deliveries_.push_back(delivery);
    condition_.notify_all();
  }

  void feedback(const MissionFeedback &) override {}

  bool wait_for_count(const std::size_t count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this, count]() {
               return deliveries_.size() >= count;
      });
  }

  std::size_t count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return deliveries_.size();
  }

  ActionResultDelivery last() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return deliveries_.back();
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<ActionResultDelivery> deliveries_;
};

class BlockingSink final : public RuntimeGoalSink
{
public:
  [[nodiscard]] const void * identity() const noexcept override {return this;}

  void deliver(const ActionResultDelivery & delivery) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    delivery_ = delivery;
    entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return released_;});
  }

  void feedback(const MissionFeedback &) override {}

  bool wait_for_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this]() {return entered_;});
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

  bool wait_for_result()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this]() {
               return delivery_.has_value();
      });
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<ActionResultDelivery> delivery_;
  bool entered_{false};
  bool released_{false};
};

class DeliveryTestSink final : public RuntimeGoalSink
{
public:
  enum class Failure {None, Feedback, Result};

  explicit DeliveryTestSink(const Failure failure = Failure::None)
  : failure_(failure)
  {
  }

  [[nodiscard]] const void * identity() const noexcept override {return this;}

  void deliver(const ActionResultDelivery & delivery) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (failure_ == Failure::Result) {
        attempted_ = true;
      } else {
        deliveries_.push_back(delivery);
      }
    }
    condition_.notify_all();
    if (failure_ == Failure::Result) {
      throw std::runtime_error("result sink failure");
    }
  }

  void feedback(const MissionFeedback &) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (failure_ == Failure::Feedback) {
      attempted_ = true;
      lock.unlock();
      condition_.notify_all();
      throw std::runtime_error("feedback sink failure");
    }
    feedback_entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return feedback_released_;});
  }

  bool wait_for_feedback()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this]() {return feedback_entered_;});
  }

  bool wait_for_attempt()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this]() {return attempted_;});
  }

  void release_feedback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    feedback_released_ = true;
    condition_.notify_all();
  }

  bool wait_for_result()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this]() {return !deliveries_.empty();});
  }

  std::size_t result_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return deliveries_.size();
  }

  ActionResultDelivery result() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return deliveries_.front();
  }

private:
  const Failure failure_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<ActionResultDelivery> deliveries_;
  bool attempted_{false};
  bool feedback_entered_{false};
  bool feedback_released_{false};
};

class Latch final
{
public:
  void signal()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    signaled_ = true;
    condition_.notify_all();
  }

  bool wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this]() {return signaled_;});
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool signaled_{false};
};

class BlockingLatch final
{
public:
  void enter()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return released_;});
  }

  bool wait_for_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this]() {return entered_;});
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_{false};
  bool released_{false};
};

struct EngineHarness final
{
  std::shared_ptr<ScriptedSteadyClock> clock =
    std::make_shared<ScriptedSteadyClock>();
  std::shared_ptr<ScriptedMotionAuthorityPort> authority =
    std::make_shared<ScriptedMotionAuthorityPort>("engine-gate");
  std::shared_ptr<ScriptedRelativeMotionPort> relative =
    std::make_shared<ScriptedRelativeMotionPort>();
  RuntimeConfig config = test_config();
  StateRecorder states;
  std::unique_ptr<RuntimeEngine> engine;

  explicit EngineHarness(
    const bool healthy = true,
    RuntimeEngine::Emergency emergency = {},
    RuntimeEngine::RefreshEndpoint refresh_endpoint = {})
  {
    relative->set_healthy(healthy);
    engine = std::make_unique<RuntimeEngine>(
      config,
      clock,
      authority,
      authority->snapshot(),
      [relative = relative](const RuntimeEngine::MotionConditioningBindings &) {
        return RuntimeEngine::ChildDependencies{relative, {}, {}};
      },
      std::move(emergency),
      RuntimeEngine::EmergencyStop{},
      std::move(refresh_endpoint),
      std::make_shared<RecordingStateSink>(states));
  }

  ~EngineHarness()
  {
    if (engine) {
      (void)engine->shutdown(
        std::chrono::steady_clock::now() + 1s);
    }
  }
};

TEST(RuntimeEngine, AdmissionProducesOneResultThroughWorker)
{
  EngineHarness harness(false);
  auto sink = std::make_shared<RecordingSink>();

  ASSERT_TRUE(harness.engine->authorize_admission("engine-goal", 1U));
  harness.engine->submit_admission(
    "engine-goal", make_goal(harness.config, "source", 1U), sink);

  ASSERT_TRUE(sink->wait_for_count(1U));
  EXPECT_EQ(sink->last().result.code, MissionResultCode::DependencyUnavailable);
}

TEST(RuntimeEngine, QueuedAdmissionDuringShutdownGetsExactlyOneTerminal)
{
  BlockingLatch refresh;
  EngineHarness harness(
    true, {}, [&refresh]() {refresh.enter();});
  harness.engine->post(RuntimeEngine::TickInput{harness.clock->now()});
  ASSERT_TRUE(refresh.wait_for_entered());

  auto sink = std::make_shared<RecordingSink>();
  ASSERT_TRUE(harness.engine->authorize_admission("queued", 1U));
  harness.engine->submit_admission(
    "queued", make_goal(harness.config, "queued-source", 1U), sink);

  Latch quiesced;
  bool shutdown_result = false;
  std::thread shutdown_thread([&]() {
      RuntimeEngine::ShutdownHooks hooks;
      hooks.begin_external_shutdown = [&quiesced](const RuntimeEngine::TimePoint) {
        quiesced.signal();
      };
      shutdown_result = harness.engine->shutdown(
        std::chrono::steady_clock::now() + 1s, std::move(hooks));
    });
  ASSERT_TRUE(quiesced.wait());
  refresh.release();
  shutdown_thread.join();

  EXPECT_TRUE(shutdown_result);
  ASSERT_TRUE(sink->wait_for_count(1U));
  EXPECT_EQ(sink->count(), 1U);
  EXPECT_EQ(sink->last().result.code, MissionResultCode::SafetyFault);
}

TEST(RuntimeEngine, BlockingFeedbackCannotDisplaceTerminalResult)
{
  auto delivery = std::make_shared<RuntimeDeliveryState>();
  delivery->start();
  auto sink = std::make_shared<DeliveryTestSink>();
  ASSERT_TRUE(delivery->submit_feedback(sink, MissionFeedback{}));
  ASSERT_TRUE(sink->wait_for_feedback());
  for (std::size_t index = 0U; index < 96U; ++index) {
    (void)delivery->submit_feedback(sink, MissionFeedback{});
  }
  const ActionResultDelivery terminal{
    OuterActionStatus::Succeeded,
    MissionResult{MissionResultCode::Succeeded, 0, "terminal"}};
  EXPECT_TRUE(delivery->submit_result(sink, terminal));

  sink->release_feedback();
  ASSERT_TRUE(sink->wait_for_result());
  EXPECT_EQ(sink->result_count(), 1U);
  EXPECT_EQ(sink->result().result.detail, "terminal");
  EXPECT_TRUE(delivery->close(std::chrono::steady_clock::now() + 1s));
}

TEST(RuntimeEngine, ThrowingFeedbackDoesNotDropTerminalResult)
{
  auto delivery = std::make_shared<RuntimeDeliveryState>();
  delivery->start();
  auto sink = std::make_shared<DeliveryTestSink>(DeliveryTestSink::Failure::Feedback);
  ASSERT_TRUE(delivery->submit_feedback(sink, MissionFeedback{}));
  ASSERT_TRUE(sink->wait_for_attempt());
  ASSERT_TRUE(delivery->submit_result(
    sink,
    ActionResultDelivery{
        OuterActionStatus::Succeeded,
        MissionResult{MissionResultCode::Succeeded, 0, "terminal"}}));
  ASSERT_TRUE(sink->wait_for_result());
  EXPECT_TRUE(delivery->close(std::chrono::steady_clock::now() + 1s));
}

TEST(RuntimeEngine, AdmissionBusyAndStaleAreFencedBeforeWorker)
{
  EngineHarness harness;
  auto first = std::make_shared<RecordingSink>();
  auto busy = std::make_shared<RecordingSink>();

  ASSERT_TRUE(harness.engine->authorize_admission("first", 1U));
  harness.engine->submit_admission(
    "first", make_goal(harness.config, "first-source", 1U), first);
  ASSERT_TRUE(harness.states.wait_for([](const RuntimeState & state) {
      return state.availability == RuntimeAvailability::Busy &&
             state.active_step != kNoActiveMissionStep;
    }));

  EXPECT_FALSE(harness.engine->authorize_admission("stale", 0U));
  ASSERT_TRUE(harness.engine->authorize_admission("busy", 1U));
  harness.engine->submit_admission(
    "busy", make_goal(harness.config, "busy-source", 1U), busy);
  ASSERT_TRUE(busy->wait_for_count(1U));
  EXPECT_EQ(busy->last().result.code, MissionResultCode::Busy);

  harness.engine->submit_cancel(first->identity());
  ASSERT_TRUE(first->wait_for_count(1U));
  EXPECT_EQ(first->last().result.code, MissionResultCode::Canceled);
}

TEST(RuntimeEngine, CancelThenLateChildCannotCompleteNextGeneration)
{
  EngineHarness harness;
  auto first = std::make_shared<RecordingSink>();
  ASSERT_TRUE(harness.engine->authorize_admission("first", 1U));
  harness.engine->submit_admission(
    "first", make_goal(harness.config, "sequence-source", 1U), first);
  ASSERT_TRUE(harness.states.wait_for([](const RuntimeState & state) {
      return state.availability == RuntimeAvailability::Busy &&
             state.active_step != kNoActiveMissionStep;
    }));
  const auto old_token = harness.relative->started_tokens().front();

  harness.engine->submit_cancel(first->identity());
  ASSERT_TRUE(first->wait_for_count(1U));
  EXPECT_EQ(first->last().result.code, MissionResultCode::Canceled);

  auto second = std::make_shared<RecordingSink>();
  ASSERT_TRUE(harness.engine->authorize_admission("second", 1U));
  harness.engine->submit_admission(
    "second", make_goal(harness.config, "sequence-source", 2U), second);
  ASSERT_TRUE(harness.states.wait_for([](const RuntimeState & state) {
      return state.availability == RuntimeAvailability::Busy &&
             state.active_step != kNoActiveMissionStep;
    }));
  const auto new_token = harness.relative->started_tokens().back();

  harness.engine->post(RuntimeEngine::ChildResultInput{old_token});
  harness.relative->complete_token(new_token);
  ASSERT_TRUE(second->wait_for_count(1U));
  EXPECT_EQ(second->last().result.code, MissionResultCode::Succeeded);
  EXPECT_EQ(first->count(), 1U);
}

TEST(RuntimeEngine, StopCommitsResultAndRotatesAdmissionEpoch)
{
  EngineHarness harness;
  auto sink = std::make_shared<RecordingSink>();
  ASSERT_TRUE(harness.engine->authorize_admission("stop-goal", 1U));
  harness.engine->submit_admission(
    "stop-goal", make_goal(harness.config, "stop-source", 1U), sink);
  ASSERT_TRUE(harness.states.wait_for([](const RuntimeState & state) {
      return state.availability == RuntimeAvailability::Busy &&
             state.active_step != kNoActiveMissionStep;
    }));

  StopResponse response;
  harness.engine->submit_stop(
    StopRequest{"stop-request", "operator", 1U, "operator stop"}, response);

  ASSERT_TRUE(sink->wait_for_count(1U));
  EXPECT_EQ(response.code, 0U);
  EXPECT_EQ(response.admission_epoch, 2U);
  EXPECT_TRUE(response.motion_inhibited);
  EXPECT_EQ(sink->last().result.code, MissionResultCode::Stopped);
}

TEST(RuntimeEngine, TerminalDeliveryQueueIsBounded)
{
  auto delivery = std::make_shared<RuntimeDeliveryState>();
  delivery->start();
  auto sink = std::make_shared<BlockingSink>();
  const ActionResultDelivery terminal{
    OuterActionStatus::Aborted,
    MissionResult{MissionResultCode::DependencyUnavailable, -1, "terminal"}};
  ASSERT_TRUE(delivery->submit_result(sink, terminal));
  ASSERT_TRUE(sink->wait_for_entered());

  for (std::size_t index = 1U; index < ActionAdmissionTracker::kCapacity; ++index) {
    ASSERT_TRUE(delivery->submit_result(sink, terminal));
  }
  EXPECT_FALSE(delivery->submit_result(sink, terminal));
  sink->release();
  ASSERT_TRUE(sink->wait_for_result());
  EXPECT_TRUE(delivery->close(std::chrono::steady_clock::now() + 1s));
}

TEST(RuntimeEngine, ThrowingDeliveryDoesNotFaultEngineDuringBoundedDestroy)
{
  EngineHarness harness(false);
  auto sink = std::make_shared<DeliveryTestSink>(DeliveryTestSink::Failure::Result);
  ASSERT_TRUE(harness.engine->authorize_admission("destroying", 1U));
  harness.engine->submit_admission(
    "destroying", make_goal(harness.config, "destroying-source", 1U), sink);
  ASSERT_TRUE(sink->wait_for_attempt());

  auto healthy = std::make_shared<RecordingSink>();
  ASSERT_TRUE(harness.engine->authorize_admission("healthy", 1U));
  harness.engine->submit_admission(
    "healthy", make_goal(harness.config, "healthy-source", 1U), healthy);
  ASSERT_TRUE(healthy->wait_for_count(1U));
  EXPECT_EQ(healthy->last().result.code, MissionResultCode::DependencyUnavailable);

  EXPECT_FALSE(harness.states.wait_for([](const RuntimeState & state) {
      return state.availability == RuntimeAvailability::Faulted;
    }, 100ms));

  const auto started = std::chrono::steady_clock::now();
  const auto shutdown_result = harness.engine->shutdown(started + 200ms);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_TRUE(shutdown_result);
  EXPECT_LT(elapsed, 500ms);
}

TEST(RuntimeEngine, BlockingSinkCannotExtendBoundedShutdown)
{
  EngineHarness harness(false);
  auto sink = std::make_shared<BlockingSink>();
  ASSERT_TRUE(harness.engine->authorize_admission("shutdown-blocked", 1U));
  harness.engine->submit_admission(
    "shutdown-blocked", make_goal(harness.config, "shutdown-source", 1U), sink);
  ASSERT_TRUE(sink->wait_for_entered());

  const auto started = std::chrono::steady_clock::now();
  const auto result = harness.engine->shutdown(started + 80ms);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_FALSE(result);
  EXPECT_LT(elapsed, 500ms);
  sink->release();
}

TEST(RuntimeEngine, BoundedShutdownClosesAdmission)
{
  EngineHarness harness;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  EXPECT_TRUE(harness.engine->shutdown(deadline));
  EXPECT_FALSE(harness.engine->authorize_admission("after-shutdown", 1U));
}

}  // namespace
}  // namespace voice_nav_mission
