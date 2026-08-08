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

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/mission_action_result_router.hpp"
#include "voice_nav_mission/runtime_admission_gate.hpp"
#include "voice_nav_mission/runtime_emergency_fence.hpp"
#include "voice_nav_mission/runtime_event_ingress.hpp"
#include "voice_nav_mission/runtime_event_queue.hpp"
#include "voice_nav_mission/runtime_execution_plane.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;

constexpr char kRuntimeId[] = "0123456789abcdef0123456789abcdef";
constexpr char kGateId[] = "fedcba9876543210fedcba9876543210";

MissionGoal make_goal()
{
  return MissionGoal{
    "plane-source", 1U, kRuntimeId, 1U,
    {MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance),
        0.5F, 0.0F, ""}}};
}

RuntimeConfig make_config()
{
  RuntimeConfig config;
  config.runtime_instance_id = kRuntimeId;
  return config;
}

class ExternalRelativeMotionPort final : public RelativeMotionPort
{
public:
  [[nodiscard]] bool healthy() const override {return true;}

  void start(
    const MotionToken & token,
    const MissionStep &,
    FeedbackCallback,
    ResultCallback) override
  {
    started_tokens_.push_back(token);
  }

  [[nodiscard]] bool cancel(
    const MotionToken & token,
    SteadyClockPort::TimePoint deadline) override
  {
    cancel_token_ = token;
    cancel_deadline_ = deadline;
    ++cancel_count_;
    return true;
  }

  void tick(SteadyClockPort::TimePoint) override {}

  [[nodiscard]] bool uses_external_completion_registry() const noexcept override
  {
    return true;
  }

  [[nodiscard]] const std::vector<MotionToken> & started_tokens() const noexcept
  {
    return started_tokens_;
  }

  [[nodiscard]] std::size_t cancel_count() const noexcept
  {
    return cancel_count_;
  }

private:
  std::vector<MotionToken> started_tokens_;
  MotionToken cancel_token_{};
  SteadyClockPort::TimePoint cancel_deadline_{};
  std::size_t cancel_count_{0U};
};

class TerminalProbe final
{
public:
  void record(const MissionResult & result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    results_.push_back(result);
    condition_.notify_all();
  }

  [[nodiscard]] bool wait_for_count(const std::size_t count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this, count]() {
               return results_.size() >= count;
           });
  }

  [[nodiscard]] std::vector<MissionResult> results() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return results_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<MissionResult> results_;
};

struct CompletionEvent
{
  MotionToken token;
};

class RuntimeExecutionPlaneFixture final
{
public:
  RuntimeExecutionPlaneFixture()
  : clock_(std::make_shared<ScriptedSteadyClock>()),
    authority_(std::make_shared<ScriptedMotionAuthorityPort>(kGateId)),
    relative_(std::make_shared<ExternalRelativeMotionPort>()),
    queue_([]() {return CompletionEvent{MotionToken{}};}),
    fence_(1U),
    ingress_(std::make_unique<RuntimeEventIngress<CompletionEvent>>(
      queue_, fence_,
        [](const CompletionEvent &) {return Queue::Lane::Control;},
        []() {},
        [this](const RuntimeEmergencyFenceSnapshot & snapshot) {
          if (plane_ && plane_->core()) {
            std::lock_guard<std::recursive_mutex> lock(
              plane_->core_serial_mutex());
            plane_->core()->fail_closed_at_epoch(
            snapshot.admission_epoch, snapshot.detail);
          }
      },
        RuntimeEventIngress<CompletionEvent>::EmergencyControlSelector{},
        [this](CompletionEvent & event) {
          before_dispatch(event);
        })),
    plane_(std::make_unique<RuntimeExecutionPlane>(
      make_config(), clock_, authority_, relative_,
        [](const RuntimeState &) {},
        [](std::uint64_t, const MissionFeedback &) {},
        [this](std::uint64_t, const MissionResult & result) {
          terminal_.record(result);
      },
      RuntimeCore::ChildFeedbackDispatcher{},
        [this](const std::uint64_t epoch) {
          return fence_.admission_allowed(epoch);
      },
        [this](const MotionToken & token) {
          return ingress_->enqueue(CompletionEvent{token});
      },
        [this](std::string detail) {
          ingress_->request_emergency(std::move(detail));
      }))
  {
    plane_->core()->observe_gate(authority_->snapshot());
    worker_ = std::thread([this]() {
          ingress_->run(
            [this](CompletionEvent & event) {
              std::lock_guard<std::recursive_mutex> lock(
                plane_->core_serial_mutex());
              plane_->core()->on_child_result(
            event.token, ChildResult{ChildResultCode::SafetyFault, "relay"});
        },
            [this](std::string detail) {
              ingress_->request_emergency(std::move(detail));
        });
    });
  }

  ~RuntimeExecutionPlaneFixture()
  {
    stop_worker();
    plane_->shutdown();
  }

  using Queue = RuntimeEventQueue<CompletionEvent>;

  [[nodiscard]] RuntimeExecutionPlane & plane() noexcept {return *plane_;}
  [[nodiscard]] ExternalRelativeMotionPort & relative() noexcept
  {
    return *relative_;
  }
  [[nodiscard]] TerminalProbe & terminal() noexcept {return terminal_;}
  [[nodiscard]] RuntimeEmergencyFence & fence() noexcept {return fence_;}
  [[nodiscard]] Queue & queue() noexcept {return queue_;}
  [[nodiscard]] NodeCompletionMailbox & mailbox() noexcept
  {
    return plane_->completion_mailbox();
  }

  [[nodiscard]] bool pause_consumer()
  {
    {
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      consumer_paused_ = true;
      dispatch_entered_ = false;
      release_dispatch_ = false;
    }
    const auto pushed = queue_.push(
      CompletionEvent{MotionToken{9000U, 1U, 1U, 1U}}, Queue::Lane::Control);
    if (pushed != Queue::PushResult::Accepted) {
      return false;
    }
    std::unique_lock<std::mutex> lock(dispatch_mutex_);
    return dispatch_condition_.wait_for(lock, 1s, [this]() {
               return dispatch_entered_;
           });
  }

  void release_consumer() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      release_dispatch_ = true;
      consumer_paused_ = false;
    }
    dispatch_condition_.notify_all();
  }

  void stop_worker() noexcept
  {
    release_consumer();
    queue_.close();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  [[nodiscard]] bool worker_joined() const noexcept
  {
    return !worker_.joinable();
  }

private:
  void before_dispatch(CompletionEvent &)
  {
    std::unique_lock<std::mutex> lock(dispatch_mutex_);
    if (!consumer_paused_) {
      return;
    }
    dispatch_entered_ = true;
    dispatch_condition_.notify_all();
    dispatch_condition_.wait(lock, [this]() {return release_dispatch_;});
  }

  std::shared_ptr<ScriptedSteadyClock> clock_;
  std::shared_ptr<ScriptedMotionAuthorityPort> authority_;
  std::shared_ptr<ExternalRelativeMotionPort> relative_;
  Queue queue_;
  RuntimeEmergencyFence fence_;
  mutable std::mutex dispatch_mutex_;
  std::condition_variable dispatch_condition_;
  bool consumer_paused_{false};
  bool dispatch_entered_{false};
  bool release_dispatch_{false};
  std::unique_ptr<RuntimeEventIngress<CompletionEvent>> ingress_;
  std::unique_ptr<RuntimeExecutionPlane> plane_;
  TerminalProbe terminal_;
  std::thread worker_;
};

enum class RejectionMode
{
  QueueClosed,
  FenceBlocked,
  ControlFull
};

class RelayRejectionTest
  : public ::testing::TestWithParam<RejectionMode>
{
};

void start_active_mission(RuntimeExecutionPlaneFixture & fixture)
{
  const auto admission = fixture.plane().core()->admit(make_goal());
  ASSERT_TRUE(admission.accepted);
  ASSERT_EQ(fixture.relative().started_tokens().size(), 1U);
}

TEST(RuntimeExecutionPlaneTest, StartFailureUsesCoreGoalTerminalExactlyOnce)
{
  RuntimeExecutionPlaneFixture fixture;
  fixture.mailbox().close();

  const auto admission = fixture.plane().core()->admit(make_goal());
  ASSERT_TRUE(admission.accepted);
  ASSERT_TRUE(fixture.terminal().wait_for_count(1U));
  const auto results = fixture.terminal().results();
  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results.front().code, MissionResultCode::SafetyFault);
  EXPECT_FALSE(fixture.plane().core()->has_active_mission());
  EXPECT_TRUE(fixture.relative().started_tokens().empty());
}

TEST(RuntimeExecutionPlaneTest, QuiescedQueuedPermitCannotStartOrDuplicateGoal)
{
  RuntimeExecutionPlaneFixture fixture;
  auto clock = std::make_shared<ScriptedSteadyClock>();
  ActionAdmissionTracker tracker([clock]() {return clock->now();});
  RuntimeAdmissionGate gate;
  const auto permit = gate.claim_start(1U);
  ASSERT_TRUE(permit.issued);
  ASSERT_TRUE(gate.begin_quiesce(
    tracker, std::chrono::steady_clock::now() + 1s));

  MissionActionAdapterBoundary adapter;
  adapter.on_accepted(
    make_goal(),
    [&fixture, &gate, permit](const MissionGoal & goal) {
      return fixture.plane().core()->admit(
        goal, [&gate, permit]() {return gate.start_allowed(permit);});
    },
    [](const std::uint64_t) {},
    [&fixture](const std::uint64_t, const ActionResultDelivery & delivery) {
      fixture.terminal().record(delivery.result);
    },
    [&fixture](const MissionResult & result) {
      fixture.terminal().record(result);
    });
  ASSERT_TRUE(fixture.terminal().wait_for_count(1U));
  const auto results = fixture.terminal().results();
  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results.front().code, MissionResultCode::SafetyFault);
  EXPECT_TRUE(fixture.relative().started_tokens().empty());
  EXPECT_FALSE(fixture.plane().core()->has_active_mission());
}

TEST_P(
  RelayRejectionTest,
  RelayRejectionFailsClosedThroughCoreGoal)
{
  RuntimeExecutionPlaneFixture fixture;
  start_active_mission(fixture);
  const auto token = fixture.relative().started_tokens().front();

  if (GetParam() == RejectionMode::QueueClosed) {
    fixture.stop_worker();
    ASSERT_TRUE(fixture.worker_joined());
  } else if (GetParam() == RejectionMode::FenceBlocked) {
    ASSERT_TRUE(fixture.fence().raise("pre-existing emergency"));
    const auto pending_fence = fixture.fence().take();
    ASSERT_TRUE(pending_fence.has_value());
    EXPECT_FALSE(fixture.fence().pending());
  } else {
    ASSERT_TRUE(fixture.pause_consumer());
    for (std::size_t index = 0U;
      index < RuntimeExecutionPlaneFixture::Queue::kControlReserve; ++index)
    {
      ASSERT_EQ(
        fixture.queue().push(
          CompletionEvent{MotionToken{index + 100U, 1U, 1U, 1U}},
          RuntimeExecutionPlaneFixture::Queue::Lane::Control),
        RuntimeExecutionPlaneFixture::Queue::PushResult::Accepted);
    }
    EXPECT_EQ(
      fixture.queue().push(
        CompletionEvent{MotionToken{999U, 1U, 1U, 1U}},
        RuntimeExecutionPlaneFixture::Queue::Lane::Control),
      RuntimeExecutionPlaneFixture::Queue::PushResult::ControlFull);
  }

  const auto record = std::make_shared<const RelativeMotionCompletionRecord>(
    RelativeMotionCompletionRecord{
        token, ChildResult{ChildResultCode::SafetyFault, "relay rejection"}});
  EXPECT_FALSE(fixture.mailbox().relay(record));
  ASSERT_TRUE(fixture.terminal().wait_for_count(1U));
  const auto results = fixture.terminal().results();
  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results.front().code, MissionResultCode::SafetyFault);
  EXPECT_EQ(results.size(), 1U);
  EXPECT_FALSE(fixture.plane().core()->has_active_mission());
  EXPECT_GE(fixture.relative().cancel_count(), 1U);
  if (GetParam() == RejectionMode::ControlFull) {
    fixture.release_consumer();
  }
  fixture.stop_worker();
  EXPECT_TRUE(fixture.worker_joined());
}

INSTANTIATE_TEST_SUITE_P(
  RelayRejection,
  RelayRejectionTest,
  ::testing::Values(
    RejectionMode::QueueClosed,
    RejectionMode::FenceBlocked,
    RejectionMode::ControlFull));

}  // namespace
}  // namespace voice_nav_mission
