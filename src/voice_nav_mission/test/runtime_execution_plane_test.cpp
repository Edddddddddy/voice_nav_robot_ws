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
            plane_->core()->fail_closed_at_epoch(
            snapshot.admission_epoch, snapshot.detail);
          }
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
    queue_.close();
    if (worker_.joinable()) {
      worker_.join();
    }
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

private:
  std::shared_ptr<ScriptedSteadyClock> clock_;
  std::shared_ptr<ScriptedMotionAuthorityPort> authority_;
  std::shared_ptr<ExternalRelativeMotionPort> relative_;
  Queue queue_;
  RuntimeEmergencyFence fence_;
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
  gate.begin_quiesce(tracker);

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
    fixture.queue().close();
  } else if (GetParam() == RejectionMode::FenceBlocked) {
    ASSERT_TRUE(fixture.fence().raise("pre-existing emergency"));
  } else {
    for (std::size_t index = 0U;
      index < RuntimeExecutionPlaneFixture::Queue::kControlReserve; ++index)
    {
      ASSERT_EQ(
        fixture.queue().push(
          CompletionEvent{MotionToken{index + 100U, 1U, 1U, 1U}},
          RuntimeExecutionPlaneFixture::Queue::Lane::Control),
        RuntimeExecutionPlaneFixture::Queue::PushResult::Accepted);
    }
  }

  const auto record = std::make_shared<const RelativeMotionCompletionRecord>(
    RelativeMotionCompletionRecord{
        token, ChildResult{ChildResultCode::SafetyFault, "relay rejection"}});
  EXPECT_FALSE(fixture.mailbox().relay(record));
  ASSERT_TRUE(fixture.terminal().wait_for_count(1U));
  const auto results = fixture.terminal().results();
  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results.front().code, MissionResultCode::SafetyFault);
  EXPECT_FALSE(fixture.plane().core()->has_active_mission());
  EXPECT_GE(fixture.relative().cancel_count(), 1U);
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
