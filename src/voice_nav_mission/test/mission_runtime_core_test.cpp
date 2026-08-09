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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/runtime_emergency_fence.hpp"

namespace voice_nav_mission
{
namespace
{

constexpr char kRuntimeId[] = "0123456789abcdef0123456789abcdef";
constexpr char kGateId[] = "fedcba9876543210fedcba9876543210";
constexpr char kOtherGateId[] = "00112233445566778899aabbccddeeff";

MissionGoal goal(
  std::uint64_t source_seq,
  std::vector<MissionStep> steps = {
      MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance),
        0.5F,
        0.0F,
        ""}
    })
{
  return MissionGoal{"source-a", source_seq, kRuntimeId, 1U, std::move(steps)};
}

RuntimeConfig config()
{
  RuntimeConfig value;
  value.runtime_instance_id = kRuntimeId;
  std::uint64_t next_id = 1U;
  value.identifier_generator = [next_id]() mutable {
      std::string id(32U, '0');
      const auto digit = static_cast<char>('0' + (next_id++ % 10U));
      id.back() = digit;
      return id;
    };
  return value;
}

struct Fixture
{
  std::shared_ptr<ScriptedSteadyClock> clock =
    std::make_shared<ScriptedSteadyClock>();
  std::shared_ptr<ScriptedMotionAuthorityPort> authority =
    std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  std::shared_ptr<ScriptedRelativeMotionPort> relative =
    std::make_shared<ScriptedRelativeMotionPort>();
  std::vector<MissionResult> results;
  std::vector<MissionFeedback> feedback;
  RuntimeCore core{
    config(),
    clock,
    authority,
    relative,
    {},
    [this](std::uint64_t, const MissionFeedback & item) {
      feedback.push_back(item);
    },
    [this](std::uint64_t, const MissionResult & item) {
      results.push_back(item);
    }};

  Fixture()
  {
    core.observe_gate(authority->snapshot());
  }
};

TEST(RuntimeCore, InvalidPlanIsRejectedBeforeDependenciesAndConsumesSequence)
{
  Fixture fixture;
  const auto invalid = fixture.core.admit(goal(
    1U,
        {MissionStep{
            static_cast<std::uint8_t>(MissionStepKind::MoveDistance),
            std::numeric_limits<float>::quiet_NaN(), 0.0F, ""}}));
  EXPECT_FALSE(invalid.accepted);
  EXPECT_EQ(invalid.result.code, MissionResultCode::InvalidPlan);
  EXPECT_EQ(fixture.authority->operations().size(), 0U);

  const auto reused_sequence = fixture.core.admit(goal(1U));
  EXPECT_FALSE(reused_sequence.accepted);
  EXPECT_EQ(reused_sequence.result.code, MissionResultCode::StaleRequest);
}

TEST(RuntimeCore, ProductionUnavailablePortDoesNotAcquireGate)
{
  Fixture fixture;
  fixture.relative->set_healthy(false);
  const auto admission = fixture.core.admit(goal(1U));
  EXPECT_FALSE(admission.accepted);
  EXPECT_EQ(admission.result.code, MissionResultCode::DependencyUnavailable);
  EXPECT_EQ(fixture.authority->operations().size(), 0U);
  EXPECT_FALSE(fixture.core.has_active_mission());
}

TEST(RuntimeCore, RevokedStartPermitPreventsRelativeMotionSideEffect)
{
  Fixture fixture;
  const auto admission = fixture.core.admit(
    goal(1U), []() {return false;});

  EXPECT_FALSE(admission.accepted);
  EXPECT_EQ(admission.result.code, MissionResultCode::SafetyFault);
  EXPECT_TRUE(fixture.relative->started_steps().empty());
  EXPECT_TRUE(fixture.authority->operations().empty());
  EXPECT_TRUE(fixture.results.empty());
  EXPECT_FALSE(fixture.core.has_active_mission());
}

TEST(RuntimeCore, FakeFSMIsOrderedBusyAndExactlyOnce)
{
  Fixture fixture;
  std::vector<MissionStep> steps{
    MissionStep{
      static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.5F, 0.0F, ""},
    MissionStep{
      static_cast<std::uint8_t>(MissionStepKind::RotateAngle), 0.0F, 1.0F, ""}};
  const auto admission = fixture.core.admit(goal(1U, steps));
  ASSERT_TRUE(admission.accepted);
  ASSERT_EQ(fixture.relative->started_steps().size(), 1U);

  const auto busy = fixture.core.admit(goal(2U));
  EXPECT_FALSE(busy.accepted);
  EXPECT_EQ(busy.result.code, MissionResultCode::Busy);

  fixture.relative->feedback(0.8);
  fixture.relative->feedback(0.4);
  ASSERT_FALSE(fixture.feedback.empty());
  EXPECT_GE(fixture.feedback.back().progress, 0.4F);
  fixture.relative->complete();
  ASSERT_EQ(fixture.relative->started_steps().size(), 2U);
  ASSERT_FALSE(fixture.feedback.empty());
  EXPECT_FLOAT_EQ(fixture.feedback.back().progress, 0.5F);
  fixture.relative->feedback(0.2);
  EXPECT_FLOAT_EQ(fixture.feedback.back().progress, 0.6F);
  fixture.relative->complete();
  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::Succeeded);
  EXPECT_EQ(fixture.results.front().failed_step, -1);
  EXPECT_FALSE(fixture.core.has_active_mission());

  // The scripted child still holds its old callback. A late duplicate cannot
  // produce a second result or reopen the Gate.
  fixture.relative->complete();
  EXPECT_EQ(fixture.results.size(), 1U);
}

TEST(RuntimeCore, ChildCallbacksCanBeQueuedAndAppliedByTheRuntimeWorker)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  std::vector<MissionFeedback> feedback;
  std::vector<MissionResult> results;
  std::vector<std::pair<MotionToken, double>> queued_feedback;
  std::vector<std::pair<MotionToken, ChildResult>> queued_results;
  RuntimeCore core(
    config(), clock, authority, relative,
    {},
    [&feedback](std::uint64_t, const MissionFeedback & item) {
      feedback.push_back(item);
    },
    [&results](std::uint64_t, const MissionResult & item) {
      results.push_back(item);
    },
    [&queued_feedback](const MotionToken & token, const double progress) {
      queued_feedback.emplace_back(token, progress);
      return true;
    },
    [&queued_results](const MotionToken & token, const ChildResult & result) {
      queued_results.emplace_back(token, result);
      return true;
    });
  core.observe_gate(authority->snapshot());

  ASSERT_TRUE(core.admit(goal(1U)).accepted);
  const auto token = relative->started_tokens().front();
  relative->feedback(0.5);
  ASSERT_EQ(queued_feedback.size(), 1U);
  EXPECT_TRUE(core.has_active_mission());
  core.on_child_feedback(queued_feedback.front().first, queued_feedback.front().second);
  EXPECT_FALSE(feedback.empty());

  relative->complete();
  ASSERT_EQ(queued_results.size(), 1U);
  EXPECT_TRUE(core.has_active_mission());
  core.on_child_result(queued_results.front().first, queued_results.front().second);

  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results.front().code, MissionResultCode::Succeeded);
  EXPECT_EQ(queued_results.front().first.mission_id, token.mission_id);
  EXPECT_FALSE(core.has_active_mission());
}

TEST(RuntimeCore, ThreeStepFeedbackUsesExactMissionBoundaries)
{
  Fixture fixture;
  const std::vector<MissionStep> steps{
    MissionStep{
      static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.5F, 0.0F, ""},
    MissionStep{
      static_cast<std::uint8_t>(MissionStepKind::RotateAngle), 0.0F, 1.0F, ""},
    MissionStep{
      static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.7F, 0.0F, ""}};

  ASSERT_TRUE(fixture.core.admit(goal(1U, steps)).accepted);
  fixture.relative->complete();
  ASSERT_FALSE(fixture.feedback.empty());
  EXPECT_FLOAT_EQ(fixture.feedback.back().progress, 1.0F / 3.0F);

  fixture.relative->feedback(0.8);
  EXPECT_FLOAT_EQ(fixture.feedback.back().progress, 0.6F);
  fixture.relative->complete();
  EXPECT_FLOAT_EQ(fixture.feedback.back().progress, 2.0F / 3.0F);

  fixture.relative->complete();
  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::Succeeded);
  EXPECT_FLOAT_EQ(fixture.feedback.back().progress, 1.0F);
}

TEST(RuntimeCore, StopRotatesEpochAndDuplicateDoesNotRotateAgain)
{
  Fixture fixture;
  const auto admission = fixture.core.admit(goal(1U));
  ASSERT_TRUE(admission.accepted);
  const auto first = fixture.core.stop(StopRequest{
        "stop-1", "stale-source-is-allowed", 99U, "operator stop"});
  EXPECT_EQ(first.code, 0U);
  EXPECT_EQ(first.admission_epoch, 2U);
  EXPECT_TRUE(first.motion_inhibited);
  EXPECT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::Stopped);

  const auto duplicate = fixture.core.stop(StopRequest{
        "stop-1", "stale-source-is-allowed", 99U, "operator stop"});
  EXPECT_EQ(duplicate.code, 1U);
  EXPECT_EQ(duplicate.admission_epoch, 2U);
  EXPECT_TRUE(duplicate.motion_inhibited);
  EXPECT_EQ(fixture.authority->inhibit_count(), 2U);
}

TEST(RuntimeCore, EmergencyFenceRotatesEpochAndRejectsOldGeneration)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);
  RuntimeEmergencyFence fence(1U);
  ASSERT_TRUE(fence.raise("control admission failed"));
  const auto snapshot = fence.take();
  ASSERT_TRUE(snapshot.has_value());

  fixture.core.fail_closed_at_epoch(snapshot->admission_epoch, snapshot->detail);

  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);
  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::SafetyFault);
  EXPECT_TRUE(fixture.core.state().availability == RuntimeAvailability::Faulted);
  const auto stale = fixture.core.admit(goal(2U));
  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.result.code, MissionResultCode::StaleRequest);
}

TEST(RuntimeCore, ActiveStopEpochExhaustionFailsTheWholeTransaction)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  auto max_epoch_config = config();
  max_epoch_config.initial_admission_epoch =
    std::numeric_limits<std::uint64_t>::max();
  std::vector<MissionResult> results;
  RuntimeCore core(
    max_epoch_config, clock, authority, relative, {}, {},
    [&results](std::uint64_t, const MissionResult & result) {
      results.push_back(result);
    });
  core.observe_gate(authority->snapshot());

  auto mission = goal(1U);
  mission.admission_epoch = std::numeric_limits<std::uint64_t>::max();
  ASSERT_TRUE(core.admit(mission).accepted);

  const auto response = core.stop(
    StopRequest{"stop-epoch-exhausted", "", 0U, "operator"});

  EXPECT_EQ(response.code, 2U);
  EXPECT_TRUE(response.motion_inhibited);
  EXPECT_EQ(response.admission_epoch,
    std::numeric_limits<std::uint64_t>::max());
  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results.front().code, MissionResultCode::SafetyFault);
  EXPECT_EQ(results.front().failed_step, 0);
  EXPECT_EQ(core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_FALSE(core.has_active_mission());
}

TEST(RuntimeCore, UnsupportedUnionIsDistinguishedFromInvalidUnion)
{
  Fixture fixture;
  const auto unsupported = fixture.core.admit(goal(
    1U,
        {MissionStep{
            static_cast<std::uint8_t>(MissionStepKind::NavigateTo), 0.0F, 0.0F,
            "place-a"}}));
  EXPECT_FALSE(unsupported.accepted);
  EXPECT_EQ(unsupported.result.code, MissionResultCode::UnsupportedStep);

  const auto invalid = fixture.core.admit(goal(
    2U,
        {MissionStep{
            static_cast<std::uint8_t>(MissionStepKind::NavigateTo), 0.1F, 0.0F,
            "place-a"}}));
  EXPECT_FALSE(invalid.accepted);
  EXPECT_EQ(invalid.result.code, MissionResultCode::InvalidPlan);
}

TEST(RuntimeCore, StartupConvergesLegacyPreparedGateToCurrentZeroProof)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  authority->set_snapshot(GateSnapshot{
        kGateId, 7U, std::string(32U, 'l'), GateState::Prepared,
        true, false, false, false});
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  RuntimeCore core(config(), clock, authority, relative);

  core.observe_gate(authority->snapshot());

  EXPECT_EQ(authority->snapshot().state, GateState::Inhibited);
  EXPECT_TRUE(authority->snapshot().zero_published);
  EXPECT_EQ(authority->inhibit_count(), 1U);
  EXPECT_EQ(core.state().availability, RuntimeAvailability::Available);
}

TEST(RuntimeCore, StartupLegacyLeaseFailureRemainsFaultedAndUnbound)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  authority->set_snapshot(GateSnapshot{
        kGateId, 7U, std::string(32U, 'l'), GateState::Armed,
        true, false, false, false});
  authority->set_inhibit_failure("startup zero convergence failed");
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  RuntimeCore core(config(), clock, authority, relative);

  core.observe_gate(authority->snapshot());

  EXPECT_EQ(authority->snapshot().state, GateState::Armed);
  EXPECT_EQ(core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_FALSE(core.usable());
  EXPECT_FALSE(core.admit(goal(1U)).accepted);
}

TEST(RuntimeCore, AdmissionBindsHealthyGateBeforeDelayedStateEvent)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  RuntimeCore core(config(), clock, authority, relative);

  const auto admission = core.admit(goal(1U));
  ASSERT_TRUE(admission.accepted);
  ASSERT_EQ(authority->snapshot().state, GateState::Armed);

  auto delayed_prepared = authority->snapshot();
  delayed_prepared.state = GateState::Prepared;
  delayed_prepared.motion_inhibited = true;
  delayed_prepared.zero_selected = true;
  delayed_prepared.zero_published = true;
  delayed_prepared.authority_live = false;
  delayed_prepared.writer_bound = false;
  core.observe_gate(delayed_prepared);

  EXPECT_EQ(authority->inhibit_count(), 0U);
  EXPECT_EQ(authority->snapshot().state, GateState::Armed);
  EXPECT_TRUE(core.has_active_mission());
}

TEST(RuntimeCore, HealthyAdmissionClearsPriorGateFaultLatchBeforeHealthEvent)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  authority->set_snapshot(GateSnapshot{
        kGateId, 0U, "", GateState::Faulted,
        false, true, true, true, "", false, false});
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  std::vector<MissionResult> results;
  RuntimeCore core(
    config(), clock, authority, relative, {}, {},
    [&results](std::uint64_t, const MissionResult & result) {
      results.push_back(result);
    });

  core.observe_gate(authority->snapshot());
  authority->set_snapshot(GateSnapshot{
        kGateId, 7U, std::string(32U, 'p'), GateState::Armed,
        true, false, false, false, "", true, true});

  const auto admission = core.admit(goal(1U));
  ASSERT_TRUE(admission.accepted);

  const auto faulted_unavailable = GateSnapshot{
    kOtherGateId, 8U, "", GateState::Faulted,
    false, true, true, true, "", false, false};
  core.observe_gate(faulted_unavailable);

  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results.front().code, MissionResultCode::SafetyFault);
  EXPECT_EQ(core.state().admission_epoch, 2U);
  EXPECT_FALSE(core.has_active_mission());
  ASSERT_EQ(authority->inhibit_count(), 1U);
  EXPECT_EQ(authority->snapshot().state, GateState::Inhibited);
  EXPECT_TRUE(authority->snapshot().motion_inhibited);
  EXPECT_TRUE(authority->snapshot().zero_selected);
  EXPECT_TRUE(authority->snapshot().zero_published);

  auto delayed_prepared = authority->snapshot();
  delayed_prepared.state = GateState::Prepared;
  delayed_prepared.lease_id = std::string(32U, 'p');
  delayed_prepared.motion_inhibited = true;
  delayed_prepared.zero_selected = true;
  delayed_prepared.zero_published = true;
  delayed_prepared.authority_live = false;
  delayed_prepared.writer_bound = false;
  core.observe_gate(delayed_prepared);

  EXPECT_EQ(results.size(), 1U);
  EXPECT_EQ(authority->inhibit_count(), 1U);
  EXPECT_EQ(core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_FALSE(core.has_active_mission());

  // The delayed Prepared sample belongs to the pre-fault Gate generation.
  // Replaying the same fault after it must remain idempotent.
  core.observe_gate(faulted_unavailable);

  EXPECT_EQ(results.size(), 1U);
  EXPECT_EQ(core.state().admission_epoch, 2U);
  EXPECT_EQ(authority->inhibit_count(), 1U);
  EXPECT_EQ(core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_FALSE(core.has_active_mission());
}

TEST(RuntimeCore, TrustedGateRecoveryRearmsFaultGeneration)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);

  const auto first_fault = GateSnapshot{
    kGateId, 3U, "", GateState::Faulted,
    false, true, true, false, "", false, false};
  fixture.core.observe_gate(first_fault);

  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::SafetyFault);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);
  EXPECT_EQ(fixture.authority->inhibit_count(), 1U);

  const auto delayed_prepared = GateSnapshot{
    kGateId, 3U, std::string(32U, 'p'), GateState::Prepared,
    true, true, true, true, "", false, false};
  fixture.core.observe_gate(delayed_prepared);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);

  const auto recovered = GateSnapshot{
    kGateId, 4U, "", GateState::Inhibited,
    true, true, true, true, "", false, false};
  fixture.core.observe_gate(recovered);

  // A delayed replay from before the trusted recovery must not roll the
  // current Gate generation back or consume the next fault latch.
  fixture.core.observe_gate(first_fault);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);
  EXPECT_EQ(fixture.authority->inhibit_count(), 1U);

  const auto second_fault = GateSnapshot{
    kGateId, 5U, "", GateState::Faulted,
    false, true, true, false, "", false, false};
  fixture.core.observe_gate(second_fault);

  EXPECT_EQ(fixture.core.state().admission_epoch, 3U);
  EXPECT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.authority->inhibit_count(), 1U);
  EXPECT_EQ(fixture.core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_FALSE(fixture.core.has_active_mission());
}

TEST(RuntimeCore, ValidatesEveryStepBeforeAcquiringTheGate)
{
  Fixture fixture;
  const auto invalid = fixture.core.admit(goal(
    1U,
      {
        MissionStep{
          static_cast<std::uint8_t>(MissionStepKind::MoveDistance),
          0.5F, 0.0F, ""},
        MissionStep{
          static_cast<std::uint8_t>(MissionStepKind::RotateAngle),
          0.5F, 0.0F, ""}}));
  EXPECT_FALSE(invalid.accepted);
  EXPECT_EQ(invalid.result.code, MissionResultCode::InvalidPlan);
  EXPECT_EQ(fixture.authority->operations().size(), 0U);
  EXPECT_EQ(fixture.relative->started_steps().size(), 0U);

  const auto accepted = fixture.core.admit(goal(2U));
  EXPECT_TRUE(accepted.accepted);
}

TEST(RuntimeCore, OpenFailureWithCleanupFailureIsSafetyFault)
{
  Fixture fixture;
  fixture.authority->set_open_failure("scripted OPEN failure");
  fixture.authority->set_inhibit_failure("scripted cleanup failure");

  const auto admission = fixture.core.admit(goal(1U));

  EXPECT_FALSE(admission.accepted);
  EXPECT_EQ(admission.result.code, MissionResultCode::SafetyFault);
  EXPECT_EQ(fixture.core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_FALSE(fixture.core.has_active_mission());
}

TEST(RuntimeCore, ChildFailureReportsTheStartedStepAndSkipsTheRest)
{
  Fixture fixture;
  const auto admission = fixture.core.admit(goal(
    1U,
      {
        MissionStep{
          static_cast<std::uint8_t>(MissionStepKind::MoveDistance),
          0.5F, 0.0F, ""},
        MissionStep{
          static_cast<std::uint8_t>(MissionStepKind::RotateAngle),
          0.0F, 1.0F, ""},
        MissionStep{
          static_cast<std::uint8_t>(MissionStepKind::MoveDistance),
          -0.5F, 0.0F, ""}}));
  ASSERT_TRUE(admission.accepted);

  fixture.relative->fail("step one failed");

  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::ExecutionFailed);
  EXPECT_EQ(fixture.results.front().failed_step, 0);
  EXPECT_EQ(fixture.relative->started_steps().size(), 1U);
}

TEST(RuntimeCore, ChildFailureCodesRemainTypedAtTheMissionBoundary)
{
  const auto mission_code_for = [](const ChildResultCode child_code) {
      Fixture fixture;
      const auto admission = fixture.core.admit(goal(1U));
      if (!admission.accepted) {
        return MissionResultCode::InternalError;
      }
      const auto token = fixture.relative->started_tokens().front();
      fixture.core.on_child_result(token, ChildResult{child_code, "typed"});
      if (fixture.results.empty()) {
        return MissionResultCode::InternalError;
      }
      return fixture.results.front().code;
    };

  EXPECT_EQ(
    mission_code_for(ChildResultCode::DependencyUnavailable),
    MissionResultCode::DependencyUnavailable);
  EXPECT_EQ(
    mission_code_for(ChildResultCode::Timeout),
    MissionResultCode::Timeout);
  EXPECT_EQ(
    mission_code_for(ChildResultCode::Failed),
    MissionResultCode::ExecutionFailed);
  EXPECT_EQ(
    mission_code_for(ChildResultCode::SafetyFault),
    MissionResultCode::SafetyFault);
}

TEST(RuntimeCore, TerminalCancelUsesTheManualSteadyClockGraceDeadline)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);
  fixture.clock->advance(std::chrono::milliseconds(30000));

  fixture.core.on_tick();

  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::Timeout);
  EXPECT_EQ(fixture.results.front().failed_step, 0);
  EXPECT_EQ(
    fixture.relative->cancel_deadline(),
    fixture.clock->now() + config().cancel_grace);
}

TEST(RuntimeCore, DependencyLossRotatesEpochOnlyOnceAndCancelsActiveMission)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);
  fixture.relative->set_healthy(false);

  fixture.core.observe_dependencies();
  fixture.core.observe_dependencies();

  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::SafetyFault);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);
  EXPECT_EQ(fixture.relative->cancel_count(), 1U);
}

TEST(RuntimeCore, GateIdentityLossRotatesEpochOnlyOnceAndFailsClosed)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);
  fixture.authority->set_snapshot(GateSnapshot{
        kOtherGateId, 0U, "", GateState::Faulted, false,
        true, true, false});
  const auto lost_gate = fixture.authority->snapshot();

  fixture.core.observe_gate(lost_gate);
  fixture.core.observe_gate(lost_gate);

  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::SafetyFault);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);
  EXPECT_EQ(fixture.core.state().availability, RuntimeAvailability::Faulted);
}

TEST(RuntimeCore, ZeroProofFailureStillCancelsAndReturnsSafetyFault)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);
  fixture.authority->set_next_failure("zero proof failed");

  const auto response = fixture.core.stop(
    StopRequest{"stop-zero-failure", "", 0U, "operator"});

  EXPECT_EQ(response.code, 2U);
  EXPECT_FALSE(response.motion_inhibited);
  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::SafetyFault);
  EXPECT_EQ(fixture.results.front().failed_step, 0);
  EXPECT_FALSE(fixture.core.has_active_mission());
  EXPECT_EQ(fixture.relative->cancel_count(), 1U);
}

TEST(RuntimeCore, StopBarrierRunsBeforeChildCancel)
{
  Fixture fixture;
  std::vector<std::string> trace;
  fixture.authority->set_inhibit_observer([&trace]() {trace.push_back("inhibit");});
  fixture.relative->set_cancel_observer([&trace]() {trace.push_back("cancel");});
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);
  trace.clear();

  ASSERT_EQ(
    fixture.core.stop(StopRequest{"stop-order", "", 0U, "operator"}).code,
    0U);

  ASSERT_FALSE(trace.empty());
  const auto cancel = std::find(trace.begin(), trace.end(), "cancel");
  ASSERT_NE(cancel, trace.end());
  EXPECT_TRUE(std::all_of(trace.begin(), cancel, [](const std::string & event) {
      return event == "inhibit";
  }));
}

TEST(RuntimeCore, ChildStartExceptionIsPreExecutionFailure)
{
  Fixture fixture;
  fixture.relative->set_start_failure("scripted start exception");

  const auto admission = fixture.core.admit(goal(1U));

  EXPECT_TRUE(admission.accepted);
  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::InternalError);
  EXPECT_EQ(fixture.results.front().failed_step, -1);
}

TEST(RuntimeCore, StopSelectsTerminalBeforeBarrierReentrantCompletion)
{
  Fixture fixture;
  bool first_barrier = true;
  fixture.authority->set_inhibit_observer([&]() {
      if (first_barrier) {
        first_barrier = false;
        fixture.relative->complete();
      }
    });
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);

  const auto response = fixture.core.stop(
    StopRequest{"stop-reentrant", "", 0U, "operator"});

  EXPECT_EQ(response.code, 0U);
  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::Stopped);
  EXPECT_EQ(fixture.relative->cancel_count(), 1U);
  EXPECT_EQ(fixture.authority->inhibit_count(), 1U);
  ASSERT_FALSE(fixture.relative->started_tokens().empty());
  const auto original = fixture.relative->started_tokens().front();
  EXPECT_EQ(fixture.relative->cancel_token().mission_id, original.mission_id);
  EXPECT_EQ(
    fixture.relative->cancel_token().mission_generation,
    original.mission_generation);
  EXPECT_EQ(
    fixture.relative->cancel_token().step_generation,
    original.step_generation);
}

TEST(RuntimeCore, CancelAcknowledgementFailureIsSafetyFault)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);
  fixture.relative->set_cancel_acknowledged(false);

  fixture.core.cancel(1U);

  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::SafetyFault);
  EXPECT_EQ(fixture.results.front().failed_step, 0);
  EXPECT_FALSE(fixture.core.has_active_mission());
}

TEST(RuntimeCore, SynchronousChildCompletionStillProducesExactlyOneResult)
{
  Fixture fixture;
  fixture.relative->set_start_completion(true);

  const auto admission = fixture.core.admit(goal(1U));

  EXPECT_TRUE(admission.accepted);
  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::Succeeded);
  EXPECT_FALSE(fixture.core.has_active_mission());
}

TEST(RuntimeCore, StartedCancelReportsTheActiveStep)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);

  fixture.core.cancel(1U);

  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::Canceled);
  EXPECT_EQ(fixture.results.front().failed_step, 0);
}

TEST(RuntimeCore, RestartCreatesNewRuntimeIdentityAtEpochOne)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  auto restart_config = config();
  restart_config.runtime_instance_id.clear();
  restart_config.identifier_generator = {};
  RuntimeCore first(restart_config, clock, authority, relative);
  RuntimeCore second(restart_config, clock, authority, relative);

  EXPECT_NE(first.state().runtime_instance_id, second.state().runtime_instance_id);
  EXPECT_EQ(first.state().admission_epoch, 1U);
  EXPECT_EQ(second.state().admission_epoch, 1U);
}

TEST(RuntimeCore, LateCallbackFromAnOldMissionCannotCompleteTheNewMission)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);
  const auto old_token = fixture.relative->started_tokens().front();
  fixture.relative->complete();
  ASSERT_EQ(fixture.results.size(), 1U);
  ASSERT_TRUE(fixture.core.admit(goal(2U)).accepted);
  const auto new_token = fixture.relative->started_tokens().back();

  fixture.relative->complete_token(old_token);
  EXPECT_EQ(fixture.results.size(), 1U);
  EXPECT_TRUE(fixture.core.has_active_mission());
  fixture.relative->complete_token(new_token);
  EXPECT_EQ(fixture.results.size(), 2U);
  EXPECT_EQ(fixture.results.back().code, MissionResultCode::Succeeded);
}

TEST(RuntimeCore, StopFingerprintSeparatesDelimiterLikeFields)
{
  Fixture fixture;
  const auto first = fixture.core.stop(
    StopRequest{"stop-1", "a", 1U, "2|r"});
  ASSERT_EQ(first.code, 0U);
  const auto collision = fixture.core.stop(
    StopRequest{"stop-1", "a|1", 2U, "r"});

  EXPECT_EQ(collision.code, 2U);
  EXPECT_TRUE(collision.motion_inhibited);
  EXPECT_EQ(fixture.core.state().availability, RuntimeAvailability::Faulted);
}

TEST(RuntimeCore, FullStopCacheFailsClosedAndCleansTheActiveMission)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  auto small_config = config();
  small_config.stop_cache_size = 1U;
  std::vector<MissionResult> results;
  RuntimeCore core(
    small_config, clock, authority, relative, {}, {},
    [&results](std::uint64_t, const MissionResult & result) {
      results.push_back(result);
    });
  core.observe_gate(authority->snapshot());
  ASSERT_EQ(core.stop(StopRequest{"stop-1", "", 0U, "first"}).code, 0U);
  ASSERT_TRUE(core.admit(MissionGoal{
        kRuntimeId, 1U, kRuntimeId, 2U,
        {MissionStep{
            static_cast<std::uint8_t>(MissionStepKind::MoveDistance),
            0.5F, 0.0F, ""}}}).accepted);

  const auto response = core.stop(StopRequest{"stop-2", "", 0U, "full"});

  EXPECT_EQ(response.code, 2U);
  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results.front().code, MissionResultCode::SafetyFault);
  EXPECT_FALSE(core.has_active_mission());
}

}  // namespace
}  // namespace voice_nav_mission
