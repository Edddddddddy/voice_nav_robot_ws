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

#include "voice_nav_mission/mission_authority_convergence.hpp"
#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/runtime_emergency_fence.hpp"

namespace voice_nav_mission
{
namespace
{

constexpr char kRuntimeId[] = "0123456789abcdef0123456789abcdef";
constexpr char kGateId[] = "fedcba9876543210fedcba9876543210";
constexpr char kOtherGateId[] = "00112233445566778899aabbccddeeff";
constexpr char kReplacementGateId[] = "ffeeddccbbaa99887766554433221100";

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

class ScriptedNavigationPort final : public NavigationPort
{
public:
  [[nodiscard]] bool healthy() const override {return healthy_;}

  void start(
    const MotionToken & token,
    const MissionStep & step,
    FeedbackCallback feedback,
    ResultCallback result) override
  {
    started_tokens.push_back(token);
    started_steps.push_back(step);
    feedback_callback = std::move(feedback);
    result_callback = std::move(result);
  }

  [[nodiscard]] bool cancel(
    const MotionToken & token,
    SteadyClockPort::TimePoint deadline) override
  {
    (void)deadline;
    ++cancel_count;
    cancel_token = token;
    return cancel_acknowledged;
  }

  void tick(SteadyClockPort::TimePoint) override {}

  void complete()
  {
    ASSERT_TRUE(static_cast<bool>(result_callback));
    result_callback(started_tokens.back(), ChildResult{
      ChildResultCode::Succeeded, "navigation complete"});
  }

  bool healthy_{true};
  bool cancel_acknowledged{true};
  std::size_t cancel_count{0U};
  MotionToken cancel_token{};
  std::vector<MotionToken> started_tokens;
  std::vector<MissionStep> started_steps;
  FeedbackCallback feedback_callback;
  ResultCallback result_callback;
};

class RecordingMapStorePort final : public MapStorePort
{
public:
  explicit RecordingMapStorePort(
    std::shared_ptr<MotionAuthorityPort> authority)
  : authority_(std::move(authority))
  {
  }

  ChildResult save(const std::string & map_id) override
  {
    saved_map_ids.push_back(map_id);
    const auto snapshot = authority_->snapshot();
    saw_inhibited_zero =
      snapshot.state == GateState::Inhibited && snapshot.motion_inhibited &&
      snapshot.zero_selected && snapshot.zero_published;
    return ChildResult{ChildResultCode::Succeeded, "map package saved"};
  }

  std::shared_ptr<MotionAuthorityPort> authority_;
  std::vector<std::string> saved_map_ids;
  bool saw_inhibited_zero{false};
};

AuthorityResult converged_stale_inhibit(
  std::chrono::milliseconds elapsed,
  bool transport_unavailable = false)
{
  using TimePoint = MissionAuthorityAdapter::TimePoint;
  TimePoint now{};
  const auto snapshot = GateSnapshot{
    kGateId, 16U, "", GateState::Inhibited,
    true, true, true, true};
  MissionAuthorityAdapter adapter(
    std::chrono::milliseconds(250),
    std::chrono::milliseconds(250),
    [&now]() {return now;},
    [&now, elapsed, snapshot, transport_unavailable](
      const AuthorityOperation &,
      AuthorityOperationKind,
      TimePoint,
      TimePoint)
    {
      now += elapsed;
      auto result = AuthorityResult{
        false,
        true,
        true,
        snapshot,
        "",
        "expected_control_seq is stale",
        true};
      result.transport_unavailable = transport_unavailable;
      return result;
    });
  return adapter.inhibit(AuthorityOperation{
        "startup-inhibit", kGateId, 15U, ""});
}

struct Fixture
{
  std::shared_ptr<ScriptedSteadyClock> clock =
    std::make_shared<ScriptedSteadyClock>();
  std::shared_ptr<ScriptedMotionAuthorityPort> authority =
    std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  std::shared_ptr<ScriptedRelativeMotionPort> relative =
    std::make_shared<ScriptedRelativeMotionPort>();
  std::vector<RuntimeState> states;
  std::vector<MissionResult> results;
  std::vector<MissionFeedback> feedback;
  RuntimeCore core{
    config(),
    clock,
    authority,
    relative,
    [this](const RuntimeState & item) {
      states.push_back(item);
    },
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

TEST(RuntimeCore, SafetyFaultedRelativePortLatchesRuntimeFault)
{
  Fixture fixture;
  fixture.relative->set_safety_faulted(true);

  const auto admission = fixture.core.admit(goal(1U));

  EXPECT_FALSE(admission.accepted);
  EXPECT_EQ(admission.result.code, MissionResultCode::SafetyFault);
  EXPECT_EQ(
    fixture.core.state().availability,
    RuntimeAvailability::Faulted);
  EXPECT_TRUE(fixture.authority->operations().empty());
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

TEST(RuntimeCore, NavigationStudyUsesNavigationPort)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  auto navigation = std::make_shared<ScriptedNavigationPort>();
  auto runtime_config = config();
  runtime_config.operating_mode = OperatingMode::Navigation;
  runtime_config.named_place_ids = {"study"};
  RuntimeCore core(
    runtime_config,
    clock,
    authority,
    relative,
    {}, {}, {}, {}, {}, {}, {}, {}, navigation);
  core.observe_gate(authority->snapshot());

  const auto admission = core.admit(goal(
    1U,
    {MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::NavigateTo),
        0.0F,
        0.0F,
        "study"}}));

  ASSERT_TRUE(admission.accepted);
  ASSERT_EQ(navigation->started_steps.size(), 1U);
  EXPECT_EQ(navigation->started_steps.front().target_id, "study");
  navigation->complete();
}

TEST(RuntimeCore, MappingSaveMapUsesMapStoreAndSucceeds)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  auto map_store = std::make_shared<RecordingMapStorePort>(authority);
  std::vector<MissionResult> results;
  auto runtime_config = config();
  runtime_config.operating_mode = OperatingMode::Mapping;
  RuntimeCore core(
    runtime_config, clock, authority, relative,
    {}, {}, [&results](std::uint64_t, const MissionResult & result) {
      results.push_back(result);
    }, {}, {}, {}, {}, {}, {}, map_store);
  core.observe_gate(authority->snapshot());

  const auto admission = core.admit(goal(
    1U,
    {MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::SaveMap),
        0.0F, 0.0F, "voice_mvp"}}));

  ASSERT_TRUE(admission.accepted);
  ASSERT_EQ(map_store->saved_map_ids, std::vector<std::string>{"voice_mvp"});
  EXPECT_TRUE(map_store->saw_inhibited_zero);
  EXPECT_EQ(authority->operations().size(), 1U);
  EXPECT_EQ(authority->inhibit_count(), 1U);
  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results.front().code, MissionResultCode::Succeeded);
  EXPECT_FALSE(results.front().detail.find("not implemented") != std::string::npos);
  EXPECT_FALSE(core.has_active_mission());
}

TEST(RuntimeCore, MappingMixedSaveMapPlanIsRejectedBeforeGateOpen)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  auto map_store = std::make_shared<RecordingMapStorePort>(authority);
  auto runtime_config = config();
  runtime_config.operating_mode = OperatingMode::Mapping;
  RuntimeCore core(
    runtime_config, clock, authority, relative,
    {}, {}, {}, {}, {}, {}, {}, {}, {}, map_store);
  core.observe_gate(authority->snapshot());

  const auto admission = core.admit(goal(
    1U,
    {
      MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance),
        0.5F, 0.0F, ""},
      MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::SaveMap),
        0.0F, 0.0F, "voice_mvp"},
    }));

  EXPECT_FALSE(admission.accepted);
  EXPECT_EQ(admission.result.code, MissionResultCode::InvalidPlan);
  EXPECT_TRUE(map_store->saved_map_ids.empty());
  EXPECT_TRUE(authority->operations().empty());
}

TEST(RuntimeCore, NavigationRejectsUnknownPlaceBeforeNavigationStart)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  auto navigation = std::make_shared<ScriptedNavigationPort>();
  auto runtime_config = config();
  runtime_config.operating_mode = OperatingMode::Navigation;
  runtime_config.named_place_ids = {"study"};
  RuntimeCore core(runtime_config, clock, authority, relative, {}, {}, {}, {},
    {}, {}, {}, {}, navigation);
  core.observe_gate(authority->snapshot());

  const auto admission = core.admit(goal(
    1U,
    {MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::NavigateTo),
        0.0F, 0.0F, "unknown"}}));

  EXPECT_FALSE(admission.accepted);
  EXPECT_EQ(admission.result.code, MissionResultCode::UnknownTarget);
  EXPECT_TRUE(navigation->started_tokens.empty());
}

TEST(RuntimeCore, NavigationCancelIgnoresStaleResult)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  auto navigation = std::make_shared<ScriptedNavigationPort>();
  auto runtime_config = config();
  runtime_config.operating_mode = OperatingMode::Navigation;
  runtime_config.named_place_ids = {"study"};
  std::vector<MissionResult> results;
  RuntimeCore core(
    runtime_config, clock, authority, relative,
    {}, {}, [&results](std::uint64_t, const MissionResult & result) {
      results.push_back(result);
    }, {}, {}, {}, {}, {}, navigation);
  core.observe_gate(authority->snapshot());

  const auto admission = core.admit(goal(
    1U,
    {MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::NavigateTo),
        0.0F, 0.0F, "study"}}));
  ASSERT_TRUE(admission.accepted);

  core.cancel(admission.mission_id);
  ASSERT_EQ(navigation->cancel_count, 1U);
  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results.front().code, MissionResultCode::Canceled);

  navigation->complete();
  EXPECT_EQ(results.size(), 1U);
  EXPECT_FALSE(core.has_active_mission());
}

TEST(RuntimeCore, NavigationStopUsesNavigationCancel)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  auto navigation = std::make_shared<ScriptedNavigationPort>();
  auto runtime_config = config();
  runtime_config.operating_mode = OperatingMode::Navigation;
  runtime_config.named_place_ids = {"study"};
  RuntimeCore core(runtime_config, clock, authority, relative, {}, {}, {}, {},
    {}, {}, {}, {}, navigation);
  core.observe_gate(authority->snapshot());

  const auto admission = core.admit(goal(
    1U,
    {MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::NavigateTo),
        0.0F, 0.0F, "study"}}));
  ASSERT_TRUE(admission.accepted);

  const auto response = core.stop(
    StopRequest{"stop-navigation", "operator", 1U, "operator stop"});
  EXPECT_EQ(response.code, 0U);
  EXPECT_TRUE(response.motion_inhibited);
  EXPECT_EQ(navigation->cancel_count, 1U);
  EXPECT_FALSE(core.has_active_mission());
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

TEST(RuntimeCore, StartupAcceptsSameGateStaleInhibitWithCurrentZeroProof)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  authority->set_snapshot(GateSnapshot{
        kGateId, 15U, "", GateState::Inhibited,
        true, true, true, false});
  authority->set_inhibit_result(converged_stale_inhibit(
      std::chrono::milliseconds(249)));
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  RuntimeCore core(config(), clock, authority, relative);

  core.observe_gate(authority->snapshot());

  EXPECT_EQ(authority->inhibit_count(), 1U);
  EXPECT_EQ(core.state().gate_state, GateState::Inhibited);
  EXPECT_EQ(core.state().availability, RuntimeAvailability::Available);
  EXPECT_TRUE(core.usable());
}

TEST(RuntimeCore, StartupRejectsStaleZeroRevokedAtAuthorityDeadline)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  authority->set_snapshot(GateSnapshot{
        kGateId, 15U, "", GateState::Inhibited,
        true, true, true, false});
  const auto deadline_result = converged_stale_inhibit(
    std::chrono::milliseconds(250));
  ASSERT_FALSE(deadline_result.zero_proven);
  authority->set_inhibit_result(deadline_result);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  RuntimeCore core(config(), clock, authority, relative);

  core.observe_gate(authority->snapshot());

  EXPECT_EQ(core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_FALSE(core.usable());
}

TEST(RuntimeCore, StartupRejectsTransportUnavailableStaleZero)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  authority->set_snapshot(GateSnapshot{
        kGateId, 15U, "", GateState::Inhibited,
        true, true, true, false});
  authority->set_inhibit_result(converged_stale_inhibit(
      std::chrono::milliseconds(249), true));
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  RuntimeCore core(config(), clock, authority, relative);

  core.observe_gate(authority->snapshot());

  EXPECT_EQ(core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_FALSE(core.usable());
}

TEST(RuntimeCore, StartupRejectsStaleInhibitWithoutEveryCurrentZeroProof)
{
  struct InvalidResult
  {
    const char * name;
    GateSnapshot snapshot;
    bool tuple_stale{true};
  };
  const auto current_zero = GateSnapshot{
    kGateId, 16U, "", GateState::Inhibited,
    true, true, true, true};
  std::vector<InvalidResult> invalid_results;
  auto add_invalid = [&invalid_results, &current_zero](
    const char * name, const auto & mutate, bool tuple_stale = true)
    {
      auto snapshot = current_zero;
      mutate(snapshot);
      invalid_results.push_back(InvalidResult{name, snapshot, tuple_stale});
    };
  add_invalid("different identity", [](auto & value) {
      value.gate_instance_id = kOtherGateId;
    });
  add_invalid("endpoint unavailable", [](auto & value) {
      value.endpoint_available = false;
    });
  add_invalid("not inhibited", [](auto & value) {
      value.state = GateState::Prepared;
    });
  add_invalid("motion not inhibited", [](auto & value) {
      value.motion_inhibited = false;
    });
  add_invalid("zero not selected", [](auto & value) {
      value.zero_selected = false;
    });
  add_invalid("zero not published", [](auto & value) {
      value.zero_published = false;
    });
  add_invalid("sequence did not advance", [](auto & value) {
      value.control_seq = 15U;
    });
  add_invalid("ordinary reject", [](auto &) {}, false);

  for (const auto & invalid : invalid_results) {
    SCOPED_TRACE(invalid.name);
    auto clock = std::make_shared<ScriptedSteadyClock>();
    auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
    authority->set_snapshot(GateSnapshot{
          kGateId, 15U, "", GateState::Inhibited,
          true, true, true, false});
    authority->set_inhibit_result(AuthorityResult{
          false,
          true,
          true,
          invalid.snapshot,
          "",
          "scripted rejection",
          invalid.tuple_stale});
    auto relative = std::make_shared<ScriptedRelativeMotionPort>();
    RuntimeCore core(config(), clock, authority, relative);

    core.observe_gate(authority->snapshot());

    EXPECT_EQ(authority->inhibit_count(), 1U);
    EXPECT_EQ(core.state().availability, RuntimeAvailability::Faulted);
    EXPECT_FALSE(core.usable());
    EXPECT_FALSE(core.admit(goal(1U)).accepted);
  }
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

TEST(RuntimeCore, SameSequenceFaultAtMaximumWatermarkFailsClosed)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  authority->set_snapshot(GateSnapshot{
        kGateId, std::numeric_limits<std::uint64_t>::max(), "",
        GateState::Inhibited, true, true, true, true, "", false, false});
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  RuntimeCore core(config(), clock, authority, relative);
  core.observe_gate(authority->snapshot());

  core.observe_gate(GateSnapshot{
        kGateId, std::numeric_limits<std::uint64_t>::max(), "",
        GateState::Faulted, true, true, true, true, "", false, false});

  EXPECT_EQ(core.state().admission_epoch, 2U);
  EXPECT_EQ(core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_FALSE(core.has_active_mission());
}

TEST(RuntimeCore, SameSequenceEndpointLossStopsActiveMission)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);
  const auto current_control_seq = fixture.authority->snapshot().control_seq;

  fixture.core.observe_gate(GateSnapshot{
        kGateId, current_control_seq, "", GateState::Faulted,
        false, true, true, true, "", false, false});

  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::SafetyFault);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);
  EXPECT_EQ(fixture.core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_FALSE(fixture.core.has_active_mission());
  EXPECT_EQ(fixture.authority->inhibit_count(), 1U);
  EXPECT_TRUE(fixture.authority->snapshot().motion_inhibited);
  EXPECT_TRUE(fixture.authority->snapshot().zero_selected);
  EXPECT_TRUE(fixture.authority->snapshot().zero_published);
}

TEST(RuntimeCore, DuplicateSameSequenceFaultDoesNotRepeatSafetyAction)
{
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  authority->set_snapshot(GateSnapshot{
        kGateId, std::numeric_limits<std::uint64_t>::max(), "",
        GateState::Inhibited, true, true, true, true, "", false, false});
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  std::vector<MissionResult> results;
  RuntimeCore core(
    config(), clock, authority, relative, {}, {},
    [&results](std::uint64_t, const MissionResult & result) {
      results.push_back(result);
    });
  core.observe_gate(authority->snapshot());
  ASSERT_TRUE(core.admit(goal(1U)).accepted);

  const auto same_sequence_fault = GateSnapshot{
    kGateId, std::numeric_limits<std::uint64_t>::max(), "",
    GateState::Faulted, true, true, true, true, "", false, false};
  core.observe_gate(same_sequence_fault);

  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(core.state().admission_epoch, 2U);
  EXPECT_EQ(authority->inhibit_count(), 1U);
  EXPECT_EQ(core.state().gate_state, GateState::Inhibited);

  core.observe_gate(same_sequence_fault);

  EXPECT_EQ(results.size(), 1U);
  EXPECT_EQ(core.state().admission_epoch, 2U);
  EXPECT_EQ(authority->inhibit_count(), 1U);
  EXPECT_EQ(core.state().gate_state, GateState::Inhibited);
  EXPECT_EQ(core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_FALSE(core.has_active_mission());
}

TEST(RuntimeCore, SameSequenceLossOfZeroProofFailsClosed)
{
  Fixture fixture;
  const auto current_control_seq = fixture.authority->snapshot().control_seq;

  fixture.core.observe_gate(GateSnapshot{
        kGateId, current_control_seq, "", GateState::Prepared,
        true, false, false, false, "", false, false});

  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);
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

TEST(RuntimeCore, IdleDependencyLossRemainsUnavailableAndRecovers)
{
  Fixture fixture;
  ASSERT_EQ(
    fixture.core.state().availability,
    RuntimeAvailability::Available);
  ASSERT_EQ(fixture.core.state().admission_epoch, 1U);
  fixture.states.clear();

  fixture.relative->set_healthy(false);
  fixture.core.observe_dependencies();

  EXPECT_EQ(
    fixture.core.state().availability,
    RuntimeAvailability::Unavailable);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);
  EXPECT_FALSE(fixture.core.has_active_mission());
  EXPECT_TRUE(fixture.results.empty());
  ASSERT_FALSE(fixture.states.empty());
  EXPECT_TRUE(std::all_of(
    fixture.states.cbegin(), fixture.states.cend(), [](const auto & state) {
        return state.availability == RuntimeAvailability::Unavailable &&
               state.admission_epoch == 2U;
    }));

  fixture.states.clear();
  fixture.relative->set_healthy(true);
  fixture.core.observe_dependencies();

  EXPECT_EQ(
    fixture.core.state().availability,
    RuntimeAvailability::Available);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);
  EXPECT_TRUE(fixture.results.empty());
  ASSERT_FALSE(fixture.states.empty());
  EXPECT_TRUE(std::all_of(
    fixture.states.cbegin(), fixture.states.cend(), [](const auto & state) {
        return state.availability == RuntimeAvailability::Available &&
               state.admission_epoch == 2U;
    }));
}

TEST(RuntimeCore, IdleSafetyFaultStillLatchesRuntimeFault)
{
  Fixture fixture;
  fixture.relative->set_safety_faulted(true);
  fixture.relative->set_healthy(false);

  fixture.core.observe_dependencies();

  EXPECT_EQ(
    fixture.core.state().availability,
    RuntimeAvailability::Faulted);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);

  fixture.relative->set_healthy(true);
  fixture.core.observe_dependencies();

  EXPECT_EQ(
    fixture.core.state().availability,
    RuntimeAvailability::Faulted);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);
}

TEST(RuntimeCore, IdleDependencyLossAtMaximumEpochFailsClosed)
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
  relative->set_healthy(false);

  core.observe_dependencies();

  EXPECT_EQ(core.state().availability, RuntimeAvailability::Faulted);
  EXPECT_EQ(
    core.state().admission_epoch,
    std::numeric_limits<std::uint64_t>::max());
  EXPECT_FALSE(core.has_active_mission());
  EXPECT_TRUE(results.empty());
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

TEST(RuntimeCore, ReplacementGateZeroRearmsAfterGateSafetyFault)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);

  fixture.core.observe_gate(GateSnapshot{
        kOtherGateId, 1U, "", GateState::Faulted, false,
        true, true, false, "", false, false});

  ASSERT_EQ(fixture.core.state().availability, RuntimeAvailability::Faulted);
  ASSERT_EQ(fixture.core.state().admission_epoch, 2U);
  const auto replacement = GateSnapshot{
    kReplacementGateId, 1U, "", GateState::Inhibited,
    true, true, true, true, "", false, false};
  fixture.authority->set_snapshot(replacement);

  fixture.core.observe_gate(replacement);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);
  EXPECT_EQ(fixture.core.state().availability, RuntimeAvailability::Faulted);

  fixture.relative->set_rearm_after_gate_replacement(true);
  // The authority may advance again after the replacement callback was
  // queued.  Rearm must commit the immutable tuple accepted by the Adapter;
  // this newer observation is handled as a subsequent Gate event.
  fixture.authority->set_snapshot(GateSnapshot{
        kOtherGateId, 2U, "", GateState::Faulted, false,
        true, true, false, "", false, false});
  fixture.core.observe_gate(replacement);

  EXPECT_EQ(fixture.core.state().admission_epoch, 3U);
  EXPECT_EQ(fixture.core.state().availability, RuntimeAvailability::Available);
  EXPECT_EQ(fixture.core.state().gate_state, GateState::Inhibited);
  EXPECT_TRUE(fixture.core.usable());
  EXPECT_FALSE(fixture.core.has_active_mission());
}

TEST(RuntimeCore, PendingReplacementGateRearmsOnTickWithoutDuplicateSnapshot)
{
  Fixture fixture;

  fixture.core.observe_gate(GateSnapshot{
        kOtherGateId, 1U, "", GateState::Faulted, false,
        true, true, false, "", false, false});
  ASSERT_EQ(fixture.core.state().admission_epoch, 2U);
  ASSERT_EQ(fixture.core.state().availability, RuntimeAvailability::Faulted);

  const auto replacement = GateSnapshot{
    kReplacementGateId, 1U, "", GateState::Inhibited,
    true, true, true, false, "", false, false};
  fixture.authority->set_snapshot(replacement);
  fixture.core.observe_gate(replacement);
  ASSERT_EQ(fixture.relative->rearm_after_gate_replacement_count(), 1U);
  ASSERT_EQ(fixture.core.state().admission_epoch, 2U);
  ASSERT_EQ(fixture.core.state().availability, RuntimeAvailability::Faulted);

  fixture.relative->set_rearm_after_gate_replacement(true);
  auto reasserted_replacement = replacement;
  reasserted_replacement.control_seq++;
  reasserted_replacement.zero_published = true;
  fixture.relative->set_rearm_accepted_snapshot(reasserted_replacement);
  fixture.core.on_tick();

  EXPECT_EQ(fixture.relative->rearm_after_gate_replacement_count(), 2U);
  EXPECT_EQ(
    fixture.relative->last_rearm_gate_instance_id(), kReplacementGateId);
  EXPECT_EQ(fixture.core.state().admission_epoch, 3U);
  EXPECT_EQ(fixture.core.state().availability, RuntimeAvailability::Available);
  EXPECT_EQ(fixture.core.state().gate_state, GateState::Inhibited);
  EXPECT_TRUE(fixture.core.usable());
}

TEST(RuntimeCore, ReplacementGateWaitsForActiveMissionTerminalBeforeRearm)
{
  Fixture fixture;
  fixture.relative->set_rearm_after_gate_replacement(true);
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);

  const auto replacement = GateSnapshot{
    kReplacementGateId, 1U, "", GateState::Inhibited,
    true, true, true, true, "", false, false};
  fixture.authority->set_snapshot(replacement);
  fixture.core.observe_gate(replacement);

  ASSERT_EQ(fixture.results.size(), 1U);
  EXPECT_EQ(fixture.results.front().code, MissionResultCode::SafetyFault);
  EXPECT_FALSE(fixture.core.has_active_mission());
  EXPECT_EQ(fixture.relative->rearm_after_gate_replacement_count(), 0U);
  EXPECT_EQ(fixture.core.state().admission_epoch, 2U);

  fixture.core.on_tick();

  EXPECT_EQ(fixture.relative->rearm_after_gate_replacement_count(), 1U);
  EXPECT_EQ(fixture.core.state().admission_epoch, 3U);
  EXPECT_EQ(fixture.core.state().availability, RuntimeAvailability::Available);
}

TEST(RuntimeCore, ReplacementGateRearmAdvancesNodeOwnedAdmissionFence)
{
  RuntimeEmergencyFence fence(1U);
  auto runtime_config = config();
  runtime_config.admission_epoch_advance =
    [&fence](const std::uint64_t current, const std::uint64_t next) {
      return fence.advance_epoch(current, next);
    };
  auto clock = std::make_shared<ScriptedSteadyClock>();
  auto authority = std::make_shared<ScriptedMotionAuthorityPort>(kGateId);
  auto relative = std::make_shared<ScriptedRelativeMotionPort>();
  RuntimeCore core(runtime_config, clock, authority, relative);
  core.observe_gate(authority->snapshot());

  core.observe_gate(GateSnapshot{
        kOtherGateId, 1U, "", GateState::Faulted, false,
        true, true, false, "", false, false});
  ASSERT_EQ(core.state().admission_epoch, 2U);
  EXPECT_FALSE(fence.admission_allowed(1U));
  EXPECT_TRUE(fence.admission_allowed(2U));

  const auto replacement = GateSnapshot{
    kReplacementGateId, 1U, "", GateState::Inhibited,
    true, true, true, true, "", false, false};
  authority->set_snapshot(replacement);
  core.observe_gate(replacement);
  relative->set_rearm_after_gate_replacement(true);
  core.observe_gate(replacement);

  EXPECT_EQ(core.state().admission_epoch, 3U);
  EXPECT_FALSE(fence.admission_allowed(2U));
  EXPECT_TRUE(fence.admission_allowed(3U));
  EXPECT_TRUE(core.usable());
}

TEST(RuntimeCore, ReplacementGateCanBecomeReadyAfterUnavailableDiscovery)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);

  const auto unavailable_replacement = GateSnapshot{
    kReplacementGateId, 1U, "", GateState::Faulted, false,
    true, true, false, "", false, false};
  fixture.authority->set_snapshot(unavailable_replacement);
  fixture.authority->set_next_failure("replacement Gate is unavailable");
  fixture.core.observe_gate(unavailable_replacement);
  ASSERT_EQ(fixture.core.state().admission_epoch, 2U);
  ASSERT_EQ(fixture.core.state().availability, RuntimeAvailability::Faulted);
  ASSERT_EQ(fixture.relative->rearm_after_gate_replacement_count(), 0U);

  fixture.relative->set_rearm_after_gate_replacement(true);
  const auto ready_replacement = GateSnapshot{
    kReplacementGateId, 2U, "", GateState::Inhibited, true,
    true, true, true, "", false, false};
  fixture.authority->set_snapshot(ready_replacement);
  fixture.core.observe_gate(ready_replacement);

  EXPECT_EQ(fixture.relative->rearm_after_gate_replacement_count(), 1U);
  EXPECT_EQ(
    fixture.relative->last_rearm_gate_instance_id(), kReplacementGateId);
  EXPECT_EQ(fixture.core.state().admission_epoch, 3U);
  EXPECT_EQ(fixture.core.state().availability, RuntimeAvailability::Available);
}

TEST(RuntimeCore, NewerReplacementSupersedesUnavailableReplacementCandidate)
{
  Fixture fixture;
  ASSERT_TRUE(fixture.core.admit(goal(1U)).accepted);

  const auto unavailable_replacement = GateSnapshot{
    kReplacementGateId, 1U, "", GateState::Faulted, false,
    true, true, false, "", false, false};
  fixture.authority->set_snapshot(unavailable_replacement);
  fixture.authority->set_next_failure("replacement Gate is unavailable");
  fixture.core.observe_gate(unavailable_replacement);
  ASSERT_EQ(fixture.core.state().admission_epoch, 2U);

  fixture.relative->set_rearm_after_gate_replacement(true);
  const auto newer_replacement = GateSnapshot{
    kOtherGateId, 1U, "", GateState::Inhibited, true,
    true, true, true, "", false, false};
  fixture.authority->set_snapshot(newer_replacement);
  fixture.core.observe_gate(newer_replacement);

  EXPECT_EQ(fixture.relative->rearm_after_gate_replacement_count(), 1U);
  EXPECT_EQ(fixture.relative->last_rearm_gate_instance_id(), kOtherGateId);
  EXPECT_EQ(fixture.core.state().admission_epoch, 3U);
  EXPECT_EQ(fixture.core.state().availability, RuntimeAvailability::Available);
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
