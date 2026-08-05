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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace voice_nav_mission
{
namespace
{

constexpr char kRuntimeId[] = "0123456789abcdef0123456789abcdef";
constexpr char kGateId[] = "fedcba9876543210fedcba9876543210";

MissionGoal goal(
  std::uint64_t source_seq,
  std::vector<MissionStep> steps = {
    MissionStep{
      static_cast<std::uint8_t>(MissionStepKind::MoveDistance),
      0.5F,
      0.0F,
      ""}})
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

TEST(RuntimeCore, FakeFSMIsOrderedBusyAndExactlyOnce)
{
  Fixture fixture;
  std::vector<MissionStep> steps{
    MissionStep{
      static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.5F, 0.0F, ""},
    MissionStep{
      static_cast<std::uint8_t>(MissionStepKind::RotateAngle), 1.0F, 0.0F, ""}};
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
  EXPECT_EQ(fixture.authority->inhibit_count(), 3U);
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

}  // namespace
}  // namespace voice_nav_mission
