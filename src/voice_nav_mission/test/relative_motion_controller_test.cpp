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

#include "voice_nav_mission/relative_motion_controller.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;

constexpr MotionToken kToken{7U, 3U, 9U, 1U};

MissionStep move(const float distance)
{
  return MissionStep{
    static_cast<std::uint8_t>(MissionStepKind::MoveDistance), distance, 0.0F, ""};
}

MissionStep rotate(const float angle)
{
  return MissionStep{
    static_cast<std::uint8_t>(MissionStepKind::RotateAngle), 0.0F, angle, ""};
}

RelativeMotionOdom odom(
  const double x,
  const double y,
  const double yaw,
  const double linear = 0.0,
  const double angular = 0.0)
{
  return RelativeMotionOdom{x, y, yaw, linear, angular};
}

TEST(RelativeMotionController, MoveUsesSignedInitialHeadingProjection)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  EXPECT_EQ(
    controller.start(kToken, move(0.5F), t0).kind,
    RelativeMotionEventKind::Running);
  EXPECT_EQ(
    controller.observe_odom(odom(0.0, 0.0, 0.0), t0).kind,
    RelativeMotionEventKind::Running);
  EXPECT_DOUBLE_EQ(controller.command().linear_x_mps, 0.25);

  const auto forward = controller.observe_odom(odom(0.20, 0.50, 0.0), t0 + 200ms);
  EXPECT_EQ(forward.kind, RelativeMotionEventKind::Running);
  EXPECT_DOUBLE_EQ(forward.command.linear_x_mps, 0.25);
  EXPECT_NEAR(forward.progress, 0.4, 1e-12);

  const auto lateral = controller.observe_odom(odom(0.10, 1.0, 0.0), t0 + 300ms);
  EXPECT_EQ(lateral.kind, RelativeMotionEventKind::Running);
  EXPECT_NEAR(lateral.progress, 0.4, 1e-12);

  const auto near_target = controller.observe_odom(odom(0.46, 0.0, 0.0), t0 + 400ms);
  EXPECT_EQ(near_target.kind, RelativeMotionEventKind::ZeroRequested);
  EXPECT_DOUBLE_EQ(near_target.command.linear_x_mps, 0.0);
  EXPECT_NEAR(near_target.progress, 0.92, 1e-12);
}

TEST(RelativeMotionController, NegativeMoveDoesNotCountBackwardProjectionAsProgress)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, move(-0.5F), t0);
  controller.observe_odom(odom(0.0, 0.0, 0.0), t0);
  EXPECT_DOUBLE_EQ(controller.command().linear_x_mps, -0.25);

  const auto wrong_way = controller.observe_odom(odom(0.20, 0.0, 0.0), t0 + 200ms);
  EXPECT_EQ(wrong_way.kind, RelativeMotionEventKind::Running);
  EXPECT_DOUBLE_EQ(wrong_way.progress, 0.0);
  EXPECT_DOUBLE_EQ(wrong_way.command.linear_x_mps, -0.25);
}

TEST(RelativeMotionController, NegativeMoveUsesTheFirstOdomAsItsReference)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, move(-0.5F), t0);
  controller.observe_odom(odom(0.45, 0.0, 0.0), t0);
  const auto moving = controller.observe_odom(
    odom(0.42, 0.0, 0.0), t0 + 1000ms);

  EXPECT_EQ(moving.kind, RelativeMotionEventKind::Running);
  EXPECT_GT(moving.progress, 0.0);
  EXPECT_DOUBLE_EQ(moving.command.linear_x_mps, -0.25);
}

TEST(RelativeMotionController, CommandLimitsHoldAtMaximumAndMinimum)
{
  const auto t0 = SteadyClockPort::TimePoint{};

  RelativeMotionController maximum_move;
  maximum_move.start(kToken, move(2.0F), t0);
  const auto max_move = maximum_move.observe_odom(odom(0.0, 0.0, 0.0), t0);
  EXPECT_DOUBLE_EQ(max_move.command.linear_x_mps, 0.25);

  RelativeMotionPolicy low_move_policy;
  low_move_policy.move_tolerance_m = 0.01;
  RelativeMotionController minimum_move(low_move_policy);
  minimum_move.start(kToken, move(0.02F), t0);
  const auto min_move = minimum_move.observe_odom(odom(0.0, 0.0, 0.0), t0);
  EXPECT_DOUBLE_EQ(min_move.command.linear_x_mps, 0.05);

  RelativeMotionController maximum_rotate;
  maximum_rotate.start(kToken, rotate(6.283185F), t0);
  const auto max_rotate = maximum_rotate.observe_odom(odom(0.0, 0.0, 0.0), t0);
  EXPECT_DOUBLE_EQ(max_rotate.command.angular_z_rps, 0.80);

  RelativeMotionPolicy low_rotate_policy;
  low_rotate_policy.rotate_tolerance_rad = 0.01;
  RelativeMotionController minimum_rotate(low_rotate_policy);
  minimum_rotate.start(kToken, rotate(0.02F), t0);
  const auto min_rotate = minimum_rotate.observe_odom(odom(0.0, 0.0, 0.0), t0);
  EXPECT_DOUBLE_EQ(min_rotate.command.angular_z_rps, 0.10);
}

TEST(RelativeMotionController, RotateUnwrapsAcrossPiWithoutChangingDirection)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, rotate(0.50F), t0);
  controller.observe_odom(odom(0.0, 0.0, 3.10), t0);
  const auto crossed = controller.observe_odom(odom(0.0, 0.0, -3.08), t0 + 100ms);

  EXPECT_EQ(crossed.kind, RelativeMotionEventKind::Running);
  EXPECT_GT(crossed.command.angular_z_rps, 0.0);
  EXPECT_NEAR(crossed.progress, 0.206, 0.01);

  const auto arrived = controller.observe_odom(odom(0.0, 0.0, -2.683), t0 + 300ms);
  EXPECT_EQ(arrived.kind, RelativeMotionEventKind::ZeroRequested);
  EXPECT_DOUBLE_EQ(arrived.command.angular_z_rps, 0.0);
}

TEST(RelativeMotionController, NegativeRotateUnwrapsAcrossPiWithoutReversing)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, rotate(-0.50F), t0);
  controller.observe_odom(odom(0.0, 0.0, -3.10), t0);
  const auto crossed = controller.observe_odom(
    odom(0.0, 0.0, 3.08), t0 + 100ms);

  EXPECT_EQ(crossed.kind, RelativeMotionEventKind::Running);
  EXPECT_LT(crossed.command.angular_z_rps, 0.0);
  EXPECT_NEAR(crossed.progress, 0.206, 0.01);
}

TEST(RelativeMotionController, PositiveAndNegativeOneAndHalfPiCrossTheWrap)
{
  const auto t0 = SteadyClockPort::TimePoint{};

  RelativeMotionController positive;
  positive.start(kToken, rotate(1.5708F), t0);
  positive.observe_odom(odom(0.0, 0.0, 3.0), t0);
  const auto positive_crossing = positive.observe_odom(
    odom(0.0, 0.0, -3.0), t0 + 100ms);
  EXPECT_EQ(positive_crossing.kind, RelativeMotionEventKind::Running);
  EXPECT_GT(positive_crossing.command.angular_z_rps, 0.0);
  const auto positive_complete = positive.observe_odom(
    odom(0.0, 0.0, -1.712385), t0 + 200ms);
  EXPECT_EQ(positive_complete.kind, RelativeMotionEventKind::ZeroRequested);
  EXPECT_GT(positive_complete.progress, 0.98);

  RelativeMotionController negative;
  negative.start(kToken, rotate(-1.5708F), t0);
  negative.observe_odom(odom(0.0, 0.0, -3.0), t0);
  const auto negative_crossing = negative.observe_odom(
    odom(0.0, 0.0, 3.0), t0 + 100ms);
  EXPECT_EQ(negative_crossing.kind, RelativeMotionEventKind::Running);
  EXPECT_LT(negative_crossing.command.angular_z_rps, 0.0);
  const auto negative_complete = negative.observe_odom(
    odom(0.0, 0.0, 1.712385), t0 + 200ms);
  EXPECT_EQ(negative_complete.kind, RelativeMotionEventKind::ZeroRequested);
  EXPECT_GT(negative_complete.progress, 0.98);
}

TEST(RelativeMotionController, PositiveAndNegativeFullTurnsTraverseTheWrap)
{
  constexpr double kPi = 3.14159265358979323846;
  const auto t0 = SteadyClockPort::TimePoint{};

  RelativeMotionController positive;
  positive.start(kToken, rotate(6.283185F), t0);
  positive.observe_odom(odom(0.0, 0.0, 0.0), t0);
  positive.observe_odom(odom(0.0, 0.0, kPi - 0.01), t0 + 100ms);
  positive.observe_odom(odom(0.0, 0.0, -kPi + 0.01), t0 + 200ms);
  const auto positive_complete = positive.observe_odom(
    odom(0.0, 0.0, 0.0), t0 + 300ms);
  EXPECT_EQ(positive_complete.kind, RelativeMotionEventKind::ZeroRequested);
  EXPECT_GT(positive_complete.progress, 0.98);
  EXPECT_DOUBLE_EQ(positive_complete.command.angular_z_rps, 0.0);

  RelativeMotionController negative;
  negative.start(kToken, rotate(-6.283185F), t0);
  negative.observe_odom(odom(0.0, 0.0, 0.0), t0);
  negative.observe_odom(odom(0.0, 0.0, -kPi + 0.01), t0 + 100ms);
  negative.observe_odom(odom(0.0, 0.0, kPi - 0.01), t0 + 200ms);
  const auto negative_complete = negative.observe_odom(
    odom(0.0, 0.0, 0.0), t0 + 300ms);
  EXPECT_EQ(negative_complete.kind, RelativeMotionEventKind::ZeroRequested);
  EXPECT_GT(negative_complete.progress, 0.98);
  EXPECT_DOUBLE_EQ(negative_complete.command.angular_z_rps, 0.0);
}

TEST(RelativeMotionController, ProgressIsMonotonicWhenTheRobotBacktracks)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, move(0.50F), t0);
  controller.observe_odom(odom(0.0, 0.0, 0.0), t0);
  const auto forward = controller.observe_odom(
    odom(0.30, 0.0, 0.0), t0 + 100ms);
  const auto backward = controller.observe_odom(
    odom(0.10, 0.0, 0.0), t0 + 200ms);

  EXPECT_NEAR(forward.progress, 0.60, 1e-12);
  EXPECT_DOUBLE_EQ(backward.progress, forward.progress);
}

TEST(RelativeMotionController, StallUsesHistoricalBestErrorAndIsBounded)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, move(0.5F), t0);
  controller.observe_odom(odom(0.0, 0.0, 0.0), t0);
  controller.observe_odom(odom(0.005, 0.0, 0.0), t0 + 500ms);
  controller.observe_odom(odom(0.005, 0.0, 0.0), t0 + 900ms);
  const auto stalled = controller.tick(t0 + 1000ms);

  EXPECT_EQ(stalled.kind, RelativeMotionEventKind::Failed);
  EXPECT_EQ(stalled.failure, RelativeMotionFailure::ExecutionFailed);
}

TEST(RelativeMotionController, MissingFreshOdomIsDependencyFailure)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, move(0.5F), t0);
  const auto failed = controller.tick(t0 + 201ms);

  EXPECT_EQ(failed.kind, RelativeMotionEventKind::Failed);
  EXPECT_EQ(failed.failure, RelativeMotionFailure::DependencyUnavailable);
}

TEST(RelativeMotionController, ConfiguredLivenessBudgetToleratesSimulatorJitter)
{
  RelativeMotionPolicy policy;
  policy.dependency_liveness_timeout = 500ms;
  RelativeMotionController controller(policy);
  const auto t0 = SteadyClockPort::TimePoint{};

  EXPECT_EQ(controller.start(kToken, move(0.5F), t0).kind, RelativeMotionEventKind::Running);
  EXPECT_EQ(
    controller.observe_odom(odom(0.0, 0.0, 0.0), t0).kind,
    RelativeMotionEventKind::Running);

  EXPECT_EQ(controller.tick(t0 + 300ms).kind, RelativeMotionEventKind::Running);
  const auto failed = controller.tick(t0 + 501ms);
  EXPECT_EQ(failed.kind, RelativeMotionEventKind::Failed);
  EXPECT_EQ(failed.failure, RelativeMotionFailure::DependencyUnavailable);
}

TEST(RelativeMotionController, DeadlineIsIndependentOfGoalMissionDeadline)
{
  RelativeMotionPolicy policy;
  policy.stall_window = 10000ms;
  RelativeMotionController controller(policy);
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, move(0.5F), t0);
  controller.observe_odom(odom(0.0, 0.0, 0.0), t0);
  for (int i = 1; i <= 6; ++i) {
    const auto sample = controller.observe_odom(
      odom(0.02 * i, 0.0, 0.0), t0 + std::chrono::milliseconds(i * 1000));
    ASSERT_EQ(sample.kind, RelativeMotionEventKind::Running);
  }
  const auto timed_out = controller.tick(t0 + 7000ms);
  EXPECT_EQ(timed_out.kind, RelativeMotionEventKind::Failed);
  EXPECT_EQ(timed_out.failure, RelativeMotionFailure::Timeout);
}

TEST(RelativeMotionController, ZeroProofThenStationarityRequiresFreshStableOdom)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, move(0.5F), t0);
  controller.observe_odom(odom(0.0, 0.0, 0.0), t0);
  ASSERT_EQ(
    controller.observe_odom(odom(0.46, 0.0, 0.0, 0.0, 0.0), t0 + 100ms).kind,
    RelativeMotionEventKind::ZeroRequested);
  EXPECT_EQ(
    controller.confirm_gate_zero(t0 + 110ms).kind,
    RelativeMotionEventKind::StationarityPending);
  EXPECT_EQ(
    controller.observe_odom(odom(0.46, 0.0, 0.0, 0.0, 0.0), t0 + 250ms).kind,
    RelativeMotionEventKind::StationarityPending);
  const auto complete = controller.observe_odom(
    odom(0.46, 0.0, 0.0, 0.0, 0.0), t0 + 450ms);
  EXPECT_EQ(complete.kind, RelativeMotionEventKind::Completed);
  EXPECT_TRUE(complete.stationarity_proven);
  EXPECT_TRUE(controller.stationarity_proven());
}

TEST(RelativeMotionController, StationarityIgnoresOdomReceivedBeforeZeroProof)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, move(0.5F), t0);
  controller.observe_odom(odom(0.0, 0.0, 0.0), t0);
  ASSERT_EQ(
    controller.observe_odom(odom(0.46, 0.0, 0.0), t0 + 100ms).kind,
    RelativeMotionEventKind::ZeroRequested);
  ASSERT_EQ(
    controller.confirm_gate_zero(t0 + 110ms).kind,
    RelativeMotionEventKind::StationarityPending);

  EXPECT_EQ(
    controller.observe_odom(odom(0.46, 0.0, 0.0), t0 + 105ms).kind,
    RelativeMotionEventKind::StationarityPending);
  EXPECT_EQ(
    controller.tick(t0 + 310ms).kind,
    RelativeMotionEventKind::StationarityPending);
  EXPECT_EQ(
    controller.observe_odom(odom(0.46, 0.0, 0.0), t0 + 310ms).kind,
    RelativeMotionEventKind::StationarityPending);
  EXPECT_EQ(
    controller.tick(t0 + 510ms).kind,
    RelativeMotionEventKind::Completed);
}

TEST(RelativeMotionController, StationarityDeadlineIsAbsoluteAtZeroProof)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, move(0.5F), t0);
  controller.observe_odom(odom(0.0, 0.0, 0.0), t0);
  controller.observe_odom(odom(0.46, 0.0, 0.0), t0 + 100ms);
  controller.confirm_gate_zero(t0 + 110ms);

  const auto failed = controller.tick(t0 + 1310ms);
  EXPECT_EQ(failed.kind, RelativeMotionEventKind::Failed);
  EXPECT_EQ(failed.failure, RelativeMotionFailure::SafetyFault);
}

TEST(RelativeMotionController, NonStationaryOdomFailsTheSafetyDeadline)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, move(0.5F), t0);
  controller.observe_odom(odom(0.0, 0.0, 0.0), t0);
  controller.observe_odom(odom(0.46, 0.0, 0.0), t0 + 100ms);
  controller.confirm_gate_zero(t0 + 110ms);
  const auto failed = controller.observe_odom(
    odom(0.46, 0.0, 0.0, 0.10, 0.0), t0 + 1311ms);

  EXPECT_EQ(failed.kind, RelativeMotionEventKind::Failed);
  EXPECT_EQ(failed.failure, RelativeMotionFailure::SafetyFault);
}

TEST(RelativeMotionController, ZeroProofHasAnIndependentBoundedDeadline)
{
  RelativeMotionController controller;
  const auto t0 = SteadyClockPort::TimePoint{};

  controller.start(kToken, move(0.5F), t0);
  (void)controller.observe_odom(odom(0.0, 0.0, 0.0), t0);
  (void)controller.request_safe_stop(RelativeMotionStopIntent::Cancel, t0 + 10ms);

  const auto failed = controller.tick(t0 + 311ms);
  EXPECT_EQ(failed.kind, RelativeMotionEventKind::Failed);
  EXPECT_EQ(failed.failure, RelativeMotionFailure::SafetyFault);
}

}  // namespace
}  // namespace voice_nav_mission
