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

#include "voice_nav_mission/mission_action_result_router.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace voice_nav_mission
{
namespace
{

struct DeliveryRecord
{
  std::uint64_t mission_id{0U};
  ActionResultDelivery delivery;
};

TEST(MissionActionResultRouter, SynchronousSuccessIsDeliveredExactlyOnce)
{
  MissionActionResultRouter router;
  std::vector<DeliveryRecord> deliveries;

  router.begin_admission();
  router.finish(7U, MissionResult{MissionResultCode::Succeeded, -1, "done"});
  EXPECT_TRUE(deliveries.empty());
  router.commit(
    7U,
    [&deliveries](std::uint64_t mission_id, const ActionResultDelivery & delivery) {
      deliveries.push_back(DeliveryRecord{mission_id, delivery});
    });
  router.finish(7U, MissionResult{
        MissionResultCode::Succeeded, -1, "late duplicate"});

  ASSERT_EQ(deliveries.size(), 1U);
  EXPECT_EQ(deliveries.front().mission_id, 7U);
  EXPECT_EQ(
    deliveries.front().delivery.status, OuterActionStatus::Succeeded);
  EXPECT_EQ(
    deliveries.front().delivery.result.code, MissionResultCode::Succeeded);
}

TEST(MissionActionResultRouter, SynchronousStartExceptionIsAbortedExactlyOnce)
{
  MissionActionResultRouter router;
  std::vector<DeliveryRecord> deliveries;

  router.begin_admission();
  router.finish(9U, MissionResult{
        MissionResultCode::InternalError, -1, "child start threw"});
  router.commit(
    9U,
    [&deliveries](std::uint64_t mission_id, const ActionResultDelivery & delivery) {
      deliveries.push_back(DeliveryRecord{mission_id, delivery});
    });
  router.finish(9U, MissionResult{
        MissionResultCode::InternalError, -1, "late duplicate"});

  ASSERT_EQ(deliveries.size(), 1U);
  EXPECT_EQ(deliveries.front().mission_id, 9U);
  EXPECT_EQ(deliveries.front().delivery.status, OuterActionStatus::Aborted);
  EXPECT_EQ(
    deliveries.front().delivery.result.code, MissionResultCode::InternalError);
}

TEST(MissionActionAdapterBoundary, SynchronousSuccessCrossesGoalHandleBoundaryOnce)
{
  MissionActionAdapterBoundary adapter;
  std::vector<DeliveryRecord> deliveries;
  std::vector<MissionResult> rejections;
  std::size_t registered = 0U;

  adapter.on_accepted(
    MissionGoal{},
    [&adapter](const MissionGoal &) {
      adapter.finish(7U, MissionResult{MissionResultCode::Succeeded, -1, "done"});
      return AdmissionResult{7U, true, {}};
    },
    [&registered](std::uint64_t) {++registered;},
    [&deliveries](std::uint64_t mission_id, const ActionResultDelivery & delivery) {
      deliveries.push_back(DeliveryRecord{mission_id, delivery});
    },
    [&rejections](const MissionResult & result) {rejections.push_back(result);});
  adapter.finish(7U, MissionResult{
        MissionResultCode::Succeeded, -1, "late duplicate"});

  ASSERT_EQ(registered, 1U);
  ASSERT_TRUE(rejections.empty());
  ASSERT_EQ(deliveries.size(), 1U);
  EXPECT_EQ(deliveries.front().mission_id, 7U);
  EXPECT_EQ(deliveries.front().delivery.status, OuterActionStatus::Succeeded);
  EXPECT_EQ(deliveries.front().delivery.result.code, MissionResultCode::Succeeded);
  EXPECT_EQ(deliveries.front().delivery.result.detail, "done");
}

TEST(MissionActionAdapterBoundary, SynchronousStartExceptionCrossesGoalHandleBoundaryOnce)
{
  MissionActionAdapterBoundary adapter;
  std::vector<DeliveryRecord> deliveries;
  std::vector<MissionResult> rejections;
  std::size_t registered = 0U;

  adapter.on_accepted(
    MissionGoal{},
    [&adapter](const MissionGoal &) {
      // Equivalent to RuntimeCore::start_step() invoking its result callback
      // before admit() returns after a child start exception.
      adapter.finish(9U, MissionResult{
          MissionResultCode::InternalError, -1, "child start threw"});
      return AdmissionResult{9U, true, {}};
    },
    [&registered](std::uint64_t) {++registered;},
    [&deliveries](std::uint64_t mission_id, const ActionResultDelivery & delivery) {
      deliveries.push_back(DeliveryRecord{mission_id, delivery});
    },
    [&rejections](const MissionResult & result) {rejections.push_back(result);});
  adapter.finish(9U, MissionResult{
        MissionResultCode::InternalError, 0, "late duplicate"});

  ASSERT_EQ(registered, 1U);
  ASSERT_TRUE(rejections.empty());
  ASSERT_EQ(deliveries.size(), 1U);
  EXPECT_EQ(deliveries.front().mission_id, 9U);
  EXPECT_EQ(deliveries.front().delivery.status, OuterActionStatus::Aborted);
  EXPECT_EQ(
    deliveries.front().delivery.result.code, MissionResultCode::InternalError);
  EXPECT_EQ(deliveries.front().delivery.result.failed_step, -1);
}

}  // namespace
}  // namespace voice_nav_mission
