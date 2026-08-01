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

#include "writer_observation.hpp"

#include <gtest/gtest.h>

#include <rmw/qos_profiles.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;

WriterGid writer_gid(std::uint8_t suffix)
{
  WriterGid gid{};
  gid.back() = suffix;
  return gid;
}

rmw_qos_profile_t candidate_qos()
{
  auto qos = rmw_qos_profile_default;
  qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos.depth = 1U;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  return qos;
}

WriterEndpointObservation endpoint(
  WriterGid gid,
  std::string node_name)
{
  return WriterEndpointObservation{
    "geometry_msgs/msg/TwistStamped",
    std::move(node_name),
    "/",
    RMW_ENDPOINT_PUBLISHER,
    candidate_qos(),
    gid};
}

TEST(WriterObservationSession, PinsUnresolvedIdentityUntilTheSameWriterResolves)
{
  WriterObservationSession session({
    "geometry_msgs/msg/TwistStamped",
    "/collision_monitor"});
  const auto first_gid = writer_gid(0x11U);

  const auto pending = session.observe(
    {endpoint(first_gid, "")}, 7ms);
  ASSERT_FALSE(pending.ready);
  EXPECT_EQ(pending.reason, Reason::WriterMetadataPending);
  EXPECT_EQ(pending.writer_gid, first_gid);

  const auto resolved = session.observe(
    {endpoint(first_gid, "collision_monitor")}, 19ms);
  ASSERT_TRUE(resolved.ready);
  EXPECT_EQ(resolved.reason, Reason::None);
  EXPECT_EQ(resolved.writer_gid, first_gid);

  const auto replacement = session.observe(
    {endpoint(writer_gid(0x22U), "collision_monitor")}, 23ms);
  EXPECT_FALSE(replacement.ready);
  EXPECT_EQ(replacement.reason, Reason::WriterMismatch);
}

TEST(WriterObservationSession, ReplacementPoisonsPinnedGenerationUntilReset)
{
  WriterObservationSession session({
    "geometry_msgs/msg/TwistStamped",
    "/collision_monitor"});
  const auto first_gid = writer_gid(0x31U);
  const auto replacement_gid = writer_gid(0x32U);

  ASSERT_EQ(
    session.observe({endpoint(first_gid, "")}, 2ms).reason,
    Reason::WriterMetadataPending);
  EXPECT_EQ(
    session.observe(
      {endpoint(replacement_gid, "collision_monitor")}, 3ms).reason,
    Reason::WriterMismatch);
  EXPECT_EQ(
    session.observe({endpoint(first_gid, "collision_monitor")}, 4ms).reason,
    Reason::WriterMismatch);

  session.reset();
  const auto next_generation = session.observe(
    {endpoint(replacement_gid, "collision_monitor")}, 1ms);
  EXPECT_TRUE(next_generation.ready);
  EXPECT_EQ(next_generation.writer_gid, replacement_gid);
}

TEST(WriterObservationSession, ConfirmedSameGidSurvivesIdentityOnlyGraphRegression)
{
  WriterObservationSession session({
    "geometry_msgs/msg/TwistStamped",
    "/collision_monitor"});
  const auto gid = writer_gid(0x41U);

  ASSERT_TRUE(
    session.observe({endpoint(gid, "collision_monitor")}, 1ms).ready);
  const auto regressed = session.observe({endpoint(gid, "")}, 2ms);

  EXPECT_TRUE(regressed.ready);
  EXPECT_EQ(regressed.reason, Reason::None);
  EXPECT_EQ(regressed.writer_gid, gid);
}

}  // namespace
}  // namespace voice_nav_mission
