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

#include <array>
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
  std::string node_name,
  std::string node_namespace = "/")
{
  return WriterEndpointObservation{
    "geometry_msgs/msg/TwistStamped",
    std::move(node_name),
    std::move(node_namespace),
    RMW_ENDPOINT_PUBLISHER,
    candidate_qos(),
    gid};
}

void expect_complete_bounded_diagnostic(const OpenBinding & observation)
{
  EXPECT_LE(observation.detail.size(), 160U);
  constexpr std::array<const char *, 7U> markers{
    "n=1", " k=", " id=", " q=", " g=", " ms=", " t="};
  std::array<std::size_t, markers.size()> positions{};
  for (std::size_t index = 0U; index < markers.size(); ++index) {
    positions[index] = observation.detail.find(markers[index]);
    ASSERT_NE(positions[index], std::string::npos)
      << "missing diagnostic field " << markers[index]
      << " in: " << observation.detail;
    if (index > 0U) {
      ASSERT_LT(positions[index - 1U], positions[index])
        << "diagnostic fields are out of order in: " << observation.detail;
    }
  }
  EXPECT_NE(
    observation.detail.rfind("; ", positions.front()),
    std::string::npos);
  for (std::size_t index = 1U; index < markers.size(); ++index) {
    const auto value_start = positions[index] +
      std::char_traits<char>::length(markers[index]);
    const auto value_end = index + 1U < markers.size() ?
      positions[index + 1U] : observation.detail.size();
    EXPECT_LT(value_start, value_end)
      << "empty diagnostic field " << markers[index]
      << " in: " << observation.detail;
  }

  const auto gid_start = positions[4] + 3U;
  const auto gid = observation.detail.substr(
    gid_start, positions[5] - gid_start);
  EXPECT_EQ(gid.size(), 32U);
  EXPECT_EQ(
    gid.find_first_not_of("0123456789abcdefABCDEF"),
    std::string::npos);
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
  EXPECT_LE(pending.detail.size(), 160U);
  for (const auto * field : {"n=1", "t=", "id=", "q=", "g=", "ms=7"}) {
    EXPECT_NE(pending.detail.find(field), std::string::npos)
      << "missing diagnostic field " << field;
  }

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

TEST(WriterObservationSession, KnownWrongNamespaceCannotEnterPending)
{
  WriterObservationSession session({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  const auto gid = writer_gid(0x51U);

  const auto partial_mismatch = session.observe(
    {endpoint(gid, "", "/unexpected")}, 1ms);
  EXPECT_FALSE(partial_mismatch.ready);
  EXPECT_EQ(partial_mismatch.reason, Reason::WriterMismatch);

  const auto valid = session.observe(
    {endpoint(gid, "collision_monitor")}, 2ms);
  EXPECT_TRUE(valid.ready);
  EXPECT_EQ(valid.writer_gid, gid);
}

TEST(WriterObservationSession, ExactUnknownIdentityMarkersConvergeForPinnedGid)
{
  WriterObservationSession session({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  const auto gid = writer_gid(0x58U);

  const auto pending = session.observe(
      {
        endpoint(
        gid,
        "_NODE_NAME_UNKNOWN_",
        "_NODE_NAMESPACE_UNKNOWN_")},
    1ms);
  ASSERT_EQ(pending.reason, Reason::WriterMetadataPending);
  EXPECT_EQ(pending.writer_gid, gid);

  const auto ready = session.observe(
    {endpoint(gid, "collision_monitor")}, 2ms);
  EXPECT_TRUE(ready.ready);
  EXPECT_EQ(ready.writer_gid, gid);
}

TEST(WriterObservationSession, KnownPartialIdentityMustAgreeBeforePending)
{
  const auto gid = writer_gid(0x59U);
  WriterObservationSession compatible({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  EXPECT_EQ(
    compatible.observe(
      {
        endpoint(
          gid,
          "collision_monitor",
          "_NODE_NAMESPACE_UNKNOWN_")},
      1ms).reason,
    Reason::WriterMetadataPending);

  WriterObservationSession contradictory({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  EXPECT_EQ(
    contradictory.observe(
      {
        endpoint(
          gid,
          "unexpected_writer",
          "_NODE_NAMESPACE_UNKNOWN_")},
      1ms).reason,
    Reason::WriterMismatch);
}

TEST(WriterObservationSession, DefinitivePolicyViolationsNeverEnterPending)
{
  const auto valid_gid = writer_gid(0x61U);
  const auto assert_terminal_but_unpinned = [valid_gid](
    WriterEndpointObservation invalid)
    {
      WriterObservationSession session({
          "geometry_msgs/msg/TwistStamped",
          "/collision_monitor"});
      const auto rejected = session.observe({std::move(invalid)}, 1ms);
      EXPECT_FALSE(rejected.ready);
      EXPECT_EQ(rejected.reason, Reason::WriterMismatch);
      EXPECT_LE(rejected.detail.size(), 160U);

      const auto valid = session.observe(
        {endpoint(valid_gid, "collision_monitor")}, 2ms);
      EXPECT_TRUE(valid.ready);
      EXPECT_EQ(valid.writer_gid, valid_gid);
    };

  auto wrong_kind = endpoint(writer_gid(0x62U), "collision_monitor");
  wrong_kind.endpoint_type = RMW_ENDPOINT_SUBSCRIPTION;
  assert_terminal_but_unpinned(std::move(wrong_kind));

  auto wrong_type = endpoint(writer_gid(0x63U), "collision_monitor");
  wrong_type.topic_type = std::string(240U, 'x');
  assert_terminal_but_unpinned(std::move(wrong_type));

  auto wrong_qos = endpoint(writer_gid(0x64U), "collision_monitor");
  wrong_qos.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  assert_terminal_but_unpinned(std::move(wrong_qos));

  auto wrong_fqn = endpoint(writer_gid(0x65U), "unexpected_writer");
  assert_terminal_but_unpinned(std::move(wrong_fqn));

  auto zero_gid = endpoint({}, "collision_monitor");
  assert_terminal_but_unpinned(std::move(zero_gid));
}

TEST(WriterObservationSession, LongVariableFieldsPreserveEveryDiagnosticMarker)
{
  WriterObservationSession long_name_session({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  const auto long_name_rejected = long_name_session.observe(
    {endpoint(writer_gid(0x66U), std::string(240U, 'n'))}, 123456ms);
  EXPECT_FALSE(long_name_rejected.ready);
  EXPECT_EQ(long_name_rejected.reason, Reason::WriterMismatch);
  expect_complete_bounded_diagnostic(long_name_rejected);

  WriterObservationSession long_namespace_session({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  const auto long_namespace_rejected = long_namespace_session.observe(
      {
        endpoint(
          writer_gid(0x67U), "collision_monitor",
          "/" + std::string(240U, 's'))},
    123456ms);
  EXPECT_FALSE(long_namespace_rejected.ready);
  EXPECT_EQ(long_namespace_rejected.reason, Reason::WriterMismatch);
  expect_complete_bounded_diagnostic(long_namespace_rejected);

  WriterObservationSession long_type_session({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  const auto long_type_rejected = long_type_session.observe(
    {WriterEndpointObservation{
        std::string(240U, 't'),
        "collision_monitor",
        "/",
        RMW_ENDPOINT_PUBLISHER,
        candidate_qos(),
        writer_gid(0x68U)}},
    123456ms);
  EXPECT_FALSE(long_type_rejected.ready);
  EXPECT_EQ(long_type_rejected.reason, Reason::WriterMismatch);
  expect_complete_bounded_diagnostic(long_type_rejected);

  auto first_name = std::string(240U, 'p');
  auto second_name = first_name;
  first_name.back() = 'a';
  second_name.back() = 'b';
  WriterObservationSession first_session({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  WriterObservationSession second_session({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  const auto first = first_session.observe(
    {endpoint(writer_gid(0x69U), first_name)}, 123456ms);
  const auto second = second_session.observe(
    {endpoint(writer_gid(0x69U), second_name)}, 123456ms);
  expect_complete_bounded_diagnostic(first);
  expect_complete_bounded_diagnostic(second);
  EXPECT_NE(first.detail, second.detail)
    << "the compact value must digest bytes beyond the visible prefix";
}

TEST(
  WriterObservationSession,
  PinnedReplacementAndTerminalReplayPreserveEveryDiagnosticMarker)
{
  WriterObservationSession session({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  const auto pinned_gid = writer_gid(0x6aU);
  const auto replacement_gid = writer_gid(0x6bU);

  ASSERT_EQ(
    session.observe({endpoint(pinned_gid, "")}, 1ms).reason,
    Reason::WriterMetadataPending);
  const auto replacement = session.observe(
    {endpoint(replacement_gid, "collision_monitor")}, 123456ms);
  ASSERT_EQ(replacement.reason, Reason::WriterMismatch);
  expect_complete_bounded_diagnostic(replacement);

  const auto replayed = session.observe(
    {endpoint(pinned_gid, "collision_monitor")}, 123457ms);
  ASSERT_EQ(replayed.reason, Reason::WriterMismatch);
  expect_complete_bounded_diagnostic(replayed);
  EXPECT_EQ(replayed.detail, replacement.detail);
}

TEST(WriterObservationSession, MissingAndDuplicateWritersStayFailClosed)
{
  WriterObservationSession session({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  const auto first_gid = writer_gid(0x71U);

  const auto missing = session.observe({}, 1ms);
  EXPECT_EQ(missing.reason, Reason::WriterUnavailable);

  const auto duplicate = session.observe(
      {
        endpoint(first_gid, "collision_monitor"),
        endpoint(writer_gid(0x72U), "collision_monitor")},
    2ms);
  EXPECT_EQ(duplicate.reason, Reason::WriterAmbiguous);

  const auto valid = session.observe(
    {endpoint(first_gid, "collision_monitor")}, 3ms);
  EXPECT_TRUE(valid.ready);
  EXPECT_EQ(valid.writer_gid, first_gid);
}

TEST(WriterObservationSession, EmptySnapshotStaysUnavailableAfterPriorObservation)
{
  WriterObservationSession session({
        "geometry_msgs/msg/TwistStamped",
        "/collision_monitor"});
  const auto gid = writer_gid(0x73U);

  ASSERT_EQ(
    session.observe({endpoint(gid, "")}, 1ms).reason,
    Reason::WriterMetadataPending);

  const auto missing = session.observe({}, 2ms);
  EXPECT_FALSE(missing.ready);
  EXPECT_EQ(missing.reason, Reason::WriterUnavailable);
  EXPECT_NE(missing.detail.find("n=0"), std::string::npos);
}

}  // namespace
}  // namespace voice_nav_mission
