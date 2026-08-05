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

#include "voice_nav_mission/mission_authority_convergence.hpp"

#include <gtest/gtest.h>

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
using Kind = AuthorityOperationKind;
using TimePoint = MissionAuthorityConvergence::TimePoint;

constexpr char kInitialGate[] = "gate-initial";
constexpr char kLatestGate[] = "gate-latest";
constexpr char kInitialLease[] = "lease-initial";
constexpr char kLatestLease[] = "lease-latest";

struct ScriptedClock
{
  TimePoint value{};

  [[nodiscard]] TimePoint now() const {return value;}
  void advance(std::chrono::milliseconds amount) {value += amount;}
};

AuthorityOperation operation(Kind kind)
{
  return AuthorityOperation{
    "logical-request-id", kInitialGate, 7U,
    kind == Kind::Prepare ? std::string{} : kInitialLease};
}

AuthorityResult response(
  bool applied,
  bool retryable,
  const char * gate_id = kInitialGate,
  std::uint64_t control_seq = 7U,
  const char * lease_id = kInitialLease,
  bool zero_proven = true,
  bool tuple_stale = false)
{
  GateSnapshot snapshot;
  snapshot.gate_instance_id = gate_id;
  snapshot.control_seq = control_seq;
  snapshot.lease_id = lease_id;
  snapshot.endpoint_available = true;
  snapshot.motion_inhibited = zero_proven;
  snapshot.zero_published = zero_proven;
  return AuthorityResult{
    applied, zero_proven, retryable, snapshot, lease_id, "scripted", tuple_stale};
}

TEST(MissionAuthorityConvergence, EveryOperationRebuildsStaleTuple)
{
  const std::array<Kind, 4> kinds{
    Kind::Prepare, Kind::Open, Kind::Renew, Kind::Inhibit};
  for (const auto kind : kinds) {
    ScriptedClock clock;
    const auto initial = operation(kind);
    std::vector<AuthorityOperation> calls;
    const auto result = MissionAuthorityConvergence::run(
      initial,
      kind,
      250ms,
      100ms,
      [&clock]() {return clock.now();},
      [&calls](
        const AuthorityOperation & current, TimePoint, TimePoint) {
        calls.push_back(current);
        if (calls.size() == 1U) {
          return response(
              false, true, kLatestGate, 8U, kLatestLease,
              current.lease_id.empty(), true);
        }
        return response(true, false, kLatestGate, 9U, kLatestLease);
        });

    ASSERT_TRUE(result.applied);
    ASSERT_EQ(calls.size(), 2U);
    EXPECT_EQ(calls[0].request_id, calls[1].request_id);
    EXPECT_EQ(calls[1].gate_instance_id, kLatestGate);
    EXPECT_EQ(calls[1].expected_control_seq, 8U);
    if (kind == Kind::Prepare) {
      EXPECT_TRUE(calls[1].lease_id.empty());
    } else {
      EXPECT_EQ(calls[1].lease_id, kLatestLease);
    }
  }
}

TEST(MissionAuthorityConvergence, TimeoutRetriesSamePayloadToApplied)
{
  const std::array<Kind, 4> kinds{
    Kind::Prepare, Kind::Open, Kind::Renew, Kind::Inhibit};
  for (const auto kind : kinds) {
    ScriptedClock clock;
    std::vector<AuthorityOperation> calls;
    const auto result = MissionAuthorityConvergence::run(
      operation(kind),
      kind,
      250ms,
      100ms,
      [&clock]() {return clock.now();},
      [&calls, &clock](
        const AuthorityOperation & current, TimePoint, TimePoint) {
        calls.push_back(current);
        if (calls.size() == 1U) {
          clock.advance(100ms);
          return response(false, true, kLatestGate, 8U, kLatestLease, true, false);
        }
        return response(true, false);
        });

    ASSERT_TRUE(result.applied);
    ASSERT_EQ(calls.size(), 2U);
    EXPECT_EQ(calls[0].request_id, calls[1].request_id);
    EXPECT_EQ(calls[0].gate_instance_id, calls[1].gate_instance_id);
    EXPECT_EQ(calls[0].expected_control_seq, calls[1].expected_control_seq);
    EXPECT_EQ(calls[0].lease_id, calls[1].lease_id);
  }
}

TEST(MissionAuthorityConvergence, DeadlineStopsRetryBeforeAnOutOfBudgetCall)
{
  ScriptedClock clock;
  std::size_t attempts = 0U;
  TimePoint observed_attempt_deadline{};
  const auto result = MissionAuthorityConvergence::run(
    operation(Kind::Renew),
    Kind::Renew,
    250ms,
    100ms,
    [&clock]() {return clock.now();},
    [&clock, &attempts, &observed_attempt_deadline](
      const AuthorityOperation &, TimePoint attempt_deadline, TimePoint) {
      ++attempts;
      observed_attempt_deadline = attempt_deadline;
      clock.advance(250ms);
      return response(false, true);
      });

  EXPECT_FALSE(result.applied);
  EXPECT_TRUE(result.retryable);
  EXPECT_EQ(attempts, 1U);
  EXPECT_EQ(observed_attempt_deadline, TimePoint{} + 100ms);
}

}  // namespace
}  // namespace voice_nav_mission
