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

#include "voice_nav_mission/action_admission_tracker.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;

class ManualSteadyClock final
{
public:
  ActionAdmissionTracker::TimePoint now() const
  {
    return now_;
  }

  void advance(const std::chrono::milliseconds amount)
  {
    now_ += amount;
  }

private:
  ActionAdmissionTracker::TimePoint now_{};
};

TEST(ActionAdmissionTrackerTest, NormalHandoffCompletesExactlyOnce)
{
  ManualSteadyClock clock;
  ActionAdmissionTracker tracker(
    [&clock]() {return clock.now();}, 100ms);

  ASSERT_TRUE(tracker.try_provision("normal-goal"));
  EXPECT_EQ(tracker.snapshot().provisional, 1U);
  {
    auto lease = tracker.enter_accepted("normal-goal");
    ASSERT_TRUE(lease.has_ticket());
    EXPECT_FALSE(lease.was_revoked());
    EXPECT_EQ(tracker.snapshot().in_flight, 1U);
    EXPECT_EQ(tracker.snapshot().callbacks_inflight, 1U);
  }

  const auto snapshot = tracker.snapshot();
  EXPECT_EQ(snapshot.provisional, 0U);
  EXPECT_EQ(snapshot.in_flight, 0U);
  EXPECT_EQ(snapshot.callbacks_inflight, 0U);
  EXPECT_TRUE(tracker.drained());
}

TEST(ActionAdmissionTrackerTest, ExpiredUnacceptedTicketIsRevokedWithoutResult)
{
  ManualSteadyClock clock;
  ActionAdmissionTracker tracker(
    [&clock]() {return clock.now();}, 100ms);

  ASSERT_TRUE(tracker.try_provision("response-timeout"));
  clock.advance(101ms);

  EXPECT_EQ(tracker.revoke_expired(clock.now()), 1U);
  const auto snapshot = tracker.snapshot();
  EXPECT_EQ(snapshot.provisional, 0U);
  EXPECT_EQ(snapshot.in_flight, 0U);
  EXPECT_EQ(snapshot.revoked, 1U);
  EXPECT_TRUE(tracker.drained());
  // No on_accepted lease exists, so this path has no application Result.
  EXPECT_EQ(snapshot.callbacks_inflight, 0U);
}

TEST(ActionAdmissionTrackerTest, QuiesceDoesNotLoseTheCallbackGap)
{
  ManualSteadyClock clock;
  ActionAdmissionTracker tracker(
    [&clock]() {return clock.now();}, 100ms);

  ASSERT_TRUE(tracker.try_provision("callback-gap"));
  tracker.begin_quiesce();
  auto lease = tracker.enter_accepted("callback-gap");

  ASSERT_TRUE(lease.has_ticket());
  EXPECT_FALSE(lease.was_revoked());
  EXPECT_TRUE(tracker.snapshot().quiescing);
  EXPECT_EQ(tracker.snapshot().provisional, 0U);
  EXPECT_EQ(tracker.snapshot().in_flight, 1U);
  lease = {};
  EXPECT_TRUE(tracker.drained());
}

TEST(ActionAdmissionTrackerTest, LateCallbackGetsOneRevokedTicket)
{
  ManualSteadyClock clock;
  ActionAdmissionTracker tracker(
    [&clock]() {return clock.now();}, 100ms);

  ASSERT_TRUE(tracker.try_provision("late-callback"));
  clock.advance(101ms);
  ASSERT_EQ(tracker.revoke_expired(clock.now()), 1U);

  auto lease = tracker.enter_accepted("late-callback");
  ASSERT_TRUE(lease.has_ticket());
  EXPECT_TRUE(lease.was_revoked());
  EXPECT_EQ(tracker.snapshot().in_flight, 1U);
  EXPECT_EQ(tracker.snapshot().revoked, 0U);
  lease = {};
  EXPECT_EQ(tracker.snapshot().in_flight, 0U);
  EXPECT_TRUE(tracker.drained());
}

TEST(ActionAdmissionTrackerTest, QuiesceRevokesProvisionalTicketsAtBound)
{
  ManualSteadyClock clock;
  ActionAdmissionTracker tracker(
    [&clock]() {return clock.now();}, 100ms);

  ASSERT_TRUE(tracker.try_provision("shutdown-bound"));
  tracker.begin_quiesce();
  EXPECT_EQ(tracker.revoke_all_provisional(clock.now()), 1U);
  EXPECT_EQ(tracker.snapshot().provisional, 0U);
  EXPECT_EQ(tracker.snapshot().in_flight, 0U);
  EXPECT_TRUE(tracker.drained());
}

}  // namespace
}  // namespace voice_nav_mission
