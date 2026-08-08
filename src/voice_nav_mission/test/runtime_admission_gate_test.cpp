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

#include "voice_nav_mission/runtime_admission_gate.hpp"

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

private:
  ActionAdmissionTracker::TimePoint now_{};
};

TEST(RuntimeAdmissionGateTest, CallbackGapIsRejectedByTheQuiesceLinearization)
{
  ManualSteadyClock clock;
  ActionAdmissionTracker tracker([&clock]() {return clock.now();});
  RuntimeAdmissionGate gate;

  ASSERT_TRUE(gate.try_provision(tracker, "callback-gap", 4U, {}));
  gate.begin_quiesce(tracker);

  bool enqueued = false;
  EXPECT_FALSE(gate.submit([&enqueued]() {
      enqueued = true;
      return true;
  }));
  EXPECT_FALSE(enqueued);
  EXPECT_TRUE(tracker.snapshot().quiescing);
}

TEST(RuntimeAdmissionGateTest, QueuedAdmissionCannotClaimStartAfterQuiesce)
{
  ManualSteadyClock clock;
  ActionAdmissionTracker tracker([&clock]() {return clock.now();});
  RuntimeAdmissionGate gate;

  ASSERT_TRUE(gate.try_provision(tracker, "queued-before-dispatch", 7U, {}));
  bool queued = false;
  ASSERT_TRUE(gate.submit([&queued]() {
      queued = true;
      return true;
  }));
  ASSERT_TRUE(queued);

  gate.begin_quiesce(tracker);
  EXPECT_FALSE(gate.claim_start(7U).issued);
}

TEST(RuntimeAdmissionGateTest, ClaimedPermitIsGenerationFenced)
{
  ManualSteadyClock clock;
  ActionAdmissionTracker tracker([&clock]() {return clock.now();});
  RuntimeAdmissionGate gate;

  const auto permit = gate.claim_start(9U);
  ASSERT_TRUE(permit.issued);
  EXPECT_TRUE(gate.start_allowed(permit));
  gate.begin_quiesce(tracker);
  EXPECT_FALSE(gate.start_allowed(permit));
}

TEST(RuntimeAdmissionGateTest, AdmissionCheckRunsAtTheLinearizationPoint)
{
  ManualSteadyClock clock;
  ActionAdmissionTracker tracker([&clock]() {return clock.now();});
  RuntimeAdmissionGate gate;
  bool allowed = true;

  ASSERT_TRUE(gate.try_provision(
    tracker, "fenced", 12U, [&allowed](const std::uint64_t) {return allowed;}));
  allowed = false;
  EXPECT_FALSE(gate.admission_allowed(
    12U, [&allowed](const std::uint64_t) {return allowed;}));
}

}  // namespace
}  // namespace voice_nav_mission
