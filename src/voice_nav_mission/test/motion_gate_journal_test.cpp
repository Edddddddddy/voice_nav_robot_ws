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

#include "gate_event_journal.hpp"
#include "voice_nav_mission/motion_gate_core.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace voice_nav_mission
{
namespace
{

constexpr char kGateId[] = "0123456789abcdef0123456789abcdef";

struct alignas(64) OneSlotRegion
{
  GateEventJournalHeader header{};
  GateEventJournalSlot slot{};
};

struct FakeClock
{
  std::array<std::uint64_t, 4U> values{100U, 200U, 300U, UINT64_MAX};
  std::size_t next{0U};

  static std::uint64_t read(void * context) noexcept
  {
    auto & clock = *static_cast<FakeClock *>(context);
    return clock.values.at(clock.next++);
  }
};

GateEventJournalIdentity initialize_region(OneSlotRegion & region)
{
  region = OneSlotRegion{};
  region.header.magic = VOICE_NAV_GATE_EVENT_JOURNAL_MAGIC;
  region.header.abi_version = VOICE_NAV_GATE_EVENT_JOURNAL_ABI_VERSION;
  region.header.header_bytes = sizeof(GateEventJournalHeader);
  region.header.slot_bytes = sizeof(GateEventJournalSlot);
  region.header.region_bytes = sizeof(region);
  region.header.capacity = 1U;
  region.header.owner_uid = 1000U;
  region.header.generation = 7U;
  region.header.nonce_hi = 0x123456789abcdef0U;
  region.header.nonce_lo = 0x0fedcba987654321U;
  region.header.header_checksum =
    gate_event_journal_header_checksum(region.header);
  gate_event_journal_store_release(
    region.header.init_state,
    VOICE_NAV_GATE_EVENT_JOURNAL_INIT_READY);
  return {
    region.header.owner_uid,
    region.header.generation,
    region.header.nonce_hi,
    region.header.nonce_lo};
}

TEST(MotionGateJournal, SuccessfulPrepareOwnsItsLinearizationFence)
{
  OneSlotRegion region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  MotionGateCore gate(MotionGateConfig{}, kGateId, 0U, &journal);

  const auto result = gate.prepare(
    ControlRequest{
      Operation::Prepare,
      "00000000000000000000000000000001",
      kGateId,
      0U,
      ""},
    MotionGateCore::SteadyTimePoint{} + std::chrono::milliseconds{10});

  ASSERT_EQ(result.code, ResultCode::Applied);
  ASSERT_EQ(result.state, State::Prepared);
  EXPECT_EQ(clock.next, 3U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    region.slot.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION);
  EXPECT_EQ(region.slot.journal_seq, 1U);
  EXPECT_EQ(region.slot.generation, identity.generation);
  EXPECT_EQ(region.slot.intent_monotonic_ns, 100U);
  EXPECT_EQ(region.slot.transition_linearization_ns, 200U);
  EXPECT_EQ(region.slot.commit_monotonic_ns, 300U);
  EXPECT_EQ(region.slot.event_code, 1U);  // PREPARE
  EXPECT_EQ(region.slot.reason, static_cast<std::uint64_t>(Reason::None));
  EXPECT_EQ(region.slot.before_state_seq, 0U);
  EXPECT_EQ(region.slot.before_control_seq, 0U);
  EXPECT_EQ(region.slot.after_state_seq, 1U);
  EXPECT_EQ(region.slot.after_control_seq, 1U);
  EXPECT_EQ(region.slot.before_lease_hi, 0U);
  EXPECT_EQ(region.slot.before_lease_lo, 0U);
  EXPECT_EQ(region.slot.after_lease_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(region.slot.after_lease_lo, UINT64_C(0x0123456789abcdee));
  EXPECT_EQ(region.slot.gate_instance_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(region.slot.gate_instance_lo, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(region.slot.flags, 0U);
  EXPECT_EQ(
    region.slot.intent_checksum,
    gate_event_journal_intent_checksum(region.slot));
  EXPECT_EQ(
    region.slot.commit_checksum,
    gate_event_journal_commit_checksum(region.slot));
}

}  // namespace
}  // namespace voice_nav_mission
