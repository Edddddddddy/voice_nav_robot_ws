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

#include <unistd.h>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

namespace voice_nav_mission
{
namespace
{

static_assert(offsetof(GateEventJournalHeader, magic) == 0U);
static_assert(offsetof(GateEventJournalHeader, abi_version) == 8U);
static_assert(offsetof(GateEventJournalHeader, header_bytes) == 16U);
static_assert(offsetof(GateEventJournalHeader, slot_bytes) == 24U);
static_assert(offsetof(GateEventJournalHeader, region_bytes) == 32U);
static_assert(offsetof(GateEventJournalHeader, capacity) == 40U);
static_assert(offsetof(GateEventJournalHeader, owner_uid) == 48U);
static_assert(offsetof(GateEventJournalHeader, generation) == 56U);
static_assert(offsetof(GateEventJournalHeader, nonce_hi) == 64U);
static_assert(offsetof(GateEventJournalHeader, nonce_lo) == 72U);
static_assert(offsetof(GateEventJournalHeader, init_state) == 80U);
static_assert(offsetof(GateEventJournalHeader, claimed_slots) == 88U);
static_assert(offsetof(GateEventJournalHeader, overflow_latched) == 96U);
static_assert(offsetof(GateEventJournalHeader, writer_pid) == 104U);
static_assert(offsetof(GateEventJournalHeader, header_checksum) == 112U);
static_assert(offsetof(GateEventJournalHeader, reserved) == 120U);

static_assert(offsetof(GateEventJournalSlot, phase) == 0U);
static_assert(offsetof(GateEventJournalSlot, record_kind) == 8U);
static_assert(offsetof(GateEventJournalSlot, journal_seq) == 16U);
static_assert(offsetof(GateEventJournalSlot, generation) == 24U);
static_assert(offsetof(GateEventJournalSlot, intent_monotonic_ns) == 32U);
static_assert(offsetof(GateEventJournalSlot, transition_linearization_ns) == 40U);
static_assert(offsetof(GateEventJournalSlot, commit_monotonic_ns) == 48U);
static_assert(offsetof(GateEventJournalSlot, intent_checksum) == 56U);
static_assert(offsetof(GateEventJournalSlot, commit_checksum) == 64U);
static_assert(offsetof(GateEventJournalSlot, event_code) == 72U);
static_assert(offsetof(GateEventJournalSlot, reason) == 80U);
static_assert(offsetof(GateEventJournalSlot, before_state_seq) == 88U);
static_assert(offsetof(GateEventJournalSlot, after_state_seq) == 96U);
static_assert(offsetof(GateEventJournalSlot, before_control_seq) == 104U);
static_assert(offsetof(GateEventJournalSlot, after_control_seq) == 112U);
static_assert(offsetof(GateEventJournalSlot, output_attempt_seq) == 120U);
static_assert(offsetof(GateEventJournalSlot, intended_output_seq) == 128U);
static_assert(offsetof(GateEventJournalSlot, ros_stamp_sec_bits) == 136U);
static_assert(offsetof(GateEventJournalSlot, ros_stamp_nanosec) == 144U);
static_assert(offsetof(GateEventJournalSlot, linear_x_bits) == 152U);
static_assert(offsetof(GateEventJournalSlot, angular_z_bits) == 160U);
static_assert(offsetof(GateEventJournalSlot, before_lease_hi) == 168U);
static_assert(offsetof(GateEventJournalSlot, before_lease_lo) == 176U);
static_assert(offsetof(GateEventJournalSlot, after_lease_hi) == 184U);
static_assert(offsetof(GateEventJournalSlot, after_lease_lo) == 192U);
static_assert(offsetof(GateEventJournalSlot, gate_instance_hi) == 200U);
static_assert(offsetof(GateEventJournalSlot, gate_instance_lo) == 208U);
static_assert(offsetof(GateEventJournalSlot, cause_transition_journal_seq) == 216U);
static_assert(offsetof(GateEventJournalSlot, flags) == 224U);
static_assert(offsetof(GateEventJournalSlot, reserved0) == 232U);
static_assert(offsetof(GateEventJournalSlot, reserved1) == 240U);
static_assert(offsetof(GateEventJournalSlot, reserved2) == 248U);

struct alignas(64) OneSlotRegion
{
  GateEventJournalHeader header{};
  GateEventJournalSlot slot{};
};

static_assert(sizeof(OneSlotRegion) == 384U);
static_assert(offsetof(OneSlotRegion, slot) == 128U);

struct FakeClock
{
  std::array<std::uint64_t, 2U> values{100U, 200U};
  std::size_t next{0U};

  static std::uint64_t read(void * context) noexcept
  {
    auto & clock = *static_cast<FakeClock *>(context);
    if (clock.next >= clock.values.size()) {
      return UINT64_MAX;
    }
    return clock.values[clock.next++];
  }
};

TEST(
  GateEventJournal,
  SuccessfulOutputIsIntentDuringPublishAndCommittedAfterReturn)
{
  OneSlotRegion region;
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

  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    GateEventJournalIdentity{
        region.header.owner_uid,
        region.header.generation,
        region.header.nonce_hi,
        region.header.nonce_lo},
    GateEventJournalClock{&FakeClock::read, &clock});

  const GateOutputIntent intent{
    41U,
    9U,
    17U,
    18U,
    23U,
    456789123U,
    0x3fd0000000000000U,
    0xbfc0000000000000U,
    0x1111222233334444U,
    0x5555666677778888U,
    3U,
    0xa5U};
  std::uint64_t publisher_calls = 0U;

  const auto outcome = journal.publish_output(
    intent,
    [&region, &publisher_calls]() {
      ++publisher_calls;
      EXPECT_EQ(
        gate_event_journal_load_acquire(region.slot.phase),
        VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT);
    });

  EXPECT_EQ(publisher_calls, 1U);
  EXPECT_EQ(clock.next, 2U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.writer_pid),
    static_cast<std::uint64_t>(getpid()));
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    1U);
  EXPECT_EQ(outcome.journal_seq, 1U);
  EXPECT_EQ(outcome.slot_index, 0U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(region.slot.record_kind, VOICE_NAV_GATE_EVENT_JOURNAL_KIND_OUTPUT_ATTEMPT);
  EXPECT_EQ(region.slot.journal_seq, 1U);
  EXPECT_EQ(region.slot.generation, region.header.generation);
  EXPECT_EQ(region.slot.intent_monotonic_ns, 100U);
  EXPECT_EQ(region.slot.transition_linearization_ns, 0U);
  EXPECT_EQ(region.slot.commit_monotonic_ns, 200U);
  EXPECT_EQ(region.slot.event_code, intent.event_code);
  EXPECT_EQ(region.slot.reason, intent.reason);
  EXPECT_EQ(region.slot.output_attempt_seq, intent.output_attempt_seq);
  EXPECT_EQ(region.slot.intended_output_seq, intent.intended_output_seq);
  EXPECT_EQ(region.slot.ros_stamp_sec_bits, intent.ros_stamp_sec_bits);
  EXPECT_EQ(region.slot.ros_stamp_nanosec, intent.ros_stamp_nanosec);
  EXPECT_EQ(region.slot.linear_x_bits, intent.linear_x_bits);
  EXPECT_EQ(region.slot.angular_z_bits, intent.angular_z_bits);
  EXPECT_EQ(region.slot.gate_instance_hi, intent.gate_instance_hi);
  EXPECT_EQ(region.slot.gate_instance_lo, intent.gate_instance_lo);
  EXPECT_EQ(
    region.slot.cause_transition_journal_seq,
    intent.cause_transition_journal_seq);
  EXPECT_EQ(region.slot.flags, intent.flags);
  EXPECT_NE(region.slot.intent_checksum, 0U);
  EXPECT_NE(region.slot.commit_checksum, 0U);
  EXPECT_EQ(
    region.slot.intent_checksum,
    gate_event_journal_intent_checksum(region.slot));
  EXPECT_EQ(
    region.slot.commit_checksum,
    gate_event_journal_commit_checksum(region.slot));
}

}  // namespace
}  // namespace voice_nav_mission
