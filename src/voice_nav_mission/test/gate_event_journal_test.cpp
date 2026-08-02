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
#include <cstring>
#include <stdexcept>

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
  std::array<std::uint64_t, 3U> values{100U, 200U, UINT64_MAX};
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

GateOutputIntent make_output_intent()
{
  return {
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
}

void expect_invalid_attach_without_writer_claim(
  void * region,
  std::size_t region_bytes,
  const GateEventJournalIdentity & identity,
  GateEventJournalHeader * header)
{
  FakeClock clock;
  EXPECT_THROW(
      {
        GateEventJournal journal(
        region,
        region_bytes,
        identity,
        GateEventJournalClock{&FakeClock::read, &clock});
        (void)journal;
      },
    std::invalid_argument);
  if (header != nullptr) {
    EXPECT_EQ(gate_event_journal_load_acquire(header->writer_pid), 0U);
  }
}

template<typename Record, typename Checksum>
void expect_checksum_changes(
  const Record & baseline,
  std::uint64_t Record::* field,
  Checksum checksum)
{
  auto mutated = baseline;
  mutated.*field ^= UINT64_C(0x9e3779b97f4a7c15);
  EXPECT_NE(checksum(mutated), checksum(baseline));
}

template<typename Record, typename Checksum>
void expect_checksum_ignores(
  const Record & baseline,
  std::uint64_t Record::* field,
  Checksum checksum)
{
  auto mutated = baseline;
  mutated.*field ^= UINT64_C(0x9e3779b97f4a7c15);
  EXPECT_EQ(checksum(mutated), checksum(baseline));
}

TEST(
  GateEventJournal,
  SuccessfulOutputIsIntentDuringPublishAndCommittedAfterReturn)
{
  OneSlotRegion region;
  const auto identity = initialize_region(region);
  ASSERT_EQ(
    region.header.header_checksum,
    UINT64_C(0xc5a43daf135b254d));

  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});

  const auto intent = make_output_intent();
  std::uint64_t publisher_calls = 0U;

  const auto outcome = journal.publish_output(
    intent,
    [&region, &publisher_calls]() {
      ++publisher_calls;
      EXPECT_EQ(
        gate_event_journal_load_acquire(region.slot.phase),
        VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT);
      EXPECT_EQ(
        gate_event_journal_load_acquire(region.header.claimed_slots),
        1U);
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
    region.slot.intent_checksum,
    UINT64_C(0x1ab28ffd4e031df7));
  EXPECT_EQ(
    region.slot.commit_checksum,
    gate_event_journal_commit_checksum(region.slot));
  EXPECT_EQ(
    region.slot.commit_checksum,
    UINT64_C(0x24237a46d3cccb33));
}

TEST(GateEventJournal, PublisherFailureLeavesTrailingIntent)
{
  OneSlotRegion region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  const auto intent = make_output_intent();
  std::uint64_t publisher_calls = 0U;

  EXPECT_THROW(
    journal.publish_output(
      intent,
      [&region, &publisher_calls]() {
        ++publisher_calls;
        EXPECT_EQ(
          gate_event_journal_load_acquire(region.header.claimed_slots),
          1U);
        EXPECT_EQ(
          gate_event_journal_load_acquire(region.slot.phase),
          VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT);
        throw std::runtime_error("injected publisher failure");
      }),
    std::runtime_error);

  EXPECT_EQ(publisher_calls, 1U);
  EXPECT_EQ(clock.next, 1U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    1U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT);
  EXPECT_EQ(region.slot.intent_monotonic_ns, 100U);
  EXPECT_EQ(region.slot.commit_monotonic_ns, 0U);
  EXPECT_EQ(region.slot.commit_checksum, 0U);
  EXPECT_EQ(
    region.slot.intent_checksum,
    gate_event_journal_intent_checksum(region.slot));
}

TEST(
  GateEventJournal,
  CapacityExhaustionLatchesOverflowWithoutPublishingOrOverwrite)
{
  OneSlotRegion region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  const auto intent = make_output_intent();
  std::uint64_t first_publisher_calls = 0U;
  std::uint64_t rejected_publisher_calls = 0U;

  journal.publish_output(
    intent,
    [&first_publisher_calls]() {
      ++first_publisher_calls;
    });
  const auto committed_slot = region.slot;

  EXPECT_THROW(
    journal.publish_output(
      intent,
      [&rejected_publisher_calls]() {
        ++rejected_publisher_calls;
      }),
    std::overflow_error);

  EXPECT_EQ(first_publisher_calls, 1U);
  EXPECT_EQ(rejected_publisher_calls, 0U);
  EXPECT_EQ(clock.next, 2U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    1U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.overflow_latched),
    1U);
  EXPECT_EQ(
    std::memcmp(
      &region.slot,
      &committed_slot,
      sizeof(GateEventJournalSlot)),
    0);
}

TEST(GateEventJournal, RejectsNonFreshGenerationBeforeWriterClaim)
{
  {
    OneSlotRegion region;
    const auto identity = initialize_region(region);
    gate_event_journal_store_release(region.header.claimed_slots, 1U);
    FakeClock clock;

    EXPECT_THROW(
        {
          GateEventJournal journal(
          &region,
          sizeof(region),
          identity,
          GateEventJournalClock{&FakeClock::read, &clock});
          (void)journal;
        },
      std::invalid_argument);
    EXPECT_EQ(
      gate_event_journal_load_acquire(region.header.writer_pid),
      0U);
  }

  {
    OneSlotRegion region;
    const auto identity = initialize_region(region);
    gate_event_journal_store_release(region.header.overflow_latched, 1U);
    FakeClock clock;

    EXPECT_THROW(
        {
          GateEventJournal journal(
          &region,
          sizeof(region),
          identity,
          GateEventJournalClock{&FakeClock::read, &clock});
          (void)journal;
        },
      std::invalid_argument);
    EXPECT_EQ(
      gate_event_journal_load_acquire(region.header.writer_pid),
      0U);
  }
}

TEST(GateEventJournal, RejectsClaimlessOccupiedSlotBeforeWriterClaim)
{
  const std::array<std::uint64_t, 2U> occupied_phases{
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT,
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED};

  for (const auto phase : occupied_phases) {
    SCOPED_TRACE(phase);
    OneSlotRegion region;
    const auto identity = initialize_region(region);
    gate_event_journal_store_release(region.slot.phase, phase);
    FakeClock clock;

    EXPECT_THROW(
        {
          GateEventJournal journal(
          &region,
          sizeof(region),
          identity,
          GateEventJournalClock{&FakeClock::read, &clock});
          (void)journal;
        },
      std::invalid_argument);
    EXPECT_EQ(
      gate_event_journal_load_acquire(region.header.writer_pid),
      0U);
    EXPECT_EQ(
      gate_event_journal_load_acquire(region.header.claimed_slots),
      0U);
  }
}

TEST(GateEventJournal, RejectsMalformedOrMismatchedAttach)
{
  using AttachMutation = void (*)(
    OneSlotRegion &,
    GateEventJournalIdentity &);
  const std::array<AttachMutation, 14U> invalid_mutations{
    [](OneSlotRegion & region, GateEventJournalIdentity &) {
      gate_event_journal_store_release(
        region.header.init_state,
        VOICE_NAV_GATE_EVENT_JOURNAL_INIT_EMPTY);
    },
    [](OneSlotRegion & region, GateEventJournalIdentity &) {
      region.header.magic ^= 1U;
    },
    [](OneSlotRegion & region, GateEventJournalIdentity &) {
      region.header.abi_version += 1U;
    },
    [](OneSlotRegion & region, GateEventJournalIdentity &) {
      region.header.header_bytes += 8U;
    },
    [](OneSlotRegion & region, GateEventJournalIdentity &) {
      region.header.slot_bytes += 8U;
    },
    [](OneSlotRegion & region, GateEventJournalIdentity &) {
      region.header.region_bytes += 1U;
    },
    [](OneSlotRegion & region, GateEventJournalIdentity &) {
      region.header.capacity = 0U;
    },
    [](OneSlotRegion & region, GateEventJournalIdentity &) {
      region.header.capacity = 2U;
    },
    [](OneSlotRegion &, GateEventJournalIdentity & identity) {
      identity.owner_uid += 1U;
    },
    [](OneSlotRegion &, GateEventJournalIdentity & identity) {
      identity.generation += 1U;
    },
    [](OneSlotRegion &, GateEventJournalIdentity & identity) {
      identity.nonce_hi ^= 1U;
    },
    [](OneSlotRegion &, GateEventJournalIdentity & identity) {
      identity.nonce_lo ^= 1U;
    },
    [](OneSlotRegion & region, GateEventJournalIdentity &) {
      region.header.reserved = 1U;
    },
    [](OneSlotRegion & region, GateEventJournalIdentity &) {
      region.header.region_bytes = UINT64_MAX;
      region.header.capacity = UINT64_MAX;
    }};

  for (std::size_t index = 0U; index < invalid_mutations.size(); ++index) {
    SCOPED_TRACE(index);
    OneSlotRegion region;
    auto identity = initialize_region(region);
    invalid_mutations[index](region, identity);
    region.header.header_checksum =
      gate_event_journal_header_checksum(region.header);
    expect_invalid_attach_without_writer_claim(
      &region,
      sizeof(region),
      identity,
      &region.header);
  }

  {
    OneSlotRegion region;
    const auto identity = initialize_region(region);
    region.header.header_checksum ^= 1U;
    expect_invalid_attach_without_writer_claim(
      &region,
      sizeof(region),
      identity,
      &region.header);
  }

  {
    OneSlotRegion region;
    const auto identity = initialize_region(region);
    expect_invalid_attach_without_writer_claim(
      nullptr,
      sizeof(region),
      identity,
      nullptr);
    expect_invalid_attach_without_writer_claim(
      &region,
      sizeof(region) - 1U,
      identity,
      &region.header);
  }

  {
    alignas(64) std::array<std::byte, sizeof(OneSlotRegion) + 1U> bytes{};
    const GateEventJournalIdentity identity{};
    expect_invalid_attach_without_writer_claim(
      bytes.data() + 1U,
      sizeof(OneSlotRegion),
      identity,
      nullptr);
  }

  {
    OneSlotRegion region;
    const auto identity = initialize_region(region);
    gate_event_journal_store_release(region.header.writer_pid, 12345U);
    FakeClock clock;

    EXPECT_THROW(
        {
          GateEventJournal journal(
          &region,
          sizeof(region),
          identity,
          GateEventJournalClock{&FakeClock::read, &clock});
          (void)journal;
        },
      std::runtime_error);
    EXPECT_EQ(
      gate_event_journal_load_acquire(region.header.writer_pid),
      12345U);
  }
}

TEST(
  GateEventJournal,
  TransitionIntentPrecedesMutationAndCommitCapturesAfterImage)
{
  OneSlotRegion region;
  const auto identity = initialize_region(region);
  FakeClock clock{{100U, 150U, 200U}, 0U};
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  const GateTransitionIntent intent{
    5U,
    8U,
    10U,
    20U,
    0x1111222233334444U,
    0x5555666677778888U,
    0x9999aaaabbbbccccU,
    0xddddeeeeffff0000U,
    0x5aU};
  std::uint64_t transition_calls = 0U;
  auto transition_binding = journal.claim_transition_binding();

  const auto outcome = transition_binding->apply_transition(
    intent,
    [&region, &transition_calls]() {
      ++transition_calls;
      EXPECT_EQ(
        gate_event_journal_load_acquire(region.header.claimed_slots),
        1U);
      EXPECT_EQ(
        gate_event_journal_load_acquire(region.slot.phase),
        VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT);
      EXPECT_EQ(region.slot.intent_monotonic_ns, 100U);
      EXPECT_EQ(region.slot.transition_linearization_ns, 150U);
      return GateTransitionAfter{11U, 21U, 0U, 0U};
    });

  EXPECT_EQ(transition_calls, 1U);
  EXPECT_EQ(clock.next, 3U);
  EXPECT_EQ(outcome.journal_seq, 1U);
  EXPECT_EQ(outcome.slot_index, 0U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    region.slot.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION);
  EXPECT_EQ(region.slot.journal_seq, 1U);
  EXPECT_EQ(region.slot.generation, region.header.generation);
  EXPECT_EQ(region.slot.intent_monotonic_ns, 100U);
  EXPECT_EQ(region.slot.transition_linearization_ns, 150U);
  EXPECT_EQ(region.slot.commit_monotonic_ns, 200U);
  EXPECT_EQ(region.slot.event_code, intent.event_code);
  EXPECT_EQ(region.slot.reason, intent.reason);
  EXPECT_EQ(region.slot.before_state_seq, intent.before_state_seq);
  EXPECT_EQ(region.slot.after_state_seq, 11U);
  EXPECT_EQ(region.slot.before_control_seq, intent.before_control_seq);
  EXPECT_EQ(region.slot.after_control_seq, 21U);
  EXPECT_EQ(region.slot.before_lease_hi, intent.before_lease_hi);
  EXPECT_EQ(region.slot.before_lease_lo, intent.before_lease_lo);
  EXPECT_EQ(region.slot.after_lease_hi, 0U);
  EXPECT_EQ(region.slot.after_lease_lo, 0U);
  EXPECT_EQ(region.slot.gate_instance_hi, intent.gate_instance_hi);
  EXPECT_EQ(region.slot.gate_instance_lo, intent.gate_instance_lo);
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

TEST(GateEventJournal, TransitionFailureLeavesLinearizedIntent)
{
  OneSlotRegion region;
  const auto identity = initialize_region(region);
  FakeClock clock{{100U, 150U, 200U}, 0U};
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  const GateTransitionIntent intent{
    6U,
    9U,
    30U,
    40U,
    0x1111222233334444U,
    0x5555666677778888U,
    0x9999aaaabbbbccccU,
    0xddddeeeeffff0000U,
    0xa5U};
  std::uint64_t transition_calls = 0U;
  auto transition_binding = journal.claim_transition_binding();

  EXPECT_THROW(
    transition_binding->apply_transition(
      intent,
      [&region, &transition_calls]() -> GateTransitionAfter {
        ++transition_calls;
        EXPECT_EQ(
          gate_event_journal_load_acquire(region.slot.phase),
          VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT);
        EXPECT_EQ(region.slot.transition_linearization_ns, 150U);
        throw std::runtime_error("injected transition failure");
      }),
    std::runtime_error);

  EXPECT_EQ(transition_calls, 1U);
  EXPECT_EQ(clock.next, 2U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    1U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT);
  EXPECT_EQ(region.slot.intent_monotonic_ns, 100U);
  EXPECT_EQ(region.slot.transition_linearization_ns, 150U);
  EXPECT_EQ(region.slot.commit_monotonic_ns, 0U);
  EXPECT_EQ(region.slot.after_state_seq, 0U);
  EXPECT_EQ(region.slot.after_control_seq, 0U);
  EXPECT_EQ(region.slot.commit_checksum, 0U);
  EXPECT_EQ(
    region.slot.intent_checksum,
    gate_event_journal_intent_checksum(region.slot));
}

TEST(GateEventJournal, ChecksumCoverageMatchesAbiV1)
{
  using HeaderField = std::uint64_t GateEventJournalHeader::*;
  const std::array<HeaderField, 11U> header_included{
    &GateEventJournalHeader::magic,
    &GateEventJournalHeader::abi_version,
    &GateEventJournalHeader::header_bytes,
    &GateEventJournalHeader::slot_bytes,
    &GateEventJournalHeader::region_bytes,
    &GateEventJournalHeader::capacity,
    &GateEventJournalHeader::owner_uid,
    &GateEventJournalHeader::generation,
    &GateEventJournalHeader::nonce_hi,
    &GateEventJournalHeader::nonce_lo,
    &GateEventJournalHeader::reserved};
  const std::array<HeaderField, 5U> header_excluded{
    &GateEventJournalHeader::init_state,
    &GateEventJournalHeader::claimed_slots,
    &GateEventJournalHeader::overflow_latched,
    &GateEventJournalHeader::writer_pid,
    &GateEventJournalHeader::header_checksum};
  const GateEventJournalHeader header{};

  for (const auto field : header_included) {
    expect_checksum_changes(
      header,
      field,
      &gate_event_journal_header_checksum);
  }
  for (const auto field : header_excluded) {
    expect_checksum_ignores(
      header,
      field,
      &gate_event_journal_header_checksum);
  }

  using SlotField = std::uint64_t GateEventJournalSlot::*;
  const std::array<SlotField, 20U> intent_included{
    &GateEventJournalSlot::record_kind,
    &GateEventJournalSlot::journal_seq,
    &GateEventJournalSlot::generation,
    &GateEventJournalSlot::intent_monotonic_ns,
    &GateEventJournalSlot::event_code,
    &GateEventJournalSlot::reason,
    &GateEventJournalSlot::before_state_seq,
    &GateEventJournalSlot::before_control_seq,
    &GateEventJournalSlot::output_attempt_seq,
    &GateEventJournalSlot::intended_output_seq,
    &GateEventJournalSlot::ros_stamp_sec_bits,
    &GateEventJournalSlot::ros_stamp_nanosec,
    &GateEventJournalSlot::linear_x_bits,
    &GateEventJournalSlot::angular_z_bits,
    &GateEventJournalSlot::before_lease_hi,
    &GateEventJournalSlot::before_lease_lo,
    &GateEventJournalSlot::gate_instance_hi,
    &GateEventJournalSlot::gate_instance_lo,
    &GateEventJournalSlot::cause_transition_journal_seq,
    &GateEventJournalSlot::flags};
  const std::array<SlotField, 12U> intent_excluded{
    &GateEventJournalSlot::phase,
    &GateEventJournalSlot::transition_linearization_ns,
    &GateEventJournalSlot::commit_monotonic_ns,
    &GateEventJournalSlot::intent_checksum,
    &GateEventJournalSlot::commit_checksum,
    &GateEventJournalSlot::after_state_seq,
    &GateEventJournalSlot::after_control_seq,
    &GateEventJournalSlot::after_lease_hi,
    &GateEventJournalSlot::after_lease_lo,
    &GateEventJournalSlot::reserved0,
    &GateEventJournalSlot::reserved1,
    &GateEventJournalSlot::reserved2};
  const std::array<SlotField, 27U> commit_included{
    &GateEventJournalSlot::intent_checksum,
    &GateEventJournalSlot::record_kind,
    &GateEventJournalSlot::journal_seq,
    &GateEventJournalSlot::generation,
    &GateEventJournalSlot::intent_monotonic_ns,
    &GateEventJournalSlot::event_code,
    &GateEventJournalSlot::reason,
    &GateEventJournalSlot::before_state_seq,
    &GateEventJournalSlot::before_control_seq,
    &GateEventJournalSlot::output_attempt_seq,
    &GateEventJournalSlot::intended_output_seq,
    &GateEventJournalSlot::ros_stamp_sec_bits,
    &GateEventJournalSlot::ros_stamp_nanosec,
    &GateEventJournalSlot::linear_x_bits,
    &GateEventJournalSlot::angular_z_bits,
    &GateEventJournalSlot::before_lease_hi,
    &GateEventJournalSlot::before_lease_lo,
    &GateEventJournalSlot::gate_instance_hi,
    &GateEventJournalSlot::gate_instance_lo,
    &GateEventJournalSlot::cause_transition_journal_seq,
    &GateEventJournalSlot::flags,
    &GateEventJournalSlot::transition_linearization_ns,
    &GateEventJournalSlot::commit_monotonic_ns,
    &GateEventJournalSlot::after_state_seq,
    &GateEventJournalSlot::after_control_seq,
    &GateEventJournalSlot::after_lease_hi,
    &GateEventJournalSlot::after_lease_lo};
  const std::array<SlotField, 5U> commit_excluded{
    &GateEventJournalSlot::phase,
    &GateEventJournalSlot::commit_checksum,
    &GateEventJournalSlot::reserved0,
    &GateEventJournalSlot::reserved1,
    &GateEventJournalSlot::reserved2};
  const GateEventJournalSlot slot{};

  for (const auto field : intent_included) {
    expect_checksum_changes(
      slot,
      field,
      &gate_event_journal_intent_checksum);
  }
  for (const auto field : intent_excluded) {
    expect_checksum_ignores(
      slot,
      field,
      &gate_event_journal_intent_checksum);
  }
  for (const auto field : commit_included) {
    expect_checksum_changes(
      slot,
      field,
      &gate_event_journal_commit_checksum);
  }
  for (const auto field : commit_excluded) {
    expect_checksum_ignores(
      slot,
      field,
      &gate_event_journal_commit_checksum);
  }
}

}  // namespace
}  // namespace voice_nav_mission
