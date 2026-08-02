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

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <type_traits>

#include "gate_event_journal.hpp"
#include "voice_nav_mission/motion_gate_core.hpp"

namespace voice_nav_mission
{
namespace
{

static_assert(!std::is_copy_constructible_v<MotionGateCore>);
static_assert(!std::is_copy_assignable_v<MotionGateCore>);
static_assert(!std::is_move_constructible_v<MotionGateCore>);
static_assert(!std::is_move_assignable_v<MotionGateCore>);

constexpr char kGateId[] = "0123456789abcdef0123456789abcdef";

template<std::size_t Capacity>
struct alignas(64) JournalRegion
{
  GateEventJournalHeader header{};
  std::array<GateEventJournalSlot, Capacity> slots{};
};

struct FakeClock
{
  std::uint64_t next_value{100U};
  std::size_t samples{0U};

  static std::uint64_t read(void * context) noexcept
  {
    auto & clock = *static_cast<FakeClock *>(context);
    const auto value = clock.next_value;
    clock.next_value += 100U;
    ++clock.samples;
    return value;
  }
};

template<std::size_t Capacity>
GateEventJournalIdentity initialize_region(JournalRegion<Capacity> & region)
{
  region = JournalRegion<Capacity>{};
  region.header.magic = VOICE_NAV_GATE_EVENT_JOURNAL_MAGIC;
  region.header.abi_version = VOICE_NAV_GATE_EVENT_JOURNAL_ABI_VERSION;
  region.header.header_bytes = sizeof(GateEventJournalHeader);
  region.header.slot_bytes = sizeof(GateEventJournalSlot);
  region.header.region_bytes = sizeof(region);
  region.header.capacity = Capacity;
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
  JournalRegion<1U> region;
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
  EXPECT_EQ(clock.samples, 3U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.slots.front().phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  const auto & slot = region.slots.front();
  EXPECT_EQ(
    slot.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION);
  EXPECT_EQ(slot.journal_seq, 1U);
  EXPECT_EQ(slot.generation, identity.generation);
  EXPECT_EQ(slot.intent_monotonic_ns, 100U);
  EXPECT_EQ(slot.transition_linearization_ns, 200U);
  EXPECT_EQ(slot.commit_monotonic_ns, 300U);
  EXPECT_EQ(slot.event_code, 1U);  // PREPARE
  EXPECT_EQ(slot.reason, static_cast<std::uint64_t>(Reason::None));
  EXPECT_EQ(slot.before_state_seq, 0U);
  EXPECT_EQ(slot.before_control_seq, 0U);
  EXPECT_EQ(slot.after_state_seq, 1U);
  EXPECT_EQ(slot.after_control_seq, 1U);
  EXPECT_EQ(slot.before_lease_hi, 0U);
  EXPECT_EQ(slot.before_lease_lo, 0U);
  EXPECT_EQ(slot.after_lease_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.after_lease_lo, UINT64_C(0x0123456789abcdee));
  EXPECT_EQ(slot.gate_instance_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.gate_instance_lo, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.flags, 0U);
  EXPECT_EQ(
    slot.intent_checksum,
    gate_event_journal_intent_checksum(slot));
  EXPECT_EQ(
    slot.commit_checksum,
    gate_event_journal_commit_checksum(slot));
}

TEST(MotionGateJournal, SuccessfulOpenUsesTheSameCoreOwnedFence)
{
  JournalRegion<2U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  MotionGateCore gate(MotionGateConfig{}, kGateId, 0U, &journal);
  const auto now = MotionGateCore::SteadyTimePoint{};
  ASSERT_EQ(
    gate.prepare(
      ControlRequest{
        Operation::Prepare,
        "00000000000000000000000000000001",
        kGateId,
        0U,
        ""},
      now).code,
    ResultCode::Applied);
  const auto prepared = gate.snapshot();
  WriterGid writer{};
  writer.front() = 0x42U;
  writer.back() = 0xe7U;

  const auto result = gate.open(
    ControlRequest{
        Operation::Open,
        "00000000000000000000000000000002",
        kGateId,
        prepared.control_seq,
        prepared.lease_id},
    now,
    [writer]() {
      return OpenBinding{true, Reason::None, writer, "writer ready"};
    });

  ASSERT_EQ(result.code, ResultCode::Applied);
  ASSERT_EQ(result.state, State::Armed);
  EXPECT_EQ(clock.samples, 6U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    2U);
  const auto & slot = region.slots[1U];
  EXPECT_EQ(
    gate_event_journal_load_acquire(slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    slot.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION);
  EXPECT_EQ(slot.journal_seq, 2U);
  EXPECT_EQ(slot.generation, identity.generation);
  EXPECT_EQ(slot.intent_monotonic_ns, 400U);
  EXPECT_EQ(slot.transition_linearization_ns, 500U);
  EXPECT_EQ(slot.commit_monotonic_ns, 600U);
  EXPECT_EQ(slot.event_code, 2U);  // OPEN
  EXPECT_EQ(slot.reason, static_cast<std::uint64_t>(Reason::None));
  EXPECT_EQ(slot.before_state_seq, 1U);
  EXPECT_EQ(slot.before_control_seq, 1U);
  EXPECT_EQ(slot.after_state_seq, 2U);
  EXPECT_EQ(slot.after_control_seq, 2U);
  EXPECT_EQ(slot.before_lease_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.before_lease_lo, UINT64_C(0x0123456789abcdee));
  EXPECT_EQ(slot.after_lease_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.after_lease_lo, UINT64_C(0x0123456789abcdee));
  EXPECT_EQ(slot.gate_instance_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.gate_instance_lo, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.flags, 0U);
  EXPECT_EQ(slot.intent_checksum, gate_event_journal_intent_checksum(slot));
  EXPECT_EQ(slot.commit_checksum, gate_event_journal_commit_checksum(slot));
}

TEST(MotionGateJournal, ReservationFailureLeavesPrepareFailClosedAndUnchanged)
{
  JournalRegion<1U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  (void)journal.publish_output(
    GateOutputIntent{
        99U, 0U, 1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
    []() noexcept {});
  MotionGateCore gate(MotionGateConfig{}, kGateId, 0U, &journal);
  const auto before = gate.snapshot();

  EXPECT_THROW(
    (void)gate.prepare(
      ControlRequest{
        Operation::Prepare,
        "00000000000000000000000000000001",
        kGateId,
        0U,
        ""},
      MotionGateCore::SteadyTimePoint{}),
    std::overflow_error);

  const auto after = gate.snapshot();
  EXPECT_EQ(after.state, before.state);
  EXPECT_EQ(after.state_seq, before.state_seq);
  EXPECT_EQ(after.control_seq, before.control_seq);
  EXPECT_EQ(after.lease_id, before.lease_id);
  EXPECT_EQ(after.candidate_topic, before.candidate_topic);
  EXPECT_EQ(after.reason, before.reason);
  EXPECT_EQ(after.detail, before.detail);
  EXPECT_TRUE(after.motion_inhibited);
  EXPECT_TRUE(after.zero_selected);
  EXPECT_TRUE(gate.selected_command().is_zero());
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    1U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.overflow_latched),
    1U);
  EXPECT_EQ(clock.samples, 2U);
}

TEST(MotionGateJournal, FullJournalCannotPreventExplicitInhibit)
{
  JournalRegion<2U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  MotionGateCore gate(MotionGateConfig{}, kGateId, 0U, &journal);
  const auto now = MotionGateCore::SteadyTimePoint{};
  ASSERT_EQ(
    gate.prepare(
      ControlRequest{
        Operation::Prepare,
        "00000000000000000000000000000001",
        kGateId,
        0U,
        ""},
      now).code,
    ResultCode::Applied);
  const auto prepared = gate.snapshot();
  WriterGid writer{};
  writer.front() = 0x42U;
  writer.back() = 0xe7U;
  ASSERT_EQ(
    gate.open(
      ControlRequest{
        Operation::Open,
        "00000000000000000000000000000002",
        kGateId,
        prepared.control_seq,
        prepared.lease_id},
      now,
      [writer]() {
        return OpenBinding{true, Reason::None, writer, "writer ready"};
      }).code,
    ResultCode::Applied);
  const auto armed = gate.snapshot();

  ControlResult result;
  EXPECT_NO_THROW(
    result = gate.inhibit(
      ControlRequest{
        Operation::Inhibit,
        "00000000000000000000000000000003",
        kGateId,
        armed.control_seq,
        armed.lease_id},
      now));

  EXPECT_EQ(result.code, ResultCode::Applied);
  EXPECT_EQ(result.state, State::Inhibited);
  EXPECT_EQ(result.control_seq, 3U);
  EXPECT_TRUE(result.lease_id.empty());
  EXPECT_TRUE(result.motion_inhibited);
  EXPECT_TRUE(result.zero_selected);
  EXPECT_TRUE(gate.selected_command().is_zero());
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    2U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.overflow_latched),
    1U);
  EXPECT_EQ(clock.samples, 6U);
}

TEST(MotionGateJournal, SuccessfulRenewUsesTheSameCoreOwnedFence)
{
  JournalRegion<3U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  MotionGateCore gate(MotionGateConfig{}, kGateId, 0U, &journal);
  const auto now = MotionGateCore::SteadyTimePoint{};
  ASSERT_EQ(
    gate.prepare(
      ControlRequest{
        Operation::Prepare,
        "00000000000000000000000000000001",
        kGateId,
        0U,
        ""},
      now).code,
    ResultCode::Applied);
  const auto prepared = gate.snapshot();
  WriterGid writer{};
  writer.front() = 0x42U;
  writer.back() = 0xe7U;
  ASSERT_EQ(
    gate.open(
      ControlRequest{
        Operation::Open,
        "00000000000000000000000000000002",
        kGateId,
        prepared.control_seq,
        prepared.lease_id},
      now,
      [writer]() {
        return OpenBinding{true, Reason::None, writer, "writer ready"};
      }).code,
    ResultCode::Applied);
  const auto armed = gate.snapshot();

  const auto result = gate.renew(
    ControlRequest{
        Operation::Renew,
        "00000000000000000000000000000003",
        kGateId,
        armed.control_seq,
        armed.lease_id},
    now + std::chrono::milliseconds{10});

  ASSERT_EQ(result.code, ResultCode::Applied);
  ASSERT_EQ(result.state, State::Armed);
  EXPECT_EQ(clock.samples, 9U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    3U);
  const auto & slot = region.slots[2U];
  EXPECT_EQ(
    gate_event_journal_load_acquire(slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    slot.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION);
  EXPECT_EQ(slot.journal_seq, 3U);
  EXPECT_EQ(slot.generation, identity.generation);
  EXPECT_EQ(slot.intent_monotonic_ns, 700U);
  EXPECT_EQ(slot.transition_linearization_ns, 800U);
  EXPECT_EQ(slot.commit_monotonic_ns, 900U);
  EXPECT_EQ(slot.event_code, 3U);  // RENEW
  EXPECT_EQ(slot.reason, static_cast<std::uint64_t>(Reason::None));
  EXPECT_EQ(slot.before_state_seq, 2U);
  EXPECT_EQ(slot.before_control_seq, 2U);
  EXPECT_EQ(slot.after_state_seq, 3U);
  EXPECT_EQ(slot.after_control_seq, 3U);
  EXPECT_EQ(slot.before_lease_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.before_lease_lo, UINT64_C(0x0123456789abcdee));
  EXPECT_EQ(slot.after_lease_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.after_lease_lo, UINT64_C(0x0123456789abcdee));
  EXPECT_EQ(slot.gate_instance_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.gate_instance_lo, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.flags, 0U);
  EXPECT_EQ(slot.intent_checksum, gate_event_journal_intent_checksum(slot));
  EXPECT_EQ(slot.commit_checksum, gate_event_journal_commit_checksum(slot));
}

TEST(MotionGateJournal, ExplicitInhibitCommitsLeaseRetirementAtOneFence)
{
  JournalRegion<3U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  MotionGateCore gate(MotionGateConfig{}, kGateId, 0U, &journal);
  const auto now = MotionGateCore::SteadyTimePoint{};
  ASSERT_EQ(
    gate.prepare(
      ControlRequest{
        Operation::Prepare,
        "00000000000000000000000000000001",
        kGateId,
        0U,
        ""},
      now).code,
    ResultCode::Applied);
  const auto prepared = gate.snapshot();
  WriterGid writer{};
  writer.front() = 0x42U;
  writer.back() = 0xe7U;
  ASSERT_EQ(
    gate.open(
      ControlRequest{
        Operation::Open,
        "00000000000000000000000000000002",
        kGateId,
        prepared.control_seq,
        prepared.lease_id},
      now,
      [writer]() {
        return OpenBinding{true, Reason::None, writer, "writer ready"};
      }).code,
    ResultCode::Applied);
  const auto armed = gate.snapshot();

  const auto result = gate.inhibit(
    ControlRequest{
        Operation::Inhibit,
        "00000000000000000000000000000003",
        kGateId,
        armed.control_seq,
        armed.lease_id},
    now);

  ASSERT_EQ(result.code, ResultCode::Applied);
  ASSERT_EQ(result.state, State::Inhibited);
  EXPECT_TRUE(result.lease_id.empty());
  EXPECT_TRUE(result.motion_inhibited);
  EXPECT_TRUE(result.zero_selected);
  EXPECT_EQ(clock.samples, 9U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    3U);
  const auto & slot = region.slots[2U];
  EXPECT_EQ(
    gate_event_journal_load_acquire(slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    slot.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION);
  EXPECT_EQ(slot.journal_seq, 3U);
  EXPECT_EQ(slot.generation, identity.generation);
  EXPECT_EQ(slot.intent_monotonic_ns, 700U);
  EXPECT_EQ(slot.transition_linearization_ns, 800U);
  EXPECT_EQ(slot.commit_monotonic_ns, 900U);
  EXPECT_EQ(slot.event_code, 4U);  // INHIBIT
  EXPECT_EQ(slot.reason, static_cast<std::uint64_t>(Reason::None));
  EXPECT_EQ(slot.before_state_seq, 2U);
  EXPECT_EQ(slot.before_control_seq, 2U);
  EXPECT_EQ(slot.after_state_seq, 3U);
  EXPECT_EQ(slot.after_control_seq, 3U);
  EXPECT_EQ(slot.before_lease_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.before_lease_lo, UINT64_C(0x0123456789abcdee));
  EXPECT_EQ(slot.after_lease_hi, 0U);
  EXPECT_EQ(slot.after_lease_lo, 0U);
  EXPECT_EQ(slot.gate_instance_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.gate_instance_lo, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.flags, 0U);
  EXPECT_EQ(slot.intent_checksum, gate_event_journal_intent_checksum(slot));
  EXPECT_EQ(slot.commit_checksum, gate_event_journal_commit_checksum(slot));
}

TEST(MotionGateJournal, ForceFaultCommitsOneTerminalTransition)
{
  JournalRegion<1U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  MotionGateCore gate(MotionGateConfig{}, kGateId, 0U, &journal);

  gate.force_fault(Reason::PublishFailed, "publisher failed");

  const auto state = gate.snapshot();
  EXPECT_EQ(state.state, State::Faulted);
  EXPECT_EQ(state.state_seq, 1U);
  EXPECT_EQ(state.control_seq, 1U);
  EXPECT_EQ(state.reason, Reason::PublishFailed);
  EXPECT_TRUE(state.lease_id.empty());
  EXPECT_TRUE(state.motion_inhibited);
  EXPECT_TRUE(state.zero_selected);
  EXPECT_EQ(clock.samples, 3U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    1U);
  const auto & slot = region.slots.front();
  EXPECT_EQ(
    gate_event_journal_load_acquire(slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    slot.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION);
  EXPECT_EQ(slot.journal_seq, 1U);
  EXPECT_EQ(slot.generation, identity.generation);
  EXPECT_EQ(slot.intent_monotonic_ns, 100U);
  EXPECT_EQ(slot.transition_linearization_ns, 200U);
  EXPECT_EQ(slot.commit_monotonic_ns, 300U);
  EXPECT_EQ(slot.event_code, 6U);  // FAULT
  EXPECT_EQ(
    slot.reason,
    static_cast<std::uint64_t>(Reason::PublishFailed));
  EXPECT_EQ(slot.before_state_seq, 0U);
  EXPECT_EQ(slot.before_control_seq, 0U);
  EXPECT_EQ(slot.after_state_seq, 1U);
  EXPECT_EQ(slot.after_control_seq, 1U);
  EXPECT_EQ(slot.before_lease_hi, 0U);
  EXPECT_EQ(slot.before_lease_lo, 0U);
  EXPECT_EQ(slot.after_lease_hi, 0U);
  EXPECT_EQ(slot.after_lease_lo, 0U);
  EXPECT_EQ(slot.gate_instance_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.gate_instance_lo, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.flags, 0U);
  EXPECT_EQ(slot.intent_checksum, gate_event_journal_intent_checksum(slot));
  EXPECT_EQ(slot.commit_checksum, gate_event_journal_commit_checksum(slot));
}

TEST(MotionGateJournal, CandidateExpiryCommitsAutomaticRetirement)
{
  JournalRegion<3U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  MotionGateCore gate(MotionGateConfig{}, kGateId, 0U, &journal);
  const auto now = MotionGateCore::SteadyTimePoint{};
  ASSERT_EQ(
    gate.prepare(
      ControlRequest{
        Operation::Prepare,
        "00000000000000000000000000000001",
        kGateId,
        0U,
        ""},
      now).code,
    ResultCode::Applied);
  const auto prepared = gate.snapshot();
  WriterGid writer{};
  writer.front() = 0x42U;
  writer.back() = 0xe7U;
  ASSERT_EQ(
    gate.open(
      ControlRequest{
        Operation::Open,
        "00000000000000000000000000000002",
        kGateId,
        prepared.control_seq,
        prepared.lease_id},
      now,
      [writer]() {
        return OpenBinding{true, Reason::None, writer, "writer ready"};
      }).code,
    ResultCode::Applied);

  EXPECT_TRUE(
    gate.tick(now + std::chrono::milliseconds{150}).is_zero());

  const auto state = gate.snapshot();
  EXPECT_EQ(state.state, State::Inhibited);
  EXPECT_EQ(state.state_seq, 3U);
  EXPECT_EQ(state.control_seq, 3U);
  EXPECT_EQ(state.reason, Reason::CandidateExpired);
  EXPECT_TRUE(state.lease_id.empty());
  const auto & slot = region.slots[2U];
  EXPECT_EQ(
    gate_event_journal_load_acquire(slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(slot.journal_seq, 3U);
  EXPECT_EQ(slot.generation, identity.generation);
  EXPECT_EQ(slot.intent_monotonic_ns, 700U);
  EXPECT_EQ(slot.transition_linearization_ns, 800U);
  EXPECT_EQ(slot.commit_monotonic_ns, 900U);
  EXPECT_EQ(slot.event_code, 5U);  // AUTOMATIC_RETIRE
  EXPECT_EQ(
    slot.reason,
    static_cast<std::uint64_t>(Reason::CandidateExpired));
  EXPECT_EQ(slot.before_state_seq, 2U);
  EXPECT_EQ(slot.before_control_seq, 2U);
  EXPECT_EQ(slot.after_state_seq, 3U);
  EXPECT_EQ(slot.after_control_seq, 3U);
  EXPECT_EQ(slot.before_lease_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.before_lease_lo, UINT64_C(0x0123456789abcdee));
  EXPECT_EQ(slot.after_lease_hi, 0U);
  EXPECT_EQ(slot.after_lease_lo, 0U);
  EXPECT_EQ(slot.intent_checksum, gate_event_journal_intent_checksum(slot));
  EXPECT_EQ(slot.commit_checksum, gate_event_journal_commit_checksum(slot));
}

TEST(MotionGateJournal, SequenceExhaustionFaultIsJournaledWithoutWrap)
{
  JournalRegion<1U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, UINT64_MAX, &journal);

  const auto result = gate.prepare(
    ControlRequest{
        Operation::Prepare,
        "00000000000000000000000000000001",
        kGateId,
        UINT64_MAX,
        ""},
    MotionGateCore::SteadyTimePoint{});

  EXPECT_EQ(result.code, ResultCode::Faulted);
  EXPECT_EQ(result.state, State::Faulted);
  EXPECT_EQ(result.reason, Reason::SequenceExhausted);
  EXPECT_EQ(result.control_seq, UINT64_MAX);
  EXPECT_EQ(clock.samples, 3U);
  const auto & slot = region.slots.front();
  EXPECT_EQ(
    gate_event_journal_load_acquire(slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(slot.event_code, 6U);  // FAULT
  EXPECT_EQ(
    slot.reason,
    static_cast<std::uint64_t>(Reason::SequenceExhausted));
  EXPECT_EQ(slot.before_state_seq, 0U);
  EXPECT_EQ(slot.after_state_seq, 1U);
  EXPECT_EQ(slot.before_control_seq, UINT64_MAX);
  EXPECT_EQ(slot.after_control_seq, UINT64_MAX);
  EXPECT_EQ(slot.intent_checksum, gate_event_journal_intent_checksum(slot));
  EXPECT_EQ(slot.commit_checksum, gate_event_journal_commit_checksum(slot));
}

TEST(MotionGateJournal, FullJournalCannotPreventForceFault)
{
  JournalRegion<1U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  (void)journal.publish_output(
    GateOutputIntent{
        99U, 0U, 1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
    []() noexcept {});
  MotionGateCore gate(MotionGateConfig{}, kGateId, 0U, &journal);

  EXPECT_NO_THROW(
    gate.force_fault(Reason::PublishFailed, "publisher failed"));

  const auto state = gate.snapshot();
  EXPECT_EQ(state.state, State::Faulted);
  EXPECT_EQ(state.control_seq, 1U);
  EXPECT_EQ(state.reason, Reason::PublishFailed);
  EXPECT_TRUE(state.motion_inhibited);
  EXPECT_TRUE(state.zero_selected);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    1U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.overflow_latched),
    1U);
  EXPECT_EQ(clock.samples, 2U);
}

}  // namespace
}  // namespace voice_nav_mission
