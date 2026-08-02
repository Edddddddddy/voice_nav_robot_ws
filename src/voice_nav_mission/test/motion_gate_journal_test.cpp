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
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

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
static_assert(!std::is_copy_constructible_v<GateTransitionJournalBinding>);
static_assert(!std::is_copy_assignable_v<GateTransitionJournalBinding>);
static_assert(!std::is_move_constructible_v<GateTransitionJournalBinding>);
static_assert(!std::is_move_assignable_v<GateTransitionJournalBinding>);
static_assert(
  !std::is_constructible_v<
    MotionGateCore,
    MotionGateConfig,
    std::string,
    std::uint64_t,
    GateEventJournal *>);

struct TransitionProbe
{
  GateTransitionAfter operator()() const noexcept
  {
    return GateTransitionAfter{};
  }
};

template<typename Binding, typename = void>
struct HasPublicTransitionEntryPoint : std::false_type {};

template<typename Binding>
struct HasPublicTransitionEntryPoint<
  Binding,
  std::void_t<decltype(
    std::declval<Binding &>().apply_transition(
        std::declval<const GateTransitionIntent &>(),
        TransitionProbe{}))>>: std::true_type {};

static_assert(
  !HasPublicTransitionEntryPoint<GateTransitionJournalBinding>::value);

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

WriterGid journal_test_writer()
{
  WriterGid writer{};
  writer.front() = 0x42U;
  writer.back() = 0xe7U;
  return writer;
}

template<std::size_t Capacity>
struct JournalGateHarness
{
  explicit JournalGateHarness(
    MotionGateConfig config = MotionGateConfig{},
    std::uint64_t initial_control_seq = 0U)
  : identity(initialize_region(region)),
    journal(
      &region,
      sizeof(region),
      identity,
      GateEventJournalClock{&FakeClock::read, &clock}),
    gate(
      std::move(config),
      kGateId,
      initial_control_seq,
      journal.claim_transition_binding())
  {
  }

  ControlResult prepare(
    MotionGateCore::SteadyTimePoint now,
    const char * request_id = "00000000000000000000000000000001")
  {
    return gate.prepare(
      ControlRequest{
          Operation::Prepare,
          request_id,
          kGateId,
          gate.snapshot().control_seq,
          ""},
      now);
  }

  ControlResult open(
    MotionGateCore::SteadyTimePoint now,
    const char * request_id = "00000000000000000000000000000002")
  {
    const auto before = gate.snapshot();
    const auto writer = journal_test_writer();
    return gate.open(
      ControlRequest{
          Operation::Open,
          request_id,
          kGateId,
          before.control_seq,
          before.lease_id},
      now,
      [writer]() {
        return OpenBinding{true, Reason::None, writer, "writer ready"};
      });
  }

  Candidate candidate() const
  {
    return Candidate{
      gate.snapshot().lease_id,
      journal_test_writer(),
      false,
      0.10,
      0.0,
      0.0,
      0.0,
      0.0,
      0.20};
  }

  JournalRegion<Capacity> region;
  GateEventJournalIdentity identity;
  FakeClock clock;
  GateEventJournal journal;
  MotionGateCore gate;
};

void expect_automatic_retirement(
  const GateEventJournalSlot & slot,
  Reason reason,
  std::uint64_t journal_seq)
{
  EXPECT_EQ(
    gate_event_journal_load_acquire(slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(slot.record_kind, VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION);
  EXPECT_EQ(slot.event_code, 5U);
  EXPECT_EQ(slot.reason, static_cast<std::uint64_t>(reason));
  EXPECT_EQ(slot.journal_seq, journal_seq);
  EXPECT_EQ(slot.intent_checksum, gate_event_journal_intent_checksum(slot));
  EXPECT_EQ(slot.commit_checksum, gate_event_journal_commit_checksum(slot));
}

TEST(MotionGateJournal, OneJournalGenerationCannotBindTwoCores)
{
  JournalRegion<1U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});

  {
    MotionGateCore first(
      MotionGateConfig{}, kGateId, 0U,
      journal.claim_transition_binding());
    EXPECT_THROW(
      (void)journal.claim_transition_binding(),
      std::logic_error);
  }

  EXPECT_THROW(
    (void)journal.claim_transition_binding(),
    std::logic_error);
}

TEST(MotionGateJournal, BindingDetectsJournalLifetimeEndBeforeMutation)
{
  JournalRegion<1U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  std::unique_ptr<GateTransitionJournalBinding> binding;
  {
    GateEventJournal journal(
      &region,
      sizeof(region),
      identity,
      GateEventJournalClock{&FakeClock::read, &clock});
    binding = journal.claim_transition_binding();
  }
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U, std::move(binding));
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
    std::runtime_error);

  const auto after = gate.snapshot();
  EXPECT_EQ(after.state, before.state);
  EXPECT_EQ(after.state_seq, before.state_seq);
  EXPECT_EQ(after.control_seq, before.control_seq);
  EXPECT_EQ(after.lease_id, before.lease_id);
  EXPECT_EQ(after.reason, before.reason);
  EXPECT_EQ(after.detail, before.detail);
  EXPECT_TRUE(after.motion_inhibited);
  EXPECT_TRUE(after.zero_selected);
}

TEST(MotionGateJournal, DetachedBindingCannotBlockAForcedSafetyFault)
{
  JournalRegion<1U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  std::unique_ptr<GateTransitionJournalBinding> binding;
  {
    GateEventJournal journal(
      &region,
      sizeof(region),
      identity,
      GateEventJournalClock{&FakeClock::read, &clock});
    binding = journal.claim_transition_binding();
  }
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U, std::move(binding));

  EXPECT_NO_THROW(
    gate.force_fault(Reason::PublishFailed, "publisher failed"));

  const auto state = gate.snapshot();
  EXPECT_EQ(state.state, State::Faulted);
  EXPECT_EQ(state.reason, Reason::PublishFailed);
  EXPECT_EQ(state.control_seq, 1U);
  EXPECT_EQ(state.output_cause_transition_journal_seq, 0U);
  EXPECT_TRUE(state.motion_inhibited);
  EXPECT_TRUE(state.zero_selected);
}

TEST(MotionGateJournal, ConstructionFaultIsAnUnjournaledInitialState)
{
  JournalRegion<1U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  auto invalid_config = MotionGateConfig{};
  invalid_config.authority_lease = std::chrono::milliseconds{0};

  MotionGateCore gate(
    invalid_config,
    kGateId,
    0U,
    journal.claim_transition_binding());

  const auto state = gate.snapshot();
  EXPECT_EQ(state.state, State::Faulted);
  EXPECT_EQ(state.reason, Reason::ConfigurationInvalid);
  EXPECT_EQ(state.control_seq, 0U);
  EXPECT_EQ(state.output_cause_transition_journal_seq, 0U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    0U);
  EXPECT_TRUE(state.motion_inhibited);
  EXPECT_TRUE(state.zero_selected);
}

TEST(MotionGateJournal, JournalBoundConstructionRejectsEmptyCapability)
{
  EXPECT_THROW(
    (void)MotionGateCore(
      MotionGateConfig{},
      kGateId,
      0U,
      std::unique_ptr<GateTransitionJournalBinding>{}),
    std::invalid_argument);
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
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U,
    journal.claim_transition_binding());

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
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U,
    journal.claim_transition_binding());
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
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U,
    journal.claim_transition_binding());
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
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U,
    journal.claim_transition_binding());
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
    gate.snapshot().output_cause_transition_journal_seq,
    0U);
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
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U,
    journal.claim_transition_binding());
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
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U,
    journal.claim_transition_binding());
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
  EXPECT_EQ(
    gate.snapshot().output_cause_transition_journal_seq,
    3U);
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

TEST(MotionGateJournal, TerminalCauseFlowsIntoTheFirstZeroOutputIntent)
{
  JournalRegion<3U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U,
    journal.claim_transition_binding());
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
  ASSERT_EQ(
    gate.inhibit(
      ControlRequest{
        Operation::Inhibit,
        "00000000000000000000000000000002",
        kGateId,
        prepared.control_seq,
        prepared.lease_id},
      now).code,
    ResultCode::Applied);
  const auto inhibited = gate.snapshot();
  ASSERT_EQ(inhibited.output_cause_transition_journal_seq, 2U);

  const auto output = journal.publish_output(
    GateOutputIntent{
        41U,
        static_cast<std::uint64_t>(Reason::None),
        1U,
        1U,
        0U,
        0U,
        0U,
        0U,
        UINT64_C(0x0123456789abcdef),
        UINT64_C(0x0123456789abcdef),
        inhibited.output_cause_transition_journal_seq,
        0U},
    []() noexcept {});

  EXPECT_EQ(output.journal_seq, 3U);
  EXPECT_EQ(output.slot_index, 2U);
  const auto & slot = region.slots[2U];
  EXPECT_EQ(
    gate_event_journal_load_acquire(slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    slot.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_OUTPUT_ATTEMPT);
  EXPECT_EQ(slot.linear_x_bits, 0U);
  EXPECT_EQ(slot.angular_z_bits, 0U);
  EXPECT_EQ(slot.cause_transition_journal_seq, 2U);
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
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U,
    journal.claim_transition_binding());

  gate.force_fault(Reason::PublishFailed, "publisher failed");

  const auto state = gate.snapshot();
  EXPECT_EQ(state.state, State::Faulted);
  EXPECT_EQ(state.state_seq, 1U);
  EXPECT_EQ(state.control_seq, 1U);
  EXPECT_EQ(state.reason, Reason::PublishFailed);
  EXPECT_TRUE(state.lease_id.empty());
  EXPECT_TRUE(state.motion_inhibited);
  EXPECT_TRUE(state.zero_selected);
  EXPECT_EQ(state.output_cause_transition_journal_seq, 1U);
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
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U,
    journal.claim_transition_binding());
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
  EXPECT_EQ(state.output_cause_transition_journal_seq, 3U);
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
    MotionGateConfig{}, kGateId, UINT64_MAX,
    journal.claim_transition_binding());

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
  EXPECT_EQ(
    gate.snapshot().output_cause_transition_journal_seq,
    1U);
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

TEST(MotionGateJournal, InhibitAtMaximumSequenceReturnsTheFaultItCommitted)
{
  JournalRegion<3U> region;
  const auto identity = initialize_region(region);
  FakeClock clock;
  GateEventJournal journal(
    &region,
    sizeof(region),
    identity,
    GateEventJournalClock{&FakeClock::read, &clock});
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, UINT64_MAX - 2U,
    journal.claim_transition_binding());
  const auto now = MotionGateCore::SteadyTimePoint{};
  ASSERT_EQ(
    gate.prepare(
      ControlRequest{
        Operation::Prepare,
        "00000000000000000000000000000001",
        kGateId,
        UINT64_MAX - 2U,
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
  const ControlRequest request{
    Operation::Inhibit,
    "00000000000000000000000000000003",
    kGateId,
    armed.control_seq,
    armed.lease_id};

  const auto result = gate.inhibit(request, now);
  const auto retry = gate.inhibit(request, now);

  EXPECT_EQ(result.code, ResultCode::Faulted);
  EXPECT_EQ(result.reason, Reason::SequenceExhausted);
  EXPECT_EQ(result.state, State::Faulted);
  EXPECT_EQ(result.control_seq, UINT64_MAX);
  EXPECT_TRUE(result.lease_id.empty());
  EXPECT_TRUE(result.motion_inhibited);
  EXPECT_TRUE(result.zero_selected);
  EXPECT_EQ(
    gate.snapshot().output_cause_transition_journal_seq,
    3U);
  EXPECT_EQ(retry.code, result.code);
  EXPECT_EQ(retry.reason, result.reason);
  EXPECT_EQ(retry.state, result.state);
  EXPECT_EQ(retry.control_seq, result.control_seq);
  EXPECT_EQ(
    gate_event_journal_load_acquire(region.header.claimed_slots),
    3U);
  const auto & slot = region.slots[2U];
  EXPECT_EQ(
    gate_event_journal_load_acquire(slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(slot.event_code, 6U);  // FAULT
  EXPECT_EQ(
    slot.reason,
    static_cast<std::uint64_t>(Reason::SequenceExhausted));
  EXPECT_EQ(slot.before_state_seq, 2U);
  EXPECT_EQ(slot.after_state_seq, 3U);
  EXPECT_EQ(slot.before_control_seq, UINT64_MAX);
  EXPECT_EQ(slot.after_control_seq, UINT64_MAX);
  EXPECT_NE(slot.before_lease_hi | slot.before_lease_lo, 0U);
  EXPECT_EQ(slot.after_lease_hi, 0U);
  EXPECT_EQ(slot.after_lease_lo, 0U);
  EXPECT_EQ(slot.intent_checksum, gate_event_journal_intent_checksum(slot));
  EXPECT_EQ(slot.commit_checksum, gate_event_journal_commit_checksum(slot));
}

TEST(MotionGateJournal, EveryAutomaticRetirementReasonUsesEventFive)
{
  {
    JournalGateHarness<2U> harness;
    ASSERT_EQ(
      harness.prepare(MotionGateCore::SteadyTimePoint{}).code,
      ResultCode::Applied);

    EXPECT_TRUE(
      harness.gate.tick(
        MotionGateCore::SteadyTimePoint{} +
        std::chrono::milliseconds{1000}).is_zero());

    EXPECT_EQ(harness.gate.snapshot().reason, Reason::PrepareExpired);
    EXPECT_EQ(harness.gate.snapshot().output_cause_transition_journal_seq, 2U);
    expect_automatic_retirement(
      harness.region.slots[1U], Reason::PrepareExpired, 2U);
  }

  {
    JournalGateHarness<3U> harness;
    const auto now = MotionGateCore::SteadyTimePoint{};
    ASSERT_EQ(harness.prepare(now).code, ResultCode::Applied);
    ASSERT_EQ(harness.open(now).code, ResultCode::Applied);

    EXPECT_TRUE(
      harness.gate.tick(
        now + std::chrono::milliseconds{250}).is_zero());

    EXPECT_EQ(harness.gate.snapshot().reason, Reason::AuthorityExpired);
    EXPECT_EQ(harness.gate.snapshot().output_cause_transition_journal_seq, 3U);
    expect_automatic_retirement(
      harness.region.slots[2U], Reason::AuthorityExpired, 3U);
  }

  {
    JournalGateHarness<3U> harness;
    const auto now = MotionGateCore::SteadyTimePoint{};
    ASSERT_EQ(harness.prepare(now).code, ResultCode::Applied);
    ASSERT_EQ(harness.open(now).code, ResultCode::Applied);
    auto candidate = harness.candidate();
    candidate.writer_gid.front() ^= 1U;

    const auto result = harness.gate.accept_candidate(candidate, now);

    EXPECT_TRUE(result.retired);
    EXPECT_EQ(result.reason, Reason::WriterMismatch);
    EXPECT_EQ(harness.gate.snapshot().output_cause_transition_journal_seq, 3U);
    expect_automatic_retirement(
      harness.region.slots[2U], Reason::WriterMismatch, 3U);
  }

  {
    JournalGateHarness<3U> harness;
    const auto now = MotionGateCore::SteadyTimePoint{};
    ASSERT_EQ(harness.prepare(now).code, ResultCode::Applied);
    ASSERT_EQ(harness.open(now).code, ResultCode::Applied);
    auto candidate = harness.candidate();
    candidate.linear_y = 0.01;

    const auto result = harness.gate.accept_candidate(candidate, now);

    EXPECT_TRUE(result.retired);
    EXPECT_EQ(result.reason, Reason::InvalidCandidate);
    EXPECT_EQ(harness.gate.snapshot().output_cause_transition_journal_seq, 3U);
    expect_automatic_retirement(
      harness.region.slots[2U], Reason::InvalidCandidate, 3U);
  }
}

TEST(MotionGateJournal, FullJournalSeparatesAdmissionFromSafetyMutation)
{
  {
    JournalGateHarness<1U> harness;
    const auto now = MotionGateCore::SteadyTimePoint{};
    ASSERT_EQ(harness.prepare(now).code, ResultCode::Applied);
    const auto before = harness.gate.snapshot();

    EXPECT_THROW(
      (void)harness.open(now),
      std::overflow_error);

    const auto after = harness.gate.snapshot();
    EXPECT_EQ(after.state, State::Prepared);
    EXPECT_EQ(after.state_seq, before.state_seq);
    EXPECT_EQ(after.control_seq, before.control_seq);
    EXPECT_EQ(after.lease_id, before.lease_id);
    EXPECT_EQ(
      gate_event_journal_load_acquire(
        harness.region.header.overflow_latched),
      1U);
  }

  {
    JournalGateHarness<2U> harness;
    const auto now = MotionGateCore::SteadyTimePoint{};
    ASSERT_EQ(harness.prepare(now).code, ResultCode::Applied);
    ASSERT_EQ(harness.open(now).code, ResultCode::Applied);
    const auto before = harness.gate.snapshot();

    EXPECT_THROW(
      (void)harness.gate.renew(
        ControlRequest{
          Operation::Renew,
          "00000000000000000000000000000003",
          kGateId,
          before.control_seq,
          before.lease_id},
        now + std::chrono::milliseconds{10}),
      std::overflow_error);

    const auto after = harness.gate.snapshot();
    EXPECT_EQ(after.state, State::Armed);
    EXPECT_EQ(after.state_seq, before.state_seq);
    EXPECT_EQ(after.control_seq, before.control_seq);
    EXPECT_EQ(after.lease_id, before.lease_id);
  }

  {
    JournalGateHarness<1U> harness;
    const auto now = MotionGateCore::SteadyTimePoint{};
    ASSERT_EQ(harness.prepare(now).code, ResultCode::Applied);

    EXPECT_NO_THROW(
      (void)harness.gate.tick(
        now + std::chrono::milliseconds{1000}));

    const auto state = harness.gate.snapshot();
    EXPECT_EQ(state.state, State::Inhibited);
    EXPECT_EQ(state.reason, Reason::PrepareExpired);
    EXPECT_EQ(state.output_cause_transition_journal_seq, 0U);
    EXPECT_TRUE(state.zero_selected);
  }

  {
    JournalGateHarness<2U> harness;
    const auto now = MotionGateCore::SteadyTimePoint{};
    ASSERT_EQ(harness.prepare(now).code, ResultCode::Applied);
    ASSERT_EQ(harness.open(now).code, ResultCode::Applied);
    auto candidate = harness.candidate();
    candidate.linear_y = 0.01;

    CandidateResult result;
    EXPECT_NO_THROW(
      result = harness.gate.accept_candidate(candidate, now));

    EXPECT_TRUE(result.retired);
    EXPECT_EQ(result.reason, Reason::InvalidCandidate);
    EXPECT_EQ(harness.gate.snapshot().state, State::Inhibited);
    EXPECT_EQ(
      harness.gate.snapshot().output_cause_transition_journal_seq,
      0U);
  }
}

TEST(MotionGateJournal, ProviderFaultsCommitOneTerminalRecordAndReplayNone)
{
  {
    JournalGateHarness<1U> harness;
    const ControlRequest request{
      Operation::Prepare,
      "00000000000000000000000000000001",
      kGateId,
      0U,
      ""};
    std::size_t calls = 0U;
    const auto provider = [&calls]() -> PrepareAdmission {
        ++calls;
        throw std::runtime_error("admission failed");
      };

    const auto result = harness.gate.prepare(
      request, MotionGateCore::SteadyTimePoint{}, provider);
    const auto replay = harness.gate.prepare(
      request, MotionGateCore::SteadyTimePoint{}, provider);

    EXPECT_EQ(result.code, ResultCode::Faulted);
    EXPECT_EQ(result.reason, Reason::InternalFailure);
    EXPECT_EQ(replay.code, ResultCode::Faulted);
    EXPECT_EQ(calls, 1U);
    EXPECT_EQ(
      gate_event_journal_load_acquire(harness.region.header.claimed_slots),
      1U);
    EXPECT_EQ(harness.region.slots[0U].event_code, 6U);
    EXPECT_EQ(
      harness.region.slots[0U].reason,
      static_cast<std::uint64_t>(Reason::InternalFailure));
  }

  {
    JournalGateHarness<2U> harness;
    const auto now = MotionGateCore::SteadyTimePoint{};
    ASSERT_EQ(harness.prepare(now).code, ResultCode::Applied);
    const auto prepared = harness.gate.snapshot();
    const ControlRequest request{
      Operation::Open,
      "00000000000000000000000000000002",
      kGateId,
      prepared.control_seq,
      prepared.lease_id};
    std::size_t calls = 0U;
    const auto provider = [&calls]() -> OpenBinding {
        ++calls;
        throw std::runtime_error("binding failed");
      };

    const auto result = harness.gate.open(request, now, provider);
    const auto replay = harness.gate.open(request, now, provider);

    EXPECT_EQ(result.code, ResultCode::Faulted);
    EXPECT_EQ(result.reason, Reason::InternalFailure);
    EXPECT_EQ(replay.code, ResultCode::Faulted);
    EXPECT_EQ(calls, 1U);
    EXPECT_EQ(
      gate_event_journal_load_acquire(harness.region.header.claimed_slots),
      2U);
    EXPECT_EQ(harness.region.slots[1U].event_code, 6U);
    EXPECT_EQ(
      harness.region.slots[1U].reason,
      static_cast<std::uint64_t>(Reason::InternalFailure));
  }
}

TEST(MotionGateJournal, NonTransitionsNeverConsumeJournalSlots)
{
  JournalGateHarness<3U> harness;
  const auto now = MotionGateCore::SteadyTimePoint{};
  const ControlRequest stale{
    Operation::Prepare,
    "00000000000000000000000000000009",
    kGateId,
    1U,
    ""};
  EXPECT_EQ(
    harness.gate.prepare(stale, now).code,
    ResultCode::Rejected);
  EXPECT_EQ(
    gate_event_journal_load_acquire(harness.region.header.claimed_slots),
    0U);

  const ControlRequest prepare{
    Operation::Prepare,
    "00000000000000000000000000000001",
    kGateId,
    0U,
    ""};
  ASSERT_EQ(harness.gate.prepare(prepare, now).code, ResultCode::Applied);
  EXPECT_EQ(
    harness.gate.prepare(prepare, now).code,
    ResultCode::Duplicate);
  EXPECT_EQ(
    gate_event_journal_load_acquire(harness.region.header.claimed_slots),
    1U);
  ASSERT_EQ(harness.open(now).code, ResultCode::Applied);
  EXPECT_TRUE(harness.gate.accept_candidate(harness.candidate(), now).accepted);
  EXPECT_EQ(
    gate_event_journal_load_acquire(harness.region.header.claimed_slots),
    2U);

  harness.gate.force_fault(Reason::PublishFailed, "publisher failed");
  harness.gate.force_fault(Reason::InternalFailure, "late fault");
  EXPECT_EQ(
    gate_event_journal_load_acquire(harness.region.header.claimed_slots),
    3U);
  EXPECT_EQ(harness.region.slots[2U].event_code, 6U);
  EXPECT_EQ(
    harness.region.slots[2U].reason,
    static_cast<std::uint64_t>(Reason::PublishFailed));
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
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, 0U,
    journal.claim_transition_binding());

  EXPECT_NO_THROW(
    gate.force_fault(Reason::PublishFailed, "publisher failed"));

  const auto state = gate.snapshot();
  EXPECT_EQ(state.state, State::Faulted);
  EXPECT_EQ(state.control_seq, 1U);
  EXPECT_EQ(state.reason, Reason::PublishFailed);
  EXPECT_TRUE(state.motion_inhibited);
  EXPECT_TRUE(state.zero_selected);
  EXPECT_EQ(state.output_cause_transition_journal_seq, 0U);
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
