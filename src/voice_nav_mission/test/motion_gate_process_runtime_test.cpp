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

#include <unistd.h>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <new>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "gate_event_journal_test_region.hpp"
#include "motion_gate_process_runtime.hpp"

namespace voice_nav_mission
{

class MotionGateProcessRuntimeTestPeer
{
public:
  static void seed_output_sequences(
    MotionGateProcessRuntime & runtime,
    std::uint64_t attempt_sequence,
    std::uint64_t publish_sequence,
    std::uint64_t zero_sequence)
  {
    runtime.output_attempt_seq_ = attempt_sequence;
    runtime.output_publish_seq_ = publish_sequence;
    runtime.zero_publish_seq_ = zero_sequence;
  }

  [[nodiscard]] static std::uint64_t output_attempt_sequence(
    const MotionGateProcessRuntime & runtime) noexcept
  {
    return runtime.output_attempt_seq_;
  }
};

namespace
{

static_assert(!std::is_copy_constructible_v<MotionGateProcessRuntime>);
static_assert(!std::is_copy_assignable_v<MotionGateProcessRuntime>);
static_assert(!std::is_move_constructible_v<MotionGateProcessRuntime>);
static_assert(!std::is_move_assignable_v<MotionGateProcessRuntime>);

std::string descriptor_for(const OwnedJournalRegion & owner)
{
  const auto identity = owner.identity();
  std::ostringstream descriptor;
  descriptor << "v1:" << identity.owner_uid
             << ':' << identity.generation
             << ':' << owner.capacity()
             << ':' << std::hex << std::nouppercase << std::setfill('0')
             << std::setw(16) << identity.nonce_hi
             << std::setw(16) << identity.nonce_lo;
  return descriptor.str();
}

struct RecordingFinalPublisher
{
  explicit RecordingFinalPublisher(OwnedJournalRegion * journal_owner)
  : owner(journal_owner)
  {
  }

  OwnedJournalRegion * owner{nullptr};
  std::vector<FinalOutputFrame> frames;
  std::vector<std::uint64_t> phases_during_publish;
  std::size_t failures_remaining{0U};

  static void publish(void * context, const FinalOutputFrame & frame)
  {
    auto & self = *static_cast<RecordingFinalPublisher *>(context);
    self.frames.push_back(frame);
    if (self.owner == nullptr) {
      self.phases_during_publish.push_back(0U);
    } else {
      const auto claimed = gate_event_journal_load_acquire(
        self.owner->header().claimed_slots);
      if (claimed == 0U) {
        self.phases_during_publish.push_back(0U);
      } else {
        self.phases_during_publish.push_back(
          gate_event_journal_load_acquire(
            self.owner->slot(claimed - 1U).phase));
      }
    }
    if (self.failures_remaining != 0U) {
      --self.failures_remaining;
      throw std::runtime_error("injected final publisher failure");
    }
  }

  [[nodiscard]] FinalOutputPublisher adapter() noexcept
  {
    return FinalOutputPublisher{&RecordingFinalPublisher::publish, this};
  }
};

struct ThrowingFaultTestAdapter
{
  std::size_t calls{0U};

  static void apply(
    void * context,
    MotionGateCore & core,
    Reason reason,
    const char * detail)
  {
    auto & self = *static_cast<ThrowingFaultTestAdapter *>(context);
    ++self.calls;
    static_cast<void>(core);
    static_cast<void>(reason);
    static_cast<void>(detail);
    throw std::bad_alloc{};
  }

  [[nodiscard]] FinalOutputFaultTestAdapter adapter() noexcept
  {
    return FinalOutputFaultTestAdapter{
      &ThrowingFaultTestAdapter::apply, this};
  }
};

void select_nonzero_command(
  MotionGateProcessRuntime & runtime,
  const std::string & gate_instance_id)
{
  const auto now = MotionGateCore::SteadyTimePoint{} +
  std::chrono::milliseconds{10};
  const auto prepared = runtime.core().prepare(
    ControlRequest{
        Operation::Prepare,
        "00000000000000000000000000000011",
        gate_instance_id,
        0U,
        ""},
    now);
  if (prepared.code != ResultCode::Applied) {
    throw std::runtime_error("test could not prepare MotionGate");
  }

  WriterGid writer_gid{};
  writer_gid[0] = 0x5aU;
  const auto opened = runtime.core().open(
    ControlRequest{
        Operation::Open,
        "00000000000000000000000000000012",
        gate_instance_id,
        prepared.control_seq,
        prepared.lease_id},
    now,
    [writer_gid]() {
      return OpenBinding{true, Reason::None, writer_gid, "ready"};
    });
  if (opened.code != ResultCode::Applied) {
    throw std::runtime_error("test could not open MotionGate");
  }

  const auto candidate = runtime.core().accept_candidate(
    Candidate{
        opened.lease_id,
        writer_gid,
        false,
        0.20,
        0.0,
        0.0,
        0.0,
        0.0,
        0.40},
    now);
  if (!candidate.accepted || runtime.core().tick(now).is_zero()) {
    throw std::runtime_error("test could not select a nonzero command");
  }
}

TEST(MotionGateProcessRuntimeTest, BothEmptyParametersDisableAttachment)
{
  const auto config = parse_gate_event_journal_test_parameters(
    GateEventJournalTestParameters{"", ""});

  EXPECT_FALSE(config.has_value());
}

TEST(MotionGateProcessRuntimeTest, CompleteParametersProduceExactConfig)
{
  const std::string name =
    "/voice_nav_gate_00112233445566778899aabbccddeeff";
  const std::string nonce =
    "123456789abcdef00fedcba987654321";
  const std::string descriptor =
    "v1:" +
    std::to_string(static_cast<std::uint64_t>(geteuid())) +
    ":7:16:" + nonce;

  const auto config = parse_gate_event_journal_test_parameters(
    GateEventJournalTestParameters{name, descriptor});

  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->shared_memory_name, name);
  EXPECT_EQ(
    config->expected_identity.owner_uid,
    static_cast<std::uint64_t>(geteuid()));
  EXPECT_EQ(config->expected_identity.generation, 7U);
  EXPECT_EQ(
    config->expected_identity.nonce_hi,
    UINT64_C(0x123456789abcdef0));
  EXPECT_EQ(
    config->expected_identity.nonce_lo,
    UINT64_C(0x0fedcba987654321));
  EXPECT_EQ(config->expected_capacity, 16U);
  EXPECT_NE(config->clock.read, nullptr);
}

TEST(MotionGateProcessRuntimeTest, MalformedParametersAreRejected)
{
  const std::string uid =
    std::to_string(static_cast<std::uint64_t>(geteuid()));
  const std::string name =
    "/voice_nav_gate_00112233445566778899aabbccddeeff";
  const std::string nonce =
    "123456789abcdef00fedcba987654321";
  const std::string descriptor =
    "v1:" + uid + ":7:16:" + nonce;
  const auto wrong_uid =
    std::to_string(static_cast<std::uint64_t>(geteuid()) + 1U);

  const std::vector<GateEventJournalTestParameters> invalid{
    {name, ""},
    {"", descriptor},
    {name, "v2:" + uid + ":7:16:" + nonce},
    {name, "v1::7:16:" + nonce},
    {name, "v1:" + uid + ":7:16"},
    {name, "v1:" + uid + ":7:16:" + nonce + ":extra"},
    {name, "v1:18446744073709551616:7:16:" + nonce},
    {name, "v1:0" + uid + ":7:16:" + nonce},
    {name, "v1:+" + uid + ":7:16:" + nonce},
    {name, "v1: " + uid + ":7:16:" + nonce},
    {name, "v1:" + uid + ":07:16:" + nonce},
    {name, "v1:" + uid + ":+7:16:" + nonce},
    {name, "v1:" + uid + ": 7:16:" + nonce},
    {name, "v1:" + uid + ":7:016:" + nonce},
    {name, "v1:" + uid + ":7:+16:" + nonce},
    {name, "v1:" + uid + ":7: 16:" + nonce},
    {name, "v1:" + wrong_uid + ":7:16:" + nonce},
    {name, "v1:" + uid + ":0:16:" + nonce},
    {name, "v1:" + uid + ":7:0:" + nonce},
    {name, "v1:" + uid + ":7:16385:" + nonce},
    {name, "v1:" + uid + ":18446744073709551616:16:" + nonce},
    {name, "v1:" + uid + ":7:16:00000000000000000000000000000000"},
    {name, "v1:" + uid + ":7:16:123456789abcdef00fedcba98765432"},
    {name, "v1:" + uid + ":7:16:123456789abcdef00fedcba9876543210"},
    {name, "v1:" + uid + ":7:16:123456789ABCDEF00fedcba987654321"},
    {name, "v1:" + uid + ":7:16:123456789abcdef00fedcba98765432g"},
    {"/voice_nav_gate_00112233445566778899aabbccddeef", descriptor},
    {"/voice_nav_other_00112233445566778899aabbccddeeff", descriptor},
    {"/voice_nav_gate_00112233445566778899AABBCCDDEEFF", descriptor},
    {"/voice_nav_gate_00112233445566778899aabbccddeefg", descriptor},
  };

  for (const auto & parameters : invalid) {
    SCOPED_TRACE(parameters.name + " | " + parameters.descriptor);
    EXPECT_THROW(
      static_cast<void>(
        parse_gate_event_journal_test_parameters(parameters)),
      std::invalid_argument);
  }
}

TEST(MotionGateProcessRuntimeTest, DefaultOffRuntimeOwnsOneFailClosedCore)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  MotionGateProcessRuntime runtime(
    MotionGateConfig{},
    gate_instance_id,
    GateEventJournalTestParameters{"", ""});

  const auto snapshot = runtime.core().snapshot();
  EXPECT_EQ(snapshot.gate_instance_id, gate_instance_id);
  EXPECT_EQ(snapshot.state, State::Inhibited);
  EXPECT_TRUE(snapshot.motion_inhibited);
  EXPECT_TRUE(snapshot.zero_selected);
}

TEST(MotionGateProcessRuntimeTest, AttachedRuntimeOwnsCoreBeforeMapping)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  OwnedJournalRegion owner(4U);

  {
    MotionGateProcessRuntime runtime(
      MotionGateConfig{},
      gate_instance_id,
      GateEventJournalTestParameters{
          owner.name(), descriptor_for(owner)});

    EXPECT_EQ(
      gate_event_journal_load_acquire(owner.header().writer_pid),
      static_cast<std::uint64_t>(getpid()));
    owner.unlink_name();

    const auto result = runtime.core().prepare(
      ControlRequest{
          Operation::Prepare,
          "00000000000000000000000000000001",
          gate_instance_id,
          0U,
          ""},
      MotionGateCore::SteadyTimePoint{} + std::chrono::milliseconds{10});
    ASSERT_EQ(result.code, ResultCode::Applied);
  }

  EXPECT_EQ(
    gate_event_journal_load_acquire(owner.header().claimed_slots),
    1U);
  const auto & slot = owner.slot(0U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    slot.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION);
  EXPECT_EQ(slot.event_code, 1U);
  EXPECT_EQ(slot.journal_seq, 1U);
  EXPECT_EQ(slot.generation, owner.identity().generation);
  EXPECT_NE(slot.intent_monotonic_ns, 0U);
  EXPECT_GE(slot.transition_linearization_ns, slot.intent_monotonic_ns);
  EXPECT_GE(slot.commit_monotonic_ns, slot.transition_linearization_ns);
}

TEST(MotionGateProcessRuntimeTest, JournaledSuccessCommitsBeforeCountersAdvance)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  OwnedJournalRegion owner(4U);
  MotionGateProcessRuntime runtime(
    MotionGateConfig{},
    gate_instance_id,
    GateEventJournalTestParameters{
        owner.name(), descriptor_for(owner)});
  RecordingFinalPublisher publisher{&owner};

  const auto result = runtime.publish_final_command(
    FinalOutputTime{true, 7, 9U}, publisher.adapter());

  ASSERT_EQ(publisher.frames.size(), 1U);
  EXPECT_TRUE(publisher.frames[0].command.is_zero());
  EXPECT_EQ(publisher.frames[0].stamp_sec, 7);
  EXPECT_EQ(publisher.frames[0].stamp_nanosec, 9U);
  ASSERT_EQ(publisher.phases_during_publish.size(), 1U);
  EXPECT_EQ(
    publisher.phases_during_publish[0],
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT);
  EXPECT_TRUE(result.published);
  EXPECT_TRUE(result.zero_published);
  EXPECT_TRUE(result.journal_committed);
  EXPECT_FALSE(result.fallback_attempted);
  EXPECT_EQ(result.failure, FinalOutputFailure::None);
  EXPECT_EQ(result.state.output_publish_seq, 1U);
  EXPECT_EQ(result.state.zero_publish_seq, 1U);
  EXPECT_TRUE(result.state.last_publication_was_zero);
  EXPECT_EQ(result.locally_consumed_terminal_cause_seq, 0U);

  ASSERT_EQ(
    gate_event_journal_load_acquire(owner.header().claimed_slots),
    1U);
  const auto & slot = owner.slot(0U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(slot.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    slot.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_OUTPUT_ATTEMPT);
  EXPECT_EQ(slot.event_code, 1U);
  EXPECT_EQ(slot.reason, static_cast<std::uint64_t>(Reason::None));
  EXPECT_EQ(slot.output_attempt_seq, 1U);
  EXPECT_EQ(slot.intended_output_seq, 1U);
  EXPECT_EQ(slot.ros_stamp_sec_bits, 7U);
  EXPECT_EQ(slot.ros_stamp_nanosec, 9U);
  EXPECT_EQ(slot.linear_x_bits, 0U);
  EXPECT_EQ(slot.angular_z_bits, 0U);
  EXPECT_EQ(slot.gate_instance_hi, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.gate_instance_lo, UINT64_C(0x0123456789abcdef));
  EXPECT_EQ(slot.cause_transition_journal_seq, 0U);
  EXPECT_EQ(slot.flags, 0U);
  EXPECT_EQ(slot.intent_checksum, gate_event_journal_intent_checksum(slot));
  EXPECT_EQ(slot.commit_checksum, gate_event_journal_commit_checksum(slot));
}

TEST(MotionGateProcessRuntimeTest, TerminalZeroBindsCauseExactlyOnce)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  OwnedJournalRegion owner(8U);
  MotionGateProcessRuntime runtime(
    MotionGateConfig{},
    gate_instance_id,
    GateEventJournalTestParameters{
        owner.name(), descriptor_for(owner)});
  const auto now = MotionGateCore::SteadyTimePoint{} +
    std::chrono::milliseconds{10};
  const auto prepared = runtime.core().prepare(
    ControlRequest{
        Operation::Prepare,
        "00000000000000000000000000000001",
        gate_instance_id,
        0U,
        ""},
    now);
  ASSERT_EQ(prepared.code, ResultCode::Applied);
  const auto inhibited = runtime.core().inhibit(
    ControlRequest{
        Operation::Inhibit,
        "00000000000000000000000000000002",
        gate_instance_id,
        prepared.control_seq,
        prepared.lease_id},
    now);
  ASSERT_EQ(inhibited.code, ResultCode::Applied);
  ASSERT_EQ(
    runtime.core().snapshot().output_cause_transition_journal_seq,
    2U);
  RecordingFinalPublisher publisher{&owner};

  const auto first = runtime.publish_final_command(
    FinalOutputTime{true, 11, 12U}, publisher.adapter());
  const auto second = runtime.publish_final_command(
    FinalOutputTime{true, 13, 14U}, publisher.adapter());

  ASSERT_EQ(publisher.frames.size(), 2U);
  EXPECT_TRUE(publisher.frames[0].command.is_zero());
  EXPECT_TRUE(publisher.frames[1].command.is_zero());
  EXPECT_TRUE(first.journal_committed);
  EXPECT_TRUE(second.journal_committed);
  EXPECT_EQ(first.locally_consumed_terminal_cause_seq, 2U);
  EXPECT_EQ(second.locally_consumed_terminal_cause_seq, 0U);
  EXPECT_EQ(first.state.output_publish_seq, 1U);
  EXPECT_EQ(first.state.zero_publish_seq, 1U);
  EXPECT_EQ(second.state.output_publish_seq, 2U);
  EXPECT_EQ(second.state.zero_publish_seq, 2U);
  EXPECT_EQ(owner.slot(2U).cause_transition_journal_seq, 2U);
  EXPECT_EQ(owner.slot(3U).cause_transition_journal_seq, 0U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(owner.slot(2U).phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    gate_event_journal_load_acquire(owner.slot(3U).phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
}

TEST(MotionGateProcessRuntimeTest, JournalOverflowFallsBackOnceAndRetiresLane)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  OwnedJournalRegion owner(2U);
  MotionGateProcessRuntime runtime(
    MotionGateConfig{},
    gate_instance_id,
    GateEventJournalTestParameters{
        owner.name(), descriptor_for(owner)});
  select_nonzero_command(runtime, gate_instance_id);
  ASSERT_EQ(
    gate_event_journal_load_acquire(owner.header().claimed_slots),
    2U);
  RecordingFinalPublisher publisher{&owner};

  const auto failed = runtime.publish_final_command(
    FinalOutputTime{true, 21, 22U}, publisher.adapter());

  ASSERT_EQ(publisher.frames.size(), 1U);
  EXPECT_TRUE(publisher.frames[0].command.is_zero());
  EXPECT_EQ(failed.failure, FinalOutputFailure::JournalFailure);
  EXPECT_TRUE(failed.fallback_attempted);
  EXPECT_TRUE(failed.published);
  EXPECT_TRUE(failed.zero_published);
  EXPECT_FALSE(failed.journal_committed);
  EXPECT_EQ(failed.state.output_publish_seq, 1U);
  EXPECT_EQ(failed.state.zero_publish_seq, 1U);
  EXPECT_EQ(failed.locally_consumed_terminal_cause_seq, 0U);
  EXPECT_EQ(runtime.core().snapshot().state, State::Faulted);
  EXPECT_EQ(runtime.core().snapshot().reason, Reason::InternalFailure);
  EXPECT_EQ(
    gate_event_journal_load_acquire(owner.header().overflow_latched),
    1U);

  const auto retired_claimed_slots = gate_event_journal_load_acquire(
    owner.header().claimed_slots);
  const auto retired = runtime.publish_final_command(
    FinalOutputTime{true, 23, 24U}, publisher.adapter());

  ASSERT_EQ(publisher.frames.size(), 2U);
  EXPECT_TRUE(publisher.frames[1].command.is_zero());
  EXPECT_EQ(retired.failure, FinalOutputFailure::JournalFailure);
  EXPECT_FALSE(retired.fallback_attempted);
  EXPECT_TRUE(retired.zero_published);
  EXPECT_EQ(retired.state.output_publish_seq, 2U);
  EXPECT_EQ(retired.state.zero_publish_seq, 2U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(owner.header().claimed_slots),
    retired_claimed_slots);
}

TEST(MotionGateProcessRuntimeTest, DdsThrowLeavesIntentAndFallsBackOnce)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  OwnedJournalRegion owner(8U);
  MotionGateProcessRuntime runtime(
    MotionGateConfig{},
    gate_instance_id,
    GateEventJournalTestParameters{
        owner.name(), descriptor_for(owner)});
  select_nonzero_command(runtime, gate_instance_id);
  RecordingFinalPublisher publisher{&owner};
  publisher.failures_remaining = 1U;

  const auto result = runtime.publish_final_command(
    FinalOutputTime{true, 31, 32U}, publisher.adapter());

  ASSERT_EQ(publisher.frames.size(), 2U);
  EXPECT_FALSE(publisher.frames[0].command.is_zero());
  EXPECT_TRUE(publisher.frames[1].command.is_zero());
  EXPECT_EQ(result.failure, FinalOutputFailure::DdsFailure);
  EXPECT_TRUE(result.fallback_attempted);
  EXPECT_TRUE(result.published);
  EXPECT_TRUE(result.zero_published);
  EXPECT_FALSE(result.journal_committed);
  EXPECT_EQ(result.state.output_publish_seq, 1U);
  EXPECT_EQ(result.state.zero_publish_seq, 1U);
  EXPECT_EQ(result.locally_consumed_terminal_cause_seq, 4U);
  ASSERT_EQ(
    gate_event_journal_load_acquire(owner.header().claimed_slots),
    4U);
  const auto & output_intent = owner.slot(2U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(output_intent.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT);
  EXPECT_EQ(
    output_intent.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_OUTPUT_ATTEMPT);
  EXPECT_EQ(output_intent.output_attempt_seq, 1U);
  EXPECT_EQ(output_intent.intended_output_seq, 1U);
  const auto & fault = owner.slot(3U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(fault.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    fault.record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION);
  EXPECT_EQ(fault.event_code, 6U);
  EXPECT_EQ(
    fault.reason,
    static_cast<std::uint64_t>(Reason::PublishFailed));

  const auto retired = runtime.publish_final_command(
    FinalOutputTime{true, 33, 34U}, publisher.adapter());
  ASSERT_EQ(publisher.frames.size(), 3U);
  EXPECT_TRUE(publisher.frames[2].command.is_zero());
  EXPECT_EQ(retired.failure, FinalOutputFailure::DdsFailure);
  EXPECT_FALSE(retired.fallback_attempted);
  EXPECT_EQ(retired.locally_consumed_terminal_cause_seq, 0U);
  EXPECT_EQ(retired.state.output_publish_seq, 2U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(owner.header().claimed_slots),
    4U);
}

TEST(MotionGateProcessRuntimeTest, DoubleDdsFailureDoesNotRecurse)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  OwnedJournalRegion owner(8U);
  MotionGateProcessRuntime runtime(
    MotionGateConfig{},
    gate_instance_id,
    GateEventJournalTestParameters{
        owner.name(), descriptor_for(owner)});
  select_nonzero_command(runtime, gate_instance_id);
  RecordingFinalPublisher failing_publisher{&owner};
  failing_publisher.failures_remaining = 2U;

  const auto failed = runtime.publish_final_command(
    FinalOutputTime{true, 41, 42U}, failing_publisher.adapter());

  ASSERT_EQ(failing_publisher.frames.size(), 2U);
  EXPECT_FALSE(failing_publisher.frames[0].command.is_zero());
  EXPECT_TRUE(failing_publisher.frames[1].command.is_zero());
  EXPECT_EQ(failed.failure, FinalOutputFailure::DirectZeroDdsFailure);
  EXPECT_TRUE(failed.fallback_attempted);
  EXPECT_FALSE(failed.published);
  EXPECT_FALSE(failed.zero_published);
  EXPECT_EQ(failed.state.output_publish_seq, 0U);
  EXPECT_EQ(failed.state.zero_publish_seq, 0U);
  EXPECT_EQ(failed.locally_consumed_terminal_cause_seq, 0U);

  RecordingFinalPublisher recovery_publisher{&owner};
  const auto recovered = runtime.publish_final_command(
    FinalOutputTime{true, 43, 44U}, recovery_publisher.adapter());
  ASSERT_EQ(recovery_publisher.frames.size(), 1U);
  EXPECT_TRUE(recovery_publisher.frames[0].command.is_zero());
  EXPECT_EQ(recovered.failure, FinalOutputFailure::DdsFailure);
  EXPECT_FALSE(recovered.fallback_attempted);
  EXPECT_TRUE(recovered.zero_published);
  EXPECT_EQ(recovered.state.output_publish_seq, 1U);
  EXPECT_EQ(recovered.state.zero_publish_seq, 1U);
  EXPECT_EQ(recovered.locally_consumed_terminal_cause_seq, 4U);

  const auto repeated = runtime.publish_final_command(
    FinalOutputTime{true, 45, 46U}, recovery_publisher.adapter());
  EXPECT_EQ(repeated.locally_consumed_terminal_cause_seq, 0U);
  EXPECT_EQ(repeated.state.output_publish_seq, 2U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(owner.header().claimed_slots),
    4U);
}

TEST(MotionGateProcessRuntimeTest, DefaultOffPathPreservesOutputSemantics)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  MotionGateProcessRuntime runtime(
    MotionGateConfig{},
    gate_instance_id,
    GateEventJournalTestParameters{"", ""});
  select_nonzero_command(runtime, gate_instance_id);
  RecordingFinalPublisher publisher{nullptr};

  const auto first = runtime.publish_final_command(
    FinalOutputTime{true, 51, 52U}, publisher.adapter());
  ASSERT_EQ(publisher.frames.size(), 1U);
  EXPECT_FALSE(publisher.frames[0].command.is_zero());
  EXPECT_EQ(first.failure, FinalOutputFailure::None);
  EXPECT_TRUE(first.published);
  EXPECT_FALSE(first.zero_published);
  EXPECT_FALSE(first.journal_committed);
  EXPECT_EQ(first.state.output_publish_seq, 1U);
  EXPECT_EQ(first.state.zero_publish_seq, 0U);
  EXPECT_FALSE(first.state.last_publication_was_zero);

  publisher.failures_remaining = 1U;
  const auto recovered = runtime.publish_final_command(
    FinalOutputTime{true, 53, 54U}, publisher.adapter());
  ASSERT_EQ(publisher.frames.size(), 3U);
  EXPECT_FALSE(publisher.frames[1].command.is_zero());
  EXPECT_TRUE(publisher.frames[2].command.is_zero());
  EXPECT_EQ(recovered.failure, FinalOutputFailure::DdsFailure);
  EXPECT_TRUE(recovered.fallback_attempted);
  EXPECT_TRUE(recovered.zero_published);
  EXPECT_EQ(recovered.state.output_publish_seq, 2U);
  EXPECT_EQ(recovered.state.zero_publish_seq, 2U);
  EXPECT_TRUE(recovered.state.last_publication_was_zero);
  EXPECT_EQ(runtime.core().snapshot().state, State::Faulted);
  EXPECT_EQ(runtime.core().snapshot().reason, Reason::PublishFailed);
}

TEST(MotionGateProcessRuntimeTest, InactiveSimulationTimeCommitsStampedZero)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  OwnedJournalRegion owner(4U);
  MotionGateProcessRuntime runtime(
    MotionGateConfig{},
    gate_instance_id,
    GateEventJournalTestParameters{
        owner.name(), descriptor_for(owner)});
  RecordingFinalPublisher publisher{&owner};

  const auto result = runtime.publish_final_command(
    FinalOutputTime{false, 61, 62U}, publisher.adapter());

  ASSERT_EQ(publisher.frames.size(), 1U);
  EXPECT_TRUE(publisher.frames[0].command.is_zero());
  EXPECT_EQ(publisher.frames[0].stamp_sec, 0);
  EXPECT_EQ(publisher.frames[0].stamp_nanosec, 0U);
  EXPECT_EQ(result.failure, FinalOutputFailure::RuntimeInvariant);
  EXPECT_TRUE(result.published);
  EXPECT_TRUE(result.zero_published);
  EXPECT_TRUE(result.journal_committed);
  EXPECT_EQ(result.locally_consumed_terminal_cause_seq, 1U);
  EXPECT_EQ(runtime.core().snapshot().state, State::Faulted);
  EXPECT_EQ(
    runtime.core().snapshot().reason,
    Reason::ConfigurationInvalid);
  ASSERT_EQ(
    gate_event_journal_load_acquire(owner.header().claimed_slots),
    2U);
  const auto & output = owner.slot(1U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(output.phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(output.ros_stamp_sec_bits, 0U);
  EXPECT_EQ(output.ros_stamp_nanosec, 0U);
  EXPECT_EQ(output.cause_transition_journal_seq, 1U);
  EXPECT_EQ(
    output.reason,
    static_cast<std::uint64_t>(Reason::ConfigurationInvalid));
}

TEST(MotionGateProcessRuntimeTest, FaultRecordingFailureCannotVetoDirectZero)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  OwnedJournalRegion owner(2U);
  ThrowingFaultTestAdapter fault_adapter;
  MotionGateProcessRuntime runtime(
    MotionGateConfig{},
    gate_instance_id,
    GateEventJournalTestParameters{
        owner.name(), descriptor_for(owner)},
    fault_adapter.adapter());
  select_nonzero_command(runtime, gate_instance_id);
  RecordingFinalPublisher publisher{&owner};

  const auto result = runtime.publish_final_command(
    FinalOutputTime{true, 71, 72U}, publisher.adapter());

  EXPECT_EQ(fault_adapter.calls, 1U);
  ASSERT_EQ(publisher.frames.size(), 1U);
  EXPECT_TRUE(publisher.frames[0].command.is_zero());
  EXPECT_EQ(result.failure, FinalOutputFailure::JournalFailure);
  EXPECT_TRUE(result.fallback_attempted);
  EXPECT_TRUE(result.zero_published);
  EXPECT_EQ(result.state.output_publish_seq, 1U);
  EXPECT_EQ(runtime.core().snapshot().state, State::Armed);
  EXPECT_FALSE(runtime.core().selected_command().is_zero());

  const auto retired = runtime.publish_final_command(
    FinalOutputTime{true, 73, 74U}, publisher.adapter());
  EXPECT_EQ(fault_adapter.calls, 1U);
  ASSERT_EQ(publisher.frames.size(), 2U);
  EXPECT_TRUE(publisher.frames[1].command.is_zero());
  EXPECT_EQ(retired.failure, FinalOutputFailure::JournalFailure);
  EXPECT_FALSE(retired.fallback_attempted);
  EXPECT_EQ(retired.state.output_publish_seq, 2U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(owner.header().claimed_slots),
    2U);
}

TEST(MotionGateProcessRuntimeTest, SuccessSequenceBoundaryNeverWraps)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  MotionGateProcessRuntime runtime(
    MotionGateConfig{},
    gate_instance_id,
    GateEventJournalTestParameters{"", ""});
  select_nonzero_command(runtime, gate_instance_id);
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  MotionGateProcessRuntimeTestPeer::seed_output_sequences(
    runtime, 0U, maximum, maximum - 1U);
  RecordingFinalPublisher publisher{nullptr};

  const auto result = runtime.publish_final_command(
    FinalOutputTime{true, 81, 82U}, publisher.adapter());

  ASSERT_EQ(publisher.frames.size(), 1U);
  EXPECT_TRUE(publisher.frames[0].command.is_zero());
  EXPECT_EQ(result.failure, FinalOutputFailure::SequenceExhausted);
  EXPECT_TRUE(result.fallback_attempted);
  EXPECT_TRUE(result.zero_published);
  EXPECT_EQ(result.state.output_publish_seq, maximum);
  EXPECT_EQ(result.state.zero_publish_seq, maximum);
  EXPECT_EQ(runtime.core().snapshot().state, State::Faulted);
  EXPECT_EQ(runtime.core().snapshot().reason, Reason::SequenceExhausted);
}

TEST(MotionGateProcessRuntimeTest, AttemptSequenceBoundaryRetiresJournal)
{
  const std::string gate_instance_id =
    "0123456789abcdef0123456789abcdef";
  OwnedJournalRegion owner(8U);
  MotionGateProcessRuntime runtime(
    MotionGateConfig{},
    gate_instance_id,
    GateEventJournalTestParameters{
        owner.name(), descriptor_for(owner)});
  select_nonzero_command(runtime, gate_instance_id);
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  MotionGateProcessRuntimeTestPeer::seed_output_sequences(
    runtime, maximum, 7U, 6U);
  RecordingFinalPublisher publisher{&owner};

  const auto result = runtime.publish_final_command(
    FinalOutputTime{true, 83, 84U}, publisher.adapter());

  ASSERT_EQ(publisher.frames.size(), 1U);
  EXPECT_TRUE(publisher.frames[0].command.is_zero());
  EXPECT_EQ(result.failure, FinalOutputFailure::SequenceExhausted);
  EXPECT_TRUE(result.fallback_attempted);
  EXPECT_TRUE(result.zero_published);
  EXPECT_EQ(result.state.output_publish_seq, 8U);
  EXPECT_EQ(result.state.zero_publish_seq, 8U);
  EXPECT_EQ(result.locally_consumed_terminal_cause_seq, 3U);
  EXPECT_EQ(
    MotionGateProcessRuntimeTestPeer::output_attempt_sequence(runtime),
    maximum);
  ASSERT_EQ(
    gate_event_journal_load_acquire(owner.header().claimed_slots),
    3U);
  EXPECT_EQ(
    owner.slot(2U).record_kind,
    VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION);
  EXPECT_EQ(owner.slot(2U).event_code, 6U);

  const auto retired = runtime.publish_final_command(
    FinalOutputTime{true, 85, 86U}, publisher.adapter());
  EXPECT_EQ(retired.failure, FinalOutputFailure::SequenceExhausted);
  EXPECT_FALSE(retired.fallback_attempted);
  EXPECT_EQ(retired.state.output_publish_seq, 9U);
  EXPECT_EQ(retired.locally_consumed_terminal_cause_seq, 0U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(owner.header().claimed_slots),
    3U);
}

}  // namespace
}  // namespace voice_nav_mission
