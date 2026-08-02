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
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "gate_event_journal_test_region.hpp"
#include "motion_gate_process_runtime.hpp"

namespace voice_nav_mission
{
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

}  // namespace
}  // namespace voice_nav_mission
