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

#include <cstdint>
#include <string>
#include <vector>

#include "motion_gate_process_runtime.hpp"

namespace voice_nav_mission
{
namespace
{

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
    {name, "v1:" + uid + ":7:16"},
    {name, "v1:" + uid + ":7:16:" + nonce + ":extra"},
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
    {name, "v1:" + uid + ":7:16:00000000000000000000000000000000"},
    {name, "v1:" + uid + ":7:16:123456789ABCDEF00fedcba987654321"},
    {name, "v1:" + uid + ":7:16:123456789abcdef00fedcba98765432g"},
    {"/voice_nav_gate_00112233445566778899aabbccddeef", descriptor},
    {"/voice_nav_other_00112233445566778899aabbccddeeff", descriptor},
    {"/voice_nav_gate_00112233445566778899AABBCCDDEEFF", descriptor},
  };

  for (const auto & parameters : invalid) {
    SCOPED_TRACE(parameters.name + " | " + parameters.descriptor);
    EXPECT_THROW(
      static_cast<void>(
        parse_gate_event_journal_test_parameters(parameters)),
      std::invalid_argument);
  }
}

}  // namespace
}  // namespace voice_nav_mission
