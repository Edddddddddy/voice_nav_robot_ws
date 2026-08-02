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

#include "hardware_write_ledger.hpp"

#include <cstdint>

namespace
{

constexpr std::uint64_t kCrc64EcmaPolynomial =
  UINT64_C(0x42f0e1eba9ea3693);

std::uint64_t crc64_byte(
  std::uint64_t checksum,
  std::uint8_t byte)
{
  checksum ^= static_cast<std::uint64_t>(byte) << 56U;
  for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
    const bool high_bit_set =
      (checksum & UINT64_C(0x8000000000000000)) != 0U;
    checksum <<= 1U;
    if (high_bit_set) {
      checksum ^= kCrc64EcmaPolynomial;
    }
  }
  return checksum;
}

std::uint64_t crc64_word(
  std::uint64_t checksum,
  std::uint64_t value)
{
  for (std::uint8_t byte_index = 0U; byte_index < 8U; ++byte_index) {
    checksum = crc64_byte(
      checksum,
      static_cast<std::uint8_t>(value & UINT64_C(0xff)));
    value >>= 8U;
  }
  return checksum;
}

std::uint64_t independent_page_checksum(
  const voice_nav_sim::HardwareWriteSnapshotPage & page)
{
  std::uint64_t checksum = 0U;
  const std::uint64_t metadata[] = {
    page.generation,
    page.interval_id,
    page.arm_fence_write_seq,
    page.seal_fence_write_seq,
    page.page_index,
    page.page_count,
    page.total_segment_count,
    page.total_invocation_count,
    page.page_segment_count,
    page.page_invocation_count,
    page.page_first_write_seq,
    page.page_last_write_seq,
    page.previous_page_checksum,
    page.oracle_faults};
  for (const auto word : metadata) {
    checksum = crc64_word(checksum, word);
  }
  for (const auto & segment : page.segments) {
    const std::uint64_t words[] = {
      segment.generation,
      segment.first_write_seq,
      segment.last_write_seq,
      segment.invocation_count,
      static_cast<std::uint64_t>(segment.sim_stamp_ns),
      static_cast<std::uint64_t>(segment.delegated_result),
      segment.left_command_bits,
      segment.right_command_bits};
    for (const auto word : words) {
      checksum = crc64_word(checksum, word);
    }
  }
  return checksum;
}

void expect_same_page(
  const voice_nav_sim::HardwareWriteSnapshotPage & actual,
  const voice_nav_sim::HardwareWriteSnapshotPage & expected)
{
  EXPECT_EQ(actual.generation, expected.generation);
  EXPECT_EQ(actual.interval_id, expected.interval_id);
  EXPECT_EQ(actual.arm_fence_write_seq, expected.arm_fence_write_seq);
  EXPECT_EQ(actual.seal_fence_write_seq, expected.seal_fence_write_seq);
  EXPECT_EQ(actual.page_index, expected.page_index);
  EXPECT_EQ(actual.page_count, expected.page_count);
  EXPECT_EQ(actual.total_segment_count, expected.total_segment_count);
  EXPECT_EQ(actual.total_invocation_count, expected.total_invocation_count);
  EXPECT_EQ(actual.page_segment_count, expected.page_segment_count);
  EXPECT_EQ(actual.page_invocation_count, expected.page_invocation_count);
  EXPECT_EQ(actual.page_first_write_seq, expected.page_first_write_seq);
  EXPECT_EQ(actual.page_last_write_seq, expected.page_last_write_seq);
  EXPECT_EQ(actual.previous_page_checksum, expected.previous_page_checksum);
  EXPECT_EQ(actual.oracle_faults, expected.oracle_faults);
  EXPECT_EQ(actual.page_checksum, expected.page_checksum);
  ASSERT_EQ(actual.segments.size(), expected.segments.size());
  for (std::size_t index = 0U; index < actual.segments.size(); ++index) {
    const auto & actual_segment = actual.segments[index];
    const auto & expected_segment = expected.segments[index];
    EXPECT_EQ(actual_segment.generation, expected_segment.generation);
    EXPECT_EQ(
      actual_segment.first_write_seq,
      expected_segment.first_write_seq);
    EXPECT_EQ(
      actual_segment.last_write_seq,
      expected_segment.last_write_seq);
    EXPECT_EQ(
      actual_segment.invocation_count,
      expected_segment.invocation_count);
    EXPECT_EQ(actual_segment.sim_stamp_ns, expected_segment.sim_stamp_ns);
    EXPECT_EQ(
      actual_segment.delegated_result,
      expected_segment.delegated_result);
    EXPECT_EQ(
      actual_segment.left_command_bits,
      expected_segment.left_command_bits);
    EXPECT_EQ(
      actual_segment.right_command_bits,
      expected_segment.right_command_bits);
  }
}

TEST(HardwareWriteLedger, SealsOneWriteAsAnImmutableChecksummedPage)
{
  voice_nav_sim::HardwareWriteLedger ledger({41U, 7U, 0U, 4U, 4U});
  EXPECT_FALSE(ledger.snapshot_page(0U).has_value());

  const voice_nav_sim::HardwareWriteRecord record{
    41U,
    1U,
    1'234'567,
    1U,
    UINT64_C(0xbff4000000000000),
    UINT64_C(0x4004000000000000)};
  ASSERT_TRUE(ledger.append(record));
  ASSERT_TRUE(ledger.seal());

  const auto first_snapshot = ledger.snapshot_page(0U);
  ASSERT_TRUE(first_snapshot.has_value());
  EXPECT_FALSE(ledger.snapshot_page(1U).has_value());
  EXPECT_EQ(first_snapshot->generation, 41U);
  EXPECT_EQ(first_snapshot->interval_id, 7U);
  EXPECT_EQ(first_snapshot->arm_fence_write_seq, 0U);
  EXPECT_EQ(first_snapshot->seal_fence_write_seq, 1U);
  EXPECT_EQ(first_snapshot->page_index, 0U);
  EXPECT_EQ(first_snapshot->page_count, 1U);
  EXPECT_EQ(first_snapshot->total_segment_count, 1U);
  EXPECT_EQ(first_snapshot->total_invocation_count, 1U);
  EXPECT_EQ(first_snapshot->page_segment_count, 1U);
  EXPECT_EQ(first_snapshot->page_invocation_count, 1U);
  EXPECT_EQ(first_snapshot->page_first_write_seq, 1U);
  EXPECT_EQ(first_snapshot->page_last_write_seq, 1U);
  EXPECT_EQ(first_snapshot->previous_page_checksum, 0U);
  EXPECT_EQ(first_snapshot->oracle_faults, 0U);
  ASSERT_EQ(first_snapshot->segments.size(), 1U);
  const auto & segment = first_snapshot->segments.front();
  EXPECT_EQ(segment.generation, record.generation);
  EXPECT_EQ(segment.first_write_seq, record.write_seq);
  EXPECT_EQ(segment.last_write_seq, record.write_seq);
  EXPECT_EQ(segment.invocation_count, 1U);
  EXPECT_EQ(segment.sim_stamp_ns, record.sim_stamp_ns);
  EXPECT_EQ(segment.delegated_result, record.delegated_result);
  EXPECT_EQ(segment.left_command_bits, record.left_command_bits);
  EXPECT_EQ(segment.right_command_bits, record.right_command_bits);
  EXPECT_EQ(
    first_snapshot->page_checksum,
    independent_page_checksum(*first_snapshot));

  const auto repeated_snapshot = ledger.snapshot_page(0U);
  ASSERT_TRUE(repeated_snapshot.has_value());
  expect_same_page(*repeated_snapshot, *first_snapshot);
}

}  // namespace
