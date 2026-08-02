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
  EXPECT_EQ(
    first_snapshot->page_checksum,
    UINT64_C(0x923c0457d54721c3));

  auto corrupted_snapshot = *first_snapshot;
  corrupted_snapshot.interval_id ^= 1U;
  EXPECT_NE(
    corrupted_snapshot.page_checksum,
    independent_page_checksum(corrupted_snapshot));

  const auto repeated_snapshot = ledger.snapshot_page(0U);
  ASSERT_TRUE(repeated_snapshot.has_value());
  expect_same_page(*repeated_snapshot, *first_snapshot);
}

TEST(HardwareWriteLedger, FoldsIdenticalConsecutiveWritesWithoutLosingCount)
{
  voice_nav_sim::HardwareWriteLedger ledger({41U, 8U, 10U, 2U, 2U});
  const voice_nav_sim::HardwareWriteRecord first{
    41U,
    11U,
    2'000'000,
    0U,
    UINT64_C(0x3ff0000000000000),
    UINT64_C(0xbff0000000000000)};
  auto second = first;
  second.write_seq = 12U;

  ASSERT_TRUE(ledger.append(first));
  ASSERT_TRUE(ledger.append(second));
  ASSERT_TRUE(ledger.seal());

  const auto snapshot = ledger.snapshot_page(0U);
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->arm_fence_write_seq, 10U);
  EXPECT_EQ(snapshot->seal_fence_write_seq, 12U);
  EXPECT_EQ(snapshot->total_segment_count, 1U);
  EXPECT_EQ(snapshot->total_invocation_count, 2U);
  EXPECT_EQ(snapshot->page_segment_count, 1U);
  EXPECT_EQ(snapshot->page_invocation_count, 2U);
  EXPECT_EQ(snapshot->page_first_write_seq, 11U);
  EXPECT_EQ(snapshot->page_last_write_seq, 12U);
  ASSERT_EQ(snapshot->segments.size(), 1U);
  const auto & segment = snapshot->segments.front();
  EXPECT_EQ(segment.first_write_seq, 11U);
  EXPECT_EQ(segment.last_write_seq, 12U);
  EXPECT_EQ(segment.invocation_count, 2U);
  EXPECT_EQ(
    segment.invocation_count,
    segment.last_write_seq - segment.first_write_seq + 1U);
  EXPECT_EQ(snapshot->page_checksum, independent_page_checksum(*snapshot));
  EXPECT_EQ(snapshot->page_checksum, UINT64_C(0x20e6650a8c19bb32));
}

TEST(HardwareWriteLedger, FinalizesASegmentWhenTheWriteTupleChanges)
{
  voice_nav_sim::HardwareWriteLedger ledger({42U, 9U, 0U, 2U, 2U});
  const voice_nav_sim::HardwareWriteRecord first{
    42U,
    1U,
    3'000'000,
    0U,
    UINT64_C(0x3fe0000000000000),
    UINT64_C(0x3fe0000000000000)};
  const voice_nav_sim::HardwareWriteRecord second{
    42U,
    2U,
    3'010'000,
    1U,
    UINT64_C(0x0000000000000000),
    UINT64_C(0x0000000000000000)};

  ASSERT_TRUE(ledger.append(first));
  ASSERT_TRUE(ledger.append(second));
  ASSERT_TRUE(ledger.seal());

  const auto snapshot = ledger.snapshot_page(0U);
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->seal_fence_write_seq, 2U);
  EXPECT_EQ(snapshot->total_segment_count, 2U);
  EXPECT_EQ(snapshot->total_invocation_count, 2U);
  EXPECT_EQ(snapshot->page_segment_count, 2U);
  EXPECT_EQ(snapshot->page_invocation_count, 2U);
  EXPECT_EQ(snapshot->page_first_write_seq, 1U);
  EXPECT_EQ(snapshot->page_last_write_seq, 2U);
  ASSERT_EQ(snapshot->segments.size(), 2U);
  EXPECT_EQ(snapshot->segments[0U].first_write_seq, 1U);
  EXPECT_EQ(snapshot->segments[0U].last_write_seq, 1U);
  EXPECT_EQ(snapshot->segments[0U].invocation_count, 1U);
  EXPECT_EQ(snapshot->segments[1U].first_write_seq, 2U);
  EXPECT_EQ(snapshot->segments[1U].last_write_seq, 2U);
  EXPECT_EQ(snapshot->segments[1U].invocation_count, 1U);
  EXPECT_EQ(snapshot->page_checksum, independent_page_checksum(*snapshot));
}

TEST(HardwareWriteLedger, PaginatesSealedSegmentsWithAChecksumChain)
{
  voice_nav_sim::HardwareWriteLedger ledger({43U, 10U, 0U, 2U, 1U});
  const voice_nav_sim::HardwareWriteRecord first{
    43U, 1U, 4'000'000, 0U, UINT64_C(1), UINT64_C(2)};
  const voice_nav_sim::HardwareWriteRecord second{
    43U, 2U, 4'010'000, 0U, UINT64_C(3), UINT64_C(4)};
  ASSERT_TRUE(ledger.append(first));
  ASSERT_TRUE(ledger.append(second));
  ASSERT_TRUE(ledger.seal());

  const auto first_page = ledger.snapshot_page(0U);
  const auto second_page = ledger.snapshot_page(1U);
  ASSERT_TRUE(first_page.has_value());
  ASSERT_TRUE(second_page.has_value());
  EXPECT_FALSE(ledger.snapshot_page(2U).has_value());
  EXPECT_EQ(first_page->page_index, 0U);
  EXPECT_EQ(second_page->page_index, 1U);
  EXPECT_EQ(first_page->page_count, 2U);
  EXPECT_EQ(second_page->page_count, 2U);
  EXPECT_EQ(first_page->total_segment_count, 2U);
  EXPECT_EQ(second_page->total_segment_count, 2U);
  EXPECT_EQ(first_page->total_invocation_count, 2U);
  EXPECT_EQ(second_page->total_invocation_count, 2U);
  EXPECT_EQ(first_page->page_segment_count, 1U);
  EXPECT_EQ(second_page->page_segment_count, 1U);
  EXPECT_EQ(first_page->page_invocation_count, 1U);
  EXPECT_EQ(second_page->page_invocation_count, 1U);
  EXPECT_EQ(first_page->page_first_write_seq, 1U);
  EXPECT_EQ(first_page->page_last_write_seq, 1U);
  EXPECT_EQ(second_page->page_first_write_seq, 2U);
  EXPECT_EQ(second_page->page_last_write_seq, 2U);
  EXPECT_EQ(first_page->previous_page_checksum, 0U);
  EXPECT_EQ(
    second_page->previous_page_checksum,
    first_page->page_checksum);
  ASSERT_EQ(first_page->segments.size(), 1U);
  ASSERT_EQ(second_page->segments.size(), 1U);
  EXPECT_EQ(first_page->segments.front().first_write_seq, 1U);
  EXPECT_EQ(second_page->segments.front().first_write_seq, 2U);
  EXPECT_EQ(
    first_page->page_checksum,
    independent_page_checksum(*first_page));
  EXPECT_EQ(
    second_page->page_checksum,
    independent_page_checksum(*second_page));
}

TEST(HardwareWriteLedger, LatchesASequenceFaultForADuplicateWrite)
{
  voice_nav_sim::HardwareWriteLedger ledger({44U, 11U, 0U, 2U, 2U});
  const voice_nav_sim::HardwareWriteRecord first{
    44U, 1U, 5'000'000, 0U, UINT64_C(5), UINT64_C(6)};
  ASSERT_TRUE(ledger.append(first));

  EXPECT_FALSE(ledger.append(first));
  EXPECT_EQ(
    ledger.oracle_faults(),
    voice_nav_sim::kHardwareWriteOracleFaultSequence);
  ASSERT_TRUE(ledger.seal());

  const auto snapshot = ledger.snapshot_page(0U);
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(
    snapshot->oracle_faults,
    voice_nav_sim::kHardwareWriteOracleFaultSequence);
  EXPECT_EQ(snapshot->total_invocation_count, 1U);
  ASSERT_EQ(snapshot->segments.size(), 1U);
  EXPECT_EQ(snapshot->segments.front().first_write_seq, 1U);
  EXPECT_EQ(snapshot->segments.front().last_write_seq, 1U);
  EXPECT_EQ(snapshot->segments.front().invocation_count, 1U);
  EXPECT_EQ(snapshot->page_checksum, independent_page_checksum(*snapshot));
}

TEST(HardwareWriteLedger, LatchesAStickyGenerationFault)
{
  voice_nav_sim::HardwareWriteLedger ledger({45U, 12U, 0U, 1U, 1U});
  const voice_nav_sim::HardwareWriteRecord stale{
    46U, 1U, 6'000'000, 0U, UINT64_C(7), UINT64_C(8)};
  EXPECT_FALSE(ledger.append(stale));
  EXPECT_EQ(
    ledger.oracle_faults(),
    voice_nav_sim::kHardwareWriteOracleFaultGeneration);

  auto current = stale;
  current.generation = 45U;
  ASSERT_TRUE(ledger.append(current));
  ASSERT_TRUE(ledger.seal());
  const auto snapshot = ledger.snapshot_page(0U);
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(
    snapshot->oracle_faults,
    voice_nav_sim::kHardwareWriteOracleFaultGeneration);
  EXPECT_EQ(snapshot->page_checksum, independent_page_checksum(*snapshot));
}

TEST(HardwareWriteLedger, LatchesANonFiniteWheelCommandFault)
{
  voice_nav_sim::HardwareWriteLedger ledger({47U, 13U, 0U, 1U, 1U});
  const voice_nav_sim::HardwareWriteRecord non_finite{
    47U,
    1U,
    7'000'000,
    0U,
    UINT64_C(0x7ff0000000000000),
    UINT64_C(0x0000000000000000)};

  EXPECT_FALSE(ledger.append(non_finite));
  EXPECT_EQ(
    ledger.oracle_faults(),
    voice_nav_sim::kHardwareWriteOracleFaultNonFiniteCommand);

  auto finite = non_finite;
  finite.left_command_bits = UINT64_C(0x3ff0000000000000);
  ASSERT_TRUE(ledger.append(finite));
  ASSERT_TRUE(ledger.seal());
  const auto snapshot = ledger.snapshot_page(0U);
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(
    snapshot->oracle_faults,
    voice_nav_sim::kHardwareWriteOracleFaultNonFiniteCommand);
  EXPECT_EQ(snapshot->page_checksum, independent_page_checksum(*snapshot));
}

TEST(HardwareWriteLedger, LatchesASimulationStampRegression)
{
  voice_nav_sim::HardwareWriteLedger ledger({48U, 14U, 0U, 2U, 2U});
  const voice_nav_sim::HardwareWriteRecord first{
    48U, 1U, 8'000'000, 0U, UINT64_C(9), UINT64_C(10)};
  const voice_nav_sim::HardwareWriteRecord regressed{
    48U, 2U, 7'999'999, 0U, UINT64_C(11), UINT64_C(12)};
  ASSERT_TRUE(ledger.append(first));

  EXPECT_FALSE(ledger.append(regressed));
  EXPECT_EQ(
    ledger.oracle_faults(),
    voice_nav_sim::kHardwareWriteOracleFaultSimulationStamp);

  auto nondecreasing = regressed;
  nondecreasing.sim_stamp_ns = first.sim_stamp_ns;
  ASSERT_TRUE(ledger.append(nondecreasing));
  ASSERT_TRUE(ledger.seal());
  const auto snapshot = ledger.snapshot_page(0U);
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(
    snapshot->oracle_faults,
    voice_nav_sim::kHardwareWriteOracleFaultSimulationStamp);
  ASSERT_EQ(snapshot->segments.size(), 2U);
  EXPECT_EQ(
    snapshot->segments[0U].sim_stamp_ns,
    snapshot->segments[1U].sim_stamp_ns);
  EXPECT_EQ(snapshot->page_checksum, independent_page_checksum(*snapshot));
}

}  // namespace
