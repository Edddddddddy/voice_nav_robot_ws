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

#include "hardware_write_ledger.hpp"

#include <algorithm>
#include <atomic>
#include <limits>
#include <stdexcept>
#include <utility>

namespace voice_nav_sim
{
namespace
{

constexpr std::uint64_t kCrc64EcmaPolynomial =
  UINT64_C(0x42f0e1eba9ea3693);

std::uint64_t crc64_byte(
  std::uint64_t checksum,
  std::uint8_t byte) noexcept
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
  std::uint64_t value) noexcept
{
  for (std::uint8_t byte_index = 0U; byte_index < 8U; ++byte_index) {
    checksum = crc64_byte(
      checksum,
      static_cast<std::uint8_t>(value & UINT64_C(0xff)));
    value >>= 8U;
  }
  return checksum;
}

std::uint64_t page_checksum(
  const HardwareWriteSnapshotPage & page) noexcept
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

}  // namespace

class HardwareWriteLedger::Impl
{
public:
  explicit Impl(HardwareWriteLedgerConfig ledger_config)
  : config(std::move(ledger_config))
  {
    if (
      config.generation == 0U || config.interval_id == 0U ||
      config.arm_fence_write_seq == std::numeric_limits<std::uint64_t>::max() ||
      config.segment_capacity == 0U ||
      config.snapshot_page_segment_limit == 0U ||
      config.snapshot_page_segment_limit > config.segment_capacity)
    {
      throw std::invalid_argument("invalid HardwareWriteLedger config");
    }
    finalized_segments =
      std::make_unique<HardwareWriteSegment[]>(config.segment_capacity);
  }

  HardwareWriteLedgerConfig config;
  std::unique_ptr<HardwareWriteSegment[]> finalized_segments;
  std::optional<HardwareWriteSegment> active_segment;
  std::size_t finalized_segment_count{0U};
  std::uint64_t total_invocation_count{0U};
  std::uint64_t seal_fence_write_seq{0U};
  std::atomic_bool sealed{false};
};

HardwareWriteLedger::HardwareWriteLedger(HardwareWriteLedgerConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

HardwareWriteLedger::~HardwareWriteLedger() = default;

bool HardwareWriteLedger::append(
  const HardwareWriteRecord & record) noexcept
{
  if (impl_->sealed.load(std::memory_order_acquire)) {
    return false;
  }
  if (record.generation != impl_->config.generation) {
    return false;
  }

  if (impl_->active_segment.has_value()) {
    auto & active = *impl_->active_segment;
    if (
      active.last_write_seq == std::numeric_limits<std::uint64_t>::max() ||
      impl_->total_invocation_count ==
      std::numeric_limits<std::uint64_t>::max() ||
      record.write_seq != active.last_write_seq + 1U ||
      record.sim_stamp_ns != active.sim_stamp_ns ||
      record.delegated_result != active.delegated_result ||
      record.left_command_bits != active.left_command_bits ||
      record.right_command_bits != active.right_command_bits)
    {
      return false;
    }
    active.last_write_seq = record.write_seq;
    ++active.invocation_count;
    ++impl_->total_invocation_count;
    return true;
  }

  if (record.write_seq != impl_->config.arm_fence_write_seq + 1U) {
    return false;
  }

  impl_->active_segment = HardwareWriteSegment{
    record.generation,
    record.write_seq,
    record.write_seq,
    1U,
    record.sim_stamp_ns,
    record.delegated_result,
    record.left_command_bits,
    record.right_command_bits};
  impl_->total_invocation_count = 1U;
  return true;
}

bool HardwareWriteLedger::seal() noexcept
{
  if (
    impl_->sealed.load(std::memory_order_acquire) ||
    !impl_->active_segment.has_value() ||
    impl_->finalized_segment_count >= impl_->config.segment_capacity)
  {
    return false;
  }

  impl_->finalized_segments[impl_->finalized_segment_count] =
    *impl_->active_segment;
  ++impl_->finalized_segment_count;
  impl_->seal_fence_write_seq = impl_->active_segment->last_write_seq;
  impl_->active_segment.reset();
  impl_->sealed.store(true, std::memory_order_release);
  return true;
}

std::optional<HardwareWriteSnapshotPage> HardwareWriteLedger::snapshot_page(
  std::uint64_t page_index) const
{
  if (
    !impl_->sealed.load(std::memory_order_acquire) || page_index != 0U ||
    impl_->finalized_segment_count == 0U)
  {
    return std::nullopt;
  }

  HardwareWriteSnapshotPage page{
    impl_->config.generation,
    impl_->config.interval_id,
    impl_->config.arm_fence_write_seq,
    impl_->seal_fence_write_seq,
    0U,
    1U,
    impl_->finalized_segment_count,
    impl_->total_invocation_count,
    impl_->finalized_segment_count,
    impl_->total_invocation_count,
    impl_->finalized_segments[0U].first_write_seq,
    impl_->finalized_segments[impl_->finalized_segment_count - 1U].last_write_seq,
    0U,
    0U,
    0U,
    {}};
  page.segments.reserve(impl_->finalized_segment_count);
  for (std::size_t index = 0U;
    index < impl_->finalized_segment_count;
    ++index)
  {
    page.segments.push_back(impl_->finalized_segments[index]);
  }
  page.page_checksum = page_checksum(page);
  return page;
}

}  // namespace voice_nav_sim
