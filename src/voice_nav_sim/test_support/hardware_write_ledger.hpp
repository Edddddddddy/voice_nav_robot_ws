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

#ifndef VOICE_NAV_SIM__HARDWARE_WRITE_LEDGER_HPP_
#define VOICE_NAV_SIM__HARDWARE_WRITE_LEDGER_HPP_

#include "hardware_write_sink.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace voice_nav_sim
{

struct HardwareWriteLedgerConfig
{
  std::uint64_t generation;
  std::uint64_t interval_id;
  std::uint64_t arm_fence_write_seq;
  std::size_t segment_capacity;
  std::size_t snapshot_page_segment_limit;
};

struct HardwareWriteSegment
{
  std::uint64_t generation;
  std::uint64_t first_write_seq;
  std::uint64_t last_write_seq;
  std::uint64_t invocation_count;
  std::int64_t sim_stamp_ns;
  std::uint8_t delegated_result;
  std::uint64_t left_command_bits;
  std::uint64_t right_command_bits;
};

struct HardwareWriteSnapshotPage
{
  std::uint64_t generation;
  std::uint64_t interval_id;
  std::uint64_t arm_fence_write_seq;
  std::uint64_t seal_fence_write_seq;
  std::uint64_t page_index;
  std::uint64_t page_count;
  std::uint64_t total_segment_count;
  std::uint64_t total_invocation_count;
  std::uint64_t page_segment_count;
  std::uint64_t page_invocation_count;
  std::uint64_t page_first_write_seq;
  std::uint64_t page_last_write_seq;
  std::uint64_t previous_page_checksum;
  std::uint64_t oracle_faults;
  std::uint64_t page_checksum;
  std::vector<HardwareWriteSegment> segments;
};

// One writer owns append() and seal(). Readers may call snapshot_page() only
// after observing a sealed page. The page checksum is CRC64-ECMA-182 over the
// metadata fields before page_checksum, followed by each segment field, with
// every integer encoded as eight least-significant-first bytes.
class HardwareWriteLedger final : public HardwareWriteSink
{
public:
  explicit HardwareWriteLedger(HardwareWriteLedgerConfig config);
  ~HardwareWriteLedger() override;

  HardwareWriteLedger(const HardwareWriteLedger &) = delete;
  HardwareWriteLedger & operator=(const HardwareWriteLedger &) = delete;
  HardwareWriteLedger(HardwareWriteLedger &&) = delete;
  HardwareWriteLedger & operator=(HardwareWriteLedger &&) = delete;

  bool append(const HardwareWriteRecord & record) noexcept override;
  bool seal() noexcept;

  [[nodiscard]] std::optional<HardwareWriteSnapshotPage> snapshot_page(
    std::uint64_t page_index) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace voice_nav_sim

#endif  // VOICE_NAV_SIM__HARDWARE_WRITE_LEDGER_HPP_
