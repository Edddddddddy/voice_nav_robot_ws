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

#ifndef VOICE_NAV_SIM__HARDWARE_WRITE_LEDGER_WRITER_HPP_
#define VOICE_NAV_SIM__HARDWARE_WRITE_LEDGER_WRITER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>

namespace voice_nav_sim
{

inline constexpr std::uint64_t kHardwareWriteDelegatedException{0x100U};

struct HardwareWriteLedgerIdentity
{
  std::uint64_t owner_uid;
  std::uint64_t generation;
  std::uint64_t nonce_hi;
  std::uint64_t nonce_lo;
};

struct HardwareWriteLedgerLayout
{
  std::uint64_t segment_capacity_per_bank;
  std::uint64_t page_segment_limit;
};

enum class HardwareWriteObservationStatus : std::uint64_t
{
  kValid = 0U,
  kMissingEntity = 1U,
  kMissingComponent = 2U,
  kEmptyComponent = 3U,
  kInspectionFailure = 4U,
};

struct HardwareWriteWheelObservation
{
  HardwareWriteObservationStatus status;
  std::uint64_t left_command_bits;
  std::uint64_t right_command_bits;
};

struct HardwareWriteTicket
{
  std::uint64_t write_seq;
  std::int64_t sim_stamp_ns;
  std::uint64_t bank_index;
  std::uint64_t bank_epoch;
  bool included;
};

class HardwareWriteJournal
{
public:
  virtual ~HardwareWriteJournal() = default;

  [[nodiscard]] virtual HardwareWriteTicket begin_write(
    std::int64_t sim_stamp_ns) noexcept = 0;

  virtual void finish_write(
    HardwareWriteTicket ticket,
    std::uint64_t delegated_result,
    HardwareWriteWheelObservation observation) noexcept = 0;
};

class HardwareWriteLedgerWriter final : public HardwareWriteJournal
{
public:
  HardwareWriteLedgerWriter(void * region, std::size_t region_bytes);
  ~HardwareWriteLedgerWriter();

  HardwareWriteLedgerWriter(const HardwareWriteLedgerWriter &) = delete;
  HardwareWriteLedgerWriter & operator=(
    const HardwareWriteLedgerWriter &) = delete;
  HardwareWriteLedgerWriter(HardwareWriteLedgerWriter &&) = delete;
  HardwareWriteLedgerWriter & operator=(
    HardwareWriteLedgerWriter &&) = delete;

  [[nodiscard]] HardwareWriteTicket begin_write(
    std::int64_t sim_stamp_ns) noexcept override;

  void finish_write(
    HardwareWriteTicket ticket,
    std::uint64_t delegated_result,
    HardwareWriteWheelObservation observation) noexcept override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace voice_nav_sim

#endif  // VOICE_NAV_SIM__HARDWARE_WRITE_LEDGER_WRITER_HPP_
