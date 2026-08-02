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

#ifndef VOICE_NAV_SIM__ATTACHED_HARDWARE_WRITE_LEDGER_HPP_
#define VOICE_NAV_SIM__ATTACHED_HARDWARE_WRITE_LEDGER_HPP_

#include "hardware_write_ledger_writer.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace voice_nav_sim
{

struct HardwareWriteLedgerAttachmentConfig
{
  std::string shared_memory_name;
  HardwareWriteLedgerIdentity expected_identity;
  HardwareWriteLedgerLayout expected_layout;
};

struct HardwareWriteLedgerDiscoveryConfig
{
  std::string shared_memory_name;
  std::string expected_nonce;
};

class AttachedHardwareWriteLedger final : public HardwareWriteJournal
{
public:
  explicit AttachedHardwareWriteLedger(
    HardwareWriteLedgerAttachmentConfig config);
  explicit AttachedHardwareWriteLedger(
    HardwareWriteLedgerDiscoveryConfig config);
  ~AttachedHardwareWriteLedger();

  AttachedHardwareWriteLedger(const AttachedHardwareWriteLedger &) = delete;
  AttachedHardwareWriteLedger & operator=(
    const AttachedHardwareWriteLedger &) = delete;
  AttachedHardwareWriteLedger(AttachedHardwareWriteLedger &&) = delete;
  AttachedHardwareWriteLedger & operator=(
    AttachedHardwareWriteLedger &&) = delete;

  [[nodiscard]] std::uint64_t claimed_writer_pid() const noexcept;
  [[nodiscard]] HardwareWriteLedgerWriter & writer() noexcept;

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

#endif  // VOICE_NAV_SIM__ATTACHED_HARDWARE_WRITE_LEDGER_HPP_
