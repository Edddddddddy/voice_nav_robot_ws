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

#include <cstdint>
#include <memory>
#include <string>

namespace voice_nav_sim
{

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

struct HardwareWriteLedgerAttachmentConfig
{
  std::string shared_memory_name;
  HardwareWriteLedgerIdentity expected_identity;
  HardwareWriteLedgerLayout expected_layout;
};

class AttachedHardwareWriteLedger
{
public:
  explicit AttachedHardwareWriteLedger(
    HardwareWriteLedgerAttachmentConfig config);
  ~AttachedHardwareWriteLedger();

  AttachedHardwareWriteLedger(const AttachedHardwareWriteLedger &) = delete;
  AttachedHardwareWriteLedger & operator=(
    const AttachedHardwareWriteLedger &) = delete;
  AttachedHardwareWriteLedger(AttachedHardwareWriteLedger &&) = delete;
  AttachedHardwareWriteLedger & operator=(
    AttachedHardwareWriteLedger &&) = delete;

  [[nodiscard]] std::uint64_t claimed_writer_pid() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace voice_nav_sim

#endif  // VOICE_NAV_SIM__ATTACHED_HARDWARE_WRITE_LEDGER_HPP_
