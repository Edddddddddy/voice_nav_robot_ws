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

#ifndef GATE_EVENT_JOURNAL_TEST_REGION_HPP_
#define GATE_EVENT_JOURNAL_TEST_REGION_HPP_

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>

#include "gate_event_journal.hpp"

namespace voice_nav_mission
{

class OwnedJournalRegion
{
public:
  explicit OwnedJournalRegion(std::size_t capacity)
  : name_(unique_name()),
    region_bytes_(
      sizeof(GateEventJournalHeader) +
      capacity * sizeof(GateEventJournalSlot))
  {
    fd_ = shm_open(
      name_.c_str(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (fd_ < 0) {
      throw system_error("shm_open owner create failed");
    }
    linked_ = true;

    try {
      if (fchmod(fd_, 0600) != 0) {
        throw system_error("fchmod owner region failed");
      }
      if (ftruncate(fd_, static_cast<off_t>(region_bytes_)) != 0) {
        throw system_error("ftruncate owner region failed");
      }
      region_ = mmap(
        nullptr,
        region_bytes_,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd_,
        0);
      if (region_ == MAP_FAILED) {
        region_ = nullptr;
        throw system_error("mmap owner region failed");
      }
      std::memset(region_, 0, region_bytes_);
      auto & header = *static_cast<GateEventJournalHeader *>(region_);
      header.magic = VOICE_NAV_GATE_EVENT_JOURNAL_MAGIC;
      header.abi_version = VOICE_NAV_GATE_EVENT_JOURNAL_ABI_VERSION;
      header.header_bytes = sizeof(GateEventJournalHeader);
      header.slot_bytes = sizeof(GateEventJournalSlot);
      header.region_bytes = region_bytes_;
      header.capacity = capacity;
      header.owner_uid = static_cast<std::uint64_t>(geteuid());
      header.generation = 7U;
      header.nonce_hi = kNonceHi;
      header.nonce_lo = kNonceLo;
      header.header_checksum = gate_event_journal_header_checksum(header);
      gate_event_journal_store_release(
        header.init_state,
        VOICE_NAV_GATE_EVENT_JOURNAL_INIT_READY);
    } catch (...) {
      cleanup();
      throw;
    }
  }

  ~OwnedJournalRegion()
  {
    cleanup();
  }

  OwnedJournalRegion(const OwnedJournalRegion &) = delete;
  OwnedJournalRegion & operator=(const OwnedJournalRegion &) = delete;

  [[nodiscard]] const std::string & name() const noexcept
  {
    return name_;
  }

  [[nodiscard]] GateEventJournalHeader & header() noexcept
  {
    return *static_cast<GateEventJournalHeader *>(region_);
  }

  [[nodiscard]] GateEventJournalIdentity identity() const noexcept
  {
    const auto & header =
      *static_cast<const GateEventJournalHeader *>(region_);
    return {
      header.owner_uid,
      header.generation,
      header.nonce_hi,
      header.nonce_lo};
  }

  [[nodiscard]] std::uint64_t capacity() const noexcept
  {
    return static_cast<const GateEventJournalHeader *>(region_)->capacity;
  }

  [[nodiscard]] GateEventJournalSlot & slot(std::size_t index) noexcept
  {
    auto * slots = reinterpret_cast<GateEventJournalSlot *>(
      static_cast<std::byte *>(region_) + sizeof(GateEventJournalHeader));
    return slots[index];
  }

  void unlink_name()
  {
    if (linked_ && shm_unlink(name_.c_str()) != 0) {
      throw system_error("shm_unlink owner region failed");
    }
    linked_ = false;
  }

  void set_mode(mode_t mode)
  {
    if (fchmod(fd_, mode) != 0) {
      throw system_error("fchmod owner region failed");
    }
  }

private:
  static constexpr std::uint64_t kNonceHi =
    UINT64_C(0x123456789abcdef0);
  static constexpr std::uint64_t kNonceLo =
    UINT64_C(0x0fedcba987654321);

  static std::string unique_name()
  {
    static std::uint64_t sequence = 0U;
    ++sequence;
    std::ostringstream stream;
    stream << "/voice_nav_gate_"
           << std::hex << std::nouppercase << std::setfill('0')
           << std::setw(16) << static_cast<std::uint64_t>(getpid())
           << std::setw(16) << sequence;
    return stream.str();
  }

  static std::system_error system_error(const char * message)
  {
    return std::system_error(errno, std::generic_category(), message);
  }

  void cleanup() noexcept
  {
    if (region_ != nullptr) {
      (void)munmap(region_, region_bytes_);
      region_ = nullptr;
    }
    if (fd_ >= 0) {
      (void)close(fd_);
      fd_ = -1;
    }
    if (linked_) {
      (void)shm_unlink(name_.c_str());
      linked_ = false;
    }
  }

  std::string name_;
  std::size_t region_bytes_{0U};
  int fd_{-1};
  void * region_{nullptr};
  bool linked_{false};
};

}  // namespace voice_nav_mission

#endif  // GATE_EVENT_JOURNAL_TEST_REGION_HPP_
