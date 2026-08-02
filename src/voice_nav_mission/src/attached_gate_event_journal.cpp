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

#include "attached_gate_event_journal.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace voice_nav_mission
{
namespace
{

constexpr char kSharedMemoryNamePrefix[] = "/voice_nav_gate_";
constexpr std::size_t kIdentifierHexCharacters = 32U;
constexpr std::size_t kMaximumCapacity = 16384U;

std::system_error system_error(const char * message)
{
  return std::system_error(errno, std::generic_category(), message);
}

bool is_lower_hex(char character) noexcept
{
  return
    (character >= '0' && character <= '9') ||
    (character >= 'a' && character <= 'f');
}

void validate_name(const std::string & name)
{
  const std::string prefix{kSharedMemoryNamePrefix};
  if (
    name.size() != prefix.size() + kIdentifierHexCharacters ||
    name.compare(0U, prefix.size(), prefix) != 0)
  {
    throw std::invalid_argument(
            "Gate event journal name must be /voice_nav_gate_<32-lower-hex>");
  }
  for (std::size_t index = prefix.size(); index < name.size(); ++index) {
    if (!is_lower_hex(name[index])) {
      throw std::invalid_argument(
              "Gate event journal name must be /voice_nav_gate_<32-lower-hex>");
    }
  }
}

std::uint64_t parse_hex_word(
  const std::string & value,
  std::size_t offset)
{
  std::uint64_t word = 0U;
  for (std::size_t index = offset; index < offset + 16U; ++index) {
    const auto character = value[index];
    if (!is_lower_hex(character)) {
      throw std::invalid_argument(
              "Gate event journal nonce must be 32 lowercase hex characters");
    }
    const auto nibble = character <= '9' ?
      static_cast<std::uint64_t>(character - '0') :
      static_cast<std::uint64_t>(character - 'a' + 10);
    word = (word << 4U) | nibble;
  }
  return word;
}

GateEventJournalIdentity expected_identity(
  const std::string & nonce_hex,
  const GateEventJournalHeader & header)
{
  if (nonce_hex.size() != kIdentifierHexCharacters) {
    throw std::invalid_argument(
            "Gate event journal nonce must be 32 lowercase hex characters");
  }
  return {
    static_cast<std::uint64_t>(geteuid()),
    header.generation,
    parse_hex_word(nonce_hex, 0U),
    parse_hex_word(nonce_hex, 16U)};
}

std::size_t validate_status(const struct stat & status)
{
  if (!S_ISREG(status.st_mode)) {
    throw std::invalid_argument("Gate event journal is not a regular object");
  }
  if (status.st_uid != geteuid()) {
    throw std::invalid_argument("Gate event journal owner UID mismatch");
  }
  if ((status.st_mode & 0777) != 0600) {
    throw std::invalid_argument("Gate event journal mode must be exactly 0600");
  }
  if (status.st_size < 0) {
    throw std::invalid_argument("Gate event journal has a negative size");
  }

  constexpr std::size_t kMinimumBytes =
    sizeof(GateEventJournalHeader) + sizeof(GateEventJournalSlot);
  constexpr std::size_t kMaximumBytes =
    sizeof(GateEventJournalHeader) +
    kMaximumCapacity * sizeof(GateEventJournalSlot);
  const auto size = static_cast<std::uintmax_t>(status.st_size);
  if (
    size < kMinimumBytes ||
    size > kMaximumBytes ||
    size > std::numeric_limits<std::size_t>::max())
  {
    throw std::invalid_argument("Gate event journal size is out of bounds");
  }
  const auto region_bytes = static_cast<std::size_t>(size);
  if (
    (region_bytes - sizeof(GateEventJournalHeader)) %
    sizeof(GateEventJournalSlot) != 0U)
  {
    throw std::invalid_argument("Gate event journal size is not slot aligned");
  }
  return region_bytes;
}

}  // namespace

struct AttachedGateEventJournal::Impl
{
  Impl(
    const std::string & name,
    const std::string & nonce_hex,
    GateEventJournalClock clock)
  {
    validate_name(name);
    if (clock.read == nullptr) {
      throw std::invalid_argument("Gate event journal clock is missing");
    }

    fd = shm_open(name.c_str(), O_RDWR | O_CLOEXEC, 0);
    if (fd < 0) {
      throw system_error("shm_open Gate event journal attach failed");
    }

    try {
      struct stat status {};
      if (fstat(fd, &status) != 0) {
        throw system_error("fstat Gate event journal failed");
      }
      region_bytes = validate_status(status);
      region = mmap(
        nullptr,
        region_bytes,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd,
        0);
      if (region == MAP_FAILED) {
        region = nullptr;
        throw system_error("mmap Gate event journal failed");
      }
      const auto & header = *static_cast<GateEventJournalHeader *>(region);
      journal = std::make_unique<GateEventJournal>(
        region,
        region_bytes,
        expected_identity(nonce_hex, header),
        clock);
    } catch (...) {
      cleanup();
      throw;
    }
  }

  ~Impl()
  {
    cleanup();
  }

  void cleanup() noexcept
  {
    journal.reset();
    if (region != nullptr) {
      (void)munmap(region, region_bytes);
      region = nullptr;
    }
    if (fd >= 0) {
      (void)close(fd);
      fd = -1;
    }
  }

  int fd{-1};
  void * region{nullptr};
  std::size_t region_bytes{0U};
  std::unique_ptr<GateEventJournal> journal;
};

AttachedGateEventJournal AttachedGateEventJournal::open_existing(
  const std::string & name,
  const std::string & nonce_hex,
  GateEventJournalClock clock)
{
  return AttachedGateEventJournal(
    std::make_unique<Impl>(name, nonce_hex, clock));
}

AttachedGateEventJournal::AttachedGateEventJournal(
  std::unique_ptr<Impl> impl) noexcept
: impl_(std::move(impl))
{
}

AttachedGateEventJournal::~AttachedGateEventJournal() = default;

AttachedGateEventJournal::AttachedGateEventJournal(
  AttachedGateEventJournal &&) noexcept = default;

AttachedGateEventJournal & AttachedGateEventJournal::operator=(
  AttachedGateEventJournal &&) noexcept = default;

GateEventJournal & AttachedGateEventJournal::writer() noexcept
{
  return *impl_->journal;
}

}  // namespace voice_nav_mission
