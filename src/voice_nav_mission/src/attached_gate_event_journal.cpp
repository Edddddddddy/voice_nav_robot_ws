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
constexpr std::uint64_t kMaximumCapacity = 16384U;

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

std::size_t validate_config(
  const GateEventJournalAttachmentConfig & config)
{
  validate_name(config.shared_memory_name);
  if (config.clock.read == nullptr) {
    throw std::invalid_argument("Gate event journal clock is missing");
  }
  if (
    config.expected_identity.owner_uid !=
    static_cast<std::uint64_t>(geteuid()))
  {
    throw std::invalid_argument(
            "Gate event journal expected owner UID must match the process");
  }
  if (config.expected_identity.generation == 0U) {
    throw std::invalid_argument(
            "Gate event journal expected generation must be nonzero");
  }
  if (
    config.expected_identity.nonce_hi == 0U &&
    config.expected_identity.nonce_lo == 0U)
  {
    throw std::invalid_argument(
            "Gate event journal expected nonce must be nonzero");
  }
  if (
    config.expected_capacity == 0U ||
    config.expected_capacity > kMaximumCapacity)
  {
    throw std::invalid_argument(
            "Gate event journal expected capacity is out of bounds");
  }
  constexpr auto header_bytes = sizeof(GateEventJournalHeader);
  constexpr auto slot_bytes = sizeof(GateEventJournalSlot);
  if (
    config.expected_capacity >
    (std::numeric_limits<std::size_t>::max() - header_bytes) / slot_bytes)
  {
    throw std::invalid_argument(
            "Gate event journal expected byte size overflows");
  }
  return header_bytes +
         static_cast<std::size_t>(config.expected_capacity) * slot_bytes;
}

void validate_status(
  const struct stat & status,
  const GateEventJournalAttachmentConfig & config,
  std::size_t expected_region_bytes)
{
  if (!S_ISREG(status.st_mode)) {
    throw std::invalid_argument("Gate event journal is not a regular object");
  }
  if (
    static_cast<std::uint64_t>(status.st_uid) !=
    config.expected_identity.owner_uid)
  {
    throw std::invalid_argument("Gate event journal owner UID mismatch");
  }
  if ((status.st_mode & 07777) != 0600) {
    throw std::invalid_argument("Gate event journal mode must be exactly 0600");
  }
  if (status.st_nlink != 1) {
    throw std::invalid_argument("Gate event journal link count must be one");
  }
  if (status.st_size < 0) {
    throw std::invalid_argument("Gate event journal has a negative size");
  }
  const auto size = static_cast<std::uintmax_t>(status.st_size);
  if (
    size != static_cast<std::uintmax_t>(expected_region_bytes))
  {
    throw std::invalid_argument(
            "Gate event journal size does not match expected capacity");
  }
}

}  // namespace

struct AttachedGateEventJournal::Impl
{
  Impl(
    GateEventJournalAttachmentConfig config)
  {
    region_bytes = validate_config(config);

    fd = shm_open(
      config.shared_memory_name.c_str(), O_RDWR | O_CLOEXEC, 0);
    if (fd < 0) {
      throw system_error("shm_open Gate event journal attach failed");
    }

    try {
      struct stat status {};
      if (fstat(fd, &status) != 0) {
        throw system_error("fstat Gate event journal failed");
      }
      validate_status(status, config, region_bytes);
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
      journal = std::make_unique<GateEventJournal>(
        region,
        region_bytes,
        config.expected_identity,
        config.clock);
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

AttachedGateEventJournal::AttachedGateEventJournal(
  GateEventJournalAttachmentConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

AttachedGateEventJournal::~AttachedGateEventJournal() = default;

GateEventJournal & AttachedGateEventJournal::journal() noexcept
{
  return *impl_->journal;
}

}  // namespace voice_nav_mission
