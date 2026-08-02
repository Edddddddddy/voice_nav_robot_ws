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

#include "attached_hardware_write_ledger.hpp"

#include "hardware_write_ledger_abi.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace voice_nav_sim
{
namespace
{

static_assert(__atomic_always_lock_free(8U, nullptr));
static_assert(
  sizeof(voice_nav_hardware_write_ledger_header_v1) ==
  VOICE_NAV_HARDWARE_WRITE_LEDGER_HEADER_BYTES);
static_assert(
  sizeof(voice_nav_hardware_write_ledger_control_v1) ==
  VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_BYTES);
static_assert(
  sizeof(voice_nav_hardware_write_ledger_bank_v1) ==
  VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_BYTES);
static_assert(
  sizeof(voice_nav_hardware_write_ledger_segment_v1) ==
  VOICE_NAV_HARDWARE_WRITE_LEDGER_SEGMENT_BYTES);
static_assert(
  sizeof(voice_nav_hardware_write_ledger_page_v1) ==
  VOICE_NAV_HARDWARE_WRITE_LEDGER_PAGE_BYTES);

constexpr char kSharedMemoryNamePrefix[] = "/voice_nav_hardware_";
constexpr std::size_t kNameSuffixHexCharacters = 16U;
constexpr std::uint64_t kMaximumSegmentCapacity = UINT64_C(16384);
constexpr std::uint64_t kInitReady = 1U;
constexpr std::uint64_t kFixedRegionBytes =
  VOICE_NAV_HARDWARE_WRITE_LEDGER_HEADER_BYTES +
  VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_BYTES +
  VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_COUNT *
  VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_BYTES;
constexpr std::uint64_t kSegmentBytesPerCapacity =
  VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_COUNT *
  VOICE_NAV_HARDWARE_WRITE_LEDGER_SEGMENT_BYTES;
constexpr std::uint64_t kCrc64EcmaPolynomial =
  UINT64_C(0x42f0e1eba9ea3693);

struct ExpectedNonce
{
  std::uint64_t hi;
  std::uint64_t lo;
};

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
    name.size() != prefix.size() + kNameSuffixHexCharacters ||
    name.compare(0U, prefix.size(), prefix) != 0)
  {
    throw std::invalid_argument(
            "Hardware-write ledger name must be "
            "/voice_nav_hardware_<16-lower-hex>");
  }
  for (std::size_t index = prefix.size(); index < name.size(); ++index) {
    if (!is_lower_hex(name[index])) {
      throw std::invalid_argument(
              "Hardware-write ledger name must be "
              "/voice_nav_hardware_<16-lower-hex>");
    }
  }
}

std::uint64_t parse_hex_word(
  const std::string & value,
  std::size_t offset)
{
  std::uint64_t result{0U};
  for (std::size_t index = offset; index < offset + 16U; ++index) {
    if (!is_lower_hex(value[index])) {
      throw std::invalid_argument(
              "Hardware-write ledger nonce must be 32 lower-hex digits");
    }
    const auto character = value[index];
    const auto nibble = character <= '9' ?
      static_cast<std::uint64_t>(character - '0') :
      static_cast<std::uint64_t>(character - 'a' + 10);
    result = (result << 4U) | nibble;
  }
  return result;
}

ExpectedNonce parse_nonce(const std::string & nonce)
{
  if (nonce.size() != 32U) {
    throw std::invalid_argument(
            "Hardware-write ledger nonce must be 32 lower-hex digits");
  }
  const ExpectedNonce parsed{
    parse_hex_word(nonce, 0U), parse_hex_word(nonce, 16U)};
  if (parsed.hi == 0U && parsed.lo == 0U) {
    throw std::invalid_argument(
            "Hardware-write ledger nonce must be nonzero");
  }
  return parsed;
}

std::size_t region_bytes_for_capacity(std::uint64_t capacity)
{
  if (
    capacity == 0U || capacity > kMaximumSegmentCapacity ||
    capacity >
    (std::numeric_limits<std::uint64_t>::max() - kFixedRegionBytes) /
    kSegmentBytesPerCapacity)
  {
    throw std::invalid_argument(
            "Hardware-write ledger segment capacity is invalid");
  }
  const auto bytes =
    kFixedRegionBytes + capacity * kSegmentBytesPerCapacity;
  if (bytes > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument(
            "Hardware-write ledger region size overflows size_t");
  }
  return static_cast<std::size_t>(bytes);
}

std::size_t expected_region_bytes(
  const HardwareWriteLedgerAttachmentConfig & config)
{
  validate_name(config.shared_memory_name);
  if (
    config.expected_identity.owner_uid !=
    static_cast<std::uint64_t>(geteuid()))
  {
    throw std::invalid_argument(
            "Hardware-write ledger owner UID must match this process");
  }
  if (config.expected_identity.generation == 0U) {
    throw std::invalid_argument(
            "Hardware-write ledger generation must be nonzero");
  }
  if (
    config.expected_identity.nonce_hi == 0U &&
    config.expected_identity.nonce_lo == 0U)
  {
    throw std::invalid_argument(
            "Hardware-write ledger nonce must be nonzero");
  }

  const auto capacity = config.expected_layout.segment_capacity_per_bank;
  const auto page_limit = config.expected_layout.page_segment_limit;
  if (
    capacity == 0U || capacity > kMaximumSegmentCapacity ||
    page_limit == 0U || page_limit > capacity)
  {
    throw std::invalid_argument(
            "Hardware-write ledger capacity or page limit is invalid");
  }

  return region_bytes_for_capacity(capacity);
}

std::size_t discovery_region_bytes(const struct stat & status)
{
  if (!S_ISREG(status.st_mode)) {
    throw std::invalid_argument(
            "Hardware-write ledger is not a regular object");
  }
  if (status.st_uid != geteuid()) {
    throw std::invalid_argument("Hardware-write ledger owner UID mismatch");
  }
  if ((status.st_mode & 07777) != 0600) {
    throw std::invalid_argument(
            "Hardware-write ledger mode must be exactly 0600");
  }
  if (status.st_nlink != 1) {
    throw std::invalid_argument(
            "Hardware-write ledger link count must be one");
  }
  if (status.st_size < 0) {
    throw std::invalid_argument(
            "Hardware-write ledger has a negative size");
  }
  const auto bytes = static_cast<std::uintmax_t>(status.st_size);
  const auto minimum = region_bytes_for_capacity(1U);
  const auto maximum = region_bytes_for_capacity(kMaximumSegmentCapacity);
  if (
    bytes < minimum || bytes > maximum ||
    (bytes - kFixedRegionBytes) % kSegmentBytesPerCapacity != 0U)
  {
    throw std::invalid_argument(
            "Hardware-write ledger discovery size is invalid");
  }
  return static_cast<std::size_t>(bytes);
}

void validate_status(
  const struct stat & status,
  const HardwareWriteLedgerAttachmentConfig & config,
  std::size_t region_bytes)
{
  if (!S_ISREG(status.st_mode)) {
    throw std::invalid_argument(
            "Hardware-write ledger is not a regular object");
  }
  if (
    static_cast<std::uint64_t>(status.st_uid) !=
    config.expected_identity.owner_uid)
  {
    throw std::invalid_argument("Hardware-write ledger owner UID mismatch");
  }
  if ((status.st_mode & 07777) != 0600) {
    throw std::invalid_argument(
            "Hardware-write ledger mode must be exactly 0600");
  }
  if (status.st_nlink != 1) {
    throw std::invalid_argument(
            "Hardware-write ledger link count must be one");
  }
  if (status.st_size < 0) {
    throw std::invalid_argument(
            "Hardware-write ledger has a negative size");
  }
  if (
    static_cast<std::uintmax_t>(status.st_size) !=
    static_cast<std::uintmax_t>(region_bytes))
  {
    throw std::invalid_argument(
            "Hardware-write ledger region size mismatch");
  }
}

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

std::uint64_t header_checksum(
  const voice_nav_hardware_write_ledger_header_v1 & header) noexcept
{
  const std::uint64_t words[] = {
    header.magic,
    header.abi_version,
    header.endian_tag,
    header.header_bytes,
    header.control_bytes,
    header.bank_bytes,
    header.segment_bytes,
    header.page_bytes,
    header.region_bytes,
    header.bank_count,
    header.segment_capacity_per_bank,
    header.page_segment_limit,
    header.owner_uid,
    header.generation,
    header.nonce_hi,
    header.nonce_lo,
    header.feature_flags,
    header.reserved0,
    header.reserved1};
  std::uint64_t checksum{0U};
  for (const auto word : words) {
    checksum = crc64_word(checksum, word);
  }
  return checksum;
}

std::uint64_t atomic_load_acquire(const std::uint64_t & value) noexcept
{
  return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

HardwareWriteLedgerAttachmentConfig discover_config(
  const void * region,
  std::size_t region_bytes,
  const HardwareWriteLedgerDiscoveryConfig & discovery,
  ExpectedNonce expected_nonce)
{
  if (
    reinterpret_cast<std::uintptr_t>(region) % alignof(std::uint64_t) != 0U)
  {
    throw std::invalid_argument(
            "Hardware-write ledger mapping is not uint64 aligned");
  }
  const auto * shared_header =
    static_cast<const voice_nav_hardware_write_ledger_header_v1 *>(region);
  if (atomic_load_acquire(shared_header->init_state) != kInitReady) {
    throw std::invalid_argument("Hardware-write ledger is not READY");
  }
  const auto header = *shared_header;
  if (
    header.magic != VOICE_NAV_HARDWARE_WRITE_LEDGER_MAGIC ||
    header.abi_version != VOICE_NAV_HARDWARE_WRITE_LEDGER_ABI_VERSION ||
    header.endian_tag != VOICE_NAV_HARDWARE_WRITE_LEDGER_ENDIAN_TAG ||
    header.header_bytes != VOICE_NAV_HARDWARE_WRITE_LEDGER_HEADER_BYTES ||
    header.control_bytes != VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_BYTES ||
    header.bank_bytes != VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_BYTES ||
    header.segment_bytes != VOICE_NAV_HARDWARE_WRITE_LEDGER_SEGMENT_BYTES ||
    header.page_bytes != VOICE_NAV_HARDWARE_WRITE_LEDGER_PAGE_BYTES ||
    header.region_bytes != region_bytes ||
    header.bank_count != VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_COUNT ||
    header.owner_uid != static_cast<std::uint64_t>(geteuid()) ||
    header.generation == 0U ||
    header.segment_capacity_per_bank == 0U ||
    header.segment_capacity_per_bank > kMaximumSegmentCapacity ||
    header.page_segment_limit == 0U ||
    header.page_segment_limit > header.segment_capacity_per_bank ||
    header.feature_flags != 0U || header.reserved0 != 0U ||
    header.reserved1 != 0U ||
    header.header_checksum != header_checksum(header))
  {
    throw std::invalid_argument(
            "Hardware-write ledger discovered ABI or checksum mismatch");
  }
  if (
    region_bytes_for_capacity(header.segment_capacity_per_bank) !=
    region_bytes)
  {
    throw std::invalid_argument(
            "Hardware-write ledger discovered capacity mismatch");
  }
  if (
    header.nonce_hi != expected_nonce.hi ||
    header.nonce_lo != expected_nonce.lo)
  {
    throw std::invalid_argument("Hardware-write ledger nonce mismatch");
  }
  return HardwareWriteLedgerAttachmentConfig{
    discovery.shared_memory_name,
    HardwareWriteLedgerIdentity{
      header.owner_uid,
      header.generation,
      header.nonce_hi,
      header.nonce_lo},
    HardwareWriteLedgerLayout{
      header.segment_capacity_per_bank,
      header.page_segment_limit}};
}

void validate_ready_region(
  const void * region,
  std::size_t region_bytes,
  const HardwareWriteLedgerAttachmentConfig & config)
{
  if (
    reinterpret_cast<std::uintptr_t>(region) % alignof(std::uint64_t) != 0U)
  {
    throw std::invalid_argument(
            "Hardware-write ledger mapping is not uint64 aligned");
  }
  const std::uint16_t endian_probe{1U};
  if (*reinterpret_cast<const std::uint8_t *>(&endian_probe) != 1U) {
    throw std::invalid_argument(
            "Hardware-write ledger requires a little-endian host");
  }

  const auto * header =
    static_cast<const voice_nav_hardware_write_ledger_header_v1 *>(region);
  if (atomic_load_acquire(header->init_state) != kInitReady) {
    throw std::invalid_argument("Hardware-write ledger is not READY");
  }
  if (
    header->magic != VOICE_NAV_HARDWARE_WRITE_LEDGER_MAGIC ||
    header->abi_version != VOICE_NAV_HARDWARE_WRITE_LEDGER_ABI_VERSION ||
    header->endian_tag != VOICE_NAV_HARDWARE_WRITE_LEDGER_ENDIAN_TAG ||
    header->header_bytes != VOICE_NAV_HARDWARE_WRITE_LEDGER_HEADER_BYTES ||
    header->control_bytes != VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_BYTES ||
    header->bank_bytes != VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_BYTES ||
    header->segment_bytes != VOICE_NAV_HARDWARE_WRITE_LEDGER_SEGMENT_BYTES ||
    header->page_bytes != VOICE_NAV_HARDWARE_WRITE_LEDGER_PAGE_BYTES ||
    header->region_bytes != region_bytes ||
    header->bank_count != VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_COUNT ||
    header->segment_capacity_per_bank !=
    config.expected_layout.segment_capacity_per_bank ||
    header->page_segment_limit !=
    config.expected_layout.page_segment_limit ||
    header->owner_uid != config.expected_identity.owner_uid ||
    header->generation != config.expected_identity.generation ||
    header->nonce_hi != config.expected_identity.nonce_hi ||
    header->nonce_lo != config.expected_identity.nonce_lo ||
    header->feature_flags != 0U || header->reserved0 != 0U ||
    header->reserved1 != 0U ||
    header->header_checksum != header_checksum(*header))
  {
    throw std::invalid_argument(
            "Hardware-write ledger ABI identity or checksum mismatch");
  }
  if (
    atomic_load_acquire(header->writer_pid) != 0U ||
    atomic_load_acquire(header->last_completed_write_seq) != 0U ||
    atomic_load_acquire(header->global_oracle_faults) != 0U)
  {
    throw std::invalid_argument(
            "Hardware-write ledger mutable header is not pristine");
  }

  const auto * payload =
    static_cast<const std::uint8_t *>(region) +
    VOICE_NAV_HARDWARE_WRITE_LEDGER_HEADER_BYTES;
  const auto payload_size =
    region_bytes - VOICE_NAV_HARDWARE_WRITE_LEDGER_HEADER_BYTES;
  if (!std::all_of(payload, payload + payload_size, [](std::uint8_t value) {
      return value == 0U;
    }))
  {
    throw std::invalid_argument(
            "Hardware-write ledger control or banks are not pristine");
  }
}

void claim_writer(void * region)
{
  auto * header =
    static_cast<voice_nav_hardware_write_ledger_header_v1 *>(region);
  const auto pid = static_cast<std::uint64_t>(getpid());
  std::uint64_t expected{0U};
  if (
    pid == 0U ||
    !__atomic_compare_exchange_n(
      &header->writer_pid,
      &expected,
      pid,
      false,
      __ATOMIC_ACQ_REL,
      __ATOMIC_ACQUIRE))
  {
    throw std::invalid_argument(
            "Hardware-write ledger Writer is already claimed");
  }
}

}  // namespace

struct AttachedHardwareWriteLedger::Impl
{
  explicit Impl(HardwareWriteLedgerAttachmentConfig attachment_config)
  : config(std::move(attachment_config))
  {
    region_bytes = expected_region_bytes(config);
    fd = shm_open(
      config.shared_memory_name.c_str(), O_RDWR | O_CLOEXEC, 0);
    if (fd < 0) {
      throw system_error("shm_open hardware-write ledger attach failed");
    }

    try {
      struct stat status {};
      if (fstat(fd, &status) != 0) {
        throw system_error("fstat hardware-write ledger failed");
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
        throw system_error("mmap hardware-write ledger failed");
      }
      validate_ready_region(region, region_bytes, config);
      auto pending_writer =
        std::make_unique<HardwareWriteLedgerWriter>(region, region_bytes);
      claim_writer(region);
      writer = std::move(pending_writer);
    } catch (...) {
      cleanup();
      throw;
    }
  }

  explicit Impl(HardwareWriteLedgerDiscoveryConfig discovery)
  {
    validate_name(discovery.shared_memory_name);
    const auto expected_nonce = parse_nonce(discovery.expected_nonce);
    fd = shm_open(
      discovery.shared_memory_name.c_str(), O_RDWR | O_CLOEXEC, 0);
    if (fd < 0) {
      throw system_error("shm_open hardware-write ledger discovery failed");
    }

    try {
      struct stat initial_status {};
      if (fstat(fd, &initial_status) != 0) {
        throw system_error("fstat hardware-write ledger discovery failed");
      }
      region_bytes = discovery_region_bytes(initial_status);
      region = mmap(
        nullptr,
        region_bytes,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd,
        0);
      if (region == MAP_FAILED) {
        region = nullptr;
        throw system_error("mmap hardware-write ledger discovery failed");
      }

      struct stat mapped_status {};
      if (fstat(fd, &mapped_status) != 0) {
        throw system_error(
                "second fstat hardware-write ledger discovery failed");
      }
      if (
        mapped_status.st_dev != initial_status.st_dev ||
        mapped_status.st_ino != initial_status.st_ino ||
        mapped_status.st_uid != initial_status.st_uid ||
        mapped_status.st_mode != initial_status.st_mode ||
        mapped_status.st_nlink != initial_status.st_nlink ||
        mapped_status.st_size != initial_status.st_size)
      {
        throw std::invalid_argument(
                "Hardware-write ledger metadata changed during discovery");
      }

      config = discover_config(
        region, region_bytes, discovery, expected_nonce);
      validate_status(mapped_status, config, region_bytes);
      validate_ready_region(region, region_bytes, config);
      auto pending_writer =
        std::make_unique<HardwareWriteLedgerWriter>(region, region_bytes);
      claim_writer(region);
      writer = std::move(pending_writer);
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
    writer.reset();
    if (region != nullptr) {
      (void)munmap(region, region_bytes);
      region = nullptr;
    }
    if (fd >= 0) {
      (void)close(fd);
      fd = -1;
    }
  }

  [[nodiscard]] std::uint64_t claimed_writer_pid() const noexcept
  {
    if (region == nullptr) {
      return 0U;
    }
    const auto * header =
      static_cast<const voice_nav_hardware_write_ledger_header_v1 *>(region);
    return atomic_load_acquire(header->writer_pid);
  }

  HardwareWriteLedgerAttachmentConfig config{};
  int fd{-1};
  void * region{nullptr};
  std::size_t region_bytes{0U};
  std::unique_ptr<HardwareWriteLedgerWriter> writer;
};

AttachedHardwareWriteLedger::AttachedHardwareWriteLedger(
  HardwareWriteLedgerAttachmentConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

AttachedHardwareWriteLedger::AttachedHardwareWriteLedger(
  HardwareWriteLedgerDiscoveryConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

AttachedHardwareWriteLedger::~AttachedHardwareWriteLedger() = default;

std::uint64_t AttachedHardwareWriteLedger::claimed_writer_pid()
const noexcept
{
  return impl_->claimed_writer_pid();
}

HardwareWriteLedgerWriter & AttachedHardwareWriteLedger::writer() noexcept
{
  return *impl_->writer;
}

HardwareWriteTicket AttachedHardwareWriteLedger::begin_write(
  std::int64_t sim_stamp_ns) noexcept
{
  return impl_->writer->begin_write(sim_stamp_ns);
}

void AttachedHardwareWriteLedger::finish_write(
  HardwareWriteTicket ticket,
  std::uint64_t delegated_result,
  HardwareWriteWheelObservation observation) noexcept
{
  impl_->writer->finish_write(ticket, delegated_result, observation);
}

}  // namespace voice_nav_sim
