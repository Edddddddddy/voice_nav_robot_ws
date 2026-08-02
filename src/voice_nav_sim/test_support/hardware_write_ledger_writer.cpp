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

#include "hardware_write_ledger_writer.hpp"

#include "hardware_write_ledger_abi.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace voice_nav_sim
{
namespace
{

static_assert(__atomic_always_lock_free(8U, nullptr));

constexpr std::uint64_t kCrc64EcmaPolynomial =
  UINT64_C(0x42f0e1eba9ea3693);
constexpr std::uint64_t kInvalidBankIndex =
  std::numeric_limits<std::uint64_t>::max();

std::uint64_t atomic_load_acquire(const std::uint64_t & value) noexcept
{
  return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

void atomic_store_release(
  std::uint64_t & destination,
  std::uint64_t value) noexcept
{
  __atomic_store_n(&destination, value, __ATOMIC_RELEASE);
}

void atomic_fetch_or_release(
  std::uint64_t & destination,
  std::uint64_t value) noexcept
{
  (void)__atomic_fetch_or(&destination, value, __ATOMIC_RELEASE);
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

template<typename Iterator>
std::uint64_t crc64_words(Iterator first, Iterator last) noexcept
{
  std::uint64_t checksum{0U};
  while (first != last) {
    checksum = crc64_word(checksum, *first);
    ++first;
  }
  return checksum;
}

std::uint64_t request_checksum(
  const voice_nav_hardware_write_ledger_header_v1 & header,
  const voice_nav_hardware_write_ledger_control_v1 & control,
  std::uint64_t request_ticket) noexcept
{
  const std::uint64_t words[] = {
    header.owner_uid,
    header.generation,
    header.nonce_hi,
    header.nonce_lo,
    control.request_op,
    control.request_flags,
    control.request_interval_id,
    control.request_bank_index,
    control.request_bank_epoch,
    control.request_segment_budget,
    control.request_invocation_budget,
    control.request_not_before_sim_stamp_ns_bits,
    request_ticket};
  return crc64_words(std::begin(words), std::end(words));
}

std::uint64_t response_checksum(
  const voice_nav_hardware_write_ledger_header_v1 & header,
  const voice_nav_hardware_write_ledger_control_v1 & control,
  std::uint64_t response_ticket) noexcept
{
  const std::uint64_t words[] = {
    header.owner_uid,
    header.generation,
    header.nonce_hi,
    header.nonce_lo,
    response_ticket,
    control.response_code,
    control.response_bank_index,
    control.response_bank_epoch,
    control.response_fence_write_seq};
  return crc64_words(std::begin(words), std::end(words));
}

bool finite_command_bits(std::uint64_t bits) noexcept
{
  static_assert(sizeof(bits) == sizeof(double));
  double value{0.0};
  std::memcpy(&value, &bits, sizeof(value));
  return std::isfinite(value);
}

bool zero_command_bits(std::uint64_t bits) noexcept
{
  static_assert(std::numeric_limits<double>::is_iec559);
  return (bits & UINT64_C(0x7fffffffffffffff)) == 0U;
}

}  // namespace

struct HardwareWriteLedgerWriter::Impl
{
  Impl(void * mapped_region, std::size_t mapped_region_bytes)
  : region(mapped_region),
    region_bytes(mapped_region_bytes),
    header(static_cast<voice_nav_hardware_write_ledger_header_v1 *>(region)),
    control(reinterpret_cast<voice_nav_hardware_write_ledger_control_v1 *>(
        static_cast<std::uint8_t *>(region) +
        VOICE_NAV_HARDWARE_WRITE_LEDGER_HEADER_BYTES))
  {
    if (
      region == nullptr ||
      region_bytes != static_cast<std::size_t>(header->region_bytes))
    {
      throw std::invalid_argument(
              "Hardware-write ledger Writer region is invalid");
    }
    bank_stride =
      VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_BYTES +
      header->segment_capacity_per_bank *
      VOICE_NAV_HARDWARE_WRITE_LEDGER_SEGMENT_BYTES;
  }

  voice_nav_hardware_write_ledger_bank_v1 * bank(
    std::uint64_t bank_index) noexcept
  {
    auto * bank_bytes =
      static_cast<std::uint8_t *>(region) +
      VOICE_NAV_HARDWARE_WRITE_LEDGER_HEADER_BYTES +
      VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_BYTES +
      bank_index * bank_stride;
    return reinterpret_cast<voice_nav_hardware_write_ledger_bank_v1 *>(
      bank_bytes);
  }

  voice_nav_hardware_write_ledger_segment_v1 * segment(
    std::uint64_t bank_index,
    std::uint64_t segment_index) noexcept
  {
    auto * segment_bytes =
      reinterpret_cast<std::uint8_t *>(bank(bank_index)) +
      VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_BYTES +
      segment_index * VOICE_NAV_HARDWARE_WRITE_LEDGER_SEGMENT_BYTES;
    return reinterpret_cast<voice_nav_hardware_write_ledger_segment_v1 *>(
      segment_bytes);
  }

  void latch_global_fault(std::uint64_t fault) noexcept
  {
    atomic_fetch_or_release(header->global_oracle_faults, fault);
  }

  void publish_response(
    std::uint64_t request_ticket,
    std::uint64_t response_code,
    std::uint64_t bank_index,
    std::uint64_t bank_epoch,
    std::uint64_t fence_write_seq) noexcept
  {
    control->response_code = response_code;
    control->response_bank_index = bank_index;
    control->response_bank_epoch = bank_epoch;
    control->response_fence_write_seq = fence_write_seq;
    control->response_checksum = response_checksum(
      *header, *control, request_ticket);
    atomic_store_release(control->response_ticket, request_ticket);
  }

  void process_arm(
    std::uint64_t request_ticket,
    std::uint64_t last_completed_write_seq) noexcept
  {
    const auto allowed_flags =
      VOICE_NAV_HARDWARE_WRITE_LEDGER_FLAG_ZERO_REQUIRED;
    if (
      control->request_interval_id == 0U ||
      (control->request_flags & ~allowed_flags) != 0U ||
      control->request_bank_index != 0U ||
      control->request_bank_epoch != 0U ||
      control->request_segment_budget == 0U ||
      control->request_segment_budget >
      header->segment_capacity_per_bank ||
      control->request_invocation_budget == 0U ||
      control->request_not_before_sim_stamp_ns_bits != 0U)
    {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      publish_response(
        request_ticket,
        VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_INVALID,
        kInvalidBankIndex,
        0U,
        last_completed_write_seq);
      return;
    }

    std::uint64_t free_bank_index{kInvalidBankIndex};
    for (std::uint64_t bank_index = 0U;
      bank_index < VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_COUNT;
      ++bank_index)
    {
      const auto state = atomic_load_acquire(bank(bank_index)->state);
      if (state == VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_ACTIVE) {
        publish_response(
          request_ticket,
          VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_BUSY,
          bank_index,
          bank(bank_index)->bank_epoch,
          last_completed_write_seq);
        return;
      }
      if (
        state == VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_FREE &&
        free_bank_index == kInvalidBankIndex)
      {
        free_bank_index = bank_index;
      }
    }
    if (free_bank_index == kInvalidBankIndex) {
      publish_response(
        request_ticket,
        VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_NO_FREE_BANK,
        kInvalidBankIndex,
        0U,
        last_completed_write_seq);
      return;
    }

    auto * selected_bank = bank(free_bank_index);
    if (selected_bank->bank_epoch == std::numeric_limits<std::uint64_t>::max()) {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_SEQUENCE);
      publish_response(
        request_ticket,
        VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_INVALID,
        free_bank_index,
        selected_bank->bank_epoch,
        last_completed_write_seq);
      return;
    }
    const auto next_bank_epoch = selected_bank->bank_epoch + 1U;
    selected_bank->bank_epoch = next_bank_epoch;
    selected_bank->interval_id = control->request_interval_id;
    selected_bank->arm_fence_write_seq = last_completed_write_seq;
    selected_bank->seal_fence_write_seq = 0U;
    selected_bank->segment_budget = control->request_segment_budget;
    selected_bank->invocation_budget = control->request_invocation_budget;
    selected_bank->predicate_flags = control->request_flags;
    selected_bank->seal_not_before_sim_stamp_ns_bits = 0U;
    selected_bank->segment_count = 0U;
    selected_bank->invocation_count = 0U;
    selected_bank->first_write_seq = 0U;
    selected_bank->last_write_seq = 0U;
    selected_bank->oracle_faults = 0U;
    selected_bank->page_count = 0U;
    selected_bank->bank_checksum = 0U;
    atomic_store_release(
      selected_bank->state,
      VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_ACTIVE);
    active_bank_index = free_bank_index;
    active_bank_epoch = next_bank_epoch;
    publish_response(
      request_ticket,
      VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_OK,
      free_bank_index,
      next_bank_epoch,
      last_completed_write_seq);
  }

  void process_control() noexcept
  {
    const auto request_ticket = atomic_load_acquire(control->request_ticket);
    const auto response_ticket = atomic_load_acquire(control->response_ticket);
    if (request_ticket == response_ticket) {
      return;
    }
    if (
      request_ticket == 0U ||
      response_ticket == std::numeric_limits<std::uint64_t>::max() ||
      request_ticket != response_ticket + 1U ||
      control->request_checksum !=
      request_checksum(*header, *control, request_ticket))
    {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      publish_response(
        request_ticket,
        VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_INVALID,
        kInvalidBankIndex,
        0U,
        atomic_load_acquire(header->last_completed_write_seq));
      return;
    }

    const auto last_completed_write_seq =
      atomic_load_acquire(header->last_completed_write_seq);
    if (control->request_op == VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_ARM) {
      process_arm(request_ticket, last_completed_write_seq);
      return;
    }
    latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
    publish_response(
      request_ticket,
      VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_INVALID,
      kInvalidBankIndex,
      0U,
      last_completed_write_seq);
  }

  HardwareWriteTicket begin_write(std::int64_t sim_stamp_ns) noexcept
  {
    if (has_outstanding_ticket) {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      return HardwareWriteTicket{
        0U, sim_stamp_ns, kInvalidBankIndex, 0U, false};
    }
    process_control();
    const auto last_completed_write_seq =
      atomic_load_acquire(header->last_completed_write_seq);
    if (last_completed_write_seq == std::numeric_limits<std::uint64_t>::max()) {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_SEQUENCE);
      return HardwareWriteTicket{
        0U, sim_stamp_ns, kInvalidBankIndex, 0U, false};
    }

    const bool included = active_bank_index != kInvalidBankIndex &&
      atomic_load_acquire(bank(active_bank_index)->state) ==
      VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_ACTIVE &&
      bank(active_bank_index)->bank_epoch == active_bank_epoch;
    outstanding_ticket = HardwareWriteTicket{
      last_completed_write_seq + 1U,
      sim_stamp_ns,
      included ? active_bank_index : kInvalidBankIndex,
      included ? active_bank_epoch : 0U,
      included};
    has_outstanding_ticket = true;
    return outstanding_ticket;
  }

  void finish_write(
    HardwareWriteTicket ticket,
    std::uint64_t delegated_result,
    HardwareWriteWheelObservation observation) noexcept
  {
    if (
      !has_outstanding_ticket || ticket.write_seq == 0U ||
      ticket.write_seq != outstanding_ticket.write_seq ||
      ticket.sim_stamp_ns != outstanding_ticket.sim_stamp_ns ||
      ticket.bank_index != outstanding_ticket.bank_index ||
      ticket.bank_epoch != outstanding_ticket.bank_epoch ||
      ticket.included != outstanding_ticket.included)
    {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      return;
    }

    if (ticket.included) {
      auto * active_bank = bank(ticket.bank_index);
      std::uint64_t faults{0U};
      if (
        atomic_load_acquire(active_bank->state) !=
        VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_ACTIVE ||
        active_bank->bank_epoch != ticket.bank_epoch ||
        active_bank->invocation_count != 0U ||
        active_bank->segment_count != 0U ||
        active_bank->invocation_budget == 0U ||
        active_bank->segment_budget == 0U)
      {
        faults |= VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL;
      }
      const auto observation_value =
        static_cast<std::uint64_t>(observation.status);
      if (
        observation_value >
        VOICE_NAV_HARDWARE_WRITE_LEDGER_OBSERVATION_EMPTY_COMPONENT)
      {
        faults |= VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_OBSERVATION;
      }
      if (delegated_result > UINT64_C(0xff)) {
        faults |= VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL;
      }
      if (observation.status != HardwareWriteObservationStatus::kValid) {
        faults |= VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_OBSERVATION;
      } else {
        if (
          !finite_command_bits(observation.left_command_bits) ||
          !finite_command_bits(observation.right_command_bits))
        {
          faults |= VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_NONFINITE;
        }
        if (
          (active_bank->predicate_flags &
          VOICE_NAV_HARDWARE_WRITE_LEDGER_FLAG_ZERO_REQUIRED) != 0U &&
          (!zero_command_bits(observation.left_command_bits) ||
          !zero_command_bits(observation.right_command_bits)))
        {
          faults |= VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_ZERO_REQUIRED;
        }
      }

      auto * first_segment = segment(ticket.bank_index, 0U);
      first_segment->generation = header->generation;
      first_segment->first_write_seq = ticket.write_seq;
      first_segment->last_write_seq = ticket.write_seq;
      first_segment->invocation_count = 1U;
      first_segment->sim_stamp_ns_bits =
        static_cast<std::uint64_t>(ticket.sim_stamp_ns);
      first_segment->observation_and_result =
        (observation_value << 8U) | (delegated_result & UINT64_C(0xff));
      first_segment->left_command_bits = observation.left_command_bits;
      first_segment->right_command_bits = observation.right_command_bits;
      active_bank->segment_count = 1U;
      active_bank->invocation_count = 1U;
      active_bank->first_write_seq = ticket.write_seq;
      active_bank->last_write_seq = ticket.write_seq;
      active_bank->oracle_faults |= faults;
      if (faults != 0U) {
        latch_global_fault(faults);
      }
    }

    atomic_store_release(
      header->last_completed_write_seq, ticket.write_seq);
    has_outstanding_ticket = false;
  }

  void * region;
  std::size_t region_bytes;
  voice_nav_hardware_write_ledger_header_v1 * header;
  voice_nav_hardware_write_ledger_control_v1 * control;
  std::uint64_t bank_stride{0U};
  std::uint64_t active_bank_index{kInvalidBankIndex};
  std::uint64_t active_bank_epoch{0U};
  bool has_outstanding_ticket{false};
  HardwareWriteTicket outstanding_ticket{
    0U, 0, kInvalidBankIndex, 0U, false};
};

HardwareWriteLedgerWriter::HardwareWriteLedgerWriter(
  void * region,
  std::size_t region_bytes)
: impl_(std::make_unique<Impl>(region, region_bytes))
{
}

HardwareWriteLedgerWriter::~HardwareWriteLedgerWriter() = default;

HardwareWriteTicket HardwareWriteLedgerWriter::begin_write(
  std::int64_t sim_stamp_ns) noexcept
{
  return impl_->begin_write(sim_stamp_ns);
}

void HardwareWriteLedgerWriter::finish_write(
  HardwareWriteTicket ticket,
  std::uint64_t delegated_result,
  HardwareWriteWheelObservation observation) noexcept
{
  impl_->finish_write(ticket, delegated_result, observation);
}

}  // namespace voice_nav_sim
