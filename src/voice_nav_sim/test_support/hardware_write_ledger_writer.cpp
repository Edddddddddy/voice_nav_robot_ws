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
constexpr std::uint64_t kWriterIdle{0U};
constexpr std::uint64_t kWriterBeginning{1U};
constexpr std::uint64_t kWriterOutstanding{2U};
constexpr std::uint64_t kWriterFinishing{3U};

struct ControlRequestSnapshot
{
  std::uint64_t op;
  std::uint64_t flags;
  std::uint64_t interval_id;
  std::uint64_t bank_index;
  std::uint64_t bank_epoch;
  std::uint64_t segment_budget;
  std::uint64_t invocation_budget;
  std::uint64_t not_before_sim_stamp_ns_bits;
  std::uint64_t checksum;
  std::uint64_t ticket;
};

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

bool atomic_compare_exchange_acq_rel(
  std::uint64_t & destination,
  std::uint64_t & expected,
  std::uint64_t desired) noexcept
{
  return __atomic_compare_exchange_n(
    &destination,
    &expected,
    desired,
    false,
    __ATOMIC_ACQ_REL,
    __ATOMIC_ACQUIRE);
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
  const ControlRequestSnapshot & request) noexcept
{
  const std::uint64_t words[] = {
    header.owner_uid,
    header.generation,
    header.nonce_hi,
    header.nonce_lo,
    request.op,
    request.flags,
    request.interval_id,
    request.bank_index,
    request.bank_epoch,
    request.segment_budget,
    request.invocation_budget,
    request.not_before_sim_stamp_ns_bits,
    request.ticket};
  return crc64_words(std::begin(words), std::end(words));
}

std::uint64_t response_checksum(
  const voice_nav_hardware_write_ledger_header_v1 & header,
  std::uint64_t request_checksum_value,
  std::uint64_t response_ticket,
  std::uint64_t response_code,
  std::uint64_t response_bank_index,
  std::uint64_t response_bank_epoch,
  std::uint64_t response_fence_write_seq) noexcept
{
  const std::uint64_t words[] = {
    header.owner_uid,
    header.generation,
    header.nonce_hi,
    header.nonce_lo,
    request_checksum_value,
    response_ticket,
    response_code,
    response_bank_index,
    response_bank_epoch,
    response_fence_write_seq};
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

std::int64_t sim_stamp_from_bits(std::uint64_t bits) noexcept
{
  static_assert(sizeof(bits) == sizeof(std::int64_t));
  std::int64_t stamp{0};
  std::memcpy(&stamp, &bits, sizeof(stamp));
  return stamp;
}

bool segment_tuple_matches(
  const voice_nav_hardware_write_ledger_segment_v1 & segment,
  std::uint64_t generation,
  std::int64_t sim_stamp_ns,
  std::uint64_t observation_and_result,
  const HardwareWriteWheelObservation & observation) noexcept
{
  return segment.generation == generation &&
         segment.sim_stamp_ns_bits ==
         static_cast<std::uint64_t>(sim_stamp_ns) &&
         segment.observation_and_result == observation_and_result &&
         segment.left_command_bits == observation.left_command_bits &&
         segment.right_command_bits == observation.right_command_bits;
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

  std::uint64_t calculate_bank_checksum(std::uint64_t bank_index) noexcept
  {
    const auto * sealed_bank = bank(bank_index);
    const std::uint64_t bank_words[] = {
      sealed_bank->bank_epoch,
      sealed_bank->interval_id,
      sealed_bank->arm_fence_write_seq,
      sealed_bank->seal_fence_write_seq,
      sealed_bank->segment_budget,
      sealed_bank->invocation_budget,
      sealed_bank->predicate_flags,
      sealed_bank->seal_not_before_sim_stamp_ns_bits,
      sealed_bank->segment_count,
      sealed_bank->invocation_count,
      sealed_bank->first_write_seq,
      sealed_bank->last_write_seq,
      sealed_bank->oracle_faults,
      sealed_bank->page_count};
    auto checksum = crc64_words(
      std::begin(bank_words), std::end(bank_words));
    for (std::uint64_t segment_index = 0U;
      segment_index < sealed_bank->segment_count;
      ++segment_index)
    {
      const auto * current_segment = segment(bank_index, segment_index);
      const std::uint64_t segment_words[] = {
        current_segment->generation,
        current_segment->first_write_seq,
        current_segment->last_write_seq,
        current_segment->invocation_count,
        current_segment->sim_stamp_ns_bits,
        current_segment->observation_and_result,
        current_segment->left_command_bits,
        current_segment->right_command_bits};
      for (const auto word : segment_words) {
        checksum = crc64_word(checksum, word);
      }
    }
    return checksum;
  }

  void latch_global_fault(std::uint64_t fault) noexcept
  {
    atomic_fetch_or_release(header->global_oracle_faults, fault);
  }

  void publish_response(
    std::uint64_t request_ticket,
    std::uint64_t request_checksum_value,
    std::uint64_t response_code,
    std::uint64_t bank_index,
    std::uint64_t bank_epoch,
    std::uint64_t fence_write_seq) noexcept
  {
    atomic_store_release(
      control->request_state,
      VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_IDLE);
    control->response_code = response_code;
    control->response_bank_index = bank_index;
    control->response_bank_epoch = bank_epoch;
    control->response_fence_write_seq = fence_write_seq;
    control->response_request_checksum = request_checksum_value;
    control->response_checksum = response_checksum(
      *header,
      request_checksum_value,
      request_ticket,
      response_code,
      bank_index,
      bank_epoch,
      fence_write_seq);
    atomic_store_release(control->response_ticket, request_ticket);
  }

  void process_arm(
    const ControlRequestSnapshot & request,
    std::uint64_t last_completed_write_seq) noexcept
  {
    const auto allowed_flags =
      VOICE_NAV_HARDWARE_WRITE_LEDGER_FLAG_ZERO_REQUIRED;
    if (
      request.interval_id == 0U ||
      (request.flags & ~allowed_flags) != 0U ||
      request.bank_index != 0U ||
      request.bank_epoch != 0U ||
      request.segment_budget == 0U ||
      request.segment_budget >
      header->segment_capacity_per_bank ||
      request.invocation_budget == 0U ||
      request.not_before_sim_stamp_ns_bits != 0U)
    {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      publish_response(
        request.ticket,
        request.checksum,
        VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_INVALID,
        kInvalidBankIndex,
        0U,
        last_completed_write_seq);
      return;
    }

    if (last_completed_write_seq == std::numeric_limits<std::uint64_t>::max()) {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_SEQUENCE);
      publish_response(
        request.ticket,
        request.checksum,
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
          request.ticket,
          request.checksum,
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
        request.ticket,
        request.checksum,
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
        request.ticket,
        request.checksum,
        VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_INVALID,
        free_bank_index,
        selected_bank->bank_epoch,
        last_completed_write_seq);
      return;
    }
    const auto next_bank_epoch = selected_bank->bank_epoch + 1U;
    selected_bank->bank_epoch = next_bank_epoch;
    selected_bank->interval_id = request.interval_id;
    selected_bank->arm_fence_write_seq = last_completed_write_seq;
    selected_bank->seal_fence_write_seq = 0U;
    selected_bank->segment_budget = request.segment_budget;
    selected_bank->invocation_budget = request.invocation_budget;
    selected_bank->predicate_flags = request.flags;
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
      request.ticket,
      request.checksum,
      VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_OK,
      free_bank_index,
      next_bank_epoch,
      last_completed_write_seq);
  }

  void process_seal(const ControlRequestSnapshot & request) noexcept
  {
    const auto allowed_flags =
      VOICE_NAV_HARDWARE_WRITE_LEDGER_FLAG_EXACT_SEAL_STAMP;
    const bool bank_index_valid =
      request.bank_index < VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_COUNT;
    const auto not_before_stamp =
      sim_stamp_from_bits(request.not_before_sim_stamp_ns_bits);
    const bool request_valid =
      request.interval_id != 0U &&
      (request.flags & ~allowed_flags) == 0U &&
      bank_index_valid &&
      request.bank_epoch != 0U &&
      request.segment_budget == 0U &&
      request.invocation_budget == 0U &&
      not_before_stamp >= 0;
    auto * selected_bank = bank_index_valid ? bank(request.bank_index) : nullptr;
    const bool active_identity_matches = request_valid &&
      active_bank_index == request.bank_index &&
      active_bank_epoch == request.bank_epoch &&
      atomic_load_acquire(selected_bank->state) ==
      VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_ACTIVE &&
      selected_bank->bank_epoch == request.bank_epoch &&
      selected_bank->interval_id == request.interval_id;
    if (!active_identity_matches) {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      publish_response(
        request.ticket,
        request.checksum,
        VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_INVALID,
        request.bank_index,
        request.bank_epoch,
        atomic_load_acquire(header->last_completed_write_seq));
      return;
    }

    selected_bank->seal_not_before_sim_stamp_ns_bits =
      request.not_before_sim_stamp_ns_bits;
    selected_bank->predicate_flags |= request.flags;
    pending_seal_ticket = request.ticket;
    pending_seal_checksum = request.checksum;
    pending_seal_bank_index = request.bank_index;
    pending_seal_bank_epoch = request.bank_epoch;
    pending_seal_not_before_sim_stamp_ns = not_before_stamp;
    pending_seal_exact_stamp =
      (request.flags &
      VOICE_NAV_HARDWARE_WRITE_LEDGER_FLAG_EXACT_SEAL_STAMP) != 0U;
    has_pending_seal = true;
  }

  void complete_pending_seal(HardwareWriteTicket ticket) noexcept
  {
    if (
      !has_pending_seal || !ticket.included ||
      ticket.bank_index != pending_seal_bank_index ||
      ticket.bank_epoch != pending_seal_bank_epoch ||
      ticket.sim_stamp_ns < pending_seal_not_before_sim_stamp_ns)
    {
      return;
    }

    auto * sealed_bank = bank(pending_seal_bank_index);
    const auto page_limit = header->page_segment_limit;
    if (
      sealed_bank->segment_count >
      header->segment_capacity_per_bank ||
      page_limit == 0U ||
      page_limit > header->segment_capacity_per_bank)
    {
      sealed_bank->oracle_faults |=
        VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL;
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      return;
    }
    if (
      pending_seal_exact_stamp &&
      ticket.sim_stamp_ns != pending_seal_not_before_sim_stamp_ns)
    {
      sealed_bank->oracle_faults |=
        VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_SIM_STAMP;
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_SIM_STAMP);
    }
    sealed_bank->seal_fence_write_seq = ticket.write_seq;
    sealed_bank->page_count = sealed_bank->segment_count == 0U ? 1U :
      sealed_bank->segment_count / page_limit +
      (sealed_bank->segment_count % page_limit == 0U ? 0U : 1U);
    sealed_bank->bank_checksum = calculate_bank_checksum(
      pending_seal_bank_index);

    const auto response_ticket = pending_seal_ticket;
    const auto response_request_checksum = pending_seal_checksum;
    const auto response_bank_index = pending_seal_bank_index;
    const auto response_bank_epoch = pending_seal_bank_epoch;
    has_pending_seal = false;
    pending_seal_ticket = 0U;
    pending_seal_checksum = 0U;
    pending_seal_bank_index = kInvalidBankIndex;
    pending_seal_bank_epoch = 0U;
    pending_seal_not_before_sim_stamp_ns = 0;
    pending_seal_exact_stamp = false;
    active_bank_index = kInvalidBankIndex;
    active_bank_epoch = 0U;

    const auto terminal_state = sealed_bank->oracle_faults == 0U ?
      VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_SEALED_OK :
      VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_SEALED_FAULT;
    atomic_store_release(sealed_bank->state, terminal_state);
    publish_response(
      response_ticket,
      response_request_checksum,
      VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_OK,
      response_bank_index,
      response_bank_epoch,
      ticket.write_seq);
  }

  void process_control() noexcept
  {
    const auto request_state = atomic_load_acquire(control->request_state);
    if (
      request_state == VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_IDLE ||
      request_state == VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_WRITING)
    {
      return;
    }
    if (
      request_state == VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_READING &&
      has_pending_seal)
    {
      return;
    }
    if (request_state != VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_READY) {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      return;
    }
    std::uint64_t expected_state =
      VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_READY;
    if (!atomic_compare_exchange_acq_rel(
        control->request_state,
        expected_state,
        VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_READING))
    {
      return;
    }
    const ControlRequestSnapshot request{
      control->request_op,
      control->request_flags,
      control->request_interval_id,
      control->request_bank_index,
      control->request_bank_epoch,
      control->request_segment_budget,
      control->request_invocation_budget,
      control->request_not_before_sim_stamp_ns_bits,
      control->request_checksum,
      control->request_ticket};

    const auto response_ticket = atomic_load_acquire(control->response_ticket);
    const auto calculated_checksum = request_checksum(*header, request);
    if (has_pending_seal) {
      if (
        request.ticket != pending_seal_ticket ||
        request.checksum != pending_seal_checksum ||
        request.checksum != calculated_checksum)
      {
        latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
        auto * active_bank = bank(pending_seal_bank_index);
        active_bank->oracle_faults |=
          VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL;
      }
      return;
    }
    if (request.ticket == response_ticket) {
      if (
        request.ticket == 0U ||
        request.ticket != last_consumed_request_ticket ||
        request.checksum != last_consumed_request_checksum ||
        request.checksum != calculated_checksum ||
        control->response_request_checksum != request.checksum)
      {
        latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      }
      atomic_store_release(
        control->request_state,
        VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_IDLE);
      return;
    }
    if (
      request.ticket == 0U ||
      response_ticket == std::numeric_limits<std::uint64_t>::max() ||
      request.ticket != response_ticket + 1U)
    {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      atomic_store_release(
        control->request_state,
        VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_IDLE);
      return;
    }

    last_consumed_request_ticket = request.ticket;
    last_consumed_request_checksum = request.checksum;
    if (request.checksum != calculated_checksum) {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      publish_response(
        request.ticket,
        request.checksum,
        VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_INVALID,
        kInvalidBankIndex,
        0U,
        atomic_load_acquire(header->last_completed_write_seq));
      return;
    }

    const auto last_completed_write_seq =
      atomic_load_acquire(header->last_completed_write_seq);
    if (request.op == VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_ARM) {
      process_arm(request, last_completed_write_seq);
      return;
    }
    if (request.op == VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_SEAL) {
      process_seal(request);
      return;
    }
    latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
    publish_response(
      request.ticket,
      request.checksum,
      VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_INVALID,
      kInvalidBankIndex,
      0U,
      last_completed_write_seq);
  }

  HardwareWriteTicket begin_write(std::int64_t sim_stamp_ns) noexcept
  {
    std::uint64_t expected_lifecycle{kWriterIdle};
    if (!atomic_compare_exchange_acq_rel(
        lifecycle, expected_lifecycle, kWriterBeginning))
    {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      return HardwareWriteTicket{
        0U, sim_stamp_ns, kInvalidBankIndex, 0U, false};
    }
    process_control();
    const auto last_completed_write_seq =
      atomic_load_acquire(header->last_completed_write_seq);
    if (last_completed_write_seq == std::numeric_limits<std::uint64_t>::max()) {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_SEQUENCE);
      atomic_store_release(lifecycle, kWriterIdle);
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
    atomic_store_release(lifecycle, kWriterOutstanding);
    return outstanding_ticket;
  }

  void finish_write(
    HardwareWriteTicket ticket,
    std::uint64_t delegated_result,
    HardwareWriteWheelObservation observation) noexcept
  {
    std::uint64_t expected_lifecycle{kWriterOutstanding};
    if (!atomic_compare_exchange_acq_rel(
        lifecycle, expected_lifecycle, kWriterFinishing))
    {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      return;
    }
    if (
      ticket.write_seq == 0U ||
      ticket.write_seq != outstanding_ticket.write_seq ||
      ticket.sim_stamp_ns != outstanding_ticket.sim_stamp_ns ||
      ticket.bank_index != outstanding_ticket.bank_index ||
      ticket.bank_epoch != outstanding_ticket.bank_epoch ||
      ticket.included != outstanding_ticket.included)
    {
      latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
      atomic_store_release(lifecycle, kWriterOutstanding);
      return;
    }

    if (ticket.included) {
      const bool bank_index_valid =
        ticket.bank_index < VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_COUNT;
      auto * active_bank = bank_index_valid ? bank(ticket.bank_index) : nullptr;
      const bool bank_structure_valid = bank_index_valid &&
        atomic_load_acquire(active_bank->state) ==
        VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_ACTIVE &&
        active_bank->bank_epoch == ticket.bank_epoch &&
        active_bank->segment_budget != 0U &&
        active_bank->segment_budget <= header->segment_capacity_per_bank &&
        active_bank->invocation_budget != 0U &&
        active_bank->segment_count <= active_bank->segment_budget &&
        active_bank->segment_count <= header->segment_capacity_per_bank &&
        ((active_bank->invocation_count == 0U &&
        active_bank->segment_count == 0U &&
        active_bank->first_write_seq == 0U &&
        active_bank->last_write_seq == 0U) ||
        (active_bank->invocation_count != 0U &&
        active_bank->segment_count != 0U &&
        active_bank->first_write_seq ==
        active_bank->arm_fence_write_seq + 1U &&
        active_bank->last_write_seq >= active_bank->first_write_seq &&
        active_bank->invocation_count ==
        active_bank->last_write_seq - active_bank->arm_fence_write_seq));
      if (!bank_structure_valid) {
        active_bank->oracle_faults |=
          VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL;
        latch_global_fault(VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL);
        atomic_store_release(
          header->last_completed_write_seq, ticket.write_seq);
        complete_pending_seal(ticket);
        atomic_store_release(lifecycle, kWriterIdle);
        return;
      }

      std::uint64_t faults{0U};
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

      const auto packed_observation_and_result =
        (observation_value << 8U) | (delegated_result & UINT64_C(0xff));
      const bool sequence_is_contiguous =
        active_bank->invocation_count == 0U ?
        ticket.write_seq == active_bank->arm_fence_write_seq + 1U :
        active_bank->last_write_seq !=
        std::numeric_limits<std::uint64_t>::max() &&
        ticket.write_seq == active_bank->last_write_seq + 1U;
      if (!sequence_is_contiguous) {
        faults |= VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_SEQUENCE;
      }
      if (active_bank->invocation_count >= active_bank->invocation_budget) {
        faults |= VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_CAPACITY;
      }

      bool record_appended{false};
      if (
        sequence_is_contiguous &&
        active_bank->invocation_count < active_bank->invocation_budget)
      {
        if (active_bank->segment_count == 0U) {
          auto * first_segment = segment(ticket.bank_index, 0U);
          *first_segment = voice_nav_hardware_write_ledger_segment_v1{
            header->generation,
            ticket.write_seq,
            ticket.write_seq,
            1U,
            static_cast<std::uint64_t>(ticket.sim_stamp_ns),
            packed_observation_and_result,
            observation.left_command_bits,
            observation.right_command_bits};
          active_bank->segment_count = 1U;
          record_appended = true;
        } else {
          auto * last_segment = segment(
            ticket.bank_index, active_bank->segment_count - 1U);
          const bool last_segment_valid =
            last_segment->generation == header->generation &&
            last_segment->invocation_count != 0U &&
            last_segment->last_write_seq == active_bank->last_write_seq &&
            last_segment->last_write_seq >= last_segment->first_write_seq &&
            last_segment->invocation_count ==
            last_segment->last_write_seq -
            last_segment->first_write_seq + 1U;
          if (!last_segment_valid) {
            faults |= VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL;
          } else {
            if (
              ticket.sim_stamp_ns <
              sim_stamp_from_bits(last_segment->sim_stamp_ns_bits))
            {
              faults |= VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_SIM_STAMP;
            }
            if (
              segment_tuple_matches(
                *last_segment,
                header->generation,
                ticket.sim_stamp_ns,
                packed_observation_and_result,
                observation))
            {
              last_segment->last_write_seq = ticket.write_seq;
              ++last_segment->invocation_count;
              record_appended = true;
            } else if (
              active_bank->segment_count < active_bank->segment_budget &&
              active_bank->segment_count <
              header->segment_capacity_per_bank)
            {
              auto * next_segment = segment(
                ticket.bank_index, active_bank->segment_count);
              *next_segment = voice_nav_hardware_write_ledger_segment_v1{
                header->generation,
                ticket.write_seq,
                ticket.write_seq,
                1U,
                static_cast<std::uint64_t>(ticket.sim_stamp_ns),
                packed_observation_and_result,
                observation.left_command_bits,
                observation.right_command_bits};
              ++active_bank->segment_count;
              record_appended = true;
            } else {
              faults |= VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_CAPACITY;
            }
          }
        }
      }

      if (record_appended) {
        if (active_bank->invocation_count == 0U) {
          active_bank->first_write_seq = ticket.write_seq;
        }
        ++active_bank->invocation_count;
        active_bank->last_write_seq = ticket.write_seq;
      }
      active_bank->oracle_faults |= faults;
      if (faults != 0U) {
        latch_global_fault(faults);
      }
    }

    atomic_store_release(
      header->last_completed_write_seq, ticket.write_seq);
    complete_pending_seal(ticket);
    atomic_store_release(lifecycle, kWriterIdle);
  }

  void * region;
  std::size_t region_bytes;
  voice_nav_hardware_write_ledger_header_v1 * header;
  voice_nav_hardware_write_ledger_control_v1 * control;
  std::uint64_t bank_stride{0U};
  std::uint64_t active_bank_index{kInvalidBankIndex};
  std::uint64_t active_bank_epoch{0U};
  std::uint64_t last_consumed_request_ticket{0U};
  std::uint64_t last_consumed_request_checksum{0U};
  bool has_pending_seal{false};
  std::uint64_t pending_seal_ticket{0U};
  std::uint64_t pending_seal_checksum{0U};
  std::uint64_t pending_seal_bank_index{kInvalidBankIndex};
  std::uint64_t pending_seal_bank_epoch{0U};
  std::int64_t pending_seal_not_before_sim_stamp_ns{0};
  bool pending_seal_exact_stamp{false};
  std::uint64_t lifecycle{kWriterIdle};
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
