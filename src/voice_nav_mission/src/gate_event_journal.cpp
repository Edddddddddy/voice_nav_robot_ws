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

#include "gate_event_journal.hpp"

#include <unistd.h>

#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace voice_nav_mission
{
namespace
{

constexpr std::uint64_t kCrc64EcmaPolynomial =
  UINT64_C(0x42f0e1eba9ea3693);

std::uint64_t crc64_ecma_byte(
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

std::uint64_t crc64_ecma_little_endian_word(
  std::uint64_t checksum,
  std::uint64_t value) noexcept
{
  for (std::uint8_t byte_index = 0U; byte_index < 8U; ++byte_index) {
    checksum = crc64_ecma_byte(
      checksum,
      static_cast<std::uint8_t>(value & UINT64_C(0xff)));
    value >>= 8U;
  }
  return checksum;
}

template<std::size_t Size>
std::uint64_t crc64_ecma_words(
  const std::array<std::uint64_t, Size> & words) noexcept
{
  std::uint64_t checksum = 0U;
  for (const auto word : words) {
    checksum = crc64_ecma_little_endian_word(checksum, word);
  }
  return checksum;
}

void latch_overflow(GateEventJournalHeader & header) noexcept
{
  gate_event_journal_store_release(header.overflow_latched, 1U);
}

}  // namespace

std::uint64_t gate_event_journal_header_checksum(
  const GateEventJournalHeader & header) noexcept
{
  return crc64_ecma_words(
    std::array<std::uint64_t, 11U>{
      header.magic,
      header.abi_version,
      header.header_bytes,
      header.slot_bytes,
      header.region_bytes,
      header.capacity,
      header.owner_uid,
      header.generation,
      header.nonce_hi,
      header.nonce_lo,
      header.reserved});
}

std::uint64_t gate_event_journal_intent_checksum(
  const GateEventJournalSlot & slot) noexcept
{
  return crc64_ecma_words(
    std::array<std::uint64_t, 20U>{
      slot.record_kind,
      slot.journal_seq,
      slot.generation,
      slot.intent_monotonic_ns,
      slot.event_code,
      slot.reason,
      slot.before_state_seq,
      slot.before_control_seq,
      slot.output_attempt_seq,
      slot.intended_output_seq,
      slot.ros_stamp_sec_bits,
      slot.ros_stamp_nanosec,
      slot.linear_x_bits,
      slot.angular_z_bits,
      slot.before_lease_hi,
      slot.before_lease_lo,
      slot.gate_instance_hi,
      slot.gate_instance_lo,
      slot.cause_transition_journal_seq,
      slot.flags});
}

std::uint64_t gate_event_journal_commit_checksum(
  const GateEventJournalSlot & slot) noexcept
{
  return crc64_ecma_words(
    std::array<std::uint64_t, 27U>{
      slot.intent_checksum,
      slot.record_kind,
      slot.journal_seq,
      slot.generation,
      slot.intent_monotonic_ns,
      slot.event_code,
      slot.reason,
      slot.before_state_seq,
      slot.before_control_seq,
      slot.output_attempt_seq,
      slot.intended_output_seq,
      slot.ros_stamp_sec_bits,
      slot.ros_stamp_nanosec,
      slot.linear_x_bits,
      slot.angular_z_bits,
      slot.before_lease_hi,
      slot.before_lease_lo,
      slot.gate_instance_hi,
      slot.gate_instance_lo,
      slot.cause_transition_journal_seq,
      slot.flags,
      slot.transition_linearization_ns,
      slot.commit_monotonic_ns,
      slot.after_state_seq,
      slot.after_control_seq,
      slot.after_lease_hi,
      slot.after_lease_lo});
}

std::uint64_t gate_event_journal_load_acquire(
  const std::uint64_t & value) noexcept
{
  return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

void gate_event_journal_store_release(
  std::uint64_t & destination,
  std::uint64_t value) noexcept
{
  __atomic_store_n(&destination, value, __ATOMIC_RELEASE);
}

GateEventJournal::GateEventJournal(
  void * region,
  std::size_t region_bytes,
  GateEventJournalIdentity expected_identity,
  GateEventJournalClock clock)
: header_(static_cast<GateEventJournalHeader *>(region)),
  slots_(region == nullptr ? nullptr : reinterpret_cast<GateEventJournalSlot *>(
      static_cast<std::byte *>(region) + sizeof(GateEventJournalHeader))),
  region_bytes_(region_bytes),
  expected_identity_(expected_identity),
  clock_(clock)
{
  if (
    region == nullptr ||
    region_bytes < sizeof(GateEventJournalHeader) + sizeof(GateEventJournalSlot) ||
    clock.read == nullptr)
  {
    throw std::invalid_argument("invalid GateEventJournal fixture");
  }
  if (
    reinterpret_cast<std::uintptr_t>(region) % alignof(GateEventJournalHeader) !=
    0U)
  {
    throw std::invalid_argument("unaligned GateEventJournal region");
  }

  const auto init_state = gate_event_journal_load_acquire(header_->init_state);
  const auto claimed_slots = gate_event_journal_load_acquire(
    header_->claimed_slots);
  const auto overflow_latched = gate_event_journal_load_acquire(
    header_->overflow_latched);
  const bool capacity_fits =
    header_->capacity <=
    (std::numeric_limits<std::size_t>::max() - sizeof(GateEventJournalHeader)) /
    sizeof(GateEventJournalSlot);
  const auto expected_region_bytes = capacity_fits ?
    sizeof(GateEventJournalHeader) +
    static_cast<std::size_t>(header_->capacity) * sizeof(GateEventJournalSlot) :
    0U;
  if (
    init_state != VOICE_NAV_GATE_EVENT_JOURNAL_INIT_READY ||
    header_->magic != VOICE_NAV_GATE_EVENT_JOURNAL_MAGIC ||
    header_->abi_version != VOICE_NAV_GATE_EVENT_JOURNAL_ABI_VERSION ||
    header_->header_bytes != sizeof(GateEventJournalHeader) ||
    header_->slot_bytes != sizeof(GateEventJournalSlot) ||
    header_->region_bytes != region_bytes_ ||
    header_->capacity == 0U ||
    !capacity_fits ||
    expected_region_bytes != region_bytes_ ||
    header_->owner_uid != expected_identity_.owner_uid ||
    header_->generation != expected_identity_.generation ||
    header_->nonce_hi != expected_identity_.nonce_hi ||
    header_->nonce_lo != expected_identity_.nonce_lo ||
    claimed_slots != 0U ||
    overflow_latched != 0U ||
    header_->reserved != 0U ||
    header_->header_checksum != gate_event_journal_header_checksum(*header_))
  {
    throw std::invalid_argument("GateEventJournal header validation failed");
  }
  for (std::size_t index = 0U;
    index < static_cast<std::size_t>(header_->capacity);
    ++index)
  {
    if (
      gate_event_journal_load_acquire(slots_[index].phase) !=
      VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_FREE)
    {
      throw std::invalid_argument("GateEventJournal contains an occupied slot");
    }
  }

  const auto writer_pid = static_cast<std::uint64_t>(getpid());
  std::uint64_t unclaimed_writer = 0U;
  if (!__atomic_compare_exchange_n(
      &header_->writer_pid,
      &unclaimed_writer,
      writer_pid,
      false,
      __ATOMIC_ACQ_REL,
      __ATOMIC_ACQUIRE))
  {
    throw std::runtime_error("GateEventJournal writer already claimed");
  }
}

GateEventJournal::Reservation GateEventJournal::reserve_slot()
{
  std::uint64_t slot_index = gate_event_journal_load_acquire(
    header_->claimed_slots);
  while (true) {
    if (slot_index >= header_->capacity || slot_index == UINT64_MAX) {
      latch_overflow(*header_);
      throw std::overflow_error("GateEventJournal capacity exhausted");
    }
    const auto next_slot = slot_index + 1U;
    if (__atomic_compare_exchange_n(
        &header_->claimed_slots,
        &slot_index,
        next_slot,
        false,
        __ATOMIC_ACQ_REL,
        __ATOMIC_ACQUIRE))
    {
      break;
    }
  }

  auto & slot = slots_[slot_index];
  if (
    gate_event_journal_load_acquire(slot.phase) !=
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_FREE)
  {
    latch_overflow(*header_);
    throw std::runtime_error("GateEventJournal claimed a non-free slot");
  }

  std::memset(
    &slot.record_kind,
    0,
    sizeof(GateEventJournalSlot) - offsetof(GateEventJournalSlot, record_kind));

  return {&slot, slot_index + 1U, slot_index};
}

GateEventJournal::Reservation GateEventJournal::begin_output(
  const GateOutputIntent & intent)
{
  const auto reservation = reserve_slot();
  auto & slot = *reservation.slot;
  slot.record_kind = VOICE_NAV_GATE_EVENT_JOURNAL_KIND_OUTPUT_ATTEMPT;
  slot.journal_seq = reservation.journal_seq;
  slot.generation = header_->generation;
  slot.intent_monotonic_ns = clock_.read(clock_.context);
  slot.event_code = intent.event_code;
  slot.reason = intent.reason;
  slot.output_attempt_seq = intent.output_attempt_seq;
  slot.intended_output_seq = intent.intended_output_seq;
  slot.ros_stamp_sec_bits = intent.ros_stamp_sec_bits;
  slot.ros_stamp_nanosec = intent.ros_stamp_nanosec;
  slot.linear_x_bits = intent.linear_x_bits;
  slot.angular_z_bits = intent.angular_z_bits;
  slot.gate_instance_hi = intent.gate_instance_hi;
  slot.gate_instance_lo = intent.gate_instance_lo;
  slot.cause_transition_journal_seq = intent.cause_transition_journal_seq;
  slot.flags = intent.flags;
  slot.intent_checksum = gate_event_journal_intent_checksum(slot);
  gate_event_journal_store_release(
    slot.phase,
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT);

  return reservation;
}

GateOutputOutcome GateEventJournal::commit_output(
  const Reservation & reservation)
{
  reservation.slot->commit_monotonic_ns = clock_.read(clock_.context);
  reservation.slot->commit_checksum =
    gate_event_journal_commit_checksum(*reservation.slot);
  gate_event_journal_store_release(
    reservation.slot->phase,
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  return {reservation.journal_seq, reservation.slot_index};
}

GateEventJournal::Reservation GateEventJournal::begin_transition(
  const GateTransitionIntent & intent)
{
  const auto reservation = reserve_slot();
  auto & slot = *reservation.slot;
  slot.record_kind = VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION;
  slot.journal_seq = reservation.journal_seq;
  slot.generation = header_->generation;
  slot.intent_monotonic_ns = clock_.read(clock_.context);
  slot.event_code = intent.event_code;
  slot.reason = intent.reason;
  slot.before_state_seq = intent.before_state_seq;
  slot.before_control_seq = intent.before_control_seq;
  slot.before_lease_hi = intent.before_lease_hi;
  slot.before_lease_lo = intent.before_lease_lo;
  slot.gate_instance_hi = intent.gate_instance_hi;
  slot.gate_instance_lo = intent.gate_instance_lo;
  slot.flags = intent.flags;
  slot.intent_checksum = gate_event_journal_intent_checksum(slot);
  return reservation;
}

void GateEventJournal::mark_transition_linearization(
  const Reservation & reservation) noexcept
{
  reservation.slot->transition_linearization_ns =
    clock_.read(clock_.context);
  gate_event_journal_store_release(
    reservation.slot->phase,
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT);
}

GateTransitionOutcome GateEventJournal::commit_transition(
  const Reservation & reservation,
  const GateTransitionAfter & after) noexcept
{
  reservation.slot->after_state_seq = after.after_state_seq;
  reservation.slot->after_control_seq = after.after_control_seq;
  reservation.slot->after_lease_hi = after.after_lease_hi;
  reservation.slot->after_lease_lo = after.after_lease_lo;
  reservation.slot->commit_monotonic_ns = clock_.read(clock_.context);
  reservation.slot->commit_checksum =
    gate_event_journal_commit_checksum(*reservation.slot);
  gate_event_journal_store_release(
    reservation.slot->phase,
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  return {reservation.journal_seq, reservation.slot_index};
}

}  // namespace voice_nav_mission
