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

#ifndef GATE_EVENT_JOURNAL_HPP_
#define GATE_EVENT_JOURNAL_HPP_

#include <cstddef>
#include <cstdint>
#include <utility>

#include "gate_event_journal_abi.h"  // NOLINT(build/include_subdir)

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "GateEventJournal ABI v1 requires a little-endian target"
#endif

namespace voice_nav_mission
{

using GateEventJournalHeader = voice_nav_gate_event_journal_header_v1;
using GateEventJournalSlot = voice_nav_gate_event_journal_slot_v1;

static_assert(sizeof(GateEventJournalHeader) == 128U);
static_assert(alignof(GateEventJournalHeader) == alignof(std::uint64_t));
static_assert(sizeof(GateEventJournalSlot) == 256U);
static_assert(alignof(GateEventJournalSlot) == alignof(std::uint64_t));
static_assert(__atomic_always_lock_free(sizeof(std::uint64_t), nullptr));

struct GateEventJournalIdentity
{
  std::uint64_t owner_uid;
  std::uint64_t generation;
  std::uint64_t nonce_hi;
  std::uint64_t nonce_lo;
};

struct GateEventJournalClock
{
  // cpplint mistakes this noexcept function type for a C-style cast.
  using ReadFunction = std::uint64_t(void *) noexcept;  // NOLINT(readability/casting)

  ReadFunction * read;
  void * context;
};

struct GateOutputIntent
{
  std::uint64_t event_code;
  std::uint64_t reason;
  std::uint64_t output_attempt_seq;
  std::uint64_t intended_output_seq;
  std::uint64_t ros_stamp_sec_bits;
  std::uint64_t ros_stamp_nanosec;
  std::uint64_t linear_x_bits;
  std::uint64_t angular_z_bits;
  std::uint64_t gate_instance_hi;
  std::uint64_t gate_instance_lo;
  std::uint64_t cause_transition_journal_seq;
  std::uint64_t flags;
};

struct GateOutputOutcome
{
  std::uint64_t journal_seq;
  std::uint64_t slot_index;
};

struct GateTransitionIntent
{
  std::uint64_t event_code;
  std::uint64_t reason;
  std::uint64_t before_state_seq;
  std::uint64_t before_control_seq;
  std::uint64_t before_lease_hi;
  std::uint64_t before_lease_lo;
  std::uint64_t gate_instance_hi;
  std::uint64_t gate_instance_lo;
  std::uint64_t flags;
};

struct GateTransitionAfter
{
  std::uint64_t after_state_seq;
  std::uint64_t after_control_seq;
  std::uint64_t after_lease_hi;
  std::uint64_t after_lease_lo;
};

struct GateTransitionOutcome
{
  std::uint64_t journal_seq;
  std::uint64_t slot_index;
};

std::uint64_t gate_event_journal_header_checksum(
  const GateEventJournalHeader & header) noexcept;

std::uint64_t gate_event_journal_intent_checksum(
  const GateEventJournalSlot & slot) noexcept;

std::uint64_t gate_event_journal_commit_checksum(
  const GateEventJournalSlot & slot) noexcept;

std::uint64_t gate_event_journal_load_acquire(
  const std::uint64_t & value) noexcept;

void gate_event_journal_store_release(
  std::uint64_t & destination,
  std::uint64_t value) noexcept;

class GateEventJournal
{
public:
  GateEventJournal(
    void * region,
    std::size_t region_bytes,
    GateEventJournalIdentity expected_identity,
    GateEventJournalClock clock);

  GateEventJournal(const GateEventJournal &) = delete;
  GateEventJournal & operator=(const GateEventJournal &) = delete;
  GateEventJournal(GateEventJournal &&) = delete;
  GateEventJournal & operator=(GateEventJournal &&) = delete;

  template<typename Publisher>
  GateOutputOutcome publish_output(
    const GateOutputIntent & intent,
    Publisher && publisher)
  {
    const auto reservation = begin_output(intent);
    std::forward<Publisher>(publisher)();
    return commit_output(reservation);
  }

  template<typename Transition>
  GateTransitionOutcome apply_transition(
    const GateTransitionIntent & intent,
    Transition && transition)
  {
    const auto reservation = begin_transition(intent);
    mark_transition_linearization(reservation);
    const GateTransitionAfter after =
      std::forward<Transition>(transition)();
    return commit_transition(reservation, after);
  }

private:
  struct Reservation
  {
    GateEventJournalSlot * slot;
    std::uint64_t journal_seq;
    std::uint64_t slot_index;
  };

  Reservation begin_output(const GateOutputIntent & intent);
  GateOutputOutcome commit_output(const Reservation & reservation);
  Reservation begin_transition(const GateTransitionIntent & intent);
  void mark_transition_linearization(const Reservation & reservation);
  GateTransitionOutcome commit_transition(
    const Reservation & reservation,
    const GateTransitionAfter & after);

  GateEventJournalHeader * header_;
  GateEventJournalSlot * slots_;
  std::size_t region_bytes_;
  GateEventJournalIdentity expected_identity_;
  GateEventJournalClock clock_;
};

}  // namespace voice_nav_mission

#endif  // GATE_EVENT_JOURNAL_HPP_
