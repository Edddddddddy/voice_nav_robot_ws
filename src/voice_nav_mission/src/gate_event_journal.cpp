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

#include <stdexcept>

namespace voice_nav_mission
{

std::uint64_t gate_event_journal_header_checksum(
  const GateEventJournalHeader & header) noexcept
{
  (void)header;
  return 0U;
}

std::uint64_t gate_event_journal_intent_checksum(
  const GateEventJournalSlot & slot) noexcept
{
  (void)slot;
  return 0U;
}

std::uint64_t gate_event_journal_commit_checksum(
  const GateEventJournalSlot & slot) noexcept
{
  (void)slot;
  return 0U;
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
    throw std::invalid_argument("invalid GateEventJournal RED fixture");
  }
}

GateEventJournal::Reservation GateEventJournal::begin_output(
  const GateOutputIntent & intent)
{
  (void)intent;
  throw std::logic_error("VN-0011A tests-first RED");
}

GateOutputOutcome GateEventJournal::commit_output(
  const Reservation & reservation)
{
  return {reservation.journal_seq, reservation.slot_index};
}

}  // namespace voice_nav_mission
