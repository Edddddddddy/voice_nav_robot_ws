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

#include <unistd.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <system_error>
#include <type_traits>
#include <utility>

#include "attached_gate_event_journal.hpp"
#include "gate_event_journal.hpp"
#include "gate_event_journal_test_region.hpp"

namespace voice_nav_mission
{
namespace
{

static_assert(!std::is_copy_constructible_v<AttachedGateEventJournal>);
static_assert(!std::is_copy_assignable_v<AttachedGateEventJournal>);
static_assert(!std::is_move_constructible_v<AttachedGateEventJournal>);
static_assert(!std::is_move_assignable_v<AttachedGateEventJournal>);

struct FakeClock
{
  std::uint64_t next_value{100U};

  static std::uint64_t read(void * context) noexcept
  {
    auto & clock = *static_cast<FakeClock *>(context);
    const auto value = clock.next_value;
    clock.next_value += 100U;
    return value;
  }
};

GateEventJournalAttachmentConfig attachment_config(
  const OwnedJournalRegion & owner,
  FakeClock & clock)
{
  return {
    owner.name(),
    owner.identity(),
    owner.capacity(),
    GateEventJournalClock{&FakeClock::read, &clock}};
}

TEST(AttachedGateEventJournal, ExistingMappingSurvivesOwnerUnlink)
{
  OwnedJournalRegion owner(2U);
  FakeClock clock;
  AttachedGateEventJournal attached(attachment_config(owner, clock));

  EXPECT_EQ(
    gate_event_journal_load_acquire(owner.header().writer_pid),
    static_cast<std::uint64_t>(getpid()));
  owner.unlink_name();
  EXPECT_THROW(
    (void)AttachedGateEventJournal(attachment_config(owner, clock)),
    std::system_error);

  const auto outcome = attached.journal().publish_output(
    GateOutputIntent{
        41U, 9U, 0U, 0U, 17U, 18U, 23U, 456U,
        UINT64_C(0x3fd0000000000000),
        UINT64_C(0xbfc0000000000000),
        0U, 0U,
        UINT64_C(0x1111222233334444),
        UINT64_C(0x5555666677778888),
        3U, 0xa5U},
    []() noexcept {});

  EXPECT_EQ(outcome.journal_seq, 1U);
  EXPECT_EQ(
    gate_event_journal_load_acquire(owner.slot(0U).phase),
    VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED);
  EXPECT_EQ(
    owner.slot(0U).commit_checksum,
    gate_event_journal_commit_checksum(owner.slot(0U)));
}

TEST(AttachedGateEventJournal, WrongExpectedGenerationDoesNotClaimWriter)
{
  OwnedJournalRegion owner(1U);
  FakeClock clock;
  auto config = attachment_config(owner, clock);
  ++config.expected_identity.generation;

  EXPECT_THROW(
    (void)AttachedGateEventJournal(std::move(config)),
    std::invalid_argument);
  EXPECT_EQ(gate_event_journal_load_acquire(owner.header().writer_pid), 0U);
}

TEST(AttachedGateEventJournal, WrongExpectedIdentityDoesNotClaimWriter)
{
  OwnedJournalRegion owner(1U);
  FakeClock clock;
  auto config = attachment_config(owner, clock);
  config.expected_identity.nonce_lo ^= 1U;

  EXPECT_THROW(
    (void)AttachedGateEventJournal(std::move(config)),
    std::invalid_argument);
  EXPECT_EQ(gate_event_journal_load_acquire(owner.header().writer_pid), 0U);
}

TEST(AttachedGateEventJournal, WrongExpectedCapacityDoesNotClaimWriter)
{
  OwnedJournalRegion owner(1U);
  FakeClock clock;
  auto config = attachment_config(owner, clock);
  ++config.expected_capacity;

  EXPECT_THROW(
    (void)AttachedGateEventJournal(std::move(config)),
    std::invalid_argument);
  EXPECT_EQ(gate_event_journal_load_acquire(owner.header().writer_pid), 0U);
}

TEST(AttachedGateEventJournal, ZeroExpectedNonceDoesNotClaimWriter)
{
  OwnedJournalRegion owner(1U);
  FakeClock clock;
  auto config = attachment_config(owner, clock);
  config.expected_identity.nonce_hi = 0U;
  config.expected_identity.nonce_lo = 0U;

  EXPECT_THROW(
    (void)AttachedGateEventJournal(std::move(config)),
    std::invalid_argument);
  EXPECT_EQ(gate_event_journal_load_acquire(owner.header().writer_pid), 0U);
}

TEST(AttachedGateEventJournal, NonPrivateModeDoesNotClaimWriter)
{
  OwnedJournalRegion owner(1U);
  owner.set_mode(0640);
  FakeClock clock;

  EXPECT_THROW(
    (void)AttachedGateEventJournal(attachment_config(owner, clock)),
    std::invalid_argument);
  EXPECT_EQ(gate_event_journal_load_acquire(owner.header().writer_pid), 0U);
}

TEST(AttachedGateEventJournal, InvalidNameDoesNotClaimWriter)
{
  OwnedJournalRegion owner(1U);
  FakeClock clock;
  auto config = attachment_config(owner, clock);
  config.shared_memory_name = "/VOICE_NAV_GATE_0123456789abcdef0123456789abcdef";

  EXPECT_THROW(
    (void)AttachedGateEventJournal(std::move(config)),
    std::invalid_argument);
  EXPECT_EQ(gate_event_journal_load_acquire(owner.header().writer_pid), 0U);
}

}  // namespace
}  // namespace voice_nav_mission
