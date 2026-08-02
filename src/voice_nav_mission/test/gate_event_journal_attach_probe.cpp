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

#include <charconv>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include "attached_gate_event_journal.hpp"  // NOLINT(build/include_subdir)

namespace
{

constexpr std::size_t kNonceCharacters = 32U;

struct DeterministicClock
{
  std::uint64_t next_value{100U};

  static std::uint64_t read(void * context) noexcept
  {
    auto & clock = *static_cast<DeterministicClock *>(context);
    const auto value = clock.next_value;
    clock.next_value += 100U;
    return value;
  }
};

std::uint64_t parse_word(
  std::string_view text,
  int base,
  const char * description)
{
  std::uint64_t value = 0U;
  const auto result = std::from_chars(
    text.data(), text.data() + text.size(), value, base);
  if (
    text.empty() || result.ec != std::errc{} ||
    result.ptr != text.data() + text.size())
  {
    throw std::invalid_argument(description);
  }
  return value;
}

bool is_lower_hex(char character) noexcept
{
  return
    (character >= '0' && character <= '9') ||
    (character >= 'a' && character <= 'f');
}

voice_nav_mission::GateEventJournalIdentity parse_identity(
  std::string_view uid,
  std::string_view generation,
  std::string_view nonce)
{
  if (nonce.size() != kNonceCharacters) {
    throw std::invalid_argument("nonce must contain 32 lowercase hex digits");
  }
  for (const auto character : nonce) {
    if (!is_lower_hex(character)) {
      throw std::invalid_argument(
              "nonce must contain 32 lowercase hex digits");
    }
  }
  return {
    parse_word(uid, 10, "owner UID is not a uint64"),
    parse_word(generation, 10, "generation is not a uint64"),
    parse_word(nonce.substr(0U, 16U), 16, "nonce high word is invalid"),
    parse_word(nonce.substr(16U, 16U), 16, "nonce low word is invalid")};
}

int run(int argc, char ** argv)
{
  if (argc != 6) {
    throw std::invalid_argument(
            "usage: probe NAME OWNER_UID GENERATION CAPACITY NONCE");
  }

  DeterministicClock clock;
  voice_nav_mission::AttachedGateEventJournal attached(
    voice_nav_mission::GateEventJournalAttachmentConfig{
      argv[1],
      parse_identity(argv[2], argv[3], argv[5]),
      parse_word(argv[4], 10, "capacity is not a uint64"),
      voice_nav_mission::GateEventJournalClock{
        &DeterministicClock::read, &clock}});

  char release = 0;
  if (!std::cin.read(&release, 1)) {
    throw std::runtime_error("parent release byte was not received");
  }

  const auto outcome = attached.journal().publish_output(
    voice_nav_mission::GateOutputIntent{
      41U,
      9U,
      17U,
      18U,
      23U,
      456U,
      0U,
      0U,
      UINT64_C(0x1111222233334444),
      UINT64_C(0x5555666677778888),
      3U,
      0xa5U},
    []() noexcept {});
  if (outcome.journal_seq != 1U || outcome.slot_index != 0U) {
    throw std::runtime_error("unexpected journal reservation outcome");
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
