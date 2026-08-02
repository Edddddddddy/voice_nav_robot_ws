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

#include "motion_gate_process_runtime.hpp"

#include <time.h>
#include <unistd.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace voice_nav_mission
{
namespace
{

std::uint64_t read_monotonic_nanoseconds(void * context) noexcept
{
  static_cast<void>(context);
  struct timespec timestamp {};
  if (
    clock_gettime(CLOCK_MONOTONIC, &timestamp) != 0 ||
    timestamp.tv_sec < 0 || timestamp.tv_nsec < 0 ||
    timestamp.tv_nsec >= 1000000000L)
  {
    return 0U;
  }

  constexpr std::uint64_t nanoseconds_per_second = UINT64_C(1000000000);
  const auto seconds = static_cast<std::uint64_t>(timestamp.tv_sec);
  const auto nanoseconds = static_cast<std::uint64_t>(timestamp.tv_nsec);
  if (
    seconds >
    (std::numeric_limits<std::uint64_t>::max() - nanoseconds) /
    nanoseconds_per_second)
  {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return seconds * nanoseconds_per_second + nanoseconds;
}

std::array<std::string, 5U> split_descriptor(
  const std::string & descriptor)
{
  std::array<std::string, 5U> fields;
  std::size_t start = 0U;
  for (std::size_t index = 0U; index < fields.size(); ++index) {
    const auto separator = descriptor.find(':', start);
    const bool final_field = index + 1U == fields.size();
    if (
      (!final_field && separator == std::string::npos) ||
      (final_field && separator != std::string::npos))
    {
      throw std::invalid_argument(
              "Gate event journal descriptor must have five fields");
    }
    const auto end = final_field ? descriptor.size() : separator;
    fields[index] = descriptor.substr(start, end - start);
    start = end + 1U;
  }
  return fields;
}

std::uint64_t parse_unsigned(
  const std::string & text,
  int base,
  const char * field_name)
{
  std::size_t consumed = 0U;
  std::uint64_t value = 0U;
  try {
    value = static_cast<std::uint64_t>(
      std::stoull(text, &consumed, base));
  } catch (const std::exception &) {
    throw std::invalid_argument(
            std::string{"Invalid Gate event journal "} + field_name);
  }
  if (consumed != text.size()) {
    throw std::invalid_argument(
            std::string{"Invalid Gate event journal "} + field_name);
  }
  return static_cast<std::uint64_t>(value);
}

}  // namespace

std::optional<GateEventJournalAttachmentConfig>
parse_gate_event_journal_test_parameters(
  const GateEventJournalTestParameters & parameters)
{
  if (parameters.name.empty() && parameters.descriptor.empty()) {
    return std::nullopt;
  }
  if (parameters.name.empty() || parameters.descriptor.empty()) {
    throw std::invalid_argument(
            "Gate event journal test parameters must be all-or-none");
  }

  const auto fields = split_descriptor(parameters.descriptor);
  if (fields[0] != "v1") {
    throw std::invalid_argument(
            "Unsupported Gate event journal descriptor version");
  }
  const auto owner_uid = parse_unsigned(fields[1], 10, "owner UID");
  const auto generation = parse_unsigned(fields[2], 10, "generation");
  const auto capacity = parse_unsigned(fields[3], 10, "capacity");
  if (owner_uid != static_cast<std::uint64_t>(geteuid())) {
    throw std::invalid_argument(
            "Gate event journal owner UID must match the process");
  }
  if (generation == 0U) {
    throw std::invalid_argument(
            "Gate event journal generation must be nonzero");
  }
  if (capacity == 0U || capacity > 16384U) {
    throw std::invalid_argument(
            "Gate event journal capacity is out of bounds");
  }
  if (fields[4].size() != 32U) {
    throw std::invalid_argument(
            "Gate event journal nonce must contain 32 hex digits");
  }
  const auto nonce_hi = parse_unsigned(fields[4].substr(0U, 16U), 16, "nonce");
  const auto nonce_lo = parse_unsigned(fields[4].substr(16U), 16, "nonce");
  if (nonce_hi == 0U && nonce_lo == 0U) {
    throw std::invalid_argument(
            "Gate event journal nonce must be nonzero");
  }

  return GateEventJournalAttachmentConfig{
    parameters.name,
    GateEventJournalIdentity{
      owner_uid,
      generation,
      nonce_hi,
      nonce_lo},
    capacity,
    GateEventJournalClock{&read_monotonic_nanoseconds, nullptr}};
}

}  // namespace voice_nav_mission
