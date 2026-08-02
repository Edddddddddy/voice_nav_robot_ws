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
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string_view>
#include <stdexcept>
#include <string>
#include <utility>
#include <system_error>

namespace voice_nav_mission
{
namespace
{

constexpr std::uint64_t kFinalCommandPublishEvent = 1U;

struct IdentifierWords
{
  std::uint64_t hi{0U};
  std::uint64_t lo{0U};
};

IdentifierWords identifier_words(std::string_view identifier) noexcept
{
  if (identifier.empty() || identifier.size() != 32U) {
    return {};
  }

  IdentifierWords words;
  for (std::size_t index = 0U; index < identifier.size(); ++index) {
    const auto character = identifier[index];
    const auto nibble = character <= '9' ?
      static_cast<std::uint64_t>(character - '0') :
      static_cast<std::uint64_t>(character - 'a' + 10);
    auto & word = index < 16U ? words.hi : words.lo;
    word = (word << 4U) | nibble;
  }
  return words;
}

std::uint64_t double_bits(double value) noexcept
{
  static_assert(sizeof(double) == sizeof(std::uint64_t));
  static_assert(std::numeric_limits<double>::is_iec559);
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

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

bool is_decimal_digit(char character) noexcept
{
  return character >= '0' && character <= '9';
}

bool is_lower_hex_digit(char character) noexcept
{
  return
    is_decimal_digit(character) ||
    (character >= 'a' && character <= 'f');
}

void validate_shared_memory_name(const std::string & name)
{
  constexpr char prefix[] = "/voice_nav_gate_";
  constexpr std::size_t suffix_characters = 32U;
  const std::string expected_prefix{prefix};
  if (
    name.size() != expected_prefix.size() + suffix_characters ||
    name.compare(0U, expected_prefix.size(), expected_prefix) != 0)
  {
    throw std::invalid_argument(
            "Invalid Gate event journal shared-memory name");
  }
  for (std::size_t index = expected_prefix.size(); index < name.size(); ++index) {
    if (!is_lower_hex_digit(name[index])) {
      throw std::invalid_argument(
              "Invalid Gate event journal shared-memory name");
    }
  }
}

std::uint64_t parse_canonical_decimal(
  const std::string & text,
  const char * field_name)
{
  if (
    text.empty() ||
    (text.size() > 1U && text.front() == '0'))
  {
    throw std::invalid_argument(
            std::string{"Invalid Gate event journal "} + field_name);
  }
  for (const auto character : text) {
    if (!is_decimal_digit(character)) {
      throw std::invalid_argument(
              std::string{"Invalid Gate event journal "} + field_name);
    }
  }

  std::uint64_t value = 0U;
  const auto result = std::from_chars(
    text.data(), text.data() + text.size(), value, 10);
  if (
    result.ec != std::errc{} ||
    result.ptr != text.data() + text.size())
  {
    throw std::invalid_argument(
            std::string{"Invalid Gate event journal "} + field_name);
  }
  return value;
}

std::uint64_t parse_lower_hex_word(
  const std::string & text)
{
  for (const auto character : text) {
    if (!is_lower_hex_digit(character)) {
      throw std::invalid_argument("Invalid Gate event journal nonce");
    }
  }

  std::uint64_t value = 0U;
  const auto result = std::from_chars(
    text.data(), text.data() + text.size(), value, 16);
  if (
    result.ec != std::errc{} ||
    result.ptr != text.data() + text.size())
  {
    throw std::invalid_argument(
            "Invalid Gate event journal nonce");
  }
  return value;
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

  validate_shared_memory_name(parameters.name);
  const auto fields = split_descriptor(parameters.descriptor);
  if (fields[0] != "v1") {
    throw std::invalid_argument(
            "Unsupported Gate event journal descriptor version");
  }
  const auto owner_uid = parse_canonical_decimal(fields[1], "owner UID");
  const auto generation = parse_canonical_decimal(fields[2], "generation");
  const auto capacity = parse_canonical_decimal(fields[3], "capacity");
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
  const auto nonce_hi = parse_lower_hex_word(fields[4].substr(0U, 16U));
  const auto nonce_lo = parse_lower_hex_word(fields[4].substr(16U));
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

MotionGateProcessRuntime::MotionGateProcessRuntime(
  MotionGateConfig config,
  std::string gate_instance_id,
  GateEventJournalTestParameters journal_parameters)
{
  auto attachment_config =
    parse_gate_event_journal_test_parameters(journal_parameters);
  if (attachment_config.has_value()) {
    attached_journal_ = std::make_unique<AttachedGateEventJournal>(
      std::move(*attachment_config));
    core_ = std::make_unique<MotionGateCore>(
      std::move(config),
      std::move(gate_instance_id),
      0U,
      attached_journal_->journal().claim_transition_binding());
    output_journal_mode_ = OutputJournalMode::Usable;
    return;
  }

  core_ = std::make_unique<MotionGateCore>(
    std::move(config), std::move(gate_instance_id));
}

MotionGateProcessRuntime::~MotionGateProcessRuntime() = default;

MotionGateCore & MotionGateProcessRuntime::core() noexcept
{
  return *core_;
}

FinalOutputState MotionGateProcessRuntime::output_state_unlocked() const noexcept
{
  return FinalOutputState{
    output_publish_seq_,
    zero_publish_seq_,
    last_publication_was_zero_};
}

FinalOutputState MotionGateProcessRuntime::output_state() const
{
  std::scoped_lock lock(output_mutex_);
  return output_state_unlocked();
}

bool MotionGateProcessRuntime::try_force_fault(
  Reason reason,
  const char * detail) noexcept
{
  try {
    core_->force_fault(reason, detail);
    return true;
  } catch (...) {
    return false;
  }
}

std::uint64_t MotionGateProcessRuntime::pending_terminal_cause(
  const Command & command) const noexcept
{
  if (!command.is_zero()) {
    return 0U;
  }
  if (core_->state_ != State::Inhibited && core_->state_ != State::Faulted) {
    return 0U;
  }
  const auto cause = core_->last_terminal_transition_journal_seq_;
  if (cause == 0U || cause == last_consumed_terminal_cause_seq_) {
    return 0U;
  }
  return cause;
}

void MotionGateProcessRuntime::record_success(
  const Command & command,
  std::uint64_t terminal_cause) noexcept
{
  if (output_publish_seq_ != std::numeric_limits<std::uint64_t>::max()) {
    ++output_publish_seq_;
  }
  last_publication_was_zero_ = command.is_zero();
  if (!last_publication_was_zero_) {
    return;
  }
  zero_publish_seq_ = output_publish_seq_;
  if (terminal_cause != 0U) {
    last_consumed_terminal_cause_seq_ = terminal_cause;
  }
}

GateOutputIntent MotionGateProcessRuntime::make_output_intent(
  const FinalOutputFrame & frame,
  std::uint64_t terminal_cause) const noexcept
{
  const auto lease = identifier_words(core_->lease_id_);
  const auto gate = identifier_words(core_->gate_instance_id_);
  return GateOutputIntent{
    kFinalCommandPublishEvent,
    static_cast<std::uint64_t>(core_->reason_),
    core_->state_seq_,
    core_->control_seq_,
    output_attempt_seq_,
    output_publish_seq_ + 1U,
    static_cast<std::uint64_t>(
      static_cast<std::int64_t>(frame.stamp_sec)),
    static_cast<std::uint64_t>(frame.stamp_nanosec),
    double_bits(frame.command.linear_x),
    double_bits(frame.command.angular_z),
    lease.hi,
    lease.lo,
    gate.hi,
    gate.lo,
    terminal_cause,
    0U};
}

FinalOutputResult MotionGateProcessRuntime::publish_direct_zero(
  FinalOutputTime time,
  FinalOutputPublisher publisher,
  FinalOutputFailure failure,
  bool fallback_attempted)
{
  const Command zero{};
  const auto terminal_cause = pending_terminal_cause(zero);
  const FinalOutputFrame frame{
    time.stamp_sec,
    time.stamp_nanosec,
    zero};
  FinalOutputResult result;
  result.failure = failure;
  result.fallback_attempted = fallback_attempted;
  try {
    publisher.publish(publisher.context, frame);
  } catch (...) {
    result.failure = FinalOutputFailure::DirectZeroDdsFailure;
    result.state = output_state_unlocked();
    return result;
  }

  record_success(zero, terminal_cause);
  result.state = output_state_unlocked();
  result.published = true;
  result.zero_published = true;
  result.locally_consumed_terminal_cause_seq = terminal_cause;
  return result;
}

FinalOutputResult MotionGateProcessRuntime::publish_final_command(
  FinalOutputTime time,
  FinalOutputPublisher publisher)
{
  std::scoped_lock lock(output_mutex_);
  FinalOutputFailure failure = FinalOutputFailure::None;
  if (publisher.publish == nullptr) {
    if (!try_force_fault(
        Reason::InternalFailure,
        "final command publisher adapter is missing"))
    {
      output_journal_mode_ = OutputJournalMode::Retired;
      retired_failure_ = FinalOutputFailure::RuntimeInvariant;
    }
    FinalOutputResult result;
    result.failure = FinalOutputFailure::RuntimeInvariant;
    result.state = output_state_unlocked();
    return result;
  }

  if (!time.simulation_time_active || time.stamp_nanosec >= 1000000000U) {
    time.stamp_sec = 0;
    time.stamp_nanosec = 0U;
    failure = FinalOutputFailure::RuntimeInvariant;
    if (!try_force_fault(
        Reason::ConfigurationInvalid,
        "simulation command clock invariant was violated"))
    {
      output_journal_mode_ = OutputJournalMode::Retired;
      retired_failure_ = failure;
      return publish_direct_zero(
        time, publisher, failure, true);
    }
  }

  const bool success_sequence_exhausted =
    output_publish_seq_ == std::numeric_limits<std::uint64_t>::max();
  const bool attempt_sequence_exhausted =
    output_journal_mode_ == OutputJournalMode::Usable &&
    output_attempt_seq_ == std::numeric_limits<std::uint64_t>::max();
  if (success_sequence_exhausted || attempt_sequence_exhausted) {
    output_journal_mode_ = OutputJournalMode::Retired;
    retired_failure_ = FinalOutputFailure::SequenceExhausted;
    (void)try_force_fault(
      Reason::SequenceExhausted,
      "final command publication sequence exhausted");
    return publish_direct_zero(
      time,
      publisher,
      FinalOutputFailure::SequenceExhausted,
      true);
  }

  if (output_journal_mode_ == OutputJournalMode::Retired) {
    return publish_direct_zero(
      time, publisher, retired_failure_, false);
  }

  const auto selected = core_->selected_;
  if (output_journal_mode_ == OutputJournalMode::Disabled) {
    const auto terminal_cause = pending_terminal_cause(selected);
    const FinalOutputFrame frame{
      time.stamp_sec,
      time.stamp_nanosec,
      selected};
    try {
      publisher.publish(publisher.context, frame);
    } catch (...) {
      if (!try_force_fault(
          Reason::PublishFailed,
          "final command DDS publication failed"))
      {
        output_journal_mode_ = OutputJournalMode::Retired;
        retired_failure_ = FinalOutputFailure::DdsFailure;
      }
      return publish_direct_zero(
        time,
        publisher,
        FinalOutputFailure::DdsFailure,
        true);
    }
    record_success(selected, terminal_cause);
    FinalOutputResult result;
    result.state = output_state_unlocked();
    result.failure = failure;
    result.published = true;
    result.zero_published = selected.is_zero();
    result.locally_consumed_terminal_cause_seq = terminal_cause;
    return result;
  }

  ++output_attempt_seq_;
  const auto terminal_cause = pending_terminal_cause(selected);
  const FinalOutputFrame frame{
    time.stamp_sec,
    time.stamp_nanosec,
    selected};
  const auto intent = make_output_intent(frame, terminal_cause);
  bool publisher_entered = false;
  try {
    attached_journal_->journal().publish_output(
      intent,
      [&publisher, &frame, &publisher_entered]() {
        publisher_entered = true;
        publisher.publish(publisher.context, frame);
      });
  } catch (...) {
    output_journal_mode_ = OutputJournalMode::Retired;
    retired_failure_ = publisher_entered ?
      FinalOutputFailure::DdsFailure :
      FinalOutputFailure::JournalFailure;
    (void)try_force_fault(
      publisher_entered ? Reason::PublishFailed : Reason::InternalFailure,
      publisher_entered ?
      "final command DDS publication failed" :
      "final command journal reservation failed");
    return publish_direct_zero(
      time, publisher, retired_failure_, true);
  }

  record_success(selected, terminal_cause);
  FinalOutputResult result;
  result.state = output_state_unlocked();
  result.failure = failure;
  result.published = true;
  result.zero_published = selected.is_zero();
  result.journal_committed = true;
  result.locally_consumed_terminal_cause_seq = terminal_cause;
  return result;
}

}  // namespace voice_nav_mission
