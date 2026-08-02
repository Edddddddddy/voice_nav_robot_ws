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

#ifndef MOTION_GATE_PROCESS_RUNTIME_HPP_
#define MOTION_GATE_PROCESS_RUNTIME_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "attached_gate_event_journal.hpp"  // NOLINT(build/include_subdir)
#include "voice_nav_mission/motion_gate_core.hpp"

namespace voice_nav_mission
{

struct GateEventJournalTestParameters
{
  std::string name;
  std::string descriptor;
};

struct FinalOutputTime
{
  bool simulation_time_active{false};
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0U};
};

struct FinalOutputFrame
{
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0U};
  Command command{};
};

struct FinalOutputPublisher
{
  using Function = void(void *, const FinalOutputFrame &);

  Function * publish{nullptr};
  void * context{nullptr};
};

enum class FinalOutputFailure : std::uint8_t
{
  None = 0,
  RuntimeInvariant = 1,
  SequenceExhausted = 2,
  JournalFailure = 3,
  DdsFailure = 4,
  DirectZeroDdsFailure = 5,
};

struct FinalOutputState
{
  std::uint64_t output_publish_seq{0U};
  std::uint64_t zero_publish_seq{0U};
  bool last_publication_was_zero{true};
};

struct FinalOutputResult
{
  FinalOutputState state{};
  FinalOutputFailure failure{FinalOutputFailure::None};
  bool published{false};
  bool zero_published{false};
  bool journal_committed{false};
  bool fallback_attempted{false};
  std::uint64_t locally_consumed_terminal_cause_seq{0U};
};

[[nodiscard]] std::optional<GateEventJournalAttachmentConfig>
parse_gate_event_journal_test_parameters(
  const GateEventJournalTestParameters & parameters);

class MotionGateProcessRuntime
{
public:
  MotionGateProcessRuntime(
    MotionGateConfig config,
    std::string gate_instance_id,
    GateEventJournalTestParameters journal_parameters);

  ~MotionGateProcessRuntime();

  MotionGateProcessRuntime(const MotionGateProcessRuntime &) = delete;
  MotionGateProcessRuntime & operator=(
    const MotionGateProcessRuntime &) = delete;
  MotionGateProcessRuntime(MotionGateProcessRuntime &&) = delete;
  MotionGateProcessRuntime & operator=(
    MotionGateProcessRuntime &&) = delete;

  [[nodiscard]] MotionGateCore & core() noexcept;

  [[nodiscard]] FinalOutputResult publish_final_command(
    FinalOutputTime time,
    FinalOutputPublisher publisher);

  [[nodiscard]] FinalOutputState output_state() const;

private:
  // Node confines Core mutations and output calls to one MutuallyExclusive
  // callback group. The Publisher Adapter must not re-enter this Runtime.
  enum class OutputJournalMode : std::uint8_t
  {
    Disabled = 0,
    Usable = 1,
    Retired = 2,
  };

  [[nodiscard]] FinalOutputState output_state_unlocked() const noexcept;
  [[nodiscard]] bool try_force_fault(
    Reason reason,
    const char * detail) noexcept;
  [[nodiscard]] std::uint64_t pending_terminal_cause(
    const Command & command) const noexcept;
  void record_success(
    const Command & command,
    std::uint64_t terminal_cause) noexcept;
  [[nodiscard]] GateOutputIntent make_output_intent(
    const FinalOutputFrame & frame,
    std::uint64_t terminal_cause) const noexcept;
  [[nodiscard]] FinalOutputResult publish_direct_zero(
    FinalOutputTime time,
    FinalOutputPublisher publisher,
    FinalOutputFailure failure,
    bool fallback_attempted);

  std::unique_ptr<AttachedGateEventJournal> attached_journal_;
  std::unique_ptr<MotionGateCore> core_;
  mutable std::mutex output_mutex_;
  OutputJournalMode output_journal_mode_{OutputJournalMode::Disabled};
  FinalOutputFailure retired_failure_{FinalOutputFailure::None};
  std::uint64_t output_attempt_seq_{0U};
  std::uint64_t output_publish_seq_{0U};
  std::uint64_t zero_publish_seq_{0U};
  std::uint64_t last_consumed_terminal_cause_seq_{0U};
  bool last_publication_was_zero_{true};
};

}  // namespace voice_nav_mission

#endif  // MOTION_GATE_PROCESS_RUNTIME_HPP_
