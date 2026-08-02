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

#ifndef ATTACHED_GATE_EVENT_JOURNAL_HPP_
#define ATTACHED_GATE_EVENT_JOURNAL_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "gate_event_journal.hpp"  // NOLINT(build/include_subdir)

namespace voice_nav_mission
{

struct GateEventJournalAttachmentConfig
{
  std::string shared_memory_name;
  GateEventJournalIdentity expected_identity;
  std::uint64_t expected_capacity;
  GateEventJournalClock clock;
};

class AttachedGateEventJournal
{
public:
  explicit AttachedGateEventJournal(
    GateEventJournalAttachmentConfig config);

  ~AttachedGateEventJournal();

  AttachedGateEventJournal(const AttachedGateEventJournal &) = delete;
  AttachedGateEventJournal & operator=(
    const AttachedGateEventJournal &) = delete;
  AttachedGateEventJournal(AttachedGateEventJournal &&) = delete;
  AttachedGateEventJournal & operator=(
    AttachedGateEventJournal &&) = delete;

  [[nodiscard]] GateEventJournal & journal() noexcept;

private:
  struct Impl;

  std::unique_ptr<Impl> impl_;
};

}  // namespace voice_nav_mission

#endif  // ATTACHED_GATE_EVENT_JOURNAL_HPP_
