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

#include <memory>
#include <string>

#include "gate_event_journal.hpp"  // NOLINT(build/include_subdir)

namespace voice_nav_mission
{

class AttachedGateEventJournal
{
public:
  static AttachedGateEventJournal open_existing(
    const std::string & name,
    const std::string & nonce_hex,
    GateEventJournalClock clock);

  ~AttachedGateEventJournal();

  AttachedGateEventJournal(const AttachedGateEventJournal &) = delete;
  AttachedGateEventJournal & operator=(
    const AttachedGateEventJournal &) = delete;
  AttachedGateEventJournal(AttachedGateEventJournal &&) noexcept;
  AttachedGateEventJournal & operator=(
    AttachedGateEventJournal &&) noexcept;

  [[nodiscard]] GateEventJournal & writer() noexcept;

private:
  struct Impl;

  explicit AttachedGateEventJournal(std::unique_ptr<Impl> impl) noexcept;

  std::unique_ptr<Impl> impl_;
};

}  // namespace voice_nav_mission

#endif  // ATTACHED_GATE_EVENT_JOURNAL_HPP_
