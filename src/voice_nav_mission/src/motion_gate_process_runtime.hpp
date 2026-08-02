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

#include <optional>
#include <string>

#include "attached_gate_event_journal.hpp"  // NOLINT(build/include_subdir)

namespace voice_nav_mission
{

struct GateEventJournalTestParameters
{
  std::string name;
  std::string descriptor;
};

[[nodiscard]] std::optional<GateEventJournalAttachmentConfig>
parse_gate_event_journal_test_parameters(
  const GateEventJournalTestParameters & parameters);

}  // namespace voice_nav_mission

#endif  // MOTION_GATE_PROCESS_RUNTIME_HPP_
