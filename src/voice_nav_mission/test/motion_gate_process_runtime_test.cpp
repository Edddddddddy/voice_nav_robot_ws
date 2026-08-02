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

#include <gtest/gtest.h>

#include "motion_gate_process_runtime.hpp"

namespace voice_nav_mission
{
namespace
{

TEST(MotionGateProcessRuntimeTest, BothEmptyParametersDisableAttachment)
{
  const auto config = parse_gate_event_journal_test_parameters(
    GateEventJournalTestParameters{"", ""});

  EXPECT_FALSE(config.has_value());
}

}  // namespace
}  // namespace voice_nav_mission
