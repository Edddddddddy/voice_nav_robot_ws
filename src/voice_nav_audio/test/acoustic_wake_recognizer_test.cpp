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

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "acoustic_wake_recognizer.hpp"
#include "gtest/gtest.h"

namespace voice_nav_audio
{
namespace
{

CleanedAudioFrame frame(const std::uint64_t sequence)
{
  CleanedAudioFrame result{};
  result.audio_generation = 1U;
  result.audio_seq = sequence;
  return result;
}

class ScriptedKeywordSpotter final : public KeywordSpotterAdapter
{
public:
  bool detected(const CleanedAudioFrame & input) noexcept override
  {
    seen.push_back(input.audio_seq);
    return input.audio_seq == 2U;
  }

  void reset() noexcept override
  {
    ++reset_count;
  }

  std::vector<std::uint64_t> seen{};
  std::size_t reset_count{0U};
};

class RecordingCommandRecognizer final : public SpeechRecognizerAdapter
{
public:
  void process_frame(
    const CleanedAudioFrame & input, SpeechEventSink & sink) noexcept override
  {
    seen.push_back(input.audio_seq);
    sink_ = &sink;
  }

  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept override
  {
    scope_ = scope;
  }

  void on_turn_scope_retired(const TurnScopeIdentity &) noexcept override {}

  void emit_stop(const CleanedAudioFrame & input)
  {
    ASSERT_NE(sink_, nullptr);
    sink_->on_speech_event(SpeechRecognitionEvent::endpoint_final(
        input, scope_, "小智停止", 1.0F, VoiceTurnKind::kStop));
  }

  std::vector<std::uint64_t> seen{};

private:
  SpeechEventSink * sink_{nullptr};
  TurnScopeIdentity scope_{};
};

class RecordingSink final : public SpeechEventSink
{
public:
  void on_speech_event(const SpeechRecognitionEvent & event) noexcept override
  {
    events.push_back(event);
  }

  std::vector<SpeechRecognitionEvent> events{};
};

TEST(AcousticWakeRecognizerTest, GatesAsrUntilAcousticWakeAndSleepsAfterStop)
{
  auto keywords = std::make_unique<ScriptedKeywordSpotter>();
  auto * const keywords_probe = keywords.get();
  auto commands = std::make_unique<RecordingCommandRecognizer>();
  auto * const commands_probe = commands.get();
  AcousticWakeRecognizer recognizer(std::move(keywords), std::move(commands));
  RecordingSink sink;

  recognizer.process_frame(frame(1U), sink);
  recognizer.process_frame(frame(2U), sink);

  EXPECT_EQ(keywords_probe->seen, (std::vector<std::uint64_t>{1U, 2U}));
  EXPECT_TRUE(commands_probe->seen.empty());
  ASSERT_EQ(sink.events.size(), 1U);
  EXPECT_EQ(sink.events.front().kind, SpeechEventKind::kWakeAccepted);

  TurnScopeIdentity scope{};
  scope.id = 1U;
  scope.audio_generation = 1U;
  scope.session_id = "session";
  scope.turn_id = "turn";
  recognizer.on_turn_scope_opened(scope);
  recognizer.process_frame(frame(3U), sink);
  EXPECT_EQ(commands_probe->seen, (std::vector<std::uint64_t>{3U}));

  commands_probe->emit_stop(frame(3U));
  recognizer.process_frame(frame(4U), sink);
  EXPECT_EQ(commands_probe->seen, (std::vector<std::uint64_t>{3U}));
  EXPECT_EQ(keywords_probe->seen, (std::vector<std::uint64_t>{1U, 2U, 4U}));
  EXPECT_EQ(keywords_probe->reset_count, 1U);
}

}  // namespace
}  // namespace voice_nav_audio
