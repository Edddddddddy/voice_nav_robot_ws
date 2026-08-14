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

#include <iostream>

#include "speech_input_core.hpp"

namespace voice_nav_audio
{
namespace
{

class ProbeSink final : public VoiceTurnSink
{
public:
  void publish(const VoiceTurnPublication & turn) noexcept override
  {
    turn_ = turn;
    published_ = true;
  }

  [[nodiscard]] bool published() const noexcept
  {
    return published_;
  }

  [[nodiscard]] const VoiceTurnPublication & turn() const noexcept
  {
    return turn_;
  }

private:
  bool published_{false};
  VoiceTurnPublication turn_{};
};

class ProbeRecognizer final : public SpeechRecognizerAdapter
{
public:
  void process_frame(
    const CleanedAudioFrame & frame,
    SpeechEventSink & sink) noexcept override
  {
    if (frame.audio_seq == 1U) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    } else if (frame.audio_seq == 2U) {
      sink.on_speech_event(
        SpeechRecognitionEvent::endpoint_final(frame, scope_, "identity probe", 0.5F));
    }
  }

  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept override
  {
    scope_ = scope;
  }

  void on_turn_scope_retired(const TurnScopeIdentity &) noexcept override
  {
    scope_ = TurnScopeIdentity{};
  }

private:
  TurnScopeIdentity scope_{};
};

CleanedAudioFrame frame(const std::uint64_t sequence)
{
  CleanedAudioFrame input{};
  input.audio_generation = 7U;
  input.audio_seq = sequence;
  return input;
}

}  // namespace
}  // namespace voice_nav_audio

int main()
{
  voice_nav_audio::ProbeRecognizer recognizer;
  voice_nav_audio::ProbeSink sink;
  voice_nav_audio::SpeechInputCore core(recognizer, sink);
  core.accept_cleaned_frame(voice_nav_audio::frame(1U));
  core.accept_cleaned_frame(voice_nav_audio::frame(2U));
  if (!sink.published()) {
    return 1;
  }
  const auto & turn = sink.turn();
  std::cout << turn.voice_instance_id << '\n' << turn.session_id << '\n' << turn.turn_id << '\n';
  return 0;
}
