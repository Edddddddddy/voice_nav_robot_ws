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

#ifndef VOICE_NAV_AUDIO__ACOUSTIC_WAKE_RECOGNIZER_HPP_
#define VOICE_NAV_AUDIO__ACOUSTIC_WAKE_RECOGNIZER_HPP_

#include <memory>
#include <mutex>

#include "speech_input_core.hpp"

namespace voice_nav_audio
{

// Package-private seam implemented by the streaming sherpa-onnx KWS and a
// deterministic test adapter. It sees cleaned PCM only while the command
// recognizer is asleep.
class KeywordSpotterAdapter
{
public:
  virtual ~KeywordSpotterAdapter() = default;

  [[nodiscard]] virtual bool detected(const CleanedAudioFrame & frame) noexcept = 0;
  virtual void reset() noexcept = 0;
};

// Deep speech-recognizer module: an acoustic wake decision is the only way to
// admit the existing VAD/ASR recognizer. STOP returns it to the sleeping state.
class AcousticWakeRecognizer final : public SpeechRecognizerAdapter, private SpeechEventSink
{
public:
  AcousticWakeRecognizer(
    std::unique_ptr<KeywordSpotterAdapter> keyword_spotter,
    std::unique_ptr<SpeechRecognizerAdapter> command_recognizer);
  ~AcousticWakeRecognizer() override;

  AcousticWakeRecognizer(const AcousticWakeRecognizer &) = delete;
  AcousticWakeRecognizer & operator=(const AcousticWakeRecognizer &) = delete;

  void shutdown() noexcept override;
  void finish_input() noexcept override;
  void process_frame(
    const CleanedAudioFrame & frame, SpeechEventSink & sink) noexcept override;
  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept override;
  void on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept override;

private:
  void on_speech_event(const SpeechRecognitionEvent & event) noexcept override;

  std::unique_ptr<KeywordSpotterAdapter> keyword_spotter_;
  std::unique_ptr<SpeechRecognizerAdapter> command_recognizer_;
  SpeechEventSink * sink_{nullptr};
  bool awake_{false};
  bool stopping_{false};
  std::recursive_mutex mutex_{};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__ACOUSTIC_WAKE_RECOGNIZER_HPP_
