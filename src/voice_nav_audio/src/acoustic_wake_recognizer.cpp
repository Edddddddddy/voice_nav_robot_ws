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

#include "acoustic_wake_recognizer.hpp"

#include <stdexcept>
#include <utility>

namespace voice_nav_audio
{

AcousticWakeRecognizer::AcousticWakeRecognizer(
  std::unique_ptr<KeywordSpotterAdapter> keyword_spotter,
  std::unique_ptr<SpeechRecognizerAdapter> command_recognizer)
: keyword_spotter_(std::move(keyword_spotter)),
  command_recognizer_(std::move(command_recognizer))
{
  if (!keyword_spotter_ || !command_recognizer_) {
    throw std::invalid_argument("acoustic wake recognizer requires KWS and command recognizers");
  }
}

AcousticWakeRecognizer::~AcousticWakeRecognizer()
{
  shutdown();
}

void AcousticWakeRecognizer::shutdown() noexcept
{
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (stopping_) {
      return;
    }
    stopping_ = true;
    sink_ = nullptr;
    awake_ = false;
  }
  command_recognizer_->shutdown();
  keyword_spotter_->reset();
}

void AcousticWakeRecognizer::finish_input() noexcept
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!stopping_ && awake_) {
    command_recognizer_->finish_input();
  }
}

void AcousticWakeRecognizer::process_frame(
  const CleanedAudioFrame & frame, SpeechEventSink & sink) noexcept
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (stopping_) {
    return;
  }
  sink_ = &sink;
  if (awake_) {
    command_recognizer_->process_frame(frame, *this);
    return;
  }
  if (keyword_spotter_->detected(frame)) {
    awake_ = true;
    sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
  }
}

void AcousticWakeRecognizer::on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!stopping_ && awake_) {
    command_recognizer_->on_turn_scope_opened(scope);
  }
}

void AcousticWakeRecognizer::on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!stopping_) {
    command_recognizer_->on_turn_scope_retired(scope);
  }
}

void AcousticWakeRecognizer::on_speech_event(const SpeechRecognitionEvent & event) noexcept
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (stopping_ || sink_ == nullptr) {
    return;
  }
  if (event.kind == SpeechEventKind::kEndpointFinal &&
    event.voice_turn_kind == VoiceTurnKind::kStop)
  {
    awake_ = false;
    keyword_spotter_->reset();
  }
  sink_->on_speech_event(event);
}

}  // namespace voice_nav_audio
