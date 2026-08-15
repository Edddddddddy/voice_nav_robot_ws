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

#include "speech_input_core.hpp"

#include <cmath>
#include <cstdio>
#include <string_view>

#include <sys/random.h>

namespace voice_nav_audio
{
namespace
{

class GetrandomVoiceIdentityGenerator final : public VoiceIdentityGenerator
{
public:
  bool generate(std::array<std::uint8_t, 16U> & bytes) noexcept override
  {
    return getrandom(bytes.data(), bytes.size(), 0) ==
           static_cast<ssize_t>(bytes.size());
  }
};

std::string hexadecimal_identity(const std::array<std::uint8_t, 16U> & bytes) noexcept
{
  constexpr char digits[] = "0123456789abcdef";
  char buffer[33U]{};
  for (std::size_t index = 0U; index < bytes.size(); ++index) {
    buffer[index * 2U] = digits[bytes[index] >> 4U];
    buffer[index * 2U + 1U] = digits[bytes[index] & 0x0fU];
  }
  return std::string(buffer, 32U);
}

std::string turn_identity(
  const std::string & voice_instance_id,
  const std::uint64_t scope_id) noexcept
{
  char buffer[34U]{};
  std::snprintf(buffer, sizeof(buffer), "t%.16s%016llx", voice_instance_id.c_str(),
    static_cast<unsigned long long>(scope_id));
  return std::string(buffer);
}

bool is_valid_utf8(const std::string_view text) noexcept
{
  std::size_t index = 0U;
  while (index < text.size()) {
    const auto first = static_cast<unsigned char>(text[index]);
    if (first <= 0x7fU) {
      ++index;
      continue;
    }

    std::size_t continuation_count = 0U;
    std::uint32_t code_point = 0U;
    if (first >= 0xc2U && first <= 0xdfU) {
      continuation_count = 1U;
      code_point = first & 0x1fU;
    } else if (first >= 0xe0U && first <= 0xefU) {
      continuation_count = 2U;
      code_point = first & 0x0fU;
    } else if (first >= 0xf0U && first <= 0xf4U) {
      continuation_count = 3U;
      code_point = first & 0x07U;
    } else {
      return false;
    }
    if (index + continuation_count >= text.size()) {
      return false;
    }
    for (std::size_t offset = 1U; offset <= continuation_count; ++offset) {
      const auto continuation = static_cast<unsigned char>(text[index + offset]);
      if ((continuation & 0xc0U) != 0x80U) {
        return false;
      }
      code_point = (code_point << 6U) | (continuation & 0x3fU);
    }
    const auto minimum = continuation_count == 1U ? 0x80U :
      continuation_count == 2U ? 0x800U : 0x10000U;
    if (code_point < minimum || code_point > 0x10ffffU ||
      (code_point >= 0xd800U && code_point <= 0xdfffU))
    {
      return false;
    }
    index += continuation_count + 1U;
  }
  return true;
}

bool valid_final(const SpeechRecognitionEvent & event) noexcept
{
  return !event.final_text.empty() && event.final_text.size() <= 512U &&
         is_valid_utf8(event.final_text) && std::isfinite(event.confidence) &&
         event.confidence >= 0.0F && event.confidence <= 1.0F &&
         (event.voice_turn_kind == VoiceTurnKind::kCommand ||
         event.voice_turn_kind == VoiceTurnKind::kStop);
}

}  // namespace

VoiceIdentityGenerator & default_voice_identity_generator() noexcept
{
  static GetrandomVoiceIdentityGenerator generator;
  return generator;
}

SpeechRecognitionEvent SpeechRecognitionEvent::wake_accepted(
  const CleanedAudioFrame & frame) noexcept
{
  SpeechRecognitionEvent event{};
  event.kind = SpeechEventKind::kWakeAccepted;
  event.audio_generation = frame.audio_generation;
  event.audio_seq = frame.audio_seq;
  return event;
}

SpeechRecognitionEvent SpeechRecognitionEvent::activity(
  const CleanedAudioFrame & frame,
  const TurnScopeIdentity & scope) noexcept
{
  SpeechRecognitionEvent event{};
  event.kind = SpeechEventKind::kActivity;
  event.audio_generation = frame.audio_generation;
  event.audio_seq = frame.audio_seq;
  event.scope = scope;
  return event;
}

SpeechRecognitionEvent SpeechRecognitionEvent::endpoint_final(
  const CleanedAudioFrame & frame,
  const TurnScopeIdentity & scope,
  std::string text,
  const float confidence,
  const VoiceTurnKind voice_turn_kind) noexcept
{
  SpeechRecognitionEvent event{};
  event.kind = SpeechEventKind::kEndpointFinal;
  event.audio_generation = frame.audio_generation;
  event.audio_seq = frame.audio_seq;
  event.scope = scope;
  event.final_text = std::move(text);
  event.confidence = confidence;
  event.voice_turn_kind = voice_turn_kind;
  return event;
}

SpeechInputCore::SpeechInputCore(
  SpeechRecognizerAdapter & recognizer,
  VoiceTurnSink & sink,
  VoiceIdentityGenerator & identity_generator) noexcept
: recognizer_(recognizer),
  sink_(sink)
{
  std::array<std::uint8_t, 16U> identity_bytes{};
  identity_ready_ = identity_generator.generate(identity_bytes);
  if (!identity_ready_) {
    return;
  }
  voice_instance_id_ = hexadecimal_identity(identity_bytes);
  session_id_ = voice_instance_id_;
}

void SpeechInputCore::accept_cleaned_frame(const CleanedAudioFrame & frame) noexcept
{
  if (!identity_ready_) {
    return;
  }
  if (has_audio_generation_ && frame.audio_generation < audio_generation_) {
    return;
  }
  if (!has_audio_generation_ || frame.audio_generation > audio_generation_) {
    retire_turn_scope();
    has_audio_generation_ = true;
    audio_generation_ = frame.audio_generation;
    audio_generation_quarantined_ = false;
  }
  if (audio_generation_quarantined_) {
    return;
  }
  if (has_audio_seq_ && latest_audio_seq_ == UINT64_MAX) {
    retire_turn_scope();
    audio_generation_quarantined_ = true;
    return;
  }
  if ((!has_audio_seq_ && frame.audio_seq == 0U) ||
    (has_audio_seq_ && frame.audio_seq != latest_audio_seq_ + 1U))
  {
    retire_turn_scope();
    if (!has_audio_seq_ || frame.audio_seq > latest_audio_seq_) {
      has_audio_seq_ = frame.audio_seq != 0U;
      latest_audio_seq_ = frame.audio_seq;
    }
    audio_generation_quarantined_ = true;
    return;
  }
  has_audio_seq_ = true;
  latest_audio_seq_ = frame.audio_seq;
  if (frame.sample_rate_hz != CleanedAudioFrame::kSampleRateHz ||
    frame.channels != CleanedAudioFrame::kChannels)
  {
    retire_turn_scope();
    audio_generation_quarantined_ = true;
    return;
  }
  recognizer_.process_frame(frame, *this);
}

void SpeechInputCore::on_speech_event(const SpeechRecognitionEvent & event) noexcept
{
  if (!accepts_event_frame(event)) {
    return;
  }
  switch (event.kind) {
    case SpeechEventKind::kWakeMiss:
      return;
    case SpeechEventKind::kWakeAccepted:
      if (event.scope.id == 0U && event.audio_seq == latest_audio_seq_) {
        open_turn_scope();
      }
      return;
    case SpeechEventKind::kActivity:
      return;
    case SpeechEventKind::kEndpointFinal:
      if (!matches_active_scope(event)) {
        return;
      }
      if (valid_final(event)) {
        VoiceTurnPublication publication{};
        publication.voice_instance_id = voice_instance_id_;
        publication.voice_seq = next_voice_seq_++;
        publication.session_id = active_scope_.session_id;
        publication.turn_id = active_scope_.turn_id;
        publication.kind = event.voice_turn_kind;
        publication.text = event.final_text;
        publication.confidence = event.confidence;
        sink_.publish(publication);
      }
      retire_turn_scope();
      return;
    case SpeechEventKind::kTimeout:
    case SpeechEventKind::kFailure:
      if (matches_active_scope(event)) {
        retire_turn_scope();
      }
      return;
  }
}

bool SpeechInputCore::accepts_event_frame(const SpeechRecognitionEvent & event) const noexcept
{
  return has_audio_generation_ && !audio_generation_quarantined_ &&
         event.audio_generation == audio_generation_ && event.audio_seq != 0U &&
         event.audio_seq <= latest_audio_seq_;
}

bool SpeechInputCore::matches_active_scope(const SpeechRecognitionEvent & event) const noexcept
{
  return has_active_scope_ && event.scope.id != 0U && event.scope.id == active_scope_.id &&
         event.scope.audio_generation == active_scope_.audio_generation &&
         event.scope.session_id == active_scope_.session_id &&
         event.scope.turn_id == active_scope_.turn_id;
}

void SpeechInputCore::open_turn_scope() noexcept
{
  retire_turn_scope();
  active_scope_.id = next_scope_id_++;
  active_scope_.audio_generation = audio_generation_;
  active_scope_.session_id = session_id_;
  active_scope_.turn_id = turn_identity(voice_instance_id_, active_scope_.id);
  has_active_scope_ = true;
  recognizer_.on_turn_scope_opened(active_scope_);
}

void SpeechInputCore::retire_turn_scope() noexcept
{
  if (!has_active_scope_) {
    return;
  }
  recognizer_.on_turn_scope_retired(active_scope_);
  has_active_scope_ = false;
  active_scope_ = TurnScopeIdentity{};
}

}  // namespace voice_nav_audio
