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

#ifndef VOICE_NAV_AUDIO__SPEECH_INPUT_CORE_HPP_
#define VOICE_NAV_AUDIO__SPEECH_INPUT_CORE_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "voice_nav_audio/audio_engine.hpp"

namespace voice_nav_audio
{

struct CleanedAudioFrame
{
  static constexpr std::uint32_t kSampleRateHz = 16000U;
  static constexpr std::uint32_t kChannels = 1U;
  static constexpr std::size_t kSamples = 160U;

  std::uint32_t sample_rate_hz{kSampleRateHz};
  std::uint32_t channels{kChannels};
  std::uint64_t audio_generation{0U};
  std::uint64_t audio_seq{0U};
  std::array<Sample, kSamples> samples{};
};

struct TurnScopeIdentity
{
  std::uint64_t id{0U};
  std::uint64_t audio_generation{0U};
  std::string session_id{};
  std::string turn_id{};
};

// Package-private seam for deterministic tests.  The default implementation
// reads exactly 16 bytes from the operating system CSPRNG.
class VoiceIdentityGenerator
{
public:
  virtual ~VoiceIdentityGenerator() = default;

  virtual bool generate(std::array<std::uint8_t, 16U> & bytes) noexcept = 0;
};

VoiceIdentityGenerator & default_voice_identity_generator() noexcept;

enum class SpeechEventKind
{
  kWakeMiss,
  kWakeAccepted,
  kActivity,
  kEndpointFinal,
  kTimeout,
  kFailure,
};

enum class VoiceTurnKind : std::uint8_t
{
  kCommand = 1U,
  kStop = 2U,
};

struct SpeechRecognitionEvent
{
  SpeechEventKind kind{SpeechEventKind::kWakeMiss};
  std::uint64_t audio_generation{0U};
  std::uint64_t audio_seq{0U};
  TurnScopeIdentity scope{};
  std::string final_text{};
  float confidence{0.0F};
  VoiceTurnKind voice_turn_kind{VoiceTurnKind::kCommand};

  [[nodiscard]] static SpeechRecognitionEvent wake_accepted(
    const CleanedAudioFrame & frame) noexcept;
  [[nodiscard]] static SpeechRecognitionEvent activity(
    const CleanedAudioFrame & frame,
    const TurnScopeIdentity & scope) noexcept;
  [[nodiscard]] static SpeechRecognitionEvent endpoint_final(
    const CleanedAudioFrame & frame,
    const TurnScopeIdentity & scope,
    std::string text,
    float confidence,
    VoiceTurnKind voice_turn_kind = VoiceTurnKind::kCommand) noexcept;
};

class SpeechEventSink
{
public:
  virtual ~SpeechEventSink() = default;

  virtual void on_speech_event(const SpeechRecognitionEvent & event) noexcept = 0;
};

// Package-private adapter seam.  Production KWS/VAD/ASR and the test fake can
// only return closed events through this sink; no partial token or decision is
// represented outside the audio process.
class SpeechRecognizerAdapter
{
public:
  virtual ~SpeechRecognizerAdapter() = default;

  virtual void process_frame(
    const CleanedAudioFrame & frame,
    SpeechEventSink & sink) noexcept = 0;
  virtual void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept = 0;
  virtual void on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept = 0;
};

struct VoiceTurnPublication
{
  std::string voice_instance_id{};
  std::uint64_t voice_seq{0U};
  std::string session_id{};
  std::string turn_id{};
  VoiceTurnKind kind{VoiceTurnKind::kCommand};
  std::string text{};
  float confidence{0.0F};
  bool during_playback{false};
};

class VoiceTurnSink
{
public:
  virtual ~VoiceTurnSink() = default;

  virtual void publish(const VoiceTurnPublication & turn) noexcept = 0;
};

// Package-private coordination seam. It admits only a wake decision and a
// bounded final turn, keeping recognizer details and ROS transport outside the
// VoicePipeline composition root.
class SpeechInputCoordination
{
public:
  virtual ~SpeechInputCoordination() = default;

  [[nodiscard]] virtual bool on_wake_accepted() noexcept = 0;
  virtual void before_turn_published(VoiceTurnPublication & turn) noexcept = 0;
};

// Package-private speech state machine.  It owns one capacity-one TurnScope
// and never exposes recognizer internals beyond a completed VoiceTurn value.
class SpeechInputCore final : private SpeechEventSink
{
public:
  SpeechInputCore(
    SpeechRecognizerAdapter & recognizer,
    VoiceTurnSink & sink,
    VoiceIdentityGenerator & identity_generator = default_voice_identity_generator(),
    SpeechInputCoordination * coordination = nullptr) noexcept;

  void accept_cleaned_frame(const CleanedAudioFrame & frame) noexcept;

private:
  void on_speech_event(const SpeechRecognitionEvent & event) noexcept override;
  [[nodiscard]] bool accepts_event_frame(const SpeechRecognitionEvent & event) const noexcept;
  [[nodiscard]] bool matches_active_scope(const SpeechRecognitionEvent & event) const noexcept;
  [[nodiscard]] bool is_duplicate_privileged_stop(
    const SpeechRecognitionEvent & event) const noexcept;
  void open_turn_scope() noexcept;
  void retire_turn_scope() noexcept;

  SpeechRecognizerAdapter & recognizer_;
  VoiceTurnSink & sink_;
  SpeechInputCoordination * coordination_{nullptr};
  std::string voice_instance_id_{};
  std::string session_id_{};
  std::uint64_t next_scope_id_{1U};
  std::uint64_t next_voice_seq_{1U};
  bool identity_ready_{false};
  bool has_audio_generation_{false};
  bool audio_generation_quarantined_{false};
  std::uint64_t audio_generation_{0U};
  bool has_audio_seq_{false};
  std::uint64_t latest_audio_seq_{0U};
  bool has_active_scope_{false};
  TurnScopeIdentity active_scope_{};
  bool has_accepted_stop_frame_{false};
  std::uint64_t accepted_stop_generation_{0U};
  std::uint64_t accepted_stop_seq_{0U};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__SPEECH_INPUT_CORE_HPP_
