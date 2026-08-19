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

#ifndef VOICE_NAV_AUDIO__SENSEVOICE_PROVIDER_HPP_
#define VOICE_NAV_AUDIO__SENSEVOICE_PROVIDER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "speech_input_core.hpp"

namespace voice_nav_audio
{

enum class SileroVadDecision
{
  kSilence,
  kSpeech,
  kEndpoint,
};

struct SileroVadResult
{
  SileroVadDecision decision{SileroVadDecision::kSilence};
  std::size_t endpoint_sample_exclusive{0U};
};

enum class SileroVadFlushStatus
{
  kEmpty,
  kUnique,
  kMultiple,
  kInvalid,
};

struct SileroVadFlushResult
{
  SileroVadFlushStatus status{SileroVadFlushStatus::kEmpty};
  std::size_t endpoint_sample_exclusive{0U};
};

// Package-private seam for the resolved Silero VAD implementation and its
// deterministic fake. It receives fixed 10 ms cleaned frames only.
class SileroVadAdapter
{
public:
  virtual ~SileroVadAdapter() = default;

  [[nodiscard]] virtual SileroVadResult process(
    const CleanedAudioFrame & frame) noexcept = 0;
  [[nodiscard]] virtual SileroVadFlushResult finish_input() noexcept
  {
    return SileroVadFlushResult{};
  }
  virtual void reset() noexcept = 0;
};

// Package-private seam for the SenseVoiceSmall int8 implementation and its
// deterministic fake. Inference is called only by the bounded worker.
class SenseVoiceAsrAdapter
{
public:
  virtual ~SenseVoiceAsrAdapter() = default;

  virtual void shutdown() noexcept {}

  virtual bool infer(
    const Sample * samples, std::size_t sample_count, std::string & labeled_text) noexcept = 0;
};

struct SenseVoiceProviderConfig
{
  static constexpr std::size_t kFramesPerSecond = 100U;
  static constexpr std::size_t kMaximumUtteranceSeconds = 15U;
  static constexpr std::size_t kDefaultMaximumUtteranceFrames =
    kFramesPerSecond * kMaximumUtteranceSeconds;

  std::size_t maximum_utterance_frames{kDefaultMaximumUtteranceFrames};
};

// Package-private continuous VoicePipeline recognizer. The VAD and ASR
// adapters are created once for the owning session; each terminal boundary is
// reset on the provider worker before the next frame is admitted.
class SenseVoiceProvider final : public SpeechRecognizerAdapter
{
public:
  SenseVoiceProvider(
    std::unique_ptr<SileroVadAdapter> vad,
    std::unique_ptr<SenseVoiceAsrAdapter> asr,
    SenseVoiceProviderConfig config = {});
  ~SenseVoiceProvider() override;

  SenseVoiceProvider(const SenseVoiceProvider &) = delete;
  SenseVoiceProvider & operator=(const SenseVoiceProvider &) = delete;

  void shutdown() noexcept override;
  void finish_input() noexcept override;

  void process_frame(
    const CleanedAudioFrame & frame,
    SpeechEventSink & sink) noexcept override;
  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept override;
  void on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept override;

private:
  class Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__SENSEVOICE_PROVIDER_HPP_
