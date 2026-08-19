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

#ifndef VOICE_NAV_AUDIO__CONTINUOUS_VAD_SESSION_HPP_
#define VOICE_NAV_AUDIO__CONTINUOUS_VAD_SESSION_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "dsp_pipeline.hpp"
#include "rclcpp/executor.hpp"
#include "speech_input_core.hpp"
#include "voice_pipeline.hpp"

namespace voice_nav_audio
{

enum class ContinuousVadPumpResult
{
  kCapturing,
  kFailed,
};

// Package-private long-lived VAD owner. Capture and the recognizer stay open
// across turns; the recognizer resets only its turn-local state on terminal
// boundaries.
class ContinuousVadSession final
{
public:
  static constexpr std::size_t kReadinessWarmupFrames{3U};

  ContinuousVadSession(
    std::unique_ptr<SpeechRecognizerAdapter> recognizer,
    DspAdapter & dsp_adapter,
    std::unique_ptr<TtsAdapter> tts,
    FullDuplexAudioDevice * device = nullptr,
    SpeechOutputTraceSink * trace = nullptr,
    StopMissionPort * stop_port = nullptr);
  ~ContinuousVadSession();

  ContinuousVadSession(const ContinuousVadSession &) = delete;
  ContinuousVadSession & operator=(const ContinuousVadSession &) = delete;

  [[nodiscard]] ContinuousVadPumpResult pump() noexcept;
  void stop() noexcept;
  void add_to_executor(rclcpp::Executor & executor);
  void remove_from_executor(rclcpp::Executor & executor);

private:
  std::unique_ptr<DspPipeline> dsp_{};
  std::unique_ptr<VoicePipeline> pipeline_{};
  bool input_publisher_active_{false};
  std::size_t readiness_warmup_frames_{0U};
  std::uint64_t next_audio_seq_{1U};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__CONTINUOUS_VAD_SESSION_HPP_
