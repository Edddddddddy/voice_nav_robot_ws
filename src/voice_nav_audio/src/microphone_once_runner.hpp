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

#ifndef VOICE_NAV_AUDIO__MICROPHONE_ONCE_RUNNER_HPP_
#define VOICE_NAV_AUDIO__MICROPHONE_ONCE_RUNNER_HPP_

#include <chrono>
#include <cstddef>
#include <memory>

#include "dsp_pipeline.hpp"
#include "voice_nav_audio/portaudio_adapter.hpp"
#include "speech_input_core.hpp"
#include "speech_output_core.hpp"
#include "speech_output_node.hpp"
#include "sensevoice_provider.hpp"
#include "voice_pipeline.hpp"

namespace voice_nav_audio
{

// Microphone-once deliberately keeps the existing DspPipeline and its 48 kHz
// to 16 kHz conversion. This adapter is the explicit no-AEC composition seam
// for the product profile; tests inject a scripted DspAdapter instead.
class MicrophoneOnceDspAdapter final : public DspAdapter
{
public:
  bool process_render(const DspFrame &) noexcept override {return true;}
  bool set_stream_delay_ms(int) noexcept override {return true;}
  bool process_capture(DspFrame &) noexcept override {return true;}
  void reset() noexcept override {}
};

struct MicrophoneOnceSpec
{
  std::size_t maximum_capture_frames{1500U};
  float delay_ms{100.0F};
};

enum class MicrophoneOnceResult
{
  kCapturing,
  kReadyForPlayback,
  kEmpty,
  kTimedOut,
  kFailed,
};

class MicrophoneOnceRunner final
{
public:
  MicrophoneOnceRunner(
    std::unique_ptr<SenseVoiceProvider> recognizer,
    std::unique_ptr<TtsAdapter> tts,
    DspAdapter & dsp_adapter,
    FullDuplexAudioDevice * device,
    SpeechOutputTraceSink * trace,
    MicrophoneOnceSpec spec);
  ~MicrophoneOnceRunner();

  MicrophoneOnceRunner(const MicrophoneOnceRunner &) = delete;
  MicrophoneOnceRunner & operator=(const MicrophoneOnceRunner &) = delete;

  void add_to_executor(rclcpp::Executor & executor);
  void remove_from_executor(rclcpp::Executor & executor);
  [[nodiscard]] MicrophoneOnceResult pump() noexcept;
  [[nodiscard]] MicrophoneOnceResult capture_until(std::chrono::milliseconds timeout) noexcept;
  [[nodiscard]] MicrophoneOnceResult expire() noexcept;
  [[nodiscard]] bool allow_playback() noexcept;

private:
  class Implementation;
  std::unique_ptr<Implementation> implementation_{};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__MICROPHONE_ONCE_RUNNER_HPP_
