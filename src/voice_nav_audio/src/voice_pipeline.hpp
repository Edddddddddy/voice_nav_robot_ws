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

#ifndef VOICE_NAV_AUDIO__VOICE_PIPELINE_HPP_
#define VOICE_NAV_AUDIO__VOICE_PIPELINE_HPP_

#include <memory>
#include <cstddef>
#include <atomic>

#include "rclcpp/executor.hpp"
#include "speech_input_node.hpp"
#include "speech_output_node.hpp"
#include "ros_stop_mission_port.hpp"
#include "voice_pipeline_coordination.hpp"
#include "voice_nav_audio/portaudio_adapter.hpp"

namespace voice_nav_audio
{

enum class VoicePipelineCaptureMode
{
  kKeepCapture,
  kStopBeforeTurnPublication
};

// Package-private composition root. It is the one owner of AudioEngine and
// places both stable ROS seams behind injected input, output, and device adapters.
class VoicePipeline final : private VoiceTurnBoundary
{
public:
  using Speak = SpeechOutputNode::Speak;

  VoicePipeline(
    std::unique_ptr<SpeechRecognizerAdapter> recognizer,
    std::unique_ptr<TtsAdapter> tts,
    FullDuplexAudioDevice & device,
    SpeechOutputTraceSink * trace = nullptr,
    StopMissionPort * stop_port = nullptr,
    VoicePipelineCaptureMode capture_mode = VoicePipelineCaptureMode::kKeepCapture,
    bool defer_input_publisher = false);
  VoicePipeline(
    std::unique_ptr<SpeechRecognizerAdapter> recognizer,
    std::unique_ptr<TtsAdapter> tts,
    FullDuplexAudioDevice * device,
    SpeechOutputTraceSink * trace = nullptr,
    StopMissionPort * stop_port = nullptr,
    VoicePipelineCaptureMode capture_mode = VoicePipelineCaptureMode::kKeepCapture,
    bool defer_input_publisher = false);
  VoicePipeline(
    std::unique_ptr<SpeechRecognizerAdapter> recognizer,
    std::unique_ptr<TtsAdapter> tts,
    SpeechOutputTraceSink * trace = nullptr,
    StopMissionPort * stop_port = nullptr,
    VoicePipelineCaptureMode capture_mode = VoicePipelineCaptureMode::kKeepCapture,
    bool defer_input_publisher = false);
  ~VoicePipeline();

  VoicePipeline(const VoicePipeline &) = delete;
  VoicePipeline & operator=(const VoicePipeline &) = delete;

  void accept_cleaned_frame(const CleanedAudioFrame & frame) noexcept;
  void finish_input() noexcept;
  [[nodiscard]] bool start_capture() noexcept;
  [[nodiscard]] bool try_pop_capture(AudioFrame & frame) noexcept;
  [[nodiscard]] bool try_pop_reference(AudioFrame & frame) noexcept;
  [[nodiscard]] bool stop_capture() noexcept;
  void abort_capture() noexcept;
  [[nodiscard]] bool allow_playback() noexcept;
  [[nodiscard]] bool capture_finished() const noexcept;
  // Consumes the package-private turn boundary event on the control pump.
  // The provider callback only sets this event and never replaces a child.
  [[nodiscard]] bool consume_turn_completed_event() noexcept;
  [[nodiscard]] bool activate_input_publisher() noexcept;
  void add_to_executor(rclcpp::Executor & executor);
  void remove_from_executor(rclcpp::Executor & executor);
  [[nodiscard]] std::size_t direct_stop_request_count() const noexcept;
  [[nodiscard]] AudioMetrics audio_metrics() const noexcept;

private:
  void on_recognizer_terminal(
    SpeechEventKind kind, bool published) noexcept override;

  AudioEngine engine_{};
  PortAudioAdapter adapter_;
  VoicePipelineCaptureMode capture_mode_{VoicePipelineCaptureMode::kKeepCapture};
  std::shared_ptr<SpeechOutputNode> output_;
  std::unique_ptr<RosStopMissionPort> owned_stop_port_;
  std::unique_ptr<VoicePipelineCoordination> coordination_;
  std::shared_ptr<SpeechInputNode> input_;
  std::atomic<bool> capture_finished_{false};
  std::atomic<bool> turn_completed_event_{false};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__VOICE_PIPELINE_HPP_
