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

#include "rclcpp/executor.hpp"
#include "speech_input_node.hpp"
#include "speech_output_node.hpp"
#include "ros_stop_mission_port.hpp"
#include "voice_pipeline_coordination.hpp"
#include "voice_nav_audio/portaudio_adapter.hpp"

namespace voice_nav_audio
{

// Package-private composition root. It is the one owner of AudioEngine and
// places both stable ROS seams behind injected input, output, and device adapters.
class VoicePipeline final
{
public:
  using Speak = SpeechOutputNode::Speak;

  VoicePipeline(
    std::unique_ptr<SpeechRecognizerAdapter> recognizer,
    std::unique_ptr<TtsAdapter> tts,
    FullDuplexAudioDevice & device,
    SpeechOutputTraceSink * trace = nullptr,
    StopMissionPort * stop_port = nullptr);
  ~VoicePipeline();

  VoicePipeline(const VoicePipeline &) = delete;
  VoicePipeline & operator=(const VoicePipeline &) = delete;

  void accept_cleaned_frame(const CleanedAudioFrame & frame) noexcept;
  void add_to_executor(rclcpp::Executor & executor);
  void remove_from_executor(rclcpp::Executor & executor);
  [[nodiscard]] std::size_t direct_stop_request_count() const noexcept;
  [[nodiscard]] AudioMetrics audio_metrics() const noexcept;

private:
  AudioEngine engine_{};
  PortAudioAdapter adapter_;
  std::shared_ptr<SpeechOutputNode> output_;
  std::unique_ptr<RosStopMissionPort> owned_stop_port_;
  std::unique_ptr<VoicePipelineCoordination> coordination_;
  std::shared_ptr<SpeechInputNode> input_;
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__VOICE_PIPELINE_HPP_
