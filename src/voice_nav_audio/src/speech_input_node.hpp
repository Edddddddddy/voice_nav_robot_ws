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

#ifndef VOICE_NAV_AUDIO__SPEECH_INPUT_NODE_HPP_
#define VOICE_NAV_AUDIO__SPEECH_INPUT_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "speech_input_core.hpp"
#include "voice_nav_interfaces/msg/voice_turn.hpp"

namespace voice_nav_audio
{

[[nodiscard]] rclcpp::QoS voice_turn_qos();

// Package-private ROS composition seam.  It has no executable, component
// registration, or input endpoint: tests inject only an in-process adapter.
class SpeechInputNode final : public rclcpp::Node, private VoiceTurnSink
{
public:
  explicit SpeechInputNode(std::unique_ptr<SpeechRecognizerAdapter> recognizer);

  void accept_cleaned_frame(const CleanedAudioFrame & frame) noexcept;

private:
  void publish(const VoiceTurnPublication & turn) noexcept override;

  std::unique_ptr<SpeechRecognizerAdapter> recognizer_{};
  std::unique_ptr<SpeechInputCore> core_{};
  rclcpp::Publisher<voice_nav_interfaces::msg::VoiceTurn>::SharedPtr turn_publisher_{};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__SPEECH_INPUT_NODE_HPP_
