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

#include "speech_input_node.hpp"

#include <stdexcept>
#include <utility>

namespace voice_nav_audio
{

rclcpp::QoS voice_turn_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

SpeechInputNode::SpeechInputNode(
  std::unique_ptr<SpeechRecognizerAdapter> recognizer,
  SpeechInputCoordination * const coordination)
: Node("voice_speech_input"),
  recognizer_(std::move(recognizer))
{
  if (!recognizer_) {
    throw std::invalid_argument("SpeechInputNode requires a SpeechRecognizerAdapter");
  }
  turn_publisher_ = create_publisher<voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", voice_turn_qos());
  core_ = std::make_unique<SpeechInputCore>(
    *recognizer_, static_cast<VoiceTurnSink &>(*this),
    default_voice_identity_generator(), coordination);
}

SpeechInputNode::~SpeechInputNode()
{
  // Stop and join the recognizer while Core, its sink, and the ROS publisher
  // are still alive. Provider destruction suppresses late worker events once
  // stopping begins, so teardown cannot publish through a dead Core.
  recognizer_->shutdown();
  recognizer_.reset();
  core_.reset();
  turn_publisher_.reset();
}

void SpeechInputNode::accept_cleaned_frame(const CleanedAudioFrame & frame) noexcept
{
  core_->accept_cleaned_frame(frame);
}

void SpeechInputNode::finish_input() noexcept
{
  core_->finish_input();
}

void SpeechInputNode::shutdown_input() noexcept
{
  recognizer_->shutdown();
}

void SpeechInputNode::publish(const VoiceTurnPublication & turn) noexcept
{
  voice_nav_interfaces::msg::VoiceTurn message{};
  message.voice_instance_id = turn.voice_instance_id;
  message.voice_seq = turn.voice_seq;
  message.session_id = turn.session_id;
  message.turn_id = turn.turn_id;
  message.kind = static_cast<std::uint8_t>(turn.kind);
  message.text = turn.text;
  message.confidence = turn.confidence;
  message.during_playback = turn.during_playback;
  turn_publisher_->publish(message);
}

}  // namespace voice_nav_audio
