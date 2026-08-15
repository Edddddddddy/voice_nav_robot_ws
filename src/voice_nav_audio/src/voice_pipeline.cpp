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

#include "voice_pipeline.hpp"

#include <stdexcept>
#include <utility>

namespace voice_nav_audio
{

VoicePipeline::VoicePipeline(
  std::unique_ptr<SpeechRecognizerAdapter> recognizer,
  std::unique_ptr<TtsAdapter> tts,
  FullDuplexAudioDevice & device,
  SpeechOutputTraceSink * const trace)
: adapter_(engine_, device),
  input_(std::make_shared<SpeechInputNode>(std::move(recognizer))),
  output_(std::make_shared<SpeechOutputNode>(engine_, std::move(tts), trace))
{
  if (adapter_.start() != AdapterStartResult::Started) {
    throw std::runtime_error("VoicePipeline could not start its AudioEngine adapter");
  }
}

VoicePipeline::~VoicePipeline()
{
  adapter_.stop();
}

void VoicePipeline::accept_cleaned_frame(const CleanedAudioFrame & frame) noexcept
{
  input_->accept_cleaned_frame(frame);
}

void VoicePipeline::add_to_executor(rclcpp::Executor & executor)
{
  executor.add_node(input_);
  executor.add_node(output_);
}

void VoicePipeline::remove_from_executor(rclcpp::Executor & executor)
{
  executor.remove_node(input_);
  executor.remove_node(output_);
}

}  // namespace voice_nav_audio
