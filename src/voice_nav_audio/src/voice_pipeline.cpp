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
  SpeechOutputTraceSink * const trace,
  StopMissionPort * const stop_port,
  const VoicePipelineCaptureMode capture_mode)
: VoicePipeline(
    std::move(recognizer), std::move(tts), &device, trace, stop_port, capture_mode)
{
}

VoicePipeline::VoicePipeline(
  std::unique_ptr<SpeechRecognizerAdapter> recognizer,
  std::unique_ptr<TtsAdapter> tts,
  FullDuplexAudioDevice * const device,
  SpeechOutputTraceSink * const trace,
  StopMissionPort * const stop_port,
  const VoicePipelineCaptureMode capture_mode)
: adapter_(engine_, device),
  capture_mode_(capture_mode),
  output_(std::make_shared<SpeechOutputNode>(engine_, std::move(tts), trace)),
  owned_stop_port_(stop_port == nullptr ? std::make_unique<RosStopMissionPort>(
      std::static_pointer_cast<rclcpp::Node>(output_)) : nullptr),
  coordination_(std::make_unique<VoicePipelineCoordination>(
      *output_,
    stop_port != nullptr ? *stop_port : static_cast<StopMissionPort &>(*owned_stop_port_),
      static_cast<VoiceTurnBoundary *>(this))),
  input_(std::make_shared<SpeechInputNode>(std::move(recognizer), coordination_.get()))
{
  if (adapter_.start() != AdapterStartResult::Started) {
    throw std::runtime_error("VoicePipeline could not start its AudioEngine adapter");
  }
}

VoicePipeline::VoicePipeline(
  std::unique_ptr<SpeechRecognizerAdapter> recognizer,
  std::unique_ptr<TtsAdapter> tts,
  SpeechOutputTraceSink * const trace,
  StopMissionPort * const stop_port,
  const VoicePipelineCaptureMode capture_mode)
: VoicePipeline(
    std::move(recognizer), std::move(tts), static_cast<FullDuplexAudioDevice *>(nullptr),
    trace, stop_port, capture_mode)
{
}

VoicePipeline::~VoicePipeline()
{
  adapter_.stop();
}

void VoicePipeline::accept_cleaned_frame(const CleanedAudioFrame & frame) noexcept
{
  input_->accept_cleaned_frame(frame);
}

void VoicePipeline::finish_input() noexcept
{
  input_->finish_input();
}

bool VoicePipeline::try_pop_capture(AudioFrame & frame) noexcept
{
  return engine_.try_pop_capture(frame);
}

bool VoicePipeline::try_pop_reference(AudioFrame & frame) noexcept
{
  return engine_.try_pop_reference(frame);
}

bool VoicePipeline::stop_capture() noexcept
{
  if (capture_mode_ == VoicePipelineCaptureMode::kStopBeforeTurnPublication) {
    return adapter_.pause_for_playback();
  }
  adapter_.stop();
  return true;
}

void VoicePipeline::abort_capture() noexcept
{
  input_->shutdown_input();
  (void)stop_capture();
}

bool VoicePipeline::allow_playback() noexcept
{
  AdapterStartResult result = AdapterStartResult::NoDevice;
  if (capture_mode_ == VoicePipelineCaptureMode::kStopBeforeTurnPublication) {
    result = adapter_.resume_playback();
  } else {
    result = adapter_.start();
  }
  return result == AdapterStartResult::Started;
}

bool VoicePipeline::capture_finished() const noexcept
{
  return capture_finished_.load(std::memory_order_acquire);
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

std::size_t VoicePipeline::direct_stop_request_count() const noexcept
{
  return owned_stop_port_ == nullptr ? 0U : owned_stop_port_->request_count();
}

AudioMetrics VoicePipeline::audio_metrics() const noexcept
{
  return engine_.metrics();
}

void VoicePipeline::on_voice_turn_published() noexcept
{
  if (capture_mode_ != VoicePipelineCaptureMode::kStopBeforeTurnPublication) {
    return;
  }
  // Publication is the linearization point for the one-shot boundary.  Pause
  // the shared full-duplex adapter before the turn can reach Agent, then let
  // the runner observe the completed boundary on its next pump.
  (void)adapter_.pause_for_playback();
  capture_finished_.store(true, std::memory_order_release);
}

}  // namespace voice_nav_audio
