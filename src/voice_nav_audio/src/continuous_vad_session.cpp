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

#include "continuous_vad_session.hpp"

#include <algorithm>
#include <utility>

namespace voice_nav_audio
{

ContinuousVadSession::ContinuousVadSession(
  std::unique_ptr<SpeechRecognizerAdapter> recognizer,
  DspAdapter & dsp_adapter,
  std::unique_ptr<TtsAdapter> tts,
  FullDuplexAudioDevice * const device,
  SpeechOutputTraceSink * const trace,
  StopMissionPort * const stop_port)
: dsp_(std::make_unique<DspPipeline>(dsp_adapter)),
  pipeline_(std::make_unique<VoicePipeline>(
      std::move(recognizer), std::move(tts), device, trace, stop_port, true))
{
}

ContinuousVadSession::~ContinuousVadSession() = default;

ContinuousVadPumpResult ContinuousVadSession::pump() noexcept
{
  if (pipeline_ == nullptr) {
    return ContinuousVadPumpResult::kFailed;
  }

  AudioFrame capture{};
  if (!pipeline_->try_pop_capture(capture)) {
    return ContinuousVadPumpResult::kCapturing;
  }
  AudioFrame reference{};
  if (!pipeline_->try_pop_reference(reference)) {
    stop();
    return ContinuousVadPumpResult::kFailed;
  }

  DspInput input{};
  input.generation = capture.generation;
  input.sequence = next_audio_seq_++;
  input.delay_ms = 100.0;
  std::copy(capture.samples.cbegin(), capture.samples.cend(), input.capture.samples.begin());
  std::copy(
    reference.samples.cbegin(), reference.samples.cend(),
    input.final_render_reference.samples.begin());
  const auto cleaned = dsp_->process(input);
  if (cleaned.status != DspStatus::kCleaned) {
    stop();
    return ContinuousVadPumpResult::kFailed;
  }

  CleanedAudioFrame frame{};
  frame.audio_generation = input.generation;
  frame.audio_seq = input.sequence;
  frame.valid_samples = frame.samples.size();
  frame.samples = cleaned.cleaned;
  if (!input_publisher_active_) {
    ++readiness_warmup_frames_;
    if (readiness_warmup_frames_ < kReadinessWarmupFrames) {
      return ContinuousVadPumpResult::kCapturing;
    }
    if (!pipeline_->activate_input_publisher()) {
      stop();
      return ContinuousVadPumpResult::kFailed;
    }
    input_publisher_active_ = true;
    return ContinuousVadPumpResult::kCapturing;
  }
  pipeline_->accept_cleaned_frame(frame);
  return ContinuousVadPumpResult::kCapturing;
}

void ContinuousVadSession::stop() noexcept
{
  if (pipeline_ != nullptr) {
    pipeline_->abort_capture();
  }
}

void ContinuousVadSession::add_to_executor(rclcpp::Executor & executor)
{
  pipeline_->add_to_executor(executor);
}

void ContinuousVadSession::remove_from_executor(rclcpp::Executor & executor)
{
  pipeline_->remove_from_executor(executor);
}

}  // namespace voice_nav_audio
