// Copyright 2026 Eddddddddy
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

#include "microphone_once_runner.hpp"
#include "sensevoice_provider.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

namespace voice_nav_audio
{
namespace
{

std::unique_ptr<SpeechRecognizerAdapter> arm_once(
  std::unique_ptr<SenseVoiceProvider> recognizer)
{
  if (!recognizer || !recognizer->arm_once()) {
    throw std::invalid_argument("MicrophoneOnceRunner requires an unarmed SenseVoiceProvider");
  }
  return recognizer;
}

}  // namespace

class MicrophoneOnceRunner::Implementation final
{
public:
  Implementation(
    std::unique_ptr<SenseVoiceProvider> recognizer,
    std::unique_ptr<TtsAdapter> tts,
    DspAdapter & dsp_adapter,
    FullDuplexAudioDevice * const device,
    SpeechOutputTraceSink * const trace,
    MicrophoneOnceSpec spec)
  : dsp_(dsp_adapter),
    spec_(spec),
    pipeline_(std::make_unique<VoicePipeline>(
      arm_once(std::move(recognizer)), std::move(tts), device, trace, nullptr,
      VoicePipelineCaptureMode::kStopBeforeTurnPublication))
  {
    if (spec_.maximum_capture_frames == 0U) {
      spec_.maximum_capture_frames = SenseVoiceProviderConfig::kDefaultMaximumUtteranceFrames;
    }
  }

  MicrophoneOnceResult pump() noexcept
  {
    if (result_ != MicrophoneOnceResult::kCapturing) {
      return result_;
    }
    if (pipeline_->capture_finished()) {
      return close_capture(MicrophoneOnceResult::kReadyForPlayback);
    }
    if (input_finished_) {
      return result_;
    }

    AudioFrame capture{};
    if (!pipeline_->try_pop_capture(capture)) {
      return result_;
    }
    AudioFrame reference{};
    if (!pipeline_->try_pop_reference(reference)) {
      return fail_capture();
    }
    if (capture_frames_ >= spec_.maximum_capture_frames) {
      pipeline_->finish_input();
      input_finished_ = true;
      return result_;
    }

    DspInput input{};
    input.generation = capture.generation;
    input.sequence = next_sequence_++;
    input.delay_ms = spec_.delay_ms;
    std::copy(capture.samples.cbegin(), capture.samples.cend(), input.capture.samples.begin());
    std::copy(
      reference.samples.cbegin(), reference.samples.cend(),
      input.final_render_reference.samples.begin());
    const auto cleaned = dsp_.process(input);
    if (cleaned.status != DspStatus::kCleaned) {
      return fail_capture();
    }

    CleanedAudioFrame frame{};
    frame.audio_generation = input.generation;
    frame.audio_seq = input.sequence;
    frame.valid_samples = frame.samples.size();
    frame.samples = cleaned.cleaned;
    pipeline_->accept_cleaned_frame(frame);
    ++capture_frames_;
    if (pipeline_->capture_finished()) {
      return close_capture(MicrophoneOnceResult::kReadyForPlayback);
    }
    if (capture_frames_ >= spec_.maximum_capture_frames) {
      pipeline_->finish_input();
      input_finished_ = true;
    }
    return result_;
  }

  MicrophoneOnceResult capture_until(const std::chrono::milliseconds timeout) noexcept
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (result_ == MicrophoneOnceResult::kCapturing &&
      std::chrono::steady_clock::now() < deadline)
    {
      (void)pump();
      if (result_ == MicrophoneOnceResult::kCapturing) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return result_ == MicrophoneOnceResult::kCapturing ? expire() : result_;
  }

  MicrophoneOnceResult expire() noexcept
  {
    if (result_ != MicrophoneOnceResult::kCapturing) {
      return result_;
    }
    pipeline_->abort_capture();
    result_ = capture_frames_ == 0U ?
      MicrophoneOnceResult::kEmpty : MicrophoneOnceResult::kTimedOut;
    capture_stopped_ = true;
    return result_;
  }

  bool allow_playback() noexcept
  {
    return result_ == MicrophoneOnceResult::kReadyForPlayback &&
           pipeline_->allow_playback();
  }

  void add_to_executor(rclcpp::Executor & executor)
  {
    pipeline_->add_to_executor(executor);
  }

  void remove_from_executor(rclcpp::Executor & executor)
  {
    pipeline_->remove_from_executor(executor);
  }

private:
  MicrophoneOnceResult close_capture(const MicrophoneOnceResult result) noexcept
  {
    if (!capture_stopped_) {
      if (!pipeline_->stop_capture()) {
        return fail_capture();
      }
      capture_stopped_ = true;
    }
    result_ = result;
    return result_;
  }

  MicrophoneOnceResult fail_capture() noexcept
  {
    pipeline_->abort_capture();
    capture_stopped_ = true;
    result_ = MicrophoneOnceResult::kFailed;
    return result_;
  }

  DspPipeline dsp_;
  MicrophoneOnceSpec spec_{};
  std::unique_ptr<VoicePipeline> pipeline_{};
  MicrophoneOnceResult result_{MicrophoneOnceResult::kCapturing};
  std::size_t capture_frames_{0U};
  std::uint64_t next_sequence_{1U};
  bool input_finished_{false};
  bool capture_stopped_{false};
};

MicrophoneOnceRunner::MicrophoneOnceRunner(
  std::unique_ptr<SenseVoiceProvider> recognizer,
  std::unique_ptr<TtsAdapter> tts,
  DspAdapter & dsp_adapter,
  FullDuplexAudioDevice * const device,
  SpeechOutputTraceSink * const trace,
  const MicrophoneOnceSpec spec)
: implementation_(std::make_unique<Implementation>(
    std::move(recognizer), std::move(tts), dsp_adapter, device, trace, spec))
{
}

MicrophoneOnceRunner::~MicrophoneOnceRunner() = default;

void MicrophoneOnceRunner::add_to_executor(rclcpp::Executor & executor)
{
  implementation_->add_to_executor(executor);
}

void MicrophoneOnceRunner::remove_from_executor(rclcpp::Executor & executor)
{
  implementation_->remove_from_executor(executor);
}

MicrophoneOnceResult MicrophoneOnceRunner::pump() noexcept
{
  return implementation_->pump();
}

MicrophoneOnceResult MicrophoneOnceRunner::capture_until(
  const std::chrono::milliseconds timeout) noexcept
{
  return implementation_->capture_until(timeout);
}

MicrophoneOnceResult MicrophoneOnceRunner::expire() noexcept
{
  return implementation_->expire();
}

bool MicrophoneOnceRunner::allow_playback() noexcept
{
  return implementation_->allow_playback();
}

}  // namespace voice_nav_audio
