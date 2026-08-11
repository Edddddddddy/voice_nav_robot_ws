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

#include <portaudio.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "voice_nav_audio/spsc_audio_ring.hpp"

namespace voice_nav_audio
{
namespace
{
constexpr double kSampleRate = 48000.0;
constexpr std::size_t kFramesPerBuffer = 480U;
constexpr std::size_t kRingSamples = 48000U;

class AudioEngineNode final : public rclcpp::Node
{
public:
  AudioEngineNode()
  : Node("voice_audio_engine")
  {
    check(Pa_Initialize(), "PortAudio initialization");
    initialized_ = true;
    check(
      Pa_OpenDefaultStream(
        &stream_, 1, 1, paFloat32, kSampleRate, kFramesPerBuffer,
        &AudioEngineNode::callback, this),
      "PortAudio full-duplex open");
    running_.store(true, std::memory_order_release);
    worker_ = std::thread([this]() {work();});
    check(Pa_StartStream(stream_), "PortAudio stream start");
    timer_ = create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        RCLCPP_INFO(
          get_logger(), "audio blocks=%lu capture_overflow=%lu playback_underflow=%lu",
          blocks_.load(), capture_overflow_.load(), playback_underflow_.load());
      });
    RCLCPP_INFO(get_logger(), "48 kHz mono full-duplex AudioEngine started");
  }

  ~AudioEngineNode() override
  {
    running_.store(false, std::memory_order_release);
    if (worker_.joinable()) {
      worker_.join();
    }
    if (stream_ != nullptr) {
      Pa_StopStream(stream_);
      Pa_CloseStream(stream_);
    }
    if (initialized_) {
      Pa_Terminate();
    }
  }

private:
  static int callback(
    const void * input, void * output,
    const unsigned long frames,  // NOLINT(runtime/int)
    const PaStreamCallbackTimeInfo *, PaStreamCallbackFlags, void * context) noexcept
  {
    return static_cast<AudioEngineNode *>(context)->transfer(input, output, frames);
  }

  int transfer(
    const void * input, void * output,
    const unsigned long frames) noexcept  // NOLINT(runtime/int)
  {
    const auto * capture = static_cast<const float *>(input);
    auto * playback = static_cast<float *>(output);
    for (std::size_t index = 0U; index < frames; ++index) {
      const float captured = capture == nullptr ? 0.0F : capture[index];
      if (!capture_ring_.push(captured)) {
        capture_overflow_.fetch_add(1U, std::memory_order_relaxed);
      }
      float rendered = 0.0F;
      if (!playback_ring_.pop(rendered)) {
        playback_underflow_.fetch_add(1U, std::memory_order_relaxed);
      }
      playback[index] = rendered;
      render_ring_.push(rendered);
    }
    return paContinue;
  }

  void work()
  {
    std::array<float, kFramesPerBuffer> capture{};
    std::array<float, kFramesPerBuffer> render{};
    while (running_.load(std::memory_order_acquire)) {
      std::size_t count = 0U;
      float sample = 0.0F;
      while (count < capture.size() && capture_ring_.pop(sample)) {
        capture[count++] = sample;
      }
      std::size_t render_count = 0U;
      while (render_count < render.size() && render_ring_.pop(sample)) {
        render[render_count++] = sample;
      }
      if (count == capture.size() && render_count == render.size()) {
        // AEC/KWS/VAD/ASR adapters consume these paired 10 ms frames here.
        blocks_.fetch_add(1U, std::memory_order_relaxed);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  static void check(const PaError error, const std::string & operation)
  {
    if (error != paNoError) {
      throw std::runtime_error(operation + " failed: " + Pa_GetErrorText(error));
    }
  }

  PaStream * stream_{nullptr};
  bool initialized_{false};
  std::atomic<bool> running_{false};
  SpscAudioRing<kRingSamples> capture_ring_;
  SpscAudioRing<kRingSamples> playback_ring_;
  SpscAudioRing<kRingSamples> render_ring_;
  std::atomic<std::uint64_t> capture_overflow_{0U};
  std::atomic<std::uint64_t> playback_underflow_{0U};
  std::atomic<std::uint64_t> blocks_{0U};
  std::thread worker_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace
}  // namespace voice_nav_audio

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<voice_nav_audio::AudioEngineNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("voice_audio_engine"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
