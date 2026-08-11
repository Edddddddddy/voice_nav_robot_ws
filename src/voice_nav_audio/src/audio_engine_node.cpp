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

#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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
    capture_fifo_ = declare_parameter<std::string>("capture_fifo", "");
    playback_fifo_ = declare_parameter<std::string>("playback_fifo", "");
    ::signal(SIGPIPE, SIG_IGN);
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
          get_logger(),
          "audio blocks=%lu fifo_frames=%lu fifo_drops=%lu capture_overflow=%lu "
          "playback_frames=%lu playback_drops=%lu playback_underflow=%lu",
          blocks_.load(), fifo_frames_.load(), fifo_drops_.load(),
          capture_overflow_.load(), playback_frames_.load(),
          playback_drops_.load(), playback_underflow_.load());
      });
    RCLCPP_INFO(get_logger(), "48 kHz mono full-duplex AudioEngine started");
  }

  ~AudioEngineNode() override
  {
    running_.store(false, std::memory_order_release);
    if (worker_.joinable()) {
      worker_.join();
    }
    close_capture_fifo();
    close_playback_fifo();
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
      read_playback_frame();
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
        write_capture_frame(capture);
        blocks_.fetch_add(1U, std::memory_order_relaxed);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  void write_capture_frame(const std::array<float, kFramesPerBuffer> & capture)
  {
    if (capture_fifo_.empty()) {
      return;
    }
    if (capture_fifo_fd_ < 0) {
      capture_fifo_fd_ = open(
        capture_fifo_.c_str(), O_WRONLY | O_NONBLOCK | O_CLOEXEC);
      if (capture_fifo_fd_ < 0) {
        fifo_drops_.fetch_add(1U, std::memory_order_relaxed);
        return;
      }
    }
    std::array<std::int16_t, kFramesPerBuffer / 3U> output{};
    for (std::size_t index = 0U; index < output.size(); ++index) {
      const auto offset = index * 3U;
      const float averaged = (
        capture[offset] + capture[offset + 1U] + capture[offset + 2U]) / 3.0F;
      const float bounded = std::clamp(averaged, -1.0F, 1.0F);
      output[index] = static_cast<std::int16_t>(
        std::lrint(bounded * 32767.0F));
    }
    const auto bytes = output.size() * sizeof(output.front());
    const auto written = ::write(capture_fifo_fd_, output.data(), bytes);
    if (written == static_cast<ssize_t>(bytes)) {
      fifo_frames_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    fifo_drops_.fetch_add(1U, std::memory_order_relaxed);
    close_capture_fifo();
  }

  void close_capture_fifo()
  {
    if (capture_fifo_fd_ >= 0) {
      close(capture_fifo_fd_);
      capture_fifo_fd_ = -1;
    }
  }

  void read_playback_frame()
  {
    if (
      playback_fifo_.empty() ||
      playback_ring_.write_available() < kFramesPerBuffer)
    {
      return;
    }
    if (playback_fifo_fd_ < 0) {
      playback_fifo_fd_ = open(
        playback_fifo_.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
      if (playback_fifo_fd_ < 0) {
        return;
      }
    }
    std::array<std::int16_t, kFramesPerBuffer> input{};
    const auto bytes = input.size() * sizeof(input.front());
    const auto received = ::read(playback_fifo_fd_, input.data(), bytes);
    if (received == static_cast<ssize_t>(bytes)) {
      for (const auto sample : input) {
        playback_ring_.push(static_cast<float>(sample) / 32768.0F);
      }
      playback_frames_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    if (received > 0) {
      playback_drops_.fetch_add(1U, std::memory_order_relaxed);
    }
    if (received >= 0) {
      close_playback_fifo();
    }
  }

  void close_playback_fifo()
  {
    if (playback_fifo_fd_ >= 0) {
      close(playback_fifo_fd_);
      playback_fifo_fd_ = -1;
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
  std::atomic<std::uint64_t> fifo_frames_{0U};
  std::atomic<std::uint64_t> fifo_drops_{0U};
  std::atomic<std::uint64_t> playback_frames_{0U};
  std::atomic<std::uint64_t> playback_drops_{0U};
  std::string capture_fifo_;
  std::string playback_fifo_;
  int capture_fifo_fd_{-1};
  int playback_fifo_fd_{-1};
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
