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

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"
#include "audio_engine_callback_test_support.hpp"
#include "portaudio_native_callback.hpp"
#include "voice_nav_audio/portaudio_adapter.hpp"

namespace voice_nav_audio
{
namespace
{

class ScriptedDevice final : public FullDuplexAudioDevice
{
public:
  bool open(
    const FullDuplexStreamSpec spec,
    const DeviceCallback callback,
    void * const context) noexcept override
  {
    open_spec = spec;
    callback_ = callback;
    context_ = context;
    return available;
  }

  void close() noexcept override
  {
    callback_ = nullptr;
    context_ = nullptr;
  }

  void fire(
    const std::array<Sample, AudioEngine::kFrameSamples> & input,
    std::array<Sample, AudioEngine::kFrameSamples> & output) const noexcept
  {
    if (callback_ != nullptr) {
      callback_(context_, input.data(), output.data(), output.size(), CallbackStatus{});
    }
  }

  [[nodiscard]] bool has_callback() const noexcept
  {
    return callback_ != nullptr;
  }

  bool available{true};
  FullDuplexStreamSpec open_spec{};

private:
  DeviceCallback callback_{nullptr};
  void * context_{nullptr};
};

class CallbackBoundaryBarrier final
{
public:
  void install() noexcept
  {
    active_ = this;
    test_support::set_callback_boundary_hook(&CallbackBoundaryBarrier::on_callback_boundary);
  }

  void uninstall() noexcept
  {
    test_support::set_callback_boundary_hook(nullptr);
    active_ = nullptr;
  }

  static void on_callback_boundary() noexcept
  {
    active_->on_entered();
  }

  void on_entered() noexcept
  {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_ = true;
    entered_condition_.notify_one();
    release_condition_.wait(lock, [this]() {return released_;});
  }

  void wait_until_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_condition_.wait(lock, [this]() {return entered_;});
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    release_condition_.notify_one();
  }

private:
  inline static CallbackBoundaryBarrier * active_{nullptr};
  mutable std::mutex mutex_;
  std::condition_variable entered_condition_;
  std::condition_variable release_condition_;
  bool entered_{false};
  bool released_{false};
};

void forward_native_callback(
  void * const context,
  const Sample * const capture,
  Sample * const device_output,
  const std::size_t frame_count,
  const CallbackStatus status) noexcept
{
  static_cast<AudioEngine *>(context)->process_callback(
    capture, device_output, frame_count, status);
}

TEST(PortAudioAdapterTest, OpensOneFixedStreamAndRestartFencesOldPlayback)
{
  AudioEngine engine;
  ScriptedDevice device;
  std::array<Sample, AudioEngine::kFrameSamples> input{};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  std::array<Sample, AudioEngine::kFrameSamples> old_playback{};
  old_playback.fill(1200);

  {
    PortAudioAdapter adapter(engine, device);
    ASSERT_EQ(adapter.start(), AdapterStartResult::Started);
    EXPECT_EQ(device.open_spec.sample_rate, AudioEngine::kSampleRate);
    EXPECT_EQ(device.open_spec.channels, AudioEngine::kChannels);
    EXPECT_EQ(device.open_spec.frames_per_buffer, AudioEngine::kFrameSamples);
    ASSERT_TRUE(engine.enqueue_playback(old_playback.data(), old_playback.size()));

    ASSERT_TRUE(adapter.restart());
    device.fire(input, output);
    EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
        return sample == 0;
      }));
    EXPECT_TRUE(device.has_callback());
  }

  EXPECT_FALSE(device.has_callback());
}

TEST(PortAudioAdapterTest, PlaybackOverflowQuarantinesAnAlreadyEnteredCallback)
{
  AudioEngine engine;
  ScriptedDevice device;
  PortAudioAdapter adapter(engine, device);
  std::array<Sample, AudioEngine::kFrameSamples> input{};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  std::array<Sample, AudioEngine::kFrameSamples> old_output{};
  std::array<Sample, AudioEngine::kFrameSamples> playback{};
  playback.fill(1200);

  ASSERT_EQ(adapter.start(), AdapterStartResult::Started);
  device.fire(input, output);
  AudioFrame discarded_capture{};
  AudioFrame discarded_reference{};
  ASSERT_TRUE(engine.try_pop_capture(discarded_capture));
  ASSERT_TRUE(engine.try_pop_reference(discarded_reference));

  const auto old_generation = engine.generation();
  const auto old_playback_generation = engine.playback_generation();
  ASSERT_TRUE(engine.enqueue_playback(playback.data(), playback.size()));
  CallbackBoundaryBarrier barrier;
  barrier.install();

  std::thread old_callback([&device, &input, &old_output]() {
      device.fire(input, old_output);
    });
  barrier.wait_until_entered();

  for (std::size_t index = 0U; index < AudioEngine::kPlaybackRingCapacity - 1U; ++index) {
    (void)engine.enqueue_playback(playback.data(), playback.size());
  }
  EXPECT_EQ(engine.metrics().playback_overflows, 1U);
  EXPECT_EQ(engine.generation(), old_generation);

  barrier.release();
  old_callback.join();
  barrier.uninstall();
  EXPECT_TRUE(std::all_of(old_output.begin(), old_output.end(), [](const Sample sample) {
      return sample == 0;
    }));
  EXPECT_EQ(engine.generation(), old_generation);

  AudioFrame quarantined_reference{};
  ASSERT_TRUE(engine.try_pop_reference(quarantined_reference));
  EXPECT_EQ(quarantined_reference.generation, old_generation);
  EXPECT_TRUE(std::all_of(
    quarantined_reference.samples.begin(), quarantined_reference.samples.end(),
      [](const Sample sample) {return sample == 0;}));
  EXPECT_EQ(quarantined_reference.samples, old_output);

  device.fire(input, output);
  EXPECT_EQ(engine.generation(), old_generation);
  EXPECT_EQ(engine.playback_generation(), old_playback_generation + 1U);
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));
  AudioFrame final_reference{};
  ASSERT_TRUE(engine.try_pop_reference(final_reference));
  EXPECT_EQ(final_reference.generation, engine.generation());
  EXPECT_TRUE(std::all_of(final_reference.samples.begin(), final_reference.samples.end(),
    [](const Sample sample) {return sample == 0;}));
}

TEST(PortAudioAdapterTest, InputUnderflowDoesNotFenceCurrentPlaybackGeneration)
{
  AudioEngine engine;
  std::array<Sample, AudioEngine::kFrameSamples> playback{};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  playback.fill(1200);
  ASSERT_TRUE(engine.enqueue_playback(playback.data(), playback.size(), 9U));
  const auto playback_generation = engine.playback_generation();
  NativePortAudioCallbackContext context{&forward_native_callback, &engine};

  ASSERT_EQ(
    native_portaudio_callback(
      nullptr, output.data(), output.size(), nullptr, 0x00000001U, &context),
    0);

  EXPECT_EQ(engine.playback_generation(), playback_generation);
  EXPECT_TRUE(std::any_of(output.begin(), output.end(), [](const Sample sample) {
      return sample != 0;
    }));
}

TEST(PortAudioAdapterTest, InputOverflowOnlyFencesCaptureAndPreservesPlaybackScope)
{
  AudioEngine engine;
  std::array<Sample, AudioEngine::kFrameSamples> capture{};
  std::array<Sample, AudioEngine::kFrameSamples> playback{};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  playback.fill(1200);
  ASSERT_TRUE(engine.enqueue_playback(playback.data(), playback.size(), 9U));
  NativePortAudioCallbackContext context{&forward_native_callback, &engine};

  capture.fill(11);
  ASSERT_EQ(
    native_portaudio_callback(
      capture.data(), output.data(), output.size(), nullptr, 0U, &context),
    0);
  ASSERT_TRUE(std::any_of(output.begin(), output.end(), [](const Sample sample) {
      return sample != 0;
    }));

  ASSERT_TRUE(engine.enqueue_playback(playback.data(), playback.size(), 9U));
  const auto capture_generation = engine.generation();
  const auto playback_generation = engine.playback_generation();
  const auto metrics_before = engine.metrics();
  capture.fill(22);
  output.fill(900);

  ASSERT_EQ(
    native_portaudio_callback(
      capture.data(), output.data(), output.size(), nullptr, 0x00000002U, &context),
    0);

  EXPECT_EQ(engine.generation(), capture_generation + 1U);
  EXPECT_EQ(engine.playback_generation(), playback_generation);
  EXPECT_EQ(engine.metrics().stale_pcm_after_fence, metrics_before.stale_pcm_after_fence);
  EXPECT_EQ(engine.metrics().xruns, metrics_before.xruns + 1U);
  EXPECT_EQ(engine.metrics().discontinuities, metrics_before.discontinuities + 1U);
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));

  AudioFrame stale_capture{};
  EXPECT_FALSE(engine.try_pop_capture(stale_capture));

  output.fill(900);
  ASSERT_EQ(
    native_portaudio_callback(
      nullptr, output.data(), output.size(), nullptr, 0x00000004U, &context),
    0);
  EXPECT_EQ(engine.playback_generation(), playback_generation + 1U);
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));
}

TEST(PortAudioAdapterTest, UnavailableDeviceFailsClosedWithoutInstallingCallback)
{
  AudioEngine engine;
  ScriptedDevice device;
  device.available = false;
  PortAudioAdapter adapter(engine, device);

  EXPECT_EQ(adapter.start(), AdapterStartResult::NoDevice);
  EXPECT_FALSE(adapter.running());
  EXPECT_FALSE(device.has_callback());
}

TEST(PortAudioAdapterTest, DefaultAdapterFailsClosedUntilPortAudioIsProvisioned)
{
  AudioEngine engine;
  PortAudioAdapter adapter(engine);

  EXPECT_EQ(adapter.start(), AdapterStartResult::NoDevice);
  EXPECT_FALSE(adapter.running());
}

}  // namespace
}  // namespace voice_nav_audio
