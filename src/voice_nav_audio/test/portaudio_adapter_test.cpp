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

TEST(PortAudioAdapterTest, OneShotPauseResumesWithoutAPlaybackGenerationFence)
{
  AudioEngine engine;
  ScriptedDevice device;
  PortAudioAdapter adapter(engine, device);
  std::array<Sample, AudioEngine::kFrameSamples> input{};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  std::array<Sample, AudioEngine::kFrameSamples> playback{};
  playback.fill(1200);

  ASSERT_EQ(adapter.start(), AdapterStartResult::Started);
  device.fire(input, output);
  AudioFrame capture{};
  AudioFrame reference{};
  ASSERT_TRUE(engine.try_pop_capture(capture));
  ASSERT_TRUE(engine.try_pop_reference(reference));
  const auto generation = engine.generation();

  ASSERT_TRUE(adapter.pause_for_playback());
  EXPECT_FALSE(device.has_callback());
  EXPECT_EQ(engine.generation(), generation);
  ASSERT_TRUE(engine.enqueue_playback(playback.data(), playback.size(), 9U));

  ASSERT_EQ(adapter.resume_playback(), AdapterStartResult::Started);
  EXPECT_EQ(engine.generation(), generation);
  device.fire(input, output);
  EXPECT_TRUE(std::any_of(output.begin(), output.end(), [](const Sample sample) {
      return sample != 0;
    }));
  PlaybackWrite write{};
  ASSERT_TRUE(engine.try_pop_playback_write(write));
  EXPECT_EQ(write.scope_id, 9U);
  EXPECT_EQ(write.generation, generation);
  AudioFrame playback_capture{};
  AudioFrame playback_reference{};
  EXPECT_FALSE(engine.try_pop_capture(playback_capture));
  EXPECT_FALSE(engine.try_pop_reference(playback_reference));
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
  ASSERT_TRUE(engine.enqueue_playback(playback.data(), playback.size()));
  CallbackBoundaryBarrier barrier;
  barrier.install();

  std::thread old_callback([&device, &input, &old_output]() {
      device.fire(input, old_output);
    });
  barrier.wait_until_entered();

  for (std::size_t index = 0U; index < AudioEngine::kRingCapacity - 1U; ++index) {
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
  EXPECT_EQ(engine.generation(), old_generation + 1U);
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));
  AudioFrame final_reference{};
  ASSERT_TRUE(engine.try_pop_reference(final_reference));
  EXPECT_EQ(final_reference.generation, engine.generation());
  EXPECT_TRUE(std::all_of(final_reference.samples.begin(), final_reference.samples.end(),
    [](const Sample sample) {return sample == 0;}));
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
