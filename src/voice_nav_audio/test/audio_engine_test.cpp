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
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "voice_nav_audio/audio_engine.hpp"

namespace voice_nav_audio
{
namespace
{

TEST(AudioEngineTest, DeliversTenMillisecondCaptureAndExactSilenceReference)
{
  AudioEngine engine;
  std::array<Sample, AudioEngine::kFrameSamples> capture_input{};
  std::array<Sample, AudioEngine::kFrameSamples> device_output{};
  for (std::size_t index = 0U; index < capture_input.size(); ++index) {
    capture_input[index] = static_cast<Sample>(index - 120);
  }

  engine.process_callback(
    capture_input.data(), device_output.data(), device_output.size(), CallbackStatus{});

  AudioFrame captured{};
  AudioFrame reference{};
  ASSERT_TRUE(engine.try_pop_capture(captured));
  ASSERT_TRUE(engine.try_pop_reference(reference));
  EXPECT_EQ(captured.samples, capture_input);
  EXPECT_EQ(reference.samples, device_output);
  EXPECT_EQ(reference.generation, engine.generation());
  EXPECT_TRUE(std::all_of(device_output.begin(), device_output.end(), [](const Sample sample) {
      return sample == 0;
    }));
}

TEST(AudioEngineTest, ReferenceEqualsFinalGainFadeSaturatedAndShortDeviceOutput)
{
  AudioEngine engine;
  std::array<Sample, 4U> playback{{10000, -10000, 10000, -10000}};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  std::array<Sample, AudioEngine::kFrameSamples> capture{};
  engine.set_playback_gain(4.0F);
  engine.request_fade_to_silence(4U);
  ASSERT_TRUE(engine.enqueue_playback(playback.data(), playback.size()));

  engine.process_callback(capture.data(), output.data(), output.size(), CallbackStatus{});

  const std::array<Sample, 4U> expected{{32767, -30000, 20000, -10000}};
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), output.begin()));
  const auto short_buffer_begin = output.begin() + static_cast<std::ptrdiff_t>(expected.size());
  EXPECT_TRUE(std::all_of(short_buffer_begin, output.end(), [](const Sample sample) {
      return sample == 0;
    }));
  AudioFrame reference{};
  ASSERT_TRUE(engine.try_pop_reference(reference));
  EXPECT_EQ(reference.samples, output);
}

TEST(AudioEngineTest, XrunAndNonFrameCallbackRotateGenerationAndRenderSilence)
{
  AudioEngine engine;
  std::array<Sample, AudioEngine::kFrameSamples> samples{};
  samples.fill(900);
  const auto first_generation = engine.generation();

  engine.process_callback(
    samples.data(), samples.data(), samples.size(), CallbackStatus{false, true});

  EXPECT_GT(engine.generation(), first_generation);
  EXPECT_EQ(engine.metrics().xruns, 1U);
  EXPECT_EQ(engine.metrics().discontinuities, 1U);
  EXPECT_TRUE(std::all_of(samples.begin(), samples.end(), [](const Sample sample) {
      return sample == 0;
    }));

  std::array<Sample, AudioEngine::kFrameSamples - 1U> short_output{};
  short_output.fill(900);
  const auto xrun_generation = engine.generation();
  engine.process_callback(nullptr, short_output.data(), short_output.size(), CallbackStatus{});

  EXPECT_GT(engine.generation(), xrun_generation);
  EXPECT_EQ(engine.metrics().discontinuities, 2U);
  EXPECT_TRUE(std::all_of(short_output.begin(), short_output.end(), [](const Sample sample) {
      return sample == 0;
    }));
}

TEST(AudioEngineTest, ProducerDiscontinuityRequestCommitsAtCallbackBoundary)
{
  AudioEngine engine;
  std::array<Sample, AudioEngine::kFrameSamples> playback{};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  std::array<Sample, AudioEngine::kFrameSamples> capture{};
  playback.fill(700);
  ASSERT_TRUE(engine.enqueue_playback(playback.data(), playback.size()));
  const auto original_generation = engine.generation();

  std::atomic<bool> request_published{false};
  std::atomic<bool> callback_completed{false};
  std::thread producer([&engine, &request_published, &callback_completed]() {
      engine.mark_discontinuity();
      request_published.store(true, std::memory_order_release);
      while (!callback_completed.load(std::memory_order_acquire)) {
      }
    });

  while (!request_published.load(std::memory_order_acquire)) {
  }
  EXPECT_EQ(engine.generation(), original_generation);

  engine.process_callback(capture.data(), output.data(), output.size(), CallbackStatus{});
  callback_completed.store(true, std::memory_order_release);
  producer.join();

  EXPECT_EQ(engine.generation(), original_generation + 1U);
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));
}

TEST(AudioEngineTest, ReportsGenerationFenceAndNoStalePcmAfterCallbackBoundary)
{
  AudioEngine engine;
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  const auto generation_before = engine.generation();
  engine.mark_discontinuity();

  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});

  const auto metrics = engine.metrics();
  EXPECT_EQ(metrics.last_fence_generation_before, generation_before);
  EXPECT_EQ(metrics.last_fence_generation_after, generation_before + 1U);
  EXPECT_EQ(metrics.stale_pcm_after_fence, 0U);
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));
}

TEST(AudioEngineTest, RingLossFencesStaleFramesAndSelectsSilence)
{
  AudioEngine engine;
  std::array<Sample, AudioEngine::kFrameSamples> playback{};
  playback.fill(700);
  for (std::size_t index = 0U; index < AudioEngine::kRingCapacity - 1U; ++index) {
    ASSERT_TRUE(engine.enqueue_playback(playback.data(), playback.size()));
  }
  EXPECT_FALSE(engine.enqueue_playback(playback.data(), playback.size()));

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});

  EXPECT_EQ(engine.metrics().playback_overflows, 1U);
  EXPECT_GE(engine.metrics().playback_underflows, 1U);
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));

  AudioFrame reference{};
  ASSERT_TRUE(engine.try_pop_reference(reference));
  EXPECT_EQ(reference.samples, output);
}

TEST(AudioEngineTest, FullReferenceRingRendersSilenceBeforePublishingUnreferencedAudio)
{
  AudioEngine engine;
  std::array<Sample, AudioEngine::kFrameSamples> playback{};
  playback.fill(1500);
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  for (std::size_t index = 0U; index < AudioEngine::kRingCapacity - 1U; ++index) {
    engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  }
  ASSERT_TRUE(engine.enqueue_playback(playback.data(), playback.size()));

  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});

  EXPECT_EQ(engine.metrics().reference_overflows, 1U);
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));
  EXPECT_GT(engine.generation(), 1U);
}

TEST(AudioEngineTest, CaptureOverflowDropsOldestCompleteFrameWithoutRotatingGeneration)
{
  AudioEngine engine;
  std::array<Sample, AudioEngine::kFrameSamples> input{};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  AudioFrame reference{};
  for (std::size_t index = 0U; index <= AudioEngine::kRingCapacity; ++index) {
    input.fill(static_cast<Sample>(index + 1U));
    engine.process_callback(input.data(), output.data(), output.size(), CallbackStatus{});
    ASSERT_TRUE(engine.try_pop_reference(reference));
  }

  EXPECT_EQ(engine.metrics().capture_overflows, 0U);
  EXPECT_EQ(engine.generation(), 1U);
  AudioFrame capture{};
  ASSERT_TRUE(engine.try_pop_capture(capture));
  EXPECT_EQ(capture.samples.front(), 2);
  EXPECT_EQ(engine.metrics().capture_overflows, 1U);
  for (Sample expected = 3; expected <= 9; ++expected) {
    ASSERT_TRUE(engine.try_pop_capture(capture));
    EXPECT_EQ(capture.samples.front(), expected);
  }
  EXPECT_FALSE(engine.try_pop_capture(capture));
}

TEST(AudioEngineTest, CaptureSequenceGapIsReportedOnceAndBalancesProducedFrames)
{
  AudioEngine engine;
  std::array<Sample, AudioEngine::kFrameSamples> input{};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  AudioFrame reference{};
  constexpr std::size_t produced = AudioEngine::kRingCapacity + 3U;
  for (std::size_t index = 0U; index < produced; ++index) {
    input.fill(static_cast<Sample>(index + 1U));
    engine.process_callback(input.data(), output.data(), output.size(), CallbackStatus{});
    ASSERT_TRUE(engine.try_pop_reference(reference));
  }

  std::size_t delivered = 0U;
  AudioFrame capture{};
  while (engine.try_pop_capture(capture)) {
    ++delivered;
    EXPECT_TRUE(std::all_of(capture.samples.begin(), capture.samples.end(),
      [&capture](const Sample sample) {return sample == capture.samples.front();}));
  }

  const auto metrics = engine.metrics();
  EXPECT_EQ(metrics.capture_produced, produced);
  EXPECT_EQ(metrics.capture_delivered, delivered);
  EXPECT_EQ(metrics.capture_reported_drops, produced - delivered);
  EXPECT_EQ(metrics.capture_delivered + metrics.capture_reported_drops, metrics.capture_produced);
}

TEST(AudioEngineTest, ConcurrentCaptureOverwriteReportsOneOrderedGapAndBalancesExactly)
{
  AudioEngine engine;
  const auto initial_generation = engine.generation();
  constexpr std::size_t warmup_frames = AudioEngine::kRingCapacity + 3U;
  constexpr std::size_t produced = warmup_frames + 96U;
  std::mutex synchronization_mutex;
  std::condition_variable reference_available;
  std::condition_variable reference_drained;
  std::condition_variable capture_enabled;
  std::condition_variable capture_ready;
  std::condition_variable capture_available;
  std::condition_variable capture_drained;
  std::size_t produced_count = 0U;
  std::size_t drained_reference_count = 0U;
  std::size_t drained_capture_through = 0U;
  bool is_capture_enabled = false;
  bool is_capture_ready = false;
  bool producer_finished = false;
  bool failed = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);

  const auto fail = [&]() {
      std::lock_guard<std::mutex> lock(synchronization_mutex);
      failed = true;
      reference_available.notify_all();
      reference_drained.notify_all();
      capture_enabled.notify_all();
      capture_ready.notify_all();
      capture_available.notify_all();
      capture_drained.notify_all();
    };

  std::thread reference_consumer([&]() {
      for (std::size_t index = 0U; index < produced; ++index) {
        bool has_reference = false;
        {
          std::unique_lock<std::mutex> lock(synchronization_mutex);
          has_reference = reference_available.wait_until(lock, deadline, [&]() {
            return failed || produced_count > index;
            }) && !failed;
        }
        if (!has_reference) {
          fail();
          return;
        }

        AudioFrame reference{};
        if (!engine.try_pop_reference(reference) || reference.generation != initial_generation ||
        !std::all_of(reference.samples.begin(), reference.samples.end(),
        [](const Sample sample) {return sample == 0;}))
        {
          fail();
          return;
        }

        {
          std::lock_guard<std::mutex> lock(synchronization_mutex);
          drained_reference_count = index + 1U;
        }
        reference_drained.notify_one();
      }
    });

  std::atomic<bool> corrupt_frame{false};
  std::vector<Sample> delivered_frame_ids;
  std::thread capture_consumer([&]() {
      bool capture_is_enabled = false;
      {
        std::unique_lock<std::mutex> lock(synchronization_mutex);
        capture_is_enabled = capture_enabled.wait_until(lock, deadline, [&]() {
          return failed || is_capture_enabled;
          }) && !failed;
        if (capture_is_enabled) {
          is_capture_ready = true;
        }
      }
      if (!capture_is_enabled) {
        fail();
        return;
      }
      capture_ready.notify_one();

      std::size_t observed_produced = 0U;
      while (true) {
        bool has_capture_to_drain = false;
        bool is_finished = false;
        {
          std::unique_lock<std::mutex> lock(synchronization_mutex);
          has_capture_to_drain = capture_available.wait_until(lock, deadline, [&]() {
            return failed || producer_finished || produced_count > observed_produced;
            }) && !failed;
          observed_produced = produced_count;
          is_finished = producer_finished;
        }
        if (!has_capture_to_drain) {
          fail();
          return;
        }

        AudioFrame capture{};
        if (engine.try_pop_capture(capture)) {
          if (capture.generation != initial_generation ||
          !std::all_of(capture.samples.begin(), capture.samples.end(),
          [&capture](const Sample sample) {return sample == capture.samples.front();}))
          {
            corrupt_frame.store(true, std::memory_order_relaxed);
            fail();
            return;
          }
          delivered_frame_ids.push_back(capture.samples.front());
          while (engine.try_pop_capture(capture)) {
            if (capture.generation != initial_generation ||
            !std::all_of(capture.samples.begin(), capture.samples.end(),
            [&capture](const Sample sample) {return sample == capture.samples.front();}))
            {
              corrupt_frame.store(true, std::memory_order_relaxed);
              fail();
              return;
            }
            delivered_frame_ids.push_back(capture.samples.front());
          }
        }
        {
          std::lock_guard<std::mutex> lock(synchronization_mutex);
          drained_capture_through = observed_produced;
        }
        capture_drained.notify_one();
        if (is_finished) {
          return;
        }
      }
      fail();
    });

  std::thread producer([&]() {
      std::array<Sample, AudioEngine::kFrameSamples> input{};
      std::array<Sample, AudioEngine::kFrameSamples> output{};
      for (std::size_t index = 0U; index < produced; ++index) {
        input.fill(static_cast<Sample>(index + 1U));
        engine.process_callback(input.data(), output.data(), output.size(), CallbackStatus{});

        {
          std::lock_guard<std::mutex> lock(synchronization_mutex);
          produced_count = index + 1U;
          if (produced_count == warmup_frames) {
            is_capture_enabled = true;
          }
        }
        reference_available.notify_one();
        if (index + 1U == warmup_frames) {
          capture_enabled.notify_one();
        }

        {
          std::unique_lock<std::mutex> lock(synchronization_mutex);
          if (!reference_drained.wait_until(lock, deadline, [&]() {
            return failed || drained_reference_count >= produced_count;
              }) || failed)
          {
            break;
          }
          if (index + 1U == warmup_frames &&
          (!capture_ready.wait_until(lock, deadline, [&]() {
            return failed || is_capture_ready;
              }) || failed))
          {
            break;
          }
          if (is_capture_enabled) {
            capture_available.notify_one();
            if (!capture_drained.wait_until(lock, deadline, [&]() {
              return failed || drained_capture_through >= produced_count;
                }) || failed)
            {
              break;
            }
          }
        }
      }

      {
        std::lock_guard<std::mutex> lock(synchronization_mutex);
        producer_finished = true;
      }
      capture_available.notify_one();
    });

  producer.join();
  reference_consumer.join();
  capture_consumer.join();

  const auto metrics = engine.metrics();
  EXPECT_FALSE(failed);
  EXPECT_FALSE(corrupt_frame.load(std::memory_order_relaxed));
  ASSERT_FALSE(delivered_frame_ids.empty());
  std::size_t gap_count = 0U;
  std::size_t missing_frame_count = 0U;
  Sample previous_frame_id = 0;
  for (const Sample frame_id : delivered_frame_ids) {
    EXPECT_GT(frame_id, previous_frame_id);
    if (frame_id - previous_frame_id > 1) {
      ++gap_count;
      missing_frame_count += static_cast<std::size_t>(frame_id - previous_frame_id - 1);
    }
    previous_frame_id = frame_id;
  }
  EXPECT_EQ(gap_count, 1U);
  EXPECT_EQ(metrics.capture_produced, produced);
  EXPECT_EQ(metrics.capture_delivered, delivered_frame_ids.size());
  EXPECT_GT(metrics.capture_reported_drops, 0U);
  EXPECT_EQ(metrics.capture_overflows, metrics.capture_reported_drops);
  EXPECT_EQ(metrics.capture_reported_drops, missing_frame_count);
  EXPECT_EQ(metrics.capture_delivered + metrics.capture_reported_drops, metrics.capture_produced);
  EXPECT_EQ(engine.generation(), initial_generation);
  EXPECT_EQ(metrics.discontinuities, 0U);
}

TEST(AudioEngineTest, InMemorySpscStressNeverLetsACallbackExceptionEscape)
{
  AudioEngine engine;
  std::atomic<bool> start{false};
  std::thread worker([&engine, &start]() {
      std::array<Sample, AudioEngine::kFrameSamples> playback{};
      playback.fill(300);
      AudioFrame capture{};
      AudioFrame reference{};
      while (!start.load(std::memory_order_acquire)) {
      }
      for (std::size_t index = 0U; index < 2000U; ++index) {
        (void)engine.enqueue_playback(playback.data(), playback.size());
        (void)engine.try_pop_capture(capture);
        (void)engine.try_pop_reference(reference);
      }
    });

  std::array<Sample, AudioEngine::kFrameSamples> capture{};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  start.store(true, std::memory_order_release);
  for (std::size_t index = 0U; index < 2000U; ++index) {
    engine.process_callback(capture.data(), output.data(), output.size(), CallbackStatus{});
  }
  worker.join();
  EXPECT_GT(engine.metrics().playback_underflows + engine.metrics().playback_overflows, 0U);
}

}  // namespace
}  // namespace voice_nav_audio
