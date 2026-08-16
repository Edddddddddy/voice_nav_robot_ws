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

#include "voice_nav_audio/audio_engine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace voice_nav_audio
{
namespace
{

constexpr std::uint32_t kUnityGainQ15 = 32768U;
constexpr std::uint32_t kMaximumGainQ15 = 4U * kUnityGainQ15;

}  // namespace

bool AudioEngine::enqueue_playback(
  const Sample * const samples,
  const std::size_t sample_count,
  const std::uint64_t scope_id) noexcept
{
  if (samples == nullptr || sample_count == 0U || sample_count > kFrameSamples) {
    return false;
  }

  PlaybackPacket packet{};
  packet.generation = generation();
  packet.scope_id = scope_id;
  packet.sample_count = sample_count;
  std::copy_n(samples, sample_count, packet.samples.begin());
  if (playback_ring_.push(packet)) {
    return true;
  }

  playback_overflows_.fetch_add(1U, std::memory_order_relaxed);
  mark_discontinuity();
  return false;
}

bool AudioEngine::try_pop_capture(AudioFrame & frame) noexcept
{
  AudioFrame candidate{};
  const auto result = capture_ring_.pop(candidate);
  if (result.reported_drops > 0U) {
    capture_overflows_.fetch_add(result.reported_drops, std::memory_order_relaxed);
    capture_reported_drops_.fetch_add(result.reported_drops, std::memory_order_relaxed);
  }
  if (!result.delivered) {
    return false;
  }
  const auto expected_generation = generation();
  if (candidate.generation != expected_generation || generation() != expected_generation) {
    capture_overflows_.fetch_add(1U, std::memory_order_relaxed);
    capture_reported_drops_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }
  frame = candidate;
  capture_delivered_.fetch_add(1U, std::memory_order_relaxed);
  return true;
}

bool AudioEngine::try_pop_reference(AudioFrame & frame) noexcept
{
  AudioFrame candidate{};
  const auto expected_generation = generation();
  for (std::size_t attempt = 0U; attempt < kRingCapacity; ++attempt) {
    if (!reference_ring_.pop(candidate)) {
      return false;
    }
    if (candidate.generation == expected_generation &&
      generation() == expected_generation)
    {
      frame = candidate;
      return true;
    }
  }
  return false;
}

bool AudioEngine::try_pop_playback_write(PlaybackWrite & write) noexcept
{
  return playback_write_ring_.pop(write);
}

void AudioEngine::set_playback_gain(float gain) noexcept
{
  if (!std::isfinite(gain) || gain < 0.0F) {
    gain = 0.0F;
  }
  const auto clamped = std::min(gain, 4.0F);
  gain_q15_.store(
    static_cast<std::uint32_t>(clamped * static_cast<float>(kUnityGainQ15) + 0.5F),
    std::memory_order_release);
}

void AudioEngine::request_fade_to_silence(const std::size_t sample_count) noexcept
{
  const auto bounded_count = static_cast<std::uint32_t>(std::min(
    sample_count, static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
  fade_sample_count_.store(bounded_count, std::memory_order_release);
  fade_request_.fetch_add(1U, std::memory_order_release);
}

void AudioEngine::mark_discontinuity() noexcept
{
  discontinuity_requests_.fetch_add(1U, std::memory_order_release);
}

std::uint64_t AudioEngine::generation() const noexcept
{
  return generation_.load(std::memory_order_acquire);
}

AudioMetrics AudioEngine::metrics() const noexcept
{
  return AudioMetrics{
    capture_overflows_.load(std::memory_order_relaxed),
    capture_produced_.load(std::memory_order_relaxed),
    capture_delivered_.load(std::memory_order_relaxed),
    capture_reported_drops_.load(std::memory_order_relaxed),
    playback_overflows_.load(std::memory_order_relaxed),
    reference_overflows_.load(std::memory_order_relaxed),
    playback_underflows_.load(std::memory_order_relaxed),
    xruns_.load(std::memory_order_relaxed),
    discontinuities_.load(std::memory_order_relaxed),
    last_fence_generation_before_.load(std::memory_order_acquire),
    last_fence_generation_after_.load(std::memory_order_acquire),
    stale_pcm_after_fence_.load(std::memory_order_acquire)};
}

}  // namespace voice_nav_audio
