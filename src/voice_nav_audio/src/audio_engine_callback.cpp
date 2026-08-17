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

#ifdef VOICE_NAV_AUDIO_TEST_CALLBACK_BOUNDARY
#include "audio_engine_callback_test_support.hpp"
#endif

#include <algorithm>
#include <limits>

namespace voice_nav_audio
{
namespace
{

constexpr std::uint32_t kUnityGainQ15 = 32768U;

}  // namespace

void AudioEngine::process_callback(
  const Sample * const capture,
  Sample * const device_output,
  const std::size_t frame_count,
  const CallbackStatus status) noexcept
{
  try {
    commit_pending_discontinuities();
    if (status.input_overflow || status.output_underflow) {
      xruns_.fetch_add(1U, std::memory_order_relaxed);
      mark_discontinuity();
      commit_pending_discontinuities();
      if (device_output != nullptr) {
        std::fill_n(device_output, frame_count, static_cast<Sample>(0));
      }
      return;
    }

    if (frame_count != kFrameSamples) {
      mark_discontinuity();
      commit_pending_discontinuities();
      if (device_output != nullptr) {
        std::fill_n(device_output, frame_count, static_cast<Sample>(0));
      }
      return;
    }

    const auto callback_generation = generation();
    const auto callback_phase = static_cast<AudioEnginePhase>(
      phase_.load(std::memory_order_acquire));
#ifdef VOICE_NAV_AUDIO_TEST_CALLBACK_BOUNDARY
    test_support::invoke_callback_boundary_hook();
#endif
    AudioFrame rendered{};
    rendered.generation = callback_generation;
    if (callback_phase == AudioEnginePhase::kCapture && !reference_ring_.can_push())
    {
      reference_overflows_.fetch_add(1U, std::memory_order_relaxed);
      mark_discontinuity();
      commit_pending_discontinuities();
      if (device_output != nullptr) {
        std::fill_n(device_output, kFrameSamples, static_cast<Sample>(0));
      }
      return;
    }
    PlaybackPacket rendered_packet{};
    const bool rendered_playback = render_playback(
      rendered, callback_generation, rendered_packet);

    // This is the publication linearization point for an already-entered
    // callback.  Both externally visible playback copies must originate from
    // this same final frame, so a request observed here quarantines them
    // together without committing the next generation early.
    if (has_pending_discontinuities()) {
      rendered.samples.fill(0);
    }
    if (device_output != nullptr) {
      std::copy(rendered.samples.begin(), rendered.samples.end(), device_output);
    }

    const bool wrote_non_silent_pcm = rendered_playback &&
      std::any_of(
      rendered.samples.begin(),
      rendered.samples.begin() + static_cast<std::ptrdiff_t>(rendered_packet.sample_count),
      [](const Sample sample) {return sample != 0;});
    if (device_output != nullptr && wrote_non_silent_pcm && rendered_packet.scope_id != 0U &&
      !has_pending_discontinuities())
    {
      const PlaybackWrite write{
        rendered_packet.scope_id, callback_generation, rendered_packet.sample_count};
      if (!playback_write_ring_.push(write)) {
        // Losing accounting would falsely inflate played.  Fail closed before
        // a subsequent callback can expose more scope-owned audio.
        mark_discontinuity();
      }
    }

    if (callback_phase == AudioEnginePhase::kPlaybackOnly) {
      return;
    }

    (void)reference_ring_.push(rendered);

    AudioFrame captured{};
    captured.generation = callback_generation;
    if (capture != nullptr) {
      std::copy_n(capture, kFrameSamples, captured.samples.begin());
    }
    capture_ring_.push_drop_oldest(captured);
    capture_produced_.fetch_add(1U, std::memory_order_relaxed);
  } catch (...) {
    mark_discontinuity();
    commit_pending_discontinuities();
    if (device_output != nullptr) {
      std::fill_n(device_output, frame_count, static_cast<Sample>(0));
    }
  }
}

bool AudioEngine::pop_playback_for_current_generation(
  PlaybackPacket & packet, const std::uint64_t expected_generation) noexcept
{
  PlaybackPacket candidate{};
  for (std::size_t attempt = 0U; attempt < kRingCapacity; ++attempt) {
    if (!playback_ring_.pop(candidate)) {
      return false;
    }
    if (candidate.generation == expected_generation) {
      packet = candidate;
      return true;
    }
    stale_pcm_after_fence_.fetch_add(1U, std::memory_order_relaxed);
  }
  return false;
}

bool AudioEngine::has_pending_discontinuities() const noexcept
{
  return discontinuity_requests_.load(std::memory_order_acquire) !=
         observed_discontinuity_requests_;
}

void AudioEngine::commit_pending_discontinuities() noexcept
{
  const auto requested = discontinuity_requests_.load(std::memory_order_acquire);
  if (requested == observed_discontinuity_requests_) {
    return;
  }
  const auto count = requested - observed_discontinuity_requests_;
  const auto generation_before = generation_.load(std::memory_order_acquire);
  const auto generation_after = generation_before + count;
  generation_.store(generation_after, std::memory_order_release);
  last_fence_generation_before_.store(generation_before, std::memory_order_release);
  last_fence_generation_after_.store(generation_after, std::memory_order_release);
  stale_pcm_after_fence_.store(0U, std::memory_order_release);
  discontinuities_.fetch_add(count, std::memory_order_relaxed);
  observed_discontinuity_requests_ = requested;
}

bool AudioEngine::render_playback(
  AudioFrame & rendered, const std::uint64_t callback_generation,
  PlaybackPacket & rendered_packet) noexcept
{
  const auto requested_fade = fade_request_.load(std::memory_order_acquire);
  if (requested_fade != observed_fade_request_) {
    observed_fade_request_ = requested_fade;
    fade_total_ = fade_sample_count_.load(std::memory_order_acquire);
    fade_remaining_ = fade_total_;
  }

  PlaybackPacket packet{};
  const bool have_playback = pop_playback_for_current_generation(packet, callback_generation);
  if (!have_playback) {
    playback_underflows_.fetch_add(1U, std::memory_order_relaxed);
  }

  const auto gain = gain_q15_.load(std::memory_order_acquire);
  for (std::size_t index = 0U; index < kFrameSamples; ++index) {
    const auto input = have_playback && index < packet.sample_count ? packet.samples[index] : 0;
    std::int64_t value = static_cast<std::int64_t>(input) * gain;
    value /= static_cast<std::int64_t>(kUnityGainQ15);
    if (fade_remaining_ > 0U && fade_total_ > 0U) {
      value *= fade_remaining_;
      value /= fade_total_;
      --fade_remaining_;
    } else if (fade_total_ > 0U) {
      value = 0;
    }
    rendered.samples[index] = saturate(value);
  }
  if (fade_remaining_ == 0U) {
    fade_total_ = 0U;
  }
  if (have_playback) {
    rendered_packet = packet;
  }
  return have_playback;
}

Sample AudioEngine::saturate(const std::int64_t value) noexcept
{
  if (value > std::numeric_limits<Sample>::max()) {
    return std::numeric_limits<Sample>::max();
  }
  if (value < std::numeric_limits<Sample>::min()) {
    return std::numeric_limits<Sample>::min();
  }
  return static_cast<Sample>(value);
}

}  // namespace voice_nav_audio
