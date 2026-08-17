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

#ifndef VOICE_NAV_AUDIO__AUDIO_ENGINE_HPP_
#define VOICE_NAV_AUDIO__AUDIO_ENGINE_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace voice_nav_audio
{

static_assert(std::atomic<std::size_t>::is_always_lock_free);
static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
static_assert(std::atomic<std::uint8_t>::is_always_lock_free);

using Sample = std::int16_t;

static_assert(std::atomic<Sample>::is_always_lock_free);

struct AudioFrame
{
  std::uint64_t generation{0U};
  std::array<Sample, 480U> samples{};
};

struct CallbackStatus
{
  bool input_overflow{false};
  bool output_underflow{false};
};

enum class AudioEnginePhase : std::uint8_t
{
  kCapture,
  kPlaybackOnly,
};

struct AudioMetrics
{
  std::uint64_t capture_overflows{0U};
  std::uint64_t capture_produced{0U};
  std::uint64_t capture_delivered{0U};
  std::uint64_t capture_reported_drops{0U};
  std::uint64_t playback_overflows{0U};
  std::uint64_t reference_overflows{0U};
  std::uint64_t playback_underflows{0U};
  std::uint64_t xruns{0U};
  std::uint64_t discontinuities{0U};
  std::uint64_t last_fence_generation_before{0U};
  std::uint64_t last_fence_generation_after{0U};
  std::uint64_t stale_pcm_after_fence{0U};
};

struct PlaybackWrite
{
  std::uint64_t scope_id{0U};
  std::uint64_t generation{0U};
  std::size_t sample_count{0U};
};

// Package-private real-time core.  Its callback only copies fixed-size PCM,
// updates atomics, and selects already-prepared output state.
class AudioEngine final
{
public:
  static constexpr std::size_t kSampleRate = 48000U;
  static constexpr std::size_t kChannels = 1U;
  static constexpr std::size_t kFrameSamples = 480U;
  static constexpr std::size_t kRingCapacity = 8U;

  AudioEngine() = default;

  [[nodiscard]] bool enqueue_playback(
    const Sample * samples, std::size_t sample_count, std::uint64_t scope_id = 0U) noexcept;
  [[nodiscard]] bool try_pop_capture(AudioFrame & frame) noexcept;
  [[nodiscard]] bool try_pop_reference(AudioFrame & frame) noexcept;
  [[nodiscard]] bool try_pop_playback_write(PlaybackWrite & write) noexcept;

  void set_playback_gain(float gain) noexcept;
  void request_fade_to_silence(std::size_t sample_count) noexcept;
  // May be called by a control/producer thread.  It only publishes a
  // lock-free request; the audio callback commits the generation fence.
  void mark_discontinuity() noexcept;
  // Capture phase publishes input/reference; playback-only phase publishes output only.
  void set_phase(AudioEnginePhase phase) noexcept;

  void process_callback(
    const Sample * capture,
    Sample * device_output,
    std::size_t frame_count,
    CallbackStatus status) noexcept;

  [[nodiscard]] std::uint64_t generation() const noexcept;
  [[nodiscard]] AudioMetrics metrics() const noexcept;

private:
  struct PlaybackPacket
  {
    std::uint64_t generation{0U};
    std::uint64_t scope_id{0U};
    std::size_t sample_count{0U};
    std::array<Sample, kFrameSamples> samples{};
  };

  template<typename Item, std::size_t Capacity>
  class SpscRing final
  {
public:
    [[nodiscard]] bool can_push() const noexcept
    {
      const auto write = write_index_.load(std::memory_order_relaxed);
      const auto next = (write + 1U) % Capacity;
      return next != read_index_.load(std::memory_order_acquire);
    }

    [[nodiscard]] bool push(const Item & item) noexcept
    {
      const auto write = write_index_.load(std::memory_order_relaxed);
      const auto next = (write + 1U) % Capacity;
      if (next == read_index_.load(std::memory_order_acquire)) {
        return false;
      }
      items_[write] = item;
      write_index_.store(next, std::memory_order_release);
      return true;
    }

    [[nodiscard]] bool pop(Item & item) noexcept
    {
      const auto read = read_index_.load(std::memory_order_relaxed);
      if (read == write_index_.load(std::memory_order_acquire)) {
        return false;
      }
      item = items_[read];
      read_index_.store((read + 1U) % Capacity, std::memory_order_release);
      return true;
    }

private:
    std::array<Item, Capacity> items_{};
    std::atomic<std::size_t> write_index_{0U};
    std::atomic<std::size_t> read_index_{0U};
  };

  // The callback is the only producer and the worker is the only consumer.
  // Atomic slot contents make overwriting the oldest complete frame free of a
  // C++ data race even when the consumer is preempted while copying a slot.
  class OverwriteAudioFrameRing final
  {
public:
    OverwriteAudioFrameRing() noexcept
    {
      for (auto & slot : slots_) {
        slot.sequence.store(0U, std::memory_order_relaxed);
        slot.generation.store(0U, std::memory_order_relaxed);
        for (auto & sample : slot.samples) {
          sample.store(0, std::memory_order_relaxed);
        }
      }
    }

    void push_drop_oldest(const AudioFrame & frame) noexcept
    {
      const auto sequence = ++producer_sequence_;
      auto & slot = slots_[(sequence - 1U) % kRingCapacity];
      slot.sequence.store((sequence * 2U) - 1U, std::memory_order_release);
      std::atomic_thread_fence(std::memory_order_release);
      slot.generation.store(frame.generation, std::memory_order_relaxed);
      for (std::size_t index = 0U; index < kFrameSamples; ++index) {
        slot.samples[index].store(frame.samples[index], std::memory_order_relaxed);
      }
      slot.sequence.store(sequence * 2U, std::memory_order_release);
      published_sequence_.store(sequence, std::memory_order_release);
    }

    struct PopResult
    {
      bool delivered{false};
      std::uint64_t reported_drops{0U};
    };

    [[nodiscard]] PopResult pop(AudioFrame & frame) noexcept
    {
      const auto published = published_sequence_.load(std::memory_order_acquire);
      auto next = consumer_sequence_ + 1U;
      if (next > published) {
        return PopResult{};
      }
      const auto oldest = published > kRingCapacity ? published - kRingCapacity + 1U : 1U;
      std::uint64_t reported_drops = 0U;
      if (next < oldest) {
        reported_drops = oldest - next;
        next = oldest;
      }

      const auto & slot = slots_[(next - 1U) % kRingCapacity];
      const auto expected_slot_sequence = next * 2U;
      if (slot.sequence.load(std::memory_order_acquire) != expected_slot_sequence) {
        consumer_sequence_ = next;
        return PopResult{false, reported_drops + 1U};
      }
      AudioFrame candidate{};
      candidate.generation = slot.generation.load(std::memory_order_relaxed);
      for (std::size_t index = 0U; index < kFrameSamples; ++index) {
        candidate.samples[index] = slot.samples[index].load(std::memory_order_relaxed);
      }
      std::atomic_thread_fence(std::memory_order_acquire);
      if (slot.sequence.load(std::memory_order_acquire) != expected_slot_sequence) {
        consumer_sequence_ = next;
        return PopResult{false, reported_drops + 1U};
      }
      frame = candidate;
      consumer_sequence_ = next;
      return PopResult{true, reported_drops};
    }

private:
    struct AtomicSlot
    {
      std::atomic<std::uint64_t> sequence{0U};
      std::atomic<std::uint64_t> generation{0U};
      std::array<std::atomic<Sample>, kFrameSamples> samples{};
    };

    std::array<AtomicSlot, kRingCapacity> slots_{};
    std::uint64_t producer_sequence_{0U};
    std::uint64_t consumer_sequence_{0U};
    std::atomic<std::uint64_t> published_sequence_{0U};
  };

  [[nodiscard]] bool pop_playback_for_current_generation(
    PlaybackPacket & packet, std::uint64_t expected_generation) noexcept;
  [[nodiscard]] bool has_pending_discontinuities() const noexcept;
  void commit_pending_discontinuities() noexcept;
  [[nodiscard]] bool render_playback(
    AudioFrame & rendered, std::uint64_t callback_generation,
    PlaybackPacket & rendered_packet) noexcept;
  static Sample saturate(std::int64_t value) noexcept;

  OverwriteAudioFrameRing capture_ring_;
  SpscRing<PlaybackPacket, kRingCapacity> playback_ring_;
  SpscRing<AudioFrame, kRingCapacity> reference_ring_;
  SpscRing<PlaybackWrite, kRingCapacity> playback_write_ring_;
  std::atomic<std::uint64_t> generation_{1U};
  std::atomic<std::uint8_t> phase_{static_cast<std::uint8_t>(AudioEnginePhase::kCapture)};
  std::atomic<std::uint64_t> discontinuity_requests_{0U};
  std::uint64_t observed_discontinuity_requests_{0U};
  std::atomic<std::uint32_t> gain_q15_{32768U};
  std::atomic<std::uint32_t> fade_sample_count_{0U};
  std::atomic<std::uint64_t> fade_request_{0U};
  std::uint64_t observed_fade_request_{0U};
  std::uint32_t fade_remaining_{0U};
  std::uint32_t fade_total_{0U};
  std::atomic<std::uint64_t> capture_overflows_{0U};
  std::atomic<std::uint64_t> capture_produced_{0U};
  std::atomic<std::uint64_t> capture_delivered_{0U};
  std::atomic<std::uint64_t> capture_reported_drops_{0U};
  std::atomic<std::uint64_t> playback_overflows_{0U};
  std::atomic<std::uint64_t> reference_overflows_{0U};
  std::atomic<std::uint64_t> playback_underflows_{0U};
  std::atomic<std::uint64_t> xruns_{0U};
  std::atomic<std::uint64_t> discontinuities_{0U};
  std::atomic<std::uint64_t> last_fence_generation_before_{0U};
  std::atomic<std::uint64_t> last_fence_generation_after_{0U};
  std::atomic<std::uint64_t> stale_pcm_after_fence_{0U};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__AUDIO_ENGINE_HPP_
