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

#ifndef VOICE_NAV_AUDIO__SPSC_AUDIO_RING_HPP_
#define VOICE_NAV_AUDIO__SPSC_AUDIO_RING_HPP_

#include <array>
#include <atomic>
#include <cstddef>

namespace voice_nav_audio
{

template<std::size_t Capacity>
class SpscAudioRing
{
  static_assert(Capacity >= 2U);

public:
  bool push(const float sample) noexcept
  {
    const auto head = head_.load(std::memory_order_relaxed);
    const auto next = increment(head);
    if (next == tail_.load(std::memory_order_acquire)) {
      return false;
    }
    samples_[head] = sample;
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(float & sample) noexcept
  {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return false;
    }
    sample = samples_[tail];
    tail_.store(increment(tail), std::memory_order_release);
    return true;
  }

  void clear() noexcept
  {
    tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
  }

  std::size_t write_available() const noexcept
  {
    const auto head = head_.load(std::memory_order_relaxed);
    const auto tail = tail_.load(std::memory_order_acquire);
    const auto used = head >= tail ? head - tail : Capacity - (tail - head);
    return Capacity - 1U - used;
  }

private:
  static constexpr std::size_t increment(const std::size_t value) noexcept
  {
    return (value + 1U) % Capacity;
  }

  std::array<float, Capacity> samples_{};
  alignas(64) std::atomic<std::size_t> head_{0U};
  alignas(64) std::atomic<std::size_t> tail_{0U};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__SPSC_AUDIO_RING_HPP_
