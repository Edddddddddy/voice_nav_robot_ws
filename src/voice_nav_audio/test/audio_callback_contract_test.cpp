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

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>

#include "gtest/gtest.h"
#include "voice_nav_audio/portaudio_adapter.hpp"

namespace
{
std::atomic<std::size_t> allocation_calls{0U};
std::atomic<bool> observe_allocations{false};

void record_allocation_call() noexcept
{
  if (observe_allocations.load(std::memory_order_relaxed)) {
    allocation_calls.fetch_add(1U, std::memory_order_relaxed);
  }
}

void * allocate(const std::size_t size)
{
  if (void * const memory = std::malloc(size)) {
    return memory;
  }
  throw std::bad_alloc();
}

void * allocate_aligned(const std::size_t size, const std::size_t alignment)
{
  const auto rounded_size = ((size + alignment - 1U) / alignment) * alignment;
  if (void * const memory = std::aligned_alloc(alignment, rounded_size)) {
    return memory;
  }
  throw std::bad_alloc();
}
}

void * operator new(const std::size_t size)
{
  record_allocation_call();
  return allocate(size);
}

void * operator new[](const std::size_t size)
{
  record_allocation_call();
  return allocate(size);
}

void * operator new(const std::size_t size, const std::align_val_t alignment)
{
  record_allocation_call();
  return allocate_aligned(size, static_cast<std::size_t>(alignment));
}

void * operator new[](const std::size_t size, const std::align_val_t alignment)
{
  record_allocation_call();
  return allocate_aligned(size, static_cast<std::size_t>(alignment));
}

void * operator new(const std::size_t size, const std::nothrow_t &) noexcept
{
  record_allocation_call();
  return std::malloc(size);
}

void * operator new[](const std::size_t size, const std::nothrow_t &) noexcept
{
  record_allocation_call();
  return std::malloc(size);
}

void operator delete(void * const memory) noexcept
{
  record_allocation_call();
  std::free(memory);
}

void operator delete(void * const memory, const std::size_t) noexcept
{
  record_allocation_call();
  std::free(memory);
}

void operator delete[](void * const memory) noexcept
{
  record_allocation_call();
  std::free(memory);
}

void operator delete[](void * const memory, const std::size_t) noexcept
{
  record_allocation_call();
  std::free(memory);
}

void operator delete(void * const memory, const std::align_val_t) noexcept
{
  record_allocation_call();
  std::free(memory);
}

void operator delete[](void * const memory, const std::align_val_t) noexcept
{
  record_allocation_call();
  std::free(memory);
}

void operator delete(void * const memory, const std::size_t, const std::align_val_t) noexcept
{
  record_allocation_call();
  std::free(memory);
}

void operator delete[](void * const memory, const std::size_t, const std::align_val_t) noexcept
{
  record_allocation_call();
  std::free(memory);
}

namespace voice_nav_audio
{
namespace
{

class CallbackDevice final : public FullDuplexAudioDevice
{
public:
  bool open(
    const FullDuplexStreamSpec,
    const DeviceCallback callback,
    void * const context) noexcept override
  {
    callback_ = callback;
    context_ = context;
    return true;
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
    ASSERT_NE(callback_, nullptr);
    callback_(context_, input.data(), output.data(), output.size(), CallbackStatus{});
  }

private:
  DeviceCallback callback_{nullptr};
  void * context_{nullptr};
};

TEST(AudioCallbackContractTest, ActualAdapterCallbackPerformsNoDynamicAllocationOrFree)
{
  AudioEngine engine;
  CallbackDevice device;
  PortAudioAdapter adapter(engine, device);
  std::array<Sample, AudioEngine::kFrameSamples> input{};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  ASSERT_EQ(adapter.start(), AdapterStartResult::Started);
  allocation_calls.store(0U, std::memory_order_relaxed);
  observe_allocations.store(true, std::memory_order_release);
  device.fire(input, output);
  observe_allocations.store(false, std::memory_order_release);

  EXPECT_EQ(allocation_calls.load(std::memory_order_relaxed), 0U);
}

}  // namespace
}  // namespace voice_nav_audio
