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

#ifndef VOICE_NAV_MISSION__RUNTIME_EMERGENCY_FENCE_HPP_
#define VOICE_NAV_MISSION__RUNTIME_EMERGENCY_FENCE_HPP_

#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace voice_nav_mission
{

struct RuntimeEmergencyFenceSnapshot
{
  std::uint64_t admission_epoch{0U};
  std::string detail;
};

class RuntimeEmergencyFence final
{
public:
  explicit RuntimeEmergencyFence(std::uint64_t initial_admission_epoch)
  : admission_epoch_(initial_admission_epoch)
  {
  }

  [[nodiscard]] bool raise(std::string detail) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (blocked_.load(std::memory_order_acquire)) {
      return false;
    }
    const auto current = admission_epoch_.load(std::memory_order_relaxed);
    if (current != std::numeric_limits<std::uint64_t>::max()) {
      admission_epoch_.store(current + 1U, std::memory_order_release);
    }
    blocked_.store(true, std::memory_order_release);
    try {
      detail_ = std::move(detail);
    } catch (...) {
      detail_.clear();
    }
    pending_.store(true, std::memory_order_release);
    return true;
  }

  [[nodiscard]] bool pending() const noexcept
  {
    return pending_.load(std::memory_order_acquire);
  }

  [[nodiscard]] bool blocked() const noexcept
  {
    return blocked_.load(std::memory_order_acquire);
  }

  [[nodiscard]] std::uint64_t admission_epoch() const noexcept
  {
    return admission_epoch_.load(std::memory_order_acquire);
  }

  [[nodiscard]] std::optional<RuntimeEmergencyFenceSnapshot> take()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.load(std::memory_order_acquire)) {
      return std::nullopt;
    }
    RuntimeEmergencyFenceSnapshot snapshot{
      admission_epoch_.load(std::memory_order_acquire), detail_};
    detail_.clear();
    pending_.store(false, std::memory_order_release);
    return snapshot;
  }

private:
  mutable std::mutex mutex_;
  std::atomic<std::uint64_t> admission_epoch_;
  std::atomic<bool> blocked_{false};
  std::atomic<bool> pending_{false};
  std::string detail_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_EMERGENCY_FENCE_HPP_
