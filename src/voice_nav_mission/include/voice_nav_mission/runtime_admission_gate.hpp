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

#ifndef VOICE_NAV_MISSION__RUNTIME_ADMISSION_GATE_HPP_
#define VOICE_NAV_MISSION__RUNTIME_ADMISSION_GATE_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "voice_nav_mission/action_admission_tracker.hpp"
#include "voice_nav_mission/runtime_transaction_plane.hpp"

namespace voice_nav_mission
{

// Node-owned linearization Module for Action admission and runtime start.
// Queue insertion is performed while this gate is held; a queued admission
// must claim a generation-bound permit again before Core or Adapter side
// effects.  Quiesce invalidates both admission and all outstanding permits.
class RuntimeAdmissionGate final
{
public:
  struct StartPermit
  {
    std::uint64_t generation{0U};
    std::uint64_t admission_epoch{0U};
    bool issued{false};
  };

  using AdmissionCheck = std::function<bool(std::uint64_t)>;
  using TimePoint = std::chrono::steady_clock::time_point;

  RuntimeAdmissionGate()
  : transaction_plane_(std::make_shared<RuntimeTransactionPlane>(1U))
  {
  }

  RuntimeAdmissionGate(const RuntimeAdmissionGate &) = delete;
  RuntimeAdmissionGate & operator=(const RuntimeAdmissionGate &) = delete;

  [[nodiscard]] bool try_provision(
    ActionAdmissionTracker & tracker,
    const std::string & uuid,
    const std::uint64_t admission_epoch,
    const AdmissionCheck & admission_check)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (quiescing_ || (admission_check && !admission_check(admission_epoch))) {
      return false;
    }
    return tracker.try_provision(uuid);
  }

  template<typename Enqueue>
  [[nodiscard]] bool submit(Enqueue && enqueue) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (quiescing_) {
      return false;
    }
    try {
      return static_cast<bool>(enqueue());
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool begin_quiesce(
    ActionAdmissionTracker & tracker,
    const TimePoint deadline)
  {
    if (!close_generation(tracker)) {
      return false;
    }
    return wait_for_transaction_drain(deadline);
  }

  // First shutdown phase: reject new admission and rotate the generation.
  // This operation does not wait for a transaction or any ROS operation.
  [[nodiscard]] bool close_generation(ActionAdmissionTracker & tracker) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (quiescing_) {
      return true;
    }
    quiescing_ = true;
    if (generation_ != std::numeric_limits<std::uint64_t>::max()) {
      ++generation_;
    }
    const auto next_generation = generation_;
    tracker.begin_quiesce();
    try {
      // This is a non-blocking linearization step.  Keep it under the same
      // gate mutex so no second caller can observe the admission generation
      // closed while the transaction plane still accepts the old generation.
      if (transaction_plane_) {
        transaction_plane_->close_generation(next_generation);
      }
    } catch (...) {
      quiesce_complete_ = true;
      quiesce_succeeded_ = false;
      return false;
    }
    return true;
  }

  // Second shutdown phase: wait without holding the gate mutex, authority
  // mutex, or any Node-owned transaction lock.
  [[nodiscard]] bool wait_for_transaction_drain(const TimePoint deadline) noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!quiescing_) {
        return false;
      }
      if (quiesce_complete_) {
        return quiesce_succeeded_;
      }
    }
    bool succeeded = true;
    try {
      if (transaction_plane_) {
        succeeded = transaction_plane_->wait_for_drain_until(deadline);
      }
    } catch (...) {
      succeeded = false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      quiesce_complete_ = true;
      quiesce_succeeded_ = succeeded;
    }
    return succeeded;
  }

  [[nodiscard]] StartPermit claim_start(
    const std::uint64_t admission_epoch) const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (quiescing_) {
      return {};
    }
    return StartPermit{generation_, admission_epoch, true};
  }

  [[nodiscard]] bool start_allowed(const StartPermit & permit) const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return permit.issued && !quiescing_ && permit.generation == generation_;
  }

  [[nodiscard]] bool admission_allowed(
    const std::uint64_t admission_epoch,
    const AdmissionCheck & admission_check = {}) const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (quiescing_ || (admission_check && !admission_check(admission_epoch))) {
      return false;
    }
    return true;
  }

  [[nodiscard]] bool quiescing() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return quiescing_;
  }

  [[nodiscard]] std::shared_ptr<RuntimeTransactionPlane> transaction_plane()
  const noexcept
  {
    return transaction_plane_;
  }

private:
  mutable std::mutex mutex_;
  std::uint64_t generation_{1U};
  bool quiescing_{false};
  bool quiesce_complete_{false};
  bool quiesce_succeeded_{true};
  std::shared_ptr<RuntimeTransactionPlane> transaction_plane_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_ADMISSION_GATE_HPP_
