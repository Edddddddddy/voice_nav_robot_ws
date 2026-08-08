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

  void begin_quiesce(ActionAdmissionTracker & tracker) noexcept
  {
    std::uint64_t next_generation = 0U;
    bool should_quiesce = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!quiescing_) {
        quiescing_ = true;
        if (generation_ != std::numeric_limits<std::uint64_t>::max()) {
          ++generation_;
        }
        next_generation = generation_;
        tracker.begin_quiesce();
        should_quiesce = true;
      }
    }
    // The transaction plane fences new side effects without waiting for an
    // already-committed RPC, and the Node admission mutex is not held here.
    if (should_quiesce && transaction_plane_) {
      transaction_plane_->quiesce(next_generation);
    }
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
  std::shared_ptr<RuntimeTransactionPlane> transaction_plane_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_ADMISSION_GATE_HPP_
