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

#ifndef VOICE_NAV_MISSION__RUNTIME_TRANSACTION_PLANE_HPP_
#define VOICE_NAV_MISSION__RUNTIME_TRANSACTION_PLANE_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <type_traits>
#include <utility>

namespace voice_nav_mission
{

enum class RuntimeTransactionSideEffect : std::uint8_t
{
  Prepare = 0,
  Open = 1,
  ControllerStart = 2,
};

// A package-private transaction Module.  A side effect gets a two-phase
// generation permit: the transaction worker may be paused before commit, but
// the final permit check and operation admission execute under this Module's
// short lock.  Quiesce closes the generation before waiting for the operation
// state to become idle; the wait never holds the Node or authority mutex.
class RuntimeTransactionPlane final
{
public:
  using BeforeCommit = std::function<void(RuntimeTransactionSideEffect)>;
  using BeforeOperation = std::function<void(RuntimeTransactionSideEffect)>;
  using QuiesceObserver = std::function<void(std::uint64_t)>;

  explicit RuntimeTransactionPlane(
    const std::uint64_t initial_generation = 0U,
    BeforeCommit before_commit = {},
    BeforeOperation before_operation = {},
    QuiesceObserver quiesce_observer = {})
  : generation_(initial_generation),
    before_commit_(std::move(before_commit)),
    before_operation_(std::move(before_operation)),
    quiesce_observer_(std::move(quiesce_observer))
  {
  }

  RuntimeTransactionPlane(const RuntimeTransactionPlane &) = delete;
  RuntimeTransactionPlane & operator=(const RuntimeTransactionPlane &) = delete;

  template<typename Operation>
  [[nodiscard]] auto submit(
    const std::uint64_t permit_generation,
    const RuntimeTransactionSideEffect side_effect,
    Operation && operation)
  -> std::optional<std::invoke_result_t<Operation>>
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (quiescing_ || permit_generation != generation_ ||
        phase_ == Phase::Pending || phase_ == Phase::InFlight)
      {
        return std::nullopt;
      }
      phase_ = Phase::Pending;
    }

    try {
      if (before_commit_) {
        before_commit_(side_effect);
      }
    } catch (...) {
      set_phase(Phase::Rejected);
      throw;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (quiescing_ || permit_generation != generation_) {
        phase_ = Phase::Rejected;
        condition_.notify_all();
        return std::nullopt;
      }
      phase_ = Phase::InFlight;
    }
    try {
      if (before_operation_) {
        before_operation_(side_effect);
      }
      auto result = std::forward<Operation>(operation)();
      set_phase(Phase::Finished);
      return result;
    } catch (...) {
      set_phase(Phase::Rejected);
      throw;
    }
  }

  // The caller first closes admission in its own short critical section, then
  // calls this method.  A pending permit is rejected at its final check and an
  // admitted operation is allowed to return before a successful barrier.
  [[nodiscard]] bool quiesce(
    const std::uint64_t next_generation,
    const std::chrono::steady_clock::time_point deadline)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      quiescing_ = true;
      generation_ = next_generation;
    }
    if (quiesce_observer_) {
      quiesce_observer_(next_generation);
    }
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_until(lock, deadline, [this]() {
               return phase_ != Phase::Pending && phase_ != Phase::InFlight;
    });
  }

  [[nodiscard]] bool quiescing() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return quiescing_;
  }

  [[nodiscard]] std::uint64_t generation() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }

private:
  enum class Phase : std::uint8_t
  {
    Idle = 0,
    Pending = 1,
    InFlight = 2,
    Finished = 3,
    Rejected = 4,
  };

  void set_phase(const Phase phase) noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      phase_ = phase;
    }
    condition_.notify_all();
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::uint64_t generation_{0U};
  bool quiescing_{false};
  Phase phase_{Phase::Idle};
  BeforeCommit before_commit_;
  BeforeOperation before_operation_;
  QuiesceObserver quiesce_observer_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_TRANSACTION_PLANE_HPP_
