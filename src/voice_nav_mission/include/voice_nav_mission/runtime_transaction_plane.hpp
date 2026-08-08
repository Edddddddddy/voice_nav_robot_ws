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
  Renew = 3,
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
  using BeforeFinish = std::function<void(RuntimeTransactionSideEffect)>;

  class Lease final
  {
public:
    Lease() = default;

    Lease(const Lease &) = delete;
    Lease & operator=(const Lease &) = delete;

    Lease(Lease && other) noexcept
    : plane_(other.plane_),
      generation_(other.generation_),
      side_effect_(other.side_effect_),
      active_(other.active_),
      finish_started_(other.finish_started_)
    {
      other.plane_ = nullptr;
      other.active_ = false;
    }

    Lease & operator=(Lease && other) noexcept
    {
      if (this != &other) {
        reject();
        plane_ = other.plane_;
        generation_ = other.generation_;
        side_effect_ = other.side_effect_;
        active_ = other.active_;
        finish_started_ = other.finish_started_;
        other.plane_ = nullptr;
        other.active_ = false;
      }
      return *this;
    }

    ~Lease()
    {
      reject();
    }

    [[nodiscard]] bool current() const noexcept
    {
      return active_ && !finish_started_ && plane_ && plane_->lease_current(generation_);
    }

    template<typename Operation>
    [[nodiscard]] auto invoke(
      const RuntimeTransactionSideEffect side_effect,
      Operation && operation)
    -> std::optional<std::invoke_result_t<Operation>>
    {
      if (!current()) {
        return std::nullopt;
      }
      if (plane_->before_operation_) {
        plane_->before_operation_(side_effect);
      }
      // Once the lease has passed the pre-operation hook, the underlying
      // operation is admitted.  It may return after quiesce closes the
      // generation; the caller still owns this lease and must perform its
      // guarded rollback or terminal handling before the lease is released.
      return std::forward<Operation>(operation)();
    }

    template<typename FinalCommit>
    [[nodiscard]] bool commit(FinalCommit && final_commit)
    {
      if (!active_ || !plane_ || finish_started_ || !current()) {
        return false;
      }
      finish_started_ = true;
      // This hook is deliberately outside the plane mutex.  It is a
      // deterministic tail barrier for package tests and also preserves the
      // short-lock rule for production final state commits.
      if (plane_->before_finish_) {
        plane_->before_finish_(side_effect_);
      }
      std::lock_guard<std::mutex> lock(plane_->mutex_);
      if (!active_ || plane_->quiescing_ || generation_ != plane_->generation_ ||
        plane_->phase_ != Phase::InFlight)
      {
        // Keep the lease InFlight until the caller has completed its
        // fail-closed rollback/zero cleanup.  Its destructor (or reject())
        // is the release point that lets quiesce return.
        return false;
      }
      if (!static_cast<bool>(std::forward<FinalCommit>(final_commit)())) {
        // The guarded outcome failed, but cleanup still belongs to this
        // transaction lease.  Do not wake quiesce before that cleanup.
        return false;
      }
      plane_->phase_ = Phase::Finished;
      active_ = false;
      plane_->condition_.notify_all();
      return true;
    }

    void reject() noexcept
    {
      if (!active_ || !plane_) {
        active_ = false;
        return;
      }
      bool changed = false;
      {
        std::lock_guard<std::mutex> lock(plane_->mutex_);
        if (plane_->phase_ == Phase::Pending || plane_->phase_ == Phase::InFlight) {
          plane_->phase_ = Phase::Rejected;
          changed = true;
        }
        active_ = false;
      }
      if (changed) {
        plane_->condition_.notify_all();
      }
    }

private:
    friend class RuntimeTransactionPlane;

    Lease(
      RuntimeTransactionPlane * plane,
      const std::uint64_t generation,
      const RuntimeTransactionSideEffect side_effect)
    : plane_(plane), generation_(generation), side_effect_(side_effect), active_(true)
    {
    }

    RuntimeTransactionPlane * plane_{nullptr};
    std::uint64_t generation_{0U};
    RuntimeTransactionSideEffect side_effect_{RuntimeTransactionSideEffect::Prepare};
    bool active_{false};
    bool finish_started_{false};
  };

  explicit RuntimeTransactionPlane(
    const std::uint64_t initial_generation = 0U,
    BeforeCommit before_commit = {},
    BeforeOperation before_operation = {},
    QuiesceObserver quiesce_observer = {},
    BeforeFinish before_finish = {})
  : generation_(initial_generation),
    before_commit_(std::move(before_commit)),
    before_operation_(std::move(before_operation)),
    quiesce_observer_(std::move(quiesce_observer)),
    before_finish_(std::move(before_finish))
  {
  }

  RuntimeTransactionPlane(const RuntimeTransactionPlane &) = delete;
  RuntimeTransactionPlane & operator=(const RuntimeTransactionPlane &) = delete;

  [[nodiscard]] std::optional<Lease> begin(
    const std::uint64_t permit_generation,
    const RuntimeTransactionSideEffect side_effect)
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
    Lease lease(this, permit_generation, side_effect);
    return std::optional<Lease>(std::move(lease));
  }

  template<typename Operation>
  [[nodiscard]] auto submit(
    const std::uint64_t permit_generation,
    const RuntimeTransactionSideEffect side_effect,
    Operation && operation)
  -> std::optional<std::invoke_result_t<Operation>>
  {
    auto lease = begin(permit_generation, side_effect);
    if (!lease.has_value()) {
      return std::nullopt;
    }
    auto result = lease->invoke(side_effect, std::forward<Operation>(operation));
    if (!result.has_value()) {
      return std::nullopt;
    }
    // Preserve the historical submit result for an in-flight RPC that was
    // allowed to return while quiesce was closing the generation.  The lease
    // remains active until this guarded outcome is attempted and its
    // destructor releases the rejected tail.
    (void)lease->commit([]() {return true;});
    return result;
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

  [[nodiscard]] bool lease_current(const std::uint64_t generation) const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return !quiescing_ && generation == generation_ && phase_ == Phase::InFlight;
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::uint64_t generation_{0U};
  bool quiescing_{false};
  Phase phase_{Phase::Idle};
  BeforeCommit before_commit_;
  BeforeOperation before_operation_;
  QuiesceObserver quiesce_observer_;
  BeforeFinish before_finish_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_TRANSACTION_PLANE_HPP_
