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
// short lock.  An already-committed RPC is tracked independently so quiesce
// never waits on its bounded call and never holds the Node admission mutex.
class RuntimeTransactionPlane final
{
public:
  using BeforeCommit = std::function<void(RuntimeTransactionSideEffect)>;

  explicit RuntimeTransactionPlane(
    const std::uint64_t initial_generation = 0U,
    BeforeCommit before_commit = {})
  : generation_(initial_generation),
    before_commit_(std::move(before_commit))
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
      if (quiescing_ || permit_generation != generation_ || pending_) {
        return std::nullopt;
      }
      pending_ = true;
    }

    try {
      if (before_commit_) {
        before_commit_(side_effect);
      }
    } catch (...) {
      clear_pending();
      throw;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (quiescing_ || permit_generation != generation_) {
        pending_ = false;
        return std::nullopt;
      }
      pending_ = false;
      in_flight_ = true;
    }
    try {
      auto result = std::forward<Operation>(operation)();
      clear_in_flight();
      return result;
    } catch (...) {
      clear_in_flight();
      throw;
    }
  }

  // The caller first closes admission in its own short critical section, then
  // calls this method.  An already-committed RPC remains fenced by its
  // generation/request checks, while this barrier closes new side effects
  // without waiting for the bounded RPC.
  void quiesce(const std::uint64_t next_generation) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    quiescing_ = true;
    generation_ = next_generation;
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
  void clear_pending() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = false;
  }

  void clear_in_flight() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    in_flight_ = false;
  }

  mutable std::mutex mutex_;
  std::uint64_t generation_{0U};
  bool quiescing_{false};
  bool pending_{false};
  bool in_flight_{false};
  BeforeCommit before_commit_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_TRANSACTION_PLANE_HPP_
