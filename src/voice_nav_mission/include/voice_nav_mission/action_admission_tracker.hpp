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

#ifndef VOICE_NAV_MISSION__ACTION_ADMISSION_TRACKER_HPP_
#define VOICE_NAV_MISSION__ACTION_ADMISSION_TRACKER_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace voice_nav_mission
{

// Package-private handoff Module for the two callbacks owned by an
// rclcpp_action Server.  The shared state outlives the Node's tracker object
// while an on_accepted lease is still in flight, so a callback can complete
// safely during bounded shutdown without dereferencing a destroyed tracker.
class ActionAdmissionTracker final
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;
  using Duration = std::chrono::steady_clock::duration;
  using Clock = std::function<TimePoint()>;

  static constexpr std::size_t kCapacity = 64U;
  static constexpr auto kDefaultHandoffDeadline = std::chrono::milliseconds(100);

  struct Snapshot
  {
    std::size_t provisional{0U};
    std::size_t revoked{0U};
    std::size_t in_flight{0U};
    std::size_t callbacks_inflight{0U};
    bool quiescing{false};
  };

  struct SharedState final
  {
    enum class EntryState
    {
      Provisional,
      Revoked,
      AcceptedInFlight
    };

    struct Entry
    {
      EntryState state;
      TimePoint deadline;
      TimePoint revoked_at;
    };

    SharedState(Clock value_clock, const Duration value_deadline)
    : clock(std::move(value_clock)),
      handoff_deadline(value_deadline)
    {
    }

    Clock clock;
    Duration handoff_deadline;
    mutable std::mutex mutex;
    std::condition_variable condition;
    std::unordered_map<std::string, Entry> entries;
    std::size_t provisional{0U};
    std::size_t revoked{0U};
    std::size_t in_flight{0U};
    std::size_t callbacks_inflight{0U};
    bool quiescing{false};
  };

  class CallbackLease final
  {
public:
    CallbackLease() = default;

    ~CallbackLease()
    {
      release();
    }

    CallbackLease(const CallbackLease &) = delete;
    CallbackLease & operator=(const CallbackLease &) = delete;

    CallbackLease(CallbackLease && other) noexcept
    : state_(std::exchange(other.state_, {})),
      uuid_(std::move(other.uuid_)),
      ticket_claimed_(std::exchange(other.ticket_claimed_, false)),
      revoked_(std::exchange(other.revoked_, false))
    {
    }

    CallbackLease & operator=(CallbackLease && other) noexcept
    {
      if (this != &other) {
        release();
        state_ = std::exchange(other.state_, {});
        uuid_ = std::move(other.uuid_);
        ticket_claimed_ = std::exchange(other.ticket_claimed_, false);
        revoked_ = std::exchange(other.revoked_, false);
      }
      return *this;
    }

    [[nodiscard]] bool has_ticket() const noexcept
    {
      return ticket_claimed_;
    }

    [[nodiscard]] bool was_revoked() const noexcept
    {
      return revoked_;
    }

private:
    friend class ActionAdmissionTracker;

    CallbackLease(
      std::shared_ptr<SharedState> state,
      std::string uuid,
      const bool ticket_claimed,
      const bool revoked)
    : state_(std::move(state)),
      uuid_(std::move(uuid)),
      ticket_claimed_(ticket_claimed),
      revoked_(revoked)
    {
    }

    void release() noexcept
    {
      if (state_) {
        ActionAdmissionTracker::complete_callback(
          state_, uuid_, ticket_claimed_);
        state_.reset();
      }
    }

    std::shared_ptr<SharedState> state_;
    std::string uuid_;
    bool ticket_claimed_{false};
    bool revoked_{false};
  };

  explicit ActionAdmissionTracker(
    Clock clock,
    const Duration handoff_deadline = kDefaultHandoffDeadline)
  {
    if (!clock || handoff_deadline <= Duration::zero()) {
      throw std::invalid_argument("Action admission tracker requires a clock and deadline");
    }
    state_ = std::make_shared<SharedState>(std::move(clock), handoff_deadline);
  }

  [[nodiscard]] std::shared_ptr<SharedState> shared_state() const noexcept
  {
    return state_;
  }

  [[nodiscard]] bool try_provision(const std::string & uuid)
  {
    const auto state = state_;
    const auto now = state->clock();
    std::lock_guard<std::mutex> lock(state->mutex);
    prune_revoked_locked(*state, now);
    if (state->quiescing || uuid.empty() || state->entries.size() >= kCapacity ||
      state->entries.find(uuid) != state->entries.end())
    {
      return false;
    }
    state->entries.emplace(
      uuid,
      SharedState::Entry{
        SharedState::EntryState::Provisional, now + state->handoff_deadline, {}});
    ++state->provisional;
    state->condition.notify_all();
    return true;
  }

  [[nodiscard]] CallbackLease enter_accepted(const std::string & uuid)
  {
    const auto state = state_;
    std::lock_guard<std::mutex> lock(state->mutex);
    ++state->callbacks_inflight;
    const auto found = state->entries.find(uuid);
    if (found == state->entries.end()) {
      return CallbackLease(state, uuid, false, false);
    }
    if (found->second.state != SharedState::EntryState::Provisional &&
      found->second.state != SharedState::EntryState::Revoked)
    {
      return CallbackLease(state, uuid, false, false);
    }
    const auto was_revoked = found->second.state == SharedState::EntryState::Revoked;
    if (!was_revoked && state->provisional > 0U) {
      --state->provisional;
    }
    if (was_revoked && state->revoked > 0U) {
      --state->revoked;
    }
    found->second.state = SharedState::EntryState::AcceptedInFlight;
    found->second.revoked_at = {};
    ++state->in_flight;
    state->condition.notify_all();
    return CallbackLease(state, uuid, true, was_revoked);
  }

  void begin_quiesce() noexcept
  {
    const auto state = state_;
    std::lock_guard<std::mutex> lock(state->mutex);
    state->quiescing = true;
    state->condition.notify_all();
  }

  [[nodiscard]] std::size_t revoke_expired(const TimePoint now)
  {
    const auto state = state_;
    std::lock_guard<std::mutex> lock(state->mutex);
    const auto revoked = revoke_provisional_locked(*state, now, false);
    prune_revoked_locked(*state, now);
    if (revoked > 0U) {
      state->condition.notify_all();
    }
    return revoked;
  }

  [[nodiscard]] std::size_t revoke_all_provisional(const TimePoint now)
  {
    const auto state = state_;
    std::lock_guard<std::mutex> lock(state->mutex);
    const auto revoked = revoke_provisional_locked(*state, now, true);
    if (revoked > 0U) {
      state->condition.notify_all();
    }
    return revoked;
  }

  [[nodiscard]] Snapshot snapshot() const noexcept
  {
    const auto state = state_;
    std::lock_guard<std::mutex> lock(state->mutex);
    return Snapshot{
      state->provisional, state->revoked, state->in_flight,
      state->callbacks_inflight, state->quiescing};
  }

  [[nodiscard]] bool drained() const noexcept
  {
    const auto state = state_;
    std::lock_guard<std::mutex> lock(state->mutex);
    return state->provisional == 0U && state->revoked == 0U &&
           state->in_flight == 0U && state->callbacks_inflight == 0U;
  }

  [[nodiscard]] bool wait_for_drain_until(
    const std::chrono::steady_clock::time_point deadline)
  {
    const auto state = state_;
    std::unique_lock<std::mutex> lock(state->mutex);
    prune_revoked_locked(*state, state->clock());
    return state->condition.wait_until(lock, deadline, [state]() {
               return state->provisional == 0U && state->revoked == 0U &&
                      state->in_flight == 0U && state->callbacks_inflight == 0U;
           });
  }

  void clear() noexcept
  {
    const auto state = state_;
    std::lock_guard<std::mutex> lock(state->mutex);
    state->entries.clear();
    state->provisional = 0U;
    state->revoked = 0U;
    state->in_flight = 0U;
    state->callbacks_inflight = 0U;
    state->condition.notify_all();
  }

private:
  [[nodiscard]] static std::size_t revoke_provisional_locked(
    SharedState & state,
    const TimePoint now,
    const bool force)
  {
    std::size_t count = 0U;
    for (auto & entry : state.entries) {
      if (entry.second.state == SharedState::EntryState::Provisional &&
        (force || now >= entry.second.deadline))
      {
        entry.second.state = SharedState::EntryState::Revoked;
        entry.second.revoked_at = now;
        if (state.provisional > 0U) {
          --state.provisional;
        }
        ++state.revoked;
        ++count;
      }
    }
    return count;
  }

  static void prune_revoked_locked(SharedState & state, const TimePoint now)
  {
    const auto retention = state.handoff_deadline * 4;
    for (auto iterator = state.entries.begin(); iterator != state.entries.end(); ) {
      if (iterator->second.state == SharedState::EntryState::Revoked &&
        now >= iterator->second.revoked_at + retention)
      {
        iterator = state.entries.erase(iterator);
        if (state.revoked > 0U) {
          --state.revoked;
        }
      } else {
        ++iterator;
      }
    }
  }

  static void complete_callback(
    const std::shared_ptr<SharedState> & state,
    const std::string & uuid,
    const bool ticket_claimed) noexcept
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (ticket_claimed) {
      const auto found = state->entries.find(uuid);
      if (found != state->entries.end()) {
        state->entries.erase(found);
        if (state->in_flight > 0U) {
          --state->in_flight;
        }
      }
    }
    if (state->callbacks_inflight > 0U) {
      --state->callbacks_inflight;
    }
    state->condition.notify_all();
  }

  std::shared_ptr<SharedState> state_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__ACTION_ADMISSION_TRACKER_HPP_
