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
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace voice_nav_mission
{

// Package-private handoff Module for the two callbacks owned by an
// rclcpp_action Server.  A provisional ticket expires independently of the
// application Result path; once on_accepted enters, CallbackLease owns the
// only accepted/in-flight transition and completes it exactly once.
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
    : tracker_(std::exchange(other.tracker_, nullptr)),
      uuid_(std::move(other.uuid_)),
      ticket_claimed_(std::exchange(other.ticket_claimed_, false)),
      revoked_(std::exchange(other.revoked_, false))
    {
    }

    CallbackLease & operator=(CallbackLease && other) noexcept
    {
      if (this != &other) {
        release();
        tracker_ = std::exchange(other.tracker_, nullptr);
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
      ActionAdmissionTracker * tracker,
      std::string uuid,
      const bool ticket_claimed,
      const bool revoked)
    : tracker_(tracker),
      uuid_(std::move(uuid)),
      ticket_claimed_(ticket_claimed),
      revoked_(revoked)
    {
    }

    void release() noexcept
    {
      if (tracker_) {
        tracker_->complete_callback(uuid_, ticket_claimed_);
        tracker_ = nullptr;
      }
    }

    ActionAdmissionTracker * tracker_{nullptr};
    std::string uuid_;
    bool ticket_claimed_{false};
    bool revoked_{false};
  };

  explicit ActionAdmissionTracker(
    Clock clock,
    const Duration handoff_deadline = kDefaultHandoffDeadline)
  : clock_(std::move(clock)),
    handoff_deadline_(handoff_deadline)
  {
    if (!clock_ || handoff_deadline_ <= Duration::zero()) {
      throw std::invalid_argument("Action admission tracker requires a clock and deadline");
    }
  }

  [[nodiscard]] bool try_provision(const std::string & uuid)
  {
    const auto now = clock_();
    std::lock_guard<std::mutex> lock(mutex_);
    prune_revoked_locked(now);
    if (quiescing_ || uuid.empty() || entries_.size() >= kCapacity ||
      entries_.find(uuid) != entries_.end())
    {
      return false;
    }
    entries_.emplace(uuid, Entry{State::Provisional, now + handoff_deadline_, {}});
    ++provisional_;
    condition_.notify_all();
    return true;
  }

  [[nodiscard]] CallbackLease enter_accepted(const std::string & uuid)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++callbacks_inflight_;
    const auto found = entries_.find(uuid);
    if (found == entries_.end()) {
      return CallbackLease(this, uuid, false, false);
    }
    if (found->second.state != State::Provisional &&
      found->second.state != State::Revoked)
    {
      return CallbackLease(this, uuid, false, false);
    }
    const auto was_revoked = found->second.state == State::Revoked;
    if (!was_revoked && provisional_ > 0U) {
      --provisional_;
    }
    if (was_revoked && revoked_ > 0U) {
      --revoked_;
    }
    found->second.state = State::AcceptedInFlight;
    found->second.revoked_at = {};
    ++in_flight_;
    condition_.notify_all();
    return CallbackLease(this, uuid, true, was_revoked);
  }

  void begin_quiesce() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    quiescing_ = true;
    condition_.notify_all();
  }

  [[nodiscard]] std::size_t revoke_expired(const TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto revoked = revoke_provisional_locked(now, false);
    prune_revoked_locked(now);
    if (revoked > 0U) {
      condition_.notify_all();
    }
    return revoked;
  }

  [[nodiscard]] std::size_t revoke_all_provisional(const TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto revoked = revoke_provisional_locked(now, true);
    if (revoked > 0U) {
      condition_.notify_all();
    }
    return revoked;
  }

  [[nodiscard]] Snapshot snapshot() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return Snapshot{provisional_, revoked_, in_flight_, callbacks_inflight_, quiescing_};
  }

  [[nodiscard]] bool drained() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return provisional_ == 0U && in_flight_ == 0U && callbacks_inflight_ == 0U;
  }

  [[nodiscard]] bool wait_for_drain_until(
    const std::chrono::steady_clock::time_point deadline)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_until(lock, deadline, [this]() {
               return provisional_ == 0U && in_flight_ == 0U && callbacks_inflight_ == 0U;
      });
  }

  void clear() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();
    provisional_ = 0U;
    revoked_ = 0U;
    in_flight_ = 0U;
    callbacks_inflight_ = 0U;
    condition_.notify_all();
  }

private:
  enum class State
  {
    Provisional,
    Revoked,
    AcceptedInFlight
  };

  struct Entry
  {
    State state;
    TimePoint deadline;
    TimePoint revoked_at;
  };

  [[nodiscard]] std::size_t revoke_provisional_locked(
    const TimePoint now,
    const bool force)
  {
    std::size_t count = 0U;
    for (auto & entry : entries_) {
      if (entry.second.state == State::Provisional &&
        (force || now >= entry.second.deadline))
      {
        entry.second.state = State::Revoked;
        entry.second.revoked_at = now;
        if (provisional_ > 0U) {
          --provisional_;
        }
        ++revoked_;
        ++count;
      }
    }
    return count;
  }

  void prune_revoked_locked(const TimePoint now)
  {
    const auto retention = handoff_deadline_ * 4;
    for (auto iterator = entries_.begin(); iterator != entries_.end(); ) {
      if (iterator->second.state == State::Revoked &&
        now >= iterator->second.revoked_at + retention)
      {
        iterator = entries_.erase(iterator);
        if (revoked_ > 0U) {
          --revoked_;
        }
      } else {
        ++iterator;
      }
    }
  }

  void complete_callback(const std::string & uuid, const bool ticket_claimed) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ticket_claimed) {
      const auto found = entries_.find(uuid);
      if (found != entries_.end()) {
        entries_.erase(found);
        if (in_flight_ > 0U) {
          --in_flight_;
        }
      }
    }
    if (callbacks_inflight_ > 0U) {
      --callbacks_inflight_;
    }
    condition_.notify_all();
  }

  Clock clock_;
  Duration handoff_deadline_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::unordered_map<std::string, Entry> entries_;
  std::size_t provisional_{0U};
  std::size_t revoked_{0U};
  std::size_t in_flight_{0U};
  std::size_t callbacks_inflight_{0U};
  bool quiescing_{false};
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__ACTION_ADMISSION_TRACKER_HPP_
