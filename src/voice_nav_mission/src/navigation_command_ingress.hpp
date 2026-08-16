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

#ifndef VOICE_NAV_MISSION__NAVIGATION_COMMAND_INGRESS_HPP_
#define VOICE_NAV_MISSION__NAVIGATION_COMMAND_INGRESS_HPP_

#include <algorithm>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

#include "voice_nav_mission/motion_gate_core.hpp"
#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/relative_motion_controller.hpp"

namespace voice_nav_mission
{
namespace detail
{

// Navigation ingress must expire before MotionGate's independent 150 ms
// candidate deadline.  The budget is deliberately package-private: it is a
// safety fence for the Nav2 adapter, not a product tuning parameter.
inline constexpr std::chrono::milliseconds kNavigationCommandFreshness{125};

// A Nav2 CancelGoal response is only a transport acknowledgement.  This
// package-private lifecycle keeps the exact goal identity live until its
// matching result callback is observed, so a replacement generation cannot
// reuse the writer while the old goal is still CANCELING.
class NavigationGoalLifecycle final
{
public:
  using GoalId = std::array<std::uint8_t, 16>;
  using TimePoint = SteadyClockPort::TimePoint;

  void begin(const GoalId & goal_id) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_id_ = goal_id;
    active_ = true;
    cancel_requested_ = false;
    cancel_accepted_ = false;
    terminal_ = false;
  }

  void reset() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    cancel_requested_ = false;
    cancel_accepted_ = false;
    terminal_ = false;
    goal_id_.fill(0U);
  }

  void request_cancel() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) {
      cancel_requested_ = true;
    }
  }

  [[nodiscard]] bool accept_cancel_response(
    const std::uint8_t return_code,
    const std::vector<GoalId> & goals_canceling) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_requested_ = true;
    if (!active_ || return_code != 0U ||
      std::find(goals_canceling.cbegin(), goals_canceling.cend(), goal_id_) ==
      goals_canceling.cend())
    {
      return false;
    }
    cancel_accepted_ = true;
    condition_.notify_all();
    return true;
  }

  [[nodiscard]] bool observe_terminal(const GoalId & goal_id) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || goal_id != goal_id_) {
      return false;
    }
    terminal_ = true;
    condition_.notify_all();
    return true;
  }

  [[nodiscard]] bool wait_for_terminal(const TimePoint deadline) noexcept
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_until(lock, deadline, [this]() {return terminal_;});
  }

  [[nodiscard]] bool cancel_accepted() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cancel_accepted_;
  }

  [[nodiscard]] bool cancel_complete() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cancel_requested_ && cancel_accepted_ && terminal_;
  }

  [[nodiscard]] bool next_generation_allowed() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return terminal_ && (!cancel_requested_ || cancel_accepted_);
  }

  [[nodiscard]] bool blocks_next_generation() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_ && (!terminal_ || (cancel_requested_ && !cancel_accepted_));
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  GoalId goal_id_{};
  bool active_{false};
  bool cancel_requested_{false};
  bool cancel_accepted_{false};
  bool terminal_{false};
};

class NavigationCommandIngress final
{
public:
  using TimePoint = SteadyClockPort::TimePoint;

  enum class Observation : std::uint8_t
  {
    Accepted = 0,
    StaleGeneration = 1,
    WriterUnavailable = 2,
    WriterMismatch = 3,
    ReceiptRegressed = 4,
    InvalidCommand = 5,
  };

  struct Sample
  {
    RelativeMotionCommand command{};
    bool freshness_expired{false};
  };

  void begin(
    const MotionToken & token,
    const TimePoint started_at,
    const WriterGid & expected_writer_gid) noexcept
  {
    active_ = true;
    token_ = token;
    writer_gid_ = expected_writer_gid;
    last_command_ = {};
    last_receipt_ = started_at;
    observed_ = false;
    expired_ = false;
    expiry_reported_ = false;
  }

  void end() noexcept
  {
    active_ = false;
    writer_gid_.reset();
    last_command_ = {};
    observed_ = false;
    expired_ = false;
    expiry_reported_ = false;
  }

  [[nodiscard]] Observation observe(
    const MotionToken & token,
    const WriterGid & writer_gid,
    const RelativeMotionCommand & command,
    const TimePoint receipt) noexcept
  {
    if (!active_ || !same_token(token_, token) || expired_) {
      return Observation::StaleGeneration;
    }
    if (gid_is_zero(writer_gid)) {
      return Observation::WriterUnavailable;
    }
    if (
      !std::isfinite(command.linear_x_mps) ||
      !std::isfinite(command.angular_z_rps))
    {
      return Observation::InvalidCommand;
    }
    if (observed_ && receipt < last_receipt_) {
      return Observation::ReceiptRegressed;
    }
    if (!writer_gid_.has_value() || *writer_gid_ != writer_gid) {
      return Observation::WriterMismatch;
    }
    last_command_ = command;
    last_receipt_ = receipt;
    observed_ = true;
    return Observation::Accepted;
  }

  [[nodiscard]] Sample command(
    const MotionToken & token,
    const TimePoint now) const noexcept
  {
    if (!active_ || !same_token(token_, token) || expired_ || !observed_) {
      return {};
    }
    if (now < last_receipt_) {
      return {};
    }
    if (now - last_receipt_ > kNavigationCommandFreshness) {
      return {{}, true};
    }
    return {last_command_, false};
  }

  [[nodiscard]] std::optional<MotionToken> take_expired_token(
    const TimePoint now) noexcept
  {
    const auto sample = command(token_, now);
    if (!sample.freshness_expired || expiry_reported_) {
      return std::nullopt;
    }
    expired_ = true;
    expiry_reported_ = true;
    return token_;
  }

private:
  [[nodiscard]] static bool same_token(
    const MotionToken & left,
    const MotionToken & right) noexcept
  {
    return left.mission_id == right.mission_id &&
           left.admission_epoch == right.admission_epoch &&
           left.mission_generation == right.mission_generation &&
           left.step_generation == right.step_generation &&
           left.admission_generation == right.admission_generation;
  }

  [[nodiscard]] static bool gid_is_zero(const WriterGid & gid) noexcept
  {
    return std::all_of(gid.cbegin(), gid.cend(), [](const auto byte) {
             return byte == 0U;
           });
  }

  bool active_{false};
  MotionToken token_{};
  std::optional<WriterGid> writer_gid_;
  RelativeMotionCommand last_command_{};
  TimePoint last_receipt_{};
  bool observed_{false};
  bool expired_{false};
  bool expiry_reported_{false};
};

}  // namespace detail
}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__NAVIGATION_COMMAND_INGRESS_HPP_
