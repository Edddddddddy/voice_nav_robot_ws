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

#ifndef VOICE_NAV_MISSION__MISSION_AUTHORITY_CONVERGENCE_HPP_
#define VOICE_NAV_MISSION__MISSION_AUTHORITY_CONVERGENCE_HPP_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

#include "voice_nav_mission/mission_runtime_core.hpp"

namespace voice_nav_mission
{

enum class AuthorityOperationKind : std::uint8_t
{
  Prepare = 0,
  Open = 1,
  Renew = 2,
  Inhibit = 3,
};

// Package-private policy shared by the ROS Adapter and its deterministic
// tests. The request ID remains unchanged across attempts; only a stale Gate
// tuple is rebuilt from the latest response snapshot.
class MissionAuthorityConvergence final
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;
  using Now = std::function<TimePoint()>;
  using Attempt = std::function<AuthorityResult(
        const AuthorityOperation &, TimePoint, TimePoint)>;
  using Refresh = std::function<void()>;

  [[nodiscard]] static AuthorityResult run(
    AuthorityOperation operation,
    AuthorityOperationKind kind,
    std::chrono::milliseconds overall_budget,
    std::chrono::milliseconds single_attempt_budget,
    Now now,
    Attempt attempt,
    Refresh refresh = {})
  {
    if (
      overall_budget.count() <= 0 || single_attempt_budget.count() <= 0 ||
      !now || !attempt)
    {
      throw std::invalid_argument("invalid Mission Authority convergence policy");
    }

    const auto overall_deadline = now() + overall_budget;
    auto current = std::move(operation);
    AuthorityResult last{
      false,
      false,
      true,
      {},
      {},
      "MotionGate control operation did not complete before its steady deadline"};
    while (now() < overall_deadline) {
      if (kind == AuthorityOperationKind::Inhibit && refresh) {
        refresh();
      }
      const auto attempt_started = now();
      const auto attempt_deadline = std::min(
        overall_deadline, attempt_started + single_attempt_budget);
      last = attempt(current, attempt_deadline, overall_deadline);
      if (now() >= overall_deadline) {
        last.applied = false;
        last.zero_proven = false;
        last.retryable = false;
        last.detail =
          "MotionGate control operation reached its steady deadline";
        return last;
      }
      if (
        last.applied &&
        (kind != AuthorityOperationKind::Inhibit || last.zero_proven))
      {
        return last;
      }
      if (!last.retryable) {
        return last;
      }
      if (last.tuple_stale && last.snapshot.endpoint_available) {
        current.gate_instance_id = last.snapshot.gate_instance_id;
        current.expected_control_seq = last.snapshot.control_seq;
        current.lease_id = kind == AuthorityOperationKind::Prepare ?
          std::string{} : last.snapshot.lease_id;
      }
    }
    return last;
  }
};

// Package-private production Adapter wiring. The ROS Adapter and deterministic
// tests use the same four-operation boundary, with one bounded overall window
// and a shorter per-service attempt window. The overall value is supplied by
// the trusted 250 ms STOP barrier; it is deliberately shared by PREPARE, OPEN,
// RENEW, and INHIBIT rather than chaining separate budgets.
class MissionAuthorityAdapter final
{
public:
  using TimePoint = MissionAuthorityConvergence::TimePoint;
  using Now = MissionAuthorityConvergence::Now;
  using Attempt = std::function<AuthorityResult(
        const AuthorityOperation &, AuthorityOperationKind, TimePoint, TimePoint)>;
  using Refresh = MissionAuthorityConvergence::Refresh;

  MissionAuthorityAdapter(
    std::chrono::milliseconds overall_budget,
    std::chrono::milliseconds single_attempt_budget,
    Now now,
    Attempt attempt,
    Refresh refresh = {})
  : overall_budget_(overall_budget),
    single_attempt_budget_(single_attempt_budget),
    now_(std::move(now)),
    attempt_(std::move(attempt)),
    refresh_(std::move(refresh))
  {
    if (
      overall_budget_.count() <= 0 || single_attempt_budget_.count() <= 0 ||
      !now_ || !attempt_)
    {
      throw std::invalid_argument("invalid Mission Authority Adapter policy");
    }
  }

  [[nodiscard]] AuthorityResult prepare(const AuthorityOperation & operation)
  {
    return run(operation, AuthorityOperationKind::Prepare);
  }

  [[nodiscard]] AuthorityResult open(const AuthorityOperation & operation)
  {
    return run(operation, AuthorityOperationKind::Open);
  }

  [[nodiscard]] AuthorityResult renew(const AuthorityOperation & operation)
  {
    return run(operation, AuthorityOperationKind::Renew);
  }

  [[nodiscard]] AuthorityResult inhibit(const AuthorityOperation & operation)
  {
    return run(operation, AuthorityOperationKind::Inhibit);
  }

private:
  [[nodiscard]] AuthorityResult run(
    const AuthorityOperation & operation,
    AuthorityOperationKind kind)
  {
    return MissionAuthorityConvergence::run(
      operation,
      kind,
      overall_budget_,
      single_attempt_budget_,
      now_,
      [this, kind](
        const AuthorityOperation & current,
        TimePoint attempt_deadline,
        TimePoint overall_deadline) {
        return attempt_(current, kind, attempt_deadline, overall_deadline);
        },
      refresh_);
  }

  std::chrono::milliseconds overall_budget_;
  std::chrono::milliseconds single_attempt_budget_;
  Now now_;
  Attempt attempt_;
  Refresh refresh_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__MISSION_AUTHORITY_CONVERGENCE_HPP_
