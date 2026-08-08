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

#ifndef VOICE_NAV_MISSION__RUNTIME_SHUTDOWN_COORDINATOR_HPP_
#define VOICE_NAV_MISSION__RUNTIME_SHUTDOWN_COORDINATOR_HPP_

#include <chrono>
#include <functional>
#include <string>
#include <utility>

namespace voice_nav_mission
{

// Package-private production Module for the Node shutdown seam.  Shutdown has
// two explicit phases: close admission/generation without waiting, then stop
// the already-running generation and wait for the joint safe conditions.  A
// failed joint barrier never claims a clean generation: it requests
// independent emergency handling, raises the admission fence, and
// synchronously asks the Node-owned Core lane to select its SAFETY_FAULT
// terminal before the outer shutdown proceeds to joins.
class RuntimeShutdownCoordinator final
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;
  using CloseAdmission = std::function<bool()>;
  using BeginRunningShutdown = std::function<void(TimePoint)>;
  using WaitForJointConditions = std::function<bool(TimePoint)>;
  using Emergency = std::function<void()>;
  using Fence = std::function<void(std::string)>;
  using CoreFailClosed = std::function<void(std::string)>;

  struct Outcome
  {
    bool transaction_drained{false};
    bool fail_closed{false};
  };

  RuntimeShutdownCoordinator(
    CloseAdmission close_admission,
    BeginRunningShutdown begin_running_shutdown,
    WaitForJointConditions wait_for_joint_conditions,
    Emergency emergency,
    Fence fence,
    CoreFailClosed core_fail_closed)
  : close_admission_(std::move(close_admission)),
    begin_running_shutdown_(std::move(begin_running_shutdown)),
    wait_for_joint_conditions_(std::move(wait_for_joint_conditions)),
    emergency_(std::move(emergency)),
    fence_(std::move(fence)),
    core_fail_closed_(std::move(core_fail_closed))
  {
  }

  RuntimeShutdownCoordinator(const RuntimeShutdownCoordinator &) = delete;
  RuntimeShutdownCoordinator & operator=(const RuntimeShutdownCoordinator &) = delete;

  [[nodiscard]] Outcome run(const TimePoint deadline) const noexcept
  {
    bool admission_closed = false;
    try {
      admission_closed = close_admission_ && close_admission_();
    } catch (...) {
      admission_closed = false;
    }
    try {
      // Even if the admission close reports a local failure, the running
      // generation must receive the independent stop request immediately.
      if (begin_running_shutdown_) {
        begin_running_shutdown_(deadline);
      }
    } catch (...) {
      admission_closed = false;
    }
    bool drained = false;
    try {
      drained = admission_closed && wait_for_joint_conditions_ &&
        wait_for_joint_conditions_(deadline);
    } catch (...) {
      drained = false;
    }
    if (drained) {
      return Outcome{true, false};
    }

    constexpr char kDetail[] =
      "transaction quiesce deadline expired; SAFETY_FAULT";
    try {
      if (emergency_) {
        emergency_();
      }
    } catch (...) {
    }
    try {
      if (fence_) {
        fence_(kDetail);
      }
    } catch (...) {
    }
    try {
      if (core_fail_closed_) {
        core_fail_closed_(kDetail);
      }
    } catch (...) {
    }
    return Outcome{false, true};
  }

private:
  CloseAdmission close_admission_;
  BeginRunningShutdown begin_running_shutdown_;
  WaitForJointConditions wait_for_joint_conditions_;
  Emergency emergency_;
  Fence fence_;
  CoreFailClosed core_fail_closed_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_SHUTDOWN_COORDINATOR_HPP_
