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

#ifndef VOICE_NAV_MISSION__RUNTIME_EVENT_INGRESS_HPP_
#define VOICE_NAV_MISSION__RUNTIME_EVENT_INGRESS_HPP_

#include <exception>
#include <functional>
#include <mutex>
#include <string>
#include <utility>

#include "voice_nav_mission/runtime_emergency_fence.hpp"
#include "voice_nav_mission/runtime_event_queue.hpp"

namespace voice_nav_mission
{

// The Node-owned admission seam keeps queue saturation and the independent
// emergency path in one production object.  It is intentionally independent
// of RuntimeCore so the direct Gate inhibit/zero call can run when the worker
// queue is full or closed.
template<typename Event>
class RuntimeEventIngress final
{
public:
  using Queue = RuntimeEventQueue<Event>;
  using LaneSelector = std::function<typename Queue::Lane(const Event &)>;
  using EmergencyCallback = std::function<void()>;
  using FenceCallback =
    std::function<void(const RuntimeEmergencyFenceSnapshot &)>;
  using EmergencyControlSelector = std::function<bool(const Event &)>;
  using BeforeDispatchCallback = std::function<void(Event &)>;
  using ExternalWakeCallback = std::function<void()>;

  RuntimeEventIngress(
    Queue & queue,
    RuntimeEmergencyFence & fence,
    LaneSelector lane_selector,
    EmergencyCallback emergency_callback,
    FenceCallback fence_callback,
    EmergencyControlSelector emergency_control_selector = {},
    BeforeDispatchCallback before_dispatch_callback = {},
    ExternalWakeCallback external_wake_callback = {})
  : queue_(queue),
    fence_(fence),
    lane_selector_(std::move(lane_selector)),
    emergency_callback_(std::move(emergency_callback)),
    fence_callback_(std::move(fence_callback)),
    emergency_control_selector_(std::move(emergency_control_selector)),
    before_dispatch_callback_(std::move(before_dispatch_callback)),
    external_wake_callback_(std::move(external_wake_callback))
  {
  }

  [[nodiscard]] bool enqueue(Event event) noexcept
  {
    bool emergency_control = false;
    try {
      emergency_control = emergency_control_selector_ &&
        emergency_control_selector_(event);
    } catch (...) {
      request_emergency("Runtime event emergency selector failed");
      return false;
    }
    if (fence_.blocked() && !emergency_control) {
      return false;
    }
    typename Queue::Lane lane;
    try {
      lane = lane_selector_(event);
    } catch (...) {
      request_emergency("Runtime event lane selection failed");
      return false;
    }
    const auto result = queue_.push(std::move(event), lane);
    if (result == Queue::PushResult::Accepted) {
      return true;
    }
    if (result == Queue::PushResult::ControlFull) {
      request_emergency("Runtime event admission failed: control lane full");
    } else if (result == Queue::PushResult::Closed) {
      request_emergency("Runtime event admission failed: queue closed");
    } else {
      request_emergency("Runtime event admission failed: normal lane full");
    }
    return false;
  }

  void request_emergency(std::string detail) noexcept
  {
    const bool first_raise = fence_.raise(std::move(detail));
    if (first_raise) {
      try {
        if (emergency_callback_) {
          emergency_callback_();
        }
      } catch (...) {
        // The fence remains latched even if an adapter callback is faulty.
      }
    }
    queue_.wake();
  }

  [[nodiscard]] typename Queue::WaitResult wait_pop(Event & event)
  {
    return queue_.wait_pop_with_wakeup(
      event,
      [this]() {return fence_.pending();},
      [this](const Event & value) {
        return emergency_control_selector_ && emergency_control_selector_(value);
      });
  }

  template<typename Dispatch, typename FaultHandler>
  void run(Dispatch && dispatch, FaultHandler && fault_handler) noexcept
  {
    Event event;
    while (true) {
      const auto wait_result = wait_pop(event);
      if (wait_result == Queue::WaitResult::Closed) {
        return;
      }
      if (wait_result == Queue::WaitResult::ExternalWake) {
        (void)process_pending_fence();
        try {
          if (external_wake_callback_) {
            external_wake_callback_();
          }
        } catch (...) {
          request_emergency("Runtime external wake callback raised");
        }
        continue;
      }
      try {
        if (before_dispatch_callback_) {
          before_dispatch_callback_(event);
        }
        bool dispatch_allowed = false;
        {
          std::lock_guard<std::mutex> lock(dispatch_mutex_);
          const bool emergency_control =
            emergency_control_selector_ && emergency_control_selector_(event);
          dispatch_allowed = emergency_control || !fence_.blocked();
          if (dispatch_allowed) {
            dispatch(event);
          }
        }
        if (!dispatch_allowed) {
          (void)process_pending_fence();
        }
      } catch (const std::exception & error) {
        try {
          fault_handler(std::string{"Runtime event worker raised: "} + error.what());
        } catch (...) {
          request_emergency("Runtime event worker fault handler raised");
        }
      } catch (...) {
        try {
          fault_handler("Runtime event worker raised an unknown exception");
        } catch (...) {
          request_emergency("Runtime event worker fault handler raised");
        }
      }
    }
  }

  [[nodiscard]] bool process_pending_fence() noexcept
  {
    std::lock_guard<std::mutex> lock(dispatch_mutex_);
    const auto snapshot = fence_.take();
    if (!snapshot.has_value()) {
      return false;
    }
    try {
      if (fence_callback_) {
        fence_callback_(*snapshot);
      }
    } catch (...) {
      // The direct emergency path has already run; do not reopen admission.
    }
    return true;
  }

  [[nodiscard]] bool blocked() const noexcept
  {
    return fence_.blocked();
  }

  [[nodiscard]] bool admission_allowed(
    const std::uint64_t expected_epoch) const noexcept
  {
    return fence_.admission_allowed(expected_epoch);
  }

private:
  Queue & queue_;
  RuntimeEmergencyFence & fence_;
  LaneSelector lane_selector_;
  EmergencyCallback emergency_callback_;
  FenceCallback fence_callback_;
  EmergencyControlSelector emergency_control_selector_;
  BeforeDispatchCallback before_dispatch_callback_;
  ExternalWakeCallback external_wake_callback_;
  mutable std::mutex dispatch_mutex_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_EVENT_INGRESS_HPP_
