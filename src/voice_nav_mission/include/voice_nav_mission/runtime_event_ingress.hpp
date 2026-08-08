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

#include <functional>
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

  RuntimeEventIngress(
    Queue & queue,
    RuntimeEmergencyFence & fence,
    LaneSelector lane_selector,
    EmergencyCallback emergency_callback,
    FenceCallback fence_callback)
  : queue_(queue),
    fence_(fence),
    lane_selector_(std::move(lane_selector)),
    emergency_callback_(std::move(emergency_callback)),
    fence_callback_(std::move(fence_callback))
  {
  }

  [[nodiscard]] bool enqueue(Event event) noexcept
  {
    if (fence_.blocked()) {
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
    (void)fence_.raise(std::move(detail));
    try {
      if (emergency_callback_) {
        emergency_callback_();
      }
    } catch (...) {
      // The fence remains latched even if an adapter callback is faulty.
    }
    queue_.wake();
  }

  [[nodiscard]] typename Queue::WaitResult wait_pop(Event & event)
  {
    return queue_.wait_pop_with_wakeup(
      event, [this]() {return fence_.pending();});
  }

  [[nodiscard]] bool process_pending_fence() noexcept
  {
    const auto snapshot = fence_.take();
    if (!snapshot.has_value()) {
      return false;
    }
    try {
      if (emergency_callback_) {
        emergency_callback_();
      }
    } catch (...) {
      // Keep the fail-closed fence latched if direct zero delivery throws.
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

private:
  Queue & queue_;
  RuntimeEmergencyFence & fence_;
  LaneSelector lane_selector_;
  EmergencyCallback emergency_callback_;
  FenceCallback fence_callback_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_EVENT_INGRESS_HPP_
