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

#ifndef VOICE_NAV_MISSION__RUNTIME_EVENT_QUEUE_HPP_
#define VOICE_NAV_MISSION__RUNTIME_EVENT_QUEUE_HPP_

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <utility>

namespace voice_nav_mission
{

// A bounded queue with physically separate normal and control lanes.  The
// queue never converts normal saturation into a closed queue: the producer
// receives NormalFull while the reserved control lane remains usable.
template<typename Event>
class RuntimeEventQueue final
{
public:
  enum class Lane
  {
    Normal,
    Control
  };

  enum class PushResult
  {
    Accepted,
    NormalFull,
    ControlFull,
    Closed
  };

  enum class WaitResult
  {
    Item,
    Closed,
    ExternalWake,
  };

  static constexpr std::size_t kCapacity = 128U;
  static constexpr std::size_t kNormalCapacity = 120U;
  static constexpr std::size_t kControlReserve = kCapacity - kNormalCapacity;

  using FaultFactory = std::function<Event()>;

  explicit RuntimeEventQueue(FaultFactory fault_factory)
  : fault_factory_(std::move(fault_factory))
  {
  }

  [[nodiscard]] PushResult push(Event event, const Lane lane) noexcept
  {
    try {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) {
          return PushResult::Closed;
        }
        if (lane == Lane::Normal) {
          if (normal_events_.size() >= kNormalCapacity ||
            normal_events_.size() + control_events_.size() >= kCapacity)
          {
            record_normal_fault_locked();
            condition_.notify_one();
            return PushResult::NormalFull;
          }
          normal_events_.push_back(std::move(event));
        } else {
          if (control_events_.size() >= kControlReserve ||
            normal_events_.size() + control_events_.size() >= kCapacity)
          {
            condition_.notify_one();
            return PushResult::ControlFull;
          }
          control_events_.push_back(std::move(event));
        }
      }
      condition_.notify_one();
      return PushResult::Accepted;
    } catch (...) {
      return lane == Lane::Normal ? PushResult::NormalFull : PushResult::ControlFull;
    }
  }

  [[nodiscard]] bool wait_pop(Event & event)
  {
    return wait_pop_result(event) == WaitResult::Item;
  }

  [[nodiscard]] WaitResult wait_pop_result(Event & event)
  {
    return wait_pop_with_wakeup(event, []() {return false;});
  }

  template<typename ExternalWakePredicate>
  [[nodiscard]] WaitResult wait_pop_with_wakeup(
    Event & event,
    ExternalWakePredicate && external_wake_pending)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this, &external_wake_pending]() {
        return closed_ || !control_events_.empty() || pending_fault_.has_value() ||
               !normal_events_.empty() || external_wake_pending();
      });
    if (!control_events_.empty()) {
      event = std::move(control_events_.front());
      control_events_.pop_front();
      return WaitResult::Item;
    }
    if (pending_fault_.has_value()) {
      event = std::move(*pending_fault_);
      pending_fault_.reset();
      return WaitResult::Item;
    }
    if (external_wake_pending()) {
      return WaitResult::ExternalWake;
    }
    if (normal_events_.empty()) {
      return WaitResult::Closed;
    }
    event = std::move(normal_events_.front());
    normal_events_.pop_front();
    return WaitResult::Item;
  }

  void wake() noexcept
  {
    condition_.notify_all();
  }

  [[nodiscard]] PushResult request_fault(Event event) noexcept
  {
    try {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) {
          return PushResult::Closed;
        }
        if (control_events_.size() < kControlReserve &&
          normal_events_.size() + control_events_.size() < kCapacity)
        {
          control_events_.push_back(std::move(event));
        } else if (!pending_fault_.has_value()) {
          pending_fault_.emplace(std::move(event));
        } else {
          return PushResult::ControlFull;
        }
      }
      condition_.notify_one();
      return PushResult::Accepted;
    } catch (...) {
      return PushResult::ControlFull;
    }
  }

  void close() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    condition_.notify_all();
  }

private:
  void record_normal_fault_locked() noexcept
  {
    if (fault_recorded_) {
      return;
    }
    fault_recorded_ = true;
    if (!fault_factory_) {
      return;
    }
    try {
      auto fault = fault_factory_();
      if (control_events_.size() < kControlReserve &&
        normal_events_.size() + control_events_.size() < kCapacity)
      {
        control_events_.push_back(std::move(fault));
      } else {
        pending_fault_.emplace(std::move(fault));
      }
    } catch (...) {
      // The caller still receives NormalFull and owns the independent
      // emergency path when fault materialization itself cannot complete.
    }
  }

  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Event> control_events_;
  std::deque<Event> normal_events_;
  std::optional<Event> pending_fault_;
  FaultFactory fault_factory_;
  bool fault_recorded_{false};
  bool closed_{false};
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_EVENT_QUEUE_HPP_
