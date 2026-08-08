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

#ifndef VOICE_NAV_MISSION__RUNTIME_TERMINAL_HANDOFF_LANE_HPP_
#define VOICE_NAV_MISSION__RUNTIME_TERMINAL_HANDOFF_LANE_HPP_

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "voice_nav_mission/mission_runtime_core.hpp"

namespace voice_nav_mission
{

// Node-owned terminal lane.  It is deliberately independent of the normal
// RuntimeEvent queue and the one-shot EmergencyFence.  Adapter relay failure
// only appends pure data here; Core is touched by this lane's worker, after
// the event worker and the lane share the RuntimeExecutionPlane serial gate.
class NodeTerminalHandoffLane final
{
public:
  using TerminalHandler = std::function<void(const MotionToken &, const ChildResult &)>;
  using FaultHandler = std::function<void(std::string)>;

  static constexpr std::size_t kCapacity = 16U;

  NodeTerminalHandoffLane(
    TerminalHandler terminal_handler,
    FaultHandler fault_handler = {})
  : terminal_handler_(std::move(terminal_handler)),
    fault_handler_(std::move(fault_handler)),
    worker_([this]() {run();})
  {
  }

  ~NodeTerminalHandoffLane()
  {
    stop();
  }

  NodeTerminalHandoffLane(const NodeTerminalHandoffLane &) = delete;
  NodeTerminalHandoffLane & operator=(const NodeTerminalHandoffLane &) = delete;

  [[nodiscard]] bool enqueue(
    const MotionToken & token,
    const ChildResult & result) noexcept
  {
    try {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_ || entries_.size() >= kCapacity) {
          return false;
        }
        entries_.push_back(TerminalEntry{token, result});
      }
      condition_.notify_one();
      return true;
    } catch (...) {
      return false;
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

  void stop() noexcept
  {
    close();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  [[nodiscard]] std::size_t pending_count() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
  }

private:
  struct TerminalEntry
  {
    MotionToken token;
    ChildResult result;
  };

  void report_fault(std::string detail) noexcept
  {
    try {
      if (fault_handler_) {
        fault_handler_(std::move(detail));
      }
    } catch (...) {
      // The lane is already fail-closed if its handler cannot be delivered.
    }
  }

  void run() noexcept
  {
    for (;; ) {
      TerminalEntry entry;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() {
            return closed_ || !entries_.empty();
          });
        if (entries_.empty()) {
          return;
        }
        entry = std::move(entries_.front());
        entries_.pop_front();
      }
      try {
        if (terminal_handler_) {
          terminal_handler_(entry.token, entry.result);
        } else {
          report_fault("Node terminal handoff has no Core handler");
        }
      } catch (const std::exception & error) {
        report_fault(std::string{"Node terminal handoff raised: "} + error.what());
      } catch (...) {
        report_fault("Node terminal handoff raised an unknown exception");
      }
    }
  }

  TerminalHandler terminal_handler_;
  FaultHandler fault_handler_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<TerminalEntry> entries_;
  bool closed_{false};
  std::thread worker_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_TERMINAL_HANDOFF_LANE_HPP_
