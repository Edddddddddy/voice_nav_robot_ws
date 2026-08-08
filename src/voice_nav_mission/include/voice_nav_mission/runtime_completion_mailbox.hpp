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

#ifndef VOICE_NAV_MISSION__RUNTIME_COMPLETION_MAILBOX_HPP_
#define VOICE_NAV_MISSION__RUNTIME_COMPLETION_MAILBOX_HPP_

#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "voice_nav_mission/runtime_completion_registry.hpp"

namespace voice_nav_mission
{

// Node-owned completion seam.  The Adapter only transfers an immutable
// record here; token admission may fail later, but the independent mailbox
// work flag still wakes a non-Adapter reaper every time.
class NodeCompletionMailbox final
{
public:
  using TokenEnqueue = std::function<bool(const MotionToken &)>;
  using EmergencyRequest = std::function<void(std::string)>;

  NodeCompletionMailbox(
    TokenEnqueue token_enqueue,
    EmergencyRequest emergency_request)
  : token_enqueue_(std::move(token_enqueue)),
    emergency_request_(std::move(emergency_request)),
    reaper_thread_([this]() {run_reaper();})
  {
  }

  ~NodeCompletionMailbox()
  {
    stop();
  }

  NodeCompletionMailbox(const NodeCompletionMailbox &) = delete;
  NodeCompletionMailbox & operator=(const NodeCompletionMailbox &) = delete;

  [[nodiscard]] bool register_delivery(
    const MotionToken & token,
    RuntimeCore::ChildResultDelivery delivery)
  {
    return registry_.register_delivery(token, std::move(delivery));
  }

  void discard(const MotionToken & token) noexcept
  {
    registry_.discard(token);
  }

  [[nodiscard]] std::optional<RuntimeCompletionDispatch> take(
    const MotionToken & token)
  {
    return registry_.take(token);
  }

  [[nodiscard]] bool relay(RelativeMotionCompletionRecordPtr record) noexcept
  {
    if (!record) {
      return false;
    }
    const auto token = record->token;
    if (!registry_.accept(record)) {
      const auto retained = registry_.reject(token, std::move(record));
      reject(
        retained ? "Node completion registry rejected a terminal record" :
        "Node completion registry rejected and could not retain a terminal record");
      return false;
    }
    bool accepted = false;
    try {
      accepted = token_enqueue_ && token_enqueue_(token);
    } catch (...) {
      accepted = false;
    }
    if (!accepted) {
      const auto retained = registry_.reject_accepted(token);
      reject("Runtime event ingress rejected a terminal completion token");
      if (!retained) {
        reject("Node completion registry could not retain a rejected terminal entry");
      }
    }
    return accepted;
  }

  void close() noexcept
  {
    registry_.close();
    request_reap();
  }

  void request_reap() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(reaper_mutex_);
      if (reaper_stopped_) {
        return;
      }
      reaper_work_pending_ = true;
    }
    reaper_condition_.notify_all();
  }

  void stop() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(reaper_mutex_);
      if (reaper_stopped_) {
        return;
      }
      reaper_stopped_ = true;
      reaper_work_pending_ = true;
    }
    registry_.close();
    reaper_condition_.notify_all();
    if (reaper_thread_.joinable()) {
      reaper_thread_.join();
    }
    registry_.reap_all();
  }

  [[nodiscard]] std::size_t entry_count() const noexcept
  {
    return registry_.entry_count();
  }

  [[nodiscard]] std::size_t rejected_count() const noexcept
  {
    return registry_.rejected_count();
  }

private:
  void reject(std::string detail) noexcept
  {
    request_reap();
    try {
      if (emergency_request_) {
        emergency_request_(std::move(detail));
      }
    } catch (...) {
      // The independent reaper still owns the record/callback even when the
      // emergency notification itself cannot be delivered.
    }
  }

  void run_reaper() noexcept
  {
    for (;; ) {
      {
        std::unique_lock<std::mutex> lock(reaper_mutex_);
        reaper_condition_.wait(lock, [this]() {
            return reaper_stopped_ || reaper_work_pending_;
          });
        reaper_work_pending_ = false;
        if (reaper_stopped_) {
          lock.unlock();
          registry_.reap_rejected();
          return;
        }
      }
      registry_.reap_rejected();
    }
  }

  TokenEnqueue token_enqueue_;
  EmergencyRequest emergency_request_;
  NodeCompletionRegistry registry_;
  std::thread reaper_thread_;
  mutable std::mutex reaper_mutex_;
  std::condition_variable reaper_condition_;
  bool reaper_work_pending_{false};
  bool reaper_stopped_{false};
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_COMPLETION_MAILBOX_HPP_
