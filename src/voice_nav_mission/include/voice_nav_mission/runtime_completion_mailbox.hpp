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
  // This callback only enqueues immutable terminal data. It must never touch
  // RuntimeCore directly; the RuntimeEngine worker owns that write.
  using TerminalEnqueue = std::function<bool(const MotionToken &, const ChildResult &)>;

  NodeCompletionMailbox(
    TokenEnqueue token_enqueue,
    EmergencyRequest emergency_request,
    TerminalEnqueue terminal_enqueue = {})
  : token_enqueue_(std::move(token_enqueue)),
    emergency_request_(std::move(emergency_request)),
    terminal_enqueue_(std::move(terminal_enqueue))
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
    const auto result = record->result;
    if (!registry_.accept(record)) {
      const auto retained = registry_.reject(token, std::move(record));
      const auto terminal_enqueued = enqueue_terminal(token, result);
      reject(
        retained ? "Node completion registry rejected a terminal record" :
        "Node completion registry rejected and could not retain a terminal record");
      if (!terminal_enqueued) {
        reject("Runtime terminal event ingress rejected a terminal record");
      }
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
      const auto terminal_enqueued = enqueue_terminal(token, result);
      reject("Runtime event ingress rejected a terminal completion token");
      if (!retained) {
        reject("Node completion registry could not retain a rejected terminal entry");
      }
      if (!terminal_enqueued) {
        reject("Runtime terminal event ingress rejected a terminal record");
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
    close();
    stop_reaper();
    registry_.reap_all();
  }

  // The mailbox has no joinable thread.  A separate Node-owned reaper calls
  // these methods so releasing a rejected delivery owner can never require a
  // mailbox to join itself.
  [[nodiscard]] bool wait_for_reaper_work() noexcept
  {
    std::unique_lock<std::mutex> lock(reaper_mutex_);
    reaper_condition_.wait(lock, [this]() {
        return reaper_stopped_ || reaper_work_pending_;
      });
    reaper_work_pending_ = false;
    return !reaper_stopped_;
  }

  void stop_reaper() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(reaper_mutex_);
      reaper_stopped_ = true;
      reaper_work_pending_ = true;
    }
    reaper_condition_.notify_all();
  }

  [[nodiscard]] std::size_t entry_count() const noexcept
  {
    return registry_.entry_count();
  }

  [[nodiscard]] std::size_t rejected_count() const noexcept
  {
    return registry_.rejected_count();
  }

  void reap_rejected() noexcept
  {
    registry_.reap_rejected();
  }

  void reap_all() noexcept
  {
    registry_.reap_all();
  }

private:
  [[nodiscard]] bool enqueue_terminal(
    const MotionToken & token,
    const ChildResult & result) noexcept
  {
    try {
      return terminal_enqueue_ && terminal_enqueue_(token, result);
    } catch (...) {
      return false;
    }
  }

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

  TokenEnqueue token_enqueue_;
  EmergencyRequest emergency_request_;
  TerminalEnqueue terminal_enqueue_;
  NodeCompletionRegistry registry_;
  mutable std::mutex reaper_mutex_;
  std::condition_variable reaper_condition_;
  bool reaper_work_pending_{false};
  bool reaper_stopped_{false};
};

// The reaper is deliberately an external owner of the mailbox's joinable
// thread.  RuntimeExecutionPlane stops this object before releasing Core or
// the mailbox, and joins it from the Node shutdown owner.
class NodeCompletionReaper final
{
public:
  explicit NodeCompletionReaper(NodeCompletionMailbox & mailbox)
  : mailbox_(mailbox),
    thread_([this]() {run();})
  {
  }

  ~NodeCompletionReaper()
  {
    stop();
  }

  NodeCompletionReaper(const NodeCompletionReaper &) = delete;
  NodeCompletionReaper & operator=(const NodeCompletionReaper &) = delete;

  void stop() noexcept
  {
    mailbox_.stop_reaper();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  [[nodiscard]] std::thread::id thread_id() const noexcept
  {
    return thread_.get_id();
  }

private:
  void run() noexcept
  {
    while (mailbox_.wait_for_reaper_work()) {
      mailbox_.reap_rejected();
    }
    mailbox_.reap_all();
  }

  NodeCompletionMailbox & mailbox_;
  std::thread thread_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_COMPLETION_MAILBOX_HPP_
