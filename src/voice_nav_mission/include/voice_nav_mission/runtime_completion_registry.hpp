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

#ifndef VOICE_NAV_MISSION__RUNTIME_COMPLETION_REGISTRY_HPP_
#define VOICE_NAV_MISSION__RUNTIME_COMPLETION_REGISTRY_HPP_

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/motion_conditioning_pipeline.hpp"

namespace voice_nav_mission
{

inline std::string completion_token_key(const MotionToken & token)
{
  std::string key;
  key.reserve(sizeof(token.mission_id) + sizeof(token.admission_epoch) +
    sizeof(token.mission_generation) + sizeof(token.step_generation) +
    sizeof(token.admission_generation));
  const auto append = [&key](const auto value) {
      key.append(reinterpret_cast<const char *>(&value), sizeof(value));
    };
  append(token.mission_id);
  append(token.admission_epoch);
  append(token.mission_generation);
  append(token.step_generation);
  append(token.admission_generation);
  return key;
}

struct RuntimeCompletionDispatch
{
  RelativeMotionCompletionRecordPtr record;
  RuntimeCore::ChildResultDelivery delivery;
};

// Node-owned handoff storage for pure Adapter terminal records.  The Adapter
// relay only transfers the immutable record here; delivery owners are never
// put back into an Adapter transaction or completion record.  Rejected
// records have a separate bounded holding area so a closed/full relay still
// transfers ownership to the Node side before its EmergencyFence reaper runs.
class NodeCompletionRegistry final
{
public:
  static constexpr std::size_t kCapacity = 16U;
  static constexpr std::size_t kRejectedCapacity = 16U;

  [[nodiscard]] bool register_delivery(
    const MotionToken & token,
    RuntimeCore::ChildResultDelivery delivery)
  {
    if (!delivery) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return false;
    }
    const auto key = completion_token_key(token);
    if (entries_.find(key) != entries_.end() || entries_.size() >= kCapacity) {
      return false;
    }
    entries_.emplace(key, Entry{token, {}, std::move(delivery)});
    return true;
  }

  // The caller retains the shared pointer when this returns false.  A
  // successful return is the one-way Adapter-to-Node ownership handoff.
  [[nodiscard]] bool accept(RelativeMotionCompletionRecordPtr & record)
  {
    if (!record) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return false;
    }
    const auto found = entries_.find(completion_token_key(record->token));
    if (found == entries_.end() || found->second.record) {
      return false;
    }
    found->second.record = std::move(record);
    return true;
  }

  // This is called by the Node relay on every failed accept.  The Adapter
  // thread may invoke the relay, but it no longer owns the record after this
  // method returns; only the Node runtime/pre-shutdown reaper clears it.
  [[nodiscard]] bool retain_rejected(RelativeMotionCompletionRecordPtr record)
  {
    if (!record) {
      return true;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (rejected_count_locked() >= kRejectedCapacity) {
      return false;
    }
    rejected_records_.push_back(std::move(record));
    return true;
  }

  // Move a rejected record and any still-unclaimed delivery owner to the
  // Node-owned rejection mailbox. The Adapter relay never destroys either
  // side of this handoff.
  [[nodiscard]] bool reject(
    const MotionToken & token,
    RelativeMotionCompletionRecordPtr record)
  {
    if (!record) {
      return true;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (rejected_count_locked() >= kRejectedCapacity) {
      return false;
    }
    const auto found = entries_.find(completion_token_key(token));
    if (found != entries_.end() && !found->second.record) {
      rejected_entries_.push_back(
        RejectedEntry{std::move(record), std::move(found->second.delivery)});
      entries_.erase(found);
    } else {
      rejected_records_.push_back(std::move(record));
    }
    return true;
  }

  // The token was accepted into the registry but could not be admitted to the
  // Runtime queue. Move its record and delivery owner to the same bounded
  // rejection mailbox before waking the non-Adapter reaper.
  [[nodiscard]] bool reject_accepted(const MotionToken & token)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (rejected_count_locked() >= kRejectedCapacity) {
      return false;
    }
    const auto found = entries_.find(completion_token_key(token));
    if (found == entries_.end() || !found->second.record) {
      return false;
    }
    rejected_entries_.push_back(
      RejectedEntry{
        std::move(found->second.record), std::move(found->second.delivery)});
    entries_.erase(found);
    return true;
  }

  [[nodiscard]] std::optional<RuntimeCompletionDispatch> take(
    const MotionToken & token)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = entries_.find(completion_token_key(token));
    if (found == entries_.end() || !found->second.record) {
      return std::nullopt;
    }
    RuntimeCompletionDispatch dispatch{
      std::move(found->second.record), std::move(found->second.delivery)};
    entries_.erase(found);
    return dispatch;
  }

  void discard(const MotionToken & token) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.erase(completion_token_key(token));
  }

  void close() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
  }

  void reap_all() noexcept
  {
    std::unordered_map<std::string, Entry> entries;
    std::deque<RejectedEntry> rejected_entries;
    std::deque<RelativeMotionCompletionRecordPtr> rejected_records;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      entries.swap(entries_);
      rejected_entries.swap(rejected_entries_);
      rejected_records.swap(rejected_records_);
    }
  }

  void reap_rejected() noexcept
  {
    std::deque<RejectedEntry> rejected_entries;
    std::deque<RelativeMotionCompletionRecordPtr> rejected_records;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      rejected_entries.swap(rejected_entries_);
      rejected_records.swap(rejected_records_);
    }
  }

  [[nodiscard]] std::size_t entry_count() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
  }

  [[nodiscard]] std::size_t rejected_count() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return rejected_count_locked();
  }

private:
  struct Entry
  {
    MotionToken token;
    RelativeMotionCompletionRecordPtr record;
    RuntimeCore::ChildResultDelivery delivery;
  };

  struct RejectedEntry
  {
    RelativeMotionCompletionRecordPtr record;
    RuntimeCore::ChildResultDelivery delivery;
  };

  [[nodiscard]] std::size_t rejected_count_locked() const noexcept
  {
    return rejected_entries_.size() + rejected_records_.size();
  }

  mutable std::mutex mutex_;
  std::unordered_map<std::string, Entry> entries_;
  std::deque<RejectedEntry> rejected_entries_;
  std::deque<RelativeMotionCompletionRecordPtr> rejected_records_;
  bool closed_{false};
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_COMPLETION_REGISTRY_HPP_
