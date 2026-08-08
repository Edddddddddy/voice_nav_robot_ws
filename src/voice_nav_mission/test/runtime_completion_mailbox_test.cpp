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

#include <gtest/gtest.h>

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "voice_nav_mission/runtime_completion_mailbox.hpp"
#include "voice_nav_mission/runtime_terminal_handoff_lane.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;

class ReapedOwner final
{
public:
  ReapedOwner(
    std::mutex & mutex, std::condition_variable & condition,
    std::size_t & reaped, std::thread::id & last_thread)
  : mutex_(mutex), condition_(condition), reaped_(reaped), last_thread_(last_thread)
  {
  }

  ~ReapedOwner()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++reaped_;
    last_thread_ = std::this_thread::get_id();
    condition_.notify_all();
  }

private:
  std::mutex & mutex_;
  std::condition_variable & condition_;
  std::size_t & reaped_;
  std::thread::id & last_thread_;
};

MotionToken token_for(const std::uint64_t value)
{
  return MotionToken{value, 3U, 7U, 1U};
}

RelativeMotionCompletionRecordPtr record_for(const MotionToken & token)
{
  return std::make_shared<const RelativeMotionCompletionRecord>(
    RelativeMotionCompletionRecord{
        token, ChildResult{ChildResultCode::SafetyFault, "mailbox rejection"}});
}

TEST(RuntimeCompletionMailboxTest, RepeatedRejectedRecordsWakeOneReaper)
{
  std::mutex mutex;
  std::condition_variable condition;
  std::size_t reaped = 0U;
  std::thread::id last_reap_thread;
  const auto adapter_thread = std::this_thread::get_id();

  NodeCompletionMailbox mailbox(
    [](const MotionToken &) {return false;},
    [](std::string) {});
  NodeCompletionReaper reaper(mailbox);

  const auto first_token = token_for(1U);
  auto first_owner = std::make_shared<ReapedOwner>(
    mutex, condition, reaped, last_reap_thread);
  ASSERT_TRUE(mailbox.register_delivery(
    first_token,
      [first_owner](const MotionToken &, const ChildResult &) {}));
  first_owner.reset();
  auto first_record = record_for(first_token);
  EXPECT_FALSE(mailbox.relay(std::move(first_record)));

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, 2s, [&]() {return reaped == 1U;}));
  }

  const auto second_token = token_for(2U);
  auto second_owner = std::make_shared<ReapedOwner>(
    mutex, condition, reaped, last_reap_thread);
  ASSERT_TRUE(mailbox.register_delivery(
    second_token,
      [second_owner](const MotionToken &, const ChildResult &) {}));
  second_owner.reset();
  auto second_record = record_for(second_token);
  EXPECT_FALSE(mailbox.relay(std::move(second_record)));

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, 2s, [&]() {return reaped == 2U;}));
  }
  EXPECT_NE(last_reap_thread, adapter_thread);
}

TEST(RuntimeCompletionMailboxTest, RepeatedConstructionAndStopAreJoinSafe)
{
  for (std::size_t iteration = 0U; iteration < 8U; ++iteration) {
    NodeCompletionMailbox mailbox(
      [](const MotionToken &) {return false;},
      [](std::string) {});
    NodeCompletionReaper reaper(mailbox);
    mailbox.request_reap();
    mailbox.stop();
    mailbox.stop();
    EXPECT_EQ(mailbox.entry_count(), 0U);
    EXPECT_EQ(mailbox.rejected_count(), 0U);
  }
}

TEST(RuntimeCompletionMailboxTest, RejectionUsesIndependentTerminalHandoffWorker)
{
  std::mutex mutex;
  std::condition_variable condition;
  std::size_t terminal_count = 0U;
  std::thread::id terminal_thread;
  ChildResultCode terminal_code = ChildResultCode::Failed;
  NodeTerminalHandoffLane terminal_lane(
    [&](const MotionToken &, const ChildResult & result) {
      std::lock_guard<std::mutex> lock(mutex);
      ++terminal_count;
      terminal_thread = std::this_thread::get_id();
      terminal_code = result.code;
      condition.notify_all();
    });
  NodeCompletionMailbox mailbox(
    [](const MotionToken &) {return false;},
    [](std::string) {},
    [&](const MotionToken & token, const ChildResult & result) {
      return terminal_lane.enqueue(token, result);
    });
  NodeCompletionReaper reaper(mailbox);

  const auto token = token_for(9U);
  ASSERT_TRUE(mailbox.register_delivery(
    token, [](const MotionToken &, const ChildResult &) {}));
  EXPECT_FALSE(mailbox.relay(record_for(token)));

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, 2s, [&]() {return terminal_count == 1U;}));
  }
  EXPECT_EQ(terminal_code, ChildResultCode::SafetyFault);
  EXPECT_NE(terminal_thread, std::this_thread::get_id());
}

}  // namespace
}  // namespace voice_nav_mission
