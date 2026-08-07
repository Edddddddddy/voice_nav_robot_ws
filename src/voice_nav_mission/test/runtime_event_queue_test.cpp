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

#include <cstddef>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "voice_nav_mission/runtime_event_queue.hpp"

namespace voice_nav_mission
{
namespace
{

TEST(RuntimeEventQueueTest, NormalSaturationLeavesControlReserveUsable)
{
  RuntimeEventQueue<int> queue([] {return 999;});

  for (std::size_t index = 0U; index < RuntimeEventQueue<int>::kNormalCapacity; ++index) {
    EXPECT_EQ(
      queue.push(static_cast<int>(index), RuntimeEventQueue<int>::Lane::Normal),
      RuntimeEventQueue<int>::PushResult::Accepted);
  }

  EXPECT_EQ(
    queue.push(1000, RuntimeEventQueue<int>::Lane::Normal),
    RuntimeEventQueue<int>::PushResult::NormalFull);
  EXPECT_EQ(
    queue.push(2000, RuntimeEventQueue<int>::Lane::Control),
    RuntimeEventQueue<int>::PushResult::Accepted);

  int event = 0;
  ASSERT_TRUE(queue.wait_pop(event));
  EXPECT_EQ(event, 999);
  ASSERT_TRUE(queue.wait_pop(event));
  EXPECT_EQ(event, 2000);
}

TEST(RuntimeEventQueueTest, ControlIntentPrecedesNormalBacklog)
{
  RuntimeEventQueue<int> queue([] {return 999;});

  for (std::size_t index = 0U; index < RuntimeEventQueue<int>::kNormalCapacity; ++index) {
    ASSERT_EQ(
      queue.push(static_cast<int>(index), RuntimeEventQueue<int>::Lane::Normal),
      RuntimeEventQueue<int>::PushResult::Accepted);
  }

  ASSERT_EQ(
    queue.push(2000, RuntimeEventQueue<int>::Lane::Control),
    RuntimeEventQueue<int>::PushResult::Accepted);

  int event = 0;
  ASSERT_TRUE(queue.wait_pop(event));
  EXPECT_EQ(event, 2000);
}

TEST(RuntimeEventQueueTest, QueuedControlIsNotOvertakenByLaterQueueFault)
{
  RuntimeEventQueue<int> queue([] {return 999;});

  ASSERT_EQ(
    queue.push(2000, RuntimeEventQueue<int>::Lane::Control),
    RuntimeEventQueue<int>::PushResult::Accepted);
  for (std::size_t index = 0U; index < RuntimeEventQueue<int>::kNormalCapacity; ++index) {
    ASSERT_EQ(
      queue.push(static_cast<int>(index), RuntimeEventQueue<int>::Lane::Normal),
      RuntimeEventQueue<int>::PushResult::Accepted);
  }
  EXPECT_EQ(
    queue.push(3000, RuntimeEventQueue<int>::Lane::Normal),
    RuntimeEventQueue<int>::PushResult::NormalFull);

  int event = 0;
  ASSERT_TRUE(queue.wait_pop(event));
  EXPECT_EQ(event, 2000);
  ASSERT_TRUE(queue.wait_pop(event));
  EXPECT_EQ(event, 999);
}

TEST(RuntimeEventQueueTest, ConcurrentControlSaturationKeepsEveryControlIntent)
{
  RuntimeEventQueue<int> queue([] {return 9999;});
  for (std::size_t index = 0U; index < RuntimeEventQueue<int>::kNormalCapacity; ++index) {
    ASSERT_EQ(
      queue.push(static_cast<int>(index), RuntimeEventQueue<int>::Lane::Normal),
      RuntimeEventQueue<int>::PushResult::Accepted);
  }

  std::mutex start_mutex;
  std::condition_variable start_condition;
  std::size_t ready = 0U;
  bool release = false;
  auto wait_for_release = [&]() {
      std::unique_lock<std::mutex> lock(start_mutex);
      ++ready;
      start_condition.notify_all();
      start_condition.wait(lock, [&]() {return release;});
    };
  RuntimeEventQueue<int>::PushResult stop_result;
  RuntimeEventQueue<int>::PushResult cancel_result;
  std::thread stop_thread([&]() {
      wait_for_release();
      stop_result = queue.push(2001, RuntimeEventQueue<int>::Lane::Control);
    });
  std::thread cancel_thread([&]() {
      wait_for_release();
      cancel_result = queue.push(2002, RuntimeEventQueue<int>::Lane::Control);
    });
  std::thread overflow_thread([&]() {
      wait_for_release();
      EXPECT_EQ(
        queue.push(3001, RuntimeEventQueue<int>::Lane::Normal),
        RuntimeEventQueue<int>::PushResult::NormalFull);
    });

  {
    std::unique_lock<std::mutex> lock(start_mutex);
    ASSERT_TRUE(start_condition.wait_for(lock, std::chrono::seconds(1), [&]() {
        return ready == 3U;
    }));
    release = true;
  }
  start_condition.notify_all();
  stop_thread.join();
  cancel_thread.join();
  overflow_thread.join();

  EXPECT_EQ(stop_result, RuntimeEventQueue<int>::PushResult::Accepted);
  EXPECT_EQ(cancel_result, RuntimeEventQueue<int>::PushResult::Accepted);

  std::vector<int> control_values;
  for (std::size_t index = 0U; index < 3U; ++index) {
    int event = 0;
    ASSERT_TRUE(queue.wait_pop(event));
    control_values.push_back(event);
  }
  EXPECT_NE(
    std::find(control_values.cbegin(), control_values.cend(), 2001),
    control_values.cend());
  EXPECT_NE(
    std::find(control_values.cbegin(), control_values.cend(), 2002),
    control_values.cend());
  EXPECT_NE(
    std::find(control_values.cbegin(), control_values.cend(), 9999),
    control_values.cend());
}

}  // namespace
}  // namespace voice_nav_mission
