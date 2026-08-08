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
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "voice_nav_mission/runtime_event_queue.hpp"
#include "voice_nav_mission/runtime_emergency_fence.hpp"
#include "voice_nav_mission/runtime_event_ingress.hpp"

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

TEST(RuntimeEventQueueTest, ControlFullUsesExternalEmergencyFenceBeforeNormalWork)
{
  RuntimeEventQueue<int> queue([] {return 9999;});
  for (std::size_t index = 0U;
    index < RuntimeEventQueue<int>::kControlReserve; ++index)
  {
    ASSERT_EQ(
      queue.push(static_cast<int>(index), RuntimeEventQueue<int>::Lane::Control),
      RuntimeEventQueue<int>::PushResult::Accepted);
  }
  ASSERT_EQ(
    queue.push(1000, RuntimeEventQueue<int>::Lane::Control),
    RuntimeEventQueue<int>::PushResult::ControlFull);

  RuntimeEmergencyFence fence(1U);
  ASSERT_TRUE(fence.raise("control lane admission failed"));
  ASSERT_FALSE(fence.raise("duplicate fence"));

  int event = 0;
  EXPECT_EQ(
    queue.wait_pop_with_wakeup(event, [&fence]() {return fence.pending();}),
    RuntimeEventQueue<int>::WaitResult::Item);
  EXPECT_EQ(event, 0);

  for (std::size_t index = 1U;
    index < RuntimeEventQueue<int>::kControlReserve; ++index)
  {
    ASSERT_EQ(
      queue.wait_pop_with_wakeup(event, [&fence]() {return fence.pending();}),
      RuntimeEventQueue<int>::WaitResult::Item);
  }
  EXPECT_EQ(
    queue.wait_pop_with_wakeup(event, [&fence]() {return fence.pending();}),
    RuntimeEventQueue<int>::WaitResult::ExternalWake);
  const auto snapshot = fence.take();
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->admission_epoch, 2U);
  EXPECT_TRUE(fence.blocked());
  EXPECT_FALSE(fence.pending());
}

TEST(RuntimeEventQueueTest, CloseDrainsAcceptedCompletionBeforeWorkerExit)
{
  RuntimeEventQueue<int> queue([] {return 9999;});
  ASSERT_EQ(
    queue.push(1, RuntimeEventQueue<int>::Lane::Normal),
    RuntimeEventQueue<int>::PushResult::Accepted);
  ASSERT_EQ(
    queue.push(2, RuntimeEventQueue<int>::Lane::Control),
    RuntimeEventQueue<int>::PushResult::Accepted);

  queue.close();

  int event = 0;
  EXPECT_EQ(queue.wait_pop_result(event), RuntimeEventQueue<int>::WaitResult::Item);
  EXPECT_EQ(event, 2);
  EXPECT_EQ(queue.wait_pop_result(event), RuntimeEventQueue<int>::WaitResult::Item);
  EXPECT_EQ(event, 1);
  EXPECT_EQ(queue.wait_pop_result(event), RuntimeEventQueue<int>::WaitResult::Closed);
  EXPECT_EQ(
    queue.push(3, RuntimeEventQueue<int>::Lane::Control),
    RuntimeEventQueue<int>::PushResult::Closed);
}

TEST(RuntimeEventIngressTest, ControlSaturationUsesProductionEmergencySeam)
{
  using Queue = RuntimeEventQueue<int>;
  Queue queue([] {return -1;});
  RuntimeEmergencyFence fence(1U);
  std::mutex zero_mutex;
  std::condition_variable zero_condition;
  std::size_t direct_zero_calls = 0U;
  bool zero_requested = false;
  std::optional<RuntimeEmergencyFenceSnapshot> failed;
  RuntimeEventIngress<int> ingress(
    queue,
    fence,
    [](const int &) {return Queue::Lane::Control;},
    [&]() {
      {
        std::lock_guard<std::mutex> lock(zero_mutex);
        ++direct_zero_calls;
        zero_requested = true;
      }
      zero_condition.notify_all();
    },
    [&failed](const RuntimeEmergencyFenceSnapshot & snapshot) {
      failed = snapshot;
    });

  for (int value = 0; value < static_cast<int>(Queue::kControlReserve); ++value) {
    EXPECT_TRUE(ingress.enqueue(value));
  }
  EXPECT_FALSE(ingress.enqueue(99));
  {
    std::unique_lock<std::mutex> lock(zero_mutex);
    ASSERT_TRUE(zero_condition.wait_for(lock, std::chrono::seconds(1), [&]() {
        return zero_requested;
    }));
  }
  EXPECT_GE(direct_zero_calls, 1U);
  EXPECT_TRUE(fence.blocked());
  EXPECT_EQ(fence.admission_epoch(), 2U);

  ASSERT_TRUE(ingress.process_pending_fence());
  ASSERT_TRUE(failed.has_value());
  EXPECT_EQ(failed->admission_epoch, 2U);
  EXPECT_NE(failed->detail.find("control lane"), std::string::npos);
  EXPECT_GE(direct_zero_calls, 2U);

  int accepted_before_fence = 0;
  ASSERT_EQ(
    queue.wait_pop_result(accepted_before_fence),
    Queue::WaitResult::Item);
  EXPECT_FALSE(ingress.enqueue(100));
}

}  // namespace
}  // namespace voice_nav_mission
