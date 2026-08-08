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
#include <chrono>
#include <mutex>
#include <optional>
#include <thread>

#include "voice_nav_mission/runtime_transaction_plane.hpp"

namespace voice_nav_mission
{
namespace
{

TEST(RuntimeTransactionPlaneTest, QuiesceAtPrepareCommitRejectsSideEffect)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool commit_entered = false;
  bool release_commit = false;
  std::size_t prepare_calls = 0U;
  RuntimeTransactionPlane plane(
    1U,
    [&](const RuntimeTransactionSideEffect side_effect) {
      ASSERT_EQ(side_effect, RuntimeTransactionSideEffect::Prepare);
      std::unique_lock<std::mutex> lock(mutex);
      commit_entered = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return release_commit;});
    });

  std::optional<int> result;
  std::thread transaction([&]() {
      result = plane.submit(
        1U,
        RuntimeTransactionSideEffect::Prepare,
        [&]() {
          ++prepare_calls;
          return 7;
        });
    });

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(1), [&]() {
        return commit_entered;
      }));
  }

  plane.quiesce(2U);
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_commit = true;
  }
  condition.notify_all();
  transaction.join();

  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(prepare_calls, 0U);
  EXPECT_TRUE(plane.quiescing());
  EXPECT_EQ(plane.generation(), 2U);
}

TEST(RuntimeTransactionPlaneTest, QuiesceDoesNotWaitForCommittedRpc)
{
  using namespace std::chrono_literals;
  std::mutex operation_mutex;
  std::condition_variable operation_condition;
  bool operation_entered = false;
  bool release_operation = false;
  std::mutex quiesce_mutex;
  std::condition_variable quiesce_condition;
  bool quiesce_completed = false;
  RuntimeTransactionPlane plane(1U);

  std::optional<int> result;
  std::thread transaction([&]() {
      result = plane.submit(
        1U,
        RuntimeTransactionSideEffect::ControllerStart,
        [&]() {
          std::unique_lock<std::mutex> lock(operation_mutex);
          operation_entered = true;
          operation_condition.notify_all();
          operation_condition.wait(lock, [&]() {return release_operation;});
          return 9;
        });
    });
  {
    std::unique_lock<std::mutex> lock(operation_mutex);
    ASSERT_TRUE(operation_condition.wait_for(lock, 1s, [&]() {
        return operation_entered;
      }));
  }

  std::thread quiescer([&]() {
      plane.quiesce(2U);
      std::lock_guard<std::mutex> lock(quiesce_mutex);
      quiesce_completed = true;
      quiesce_condition.notify_all();
    });
  {
    std::unique_lock<std::mutex> lock(quiesce_mutex);
    const auto completed_before_release = quiesce_condition.wait_for(lock, 200ms, [&]() {
          return quiesce_completed;
        });
    EXPECT_TRUE(completed_before_release);
  }
  EXPECT_TRUE(plane.quiescing());
  EXPECT_EQ(plane.generation(), 2U);

  {
    std::lock_guard<std::mutex> lock(operation_mutex);
    release_operation = true;
  }
  operation_condition.notify_all();
  transaction.join();
  quiescer.join();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, 9);
}

}  // namespace
}  // namespace voice_nav_mission
