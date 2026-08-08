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
  bool quiesce_started = false;
  std::size_t prepare_calls = 0U;
  RuntimeTransactionPlane plane(
    1U,
    [&](const RuntimeTransactionSideEffect side_effect) {
      ASSERT_EQ(side_effect, RuntimeTransactionSideEffect::Prepare);
      std::unique_lock<std::mutex> lock(mutex);
      commit_entered = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return release_commit;});
    },
    {},
    [&](const std::uint64_t generation) {
      ASSERT_EQ(generation, 2U);
      std::lock_guard<std::mutex> lock(mutex);
      quiesce_started = true;
      condition.notify_all();
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

  std::optional<bool> quiesce_result;
  std::thread quiescer([&]() {
      quiesce_result = plane.quiesce(
        2U, std::chrono::steady_clock::now() + std::chrono::seconds(1));
    });
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(1), [&]() {
        return quiesce_started;
      }));
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_commit = true;
  }
  condition.notify_all();
  transaction.join();
  quiescer.join();

  ASSERT_TRUE(quiesce_result.has_value());
  EXPECT_TRUE(*quiesce_result);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(prepare_calls, 0U);
  EXPECT_TRUE(plane.quiescing());
  EXPECT_EQ(plane.generation(), 2U);
}

TEST(RuntimeTransactionPlaneTest, QuiesceWaitsForCommittedRpc)
{
  using namespace std::chrono_literals;
  std::mutex operation_mutex;
  std::condition_variable operation_condition;
  bool operation_entered = false;
  bool release_operation = false;
  std::mutex quiesce_mutex;
  std::condition_variable quiesce_condition;
  bool quiesce_completed = false;
  std::optional<bool> quiesce_result;
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
      quiesce_result = plane.quiesce(
        2U, std::chrono::steady_clock::now() + std::chrono::seconds(1));
      std::lock_guard<std::mutex> lock(quiesce_mutex);
      quiesce_completed = true;
      quiesce_condition.notify_all();
    });
  {
    std::unique_lock<std::mutex> lock(quiesce_mutex);
    const auto completed_before_release = quiesce_condition.wait_for(lock, 200ms, [&]() {
          return quiesce_completed;
        });
    EXPECT_FALSE(completed_before_release);
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
  ASSERT_TRUE(quiesce_result.has_value());
  EXPECT_TRUE(*quiesce_result);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, 9);
}

TEST(RuntimeTransactionPlaneTest, QuiesceWaitsForOperationAfterFinalPermit)
{
  using namespace std::chrono_literals;
  std::mutex mutex;
  std::condition_variable condition;
  bool operation_gate_entered = false;
  bool release_operation_gate = false;
  bool operation_called = false;
  std::optional<int> result;
  RuntimeTransactionPlane plane(
    1U,
    {},
    [&](const RuntimeTransactionSideEffect side_effect) {
      EXPECT_EQ(side_effect, RuntimeTransactionSideEffect::Open);
      std::unique_lock<std::mutex> lock(mutex);
      operation_gate_entered = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return release_operation_gate;});
    });

  std::thread transaction([&]() {
      result = plane.submit(
        1U,
        RuntimeTransactionSideEffect::Open,
        [&]() {
          std::lock_guard<std::mutex> lock(mutex);
          operation_called = true;
          return 11;
        });
    });
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, 1s, [&]() {
        return operation_gate_entered;
      }));
  }

  std::optional<bool> quiesce_result;
  bool quiesce_completed = false;
  std::thread quiescer([&]() {
      quiesce_result = plane.quiesce(2U, std::chrono::steady_clock::now() + 1s);
      std::lock_guard<std::mutex> lock(mutex);
      quiesce_completed = true;
      condition.notify_all();
    });
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_FALSE(condition.wait_for(lock, 100ms, [&]() {
        return quiesce_completed;
      }));
    release_operation_gate = true;
  }
  condition.notify_all();

  transaction.join();
  quiescer.join();
  ASSERT_TRUE(quiesce_result.has_value());
  EXPECT_TRUE(*quiesce_result);
  EXPECT_TRUE(operation_called);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, 11);
}

TEST(RuntimeTransactionPlaneTest, ExpiredQuiesceDeadlineKeepsGenerationFenced)
{
  using namespace std::chrono_literals;
  std::mutex mutex;
  std::condition_variable condition;
  bool operation_gate_entered = false;
  bool release_operation_gate = false;
  std::optional<int> result;
  RuntimeTransactionPlane plane(
    1U,
    {},
    [&](const RuntimeTransactionSideEffect) {
      std::unique_lock<std::mutex> lock(mutex);
      operation_gate_entered = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return release_operation_gate;});
    });

  std::thread transaction([&]() {
      result = plane.submit(
        1U,
        RuntimeTransactionSideEffect::ControllerStart,
        []() {return 13;});
    });
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, 1s, [&]() {
        return operation_gate_entered;
      }));
  }

  EXPECT_FALSE(plane.quiesce(
    2U, std::chrono::steady_clock::now() - 1ms));
  EXPECT_TRUE(plane.quiescing());
  EXPECT_EQ(plane.generation(), 2U);
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_operation_gate = true;
  }
  condition.notify_all();
  transaction.join();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, 13);
}

}  // namespace
}  // namespace voice_nav_mission
