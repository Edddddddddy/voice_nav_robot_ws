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

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "voice_nav_mission/relative_motion_ros_adapter.hpp"

namespace voice_nav_mission
{
namespace
{

class BlockingAuthority final : public MotionAuthorityPort
{
public:
  BlockingAuthority()
  : snapshot_{
      "gate-adapter-test", 1U, {}, GateState::Inhibited, true, true, true, true,
      {}, false, false}
  {
  }

  GateSnapshot snapshot() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

  AuthorityResult prepare(const AuthorityOperation &) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    prepare_entered_ = true;
    prepare_condition_.notify_all();
    prepare_condition_.wait(lock, [this]() {return release_prepare_;});
    snapshot_.control_seq++;
    snapshot_.lease_id = "lease-adapter-test";
    snapshot_.candidate_topic = "/candidate/adapter-test";
    snapshot_.state = GateState::Prepared;
    snapshot_.motion_inhibited = true;
    snapshot_.zero_selected = true;
    snapshot_.zero_published = true;
    snapshot_.authority_live = false;
    snapshot_.writer_bound = false;
    return AuthorityResult{
      true, true, false, snapshot_, snapshot_.lease_id, "prepared"};
  }

  AuthorityResult open(const AuthorityOperation &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return AuthorityResult{false, false, false, snapshot_, {}, "unexpected open"};
  }

  AuthorityResult renew(const AuthorityOperation &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return AuthorityResult{false, false, false, snapshot_, {}, "unexpected renew"};
  }

  AuthorityResult inhibit(const AuthorityOperation &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++inhibit_count_;
    snapshot_.control_seq++;
    snapshot_.state = GateState::Inhibited;
    snapshot_.motion_inhibited = true;
    snapshot_.zero_selected = true;
    snapshot_.zero_published = true;
    snapshot_.authority_live = false;
    snapshot_.writer_bound = false;
    return AuthorityResult{
      true, true, false, snapshot_, snapshot_.lease_id, "inhibited"};
  }

  bool wait_for_prepare()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return prepare_condition_.wait_for(
      lock, std::chrono::seconds(1), [this]() {return prepare_entered_;});
  }

  void release_prepare()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    release_prepare_ = true;
    prepare_condition_.notify_all();
  }

  std::size_t inhibit_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return inhibit_count_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable prepare_condition_;
  GateSnapshot snapshot_;
  bool prepare_entered_{false};
  bool release_prepare_{false};
  std::size_t inhibit_count_{0U};
};

class RelativeMotionRosAdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(
  RelativeMotionRosAdapterTest,
  CancelDuringPrepareUsesIndependentZeroAndUniqueCleanup)
{
  auto node = std::make_shared<rclcpp::Node>("relative_motion_adapter_test");
  auto authority = std::make_shared<BlockingAuthority>();
  MotionConditioningConfig conditioning_config;
  conditioning_config.component_rpc_timeout = std::chrono::milliseconds(100);
  conditioning_config.prepare_open_deadline = std::chrono::seconds(1);
  conditioning_config.stop_barrier = std::chrono::milliseconds(100);
  RelativeMotionRosAdapter adapter(*node, authority, {}, conditioning_config);

  std::atomic<std::size_t> result_count{0U};
  const MotionToken token{17U, 3U, 5U, 1U};
  adapter.start(
    token,
    MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.1F, 0.0F, {}},
    {},
    [&result_count](const MotionToken &, const ChildResult &) {
      result_count.fetch_add(1U);
    });
  ASSERT_TRUE(authority->wait_for_prepare());

  adapter.request_emergency_stop();
  authority->release_prepare();

  EXPECT_TRUE(adapter.emergency_stop(
    std::chrono::steady_clock::now() + std::chrono::seconds(2)));
  EXPECT_TRUE(adapter.zero_proven());
  EXPECT_EQ(authority->inhibit_count(), 1U);
  EXPECT_EQ(result_count.load(), 0U);

  adapter.shutdown();
  adapter.shutdown();
}

TEST_F(
  RelativeMotionRosAdapterTest,
  ShutdownDrainsActivePrepareBeforeReleasingAdapterResources)
{
  auto node = std::make_shared<rclcpp::Node>("relative_motion_adapter_shutdown_test");
  auto authority = std::make_shared<BlockingAuthority>();
  MotionConditioningConfig conditioning_config;
  conditioning_config.component_rpc_timeout = std::chrono::milliseconds(100);
  conditioning_config.prepare_open_deadline = std::chrono::seconds(1);
  conditioning_config.stop_barrier = std::chrono::milliseconds(100);
  RelativeMotionRosAdapter adapter(*node, authority, {}, conditioning_config);

  std::atomic<std::size_t> result_count{0U};
  const MotionToken token{18U, 3U, 6U, 1U};
  adapter.start(
    token,
    MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.1F, 0.0F, {}},
    {},
    [&result_count](const MotionToken &, const ChildResult &) {
      result_count.fetch_add(1U);
    });
  ASSERT_TRUE(authority->wait_for_prepare());

  std::atomic<bool> shutdown_finished{false};
  std::thread shutdown_thread([&]() {
      adapter.shutdown();
      shutdown_finished.store(true);
    });
  authority->release_prepare();
  shutdown_thread.join();

  EXPECT_TRUE(shutdown_finished.load());
  EXPECT_TRUE(adapter.zero_proven());
  EXPECT_EQ(authority->inhibit_count(), 1U);
  EXPECT_EQ(result_count.load(), 1U);
  adapter.shutdown();
  EXPECT_EQ(result_count.load(), 1U);
}

}  // namespace
}  // namespace voice_nav_mission
