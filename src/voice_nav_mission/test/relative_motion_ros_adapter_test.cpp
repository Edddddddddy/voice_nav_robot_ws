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
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "voice_nav_mission/relative_motion_ros_adapter.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;

class CallbackBarrier final
{
public:
  void enter()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++entered_;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return released_;});
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

  void mark_shutdown_wait()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_waiting_ = true;
    condition_.notify_all();
  }

  bool wait_for_entries(const std::size_t count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this, count]() {
               return entered_ >= count;
             });
  }

  bool wait_for_shutdown_wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this]() {return shutdown_waiting_;});
  }

  std::size_t entries() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return entered_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::size_t entered_{0U};
  bool released_{false};
  bool shutdown_waiting_{false};
};

class ExternalCompletionRelay final
{
public:
  ExternalCompletionRelay()
  : worker_([this]() {run();})
  {
  }

  ~ExternalCompletionRelay()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  [[nodiscard]] bool publish(RelativeMotionCompletionRecordPtr record)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopped_) {
        return false;
      }
      records_.push_back(std::move(record));
    }
    condition_.notify_one();
    return true;
  }

  [[nodiscard]] bool wait_for_deliveries(const std::size_t count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 1s, [this, count]() {
               return delivered_ >= count;
             });
  }

  [[nodiscard]] std::size_t deliveries() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return delivered_;
  }

  [[nodiscard]] std::thread::id last_delivery_thread() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_delivery_thread_;
  }

private:
  void run()
  {
    for (;; ) {
      RelativeMotionCompletionRecordPtr record;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() {
            return stopped_ || !records_.empty();
          });
        if (records_.empty() && stopped_) {
          return;
        }
        record = std::move(records_.front());
        records_.pop_front();
      }
      if (record && record->delivery) {
        try {
          record->delivery(record->token, record->result);
        } catch (...) {
          // The relay remains available for the next immutable record.
        }
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++delivered_;
        last_delivery_thread_ = std::this_thread::get_id();
      }
      condition_.notify_all();
    }
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<RelativeMotionCompletionRecordPtr> records_;
  std::thread worker_;
  std::size_t delivered_{0U};
  std::thread::id last_delivery_thread_{};
  bool stopped_{false};
};

std::shared_ptr<ExternalCompletionRelay> install_completion_relay(
  MotionConditioningConfig & config)
{
  auto relay = std::make_shared<ExternalCompletionRelay>();
  config.completion_relay = [relay](RelativeMotionCompletionRecordPtr record) {
      return relay->publish(std::move(record));
    };
  return relay;
}

template<typename PublisherT>
bool wait_for_subscription(
  rclcpp::Node & node,
  const PublisherT & publisher)
{
  const auto graph_event = node.get_graph_event();
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (publisher->get_subscription_count() == 0U &&
    std::chrono::steady_clock::now() < deadline)
  {
    node.wait_for_graph_change(
      graph_event,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline - std::chrono::steady_clock::now()));
    graph_event->check_and_clear();
  }
  return publisher->get_subscription_count() != 0U;
}

struct OdomPublisher
{
  using Message = nav_msgs::msg::Odometry;
  using Publisher = rclcpp::Publisher<Message>;

  auto create_publisher(rclcpp::Node & node, const MotionConditioningConfig & config)
  {
    return node.create_publisher<Message>(config.odom_topic, rclcpp::SensorDataQoS());
  }

  void send(Publisher & publisher)
  {
    Message message;
    message.pose.pose.orientation.w = 1.0;
    publisher.publish(message);
  }
};

struct ScanPublisher
{
  using Message = sensor_msgs::msg::LaserScan;
  using Publisher = rclcpp::Publisher<Message>;

  auto create_publisher(rclcpp::Node & node, const MotionConditioningConfig & config)
  {
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    return node.create_publisher<Message>(config.scan_topic, qos);
  }

  void send(Publisher & publisher)
  {
    publisher.publish(Message{});
  }
};

struct ClockPublisher
{
  using Message = rosgraph_msgs::msg::Clock;
  using Publisher = rclcpp::Publisher<Message>;

  auto create_publisher(rclcpp::Node & node, const MotionConditioningConfig & config)
  {
    return node.create_publisher<Message>(config.clock_topic, rclcpp::ClockQoS());
  }

  void send(Publisher & publisher)
  {
    static std::int32_t stamp = 1;
    Message message;
    message.clock.sec = stamp++;
    publisher.publish(message);
  }
};

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

template<typename Publish>
void run_source_shutdown_barrier(
  const std::string & suffix,
  std::function<void(MotionConditioningConfig &, CallbackBarrier &)> configure,
  Publish publish)
{
  auto node = std::make_shared<rclcpp::Node>("relative_motion_adapter_" + suffix);
  auto authority = std::make_shared<BlockingAuthority>();
  MotionConditioningConfig conditioning_config;
  conditioning_config.odom_topic = "/adapter_barrier/" + suffix + "/odom";
  conditioning_config.scan_topic = "/adapter_barrier/" + suffix + "/scan";
  conditioning_config.clock_topic = "/adapter_barrier/" + suffix + "/clock";
  conditioning_config.component_rpc_timeout = 100ms;
  conditioning_config.prepare_open_deadline = 1s;
  conditioning_config.stop_barrier = 100ms;
  install_completion_relay(conditioning_config);
  CallbackBarrier callback_barrier;
  configure(conditioning_config, callback_barrier);
  RelativeMotionRosAdapter adapter(*node, authority, {}, conditioning_config);

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 1U);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  auto publisher = publish.create_publisher(*node, conditioning_config);
  ASSERT_TRUE(wait_for_subscription(*node, publisher));
  publish.send(*publisher);
  ASSERT_TRUE(callback_barrier.wait_for_entries(1U));

  // The single executor thread leaves the second message queued behind the
  // blocked callback. Shutdown closes ingress before that queued callback can
  // reach the Impl handler.
  publish.send(*publisher);

  std::atomic<bool> shutdown_finished{false};
  std::thread shutdown_thread([&]() {
      adapter.shutdown();
      shutdown_finished.store(true);
    });
  ASSERT_TRUE(callback_barrier.wait_for_shutdown_wait());
  EXPECT_FALSE(shutdown_finished.load());
  callback_barrier.release();
  shutdown_thread.join();

  EXPECT_EQ(callback_barrier.entries(), 1U);
  executor.cancel();
  spin_thread.join();
}

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
  install_completion_relay(conditioning_config);
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
  install_completion_relay(conditioning_config);
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

TEST_F(
  RelativeMotionRosAdapterTest,
  BeginShutdownWaitsForInternalResultBeforeFinalize)
{
  auto node = std::make_shared<rclcpp::Node>("relative_motion_adapter_phases");
  auto authority = std::make_shared<BlockingAuthority>();
  MotionConditioningConfig conditioning_config;
  conditioning_config.component_rpc_timeout = 100ms;
  conditioning_config.prepare_open_deadline = 1s;
  conditioning_config.stop_barrier = 100ms;
  install_completion_relay(conditioning_config);
  RelativeMotionRosAdapter adapter(*node, authority, {}, conditioning_config);

  CallbackBarrier result_barrier;
  std::atomic<std::size_t> result_count{0U};
  const MotionToken token{19U, 3U, 7U, 1U};
  adapter.start(
    token,
    MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.1F, 0.0F, {}},
    {},
    [&result_count, &result_barrier](const MotionToken &, const ChildResult &) {
      result_count.fetch_add(1U);
      result_barrier.enter();
    });
  ASSERT_TRUE(authority->wait_for_prepare());

  adapter.begin_shutdown();
  authority->release_prepare();
  ASSERT_TRUE(result_barrier.wait_for_entries(1U));

  std::promise<void> finalize_promise;
  auto finalize_future = finalize_promise.get_future();
  std::thread finalize_thread([&]() {
      adapter.finalize_shutdown();
      finalize_promise.set_value();
    });
  EXPECT_EQ(finalize_future.wait_for(0ms), std::future_status::timeout);

  result_barrier.release();
  ASSERT_EQ(
    finalize_future.wait_for(2s),
    std::future_status::ready);
  finalize_thread.join();

  EXPECT_EQ(result_count.load(), 1U);
  adapter.finalize_shutdown();
  adapter.shutdown();
}

TEST_F(
  RelativeMotionRosAdapterTest,
  StartDuringQuiesceFailsClosedAndHealthyIsFalse)
{
  auto node = std::make_shared<rclcpp::Node>("relative_motion_adapter_quiesce");
  auto authority = std::make_shared<BlockingAuthority>();
  MotionConditioningConfig conditioning_config;
  conditioning_config.component_rpc_timeout = 100ms;
  conditioning_config.prepare_open_deadline = 1s;
  conditioning_config.stop_barrier = 100ms;
  install_completion_relay(conditioning_config);
  RelativeMotionRosAdapter adapter(*node, authority, {}, conditioning_config);

  adapter.begin_shutdown();
  EXPECT_FALSE(adapter.healthy());
  std::atomic<std::size_t> result_count{0U};
  std::atomic<ChildResultCode> result_code{ChildResultCode::Failed};
  adapter.start(
    MotionToken{20U, 3U, 8U, 1U},
    MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.1F, 0.0F, {}},
    {},
    [&result_count, &result_code](const MotionToken &, const ChildResult & result) {
      result_code.store(result.code);
      result_count.fetch_add(1U);
    });

  EXPECT_EQ(result_count.load(), 1U);
  EXPECT_EQ(result_code.load(), ChildResultCode::SafetyFault);
  adapter.finalize_shutdown();
}

TEST_F(RelativeMotionRosAdapterTest, PortStartRejectsRotatedAdmissionEpoch)
{
  auto node = std::make_shared<rclcpp::Node>("relative_motion_adapter_fence");
  auto authority = std::make_shared<BlockingAuthority>();
  MotionConditioningConfig conditioning_config;
  conditioning_config.admission_fence_check = [](const std::uint64_t) {
      return false;
    };
  install_completion_relay(conditioning_config);
  RelativeMotionRosAdapter adapter(*node, authority, {}, conditioning_config);

  std::atomic<std::size_t> result_count{0U};
  std::atomic<ChildResultCode> result_code{ChildResultCode::Failed};
  adapter.start(
    MotionToken{22U, 4U, 10U, 1U},
    MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.1F, 0.0F, {}},
    {},
    [&result_count, &result_code](const MotionToken &, const ChildResult & result) {
      result_code.store(result.code);
      result_count.fetch_add(1U);
    });

  EXPECT_EQ(result_count.load(), 1U);
  EXPECT_EQ(result_code.load(), ChildResultCode::SafetyFault);
  EXPECT_EQ(authority->inhibit_count(), 0U);
}

TEST_F(
  RelativeMotionRosAdapterTest,
  ExternalRelayStartFailureMayReleaseLastAdapterOwner)
{
  auto node = std::make_shared<rclcpp::Node>(
    "relative_motion_adapter_external_start_failure");
  auto authority = std::make_shared<BlockingAuthority>();
  MotionConditioningConfig conditioning_config;
  conditioning_config.component_rpc_timeout = 100ms;
  conditioning_config.prepare_open_deadline = 1s;
  conditioning_config.stop_barrier = 100ms;
  auto relay = install_completion_relay(conditioning_config);
  auto adapter = std::make_shared<RelativeMotionRosAdapter>(
    *node, authority, RelativeMotionPolicy{}, conditioning_config);
  const auto weak_adapter = std::weak_ptr<RelativeMotionRosAdapter>(adapter);
  std::atomic<std::size_t> terminal_count{0U};
  adapter->start(
    MotionToken{23U, 5U, 11U, 1U},
    MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.1F, 0.0F, {}},
    {},
    [held = adapter, &terminal_count](const MotionToken &, const ChildResult &) mutable {
      ++terminal_count;
      held.reset();
    });
  ASSERT_TRUE(authority->wait_for_prepare());
  authority->release_prepare();
  adapter.reset();

  ASSERT_TRUE(relay->wait_for_deliveries(1U));
  EXPECT_EQ(terminal_count.load(), 1U);
  EXPECT_EQ(relay->deliveries(), 1U);
  EXPECT_NE(relay->last_delivery_thread(), std::this_thread::get_id());
  EXPECT_TRUE(weak_adapter.expired());
}

TEST_F(
  RelativeMotionRosAdapterTest,
  ExternalRelayEmergencyMayReleaseLastAdapterOwner)
{
  auto node = std::make_shared<rclcpp::Node>(
    "relative_motion_adapter_external_emergency");
  auto authority = std::make_shared<BlockingAuthority>();
  MotionConditioningConfig conditioning_config;
  conditioning_config.component_rpc_timeout = 100ms;
  conditioning_config.prepare_open_deadline = 1s;
  conditioning_config.stop_barrier = 100ms;
  auto relay = install_completion_relay(conditioning_config);
  auto adapter = std::make_shared<RelativeMotionRosAdapter>(
    *node, authority, RelativeMotionPolicy{}, conditioning_config);
  const auto weak_adapter = std::weak_ptr<RelativeMotionRosAdapter>(adapter);
  std::atomic<std::size_t> terminal_count{0U};
  adapter->start(
    MotionToken{24U, 5U, 12U, 1U},
    MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.1F, 0.0F, {}},
    {},
    [held = adapter, &terminal_count](const MotionToken &, const ChildResult &) mutable {
      ++terminal_count;
      held.reset();
    });
  ASSERT_TRUE(authority->wait_for_prepare());

  adapter->begin_shutdown();
  authority->release_prepare();
  adapter.reset();

  ASSERT_TRUE(relay->wait_for_deliveries(1U));
  EXPECT_EQ(terminal_count.load(), 1U);
  EXPECT_EQ(relay->deliveries(), 1U);
  EXPECT_NE(relay->last_delivery_thread(), std::this_thread::get_id());
  EXPECT_TRUE(weak_adapter.expired());
}

TEST_F(
  RelativeMotionRosAdapterTest,
  MultiThreadedExecutorDrainsQueuedAndInflightOdomIngress)
{
  run_source_shutdown_barrier(
    "odom",
    [](MotionConditioningConfig & config, CallbackBarrier & barrier) {
      config.before_adapter_odom_callback = [&barrier]() {barrier.enter();};
      config.before_adapter_ingress_wait = [&barrier]() {
        barrier.mark_shutdown_wait();
      };
    },
    OdomPublisher{});
}

TEST_F(
  RelativeMotionRosAdapterTest,
  MultiThreadedExecutorDrainsQueuedAndInflightScanIngress)
{
  run_source_shutdown_barrier(
    "scan",
    [](MotionConditioningConfig & config, CallbackBarrier & barrier) {
      config.before_adapter_scan_callback = [&barrier]() {barrier.enter();};
      config.before_adapter_ingress_wait = [&barrier]() {
        barrier.mark_shutdown_wait();
      };
    },
    ScanPublisher{});
}

TEST_F(
  RelativeMotionRosAdapterTest,
  MultiThreadedExecutorDrainsQueuedAndInflightClockIngress)
{
  run_source_shutdown_barrier(
    "clock",
    [](MotionConditioningConfig & config, CallbackBarrier & barrier) {
      config.before_adapter_clock_callback = [&barrier]() {barrier.enter();};
      config.before_adapter_ingress_wait = [&barrier]() {
        barrier.mark_shutdown_wait();
      };
    },
    ClockPublisher{});
}

TEST_F(
  RelativeMotionRosAdapterTest,
  MultiThreadedExecutorDrainsRawTimerAndCommandSupplier)
{
  auto node = std::make_shared<rclcpp::Node>("relative_motion_adapter_raw_barrier");
  auto authority = std::make_shared<BlockingAuthority>();
  MotionConditioningConfig conditioning_config;
  conditioning_config.component_rpc_timeout = 100ms;
  conditioning_config.prepare_open_deadline = 1s;
  conditioning_config.stop_barrier = 100ms;
  CallbackBarrier command_barrier;
  conditioning_config.before_adapter_command_supplier = [&command_barrier]() {
      command_barrier.enter();
    };
  conditioning_config.before_adapter_ingress_wait = [&command_barrier]() {
      command_barrier.mark_shutdown_wait();
    };
  install_completion_relay(conditioning_config);
  RelativeMotionRosAdapter adapter(*node, authority, {}, conditioning_config);

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  ASSERT_TRUE(detail::RelativeMotionRosAdapterTestAccess::start_raw_producer(
    adapter, "/adapter_barrier/raw_command"));
  ASSERT_TRUE(command_barrier.wait_for_entries(1U));

  std::atomic<bool> shutdown_finished{false};
  std::thread shutdown_thread([&]() {
      adapter.shutdown();
      shutdown_finished.store(true);
    });
  ASSERT_TRUE(command_barrier.wait_for_shutdown_wait());
  EXPECT_FALSE(shutdown_finished.load());
  command_barrier.release();
  shutdown_thread.join();

  EXPECT_EQ(command_barrier.entries(), 1U);
  executor.cancel();
  spin_thread.join();
}

TEST_F(
  RelativeMotionRosAdapterTest,
  DestructorWithoutExplicitShutdownDrainsActiveExecutorCallbacks)
{
  auto node = std::make_shared<rclcpp::Node>(
    "relative_motion_adapter_implicit_shutdown");
  auto authority = std::make_shared<BlockingAuthority>();
  MotionConditioningConfig conditioning_config;
  conditioning_config.odom_topic = "/adapter_implicit/odom";
  conditioning_config.scan_topic = "/adapter_implicit/scan";
  conditioning_config.clock_topic = "/adapter_implicit/clock";
  conditioning_config.component_rpc_timeout = 100ms;
  conditioning_config.prepare_open_deadline = 1s;
  conditioning_config.stop_barrier = 100ms;
  CallbackBarrier odom_barrier;
  CallbackBarrier scan_barrier;
  CallbackBarrier clock_barrier;
  CallbackBarrier command_barrier;
  CallbackBarrier ingress_barrier;
  conditioning_config.before_adapter_odom_callback = [&odom_barrier]() {
      odom_barrier.enter();
    };
  conditioning_config.before_adapter_scan_callback = [&scan_barrier]() {
      scan_barrier.enter();
    };
  conditioning_config.before_adapter_clock_callback = [&clock_barrier]() {
      clock_barrier.enter();
    };
  conditioning_config.before_adapter_command_supplier = [&command_barrier]() {
      command_barrier.enter();
    };
  conditioning_config.before_adapter_ingress_wait = [&ingress_barrier]() {
      ingress_barrier.mark_shutdown_wait();
    };
  install_completion_relay(conditioning_config);

  auto adapter = std::make_unique<RelativeMotionRosAdapter>(
    *node, authority, RelativeMotionPolicy{}, conditioning_config);
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  auto odom_publisher = OdomPublisher{}.create_publisher(*node, conditioning_config);
  auto scan_publisher = ScanPublisher{}.create_publisher(*node, conditioning_config);
  auto clock_publisher = ClockPublisher{}.create_publisher(*node, conditioning_config);
  ASSERT_TRUE(wait_for_subscription(*node, odom_publisher));
  ASSERT_TRUE(wait_for_subscription(*node, scan_publisher));
  ASSERT_TRUE(wait_for_subscription(*node, clock_publisher));
  ASSERT_TRUE(detail::RelativeMotionRosAdapterTestAccess::start_raw_producer(
      *adapter, "/adapter_implicit/raw"));
  ASSERT_TRUE(command_barrier.wait_for_entries(1U));

  std::mutex result_mutex;
  std::condition_variable result_condition;
  std::size_t result_count = 0U;
  const MotionToken token{21U, 3U, 9U, 1U};
  adapter->start(
    token,
    MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance), 0.1F, 0.0F, {}},
    {},
    [&](const MotionToken &, const ChildResult &) {
      std::lock_guard<std::mutex> lock(result_mutex);
      ++result_count;
      result_condition.notify_all();
    });
  ASSERT_TRUE(authority->wait_for_prepare());

  OdomPublisher{}.send(*odom_publisher);
  ScanPublisher{}.send(*scan_publisher);
  ClockPublisher{}.send(*clock_publisher);
  ASSERT_TRUE(odom_barrier.wait_for_entries(1U));
  ASSERT_TRUE(scan_barrier.wait_for_entries(1U));
  ASSERT_TRUE(clock_barrier.wait_for_entries(1U));
  OdomPublisher{}.send(*odom_publisher);
  ScanPublisher{}.send(*scan_publisher);
  ClockPublisher{}.send(*clock_publisher);

  auto owned_adapter = std::move(adapter);
  std::promise<void> destructor_promise;
  auto destructor_future = destructor_promise.get_future();
  std::thread destructor_thread(
    [owned = std::move(owned_adapter), &destructor_promise]() mutable {
      owned.reset();
      destructor_promise.set_value();
    });

  authority->release_prepare();
  ASSERT_TRUE(ingress_barrier.wait_for_shutdown_wait());
  EXPECT_EQ(destructor_future.wait_for(0ms), std::future_status::timeout);
  {
    std::unique_lock<std::mutex> lock(result_mutex);
    ASSERT_TRUE(result_condition.wait_for(lock, 2s, [&]() {
        return result_count == 1U;
      }));
  }

  command_barrier.release();
  odom_barrier.release();
  scan_barrier.release();
  clock_barrier.release();
  ASSERT_EQ(
    destructor_future.wait_for(2s),
    std::future_status::ready);
  destructor_thread.join();

  executor.cancel();
  spin_thread.join();

  EXPECT_EQ(result_count, 1U);
  EXPECT_EQ(odom_barrier.entries(), 1U);
  EXPECT_EQ(scan_barrier.entries(), 1U);
  EXPECT_EQ(clock_barrier.entries(), 1U);
  EXPECT_EQ(command_barrier.entries(), 1U);
}

}  // namespace
}  // namespace voice_nav_mission
