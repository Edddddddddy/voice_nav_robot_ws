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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#define main motion_gate_embedded_main
#include "../src/motion_gate_node.cpp"  // NOLINT(build/include)
#undef main

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp_components/component_manager.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "voice_nav_mission/msg/internal_motion_gate_state.hpp"
#include "voice_nav_mission/motion_authority_ros_adapter.hpp"
#include "voice_nav_mission/motion_conditioning_pipeline.hpp"
#include "voice_nav_mission/srv/internal_motion_gate_control.hpp"
#include "voice_nav_mission/relative_motion_ros_adapter.hpp"
#include "voice_nav_mission/runtime_event_queue.hpp"
#include "voice_nav_mission/runtime_execution_plane.hpp"
#include "voice_nav_mission/runtime_shutdown_coordinator.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using Odometry = nav_msgs::msg::Odometry;
using LaserScan = sensor_msgs::msg::LaserScan;
using Clock = rosgraph_msgs::msg::Clock;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using GateControl = voice_nav_mission::srv::InternalMotionGateControl;
using GateStateMessage = voice_nav_mission::msg::InternalMotionGateState;
constexpr char kIntegrationRuntimeId[] =
  "0123456789abcdef0123456789abcdef";

class SyntheticProducer final : public MotionProducerPort
{
public:
  explicit SyntheticProducer(rclcpp::Node & node)
  : node_(node) {}

  bool start(const std::string & raw_topic) override
  {
    publisher_ = node_.create_publisher<TwistStamped>(
      raw_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
    timer_ = node_.create_wall_timer(50ms, [this]() {
          if (!publisher_) {
            return;
          }
          TwistStamped message;
          message.header.stamp = node_.get_clock()->now();
          message.header.frame_id = "base_footprint";
          message.twist.linear.x = 0.12;
          publisher_->publish(message);
    });
    ++start_count;
    return true;
  }

  void stop() override
  {
    timer_.reset();
    publisher_.reset();
    ++stop_count;
  }

  std::size_t start_count{0U};
  std::size_t stop_count{0U};

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<TwistStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class ConditioningGraphNode final : public rclcpp::Node
{
public:
  ConditioningGraphNode()
  : Node(
      "conditioning_graph_test",
      rclcpp::NodeOptions().append_parameter_override("use_sim_time", true))
  {
    clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS());
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS());
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS());
    tf_static_publisher_ = create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    tf2_msgs::msg::TFMessage static_transforms;
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.frame_id = "odom";
    odom_to_base.child_frame_id = "base_footprint";
    odom_to_base.transform.rotation.w = 1.0;
    static_transforms.transforms.push_back(odom_to_base);
    tf_static_publisher_->publish(static_transforms);

    controller_service_ = create_service<ListControllers>(
      "/controller_manager/list_controllers",
      [](
        const std::shared_ptr<ListControllers::Request>,
        std::shared_ptr<ListControllers::Response> response) {
        controller_manager_msgs::msg::ControllerState controller;
        controller.name = "diff_drive_controller";
        controller.state = "active";
        response->controller = {controller};
      });

    clock_timer_ = create_wall_timer(10ms, [this]() {
          rosgraph_msgs::msg::Clock message;
          message.clock = rclcpp::Clock(RCL_SYSTEM_TIME).now();
          clock_publisher_->publish(message);
    });
    sensor_timer_ = create_wall_timer(50ms, [this]() {
          const auto stamp = get_clock()->now();
          sensor_msgs::msg::LaserScan scan;
          scan.header.stamp = stamp;
          scan.header.frame_id = "base_footprint";
          scan.angle_min = -1.0F;
          scan.angle_max = 1.0F;
          scan.angle_increment = 1.0F;
          scan.range_min = 0.05F;
          scan.range_max = 10.0F;
          scan.ranges = {10.0F, 10.0F, 10.0F};
          scan_publisher_->publish(scan);

          nav_msgs::msg::Odometry odom;
          odom.header.stamp = stamp;
          odom.header.frame_id = "odom";
          odom.child_frame_id = "base_footprint";
          odom_publisher_->publish(odom);
    });
  }

private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_publisher_;
  rclcpp::Service<ListControllers>::SharedPtr controller_service_;
  rclcpp::TimerBase::SharedPtr clock_timer_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;
};

class ControllerObservationNode final : public rclcpp::Node
{
public:
  ControllerObservationNode()
  : Node("diff_drive_controller")
  {
    subscription_ = create_subscription<TwistStamped>(
      "/diff_drive_controller/cmd_vel",
      rclcpp::SystemDefaultsQoS(),
      [this](const TwistStamped::ConstSharedPtr message) {
        if (std::abs(message->twist.linear.x) > 0.001 ||
        std::abs(message->twist.angular.z) > 0.001)
        {
          ++nonzero_count;
        }
        if (message->twist.linear.x > 0.25) {
          ++high_velocity_count;
        }
      });
  }

  std::atomic<std::size_t> nonzero_count{0U};
  std::atomic<std::size_t> high_velocity_count{0U};

private:
  rclcpp::Subscription<TwistStamped>::SharedPtr subscription_;
};

class CountLatch final
{
public:
  void observe()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++count_;
    condition_.notify_all();
  }

  [[nodiscard]] bool wait_for_at_least(
    const std::size_t expected,
    const std::chrono::steady_clock::duration timeout = 3s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this, expected]() {
               return count_ >= expected;
           });
  }

  [[nodiscard]] std::size_t value() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return count_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::size_t count_{0U};
};

class TerminalProbe final
{
public:
  void record(const MissionResult & result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    results_.push_back(result);
    condition_.notify_all();
  }

  [[nodiscard]] bool wait_until(
    const std::chrono::steady_clock::time_point deadline)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_until(lock, deadline, [this]() {
               return !results_.empty();
           });
  }

  [[nodiscard]] std::vector<MissionResult> results() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return results_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<MissionResult> results_;
};

class IntegrationSteadyClock final : public SteadyClockPort
{
public:
  [[nodiscard]] TimePoint now() const override
  {
    return std::chrono::steady_clock::now();
  }
};

struct RuntimeCompletionEvent
{
  MotionToken token;
};

class RuntimeCompletionWorker final
{
public:
  using Queue = RuntimeEventQueue<RuntimeCompletionEvent>;

  RuntimeCompletionWorker()
  : queue_([]() {return RuntimeCompletionEvent{MotionToken{}};})
  {
  }

  ~RuntimeCompletionWorker()
  {
    queue_.close();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void start(const std::shared_ptr<RuntimeExecutionPlane> & plane)
  {
    plane_ = plane;
    thread_ = std::thread([this]() {
          RuntimeCompletionEvent event{};
          while (queue_.wait_pop_result(event) == Queue::WaitResult::Item) {
            const auto dispatch = plane_->completion_mailbox().take(event.token);
            if (dispatch.has_value() && dispatch->delivery) {
              dispatch->delivery(dispatch->record->token, dispatch->record->result);
            }
          }
      });
  }

  [[nodiscard]] bool enqueue(const MotionToken & token)
  {
    return queue_.push(
      RuntimeCompletionEvent{token}, Queue::Lane::Control) ==
           Queue::PushResult::Accepted;
  }

private:
  Queue queue_;
  std::shared_ptr<RuntimeExecutionPlane> plane_;
  std::thread thread_;
};

class RclcppGuard final
{
public:
  RclcppGuard()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  ~RclcppGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class SpinGuard final
{
public:
  explicit SpinGuard(
    const std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> & executor)
  : executor_(executor), thread_([this]() {executor_->spin();}) {}

  ~SpinGuard()
  {
    executor_->cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread thread_;
};

template<typename PredicateT>
bool wait_for(PredicateT predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
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

std::vector<std::uint8_t> collision_writer_gid(
  const rclcpp::Node & node,
  const std::string & topic)
{
  const auto endpoints = node.get_publishers_info_by_topic(topic);
  std::vector<std::uint8_t> result;
  for (const auto & endpoint : endpoints) {
    if (endpoint.node_name() == "collision_monitor" &&
      endpoint.topic_type() == "geometry_msgs/msg/TwistStamped")
    {
      const auto & gid = endpoint.endpoint_gid();
      result.assign(gid.cbegin(), gid.cend());
      break;
    }
  }
  return result;
}

TEST(MotionConditioningPipelineIntegration, RealComponentsHandoverTwoLeases)
{
  RclcppGuard rclcpp_guard;
  rclcpp::NodeOptions gate_options;
  gate_options.append_parameter_override("use_sim_time", true);
  gate_options.append_parameter_override("prepare_timeout_ms", 6000);
  gate_options.append_parameter_override("writer_graph_timeout_ms", 1000);
  gate_options.append_parameter_override(
    "expected_candidate_writer_fqn", "/collision_monitor");

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions{}, 8U);
  auto gate = std::make_shared<MotionGateNode>(gate_options);
  auto container = std::make_shared<rclcpp_components::ComponentManager>(
    executor, "motion_conditioning_container",
    rclcpp::NodeOptions().use_intra_process_comms(false));
  auto graph = std::make_shared<ConditioningGraphNode>();
  auto controller = std::make_shared<ControllerObservationNode>();
  auto pipeline_node = std::make_shared<rclcpp::Node>(
    "conditioning_runtime_test",
    rclcpp::NodeOptions().append_parameter_override("use_sim_time", true));

  executor->add_node(gate);
  executor->add_node(container);
  executor->add_node(graph);
  executor->add_node(controller);
  executor->add_node(pipeline_node);
  SpinGuard spin_guard(executor);
  ASSERT_TRUE(wait_for(
      [&graph]() {return graph->get_clock()->ros_time_is_active();}, 1s));
  std::this_thread::sleep_for(250ms);

  auto authority = std::make_shared<RosMotionAuthorityPort>(
    *pipeline_node, 100ms, 250ms, [](const GateSnapshot &) {});
  ASSERT_TRUE(wait_for(
      [&authority]() {
        return authority->snapshot().gate_instance_id.size() == 32U;
      }, 1s));
  auto producer = std::make_shared<SyntheticProducer>(*pipeline_node);
  MotionConditioningConfig config;
  config.component_rpc_timeout = 2s;
  config.writer_graph_timeout = 1s;
  config.prepare_open_deadline = 4s;
  config.renew_period = 100ms;
  MotionConditioningPipeline pipeline(
    *pipeline_node, authority, producer, config);

  const auto prepared_one = pipeline.prepare();
  ASSERT_TRUE(prepared_one.ok) << prepared_one.detail;
  ASSERT_EQ(prepared_one.candidate_topic.rfind(
      "/voice_nav_internal/motion_gate/candidate/lease_", 0U), 0U);
  const auto first_topic = prepared_one.candidate_topic;
  const auto started_one = pipeline.start();
  ASSERT_TRUE(started_one.ok) << started_one.detail;
  ASSERT_TRUE(wait_for(
      [&controller]() {return controller->nonzero_count.load() > 0U;}, 3s));
  const auto first_gid = collision_writer_gid(*pipeline_node, first_topic);
  ASSERT_FALSE(first_gid.empty());
  EXPECT_EQ(producer->start_count, 1U);

  const auto first_stop_count = controller->nonzero_count.load();
  const auto stopped_one = pipeline.stop();
  ASSERT_TRUE(stopped_one.ok) << stopped_one.detail;
  EXPECT_TRUE(wait_for(
      [&pipeline_node, &first_topic]() {
        return pipeline_node->get_publishers_info_by_topic(first_topic).empty();
      }, 2s));
  const auto after_stop_count = controller->nonzero_count.load();
  EXPECT_GE(after_stop_count, first_stop_count);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(controller->nonzero_count.load(), after_stop_count);

  const auto prepared_two = pipeline.prepare();
  ASSERT_TRUE(prepared_two.ok) << prepared_two.detail;
  EXPECT_NE(prepared_two.candidate_topic, first_topic);
  const auto second_generation_baseline = controller->nonzero_count.load();
  const auto started_two = pipeline.start();
  ASSERT_TRUE(started_two.ok) << started_two.detail;
  ASSERT_TRUE(wait_for(
      [&controller, second_generation_baseline]() {
        return controller->nonzero_count.load() > second_generation_baseline;
      }, 3s));
  const auto second_gid = collision_writer_gid(
    *pipeline_node, prepared_two.candidate_topic);
  ASSERT_FALSE(second_gid.empty());
  EXPECT_NE(second_gid, first_gid);
  EXPECT_EQ(producer->start_count, 2U);
  EXPECT_EQ(
    pipeline_node->get_subscriptions_info_by_topic(first_topic).size(), 0U);

  auto late_old_publisher = pipeline_node->create_publisher<TwistStamped>(
    first_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
  TwistStamped late_message;
  late_message.header.stamp = pipeline_node->get_clock()->now();
  late_message.header.frame_id = "base_footprint";
  late_message.twist.linear.x = 0.4;
  const auto late_sample_baseline = controller->high_velocity_count.load();
  late_old_publisher->publish(late_message);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(controller->high_velocity_count.load(), late_sample_baseline);
  EXPECT_EQ(
    pipeline_node->get_subscriptions_info_by_topic(first_topic).size(), 0U);
  late_old_publisher.reset();

  const auto stopped_two = pipeline.stop();
  ASSERT_TRUE(stopped_two.ok) << stopped_two.detail;
  EXPECT_TRUE(wait_for(
      [&pipeline_node, &prepared_two]() {
        return pipeline_node->get_publishers_info_by_topic(
          prepared_two.candidate_topic).empty();
      }, 2s));
}

TEST(
  MotionConditioningPipelineIntegration,
  ProductionAdapterShutdownRetainsStationarityOdomAndFreezesNodeCounters)
{
  RclcppGuard rclcpp_guard;
  rclcpp::NodeOptions gate_options;
  gate_options.append_parameter_override("use_sim_time", true);
  gate_options.append_parameter_override("prepare_timeout_ms", 6000);
  gate_options.append_parameter_override("writer_graph_timeout_ms", 1000);
  gate_options.append_parameter_override(
    "expected_candidate_writer_fqn", "/collision_monitor");

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions{}, 16U);
  auto gate = std::make_shared<MotionGateNode>(gate_options);
  auto container = std::make_shared<rclcpp_components::ComponentManager>(
    executor, "motion_conditioning_container",
    rclcpp::NodeOptions().use_intra_process_comms(false));
  auto graph = std::make_shared<ConditioningGraphNode>();
  auto controller = std::make_shared<ControllerObservationNode>();
  auto pipeline_node = std::make_shared<rclcpp::Node>(
    "production_adapter_shutdown_test",
    rclcpp::NodeOptions().append_parameter_override("use_sim_time", true));

  CountLatch gate_updates;
  auto authority = std::make_shared<RosMotionAuthorityPort>(
    *pipeline_node, 100ms, 250ms,
    [&gate_updates](const GateSnapshot &) {gate_updates.observe();});

  CountLatch adapter_odom;
  CountLatch adapter_scan;
  CountLatch adapter_clock;
  CountLatch renew_callbacks;
  CountLatch raw_output;
  CountLatch core_running;
  TerminalProbe terminal;
  std::shared_ptr<RuntimeExecutionPlane> plane;
  std::atomic<bool> admission_closed{false};
  std::atomic<std::size_t> emergency_count{0U};

  MotionConditioningConfig conditioning_config;
  conditioning_config.component_rpc_timeout = 2s;
  conditioning_config.writer_graph_timeout = 1s;
  conditioning_config.prepare_open_deadline = 4s;
  conditioning_config.stop_barrier = 250ms;
  conditioning_config.before_adapter_odom_callback = [&adapter_odom]() {
      adapter_odom.observe();
    };
  conditioning_config.before_adapter_scan_callback = [&adapter_scan]() {
      adapter_scan.observe();
    };
  conditioning_config.before_adapter_clock_callback = [&adapter_clock]() {
      adapter_clock.observe();
    };
  conditioning_config.before_renew_callback = [&renew_callbacks]() {
      renew_callbacks.observe();
    };
  conditioning_config.completion_relay = [&plane](
    RelativeMotionCompletionRecordPtr record) {
      return plane && plane->completion_mailbox().relay(std::move(record));
    };

  auto adapter = std::make_shared<RelativeMotionRosAdapter>(
    *pipeline_node, authority, RelativeMotionPolicy{}, conditioning_config);
  auto raw_subscription = pipeline_node->create_subscription<TwistStamped>(
    conditioning_config.raw_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [&raw_output](const TwistStamped::ConstSharedPtr) {
      raw_output.observe();
    });

  RuntimeConfig runtime_config;
  runtime_config.runtime_instance_id = kIntegrationRuntimeId;
  runtime_config.initial_admission_epoch = 1U;
  RuntimeCompletionWorker completion_worker;
  plane = std::shared_ptr<RuntimeExecutionPlane>(new RuntimeExecutionPlane(
    runtime_config,
    std::make_shared<IntegrationSteadyClock>(),
    authority,
    adapter,
        [&core_running](const RuntimeState & state) {
          if (state.active_step != kNoActiveMissionStep) {
            core_running.observe();
          }
    },
        {},
        [&terminal](std::uint64_t, const MissionResult & result) {
          terminal.record(result);
    },
        {},
        [&admission_closed](std::uint64_t) {
          return !admission_closed.load();
    },
        [&completion_worker](const MotionToken & token) {
          return completion_worker.enqueue(token);
    },
        [&emergency_count](std::string) {
          emergency_count.fetch_add(1U);
    }));
  completion_worker.start(plane);

  executor->add_node(gate);
  executor->add_node(container);
  executor->add_node(graph);
  executor->add_node(controller);
  executor->add_node(pipeline_node);
  SpinGuard spin_guard(executor);

  ASSERT_TRUE(gate_updates.wait_for_at_least(1U));
  ASSERT_TRUE(adapter_odom.wait_for_at_least(1U));
  ASSERT_TRUE(adapter_scan.wait_for_at_least(1U));
  ASSERT_TRUE(adapter_clock.wait_for_at_least(1U));
  {
    std::lock_guard<std::recursive_mutex> lock(plane->core_serial_mutex());
    plane->core()->on_tick();
  }

  MissionGoal goal;
  goal.source_instance_id = "production-adapter-shutdown";
  goal.source_seq = 1U;
  goal.runtime_instance_id = kIntegrationRuntimeId;
  goal.admission_epoch = 1U;
  goal.steps.push_back(MissionStep{
        static_cast<std::uint8_t>(MissionStepKind::MoveDistance),
        0.5F, 0.0F, {}});
  AdmissionResult admission;
  {
    std::lock_guard<std::recursive_mutex> lock(plane->core_serial_mutex());
    admission = plane->core()->admit(goal);
  }
  ASSERT_TRUE(admission.accepted) << admission.result.detail;
  ASSERT_TRUE(core_running.wait_for_at_least(1U));
  ASSERT_TRUE(renew_callbacks.wait_for_at_least(1U));
  ASSERT_TRUE(raw_output.wait_for_at_least(1U));

  RuntimeShutdownCoordinator coordinator(
    [&admission_closed]() {
      admission_closed.store(true);
      return true;
    },
    [&adapter](const RuntimeShutdownCoordinator::TimePoint deadline) {
      detail::begin_relative_motion_shutdown(*adapter, deadline);
    },
    [&adapter, &authority, &plane, &terminal](
      const RuntimeShutdownCoordinator::TimePoint deadline) {
      if (!detail::wait_for_relative_motion_internal_completion(
          *adapter, deadline) || !terminal.wait_until(deadline))
      {
        return false;
      }
      const auto snapshot = authority->snapshot();
      bool core_idle = false;
      {
        std::lock_guard<std::recursive_mutex> lock(plane->core_serial_mutex());
        core_idle = !plane->core()->has_active_mission();
      }
      return adapter->zero_proven() &&
             snapshot.state == GateState::Inhibited &&
             snapshot.motion_inhibited && snapshot.zero_selected &&
             snapshot.zero_published && core_idle;
    },
    []() {},
    [](std::string) {},
    [](std::string) {});

  const auto shutdown_outcome = coordinator.run(
    std::chrono::steady_clock::now() + 1450ms);
  ASSERT_TRUE(shutdown_outcome.transaction_drained);
  EXPECT_FALSE(shutdown_outcome.fail_closed);
  EXPECT_EQ(emergency_count.load(), 0U);
  ASSERT_TRUE(adapter->zero_proven());
  const auto raw_at_barrier = raw_output.value();
  const auto renew_at_barrier = renew_callbacks.value();
  const auto running_at_barrier = core_running.value();
  const auto terminal_at_barrier = terminal.results().size();
  const auto control_seq_at_barrier = authority->snapshot().control_seq;

  auto odom_publisher = pipeline_node->create_publisher<Odometry>(
    conditioning_config.odom_topic, rclcpp::SensorDataQoS());
  auto scan_publisher = pipeline_node->create_publisher<LaserScan>(
    conditioning_config.scan_topic, rclcpp::SensorDataQoS());
  auto clock_publisher = pipeline_node->create_publisher<Clock>(
    conditioning_config.clock_topic, rclcpp::ClockQoS());
  ASSERT_TRUE(wait_for_subscription(*pipeline_node, odom_publisher));
  ASSERT_TRUE(wait_for_subscription(*pipeline_node, scan_publisher));
  ASSERT_TRUE(wait_for_subscription(*pipeline_node, clock_publisher));

  Odometry static_odom;
  static_odom.pose.pose.orientation.w = 1.0;
  odom_publisher->publish(static_odom);
  scan_publisher->publish(LaserScan{});
  Clock clock_message;
  clock_message.clock = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  clock_publisher->publish(clock_message);
  ASSERT_TRUE(adapter_odom.wait_for_at_least(2U));

  EXPECT_EQ(raw_output.value(), raw_at_barrier);
  EXPECT_EQ(renew_callbacks.value(), renew_at_barrier);
  EXPECT_EQ(core_running.value(), running_at_barrier);
  EXPECT_EQ(terminal.results().size(), terminal_at_barrier);
  EXPECT_EQ(authority->snapshot().control_seq, control_seq_at_barrier);
  bool core_idle_after_barrier = false;
  {
    std::lock_guard<std::recursive_mutex> lock(plane->core_serial_mutex());
    core_idle_after_barrier = !plane->core()->has_active_mission();
  }
  EXPECT_TRUE(core_idle_after_barrier);

  adapter->finalize_shutdown();
  plane->shutdown();
}

TEST(MotionConditioningPipelineIntegration, RosAuthorityRetriesWithinOverallDeadline)
{
  RclcppGuard rclcpp_guard;
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions{}, 4U);
  auto server = std::make_shared<rclcpp::Node>("motion_gate_retry_server");
  auto client = std::make_shared<rclcpp::Node>("motion_gate_retry_client");
  auto callback_group = server->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  auto state_publisher = server->create_publisher<GateStateMessage>(
    "/motion_gate/internal/state",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  std::atomic<std::size_t> service_calls{0U};
  auto service = server->create_service<GateControl>(
    "/motion_gate/internal/control",
    [&service_calls](
      const std::shared_ptr<GateControl::Request> request,
      std::shared_ptr<GateControl::Response> response) {
      const auto call = ++service_calls;
      if (call == 1U) {
        std::this_thread::sleep_for(140ms);
      }
      response->code = GateControl::Response::APPLIED;
      response->reason = GateControl::Response::NONE;
      response->gate_instance_id = request->gate_instance_id;
      response->control_seq = request->expected_control_seq;
      response->lease_id = request->lease_id;
      response->state = GateStateMessage::ARMED;
      response->candidate_topic = "/candidate/retry";
      response->motion_inhibited = false;
      response->authority_live = true;
      response->candidate_fresh = true;
      response->writer_bound = true;
      response->zero_selected = false;
      response->zero_published = false;
      response->detail = "retry applied";
    },
    rmw_qos_profile_services_default,
    callback_group);
  auto state_timer = server->create_wall_timer(20ms, [state_publisher]() {
        GateStateMessage message;
        message.gate_instance_id = "0123456789abcdef0123456789abcdef";
        message.state_seq = 1U;
        message.control_seq = 7U;
        message.state = GateStateMessage::ARMED;
        message.lease_id = "lease-retry";
        message.candidate_topic = "/candidate/retry";
        message.motion_inhibited = false;
        message.authority_live = true;
        message.candidate_fresh = true;
        message.writer_bound = true;
        state_publisher->publish(message);
      });
  executor->add_node(server);
  executor->add_node(client);
  SpinGuard spin_guard(executor);

  auto authority = std::make_shared<RosMotionAuthorityPort>(
    *client, 100ms, 250ms, [](const GateSnapshot &) {});
  ASSERT_TRUE(wait_for(
      [&authority]() {
        return authority->snapshot().gate_instance_id ==
               "0123456789abcdef0123456789abcdef";
      }, 1s));

  const auto result = authority->renew(AuthorityOperation{
        std::string(32U, 'a'),
        "0123456789abcdef0123456789abcdef",
        7U,
        "lease-retry"});

  EXPECT_TRUE(result.applied);
  EXPECT_FALSE(result.retryable);
  EXPECT_GE(service_calls.load(), 2U);
  (void)service;
  (void)state_timer;
}

}  // namespace
}  // namespace voice_nav_mission
