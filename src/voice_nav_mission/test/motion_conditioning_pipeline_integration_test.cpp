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
#include <memory>
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

#include "voice_nav_mission/motion_authority_ros_adapter.hpp"
#include "voice_nav_mission/motion_conditioning_pipeline.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using ListControllers = controller_manager_msgs::srv::ListControllers;

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
  EXPECT_EQ(controller->nonzero_count.load(), first_stop_count);

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

}  // namespace
}  // namespace voice_nav_mission
