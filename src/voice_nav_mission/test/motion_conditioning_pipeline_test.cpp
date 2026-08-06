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
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <composition_interfaces/srv/load_node.hpp>
#include <composition_interfaces/srv/unload_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>

#include "voice_nav_mission/mission_authority_convergence.hpp"
#include "voice_nav_mission/motion_conditioning_pipeline.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;
using LoadNode = composition_interfaces::srv::LoadNode;
using UnloadNode = composition_interfaces::srv::UnloadNode;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;

class FakeAuthority final : public MotionAuthorityPort
{
public:
  GateSnapshot snapshot() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

  AuthorityResult prepare(const AuthorityOperation &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back(AuthorityOperationKind::Prepare);
    ++generation_;
    snapshot_.control_seq++;
    snapshot_.lease_id = "lease-" + std::to_string(generation_);
    snapshot_.candidate_topic = "/candidate/lease_" + std::to_string(generation_);
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
    calls_.push_back(AuthorityOperationKind::Open);
    snapshot_.control_seq++;
    snapshot_.state = GateState::Armed;
    snapshot_.motion_inhibited = false;
    snapshot_.zero_selected = false;
    snapshot_.authority_live = true;
    snapshot_.writer_bound = writer_bound_;
    return AuthorityResult{
      true, false, false, snapshot_, snapshot_.lease_id, "opened"};
  }

  AuthorityResult renew(const AuthorityOperation &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back(AuthorityOperationKind::Renew);
    snapshot_.control_seq++;
    snapshot_.authority_live = authority_live_;
    return AuthorityResult{
      true, false, false, snapshot_, snapshot_.lease_id, "renewed"};
  }

  AuthorityResult inhibit(const AuthorityOperation &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back(AuthorityOperationKind::Inhibit);
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

  void set_writer_bound(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    writer_bound_ = value;
  }

  void set_authority_live(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    authority_live_ = value;
  }

  std::vector<AuthorityOperationKind> calls() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

private:
  mutable std::mutex mutex_;
  GateSnapshot snapshot_{
    "gate-test", 1U, {}, GateState::Inhibited, true, true, true, true,
    {}, false, false};
  std::uint64_t generation_{0U};
  bool writer_bound_{true};
  bool authority_live_{true};
  std::vector<AuthorityOperationKind> calls_;
};

class FakeProducer final : public MotionProducerPort
{
public:
  bool start(const std::string & raw_topic) override
  {
    started_topics.push_back(raw_topic);
    ++start_count;
    return allow_start;
  }

  void stop() override {++stop_count;}

  bool allow_start{true};
  std::size_t start_count{0U};
  std::size_t stop_count{0U};
  std::vector<std::string> started_topics;
};

class FakeComponentGraph final
{
public:
  FakeComponentGraph()
  : container_(std::make_shared<rclcpp::Node>("motion_conditioning_container")),
    collision_(std::make_shared<rclcpp::Node>("collision_monitor")),
    smoother_(std::make_shared<rclcpp::Node>("velocity_smoother")),
    sensors_(std::make_shared<rclcpp::Node>("conditioning_sensors"))
  {
    load_service_ = container_->create_service<LoadNode>(
      "/motion_conditioning_container/_container/load_node",
      [this](
        const std::shared_ptr<LoadNode::Request> request,
        std::shared_ptr<LoadNode::Response> response) {
        response->success = true;
        response->unique_id = next_id_++;
        response->full_node_name = "/" + request->node_name;
        loaded_[response->unique_id] = request->node_name;
        if (request->node_name == "collision_monitor") {
          collision_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
          collision_candidate_ = collision_->create_generic_publisher(
            parameter_string(request, "cmd_vel_out_topic"),
            "geometry_msgs/msg/TwistStamped", rclcpp::QoS(1));
          collision_events_ = collision_->create_generic_publisher(
            parameter_string(request, "state_topic"),
            "nav2_msgs/msg/CollisionMonitorState", rclcpp::QoS(1));
        } else if (request->node_name == "velocity_smoother") {
          smoother_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
        }
      });
    unload_service_ = container_->create_service<UnloadNode>(
      "/motion_conditioning_container/_container/unload_node",
      [this](
        const std::shared_ptr<UnloadNode::Request> request,
        std::shared_ptr<UnloadNode::Response> response) {
        response->success = loaded_.erase(request->unique_id) == 1U;
        if (response->success) {
          collision_candidate_.reset();
          collision_events_.reset();
          collision_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
          smoother_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
      });

    create_lifecycle_services(
      collision_, "/collision_monitor", &collision_state_);
    create_lifecycle_services(
      smoother_, "/velocity_smoother", &smoother_state_);
    scan_publisher_ = sensors_->create_generic_publisher(
      "/scan", "sensor_msgs/msg/LaserScan", rclcpp::QoS(1));
    odom_publisher_ = sensors_->create_generic_publisher(
      "/odom", "nav_msgs/msg/Odometry", rclcpp::QoS(1));
  }

  std::vector<rclcpp::Node::SharedPtr> nodes() const
  {
    return {container_, collision_, smoother_, sensors_};
  }

private:
  static std::string parameter_string(
    const std::shared_ptr<LoadNode::Request> & request,
    const std::string & name)
  {
    for (const auto & value : request->parameters) {
      if (value.name == name && value.value.type ==
        rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
      {
        return value.value.string_value;
      }
    }
    return {};
  }

  void create_lifecycle_services(
    const rclcpp::Node::SharedPtr & node,
    const std::string & fqn,
    std::uint8_t * state)
  {
    change_services_.push_back(node->create_service<ChangeState>(
      fqn + "/change_state",
        [state](
          const std::shared_ptr<ChangeState::Request> request,
          std::shared_ptr<ChangeState::Response> response) {
          switch (request->transition.id) {
            case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
              *state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
              response->success = true;
              return;
            case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
              *state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
              response->success = true;
              return;
            case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN:
            case lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN:
              *state = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
              response->success = true;
              return;
            default:
              response->success = false;
              return;
          }
      }));
    get_services_.push_back(node->create_service<GetState>(
      fqn + "/get_state",
        [state](
          const std::shared_ptr<GetState::Request>,
          std::shared_ptr<GetState::Response> response) {
          response->current_state.id = *state;
          response->current_state.label = "fake";
      }));
  }

  rclcpp::Node::SharedPtr container_;
  rclcpp::Node::SharedPtr collision_;
  rclcpp::Node::SharedPtr smoother_;
  rclcpp::Node::SharedPtr sensors_;
  rclcpp::Service<LoadNode>::SharedPtr load_service_;
  rclcpp::Service<UnloadNode>::SharedPtr unload_service_;
  std::vector<rclcpp::Service<ChangeState>::SharedPtr> change_services_;
  std::vector<rclcpp::Service<GetState>::SharedPtr> get_services_;
  std::unordered_map<std::uint64_t, std::string> loaded_;
  std::uint64_t next_id_{1U};
  std::uint8_t collision_state_{
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN};
  std::uint8_t smoother_state_{
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN};
  rclcpp::GenericPublisher::SharedPtr collision_candidate_;
  rclcpp::GenericPublisher::SharedPtr collision_events_;
  rclcpp::GenericPublisher::SharedPtr scan_publisher_;
  rclcpp::GenericPublisher::SharedPtr odom_publisher_;
};

class MotionConditioningPipelineTest : public ::testing::Test
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

  void SetUp() override
  {
    graph = std::make_unique<FakeComponentGraph>();
    authority = std::make_shared<FakeAuthority>();
    producer = std::make_shared<FakeProducer>();
    client = std::make_shared<rclcpp::Node>("conditioning_client");
    collision_state_publisher = client->create_publisher<
      nav2_msgs::msg::CollisionMonitorState>(
      "/voice_nav_internal/motion/collision_state",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions{}, 4U);
    for (const auto & node : graph->nodes()) {
      executor->add_node(node);
    }
    executor->add_node(client);
    spinning = true;
    spin_thread = std::thread([this]() {
          executor->spin();
    });
  }

  void TearDown() override
  {
    spinning = false;
    executor->cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
    executor->remove_node(client);
    for (const auto & node : graph->nodes()) {
      executor->remove_node(node);
    }
    client.reset();
    collision_state_publisher.reset();
    producer.reset();
    authority.reset();
    graph.reset();
    executor.reset();
  }

  MotionConditioningConfig config()
  {
    MotionConditioningConfig value;
    value.component_rpc_timeout = 200ms;
    value.writer_graph_timeout = 200ms;
    value.prepare_open_deadline = 1s;
    value.renew_period = 10ms;
    value.control_response_deadline = 100ms;
    value.stop_barrier = 100ms;
    auto request = std::make_shared<std::uint64_t>(0U);
    value.request_id_generator = [request]() {
        return std::string(31U, '0') +
               static_cast<char>('1' + (*request)++ % 8U);
      };
    return value;
  }

  std::unique_ptr<FakeComponentGraph> graph;
  std::shared_ptr<FakeAuthority> authority;
  std::shared_ptr<FakeProducer> producer;
  rclcpp::Node::SharedPtr client;
  rclcpp::Publisher<nav2_msgs::msg::CollisionMonitorState>::SharedPtr
    collision_state_publisher;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
  bool spinning{false};
  std::thread spin_thread;
};

TEST_F(MotionConditioningPipelineTest, GateCandidateAndWriterBindingAreRequired)
{
  authority->set_writer_bound(false);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto prepared = pipeline.prepare();
  ASSERT_TRUE(prepared.ok);
  EXPECT_EQ(prepared.candidate_topic, "/candidate/lease_1");
  EXPECT_EQ(prepared.lease_id, "lease-1");

  const auto started = pipeline.start();
  EXPECT_FALSE(started.ok);
  EXPECT_EQ(started.state, MotionConditioningState::Failed);
  EXPECT_EQ(producer->start_count, 0U);
  EXPECT_EQ(started.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(started.zero_proven);

  const auto calls = authority->calls();
  ASSERT_GE(calls.size(), 3U);
  EXPECT_EQ(calls[0], AuthorityOperationKind::Prepare);
  EXPECT_EQ(calls[1], AuthorityOperationKind::Open);
  EXPECT_EQ(calls.back(), AuthorityOperationKind::Inhibit);
}

TEST_F(MotionConditioningPipelineTest, CollisionStopIsReportedAndFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  ASSERT_TRUE(pipeline.start().ok);

  nav2_msgs::msg::CollisionMonitorState stop;
  stop.action_type = nav2_msgs::msg::CollisionMonitorState::STOP;
  stop.polygon_name = "stop_zone";
  collision_state_publisher->publish(stop);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(pipeline.state(), MotionConditioningState::Failed);
  ASSERT_TRUE(pipeline.last_result().collision_stop);
  EXPECT_EQ(
    pipeline.last_result().failure,
    MotionConditioningFailure::ExecutionFailed);
  EXPECT_GE(producer->stop_count, 1U);
  EXPECT_TRUE(pipeline.last_result().zero_proven);
}

TEST_F(MotionConditioningPipelineTest, RenewAuthorityLossFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  ASSERT_TRUE(pipeline.start().ok);
  authority->set_authority_live(false);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }

  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_EQ(
    pipeline.last_result().failure,
    MotionConditioningFailure::SafetyFault);
  EXPECT_GE(producer->stop_count, 1U);
  const auto calls = authority->calls();
  EXPECT_NE(
    std::find(calls.cbegin(), calls.cend(), AuthorityOperationKind::Renew),
    calls.cend());
}

}  // namespace
}  // namespace voice_nav_mission
