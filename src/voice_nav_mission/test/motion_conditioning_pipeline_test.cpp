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
#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <composition_interfaces/srv/load_node.hpp>
#include <composition_interfaces/srv/list_nodes.hpp>
#include <composition_interfaces/srv/unload_node.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "voice_nav_mission/mission_authority_convergence.hpp"
#include "voice_nav_mission/motion_conditioning_pipeline.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;
using LoadNode = composition_interfaces::srv::LoadNode;
using ListNodes = composition_interfaces::srv::ListNodes;
using UnloadNode = composition_interfaces::srv::UnloadNode;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using ListControllers = controller_manager_msgs::srv::ListControllers;

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
    snapshot_.zero_published = prepare_zero_proof_;
    snapshot_.authority_live = false;
    snapshot_.writer_bound = false;
    return AuthorityResult{
      true, prepare_zero_proof_, false, snapshot_, snapshot_.lease_id,
      "prepared"};
  }

  AuthorityResult open(const AuthorityOperation &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back(AuthorityOperationKind::Open);
    if (throw_on_open_) {
      throw std::runtime_error("scripted MotionGate OPEN failure");
    }
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
    std::unique_lock<std::mutex> lock(mutex_);
    calls_.push_back(AuthorityOperationKind::Renew);
    if (block_renew_) {
      renew_entered_ = true;
      renew_cv_.notify_all();
      renew_cv_.wait(lock, [this]() {return release_renew_;});
    }
    if (throw_on_renew_) {
      throw std::runtime_error("scripted MotionGate RENEW failure");
    }
    snapshot_.control_seq++;
    ++renew_count_;
    snapshot_.authority_live = authority_live_ &&
      (!renew_failure_after_.has_value() || renew_count_ <= *renew_failure_after_);
    return AuthorityResult{
      true, false, false, snapshot_, snapshot_.lease_id, "renewed"};
  }

  AuthorityResult inhibit(const AuthorityOperation &) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    calls_.push_back(AuthorityOperationKind::Inhibit);
    if (block_inhibit_) {
      inhibit_entered_ = true;
      inhibit_cv_.notify_all();
      inhibit_cv_.wait(lock, [this]() {return release_inhibit_;});
    }
    snapshot_.control_seq++;
    snapshot_.state = GateState::Inhibited;
    snapshot_.motion_inhibited = true;
    snapshot_.zero_selected = true;
    snapshot_.zero_published = inhibit_zero_proof_;
    snapshot_.authority_live = false;
    snapshot_.writer_bound = false;
    return AuthorityResult{
      true, inhibit_zero_proof_, false, snapshot_, snapshot_.lease_id,
      "inhibited"};
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

  void set_prepare_zero_proof(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prepare_zero_proof_ = value;
  }

  void set_inhibit_zero_proof(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    inhibit_zero_proof_ = value;
  }

  void set_initial_zero_proof(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.zero_published = value;
  }

  void set_renew_failure_after(std::size_t successful_renews)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    renew_failure_after_ = successful_renews;
  }

  void set_throw_on_open(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    throw_on_open_ = value;
  }

  void set_throw_on_renew(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    throw_on_renew_ = value;
  }

  void block_renew()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block_renew_ = true;
    renew_entered_ = false;
    release_renew_ = false;
  }

  bool wait_for_renew(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return renew_cv_.wait_for(
      lock, timeout, [this]() {return renew_entered_;});
  }

  void release_blocked_renew()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    release_renew_ = true;
    renew_cv_.notify_all();
  }

  void set_initial_armed_snapshot()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.state = GateState::Armed;
    snapshot_.motion_inhibited = false;
    snapshot_.zero_selected = false;
    snapshot_.zero_published = false;
    snapshot_.authority_live = true;
    snapshot_.writer_bound = true;
  }

  void block_inhibit()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block_inhibit_ = true;
    inhibit_entered_ = false;
    release_inhibit_ = false;
  }

  bool wait_for_inhibit(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return inhibit_cv_.wait_for(
      lock, timeout, [this]() {return inhibit_entered_;});
  }

  void release_blocked_inhibit()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    release_inhibit_ = true;
    inhibit_cv_.notify_all();
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
  bool prepare_zero_proof_{true};
  bool inhibit_zero_proof_{true};
  std::size_t renew_count_{0U};
  std::optional<std::size_t> renew_failure_after_;
  bool throw_on_open_{false};
  bool throw_on_renew_{false};
  bool block_renew_{false};
  bool renew_entered_{false};
  bool release_renew_{false};
  bool block_inhibit_{false};
  bool inhibit_entered_{false};
  bool release_inhibit_{false};
  std::condition_variable inhibit_cv_;
  std::condition_variable renew_cv_;
  std::vector<AuthorityOperationKind> calls_;
};

class FakeProducer final : public MotionProducerPort
{
public:
  bool start(const std::string & raw_topic) override
  {
    started_topics.push_back(raw_topic);
    ++start_count;
    if (block_start) {
      std::unique_lock<std::mutex> lock(start_mutex);
      start_entered = true;
      start_cv.notify_all();
      start_cv.wait(lock, [this]() {return release_start;});
    }
    if (throw_on_start) {
      throw std::runtime_error("scripted producer start failure");
    }
    return allow_start;
  }

  void stop() override
  {
    ++stop_count;
    if (throw_on_stop) {
      throw std::runtime_error("scripted producer stop failure");
    }
  }

  void wait_for_start()
  {
    std::unique_lock<std::mutex> lock(start_mutex);
    start_cv.wait(lock, [this]() {return start_entered;});
  }

  void release_blocked_start()
  {
    std::lock_guard<std::mutex> lock(start_mutex);
    release_start = true;
    start_cv.notify_all();
  }

  bool allow_start{true};
  bool throw_on_start{false};
  bool throw_on_stop{false};
  bool block_start{false};
  std::size_t start_count{0U};
  std::size_t stop_count{0U};
  std::vector<std::string> started_topics;

private:
  std::mutex start_mutex;
  std::condition_variable start_cv;
  bool start_entered{false};
  bool release_start{false};
};

class FakeComponentGraph final
{
public:
  FakeComponentGraph()
  : container_(std::make_shared<rclcpp::Node>("motion_conditioning_container")),
    collision_(std::make_shared<rclcpp::Node>("collision_monitor")),
    smoother_(std::make_shared<rclcpp::Node>("velocity_smoother")),
    sensors_(std::make_shared<rclcpp::Node>("conditioning_sensors")),
    unload_callback_group_(container_->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant))
  {
    load_service_ = container_->create_service<LoadNode>(
      "/motion_conditioning_container/_container/load_node",
      [this](
        const std::shared_ptr<LoadNode::Request> request,
        std::shared_ptr<LoadNode::Response> response) {
        std::this_thread::sleep_for(load_delay_);
        response->success = true;
        response->unique_id = next_id_++;
        response->full_node_name = wrong_fqn_ ?
        "/wrong_" + request->node_name : "/" + request->node_name;
        {
          std::lock_guard<std::mutex> lock(graph_mutex_);
          loaded_[response->unique_id] = response->full_node_name;
        }
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
    list_nodes_service_ = container_->create_service<ListNodes>(
      "/motion_conditioning_container/_container/list_nodes",
      [this](
        const std::shared_ptr<ListNodes::Request>,
        std::shared_ptr<ListNodes::Response> response) {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        for (const auto & entry : loaded_) {
          response->unique_ids.push_back(entry.first);
          response->full_node_names.push_back(entry.second);
        }
      });
    unload_service_ = container_->create_service<UnloadNode>(
      "/motion_conditioning_container/_container/unload_node",
      [this](
        const std::shared_ptr<UnloadNode::Request> request,
        std::shared_ptr<UnloadNode::Response> response) {
        std::chrono::milliseconds delay;
        {
          std::lock_guard<std::mutex> lock(graph_mutex_);
          unload_requests_.push_back(request->unique_id);
          const auto delay_iterator = unload_delays_.find(request->unique_id);
          delay = delay_iterator != unload_delays_.cend() ?
          delay_iterator->second : unload_delay_;
        }
        std::this_thread::sleep_for(delay);
        {
          std::lock_guard<std::mutex> lock(graph_mutex_);
          unload_completed_.push_back(request->unique_id);
          ++unload_count_;
          response->success = loaded_.erase(request->unique_id) == 1U;
        }
        if (response->success) {
          collision_candidate_.reset();
          collision_events_.reset();
          collision_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
          smoother_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
      }, rmw_qos_profile_services_default, unload_callback_group_);
    controller_service_ = container_->create_service<ListControllers>(
      "/controller_manager/list_controllers",
      [this](
        const std::shared_ptr<ListControllers::Request>,
        std::shared_ptr<ListControllers::Response> response) {
        controller_manager_msgs::msg::ControllerState controller;
        controller.name = "diff_drive_controller";
        controller.state = controller_active_ ? "active" : "inactive";
        response->controller = {controller};
      });

    create_lifecycle_services(
      collision_, "/collision_monitor", &collision_state_);
    create_lifecycle_services(
      smoother_, "/velocity_smoother", &smoother_state_);
    scan_publisher_ = sensors_->create_generic_publisher(
      "/scan", "sensor_msgs/msg/LaserScan", rclcpp::QoS(1));
    odom_publisher_ = sensors_->create_generic_publisher(
      "/odom", "nav_msgs/msg/Odometry", rclcpp::QoS(1));
    scan_message_publisher_ = sensors_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS());
    odom_message_publisher_ = sensors_->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS());
    clock_publisher_ = sensors_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS());
    health_timer_ = sensors_->create_wall_timer(20ms, [this]() {
          rosgraph_msgs::msg::Clock clock;
          if (publish_clock_) {
            {
              std::lock_guard<std::mutex> lock(health_mutex_);
              clock.clock = freeze_clock_ ?
              frozen_clock_ : rclcpp::Clock(RCL_SYSTEM_TIME).now();
            }
            clock_publisher_->publish(clock);
          }
          sensor_msgs::msg::LaserScan scan;
          scan.header.stamp = clock.clock;
          scan.ranges = {10.0F, 10.0F};
          if (publish_scan_) {
            scan_message_publisher_->publish(scan);
          }
          nav_msgs::msg::Odometry odom;
          odom.header.stamp = clock.clock;
          if (publish_odom_) {
            odom_message_publisher_->publish(odom);
          }
        });
  }

  std::vector<rclcpp::Node::SharedPtr> nodes() const
  {
    return {container_, collision_, smoother_, sensors_};
  }

  std::size_t loaded_count() const
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return loaded_.size();
  }

  std::size_t unload_count() const
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return unload_count_;
  }

  void set_activation_delay(std::chrono::milliseconds delay)
  {
    activation_delay_ = delay;
  }

  void set_load_delay(std::chrono::milliseconds delay)
  {
    load_delay_ = delay;
  }

  void set_unload_delay(std::chrono::milliseconds delay)
  {
    unload_delay_ = delay;
  }

  void set_unload_delay_for(
    std::uint64_t unique_id,
    std::chrono::milliseconds delay)
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    unload_delays_[unique_id] = delay;
  }

  std::vector<std::uint64_t> unload_requests() const
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return unload_requests_;
  }

  std::vector<std::uint64_t> unload_completed() const
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return unload_completed_;
  }

  void set_wrong_fqn(bool value)
  {
    wrong_fqn_ = value;
  }

  void set_health_sources(bool scan, bool odom, bool clock)
  {
    publish_scan_ = scan;
    publish_odom_ = odom;
    publish_clock_ = clock;
  }

  void set_clock_frozen(bool value)
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    freeze_clock_ = value;
    if (value) {
      frozen_clock_ = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    }
  }

  void enable_activation_barrier()
  {
    std::lock_guard<std::mutex> lock(activation_mutex_);
    activation_barrier_ = true;
    activation_entered_ = false;
    activation_released_ = false;
  }

  void wait_for_activation_entry()
  {
    std::unique_lock<std::mutex> lock(activation_mutex_);
    activation_cv_.wait(lock, [this]() {return activation_entered_;});
  }

  void release_activation()
  {
    std::lock_guard<std::mutex> lock(activation_mutex_);
    activation_released_ = true;
    activation_cv_.notify_all();
  }

  void remove_lifecycle_services(bool collision, bool smoother)
  {
    if (!collision) {
      change_services_[0].reset();
      get_services_[0].reset();
    }
    if (!smoother) {
      change_services_[1].reset();
      get_services_[1].reset();
    }
  }

  void set_controller_active(bool value)
  {
    controller_active_ = value;
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
        [this, state, fqn](
          const std::shared_ptr<ChangeState::Request> request,
          std::shared_ptr<ChangeState::Response> response) {
          switch (request->transition.id) {
            case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
              *state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
              response->success = true;
              return;
            case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
              if (fqn == "/collision_monitor") {
                std::unique_lock<std::mutex> lock(activation_mutex_);
                if (activation_barrier_) {
                  activation_entered_ = true;
                  activation_cv_.notify_all();
                  activation_cv_.wait(
                    lock, [this]() {return activation_released_;});
                }
              }
              std::this_thread::sleep_for(activation_delay_);
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
  rclcpp::Service<ListNodes>::SharedPtr list_nodes_service_;
  rclcpp::Service<UnloadNode>::SharedPtr unload_service_;
  rclcpp::CallbackGroup::SharedPtr unload_callback_group_;
  rclcpp::Service<ListControllers>::SharedPtr controller_service_;
  std::vector<rclcpp::Service<ChangeState>::SharedPtr> change_services_;
  std::vector<rclcpp::Service<GetState>::SharedPtr> get_services_;
  std::unordered_map<std::uint64_t, std::string> loaded_;
  mutable std::mutex graph_mutex_;
  std::uint64_t next_id_{1U};
  std::size_t unload_count_{0U};
  std::vector<std::uint64_t> unload_requests_;
  std::vector<std::uint64_t> unload_completed_;
  std::unordered_map<std::uint64_t, std::chrono::milliseconds> unload_delays_;
  std::uint8_t collision_state_{
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN};
  std::uint8_t smoother_state_{
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN};
  std::chrono::milliseconds activation_delay_{};
  std::chrono::milliseconds load_delay_{};
  std::chrono::milliseconds unload_delay_{};
  bool wrong_fqn_{false};
  bool publish_scan_{true};
  bool publish_odom_{true};
  bool publish_clock_{true};
  bool controller_active_{true};
  bool freeze_clock_{false};
  rclcpp::Time frozen_clock_;
  std::mutex health_mutex_;
  std::mutex activation_mutex_;
  std::condition_variable activation_cv_;
  bool activation_barrier_{false};
  bool activation_entered_{false};
  bool activation_released_{false};
  rclcpp::GenericPublisher::SharedPtr collision_candidate_;
  rclcpp::GenericPublisher::SharedPtr collision_events_;
  rclcpp::GenericPublisher::SharedPtr scan_publisher_;
  rclcpp::GenericPublisher::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_message_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_message_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::TimerBase::SharedPtr health_timer_;
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
    graph->set_health_sources(true, true, false);
    authority = std::make_shared<FakeAuthority>();
    producer = std::make_shared<FakeProducer>();
    client = std::make_shared<rclcpp::Node>(
      "conditioning_client",
      rclcpp::NodeOptions().append_parameter_override("use_sim_time", true));
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

  MotionConditioningConfig config(bool enable_clock = true)
  {
    graph->set_health_sources(true, true, enable_clock);
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

TEST_F(MotionConditioningPipelineTest, PrepareRequiresGateZeroProof)
{
  authority->set_prepare_zero_proof(false);
  authority->set_inhibit_zero_proof(false);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
  EXPECT_EQ(producer->start_count, 0U);
}

TEST_F(MotionConditioningPipelineTest, NoLeaseRequiresCurrentGateZeroProof)
{
  authority->set_initial_zero_proof(false);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();
  const auto calls = authority->calls();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(result.zero_proven);
  EXPECT_TRUE(calls.empty());
}

TEST_F(MotionConditioningPipelineTest, StopWithoutLeaseRequiresInhibitedZeroSnapshot)
{
  authority->set_initial_armed_snapshot();
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.stop();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.state, MotionConditioningState::Failed);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(result.zero_proven);
  EXPECT_EQ(producer->stop_count, 1U);
}

TEST_F(MotionConditioningPipelineTest, CollisionStopIsReportedAndFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
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

TEST_F(MotionConditioningPipelineTest, LeaseLossDuringActivationNeverStartsProducer)
{
  graph->set_activation_delay(150ms);
  authority->set_renew_failure_after(1U);
  auto pipeline_config = config();
  pipeline_config.renew_period = 20ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);

  const auto result = pipeline.start();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_EQ(producer->start_count, 0U);
  const auto calls = authority->calls();
  EXPECT_GE(
    std::count(
      calls.cbegin(), calls.cend(),
      AuthorityOperationKind::Renew), 2);
}

TEST_F(MotionConditioningPipelineTest, StopAtActivationBarrierRejectsLateProducerStart)
{
  graph->enable_activation_barrier();
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);

  std::optional<MotionConditioningResult> start_result;
  std::thread start_thread([&]() {start_result = pipeline.start();});
  graph->wait_for_activation_entry();

  const auto stopped = pipeline.stop();
  EXPECT_TRUE(stopped.zero_proven);
  graph->release_activation();
  start_thread.join();

  ASSERT_TRUE(start_result.has_value());
  EXPECT_FALSE(start_result->ok);
  EXPECT_EQ(producer->start_count, 0U);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Stopped);
  EXPECT_EQ(pipeline.last_result().state, stopped.state);
  EXPECT_EQ(pipeline.last_result().failure, stopped.failure);
  EXPECT_EQ(pipeline.last_result().zero_proven, stopped.zero_proven);
}

TEST_F(MotionConditioningPipelineTest, ProducerFalseFailsClosedAndCleansUp)
{
  producer->allow_start = false;
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);

  const auto result = pipeline.start();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.state, MotionConditioningState::Failed);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_GE(producer->stop_count, 1U);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, ProducerThrowFailsClosedAndCleansUp)
{
  producer->throw_on_start = true;
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  ASSERT_TRUE(pipeline.prepare().ok);
  MotionConditioningResult result;
  EXPECT_NO_THROW(result = pipeline.start());

  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(
    result.failure == MotionConditioningFailure::InternalError ||
    result.failure == MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_GE(producer->stop_count, 1U);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, ProducerStopFailureMakesStopSafetyFault)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  producer->throw_on_stop = true;

  const auto result = pipeline.stop();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.state, MotionConditioningState::Failed);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, RenewThrowFailsClosedAndStopsActivation)
{
  authority->set_throw_on_renew(true);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  ASSERT_TRUE(pipeline.prepare().ok);
  MotionConditioningResult result;
  EXPECT_NO_THROW(result = pipeline.start());

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.state, MotionConditioningState::Failed);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, OpenThrowFailsClosedAndCleansUp)
{
  authority->set_throw_on_open(true);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  ASSERT_TRUE(pipeline.prepare().ok);
  MotionConditioningResult result;
  EXPECT_NO_THROW(result = pipeline.start());

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::InternalError);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, LateLoadResponseIsReconciledAndBlocksNextPrepare)
{
  graph->set_load_delay(250ms);
  auto pipeline_config = config();
  pipeline_config.component_rpc_timeout = 20ms;
  pipeline_config.writer_graph_timeout = 100ms;
  pipeline_config.prepare_open_deadline = 100ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  const auto first = pipeline.prepare();
  EXPECT_FALSE(first.ok);
  EXPECT_EQ(first.failure, MotionConditioningFailure::SafetyFault);
  const auto calls_after_first = authority->calls();
  const auto second = pipeline.prepare();
  const auto calls_after_second = authority->calls();
  EXPECT_FALSE(second.ok);
  EXPECT_EQ(second.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_EQ(
    std::count(
      calls_after_second.cbegin(), calls_after_second.cend(),
      AuthorityOperationKind::Prepare),
    std::count(
      calls_after_first.cbegin(), calls_after_first.cend(),
      AuthorityOperationKind::Prepare));
}

TEST_F(MotionConditioningPipelineTest, FqnMismatchIsUnloadedBeforePrepareFails)
{
  graph->set_wrong_fqn(true);
  graph->remove_lifecycle_services(false, false);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, AbsentLifecycleServicesDoNotHideActualUnload)
{
  graph->remove_lifecycle_services(false, false);
  auto pipeline_config = config();
  pipeline_config.component_rpc_timeout = 80ms;
  pipeline_config.writer_graph_timeout = 120ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, LifecycleTimeoutRetainsIndependentUnloadBudget)
{
  auto pipeline_config = config();
  pipeline_config.component_rpc_timeout = 2s;
  pipeline_config.writer_graph_timeout = 1s;
  pipeline_config.prepare_open_deadline = 4s;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  graph->remove_lifecycle_services(false, false);

  const auto result = pipeline.stop();

  EXPECT_TRUE(result.ok);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
  EXPECT_EQ(graph->unload_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, EachUniqueIdGetsAnIndependentUnloadBudget)
{
  auto pipeline_config = config();
  pipeline_config.component_rpc_timeout = 5ms;
  pipeline_config.writer_graph_timeout = 150ms;
  pipeline_config.prepare_open_deadline = 2s;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  ASSERT_TRUE(pipeline.prepare().ok);
  graph->set_unload_delay_for(1U, 500ms);

  const auto result = pipeline.stop();
  const auto unload_requests = graph->unload_requests();
  const auto unload_completed = graph->unload_completed();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_NE(
    std::find(unload_requests.cbegin(), unload_requests.cend(), 1U),
    unload_requests.cend());
  EXPECT_NE(
    std::find(unload_requests.cbegin(), unload_requests.cend(), 2U),
    unload_requests.cend());
  EXPECT_NE(
    std::find(unload_completed.cbegin(), unload_completed.cend(), 2U),
    unload_completed.cend());
  EXPECT_EQ(
    std::find(unload_completed.cbegin(), unload_completed.cend(), 1U),
    unload_completed.cend());
}

TEST_F(MotionConditioningPipelineTest, SensorLivenessLossFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  graph->set_health_sources(false, true, true);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_TRUE(pipeline.last_result().zero_proven);
  EXPECT_GE(producer->stop_count, 1U);
}

TEST_F(MotionConditioningPipelineTest, OdomLivenessLossFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  graph->set_health_sources(true, false, true);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_TRUE(pipeline.last_result().zero_proven);
}

TEST_F(MotionConditioningPipelineTest, ClockLivenessLossFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  graph->set_health_sources(true, true, false);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_TRUE(pipeline.last_result().zero_proven);
}

TEST_F(MotionConditioningPipelineTest, FrozenClockProgressCannotStartProducer)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  graph->set_clock_frozen(true);
  std::this_thread::sleep_for(250ms);

  const auto result = pipeline.start();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(producer->start_count, 0U);
  EXPECT_TRUE(result.zero_proven);
}

TEST_F(MotionConditioningPipelineTest, FirstFrozenClockSampleCannotStartProducer)
{
  auto pipeline_config = config(false);
  graph->set_clock_frozen(true);
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  graph->set_health_sources(true, true, true);

  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);

  const auto result = pipeline.start();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(producer->start_count, 0U);
  EXPECT_TRUE(result.zero_proven);
}

TEST_F(MotionConditioningPipelineTest, InactiveControllerFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  graph->set_controller_active(false);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_TRUE(pipeline.last_result().zero_proven);
  EXPECT_GE(producer->stop_count, 1U);
}

TEST_F(MotionConditioningPipelineTest, StopAndFailShareOneTeardownOwner)
{
  auto pipeline_config = config();
  pipeline_config.renew_period = 1s;
  MotionConditioningPipeline pipeline(*client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  authority->block_inhibit();

  std::optional<MotionConditioningResult> stop_result;
  std::thread stop_thread([&]() {stop_result = pipeline.stop();});
  const bool inhibit_entered = authority->wait_for_inhibit();
  if (!inhibit_entered) {
    authority->release_blocked_inhibit();
  }
  ASSERT_TRUE(inhibit_entered);

  std::optional<MotionConditioningResult> fail_result;
  std::thread fail_thread([&]() {
      fail_result = pipeline.fail(
        MotionConditioningFailure::SafetyFault,
        "dependency failed during STOP");
    });
  std::this_thread::sleep_for(20ms);
  authority->release_blocked_inhibit();
  stop_thread.join();
  fail_thread.join();

  ASSERT_TRUE(stop_result.has_value());
  ASSERT_TRUE(fail_result.has_value());
  EXPECT_EQ(stop_result->state, fail_result->state);
  EXPECT_EQ(stop_result->failure, fail_result->failure);
  EXPECT_EQ(stop_result->zero_proven, fail_result->zero_proven);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(),
      AuthorityOperationKind::Inhibit), 1);
  EXPECT_EQ(graph->unload_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, TerminalRecordIgnoresLateFailureAndCollision)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);

  const auto stopped = pipeline.stop();
  ASSERT_TRUE(stopped.ok);
  const auto terminal = pipeline.last_result();
  const auto fail_result = pipeline.fail(
    MotionConditioningFailure::SafetyFault,
    "late failure must not replace STOP");

  nav2_msgs::msg::CollisionMonitorState collision_stop;
  collision_stop.action_type = nav2_msgs::msg::CollisionMonitorState::STOP;
  collision_stop.polygon_name = "stop_zone";
  collision_state_publisher->publish(collision_stop);
  std::this_thread::sleep_for(50ms);

  const auto calls = authority->calls();
  EXPECT_TRUE(fail_result.ok);
  EXPECT_EQ(pipeline.last_result().state, terminal.state);
  EXPECT_EQ(pipeline.last_result().failure, terminal.failure);
  EXPECT_EQ(pipeline.last_result().collision_stop, terminal.collision_stop);
  EXPECT_EQ(pipeline.last_result().zero_proven, terminal.zero_proven);
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(),
      AuthorityOperationKind::Inhibit), 1);
}

TEST_F(MotionConditioningPipelineTest, StopWaitsForRenewFailureTeardownOwner)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  authority->block_inhibit();
  authority->set_throw_on_renew(true);
  const bool inhibit_entered = authority->wait_for_inhibit();
  if (!inhibit_entered) {
    authority->release_blocked_inhibit();
  }
  ASSERT_TRUE(inhibit_entered);

  std::optional<MotionConditioningResult> stop_result;
  std::thread stop_thread([&]() {stop_result = pipeline.stop();});
  std::this_thread::sleep_for(20ms);
  authority->release_blocked_inhibit();
  stop_thread.join();

  ASSERT_TRUE(stop_result.has_value());
  EXPECT_EQ(stop_result->state, MotionConditioningState::Failed);
  EXPECT_EQ(stop_result->failure, MotionConditioningFailure::SafetyFault);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(),
      AuthorityOperationKind::Inhibit), 1);
  EXPECT_EQ(graph->unload_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, ExternalFailureWaitsForActiveRenew)
{
  auto pipeline_config = config();
  pipeline_config.renew_period = 20ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);

  authority->block_renew();
  ASSERT_TRUE(authority->wait_for_renew());

  std::optional<MotionConditioningResult> failure_result;
  std::thread failure_thread([&]() {
      failure_result = pipeline.fail(
        MotionConditioningFailure::SafetyFault,
        "external dependency failure");
    });
  std::this_thread::sleep_for(20ms);
  authority->release_blocked_renew();
  failure_thread.join();

  ASSERT_TRUE(failure_result.has_value());
  EXPECT_FALSE(failure_result->ok);
  EXPECT_EQ(failure_result->state, MotionConditioningState::Failed);
  EXPECT_EQ(failure_result->failure, MotionConditioningFailure::SafetyFault);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(),
      AuthorityOperationKind::Inhibit), 1);
  EXPECT_EQ(graph->unload_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, RenewAuthorityLossFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
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
