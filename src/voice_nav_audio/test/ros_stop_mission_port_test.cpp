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

#include <atomic>
#include <chrono>
#include <array>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_stop_mission_port.hpp"
#include "voice_nav_interfaces/srv/stop_mission.hpp"

namespace voice_nav_audio
{
namespace
{

using StopMission = voice_nav_interfaces::srv::StopMission;
using namespace std::chrono_literals;

class RclcppContextGuard final
{
public:
  RclcppContextGuard()
  {
    rclcpp::init(0, nullptr);
  }

  ~RclcppContextGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class CapturingResponseSink final : public StopMissionResponseSink
{
public:
  void on_response(const StopMissionResponse & response) noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    responses_.push_back(response);
    condition_.notify_all();
  }

  [[nodiscard]] bool wait_for_count(
    const std::size_t count, const std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this, count]() {
      return responses_.size() >= count;
    });
  }

  [[nodiscard]] std::vector<StopMissionResponse> responses() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return responses_;
  }

private:
  mutable std::mutex mutex_{};
  std::condition_variable condition_{};
  std::vector<StopMissionResponse> responses_{};
};

StopMissionRequest request(const std::string & request_id)
{
  return StopMissionRequest{request_id, "voice-instance", 7U, "voice_stop"};
}

class ExecutorRunner final
{
public:
  explicit ExecutorRunner(rclcpp::executors::MultiThreadedExecutor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorRunner()
  {
    executor_.cancel();
    thread_.join();
  }

private:
  rclcpp::executors::MultiThreadedExecutor & executor_;
  std::thread thread_;
};

TEST(RosStopMissionPortTest, UnavailableServiceFailsOnceWithoutWaitingOrRetry)
{
  RclcppContextGuard context;
  auto port_node = std::make_shared<rclcpp::Node>("stop_service_unavailable_client");
  RosStopMissionPort port(port_node);
  CapturingResponseSink sink;

  port.request(request("unavailable"), sink);

  ASSERT_TRUE(sink.wait_for_count(1U, 100ms));
  const auto responses = sink.responses();
  ASSERT_EQ(responses.size(), 1U);
  EXPECT_EQ(responses.front().code, StopMissionCode::TransportFailure);
  EXPECT_EQ(port.request_count(), 0U);
}

TEST(RosStopMissionPortTest, MapsEveryKnownResponseCodeAndMotionFlag)
{
  RclcppContextGuard context;
  auto port_node = std::make_shared<rclcpp::Node>("stop_service_applied_client");
  RosStopMissionPort port(port_node);
  auto server_node = std::make_shared<rclcpp::Node>("stop_service_applied_server");
  auto probe_node = std::make_shared<rclcpp::Node>("stop_service_applied_probe");
  auto probe_client = probe_node->create_client<StopMission>("/mission/stop");
  std::mutex server_mutex;
  StopMission::Request received_request{};
  std::atomic<std::uint16_t> response_code{StopMission::Response::APPLIED};
  std::atomic<bool> response_motion_inhibited{false};
  auto server = server_node->create_service<StopMission>(
    "/mission/stop",
    [&server_mutex, &received_request, &response_code, &response_motion_inhibited](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<StopMission::Request> incoming,
      const std::shared_ptr<StopMission::Response> response) {
      {
        std::lock_guard<std::mutex> lock(server_mutex);
        received_request = *incoming;
        response->code = response_code.load();
        response->motion_inhibited = response_motion_inhibited.load();
      }
    });
  ASSERT_NE(server, nullptr);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(port_node);
  executor.add_node(server_node);
  executor.add_node(probe_node);
  ExecutorRunner runner(executor);
  ASSERT_TRUE(probe_client->wait_for_service(2s));

  struct MappingCase
  {
    std::uint16_t ros_code;
    StopMissionCode expected_code;
    bool motion_inhibited;
  };
  constexpr std::array<MappingCase, 6U> cases{{
    {StopMission::Response::APPLIED, StopMissionCode::Applied, false},
    {StopMission::Response::APPLIED, StopMissionCode::Applied, true},
    {StopMission::Response::DUPLICATE, StopMissionCode::Duplicate, false},
    {StopMission::Response::DUPLICATE, StopMissionCode::Duplicate, true},
    {StopMission::Response::SAFETY_FAULT, StopMissionCode::SafetyFault, false},
    {StopMission::Response::SAFETY_FAULT, StopMissionCode::SafetyFault, true},
  }};
  for (std::size_t index = 0U; index < cases.size(); ++index) {
    SCOPED_TRACE(index);
    response_code.store(cases.at(index).ros_code);
    response_motion_inhibited.store(cases.at(index).motion_inhibited);
    CapturingResponseSink sink;
    port.request(request("mapped-" + std::to_string(index)), sink);

    ASSERT_TRUE(sink.wait_for_count(1U, 2s));
    const auto responses = sink.responses();
    ASSERT_EQ(responses.size(), 1U);
    EXPECT_EQ(responses.front().code, cases.at(index).expected_code);
    EXPECT_EQ(responses.front().motion_inhibited, cases.at(index).motion_inhibited);
  }
  {
    std::lock_guard<std::mutex> lock(server_mutex);
    EXPECT_EQ(received_request.request_id, "mapped-5");
    EXPECT_EQ(received_request.source_instance_id, "voice-instance");
    EXPECT_EQ(received_request.source_seq, 7U);
    EXPECT_EQ(received_request.reason, "voice_stop");
  }
  EXPECT_EQ(port.request_count(), cases.size());
}

TEST(RosStopMissionPortTest, CapacityOneTimeoutAndLateResponseHaveNoRetry)
{
  RclcppContextGuard context;
  auto port_node = std::make_shared<rclcpp::Node>("stop_service_timeout_client");
  RosStopMissionPort port(port_node);
  auto server_node = std::make_shared<rclcpp::Node>("stop_service_timeout_server");
  auto probe_node = std::make_shared<rclcpp::Node>("stop_service_timeout_probe");
  auto probe_client = probe_node->create_client<StopMission>("/mission/stop");
  std::mutex server_mutex;
  std::condition_variable server_condition;
  bool entered = false;
  bool release = false;
  bool returned = false;
  auto server = server_node->create_service<StopMission>(
    "/mission/stop",
    [&server_mutex, &server_condition, &entered, &release, &returned](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<StopMission::Request>,
      const std::shared_ptr<StopMission::Response> response) {
      {
        std::unique_lock<std::mutex> lock(server_mutex);
        entered = true;
        server_condition.notify_all();
        server_condition.wait(lock, [&release]() {return release;});
        response->code = StopMission::Response::APPLIED;
        response->motion_inhibited = true;
        returned = true;
      }
      server_condition.notify_all();
    });
  ASSERT_NE(server, nullptr);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(port_node);
  executor.add_node(server_node);
  executor.add_node(probe_node);
  ExecutorRunner runner(executor);
  ASSERT_TRUE(probe_client->wait_for_service(2s));

  CapturingResponseSink first_sink;
  CapturingResponseSink second_sink;
  port.request(request("timeout-first"), first_sink);
  {
    std::unique_lock<std::mutex> lock(server_mutex);
    ASSERT_TRUE(server_condition.wait_for(lock, 2s, [&entered]() {return entered;}));
  }
  port.request(request("capacity-second"), second_sink);

  ASSERT_TRUE(second_sink.wait_for_count(1U, 100ms));
  ASSERT_EQ(second_sink.responses().size(), 1U);
  EXPECT_EQ(second_sink.responses().front().code, StopMissionCode::TransportFailure);
  EXPECT_EQ(port.request_count(), 1U);

  ASSERT_TRUE(first_sink.wait_for_count(1U, 1500ms));
  ASSERT_EQ(first_sink.responses().size(), 1U);
  EXPECT_EQ(first_sink.responses().front().code, StopMissionCode::Timeout);

  {
    std::lock_guard<std::mutex> lock(server_mutex);
    release = true;
  }
  server_condition.notify_all();
  {
    std::unique_lock<std::mutex> lock(server_mutex);
    ASSERT_TRUE(server_condition.wait_for(lock, 2s, [&returned]() {return returned;}));
  }
  EXPECT_FALSE(first_sink.wait_for_count(2U, 300ms));
  EXPECT_EQ(first_sink.responses().size(), 1U);
}

}  // namespace
}  // namespace voice_nav_audio
