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

#ifndef VOICE_NAV_AUDIO__ROS_STOP_MISSION_PORT_HPP_
#define VOICE_NAV_AUDIO__ROS_STOP_MISSION_PORT_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "voice_nav_interfaces/srv/stop_mission.hpp"
#include "voice_pipeline_coordination.hpp"

namespace voice_nav_audio
{

// Package-private adapter for the existing /mission/stop service. It owns no
// motion authority and never blocks, retries, or creates a second endpoint.
class RosStopMissionPort final : public StopMissionPort
{
public:
  explicit RosStopMissionPort(rclcpp::Node::SharedPtr node);
  ~RosStopMissionPort() override;

  [[nodiscard]] std::size_t request_count() const noexcept;

  void request(
    const StopMissionRequest & request,
    StopMissionResponseSink & response_sink) noexcept override;

private:
  using StopMission = voice_nav_interfaces::srv::StopMission;

  struct State;

  struct Pending
  {
    Pending(
      const std::shared_ptr<State> & state_value,
      StopMissionResponseSink & response_sink_value,
      const std::uint64_t generation_value) noexcept
    : state(state_value), response_sink(&response_sink_value), generation(generation_value)
    {
    }

    std::weak_ptr<State> state;
    StopMissionResponseSink * response_sink{nullptr};
    std::uint64_t generation{0U};
    std::atomic<bool> completed{false};
    rclcpp::TimerBase::SharedPtr timeout_timer{};
  };

  struct State
  {
    mutable std::mutex mutex{};
    std::weak_ptr<Pending> in_flight{};
    bool alive{true};
    std::uint64_t next_generation{1U};
    std::size_t request_count{0U};
  };

  static void complete_pending(
    const std::shared_ptr<Pending> & pending,
    const StopMissionResponse & response) noexcept;
  [[nodiscard]] static StopMissionResponse map_response(
    const std::shared_ptr<const StopMission::Response> & response) noexcept;

  rclcpp::Node::SharedPtr node_{};
  rclcpp::Client<StopMission>::SharedPtr client_{};
  std::shared_ptr<State> state_{};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__ROS_STOP_MISSION_PORT_HPP_
