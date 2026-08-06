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

#ifndef VOICE_NAV_MISSION__MOTION_CONDITIONING_PIPELINE_HPP_
#define VOICE_NAV_MISSION__MOTION_CONDITIONING_PIPELINE_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "voice_nav_mission/mission_runtime_core.hpp"

namespace voice_nav_mission
{

enum class MotionConditioningState : std::uint8_t
{
  Stopped = 0,
  Prepared = 1,
  Running = 2,
  Failed = 3,
};

enum class MotionConditioningFailure : std::uint8_t
{
  None = 0,
  DependencyUnavailable = 1,
  SafetyFault = 2,
  ExecutionFailed = 3,
  InternalError = 4,
};

struct MotionConditioningResult
{
  bool ok{false};
  MotionConditioningState state{MotionConditioningState::Stopped};
  MotionConditioningFailure failure{MotionConditioningFailure::None};
  bool zero_proven{false};
  bool collision_stop{false};
  std::string lease_id;
  std::string candidate_topic;
  std::string detail;
};

struct MotionConditioningConfig
{
  std::chrono::milliseconds component_rpc_timeout{2000};
  std::chrono::milliseconds writer_graph_timeout{1000};
  std::chrono::milliseconds prepare_open_deadline{4000};
  std::chrono::milliseconds renew_period{100};
  std::chrono::milliseconds control_response_deadline{100};
  std::chrono::milliseconds stop_barrier{250};
  std::string container_fqn{"/motion_conditioning_container"};
  std::string raw_topic{"/voice_nav_internal/motion/raw"};
  std::string smoothed_topic{"/voice_nav_internal/motion/smoothed"};
  std::string scan_topic{"/scan"};
  std::string odom_topic{"/odom"};
  std::string collision_state_topic{
    "/voice_nav_internal/motion/collision_state"};
  std::function<std::string()> request_id_generator;
};

class MotionProducerPort
{
public:
  virtual ~MotionProducerPort() = default;
  [[nodiscard]] virtual bool start(const std::string & raw_topic) = 0;
  virtual void stop() = 0;
};

class MotionConditioningPipeline final
{
public:
  MotionConditioningPipeline(
    rclcpp::Node & node,
    std::shared_ptr<MotionAuthorityPort> authority,
    std::shared_ptr<MotionProducerPort> producer,
    MotionConditioningConfig config = {});
  ~MotionConditioningPipeline();

  MotionConditioningPipeline(const MotionConditioningPipeline &) = delete;
  MotionConditioningPipeline & operator=(const MotionConditioningPipeline &) = delete;

  [[nodiscard]] MotionConditioningResult prepare();
  [[nodiscard]] MotionConditioningResult start();
  [[nodiscard]] MotionConditioningResult stop();
  [[nodiscard]] MotionConditioningResult fail(
    MotionConditioningFailure failure,
    std::string detail);

  [[nodiscard]] MotionConditioningState state() const noexcept;
  [[nodiscard]] MotionConditioningResult last_result() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__MOTION_CONDITIONING_PIPELINE_HPP_
