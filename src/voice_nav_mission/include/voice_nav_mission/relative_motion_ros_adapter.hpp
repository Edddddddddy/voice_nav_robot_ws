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

#ifndef VOICE_NAV_MISSION__RELATIVE_MOTION_ROS_ADAPTER_HPP_
#define VOICE_NAV_MISSION__RELATIVE_MOTION_ROS_ADAPTER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/motion_conditioning_pipeline.hpp"
#include "voice_nav_mission/relative_motion_controller.hpp"

namespace voice_nav_mission
{

// ROS Adapter for the production RelativeMotionPort. The ROS-free controller
// remains the deep policy Module; this Adapter owns only odometry ingress,
// raw TwistStamped publication, and delegation to the #35 conditioning
// Module. Gate/component/writer handover logic is intentionally not present.
class RelativeMotionRosAdapter final : public RelativeMotionPort
{
public:
  RelativeMotionRosAdapter(
    rclcpp::Node & node,
    std::shared_ptr<MotionAuthorityPort> authority,
    RelativeMotionPolicy policy = {},
    MotionConditioningConfig conditioning_config = {});
  ~RelativeMotionRosAdapter() override;

  RelativeMotionRosAdapter(const RelativeMotionRosAdapter &) = delete;
  RelativeMotionRosAdapter & operator=(const RelativeMotionRosAdapter &) = delete;

  [[nodiscard]] bool healthy() const override;
  void start(
    const MotionToken & token,
    const MissionStep & step,
    FeedbackCallback feedback,
    ResultCallback result) override;
  [[nodiscard]] bool cancel(
    const MotionToken & token,
    SteadyClockPort::TimePoint deadline) override;
  void tick(SteadyClockPort::TimePoint now) override;
  [[nodiscard]] bool owns_authority_lifecycle() const noexcept override;
  [[nodiscard]] bool zero_proven() const noexcept override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RELATIVE_MOTION_ROS_ADAPTER_HPP_
