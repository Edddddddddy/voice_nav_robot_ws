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

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/motion_conditioning_pipeline.hpp"
#include "voice_nav_mission/relative_motion_controller.hpp"

namespace voice_nav_mission
{

class RelativeMotionRosAdapter;

namespace detail
{
class RelativeMotionRosAdapterTestAccess;
void begin_relative_motion_shutdown(
  RelativeMotionRosAdapter & adapter,
  SteadyClockPort::TimePoint deadline) noexcept;
[[nodiscard]] bool wait_for_relative_motion_internal_completion(
  RelativeMotionRosAdapter & adapter,
  SteadyClockPort::TimePoint deadline) noexcept;
}

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
  [[nodiscard]] bool uses_external_completion_registry() const noexcept override;
  void start(
    const MotionToken & token,
    const MissionStep & step,
    FeedbackCallback feedback,
    ResultCallback result) override;
  [[nodiscard]] bool cancel(
    const MotionToken & token,
    SteadyClockPort::TimePoint deadline) override;
  // Independent fail-closed path used when the Runtime event queue or worker
  // cannot accept/serialize a control event.  It never calls RuntimeCore.
  void request_emergency_stop() noexcept;
  [[nodiscard]] bool emergency_stop(SteadyClockPort::TimePoint deadline);
  void begin_shutdown() noexcept;
  void wait_for_internal_completion() noexcept;
  void finalize_shutdown() noexcept;
  // Explicitly drains all adapter transactions before its owned resources are
  // released by the Mission Runtime Node.
  void shutdown() noexcept;
  void tick(SteadyClockPort::TimePoint now) override;
  [[nodiscard]] bool owns_authority_lifecycle() const noexcept override;
  [[nodiscard]] bool zero_proven() const noexcept override;
  [[nodiscard]] bool safety_faulted() const noexcept override;
  [[nodiscard]] bool rearm_after_gate_replacement(
    const GateSnapshot & snapshot) noexcept override;

private:
  friend class detail::RelativeMotionRosAdapterTestAccess;
  friend void detail::begin_relative_motion_shutdown(
    RelativeMotionRosAdapter &, SteadyClockPort::TimePoint) noexcept;
  friend bool detail::wait_for_relative_motion_internal_completion(
    RelativeMotionRosAdapter &, SteadyClockPort::TimePoint) noexcept;
  class Impl;
  void begin_shutdown_until(SteadyClockPort::TimePoint deadline) noexcept;
  [[nodiscard]] bool wait_for_internal_completion_until(
    SteadyClockPort::TimePoint deadline) noexcept;
  std::shared_ptr<Impl> impl_;
};

namespace detail
{

// Package-private production seam used only by deterministic Adapter tests;
// it does not add a ROS endpoint or a public ROS IDL.
class RelativeMotionRosAdapterTestAccess final
{
public:
  static bool start_raw_producer(
    RelativeMotionRosAdapter & adapter,
    const std::string & raw_topic);
};

}  // namespace detail

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RELATIVE_MOTION_ROS_ADAPTER_HPP_
