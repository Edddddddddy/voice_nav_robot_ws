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

#ifndef VOICE_NAV_MISSION__MOTION_AUTHORITY_ROS_ADAPTER_HPP_
#define VOICE_NAV_MISSION__MOTION_AUTHORITY_ROS_ADAPTER_HPP_

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

#include "voice_nav_mission/mission_authority_convergence.hpp"
#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/msg/internal_motion_gate_state.hpp"
#include "voice_nav_mission/srv/internal_motion_gate_control.hpp"

namespace voice_nav_mission
{

namespace detail
{

// Package-private monotonic merge seam shared by the ROS Adapter and its
// deterministic interleaving tests.  Once identity A is retired by B, no
// delayed state or service response from A can restore it.
class GateSnapshotWatermark final
{
public:
  [[nodiscard]] bool merge(
    const GateSnapshot & incoming,
    GateSnapshot & accepted);
  [[nodiscard]] const GateSnapshot & snapshot() const noexcept;
  [[nodiscard]] bool set_endpoint_available(
    bool available,
    GateSnapshot & accepted) noexcept;

private:
  GateSnapshot snapshot_{};
  std::unordered_set<std::string> retired_gate_instance_ids_;
  std::deque<std::string> retired_gate_instance_order_;
};

}  // namespace detail

class RosMotionAuthorityPort final : public MotionAuthorityPort
{
public:
  using GateChangedCallback = std::function<void(const GateSnapshot &)>;

  RosMotionAuthorityPort(
    rclcpp::Node & node,
    std::chrono::milliseconds control_response_deadline,
    std::chrono::milliseconds stop_barrier,
    GateChangedCallback callback);

  [[nodiscard]] GateSnapshot snapshot() const override;
  [[nodiscard]] AuthorityResult prepare(
    const AuthorityOperation & operation) override;
  [[nodiscard]] AuthorityResult open(
    const AuthorityOperation & operation) override;
  [[nodiscard]] AuthorityResult renew(
    const AuthorityOperation & operation) override;
  [[nodiscard]] AuthorityResult inhibit(
    const AuthorityOperation & operation) override;
  [[nodiscard]] std::optional<GateSnapshot> accept_rearm_snapshot(
    const GateSnapshot & candidate) const noexcept override;

  void refresh_endpoint();

private:
  [[nodiscard]] bool graph_endpoint_available() const;
  [[nodiscard]] static std::uint8_t operation_code(
    AuthorityOperationKind kind);
  [[nodiscard]] AuthorityResult send_once(
    const AuthorityOperation & operation,
    std::uint8_t operation_code,
    std::chrono::steady_clock::time_point rpc_deadline,
    std::chrono::steady_clock::time_point overall_deadline);
  [[nodiscard]] AuthorityResult unavailable(
    std::string detail,
    bool retryable) const;

  rclcpp::Node & node_;
  std::chrono::milliseconds control_response_deadline_;
  std::chrono::milliseconds stop_barrier_;
  std::unique_ptr<MissionAuthorityAdapter> authority_adapter_;
  GateChangedCallback callback_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Client<voice_nav_mission::srv::InternalMotionGateControl>::SharedPtr
    client_;
  rclcpp::Subscription<voice_nav_mission::msg::InternalMotionGateState>::SharedPtr
    subscription_;
  mutable std::mutex mutex_;
  detail::GateSnapshotWatermark snapshot_watermark_;
  bool state_sample_available_{false};
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__MOTION_AUTHORITY_ROS_ADAPTER_HPP_
