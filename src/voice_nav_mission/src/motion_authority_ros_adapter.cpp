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

#include "voice_nav_mission/motion_authority_ros_adapter.hpp"

#include <rmw/qos_profiles.h>

#include <future>
#include <optional>
#include <string>
#include <utility>

namespace voice_nav_mission
{
namespace
{

using GateControl = voice_nav_mission::srv::InternalMotionGateControl;
using GateStateMessage = voice_nav_mission::msg::InternalMotionGateState;

constexpr char kGateControlService[] = "/motion_gate/internal/control";
constexpr char kGateStateTopic[] = "/motion_gate/internal/state";

GateState gate_state_from_message(std::uint8_t state)
{
  switch (state) {
    case GateStateMessage::INHIBITED:
      return GateState::Inhibited;
    case GateStateMessage::PREPARED:
      return GateState::Prepared;
    case GateStateMessage::ARMED:
      return GateState::Armed;
    default:
      return GateState::Faulted;
  }
}

}  // namespace

bool detail::GateSnapshotWatermark::merge(
  const GateSnapshot & incoming,
  GateSnapshot & accepted)
{
  if (incoming.gate_instance_id.empty()) {
    return false;
  }
  if (snapshot_.gate_instance_id.empty()) {
    snapshot_ = incoming;
    accepted = snapshot_;
    return true;
  }
  if (incoming.gate_instance_id != snapshot_.gate_instance_id) {
    if (retired_gate_instance_ids_.count(incoming.gate_instance_id) != 0U) {
      return false;
    }
    retired_gate_instance_ids_.insert(snapshot_.gate_instance_id);
    snapshot_ = incoming;
    accepted = snapshot_;
    return true;
  }
  if (incoming.control_seq < snapshot_.control_seq) {
    return false;
  }
  if (incoming.control_seq == snapshot_.control_seq) {
    const bool same_control_tuple =
      incoming.lease_id == snapshot_.lease_id &&
      incoming.state == snapshot_.state &&
      incoming.motion_inhibited == snapshot_.motion_inhibited &&
      incoming.zero_selected == snapshot_.zero_selected &&
      incoming.candidate_topic == snapshot_.candidate_topic &&
      incoming.authority_live == snapshot_.authority_live &&
      incoming.writer_bound == snapshot_.writer_bound;
    if (!same_control_tuple) {
      return false;
    }
    snapshot_.endpoint_available = incoming.endpoint_available;
    snapshot_.zero_published =
      snapshot_.endpoint_available && snapshot_.zero_published &&
      incoming.zero_published;
    accepted = snapshot_;
    return true;
  }
  snapshot_ = incoming;
  accepted = snapshot_;
  return true;
}

const GateSnapshot & detail::GateSnapshotWatermark::snapshot() const noexcept
{
  return snapshot_;
}

bool detail::GateSnapshotWatermark::set_endpoint_available(
  const bool available,
  GateSnapshot & accepted) noexcept
{
  if (
    snapshot_.endpoint_available == available &&
    (available || !snapshot_.zero_published))
  {
    return false;
  }
  snapshot_.endpoint_available = available;
  if (!available) {
    snapshot_.zero_published = false;
  }
  accepted = snapshot_;
  return true;
}

RosMotionAuthorityPort::RosMotionAuthorityPort(
  rclcpp::Node & node,
  std::chrono::milliseconds control_response_deadline,
  std::chrono::milliseconds stop_barrier,
  GateChangedCallback callback)
: node_(node),
  control_response_deadline_(control_response_deadline),
  stop_barrier_(stop_barrier),
  callback_(std::move(callback))
{
  client_callback_group_ = node_.create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  client_ = node_.create_client<GateControl>(
    kGateControlService,
    rmw_qos_profile_services_default,
    client_callback_group_);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.reliable().transient_local();
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = client_callback_group_;
  subscription_ = node_.create_subscription<GateStateMessage>(
    kGateStateTopic,
    qos,
    [this](const GateStateMessage::ConstSharedPtr message) {
      const bool graph_available = graph_endpoint_available();
      GateSnapshot snapshot;
      bool accepted = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto incoming = GateSnapshot{
          message->gate_instance_id,
          message->control_seq,
          message->lease_id,
          gate_state_from_message(message->state),
          graph_available,
          message->motion_inhibited,
          message->zero_selected,
          graph_available && message->zero_publish_seq != 0U &&
          message->zero_publish_seq >= message->output_publish_seq,
          message->candidate_topic,
          message->authority_live,
          message->writer_bound};
        accepted = snapshot_watermark_.merge(incoming, snapshot);
        if (accepted) {
          state_sample_available_ = graph_available;
        }
      }
      if (accepted && callback_) {
        callback_(snapshot);
      }
    },
    subscription_options);
  authority_adapter_ = std::make_unique<MissionAuthorityAdapter>(
    stop_barrier_,
    control_response_deadline_,
    []() {return std::chrono::steady_clock::now();},
    [this](
      const AuthorityOperation & current,
      AuthorityOperationKind kind,
      std::chrono::steady_clock::time_point attempt_deadline,
      std::chrono::steady_clock::time_point overall_deadline) {
      return send_once(
        current,
        operation_code(kind),
        attempt_deadline,
        overall_deadline);
    },
    [this]() {refresh_endpoint();});
}

GateSnapshot RosMotionAuthorityPort::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_watermark_.snapshot();
}

AuthorityResult RosMotionAuthorityPort::prepare(
  const AuthorityOperation & operation)
{
  return authority_adapter_->prepare(operation);
}

AuthorityResult RosMotionAuthorityPort::open(
  const AuthorityOperation & operation)
{
  return authority_adapter_->open(operation);
}

AuthorityResult RosMotionAuthorityPort::renew(
  const AuthorityOperation & operation)
{
  return authority_adapter_->renew(operation);
}

AuthorityResult RosMotionAuthorityPort::inhibit(
  const AuthorityOperation & operation)
{
  return authority_adapter_->inhibit(operation);
}

std::optional<GateSnapshot> RosMotionAuthorityPort::accept_rearm_snapshot(
  const GateSnapshot & candidate) const noexcept
{
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto current = snapshot_watermark_.snapshot();
    if (
      current.gate_instance_id != candidate.gate_instance_id ||
      current.control_seq < candidate.control_seq ||
      !current.endpoint_available || current.state != GateState::Inhibited ||
      !current.motion_inhibited || !current.zero_selected ||
      !current.zero_published)
    {
      return std::nullopt;
    }
    return current;
  } catch (...) {
    return std::nullopt;
  }
}

void RosMotionAuthorityPort::refresh_endpoint()
{
  const bool available = graph_endpoint_available();
  std::optional<GateSnapshot> changed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!available) {
      GateSnapshot snapshot;
      if (snapshot_watermark_.set_endpoint_available(false, snapshot)) {
        changed = snapshot;
      }
      state_sample_available_ = false;
    } else {
      const bool endpoint_recovered =
        state_sample_available_ &&
        !snapshot_watermark_.snapshot().endpoint_available;
      if (endpoint_recovered) {
        GateSnapshot snapshot;
        if (snapshot_watermark_.set_endpoint_available(true, snapshot)) {
          changed = snapshot;
        }
      }
    }
  }
  if (changed.has_value() && callback_) {
    callback_(*changed);
  }
}

bool RosMotionAuthorityPort::graph_endpoint_available() const
{
  return client_->service_is_ready() &&
         node_.count_publishers(kGateStateTopic) > 0U;
}

std::uint8_t RosMotionAuthorityPort::operation_code(
  AuthorityOperationKind kind)
{
  switch (kind) {
    case AuthorityOperationKind::Prepare:
      return GateControl::Request::PREPARE;
    case AuthorityOperationKind::Open:
      return GateControl::Request::OPEN;
    case AuthorityOperationKind::Renew:
      return GateControl::Request::RENEW;
    case AuthorityOperationKind::Inhibit:
      return GateControl::Request::INHIBIT;
  }
  return GateControl::Request::PREPARE;
}

AuthorityResult RosMotionAuthorityPort::send_once(
  const AuthorityOperation & operation,
  std::uint8_t operation_code_value,
  std::chrono::steady_clock::time_point rpc_deadline,
  std::chrono::steady_clock::time_point overall_deadline)
{
  const auto now = std::chrono::steady_clock::now();
  if (now >= overall_deadline) {
    return unavailable(
      "MotionGate control operation reached its steady deadline", false);
  }
  if (now >= rpc_deadline) {
    return unavailable(
      "MotionGate control operation reached its single-operation deadline",
      now < overall_deadline);
  }
  const auto remaining = [&rpc_deadline]() {
      const auto current = std::chrono::steady_clock::now();
      return current >= rpc_deadline ? std::chrono::milliseconds(0) :
             std::chrono::duration_cast<std::chrono::milliseconds>(
        rpc_deadline - current);
    };
  if (!client_->wait_for_service(remaining())) {
    return unavailable(
      "MotionGate control service is unavailable",
      std::chrono::steady_clock::now() < overall_deadline);
  }
  if (std::chrono::steady_clock::now() >= overall_deadline) {
    return unavailable(
      "MotionGate control operation reached its steady deadline", false);
  }
  auto request = std::make_shared<GateControl::Request>();
  request->operation = operation_code_value;
  request->request_id = operation.request_id;
  request->gate_instance_id = operation.gate_instance_id;
  request->expected_control_seq = operation.expected_control_seq;
  request->lease_id = operation.lease_id;
  auto future = client_->async_send_request(request);
  if (future.wait_for(remaining()) != std::future_status::ready) {
    return unavailable(
      "MotionGate control response exceeded its single-operation deadline",
      std::chrono::steady_clock::now() < overall_deadline);
  }
  if (std::chrono::steady_clock::now() >= overall_deadline) {
    return unavailable(
      "MotionGate control operation reached its steady deadline", false);
  }
  const auto response = future.get();
  const bool endpoint_available = graph_endpoint_available();
  GateSnapshot snapshot;
  bool tuple_accepted = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto incoming = GateSnapshot{
      response->gate_instance_id,
      response->control_seq,
      response->lease_id,
      gate_state_from_message(response->state),
      endpoint_available,
      response->motion_inhibited,
      response->zero_selected,
      response->zero_published && endpoint_available,
      response->candidate_topic,
      response->authority_live,
      response->writer_bound};
    tuple_accepted = snapshot_watermark_.merge(incoming, snapshot);
    if (!tuple_accepted) {
      snapshot = snapshot_watermark_.snapshot();
    } else {
      state_sample_available_ = snapshot.endpoint_available;
    }
  }
  if (tuple_accepted && callback_) {
    callback_(snapshot);
  }
  const bool response_applied =
    response->code == GateControl::Response::APPLIED ||
    response->code == GateControl::Response::DUPLICATE;
  const bool applied = tuple_accepted && response_applied;
  const bool zero = response->motion_inhibited && response->zero_selected &&
    response->zero_published;
  const bool retryable =
    !tuple_accepted ||
    response->reason == GateControl::Response::STALE_GATE ||
    response->reason == GateControl::Response::STALE_SEQUENCE ||
    response->reason == GateControl::Response::STALE_LEASE;
  auto result = AuthorityResult{
    applied,
    zero,
    retryable,
    snapshot,
    response->lease_id,
    response->detail};
  if (!applied ||
    (operation_code_value == GateControl::Request::RENEW &&
    (!response->authority_live || response->motion_inhibited)))
  {
    RCLCPP_WARN(
      node_.get_logger(),
      "MotionGate control rejected: operation=%u code=%u reason=%u state=%u detail=%s",
      static_cast<unsigned int>(operation_code_value),
      static_cast<unsigned int>(response->code),
      static_cast<unsigned int>(response->reason),
      static_cast<unsigned int>(response->state),
      response->detail.c_str());
  }
  result.tuple_stale = retryable;
  return result;
}

AuthorityResult RosMotionAuthorityPort::unavailable(
  std::string detail,
  bool retryable) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return AuthorityResult{
    false, false, retryable, snapshot_watermark_.snapshot(), {}, std::move(detail)};
}

}  // namespace voice_nav_mission
