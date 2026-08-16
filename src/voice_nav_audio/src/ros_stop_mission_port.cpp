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

#include "ros_stop_mission_port.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace voice_nav_audio
{
namespace
{

constexpr auto kStopResponseDeadline = std::chrono::seconds(1);

StopMissionResponse transport_failure() noexcept
{
  return StopMissionResponse{StopMissionCode::TransportFailure, false};
}

}  // namespace

RosStopMissionPort::RosStopMissionPort(rclcpp::Node::SharedPtr node)
: node_(std::move(node)), state_(std::make_shared<State>())
{
  if (node_ == nullptr) {
    throw std::invalid_argument("RosStopMissionPort requires an existing Voice node");
  }
  client_ = node_->create_client<StopMission>("/mission/stop");
}

RosStopMissionPort::~RosStopMissionPort()
{
  const auto state = state_;
  if (state != nullptr) {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->alive = false;
    state->in_flight.reset();
  }
}

std::size_t RosStopMissionPort::request_count() const noexcept
{
  if (state_ == nullptr) {
    return 0U;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->request_count;
}

void RosStopMissionPort::request(
  const StopMissionRequest & request,
  StopMissionResponseSink & response_sink) noexcept
{
  if (state_ == nullptr || client_ == nullptr || node_ == nullptr) {
    response_sink.on_response(transport_failure());
    return;
  }

  std::shared_ptr<Pending> pending{};
  bool capacity_available = false;
  try {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->alive && state_->in_flight.expired()) {
      pending = std::make_shared<Pending>(state_, response_sink, state_->next_generation++);
      state_->in_flight = pending;
      capacity_available = true;
    }
  } catch (...) {
    response_sink.on_response(transport_failure());
    return;
  }
  if (!capacity_available) {
    response_sink.on_response(transport_failure());
    return;
  }

  auto finish_transport_failure = [&pending]() noexcept {
      complete_pending(pending, transport_failure());
    };
  try {
    if (!client_->service_is_ready()) {
      finish_transport_failure();
      return;
    }

    auto ros_request = std::make_shared<StopMission::Request>();
    ros_request->request_id = request.request_id;
    ros_request->source_instance_id = request.source_instance_id;
    ros_request->source_seq = request.source_seq;
    ros_request->reason = request.reason;
    pending->timeout_timer = node_->create_wall_timer(
      kStopResponseDeadline,
      [pending]() {
        complete_pending(pending, StopMissionResponse{StopMissionCode::Timeout, false});
      });
    (void)client_->async_send_request(
      ros_request,
      [pending](typename rclcpp::Client<StopMission>::SharedFuture future) {
        try {
          RosStopMissionPort::complete_pending(
            pending, RosStopMissionPort::map_response(future.get()));
        } catch (...) {
          RosStopMissionPort::complete_pending(pending, transport_failure());
        }
      });
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      if (state_->alive) {
        ++state_->request_count;
      }
    }
  } catch (...) {
    finish_transport_failure();
  }
}

void RosStopMissionPort::complete_pending(
  const std::shared_ptr<Pending> & pending,
  const StopMissionResponse & response) noexcept
{
  if (pending == nullptr || pending->completed.exchange(true)) {
    return;
  }
  auto timer = pending->timeout_timer;
  pending->timeout_timer.reset();
  if (timer != nullptr) {
    timer->cancel();
  }

  const auto state = pending->state.lock();
  if (state == nullptr) {
    return;
  }
  StopMissionResponseSink * response_sink = nullptr;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    const auto current = state->in_flight.lock();
    if (!state->alive || current.get() != pending.get() ||
      current->generation != pending->generation)
    {
      return;
    }
    state->in_flight.reset();
    response_sink = pending->response_sink;
  }
  if (response_sink != nullptr) {
    response_sink->on_response(response);
  }
}

StopMissionResponse RosStopMissionPort::map_response(
  const std::shared_ptr<const StopMission::Response> & response) noexcept
{
  if (response == nullptr) {
    return transport_failure();
  }
  switch (response->code) {
    case StopMission::Response::APPLIED:
      return StopMissionResponse{StopMissionCode::Applied, response->motion_inhibited};
    case StopMission::Response::DUPLICATE:
      return StopMissionResponse{StopMissionCode::Duplicate, response->motion_inhibited};
    case StopMission::Response::SAFETY_FAULT:
      return StopMissionResponse{StopMissionCode::SafetyFault, response->motion_inhibited};
    default:
      return transport_failure();
  }
}

}  // namespace voice_nav_audio
