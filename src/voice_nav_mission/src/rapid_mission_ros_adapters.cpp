// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "voice_nav_mission/rapid_mission_ros_adapters.hpp"

#include <chrono>
#include <future>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

#include <rclcpp_action/rclcpp_action.hpp>

#include "voice_nav_interfaces/action/execute_mission.hpp"
#include "voice_nav_interfaces/msg/mission_step.hpp"

namespace voice_nav_mission
{
namespace
{

using ExecuteMission = voice_nav_interfaces::action::ExecuteMission;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteMission>;

[[nodiscard]] bool same_token(
  const MotionToken & left, const MotionToken & right) noexcept
{
  return left.mission_id == right.mission_id &&
         left.admission_epoch == right.admission_epoch &&
         left.mission_generation == right.mission_generation &&
         left.step_generation == right.step_generation &&
         left.admission_generation == right.admission_generation;
}

[[nodiscard]] ChildResultCode child_code(const std::uint16_t code) noexcept
{
  if (code == ExecuteMission::Result::SUCCEEDED) {
    return ChildResultCode::Succeeded;
  }
  if (code == ExecuteMission::Result::DEPENDENCY_UNAVAILABLE) {
    return ChildResultCode::DependencyUnavailable;
  }
  if (code == ExecuteMission::Result::TIMEOUT) {
    return ChildResultCode::Timeout;
  }
  if (code == ExecuteMission::Result::SAFETY_FAULT) {
    return ChildResultCode::SafetyFault;
  }
  if (code == ExecuteMission::Result::INTERNAL_ERROR) {
    return ChildResultCode::InternalError;
  }
  return ChildResultCode::Failed;
}

}  // namespace

class RapidMissionDelegate::Impl final
  : public std::enable_shared_from_this<RapidMissionDelegate::Impl>
{
public:
  Impl(
    rclcpp::Node & node,
    const std::string & action_name)
  : client_(rclcpp_action::create_client<ExecuteMission>(&node, action_name))
  {
    if (action_name.empty() || action_name == "/mission/execute") {
      throw std::invalid_argument(
              "rapid delegate action must be non-empty and private");
    }
  }

  [[nodiscard]] bool healthy() const
  {
    return client_->action_server_is_ready();
  }

  void start(
    const MotionToken & token,
    const MissionStep & step,
    MissionChildPort::FeedbackCallback feedback,
    MissionChildPort::ResultCallback result)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (token_.has_value()) {
        throw std::runtime_error("rapid Mission delegate is already active");
      }
      token_ = token;
      feedback_ = std::move(feedback);
      result_ = std::move(result);
      goal_handle_.reset();
    }

    ExecuteMission::Goal goal;
    goal.source_instance_id = "runtime-rapid-port";
    goal.source_seq = token.step_generation;
    goal.runtime_instance_id = "";
    goal.admission_epoch = 0U;
    voice_nav_interfaces::msg::MissionStep message;
    message.kind = step.kind;
    message.distance_m = step.distance_m;
    message.angle_rad = step.angle_rad;
    message.target_id = step.target_id;
    goal.steps.push_back(std::move(message));

    rclcpp_action::Client<ExecuteMission>::SendGoalOptions options;
    options.goal_response_callback =
      [weak = weak_from_this(), token](const GoalHandle::SharedPtr handle) {
        if (const auto self = weak.lock()) {
          self->on_goal_response(token, handle);
        }
      };
    options.feedback_callback =
      [weak = weak_from_this(), token](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const ExecuteMission::Feedback> feedback_message) {
        if (const auto self = weak.lock()) {
          self->on_feedback(token, feedback_message->progress);
        }
      };
    options.result_callback =
      [weak = weak_from_this(), token](const GoalHandle::WrappedResult & wrapped) {
        if (const auto self = weak.lock()) {
          ChildResult result_value;
          if (wrapped.result) {
            result_value.code = child_code(wrapped.result->code);
            result_value.detail = wrapped.result->detail;
          } else {
            result_value.code = ChildResultCode::InternalError;
            result_value.detail = "rapid delegate returned no Result";
          }
          self->complete(token, std::move(result_value));
        }
      };
    try {
      (void)client_->async_send_goal(goal, options);
    } catch (const std::exception & error) {
      complete(token, ChildResult{
          ChildResultCode::DependencyUnavailable,
          std::string{"rapid delegate send failed: "} + error.what()});
    }
  }

  [[nodiscard]] bool cancel(
    const MotionToken & token,
    const SteadyClockPort::TimePoint deadline)
  {
    GoalHandle::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!token_.has_value() || !same_token(*token_, token)) {
        return true;
      }
      handle = goal_handle_;
      token_.reset();
      feedback_ = {};
      result_ = {};
      goal_handle_.reset();
    }
    if (!handle) {
      return true;
    }
    try {
      const auto response = client_->async_cancel_goal(handle);
      return response.wait_until(deadline) == std::future_status::ready;
    } catch (...) {
      return false;
    }
  }

private:
  void on_goal_response(
    const MotionToken & token,
    const GoalHandle::SharedPtr & handle)
  {
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stale = !token_.has_value() || !same_token(*token_, token);
      if (!stale) {
        if (!handle) {
          // Complete after releasing the lock.
        } else {
          goal_handle_ = handle;
          return;
        }
      }
    }
    if (stale && handle) {
      (void)client_->async_cancel_goal(handle);
      return;
    }
    if (!handle) {
      complete(token, ChildResult{
          ChildResultCode::DependencyUnavailable,
          "rapid delegate rejected the child Goal"});
    }
  }

  void on_feedback(const MotionToken & token, const double progress)
  {
    MissionChildPort::FeedbackCallback callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!token_.has_value() || !same_token(*token_, token)) {
        return;
      }
      callback = feedback_;
    }
    if (callback) {
      callback(token, progress);
    }
  }

  void complete(const MotionToken & token, ChildResult result)
  {
    MissionChildPort::ResultCallback callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!token_.has_value() || !same_token(*token_, token)) {
        return;
      }
      callback = std::move(result_);
      token_.reset();
      feedback_ = {};
      goal_handle_.reset();
    }
    if (callback) {
      callback(token, result);
    }
  }

  rclcpp_action::Client<ExecuteMission>::SharedPtr client_;
  mutable std::mutex mutex_;
  std::optional<MotionToken> token_;
  MissionChildPort::FeedbackCallback feedback_;
  MissionChildPort::ResultCallback result_;
  GoalHandle::SharedPtr goal_handle_;
};

RapidMissionDelegate::RapidMissionDelegate(
  rclcpp::Node & node, std::string action_name)
: impl_(std::make_shared<Impl>(node, action_name))
{
}

RapidMissionDelegate::~RapidMissionDelegate() = default;

bool RapidMissionDelegate::healthy() const
{
  return impl_->healthy();
}

void RapidMissionDelegate::start(
  const MotionToken & token,
  const MissionStep & step,
  MissionChildPort::FeedbackCallback feedback,
  MissionChildPort::ResultCallback result)
{
  impl_->start(token, step, std::move(feedback), std::move(result));
}

bool RapidMissionDelegate::cancel(
  const MotionToken & token,
  const SteadyClockPort::TimePoint deadline)
{
  return impl_->cancel(token, deadline);
}

RapidNavigationPort::RapidNavigationPort(
  std::shared_ptr<RapidMissionDelegate> delegate)
: delegate_(std::move(delegate))
{
  if (!delegate_) {
    throw std::invalid_argument("RapidNavigationPort requires a delegate");
  }
}

bool RapidNavigationPort::healthy() const
{
  return delegate_->healthy();
}

void RapidNavigationPort::start(
  const MotionToken & token,
  const MissionStep & step,
  FeedbackCallback feedback,
  ResultCallback result)
{
  delegate_->start(token, step, std::move(feedback), std::move(result));
}

bool RapidNavigationPort::cancel(
  const MotionToken & token,
  const SteadyClockPort::TimePoint deadline)
{
  return delegate_->cancel(token, deadline);
}

RapidRelativeMotionPort::RapidRelativeMotionPort(
  std::shared_ptr<RapidMissionDelegate> delegate)
: delegate_(std::move(delegate))
{
  if (!delegate_) {
    throw std::invalid_argument("RapidRelativeMotionPort requires a delegate");
  }
}

bool RapidRelativeMotionPort::healthy() const
{
  return delegate_->healthy();
}

void RapidRelativeMotionPort::start(
  const MotionToken & token,
  const MissionStep & step,
  FeedbackCallback feedback,
  ResultCallback result)
{
  delegate_->start(token, step, std::move(feedback), std::move(result));
}

bool RapidRelativeMotionPort::cancel(
  const MotionToken & token,
  const SteadyClockPort::TimePoint deadline)
{
  return delegate_->cancel(token, deadline);
}

RapidMapStorePort::RapidMapStorePort(
  std::shared_ptr<RapidMissionDelegate> delegate)
: delegate_(std::move(delegate))
{
  if (!delegate_) {
    throw std::invalid_argument("RapidMapStorePort requires a delegate");
  }
}

bool RapidMapStorePort::healthy() const
{
  return delegate_->healthy();
}

void RapidMapStorePort::start(
  const MotionToken & token,
  const MissionStep & step,
  FeedbackCallback feedback,
  ResultCallback result)
{
  delegate_->start(token, step, std::move(feedback), std::move(result));
}

bool RapidMapStorePort::cancel(
  const MotionToken & token,
  const SteadyClockPort::TimePoint deadline)
{
  return delegate_->cancel(token, deadline);
}

}  // namespace voice_nav_mission
