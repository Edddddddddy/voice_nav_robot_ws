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

#include "speech_output_node.hpp"

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace voice_nav_audio
{
namespace
{

std::unique_ptr<TtsAdapter> checked_tts(std::unique_ptr<TtsAdapter> tts)
{
  if (!tts) {
    throw std::invalid_argument("SpeechOutputNode requires a TtsAdapter");
  }
  return tts;
}

constexpr std::uint64_t kNanosecondsPerSecond = 1000000000U;

}  // namespace

SpeechOutputNode::SpeechOutputNode(AudioEngine & engine, std::unique_ptr<TtsAdapter> tts)
: Node("voice_speech_output"), engine_(engine), tts_(checked_tts(std::move(tts))),
  core_(engine_, *tts_, static_cast<SpeechOutputObserver &>(*this))
{
  action_server_ = rclcpp_action::create_server<Speak>(
    this, "/voice/speak",
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Speak::Goal> goal) {
      return handle_goal(uuid, std::move(goal));
    },
    [this](const std::shared_ptr<GoalHandleSpeak> goal_handle) {
      return handle_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleSpeak> goal_handle) {
      handle_accepted(goal_handle);
    });
  pump_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {pump();});
}

SpeechOutputNode::~SpeechOutputNode()
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (tts_worker_.joinable()) {
    tts_worker_.join();
  }
}

void SpeechOutputNode::pump() noexcept
{
  try {
    std::uint64_t canceled_scope_id = 0U;
    {
      std::lock_guard<std::mutex> lock(handles_mutex_);
      if (cancel_requested_scope_id_ != 0U) {
        const auto found = handles_.find(cancel_requested_scope_id_);
        if (found == handles_.end()) {
          cancel_requested_scope_id_ = 0U;
        } else if (found->second->is_canceling()) {
          canceled_scope_id = cancel_requested_scope_id_;
          cancel_requested_scope_id_ = 0U;
        } else {
          return;
        }
      }
    }

    std::uint64_t scope_id = 0U;
    {
      std::lock_guard<std::mutex> lock(core_mutex_);
      if (canceled_scope_id != 0U) {
        (void)core_.cancel(canceled_scope_id);
      } else {
        (void)core_.advance();
        scope_id = core_.ready_scope_id();
      }
    }
    if (scope_id != 0U) {
      start_worker(scope_id);
    }
  } catch (...) {
  }
}

rclcpp_action::GoalResponse SpeechOutputNode::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Speak::Goal>)
{
  // Invalid and concurrent goals are accepted so clients receive the frozen,
  // structured exactly-once Speak Result rather than a transport-only reject.
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SpeechOutputNode::handle_cancel(
  const std::shared_ptr<GoalHandleSpeak> goal_handle)
{
  std::uint64_t scope_id = 0U;
  {
    std::lock_guard<std::mutex> lock(handles_mutex_);
    const auto found = scope_by_handle_.find(goal_handle.get());
    if (found == scope_by_handle_.end()) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    scope_id = found->second;
    cancel_requested_scope_id_ = scope_id;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SpeechOutputNode::handle_accepted(const std::shared_ptr<GoalHandleSpeak> goal_handle)
{
  SpeechAdmission admission{};
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    admission = core_.start(domain_goal(*goal_handle->get_goal()));
  }
  if (admission.has_immediate_result) {
    complete(goal_handle, admission.immediate_result);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(handles_mutex_);
    handles_.emplace(admission.scope_id, goal_handle);
    scope_by_handle_.emplace(goal_handle.get(), admission.scope_id);
  }
  if (admission.start_synthesis) {
    start_worker(admission.scope_id);
  }
}

void SpeechOutputNode::start_worker(const std::uint64_t scope_id)
{
  std::lock_guard<std::mutex> worker_lock(worker_mutex_);
  if (tts_worker_.joinable()) {
    tts_worker_.join();
  }
  tts_worker_ = std::thread([this, scope_id]() {
      std::lock_guard<std::mutex> core_lock(core_mutex_);
      (void)core_.begin_synthesis(scope_id);
    });
}

void SpeechOutputNode::complete(
  const std::shared_ptr<GoalHandleSpeak> & goal_handle, const SpeechResult & result) noexcept
{
  try {
    auto action_result = std::make_shared<Speak::Result>();
    action_result->code = static_cast<std::uint16_t>(result.code);
    action_result->detail = result.detail;
    if (result.code == SpeechResultCode::Completed) {
      goal_handle->succeed(action_result);
    } else if (result.code == SpeechResultCode::Canceled && goal_handle->is_canceling()) {
      goal_handle->canceled(action_result);
    } else {
      goal_handle->abort(action_result);
    }
  } catch (...) {
  }
}

SpeechGoal SpeechOutputNode::domain_goal(const Speak::Goal & goal)
{
  SpeechPriority priority = SpeechPriority::Normal;
  if (goal.priority == Speak::Goal::URGENT) {
    priority = SpeechPriority::Urgent;
  } else if (goal.priority != Speak::Goal::NORMAL) {
    priority = static_cast<SpeechPriority>(goal.priority);
  }
  return SpeechGoal{
    goal.source_instance_id, goal.source_seq, goal.session_id, goal.turn_id, priority,
    goal.text, goal.allow_barge_in};
}

void SpeechOutputNode::on_played(
  const std::uint64_t scope_id, const std::uint64_t samples) noexcept
{
  try {
    std::shared_ptr<GoalHandleSpeak> goal_handle{};
    {
      std::lock_guard<std::mutex> lock(handles_mutex_);
      const auto found = handles_.find(scope_id);
      if (found == handles_.end()) {
        return;
      }
      goal_handle = found->second;
    }
    auto feedback = std::make_shared<Speak::Feedback>();
    feedback->played.sec = static_cast<std::int32_t>(samples / AudioEngine::kSampleRate);
    feedback->played.nanosec = static_cast<std::uint32_t>(
      (samples % AudioEngine::kSampleRate) * kNanosecondsPerSecond / AudioEngine::kSampleRate);
    goal_handle->publish_feedback(feedback);
  } catch (...) {
  }
}

void SpeechOutputNode::on_result(const SpeechResult & result) noexcept
{
  try {
    std::shared_ptr<GoalHandleSpeak> goal_handle{};
    {
      std::lock_guard<std::mutex> lock(handles_mutex_);
      const auto found = handles_.find(result.scope_id);
      if (found == handles_.end()) {
        return;
      }
      goal_handle = found->second;
      scope_by_handle_.erase(goal_handle.get());
      handles_.erase(found);
      if (cancel_requested_scope_id_ == result.scope_id) {
        cancel_requested_scope_id_ = 0U;
      }
    }
    complete(goal_handle, result);
  } catch (...) {
  }
}

}  // namespace voice_nav_audio
