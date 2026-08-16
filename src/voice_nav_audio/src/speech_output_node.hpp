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

#ifndef VOICE_NAV_AUDIO__SPEECH_OUTPUT_NODE_HPP_
#define VOICE_NAV_AUDIO__SPEECH_OUTPUT_NODE_HPP_

#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "speech_output_core.hpp"
#include "voice_nav_interfaces/action/speak.hpp"

namespace voice_nav_audio
{

// Package-private observation seam for composition tests. It exposes no ROS
// endpoint and receives only playback facts already produced by this node.
class SpeechOutputTraceSink
{
public:
  virtual ~SpeechOutputTraceSink() = default;

  virtual void on_played(std::uint64_t scope_id, std::uint64_t samples) noexcept = 0;
  virtual void on_result(const SpeechResult & result) noexcept = 0;
};

class SpeechOutputNode final : public rclcpp::Node, public SpeechOutputControl,
  private SpeechOutputObserver
{
public:
  using Speak = voice_nav_interfaces::action::Speak;
  using GoalHandleSpeak = rclcpp_action::ServerGoalHandle<Speak>;

  SpeechOutputNode(
    AudioEngine & engine, std::unique_ptr<TtsAdapter> tts,
    SpeechOutputTraceSink * trace = nullptr);
  ~SpeechOutputNode() override;

  void pump() noexcept;
  [[nodiscard]] bool admit_ordinary_wake() noexcept override;
  [[nodiscard]] bool interrupt_for_stop() noexcept override;

private:
  [[nodiscard]] rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Speak::Goal> goal);
  [[nodiscard]] rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSpeak> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleSpeak> goal_handle);
  void start_worker(std::uint64_t scope_id);
  void complete(
    const std::shared_ptr<GoalHandleSpeak> & goal_handle, const SpeechResult & result) noexcept;
  [[nodiscard]] static SpeechGoal domain_goal(const Speak::Goal & goal);

  void on_played(std::uint64_t scope_id, std::uint64_t samples) noexcept override;
  void on_result(const SpeechResult & result) noexcept override;

  AudioEngine & engine_;
  std::unique_ptr<TtsAdapter> tts_;
  SpeechOutputCore core_;
  rclcpp_action::Server<Speak>::SharedPtr action_server_{};
  rclcpp::TimerBase::SharedPtr pump_timer_{};
  std::mutex core_mutex_{};
  std::mutex handles_mutex_{};
  std::mutex worker_mutex_{};
  std::thread tts_worker_{};
  std::unordered_map<std::uint64_t, std::shared_ptr<GoalHandleSpeak>> handles_{};
  std::unordered_map<const GoalHandleSpeak *, std::uint64_t> scope_by_handle_{};
  std::uint64_t cancel_requested_scope_id_{0U};
  SpeechOutputTraceSink * trace_{nullptr};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__SPEECH_OUTPUT_NODE_HPP_
