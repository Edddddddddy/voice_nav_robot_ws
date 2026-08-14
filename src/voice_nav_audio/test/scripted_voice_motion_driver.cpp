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

// Test-only two-turn driver.  It owns no installed executable or endpoint.

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "speech_input_node.hpp"
#include "voice_nav_interfaces/msg/mission_state.hpp"

namespace voice_nav_audio
{
namespace
{

using namespace std::chrono_literals;

class ScriptedRecognizer final : public SpeechRecognizerAdapter
{
public:
  void process_frame(
    const CleanedAudioFrame & frame,
    SpeechEventSink & sink) noexcept override
  {
    if (frame.audio_seq == 1U || frame.audio_seq == 4U) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    } else if (frame.audio_seq == 2U || frame.audio_seq == 5U) {
      sink.on_speech_event(SpeechRecognitionEvent::activity(frame, active_scope_));
    } else if (frame.audio_seq == 3U) {
      sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
        frame, active_scope_, "前进半米", 1.0F));
    } else if (frame.audio_seq == 6U) {
      sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
        frame, active_scope_, "绕个弯", 1.0F));
    }
  }

  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept override
  {
    active_scope_ = scope;
  }

  void on_turn_scope_retired(const TurnScopeIdentity &) noexcept override
  {
    active_scope_ = TurnScopeIdentity{};
  }

private:
  TurnScopeIdentity active_scope_{};
};

CleanedAudioFrame cleaned_frame(const std::uint64_t sequence)
{
  CleanedAudioFrame frame{};
  frame.audio_generation = 1U;
  frame.audio_seq = sequence;
  frame.samples.fill(0);
  return frame;
}

bool has_endpoint(
  const std::vector<rclcpp::TopicEndpointInfo> & endpoints,
  const std::string & node_name)
{
  return std::any_of(
    endpoints.cbegin(), endpoints.cend(), [&node_name](const auto & endpoint) {
      return endpoint.node_name() == node_name && endpoint.node_namespace() == "/";
    });
}

bool graph_is_ready(rclcpp::Node & node)
{
  return has_endpoint(
    node.get_subscriptions_info_by_topic("/voice/turn"), "agent_node") &&
         has_endpoint(
    node.get_subscriptions_info_by_topic("/mission/execute/_action/status"),
    "agent_node") &&
         has_endpoint(
    node.get_publishers_info_by_topic("/mission/state"), "mission_runtime_node") &&
         has_endpoint(
    node.get_publishers_info_by_topic("/voice/speak/_action/status"),
    "voice_agent_gazebo_probe");
}

bool wait_for_graph(rclcpp::Node & node)
{
  const auto graph_event = node.get_graph_event();
  const auto deadline = std::chrono::steady_clock::now() + 30s;
  while (!graph_is_ready(node) && std::chrono::steady_clock::now() < deadline) {
    node.wait_for_graph_change(
      graph_event,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline - std::chrono::steady_clock::now()));
    graph_event->check_and_clear();
  }
  return graph_is_ready(node);
}

class CompletionObserver final
{
public:
  CompletionObserver()
  : node_(std::make_shared<rclcpp::Node>("scripted_voice_motion_driver_observer"))
  {
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_subscription_ = node_->create_subscription<voice_nav_interfaces::msg::MissionState>(
      "/mission/state", state_qos,
      [this](const voice_nav_interfaces::msg::MissionState::SharedPtr state) {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool gate_armed = state->gate_state ==
        voice_nav_interfaces::msg::MissionState::GATE_ARMED;
        runtime_ready_ = state->availability ==
        voice_nav_interfaces::msg::MissionState::AVAILABLE &&
        state->gate_state == voice_nav_interfaces::msg::MissionState::GATE_INHIBITED;
        if (gate_armed) {
          gate_was_armed_ = true;
        } else if (gate_was_armed_ && runtime_ready_) {
          ++completed_gate_epochs_;
          gate_was_armed_ = false;
        }
        condition_.notify_all();
      });
    mission_status_subscription_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/mission/execute/_action/status", rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr status_array) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & status : status_array->status_list) {
          if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
            successful_missions_.emplace(
              reinterpret_cast<const char *>(status.goal_info.goal_id.uuid.data()),
              status.goal_info.goal_id.uuid.size());
          }
        }
        condition_.notify_all();
      });
    speak_status_subscription_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/voice/speak/_action/status", rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr status_array) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & status : status_array->status_list) {
          if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
            successful_speeches_.emplace(
              reinterpret_cast<const char *>(status.goal_info.goal_id.uuid.data()),
              status.goal_info.goal_id.uuid.size());
          }
        }
        condition_.notify_all();
      });
  }

  [[nodiscard]] const rclcpp::Node::SharedPtr & node() const noexcept {return node_;}

  [[nodiscard]] bool wait_for_runtime_ready()
  {
    return wait_for([](const auto & self) {
               return self.runtime_ready_;
    });
  }
  [[nodiscard]] bool wait_for_first_completion()
  {
    return wait_for([](const auto & self) {
               return self.successful_missions_.size() >= 1U &&
                      self.successful_speeches_.size() >= 1U;
    });
  }
  [[nodiscard]] bool wait_for_first_safe_gate_epoch()
  {
    return wait_for([](const auto & self) {
               return self.completed_gate_epochs_ >= 1U;
    });
  }
  [[nodiscard]] bool wait_for_second_completion()
  {
    return wait_for([](const auto & self) {
               return self.successful_missions_.size() >= 2U &&
                      self.successful_speeches_.size() >= 2U;
    });
  }

private:
  template<typename Predicate>
  bool wait_for(Predicate predicate)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_until(
      lock, std::chrono::steady_clock::now() + 60s,
      [this, &predicate]() {return predicate(*this);});
  }

  rclcpp::Node::SharedPtr node_{};
  rclcpp::Subscription<voice_nav_interfaces::msg::MissionState>::SharedPtr state_subscription_{};
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr mission_status_subscription_{};
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr speak_status_subscription_{};
  std::mutex mutex_{};
  std::condition_variable condition_{};
  bool runtime_ready_{false};
  bool gate_was_armed_{false};
  std::size_t completed_gate_epochs_{0U};
  std::set<std::string> successful_missions_{};
  std::set<std::string> successful_speeches_{};
};

}  // namespace
}  // namespace voice_nav_audio

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto speech = std::make_shared<voice_nav_audio::SpeechInputNode>(
    std::make_unique<voice_nav_audio::ScriptedRecognizer>());
  if (!voice_nav_audio::wait_for_graph(*speech)) {
    RCLCPP_ERROR(speech->get_logger(), "scripted motion smoke graph did not converge");
    speech.reset();
    rclcpp::shutdown();
    return 1;
  }

  voice_nav_audio::CompletionObserver observer;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer.node());
  std::thread spin_thread([&executor]() {executor.spin();});
  if (!observer.wait_for_runtime_ready()) {
    RCLCPP_ERROR(speech->get_logger(), "Mission Runtime did not become available");
    executor.cancel();
    spin_thread.join();
    speech.reset();
    rclcpp::shutdown();
    return 1;
  }

  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    speech->accept_cleaned_frame(voice_nav_audio::cleaned_frame(sequence));
  }
  if (!observer.wait_for_first_completion()) {
    RCLCPP_ERROR(speech->get_logger(), "first Voice Mission did not complete with Speak");
    executor.cancel();
    spin_thread.join();
    speech.reset();
    rclcpp::shutdown();
    return 1;
  }
  if (!observer.wait_for_first_safe_gate_epoch()) {
    RCLCPP_ERROR(speech->get_logger(), "first Voice Mission did not return Gate to safe idle");
    executor.cancel();
    spin_thread.join();
    speech.reset();
    rclcpp::shutdown();
    return 1;
  }
  for (std::uint64_t sequence = 4U; sequence <= 6U; ++sequence) {
    speech->accept_cleaned_frame(voice_nav_audio::cleaned_frame(sequence));
  }
  if (!observer.wait_for_second_completion()) {
    RCLCPP_ERROR(speech->get_logger(), "second Voice Mission did not complete with Speak");
    executor.cancel();
    spin_thread.join();
    speech.reset();
    rclcpp::shutdown();
    return 1;
  }
  executor.cancel();
  spin_thread.join();
  speech.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
