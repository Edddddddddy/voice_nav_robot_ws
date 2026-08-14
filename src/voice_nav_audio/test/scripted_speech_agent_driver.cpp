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

// Non-installed test driver for the Voice -> installed Agent tracer bullet.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "speech_input_node.hpp"

namespace voice_nav_audio
{
namespace
{

using namespace std::chrono_literals;

class ScriptedRecognizer final : public SpeechRecognizerAdapter
{
public:
  explicit ScriptedRecognizer(std::string final_text)
  : final_text_(std::move(final_text))
  {
  }

  void process_frame(
    const CleanedAudioFrame & frame,
    SpeechEventSink & sink) noexcept override
  {
    if (frame.audio_seq == 1U) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    } else if (frame.audio_seq == 2U) {
      sink.on_speech_event(SpeechRecognitionEvent::activity(frame, active_scope_));
    } else if (frame.audio_seq == 3U) {
      sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
        frame, active_scope_, final_text_, 1.0F));
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
  std::string final_text_{};
};

CleanedAudioFrame cleaned_frame(const std::uint64_t sequence)
{
  CleanedAudioFrame frame{};
  frame.audio_generation = 1U;
  frame.audio_seq = sequence;
  frame.samples.fill(0);
  return frame;
}

bool has_agent_endpoint(
  const std::vector<rclcpp::TopicEndpointInfo> & endpoints,
  const std::string & node_name)
{
  return std::any_of(
    endpoints.cbegin(), endpoints.cend(), [&node_name](const auto & endpoint) {
      return endpoint.node_name() == node_name && endpoint.node_namespace() == "/";
    });
}

bool downstream_is_ready(rclcpp::Node & node)
{
  const auto voice_subscriptions = node.get_subscriptions_info_by_topic("/voice/turn");
  const auto mission_status_subscriptions = node.get_subscriptions_info_by_topic(
    "/mission/execute/_action/status");
  const auto mission_status_publishers = node.get_publishers_info_by_topic(
    "/mission/execute/_action/status");
  return has_agent_endpoint(voice_subscriptions, "agent_node") &&
         has_agent_endpoint(mission_status_subscriptions, "agent_node") &&
         has_agent_endpoint(mission_status_publishers, "scripted_voice_agent_probe");
}

bool wait_for_downstream(rclcpp::Node & node)
{
  const auto graph_event = node.get_graph_event();
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (!downstream_is_ready(node) && std::chrono::steady_clock::now() < deadline) {
    node.wait_for_graph_change(
      graph_event,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline - std::chrono::steady_clock::now()));
    graph_event->check_and_clear();
  }
  return downstream_is_ready(node);
}

}  // namespace
}  // namespace voice_nav_audio

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto scenario = std::getenv("VOICE_NAV_SCRIPTED_SCENARIO");
  const std::string final_text = scenario != nullptr && std::string(scenario) == "llm" ?
    "绕个弯" : "前进半米";
  auto speech = std::make_shared<voice_nav_audio::SpeechInputNode>(
    std::make_unique<voice_nav_audio::ScriptedRecognizer>(final_text));
  if (!voice_nav_audio::wait_for_downstream(*speech)) {
    RCLCPP_ERROR(speech->get_logger(), "scripted smoke prerequisites did not converge");
    speech.reset();
    rclcpp::shutdown();
    return 1;
  }

  speech->accept_cleaned_frame(voice_nav_audio::cleaned_frame(1U));
  speech->accept_cleaned_frame(voice_nav_audio::cleaned_frame(2U));
  speech->accept_cleaned_frame(voice_nav_audio::cleaned_frame(3U));
  rclcpp::spin(speech);
  speech.reset();
  rclcpp::shutdown();
  return 0;
}
