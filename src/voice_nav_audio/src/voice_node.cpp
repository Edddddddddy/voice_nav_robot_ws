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

// Installed continuous VoiceNav composition root. The provider, VAD, DSP,
// full-duplex device and TTS pipeline all live for the process lifetime;
// terminal turns reset state inside the provider worker.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "chaowen_tts_adapter.hpp"
#include "continuous_vad_session.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensevoice_sherpa_adapter.hpp"
#include "vad_auto_dsp_adapter.hpp"
#include "voice_node_executor.hpp"

namespace voice_nav_audio
{
namespace
{

using Clock = std::chrono::steady_clock;
std::string parameter_or_environment(
  rclcpp::Node & node, const char * const parameter, const char * const environment)
{
  std::string value{};
  node.get_parameter(parameter, value);
  if (!value.empty()) {
    return value;
  }
  const auto * const from_environment = std::getenv(environment);
  return from_environment == nullptr ? std::string{} : std::string(from_environment);
}

class VoiceNode final : public rclcpp::Node, public SpeechOutputTraceSink
{
public:
  VoiceNode()
  : Node("voice_node")
  {
    declare_parameter("input_profile", "vad_auto");
    declare_parameter("chaowen_tts_root", "");
    declare_parameter("silero_vad_model", "");
    declare_parameter("sensevoice_model", "");
    declare_parameter("sensevoice_tokens", "");
    declare_parameter("kws_root", "");
  }

  [[nodiscard]] bool wait_for_agent(const std::chrono::seconds timeout)
  {
    const auto graph_event = get_graph_event();
    const auto deadline = Clock::now() + timeout;
    while (!has_agent_subscription() && Clock::now() < deadline) {
      wait_for_graph_change(
        graph_event,
        std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - Clock::now()));
      graph_event->check_and_clear();
    }
    return has_agent_subscription();
  }

  void on_played(const std::uint64_t, const std::uint64_t) noexcept override {}

  void on_result(const SpeechResult & result) noexcept override
  {
    RCLCPP_INFO(
      get_logger(), "speak_result code=%u played_samples=%llu detail=%s",
      static_cast<unsigned int>(result.code),
      static_cast<unsigned long long>(result.played_samples), result.detail.c_str());
  }

private:
  [[nodiscard]] bool has_agent_subscription() const
  {
    const auto endpoints = get_subscriptions_info_by_topic("/voice/turn");
    return std::any_of(endpoints.cbegin(), endpoints.cend(), [](const auto & endpoint) {
      return endpoint.node_name() == "agent_node" && endpoint.node_namespace() == "/";
    });
  }

};

}  // namespace
}  // namespace voice_nav_audio

int main(int argc, char ** argv)
{
  // Construct the executor after rclcpp::init(), but keep its owner in the
  // outer scope before the guard so exception teardown destroys the guard
  // (and joins its thread) while the executor is still alive.
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::shared_ptr<voice_nav_audio::VoiceNode> node;
  std::unique_ptr<voice_nav_audio::ContinuousVadSession> session;
  std::unique_ptr<voice_nav_audio::DspAdapter> dsp;
  std::unique_ptr<voice_nav_audio::VoiceNodeExecutorGuard> executor_guard;

  try {
    rclcpp::init(argc, argv);
    node = std::make_shared<voice_nav_audio::VoiceNode>();

    std::string input_profile{};
    node->get_parameter("input_profile", input_profile);
    if (input_profile != "vad_auto") {
      throw std::invalid_argument("voice_node only supports the continuous vad_auto profile");
    }

    const auto vad_path = voice_nav_audio::parameter_or_environment(
      *node, "silero_vad_model", "VOICE_NAV_SENSEVOICE_VAD_MODEL");
    const auto model_path = voice_nav_audio::parameter_or_environment(
      *node, "sensevoice_model", "VOICE_NAV_SENSEVOICE_MODEL");
    const auto tokens_path = voice_nav_audio::parameter_or_environment(
      *node, "sensevoice_tokens", "VOICE_NAV_SENSEVOICE_TOKENS");
    const auto kws_root_value = voice_nav_audio::parameter_or_environment(
      *node, "kws_root", "VOICE_NAV_KWS_ROOT");
    const auto tts_root = voice_nav_audio::parameter_or_environment(
      *node, "chaowen_tts_root", "VOICE_NAV_CHAOWEN_TTS_ROOT");
    if (vad_path.empty() || model_path.empty() || tokens_path.empty() || kws_root_value.empty() ||
      tts_root.empty())
    {
      throw std::invalid_argument("continuous voice assets are incomplete");
    }

    const std::filesystem::path kws_root{kws_root_value};

    auto provider = voice_nav_audio::make_sherpa_sensevoice_provider(
      voice_nav_audio::SherpaSenseVoiceAssetPaths{
      vad_path,
      model_path,
      tokens_path,
      (kws_root / "encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx").string(),
      (kws_root / "decoder-epoch-13-avg-2-chunk-16-left-64.onnx").string(),
      (kws_root / "joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx").string(),
      (kws_root / "tokens.txt").string(),
      (kws_root / "keywords.txt").string()});
    dsp = voice_nav_audio::make_vad_auto_dsp_adapter();
    if (provider == nullptr || dsp == nullptr) {
      throw std::runtime_error("continuous voice requires verified VAD/DSP adapters");
    }
    session = std::make_unique<voice_nav_audio::ContinuousVadSession>(
      std::move(provider), *dsp,
      std::make_unique<voice_nav_audio::ChaowenTtsAdapter>(tts_root), nullptr, node.get());

    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    session->add_to_executor(*executor);
    executor_guard = std::make_unique<voice_nav_audio::VoiceNodeExecutorGuard>(*executor);
    if (!node->wait_for_agent(std::chrono::seconds(20))) {
      throw std::runtime_error("agent_node did not subscribe to /voice/turn");
    }

    while (rclcpp::ok()) {
      if (session->pump() == voice_nav_audio::ContinuousVadPumpResult::kFailed) {
        throw std::runtime_error("continuous VAD capture pump failed");
      }
      std::this_thread::yield();
    }

    executor_guard->stop();
    session->stop();
    session.reset();
    node.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 0;
  } catch (const std::exception & error) {
    if (executor_guard != nullptr) {
      executor_guard->stop();
    }
    if (session != nullptr) {
      session->stop();
    }
    session.reset();
    dsp.reset();
    node.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << "voice_node: " << error.what() << '\n';
    return 1;
  }
}
