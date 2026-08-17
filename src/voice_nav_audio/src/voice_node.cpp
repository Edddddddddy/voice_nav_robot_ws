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

// Installed, headless VoiceNav composition root for bounded SenseVoice WAV and
// microphone-once input. The explicit real_model_gate profile retains the
// locked fixture evidence; this process owns only the real provider and the
// selected package-private composition seam.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "chaowen_tts_adapter.hpp"
#include "file_audio_device.hpp"
#include "microphone_once_runner.hpp"
#include "sensevoice_sherpa_adapter.hpp"
#include "sherpa-onnx/c-api/c-api.h"
#include "speech_input_node.hpp"
#include "voice_input_policy.hpp"
#include "voice_pipeline.hpp"
#include "voice_nav_interfaces/msg/voice_turn.hpp"

namespace voice_nav_audio
{
namespace
{

using Clock = std::chrono::steady_clock;
using VoiceTurn = voice_nav_interfaces::msg::VoiceTurn;

enum class InputProfile
{
  kSenseVoiceWav,
  kMicrophoneOnce,
  kRealModelGate,
};

constexpr std::size_t kExpectedWaveBytes = 178988U;
constexpr std::size_t kExpectedWaveSamples = 89472U;
constexpr char kExpectedText[] = "开放时间早上9点至下午5点。";
constexpr char kExpectedWaveSha256[] =
  "b77f1794fe374a0ba1ee1dc458bfaf9349496cbbfc32780c50ba3c5a7ad8e373";

#define VOICE_NAV_SENSEVOICE_ASSET(name, bytes, sha256) \
  constexpr std::size_t kExpected##name##Bytes = bytes; \
  constexpr char kExpected##name##Sha256[] = sha256;
#include "sensevoice_runtime_asset_manifest.def"
#undef VOICE_NAV_SENSEVOICE_ASSET

struct WaveDeleter
{
  void operator()(const SherpaOnnxWave * wave) const noexcept
  {
    if (wave != nullptr) {
      SherpaOnnxFreeWave(wave);
    }
  }
};

using WavePtr = std::unique_ptr<const SherpaOnnxWave, WaveDeleter>;

InputProfile parse_input_profile(const std::string & value)
{
  if (value == "sensevoice_wav") {
    return InputProfile::kSenseVoiceWav;
  }
  if (value == "microphone_once") {
    return InputProfile::kMicrophoneOnce;
  }
  if (value == "real_model_gate") {
    return InputProfile::kRealModelGate;
  }
  throw std::invalid_argument("unsupported voice_node input profile");
}

WavePtr read_input_wave(const std::string & path)
{
  const auto validation = validate_input_wav(path);
  if (!validation.accepted) {
    throw std::invalid_argument(validation.reason);
  }
  WavePtr wave{SherpaOnnxReadWave(path.c_str())};
  const auto maximum_samples =
    SenseVoiceProviderConfig::kDefaultMaximumUtteranceFrames * CleanedAudioFrame::kSamples;
  if (!wave || wave->sample_rate != static_cast<int32_t>(CleanedAudioFrame::kSampleRateHz) ||
    wave->num_samples <= 0 || static_cast<std::size_t>(wave->num_samples) > maximum_samples)
  {
    throw std::invalid_argument("input_wav_unsupported_format");
  }
  return wave;
}

struct ObservedTurn
{
  std::uint8_t kind{0U};
  std::string text{};
  std::uint64_t voice_seq{0U};
};

std::string json_escape(const std::string & value)
{
  std::ostringstream escaped;
  escaped << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (character < 0x20U) {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<unsigned int>(character) << std::dec;
        } else {
          escaped << static_cast<char>(character);
        }
        break;
    }
  }
  escaped << '"';
  return escaped.str();
}

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
    declare_parameter("input_profile", "sensevoice_wav");
    declare_parameter("input_wav", "");
    declare_parameter("output_wav", "");
    declare_parameter("chaowen_tts_root", "");
    declare_parameter("silero_vad_model", "");
    declare_parameter("sensevoice_model", "");
    declare_parameter("sensevoice_tokens", "");
    declare_parameter("result_path", "");
    declare_parameter("exact_head", "unknown");
    turn_subscription_ = create_subscription<VoiceTurn>(
      "/voice/turn", voice_turn_qos(),
      [this](const VoiceTurn::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        turns_.push_back(ObservedTurn{message->kind, message->text, message->voice_seq});
        condition_.notify_all();
      });
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

  [[nodiscard]] bool wait_for_turn(const std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return !turns_.empty();});
  }

  [[nodiscard]] std::vector<ObservedTurn> turns() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return turns_;
  }

  [[nodiscard]] bool wait_for_speak(const std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return !speak_results_.empty();});
  }

  [[nodiscard]] std::vector<SpeechResult> speak_results() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return speak_results_;
  }

  void on_played(const std::uint64_t, const std::uint64_t) noexcept override
  {
  }

  void on_result(const SpeechResult & result) noexcept override
  {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      speak_results_.push_back(result);
      condition_.notify_all();
    } catch (...) {
    }
  }

private:
  [[nodiscard]] bool has_agent_subscription() const
  {
    const auto endpoints = get_subscriptions_info_by_topic("/voice/turn");
    return std::any_of(endpoints.cbegin(), endpoints.cend(), [](const auto & endpoint) {
               return endpoint.node_name() == "agent_node" && endpoint.node_namespace() == "/";
    });
  }

  rclcpp::Subscription<VoiceTurn>::SharedPtr turn_subscription_{};
  mutable std::mutex mutex_{};
  std::condition_variable condition_{};
  std::vector<ObservedTurn> turns_{};
  std::vector<SpeechResult> speak_results_{};
};

Sample sample_from_float(const float value) noexcept
{
  const auto scaled = std::llround(static_cast<double>(value) * 32768.0);
  const auto clamped = std::clamp(
    scaled,
    static_cast<long long>(std::numeric_limits<Sample>::min()),
    static_cast<long long>(std::numeric_limits<Sample>::max()));
  return static_cast<Sample>(clamped);
}

CleanedAudioFrame make_frame(
  const SherpaOnnxWave & wave,
  const std::uint64_t sequence,
  const std::size_t offset)
{
  CleanedAudioFrame frame{};
  frame.audio_generation = 1U;
  frame.audio_seq = sequence;
  const auto remaining = static_cast<std::size_t>(wave.num_samples) - offset;
  frame.valid_samples = std::min(CleanedAudioFrame::kSamples, remaining);
  for (std::size_t index = 0U; index < frame.samples.size(); ++index) {
    const auto sample_index = offset + index;
    frame.samples[index] = sample_index < static_cast<std::size_t>(wave.num_samples) ?
      sample_from_float(wave.samples[sample_index]) : Sample{0};
  }
  return frame;
}

void feed_frame(
  SpeechInputNode & speech,
  const SherpaOnnxWave & wave,
  const std::uint64_t sequence,
  const std::size_t offset)
{
  speech.accept_cleaned_frame(make_frame(wave, sequence, offset));
}

void feed_frame(
  VoicePipeline & pipeline,
  const SherpaOnnxWave & wave,
  const std::uint64_t sequence,
  const std::size_t offset)
{
  pipeline.accept_cleaned_frame(make_frame(wave, sequence, offset));
}

void write_report(
  const std::string & report_path,
  const std::string & head,
  const std::string & wav_path,
  const std::string & vad_path,
  const std::string & model_path,
  const std::string & tokens_path,
  const std::vector<ObservedTurn> & turns,
  const std::string & status,
  const std::string & detail,
  const std::uint64_t elapsed_ms)
{
  const std::filesystem::path final_path(report_path);
  const auto parent = final_path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  std::error_code exists_error;
  if (std::filesystem::exists(final_path, exists_error) || exists_error) {
    throw std::runtime_error("refusing to overwrite installed voice_node report");
  }
  const auto temp_path = final_path.string() + ".tmp." +
    std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
  std::ofstream report(temp_path, std::ios::binary | std::ios::trunc);
  if (!report) {
    throw std::runtime_error("cannot write installed voice_node report");
  }
  report << "{\n"
         << "  \"schema_version\": \"voice_nav.real_model_gate.v1\",\n"
         << "  \"status\": " << json_escape(status) << ",\n"
         << "  \"detail\": " << json_escape(detail) << ",\n"
         << "  \"exact_head\": " << json_escape(head) << ",\n"
         << "  \"assets\": {\n"
         << "    \"wav\": {\"path\": " << json_escape(wav_path)
         << ", \"expected_size\": " << kExpectedWaveBytes
         << ", \"expected_sha256\": " << json_escape(kExpectedWaveSha256) << "},\n"
         << "    \"sensevoice_model\": {\"path\": " << json_escape(model_path)
         << ", \"expected_size\": " << kExpectedSenseVoiceModelBytes
         << ", \"expected_sha256\": " << json_escape(kExpectedSenseVoiceModelSha256) << "},\n"
         << "    \"tokens\": {\"path\": " << json_escape(tokens_path)
         << ", \"expected_size\": " << kExpectedTokensBytes
         << ", \"expected_sha256\": " << json_escape(kExpectedTokensSha256) << "},\n"
         << "    \"silero_vad\": {\"path\": " << json_escape(vad_path)
         << ", \"expected_size\": " << kExpectedVadBytes
         << ", \"expected_sha256\": " << json_escape(kExpectedVadSha256) << "}\n"
         << "  },\n"
         << "  \"audio\": {\"sample_rate_hz\": 16000, \"channels\": 1,"
         << " \"samples\": " << kExpectedWaveSamples << "},\n"
         << "  \"provider\": {\"voice_turn_count\": " << turns.size()
         << ", \"command_count\": "
         << std::count_if(turns.cbegin(), turns.cend(), [](const auto & turn) {
      return turn.kind == VoiceTurn::COMMAND;
    }) << "},\n"
         << "  \"turns\": [";
  for (std::size_t index = 0U; index < turns.size(); ++index) {
    if (index != 0U) {
      report << ',';
    }
    report << "{\"kind\": " << static_cast<unsigned int>(turns[index].kind)
           << ", \"voice_seq\": " << turns[index].voice_seq
           << ", \"text\": " << json_escape(turns[index].text) << '}';
  }
  report << "],\n"
         << "  \"elapsed_ms\": " << elapsed_ms << "\n"
         << "}\n";
  report.flush();
  if (!report) {
    std::filesystem::remove(temp_path);
    throw std::runtime_error("cannot flush installed voice_node report");
  }
  report.close();
  if (::link(temp_path.c_str(), final_path.string().c_str()) != 0) {
    std::filesystem::remove(temp_path);
    throw std::runtime_error("cannot publish installed voice_node report without replacement");
  }
  if (::unlink(temp_path.c_str()) != 0) {
    throw std::runtime_error("cannot remove installed voice_node temporary report");
  }
}

}  // namespace
}  // namespace voice_nav_audio

int main(int argc, char ** argv)
{
  std::string report_path{};
  std::string wav_path{};
  std::string output_wav_path{};
  std::string chaowen_tts_root{};
  std::string vad_path{};
  std::string model_path{};
  std::string tokens_path{};
  std::string exact_head = "unknown";
  std::vector<voice_nav_audio::ObservedTurn> turns;
  std::string status = "failed";
  std::string detail = "installed voice_node did not complete";
  const auto started_at = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&started_at]() {
      return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started_at).count());
    };
  std::shared_ptr<voice_nav_audio::VoiceNode> node;
  std::shared_ptr<voice_nav_audio::SpeechInputNode> speech;
  std::unique_ptr<voice_nav_audio::VoicePipeline> pipeline;
  std::unique_ptr<voice_nav_audio::MicrophoneOnceRunner> microphone_once;
  std::unique_ptr<voice_nav_audio::MicrophoneOnceDspAdapter> microphone_dsp;
  std::unique_ptr<voice_nav_audio::FileAudioDevice> file_device;
  voice_nav_audio::WavePtr wave{nullptr};
  bool microphone_profile = false;
  bool real_model_gate = false;

  try {
    rclcpp::init(argc, argv);
    node = std::make_shared<voice_nav_audio::VoiceNode>();
    std::string profile{};
    node->get_parameter("input_profile", profile);
    const auto input_profile = voice_nav_audio::parse_input_profile(profile);
    microphone_profile = input_profile == voice_nav_audio::InputProfile::kMicrophoneOnce;
    real_model_gate = input_profile == voice_nav_audio::InputProfile::kRealModelGate;
    if (!microphone_profile) {
      output_wav_path = voice_nav_audio::parameter_or_environment(
        *node, "output_wav", "VOICE_NAV_OUTPUT_WAV");
      chaowen_tts_root = voice_nav_audio::parameter_or_environment(
        *node, "chaowen_tts_root", "VOICE_NAV_CHAOWEN_TTS_ROOT");
    } else {
      const auto * const environment_tts_root = std::getenv("VOICE_NAV_CHAOWEN_TTS_ROOT");
      chaowen_tts_root = environment_tts_root == nullptr ?
        std::string{} : std::string(environment_tts_root);
    }
    if (real_model_gate && !output_wav_path.empty()) {
      throw std::invalid_argument("output_wav requires the sensevoice_wav input profile");
    }
    if (real_model_gate) {
      node->get_parameter("result_path", report_path);
      if (report_path.empty()) {
        const auto environment_report = std::getenv("VOICE_NAV_REAL_GATE_REPORT");
        if (environment_report != nullptr && environment_report[0] != '\0') {
          report_path = environment_report;
        }
      }
      node->get_parameter("exact_head", exact_head);
      if (exact_head.empty() || exact_head == "unknown") {
        const auto environment_head = std::getenv("VOICE_NAV_REAL_GATE_HEAD");
        if (environment_head != nullptr && environment_head[0] != '\0') {
          exact_head = environment_head;
        }
      }
      if (report_path.empty() || exact_head.empty() || exact_head == "unknown") {
        throw std::invalid_argument("real_model_gate evidence parameters are incomplete");
      }
    }
    if (!microphone_profile) {
      wav_path = voice_nav_audio::parameter_or_environment(
        *node, "input_wav", "VOICE_NAV_SENSEVOICE_WAV");
      vad_path = voice_nav_audio::parameter_or_environment(
        *node, "silero_vad_model", "VOICE_NAV_SENSEVOICE_VAD_MODEL");
      model_path = voice_nav_audio::parameter_or_environment(
        *node, "sensevoice_model", "VOICE_NAV_SENSEVOICE_MODEL");
      tokens_path = voice_nav_audio::parameter_or_environment(
        *node, "sensevoice_tokens", "VOICE_NAV_SENSEVOICE_TOKENS");
      if (wav_path.empty() || vad_path.empty() || model_path.empty() || tokens_path.empty()) {
        throw std::invalid_argument("SenseVoice asset parameters are incomplete");
      }
      wave = voice_nav_audio::read_input_wave(wav_path);
      if (real_model_gate) {
        if (std::filesystem::file_size(wav_path) != voice_nav_audio::kExpectedWaveBytes) {
          throw std::invalid_argument("locked zh.wav size mismatch");
        }
        if (wave->num_samples != static_cast<int32_t>(voice_nav_audio::kExpectedWaveSamples)) {
          throw std::invalid_argument("locked zh.wav format mismatch");
        }
      }
    }

    if (!output_wav_path.empty()) {
      const std::filesystem::path output_path(output_wav_path);
      if (!output_path.is_absolute() || std::filesystem::is_symlink(output_path) ||
        std::filesystem::exists(output_path))
      {
        throw std::invalid_argument("output_wav must be an absolute new path");
      }
      const auto parent = output_path.parent_path();
      if (parent.empty() || !std::filesystem::is_directory(parent) ||
        ::access(parent.c_str(), W_OK) != 0)
      {
        throw std::invalid_argument("output_wav parent is not writable");
      }
      const std::filesystem::path tts_root(chaowen_tts_root);
      if (chaowen_tts_root.empty() || !tts_root.is_absolute() ||
        std::filesystem::is_symlink(tts_root) || !std::filesystem::is_directory(tts_root))
      {
        throw std::invalid_argument("Chaowen TTS root is not a verified directory");
      }
    }

    if (microphone_profile) {
      const auto * const environment_vad = std::getenv("VOICE_NAV_SENSEVOICE_VAD_MODEL");
      const auto * const environment_model = std::getenv("VOICE_NAV_SENSEVOICE_MODEL");
      const auto * const environment_tokens = std::getenv("VOICE_NAV_SENSEVOICE_TOKENS");
      vad_path = environment_vad == nullptr ? std::string{} : std::string(environment_vad);
      model_path = environment_model == nullptr ? std::string{} : std::string(environment_model);
      tokens_path = environment_tokens == nullptr ?
        std::string{} : std::string(environment_tokens);
      if (vad_path.empty() || model_path.empty() || tokens_path.empty() ||
        chaowen_tts_root.empty())
      {
        throw std::invalid_argument("microphone_once assets are incomplete");
      }
      auto provider = voice_nav_audio::make_sherpa_sensevoice_provider(
        voice_nav_audio::SherpaSenseVoiceAssetPaths{vad_path, model_path, tokens_path});
      microphone_dsp = std::make_unique<voice_nav_audio::MicrophoneOnceDspAdapter>();
      microphone_once = std::make_unique<voice_nav_audio::MicrophoneOnceRunner>(
        std::move(provider),
        std::make_unique<voice_nav_audio::ChaowenTtsAdapter>(chaowen_tts_root),
        *microphone_dsp, nullptr, node.get(),
        voice_nav_audio::MicrophoneOnceSpec{});
    } else {
      auto provider = voice_nav_audio::make_sherpa_sensevoice_provider(
        voice_nav_audio::SherpaSenseVoiceAssetPaths{vad_path, model_path, tokens_path});
      if (!provider->arm_once()) {
        throw std::runtime_error("SenseVoice provider could not arm once");
      }
      if (output_wav_path.empty()) {
        speech = std::make_shared<voice_nav_audio::SpeechInputNode>(std::move(provider));
      } else {
        file_device = std::make_unique<voice_nav_audio::FileAudioDevice>(output_wav_path);
        pipeline = std::make_unique<voice_nav_audio::VoicePipeline>(
          std::move(provider),
          std::make_unique<voice_nav_audio::ChaowenTtsAdapter>(chaowen_tts_root),
          *file_device, node.get());
      }
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    if (microphone_once) {
      microphone_once->add_to_executor(executor);
    } else if (pipeline) {
      pipeline->add_to_executor(executor);
    } else {
      executor.add_node(speech);
    }
    std::thread spin_thread([&executor]() {executor.spin();});

    if (!node->wait_for_agent(std::chrono::seconds(20))) {
      executor.cancel();
      spin_thread.join();
      throw std::runtime_error("agent_node did not subscribe to /voice/turn");
    }
    bool got_turn = false;
    bool got_speak = false;
    if (microphone_once) {
      const auto capture_result = microphone_once->capture_until(std::chrono::seconds(20));
      if (capture_result != voice_nav_audio::MicrophoneOnceResult::kReadyForPlayback ||
        !microphone_once->allow_playback())
      {
        executor.cancel();
        spin_thread.join();
        throw std::runtime_error("microphone_once capture did not reach playback");
      }
      got_turn = node->wait_for_turn(std::chrono::seconds(20));
      got_speak = got_turn && node->wait_for_speak(std::chrono::seconds(45));
    } else {
      std::uint64_t sequence = 1U;
      for (std::size_t offset = 0U;
        offset < static_cast<std::size_t>(wave->num_samples);
        offset += voice_nav_audio::CleanedAudioFrame::kSamples)
      {
        if (pipeline) {
          voice_nav_audio::feed_frame(*pipeline, *wave, sequence++, offset);
        } else {
          voice_nav_audio::feed_frame(*speech, *wave, sequence++, offset);
        }
      }
      if (pipeline) {
        pipeline->finish_input();
      } else {
        speech->finish_input();
      }
      got_turn = node->wait_for_turn(std::chrono::seconds(45));
      got_speak = !pipeline || (got_turn &&
        node->wait_for_speak(std::chrono::seconds(45)));
    }
    turns = node->turns();
    executor.cancel();
    spin_thread.join();
    if (!got_turn || turns.size() != 1U ||
      turns.front().kind != voice_nav_interfaces::msg::VoiceTurn::COMMAND ||
      turns.front().voice_seq != 1U)
    {
      throw std::runtime_error("installed voice_node did not produce one VoiceTurn");
    }
    if (pipeline || microphone_once) {
      const auto speak_results = node->speak_results();
      if (!got_speak || speak_results.size() != 1U ||
        speak_results.front().code != voice_nav_audio::SpeechResultCode::Completed ||
        speak_results.front().played_samples == 0U)
      {
        throw std::runtime_error("installed voice_node did not complete one Speak output");
      }
    }
    if (real_model_gate && turns.front().text != voice_nav_audio::kExpectedText) {
      throw std::runtime_error("real-model gate did not produce the exact VoiceTurn");
    }
    status = "passed";
    detail = microphone_profile ?
      "installed microphone_once produced one COMMAND VoiceTurn and completed Speak" :
      "installed real SenseVoiceProvider produced one COMMAND VoiceTurn";
    if (real_model_gate) {
      detail = "installed real SenseVoiceProvider produced one exact COMMAND VoiceTurn";
    }
    if (real_model_gate) {
      voice_nav_audio::write_report(
        report_path, exact_head, wav_path, vad_path, model_path, tokens_path, turns, status, detail,
        elapsed_ms());
    }
    microphone_once.reset();
    microphone_dsp.reset();
    pipeline.reset();
    if (file_device && !file_device->commit()) {
      throw std::runtime_error("file-backed Speak output could not be committed");
    }
    file_device.reset();
    speech.reset();
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    detail = error.what();
    microphone_once.reset();
    microphone_dsp.reset();
    pipeline.reset();
    if (speech) {
      speech.reset();
    }
    file_device.reset();
    node.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  if (real_model_gate) {
    try {
      voice_nav_audio::write_report(
        report_path, exact_head, wav_path, vad_path, model_path, tokens_path, turns, status, detail,
        elapsed_ms());
    } catch (...) {
      // Preserve the original gate failure when evidence publication fails.
    }
  }
  return 1;
}
