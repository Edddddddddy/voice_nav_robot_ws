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

// Non-installed real-model gate.  It feeds the locked WAV through the actual
// sherpa adapters and the existing SpeechInputNode; it is never a microphone
// or a production ROS executable.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include <sys/resource.h>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensevoice_sherpa_adapter.hpp"
#include "sherpa-onnx/c-api/c-api.h"
#include "speech_input_node.hpp"
#include "voice_nav_interfaces/msg/voice_turn.hpp"

namespace voice_nav_audio
{
namespace
{

using Clock = std::chrono::steady_clock;
using VoiceTurn = voice_nav_interfaces::msg::VoiceTurn;

constexpr std::size_t kExpectedWaveBytes = 178988U;
constexpr std::size_t kExpectedSenseVoiceModelBytes = 239233841U;
constexpr std::size_t kExpectedTokensBytes = 315894U;
constexpr std::size_t kExpectedVadBytes = 212860U;
constexpr std::size_t kExpectedWaveSamples = 89472U;
constexpr std::size_t kMaxObservedTurns = 1U;
constexpr char kExpectedText[] = "开放时间早上9点至下午5点。";
constexpr char kExpectedWaveSha256[] =
  "b77f1794fe374a0ba1ee1dc458bfaf9349496cbbfc32780c50ba3c5a7ad8e373";
constexpr char kExpectedSenseVoiceModelSha256[] =
  "c71f0ce00bec95b07744e116345e33d8cbbe08cef896382cf907bf4b51a2cd51";
constexpr char kExpectedTokensSha256[] =
  "f449eb28dc567533d7fa59be34e2abca8784f771850c78a47fb731a31429a1dc";
constexpr char kExpectedVadSha256[] =
  "c36d490aff5ab924ca6c7aeec4d8f6bd3d22db6fa17611b9c5b17eae58ac3a20";

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

struct ObservedTurn
{
  std::uint8_t kind{0U};
  std::string text{};
  std::uint64_t voice_seq{0U};
};

class TurnObserver final : public rclcpp::Node
{
public:
  TurnObserver()
  : Node("sensevoice_real_model_gate_observer")
  {
    subscription_ = create_subscription<VoiceTurn>(
      "/voice/turn", voice_turn_qos(),
      [this](const VoiceTurn::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        turns_.push_back(ObservedTurn{message->kind, message->text, message->voice_seq});
        condition_.notify_all();
      });
  }

  bool wait_for_turn(const std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return !turns_.empty();});
  }

  std::vector<ObservedTurn> turns() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return turns_;
  }

private:
  rclcpp::Subscription<VoiceTurn>::SharedPtr subscription_{};
  mutable std::mutex mutex_{};
  std::condition_variable condition_{};
  std::vector<ObservedTurn> turns_{};
};

std::string environment(const char * const name)
{
  const auto * const value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    throw std::runtime_error(std::string("missing required gate environment: ") + name);
  }
  return value;
}

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

std::uint64_t peak_rss_kb() noexcept
{
  struct rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0 || usage.ru_maxrss <= 0) {
    return 0U;
  }
  // Linux reports ru_maxrss in KiB; this gate runs in the locked WSL build.
  return static_cast<std::uint64_t>(usage.ru_maxrss);
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
  const auto parent = std::filesystem::path(report_path).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  const std::filesystem::path final_path(report_path);
  const std::filesystem::path temp_path =
    final_path.string() + ".tmp." + std::to_string(static_cast<long long>(::getpid()));
  std::error_code filesystem_error;
  std::filesystem::remove(final_path, filesystem_error);
  if (filesystem_error) {
    throw std::runtime_error("cannot clear real-model gate report");
  }
  std::filesystem::remove(temp_path, filesystem_error);
  if (filesystem_error) {
    throw std::runtime_error("cannot clear real-model gate temporary report");
  }

  try {
    std::ofstream report(temp_path, std::ios::binary | std::ios::trunc);
    if (!report) {
      throw std::runtime_error("cannot write real-model gate temporary report");
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
           })
           << "},\n"
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
           << "  \"elapsed_ms\": " << elapsed_ms << ",\n"
           << "  \"peak_rss_kb\": " << peak_rss_kb() << "\n"
           << "}\n";
    report.flush();
    if (!report) {
      throw std::runtime_error("cannot flush real-model gate temporary report");
    }
    report.close();
    if (!report) {
      throw std::runtime_error("cannot close real-model gate temporary report");
    }
  } catch (...) {
    std::error_code cleanup_error;
    std::filesystem::remove(temp_path, cleanup_error);
    std::filesystem::remove(final_path, cleanup_error);
    throw;
  }

  std::filesystem::rename(temp_path, final_path, filesystem_error);
  if (filesystem_error) {
    std::error_code cleanup_error;
    std::filesystem::remove(temp_path, cleanup_error);
    std::filesystem::remove(final_path, cleanup_error);
    throw std::runtime_error("cannot publish real-model gate report atomically");
  }
}

bool has_agent_voice_subscription(rclcpp::Node & node)
{
  const auto endpoints = node.get_subscriptions_info_by_topic("/voice/turn");
  return std::any_of(endpoints.cbegin(), endpoints.cend(), [](const auto & endpoint) {
             return endpoint.node_name() == "agent_node" && endpoint.node_namespace() == "/";
    });
}

bool wait_for_agent(rclcpp::Node & node)
{
  const auto graph_event = node.get_graph_event();
  const auto deadline = Clock::now() + std::chrono::seconds(20);
  while (!has_agent_voice_subscription(node) && Clock::now() < deadline) {
    node.wait_for_graph_change(
      graph_event,
      std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - Clock::now()));
    graph_event->check_and_clear();
  }
  return has_agent_voice_subscription(node);
}

Sample sample_from_float(const float value) noexcept
{
  const auto scaled = std::llround(static_cast<double>(value) * 32768.0);
  const auto clamped = std::clamp(
    scaled,
    static_cast<long long>(std::numeric_limits<Sample>::min()),
    static_cast<long long>(std::numeric_limits<Sample>::max()));
  return static_cast<Sample>(clamped);
}

void feed_frame(
  SpeechInputNode & speech,
  const SherpaOnnxWave & wave,
  const std::uint64_t sequence,
  const std::size_t offset)
{
  CleanedAudioFrame frame{};
  frame.audio_generation = 1U;
  frame.audio_seq = sequence;
  const auto remaining = static_cast<std::size_t>(wave.num_samples) - offset;
  frame.valid_samples = std::min(
    CleanedAudioFrame::kSamples, remaining);
  for (std::size_t index = 0U; index < frame.samples.size(); ++index) {
    const auto sample_index = offset + index;
    frame.samples[index] = sample_index < static_cast<std::size_t>(wave.num_samples) ?
      sample_from_float(wave.samples[sample_index]) : Sample{0};
  }
  speech.accept_cleaned_frame(frame);
}

}  // namespace
}  // namespace voice_nav_audio

int main(int argc, char ** argv)
{
  const auto report_path_value = std::getenv("VOICE_NAV_REAL_GATE_REPORT");
  const std::string report_path = report_path_value == nullptr ?
    "voice_nav_real_model_gate.json" : report_path_value;
  const auto started = voice_nav_audio::Clock::now();
  std::string head;
  std::string wav_path;
  std::string vad_path;
  std::string model_path;
  std::string tokens_path;
  std::vector<voice_nav_audio::ObservedTurn> turns;
  std::string status = "failed";
  std::string detail = "gate did not complete";
  std::shared_ptr<voice_nav_audio::SpeechInputNode> speech;
  std::shared_ptr<voice_nav_audio::TurnObserver> observer;

  try {
    head = voice_nav_audio::environment("VOICE_NAV_REAL_GATE_HEAD");
    wav_path = voice_nav_audio::environment("VOICE_NAV_SENSEVOICE_WAV");
    vad_path = voice_nav_audio::environment("VOICE_NAV_SENSEVOICE_VAD_MODEL");
    model_path = voice_nav_audio::environment("VOICE_NAV_SENSEVOICE_MODEL");
    tokens_path = voice_nav_audio::environment("VOICE_NAV_SENSEVOICE_TOKENS");

    if (std::filesystem::file_size(wav_path) != voice_nav_audio::kExpectedWaveBytes) {
      throw std::runtime_error("locked zh.wav size mismatch");
    }
    voice_nav_audio::WavePtr wave{SherpaOnnxReadWave(wav_path.c_str())};
    if (!wave || wave->sample_rate != 16000 || wave->num_samples !=
      static_cast<int32_t>(voice_nav_audio::kExpectedWaveSamples))
    {
      throw std::runtime_error("locked zh.wav format mismatch");
    }

    rclcpp::init(argc, argv);
    observer = std::make_shared<voice_nav_audio::TurnObserver>();
    auto provider = voice_nav_audio::make_sherpa_sensevoice_provider(
      voice_nav_audio::SherpaSenseVoiceAssetPaths{vad_path, model_path, tokens_path});
    if (!provider->arm_once()) {
      throw std::runtime_error("actual SenseVoice provider could not arm once");
    }
    speech = std::make_shared<voice_nav_audio::SpeechInputNode>(std::move(provider));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(speech);
    executor.add_node(observer);
    std::thread spin_thread([&executor]() {executor.spin();});

    if (!voice_nav_audio::wait_for_agent(*observer)) {
      executor.cancel();
      spin_thread.join();
      throw std::runtime_error("agent_node did not subscribe to /voice/turn");
    }

    std::uint64_t sequence = 1U;
    for (std::size_t offset = 0U; offset < voice_nav_audio::kExpectedWaveSamples;
      offset += voice_nav_audio::CleanedAudioFrame::kSamples)
    {
      voice_nav_audio::feed_frame(*speech, *wave, sequence++, offset);
    }
    speech->finish_input();

    const bool got_turn = observer->wait_for_turn(std::chrono::seconds(30));
    turns = observer->turns();
    executor.cancel();
    spin_thread.join();
    if (!got_turn || turns.size() != voice_nav_audio::kMaxObservedTurns ||
      turns.front().kind != voice_nav_audio::VoiceTurn::COMMAND ||
      turns.front().voice_seq != 1U ||
      turns.front().text != voice_nav_audio::kExpectedText)
    {
      throw std::runtime_error("actual provider did not produce the exact one VoiceTurn");
    }
    if (voice_nav_audio::peak_rss_kb() == 0U) {
      throw std::runtime_error("RUSAGE_SELF returned no peak RSS evidence");
    }
    status = "passed";
    detail = "actual SenseVoiceProvider produced one exact COMMAND VoiceTurn";
    speech.reset();
    observer.reset();
    rclcpp::shutdown();
  } catch (const std::exception & error) {
    detail = error.what();
    if (speech) {
      speech.reset();
    }
    observer.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    voice_nav_audio::Clock::now() - started).count();
  try {
    voice_nav_audio::write_report(
      report_path, head, wav_path, vad_path, model_path, tokens_path, turns, status, detail,
      static_cast<std::uint64_t>(elapsed));
  } catch (const std::exception & error) {
    detail = error.what();
    status = "failed";
  }
  return status == "passed" ? 0 : 1;
}
