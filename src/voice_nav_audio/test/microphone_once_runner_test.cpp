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

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "dsp_pipeline.hpp"
#include "microphone_once_runner.hpp"
#include "sensevoice_provider.hpp"
#include "voice_pipeline.hpp"

namespace voice_nav_audio
{
namespace
{

using namespace std::chrono_literals;

class ContextGuard final
{
public:
  ContextGuard()
  {
    rclcpp::init(0, nullptr);
  }

  ~ContextGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class ManualFullDuplexDevice final : public FullDuplexAudioDevice
{
public:
  bool open(
    const FullDuplexStreamSpec spec, const DeviceCallback callback,
    void * const context) noexcept override
  {
    if (spec.sample_rate != AudioEngine::kSampleRate ||
      spec.channels != AudioEngine::kChannels ||
      spec.frames_per_buffer != AudioEngine::kFrameSamples || callback == nullptr ||
      context == nullptr)
    {
      return false;
    }
    callback_ = callback;
    context_ = context;
    open_count_++;
    return true;
  }

  void close() noexcept override
  {
    if (callback_ != nullptr) {
      close_count_++;
    }
    callback_ = nullptr;
    context_ = nullptr;
  }

  [[nodiscard]] bool emit_capture(const Sample value)
  {
    if (callback_ == nullptr || context_ == nullptr) {
      return false;
    }
    std::array<Sample, AudioEngine::kFrameSamples> capture{};
    std::array<Sample, AudioEngine::kFrameSamples> output{};
    capture.fill(value);
    callback_(context_, capture.data(), output.data(), output.size(), CallbackStatus{});
    last_output_ = output;
    return true;
  }

  [[nodiscard]] bool is_open() const noexcept
  {
    return callback_ != nullptr;
  }

  [[nodiscard]] std::size_t open_count() const noexcept {return open_count_;}
  [[nodiscard]] std::size_t close_count() const noexcept {return close_count_;}
  [[nodiscard]] const auto & last_output() const noexcept {return last_output_;}

private:
  DeviceCallback callback_{nullptr};
  void * context_{nullptr};
  std::size_t open_count_{0U};
  std::size_t close_count_{0U};
  std::array<Sample, AudioEngine::kFrameSamples> last_output_{};
};

class ScriptedVad final : public SileroVadAdapter
{
public:
  [[nodiscard]] SileroVadResult process(const CleanedAudioFrame &) noexcept override
  {
    ++process_count;
    if (process_count == 1U) {
      return SileroVadResult{SileroVadDecision::kSpeech, 0U};
    }
    if (process_count == 2U) {
      return SileroVadResult{SileroVadDecision::kEndpoint, 320U};
    }
    return SileroVadResult{};
  }

  [[nodiscard]] SileroVadFlushResult finish_input() noexcept override
  {
    return SileroVadFlushResult{};
  }

  void reset() noexcept override {}

  std::size_t process_count{0U};
};

class SpeechOnlyVad final : public SileroVadAdapter
{
public:
  [[nodiscard]] SileroVadResult process(const CleanedAudioFrame &) noexcept override
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      ++process_count;
    }
    condition.notify_all();
    return SileroVadResult{SileroVadDecision::kSpeech, 0U};
  }

  [[nodiscard]] SileroVadFlushResult finish_input() noexcept override
  {
    return SileroVadFlushResult{};
  }

  void reset() noexcept override {}

  bool wait_processed()
  {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, 2s, [this]() {return process_count != 0U;});
  }

  std::size_t process_count{0U};

private:
  std::mutex mutex{};
  std::condition_variable condition{};
};

class ScriptedAsr final : public SenseVoiceAsrAdapter
{
public:
  bool infer(
    const Sample * const samples, const std::size_t sample_count,
    std::string & labeled_text) noexcept override
  {
    last_sample_count = sample_count;
    first_sample = samples == nullptr || sample_count == 0U ? 0 : samples[0];
    labeled_text = "前进半米";
    return true;
  }

  std::size_t last_sample_count{0U};
  Sample first_sample{0};
};

class PassthroughDsp final : public DspAdapter
{
public:
  bool process_render(const DspFrame &) noexcept override
  {
    call_order.push_back("render");
    return true;
  }

  bool set_stream_delay_ms(const int milliseconds) noexcept override
  {
    delay_ms = milliseconds;
    call_order.push_back("delay");
    return true;
  }

  bool process_capture(DspFrame &) noexcept override
  {
    call_order.push_back("capture");
    return true;
  }

  void reset() noexcept override
  {
    call_order.push_back("reset");
  }

  std::vector<std::string> call_order{};
  int delay_ms{0};
};

class FailingDsp final : public DspAdapter
{
public:
  bool process_render(const DspFrame &) noexcept override {return true;}
  bool set_stream_delay_ms(const int) noexcept override {return true;}
  bool process_capture(DspFrame &) noexcept override {return false;}
  void reset() noexcept override {}
};

class Trace final : public SpeechOutputTraceSink
{
public:
  void on_played(const std::uint64_t, const std::uint64_t samples) noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex);
    played_samples = samples;
    condition.notify_all();
  }

  void on_result(const SpeechResult & value) noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex);
    result = value;
    ++result_count;
    result_received = true;
    condition.notify_all();
  }

  bool wait_for_result()
  {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, 2s, [this]() {return result_received;});
  }

  std::mutex mutex{};
  std::condition_variable condition{};
  SpeechResult result{};
  std::uint64_t played_samples{0U};
  std::size_t result_count{0U};
  bool result_received{false};
};

class RecordingTts final : public TtsAdapter
{
public:
  explicit RecordingTts(
    ManualFullDuplexDevice & device, const std::size_t frame_count = 1U)
  : device_(device), frame_count_(frame_count)
  {
  }

  void start(const TtsRequest & request, TtsSink & sink) noexcept override
  {
    // This assertion is intentionally at the synthesis entrypoint: the
    // publication boundary must have closed capture before Agent can ask for
    // Speak. The runner may reopen the same device for the later PCM pass.
    EXPECT_GT(device_.close_count(), 0U);
    EXPECT_FALSE(device_.is_open());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      started_ = true;
    }
    condition_.notify_all();
    std::array<Sample, 147U> pcm{};
    for (std::size_t frame = 0U; frame < frame_count_; ++frame) {
      pcm.fill(static_cast<Sample>(1000 + frame));
      (void)sink.on_pcm(request.scope_id, 22050U, 1U, pcm.data(), pcm.size());
    }
    sink.on_complete(request.scope_id);
  }

  void cancel(const std::uint64_t) noexcept override {}

  bool wait_started()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this]() {return started_;});
  }

private:
  ManualFullDuplexDevice & device_;
  std::size_t frame_count_{1U};
  std::mutex mutex_{};
  std::condition_variable condition_{};
  bool started_{false};
};

}  // namespace

TEST(MicrophoneOnceRunnerTest, EndsOneCaptureBeforeTheOnlySpeakPlayback)
{
  ContextGuard context;
  ManualFullDuplexDevice device;
  auto vad = std::make_unique<ScriptedVad>();
  auto * const scripted_vad = vad.get();
  auto asr = std::make_unique<ScriptedAsr>();
  auto * const scripted_asr = asr.get();
  auto recognizer = std::make_unique<SenseVoiceProvider>(std::move(vad), std::move(asr));
  PassthroughDsp dsp_adapter;
  Trace trace;
  auto tts = std::make_unique<RecordingTts>(device);
  auto * const recording_tts = tts.get();
  MicrophoneOnceRunner runner(
    std::move(recognizer), std::move(tts), dsp_adapter, &device, &trace,
    MicrophoneOnceSpec{2U, 100.0F});
  ASSERT_TRUE(device.is_open());

  auto observer = std::make_shared<rclcpp::Node>("microphone_once_runner_observer");
  std::mutex turn_mutex;
  std::condition_variable turn_condition;
  std::size_t turn_count = 0U;
  const auto turn_subscription =
    observer->create_subscription<voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", voice_turn_qos(),
    [&turn_mutex, &turn_condition, &turn_count](
      const voice_nav_interfaces::msg::VoiceTurn::SharedPtr) {
      std::lock_guard<std::mutex> lock(turn_mutex);
      ++turn_count;
      turn_condition.notify_all();
      });
  const auto client = rclcpp_action::create_client<VoicePipeline::Speak>(observer, "/voice/speak");
  rclcpp::executors::MultiThreadedExecutor executor;
  runner.add_to_executor(executor);
  executor.add_node(observer);
  std::thread spin_thread([&executor]() {executor.spin();});
  ASSERT_TRUE(client->wait_for_action_server(2s));

  ASSERT_EQ(runner.pump(), MicrophoneOnceResult::kCapturing);
  ASSERT_TRUE(device.emit_capture(7));
  ASSERT_EQ(runner.pump(), MicrophoneOnceResult::kCapturing);
  ASSERT_TRUE(device.emit_capture(11));
  ASSERT_EQ(runner.pump(), MicrophoneOnceResult::kCapturing);

  {
    std::unique_lock<std::mutex> lock(turn_mutex);
    ASSERT_TRUE(turn_condition.wait_for(lock, 2s, [&turn_count]() {return turn_count == 1U;}));
  }
  EXPECT_EQ(device.close_count(), 1U);
  EXPECT_FALSE(device.is_open());
  ASSERT_EQ(runner.pump(), MicrophoneOnceResult::kReadyForPlayback);
  EXPECT_EQ(turn_count, 1U);
  EXPECT_EQ(scripted_vad->process_count, 2U);
  EXPECT_EQ(scripted_asr->last_sample_count, 320U);
  EXPECT_EQ(dsp_adapter.delay_ms, 100);
  EXPECT_EQ(dsp_adapter.call_order, (std::vector<std::string>{
      "render", "delay", "capture", "render", "delay", "capture"}));
  EXPECT_EQ(device.close_count(), 1U);
  EXPECT_FALSE(device.is_open());

  VoicePipeline::Speak::Goal goal{};
  goal.source_instance_id = "voice-instance";
  goal.source_seq = 1U;
  goal.session_id = "session";
  goal.turn_id = "turn";
  goal.priority = VoicePipeline::Speak::Goal::NORMAL;
  goal.text = "任务已完成";
  client->async_send_goal(goal);
  ASSERT_TRUE(recording_tts->wait_started());
  EXPECT_FALSE(device.is_open());
  ASSERT_TRUE(runner.allow_playback());
  ASSERT_TRUE(device.is_open());
  ASSERT_TRUE(device.emit_capture(19));
  EXPECT_EQ(runner.pump(), MicrophoneOnceResult::kReadyForPlayback);
  EXPECT_EQ(scripted_vad->process_count, 2U);
  EXPECT_EQ(turn_count, 1U);
  ASSERT_TRUE(device.emit_capture(0));
  ASSERT_TRUE(device.emit_capture(0));
  ASSERT_TRUE(trace.wait_for_result());
  EXPECT_EQ(trace.result.code, SpeechResultCode::Completed);
  EXPECT_GT(trace.played_samples, 0U);

  executor.cancel();
  spin_thread.join();
  runner.remove_from_executor(executor);
  (void)turn_subscription;
}

TEST(MicrophoneOnceRunnerTest, KeepsMultiFrameSpeakCompletedAfterPlaybackOnlyCallbacks)
{
  ContextGuard context;
  ManualFullDuplexDevice device;
  auto vad = std::make_unique<ScriptedVad>();
  auto recognizer = std::make_unique<SenseVoiceProvider>(
    std::move(vad), std::make_unique<ScriptedAsr>());
  PassthroughDsp dsp_adapter;
  Trace trace;
  auto tts = std::make_unique<RecordingTts>(device, 4U);
  auto * const recording_tts = tts.get();
  MicrophoneOnceRunner runner(
    std::move(recognizer), std::move(tts), dsp_adapter, &device, &trace,
    MicrophoneOnceSpec{2U, 100.0F});
  ASSERT_TRUE(device.is_open());

  auto observer = std::make_shared<rclcpp::Node>("microphone_once_playback_only_observer");
  std::mutex turn_mutex;
  std::condition_variable turn_condition;
  std::size_t turn_count = 0U;
  const auto turn_subscription =
    observer->create_subscription<voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", voice_turn_qos(),
    [&turn_mutex, &turn_condition, &turn_count](
      const voice_nav_interfaces::msg::VoiceTurn::SharedPtr) {
      std::lock_guard<std::mutex> lock(turn_mutex);
      ++turn_count;
      turn_condition.notify_all();
    });
  const auto client = rclcpp_action::create_client<VoicePipeline::Speak>(observer, "/voice/speak");
  rclcpp::executors::MultiThreadedExecutor executor;
  runner.add_to_executor(executor);
  executor.add_node(observer);
  std::thread spin_thread([&executor]() {executor.spin();});
  ASSERT_TRUE(client->wait_for_action_server(2s));

  ASSERT_EQ(runner.pump(), MicrophoneOnceResult::kCapturing);
  ASSERT_TRUE(device.emit_capture(7));
  ASSERT_EQ(runner.pump(), MicrophoneOnceResult::kCapturing);
  ASSERT_TRUE(device.emit_capture(11));
  ASSERT_EQ(runner.pump(), MicrophoneOnceResult::kCapturing);

  {
    std::unique_lock<std::mutex> lock(turn_mutex);
    ASSERT_TRUE(turn_condition.wait_for(lock, 2s, [&turn_count]() {
        return turn_count == 1U;
      }));
  }
  ASSERT_EQ(runner.pump(), MicrophoneOnceResult::kReadyForPlayback);
  EXPECT_EQ(turn_count, 1U);

  VoicePipeline::Speak::Goal goal{};
  goal.source_instance_id = "voice-instance";
  goal.source_seq = 1U;
  goal.session_id = "session";
  goal.turn_id = "turn";
  goal.priority = VoicePipeline::Speak::Goal::NORMAL;
  goal.text = "任务已完成";
  client->async_send_goal(goal);
  ASSERT_TRUE(recording_tts->wait_started());
  ASSERT_TRUE(runner.allow_playback());
  for (std::size_t callback = 0U; callback < AudioEngine::kRingCapacity + 1U; ++callback) {
    ASSERT_TRUE(device.emit_capture(static_cast<Sample>(callback)));
  }
  ASSERT_TRUE(trace.wait_for_result());
  EXPECT_EQ(trace.result.code, SpeechResultCode::Completed);
  EXPECT_EQ(trace.result_count, 1U);
  EXPECT_EQ(turn_count, 1U);

  executor.cancel();
  spin_thread.join();
  runner.remove_from_executor(executor);
  (void)turn_subscription;
}

TEST(MicrophoneOnceRunnerTest, EmptyCaptureExpiresWithoutAFalseTurn)
{
  ContextGuard context;
  ManualFullDuplexDevice device;
  auto vad = std::make_unique<SpeechOnlyVad>();
  auto * const scripted_vad = vad.get();
  auto recognizer = std::make_unique<SenseVoiceProvider>(
    std::move(vad), std::make_unique<ScriptedAsr>());
  PassthroughDsp dsp_adapter;
  MicrophoneOnceRunner runner(
    std::move(recognizer), std::make_unique<RecordingTts>(device), dsp_adapter, &device, nullptr,
    MicrophoneOnceSpec{2U, 100.0F});

  EXPECT_EQ(runner.expire(), MicrophoneOnceResult::kEmpty);
  EXPECT_EQ(scripted_vad->process_count, 0U);
  EXPECT_FALSE(device.is_open());
}

TEST(MicrophoneOnceRunnerTest, CaptureTimeoutStopsWithoutAFalseTurn)
{
  ContextGuard context;
  ManualFullDuplexDevice device;
  auto vad = std::make_unique<SpeechOnlyVad>();
  auto * const scripted_vad = vad.get();
  auto recognizer = std::make_unique<SenseVoiceProvider>(
    std::move(vad), std::make_unique<ScriptedAsr>());
  PassthroughDsp dsp_adapter;
  MicrophoneOnceRunner runner(
    std::move(recognizer), std::make_unique<RecordingTts>(device), dsp_adapter, &device, nullptr,
    MicrophoneOnceSpec{1U, 100.0F});

  ASSERT_TRUE(device.emit_capture(7));
  EXPECT_EQ(runner.pump(), MicrophoneOnceResult::kCapturing);
  ASSERT_TRUE(scripted_vad->wait_processed());
  EXPECT_EQ(runner.expire(), MicrophoneOnceResult::kTimedOut);
  EXPECT_EQ(scripted_vad->process_count, 1U);
  EXPECT_FALSE(device.is_open());
}

TEST(MicrophoneOnceRunnerTest, DspFailureStopsWithoutAFalseTurn)
{
  ContextGuard context;
  ManualFullDuplexDevice device;
  auto vad = std::make_unique<ScriptedVad>();
  auto * const scripted_vad = vad.get();
  auto asr = std::make_unique<ScriptedAsr>();
  auto * const scripted_asr = asr.get();
  auto recognizer = std::make_unique<SenseVoiceProvider>(std::move(vad), std::move(asr));
  FailingDsp dsp_adapter;
  MicrophoneOnceRunner runner(
    std::move(recognizer), std::make_unique<RecordingTts>(device), dsp_adapter, &device, nullptr,
    MicrophoneOnceSpec{2U, 100.0F});

  ASSERT_TRUE(device.emit_capture(7));
  EXPECT_EQ(runner.pump(), MicrophoneOnceResult::kFailed);
  EXPECT_EQ(scripted_vad->process_count, 0U);
  EXPECT_EQ(scripted_asr->last_sample_count, 0U);
  EXPECT_FALSE(device.is_open());
}

}  // namespace voice_nav_audio
