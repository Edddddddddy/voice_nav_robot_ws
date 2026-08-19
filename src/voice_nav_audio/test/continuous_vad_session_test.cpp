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
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "continuous_vad_session.hpp"
#include "gtest/gtest.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "voice_nav_interfaces/msg/voice_turn.hpp"

namespace voice_nav_audio
{
namespace
{

class ManualDevice final : public FullDuplexAudioDevice
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
    ++open_count;
    return true;
  }

  void close() noexcept override
  {
    callback_ = nullptr;
    context_ = nullptr;
  }

  bool emit_capture(const Sample value)
  {
    if (callback_ == nullptr || context_ == nullptr) {
      return false;
    }
    std::array<Sample, AudioEngine::kFrameSamples> capture{};
    std::array<Sample, AudioEngine::kFrameSamples> output{};
    capture.fill(value);
    callback_(context_, capture.data(), output.data(), output.size(), CallbackStatus{});
    last_output_nonzero = false;
    for (const auto sample : output) {
      last_output_nonzero = last_output_nonzero || sample != 0;
    }
    return true;
  }

  [[nodiscard]] bool is_open() const noexcept
  {
    return callback_ != nullptr && context_ != nullptr;
  }

  std::size_t open_count{0U};
  bool last_output_nonzero{false};

private:
  DeviceCallback callback_{nullptr};
  void * context_{nullptr};
};

class FakeDsp final : public DspAdapter
{
public:
  bool process_render(const DspFrame & frame) noexcept override
  {
    for (const auto sample : frame.samples) {
      saw_nonzero_reference = saw_nonzero_reference || sample != 0;
    }
    return true;
  }

  bool set_stream_delay_ms(int) noexcept override {return true;}

  bool process_capture(DspFrame &) noexcept override {return capture_succeeds;}

  void reset() noexcept override {}

  bool saw_nonzero_reference{false};
  bool capture_succeeds{true};
};

class ContinuousRecognizer final : public SpeechRecognizerAdapter
{
public:
  void process_frame(
    const CleanedAudioFrame & frame, SpeechEventSink & sink) noexcept override
  {
    ++process_count;
    if (failure_mode) {
      SpeechRecognitionEvent failure{};
      failure.kind = SpeechEventKind::kFailure;
      failure.audio_generation = frame.audio_generation;
      failure.audio_seq = frame.audio_seq;
      sink.on_speech_event(failure);
      return;
    }
    sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
      frame, scope, "连续回合", 1.0F));
  }

  void on_turn_scope_opened(const TurnScopeIdentity & value) noexcept override
  {
    ++scope_open_count;
    scope = value;
  }

  void on_turn_scope_retired(const TurnScopeIdentity &) noexcept override
  {
    ++scope_retired_count;
    scope = TurnScopeIdentity{};
  }

  std::size_t process_count{0U};
  std::size_t scope_open_count{0U};
  std::size_t scope_retired_count{0U};
  bool failure_mode{false};

private:
  TurnScopeIdentity scope{};
};

class FakeTts final : public TtsAdapter
{
public:
  void start(const TtsRequest & request, TtsSink & sink) noexcept override
  {
    std::array<Sample, 160U> pcm{};
    pcm.fill(1000);
    (void)sink.on_pcm(request.scope_id, 22050U, 1U, pcm.data(), pcm.size());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      started = true;
    }
    started_condition_.notify_all();
  }

  void cancel(std::uint64_t) noexcept override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++cancel_count;
    }
    canceled_condition_.notify_all();
  }

  [[nodiscard]] bool wait_started()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return started_condition_.wait_for(lock, std::chrono::seconds(2), [this]() {
      return started;
    });
  }

  [[nodiscard]] bool wait_canceled()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return canceled_condition_.wait_for(lock, std::chrono::seconds(2), [this]() {
      return cancel_count != 0U;
    });
  }

  bool started{false};
  std::size_t cancel_count{0U};

private:
  std::mutex mutex_{};
  std::condition_variable started_condition_{};
  std::condition_variable canceled_condition_{};
};

class Trace final : public SpeechOutputTraceSink
{
public:
  void on_played(std::uint64_t, std::uint64_t) noexcept override {}

  void on_result(const SpeechResult & result) noexcept override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      results.push_back(result);
    }
    condition_.notify_all();
  }

  [[nodiscard]] bool wait_for_barge_in()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::seconds(2), [this]() {
      for (const auto & result : results) {
        if (result.code == SpeechResultCode::BargedIn) {
          return true;
        }
      }
      return false;
    });
  }

  std::vector<SpeechResult> results{};

private:
  std::mutex mutex_{};
  std::condition_variable condition_{};
};

class ExecutorRunner final
{
public:
  explicit ExecutorRunner(rclcpp::executors::MultiThreadedExecutor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorRunner()
  {
    executor_.cancel();
    thread_.join();
  }

private:
  rclcpp::executors::MultiThreadedExecutor & executor_;
  std::thread thread_{};
};

class RclcppContextGuard final
{
public:
  RclcppContextGuard() {rclcpp::init(0, nullptr);}

  ~RclcppContextGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

void warm_up(ContinuousVadSession & session, ManualDevice & device)
{
  for (std::size_t frame = 0U;
    frame < ContinuousVadSession::kReadinessWarmupFrames; ++frame)
  {
    ASSERT_TRUE(device.emit_capture(0));
    ASSERT_EQ(session.pump(), ContinuousVadPumpResult::kCapturing);
  }
}

}  // namespace

TEST(ContinuousVadSessionTest, KeepsOneRecognizerAndDeviceForTwoContinuousTurns)
{
  RclcppContextGuard context;
  auto recognizer = std::make_unique<ContinuousRecognizer>();
  auto * const recognizer_view = recognizer.get();
  FakeDsp dsp;
  ManualDevice device;
  auto session = std::make_unique<ContinuousVadSession>(
    std::move(recognizer), dsp, std::make_unique<FakeTts>(), &device);
  auto observer = std::make_shared<rclcpp::Node>("continuous_vad_identity_observer");
  std::vector<voice_nav_interfaces::msg::VoiceTurn> turns;
  std::mutex turn_mutex;
  std::condition_variable turn_condition;
  const auto subscription = observer->create_subscription<
    voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", voice_turn_qos(),
    [&turns, &turn_mutex, &turn_condition](
      const voice_nav_interfaces::msg::VoiceTurn::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(turn_mutex);
          turns.push_back(*message);
        }
        turn_condition.notify_all();
      });
  rclcpp::executors::MultiThreadedExecutor executor;
  session->add_to_executor(executor);
  executor.add_node(observer);
  ExecutorRunner executor_runner(executor);

  warm_up(*session, device);
  ASSERT_TRUE(device.emit_capture(100));
  ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  {
    std::unique_lock<std::mutex> lock(turn_mutex);
    ASSERT_TRUE(turn_condition.wait_for(lock, std::chrono::seconds(2), [&turns]() {
      return turns.size() >= 1U;
    }));
  }
  ASSERT_TRUE(device.emit_capture(100));
  ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  {
    std::unique_lock<std::mutex> lock(turn_mutex);
    ASSERT_TRUE(turn_condition.wait_for(lock, std::chrono::seconds(2), [&turns]() {
      return turns.size() >= 2U;
    }));
  }

  EXPECT_EQ(recognizer_view->process_count, 2U);
  EXPECT_EQ(recognizer_view->scope_open_count, 2U);
  EXPECT_EQ(recognizer_view->scope_retired_count, 2U);
  EXPECT_EQ(device.open_count, 1U);
  EXPECT_TRUE(device.is_open());
  {
    std::lock_guard<std::mutex> lock(turn_mutex);
    ASSERT_EQ(turns.size(), 2U);
    EXPECT_FALSE(turns[0U].voice_instance_id.empty());
    EXPECT_EQ(turns[0U].voice_instance_id, turns[1U].voice_instance_id);
    EXPECT_FALSE(turns[0U].session_id.empty());
    EXPECT_EQ(turns[0U].session_id, turns[1U].session_id);
    EXPECT_EQ(turns[0U].voice_seq, 1U);
    EXPECT_EQ(turns[1U].voice_seq, 2U);
  }
  executor.remove_node(observer);
  session->remove_from_executor(executor);
  session->stop();
  (void)subscription;
}

TEST(ContinuousVadSessionTest, DefersVoiceTurnPublisherUntilThreeCleanedWarmupFrames)
{
  RclcppContextGuard context;
  auto recognizer = std::make_unique<ContinuousRecognizer>();
  recognizer->failure_mode = true;
  FakeDsp dsp;
  ManualDevice device;
  auto session = std::make_unique<ContinuousVadSession>(
    std::move(recognizer), dsp, std::make_unique<FakeTts>(), &device);
  auto observer = std::make_shared<rclcpp::Node>("continuous_vad_readiness_observer");
  const auto has_frontend = [&observer]() {
      const auto publishers = observer->get_publishers_info_by_topic("/voice/turn");
      return std::any_of(publishers.cbegin(), publishers.cend(), [](const auto & endpoint) {
          return endpoint.node_name() == "voice_speech_input";
        });
    };
  EXPECT_FALSE(has_frontend());

  for (std::size_t frame = 1U;
    frame < ContinuousVadSession::kReadinessWarmupFrames; ++frame)
  {
    ASSERT_TRUE(device.emit_capture(0));
    ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
    EXPECT_FALSE(has_frontend());
  }
  ASSERT_TRUE(device.emit_capture(0));
  ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  EXPECT_TRUE(has_frontend());
  session->stop();
}

TEST(ContinuousVadSessionTest, BargeInKeepsCaptureOpenAndCancelsPlaybackOnce)
{
  RclcppContextGuard context;
  auto recognizer = std::make_unique<ContinuousRecognizer>();
  FakeDsp dsp;
  ManualDevice device;
  auto tts = std::make_unique<FakeTts>();
  auto * const fake_tts = tts.get();
  Trace trace;
  auto session = std::make_unique<ContinuousVadSession>(
    std::move(recognizer), dsp, std::move(tts), &device, &trace);
  auto observer = std::make_shared<rclcpp::Node>("continuous_vad_barge_observer");
  std::size_t turn_count = 0U;
  std::mutex turn_mutex;
  std::condition_variable turn_condition;
  voice_nav_interfaces::msg::VoiceTurn turn{};
  const auto subscription = observer->create_subscription<
    voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", voice_turn_qos(),
    [&turn_count, &turn_mutex, &turn_condition, &turn](
      const voice_nav_interfaces::msg::VoiceTurn::SharedPtr message) {
        std::lock_guard<std::mutex> lock(turn_mutex);
        turn = *message;
        ++turn_count;
        turn_condition.notify_all();
      });
  auto client = rclcpp_action::create_client<VoicePipeline::Speak>(observer, "/voice/speak");
  rclcpp::executors::MultiThreadedExecutor executor;
  session->add_to_executor(executor);
  executor.add_node(observer);
  ExecutorRunner executor_runner(executor);

  warm_up(*session, device);
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
  ASSERT_TRUE(device.emit_capture(100));
  ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  {
    std::unique_lock<std::mutex> lock(turn_mutex);
    ASSERT_TRUE(turn_condition.wait_for(lock, std::chrono::seconds(2), [&turn_count]() {
      return turn_count == 1U;
    }));
  }

  VoicePipeline::Speak::Goal goal{};
  goal.source_instance_id = turn.voice_instance_id;
  goal.source_seq = turn.voice_seq;
  goal.session_id = turn.session_id;
  goal.turn_id = turn.turn_id;
  goal.priority = VoicePipeline::Speak::Goal::NORMAL;
  goal.text = "正在播报";
  goal.allow_barge_in = true;
  const auto goal_future = client->async_send_goal(goal);
  ASSERT_EQ(goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_NE(goal_future.get(), nullptr);
  ASSERT_TRUE(fake_tts->wait_started());

  ASSERT_TRUE(device.emit_capture(100));
  ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  EXPECT_TRUE(device.is_open());
  EXPECT_TRUE(device.last_output_nonzero);
  EXPECT_TRUE(dsp.saw_nonzero_reference);
  EXPECT_TRUE(fake_tts->wait_canceled());
  EXPECT_TRUE(trace.wait_for_barge_in());
  {
    std::unique_lock<std::mutex> lock(turn_mutex);
    ASSERT_TRUE(turn_condition.wait_for(lock, std::chrono::seconds(2), [&turn_count]() {
      return turn_count == 2U;
    }));
  }

  executor.remove_node(observer);
  session->remove_from_executor(executor);
  session->stop();
  session.reset();
  (void)subscription;
}

TEST(ContinuousVadSessionTest, StopsInputAfterDspFailure)
{
  RclcppContextGuard context;
  auto recognizer = std::make_unique<ContinuousRecognizer>();
  auto * const recognizer_view = recognizer.get();
  FakeDsp dsp;
  dsp.capture_succeeds = false;
  ManualDevice device;
  auto session = std::make_unique<ContinuousVadSession>(
    std::move(recognizer), dsp, std::make_unique<FakeTts>(), &device);

  ASSERT_TRUE(device.emit_capture(100));
  EXPECT_EQ(session->pump(), ContinuousVadPumpResult::kFailed);
  EXPECT_FALSE(device.is_open());
  EXPECT_EQ(recognizer_view->process_count, 0U);
}

TEST(ContinuousVadSessionTest, ShutdownStopsCaptureBeforeTheRecognizerCanProcessAgain)
{
  RclcppContextGuard context;
  auto recognizer = std::make_unique<ContinuousRecognizer>();
  auto * const recognizer_view = recognizer.get();
  FakeDsp dsp;
  ManualDevice device;
  auto session = std::make_unique<ContinuousVadSession>(
    std::move(recognizer), dsp, std::make_unique<FakeTts>(), &device);
  warm_up(*session, device);
  session->stop();
  EXPECT_FALSE(device.is_open());
  EXPECT_FALSE(device.emit_capture(100));
  EXPECT_EQ(recognizer_view->process_count, 0U);
}

}  // namespace voice_nav_audio
