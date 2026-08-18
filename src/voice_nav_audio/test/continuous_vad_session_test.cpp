// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "continuous_vad_session.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
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

enum class FakeTerminalMode
{
  kValidFinal,
  kFailureBeforeWake,
  kTimeout,
  kInvalidFinal,
};

class FakeChild final : public SpeechRecognizerAdapter
{
public:
  explicit FakeChild(
    const bool publish_turn = true, const std::size_t endpoint_frame = 1U,
    const FakeTerminalMode terminal_mode = FakeTerminalMode::kValidFinal) noexcept
  : publish_turn_(publish_turn), endpoint_frame_(endpoint_frame), terminal_mode_(terminal_mode)
  {
  }

  void process_frame(
    const CleanedAudioFrame & frame, SpeechEventSink & sink) noexcept override
  {
    ++frame_count;
    if (!publish_turn_) {
      return;
    }
    if (frame_count == 1U && terminal_mode_ == FakeTerminalMode::kFailureBeforeWake) {
      SpeechRecognitionEvent failure{};
      failure.kind = SpeechEventKind::kFailure;
      failure.audio_generation = frame.audio_generation;
      failure.audio_seq = frame.audio_seq;
      sink.on_speech_event(failure);
      return;
    }
    if (frame_count == 1U && terminal_mode_ == FakeTerminalMode::kTimeout) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    }
    if (frame_count == 1U && terminal_mode_ == FakeTerminalMode::kInvalidFinal) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
      sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
        frame, scope, {}, 1.0F));
      return;
    }
    if (frame_count == 1U && terminal_mode_ == FakeTerminalMode::kTimeout) {
      SpeechRecognitionEvent timeout{};
      timeout.kind = SpeechEventKind::kTimeout;
      timeout.audio_generation = frame.audio_generation;
      timeout.audio_seq = frame.audio_seq;
      timeout.scope = scope;
      sink.on_speech_event(timeout);
      return;
    }
    if (frame_count == 1U && terminal_mode_ == FakeTerminalMode::kValidFinal) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    }
    if (terminal_mode_ == FakeTerminalMode::kValidFinal && frame_count == endpoint_frame_) {
      sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
        frame, scope, "前进半米", 1.0F));
    }
  }

  void on_turn_scope_opened(const TurnScopeIdentity & value) noexcept override
  {
    scope = value;
  }

  void on_turn_scope_retired(const TurnScopeIdentity &) noexcept override
  {
    scope = TurnScopeIdentity{};
  }

  std::size_t frame_count{0U};
  TurnScopeIdentity scope{};

private:
  bool publish_turn_{true};
  std::size_t endpoint_frame_{1U};
  FakeTerminalMode terminal_mode_{FakeTerminalMode::kValidFinal};
};

class FakeFactory final : public OneShotRecognizerFactory
{
public:
  std::unique_ptr<SpeechRecognizerAdapter> create_armed() override
  {
    ++create_count;
    if (fail_after_first && create_count > 1U) {
      return nullptr;
    }
    auto child = std::make_unique<FakeChild>(publish_turn, endpoint_frame, terminal_mode);
    children.push_back(child.get());
    return child;
  }

  std::size_t create_count{0U};
  std::vector<FakeChild *> children{};
  bool publish_turn{true};
  std::size_t endpoint_frame{1U};
  FakeTerminalMode terminal_mode{FakeTerminalMode::kValidFinal};
  bool fail_after_first{false};
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
  RclcppContextGuard()
  {
    rclcpp::init(0, nullptr);
  }

  ~RclcppContextGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

}  // namespace

TEST(ContinuousVadSessionTest, PublishesReadinessOnlyAfterThreeCleanedWarmupFrames)
{
  RclcppContextGuard context;
  FakeFactory factory;
  factory.publish_turn = false;
  FakeDsp dsp;
  ManualDevice device;
  auto session = std::make_unique<ContinuousVadSession>(
    factory, dsp, std::make_unique<FakeTts>(), &device);
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

  const auto graph_event = observer->get_graph_event();
  ASSERT_TRUE(device.emit_capture(0));
  ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  if (!has_frontend()) {
    observer->wait_for_graph_change(graph_event, std::chrono::seconds(2));
  }
  EXPECT_TRUE(has_frontend());
  session->stop();
}

TEST(ContinuousVadSessionTest, ConstructorCreatesTheFirstOneShotChildOnce)
{
  RclcppContextGuard context;
  FakeFactory factory;
  FakeDsp dsp;
  ManualDevice device;

  auto session = std::make_unique<ContinuousVadSession>(
    factory, dsp, std::make_unique<FakeTts>(), &device);
  EXPECT_EQ(factory.create_count, 1U);
  EXPECT_EQ(device.open_count, 1U);
  session.reset();
}

TEST(ContinuousVadSessionTest, StartsTheNextChildOnThePumpAfterACompletedTurn)
{
  RclcppContextGuard context;
  FakeFactory factory;
  FakeDsp dsp;
  ManualDevice device;
  auto session = std::make_unique<ContinuousVadSession>(
    factory, dsp, std::make_unique<FakeTts>(), &device);
  auto observer = std::make_shared<rclcpp::Node>("continuous_vad_turn_observer");
  std::mutex turn_mutex;
  std::condition_variable turn_condition;
  std::vector<voice_nav_interfaces::msg::VoiceTurn> turns;
  const auto subscription = observer->create_subscription<
    voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", voice_turn_qos(),
    [&turn_mutex, &turn_condition, &turns](
      const voice_nav_interfaces::msg::VoiceTurn::SharedPtr message) {
        std::lock_guard<std::mutex> lock(turn_mutex);
        turns.push_back(*message);
        turn_condition.notify_all();
      });
  rclcpp::executors::MultiThreadedExecutor executor;
  session->add_to_executor(executor);
  executor.add_node(observer);
  ExecutorRunner executor_runner(executor);

  for (std::size_t frame = 0U;
    frame < ContinuousVadSession::kReadinessWarmupFrames; ++frame)
  {
    ASSERT_TRUE(device.emit_capture(0));
    ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  }
  ASSERT_EQ(factory.create_count, 1U);
  ASSERT_TRUE(device.emit_capture(100));
  EXPECT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  ASSERT_EQ(factory.children.front()->frame_count, 1U);
  {
    std::unique_lock<std::mutex> lock(turn_mutex);
    ASSERT_TRUE(turn_condition.wait_for(lock, std::chrono::seconds(2), [&turns]() {
      return turns.size() == 1U;
    }));
  }

  ASSERT_TRUE(device.emit_capture(100));
  EXPECT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  EXPECT_EQ(factory.create_count, 2U);
  EXPECT_EQ(factory.children.back()->frame_count, 1U);
  {
    std::unique_lock<std::mutex> lock(turn_mutex);
    ASSERT_TRUE(turn_condition.wait_for(lock, std::chrono::seconds(2), [&turns]() {
      return turns.size() == 2U;
    }));
  }
  ASSERT_EQ(turns[0].voice_instance_id, turns[1].voice_instance_id);
  ASSERT_EQ(turns[0].session_id, turns[1].session_id);
  EXPECT_EQ(turns[0].voice_seq, 1U);
  EXPECT_EQ(turns[1].voice_seq, 2U);
  EXPECT_EQ(device.open_count, 1U);
  EXPECT_TRUE(device.is_open());

  executor.remove_node(observer);
  session->remove_from_executor(executor);
  session->stop();
  session.reset();
}

TEST(ContinuousVadSessionTest, ReArmsAfterEveryRecognizerTerminalWithoutWaiting)
{
  RclcppContextGuard context;
  const std::array<FakeTerminalMode, 4U> terminal_modes{
    FakeTerminalMode::kFailureBeforeWake,
    FakeTerminalMode::kTimeout,
    FakeTerminalMode::kInvalidFinal,
    FakeTerminalMode::kValidFinal,
  };
  for (const auto mode : terminal_modes) {
    SCOPED_TRACE(static_cast<int>(mode));
    FakeFactory factory;
    factory.terminal_mode = mode;
    FakeDsp dsp;
    ManualDevice device;
    auto session = std::make_unique<ContinuousVadSession>(
      factory, dsp, std::make_unique<FakeTts>(), &device);

    for (std::size_t frame = 0U;
      frame < ContinuousVadSession::kReadinessWarmupFrames; ++frame)
    {
      ASSERT_TRUE(device.emit_capture(0));
      ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
    }
    ASSERT_EQ(factory.create_count, 1U);

    ASSERT_TRUE(device.emit_capture(100));
    ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
    ASSERT_EQ(factory.create_count, 1U);
    EXPECT_TRUE(device.is_open());

    // The terminal callback is consumed by the next control pump, with no
    // sleep/retry window and without restarting the full-duplex device.
    ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
    EXPECT_EQ(factory.create_count, 2U);
    EXPECT_EQ(device.open_count, 1U);
    EXPECT_TRUE(device.is_open());
    EXPECT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
    EXPECT_EQ(factory.create_count, 2U);
    session->stop();
  }
}

TEST(ContinuousVadSessionTest, ReArmFactoryFailureStopsCaptureWithoutThrowing)
{
  RclcppContextGuard context;
  FakeFactory factory;
  factory.fail_after_first = true;
  FakeDsp dsp;
  ManualDevice device;
  auto session = std::make_unique<ContinuousVadSession>(
    factory, dsp, std::make_unique<FakeTts>(), &device);
  for (std::size_t frame = 0U;
    frame < ContinuousVadSession::kReadinessWarmupFrames; ++frame)
  {
    ASSERT_TRUE(device.emit_capture(0));
    ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  }
  ASSERT_TRUE(device.emit_capture(100));
  ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  EXPECT_EQ(session->pump(), ContinuousVadPumpResult::kFailed);
  EXPECT_EQ(factory.create_count, 2U);
  EXPECT_FALSE(device.is_open());
}

TEST(ContinuousVadSessionTest, DspFailureStopsCaptureWithoutThrowing)
{
  RclcppContextGuard context;
  FakeFactory factory;
  FakeDsp dsp;
  dsp.capture_succeeds = false;
  ManualDevice device;
  auto session = std::make_unique<ContinuousVadSession>(
    factory, dsp, std::make_unique<FakeTts>(), &device);

  ASSERT_TRUE(device.emit_capture(100));
  EXPECT_EQ(session->pump(), ContinuousVadPumpResult::kFailed);
  EXPECT_FALSE(device.is_open());
}

TEST(ContinuousVadSessionTest, PublisherActivationFailureStopsCaptureWithoutThrowing)
{
  rclcpp::init(0, nullptr);
  FakeFactory factory;
  FakeDsp dsp;
  ManualDevice device;
  auto session = std::make_unique<ContinuousVadSession>(
    factory, dsp, std::make_unique<FakeTts>(), &device);

  for (std::size_t frame = 0U;
    frame + 1U < ContinuousVadSession::kReadinessWarmupFrames; ++frame)
  {
    ASSERT_TRUE(device.emit_capture(0));
    ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  }
  rclcpp::shutdown();
  ASSERT_TRUE(device.emit_capture(0));
  EXPECT_EQ(session->pump(), ContinuousVadPumpResult::kFailed);
  EXPECT_FALSE(device.is_open());
  session.reset();
}

TEST(ContinuousVadSessionTest, BargeInKeepsCaptureOpenForTheNextCleanedTurn)
{
  RclcppContextGuard context;
  FakeFactory factory;
  FakeDsp dsp;
  ManualDevice device;
  auto tts = std::make_unique<FakeTts>();
  auto * const fake_tts = tts.get();
  Trace trace;
  auto session = std::make_unique<ContinuousVadSession>(
    factory, dsp, std::move(tts), &device, &trace);
  auto observer = std::make_shared<rclcpp::Node>("continuous_vad_barge_observer");
  std::mutex turn_mutex;
  std::condition_variable turn_condition;
  voice_nav_interfaces::msg::VoiceTurn turn{};
  std::size_t turn_count = 0U;
  const auto subscription = observer->create_subscription<
    voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", voice_turn_qos(),
    [&turn_mutex, &turn_condition, &turn, &turn_count](
      const voice_nav_interfaces::msg::VoiceTurn::SharedPtr message) {
        std::lock_guard<std::mutex> lock(turn_mutex);
        turn = *message;
        ++turn_count;
        turn_condition.notify_all();
      });
  auto client = rclcpp_action::create_client<VoicePipeline::Speak>(
    observer, "/voice/speak");
  rclcpp::executors::MultiThreadedExecutor executor;
  session->add_to_executor(executor);
  executor.add_node(observer);
  ExecutorRunner executor_runner(executor);

  for (std::size_t frame = 0U;
    frame < ContinuousVadSession::kReadinessWarmupFrames; ++frame)
  {
    ASSERT_TRUE(device.emit_capture(0));
    ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  }
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
  ASSERT_EQ(
    goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_NE(goal_future.get(), nullptr);
  ASSERT_TRUE(fake_tts->wait_started());

  factory.endpoint_frame = 2U;
  ASSERT_TRUE(device.emit_capture(100));
  ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  EXPECT_EQ(factory.create_count, 2U);
  EXPECT_EQ(device.open_count, 1U);
  EXPECT_TRUE(device.is_open());
  EXPECT_TRUE(device.last_output_nonzero);
  EXPECT_TRUE(dsp.saw_nonzero_reference);
  EXPECT_TRUE(fake_tts->wait_canceled());
  EXPECT_TRUE(trace.wait_for_barge_in());

  ASSERT_TRUE(device.emit_capture(100));
  ASSERT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
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

TEST(ContinuousVadSessionTest, SilenceAndReferenceFreeCapturePublishNoTurn)
{
  RclcppContextGuard context;
  FakeFactory factory;
  factory.publish_turn = false;
  FakeDsp dsp;
  ManualDevice device;
  auto session = std::make_unique<ContinuousVadSession>(
    factory, dsp, std::make_unique<FakeTts>(), &device);
  auto observer = std::make_shared<rclcpp::Node>("continuous_vad_silence_observer");
  std::mutex turn_mutex;
  std::size_t turn_count = 0U;
  const auto subscription = observer->create_subscription<
    voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", voice_turn_qos(),
    [&turn_mutex, &turn_count](const voice_nav_interfaces::msg::VoiceTurn::SharedPtr) {
      std::lock_guard<std::mutex> lock(turn_mutex);
      ++turn_count;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  session->add_to_executor(executor);
  executor.add_node(observer);

  for (std::size_t frame = 0U;
    frame < ContinuousVadSession::kReadinessWarmupFrames; ++frame)
  {
    ASSERT_TRUE(device.emit_capture(0));
    EXPECT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  }
  executor.spin_some();
  {
    std::lock_guard<std::mutex> lock(turn_mutex);
    EXPECT_EQ(turn_count, 0U);
  }
  EXPECT_EQ(factory.create_count, 1U);
  EXPECT_EQ(device.open_count, 1U);
  EXPECT_TRUE(device.is_open());

  executor.remove_node(observer);
  session->remove_from_executor(executor);
  session->stop();
  session.reset();
  (void)subscription;
}

TEST(ContinuousVadSessionTest, RecoversAfterStartupBacklogBeforeFirstPump)
{
  RclcppContextGuard context;
  FakeFactory factory;
  factory.publish_turn = false;
  FakeDsp dsp;
  ManualDevice device;
  auto session = std::make_unique<ContinuousVadSession>(
    factory, dsp, std::make_unique<FakeTts>(), &device);
  for (std::size_t index = 0U; index < AudioEngine::kRingCapacity + 2U; ++index) {
    ASSERT_TRUE(device.emit_capture(100));
  }
  for (std::size_t index = 0U; index < AudioEngine::kRingCapacity + 2U; ++index) {
    EXPECT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  }
  for (std::size_t frame = 0U;
    frame <= ContinuousVadSession::kReadinessWarmupFrames; ++frame)
  {
    ASSERT_TRUE(device.emit_capture(static_cast<Sample>(200U + frame)));
    EXPECT_EQ(session->pump(), ContinuousVadPumpResult::kCapturing);
  }

  ASSERT_FALSE(factory.children.empty());
  EXPECT_GT(factory.children.front()->frame_count, 0U);
  session->stop();
}

}  // namespace voice_nav_audio
