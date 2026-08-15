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
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "voice_pipeline.hpp"

namespace voice_nav_audio
{
namespace
{

using namespace std::chrono_literals;

class ManualDevice final : public FullDuplexAudioDevice
{
public:
  bool open(
    const FullDuplexStreamSpec spec, const DeviceCallback callback,
    void * context) noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (spec.sample_rate != AudioEngine::kSampleRate || spec.channels != AudioEngine::kChannels ||
      spec.frames_per_buffer != AudioEngine::kFrameSamples || callback == nullptr ||
      context == nullptr)
    {
      return false;
    }
    callback_ = callback;
    context_ = context;
    opened_ = true;
    opened_condition_.notify_all();
    return true;
  }

  void close() noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = nullptr;
    context_ = nullptr;
    opened_ = false;
  }

  [[nodiscard]] bool wait_opened()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return opened_condition_.wait_for(lock, 2s, [this]() {return opened_;});
  }

  [[nodiscard]] std::array<Sample, AudioEngine::kFrameSamples> render_once()
  {
    DeviceCallback callback{};
    void * context{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      callback = callback_;
      context = context_;
    }
    std::array<Sample, AudioEngine::kFrameSamples> capture{};
    std::array<Sample, AudioEngine::kFrameSamples> output{};
    callback(context, capture.data(), output.data(), output.size(), CallbackStatus{});
    // The scripted recognizer receives cleaned frames directly. Drain the
    // manual device's unused raw side to avoid an artificial generation fence.
    auto * const engine = static_cast<AudioEngine *>(context);
    AudioFrame ignored{};
    while (engine->try_pop_reference(ignored)) {
    }
    while (engine->try_pop_capture(ignored)) {
    }
    return output;
  }

private:
  std::mutex mutex_;
  std::condition_variable opened_condition_;
  DeviceCallback callback_{nullptr};
  void * context_{nullptr};
  bool opened_{false};
};

class RejectingDevice final : public FullDuplexAudioDevice
{
public:
  bool open(const FullDuplexStreamSpec, DeviceCallback, void *) noexcept override
  {
    return false;
  }

  void close() noexcept override
  {
  }
};

class ScriptedRecognizer final : public SpeechRecognizerAdapter
{
public:
  void process_frame(const CleanedAudioFrame & frame, SpeechEventSink & sink) noexcept override
  {
    if (frame.audio_seq == 1U) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    } else if (frame.audio_seq == 2U) {
      sink.on_speech_event(SpeechRecognitionEvent::activity(frame, scope_));
    } else if (frame.audio_seq == 3U) {
      sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
        frame, scope_, "前进半米", 1.0F));
    }
  }

  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept override
  {
    scope_ = scope;
  }

  void on_turn_scope_retired(const TurnScopeIdentity &) noexcept override
  {
    scope_ = TurnScopeIdentity{};
  }

private:
  TurnScopeIdentity scope_{};
};

class DeterministicFakeTts final : public TtsAdapter
{
public:
  void start(const TtsRequest & request, TtsSink & sink) noexcept override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      started_ = true;
    }
    started_condition_.notify_all();
    std::array<Sample, 147U> pcm{};
    pcm.fill(1000);
    (void)sink.on_pcm(request.scope_id, 22050U, 1U, pcm.data(), pcm.size());
    sink.on_complete(request.scope_id);
  }

  void cancel(std::uint64_t) noexcept override
  {
  }

  [[nodiscard]] bool wait_started()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return started_condition_.wait_for(lock, 2s, [this]() {return started_;});
  }

private:
  std::mutex mutex_;
  std::condition_variable started_condition_;
  bool started_{false};
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
  std::thread thread_;
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

CleanedAudioFrame frame(const std::uint64_t sequence)
{
  CleanedAudioFrame input{};
  input.audio_generation = 1U;
  input.audio_seq = sequence;
  input.samples.fill(0);
  return input;
}

TEST(VoicePipelineTest, SharesOneEngineForVoiceTurnAndActualSpeakPlayback)
{
  RclcppContextGuard rclcpp_context;
  {
    ManualDevice device;
    auto tts = std::make_unique<DeterministicFakeTts>();
    auto * const fake_tts = tts.get();
    auto pipeline = std::make_unique<VoicePipeline>(
      std::make_unique<ScriptedRecognizer>(), std::move(tts), device);
    ASSERT_TRUE(device.wait_opened());
    const auto initial = device.render_once();
    EXPECT_TRUE(std::all_of(
      initial.cbegin(), initial.cend(),
        [](const Sample sample) {return sample == 0;}));

    auto observer = std::make_shared<rclcpp::Node>("voice_pipeline_observer");
    auto client = rclcpp_action::create_client<VoicePipeline::Speak>(observer, "/voice/speak");
    std::mutex mutex;
    std::condition_variable condition;
    std::size_t voice_turn_count = 0U;
    std::size_t feedback_count = 0U;
    bool result_received = false;
    voice_nav_interfaces::msg::VoiceTurn turn{};
    rclcpp_action::ClientGoalHandle<VoicePipeline::Speak>::WrappedResult result{};
    const auto turn_subscription = observer->create_subscription<voice_nav_interfaces::msg::VoiceTurn>(
      "/voice/turn", voice_turn_qos(),
      [&mutex, &condition, &voice_turn_count, &turn](
        const voice_nav_interfaces::msg::VoiceTurn::SharedPtr received) {
        std::lock_guard<std::mutex> lock(mutex);
        turn = *received;
        ++voice_turn_count;
        condition.notify_all();
      });

    rclcpp::executors::MultiThreadedExecutor executor;
    pipeline->add_to_executor(executor);
    executor.add_node(observer);
    ExecutorRunner runner(executor);
    ASSERT_TRUE(client->wait_for_action_server(2s));

    pipeline->accept_cleaned_frame(frame(1U));
    pipeline->accept_cleaned_frame(frame(2U));
    pipeline->accept_cleaned_frame(frame(3U));
    {
      std::unique_lock<std::mutex> lock(mutex);
      ASSERT_TRUE(condition.wait_for(lock, 2s, [&voice_turn_count]() {
          return voice_turn_count == 1U;
        }));
    }
    EXPECT_EQ(turn.text, "前进半米");

    VoicePipeline::Speak::Goal goal{};
    goal.source_instance_id = turn.voice_instance_id;
    goal.source_seq = turn.voice_seq;
    goal.session_id = turn.session_id;
    goal.turn_id = turn.turn_id;
    goal.priority = VoicePipeline::Speak::Goal::NORMAL;
    goal.text = "已完成";
    rclcpp_action::Client<VoicePipeline::Speak>::SendGoalOptions options{};
    options.feedback_callback = [&mutex, &condition, &feedback_count](
      rclcpp_action::ClientGoalHandle<VoicePipeline::Speak>::SharedPtr,
      const std::shared_ptr<const VoicePipeline::Speak::Feedback>) {
        std::lock_guard<std::mutex> lock(mutex);
        ++feedback_count;
        condition.notify_all();
      };
    options.result_callback = [&mutex, &condition, &result_received, &result](
      const rclcpp_action::ClientGoalHandle<VoicePipeline::Speak>::WrappedResult & received) {
        std::lock_guard<std::mutex> lock(mutex);
        result = received;
        result_received = true;
        condition.notify_all();
      };
    client->async_send_goal(goal, options);
    ASSERT_TRUE(fake_tts->wait_started());

    bool rendered_nonzero = false;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    std::unique_lock<std::mutex> lock(mutex);
    while (feedback_count < 1U || !result_received) {
      lock.unlock();
      const auto rendered = device.render_once();
      rendered_nonzero = rendered_nonzero || std::any_of(
        rendered.cbegin(), rendered.cend(), [](const Sample sample) {return sample != 0;});
      lock.lock();
      (void)condition.wait_until(
        lock, std::min(deadline, std::chrono::steady_clock::now() + 10ms));
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
    }
    EXPECT_TRUE(rendered_nonzero);
    ASSERT_TRUE(feedback_count >= 1U && result_received);
    ASSERT_NE(result.result, nullptr);
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(result.result->code, VoicePipeline::Speak::Result::COMPLETED);
    lock.unlock();

    executor.remove_node(observer);
    pipeline->remove_from_executor(executor);
    (void)turn_subscription;
  }
}

TEST(VoicePipelineTest, FailsClosedWhenTheInjectedDeviceCannotOpen)
{
  RclcppContextGuard rclcpp_context;
  {
    RejectingDevice device;
    EXPECT_THROW(
      (void)std::make_unique<VoicePipeline>(
        std::make_unique<ScriptedRecognizer>(), std::make_unique<DeterministicFakeTts>(), device),
      std::runtime_error);
  }
}

}  // namespace
}  // namespace voice_nav_audio
