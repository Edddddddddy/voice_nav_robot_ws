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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "sensevoice_provider.hpp"
#include "speech_input_node.hpp"
#include "voice_nav_interfaces/msg/voice_turn.hpp"

namespace voice_nav_audio
{
namespace
{

class NodeHappyPathRecognizer final : public SpeechRecognizerAdapter
{
public:
  void process_frame(
    const CleanedAudioFrame & frame,
    SpeechEventSink & sink) noexcept override
  {
    if (frame.audio_seq == 1U) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    } else if (frame.audio_seq == 2U) {
      sink.on_speech_event(SpeechRecognitionEvent::activity(frame, active_scope_));
    } else if (frame.audio_seq == 3U) {
      sink.on_speech_event(
        SpeechRecognitionEvent::endpoint_final(frame, active_scope_, "前进一米", 0.75F));
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

  void finish_input() noexcept override
  {
    ++finish_input_calls;
  }

  std::size_t finish_input_calls{0U};

private:
  TurnScopeIdentity active_scope_{};
};

class NodeBarrierVad final : public SileroVadAdapter
{
public:
  SileroVadResult process(const CleanedAudioFrame &) noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++calls_;
    return calls_ == 1U ?
           SileroVadResult{SileroVadDecision::kSpeech, 0U} :
           SileroVadResult{SileroVadDecision::kEndpoint, 2U * CleanedAudioFrame::kSamples};
  }

  SileroVadFlushResult finish_input() noexcept override
  {
    return SileroVadFlushResult{
      SileroVadFlushStatus::kUnique, 2U * CleanedAudioFrame::kSamples};
  }

  void reset() noexcept override
  {
  }

private:
  std::mutex mutex_;
  std::size_t calls_{0U};
};

class NodeBarrierAsr final : public SenseVoiceAsrAdapter
{
public:
  bool infer(
    const Sample *, const std::size_t, std::string & labeled_text) noexcept override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return released_;});
    labeled_text = "开放时间早上9点至下午5点。";
    return true;
  }

  void shutdown() noexcept override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_requested_ = true;
    }
    condition_.notify_all();
  }

  bool wait_until_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::seconds(2), [this]() {return entered_;});
  }

  bool wait_until_shutdown()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::seconds(2), [this]() {
               return shutdown_requested_;
      });
  }

  void release() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      released_ = true;
    }
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_{false};
  bool released_{false};
  bool shutdown_requested_{false};
};

CleanedAudioFrame frame(const std::uint64_t sequence, const std::size_t valid_samples = 160U)
{
  CleanedAudioFrame input{};
  input.audio_generation = 7U;
  input.audio_seq = sequence;
  input.valid_samples = valid_samples;
  input.samples.fill(100);
  return input;
}

TEST(SpeechInputNodeTest, HeadlessCompositionPublishesOnlyOneFinalVoiceTurn)
{
  const auto configured_qos = voice_turn_qos().get_rmw_qos_profile();
  EXPECT_EQ(configured_qos.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(configured_qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  EXPECT_EQ(configured_qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(configured_qos.depth, 1U);

  rclcpp::init(0, nullptr);
  auto recognizer = std::make_unique<NodeHappyPathRecognizer>();
  auto * const recognizer_probe = recognizer.get();
  auto speech = std::make_shared<SpeechInputNode>(std::move(recognizer));
  auto observer = std::make_shared<rclcpp::Node>("speech_input_observer");
  std::mutex mutex;
  std::condition_variable received;
  std::size_t publication_count = 0U;
  voice_nav_interfaces::msg::VoiceTurn latest{};
  auto subscription = observer->create_subscription<voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
    [&mutex, &received, &publication_count, &latest](
      const voice_nav_interfaces::msg::VoiceTurn::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex);
      latest = *message;
      ++publication_count;
      received.notify_all();
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(speech);
  executor.add_node(observer);
  std::thread spin_thread([&executor]() {executor.spin();});

  try {
    for (std::size_t attempt =
      0U; attempt < 100U && observer->count_publishers("/voice/turn") == 0U;
      ++attempt)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(observer->count_publishers("/voice/turn"), 1U);
    const auto publishers = observer->get_publishers_info_by_topic("/voice/turn");
    ASSERT_EQ(publishers.size(), 1U);
    const auto qos = publishers.front().qos_profile();
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
    const auto topics = speech->get_topic_names_and_types();
    for (const auto * forbidden : {
          "/voice/audio", "/voice/frame", "/voice/partial", "/voice/kws", "/voice/vad",
          "/voice/scripted_input"})
    {
      EXPECT_EQ(topics.count(forbidden), 0U);
    }

    speech->accept_cleaned_frame(frame(1U));
    speech->accept_cleaned_frame(frame(2U));
    speech->accept_cleaned_frame(frame(3U, 32U));
    speech->finish_input();
    EXPECT_EQ(recognizer_probe->finish_input_calls, 1U);

    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(received.wait_for(lock, std::chrono::seconds(2), [&publication_count]() {
        return publication_count == 1U;
      }));
    EXPECT_EQ(latest.kind, voice_nav_interfaces::msg::VoiceTurn::COMMAND);
    EXPECT_EQ(latest.text, "前进一米");
    EXPECT_FLOAT_EQ(latest.confidence, 0.75F);
    EXPECT_FALSE(latest.voice_instance_id.empty());
    EXPECT_EQ(latest.voice_seq, 1U);
    EXPECT_FALSE(latest.session_id.empty());
    EXPECT_FALSE(latest.turn_id.empty());
    EXPECT_FALSE(latest.during_playback);
    lock.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::lock_guard<std::mutex> count_lock(mutex);
    EXPECT_EQ(publication_count, 1U);
  } catch (...) {
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    throw;
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  (void)subscription;
}

TEST(SpeechInputNodeTest, DestructionWaitsForRecognizerBeforeAsrShutdownAndSuppressesLateFinal)
{
  rclcpp::init(0, nullptr);
  auto vad = std::make_unique<NodeBarrierVad>();
  auto asr = std::make_unique<NodeBarrierAsr>();
  auto * const asr_probe = asr.get();
  auto provider = std::make_unique<SenseVoiceProvider>(std::move(vad), std::move(asr));
  auto speech = std::make_unique<SpeechInputNode>(std::move(provider));
  auto observer = std::make_shared<rclcpp::Node>("speech_input_teardown_observer");
  std::mutex count_mutex;
  std::size_t publication_count = 0U;
  auto subscription = observer->create_subscription<voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", voice_turn_qos(),
    [&count_mutex, &publication_count](
      const voice_nav_interfaces::msg::VoiceTurn::SharedPtr) {
      std::lock_guard<std::mutex> lock(count_mutex);
      ++publication_count;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  std::thread spin_thread([&executor]() {executor.spin();});

  speech->accept_cleaned_frame(frame(1U));
  speech->accept_cleaned_frame(frame(2U));
  speech->finish_input();
  ASSERT_TRUE(asr_probe->wait_until_entered());

  std::atomic<bool> destroyer_started{false};
  std::thread destroyer([&speech, &destroyer_started]() {
      destroyer_started.store(true, std::memory_order_release);
      speech.reset();
    });
  while (!destroyer_started.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  // The destructor must first revoke provider admission; keep the ASR
  // blocked long enough for that stop-input phase to acquire the provider
  // queue before releasing the in-flight VAD/ASR sequence.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  asr_probe->release();
  EXPECT_TRUE(asr_probe->wait_until_shutdown());
  destroyer.join();

  {
    std::lock_guard<std::mutex> lock(count_mutex);
    EXPECT_EQ(publication_count, 0U);
  }
  executor.cancel();
  spin_thread.join();
  subscription.reset();
  observer.reset();
  rclcpp::shutdown();
}

}  // namespace
}  // namespace voice_nav_audio
