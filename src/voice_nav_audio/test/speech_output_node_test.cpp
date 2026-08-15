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

#include <array>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "speech_output_node.hpp"

namespace voice_nav_audio
{
namespace
{

using namespace std::chrono_literals;

class ImmediateFakeTts final : public TtsAdapter
{
public:
  void start(const TtsRequest & request, TtsSink & sink) noexcept override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      started_ = true;
      text_ = request.text;
    }
    started_condition_.notify_all();
    std::array<Sample, 147U> pcm{};
    pcm.fill(1000);
    (void)sink.on_pcm(request.scope_id, 22050U, 1U, pcm.data(), pcm.size());
    sink.on_complete(request.scope_id);
  }

  void cancel(const std::uint64_t) noexcept override
  {
  }

  [[nodiscard]] bool wait_started()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return started_condition_.wait_for(lock, 2s, [this]() {return started_;});
  }

  [[nodiscard]] std::string text() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return text_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable started_condition_;
  bool started_{false};
  std::string text_{};
};

class CancellableFakeTts final : public TtsAdapter
{
public:
  void start(const TtsRequest &, TtsSink &) noexcept override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      started_ = true;
    }
    started_condition_.notify_all();
  }

  void cancel(const std::uint64_t) noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++cancel_count_;
  }

  [[nodiscard]] bool wait_started()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return started_condition_.wait_for(lock, 2s, [this]() {return started_;});
  }

  [[nodiscard]] std::size_t cancel_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cancel_count_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable started_condition_;
  bool started_{false};
  std::size_t cancel_count_{0U};
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

TEST(SpeechOutputNodeTest, HeadlessActionUsesOnlySpeakAndReportsActualPlayedPcm)
{
  rclcpp::init(0, nullptr);
  AudioEngine engine;
  auto tts = std::make_unique<ImmediateFakeTts>();
  auto * const fake = tts.get();
  auto output = std::make_shared<SpeechOutputNode>(engine, std::move(tts));
  auto client_node = std::make_shared<rclcpp::Node>("speech_output_action_client");
  auto client = rclcpp_action::create_client<SpeechOutputNode::Speak>(client_node, "/voice/speak");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(output);
  executor.add_node(client_node);
  ExecutorRunner runner(executor);

  ASSERT_TRUE(client->wait_for_action_server(2s));
  bool has_speak_action_topic = false;
  for (const auto & topic : output->get_topic_names_and_types()) {
    if (topic.first.rfind("/voice/", 0U) == 0U) {
      EXPECT_EQ(topic.first.rfind("/voice/speak/_action/", 0U), 0U);
      has_speak_action_topic = true;
    }
  }
  EXPECT_TRUE(has_speak_action_topic);

  SpeechOutputNode::Speak::Goal goal{};
  goal.source_instance_id = "voice-instance";
  goal.source_seq = 7U;
  goal.session_id = "session";
  goal.turn_id = "turn";
  goal.priority = SpeechOutputNode::Speak::Goal::NORMAL;
  goal.text = "你好，世界";
  goal.allow_barge_in = true;

  std::mutex result_mutex;
  std::condition_variable result_condition;
  bool have_result = false;
  rclcpp_action::ClientGoalHandle<SpeechOutputNode::Speak>::WrappedResult result{};
  rclcpp_action::Client<SpeechOutputNode::Speak>::SendGoalOptions options{};
  options.feedback_callback = [](
      rclcpp_action::ClientGoalHandle<SpeechOutputNode::Speak>::SharedPtr,
      const std::shared_ptr<const SpeechOutputNode::Speak::Feedback> feedback) {
      EXPECT_EQ(feedback->played.sec, 0);
      EXPECT_EQ(feedback->played.nanosec, 6666666U);
    };
  options.result_callback = [&result_mutex, &result_condition, &have_result, &result](
      const rclcpp_action::ClientGoalHandle<SpeechOutputNode::Speak>::WrappedResult & received) {
      std::lock_guard<std::mutex> lock(result_mutex);
      result = received;
      have_result = true;
      result_condition.notify_all();
    };
  client->async_send_goal(goal, options);

  ASSERT_TRUE(fake->wait_started());
  EXPECT_EQ(fake->text(), goal.text);
  {
    std::lock_guard<std::mutex> lock(result_mutex);
    EXPECT_FALSE(have_result);
  }

  std::array<Sample, AudioEngine::kFrameSamples> device_output{};
  engine.process_callback(nullptr, device_output.data(), device_output.size(), CallbackStatus{});
  output->pump();

  std::unique_lock<std::mutex> result_lock(result_mutex);
  ASSERT_TRUE(result_condition.wait_for(result_lock, 2s, [&have_result]() {return have_result;}));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_EQ(result.result->code, SpeechOutputNode::Speak::Result::COMPLETED);

  executor.remove_node(output);
  executor.remove_node(client_node);
  rclcpp::shutdown();
}

TEST(SpeechOutputNodeTest, CancelAcceptsBeforePublishingOneCanceledResult)
{
  rclcpp::init(0, nullptr);
  AudioEngine engine;
  auto tts = std::make_unique<CancellableFakeTts>();
  auto * const fake = tts.get();
  auto output = std::make_shared<SpeechOutputNode>(engine, std::move(tts));
  auto client_node = std::make_shared<rclcpp::Node>("speech_output_cancel_client");
  auto client = rclcpp_action::create_client<SpeechOutputNode::Speak>(client_node, "/voice/speak");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(output);
  executor.add_node(client_node);
  ExecutorRunner runner(executor);

  ASSERT_TRUE(client->wait_for_action_server(2s));
  SpeechOutputNode::Speak::Goal goal{};
  goal.source_instance_id = "voice-instance";
  goal.source_seq = 7U;
  goal.session_id = "session";
  goal.turn_id = "turn";
  goal.priority = SpeechOutputNode::Speak::Goal::NORMAL;
  goal.text = "请取消";

  std::mutex result_mutex;
  std::condition_variable result_condition;
  bool have_goal = false;
  bool have_result = false;
  std::size_t result_count = 0U;
  rclcpp_action::ClientGoalHandle<SpeechOutputNode::Speak>::SharedPtr accepted_goal{};
  rclcpp_action::ClientGoalHandle<SpeechOutputNode::Speak>::WrappedResult result{};
  rclcpp_action::Client<SpeechOutputNode::Speak>::SendGoalOptions options{};
  options.goal_response_callback = [&result_mutex, &result_condition, &have_goal, &accepted_goal](
      rclcpp_action::ClientGoalHandle<SpeechOutputNode::Speak>::SharedPtr goal_handle) {
      std::lock_guard<std::mutex> lock(result_mutex);
      accepted_goal = std::move(goal_handle);
      have_goal = true;
      result_condition.notify_all();
    };
  options.result_callback = [&result_mutex, &result_condition, &have_result, &result_count, &result](
      const rclcpp_action::ClientGoalHandle<SpeechOutputNode::Speak>::WrappedResult & received) {
      std::lock_guard<std::mutex> lock(result_mutex);
      result = received;
      ++result_count;
      have_result = true;
      result_condition.notify_all();
    };
  client->async_send_goal(goal, options);

  {
    std::unique_lock<std::mutex> lock(result_mutex);
    ASSERT_TRUE(result_condition.wait_for(lock, 2s, [&have_goal]() {return have_goal;}));
    ASSERT_NE(accepted_goal, nullptr);
  }
  ASSERT_TRUE(fake->wait_started());

  const auto cancel_future = client->async_cancel_goal(accepted_goal);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  const auto cancel_response = cancel_future.get();
  ASSERT_EQ(cancel_response->goals_canceling.size(), 1U);

  std::unique_lock<std::mutex> lock(result_mutex);
  ASSERT_TRUE(result_condition.wait_for(lock, 2s, [&have_result]() {return have_result;}));
  EXPECT_EQ(result_count, 1U);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_EQ(result.result->code, SpeechOutputNode::Speak::Result::CANCELED);
  EXPECT_EQ(fake->cancel_count(), 1U);

  executor.remove_node(output);
  executor.remove_node(client_node);
  rclcpp::shutdown();
}

}  // namespace
}  // namespace voice_nav_audio
