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

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <thread>
#include <utility>

#include "rcl_interfaces/srv/set_parameters.hpp"

#define main voice_nav_audio_scripted_voice_demo_main
#include "../src/scripted_voice_demo.cpp"  // NOLINT(build/include)
#undef main

namespace voice_nav_audio
{
namespace
{

using namespace std::chrono_literals;

class FakeSessionAdmission final : public SessionCommandAdmission
{
public:
  explicit FakeSessionAdmission(const InitialSafetyStability & stability)
  : stability_(stability)
  {
  }

  [[nodiscard]] bool session_can_accept_command() const override
  {
    return stability_.is_stable();
  }

  void begin_session_command() override {active = true;}

  [[nodiscard]] bool session_finished_safely() const override
  {
    return finished_safely;
  }

  void finish_session_command() override {active = false;}

  const InitialSafetyStability & stability_;
  bool active{false};
  bool finished_safely{false};
};

class CollectingVoiceTurnSink final : public VoiceTurnSink
{
public:
  void publish(const VoiceTurnPublication & turn) noexcept override
  {
    turns.push_back(turn);
  }

  std::vector<VoiceTurnPublication> turns{};
};

class FixedVoiceIdentityGenerator final : public VoiceIdentityGenerator
{
public:
  bool generate(std::array<std::uint8_t, 16U> & bytes) noexcept override
  {
    for (std::size_t index = 0U; index < bytes.size(); ++index) {
      bytes[index] = static_cast<std::uint8_t>(index);
    }
    return true;
  }
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

class ExecutorRunner final
{
public:
  explicit ExecutorRunner(rclcpp::executors::SingleThreadedExecutor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorRunner()
  {
    stop();
  }

  void stop()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::executors::SingleThreadedExecutor & executor_;
  std::thread thread_;
};

class JoiningThread final
{
public:
  explicit JoiningThread(std::thread thread)
  : thread_(std::move(thread))
  {
  }

  ~JoiningThread()
  {
    join();
  }

  void join()
  {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  std::thread thread_;
};

class BarrierStopMissionPort final : public StopMissionPort
{
public:
  void request(
    const StopMissionRequest & request,
    StopMissionResponseSink &) noexcept override
  {
    requests.push_back(request);
  }

  std::vector<StopMissionRequest> requests{};
};

TEST(InitialSafetyStability, RequiresTwoSecondsAndResetsOnUnsafeSample)
{
  auto now = std::chrono::steady_clock::time_point{};
  InitialSafetyStability stability([&now]() {return now;});

  EXPECT_FALSE(stability.observe(false));
  EXPECT_FALSE(stability.observe(true));

  now += 1999ms;
  EXPECT_FALSE(stability.is_stable());

  now += 1ms;
  EXPECT_TRUE(stability.is_stable());

  EXPECT_FALSE(stability.observe(false));
  EXPECT_FALSE(stability.is_stable());

  now += 2s;
  EXPECT_FALSE(stability.is_stable());
  EXPECT_FALSE(stability.observe(true));
  now += 2s;
  EXPECT_TRUE(stability.is_stable());
}

TEST(VoiceCommandGateway, RejectsUntilInitialSafetyIsStable)
{
  auto now = std::chrono::steady_clock::time_point{};
  InitialSafetyStability stability([&now]() {return now;});
  FakeSessionAdmission admission(stability);
  VoiceCommandGateway gateway(admission);

  const auto early = gateway.on_parameters({
      rclcpp::Parameter("command_text", "前进半米")});
  EXPECT_FALSE(early.successful);
  EXPECT_FALSE(gateway.take().has_value());

  EXPECT_FALSE(stability.observe(true));
  now += 2s;
  const auto accepted = gateway.on_parameters({
      rclcpp::Parameter("command_text", "前进半米")});
  ASSERT_TRUE(accepted.successful) << accepted.reason;
  const auto command = gateway.take();
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(*command, "前进半米");
}

TEST(VoiceCommandGateway, AcceptsExactStopPhrasesWhileBusyAndPublishesFormalStopTurns)
{
  auto now = std::chrono::steady_clock::time_point{};
  InitialSafetyStability stability([&now]() {return now;});
  FakeSessionAdmission admission(stability);
  VoiceCommandGateway gateway(admission);

  EXPECT_FALSE(stability.observe(true));
  now += 2s;
  const auto command_result = gateway.on_parameters({
      rclcpp::Parameter("command_text", "前进两米")});
  ASSERT_TRUE(command_result.successful) << command_result.reason;
  const auto command = gateway.take();
  ASSERT_TRUE(command.has_value());
  ASSERT_TRUE(admission.active);

  EXPECT_FALSE(gateway.on_parameters({
      rclcpp::Parameter("command_text", "右转九十度")}).successful);

  ScriptedRecognizer recognizer(ScriptedScenario::kSession);
  CollectingVoiceTurnSink sink;
  FixedVoiceIdentityGenerator identity_generator;
  SpeechInputCore input(recognizer, sink, identity_generator);
  std::uint64_t next_audio_sequence = 1U;
  recognizer.start_session_command(*command, next_audio_sequence);
  for (std::uint64_t offset = 0U; offset < 3U; ++offset) {
    input.accept_cleaned_frame(cleaned_frame(next_audio_sequence + offset));
  }
  next_audio_sequence += 3U;

  ASSERT_EQ(sink.turns.size(), 1U);
  EXPECT_EQ(sink.turns.front().kind, VoiceTurnKind::kCommand);
  const auto voice_instance_id = sink.turns.front().voice_instance_id;
  EXPECT_EQ(sink.turns.front().voice_seq, 1U);

  const std::array<const char *, 2U> stop_phrases{"小智停止", "紧急停止"};
  for (std::size_t stop_phrase_index = 0U;
    stop_phrase_index < stop_phrases.size(); ++stop_phrase_index)
  {
    const auto * const stop_phrase = stop_phrases.at(stop_phrase_index);
    SCOPED_TRACE(stop_phrase);
    const auto stop_result = gateway.on_parameters({
        rclcpp::Parameter("command_text", stop_phrase)});
    ASSERT_TRUE(stop_result.successful) << stop_result.reason;
    const auto stop_command = gateway.take();
    ASSERT_TRUE(stop_command.has_value());
    EXPECT_EQ(*stop_command, stop_phrase);

    recognizer.start_session_command(*stop_command, next_audio_sequence);
    for (std::uint64_t offset = 0U; offset < 3U; ++offset) {
      input.accept_cleaned_frame(cleaned_frame(next_audio_sequence + offset));
    }
    next_audio_sequence += 3U;

    const auto expected_turn_count = 2U + stop_phrase_index;
    ASSERT_EQ(sink.turns.size(), expected_turn_count);
    const auto & stop_turn = sink.turns.back();
    EXPECT_EQ(stop_turn.kind, VoiceTurnKind::kStop);
    EXPECT_EQ(stop_turn.text, stop_phrase);
    EXPECT_EQ(stop_turn.voice_instance_id, voice_instance_id);
    EXPECT_EQ(stop_turn.voice_seq, expected_turn_count);
    EXPECT_FALSE(gateway.on_parameters({
        rclcpp::Parameter("command_text", "右转九十度")}).successful);
  }
}

TEST(VoiceCommandGateway, ReleasesAfterStopAtSafeBarrierOnlyAfterTwoHundredMilliseconds)
{
  RclcppContextGuard rclcpp_context;
  PlaybackEvidenceRecorder recorder;
  ManualDevice device(recorder);
  auto now = std::chrono::steady_clock::time_point{};
  CompletionObserver observer(device, [&now]() {return now;});
  rclcpp::executors::SingleThreadedExecutor executor;
  observer.start(executor);
  std::thread spin_thread([&executor]() {executor.spin();});

  const auto state_publisher = observer.node()->create_publisher<
    voice_nav_interfaces::msg::MissionState>(
    "/mission/state", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  const auto command_publisher = observer.node()->create_publisher<
    geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", rclcpp::QoS(1));
  const auto odometry_publisher = observer.node()->create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(1));
  const auto turn_publisher = observer.node()->create_publisher<
    voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  const auto speak_status_publisher = observer.node()->create_publisher<
    action_msgs::msg::GoalStatusArray>(
    "/voice/speak/_action/status", rclcpp::QoS(rclcpp::KeepLast(10)));

  const auto wait_for_subscriptions = [](const auto & ... publishers) {
      for (std::size_t attempt = 0U; attempt < 10000U; ++attempt) {
        if (((publishers->get_subscription_count() > 0U) && ...)) {
          return true;
        }
        std::this_thread::yield();
      }
      return false;
    };
  ASSERT_TRUE(wait_for_subscriptions(
      state_publisher, command_publisher, odometry_publisher,
      turn_publisher, speak_status_publisher));

  voice_nav_interfaces::msg::MissionState state{};
  state.availability = voice_nav_interfaces::msg::MissionState::AVAILABLE;
  state.gate_state = voice_nav_interfaces::msg::MissionState::GATE_INHIBITED;
  state.active_step = std::numeric_limits<std::uint32_t>::max();
  geometry_msgs::msg::TwistStamped command{};
  nav_msgs::msg::Odometry odometry{};
  odometry.pose.pose.orientation.w = 1.0;
  const auto publish_safe_sample = [&]() {
      state_publisher->publish(state);
      command_publisher->publish(command);
      odometry_publisher->publish(odometry);
    };

  for (std::size_t attempt = 0U; attempt < 1000U; ++attempt) {
    publish_safe_sample();
    std::this_thread::yield();
  }
  now += 2s;
  bool safe_barrier_ready = false;
  for (std::size_t attempt = 0U; attempt < 1000U && !safe_barrier_ready; ++attempt) {
    publish_safe_sample();
    safe_barrier_ready = observer.session_can_accept_command();
    std::this_thread::yield();
  }
  ASSERT_TRUE(safe_barrier_ready);

  voice_nav_interfaces::msg::VoiceTurn turn{};
  turn.voice_instance_id = "voice-instance";
  turn.voice_seq = 1U;
  turn.session_id = "session";
  turn.turn_id = "baseline-turn";
  turn.kind = voice_nav_interfaces::msg::VoiceTurn::COMMAND;
  turn.text = "前进";
  turn.confidence = 1.0F;
  action_msgs::msg::GoalStatusArray speak_status{};
  speak_status.status_list.emplace_back();
  speak_status.status_list.front().status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
  speak_status.status_list.front().goal_info.goal_id.uuid[0] = 1U;

  const auto publish_reply = [&]() {
    turn_publisher->publish(turn);
    speak_status_publisher->publish(speak_status);
    publish_safe_sample();
  };
  publish_reply();
  ASSERT_TRUE(observer.wait_for_command_outcome());

  VoiceCommandGateway gateway(observer);
  const auto accepted = gateway.on_parameters({
      rclcpp::Parameter("command_text", "前进")});
  ASSERT_TRUE(accepted.successful) << accepted.reason;
  const auto command_to_run = gateway.take();
  ASSERT_TRUE(command_to_run.has_value());
  EXPECT_EQ(*command_to_run, "前进");
  EXPECT_FALSE(gateway.on_parameters({
      rclcpp::Parameter("command_text", "前进")}).successful);

  const auto stop_accepted = gateway.on_parameters({
      rclcpp::Parameter("command_text", "小智停止")});
  ASSERT_TRUE(stop_accepted.successful) << stop_accepted.reason;
  const auto stop_command = gateway.take();
  ASSERT_TRUE(stop_command.has_value());
  EXPECT_EQ(*stop_command, "小智停止");
  EXPECT_FALSE(gateway.on_parameters({
      rclcpp::Parameter("command_text", "右转九十度")}).successful);

  turn.voice_seq = 2U;
  turn.turn_id = "current-turn";
  turn.kind = voice_nav_interfaces::msg::VoiceTurn::STOP;
  turn.text = "小智停止";
  speak_status.status_list.front().goal_info.goal_id.uuid[0] = 2U;
  bool fresh_reply_observed = false;
  for (std::size_t attempt = 0U; attempt < 1000U && !fresh_reply_observed; ++attempt) {
    publish_reply();
    const auto summary = observer.summary();
    fresh_reply_observed = summary.turns.size() >= 2U &&
      summary.successful_speech_count == 2U;
    std::this_thread::yield();
  }
  const auto fresh_reply_summary = observer.summary();
  EXPECT_TRUE(fresh_reply_observed)
    << "turns=" << fresh_reply_summary.turns.size()
    << " successful_speeches=" << fresh_reply_summary.successful_speech_count;

  now += 199ms;
  publish_safe_sample();
  gateway.complete_if_safe();
  EXPECT_FALSE(gateway.on_parameters({
      rclcpp::Parameter("command_text", "前进")}).successful);

  now += 1ms;
  publish_safe_sample();
  gateway.complete_if_safe();
  const auto resubmitted = gateway.on_parameters({
      rclcpp::Parameter("command_text", "前进")});
  EXPECT_TRUE(resubmitted.successful) << resubmitted.reason;

  const auto summary = observer.summary();
  EXPECT_EQ(summary.unique_mission_count, 0U);
  EXPECT_EQ(summary.successful_mission_count, 0U);
  EXPECT_FALSE(summary.controller_nonzero_observed);
  EXPECT_TRUE(summary.final_gate_inhibited);
  EXPECT_TRUE(summary.final_command_is_zero);
  EXPECT_TRUE(summary.final_odometry_is_stationary);

  executor.cancel();
  spin_thread.join();
  executor.remove_node(observer.node());
}

TEST(CompletionObserver, StopProductRequiresFinalStationaryHoldAfterStopCompletion)
{
  RclcppContextGuard rclcpp_context;
  PlaybackEvidenceRecorder recorder;
  ManualDevice device(recorder);
  std::mutex now_mutex;
  auto now = std::chrono::steady_clock::time_point{};
  CompletionObserver observer(device, [&now, &now_mutex]() {
      std::lock_guard<std::mutex> lock(now_mutex);
      return now;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  observer.start(executor);
  ExecutorRunner executor_runner(executor);

  const auto state_publisher = observer.node()->create_publisher<
    voice_nav_interfaces::msg::MissionState>(
    "/mission/state", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  const auto command_publisher = observer.node()->create_publisher<
    geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", rclcpp::QoS(1));
  const auto odometry_publisher = observer.node()->create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(1));
  const auto turn_publisher = observer.node()->create_publisher<
    voice_nav_interfaces::msg::VoiceTurn>(
    "/voice/turn", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  const auto mission_status_publisher = observer.node()->create_publisher<
    action_msgs::msg::GoalStatusArray>(
    "/mission/execute/_action/status", rclcpp::QoS(rclcpp::KeepLast(10)));
  const auto speak_status_publisher = observer.node()->create_publisher<
    action_msgs::msg::GoalStatusArray>(
    "/voice/speak/_action/status", rclcpp::QoS(rclcpp::KeepLast(10)));
  const auto wait_for_subscriptions = [&]() {
      for (std::size_t attempt = 0U; attempt < 10000U; ++attempt) {
        if (state_publisher->get_subscription_count() > 0U &&
          command_publisher->get_subscription_count() > 0U &&
          odometry_publisher->get_subscription_count() > 0U &&
          turn_publisher->get_subscription_count() > 0U &&
          mission_status_publisher->get_subscription_count() > 0U &&
          speak_status_publisher->get_subscription_count() > 0U)
        {
          return true;
        }
        std::this_thread::yield();
      }
      return false;
    };
  ASSERT_TRUE(wait_for_subscriptions());

  voice_nav_interfaces::msg::MissionState state{};
  state.availability = voice_nav_interfaces::msg::MissionState::AVAILABLE;
  state.active_step = 0U;
  state.gate_state = voice_nav_interfaces::msg::MissionState::GATE_ARMED;
  geometry_msgs::msg::TwistStamped nonzero_command{};
  nonzero_command.twist.linear.x = 0.2;
  geometry_msgs::msg::TwistStamped zero_command{};
  nav_msgs::msg::Odometry moving_odometry{};
  moving_odometry.pose.pose.orientation.w = 1.0;
  moving_odometry.twist.twist.linear.x = 0.2;
  nav_msgs::msg::Odometry stationary_odometry{};
  stationary_odometry.pose.pose.orientation.w = 1.0;

  voice_nav_interfaces::msg::VoiceTurn command_turn{};
  command_turn.voice_instance_id = "voice-instance";
  command_turn.voice_seq = 1U;
  command_turn.session_id = "session";
  command_turn.turn_id = "command-turn";
  command_turn.kind = voice_nav_interfaces::msg::VoiceTurn::COMMAND;
  command_turn.text = "前进 2 米";
  voice_nav_interfaces::msg::VoiceTurn stop_turn = command_turn;
  stop_turn.voice_seq = 2U;
  stop_turn.turn_id = "stop-turn";
  stop_turn.kind = voice_nav_interfaces::msg::VoiceTurn::STOP;
  stop_turn.text = "小智停止";

  action_msgs::msg::GoalStatusArray mission_status{};
  mission_status.status_list.emplace_back();
  mission_status.status_list.front().status = action_msgs::msg::GoalStatus::STATUS_CANCELED;
  mission_status.status_list.front().goal_info.goal_id.uuid[0] = 1U;
  action_msgs::msg::GoalStatusArray speak_status{};
  speak_status.status_list.emplace_back();
  speak_status.status_list.front().status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
  speak_status.status_list.front().goal_info.goal_id.uuid[0] = 2U;

  const auto publish_safe_sample = [&]() {
      state_publisher->publish(state);
      command_publisher->publish(zero_command);
      odometry_publisher->publish(stationary_odometry);
    };
  const auto publish_command_turn = [&]() {
      turn_publisher->publish(command_turn);
    };
  const auto publish_stop_records = [&]() {
      turn_publisher->publish(stop_turn);
      mission_status_publisher->publish(mission_status);
      speak_status_publisher->publish(speak_status);
    };
  const auto wait_for_stop_samples = [&]() {
      for (std::size_t attempt = 0U; attempt < 1000U; ++attempt) {
        publish_safe_sample();
        const auto summary = observer.summary();
        if (summary.turns.size() == 2U && summary.unique_mission_count == 1U &&
          summary.terminal_non_success_mission_count == 1U &&
          summary.successful_speech_count == 1U && summary.gate_inhibited_after_motion &&
          summary.final_command_is_zero && summary.final_odometry_is_stationary &&
          summary.post_stop_odom_samples >= 4U)
        {
          return true;
        }
        std::this_thread::yield();
      }
      return false;
    };

  state.gate_state = voice_nav_interfaces::msg::MissionState::GATE_ARMED;
  state.active_step = 0U;
  bool armed_state_observed = false;
  for (std::size_t attempt = 0U; attempt < 1000U; ++attempt) {
    state_publisher->publish(state);
    armed_state_observed = observer.summary().gate_armed_observed;
    if (armed_state_observed) {
      break;
    }
    std::this_thread::yield();
  }
  ASSERT_TRUE(armed_state_observed);
  odometry_publisher->publish(moving_odometry);
  std::size_t nonzero_published = 0U;
  bool armed_motion_observed = false;
  for (std::size_t attempt = 0U; attempt < 1000U; ++attempt) {
    command_publisher->publish(nonzero_command);
    ++nonzero_published;
    for (std::size_t callback_attempt = 0U; callback_attempt < 32U; ++callback_attempt) {
      const auto summary = observer.summary();
      armed_motion_observed = summary.controller_nonzero_observed &&
        !summary.final_gate_inhibited;
      if (armed_motion_observed) {
        break;
      }
      std::this_thread::yield();
    }
    if (armed_motion_observed) {
      break;
    }
    std::this_thread::yield();
  }
  if (!armed_motion_observed) {
    const auto stalled = observer.summary();
    ADD_FAILURE() << "armed motion did not drain: gate_armed="
                  << stalled.gate_armed_observed
                  << " controller_nonzero=" << stalled.controller_nonzero_observed
                  << " callback_count=" << stalled.controller_nonzero_callback_count
                  << " published_count=" << nonzero_published
                  << " final_gate=" << stalled.final_gate_inhibited;
    return;
  }
  const auto command_callbacks_before_drain = observer.summary().command_callback_count;
  command_publisher->publish(zero_command);
  bool nonzero_queue_drained = false;
  for (std::size_t attempt = 0U; attempt < 1000U; ++attempt) {
    const auto summary = observer.summary();
    nonzero_queue_drained = !summary.final_gate_inhibited &&
      summary.command_callback_count > command_callbacks_before_drain;
    if (nonzero_queue_drained) {
      break;
    }
    std::this_thread::yield();
  }
  ASSERT_TRUE(nonzero_queue_drained);
  publish_command_turn();
  bool command_turn_observed = false;
  for (std::size_t attempt = 0U; attempt < 1000U; ++attempt) {
    command_turn_observed = observer.summary().turns.size() == 1U;
    if (command_turn_observed) {
      break;
    }
    std::this_thread::yield();
  }
  ASSERT_TRUE(command_turn_observed);
  state.gate_state = voice_nav_interfaces::msg::MissionState::GATE_INHIBITED;
  state.active_step = std::numeric_limits<std::uint32_t>::max();
  bool stop_barrier_observed = false;
  for (std::size_t attempt = 0U; attempt < 1000U; ++attempt) {
    publish_safe_sample();
    const auto summary = observer.summary();
    stop_barrier_observed = summary.gate_inhibited_after_motion &&
      summary.post_stop_odom_samples >= 4U;
    if (stop_barrier_observed) {
      break;
    }
    std::this_thread::yield();
  }
  ASSERT_TRUE(stop_barrier_observed);
  publish_stop_records();
  if (!wait_for_stop_samples()) {
    const auto stalled = observer.summary();
    ADD_FAILURE() << "STOP observer did not converge: turns=" << stalled.turns.size()
                  << " goals=" << stalled.unique_mission_count
                  << " terminal_non_success=" << stalled.terminal_non_success_mission_count
                  << " successful_speeches=" << stalled.successful_speech_count
                  << " gate_transition=" << stalled.gate_inhibited_after_motion
                  << " final_gate=" << stalled.final_gate_inhibited
                  << " command_zero=" << stalled.final_command_is_zero
                  << " odom_stationary=" << stalled.final_odometry_is_stationary
                  << " post_stop_odom=" << stalled.post_stop_odom_samples;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(now_mutex);
    now += 199ms;
  }
  EXPECT_FALSE(observer.wait_for_stop_completion_and_final_stationarity(20ms));

  {
    std::lock_guard<std::mutex> lock(now_mutex);
    now += 1ms;
  }
  const bool final_stationarity_observed =
    observer.wait_for_stop_completion_and_final_stationarity(20ms);
  if (!final_stationarity_observed) {
    const auto stalled = observer.summary();
    ADD_FAILURE() << "final stationarity did not converge: hold_ms="
                  << stalled.final_stationary_hold_ms
                  << " runtime_ready=" << stalled.final_gate_inhibited
                  << " command_zero=" << stalled.final_command_is_zero
                  << " odom_stationary=" << stalled.final_odometry_is_stationary
                  << " turns=" << stalled.turns.size()
                  << " goals=" << stalled.unique_mission_count
                  << " terminal_non_success=" << stalled.terminal_non_success_mission_count
                  << " successful_speeches=" << stalled.successful_speech_count
                  << " gate_transition=" << stalled.gate_inhibited_after_motion
                  << " post_stop_odom=" << stalled.post_stop_odom_samples
                  << " post_stop_nonzero=" << stalled.post_stop_nonzero_command_observed;
  }
  EXPECT_TRUE(final_stationarity_observed);
}

TEST(CompletionObserver, CapturesStartupZeroSampleBeforeGraphWait)
{
  RclcppContextGuard rclcpp_context;
  PlaybackEvidenceRecorder recorder;
  ManualDevice device(recorder);
  auto now = std::chrono::steady_clock::time_point{};
  CompletionObserver observer(device, [&now]() {return now;});
  rclcpp::executors::SingleThreadedExecutor executor;
  observer.start(executor);
  std::thread spin_thread([&executor]() {executor.spin();});

  const auto state_publisher = observer.node()->create_publisher<
    voice_nav_interfaces::msg::MissionState>(
    "/mission/state", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  const auto command_publisher = observer.node()->create_publisher<
    geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", rclcpp::QoS(1));
  const auto odometry_publisher = observer.node()->create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(1));

  voice_nav_interfaces::msg::MissionState state{};
  state.availability = voice_nav_interfaces::msg::MissionState::AVAILABLE;
  state.gate_state = voice_nav_interfaces::msg::MissionState::GATE_INHIBITED;
  geometry_msgs::msg::TwistStamped command{};
  nav_msgs::msg::Odometry odometry{};
  odometry.pose.pose.orientation.w = 1.0;

  for (std::size_t attempt = 0U; attempt < 1000U; ++attempt) {
    state_publisher->publish(state);
    command_publisher->publish(command);
    odometry_publisher->publish(odometry);
    std::this_thread::yield();
  }
  now += 2s;

  bool accepted = false;
  for (std::size_t attempt = 0U; attempt < 1000U && !accepted; ++attempt) {
    state_publisher->publish(state);
    command_publisher->publish(command);
    odometry_publisher->publish(odometry);
    accepted = observer.session_can_accept_command();
    std::this_thread::yield();
  }

  EXPECT_TRUE(accepted);
  executor.cancel();
  spin_thread.join();
  executor.remove_node(observer.node());
}

TEST(SessionGateway, ExposesServiceOnlyAfterObserverSafeBarrier)
{
  RclcppContextGuard rclcpp_context;
  PlaybackEvidenceRecorder recorder;
  ManualDevice device(recorder);
  auto now = std::chrono::steady_clock::time_point{};
  CompletionObserver observer(device, [&now]() {return now;});
  rclcpp::executors::SingleThreadedExecutor executor;
  observer.start(executor);
  auto probe = std::make_shared<rclcpp::Node>("session_gateway_readiness_probe");
  auto client = probe->create_client<rcl_interfaces::srv::SetParameters>(
    "/voice_nav_command_gateway/set_parameters");
  executor.add_node(probe);
  std::thread spin_thread([&executor]() {executor.spin();});

  const auto state_publisher = observer.node()->create_publisher<
    voice_nav_interfaces::msg::MissionState>(
    "/mission/state", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  const auto command_publisher = observer.node()->create_publisher<
    geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", rclcpp::QoS(1));
  const auto odometry_publisher = observer.node()->create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(1));

  voice_nav_interfaces::msg::MissionState state{};
  state.availability = voice_nav_interfaces::msg::MissionState::AVAILABLE;
  state.gate_state = voice_nav_interfaces::msg::MissionState::GATE_INHIBITED;
  state.active_step = std::numeric_limits<std::uint32_t>::max();
  geometry_msgs::msg::TwistStamped command{};
  nav_msgs::msg::Odometry odometry{};
  odometry.pose.pose.orientation.w = 1.0;
  const auto publish_safe_sample = [&]() {
      state_publisher->publish(state);
      command_publisher->publish(command);
      odometry_publisher->publish(odometry);
    };

  EXPECT_FALSE(client->wait_for_service(0s));
  for (std::size_t attempt = 0U; attempt < 1000U; ++attempt) {
    publish_safe_sample();
    std::this_thread::yield();
  }
  now += 2s;
  bool barrier_ready = false;
  for (std::size_t attempt = 0U; attempt < 1000U && !barrier_ready; ++attempt) {
    publish_safe_sample();
    barrier_ready = observer.session_can_accept_command();
    std::this_thread::yield();
  }
  ASSERT_TRUE(barrier_ready);
  ASSERT_TRUE(observer.wait_for_session_command_ready());
  EXPECT_FALSE(client->wait_for_service(0s));

  rclcpp::NodeOptions configuration_options;
  configuration_options.arguments({
    "--ros-args", "-r", "scripted_voice_demo_configuration:__node:=voice_nav_command_gateway"});
  std::shared_ptr<rclcpp::Node> configuration;
  std::unique_ptr<VoiceCommandGateway> gateway;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback;
  expose_session_gateway(
    "session", "", observer, configuration_options, executor,
    configuration, gateway, parameter_callback);

  bool service_ready = false;
  for (std::size_t attempt = 0U; attempt < 1000U && !service_ready; ++attempt) {
    service_ready = client->wait_for_service(0s);
    std::this_thread::yield();
  }
  ASSERT_TRUE(service_ready);

  state.gate_state = voice_nav_interfaces::msg::MissionState::GATE_ARMED;
  command.twist.linear.x = 0.1;
  bool observer_jittered = false;
  for (std::size_t attempt = 0U; attempt < 1000U && !observer_jittered; ++attempt) {
    publish_safe_sample();
    observer_jittered = !observer.session_can_accept_command();
    std::this_thread::yield();
  }
  ASSERT_TRUE(observer_jittered);

  const auto accepted = gateway->on_parameters({
      rclcpp::Parameter("command_text", "右转九十度")});
  ASSERT_TRUE(accepted.successful) << accepted.reason;
  const auto busy = gateway->on_parameters({
      rclcpp::Parameter("command_text", "前进")});
  EXPECT_FALSE(busy.successful);
  EXPECT_EQ(busy.reason, "command_text is busy; wait for the safe stationary barrier");

  executor.cancel();
  spin_thread.join();
  executor.remove_node(configuration);
  executor.remove_node(probe);
  executor.remove_node(observer.node());
}

TEST(
  StopPlaybackBarrier,
  ControllerNonzeroCanPrecedeSpeakButStopInjectionWaitsForRealPlaybackEvidence)
{
  RclcppContextGuard rclcpp_context;
  PlaybackEvidenceRecorder recorder;
  ManualDevice device(recorder);
  BarrierStopMissionPort stop_port;
  auto pipeline = std::make_unique<VoicePipeline>(
    std::make_unique<ScriptedRecognizer>(ScriptedScenario::kStop),
    std::make_unique<DeterministicFakeTts>(recorder, true), device, &recorder, &stop_port);
  rclcpp::executors::SingleThreadedExecutor executor;
  pipeline->add_to_executor(executor);
  CompletionObserver observer(device);
  observer.start(executor);
  auto client_node = std::make_shared<rclcpp::Node>("stop_playback_barrier_probe");
  auto client = rclcpp_action::create_client<VoicePipeline::Speak>(client_node, "/voice/speak");
  executor.add_node(client_node);
  ExecutorRunner executor_runner(executor);
  if (!client->wait_for_action_server(2s)) {
    ADD_FAILURE() << "VoicePipeline Speak action server did not become ready";
    return;
  }

  // Mirror the real controller-nonzero wait's first ManualDevice callback so
  // the adapter startup discontinuity is committed before the first Speak.
  device.consume_once();
  const bool controller_nonzero_observed = true;
  EXPECT_TRUE(controller_nonzero_observed);
  std::mutex barrier_mutex;
  std::condition_variable barrier_condition;
  bool barrier_started = false;
  bool barrier_finished = false;
  bool barrier_succeeded = false;
  JoiningThread barrier_thread(std::thread([&]() {
      {
        std::lock_guard<std::mutex> lock(barrier_mutex);
        barrier_started = true;
      }
      barrier_condition.notify_all();
      const auto result = observer.wait_for_first_speak_playback(recorder, 2s);
      {
        std::lock_guard<std::mutex> lock(barrier_mutex);
        barrier_succeeded = result;
        barrier_finished = true;
      }
      barrier_condition.notify_all();
    }));

  bool barrier_started_before_speak = false;
  {
    std::unique_lock<std::mutex> lock(barrier_mutex);
    barrier_started_before_speak = barrier_condition.wait_for(
      lock, 1s, [&]() {return barrier_started;});
  }
  EXPECT_TRUE(barrier_started_before_speak);
  if (!barrier_started_before_speak) {
    ADD_FAILURE() << "playback barrier did not start before Speak";
    return;
  }
  const auto before_speak = recorder.snapshot();
  bool barrier_finished_before_speak = false;
  {
    std::lock_guard<std::mutex> lock(barrier_mutex);
    barrier_finished_before_speak = barrier_finished;
  }
  EXPECT_FALSE(barrier_finished_before_speak);
  EXPECT_TRUE(before_speak.tts_texts.empty());
  EXPECT_EQ(before_speak.nonzero_callback_count, 0U);
  EXPECT_TRUE(before_speak.feedback_scope_ids.empty());

  VoicePipeline::Speak::Goal goal{};
  goal.source_instance_id = "voice-instance";
  goal.source_seq = 1U;
  goal.session_id = "session";
  goal.turn_id = "speak-turn";
  goal.priority = VoicePipeline::Speak::Goal::NORMAL;
  goal.text = "正在移动";
  goal.allow_barge_in = true;
  auto goal_future = client->async_send_goal(goal);
  const bool speak_goal_accepted = goal_future.wait_for(1s) == std::future_status::ready;
  EXPECT_TRUE(speak_goal_accepted);
  if (!speak_goal_accepted) {
    ADD_FAILURE() << "Speak goal was not accepted before playback barrier deadline";
    return;
  }

  bool barrier_finished_after_speak = false;
  {
    std::unique_lock<std::mutex> lock(barrier_mutex);
    barrier_finished_after_speak = barrier_condition.wait_for(
      lock, 1s, [&]() {return barrier_finished;});
  }
  EXPECT_TRUE(barrier_finished_after_speak);
  if (!barrier_finished_after_speak) {
    const auto stalled = recorder.snapshot();
    ADD_FAILURE() << "Speak playback evidence barrier did not converge: tts_started="
                  << stalled.tts_texts.size()
                  << " nonzero_callbacks=" << stalled.nonzero_callback_count
                  << " feedback_scopes=" << stalled.feedback_scope_ids.size()
                  << " completion_scopes=" << stalled.completion_scope_ids.size()
                  << " barged_in=" << stalled.barged_in_count;
    return;
  }
  {
    std::lock_guard<std::mutex> lock(barrier_mutex);
    EXPECT_TRUE(barrier_succeeded);
  }
  barrier_thread.join();

  const auto before_stop = recorder.snapshot();
  ASSERT_EQ(before_stop.tts_texts.size(), 1U);
  ASSERT_GE(before_stop.nonzero_callback_count, 1U);
  ASSERT_EQ(before_stop.feedback_scope_ids.size(), 1U);

  pipeline->accept_cleaned_frame(cleaned_frame(4U));
  pipeline->accept_cleaned_frame(cleaned_frame(5U));
  pipeline->accept_cleaned_frame(cleaned_frame(6U));

  const auto after_stop = recorder.snapshot();
  device.consume_once();
  const auto after_stop_audio = pipeline->audio_metrics();
  EXPECT_EQ(stop_port.requests.size(), 1U);
  EXPECT_GT(after_stop_audio.last_fence_generation_before, 0U);
  EXPECT_EQ(
    after_stop_audio.last_fence_generation_after,
    after_stop_audio.last_fence_generation_before + 1U);
  EXPECT_EQ(after_stop_audio.stale_pcm_after_fence, 0U);
  EXPECT_EQ(after_stop.barged_in_count, 1U);
  EXPECT_EQ(after_stop.feedback_scope_ids.size(), 1U);

  executor_runner.stop();
  executor.remove_node(client_node);
  executor.remove_node(observer.node());
  pipeline->remove_from_executor(executor);
  pipeline.reset();
}

}  // namespace
}  // namespace voice_nav_audio
