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

TEST(VoiceCommandGateway, ReleasesAfterFreshClarificationReplyAtSafeBarrier)
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
  bool baseline_reply_observed = false;
  for (std::size_t attempt = 0U; attempt < 1000U && !baseline_reply_observed; ++attempt) {
    publish_reply();
    const auto summary = observer.summary();
    baseline_reply_observed = summary.turns.size() == 1U &&
      summary.successful_speech_count == 1U;
    std::this_thread::yield();
  }
  ASSERT_TRUE(baseline_reply_observed);

  VoiceCommandGateway gateway(observer);
  const auto accepted = gateway.on_parameters({
      rclcpp::Parameter("command_text", "前进")});
  ASSERT_TRUE(accepted.successful) << accepted.reason;
  const auto command_to_run = gateway.take();
  ASSERT_TRUE(command_to_run.has_value());
  EXPECT_EQ(*command_to_run, "前进");
  EXPECT_FALSE(gateway.on_parameters({
      rclcpp::Parameter("command_text", "前进")}).successful);

  turn.voice_seq = 2U;
  turn.turn_id = "current-turn";
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

}  // namespace
}  // namespace voice_nav_audio
