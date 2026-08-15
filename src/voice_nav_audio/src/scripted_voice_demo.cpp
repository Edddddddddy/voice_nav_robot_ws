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

// Simulation-only two-turn demo.  It owns no motion authority or endpoint
// beyond the existing VoicePipeline public seams.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "voice_pipeline.hpp"
#include "voice_nav_interfaces/msg/mission_state.hpp"
#include "voice_nav_interfaces/msg/voice_turn.hpp"

namespace voice_nav_audio
{
namespace
{

using namespace std::chrono_literals;

enum class ScriptedScenario
{
  kMove,
  kStop,
  kRoute,
};

std::optional<ScriptedScenario> scripted_scenario(const std::string & value)
{
  if (value == "move") {
    return ScriptedScenario::kMove;
  }
  if (value == "stop") {
    return ScriptedScenario::kStop;
  }
  if (value == "route") {
    return ScriptedScenario::kRoute;
  }
  return std::nullopt;
}

bool load_optional_exact_head(std::optional<std::string> & head)
{
  const char * const value = std::getenv("VOICE_NAV_EXACT_HEAD");
  if (value == nullptr) {
    head.reset();
    return true;
  }
  const std::string candidate(value);
  const auto valid_character = [](const char character) {
      return (character >= '0' && character <= '9') ||
             (character >= 'a' && character <= 'f');
    };
  if (candidate.size() != 40U || !std::all_of(
      candidate.cbegin(), candidate.cend(), valid_character))
  {
    return false;
  }
  head = candidate;
  return true;
}

class ScriptedRecognizer final : public SpeechRecognizerAdapter
{
public:
  explicit ScriptedRecognizer(const ScriptedScenario scenario)
  : scenario_(scenario)
  {
  }

  void process_frame(
    const CleanedAudioFrame & frame,
    SpeechEventSink & sink) noexcept override
  {
    if (frame.audio_seq == 1U || frame.audio_seq == 4U) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    } else if (frame.audio_seq == 2U || frame.audio_seq == 5U) {
      sink.on_speech_event(SpeechRecognitionEvent::activity(frame, active_scope_));
    } else if (frame.audio_seq == 3U) {
      sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
        frame, active_scope_, scenario_ == ScriptedScenario::kMove ? "绕到大厅" :
        scenario_ == ScriptedScenario::kRoute ? "前进半米然后左转九十度" : "前进 2 米",
        1.0F));
    } else if (frame.audio_seq == 6U) {
      sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
        frame, active_scope_, scenario_ == ScriptedScenario::kMove ? "半米" : "停止", 1.0F,
        scenario_ == ScriptedScenario::kMove ? VoiceTurnKind::kCommand : VoiceTurnKind::kStop));
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

private:
  ScriptedScenario scenario_;
  TurnScopeIdentity active_scope_{};
};

CleanedAudioFrame cleaned_frame(const std::uint64_t sequence)
{
  CleanedAudioFrame frame{};
  frame.audio_generation = 1U;
  frame.audio_seq = sequence;
  frame.samples.fill(0);
  return frame;
}

class PlaybackEvidenceRecorder final : public SpeechOutputTraceSink
{
public:
  struct Snapshot
  {
    std::vector<std::string> tts_texts{};
    std::size_t nonzero_callback_count{0U};
    std::set<std::uint64_t> feedback_scope_ids{};
    std::vector<std::uint64_t> completion_scope_ids{};
    std::vector<SpeechResultCode> completion_codes{};
  };

  void record_tts_text(const std::string & text)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.tts_texts.push_back(text);
  }

  void record_nonzero_callback() noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++snapshot_.nonzero_callback_count;
  }

  void on_played(const std::uint64_t scope_id, const std::uint64_t samples) noexcept override
  {
    if (scope_id == 0U || samples == 0U) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.feedback_scope_ids.insert(scope_id);
  }

  void on_result(const SpeechResult & result) noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.completion_scope_ids.push_back(result.scope_id);
    snapshot_.completion_codes.push_back(result.code);
  }

  [[nodiscard]] Snapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

private:
  mutable std::mutex mutex_;
  Snapshot snapshot_{};
};

class DeterministicFakeTts final : public TtsAdapter
{
public:
  explicit DeterministicFakeTts(PlaybackEvidenceRecorder & recorder)
  : recorder_(recorder)
  {
  }

  void start(const TtsRequest & request, TtsSink & sink) noexcept override
  {
    recorder_.record_tts_text(request.text);
    std::array<Sample, 147U> pcm{};
    pcm.fill(1000);
    (void)sink.on_pcm(request.scope_id, 22050U, 1U, pcm.data(), pcm.size());
    sink.on_complete(request.scope_id);
  }

  void cancel(std::uint64_t) noexcept override
  {
  }

private:
  PlaybackEvidenceRecorder & recorder_;
};

class ManualDevice final : public FullDuplexAudioDevice
{
public:
  explicit ManualDevice(PlaybackEvidenceRecorder & recorder)
  : recorder_(recorder)
  {
  }

  bool open(
    const FullDuplexStreamSpec spec, const DeviceCallback callback,
    void * context) noexcept override
  {
    if (spec.sample_rate != AudioEngine::kSampleRate || spec.channels != AudioEngine::kChannels ||
      spec.frames_per_buffer != AudioEngine::kFrameSamples || callback == nullptr ||
      context == nullptr)
    {
      return false;
    }
    callback_ = callback;
    context_ = context;
    return true;
  }

  void close() noexcept override
  {
    callback_ = nullptr;
    context_ = nullptr;
  }

  void consume_once() noexcept
  {
    if (callback_ == nullptr || context_ == nullptr) {
      return;
    }
    std::array<Sample, AudioEngine::kFrameSamples> capture{};
    std::array<Sample, AudioEngine::kFrameSamples> output{};
    callback_(context_, capture.data(), output.data(), output.size(), CallbackStatus{});
    // The scripted recognizer receives cleaned frames directly. Drain the
    // manual device's unused raw side so the real callback can continue to
    // render playback without an artificial AudioEngine generation fence.
    auto * const engine = static_cast<AudioEngine *>(context_);
    AudioFrame ignored{};
    while (engine->try_pop_reference(ignored)) {
    }
    while (engine->try_pop_capture(ignored)) {
    }
    if (std::any_of(
        output.cbegin(), output.cend(), [](const Sample sample) {return sample != 0;}))
    {
      recorder_.record_nonzero_callback();
    }
  }

private:
  PlaybackEvidenceRecorder & recorder_;
  DeviceCallback callback_{nullptr};
  void * context_{nullptr};
};

bool has_endpoint(
  const std::vector<rclcpp::TopicEndpointInfo> & endpoints,
  const std::string & node_name)
{
  return std::any_of(
    endpoints.cbegin(), endpoints.cend(), [&node_name](const auto & endpoint) {
      return endpoint.node_name() == node_name && endpoint.node_namespace() == "/";
    });
}

bool graph_is_ready(rclcpp::Node & node)
{
  return has_endpoint(
    node.get_subscriptions_info_by_topic("/voice/turn"), "agent_node") &&
         has_endpoint(
    node.get_subscriptions_info_by_topic("/mission/execute/_action/status"),
    "agent_node") &&
         has_endpoint(
    node.get_publishers_info_by_topic("/mission/state"), "mission_runtime_node") &&
         has_endpoint(
    node.get_publishers_info_by_topic("/voice/speak/_action/status"),
    "voice_speech_output");
}

bool is_zero(const geometry_msgs::msg::TwistStamped & command)
{
  const auto & twist = command.twist;
  return std::abs(twist.linear.x) <= 1.0e-6 &&
         std::abs(twist.linear.y) <= 1.0e-6 &&
         std::abs(twist.linear.z) <= 1.0e-6 &&
         std::abs(twist.angular.x) <= 1.0e-6 &&
         std::abs(twist.angular.y) <= 1.0e-6 &&
         std::abs(twist.angular.z) <= 1.0e-6;
}

bool is_stationary(const nav_msgs::msg::Odometry & odometry)
{
  const auto & twist = odometry.twist.twist;
  return std::abs(twist.linear.x) <= 0.01 &&
         std::abs(twist.angular.z) <= 0.02;
}

double yaw_from_odometry(const nav_msgs::msg::Odometry & odometry)
{
  const auto & orientation = odometry.pose.pose.orientation;
  return std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
}

double wrapped_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool wait_for_graph(rclcpp::Node & node)
{
  const auto graph_event = node.get_graph_event();
  const auto deadline = std::chrono::steady_clock::now() + 30s;
  while (!graph_is_ready(node) && std::chrono::steady_clock::now() < deadline) {
    node.wait_for_graph_change(
      graph_event,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline - std::chrono::steady_clock::now()));
    graph_event->check_and_clear();
  }
  return graph_is_ready(node);
}

class CompletionObserver final
{
public:
  struct DemoSummary
  {
    std::vector<voice_nav_interfaces::msg::VoiceTurn> turns{};
    std::size_t unique_mission_count{0U};
    std::size_t successful_mission_count{0U};
    std::size_t terminal_non_success_mission_count{0U};
    std::size_t successful_speech_count{0U};
    double displacement_m{0.0};
    double yaw_delta_rad{0.0};
    std::vector<std::uint32_t> active_step_sequence{};
    std::set<std::uint32_t> armed_nonzero_controller_steps{};
    bool controller_nonzero_observed{false};
    bool post_stop_nonzero_command_observed{false};
    bool final_gate_inhibited{false};
    bool final_zero_stationary{false};
    bool gate_inhibited_after_motion{false};
    bool final_command_is_zero{false};
    bool final_odometry_is_stationary{false};
    std::int64_t final_stationary_hold_ms{0};
    std::size_t post_stop_odom_samples{0U};
  };

  explicit CompletionObserver(ManualDevice & device)
  : device_(device), node_(std::make_shared<rclcpp::Node>("scripted_voice_motion_driver_observer"))
  {
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_subscription_ = node_->create_subscription<voice_nav_interfaces::msg::MissionState>(
      "/mission/state", state_qos,
      [this](const voice_nav_interfaces::msg::MissionState::SharedPtr state) {
        std::lock_guard<std::mutex> lock(mutex_);
        runtime_ready_ = state->availability ==
        voice_nav_interfaces::msg::MissionState::AVAILABLE &&
        state->gate_state == voice_nav_interfaces::msg::MissionState::GATE_INHIBITED;
        if (state->gate_state == voice_nav_interfaces::msg::MissionState::GATE_ARMED) {
          gate_armed_observed_ = true;
        }
        latest_gate_armed_ = state->gate_state ==
          voice_nav_interfaces::msg::MissionState::GATE_ARMED;
        latest_active_step_ = state->active_step;
        if (state->active_step <= 1U &&
          (active_step_sequence_.empty() || active_step_sequence_.back() != state->active_step))
        {
          active_step_sequence_.push_back(state->active_step);
        }
        if (gate_armed_observed_ &&
          state->gate_state == voice_nav_interfaces::msg::MissionState::GATE_INHIBITED)
        {
          gate_inhibited_after_motion_ = true;
        }
        final_gate_inhibited_ = state->gate_state ==
        voice_nav_interfaces::msg::MissionState::GATE_INHIBITED;
        condition_.notify_all();
      });
    clock_subscription_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::SensorDataQoS(),
      [this](const rosgraph_msgs::msg::Clock::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        clock_received_ = true;
        condition_.notify_all();
      });
    voice_turn_subscription_ = node_->create_subscription<voice_nav_interfaces::msg::VoiceTurn>(
      "/voice/turn", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const voice_nav_interfaces::msg::VoiceTurn::SharedPtr turn) {
        std::lock_guard<std::mutex> lock(mutex_);
        turns_.push_back(*turn);
        condition_.notify_all();
      });
    mission_status_subscription_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/mission/execute/_action/status", rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr status_array) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & status : status_array->status_list) {
          mission_goal_ids_.emplace(
            reinterpret_cast<const char *>(status.goal_info.goal_id.uuid.data()),
            status.goal_info.goal_id.uuid.size());
          if (first_turn_idle_window_) {
            ++first_turn_mission_statuses_;
          }
          if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
            successful_missions_.emplace(
              reinterpret_cast<const char *>(status.goal_info.goal_id.uuid.data()),
              status.goal_info.goal_id.uuid.size());
          }
          if (status.status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
            status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED)
          {
            terminal_non_success_missions_.emplace(
              reinterpret_cast<const char *>(status.goal_info.goal_id.uuid.data()),
              status.goal_info.goal_id.uuid.size());
          }
        }
        condition_.notify_all();
      });
    command_subscription_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/diff_drive_controller/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr command) {
        std::lock_guard<std::mutex> lock(mutex_);
        final_command_is_zero_ = is_zero(*command);
        controller_nonzero_observed_ = controller_nonzero_observed_ || !is_zero(*command);
        if (!is_zero(*command) && latest_gate_armed_ && latest_active_step_ <= 1U) {
          armed_nonzero_controller_steps_.insert(latest_active_step_);
        }
        if (gate_inhibited_after_motion_) {
          post_stop_nonzero_command_observed_ =
            post_stop_nonzero_command_observed_ || !is_zero(*command);
          if (!is_zero(*command)) {
            final_stationary_started_at_.reset();
          }
        }
        if (first_turn_idle_window_) {
          ++first_turn_command_samples_;
          first_turn_nonzero_command_ = first_turn_nonzero_command_ || !is_zero(*command);
        }
        condition_.notify_all();
      });
    odometry_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const nav_msgs::msg::Odometry::SharedPtr odometry) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto yaw = yaw_from_odometry(*odometry);
        if (!have_initial_odometry_) {
          initial_odometry_ = *odometry;
          initial_yaw_ = yaw;
          have_initial_odometry_ = true;
        }
        if (have_previous_yaw_) {
          yaw_delta_rad_ += wrapped_angle(yaw - previous_yaw_);
        } else {
          previous_yaw_ = yaw;
          have_previous_yaw_ = true;
        }
        previous_yaw_ = yaw;
        latest_odometry_ = *odometry;
        have_latest_odometry_ = true;
        final_odometry_is_stationary_ = is_stationary(*odometry);
        if (gate_inhibited_after_motion_) {
          ++post_stop_odom_samples_;
          if (final_odometry_is_stationary_) {
            if (!final_stationary_started_at_.has_value()) {
              final_stationary_started_at_ = std::chrono::steady_clock::now();
            }
          } else {
            final_stationary_started_at_.reset();
          }
        }
        if (first_turn_idle_window_) {
          ++first_turn_odom_samples_;
          first_turn_moving_ = first_turn_moving_ || !is_stationary(*odometry);
        }
        condition_.notify_all();
      });
    speak_status_subscription_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/voice/speak/_action/status", rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr status_array) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & status : status_array->status_list) {
          if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
            successful_speeches_.emplace(
              reinterpret_cast<const char *>(status.goal_info.goal_id.uuid.data()),
              status.goal_info.goal_id.uuid.size());
          }
        }
        condition_.notify_all();
      });
  }

  [[nodiscard]] const rclcpp::Node::SharedPtr & node() const noexcept {return node_;}

  [[nodiscard]] bool wait_for_runtime_ready()
  {
    return wait_for([](const auto & self) {
               return self.clock_received_ && self.runtime_ready_;
    }, 120s);
  }
  [[nodiscard]] bool wait_for_first_clarification()
  {
    return wait_for([](const auto & self) {
               return self.successful_missions_.empty() &&
                      self.successful_speeches_.size() >= 1U;
    });
  }
  void begin_first_turn_idle_window()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    first_turn_idle_window_ = true;
    first_turn_mission_statuses_ = 0U;
    first_turn_command_samples_ = 0U;
    first_turn_odom_samples_ = 0U;
    first_turn_nonzero_command_ = false;
    first_turn_moving_ = false;
  }
  [[nodiscard]] bool wait_for_first_turn_idle_samples()
  {
    return wait_for([](const auto & self) {
               return self.successful_missions_.empty() &&
                      self.first_turn_mission_statuses_ == 0U &&
                      self.first_turn_command_samples_ >= 4U &&
                      self.first_turn_odom_samples_ >= 4U &&
                      !self.first_turn_nonzero_command_ &&
                      !self.first_turn_moving_;
    });
  }
  [[nodiscard]] bool wait_for_mission_completion()
  {
    return wait_for([](const auto & self) {
               return self.successful_missions_.size() == 1U &&
                       self.successful_speeches_.size() >= 2U && self.turns_.size() == 2U;
    });
  }
  [[nodiscard]] bool wait_for_route_completion()
  {
    return wait_for([](const auto & self) {
               return self.successful_missions_.size() == 1U &&
                      self.successful_speeches_.size() == 1U && self.turns_.size() == 1U &&
                      self.active_step_sequence_ == std::vector<std::uint32_t>{0U, 1U} &&
                      self.armed_nonzero_controller_steps_ == std::set<std::uint32_t>{0U, 1U};
    });
  }
  [[nodiscard]] bool wait_for_controller_nonzero()
  {
    return wait_for([](const auto & self) {
               return self.mission_goal_ids_.size() == 1U && self.controller_nonzero_observed_;
    });
  }
  [[nodiscard]] bool wait_for_stop_completion()
  {
    return wait_for([](const auto & self) {
               return self.turns_.size() == 2U && self.turns_.front().kind ==
               voice_nav_interfaces::msg::VoiceTurn::COMMAND && self.turns_.back().kind ==
               voice_nav_interfaces::msg::VoiceTurn::STOP &&
               self.mission_goal_ids_.size() == 1U &&
               self.terminal_non_success_missions_.size() == 1U &&
               self.successful_speeches_.size() == 1U && self.gate_inhibited_after_motion_ &&
               self.final_command_is_zero_ && self.final_odometry_is_stationary_ &&
               self.post_stop_odom_samples_ >= 4U &&
               !self.post_stop_nonzero_command_observed_;
    });
  }
  [[nodiscard]] bool wait_for_final_zero_and_stationarity()
  {
    return wait_for([](const auto & self) {
               return self.runtime_ready_ && self.final_command_is_zero_ &&
                       self.final_odometry_is_stationary_ &&
                       self.final_stationary_hold_ms() >= 200;
    });
  }
  [[nodiscard]] std::size_t successful_speech_count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return successful_speeches_.size();
  }
  [[nodiscard]] DemoSummary summary()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    DemoSummary result{};
    result.turns = turns_;
    result.unique_mission_count = mission_goal_ids_.size();
    result.successful_mission_count = successful_missions_.size();
    result.terminal_non_success_mission_count = terminal_non_success_missions_.size();
    result.successful_speech_count = successful_speeches_.size();
    if (have_initial_odometry_ && have_latest_odometry_) {
      const auto delta_x = latest_odometry_.pose.pose.position.x -
        initial_odometry_.pose.pose.position.x;
      const auto delta_y = latest_odometry_.pose.pose.position.y -
        initial_odometry_.pose.pose.position.y;
      result.displacement_m = delta_x * std::cos(initial_yaw_) +
        delta_y * std::sin(initial_yaw_);
    }
    result.yaw_delta_rad = yaw_delta_rad_;
    result.active_step_sequence = active_step_sequence_;
    result.armed_nonzero_controller_steps = armed_nonzero_controller_steps_;
    result.controller_nonzero_observed = controller_nonzero_observed_;
    result.post_stop_nonzero_command_observed = post_stop_nonzero_command_observed_;
    result.final_gate_inhibited = final_gate_inhibited_;
    result.final_zero_stationary = final_command_is_zero_ && final_odometry_is_stationary_;
    result.gate_inhibited_after_motion = gate_inhibited_after_motion_;
    result.final_command_is_zero = final_command_is_zero_;
    result.final_odometry_is_stationary = final_odometry_is_stationary_;
    result.final_stationary_hold_ms = final_stationary_hold_ms();
    result.post_stop_odom_samples = post_stop_odom_samples_;
    return result;
  }

private:
  [[nodiscard]] std::int64_t final_stationary_hold_ms() const
  {
    if (!final_stationary_started_at_.has_value()) {
      return 0;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - *final_stationary_started_at_).count();
  }

  template<typename Predicate>
  bool wait_for(Predicate predicate, const std::chrono::seconds timeout = 60s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate(*this)) {
      if (condition_.wait_until(lock, std::min(
          deadline, std::chrono::steady_clock::now() + 10ms)) == std::cv_status::timeout &&
        std::chrono::steady_clock::now() >= deadline)
      {
        return false;
      }
      if (!predicate(*this)) {
        lock.unlock();
        device_.consume_once();
        lock.lock();
      }
    }
    return true;
  }

  ManualDevice & device_;
  rclcpp::Node::SharedPtr node_{};
  rclcpp::Subscription<voice_nav_interfaces::msg::MissionState>::SharedPtr state_subscription_{};
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_{};
  rclcpp::Subscription<voice_nav_interfaces::msg::VoiceTurn>::SharedPtr voice_turn_subscription_{};
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr mission_status_subscription_{};
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr speak_status_subscription_{};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_{};
  std::mutex mutex_{};
  std::condition_variable condition_{};
  bool runtime_ready_{false};
  bool clock_received_{false};
  bool final_gate_inhibited_{false};
  bool final_command_is_zero_{false};
  bool final_odometry_is_stationary_{false};
  bool latest_gate_armed_{false};
  std::uint32_t latest_active_step_{std::numeric_limits<std::uint32_t>::max()};
  bool have_initial_odometry_{false};
  bool have_latest_odometry_{false};
  bool first_turn_idle_window_{false};
  std::size_t first_turn_mission_statuses_{0U};
  std::size_t first_turn_command_samples_{0U};
  std::size_t first_turn_odom_samples_{0U};
  bool first_turn_nonzero_command_{false};
  bool first_turn_moving_{false};
  bool gate_armed_observed_{false};
  bool gate_inhibited_after_motion_{false};
  bool controller_nonzero_observed_{false};
  bool post_stop_nonzero_command_observed_{false};
  std::size_t post_stop_odom_samples_{0U};
  std::vector<std::uint32_t> active_step_sequence_{};
  std::set<std::uint32_t> armed_nonzero_controller_steps_{};
  std::set<std::string> mission_goal_ids_{};
  std::set<std::string> successful_missions_{};
  std::set<std::string> terminal_non_success_missions_{};
  std::set<std::string> successful_speeches_{};
  std::vector<voice_nav_interfaces::msg::VoiceTurn> turns_{};
  nav_msgs::msg::Odometry initial_odometry_{};
  nav_msgs::msg::Odometry latest_odometry_{};
  bool have_previous_yaw_{false};
  double previous_yaw_{0.0};
  double initial_yaw_{0.0};
  double yaw_delta_rad_{0.0};
  std::optional<std::chrono::steady_clock::time_point> final_stationary_started_at_{};
};

}  // namespace
}  // namespace voice_nav_audio

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions configuration_options;
  configuration_options.start_parameter_services(false).start_parameter_event_publisher(false);
  auto configuration = std::make_shared<rclcpp::Node>(
    "scripted_voice_demo_configuration", configuration_options);
  const auto configured_scenario = configuration->declare_parameter<std::string>("scenario", "move");
  const auto scenario = voice_nav_audio::scripted_scenario(configured_scenario);
  if (!scenario.has_value()) {
    RCLCPP_ERROR(
      configuration->get_logger(),
      "scenario must be one of move|stop|route; received '%s'", configured_scenario.c_str());
    configuration.reset();
    rclcpp::shutdown();
    return 1;
  }
  std::optional<std::string> exact_head;
  if (!voice_nav_audio::load_optional_exact_head(exact_head)) {
    RCLCPP_ERROR(
      configuration->get_logger(),
      "VOICE_NAV_EXACT_HEAD must be a lowercase 40-character Git commit");
    configuration.reset();
    rclcpp::shutdown();
    return 1;
  }
  configuration.reset();
  voice_nav_audio::PlaybackEvidenceRecorder recorder;
  voice_nav_audio::ManualDevice device(recorder);
  auto pipeline = std::make_unique<voice_nav_audio::VoicePipeline>(
    std::make_unique<voice_nav_audio::ScriptedRecognizer>(*scenario),
    std::make_unique<voice_nav_audio::DeterministicFakeTts>(recorder), device, &recorder);
  auto graph_probe = std::make_shared<rclcpp::Node>("scripted_voice_demo_graph_probe");
  if (!voice_nav_audio::wait_for_graph(*graph_probe)) {
    RCLCPP_ERROR(graph_probe->get_logger(), "scripted motion smoke graph did not converge");
    pipeline.reset();
    graph_probe.reset();
    rclcpp::shutdown();
    return 1;
  }

  voice_nav_audio::CompletionObserver observer(device);
  rclcpp::executors::SingleThreadedExecutor executor;
  pipeline->add_to_executor(executor);
  executor.add_node(observer.node());
  std::thread spin_thread([&executor]() {executor.spin();});
  if (!observer.wait_for_runtime_ready()) {
    RCLCPP_ERROR(graph_probe->get_logger(), "Mission Runtime did not become available");
    executor.cancel();
    spin_thread.join();
    pipeline->remove_from_executor(executor);
    pipeline.reset();
    graph_probe.reset();
    rclcpp::shutdown();
    return 1;
  }

  if (*scenario == voice_nav_audio::ScriptedScenario::kMove) {
    for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
      pipeline->accept_cleaned_frame(voice_nav_audio::cleaned_frame(sequence));
    }
    if (!observer.wait_for_first_clarification()) {
      RCLCPP_ERROR(graph_probe->get_logger(), "first Voice turn did not clarify without Mission");
      executor.cancel();
      spin_thread.join();
      pipeline->remove_from_executor(executor);
      pipeline.reset();
      graph_probe.reset();
      rclcpp::shutdown();
      return 1;
    }
    // Keep the first clarification observable before the scripted follow-up.
    observer.begin_first_turn_idle_window();
    if (!observer.wait_for_first_turn_idle_samples()) {
      RCLCPP_ERROR(graph_probe->get_logger(), "first Voice turn was not sampled as idle");
      executor.cancel();
      spin_thread.join();
      pipeline->remove_from_executor(executor);
      pipeline.reset();
      graph_probe.reset();
      rclcpp::shutdown();
      return 1;
    }
    for (std::uint64_t sequence = 4U; sequence <= 6U; ++sequence) {
      pipeline->accept_cleaned_frame(voice_nav_audio::cleaned_frame(sequence));
    }
    if (!observer.wait_for_mission_completion() || !observer.wait_for_final_zero_and_stationarity()) {
      RCLCPP_ERROR(graph_probe->get_logger(), "follow-up Voice Mission did not complete safely");
      executor.cancel();
      spin_thread.join();
      pipeline->remove_from_executor(executor);
      pipeline.reset();
      graph_probe.reset();
      rclcpp::shutdown();
      return 1;
    }
  } else if (*scenario == voice_nav_audio::ScriptedScenario::kRoute) {
    observer.begin_first_turn_idle_window();
    if (!observer.wait_for_first_turn_idle_samples()) {
      RCLCPP_ERROR(
        graph_probe->get_logger(),
        "route Voice Mission did not observe a stable idle controller and odometry barrier");
      executor.cancel();
      spin_thread.join();
      pipeline->remove_from_executor(executor);
      pipeline.reset();
      graph_probe.reset();
      rclcpp::shutdown();
      return 1;
    }
    for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
      pipeline->accept_cleaned_frame(voice_nav_audio::cleaned_frame(sequence));
    }
    if (!observer.wait_for_route_completion() || !observer.wait_for_final_zero_and_stationarity()) {
      const auto route_summary = observer.summary();
      RCLCPP_ERROR(
        graph_probe->get_logger(),
        "route Voice Mission did not prove ordered motion safely: turns=%zu goals=%zu successes=%zu "
        "steps=%zu controller_steps=%zu gate=%d command_zero=%d odom_stationary=%d hold_ms=%ld",
        route_summary.turns.size(), route_summary.unique_mission_count,
        route_summary.successful_mission_count, route_summary.active_step_sequence.size(),
        route_summary.armed_nonzero_controller_steps.size(), route_summary.final_gate_inhibited,
        route_summary.final_command_is_zero, route_summary.final_odometry_is_stationary,
        static_cast<long>(route_summary.final_stationary_hold_ms));
      executor.cancel();
      spin_thread.join();
      pipeline->remove_from_executor(executor);
      pipeline.reset();
      graph_probe.reset();
      rclcpp::shutdown();
      return 1;
    }
  } else {
    for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
      pipeline->accept_cleaned_frame(voice_nav_audio::cleaned_frame(sequence));
    }
    if (!observer.wait_for_controller_nonzero()) {
      RCLCPP_ERROR(graph_probe->get_logger(), "MOVE_DISTANCE did not reach a nonzero controller output");
      executor.cancel();
      spin_thread.join();
      pipeline->remove_from_executor(executor);
      pipeline.reset();
      graph_probe.reset();
      rclcpp::shutdown();
      return 1;
    }
    for (std::uint64_t sequence = 4U; sequence <= 6U; ++sequence) {
      pipeline->accept_cleaned_frame(voice_nav_audio::cleaned_frame(sequence));
    }
    if (!observer.wait_for_stop_completion()) {
      const auto stop_summary = observer.summary();
      RCLCPP_ERROR(
        graph_probe->get_logger(),
        "scripted STOP did not return to an inhibited, stationary state: turns=%zu goals=%zu "
        "terminal_non_success=%zu speaks=%zu gate_transition=%d final_gate=%d command_zero=%d "
        "odom_stationary=%d post_stop_odom=%zu post_stop_nonzero=%d",
        stop_summary.turns.size(), stop_summary.unique_mission_count,
        stop_summary.terminal_non_success_mission_count, stop_summary.successful_speech_count,
        stop_summary.gate_inhibited_after_motion, stop_summary.final_gate_inhibited,
        stop_summary.final_command_is_zero, stop_summary.final_odometry_is_stationary,
        stop_summary.post_stop_odom_samples, stop_summary.post_stop_nonzero_command_observed);
      executor.cancel();
      spin_thread.join();
      pipeline->remove_from_executor(executor);
      pipeline.reset();
      graph_probe.reset();
      rclcpp::shutdown();
      return 1;
    }
  }
  const auto playback = recorder.snapshot();
  const std::vector<std::string> expected_tts_texts = *scenario ==
    voice_nav_audio::ScriptedScenario::kMove ?
    std::vector<std::string>{"请说明需要前进多少米。", "任务已完成。"} :
    *scenario == voice_nav_audio::ScriptedScenario::kRoute ?
    std::vector<std::string>{"任务已完成。"} : std::vector<std::string>{"已停止。"};
  const std::set<std::uint64_t> completed_scope_ids(
    playback.completion_scope_ids.cbegin(), playback.completion_scope_ids.cend());
  const auto completed_only = std::all_of(
    playback.completion_codes.cbegin(), playback.completion_codes.cend(),
    [](const auto code) {return code == voice_nav_audio::SpeechResultCode::Completed;});
  if (playback.tts_texts != expected_tts_texts ||
    playback.nonzero_callback_count < expected_tts_texts.size() ||
    playback.feedback_scope_ids.size() != expected_tts_texts.size() ||
    playback.completion_scope_ids.size() != expected_tts_texts.size() ||
    completed_scope_ids.size() != expected_tts_texts.size() || !completed_only ||
    observer.successful_speech_count() != expected_tts_texts.size())
  {
    RCLCPP_ERROR(graph_probe->get_logger(), "real Speak playback evidence did not converge");
    executor.cancel();
    spin_thread.join();
    pipeline->remove_from_executor(executor);
    pipeline.reset();
    graph_probe.reset();
    rclcpp::shutdown();
    return 1;
  }
  const auto summary = observer.summary();
  if (*scenario == voice_nav_audio::ScriptedScenario::kMove) {
    std::cout << "EVIDENCE scripted_voice_demo "
              << "{\"schema_version\":1,\"simulation_only\":true,\"node_graph\":[\"agent_node\","
              << "\"mission_runtime_node\",\"motion_gate_node\",\"voice_speech_input\","
              << "\"voice_speech_output\"],\"voice\":{\"voice_instance_id\":"
              << std::quoted(summary.turns.front().voice_instance_id)
              << ",\"session_id\":" << std::quoted(summary.turns.front().session_id)
              << ",\"voice_seq\":[" << summary.turns.front().voice_seq << ","
              << summary.turns.back().voice_seq << "]},\"mission_success_count\":"
              << summary.successful_mission_count << ",\"speak_completed_count\":"
              << summary.successful_speech_count << ",\"motion\":{\"displacement_m\":"
              << summary.displacement_m << ",\"final_gate_inhibited\":"
              << (summary.final_gate_inhibited ? "true" : "false")
              << ",\"final_zero_stationary\":"
              << (summary.final_zero_stationary ? "true" : "false")
              << "},\"REAL_AUDIO_MODELS\":\"NOT_RUN\",\"REAL_LLM_CORPUS\":\"NOT_RUN\"}"
              << std::endl;
  } else if (*scenario == voice_nav_audio::ScriptedScenario::kStop) {
    const auto & command = summary.turns.front();
    const auto & stop = summary.turns.back();
    std::cout << "EVIDENCE scripted_voice_demo "
              << "{\"schema_version\":2,\"head\":";
    if (exact_head.has_value()) {
      std::cout << std::quoted(*exact_head);
    } else {
      std::cout << "null";
    }
    std::cout << ",\"scenario\":\"stop\",\"simulation_only\":true,\"voice\":{\"turns\":[{"
              << "\"voice_instance_id\":" << std::quoted(command.voice_instance_id)
              << ",\"voice_seq\":" << command.voice_seq
              << ",\"session_id\":" << std::quoted(command.session_id)
              << ",\"turn_id\":" << std::quoted(command.turn_id)
              << ",\"kind\":" << static_cast<unsigned int>(command.kind)
              << ",\"text\":" << std::quoted(command.text) << "},{\"voice_instance_id\":"
              << std::quoted(stop.voice_instance_id)
              << ",\"voice_seq\":" << stop.voice_seq
              << ",\"session_id\":" << std::quoted(stop.session_id)
              << ",\"turn_id\":" << std::quoted(stop.turn_id)
              << ",\"kind\":" << static_cast<unsigned int>(stop.kind)
              << ",\"text\":" << std::quoted(stop.text) << "}]},\"missions\":{\"unique_goal_count\":"
              << summary.unique_mission_count << ",\"terminal_non_success_goal_count\":"
              << summary.terminal_non_success_mission_count << "},\"stop\":{\"turn_count\":1,"
              << "\"controller_nonzero_before_stop\":"
              << (summary.controller_nonzero_observed ? "true" : "false")
              << ",\"post_stop_nonzero_command_observed\":"
              << (summary.post_stop_nonzero_command_observed ? "true" : "false")
              << "},\"motion\":{\"displacement_m\":" << summary.displacement_m
              << ",\"final_gate_inhibited\":"
              << (summary.final_gate_inhibited ? "true" : "false")
              << ",\"final_zero_stationary\":"
              << (summary.final_zero_stationary ? "true" : "false")
              << "},\"speak_completed_count\":" << summary.successful_speech_count
              << ",\"teardown\":\"bounded_clean_exit\",\"REAL_AUDIO_MODELS\":\"NOT_RUN\","
              << "\"REAL_LLM_CORPUS\":\"NOT_RUN\"}" << std::endl;
  } else {
    const auto & route = summary.turns.front();
    std::cout << "EVIDENCE scripted_voice_demo "
              << "{\"schema_version\":3,\"head\":";
    if (exact_head.has_value()) {
      std::cout << std::quoted(*exact_head);
    } else {
      std::cout << "null";
    }
    std::cout << ",\"scenario\":\"route\",\"simulation_only\":true,\"voice\":{\"turns\":[{"
              << "\"voice_instance_id\":" << std::quoted(route.voice_instance_id)
              << ",\"voice_seq\":" << route.voice_seq
              << ",\"session_id\":" << std::quoted(route.session_id)
              << ",\"turn_id\":" << std::quoted(route.turn_id)
              << ",\"kind\":" << static_cast<unsigned int>(route.kind)
              << ",\"text\":" << std::quoted(route.text)
              << "}],\"speak_completed_count\":" << summary.successful_speech_count
              << "},\"provider\":{\"llm_http_request_count\":0},\"missions\":{"
              << "\"unique_goal_count\":" << summary.unique_mission_count
              << ",\"successful_goal_count\":" << summary.successful_mission_count
              << ",\"active_step_sequence\":[" << summary.active_step_sequence.at(0U) << ","
              << summary.active_step_sequence.at(1U)
              << "],\"armed_nonzero_controller_steps\":["
              << *summary.armed_nonzero_controller_steps.cbegin() << ","
              << *summary.armed_nonzero_controller_steps.crbegin()
              << "]},\"motion\":{\"displacement_m\":" << summary.displacement_m
              << ",\"yaw_delta_rad\":" << summary.yaw_delta_rad
              << ",\"final_gate_inhibited\":"
              << (summary.final_gate_inhibited ? "true" : "false")
              << ",\"final_command_is_zero\":"
              << (summary.final_command_is_zero ? "true" : "false")
              << ",\"final_odometry_is_stationary\":"
              << (summary.final_odometry_is_stationary ? "true" : "false")
              << ",\"stationary_hold_ms\":" << summary.final_stationary_hold_ms
              << "},\"teardown\":\"bounded_clean_exit\",\"REAL_AUDIO_MODELS\":\"NOT_RUN\","
              << "\"REAL_LLM_CORPUS\":\"NOT_RUN\"}" << std::endl;
  }
  executor.cancel();
  spin_thread.join();
  pipeline->remove_from_executor(executor);
  pipeline.reset();
  graph_probe.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
