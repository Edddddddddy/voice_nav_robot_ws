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

// Test-only two-turn driver.  It owns no installed executable or endpoint.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
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
#include "voice_pipeline.hpp"
#include "voice_nav_interfaces/msg/mission_state.hpp"

namespace voice_nav_audio
{
namespace
{

using namespace std::chrono_literals;

class ScriptedRecognizer final : public SpeechRecognizerAdapter
{
public:
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
        frame, active_scope_, "绕到大厅", 1.0F));
    } else if (frame.audio_seq == 6U) {
      sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
        frame, active_scope_, "半米", 1.0F));
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
        condition_.notify_all();
      });
    mission_status_subscription_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/mission/execute/_action/status", rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr status_array) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & status : status_array->status_list) {
          if (first_turn_idle_window_) {
            ++first_turn_mission_statuses_;
          }
          if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
            successful_missions_.emplace(
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
               return self.runtime_ready_;
    });
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
                      self.successful_speeches_.size() >= 2U;
    });
  }
  [[nodiscard]] std::size_t successful_speech_count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return successful_speeches_.size();
  }

private:
  template<typename Predicate>
  bool wait_for(Predicate predicate)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline = std::chrono::steady_clock::now() + 60s;
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
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr mission_status_subscription_{};
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr speak_status_subscription_{};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_{};
  std::mutex mutex_{};
  std::condition_variable condition_{};
  bool runtime_ready_{false};
  bool first_turn_idle_window_{false};
  std::size_t first_turn_mission_statuses_{0U};
  std::size_t first_turn_command_samples_{0U};
  std::size_t first_turn_odom_samples_{0U};
  bool first_turn_nonzero_command_{false};
  bool first_turn_moving_{false};
  std::set<std::string> successful_missions_{};
  std::set<std::string> successful_speeches_{};
};

}  // namespace
}  // namespace voice_nav_audio

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  voice_nav_audio::PlaybackEvidenceRecorder recorder;
  voice_nav_audio::ManualDevice device(recorder);
  auto pipeline = std::make_unique<voice_nav_audio::VoicePipeline>(
    std::make_unique<voice_nav_audio::ScriptedRecognizer>(),
    std::make_unique<voice_nav_audio::DeterministicFakeTts>(recorder), device, &recorder);
  auto graph_probe = std::make_shared<rclcpp::Node>("scripted_voice_motion_driver_graph_probe");
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
  // This test-only barrier gives the Python observer a bounded, sampled
  // first-turn idle window rather than racing directly into the follow-up.
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
  if (!observer.wait_for_mission_completion()) {
    RCLCPP_ERROR(graph_probe->get_logger(), "follow-up Voice Mission did not complete with Speak");
    executor.cancel();
    spin_thread.join();
    pipeline->remove_from_executor(executor);
    pipeline.reset();
    graph_probe.reset();
    rclcpp::shutdown();
    return 1;
  }
  const auto playback = recorder.snapshot();
  const std::vector<std::string> expected_tts_texts{
    "请说明需要前进多少米。", "任务已完成。"};
  const std::set<std::uint64_t> completed_scope_ids(
    playback.completion_scope_ids.cbegin(), playback.completion_scope_ids.cend());
  const auto completed_only = std::all_of(
    playback.completion_codes.cbegin(), playback.completion_codes.cend(),
    [](const auto code) {return code == voice_nav_audio::SpeechResultCode::Completed;});
  if (playback.tts_texts != expected_tts_texts || playback.nonzero_callback_count < 2U ||
    playback.feedback_scope_ids.size() != 2U || playback.completion_scope_ids.size() != 2U ||
    completed_scope_ids.size() != 2U || !completed_only || observer.successful_speech_count() != 2U)
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
  std::cout <<
    "EVIDENCE issue142_voice_pipeline "
    "{\"schema_version\":1,\"tts_texts\":[\"请说明需要前进多少米。\",\"任务已完成。\"],"
    "\"manual_nonzero_pcm\":true,\"played_feedback_scope_count\":2,"
    "\"completed_scope_count\":2,\"first_completed_before_followup\":true}" <<
    std::endl;
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
