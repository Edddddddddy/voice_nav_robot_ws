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

#ifndef VOICE_NAV_MISSION__RELATIVE_MOTION_CONTROLLER_HPP_
#define VOICE_NAV_MISSION__RELATIVE_MOTION_CONTROLLER_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "voice_nav_mission/mission_runtime_core.hpp"

namespace voice_nav_mission
{

struct RelativeMotionOdom
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double linear_x_mps{0.0};
  double angular_z_rps{0.0};
};

struct RelativeMotionCommand
{
  double linear_x_mps{0.0};
  double angular_z_rps{0.0};
};

enum class RelativeMotionEventKind : std::uint8_t
{
  None = 0,
  Running = 1,
  ZeroRequested = 2,
  StationarityPending = 3,
  Completed = 4,
  Failed = 5,
};

enum class RelativeMotionFailure : std::uint8_t
{
  None = 0,
  DependencyUnavailable = 1,
  ExecutionFailed = 2,
  Timeout = 3,
  SafetyFault = 4,
  InternalError = 5,
};

enum class RelativeMotionStopIntent : std::uint8_t
{
  Completion = 0,
  Cancel = 1,
  Failure = 2,
};

struct RelativeMotionEvent
{
  RelativeMotionEventKind kind{RelativeMotionEventKind::None};
  RelativeMotionFailure failure{RelativeMotionFailure::None};
  RelativeMotionStopIntent stop_intent{RelativeMotionStopIntent::Completion};
  RelativeMotionCommand command{};
  double progress{0.0};
  bool zero_requested{false};
  bool stationarity_proven{false};
  std::string detail;
};

struct RelativeMotionPolicy
{
  double move_tolerance_m{0.05};
  double rotate_tolerance_rad{0.08};
  double move_min_command_mps{0.05};
  double move_max_command_mps{0.25};
  double rotate_min_command_rps{0.10};
  double rotate_max_command_rps{0.80};
  double stationarity_linear_tolerance_mps{0.01};
  double stationarity_angular_tolerance_rps{0.02};
  double move_stall_improvement_m{0.01};
  double rotate_stall_improvement_rad{0.02};
  std::chrono::milliseconds dependency_liveness_timeout{200};
  std::chrono::milliseconds stall_window{1000};
  std::chrono::milliseconds zero_proof_deadline{300};
  std::chrono::milliseconds stationarity_window{200};
  std::chrono::milliseconds stationarity_deadline{1200};
};

// Deep, ROS-free implementation of the product MOVE/ROTATE semantics. The
// Interface accepts only a fenced token, a step, fresh odometry, and steady
// time. It owns projection/yaw-unwrapping, command limits, monotonic progress,
// stall/deadline checks, safe-zero and stationarity evidence.
class RelativeMotionController final
{
public:
  using TimePoint = SteadyClockPort::TimePoint;

  explicit RelativeMotionController(RelativeMotionPolicy policy = {});

  [[nodiscard]] RelativeMotionEvent start(
    const MotionToken & token,
    const MissionStep & step,
    TimePoint now);
  [[nodiscard]] RelativeMotionEvent observe_odom(
    const RelativeMotionOdom & odom,
    TimePoint now);
  [[nodiscard]] RelativeMotionEvent tick(TimePoint now);
  [[nodiscard]] RelativeMotionEvent request_safe_stop(
    RelativeMotionStopIntent intent,
    TimePoint now);
  [[nodiscard]] RelativeMotionEvent confirm_gate_zero(TimePoint now);

  [[nodiscard]] bool active() const noexcept;
  [[nodiscard]] bool stationarity_proven() const noexcept;
  [[nodiscard]] bool has_odom() const noexcept;
  [[nodiscard]] const MotionToken & token() const noexcept;
  [[nodiscard]] RelativeMotionCommand command() const noexcept;
  [[nodiscard]] double progress() const noexcept;

private:
  enum class Mode : std::uint8_t
  {
    None = 0,
    Move = 1,
    Rotate = 2,
  };

  enum class State : std::uint8_t
  {
    Idle = 0,
    Running = 1,
    ZeroRequested = 2,
    Stationarity = 3,
    Completed = 4,
    Failed = 5,
  };

  [[nodiscard]] RelativeMotionEvent event(
    RelativeMotionEventKind kind,
    RelativeMotionFailure failure = RelativeMotionFailure::None,
    std::string detail = {}) const;
  [[nodiscard]] RelativeMotionEvent fail(
    RelativeMotionFailure failure,
    std::string detail);
  [[nodiscard]] RelativeMotionEvent evaluate(TimePoint now);
  [[nodiscard]] RelativeMotionEvent evaluate_stationarity(TimePoint now);
  [[nodiscard]] bool valid_odom(const RelativeMotionOdom & odom) const noexcept;
  [[nodiscard]] bool valid_step(
    const MissionStep & step,
    Mode & mode,
    double & target) const noexcept;
  [[nodiscard]] double error() const noexcept;
  [[nodiscard]] double absolute_error() const noexcept;
  [[nodiscard]] double tolerance() const noexcept;
  [[nodiscard]] double stall_improvement() const noexcept;
  [[nodiscard]] double target_magnitude() const noexcept;
  [[nodiscard]] std::chrono::milliseconds step_deadline() const noexcept;
  [[nodiscard]] static double normalize_angle(double angle) noexcept;
  [[nodiscard]] static double sign(double value) noexcept;
  [[nodiscard]] static double clamp_command(
    double value,
    double minimum,
    double maximum) noexcept;

  RelativeMotionPolicy policy_;
  MotionToken token_{};
  MissionStep step_{};
  Mode mode_{Mode::None};
  State state_{State::Idle};
  RelativeMotionStopIntent stop_intent_{RelativeMotionStopIntent::Completion};
  RelativeMotionCommand command_{};
  RelativeMotionOdom odom_{};
  bool has_odom_{false};
  double start_x_m_{0.0};
  double start_y_m_{0.0};
  double start_yaw_rad_{0.0};
  double last_raw_yaw_rad_{0.0};
  double unwrapped_delta_rad_{0.0};
  double current_error_{0.0};
  double historical_progress_{0.0};
  double stall_window_start_absolute_error_{0.0};
  double stall_best_absolute_error_{0.0};
  std::optional<TimePoint> started_at_;
  std::optional<TimePoint> stall_window_started_at_;
  std::optional<TimePoint> last_odom_at_;
  std::optional<TimePoint> zero_requested_at_;
  std::optional<TimePoint> gate_zero_at_;
  std::optional<TimePoint> stationary_since_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RELATIVE_MOTION_CONTROLLER_HPP_
