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

#include "voice_nav_mission/relative_motion_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace voice_nav_mission
{
namespace
{

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;
constexpr std::uint8_t kMoveKind =
  static_cast<std::uint8_t>(MissionStepKind::MoveDistance);
constexpr std::uint8_t kRotateKind =
  static_cast<std::uint8_t>(MissionStepKind::RotateAngle);

bool finite_positive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

RelativeMotionController::RelativeMotionController(RelativeMotionPolicy policy)
: policy_(std::move(policy))
{
  if (
    !finite_positive(policy_.move_tolerance_m) ||
    !finite_positive(policy_.rotate_tolerance_rad) ||
    !finite_positive(policy_.move_min_command_mps) ||
    !finite_positive(policy_.move_max_command_mps) ||
    policy_.move_min_command_mps > policy_.move_max_command_mps ||
    !finite_positive(policy_.rotate_min_command_rps) ||
    !finite_positive(policy_.rotate_max_command_rps) ||
    policy_.rotate_min_command_rps > policy_.rotate_max_command_rps ||
    !finite_positive(policy_.stationarity_linear_tolerance_mps) ||
    !finite_positive(policy_.stationarity_angular_tolerance_rps) ||
    !finite_positive(policy_.move_stall_improvement_m) ||
    !finite_positive(policy_.rotate_stall_improvement_rad) ||
    policy_.dependency_liveness_timeout.count() <= 0 ||
    policy_.stall_window.count() <= 0 ||
    policy_.zero_proof_deadline.count() <= 0 ||
    policy_.stationarity_window.count() <= 0 ||
    policy_.stationarity_deadline.count() <= 0)
  {
    throw std::invalid_argument("invalid relative-motion policy");
  }
}

RelativeMotionEvent RelativeMotionController::start(
  const MotionToken & token,
  const MissionStep & step,
  const TimePoint now)
{
  Mode mode = Mode::None;
  double target = 0.0;
  if (!valid_step(step, mode, target)) {
    return fail(
      RelativeMotionFailure::InternalError,
      "RelativeMotionController received an invalid step");
  }
  token_ = token;
  step_ = step;
  mode_ = mode;
  state_ = State::Running;
  stop_intent_ = RelativeMotionStopIntent::Completion;
  command_ = {};
  odom_ = {};
  has_odom_ = false;
  start_x_m_ = 0.0;
  start_y_m_ = 0.0;
  start_yaw_rad_ = 0.0;
  last_raw_yaw_rad_ = 0.0;
  unwrapped_delta_rad_ = 0.0;
  current_error_ = target;
  historical_progress_ = 0.0;
  stall_window_start_absolute_error_ = std::abs(target);
  stall_best_absolute_error_ = std::abs(target);
  started_at_ = now;
  stall_window_started_at_.reset();
  last_odom_at_.reset();
  zero_requested_at_.reset();
  gate_zero_at_.reset();
  stationary_since_.reset();
  return event(RelativeMotionEventKind::Running);
}

RelativeMotionEvent RelativeMotionController::observe_odom(
  const RelativeMotionOdom & odom,
  const TimePoint now)
{
  if (state_ == State::Idle || state_ == State::Completed) {
    return event(RelativeMotionEventKind::None);
  }
  if (!valid_odom(odom)) {
    return fail(
      RelativeMotionFailure::DependencyUnavailable,
      "odometry contains a non-finite pose or velocity");
  }

  if (state_ == State::Stationarity && gate_zero_at_.has_value() &&
    now < *gate_zero_at_)
  {
    return event(RelativeMotionEventKind::StationarityPending);
  }

  odom_ = odom;
  if (!has_odom_) {
    start_x_m_ = odom.x_m;
    start_y_m_ = odom.y_m;
    start_yaw_rad_ = odom.yaw_rad;
    last_raw_yaw_rad_ = odom.yaw_rad;
    unwrapped_delta_rad_ = 0.0;
    has_odom_ = true;
    stall_window_started_at_ = now;
    last_odom_at_ = now;
    stall_window_start_absolute_error_ = std::abs(error());
    stall_best_absolute_error_ = std::abs(error());
  } else if (mode_ == Mode::Rotate) {
    unwrapped_delta_rad_ += normalize_angle(odom.yaw_rad - last_raw_yaw_rad_);
    last_raw_yaw_rad_ = odom.yaw_rad;
  }
  last_odom_at_ = now;

  return evaluate(now);
}

RelativeMotionEvent RelativeMotionController::tick(const TimePoint now)
{
  if (state_ == State::Idle) {
    return event(RelativeMotionEventKind::None);
  }
  if (state_ == State::Failed) {
    return event(RelativeMotionEventKind::Failed, RelativeMotionFailure::InternalError);
  }
  if (state_ == State::Completed) {
    auto completed = event(RelativeMotionEventKind::Completed);
    completed.stationarity_proven = true;
    return completed;
  }
  return evaluate(now);
}

RelativeMotionEvent RelativeMotionController::request_safe_stop(
  const RelativeMotionStopIntent intent,
  const TimePoint now)
{
  if (state_ == State::Idle || state_ == State::Failed || state_ == State::Completed) {
    return event(RelativeMotionEventKind::None);
  }
  stop_intent_ = intent;
  state_ = State::ZeroRequested;
  command_ = {};
  zero_requested_at_ = now;
  gate_zero_at_.reset();
  stationary_since_.reset();
  auto requested = event(RelativeMotionEventKind::ZeroRequested);
  requested.zero_requested = true;
  requested.stop_intent = stop_intent_;
  return requested;
}

RelativeMotionEvent RelativeMotionController::confirm_gate_zero(const TimePoint now)
{
  if (state_ != State::ZeroRequested && state_ != State::Running) {
    return event(RelativeMotionEventKind::None);
  }
  if (zero_requested_at_.has_value() &&
    now - *zero_requested_at_ > policy_.zero_proof_deadline)
  {
    return fail(
      RelativeMotionFailure::SafetyFault,
      "Gate zero was not proven before the bounded zero-proof deadline");
  }
  if (state_ == State::Running) {
    stop_intent_ = RelativeMotionStopIntent::Completion;
    state_ = State::ZeroRequested;
    zero_requested_at_ = now;
  }
  gate_zero_at_ = now;
  state_ = State::Stationarity;
  stationary_since_.reset();
  return event(RelativeMotionEventKind::StationarityPending);
}

bool RelativeMotionController::active() const noexcept
{
  return state_ != State::Idle && state_ != State::Failed &&
         state_ != State::Completed;
}

bool RelativeMotionController::stationarity_proven() const noexcept
{
  return state_ == State::Completed;
}

bool RelativeMotionController::has_odom() const noexcept
{
  return has_odom_;
}

const MotionToken & RelativeMotionController::token() const noexcept
{
  return token_;
}

RelativeMotionCommand RelativeMotionController::command() const noexcept
{
  return command_;
}

double RelativeMotionController::progress() const noexcept
{
  return historical_progress_;
}

RelativeMotionEvent RelativeMotionController::event(
  const RelativeMotionEventKind kind,
  const RelativeMotionFailure failure,
  std::string detail) const
{
  return RelativeMotionEvent{
    kind,
    failure,
    stop_intent_,
    command_,
    historical_progress_,
    kind == RelativeMotionEventKind::ZeroRequested ||
    kind == RelativeMotionEventKind::StationarityPending,
    state_ == State::Completed,
    std::move(detail)};
}

RelativeMotionEvent RelativeMotionController::fail(
  const RelativeMotionFailure failure,
  std::string detail)
{
  state_ = State::Failed;
  command_ = {};
  return event(RelativeMotionEventKind::Failed, failure, std::move(detail));
}

RelativeMotionEvent RelativeMotionController::evaluate(const TimePoint now)
{
  command_ = {};
  if (state_ == State::ZeroRequested) {
    if (zero_requested_at_.has_value() &&
      now - *zero_requested_at_ > policy_.zero_proof_deadline)
    {
      return fail(
        RelativeMotionFailure::SafetyFault,
        "Gate zero was not proven before the bounded zero-proof deadline");
    }
    return event(RelativeMotionEventKind::ZeroRequested);
  }
  if (state_ == State::Stationarity) {
    return evaluate_stationarity(now);
  }
  if (state_ != State::Running) {
    return event(RelativeMotionEventKind::None);
  }
  if (!started_at_.has_value()) {
    return fail(
      RelativeMotionFailure::InternalError,
      "relative-motion start time is missing");
  }
  if (now >= *started_at_ + step_deadline()) {
    return fail(
      RelativeMotionFailure::Timeout,
      "relative-motion step deadline elapsed");
  }
  if (!has_odom_) {
    if (now - *started_at_ > policy_.dependency_liveness_timeout) {
      return fail(
        RelativeMotionFailure::DependencyUnavailable,
        "fresh odometry was not received before the dependency deadline");
    }
    return event(RelativeMotionEventKind::Running);
  }
  if (!last_odom_at_.has_value() ||
    now - *last_odom_at_ > policy_.dependency_liveness_timeout)
  {
    return fail(
      RelativeMotionFailure::DependencyUnavailable,
      "odometry stopped being fresh during relative motion");
  }

  current_error_ = error();
  const auto absolute = absolute_error();
  if (target_magnitude() > 0.0) {
    historical_progress_ = std::max(
      historical_progress_,
      std::clamp(
        (target_magnitude() - absolute) / target_magnitude(), 0.0, 1.0));
  }
  if (!stall_window_started_at_.has_value()) {
    stall_window_started_at_ = now;
    stall_window_start_absolute_error_ = absolute;
    stall_best_absolute_error_ = absolute;
  }
  if (absolute < stall_best_absolute_error_) {
    stall_best_absolute_error_ = absolute;
  }
  if (now - *stall_window_started_at_ >= policy_.stall_window) {
    if (std::abs(error()) <= tolerance()) {
      return request_safe_stop(RelativeMotionStopIntent::Completion, now);
    }
    const auto improvement = stall_window_start_absolute_error_ -
      stall_best_absolute_error_;
    if (improvement < stall_improvement()) {
      return fail(
        RelativeMotionFailure::ExecutionFailed,
        "relative-motion error stalled inside its bounded window");
    }
    stall_window_started_at_ = now;
    stall_window_start_absolute_error_ = absolute;
    stall_best_absolute_error_ = absolute;
  }

  if (absolute <= tolerance()) {
    return request_safe_stop(RelativeMotionStopIntent::Completion, now);
  }
  const auto signed_error = sign(error());
  if (mode_ == Mode::Move) {
    command_.linear_x_mps = signed_error * clamp_command(
      absolute, policy_.move_min_command_mps, policy_.move_max_command_mps);
  } else {
    command_.angular_z_rps = signed_error * clamp_command(
      1.5 * absolute,
      policy_.rotate_min_command_rps,
      policy_.rotate_max_command_rps);
  }
  auto running = event(RelativeMotionEventKind::Running);
  running.zero_requested = false;
  return running;
}

RelativeMotionEvent RelativeMotionController::evaluate_stationarity(const TimePoint now)
{
  command_ = {};
  if (!gate_zero_at_.has_value()) {
    return event(RelativeMotionEventKind::ZeroRequested);
  }
  if (now >= *gate_zero_at_ + policy_.stationarity_deadline) {
    return fail(
      RelativeMotionFailure::SafetyFault,
      "odometry did not prove stationarity before the safety deadline");
  }
  if (!has_odom_ || !last_odom_at_.has_value() ||
    *last_odom_at_ < *gate_zero_at_)
  {
    return event(RelativeMotionEventKind::StationarityPending);
  }
  if (
    now - *last_odom_at_ > policy_.dependency_liveness_timeout)
  {
    return fail(
      RelativeMotionFailure::DependencyUnavailable,
      "fresh odometry was unavailable while proving stationarity");
  }
  const bool stationary =
    std::abs(odom_.linear_x_mps) <= policy_.stationarity_linear_tolerance_mps &&
    std::abs(odom_.angular_z_rps) <= policy_.stationarity_angular_tolerance_rps;
  if (!stationary) {
    stationary_since_.reset();
    return event(RelativeMotionEventKind::StationarityPending);
  }
  if (!stationary_since_.has_value()) {
    stationary_since_ = now;
  }
  if (now - *stationary_since_ >= policy_.stationarity_window) {
    state_ = State::Completed;
    auto completed = event(RelativeMotionEventKind::Completed);
    completed.stationarity_proven = true;
    return completed;
  }
  return event(RelativeMotionEventKind::StationarityPending);
}

bool RelativeMotionController::valid_odom(const RelativeMotionOdom & odom) const noexcept
{
  return std::isfinite(odom.x_m) && std::isfinite(odom.y_m) &&
         std::isfinite(odom.yaw_rad) && std::isfinite(odom.linear_x_mps) &&
         std::isfinite(odom.angular_z_rps);
}

bool RelativeMotionController::valid_step(
  const MissionStep & step,
  Mode & mode,
  double & target) const noexcept
{
  if (step.kind == kMoveKind && std::isfinite(step.distance_m) &&
    step.distance_m != 0.0F && step.angle_rad == 0.0F && step.target_id.empty())
  {
    mode = Mode::Move;
    target = step.distance_m;
    return true;
  }
  if (step.kind == kRotateKind && std::isfinite(step.angle_rad) &&
    std::abs(static_cast<double>(step.angle_rad)) <= kTwoPi &&
    step.angle_rad != 0.0F && step.distance_m == 0.0F && step.target_id.empty())
  {
    mode = Mode::Rotate;
    target = step.angle_rad;
    return true;
  }
  return false;
}

double RelativeMotionController::error() const noexcept
{
  if (mode_ == Mode::Move) {
    const auto projection = (odom_.x_m - start_x_m_) * std::cos(start_yaw_rad_) +
      (odom_.y_m - start_y_m_) * std::sin(start_yaw_rad_);
    return static_cast<double>(step_.distance_m) - projection;
  }
  return static_cast<double>(step_.angle_rad) - unwrapped_delta_rad_;
}

double RelativeMotionController::absolute_error() const noexcept
{
  return std::abs(error());
}

double RelativeMotionController::tolerance() const noexcept
{
  return mode_ == Mode::Move ? policy_.move_tolerance_m : policy_.rotate_tolerance_rad;
}

double RelativeMotionController::stall_improvement() const noexcept
{
  return mode_ == Mode::Move ?
         policy_.move_stall_improvement_m : policy_.rotate_stall_improvement_rad;
}

double RelativeMotionController::target_magnitude() const noexcept
{
  return mode_ == Mode::Move ?
         std::abs(static_cast<double>(step_.distance_m)) :
         std::abs(static_cast<double>(step_.angle_rad));
}

std::chrono::milliseconds RelativeMotionController::step_deadline() const noexcept
{
  const auto maximum = mode_ == Mode::Move ?
    policy_.move_max_command_mps : policy_.rotate_max_command_rps;
  const auto seconds = 2.0 + 2.5 * target_magnitude() / maximum;
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(seconds));
}

double RelativeMotionController::normalize_angle(const double angle) noexcept
{
  auto result = angle;
  while (result > kPi) {
    result -= kTwoPi;
  }
  while (result < -kPi) {
    result += kTwoPi;
  }
  return result;
}

double RelativeMotionController::sign(const double value) noexcept
{
  return value < 0.0 ? -1.0 : 1.0;
}

double RelativeMotionController::clamp_command(
  const double value,
  const double minimum,
  const double maximum) noexcept
{
  return std::clamp(value, minimum, maximum);
}

}  // namespace voice_nav_mission
