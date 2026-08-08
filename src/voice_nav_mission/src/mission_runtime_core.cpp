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

#include "voice_nav_mission/mission_runtime_core.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

#if defined(__linux__)
#include <sys/random.h>
#endif

namespace voice_nav_mission
{
namespace
{

constexpr std::size_t kMaximumRuntimeIdLength = 36U;
constexpr std::size_t kMaximumSourceIdLength = 36U;
constexpr std::size_t kMaximumStopReasonLength = 160U;
constexpr std::size_t kMaximumTargetIdLength = 64U;
constexpr std::uint8_t kNavigateToKind =
  static_cast<std::uint8_t>(MissionStepKind::NavigateTo);
constexpr std::uint8_t kSaveMapKind =
  static_cast<std::uint8_t>(MissionStepKind::SaveMap);

std::string bounded_detail(std::string detail)
{
  if (detail.size() > kMaximumStopReasonLength) {
    detail.resize(kMaximumStopReasonLength);
  }
  return detail;
}

std::string make_random_identifier()
{
  std::array<std::uint8_t, 16U> bytes{};
#if defined(__linux__)
  std::size_t offset = 0U;
  while (offset < bytes.size()) {
    const auto count = ::getrandom(
      bytes.data() + offset, bytes.size() - offset, 0U);
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error("OS CSPRNG is unavailable");
    }
    if (count == 0) {
      throw std::runtime_error("OS CSPRNG returned no entropy");
    }
    offset += static_cast<std::size_t>(count);
  }
#else
  std::random_device random;
  for (auto & byte : bytes) {
    byte = static_cast<std::uint8_t>(random());
  }
#endif

  std::ostringstream stream;
  stream << std::hex << std::nouppercase << std::setfill('0');
  for (const auto byte : bytes) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

}  // namespace

RuntimeCore::RuntimeCore(
  RuntimeConfig config,
  std::shared_ptr<SteadyClockPort> clock,
  std::shared_ptr<MotionAuthorityPort> authority,
  std::shared_ptr<RelativeMotionPort> relative_motion,
  StateCallback state_callback,
  FeedbackCallback feedback_callback,
  ResultCallback result_callback,
  ChildFeedbackDispatcher child_feedback_dispatcher,
  ChildResultDispatcher child_result_dispatcher,
  AdmissionFenceCheck admission_fence_check,
  ChildResultRegistrar child_result_registrar,
  ChildResultUnregistrar child_result_unregistrar)
: config_(std::move(config)),
  clock_(std::move(clock)),
  authority_(std::move(authority)),
  relative_motion_(std::move(relative_motion)),
  state_callback_(std::move(state_callback)),
  feedback_callback_(std::move(feedback_callback)),
  result_callback_(std::move(result_callback)),
  child_feedback_dispatcher_(std::move(child_feedback_dispatcher)),
  child_result_dispatcher_(std::move(child_result_dispatcher)),
  admission_fence_check_(std::move(admission_fence_check)),
  child_result_registrar_(std::move(child_result_registrar)),
  child_result_unregistrar_(std::move(child_result_unregistrar))
{
  if (!clock_ || !authority_ || !relative_motion_) {
    throw std::invalid_argument("Runtime Core requires clock and all ports");
  }
  if (
    config_.max_steps == 0U || config_.max_steps > 3U ||
    config_.source_cache_size == 0U || config_.source_cache_size > 64U ||
    config_.stop_cache_size == 0U || config_.stop_cache_size > 64U ||
    config_.initial_admission_epoch == 0U ||
    config_.mission_deadline.count() <= 0 ||
    config_.gate_discovery_deadline.count() <= 0 ||
    config_.control_response_deadline.count() <= 0 ||
    config_.stop_barrier.count() <= 0 || config_.cancel_grace.count() <= 0 ||
    config_.stationarity_deadline.count() <= 0)
  {
    throw std::invalid_argument("invalid trusted Runtime configuration");
  }
  if (
    !std::isfinite(config_.move_distance_min_m) ||
    !std::isfinite(config_.move_distance_max_m) ||
    !std::isfinite(config_.rotate_angle_min_rad) ||
    !std::isfinite(config_.rotate_angle_max_rad) ||
    config_.move_distance_min_m <= 0.0F ||
    config_.move_distance_min_m > config_.move_distance_max_m ||
    config_.move_distance_max_m <= 0.0F ||
    config_.rotate_angle_min_rad <= 0.0F ||
    config_.rotate_angle_min_rad > config_.rotate_angle_max_rad ||
    config_.rotate_angle_max_rad <= 0.0F)
  {
    throw std::invalid_argument("invalid trusted Mission step policy");
  }

  state_.operating_mode = config_.operating_mode;
  state_.admission_epoch = config_.initial_admission_epoch;
  state_.max_steps = config_.max_steps;
  state_.named_place_ids = config_.named_place_ids;
  state_.runtime_instance_id = config_.runtime_instance_id.empty() ?
    make_random_identifier() : config_.runtime_instance_id;
  if (!valid_bounded_id(
      state_.runtime_instance_id, kMaximumRuntimeIdLength, false))
  {
    throw std::invalid_argument("runtime_instance_id is outside the IDL bound");
  }
  gate_snapshot_ = authority_->snapshot();
  gate_discovery_started_at_ = clock_->now();
  state_.gate_state = gate_snapshot_.endpoint_available ?
    gate_snapshot_.state : GateState::Faulted;
  state_.availability = RuntimeAvailability::Unavailable;
  last_relative_healthy_ = relative_motion_->healthy();
  relative_health_initialized_ = true;
  publish_state();
}

AdmissionResult RuntimeCore::admit(
  const MissionGoal & goal,
  StartPermitCheck start_permit_check,
  const std::uint64_t admission_generation)
{
  MissionResult result;
  if (!valid_bounded_id(
      goal.source_instance_id, kMaximumSourceIdLength, false))
  {
    return AdmissionResult{0U, false, reject(
      MissionResultCode::InvalidPlan,
      "source_instance_id must be non-empty and within 36 characters")};
  }
  if (
    goal.runtime_instance_id != state_.runtime_instance_id ||
    goal.admission_epoch != state_.admission_epoch)
  {
    return AdmissionResult{0U, false, reject(
      MissionResultCode::StaleRequest,
      "Runtime instance or admission epoch is stale")};
  }
  if (!admission_allowed(state_.admission_epoch)) {
    state_.availability = RuntimeAvailability::Faulted;
    publish_state();
    return AdmissionResult{0U, false, reject(
      MissionResultCode::SafetyFault,
      "Runtime admission fence is active")};
  }
  if (!consume_source_sequence(goal, result)) {
    return AdmissionResult{0U, false, result};
  }
  if (!validate_goal(goal, result)) {
    return AdmissionResult{0U, false, result};
  }
  if (active_.has_value()) {
    return AdmissionResult{0U, false, reject(
      MissionResultCode::Busy,
      "another Mission already owns the single execution slot")};
  }
  if (state_.availability == RuntimeAvailability::Faulted) {
    return AdmissionResult{0U, false, reject(
      MissionResultCode::SafetyFault,
      "Runtime is faulted and remains fail-closed")};
  }
  if (!relative_motion_->healthy()) {
    return AdmissionResult{0U, false, reject(
      MissionResultCode::DependencyUnavailable,
      "RelativeMotionPort is unavailable")};
  }
  gate_snapshot_ = authority_->snapshot();
  if (gate_snapshot_.state == GateState::Faulted &&
    gate_snapshot_.endpoint_available)
  {
    return AdmissionResult{0U, false, reject(
      MissionResultCode::SafetyFault,
      "MotionGate is faulted")};
  }
  if (!gate_is_healthy(gate_snapshot_)) {
    if (
      gate_snapshot_.endpoint_available &&
      gate_snapshot_.state != GateState::Armed &&
      !zero_is_proven(gate_snapshot_))
    {
      state_.availability = RuntimeAvailability::Faulted;
      publish_state();
      return AdmissionResult{0U, false, reject(
        MissionResultCode::SafetyFault,
        "MotionGate zero proof is not trustworthy")};
    }
    return AdmissionResult{0U, false, reject(
      MissionResultCode::DependencyUnavailable,
      "MotionGate endpoint is unavailable")};
  }

  // The Node-owned start permit is checked again immediately before any
  // authority transaction.  A queued AdmitEvent that loses the quiesce
  // generation cannot enter PREPARE, even if it passed validation earlier.
  if (start_permit_check && !start_permit_check()) {
    return AdmissionResult{0U, false, reject(
      MissionResultCode::SafetyFault,
      "Runtime start permit was revoked before MotionGate PREPARE")};
  }

  const auto mission_id = next_mission_id_++;
  if (mission_id == 0U || next_generation_ == 0U) {
    state_.availability = RuntimeAvailability::Faulted;
    publish_state();
    return AdmissionResult{0U, false, reject(
      MissionResultCode::SafetyFault,
      "Mission generation exhausted")};
  }

  if (!relative_motion_->owns_authority_lifecycle()) {
    const auto prepare = authority_->prepare(make_operation());
    if (!prepare.applied) {
      return AdmissionResult{0U, false, reject(
        MissionResultCode::DependencyUnavailable,
        prepare.detail.empty() ? "MotionGate PREPARE failed" : prepare.detail)};
    }
    gate_snapshot_ = prepare.snapshot;
    current_lease_id_ = prepare.lease_id;
    if (!admission_allowed(state_.admission_epoch) ||
      (start_permit_check && !start_permit_check()))
    {
      const bool cleanup = inhibit_and_prove_zero();
      state_.availability = RuntimeAvailability::Faulted;
      publish_state();
      return AdmissionResult{0U, false, reject(
        MissionResultCode::SafetyFault,
        cleanup ? "Runtime admission fence raised after MotionGate PREPARE" :
        "Runtime admission fence raised after PREPARE and zero cleanup failed")};
    }
    if (start_permit_check && !start_permit_check()) {
      const bool cleanup = inhibit_and_prove_zero();
      state_.availability = RuntimeAvailability::Faulted;
      publish_state();
      return AdmissionResult{0U, false, reject(
        MissionResultCode::SafetyFault,
        cleanup ? "Runtime start permit was revoked before MotionGate OPEN" :
        "Runtime start permit was revoked and zero cleanup failed")};
    }
    const auto open = authority_->open(make_operation(current_lease_id_));
    if (!open.applied) {
      const bool cleanup = inhibit_and_prove_zero();
      if (!cleanup) {
        state_.availability = RuntimeAvailability::Faulted;
        publish_state();
        return AdmissionResult{0U, false, reject(
          MissionResultCode::SafetyFault,
          "MotionGate OPEN failed and zero cleanup could not be proven")};
      }
      return AdmissionResult{0U, false, reject(
        MissionResultCode::DependencyUnavailable,
        open.detail.empty() ? "MotionGate OPEN failed" : open.detail)};
    }
    gate_snapshot_ = open.snapshot;
  }

  if (start_permit_check && !start_permit_check()) {
    const bool cleanup = inhibit_and_prove_zero();
    state_.availability = RuntimeAvailability::Faulted;
    publish_state();
    return AdmissionResult{0U, false, reject(
      MissionResultCode::SafetyFault,
      cleanup ? "Runtime start permit was revoked after MotionGate OPEN" :
      "Runtime start permit was revoked and zero cleanup failed")};
  }

  active_ = ActiveMission{
    mission_id,
    next_generation_++,
    1U,
    0U,
    0U,
    state_.admission_epoch,
    admission_generation,
    goal.steps,
    clock_->now() + config_.mission_deadline,
    {},
    std::move(start_permit_check),
    false,
    false};
  last_feedback_progress_ = 0.0;
  state_.active_step = kNoActiveMissionStep;
  state_.availability = RuntimeAvailability::Busy;
  publish_state();
  start_step();
  return AdmissionResult{mission_id, true, {}};
}

void RuntimeCore::cancel(std::uint64_t mission_id)
{
  if (!active_.has_value() || active_->id != mission_id) {
    return;
  }
  select_terminal_and_stop(
    MissionResultCode::Canceled,
    "Mission canceled by the Action client");
}

StopResponse RuntimeCore::stop(const StopRequest & request)
{
  const auto execute_terminal = [this](
    MissionResultCode code,
    const std::string & detail,
    bool rotate_epoch) {
      if (active_.has_value()) {
        return select_terminal_and_stop(code, detail, rotate_epoch);
      }
      bool epoch_ok = true;
      if (rotate_epoch) {
        epoch_ok = this->rotate_epoch();
      }
      bool zero = false;
      bool canceled = true;
      if (relative_motion_->owns_authority_lifecycle()) {
        try {
          canceled = relative_motion_->cancel(
            MotionToken{}, clock_->now() + config_.stationarity_deadline);
          zero = canceled && relative_motion_->zero_proven();
        } catch (...) {
          canceled = false;
        }
      } else {
        zero = inhibit_and_prove_zero();
      }
      return TerminalOutcome{zero, canceled, epoch_ok};
    };

  if (!valid_bounded_id(request.request_id, kMaximumRuntimeIdLength, false)) {
    state_.availability = RuntimeAvailability::Faulted;
    const auto outcome = execute_terminal(
      MissionResultCode::SafetyFault,
      "STOP request_id is empty or outside the IDL bound",
      false);
    publish_state();
    return make_stop_response(
      2U, outcome.zero_proven,
      "STOP request_id is empty or outside the IDL bound");
  }
  if (!valid_bounded_id(
      request.source_instance_id, kMaximumSourceIdLength, true) ||
    request.reason.size() > kMaximumStopReasonLength)
  {
    state_.availability = RuntimeAvailability::Faulted;
    const auto outcome = execute_terminal(
      MissionResultCode::SafetyFault,
      "STOP metadata is outside the trusted bounds",
      false);
    publish_state();
    return make_stop_response(
      2U, outcome.zero_proven, "STOP metadata is outside the trusted bounds");
  }

  const auto fingerprint = stop_fingerprint(request);
  const auto cached = stop_cache_.find(request.request_id);
  if (cached != stop_cache_.end()) {
    if (cached->second.fingerprint != fingerprint) {
      state_.availability = RuntimeAvailability::Faulted;
      const auto outcome = execute_terminal(
        MissionResultCode::SafetyFault,
        "STOP request_id collision; Runtime is faulted",
        false);
      publish_state();
      return make_stop_response(
        2U, outcome.zero_proven, "STOP request_id collision; Runtime is faulted");
    }

    const auto outcome = execute_terminal(
      MissionResultCode::Stopped,
      "duplicate STOP reasserted the current safety barrier",
      false);
    const bool safe = outcome.zero_proven && outcome.cancel_acknowledged;
    return make_stop_response(
      safe ? 1U : 2U,
      outcome.zero_proven,
      safe ? "duplicate STOP reasserted Gate inhibit and zero" :
      "duplicate STOP could not prove Gate zero");
  }

  if (stop_cache_.size() >= config_.stop_cache_size) {
    state_.availability = RuntimeAvailability::Faulted;
    const auto outcome = execute_terminal(
      MissionResultCode::SafetyFault,
      "STOP idempotency cache is full; no request was evicted",
      false);
    publish_state();
    return make_stop_response(
      2U, outcome.zero_proven,
      "STOP idempotency cache is full; no request was evicted");
  }
  stop_cache_.emplace(request.request_id, StopCacheEntry{fingerprint});

  const auto outcome = execute_terminal(
    MissionResultCode::Stopped,
    "Mission stopped by Operational Stop",
    true);
  const bool safe = outcome.zero_proven && outcome.cancel_acknowledged &&
    outcome.epoch_advanced;
  if (!safe) {
    state_.availability = RuntimeAvailability::Faulted;
    publish_state();
    return make_stop_response(
      2U, outcome.zero_proven,
      "STOP transaction could not prove complete safe cleanup");
  }
  set_availability_from_dependencies();
  publish_state();
  return make_stop_response(0U, true, "Operational Stop applied");
}

void RuntimeCore::observe_gate(const GateSnapshot & snapshot)
{
  const bool was_healthy = gate_is_healthy(gate_snapshot_);
  const bool identity_changed =
    gate_bound_ && !gate_snapshot_.gate_instance_id.empty() &&
    !snapshot.gate_instance_id.empty() &&
    gate_snapshot_.gate_instance_id != snapshot.gate_instance_id;
  gate_snapshot_ = snapshot;
  state_.gate_state = snapshot.endpoint_available ?
    snapshot.state : GateState::Faulted;

  if (!gate_bound_) {
    if (
      snapshot.endpoint_available && snapshot.state != GateState::Faulted &&
      !startup_gate_is_ready(snapshot))
    {
      const bool startup_barrier_proven = inhibit_and_prove_zero();
      gate_snapshot_ = authority_->snapshot();
      state_.gate_state = gate_snapshot_.endpoint_available ?
        gate_snapshot_.state : GateState::Faulted;
      if (!startup_barrier_proven) {
        state_.availability = RuntimeAvailability::Faulted;
        gate_bound_ = false;
        gate_fault_handled_ = true;
        publish_state();
        return;
      }
    }
    const bool is_ready = startup_gate_is_ready(gate_snapshot_);
    gate_bound_ = is_ready;
    gate_fault_handled_ = !is_ready;
    if (!is_ready && snapshot.endpoint_available) {
      state_.availability = RuntimeAvailability::Faulted;
      publish_state();
      return;
    }
    set_availability_from_dependencies();
    publish_state();
    return;
  }
  const bool is_healthy = gate_is_healthy(gate_snapshot_);
  if (identity_changed || (!is_healthy && was_healthy)) {
    if (!gate_fault_handled_) {
      gate_fault_handled_ = true;
      (void)rotate_epoch();
      if (active_.has_value()) {
        select_terminal_and_stop(
          MissionResultCode::SafetyFault,
          "MotionGate health or identity changed during Mission");
      }
      state_.availability =
        snapshot.state == GateState::Faulted || !snapshot.endpoint_available ||
        (snapshot.endpoint_available && !zero_is_proven(snapshot)) ?
        RuntimeAvailability::Faulted : RuntimeAvailability::Unavailable;
      publish_state();
    }
    return;
  }
  if (is_healthy) {
    gate_fault_handled_ = false;
    gate_bound_ = true;
  }
  set_availability_from_dependencies();
  publish_state();
}

void RuntimeCore::observe_dependencies()
{
  const bool healthy = relative_motion_->healthy();
  if (relative_health_initialized_ && last_relative_healthy_ && !healthy) {
    (void)rotate_epoch();
    if (active_.has_value()) {
      select_terminal_and_stop(
        MissionResultCode::SafetyFault,
        "RelativeMotionPort became unhealthy during Mission");
    } else {
      state_.availability = RuntimeAvailability::Faulted;
      publish_state();
    }
  }
  last_relative_healthy_ = healthy;
  relative_health_initialized_ = true;
  if (!active_.has_value() && state_.availability != RuntimeAvailability::Faulted) {
    set_availability_from_dependencies();
    publish_state();
  }
}

void RuntimeCore::on_tick()
{
  observe_dependencies();
  const auto now = clock_->now();
  if (
    !gate_bound_ && !gate_snapshot_.endpoint_available &&
    !gate_discovery_timed_out_ &&
    now >= gate_discovery_started_at_ + config_.gate_discovery_deadline)
  {
    gate_discovery_timed_out_ = true;
    state_.availability = RuntimeAvailability::Unavailable;
    publish_state();
  }
  relative_motion_->tick(now);
  if (!active_.has_value()) {
    return;
  }
  if (now >= active_->deadline) {
    select_terminal_and_stop(
      MissionResultCode::Timeout,
      "Mission deadline elapsed on the injected steady clock");
    return;
  }
  if (!relative_motion_->owns_authority_lifecycle()) {
    const auto renewed = authority_->renew(make_operation(current_lease_id_));
    if (!renewed.applied) {
      select_terminal_and_stop(
        MissionResultCode::SafetyFault,
        renewed.detail.empty() ? "MotionGate authority renewal failed" : renewed.detail);
    }
  }
}

RuntimeState RuntimeCore::state() const
{
  return state_;
}

bool RuntimeCore::usable() const noexcept
{
  return state_.availability != RuntimeAvailability::Faulted;
}

bool RuntimeCore::has_active_mission() const noexcept
{
  return active_.has_value();
}

bool RuntimeCore::admission_allowed(const std::uint64_t admission_epoch) const
{
  return !admission_fence_check_ || admission_fence_check_(admission_epoch);
}

MissionResult RuntimeCore::reject(
  MissionResultCode code,
  std::string detail) const
{
  return MissionResult{code, -1, bounded_detail(std::move(detail))};
}

bool RuntimeCore::validate_goal(
  const MissionGoal & goal,
  MissionResult & result) const
{
  if (goal.steps.empty() || goal.steps.size() > config_.max_steps) {
    result = reject(
      MissionResultCode::InvalidPlan,
      "Mission must contain between one and three steps");
    return false;
  }
  for (std::size_t index = 0U; index < goal.steps.size(); ++index) {
    if (!validate_step(goal.steps[index], result)) {
      result.failed_step = -1;
      return false;
    }
    const auto kind = goal.steps[index].kind;
    if (kind == kNavigateToKind || kind == kSaveMapKind) {
      result = reject(
        MissionResultCode::UnsupportedStep,
        "NAVIGATE_TO and SAVE_MAP are not implemented in this Runtime slice");
      return false;
    }
  }
  return true;
}

bool RuntimeCore::validate_step(
  const MissionStep & step,
  MissionResult & result) const
{
  const auto move = static_cast<std::uint8_t>(MissionStepKind::MoveDistance);
  const auto rotate = static_cast<std::uint8_t>(MissionStepKind::RotateAngle);
  const auto finite_distance = std::isfinite(step.distance_m);
  const auto finite_angle = std::isfinite(step.angle_rad);
  const auto distance_in_range =
    (step.distance_m >= -config_.move_distance_max_m &&
    step.distance_m <= -config_.move_distance_min_m) ||
    (step.distance_m >= config_.move_distance_min_m &&
    step.distance_m <= config_.move_distance_max_m);
  const auto angle_in_range =
    (step.angle_rad >= -config_.rotate_angle_max_rad &&
    step.angle_rad <= -config_.rotate_angle_min_rad) ||
    (step.angle_rad >= config_.rotate_angle_min_rad &&
    step.angle_rad <= config_.rotate_angle_max_rad);
  if (step.target_id.size() > kMaximumTargetIdLength) {
    result = reject(
      MissionResultCode::InvalidPlan,
      "step target_id is outside the 64-character bound");
    return false;
  }
  if (step.kind == move) {
    if (
      !finite_distance || !distance_in_range || step.angle_rad != 0.0F ||
      !step.target_id.empty())
    {
      result = reject(
        MissionResultCode::InvalidPlan,
        "MOVE_DISTANCE has an invalid distance or unused payload");
      return false;
    }
    return true;
  }
  if (step.kind == rotate) {
    if (
      !finite_angle || !angle_in_range || step.distance_m != 0.0F ||
      !step.target_id.empty())
    {
      result = reject(
        MissionResultCode::InvalidPlan,
        "ROTATE_ANGLE has an invalid angle or unused payload");
      return false;
    }
    return true;
  }
  if (step.kind == kNavigateToKind || step.kind == kSaveMapKind) {
    if (
      !finite_distance || !finite_angle || step.distance_m != 0.0F ||
      step.angle_rad != 0.0F || step.target_id.empty())
    {
      result = reject(
        MissionResultCode::InvalidPlan,
        "unsupported step has an invalid union payload");
      return false;
    }
    return true;
  }
  result = reject(MissionResultCode::InvalidPlan, "unknown Mission step kind");
  return false;
}

bool RuntimeCore::consume_source_sequence(
  const MissionGoal & goal,
  MissionResult & result)
{
  const auto found = source_sequences_.find(goal.source_instance_id);
  if (found == source_sequences_.end()) {
    if (source_sequences_.size() >= config_.source_cache_size) {
      result = reject(
        MissionResultCode::DependencyUnavailable,
        "source sequence cache is full; no source was evicted");
      return false;
    }
    source_sequences_.emplace(goal.source_instance_id, goal.source_seq);
    return true;
  }
  if (goal.source_seq <= found->second) {
    result = reject(
      MissionResultCode::StaleRequest,
      "source_seq is not strictly greater than the last accepted sequence");
    return false;
  }
  found->second = goal.source_seq;
  return true;
}

bool RuntimeCore::gate_is_healthy(const GateSnapshot & snapshot) const
{
  return snapshot.endpoint_available && !snapshot.gate_instance_id.empty() &&
         snapshot.state != GateState::Faulted &&
         (snapshot.state == GateState::Armed || zero_is_proven(snapshot));
}

bool RuntimeCore::startup_gate_is_ready(const GateSnapshot & snapshot) const
{
  return snapshot.endpoint_available && !snapshot.gate_instance_id.empty() &&
         snapshot.state == GateState::Inhibited && zero_is_proven(snapshot);
}

bool RuntimeCore::zero_is_proven(const GateSnapshot & snapshot) const
{
  return snapshot.endpoint_available && snapshot.motion_inhibited &&
         snapshot.zero_selected && snapshot.zero_published;
}

AuthorityOperation RuntimeCore::make_operation(const std::string & lease_id) const
{
  return AuthorityOperation{
    new_identifier(),
    gate_snapshot_.gate_instance_id,
    gate_snapshot_.control_seq,
    lease_id.empty() ? gate_snapshot_.lease_id : lease_id};
}

std::string RuntimeCore::new_identifier() const
{
  return config_.identifier_generator ? config_.identifier_generator() :
         make_random_identifier();
}

bool RuntimeCore::rotate_epoch()
{
  if (!increment_epoch(state_.admission_epoch)) {
    state_.availability = RuntimeAvailability::Faulted;
    publish_state();
    return false;
  }
  publish_state();
  return true;
}

void RuntimeCore::set_availability_from_dependencies()
{
  if (state_.availability == RuntimeAvailability::Faulted) {
    return;
  }
  if (active_.has_value()) {
    state_.availability = RuntimeAvailability::Busy;
    return;
  }
  gate_snapshot_ = authority_->snapshot();
  state_.gate_state = gate_snapshot_.endpoint_available ?
    gate_snapshot_.state : GateState::Faulted;
  const bool gate_requires_fault =
    gate_snapshot_.state == GateState::Faulted ||
    (gate_snapshot_.state != GateState::Armed &&
    !zero_is_proven(gate_snapshot_));
  if (!gate_snapshot_.endpoint_available) {
    state_.availability = RuntimeAvailability::Unavailable;
  } else if (gate_requires_fault) {
    state_.availability = RuntimeAvailability::Faulted;
  } else if (!gate_is_healthy(gate_snapshot_) || !relative_motion_->healthy()) {
    state_.availability = RuntimeAvailability::Unavailable;
  } else {
    state_.availability = RuntimeAvailability::Available;
  }
}

void RuntimeCore::publish_state()
{
  if (state_callback_) {
    state_callback_(state_);
  }
}

void RuntimeCore::publish_feedback(
  FeedbackPhase phase,
  std::uint32_t step_index,
  double progress)
{
  if (!active_.has_value() || !feedback_callback_) {
    return;
  }
  const auto total = static_cast<double>(active_->steps.size());
  const auto clamped_child = std::clamp(progress, 0.0, 1.0);
  const auto mission_progress = phase == FeedbackPhase::SafeStopping ?
    last_feedback_progress_ :
    (static_cast<double>(active_->completed_steps) + clamped_child) / total;
  const auto monotonic = std::clamp(
    std::max(last_feedback_progress_, mission_progress), 0.0, 1.0);
  last_feedback_progress_ = monotonic;
  feedback_callback_(
    active_->id,
    MissionFeedback{
      phase,
      std::max(active_->step_index, step_index),
      static_cast<float>(monotonic)});
}

void RuntimeCore::start_step()
{
  if (!active_.has_value()) {
    return;
  }
  if (!admission_allowed(active_->admission_epoch)) {
    select_terminal_and_stop(
      MissionResultCode::SafetyFault,
      "Runtime admission fence raised before RelativeMotionPort start");
    return;
  }
  if (active_->start_permit_check && !active_->start_permit_check()) {
    select_terminal_and_stop(
      MissionResultCode::SafetyFault,
      "Runtime start permit was revoked before RelativeMotionPort start");
    return;
  }
  publish_feedback(FeedbackPhase::Validating, active_->step_index, 0.0);
  const MotionToken token{
    active_->id,
    active_->admission_epoch,
    active_->generation,
    active_->step_generation,
    active_->admission_generation};
  active_->child_token = token;
  if (!admission_allowed(token.admission_epoch)) {
    select_terminal_and_stop(
      MissionResultCode::SafetyFault,
      "Runtime admission fence raised before RelativeMotionPort start");
    return;
  }
  bool completion_registered = false;
  try {
    if (relative_motion_->uses_external_completion_registry()) {
      if (!child_result_registrar_ || !child_result_registrar_(
          token,
          [this](const MotionToken & callback_token, const ChildResult & result) {
            on_child_result(callback_token, result);
          }))
      {
        select_terminal_and_stop(
          MissionResultCode::SafetyFault,
          "Node completion registry rejected RelativeMotionPort start");
        return;
      }
      completion_registered = true;
    }
    active_->child_started = true;
    RelativeMotionPort::ResultCallback result_callback;
    if (!relative_motion_->uses_external_completion_registry()) {
      result_callback = [this](
        const MotionToken & callback_token, const ChildResult & result) {
          if (child_result_dispatcher_) {
            (void)child_result_dispatcher_(callback_token, result);
          } else {
            on_child_result(callback_token, result);
          }
        };
    }
    relative_motion_->start(
      token,
      active_->steps[active_->step_index],
      [this](const MotionToken & callback_token, double progress) {
        if (child_feedback_dispatcher_) {
          (void)child_feedback_dispatcher_(callback_token, progress);
        } else {
          on_child_feedback(callback_token, progress);
        }
      },
      std::move(result_callback));
    if (
      active_.has_value() && active_->id == token.mission_id &&
      active_->generation == token.mission_generation &&
      active_->step_generation == token.step_generation)
    {
      state_.active_step = active_->step_index;
      publish_state();
    }
  } catch (const std::exception & error) {
    if (completion_registered && child_result_unregistrar_) {
      child_result_unregistrar_(token);
    }
    if (active_.has_value() && active_->id == token.mission_id) {
      active_->child_started = false;
    }
    select_terminal_and_stop(
      MissionResultCode::InternalError,
      std::string{"child start threw: "} + error.what());
  } catch (...) {
    if (completion_registered && child_result_unregistrar_) {
      child_result_unregistrar_(token);
    }
    if (active_.has_value() && active_->id == token.mission_id) {
      active_->child_started = false;
    }
    select_terminal_and_stop(
      MissionResultCode::InternalError,
      "child start threw an unknown exception");
  }
}

void RuntimeCore::on_child_feedback(
  const MotionToken & token,
  double progress)
{
  if (
    !active_.has_value() || token.mission_id != active_->id ||
    token.admission_epoch != active_->admission_epoch ||
    token.mission_generation != active_->generation ||
    token.step_generation != active_->step_generation)
  {
    return;
  }
  publish_feedback(FeedbackPhase::Executing, active_->step_index, progress);
}

void RuntimeCore::on_child_result(
  const MotionToken & token,
  const ChildResult & result)
{
  if (
    !active_.has_value() || token.mission_id != active_->id ||
    token.admission_epoch != active_->admission_epoch ||
    token.mission_generation != active_->generation ||
    token.step_generation != active_->step_generation)
  {
    return;
  }
  if (result.code != ChildResultCode::Succeeded) {
    MissionResultCode result_code = MissionResultCode::ExecutionFailed;
    if (result.code == ChildResultCode::DependencyUnavailable) {
      result_code = MissionResultCode::DependencyUnavailable;
    } else if (result.code == ChildResultCode::Timeout) {
      result_code = MissionResultCode::Timeout;
    } else if (result.code == ChildResultCode::SafetyFault) {
      result_code = MissionResultCode::SafetyFault;
    } else if (result.code == ChildResultCode::InternalError) {
      result_code = MissionResultCode::InternalError;
    }
    select_terminal_and_stop(
      result_code,
      result.detail.empty() ? "relative-motion child failed" : result.detail);
    return;
  }
  // The completed child contributes its terminal progress exactly once. The
  // completed_steps counter is advanced only after publishing that boundary,
  // so a two-step Mission reports 0.5 rather than 1.0 after step zero.
  publish_feedback(FeedbackPhase::Executing, active_->step_index, 1.0);
  ++active_->completed_steps;
  if (active_->completed_steps >= active_->steps.size()) {
    select_terminal_and_stop(
      MissionResultCode::Succeeded,
      "Mission completed");
    return;
  }
  ++active_->step_index;
  ++active_->step_generation;
  start_step();
}

void RuntimeCore::fail_closed(std::string detail)
{
  if (active_.has_value()) {
    select_terminal_and_stop(
      MissionResultCode::SafetyFault,
      std::move(detail));
    return;
  }
  state_.availability = RuntimeAvailability::Faulted;
  publish_state();
}

void RuntimeCore::fail_closed_at_epoch(
  const std::uint64_t admission_epoch,
  std::string detail)
{
  if (admission_epoch > state_.admission_epoch) {
    state_.admission_epoch = admission_epoch;
  }
  if (active_.has_value()) {
    (void)select_terminal_and_stop(
      MissionResultCode::SafetyFault, std::move(detail));
  }
  state_.availability = RuntimeAvailability::Faulted;
  publish_state();
}

RuntimeCore::TerminalOutcome RuntimeCore::select_terminal_and_stop(
  MissionResultCode code,
  std::string detail,
  bool rotate_epoch)
{
  if (!active_.has_value()) {
    return TerminalOutcome{};
  }
  if (active_->terminal_selected) {
    return TerminalOutcome{};
  }
  const auto token = active_->child_token;
  const auto child_started = active_->child_started;
  active_->terminal_selected = true;
  ++active_->generation;
  bool epoch_advanced = true;
  if (rotate_epoch && !increment_epoch(state_.admission_epoch)) {
    epoch_advanced = false;
    state_.availability = RuntimeAvailability::Faulted;
    publish_state();
    detail = "admission epoch exhausted while stopping Mission";
    code = MissionResultCode::SafetyFault;
  }
  publish_feedback(FeedbackPhase::SafeStopping, active_->step_index, 0.0);
  bool canceled = true;
  bool zero = false;
  if (relative_motion_->owns_authority_lifecycle()) {
    try {
      canceled = relative_motion_->cancel(
        token, clock_->now() + config_.stationarity_deadline);
      zero = canceled && relative_motion_->zero_proven();
    } catch (...) {
      canceled = false;
    }
  } else {
    zero = inhibit_and_prove_zero();
    try {
      canceled = relative_motion_->cancel(
        token, clock_->now() + config_.cancel_grace);
    } catch (...) {
      canceled = false;
    }
  }
  if (!zero || !canceled) {
    state_.availability = RuntimeAvailability::Faulted;
    finish_active(MissionResult{
        MissionResultCode::SafetyFault,
        child_started ? static_cast<std::int32_t>(active_->step_index) : -1,
        !zero ? "terminal barrier could not prove Gate inhibit and zero" :
        "relative-motion cancel did not acknowledge before its deadline"});
    return TerminalOutcome{zero, canceled, epoch_advanced};
  }
  const auto failed_step = child_started && code != MissionResultCode::Succeeded ?
    static_cast<std::int32_t>(active_->step_index) : -1;
  finish_active(MissionResult{code, failed_step, bounded_detail(std::move(detail))});
  return TerminalOutcome{zero, canceled, epoch_advanced};
}

void RuntimeCore::finish_active(const MissionResult & result)
{
  if (!active_.has_value()) {
    return;
  }
  const auto id = active_->id;
  active_.reset();
  state_.active_step = kNoActiveMissionStep;
  set_availability_from_dependencies();
  publish_state();
  if (result_callback_) {
    result_callback_(id, result);
  }
}

bool RuntimeCore::inhibit_and_prove_zero()
{
  gate_snapshot_ = authority_->snapshot();
  const auto result = authority_->inhibit(make_operation());
  gate_snapshot_ = result.snapshot;
  state_.gate_state = gate_snapshot_.endpoint_available ?
    gate_snapshot_.state : GateState::Faulted;
  return result.applied && result.zero_proven && zero_is_proven(gate_snapshot_);
}

StopResponse RuntimeCore::make_stop_response(
  std::uint16_t code,
  bool motion_inhibited,
  std::string detail) const
{
  return StopResponse{
    code,
    state_.runtime_instance_id,
    state_.admission_epoch,
    motion_inhibited,
    bounded_detail(std::move(detail))};
}

std::string RuntimeCore::stop_fingerprint(const StopRequest & request)
{
  const auto encode = [](const std::string & value) {
      return std::to_string(value.size()) + ":" + value;
    };
  return encode(request.source_instance_id) +
         encode(std::to_string(request.source_seq)) + encode(request.reason);
}

bool RuntimeCore::valid_bounded_id(
  const std::string & value,
  std::size_t maximum,
  bool allow_empty)
{
  return (allow_empty || !value.empty()) && value.size() <= maximum;
}

bool RuntimeCore::increment_epoch(std::uint64_t & epoch)
{
  if (epoch == std::numeric_limits<std::uint64_t>::max()) {
    return false;
  }
  ++epoch;
  return true;
}

ScriptedMotionAuthorityPort::ScriptedMotionAuthorityPort(
  std::string gate_instance_id)
: snapshot_{
    std::move(gate_instance_id), 0U, "", GateState::Inhibited,
    true, true, true, true}
{
}

AuthorityResult ScriptedMotionAuthorityPort::prepare(
  const AuthorityOperation & operation)
{
  return apply(operation, GateState::Prepared, true);
}

AuthorityResult ScriptedMotionAuthorityPort::open(
  const AuthorityOperation & operation)
{
  if (!next_open_failure_.empty()) {
    const auto detail = std::exchange(next_open_failure_, {});
    operations_.push_back(operation);
    return AuthorityResult{false, false, false, snapshot_, {}, detail};
  }
  return apply(operation, GateState::Armed, false);
}

AuthorityResult ScriptedMotionAuthorityPort::renew(
  const AuthorityOperation & operation)
{
  operations_.push_back(operation);
  if (!next_failure_.empty()) {
    const auto detail = std::exchange(next_failure_, {});
    return AuthorityResult{false, false, false, snapshot_, {}, detail};
  }
  return AuthorityResult{
    true, false, false, snapshot_, snapshot_.gate_instance_id, "renewed"};
}

AuthorityResult ScriptedMotionAuthorityPort::inhibit(
  const AuthorityOperation & operation)
{
  ++inhibit_count_;
  if (inhibit_observer_) {
    inhibit_observer_();
  }
  if (!next_inhibit_failure_.empty()) {
    const auto detail = std::exchange(next_inhibit_failure_, {});
    operations_.push_back(operation);
    return AuthorityResult{false, false, false, snapshot_, {}, detail};
  }
  return apply(operation, GateState::Inhibited, true);
}

void ScriptedMotionAuthorityPort::set_next_failure(std::string detail)
{
  next_failure_ = std::move(detail);
}

AuthorityResult ScriptedMotionAuthorityPort::apply(
  const AuthorityOperation & operation,
  GateState next_state,
  bool inhibited)
{
  operations_.push_back(operation);
  if (!next_failure_.empty()) {
    const auto detail = std::exchange(next_failure_, {});
    return AuthorityResult{false, false, false, snapshot_, {}, detail};
  }
  if (snapshot_.control_seq != std::numeric_limits<std::uint64_t>::max()) {
    ++snapshot_.control_seq;
  }
  snapshot_.state = next_state;
  snapshot_.lease_id = next_state == GateState::Armed ?
    (operation.lease_id.empty() ? std::string(32U, 'p') : operation.lease_id) :
    (next_state == GateState::Prepared ? std::string(32U, 'p') : std::string{});
  snapshot_.motion_inhibited = inhibited;
  snapshot_.zero_selected = inhibited;
  snapshot_.zero_published = inhibited;
  snapshot_.endpoint_available = true;
  if (next_state == GateState::Prepared || next_state == GateState::Armed) {
    snapshot_.zero_published = true;
  }
  if (next_state == GateState::Prepared || next_state == GateState::Armed) {
    snapshot_.gate_instance_id = snapshot_.gate_instance_id;
  }
  const auto lease = snapshot_.lease_id;
  return AuthorityResult{
    true,
    inhibited,
    false,
    snapshot_,
    next_state == GateState::Armed ? lease : std::string{},
    "applied"};
}

void ScriptedRelativeMotionPort::start(
  const MotionToken & token,
  const MissionStep & step,
  FeedbackCallback feedback,
  ResultCallback result)
{
  if (!next_start_failure_.empty()) {
    const auto detail = std::exchange(next_start_failure_, {});
    throw std::runtime_error(detail);
  }
  token_ = token;
  feedback_callback_ = std::move(feedback);
  result_callback_ = std::move(result);
  children_.push_back(ChildCallbacks{
      token, feedback_callback_, result_callback_});
  started_steps_.push_back(step);
  if (start_completion_ && result_callback_) {
    result_callback_(token, ChildResult{ChildResultCode::Succeeded, "sync complete"});
  }
}

bool ScriptedRelativeMotionPort::cancel(
  const MotionToken & token, SteadyClockPort::TimePoint deadline)
{
  ++cancel_count_;
  cancel_token_ = token;
  cancel_deadline_ = deadline;
  if (cancel_observer_) {
    cancel_observer_();
  }
  return cancel_acknowledged_;
}

void ScriptedRelativeMotionPort::feedback(double progress)
{
  if (token_.has_value() && feedback_callback_) {
    feedback_callback_(*token_, progress);
  }
}

void ScriptedRelativeMotionPort::complete()
{
  if (token_.has_value() && result_callback_) {
    result_callback_(*token_, ChildResult{ChildResultCode::Succeeded, "complete"});
  }
}

void ScriptedRelativeMotionPort::fail(std::string detail)
{
  if (token_.has_value() && result_callback_) {
    result_callback_(*token_, ChildResult{ChildResultCode::Failed, std::move(detail)});
  }
}

void ScriptedRelativeMotionPort::timeout(std::string detail)
{
  if (token_.has_value() && result_callback_) {
    result_callback_(*token_, ChildResult{ChildResultCode::Timeout, std::move(detail)});
  }
}

void ScriptedRelativeMotionPort::complete_token(const MotionToken & token)
{
  for (const auto & child : children_) {
    if (child.token.mission_id == token.mission_id &&
      child.token.admission_epoch == token.admission_epoch &&
      child.token.mission_generation == token.mission_generation &&
      child.token.step_generation == token.step_generation && child.result)
    {
      child.result(child.token, ChildResult{ChildResultCode::Succeeded, "complete"});
      return;
    }
  }
}

void ScriptedRelativeMotionPort::feedback_token(
  const MotionToken & token, double progress)
{
  for (const auto & child : children_) {
    if (child.token.mission_id == token.mission_id &&
      child.token.admission_epoch == token.admission_epoch &&
      child.token.mission_generation == token.mission_generation &&
      child.token.step_generation == token.step_generation && child.feedback)
    {
      child.feedback(child.token, progress);
      return;
    }
  }
}

}  // namespace voice_nav_mission
