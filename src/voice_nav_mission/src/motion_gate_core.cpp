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

#include "voice_nav_mission/motion_gate_core.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace voice_nav_mission
{
namespace
{

constexpr char kCandidateTopicPrefix[] =
  "/voice_nav_internal/motion_gate/candidate/lease_";
constexpr std::size_t kMaximumDetailLength = 160U;

std::string bounded_detail(std::string detail)
{
  if (detail.size() > kMaximumDetailLength) {
    detail.resize(kMaximumDetailLength);
  }
  return detail;
}

}  // namespace

bool Command::is_zero() const noexcept
{
  return linear_x == 0.0 && angular_z == 0.0;
}

MotionGateCore::MotionGateCore(
  MotionGateConfig config,
  std::string gate_instance_id,
  std::uint64_t initial_control_seq)
: config_(std::move(config)),
  gate_instance_id_(std::move(gate_instance_id)),
  control_seq_(initial_control_seq)
{
  selected_ = zero_command();
  detail_ = "motion inhibited";

  if (!config_is_valid(config_) || !valid_identifier(gate_instance_id_)) {
    state_ = State::Faulted;
    reason_ = Reason::ConfigurationInvalid;
    detail_ = "invalid MotionGate configuration or Gate instance identifier";
    advance_state_seq();
  }
}

ControlResult MotionGateCore::prepare(
  const ControlRequest & request,
  SteadyTimePoint now,
  const PrepareAdmissionProvider & admission_provider)
{
  reconcile_deadlines(now);

  // Keep this lookup local: PREPARE is the protocol entry point and must
  // prove that idempotence precedes all state mutation.
  if (const auto replay = replay_or_collision(request)) {
    return *replay;
  }

  ControlResult rejection;
  if (!validate_common(request, Operation::Prepare, false, rejection)) {
    return rejection;
  }
  if (state_ != State::Inhibited) {
    return reject(
      request, Reason::InvalidState,
      "PREPARE requires the inhibited state");
  }
  if (request.expected_control_seq != control_seq_) {
    return reject(
      request, Reason::StaleSequence,
      "PREPARE expected_control_seq is stale", false);
  }

  if (admission_provider) {
    PrepareAdmission admission;
    try {
      admission = admission_provider();
    } catch (const std::exception & error) {
      force_fault(
        Reason::InternalFailure,
        std::string{"prepare admission provider failed: "} + error.what());
      auto fault = result_from_snapshot(
        ResultCode::Faulted, reason_, detail_);
      remember(request, fault);
      return fault;
    } catch (...) {
      force_fault(
        Reason::InternalFailure,
        "prepare admission provider failed with an unknown exception");
      auto fault = result_from_snapshot(
        ResultCode::Faulted, reason_, detail_);
      remember(request, fault);
      return fault;
    }
    if (!admission.allowed) {
      const auto admission_reason =
        admission.reason == Reason::WriterStillPresent ?
        admission.reason : Reason::InternalFailure;
      return reject(
        request,
        admission_reason,
        admission.detail.empty() ?
        "PREPARE admission was rejected" : admission.detail);
    }
  }

  if (!advance_control_seq()) {
    auto fault = result_from_snapshot(
      ResultCode::Faulted, Reason::SequenceExhausted,
      "control sequence exhausted while preparing");
    remember(request, fault);
    return fault;
  }

  lease_id_ = make_lease_id(control_seq_);
  candidate_topic_ = make_candidate_topic(lease_id_);
  bound_writer_gid_.fill(0U);
  writer_bound_ = false;
  candidate_fresh_ = false;
  selected_ = zero_command();
  prepare_deadline_ = now + config_.prepare_timeout;
  authority_deadline_ = SteadyTimePoint{};
  candidate_deadline_ = SteadyTimePoint{};
  state_ = State::Prepared;
  reason_ = Reason::None;
  detail_ = "lease prepared";
  advance_state_seq();

  auto result = applied(request);
  remember(request, result);
  return result;
}

ControlResult MotionGateCore::open(
  const ControlRequest & request,
  SteadyTimePoint now,
  const OpenBindingProvider & binding_provider)
{
  reconcile_deadlines(now);
  if (const auto replay = replay_or_collision(request)) {
    return *replay;
  }

  ControlResult rejection;
  if (!validate_common(request, Operation::Open, true, rejection)) {
    return rejection;
  }
  if (state_ != State::Prepared) {
    return reject(
      request, Reason::InvalidState,
      "OPEN requires the prepared state");
  }
  if (request.expected_control_seq != control_seq_) {
    return reject(
      request, Reason::StaleSequence,
      "OPEN expected_control_seq is stale", false);
  }
  if (request.lease_id != lease_id_) {
    return reject(
      request, Reason::StaleLease,
      "OPEN lease_id is not the current prepared lease", false);
  }
  if (now >= prepare_deadline_) {
    retire_lease(Reason::PrepareExpired, "prepared lease expired");
    return reject(
      request, Reason::PrepareExpired,
      "OPEN reached the prepare deadline");
  }
  if (!binding_provider) {
    return reject(
      request, Reason::WriterUnavailable,
      "OPEN has no Gate-local writer binding provider");
  }

  OpenBinding binding;
  try {
    binding = binding_provider();
  } catch (const std::exception & error) {
    force_fault(
      Reason::InternalFailure,
      std::string{"writer binding provider failed: "} + error.what());
    auto fault = result_from_snapshot(
      ResultCode::Faulted, reason_, detail_);
    remember(request, fault);
    return fault;
  } catch (...) {
    force_fault(
      Reason::InternalFailure,
      "writer binding provider failed with an unknown exception");
    auto fault = result_from_snapshot(
      ResultCode::Faulted, reason_, detail_);
    remember(request, fault);
    return fault;
  }

  if (!binding.ready) {
    auto binding_reason = binding.reason;
    if (
      binding_reason != Reason::WriterUnavailable &&
      binding_reason != Reason::WriterAmbiguous &&
      binding_reason != Reason::WriterMismatch &&
      binding_reason != Reason::WriterStillPresent &&
      binding_reason != Reason::WriterMetadataPending)
    {
      binding_reason = Reason::WriterUnavailable;
    }
    return reject(
      request, binding_reason,
      binding.detail.empty() ? "candidate writer is not ready" :
      binding.detail);
  }
  if (binding.reason != Reason::None) {
    force_fault(
      Reason::InternalFailure,
      "writer binding provider returned ready with a non-NONE reason");
    auto fault = result_from_snapshot(
      ResultCode::Faulted, reason_, detail_);
    remember(request, fault);
    return fault;
  }
  if (!gid_is_nonzero(binding.writer_gid)) {
    return reject(
      request, Reason::WriterUnavailable,
      "candidate writer has an all-zero GID");
  }
  if (!advance_control_seq()) {
    auto fault = result_from_snapshot(
      ResultCode::Faulted, Reason::SequenceExhausted,
      "control sequence exhausted while opening");
    remember(request, fault);
    return fault;
  }

  bound_writer_gid_ = binding.writer_gid;
  writer_bound_ = true;
  state_ = State::Armed;
  authority_deadline_ = now + config_.authority_lease;
  candidate_deadline_ = now + config_.candidate_freshness;
  candidate_fresh_ = false;
  selected_ = zero_command();
  reason_ = Reason::None;
  detail_ = "lease armed; awaiting a fresh candidate";
  advance_state_seq();

  auto result = applied(request);
  remember(request, result);
  return result;
}

void MotionGateCore::start_armed_window(SteadyTimePoint now)
{
  if (state_ != State::Armed) {
    return;
  }
  authority_deadline_ = now + config_.authority_lease;
  candidate_deadline_ = now + config_.candidate_freshness;
  candidate_fresh_ = false;
  selected_ = zero_command();
  reason_ = Reason::None;
  detail_ = "lease armed; awaiting a fresh candidate";
  advance_state_seq();
}

ControlResult MotionGateCore::renew(
  const ControlRequest & request,
  SteadyTimePoint now)
{
  if (state_ == State::Prepared && now >= prepare_deadline_) {
    retire_lease(Reason::PrepareExpired, "prepared lease expired");
  } else if (state_ == State::Armed && now >= authority_deadline_) {
    retire_lease(Reason::AuthorityExpired, "authority lease expired");
  } else if (state_ == State::Armed && now >= candidate_deadline_) {
    retire_lease(Reason::CandidateExpired, "candidate freshness expired");
  }

  if (const auto replay = replay_or_collision(request)) {
    return *replay;
  }

  ControlResult rejection;
  if (!validate_common(request, Operation::Renew, true, rejection)) {
    return rejection;
  }
  if (state_ != State::Armed) {
    return reject(
      request, Reason::InvalidState,
      "RENEW requires the armed state");
  }
  if (request.expected_control_seq != control_seq_) {
    return reject(
      request, Reason::StaleSequence,
      "RENEW expected_control_seq is stale", false);
  }
  if (request.lease_id != lease_id_) {
    return reject(
      request, Reason::StaleLease,
      "RENEW lease_id is not the current armed lease", false);
  }
  if (now >= authority_deadline_) {
    retire_lease(Reason::AuthorityExpired, "authority lease expired");
    return reject(
      request, Reason::AuthorityExpired,
      "RENEW reached the authority deadline");
  }
  if (!advance_control_seq()) {
    auto fault = result_from_snapshot(
      ResultCode::Faulted, Reason::SequenceExhausted,
      "control sequence exhausted while renewing");
    remember(request, fault);
    return fault;
  }

  const auto authority_lease = config_.authority_lease;
  authority_deadline_ = now + authority_lease;
  if (!candidate_fresh_) {
    candidate_deadline_ = now + config_.candidate_freshness;
  }
  reason_ = Reason::None;
  detail_ = "authority renewed";
  advance_state_seq();

  auto result = applied(request);
  remember(request, result);
  return result;
}

ControlResult MotionGateCore::inhibit(
  const ControlRequest & request,
  SteadyTimePoint now)
{
  reconcile_deadlines(now);

  // Idempotent STOP retries are served before validating current state.
  if (const auto replay = replay_or_collision(request)) {
    return *replay;
  }

  ControlResult rejection;
  const bool lease_required = state_ != State::Inhibited;
  if (!validate_common(request, Operation::Inhibit, lease_required, rejection)) {
    return rejection;
  }
  if (state_ == State::Inhibited) {
    if (!advance_control_seq()) {
      auto fault = result_from_snapshot(
        ResultCode::Faulted, Reason::SequenceExhausted,
        "control sequence exhausted while inhibiting");
      remember(request, fault);
      return fault;
    }
    reason_ = Reason::None;
    detail_ = "inhibit reasserted";
    advance_state_seq();
    auto result = applied(request);
    remember(request, result);
    return result;
  }
  if (state_ != State::Prepared && state_ != State::Armed) {
    return reject(
      request, Reason::InvalidState,
      "INHIBIT requires the current prepared or armed lease");
  }
  if (request.expected_control_seq != control_seq_) {
    return reject(
      request, Reason::StaleSequence,
      "INHIBIT expected_control_seq is stale", false);
  }
  if (request.lease_id != lease_id_) {
    return reject(
      request, Reason::StaleLease,
      "INHIBIT lease_id is not the current lease", false);
  }

  retire_lease(Reason::None, "lease inhibited");
  auto result = applied(request);
  remember(request, result);
  return result;
}

CandidateResult MotionGateCore::accept_candidate(
  const Candidate & candidate,
  SteadyTimePoint now)
{
  reconcile_deadlines(now);

  if (!valid_identifier(candidate.lease_id)) {
    return CandidateResult{
      false, false, Reason::InvalidCandidate, zero_command()};
  }
  if (state_ != State::Armed || candidate.lease_id != lease_id_) {
    return CandidateResult{
      false, false, Reason::StaleLease, zero_command()};
  }

  if (
    candidate.from_intra_process ||
    !gid_is_nonzero(candidate.writer_gid) ||
    candidate.writer_gid != bound_writer_gid_)
  {
    retire_lease(
      Reason::WriterMismatch,
      "current candidate did not match the bound inter-process writer");
    return CandidateResult{
      false, true, Reason::WriterMismatch, zero_command()};
  }

  const bool all_finite =
    std::isfinite(candidate.linear_x) &&
    std::isfinite(candidate.linear_y) &&
    std::isfinite(candidate.linear_z) &&
    std::isfinite(candidate.angular_x) &&
    std::isfinite(candidate.angular_y) &&
    std::isfinite(candidate.angular_z);
  const bool unsupported_axes_are_zero =
    candidate.linear_y == 0.0 &&
    candidate.linear_z == 0.0 &&
    candidate.angular_x == 0.0 &&
    candidate.angular_y == 0.0;
  if (!all_finite || !unsupported_axes_are_zero) {
    retire_lease(
      Reason::InvalidCandidate,
      "current candidate contained a non-finite or unsupported axis");
    return CandidateResult{
      false, true, Reason::InvalidCandidate, zero_command()};
  }

  selected_.linear_x = std::clamp(
    candidate.linear_x, config_.linear_x_min, config_.linear_x_max);
  selected_.angular_z = std::clamp(
    candidate.angular_z, config_.angular_z_min, config_.angular_z_max);
  candidate_deadline_ = now + config_.candidate_freshness;
  candidate_fresh_ = true;
  reason_ = Reason::None;
  detail_ = "fresh candidate selected";
  advance_state_seq();

  return CandidateResult{true, false, Reason::None, selected_};
}

Command MotionGateCore::tick(SteadyTimePoint now)
{
  if (state_ == State::Prepared && now >= prepare_deadline_) {
    retire_lease(Reason::PrepareExpired, "prepared lease expired");
  } else if (state_ == State::Armed && now >= authority_deadline_) {
    retire_lease(Reason::AuthorityExpired, "authority lease expired");
  } else if (state_ == State::Armed && now >= candidate_deadline_) {
    retire_lease(Reason::CandidateExpired, "candidate freshness expired");
  }

  if (state_ != State::Armed || !candidate_fresh_) {
    return zero_command();
  }
  return selected_;
}

Snapshot MotionGateCore::snapshot() const
{
  const bool armed = state_ == State::Armed;
  return Snapshot{
    gate_instance_id_,
    state_seq_,
    control_seq_,
    state_,
    lease_id_,
    candidate_topic_,
    bound_writer_gid_,
    !armed,
    armed,
    armed && candidate_fresh_,
    armed && writer_bound_,
    !armed || !candidate_fresh_ || selected_.is_zero(),
    reason_,
    detail_};
}

Command MotionGateCore::selected_command() const noexcept
{
  if (state_ != State::Armed || !candidate_fresh_) {
    return zero_command();
  }
  return selected_;
}

void MotionGateCore::force_fault(Reason reason, std::string detail)
{
  if (state_ == State::Faulted) {
    return;
  }

  if (control_seq_ == std::numeric_limits<std::uint64_t>::max()) {
    reason = Reason::SequenceExhausted;
    detail = "control sequence exhausted while entering fault";
  } else {
    ++control_seq_;
  }

  state_ = State::Faulted;
  lease_id_.clear();
  candidate_topic_.clear();
  bound_writer_gid_.fill(0U);
  writer_bound_ = false;
  candidate_fresh_ = false;
  selected_ = zero_command();
  prepare_deadline_ = SteadyTimePoint{};
  authority_deadline_ = SteadyTimePoint{};
  candidate_deadline_ = SteadyTimePoint{};
  reason_ = reason == Reason::None ? Reason::InternalFailure : reason;
  detail_ = bounded_detail(
    detail.empty() ? "MotionGate faulted" : std::move(detail));
  advance_state_seq();
}

std::optional<ControlResult> MotionGateCore::replay_or_collision(
  const ControlRequest & request) const
{
  if (!valid_identifier(request.request_id)) {
    return std::nullopt;
  }
  const auto cached = request_id_cache_.find(request.request_id);
  if (cached == request_id_cache_.end()) {
    return std::nullopt;
  }
  if (cached->second.logical_fingerprint != logical_request_fingerprint(request)) {
    return result_from_snapshot(
      ResultCode::Rejected, Reason::RequestIdCollision,
      "request_id was reused with a different request body");
  }
  if (!cached->second.replayable) {
    return std::nullopt;
  }
  if (state_ == State::Faulted) {
    return result_from_snapshot(ResultCode::Faulted, reason_, detail_);
  }

  return result_from_snapshot(
    ResultCode::Duplicate,
    cached->second.result.reason,
    cached->second.result.detail);
}

void MotionGateCore::remember(
  const ControlRequest & request,
  const ControlResult & result,
  bool replayable)
{
  if (
    !valid_identifier(request.request_id) ||
    config_.request_cache_size == 0U)
  {
    return;
  }

  const auto logical_fingerprint = logical_request_fingerprint(request);
  const auto existing = request_id_cache_.find(request.request_id);
  if (existing != request_id_cache_.end()) {
    if (existing->second.logical_fingerprint == logical_fingerprint) {
      existing->second.result = result;
      existing->second.replayable = replayable;
    }
    return;
  }

  while (request_id_cache_.size() >= config_.request_cache_size) {
    const auto oldest = request_cache_order_.front();
    request_cache_order_.pop_front();
    request_id_cache_.erase(oldest);
  }

  request_cache_order_.push_back(request.request_id);
  request_id_cache_.emplace(
    request.request_id,
    CachedRequest{logical_fingerprint, result, replayable});
}

ControlResult MotionGateCore::reject(
  const ControlRequest & request,
  Reason reason,
  std::string detail,
  bool cache)
{
  const bool faulted = state_ == State::Faulted;
  auto result = result_from_snapshot(
    faulted ? ResultCode::Faulted : ResultCode::Rejected,
    faulted ? reason_ : reason,
    faulted ? detail_ : std::move(detail));
  const bool stale_tuple =
    reason == Reason::StaleGate || reason == Reason::StaleSequence ||
    reason == Reason::StaleLease;
  if (cache || stale_tuple) {
    remember(request, result, !stale_tuple);
  }
  return result;
}

ControlResult MotionGateCore::applied(const ControlRequest & request)
{
  (void)request;
  return result_from_snapshot(ResultCode::Applied, Reason::None, detail_);
}

ControlResult MotionGateCore::result_from_snapshot(
  ResultCode code,
  Reason reason,
  std::string detail) const
{
  const auto state = snapshot();
  return ControlResult{
    code,
    reason,
    state.gate_instance_id,
    state.control_seq,
    state.state,
    state.lease_id,
    state.candidate_topic,
    state.bound_writer_gid,
    state.motion_inhibited,
    state.authority_live,
    state.candidate_fresh,
    state.writer_bound,
    state.zero_selected,
    bounded_detail(std::move(detail))};
}

bool MotionGateCore::validate_common(
  const ControlRequest & request,
  Operation expected,
  bool lease_required,
  ControlResult & rejection)
{
  if (!valid_identifier(request.request_id)) {
    rejection = reject(
      request, Reason::InvalidRequest,
      "request_id must be exactly 32 lowercase hexadecimal characters",
      false);
    return false;
  }
  if (request.operation != expected) {
    rejection = reject(
      request, Reason::InvalidRequest,
      "operation does not match the selected control method");
    return false;
  }
  if (!valid_identifier(request.gate_instance_id)) {
    rejection = reject(
      request, Reason::InvalidRequest,
      "gate_instance_id must be exactly 32 lowercase hexadecimal characters");
    return false;
  }
  if (request.gate_instance_id != gate_instance_id_) {
    rejection = reject(
      request, Reason::StaleGate,
      "request targets a different Gate instance", false);
    return false;
  }
  if (state_ == State::Faulted) {
    rejection = reject(request, reason_, detail_);
    return false;
  }
  if (lease_required) {
    if (!valid_identifier(request.lease_id)) {
      rejection = reject(
        request, Reason::InvalidRequest,
        "lease_id must be exactly 32 lowercase hexadecimal characters");
      return false;
    }
  } else if (!request.lease_id.empty()) {
    rejection = reject(
      request, Reason::InvalidRequest,
      "this operation must not carry a lease_id");
    return false;
  }
  if (request.expected_control_seq != control_seq_) {
    rejection = reject(
      request, Reason::StaleSequence,
      "expected_control_seq does not match the Gate control sequence",
      false);
    return false;
  }
  return true;
}

bool MotionGateCore::advance_control_seq()
{
  if (control_seq_ == std::numeric_limits<std::uint64_t>::max()) {
    state_ = State::Faulted;
    lease_id_.clear();
    candidate_topic_.clear();
    bound_writer_gid_.fill(0U);
    writer_bound_ = false;
    candidate_fresh_ = false;
    selected_ = zero_command();
    reason_ = Reason::SequenceExhausted;
    detail_ = "control sequence exhausted";
    advance_state_seq();
    return false;
  }
  ++control_seq_;
  return true;
}

void MotionGateCore::advance_state_seq()
{
  if (state_seq_ != std::numeric_limits<std::uint64_t>::max()) {
    ++state_seq_;
  }
}

void MotionGateCore::reconcile_deadlines(SteadyTimePoint now)
{
  if (state_ == State::Prepared && now >= prepare_deadline_) {
    retire_lease(Reason::PrepareExpired, "prepared lease expired");
  } else if (state_ == State::Armed && now >= authority_deadline_) {
    retire_lease(Reason::AuthorityExpired, "authority lease expired");
  } else if (state_ == State::Armed && now >= candidate_deadline_) {
    retire_lease(Reason::CandidateExpired, "candidate freshness expired");
  }
}

void MotionGateCore::retire_lease(Reason reason, std::string detail)
{
  if (state_ == State::Faulted) {
    selected_ = zero_command();
    return;
  }
  if (state_ != State::Prepared && state_ != State::Armed) {
    selected_ = zero_command();
    return;
  }
  if (!advance_control_seq()) {
    return;
  }

  state_ = State::Inhibited;
  lease_id_.clear();
  candidate_topic_.clear();
  bound_writer_gid_.fill(0U);
  writer_bound_ = false;
  candidate_fresh_ = false;
  selected_ = zero_command();
  prepare_deadline_ = SteadyTimePoint{};
  authority_deadline_ = SteadyTimePoint{};
  candidate_deadline_ = SteadyTimePoint{};
  reason_ = reason;
  detail_ = bounded_detail(
    detail.empty() ? "lease retired" : std::move(detail));
  advance_state_seq();
}

std::string MotionGateCore::make_lease_id(
  std::uint64_t next_control_seq) const
{
  std::uint64_t lower_half = 0U;
  for (std::size_t index = 16U; index < gate_instance_id_.size(); ++index) {
    const auto character = gate_instance_id_[index];
    const auto nibble = character <= '9' ?
      static_cast<std::uint64_t>(character - '0') :
      static_cast<std::uint64_t>(character - 'a' + 10);
    lower_half = (lower_half << 4U) | nibble;
  }
  lower_half ^= next_control_seq;

  std::ostringstream stream;
  stream << gate_instance_id_.substr(0U, 16U)
         << std::hex << std::nouppercase << std::setfill('0')
         << std::setw(16) << lower_half;
  return stream.str();
}

std::string MotionGateCore::make_candidate_topic(
  const std::string & lease_id) const
{
  return std::string{kCandidateTopicPrefix} + lease_id;
}

std::string MotionGateCore::logical_request_fingerprint(
  const ControlRequest & request)
{
  // Authority tuple fields are intentionally excluded. The request ID is the
  // cache key; the operation is the only immutable business payload in the
  // private control protocol.
  return std::to_string(static_cast<std::uint8_t>(request.operation));
}

bool MotionGateCore::valid_identifier(const std::string & value)
{
  if (value.size() != 32U) {
    return false;
  }
  return std::all_of(
    value.begin(), value.end(),
    [](char character) {
      return (character >= '0' && character <= '9') ||
             (character >= 'a' && character <= 'f');
    });
}

bool MotionGateCore::gid_is_nonzero(const WriterGid & gid)
{
  return std::any_of(
    gid.begin(), gid.end(),
    [](std::uint8_t value) {return value != 0U;});
}

bool MotionGateCore::config_is_valid(const MotionGateConfig & config)
{
  const bool deadlines_valid =
    config.authority_lease.count() > 0 &&
    config.candidate_freshness.count() > 0 &&
    config.prepare_timeout.count() > 0 &&
    config.candidate_freshness < config.authority_lease &&
    config.authority_lease < std::chrono::milliseconds{350} &&
  config.prepare_timeout <= std::chrono::seconds{10};
  const bool limits_finite =
    std::isfinite(config.linear_x_min) &&
    std::isfinite(config.linear_x_max) &&
    std::isfinite(config.angular_z_min) &&
    std::isfinite(config.angular_z_max);
  const bool limits_ordered =
    config.linear_x_min >= -0.20 &&
    config.linear_x_min <= 0.0 &&
    config.linear_x_max >= 0.0 &&
    config.linear_x_max <= 0.40 &&
    config.linear_x_min <= config.linear_x_max &&
    config.angular_z_min >= -1.20 &&
    config.angular_z_min <= 0.0 &&
    config.angular_z_max >= 0.0 &&
    config.angular_z_max <= 1.20 &&
    config.angular_z_min <= config.angular_z_max;
  return deadlines_valid && limits_finite && limits_ordered &&
         config.request_cache_size > 0U &&
         config.request_cache_size <= 1024U;
}

Command MotionGateCore::zero_command() noexcept
{
  return Command{};
}

}  // namespace voice_nav_mission
