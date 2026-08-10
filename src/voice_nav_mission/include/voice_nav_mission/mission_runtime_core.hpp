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

#ifndef VOICE_NAV_MISSION__MISSION_RUNTIME_CORE_HPP_
#define VOICE_NAV_MISSION__MISSION_RUNTIME_CORE_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace voice_nav_mission
{

inline constexpr std::uint32_t kNoActiveMissionStep =
  std::numeric_limits<std::uint32_t>::max();
inline constexpr std::uint32_t kSupportedMissionStepMask = 3U;

enum class OperatingMode : std::uint8_t
{
  Mapping = 1,
  Navigation = 2,
};

enum class RuntimeAvailability : std::uint8_t
{
  Unavailable = 0,
  Available = 1,
  Busy = 2,
  Faulted = 3,
};

enum class GateState : std::uint8_t
{
  Inhibited = 0,
  Prepared = 1,
  Armed = 2,
  Faulted = 3,
};

enum class MissionStepKind : std::uint8_t
{
  MoveDistance = 1,
  RotateAngle = 2,
  NavigateTo = 3,
  SaveMap = 4,
};

enum class MissionResultCode : std::uint16_t
{
  Succeeded = 0,
  InvalidPlan = 10,
  Busy = 11,
  ModeMismatch = 12,
  UnknownTarget = 13,
  StaleRequest = 14,
  UnsupportedStep = 15,
  DependencyUnavailable = 20,
  ExecutionFailed = 21,
  Timeout = 22,
  Canceled = 30,
  Stopped = 31,
  SafetyFault = 32,
  InternalError = 99,
};

enum class FeedbackPhase : std::uint8_t
{
  Validating = 1,
  Executing = 2,
  SafeStopping = 3,
};

struct MissionStep
{
  std::uint8_t kind{0U};
  float distance_m{0.0F};
  float angle_rad{0.0F};
  std::string target_id;
};

struct MissionGoal
{
  std::string source_instance_id;
  std::uint64_t source_seq{0U};
  std::string runtime_instance_id;
  std::uint64_t admission_epoch{0U};
  std::vector<MissionStep> steps;
};

struct MissionResult
{
  MissionResultCode code{MissionResultCode::InternalError};
  std::int32_t failed_step{-1};
  std::string detail;
};

struct MissionFeedback
{
  FeedbackPhase phase{FeedbackPhase::Validating};
  std::uint32_t step_index{0U};
  float progress{0.0F};
};

struct StopRequest
{
  std::string request_id;
  std::string source_instance_id;
  std::uint64_t source_seq{0U};
  std::string reason;
};

struct StopResponse
{
  std::uint16_t code{2U};
  std::string runtime_instance_id;
  std::uint64_t admission_epoch{0U};
  bool motion_inhibited{false};
  std::string detail;
};

struct RuntimeState
{
  std::string runtime_instance_id;
  std::uint64_t admission_epoch{1U};
  OperatingMode operating_mode{OperatingMode::Mapping};
  RuntimeAvailability availability{RuntimeAvailability::Unavailable};
  GateState gate_state{GateState::Faulted};
  std::uint32_t active_step{kNoActiveMissionStep};
  std::uint32_t supported_step_mask{kSupportedMissionStepMask};
  std::uint8_t max_steps{3U};
  std::vector<std::string> named_place_ids;
};

struct GateSnapshot
{
  std::string gate_instance_id;
  std::uint64_t control_seq{0U};
  std::string lease_id;
  GateState state{GateState::Faulted};
  bool endpoint_available{false};
  bool motion_inhibited{true};
  bool zero_selected{true};
  bool zero_published{false};
  std::string candidate_topic;
  bool authority_live{false};
  bool writer_bound{false};
};

struct AuthorityOperation
{
  std::string request_id;
  std::string gate_instance_id;
  std::uint64_t expected_control_seq{0U};
  std::string lease_id;
};

struct AuthorityResult
{
  bool applied{false};
  bool zero_proven{false};
  bool retryable{false};
  GateSnapshot snapshot;
  std::string lease_id;
  std::string detail;
  bool tuple_stale{false};
};

class SteadyClockPort
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  virtual ~SteadyClockPort() = default;
  [[nodiscard]] virtual TimePoint now() const = 0;
};

class MotionAuthorityPort
{
public:
  virtual ~MotionAuthorityPort() = default;

  [[nodiscard]] virtual GateSnapshot snapshot() const = 0;
  [[nodiscard]] virtual AuthorityResult prepare(
    const AuthorityOperation & operation) = 0;
  [[nodiscard]] virtual AuthorityResult open(
    const AuthorityOperation & operation) = 0;
  [[nodiscard]] virtual AuthorityResult renew(
    const AuthorityOperation & operation) = 0;
  [[nodiscard]] virtual AuthorityResult inhibit(
    const AuthorityOperation & operation) = 0;
};

struct MotionToken
{
  std::uint64_t mission_id{0U};
  std::uint64_t admission_epoch{0U};
  std::uint64_t mission_generation{0U};
  std::uint64_t step_generation{0U};
  std::uint64_t admission_generation{0U};
};

enum class ChildResultCode : std::uint8_t
{
  Succeeded = 0,
  Failed = 1,
  Timeout = 2,
  DependencyUnavailable = 3,
  SafetyFault = 4,
  InternalError = 5,
};

struct ChildResult
{
  ChildResultCode code{ChildResultCode::Failed};
  std::string detail;
};

class RelativeMotionPort
{
public:
  using FeedbackCallback = std::function<void(const MotionToken &, double)>;
  using ResultCallback = std::function<void(const MotionToken &, const ChildResult &)>;

  virtual ~RelativeMotionPort() = default;
  [[nodiscard]] virtual bool healthy() const = 0;
  virtual void start(
    const MotionToken & token,
    const MissionStep & step,
    FeedbackCallback feedback,
    ResultCallback result) = 0;
  [[nodiscard]] virtual bool cancel(
    const MotionToken & token,
    SteadyClockPort::TimePoint deadline) = 0;
  virtual void tick(SteadyClockPort::TimePoint now) = 0;

  // The production ROS Adapter owns the #35 conditioning lifecycle. The
  // default keeps deterministic package-private fakes compatible with the
  // existing Core contract.
  [[nodiscard]] virtual bool owns_authority_lifecycle() const noexcept
  {
    return false;
  }

  // Production adapters publish a pure-data completion record to a
  // Node-owned registry.  Deterministic package-private ports retain the
  // legacy callback path so the ROS-free Core tests remain small.
  [[nodiscard]] virtual bool uses_external_completion_registry() const noexcept
  {
    return false;
  }

  [[nodiscard]] virtual bool zero_proven() const noexcept
  {
    return true;
  }

  // A production adapter may be permanently safety-faulted while it performs
  // bounded startup reconciliation.  RuntimeCore keeps that distinction from
  // an ordinary dependency-unavailable condition so availability is latched
  // Faulted and admission cannot reopen.
  [[nodiscard]] virtual bool safety_faulted() const noexcept
  {
    return false;
  }

  // A replacement MotionGate may re-establish an inhibited, published zero
  // after the previous Gate process disappeared.  Production adapters may
  // release only the narrowly-scoped Gate-loss latch through this seam; the
  // default keeps deterministic ports fail-closed.
  [[nodiscard]] virtual bool rearm_after_gate_replacement(
    const GateSnapshot &) noexcept
  {
    return false;
  }
};

struct RuntimeConfig
{
  OperatingMode operating_mode{OperatingMode::Mapping};
  std::chrono::milliseconds mission_deadline{30000};
  std::chrono::milliseconds gate_discovery_deadline{2000};
  std::chrono::milliseconds control_response_deadline{100};
  std::chrono::milliseconds stop_barrier{250};
  std::chrono::milliseconds cancel_grace{250};
  std::chrono::milliseconds stationarity_deadline{1200};
  std::size_t source_cache_size{64U};
  std::size_t stop_cache_size{64U};
  std::uint8_t max_steps{3U};
  std::uint64_t initial_admission_epoch{1U};
  float move_distance_min_m{0.05F};
  float move_distance_max_m{2.0F};
  float rotate_angle_min_rad{0.05F};
  float rotate_angle_max_rad{6.283185F};
  std::vector<std::string> named_place_ids;
  std::string runtime_instance_id;
  std::function<std::string()> identifier_generator;
};

struct AdmissionResult
{
  std::uint64_t mission_id{0U};
  bool accepted{false};
  MissionResult result;
};

class RuntimeCore final
{
public:
  using StateCallback = std::function<void(const RuntimeState &)>;
  using FeedbackCallback = std::function<void(
        std::uint64_t, const MissionFeedback &)>;
  using ResultCallback = std::function<void(
        std::uint64_t, const MissionResult &)>;
  using ChildFeedbackDispatcher = std::function<bool(const MotionToken &, double)>;
  using ChildResultDispatcher = std::function<bool(
        const MotionToken &, const ChildResult &)>;
  using ChildResultDelivery = std::function<void(
        const MotionToken &, const ChildResult &)>;
  using ChildResultRegistrar = std::function<bool(
        const MotionToken &, ChildResultDelivery)>;
  using ChildResultUnregistrar = std::function<void(const MotionToken &)>;
  using AdmissionFenceCheck = std::function<bool(std::uint64_t)>;
  using StartPermitCheck = std::function<bool()>;

  RuntimeCore(
    RuntimeConfig config,
    std::shared_ptr<SteadyClockPort> clock,
    std::shared_ptr<MotionAuthorityPort> authority,
    std::shared_ptr<RelativeMotionPort> relative_motion,
    StateCallback state_callback = {},
    FeedbackCallback feedback_callback = {},
    ResultCallback result_callback = {},
    ChildFeedbackDispatcher child_feedback_dispatcher = {},
    ChildResultDispatcher child_result_dispatcher = {},
    AdmissionFenceCheck admission_fence_check = {},
    ChildResultRegistrar child_result_registrar = {},
    ChildResultUnregistrar child_result_unregistrar = {});

  [[nodiscard]] AdmissionResult admit(
    const MissionGoal & goal,
    StartPermitCheck start_permit_check = {},
    std::uint64_t admission_generation = 0U);
  void cancel(std::uint64_t mission_id);
  [[nodiscard]] StopResponse stop(const StopRequest & request);
  void observe_gate(const GateSnapshot & snapshot);
  void observe_dependencies();
  void on_tick();
  void on_child_feedback(const MotionToken & token, double progress);
  void on_child_result(const MotionToken & token, const ChildResult & result);
  void fail_closed(std::string detail);
  void fail_closed_at_epoch(std::uint64_t admission_epoch, std::string detail);

  [[nodiscard]] RuntimeState state() const;
  [[nodiscard]] bool usable() const noexcept;
  [[nodiscard]] bool has_active_mission() const noexcept;

private:
  struct ActiveMission
  {
    std::uint64_t id{0U};
    std::uint64_t generation{0U};
    std::uint64_t step_generation{0U};
    std::uint32_t step_index{0U};
    std::uint32_t completed_steps{0U};
    std::uint64_t admission_epoch{0U};
    std::uint64_t admission_generation{0U};
    std::vector<MissionStep> steps;
    SteadyClockPort::TimePoint deadline{};
    MotionToken child_token;
    StartPermitCheck start_permit_check;
    bool child_started{false};
    bool terminal_selected{false};
  };

  struct StopCacheEntry
  {
    std::string fingerprint;
  };

  struct TerminalOutcome
  {
    bool zero_proven{false};
    bool cancel_acknowledged{false};
    bool epoch_advanced{true};
  };

  [[nodiscard]] MissionResult reject(
    MissionResultCode code,
    std::string detail) const;
  [[nodiscard]] bool validate_goal(
    const MissionGoal & goal,
    MissionResult & result) const;
  [[nodiscard]] bool validate_step(
    const MissionStep & step,
    MissionResult & result) const;
  [[nodiscard]] bool consume_source_sequence(
    const MissionGoal & goal,
    MissionResult & result);
  [[nodiscard]] bool gate_is_healthy(const GateSnapshot & snapshot) const;
  [[nodiscard]] bool startup_gate_is_ready(const GateSnapshot & snapshot) const;
  [[nodiscard]] bool zero_is_proven(const GateSnapshot & snapshot) const;
  [[nodiscard]] AuthorityOperation make_operation(
    const std::string & lease_id = {}) const;
  [[nodiscard]] std::string new_identifier() const;
  [[nodiscard]] bool rotate_epoch();
  [[nodiscard]] bool admission_allowed(std::uint64_t admission_epoch) const;
  void set_availability_from_dependencies(bool allow_fault_rearm = false);
  void publish_state();
  void publish_feedback(
    FeedbackPhase phase,
    std::uint32_t step_index,
    double progress);
  void start_step();
  TerminalOutcome select_terminal_and_stop(
    MissionResultCode code,
    std::string detail,
    bool rotate_epoch = false);
  void finish_active(const MissionResult & result);
  [[nodiscard]] bool inhibit_and_prove_zero();
  [[nodiscard]] StopResponse make_stop_response(
    std::uint16_t code,
    bool motion_inhibited,
    std::string detail) const;
  [[nodiscard]] static std::string stop_fingerprint(
    const StopRequest & request);
  [[nodiscard]] static bool valid_bounded_id(
    const std::string & value,
    std::size_t maximum,
    bool allow_empty);
  [[nodiscard]] static bool increment_epoch(std::uint64_t & epoch);

  RuntimeConfig config_;
  std::shared_ptr<SteadyClockPort> clock_;
  std::shared_ptr<MotionAuthorityPort> authority_;
  std::shared_ptr<RelativeMotionPort> relative_motion_;
  StateCallback state_callback_;
  FeedbackCallback feedback_callback_;
  ResultCallback result_callback_;
  ChildFeedbackDispatcher child_feedback_dispatcher_;
  ChildResultDispatcher child_result_dispatcher_;
  AdmissionFenceCheck admission_fence_check_;
  ChildResultRegistrar child_result_registrar_;
  ChildResultUnregistrar child_result_unregistrar_;
  RuntimeState state_;
  GateSnapshot gate_snapshot_;
  bool gate_bound_{false};
  bool gate_fault_handled_{false};
  bool gate_replacement_pending_{false};
  std::optional<GateSnapshot> gate_fault_snapshot_;
  bool relative_health_initialized_{false};
  bool last_relative_healthy_{false};
  SteadyClockPort::TimePoint gate_discovery_started_at_{};
  bool gate_discovery_timed_out_{false};
  std::map<std::string, std::uint64_t> source_sequences_;
  std::map<std::string, StopCacheEntry> stop_cache_;
  std::optional<ActiveMission> active_;
  std::uint64_t next_mission_id_{1U};
  std::uint64_t next_generation_{1U};
  double last_feedback_progress_{0.0};
  std::string current_lease_id_;
};

// Deterministic fakes are intentionally package-private and are not installed.
class ScriptedSteadyClock final : public SteadyClockPort
{
public:
  [[nodiscard]] TimePoint now() const override {return now_;}
  void set(TimePoint value) {now_ = value;}
  void advance(std::chrono::milliseconds amount) {now_ += amount;}

private:
  TimePoint now_{};
};

class ScriptedMotionAuthorityPort final : public MotionAuthorityPort
{
public:
  explicit ScriptedMotionAuthorityPort(std::string gate_instance_id);

  [[nodiscard]] GateSnapshot snapshot() const override {return snapshot_;}
  [[nodiscard]] AuthorityResult prepare(
    const AuthorityOperation & operation) override;
  [[nodiscard]] AuthorityResult open(
    const AuthorityOperation & operation) override;
  [[nodiscard]] AuthorityResult renew(
    const AuthorityOperation & operation) override;
  [[nodiscard]] AuthorityResult inhibit(
    const AuthorityOperation & operation) override;

  void set_next_failure(std::string detail);
  void set_open_failure(std::string detail)
  {
    next_open_failure_ = std::move(detail);
  }
  void set_inhibit_failure(std::string detail)
  {
    next_inhibit_failure_ = std::move(detail);
  }
  void set_snapshot(GateSnapshot snapshot) {snapshot_ = std::move(snapshot);}
  void set_inhibit_observer(std::function<void()> observer)
  {
    inhibit_observer_ = std::move(observer);
  }
  [[nodiscard]] std::size_t inhibit_count() const noexcept {return inhibit_count_;}
  [[nodiscard]] const std::vector<AuthorityOperation> & operations() const noexcept
  {
    return operations_;
  }

private:
  [[nodiscard]] AuthorityResult apply(
    const AuthorityOperation & operation,
    GateState next_state,
    bool inhibited);

  GateSnapshot snapshot_;
  std::vector<AuthorityOperation> operations_;
  std::string next_failure_;
  std::string next_open_failure_;
  std::string next_inhibit_failure_;
  std::function<void()> inhibit_observer_;
  std::size_t inhibit_count_{0U};
};

class ScriptedRelativeMotionPort final : public RelativeMotionPort
{
public:
  [[nodiscard]] bool healthy() const override {return healthy_;}
  [[nodiscard]] bool safety_faulted() const noexcept override
  {
    return safety_faulted_;
  }
  [[nodiscard]] bool rearm_after_gate_replacement(
    const GateSnapshot &) noexcept override
  {
    return rearm_after_gate_replacement_;
  }
  void start(
    const MotionToken & token,
    const MissionStep & step,
    FeedbackCallback feedback,
    ResultCallback result) override;
  [[nodiscard]] bool cancel(
    const MotionToken & token,
    SteadyClockPort::TimePoint deadline) override;
  void tick(SteadyClockPort::TimePoint) override {}

  void set_healthy(bool value) {healthy_ = value;}
  void set_safety_faulted(bool value) {safety_faulted_ = value;}
  void set_rearm_after_gate_replacement(bool value)
  {
    rearm_after_gate_replacement_ = value;
  }
  void set_cancel_acknowledged(bool value) {cancel_acknowledged_ = value;}
  void set_start_completion(bool value) {start_completion_ = value;}
  void set_start_failure(std::string detail) {next_start_failure_ = std::move(detail);}
  void set_cancel_observer(std::function<void()> observer)
  {
    cancel_observer_ = std::move(observer);
  }
  void feedback(double progress);
  void complete();
  void fail(std::string detail = "scripted child failure");
  void timeout(std::string detail = "scripted child timeout");
  void complete_token(const MotionToken & token);
  void feedback_token(const MotionToken & token, double progress);
  [[nodiscard]] const std::vector<MissionStep> & started_steps() const noexcept
  {
    return started_steps_;
  }
  [[nodiscard]] std::vector<MotionToken> started_tokens() const
  {
    std::vector<MotionToken> tokens;
    tokens.reserve(children_.size());
    for (const auto & child : children_) {
      tokens.push_back(child.token);
    }
    return tokens;
  }
  [[nodiscard]] std::size_t cancel_count() const noexcept {return cancel_count_;}
  [[nodiscard]] const MotionToken & cancel_token() const noexcept
  {
    return cancel_token_;
  }
  [[nodiscard]] SteadyClockPort::TimePoint cancel_deadline() const noexcept
  {
    return cancel_deadline_;
  }

private:
  struct ChildCallbacks
  {
    MotionToken token;
    FeedbackCallback feedback;
    ResultCallback result;
  };

  bool healthy_{true};
  bool safety_faulted_{false};
  bool rearm_after_gate_replacement_{false};
  bool cancel_acknowledged_{true};
  bool start_completion_{false};
  std::string next_start_failure_;
  std::function<void()> cancel_observer_;
  std::optional<MotionToken> token_;
  FeedbackCallback feedback_callback_;
  ResultCallback result_callback_;
  std::vector<ChildCallbacks> children_;
  std::vector<MissionStep> started_steps_;
  std::size_t cancel_count_{0U};
  MotionToken cancel_token_{};
  SteadyClockPort::TimePoint cancel_deadline_{};
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__MISSION_RUNTIME_CORE_HPP_
