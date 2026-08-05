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

#ifndef VOICE_NAV_MISSION__MOTION_GATE_CORE_HPP_
#define VOICE_NAV_MISSION__MOTION_GATE_CORE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>

namespace voice_nav_mission
{

inline constexpr std::size_t kWriterGidSize = 16U;
using WriterGid = std::array<std::uint8_t, kWriterGidSize>;

enum class State : std::uint8_t
{
  Inhibited = 0,
  Prepared = 1,
  Armed = 2,
  Faulted = 3,
};

enum class Operation : std::uint8_t
{
  Prepare = 1,
  Open = 2,
  Renew = 3,
  Inhibit = 4,
};

enum class ResultCode : std::uint16_t
{
  Applied = 0,
  Duplicate = 1,
  Rejected = 2,
  Faulted = 3,
};

enum class Reason : std::uint16_t
{
  None = 0,
  InvalidRequest = 1,
  StaleGate = 2,
  StaleSequence = 3,
  InvalidState = 4,
  StaleLease = 5,
  RequestIdCollision = 6,
  PrepareExpired = 7,
  AuthorityExpired = 8,
  CandidateExpired = 9,
  WriterUnavailable = 10,
  WriterAmbiguous = 11,
  WriterMismatch = 12,
  WriterStillPresent = 13,
  InvalidCandidate = 14,
  SequenceExhausted = 15,
  ConfigurationInvalid = 16,
  PublishFailed = 17,
  InternalFailure = 18,
  WriterMetadataPending = 19,
};

struct MotionGateConfig
{
  std::chrono::milliseconds authority_lease{250};
  std::chrono::milliseconds candidate_freshness{150};
  std::chrono::milliseconds prepare_timeout{1000};
  double linear_x_min{-0.20};
  double linear_x_max{0.40};
  double angular_z_min{-1.20};
  double angular_z_max{1.20};
  std::size_t request_cache_size{64U};
};

struct ControlRequest
{
  Operation operation{Operation::Prepare};
  std::string request_id;
  std::string gate_instance_id;
  std::uint64_t expected_control_seq{0U};
  std::string lease_id;
};

struct ControlResult
{
  ResultCode code{ResultCode::Rejected};
  Reason reason{Reason::InvalidRequest};
  std::string gate_instance_id;
  std::uint64_t control_seq{0U};
  State state{State::Inhibited};
  std::string lease_id;
  std::string candidate_topic;
  WriterGid bound_writer_gid{};
  bool motion_inhibited{true};
  bool authority_live{false};
  bool candidate_fresh{false};
  bool writer_bound{false};
  bool zero_selected{true};
  std::string detail;
};

struct OpenBinding
{
  bool ready{false};
  Reason reason{Reason::WriterUnavailable};
  WriterGid writer_gid{};
  std::string detail;
};

struct PrepareAdmission
{
  bool allowed{true};
  Reason reason{Reason::None};
  std::string detail;
};

using PrepareAdmissionProvider = std::function<PrepareAdmission()>;
using OpenBindingProvider = std::function<OpenBinding()>;

struct Candidate
{
  std::string lease_id;
  WriterGid writer_gid{};
  bool from_intra_process{false};
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
};

struct Command
{
  double linear_x{0.0};
  double angular_z{0.0};

  [[nodiscard]] bool is_zero() const noexcept;
};

struct CandidateResult
{
  bool accepted{false};
  bool retired{false};
  Reason reason{Reason::None};
  Command selected{};
};

struct Snapshot
{
  std::string gate_instance_id;
  std::uint64_t state_seq{0U};
  std::uint64_t control_seq{0U};
  State state{State::Inhibited};
  std::string lease_id;
  std::string candidate_topic;
  WriterGid bound_writer_gid{};
  bool motion_inhibited{true};
  bool authority_live{false};
  bool candidate_fresh{false};
  bool writer_bound{false};
  bool zero_selected{true};
  Reason reason{Reason::None};
  std::string detail;
};

class MotionGateCore
{
public:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;

  MotionGateCore(
    MotionGateConfig config,
    std::string gate_instance_id,
    std::uint64_t initial_control_seq = 0U);

  [[nodiscard]] ControlResult prepare(
    const ControlRequest & request,
    SteadyTimePoint now,
    const PrepareAdmissionProvider & admission_provider = {});

  [[nodiscard]] ControlResult open(
    const ControlRequest & request,
    SteadyTimePoint now,
    const OpenBindingProvider & binding_provider);

  [[nodiscard]] ControlResult renew(
    const ControlRequest & request,
    SteadyTimePoint now);

  [[nodiscard]] ControlResult inhibit(
    const ControlRequest & request,
    SteadyTimePoint now);

  [[nodiscard]] CandidateResult accept_candidate(
    const Candidate & candidate,
    SteadyTimePoint now);

  [[nodiscard]] Command tick(SteadyTimePoint now);
  [[nodiscard]] Snapshot snapshot() const;
  [[nodiscard]] Command selected_command() const noexcept;

  void force_fault(Reason reason, std::string detail);

private:
  struct CachedRequest
  {
    std::string logical_fingerprint;
    ControlResult result;
    bool replayable{true};
  };

  [[nodiscard]] std::optional<ControlResult> replay_or_collision(
    const ControlRequest & request) const;
  void remember(
    const ControlRequest & request,
    const ControlResult & result,
    bool replayable = true);
  [[nodiscard]] ControlResult reject(
    const ControlRequest & request,
    Reason reason,
    std::string detail,
    bool cache = true);
  [[nodiscard]] ControlResult applied(
    const ControlRequest & request);
  [[nodiscard]] ControlResult result_from_snapshot(
    ResultCode code,
    Reason reason,
    std::string detail) const;
  [[nodiscard]] bool validate_common(
    const ControlRequest & request,
    Operation expected,
    bool lease_required,
    ControlResult & rejection);
  [[nodiscard]] bool advance_control_seq();
  void advance_state_seq();
  void reconcile_deadlines(SteadyTimePoint now);
  void retire_lease(Reason reason, std::string detail);
  [[nodiscard]] std::string make_lease_id(
    std::uint64_t next_control_seq) const;
  [[nodiscard]] std::string make_candidate_topic(
    const std::string & lease_id) const;
  [[nodiscard]] static std::string logical_request_fingerprint(
    const ControlRequest & request);
  [[nodiscard]] static bool valid_identifier(const std::string & value);
  [[nodiscard]] static bool gid_is_nonzero(const WriterGid & gid);
  [[nodiscard]] static bool config_is_valid(
    const MotionGateConfig & config);
  [[nodiscard]] static Command zero_command() noexcept;

  MotionGateConfig config_;
  std::string gate_instance_id_;
  State state_{State::Inhibited};
  std::uint64_t state_seq_{0U};
  std::uint64_t control_seq_{0U};
  std::string lease_id_;
  std::string candidate_topic_;
  WriterGid bound_writer_gid_{};
  bool writer_bound_{false};
  bool candidate_fresh_{false};
  Command selected_{};
  Reason reason_{Reason::None};
  std::string detail_;
  SteadyTimePoint prepare_deadline_{};
  SteadyTimePoint authority_deadline_{};
  SteadyTimePoint candidate_deadline_{};
  std::unordered_map<std::string, CachedRequest> request_id_cache_;
  std::deque<std::string> request_cache_order_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__MOTION_GATE_CORE_HPP_
