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

#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "voice_nav_mission/mission_authority_convergence.hpp"
#include "voice_nav_mission/motion_gate_core.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;
using Kind = AuthorityOperationKind;
using TimePoint = MissionAuthorityConvergence::TimePoint;

constexpr char kInitialGate[] = "gate-initial";
constexpr char kLatestGate[] = "gate-latest";
constexpr char kInitialLease[] = "lease-initial";
constexpr char kLatestLease[] = "lease-latest";

struct ScriptedClock
{
  TimePoint value{};

  [[nodiscard]] TimePoint now() const {return value;}
  void advance(std::chrono::milliseconds amount) {value += amount;}
};

AuthorityOperation operation(Kind kind)
{
  return AuthorityOperation{
    "logical-request-id", kInitialGate, 7U,
    kind == Kind::Prepare ? std::string{} : kInitialLease};
}

AuthorityResult response(
  bool applied,
  bool retryable,
  const char * gate_id = kInitialGate,
  std::uint64_t control_seq = 7U,
  const char * lease_id = kInitialLease,
  bool zero_proven = true,
  bool tuple_stale = false)
{
  GateSnapshot snapshot;
  snapshot.gate_instance_id = gate_id;
  snapshot.control_seq = control_seq;
  snapshot.lease_id = lease_id;
  snapshot.endpoint_available = true;
  snapshot.motion_inhibited = zero_proven;
  snapshot.zero_published = zero_proven;
  return AuthorityResult{
    applied, zero_proven, retryable, snapshot, lease_id, "scripted", tuple_stale};
}

constexpr char kCoreGateId[] = "0123456789abcdef0123456789abcdef";
constexpr char kOldCoreGateId[] = "fedcba9876543210fedcba9876543210";

std::string core_identifier(std::uint64_t value)
{
  std::ostringstream stream;
  stream << std::hex << std::nouppercase << std::setfill('0')
         << std::setw(32) << value;
  return stream.str();
}

WriterGid core_writer_gid()
{
  WriterGid gid{};
  gid.front() = 0x42U;
  gid.back() = 0xa5U;
  return gid;
}

Operation core_operation(AuthorityOperationKind kind)
{
  switch (kind) {
    case AuthorityOperationKind::Prepare:
      return Operation::Prepare;
    case AuthorityOperationKind::Open:
      return Operation::Open;
    case AuthorityOperationKind::Renew:
      return Operation::Renew;
    case AuthorityOperationKind::Inhibit:
      return Operation::Inhibit;
  }
  return Operation::Prepare;
}

GateSnapshot core_snapshot(const ControlResult & result)
{
  return GateSnapshot{
    result.gate_instance_id,
    result.control_seq,
    result.lease_id,
    result.state == State::Inhibited ? GateState::Inhibited :
    (result.state == State::Prepared ? GateState::Prepared :
    (result.state == State::Armed ? GateState::Armed : GateState::Faulted)),
    true,
    result.motion_inhibited,
    result.zero_selected,
    result.zero_selected};
}

GateSnapshot core_gate_snapshot(const voice_nav_mission::Snapshot & snapshot)
{
  return GateSnapshot{
    snapshot.gate_instance_id,
    snapshot.control_seq,
    snapshot.lease_id,
    snapshot.state == State::Inhibited ? GateState::Inhibited :
    (snapshot.state == State::Prepared ? GateState::Prepared :
    (snapshot.state == State::Armed ? GateState::Armed : GateState::Faulted)),
    true,
    snapshot.motion_inhibited,
    snapshot.zero_selected,
    snapshot.zero_selected};
}

AuthorityResult core_authority_result(const ControlResult & result)
{
  const bool stale =
    result.reason == Reason::StaleGate ||
    result.reason == Reason::StaleSequence ||
    result.reason == Reason::StaleLease;
  return AuthorityResult{
    result.code == ResultCode::Applied || result.code == ResultCode::Duplicate,
    result.motion_inhibited && result.zero_selected,
    stale,
    core_snapshot(result),
    result.lease_id,
    result.detail,
    stale};
}

ControlResult apply_core(
  MotionGateCore & gate,
  const AuthorityOperation & operation,
  AuthorityOperationKind kind,
  MotionGateCore::SteadyTimePoint now)
{
  const auto request = ControlRequest{
    core_operation(kind),
    operation.request_id,
    operation.gate_instance_id,
    operation.expected_control_seq,
    operation.lease_id};
  switch (kind) {
    case AuthorityOperationKind::Prepare:
      return gate.prepare(request, now);
    case AuthorityOperationKind::Open:
      return gate.open(request, now, []() {
                 return OpenBinding{true, Reason::None, core_writer_gid(), "ready"};
      });
    case AuthorityOperationKind::Renew:
      return gate.renew(request, now);
    case AuthorityOperationKind::Inhibit:
      return gate.inhibit(request, now);
  }
  return ControlResult{};
}

GateSnapshot setup_core(
  MotionGateCore & gate,
  AuthorityOperationKind kind,
  std::uint64_t request_number)
{
  if (kind == AuthorityOperationKind::Prepare) {
    return core_gate_snapshot(gate.snapshot());
  }
  const auto initial = gate.snapshot();
  const auto prepared = gate.prepare(ControlRequest{
        Operation::Prepare,
        core_identifier(request_number),
        initial.gate_instance_id,
        initial.control_seq,
        ""}, MotionGateCore::SteadyTimePoint{});
  if (kind == AuthorityOperationKind::Renew) {
    const auto state = gate.snapshot();
    (void)gate.open(ControlRequest{
          Operation::Open,
          core_identifier(request_number + 1U),
          state.gate_instance_id,
          state.control_seq,
          state.lease_id}, MotionGateCore::SteadyTimePoint{}, []() {
        return OpenBinding{true, Reason::None, core_writer_gid(), "ready"};
      });
  }
  (void)prepared;
  return core_gate_snapshot(gate.snapshot());
}

AuthorityOperation operation_from_snapshot(
  AuthorityOperationKind kind,
  const GateSnapshot & snapshot,
  std::uint64_t request_number,
  const std::string & gate_id = kCoreGateId)
{
  return AuthorityOperation{
    core_identifier(request_number),
    gate_id,
    snapshot.control_seq,
    kind == AuthorityOperationKind::Prepare ? std::string{} : snapshot.lease_id};
}

AuthorityResult run_adapter(
  MissionAuthorityAdapter & adapter,
  AuthorityOperationKind kind,
  const AuthorityOperation & operation)
{
  switch (kind) {
    case AuthorityOperationKind::Prepare:
      return adapter.prepare(operation);
    case AuthorityOperationKind::Open:
      return adapter.open(operation);
    case AuthorityOperationKind::Renew:
      return adapter.renew(operation);
    case AuthorityOperationKind::Inhibit:
      return adapter.inhibit(operation);
  }
  return AuthorityResult{};
}

TEST(MissionAuthorityConvergence, EveryOperationRebuildsStaleTuple)
{
  const std::array<Kind, 4> kinds{
    Kind::Prepare, Kind::Open, Kind::Renew, Kind::Inhibit};
  for (const auto kind : kinds) {
    ScriptedClock clock;
    const auto initial = operation(kind);
    std::vector<AuthorityOperation> calls;
    const auto result = MissionAuthorityConvergence::run(
      initial,
      kind,
      250ms,
      100ms,
      [&clock]() {return clock.now();},
      [&calls](
        const AuthorityOperation & current, TimePoint, TimePoint) {
        calls.push_back(current);
        if (calls.size() == 1U) {
          return response(
              false, true, kLatestGate, 8U, kLatestLease,
              current.lease_id.empty(), true);
        }
        return response(true, false, kLatestGate, 9U, kLatestLease);
        });

    ASSERT_TRUE(result.applied);
    ASSERT_EQ(calls.size(), 2U);
    EXPECT_EQ(calls[0].request_id, calls[1].request_id);
    EXPECT_EQ(calls[1].gate_instance_id, kLatestGate);
    EXPECT_EQ(calls[1].expected_control_seq, 8U);
    if (kind == Kind::Prepare) {
      EXPECT_TRUE(calls[1].lease_id.empty());
    } else {
      EXPECT_EQ(calls[1].lease_id, kLatestLease);
    }
  }
}

TEST(MissionAuthorityConvergence, TimeoutRetriesSamePayloadToApplied)
{
  const std::array<Kind, 4> kinds{
    Kind::Prepare, Kind::Open, Kind::Renew, Kind::Inhibit};
  for (const auto kind : kinds) {
    ScriptedClock clock;
    std::vector<AuthorityOperation> calls;
    const auto result = MissionAuthorityConvergence::run(
      operation(kind),
      kind,
      250ms,
      100ms,
      [&clock]() {return clock.now();},
      [&calls, &clock](
        const AuthorityOperation & current, TimePoint, TimePoint) {
        calls.push_back(current);
        if (calls.size() == 1U) {
          clock.advance(100ms);
          return response(false, true, kLatestGate, 8U, kLatestLease, true, false);
        }
        return response(true, false);
        });

    ASSERT_TRUE(result.applied);
    ASSERT_EQ(calls.size(), 2U);
    EXPECT_EQ(calls[0].request_id, calls[1].request_id);
    EXPECT_EQ(calls[0].gate_instance_id, calls[1].gate_instance_id);
    EXPECT_EQ(calls[0].expected_control_seq, calls[1].expected_control_seq);
    EXPECT_EQ(calls[0].lease_id, calls[1].lease_id);
  }
}

TEST(MissionAuthorityConvergence, DeadlineStopsRetryBeforeAnOutOfBudgetCall)
{
  ScriptedClock clock;
  std::size_t attempts = 0U;
  TimePoint observed_attempt_deadline{};
  const auto result = MissionAuthorityConvergence::run(
    operation(Kind::Renew),
    Kind::Renew,
    250ms,
    100ms,
    [&clock]() {return clock.now();},
    [&clock, &attempts, &observed_attempt_deadline](
      const AuthorityOperation &, TimePoint attempt_deadline, TimePoint) {
      ++attempts;
      observed_attempt_deadline = attempt_deadline;
      clock.advance(250ms);
      return response(false, true);
      });

  EXPECT_FALSE(result.applied);
  EXPECT_FALSE(result.retryable);
  EXPECT_EQ(attempts, 1U);
  EXPECT_EQ(observed_attempt_deadline, TimePoint{} + 100ms);
}

TEST(MissionAuthorityConvergence, AttemptDeadlineClampsToOverallRemainingBudget)
{
  ScriptedClock clock;
  std::vector<TimePoint> attempt_deadlines;
  const auto result = MissionAuthorityConvergence::run(
    operation(Kind::Renew),
    Kind::Renew,
    250ms,
    100ms,
    [&clock]() {return clock.now();},
    [&clock, &attempt_deadlines](
      const AuthorityOperation &, TimePoint attempt_deadline, TimePoint) {
      attempt_deadlines.push_back(attempt_deadline);
      if (attempt_deadlines.size() == 1U) {
        clock.advance(200ms);
        return response(false, true);
      }
      clock.advance(50ms);
      return response(false, true);
      });

  EXPECT_FALSE(result.applied);
  ASSERT_EQ(attempt_deadlines.size(), 2U);
  EXPECT_EQ(attempt_deadlines[0], TimePoint{} + 100ms);
  EXPECT_EQ(attempt_deadlines[1], TimePoint{} + 250ms);
}

TEST(MissionAuthorityAdapter, CoreWiringRebuildsStaleGateTupleForEveryOperation)
{
  const std::array<Kind, 4> kinds{
    Kind::Prepare, Kind::Open, Kind::Renew, Kind::Inhibit};
  for (const auto kind : kinds) {
    SCOPED_TRACE(static_cast<int>(kind));
    MotionGateCore gate(MotionGateConfig{}, kCoreGateId);
    const auto current_snapshot = setup_core(gate, kind, 1000U);
    auto initial = operation_from_snapshot(kind, current_snapshot, 2000U, kOldCoreGateId);
    std::vector<AuthorityOperation> calls;
    ScriptedClock clock;
    MissionAuthorityAdapter adapter(
      250ms,
      100ms,
      [&clock]() {return clock.now();},
      [&gate, &calls](
        const AuthorityOperation & operation,
        Kind operation_kind,
        TimePoint,
        TimePoint) {
        calls.push_back(operation);
        return core_authority_result(
          apply_core(gate, operation, operation_kind, MotionGateCore::SteadyTimePoint{}));
      });

    const auto result = run_adapter(adapter, kind, initial);

    ASSERT_TRUE(result.applied);
    ASSERT_EQ(calls.size(), 2U);
    EXPECT_EQ(calls[0].request_id, calls[1].request_id);
    EXPECT_EQ(calls[1].gate_instance_id, kCoreGateId);
    EXPECT_EQ(calls[1].expected_control_seq, current_snapshot.control_seq);
    if (kind == Kind::Prepare) {
      EXPECT_TRUE(calls[1].lease_id.empty());
    } else {
      EXPECT_EQ(calls[1].lease_id, current_snapshot.lease_id);
    }
  }
}

TEST(MissionAuthorityAdapter, CoreWiringRetriesTimeoutWithIdenticalPayload)
{
  const std::array<Kind, 4> kinds{
    Kind::Prepare, Kind::Open, Kind::Renew, Kind::Inhibit};
  for (const auto kind : kinds) {
    MotionGateCore gate(MotionGateConfig{}, kCoreGateId);
    const auto current_snapshot = setup_core(gate, kind, 3000U);
    const auto initial = operation_from_snapshot(kind, current_snapshot, 4000U);
    std::vector<AuthorityOperation> calls;
    ScriptedClock clock;
    MissionAuthorityAdapter adapter(
      250ms,
      100ms,
      [&clock]() {return clock.now();},
      [&gate, &calls](
        const AuthorityOperation & operation,
        Kind operation_kind,
        TimePoint,
        TimePoint) {
        calls.push_back(operation);
        const auto result = core_authority_result(
          apply_core(gate, operation, operation_kind, MotionGateCore::SteadyTimePoint{}));
        if (calls.size() == 1U) {
          auto timeout = result;
          timeout.applied = false;
          timeout.zero_proven = false;
          timeout.retryable = true;
          timeout.tuple_stale = false;
          timeout.detail = "scripted response timeout";
          return timeout;
        }
        return result;
      });

    const auto result = run_adapter(adapter, kind, initial);

    ASSERT_TRUE(result.applied);
    ASSERT_EQ(calls.size(), 2U);
    EXPECT_EQ(calls[0].request_id, calls[1].request_id);
    EXPECT_EQ(calls[0].gate_instance_id, calls[1].gate_instance_id);
    EXPECT_EQ(calls[0].expected_control_seq, calls[1].expected_control_seq);
    EXPECT_EQ(calls[0].lease_id, calls[1].lease_id);
  }
}

TEST(MissionAuthorityAdapter, CoreWiringRebuildsStaleSequenceAndLease)
{
  const std::array<Kind, 4> kinds{
    Kind::Prepare, Kind::Open, Kind::Renew, Kind::Inhibit};
  for (const auto kind : kinds) {
    MotionGateCore gate(MotionGateConfig{}, kCoreGateId);
    const auto current_snapshot = setup_core(gate, kind, 3500U);
    auto initial = operation_from_snapshot(kind, current_snapshot, 4500U);
    initial.expected_control_seq += 1U;
    std::vector<AuthorityOperation> calls;
    ScriptedClock clock;
    MissionAuthorityAdapter adapter(
      250ms,
      100ms,
      [&clock]() {return clock.now();},
      [&gate, &calls](
        const AuthorityOperation & operation,
        Kind operation_kind,
        TimePoint,
        TimePoint) {
        calls.push_back(operation);
        return core_authority_result(
          apply_core(gate, operation, operation_kind, MotionGateCore::SteadyTimePoint{}));
      });

    const auto result = run_adapter(adapter, kind, initial);

    ASSERT_TRUE(result.applied);
    ASSERT_EQ(calls.size(), 2U);
    EXPECT_EQ(calls[0].request_id, calls[1].request_id);
    EXPECT_EQ(calls[1].expected_control_seq, current_snapshot.control_seq);
  }

  const std::array<Kind, 3> lease_kinds{
    Kind::Open, Kind::Renew, Kind::Inhibit};
  for (const auto kind : lease_kinds) {
    MotionGateCore gate(MotionGateConfig{}, kCoreGateId);
    const auto current_snapshot = setup_core(gate, kind, 5500U);
    auto initial = operation_from_snapshot(kind, current_snapshot, 6500U);
    initial.lease_id = core_identifier(9999U);
    std::vector<AuthorityOperation> calls;
    ScriptedClock clock;
    MissionAuthorityAdapter adapter(
      250ms,
      100ms,
      [&clock]() {return clock.now();},
      [&gate, &calls](
        const AuthorityOperation & operation,
        Kind operation_kind,
        TimePoint,
        TimePoint) {
        calls.push_back(operation);
        return core_authority_result(
          apply_core(gate, operation, operation_kind, MotionGateCore::SteadyTimePoint{}));
      });

    const auto result = run_adapter(adapter, kind, initial);

    ASSERT_TRUE(result.applied);
    ASSERT_EQ(calls.size(), 2U);
    EXPECT_EQ(calls[0].request_id, calls[1].request_id);
    EXPECT_EQ(calls[1].lease_id, current_snapshot.lease_id);
  }
}

TEST(MissionAuthorityAdapter, GenerationBoundInhibitNeverSuppressesReplacementGate)
{
  const std::array<Kind, 3> replacement_states{
    Kind::Prepare, Kind::Open, Kind::Renew};
  for (const auto replacement_state : replacement_states) {
    SCOPED_TRACE(static_cast<int>(replacement_state));
    MotionGateCore replacement_gate(MotionGateConfig{}, kCoreGateId);
    const auto replacement_snapshot = setup_core(
      replacement_gate, replacement_state, 7500U);
    ASSERT_TRUE(
      replacement_snapshot.state == GateState::Inhibited ||
      replacement_snapshot.state == GateState::Prepared ||
      replacement_snapshot.state == GateState::Armed);

    AuthorityOperation generation_a_teardown{
      core_identifier(7600U),
      kOldCoreGateId,
      17U,
      core_identifier(7700U)};
    generation_a_teardown.gate_instance_bound = true;
    std::vector<AuthorityOperation> calls;
    ScriptedClock clock;
    MissionAuthorityAdapter adapter(
      250ms,
      100ms,
      [&clock]() {return clock.now();},
      [&replacement_gate, &calls](
        const AuthorityOperation & operation,
        Kind operation_kind,
        TimePoint,
        TimePoint) {
        calls.push_back(operation);
        return core_authority_result(
          apply_core(
            replacement_gate, operation, operation_kind,
            MotionGateCore::SteadyTimePoint{}));
      });

    const auto result = adapter.inhibit(generation_a_teardown);
    const auto final_snapshot = core_gate_snapshot(replacement_gate.snapshot());

    EXPECT_FALSE(result.applied);
    EXPECT_FALSE(result.zero_proven);
    ASSERT_EQ(calls.size(), 1U);
    EXPECT_EQ(calls.front().gate_instance_id, kOldCoreGateId);
    EXPECT_NE(calls.front().lease_id, replacement_snapshot.lease_id);
    EXPECT_EQ(final_snapshot.gate_instance_id, kCoreGateId);
    EXPECT_EQ(final_snapshot.state, replacement_snapshot.state);
    EXPECT_EQ(final_snapshot.lease_id, replacement_snapshot.lease_id);
    EXPECT_EQ(
      final_snapshot.motion_inhibited,
      replacement_snapshot.motion_inhibited);
  }
}

TEST(MissionAuthorityAdapter, GenerationBoundEmptyLeaseReassertsSameGateZero)
{
  MotionGateCore gate(MotionGateConfig{}, kCoreGateId);
  const auto initial_snapshot = core_gate_snapshot(gate.snapshot());
  AuthorityOperation teardown = operation_from_snapshot(
    Kind::Inhibit, initial_snapshot, 7800U);
  teardown.lease_id.clear();
  teardown.gate_instance_bound = true;
  std::vector<AuthorityOperation> calls;
  ScriptedClock clock;
  MissionAuthorityAdapter adapter(
    250ms,
    100ms,
    [&clock]() {return clock.now();},
    [&gate, &calls](
      const AuthorityOperation & operation,
      Kind operation_kind,
      TimePoint,
      TimePoint) {
      calls.push_back(operation);
      return core_authority_result(
        apply_core(
          gate, operation, operation_kind,
          MotionGateCore::SteadyTimePoint{}));
    });

  const auto result = adapter.inhibit(teardown);

  EXPECT_TRUE(result.applied);
  EXPECT_TRUE(result.zero_proven);
  ASSERT_EQ(calls.size(), 1U);
  EXPECT_EQ(calls.front().gate_instance_id, kCoreGateId);
  EXPECT_TRUE(calls.front().lease_id.empty());
  EXPECT_EQ(gate.snapshot().state, State::Inhibited);
}

TEST(MissionAuthorityAdapter, CoreWiringStopsAfterThe250msOverallDeadline)
{
  const std::array<Kind, 4> kinds{
    Kind::Prepare, Kind::Open, Kind::Renew, Kind::Inhibit};
  for (const auto kind : kinds) {
    MotionGateCore gate(MotionGateConfig{}, kCoreGateId);
    const auto current_snapshot = setup_core(gate, kind, 5000U);
    const auto initial = operation_from_snapshot(kind, current_snapshot, 6000U);
    ScriptedClock clock;
    std::size_t attempts = 0U;
    TimePoint observed_attempt_deadline{};
    MissionAuthorityAdapter adapter(
      250ms,
      100ms,
      [&clock]() {return clock.now();},
      [&gate, &clock, &attempts, &observed_attempt_deadline](
        const AuthorityOperation & operation,
        Kind operation_kind,
        TimePoint attempt_deadline,
        TimePoint) {
        ++attempts;
        observed_attempt_deadline = attempt_deadline;
        const auto result = core_authority_result(
          apply_core(gate, operation, operation_kind, MotionGateCore::SteadyTimePoint{}));
        clock.advance(250ms);
        auto timeout = result;
        timeout.applied = false;
        timeout.zero_proven = false;
        timeout.retryable = true;
        timeout.tuple_stale = false;
        timeout.detail = "scripted overall deadline exhaustion";
        return timeout;
      });

    const auto result = run_adapter(adapter, kind, initial);

    EXPECT_FALSE(result.applied);
    EXPECT_EQ(attempts, 1U);
    EXPECT_EQ(observed_attempt_deadline, TimePoint{} + 100ms);
  }
}

TEST(MissionAuthorityAdapter, CoreWiringKeepsCrossOperationCollision)
{
  MotionGateCore gate(MotionGateConfig{}, kCoreGateId);
  ScriptedClock clock;
  Reason last_reason = Reason::None;
  MissionAuthorityAdapter adapter(
    250ms,
    100ms,
    [&clock]() {return clock.now();},
    [&gate, &last_reason](
      const AuthorityOperation & operation,
      Kind kind,
      TimePoint,
      TimePoint) {
      const auto result = apply_core(
          gate, operation, kind, MotionGateCore::SteadyTimePoint{});
      last_reason = result.reason;
      return core_authority_result(result);
    });

  const auto initial = operation_from_snapshot(
    Kind::Prepare, core_gate_snapshot(gate.snapshot()), 7000U);
  ASSERT_TRUE(adapter.prepare(initial).applied);
  const auto prepared = gate.snapshot();
  const auto collision = AuthorityOperation{
    initial.request_id,
    prepared.gate_instance_id,
    prepared.control_seq,
    prepared.lease_id};
  const auto result = adapter.open(collision);

  EXPECT_FALSE(result.applied);
  EXPECT_FALSE(result.retryable);
  EXPECT_EQ(last_reason, Reason::RequestIdCollision);
}

}  // namespace
}  // namespace voice_nav_mission
