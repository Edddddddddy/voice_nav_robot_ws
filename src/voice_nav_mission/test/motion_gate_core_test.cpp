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

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;

constexpr char kGateId[] = "0123456789abcdef0123456789abcdef";
constexpr char kRequestId[] = "00000000000000000000000000000001";
constexpr char kOtherGateId[] = "fedcba9876543210fedcba9876543210";

MotionGateCore::SteadyTimePoint at(std::chrono::milliseconds offset)
{
  return MotionGateCore::SteadyTimePoint{} + offset;
}

ControlRequest prepare_request(
  const std::string & request_id = kRequestId,
  std::uint64_t expected_control_seq = 0U)
{
  return ControlRequest{
    Operation::Prepare, request_id, kGateId, expected_control_seq, ""};
}

std::string identifier(std::uint64_t value)
{
  std::ostringstream stream;
  stream << std::hex << std::nouppercase << std::setfill('0')
         << std::setw(32) << value;
  return stream.str();
}

bool is_lower_hex_identifier(const std::string & value)
{
  if (value.size() != 32U) {
    return false;
  }
  for (const auto character : value) {
    if (
      (character < '0' || character > '9') &&
      (character < 'a' || character > 'f'))
    {
      return false;
    }
  }
  return true;
}

WriterGid writer_gid(std::uint8_t discriminator = 0x42U)
{
  WriterGid gid{};
  gid.front() = discriminator;
  gid.back() = static_cast<std::uint8_t>(discriminator ^ 0xa5U);
  return gid;
}

ControlRequest lease_request(
  Operation operation,
  std::uint64_t request_number,
  const Snapshot & state)
{
  return ControlRequest{
    operation,
    identifier(request_number),
    state.gate_instance_id,
    state.control_seq,
    state.lease_id};
}

ControlResult prepare_with(
  MotionGateCore & gate,
  std::uint64_t request_number,
  MotionGateCore::SteadyTimePoint now)
{
  const auto state = gate.snapshot();
  return gate.prepare(
    ControlRequest{
        Operation::Prepare,
        identifier(request_number),
        state.gate_instance_id,
        state.control_seq,
        ""},
    now);
}

ControlResult open_with(
  MotionGateCore & gate,
  std::uint64_t request_number,
  MotionGateCore::SteadyTimePoint now,
  const WriterGid & gid)
{
  const auto state = gate.snapshot();
  return gate.open(
    lease_request(Operation::Open, request_number, state),
    now,
    [gid]() {
      return OpenBinding{true, Reason::None, gid, "writer ready"};
    });
}

struct ArmedGate
{
  MotionGateCore gate;
  WriterGid writer;
  std::string lease;
};

ArmedGate make_armed_gate(
  MotionGateConfig config = MotionGateConfig{},
  MotionGateCore::SteadyTimePoint now = at(0ms),
  std::uint64_t initial_control_seq = 0U)
{
  ArmedGate context{
    MotionGateCore{config, kGateId, initial_control_seq},
    writer_gid(),
    ""};
  EXPECT_EQ(
    prepare_with(context.gate, 1U, now).code,
    ResultCode::Applied);
  context.lease = context.gate.snapshot().lease_id;
  EXPECT_EQ(
    open_with(context.gate, 2U, now, context.writer).code,
    ResultCode::Applied);
  return context;
}

Candidate valid_candidate(const ArmedGate & context)
{
  Candidate candidate;
  candidate.lease_id = context.lease;
  candidate.writer_gid = context.writer;
  candidate.linear_x = 0.1;
  candidate.angular_z = -0.2;
  return candidate;
}

TEST(MotionGateCore, StartsFailClosed)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);

  const auto state = gate.snapshot();
  EXPECT_EQ(state.state, State::Inhibited);
  EXPECT_TRUE(state.motion_inhibited);
  EXPECT_FALSE(state.authority_live);
  EXPECT_FALSE(state.candidate_fresh);
  EXPECT_FALSE(state.writer_bound);
  EXPECT_TRUE(state.zero_selected);
  EXPECT_TRUE(gate.selected_command().is_zero());
  EXPECT_TRUE(gate.tick(at(0ms)).is_zero());
}

TEST(MotionGateCore, InhibitReassertionAdvancesTheGateWideControlSequence)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  const auto first = gate.inhibit(
    ControlRequest{
        Operation::Inhibit, identifier(10U), kGateId, 0U, ""}, at(0ms));
  ASSERT_EQ(first.code, ResultCode::Applied);
  EXPECT_EQ(first.control_seq, 1U);

  const auto stale = gate.inhibit(
    ControlRequest{
        Operation::Inhibit, identifier(11U), kGateId, 0U, ""}, at(1ms));
  EXPECT_EQ(stale.code, ResultCode::Rejected);
  EXPECT_EQ(stale.reason, Reason::StaleSequence);
  EXPECT_EQ(gate.snapshot().control_seq, 1U);

  const auto second = gate.inhibit(
    ControlRequest{
        Operation::Inhibit, identifier(11U), kGateId, 1U, ""}, at(2ms));
  EXPECT_EQ(second.code, ResultCode::Applied);
  EXPECT_EQ(second.control_seq, 2U);
}

TEST(MotionGateCore, InvalidConfigurationStartsFaultedAndCannotPrepare)
{
  auto config = MotionGateConfig{};
  config.candidate_freshness = config.authority_lease;
  MotionGateCore gate(config, kGateId);

  EXPECT_EQ(gate.snapshot().state, State::Faulted);
  EXPECT_EQ(gate.snapshot().reason, Reason::ConfigurationInvalid);

  const auto result = gate.prepare(prepare_request(), at(0ms));
  EXPECT_EQ(result.code, ResultCode::Faulted);
  EXPECT_EQ(result.reason, Reason::ConfigurationInvalid);
  EXPECT_TRUE(result.zero_selected);
  EXPECT_TRUE(gate.selected_command().is_zero());
}

TEST(MotionGateCore, RejectsEveryInvalidTrustedConfigurationFailClosed)
{
  std::vector<MotionGateConfig> invalid;

  auto config = MotionGateConfig{};
  config.authority_lease = 0ms;
  invalid.push_back(config);
  config = MotionGateConfig{};
  config.candidate_freshness = 0ms;
  invalid.push_back(config);
  config = MotionGateConfig{};
  config.prepare_timeout = -1ms;
  invalid.push_back(config);
  config = MotionGateConfig{};
  config.candidate_freshness = config.authority_lease + 1ms;
  invalid.push_back(config);
  config = MotionGateConfig{};
  config.linear_x_min = std::numeric_limits<double>::quiet_NaN();
  invalid.push_back(config);
  config = MotionGateConfig{};
  config.angular_z_max = std::numeric_limits<double>::infinity();
  invalid.push_back(config);
  config = MotionGateConfig{};
  config.linear_x_min = 0.1;
  invalid.push_back(config);
  config = MotionGateConfig{};
  config.angular_z_max = -0.1;
  invalid.push_back(config);
  config = MotionGateConfig{};
  config.request_cache_size = 0U;
  invalid.push_back(config);

  for (std::size_t index = 0U; index < invalid.size(); ++index) {
    SCOPED_TRACE(index);
    MotionGateCore gate(invalid[index], kGateId);
    EXPECT_EQ(gate.snapshot().state, State::Faulted);
    EXPECT_EQ(gate.snapshot().reason, Reason::ConfigurationInvalid);
    EXPECT_TRUE(gate.selected_command().is_zero());
  }

  MotionGateCore invalid_gate(MotionGateConfig{}, "NOT-LOWER-HEX");
  EXPECT_EQ(invalid_gate.snapshot().state, State::Faulted);
  EXPECT_EQ(invalid_gate.snapshot().reason, Reason::ConfigurationInvalid);
}

TEST(MotionGateCore, ControlSequenceExhaustionFaultsClosed)
{
  MotionGateCore gate(
    MotionGateConfig{}, kGateId, std::numeric_limits<std::uint64_t>::max());

  const auto result = gate.prepare(
    prepare_request(kRequestId, std::numeric_limits<std::uint64_t>::max()),
    at(0ms));

  EXPECT_EQ(result.code, ResultCode::Faulted);
  EXPECT_EQ(result.reason, Reason::SequenceExhausted);
  EXPECT_EQ(result.control_seq, std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(result.state, State::Faulted);
  EXPECT_TRUE(result.motion_inhibited);
  EXPECT_TRUE(result.zero_selected);
}

TEST(MotionGateCore, PrepareRequiresCanonicalIdentifiersAndEmptyLeaseUnionArm)
{
  struct Rejection
  {
    ControlRequest request;
    Reason reason;
  };

  auto uppercase_id = prepare_request(
    "0000000000000000000000000000000A");
  auto short_id = prepare_request("0000000000000000000000000000001");
  auto wrong_operation = prepare_request(identifier(3U));
  wrong_operation.operation = Operation::Open;
  auto malformed_gate = prepare_request(identifier(4U));
  malformed_gate.gate_instance_id =
    "0123456789ABCDEF0123456789ABCDEF";
  auto stale_gate = prepare_request(identifier(5U));
  stale_gate.gate_instance_id = kOtherGateId;
  auto nonempty_lease = prepare_request(identifier(6U));
  nonempty_lease.lease_id = identifier(99U);
  auto stale_sequence = prepare_request(identifier(7U), 1U);

  const std::vector<Rejection> cases{
    {uppercase_id, Reason::InvalidRequest},
    {short_id, Reason::InvalidRequest},
    {wrong_operation, Reason::InvalidRequest},
    {malformed_gate, Reason::InvalidRequest},
    {stale_gate, Reason::StaleGate},
    {nonempty_lease, Reason::InvalidRequest},
    {stale_sequence, Reason::StaleSequence},
  };

  for (std::size_t index = 0U; index < cases.size(); ++index) {
    SCOPED_TRACE(index);
    MotionGateCore gate(MotionGateConfig{}, kGateId);
    const auto result = gate.prepare(cases[index].request, at(0ms));
    EXPECT_EQ(result.code, ResultCode::Rejected);
    EXPECT_EQ(result.reason, cases[index].reason);
    EXPECT_EQ(gate.snapshot().state, State::Inhibited);
    EXPECT_EQ(gate.snapshot().control_seq, 0U);
    EXPECT_TRUE(gate.selected_command().is_zero());
  }
}

TEST(MotionGateCore, PrepareCreatesOpaqueUniqueLeaseAndCandidateTopic)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);

  const auto first = prepare_with(gate, 1U, at(0ms));
  ASSERT_EQ(first.code, ResultCode::Applied);
  EXPECT_EQ(first.state, State::Prepared);
  EXPECT_EQ(first.control_seq, 1U);
  EXPECT_TRUE(first.motion_inhibited);
  EXPECT_FALSE(first.authority_live);
  EXPECT_TRUE(first.zero_selected);
  EXPECT_TRUE(is_lower_hex_identifier(first.lease_id));
  EXPECT_EQ(
    first.candidate_topic,
    "/voice_nav_internal/motion_gate/candidate/lease_" + first.lease_id);

  const auto writer = writer_gid();
  ASSERT_EQ(open_with(gate, 2U, at(1ms), writer).code, ResultCode::Applied);
  auto inhibit = lease_request(Operation::Inhibit, 3U, gate.snapshot());
  ASSERT_EQ(gate.inhibit(inhibit, at(2ms)).code, ResultCode::Applied);
  const auto second = prepare_with(gate, 4U, at(3ms));
  ASSERT_EQ(second.code, ResultCode::Applied);
  EXPECT_TRUE(is_lower_hex_identifier(second.lease_id));
  EXPECT_NE(second.lease_id, first.lease_id);
  EXPECT_EQ(second.control_seq, 4U);
}

TEST(MotionGateCore, PrepareDuplicateDoesNotExtendDeadline)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  const auto request = prepare_request(identifier(1U));

  const auto applied = gate.prepare(request, at(0ms));
  ASSERT_EQ(applied.code, ResultCode::Applied);
  const auto duplicate = gate.prepare(request, at(999ms));
  EXPECT_EQ(duplicate.code, ResultCode::Duplicate);
  EXPECT_EQ(gate.snapshot().control_seq, 1U);
  EXPECT_EQ(gate.snapshot().state, State::Prepared);

  EXPECT_TRUE(gate.tick(at(1000ms)).is_zero());
  EXPECT_EQ(gate.snapshot().state, State::Inhibited);
  EXPECT_EQ(gate.snapshot().reason, Reason::PrepareExpired);
  EXPECT_EQ(gate.snapshot().control_seq, 2U);
}

TEST(MotionGateCore, SameRequestIdWithDifferentBodyIsCollision)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  auto original = prepare_request(identifier(1U));
  ASSERT_EQ(gate.prepare(original, at(0ms)).code, ResultCode::Applied);

  auto collision = original;
  collision.operation = Operation::Open;
  const auto result = gate.prepare(collision, at(1ms));

  EXPECT_EQ(result.code, ResultCode::Rejected);
  EXPECT_EQ(result.reason, Reason::RequestIdCollision);
  EXPECT_EQ(gate.snapshot().state, State::Prepared);
  EXPECT_EQ(gate.snapshot().control_seq, 1U);
}

TEST(MotionGateCore, RequestReplayCacheIsStrictlyBounded)
{
  auto config = MotionGateConfig{};
  config.request_cache_size = 2U;
  MotionGateCore gate(config, kGateId);
  ASSERT_EQ(
    gate.prepare(prepare_request(identifier(99U)), at(0ms)).code,
    ResultCode::Applied);

  for (std::uint64_t request_number = 1U;
    request_number <= 2U; ++request_number)
  {
    auto request = prepare_request(
      identifier(request_number), gate.snapshot().control_seq);
    EXPECT_EQ(
      gate.prepare(request, at(0ms)).code,
      ResultCode::Rejected);
  }

  auto retained = prepare_request(identifier(2U));
  EXPECT_EQ(gate.prepare(retained, at(0ms)).code, ResultCode::Duplicate);

  auto evicted = prepare_request(identifier(99U));
  EXPECT_EQ(gate.prepare(evicted, at(0ms)).code, ResultCode::Rejected);
}

TEST(MotionGateCore, PrepareDeadlineIsExclusiveForOpen)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 1U, at(0ms)).code, ResultCode::Applied);

  const auto result = open_with(gate, 2U, at(1000ms), writer_gid());

  EXPECT_EQ(result.code, ResultCode::Rejected);
  EXPECT_EQ(gate.snapshot().state, State::Inhibited);
  EXPECT_EQ(gate.snapshot().reason, Reason::PrepareExpired);
  EXPECT_EQ(gate.snapshot().control_seq, 2U);
}

TEST(MotionGateCore, LeaseOperationsEnforceTheirTaggedUnionAndCasFields)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 1U, at(0ms)).code, ResultCode::Applied);

  auto missing_lease = lease_request(Operation::Open, 2U, gate.snapshot());
  missing_lease.lease_id.clear();
  EXPECT_EQ(
    gate.open(missing_lease, at(1ms), {}).reason,
    Reason::InvalidRequest);

  auto wrong_operation = lease_request(
    Operation::Renew, 3U, gate.snapshot());
  EXPECT_EQ(
    gate.open(wrong_operation, at(1ms), {}).reason,
    Reason::InvalidRequest);

  auto uppercase_lease = lease_request(
    Operation::Open, 4U, gate.snapshot());
  uppercase_lease.lease_id.front() = 'A';
  EXPECT_EQ(
    gate.open(uppercase_lease, at(1ms), {}).reason,
    Reason::InvalidRequest);

  auto other_lease = lease_request(Operation::Open, 5U, gate.snapshot());
  other_lease.lease_id = identifier(999U);
  EXPECT_EQ(
    gate.open(other_lease, at(1ms), {}).reason,
    Reason::StaleLease);

  auto stale_sequence = lease_request(
    Operation::Open, 6U, gate.snapshot());
  stale_sequence.expected_control_seq += 1U;
  EXPECT_EQ(
    gate.open(stale_sequence, at(1ms), {}).reason,
    Reason::StaleSequence);

  ASSERT_EQ(
    open_with(gate, 7U, at(1ms), writer_gid()).code,
    ResultCode::Applied);

  auto renew_without_lease = lease_request(
    Operation::Renew, 8U, gate.snapshot());
  renew_without_lease.lease_id.clear();
  EXPECT_EQ(
    gate.renew(renew_without_lease, at(2ms)).reason,
    Reason::InvalidRequest);

  auto inhibit_without_lease = lease_request(
    Operation::Inhibit, 9U, gate.snapshot());
  inhibit_without_lease.lease_id.clear();
  EXPECT_EQ(
    gate.inhibit(inhibit_without_lease, at(2ms)).reason,
    Reason::InvalidRequest);

  EXPECT_EQ(gate.snapshot().state, State::Armed);
  EXPECT_EQ(gate.snapshot().control_seq, 2U);
}

TEST(MotionGateCore, OpenBindingFailureIsIdempotentAndDoesNotMutateLease)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 1U, at(0ms)).code, ResultCode::Applied);
  const auto request = lease_request(
    Operation::Open, 2U, gate.snapshot());

  const auto unavailable = gate.open(
    request, at(10ms),
    []() {
      return OpenBinding{
      false, Reason::WriterAmbiguous, {}, "two candidate writers"};
    });
  EXPECT_EQ(unavailable.code, ResultCode::Rejected);
  EXPECT_EQ(unavailable.reason, Reason::WriterAmbiguous);
  EXPECT_EQ(gate.snapshot().state, State::Prepared);
  EXPECT_EQ(gate.snapshot().control_seq, 1U);

  const auto retry = gate.open(
    request, at(20ms),
    []() {
      return OpenBinding{true, Reason::None, writer_gid(), "ready"};
    });
  EXPECT_EQ(retry.code, ResultCode::Duplicate);
  EXPECT_EQ(retry.reason, Reason::WriterAmbiguous);
  EXPECT_EQ(gate.snapshot().state, State::Prepared);

  const auto success = open_with(gate, 3U, at(20ms), writer_gid());
  EXPECT_EQ(success.code, ResultCode::Applied);
  EXPECT_EQ(success.state, State::Armed);
}

TEST(MotionGateCore, OpenRejectsMissingOrZeroWriterBinding)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 1U, at(0ms)).code, ResultCode::Applied);

  auto first = lease_request(Operation::Open, 2U, gate.snapshot());
  auto no_provider = gate.open(first, at(1ms), OpenBindingProvider{});
  EXPECT_EQ(no_provider.reason, Reason::WriterUnavailable);
  EXPECT_EQ(gate.snapshot().state, State::Prepared);

  auto second = lease_request(Operation::Open, 3U, gate.snapshot());
  auto zero_gid = gate.open(
    second, at(2ms),
    []() {
      return OpenBinding{true, Reason::None, {}, "zero gid"};
    });
  EXPECT_EQ(zero_gid.reason, Reason::WriterUnavailable);
  EXPECT_EQ(gate.snapshot().state, State::Prepared);
}

TEST(MotionGateCore, OpenFaultsClosedWhenReadyBindingCarriesNonNoneReason)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 1U, at(0ms)).code, ResultCode::Applied);
  const auto request = lease_request(
    Operation::Open, 2U, gate.snapshot());

  const auto result = gate.open(
    request, at(1ms),
    []() {
      return OpenBinding{
      true,
      Reason::WriterMetadataPending,
      writer_gid(),
      "contradictory ready binding"};
    });

  EXPECT_EQ(result.code, ResultCode::Faulted);
  EXPECT_EQ(result.reason, Reason::InternalFailure);
  EXPECT_EQ(result.state, State::Faulted);
  EXPECT_TRUE(result.motion_inhibited);
  EXPECT_TRUE(result.zero_selected);
  EXPECT_FALSE(result.writer_bound);
  EXPECT_TRUE(result.lease_id.empty());
  EXPECT_TRUE(gate.selected_command().is_zero());
}

TEST(MotionGateCore, OpenProviderExceptionFaultsClosed)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 1U, at(0ms)).code, ResultCode::Applied);
  auto request = lease_request(Operation::Open, 2U, gate.snapshot());

  const auto result = gate.open(
    request, at(1ms),
    []() -> OpenBinding {
      throw std::runtime_error("graph failure");
    });

  EXPECT_EQ(result.code, ResultCode::Faulted);
  EXPECT_EQ(result.reason, Reason::InternalFailure);
  EXPECT_EQ(gate.snapshot().state, State::Faulted);
  EXPECT_TRUE(gate.selected_command().is_zero());
}

TEST(MotionGateCore, OpenDuplicateDoesNotExtendCandidateDeadline)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 1U, at(0ms)).code, ResultCode::Applied);
  const auto request = lease_request(Operation::Open, 2U, gate.snapshot());
  const auto writer = writer_gid();
  const auto provider = [writer]() {
      return OpenBinding{true, Reason::None, writer, "ready"};
    };
  ASSERT_EQ(
    gate.open(request, at(0ms), provider).code,
    ResultCode::Applied);

  EXPECT_EQ(
    gate.open(request, at(149ms), provider).code,
    ResultCode::Duplicate);
  EXPECT_EQ(gate.snapshot().state, State::Armed);

  EXPECT_TRUE(gate.tick(at(150ms)).is_zero());
  EXPECT_EQ(gate.snapshot().state, State::Inhibited);
  EXPECT_EQ(gate.snapshot().reason, Reason::CandidateExpired);
  EXPECT_EQ(gate.snapshot().control_seq, 3U);
}

TEST(MotionGateCore, OpenSnapshotExposesBoundWriterAndLiveAuthority)
{
  const auto context = make_armed_gate();
  const auto state = context.gate.snapshot();

  EXPECT_EQ(state.state, State::Armed);
  EXPECT_EQ(state.control_seq, 2U);
  EXPECT_EQ(state.lease_id, context.lease);
  EXPECT_EQ(state.bound_writer_gid, context.writer);
  EXPECT_FALSE(state.motion_inhibited);
  EXPECT_TRUE(state.authority_live);
  EXPECT_FALSE(state.candidate_fresh);
  EXPECT_TRUE(state.writer_bound);
  EXPECT_TRUE(state.zero_selected);
}

TEST(MotionGateCore, CandidateClampsOnlySupportedFiniteAxes)
{
  auto context = make_armed_gate();
  struct Sample
  {
    double input_linear;
    double input_angular;
    double expected_linear;
    double expected_angular;
  };
  const std::vector<Sample> samples{
    {-100.0, -100.0, -0.2, -1.2},
    {-0.2, -1.2, -0.2, -1.2},
    {-0.125, 0.0, -0.125, 0.0},
    {0.0, 0.75, 0.0, 0.75},
    {0.4, 1.2, 0.4, 1.2},
    {100.0, 100.0, 0.4, 1.2},
  };

  for (std::size_t index = 0U; index < samples.size(); ++index) {
    SCOPED_TRACE(index);
    auto candidate = valid_candidate(context);
    candidate.linear_x = samples[index].input_linear;
    candidate.angular_z = samples[index].input_angular;
    const auto result = context.gate.accept_candidate(
      candidate, at(std::chrono::milliseconds{1 + static_cast<int>(index)}));
    ASSERT_TRUE(result.accepted);
    EXPECT_FALSE(result.retired);
    EXPECT_EQ(result.reason, Reason::None);
    EXPECT_DOUBLE_EQ(result.selected.linear_x, samples[index].expected_linear);
    EXPECT_DOUBLE_EQ(result.selected.angular_z, samples[index].expected_angular);
    EXPECT_DOUBLE_EQ(
      context.gate.selected_command().linear_x,
      samples[index].expected_linear);
    EXPECT_DOUBLE_EQ(
      context.gate.selected_command().angular_z,
      samples[index].expected_angular);
  }

  const auto state = context.gate.snapshot();
  EXPECT_EQ(state.state, State::Armed);
  EXPECT_TRUE(state.candidate_fresh);
  EXPECT_FALSE(state.motion_inhibited);
}

TEST(MotionGateCore, EveryNonFiniteAxisRetiresCurrentLease)
{
  struct Axis
  {
    const char * name;
    double Candidate::* member;
  };
  const std::vector<Axis> axes{
    {"linear_x", &Candidate::linear_x},
    {"linear_y", &Candidate::linear_y},
    {"linear_z", &Candidate::linear_z},
    {"angular_x", &Candidate::angular_x},
    {"angular_y", &Candidate::angular_y},
    {"angular_z", &Candidate::angular_z},
  };
  const std::vector<double> invalid_values{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
  };

  for (const auto & axis : axes) {
    for (const auto value : invalid_values) {
      SCOPED_TRACE(axis.name);
      auto context = make_armed_gate();
      auto candidate = valid_candidate(context);
      candidate.*(axis.member) = value;

      const auto result = context.gate.accept_candidate(candidate, at(1ms));

      EXPECT_FALSE(result.accepted);
      EXPECT_TRUE(result.retired);
      EXPECT_EQ(result.reason, Reason::InvalidCandidate);
      EXPECT_EQ(context.gate.snapshot().state, State::Inhibited);
      EXPECT_EQ(context.gate.snapshot().reason, Reason::InvalidCandidate);
      EXPECT_EQ(context.gate.snapshot().control_seq, 3U);
      EXPECT_TRUE(context.gate.selected_command().is_zero());
    }
  }
}

TEST(MotionGateCore, EveryUnsupportedNonzeroAxisRetiresCurrentLease)
{
  struct Axis
  {
    const char * name;
    double Candidate::* member;
  };
  const std::vector<Axis> axes{
    {"linear_y", &Candidate::linear_y},
    {"linear_z", &Candidate::linear_z},
    {"angular_x", &Candidate::angular_x},
    {"angular_y", &Candidate::angular_y},
  };

  for (const auto & axis : axes) {
    for (const auto value : {-0.001, 0.001}) {
      SCOPED_TRACE(axis.name);
      auto context = make_armed_gate();
      auto candidate = valid_candidate(context);
      candidate.*(axis.member) = value;

      const auto result = context.gate.accept_candidate(candidate, at(1ms));

      EXPECT_FALSE(result.accepted);
      EXPECT_TRUE(result.retired);
      EXPECT_EQ(result.reason, Reason::InvalidCandidate);
      EXPECT_EQ(context.gate.snapshot().state, State::Inhibited);
      EXPECT_TRUE(context.gate.selected_command().is_zero());
    }
  }
}

TEST(MotionGateCore, CurrentCandidateMustMatchBoundInterProcessWriter)
{
  enum class Failure
  {
    IntraProcess,
    ZeroGid,
    WrongGid,
  };

  for (const auto failure : {
        Failure::IntraProcess, Failure::ZeroGid, Failure::WrongGid})
  {
    SCOPED_TRACE(static_cast<int>(failure));
    auto context = make_armed_gate();
    auto candidate = valid_candidate(context);
    if (failure == Failure::IntraProcess) {
      candidate.from_intra_process = true;
    } else if (failure == Failure::ZeroGid) {
      candidate.writer_gid.fill(0U);
    } else {
      candidate.writer_gid = writer_gid(0x24U);
    }

    const auto result = context.gate.accept_candidate(candidate, at(1ms));

    EXPECT_FALSE(result.accepted);
    EXPECT_TRUE(result.retired);
    EXPECT_EQ(result.reason, Reason::WriterMismatch);
    EXPECT_EQ(context.gate.snapshot().state, State::Inhibited);
    EXPECT_EQ(context.gate.snapshot().control_seq, 3U);
    EXPECT_TRUE(context.gate.selected_command().is_zero());
  }
}

TEST(MotionGateCore, StaleLeaseCandidateCannotRetireOrDriveNewLease)
{
  auto context = make_armed_gate();
  const auto old_lease = context.lease;
  auto stop = lease_request(Operation::Inhibit, 3U, context.gate.snapshot());
  ASSERT_EQ(context.gate.inhibit(stop, at(1ms)).code, ResultCode::Applied);
  ASSERT_EQ(
    prepare_with(context.gate, 4U, at(2ms)).code,
    ResultCode::Applied);
  context.lease = context.gate.snapshot().lease_id;
  context.writer = writer_gid(0x55U);
  ASSERT_EQ(
    open_with(context.gate, 5U, at(2ms), context.writer).code,
    ResultCode::Applied);
  ASSERT_NE(context.lease, old_lease);
  const auto current_seq = context.gate.snapshot().control_seq;

  auto stale = valid_candidate(context);
  stale.lease_id = old_lease;
  stale.from_intra_process = true;
  stale.writer_gid.fill(0U);
  stale.linear_x = std::numeric_limits<double>::quiet_NaN();
  stale.linear_y = 1.0;

  const auto result = context.gate.accept_candidate(stale, at(3ms));

  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.retired);
  EXPECT_EQ(result.reason, Reason::StaleLease);
  EXPECT_EQ(context.gate.snapshot().state, State::Armed);
  EXPECT_EQ(context.gate.snapshot().control_seq, current_seq);
  EXPECT_EQ(context.gate.snapshot().lease_id, context.lease);
  EXPECT_TRUE(context.gate.selected_command().is_zero());
}

TEST(MotionGateCore, MalformedLeaseCandidateIsDroppedWithoutMutation)
{
  auto context = make_armed_gate();
  const auto state_before = context.gate.snapshot();
  auto malformed = valid_candidate(context);
  malformed.lease_id = "bad";

  const auto result = context.gate.accept_candidate(malformed, at(1ms));

  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.retired);
  EXPECT_EQ(result.reason, Reason::InvalidCandidate);
  EXPECT_EQ(context.gate.snapshot().state, State::Armed);
  EXPECT_EQ(context.gate.snapshot().control_seq, state_before.control_seq);
  EXPECT_EQ(context.gate.snapshot().state_seq, state_before.state_seq);
}

TEST(MotionGateCore, CandidateSamplesNeverRenewAuthority)
{
  auto context = make_armed_gate();
  auto candidate = valid_candidate(context);
  ASSERT_TRUE(
    context.gate.accept_candidate(candidate, at(149ms)).accepted);
  ASSERT_TRUE(
    context.gate.accept_candidate(candidate, at(249ms)).accepted);

  EXPECT_FALSE(context.gate.tick(at(249ms)).is_zero());
  EXPECT_TRUE(context.gate.tick(at(250ms)).is_zero());
  EXPECT_EQ(context.gate.snapshot().state, State::Inhibited);
  EXPECT_EQ(context.gate.snapshot().reason, Reason::AuthorityExpired);
  EXPECT_EQ(context.gate.snapshot().control_seq, 3U);
}

TEST(MotionGateCore, CandidateFreshnessExpiresExactlyAtDeadline)
{
  auto context = make_armed_gate();
  auto candidate = valid_candidate(context);
  ASSERT_TRUE(
    context.gate.accept_candidate(candidate, at(10ms)).accepted);

  auto first_renew = lease_request(
    Operation::Renew, 3U, context.gate.snapshot());
  ASSERT_EQ(
    context.gate.renew(first_renew, at(90ms)).code,
    ResultCode::Applied);

  auto second_renew = lease_request(
    Operation::Renew, 4U, context.gate.snapshot());
  ASSERT_EQ(
    context.gate.renew(second_renew, at(140ms)).code,
    ResultCode::Applied);

  EXPECT_FALSE(context.gate.tick(at(159ms)).is_zero());
  EXPECT_TRUE(context.gate.tick(at(160ms)).is_zero());
  EXPECT_EQ(context.gate.snapshot().state, State::Inhibited);
  EXPECT_EQ(context.gate.snapshot().reason, Reason::CandidateExpired);
  EXPECT_EQ(context.gate.snapshot().control_seq, 5U);
}

TEST(MotionGateCore, RenewUsesGlobalCasAndPreservesSelectedCandidate)
{
  auto context = make_armed_gate();
  auto candidate = valid_candidate(context);
  ASSERT_TRUE(context.gate.accept_candidate(candidate, at(100ms)).accepted);
  const auto selected = context.gate.selected_command();

  auto stale = lease_request(
    Operation::Renew, 3U, context.gate.snapshot());
  stale.expected_control_seq -= 1U;
  const auto stale_result = context.gate.renew(stale, at(100ms));
  EXPECT_EQ(stale_result.reason, Reason::StaleSequence);
  EXPECT_EQ(context.gate.snapshot().control_seq, 2U);

  auto renew = lease_request(
    Operation::Renew, 4U, context.gate.snapshot());
  const auto renewed = context.gate.renew(renew, at(100ms));
  EXPECT_EQ(renewed.code, ResultCode::Applied);
  EXPECT_EQ(renewed.control_seq, 3U);
  EXPECT_DOUBLE_EQ(
    context.gate.selected_command().linear_x, selected.linear_x);
  EXPECT_DOUBLE_EQ(
    context.gate.selected_command().angular_z, selected.angular_z);
}

TEST(MotionGateCore, RenewAtAuthorityDeadlineCannotResurrectLease)
{
  auto context = make_armed_gate();
  auto candidate = valid_candidate(context);
  ASSERT_TRUE(
    context.gate.accept_candidate(candidate, at(149ms)).accepted);
  ASSERT_TRUE(
    context.gate.accept_candidate(candidate, at(249ms)).accepted);
  auto renew = lease_request(
    Operation::Renew, 3U, context.gate.snapshot());

  const auto result = context.gate.renew(renew, at(250ms));

  EXPECT_EQ(result.code, ResultCode::Rejected);
  EXPECT_EQ(context.gate.snapshot().state, State::Inhibited);
  EXPECT_EQ(context.gate.snapshot().reason, Reason::AuthorityExpired);
  EXPECT_EQ(context.gate.snapshot().control_seq, 3U);
  EXPECT_TRUE(context.gate.selected_command().is_zero());
}

TEST(MotionGateCore, InhibitRequiresExactCurrentGateSequenceAndLeaseTuple)
{
  auto context = make_armed_gate();

  auto wrong_gate = lease_request(
    Operation::Inhibit, 3U, context.gate.snapshot());
  wrong_gate.gate_instance_id = kOtherGateId;
  EXPECT_EQ(
    context.gate.inhibit(wrong_gate, at(1ms)).reason,
    Reason::StaleGate);
  EXPECT_EQ(context.gate.snapshot().state, State::Armed);

  auto wrong_sequence = lease_request(
    Operation::Inhibit, 4U, context.gate.snapshot());
  wrong_sequence.expected_control_seq += 1U;
  EXPECT_EQ(
    context.gate.inhibit(wrong_sequence, at(1ms)).reason,
    Reason::StaleSequence);
  EXPECT_EQ(context.gate.snapshot().state, State::Armed);

  auto wrong_lease = lease_request(
    Operation::Inhibit, 5U, context.gate.snapshot());
  wrong_lease.lease_id = identifier(999U);
  EXPECT_EQ(
    context.gate.inhibit(wrong_lease, at(1ms)).reason,
    Reason::StaleLease);
  EXPECT_EQ(context.gate.snapshot().state, State::Armed);

  auto current = lease_request(
    Operation::Inhibit, 6U, context.gate.snapshot());
  const auto applied = context.gate.inhibit(current, at(1ms));
  EXPECT_EQ(applied.code, ResultCode::Applied);
  EXPECT_EQ(applied.control_seq, 3U);
  EXPECT_EQ(applied.state, State::Inhibited);
  EXPECT_TRUE(applied.motion_inhibited);
  EXPECT_TRUE(applied.zero_selected);
  EXPECT_TRUE(context.gate.selected_command().is_zero());

  const auto duplicate = context.gate.inhibit(current, at(2ms));
  EXPECT_EQ(duplicate.code, ResultCode::Duplicate);
  EXPECT_EQ(context.gate.snapshot().control_seq, 3U);
}

TEST(MotionGateCore, InhibitCanRetirePreparedLease)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 1U, at(0ms)).code, ResultCode::Applied);
  auto request = lease_request(Operation::Inhibit, 2U, gate.snapshot());

  const auto result = gate.inhibit(request, at(1ms));

  EXPECT_EQ(result.code, ResultCode::Applied);
  EXPECT_EQ(result.state, State::Inhibited);
  EXPECT_EQ(result.control_seq, 2U);
  EXPECT_TRUE(result.lease_id.empty());
  EXPECT_TRUE(gate.selected_command().is_zero());
}

TEST(MotionGateCore, StaleInhibitCanConvergeWithTheSameRequestId)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 1U, at(0ms)).code, ResultCode::Applied);
  const auto current = gate.snapshot();
  auto retry = lease_request(Operation::Inhibit, 2U, current);
  retry.expected_control_seq = current.control_seq - 1U;

  const auto stale = gate.inhibit(retry, at(1ms));
  EXPECT_EQ(stale.code, ResultCode::Rejected);
  EXPECT_EQ(stale.reason, Reason::StaleSequence);

  retry.expected_control_seq = current.control_seq;
  const auto converged = gate.inhibit(retry, at(2ms));
  EXPECT_EQ(converged.code, ResultCode::Applied);
  EXPECT_EQ(converged.state, State::Inhibited);
}

TEST(MotionGateCore, AllControlOperationsAllowStaleTupleRebuild)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);

  auto prepare = prepare_request(identifier(20U), 1U);
  EXPECT_EQ(gate.prepare(prepare, at(0ms)).reason, Reason::StaleSequence);
  prepare.expected_control_seq = 0U;
  ASSERT_EQ(gate.prepare(prepare, at(1ms)).code, ResultCode::Applied);

  auto open = lease_request(Operation::Open, 21U, gate.snapshot());
  --open.expected_control_seq;
  EXPECT_EQ(gate.open(open, at(2ms), {}).reason, Reason::StaleSequence);
  open.expected_control_seq = gate.snapshot().control_seq;
  const auto provider = []() {
      return OpenBinding{true, Reason::None, writer_gid(), "writer ready"};
    };
  ASSERT_EQ(gate.open(open, at(3ms), provider).code, ResultCode::Applied);

  auto renew = lease_request(Operation::Renew, 22U, gate.snapshot());
  --renew.expected_control_seq;
  EXPECT_EQ(gate.renew(renew, at(4ms)).reason, Reason::StaleSequence);
  renew.expected_control_seq = gate.snapshot().control_seq;
  ASSERT_EQ(gate.renew(renew, at(5ms)).code, ResultCode::Applied);

  auto inhibit = lease_request(Operation::Inhibit, 23U, gate.snapshot());
  --inhibit.expected_control_seq;
  EXPECT_EQ(gate.inhibit(inhibit, at(6ms)).reason, Reason::StaleSequence);
  inhibit.expected_control_seq = gate.snapshot().control_seq;
  EXPECT_EQ(gate.inhibit(inhibit, at(7ms)).code, ResultCode::Applied);
}

TEST(MotionGateCore, StaleGateTupleRebuildsAcrossAllOperations)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);

  auto prepare = prepare_request(identifier(30U));
  prepare.gate_instance_id = kOtherGateId;
  EXPECT_EQ(
    gate.prepare(prepare, at(0ms)).reason,
    Reason::StaleGate);
  prepare.gate_instance_id = kGateId;
  ASSERT_EQ(gate.prepare(prepare, at(1ms)).code, ResultCode::Applied);

  auto open = lease_request(Operation::Open, 31U, gate.snapshot());
  open.gate_instance_id = kOtherGateId;
  EXPECT_EQ(gate.open(open, at(2ms), {}).reason, Reason::StaleGate);
  open.gate_instance_id = kGateId;
  ASSERT_EQ(
    gate.open(open, at(3ms), []() {
      return OpenBinding{true, Reason::None, writer_gid(), "writer ready"};
      }).code,
    ResultCode::Applied);

  auto renew = lease_request(Operation::Renew, 32U, gate.snapshot());
  renew.gate_instance_id = kOtherGateId;
  EXPECT_EQ(gate.renew(renew, at(4ms)).reason, Reason::StaleGate);
  renew.gate_instance_id = kGateId;
  ASSERT_EQ(gate.renew(renew, at(5ms)).code, ResultCode::Applied);

  auto inhibit = lease_request(Operation::Inhibit, 33U, gate.snapshot());
  inhibit.gate_instance_id = kOtherGateId;
  EXPECT_EQ(gate.inhibit(inhibit, at(6ms)).reason, Reason::StaleGate);
  inhibit.gate_instance_id = kGateId;
  ASSERT_EQ(gate.inhibit(inhibit, at(7ms)).code, ResultCode::Applied);

  auto operation_collision = prepare;
  operation_collision.operation = Operation::Open;
  operation_collision.gate_instance_id = kGateId;
  operation_collision.expected_control_seq = gate.snapshot().control_seq;
  EXPECT_EQ(
    gate.open(operation_collision, at(8ms), {}).reason,
    Reason::RequestIdCollision);
}

TEST(MotionGateCore, OldOpenReplayNeverResurrectsRetiredLease)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 1U, at(0ms)).code, ResultCode::Applied);
  const auto old_open = lease_request(Operation::Open, 2U, gate.snapshot());
  const auto writer = writer_gid();
  const auto provider = [writer]() {
      return OpenBinding{true, Reason::None, writer, "ready"};
    };
  ASSERT_EQ(
    gate.open(old_open, at(0ms), provider).code,
    ResultCode::Applied);
  auto stop = lease_request(Operation::Inhibit, 3U, gate.snapshot());
  ASSERT_EQ(gate.inhibit(stop, at(1ms)).code, ResultCode::Applied);
  ASSERT_EQ(prepare_with(gate, 4U, at(2ms)).code, ResultCode::Applied);
  const auto current = gate.snapshot();

  const auto replay = gate.open(old_open, at(3ms), provider);

  EXPECT_EQ(replay.code, ResultCode::Duplicate);
  EXPECT_EQ(gate.snapshot().state, State::Prepared);
  EXPECT_EQ(gate.snapshot().lease_id, current.lease_id);
  EXPECT_EQ(gate.snapshot().control_seq, current.control_seq);
  EXPECT_TRUE(gate.selected_command().is_zero());
}

TEST(MotionGateCore, AutomaticExpiryRotatesCasAndCanExhaustIt)
{
  MotionGateCore gate(
    MotionGateConfig{}, kGateId,
    std::numeric_limits<std::uint64_t>::max() - 1U);
  ASSERT_EQ(
    prepare_with(gate, 1U, at(0ms)).code,
    ResultCode::Applied);
  ASSERT_EQ(
    gate.snapshot().control_seq,
    std::numeric_limits<std::uint64_t>::max());

  EXPECT_TRUE(gate.tick(at(1000ms)).is_zero());
  EXPECT_EQ(gate.snapshot().state, State::Faulted);
  EXPECT_EQ(gate.snapshot().reason, Reason::SequenceExhausted);
  EXPECT_EQ(
    gate.snapshot().control_seq,
    std::numeric_limits<std::uint64_t>::max());
}

TEST(MotionGateCore, InvalidCandidateAtSequenceLimitFaultsClosed)
{
  auto context = make_armed_gate(
    MotionGateConfig{}, at(0ms),
    std::numeric_limits<std::uint64_t>::max() - 2U);
  ASSERT_EQ(
    context.gate.snapshot().control_seq,
    std::numeric_limits<std::uint64_t>::max());
  auto candidate = valid_candidate(context);
  candidate.linear_y = 1.0;

  const auto result = context.gate.accept_candidate(candidate, at(1ms));

  EXPECT_TRUE(result.retired);
  EXPECT_EQ(context.gate.snapshot().state, State::Faulted);
  EXPECT_EQ(context.gate.snapshot().reason, Reason::SequenceExhausted);
  EXPECT_TRUE(context.gate.selected_command().is_zero());
}

TEST(MotionGateCore, ForceFaultRotatesCasAndLatchesFirstFault)
{
  auto context = make_armed_gate();
  auto candidate = valid_candidate(context);
  ASSERT_TRUE(context.gate.accept_candidate(candidate, at(1ms)).accepted);
  const auto state_seq_before = context.gate.snapshot().state_seq;

  context.gate.force_fault(Reason::PublishFailed, "zero publish failed");

  EXPECT_EQ(context.gate.snapshot().state, State::Faulted);
  EXPECT_EQ(context.gate.snapshot().reason, Reason::PublishFailed);
  EXPECT_EQ(context.gate.snapshot().control_seq, 3U);
  EXPECT_GT(context.gate.snapshot().state_seq, state_seq_before);
  EXPECT_TRUE(context.gate.snapshot().motion_inhibited);
  EXPECT_TRUE(context.gate.selected_command().is_zero());

  context.gate.force_fault(Reason::InternalFailure, "late fault");
  EXPECT_EQ(context.gate.snapshot().reason, Reason::PublishFailed);
  EXPECT_EQ(context.gate.snapshot().control_seq, 3U);
}

TEST(MotionGateCore, PrepareAdmissionSuccessIsAppliedOnceAndReplayedByCore)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  const auto request = prepare_request(identifier(100U));
  std::size_t provider_calls = 0U;
  const auto provider = [&provider_calls]() {
      ++provider_calls;
      return PrepareAdmission{
      true, Reason::None, "retired writer is absent"};
    };

  const auto applied = gate.prepare(request, at(0ms), provider);
  ASSERT_EQ(applied.code, ResultCode::Applied);
  EXPECT_EQ(applied.state, State::Prepared);
  EXPECT_EQ(provider_calls, 1U);

  const auto duplicate = gate.prepare(request, at(1ms), provider);
  EXPECT_EQ(duplicate.code, ResultCode::Duplicate);
  EXPECT_EQ(duplicate.state, gate.snapshot().state);
  EXPECT_EQ(duplicate.control_seq, gate.snapshot().control_seq);
  EXPECT_EQ(duplicate.lease_id, gate.snapshot().lease_id);
  EXPECT_EQ(provider_calls, 1U);
}

TEST(MotionGateCore, PrepareAdmissionRejectionUsesGlobalRequestCache)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  const auto request = prepare_request(identifier(101U));
  std::size_t admission_calls = 0U;
  const auto admission_provider = [&admission_calls]() {
      ++admission_calls;
      return PrepareAdmission{
      false,
      Reason::WriterStillPresent,
      "the retired candidate writer remains discoverable"};
    };

  const auto rejected =
    gate.prepare(request, at(0ms), admission_provider);
  ASSERT_EQ(rejected.code, ResultCode::Rejected);
  EXPECT_EQ(rejected.reason, Reason::WriterStillPresent);
  EXPECT_EQ(rejected.state, State::Inhibited);
  EXPECT_EQ(gate.snapshot().control_seq, 0U);
  EXPECT_EQ(admission_calls, 1U);

  const auto duplicate =
    gate.prepare(request, at(1ms), admission_provider);
  EXPECT_EQ(duplicate.code, ResultCode::Duplicate);
  EXPECT_EQ(duplicate.reason, Reason::WriterStillPresent);
  EXPECT_EQ(duplicate.state, State::Inhibited);
  EXPECT_EQ(admission_calls, 1U);

  auto collision = request;
  collision.operation = Operation::Open;
  const auto collision_result =
    gate.prepare(collision, at(2ms), admission_provider);
  EXPECT_EQ(collision_result.code, ResultCode::Rejected);
  EXPECT_EQ(collision_result.reason, Reason::RequestIdCollision);
  EXPECT_EQ(admission_calls, 1U);

  std::size_t open_provider_calls = 0U;
  ControlRequest cross_operation{
    Operation::Open,
    request.request_id,
    kGateId,
    gate.snapshot().control_seq,
    identifier(999U)};
  const auto cross_operation_result = gate.open(
    cross_operation,
    at(3ms),
    [&open_provider_calls]() {
      ++open_provider_calls;
      return OpenBinding{
      true, Reason::None, writer_gid(), "must not run"};
    });
  EXPECT_EQ(cross_operation_result.code, ResultCode::Rejected);
  EXPECT_EQ(
    cross_operation_result.reason,
    Reason::RequestIdCollision);
  EXPECT_EQ(open_provider_calls, 0U);
  EXPECT_EQ(admission_calls, 1U);
  EXPECT_EQ(gate.snapshot().state, State::Inhibited);
}

TEST(MotionGateCore, PrepareAdmissionExceptionFaultsAndIsNotRepeated)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  const auto request = prepare_request(identifier(102U));
  std::size_t provider_calls = 0U;
  const auto provider = [&provider_calls]() -> PrepareAdmission {
      ++provider_calls;
      throw std::runtime_error(std::string(400U, 'x'));
    };

  const auto fault = gate.prepare(request, at(0ms), provider);
  ASSERT_EQ(fault.code, ResultCode::Faulted);
  EXPECT_EQ(fault.reason, Reason::InternalFailure);
  EXPECT_EQ(fault.state, State::Faulted);
  EXPECT_TRUE(fault.motion_inhibited);
  EXPECT_TRUE(fault.zero_selected);
  EXPECT_LE(fault.detail.size(), 160U);
  EXPECT_LE(gate.snapshot().detail.size(), 160U);
  EXPECT_EQ(provider_calls, 1U);

  const auto retry = gate.prepare(request, at(1ms), provider);
  EXPECT_EQ(retry.code, ResultCode::Faulted);
  EXPECT_EQ(retry.reason, Reason::InternalFailure);
  EXPECT_EQ(retry.state, State::Faulted);
  EXPECT_LE(retry.detail.size(), 160U);
  EXPECT_EQ(provider_calls, 1U);
}

TEST(MotionGateCore, OpenPreservesWriterMismatchAndCachesItsRejection)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 110U, at(0ms)).code, ResultCode::Applied);
  const auto request =
    lease_request(Operation::Open, 111U, gate.snapshot());
  std::size_t provider_calls = 0U;
  const auto provider = [&provider_calls]() {
      ++provider_calls;
      return OpenBinding{
      false,
      Reason::WriterMismatch,
      {},
      "candidate writer owner or QoS does not match policy"};
    };

  const auto rejected = gate.open(request, at(1ms), provider);
  ASSERT_EQ(rejected.code, ResultCode::Rejected);
  EXPECT_EQ(rejected.reason, Reason::WriterMismatch);
  EXPECT_EQ(rejected.state, State::Prepared);
  EXPECT_EQ(provider_calls, 1U);

  const auto duplicate = gate.open(request, at(2ms), provider);
  EXPECT_EQ(duplicate.code, ResultCode::Duplicate);
  EXPECT_EQ(duplicate.reason, Reason::WriterMismatch);
  EXPECT_EQ(duplicate.state, gate.snapshot().state);
  EXPECT_EQ(duplicate.control_seq, gate.snapshot().control_seq);
  EXPECT_EQ(provider_calls, 1U);
}

TEST(MotionGateCore, OpenPreservesTypedWriterMetadataPendingWithoutBinding)
{
  MotionGateCore gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(gate, 150U, at(0ms)).code, ResultCode::Applied);
  const auto writer = writer_gid();
  const auto request =
    lease_request(Operation::Open, 151U, gate.snapshot());
  std::size_t provider_calls = 0U;
  const auto pending_provider = [&provider_calls, writer]() {
      ++provider_calls;
      return OpenBinding{
      false,
      Reason::WriterMetadataPending,
      writer,
      "candidate writer identity is unresolved"};
    };

  const auto pending = gate.open(request, at(1ms), pending_provider);
  ASSERT_EQ(pending.code, ResultCode::Rejected);
  EXPECT_EQ(pending.reason, Reason::WriterMetadataPending);
  EXPECT_EQ(pending.state, State::Prepared);
  EXPECT_EQ(pending.control_seq, 1U);
  EXPECT_FALSE(pending.writer_bound);
  EXPECT_TRUE(std::all_of(
      pending.bound_writer_gid.cbegin(),
      pending.bound_writer_gid.cend(),
      [](std::uint8_t value) {return value == 0U;}));
  EXPECT_TRUE(pending.motion_inhibited);
  EXPECT_TRUE(pending.zero_selected);
  EXPECT_EQ(provider_calls, 1U);

  const auto duplicate = gate.open(request, at(2ms), pending_provider);
  EXPECT_EQ(duplicate.code, ResultCode::Duplicate);
  EXPECT_EQ(duplicate.reason, Reason::WriterMetadataPending);
  EXPECT_EQ(provider_calls, 1U);

  const auto opened = open_with(gate, 152U, at(3ms), writer);
  EXPECT_EQ(opened.code, ResultCode::Applied);
  EXPECT_EQ(opened.state, State::Armed);
  EXPECT_EQ(opened.bound_writer_gid, writer);
}

TEST(MotionGateCore, DuplicateAfterDeadlineCarriesCurrentSnapshot)
{
  MotionGateCore prepared_gate(MotionGateConfig{}, kGateId);
  const auto prepare = prepare_request(identifier(120U));
  ASSERT_EQ(
    prepared_gate.prepare(prepare, at(0ms)).code,
    ResultCode::Applied);

  const auto expired_prepare =
    prepared_gate.prepare(prepare, at(1000ms));
  const auto inhibited_after_prepare = prepared_gate.snapshot();
  EXPECT_EQ(expired_prepare.code, ResultCode::Duplicate);
  EXPECT_EQ(expired_prepare.state, State::Inhibited);
  EXPECT_EQ(
    expired_prepare.control_seq,
    inhibited_after_prepare.control_seq);
  EXPECT_EQ(expired_prepare.lease_id, inhibited_after_prepare.lease_id);
  EXPECT_EQ(
    expired_prepare.candidate_topic,
    inhibited_after_prepare.candidate_topic);
  EXPECT_EQ(
    expired_prepare.motion_inhibited,
    inhibited_after_prepare.motion_inhibited);
  EXPECT_EQ(
    expired_prepare.zero_selected,
    inhibited_after_prepare.zero_selected);

  MotionGateCore armed_gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(prepare_with(armed_gate, 121U, at(0ms)).code, ResultCode::Applied);
  const auto open =
    lease_request(Operation::Open, 122U, armed_gate.snapshot());
  std::size_t provider_calls = 0U;
  const auto provider = [&provider_calls]() {
      ++provider_calls;
      return OpenBinding{
      true, Reason::None, writer_gid(), "writer ready"};
    };
  ASSERT_EQ(
    armed_gate.open(open, at(0ms), provider).code,
    ResultCode::Applied);

  const auto expired_open = armed_gate.open(open, at(150ms), provider);
  const auto inhibited_after_open = armed_gate.snapshot();
  EXPECT_EQ(expired_open.code, ResultCode::Duplicate);
  EXPECT_EQ(expired_open.state, State::Inhibited);
  EXPECT_EQ(expired_open.control_seq, inhibited_after_open.control_seq);
  EXPECT_EQ(expired_open.lease_id, inhibited_after_open.lease_id);
  EXPECT_TRUE(expired_open.motion_inhibited);
  EXPECT_TRUE(expired_open.zero_selected);
  EXPECT_EQ(provider_calls, 1U);
}

TEST(MotionGateCore, OldInhibitDuplicateReportsNewLeaseWithoutMutatingIt)
{
  auto context = make_armed_gate();
  auto first_candidate = valid_candidate(context);
  first_candidate.linear_x = 0.15;
  ASSERT_TRUE(
    context.gate.accept_candidate(first_candidate, at(1ms)).accepted);
  const auto old_inhibit =
    lease_request(Operation::Inhibit, 130U, context.gate.snapshot());
  ASSERT_EQ(
    context.gate.inhibit(old_inhibit, at(2ms)).code,
    ResultCode::Applied);

  ASSERT_EQ(
    prepare_with(context.gate, 131U, at(3ms)).code,
    ResultCode::Applied);
  ASSERT_EQ(
    open_with(context.gate, 132U, at(4ms), context.writer).code,
    ResultCode::Applied);
  auto new_candidate = valid_candidate(context);
  new_candidate.lease_id = context.gate.snapshot().lease_id;
  new_candidate.linear_x = 0.20;
  ASSERT_TRUE(
    context.gate.accept_candidate(new_candidate, at(5ms)).accepted);
  const auto current = context.gate.snapshot();
  const auto current_command = context.gate.selected_command();

  const auto duplicate =
    context.gate.inhibit(old_inhibit, at(6ms));

  EXPECT_EQ(duplicate.code, ResultCode::Duplicate);
  EXPECT_EQ(duplicate.state, State::Armed);
  EXPECT_EQ(duplicate.control_seq, current.control_seq);
  EXPECT_EQ(duplicate.lease_id, current.lease_id);
  EXPECT_FALSE(duplicate.motion_inhibited);
  EXPECT_TRUE(duplicate.authority_live);
  EXPECT_TRUE(duplicate.candidate_fresh);
  EXPECT_EQ(context.gate.snapshot().control_seq, current.control_seq);
  EXPECT_EQ(context.gate.snapshot().lease_id, current.lease_id);
  EXPECT_DOUBLE_EQ(
    context.gate.selected_command().linear_x,
    current_command.linear_x);
  EXPECT_DOUBLE_EQ(
    context.gate.selected_command().angular_z,
    current_command.angular_z);
}

TEST(MotionGateCore, EveryExternallyVisibleDetailIsBounded)
{
  const std::string long_detail(500U, 'd');

  MotionGateCore admission_gate(MotionGateConfig{}, kGateId);
  const auto admission_result = admission_gate.prepare(
    prepare_request(identifier(140U)),
    at(0ms),
    [&long_detail]() {
      return PrepareAdmission{
      false, Reason::WriterStillPresent, long_detail};
    });
  EXPECT_LE(admission_result.detail.size(), 160U);
  EXPECT_LE(admission_gate.snapshot().detail.size(), 160U);

  MotionGateCore binding_gate(MotionGateConfig{}, kGateId);
  ASSERT_EQ(
    prepare_with(binding_gate, 141U, at(0ms)).code,
    ResultCode::Applied);
  const auto binding_result = binding_gate.open(
    lease_request(Operation::Open, 142U, binding_gate.snapshot()),
    at(1ms),
    [&long_detail]() {
      return OpenBinding{
      false, Reason::WriterMismatch, {}, long_detail};
    });
  EXPECT_LE(binding_result.detail.size(), 160U);
  EXPECT_LE(binding_gate.snapshot().detail.size(), 160U);

  MotionGateCore fault_gate(MotionGateConfig{}, kGateId);
  fault_gate.force_fault(Reason::InternalFailure, long_detail);
  EXPECT_LE(fault_gate.snapshot().detail.size(), 160U);
  const auto fault_result =
    fault_gate.prepare(prepare_request(identifier(143U)), at(0ms));
  EXPECT_LE(fault_result.detail.size(), 160U);
}

TEST(MotionGateCore, NegativeSignedZeroAxesAreValidCandidates)
{
  auto context = make_armed_gate();
  const double negative_zero = std::copysign(0.0, -1.0);
  Candidate candidate;
  candidate.lease_id = context.gate.snapshot().lease_id;
  candidate.writer_gid = context.writer;
  candidate.linear_x = negative_zero;
  candidate.linear_y = negative_zero;
  candidate.linear_z = negative_zero;
  candidate.angular_x = negative_zero;
  candidate.angular_y = negative_zero;
  candidate.angular_z = negative_zero;

  const auto result =
    context.gate.accept_candidate(candidate, at(1ms));

  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.retired);
  EXPECT_EQ(result.reason, Reason::None);
  EXPECT_TRUE(result.selected.is_zero());
  EXPECT_EQ(context.gate.snapshot().state, State::Armed);
  EXPECT_TRUE(context.gate.snapshot().candidate_fresh);
  EXPECT_TRUE(context.gate.snapshot().writer_bound);
}

TEST(MotionGateCore, FixedSeedEventSequenceMaintainsFailClosedInvariants)
{
  auto context = make_armed_gate();
  auto & gate = context.gate;
  std::mt19937 random(0x5a17f00dU);
  std::uint64_t next_request_number = 1000U;
  auto now = 1ms;
  bool fault_was_seen = false;

  for (std::size_t iteration = 0U; iteration < 600U; ++iteration) {
    SCOPED_TRACE(iteration);
    const auto before = gate.snapshot();
    now += std::chrono::milliseconds(random() % 41U);

    if (iteration == 500U) {
      gate.force_fault(
        Reason::InternalFailure,
        std::string(400U, 'f'));
    } else {
      switch (random() % 8U) {
        case 0U:
          (void)gate.tick(at(now));
          break;
        case 1U:
          {
            const auto request = ControlRequest{
              Operation::Prepare,
              identifier(next_request_number++),
              before.gate_instance_id,
              before.control_seq,
              ""};
            const bool admit = (random() % 4U) != 0U;
            (void)gate.prepare(
            request,
            at(now),
              [admit]() {
                return admit ?
                       PrepareAdmission{
                         true, Reason::None, "writer absent"} :
                       PrepareAdmission{
                         false, Reason::WriterStillPresent, "writer present"};
            });
            break;
          }
        case 2U:
          {
            const auto request = ControlRequest{
              Operation::Open,
              identifier(next_request_number++),
              before.gate_instance_id,
              before.control_seq,
              before.lease_id.empty() ? identifier(9000U) : before.lease_id};
            const auto binding_mode = random() % 5U;
            (void)gate.open(
            request,
            at(now),
              [binding_mode]() {
                if (binding_mode == 0U) {
                  return OpenBinding{
                    false, Reason::WriterMismatch, {}, "writer mismatch"};
                }
                if (binding_mode == 1U) {
                  return OpenBinding{
                    false, Reason::WriterAmbiguous, {}, "two writers"};
                }
                return OpenBinding{
                  true, Reason::None, writer_gid(), "writer ready"};
            });
            break;
          }
        case 3U:
          {
            const auto request = ControlRequest{
              Operation::Renew,
              identifier(next_request_number++),
              before.gate_instance_id,
              before.control_seq,
              before.lease_id.empty() ? identifier(9001U) : before.lease_id};
            (void)gate.renew(request, at(now));
            break;
          }
        case 4U:
          {
            const auto request = ControlRequest{
              Operation::Inhibit,
              identifier(next_request_number++),
              before.gate_instance_id,
              before.control_seq,
              before.lease_id.empty() ? identifier(9002U) : before.lease_id};
            (void)gate.inhibit(request, at(now));
            break;
          }
        case 5U:
          {
            Candidate candidate;
            candidate.lease_id =
              before.lease_id.empty() ? identifier(9003U) : before.lease_id;
            candidate.writer_gid =
              before.writer_bound ? before.bound_writer_gid : writer_gid();
            candidate.linear_x =
              static_cast<double>(
              static_cast<int>(random() % 161U) - 80) / 100.0;
            candidate.angular_z =
              static_cast<double>(
              static_cast<int>(random() % 401U) - 200) / 100.0;
            (void)gate.accept_candidate(candidate, at(now));
            break;
          }
        case 6U:
          {
            Candidate candidate;
            candidate.lease_id =
              before.lease_id.empty() ? identifier(9004U) : before.lease_id;
            candidate.writer_gid =
              before.writer_bound ? before.bound_writer_gid : writer_gid();
            if ((random() % 2U) == 0U) {
              candidate.writer_gid = writer_gid(0x7eU);
            } else {
              candidate.linear_y = 0.01;
            }
            (void)gate.accept_candidate(candidate, at(now));
            break;
          }
        default:
          (void)gate.tick(at(now));
          break;
      }
    }

    const auto after = gate.snapshot();
    const auto selected = gate.selected_command();
    EXPECT_GE(after.control_seq, before.control_seq);
    EXPECT_GE(after.state_seq, before.state_seq);
    EXPECT_LE(after.detail.size(), 160U);
    EXPECT_TRUE(std::isfinite(selected.linear_x));
    EXPECT_TRUE(std::isfinite(selected.angular_z));
    EXPECT_GE(selected.linear_x, -0.20);
    EXPECT_LE(selected.linear_x, 0.40);
    EXPECT_GE(selected.angular_z, -1.20);
    EXPECT_LE(selected.angular_z, 1.20);
    EXPECT_EQ(after.zero_selected, selected.is_zero());

    if (after.state == State::Armed) {
      EXPECT_FALSE(after.motion_inhibited);
      EXPECT_TRUE(after.authority_live);
      EXPECT_TRUE(after.writer_bound);
      EXPECT_FALSE(after.lease_id.empty());
      EXPECT_FALSE(after.candidate_topic.empty());
      EXPECT_TRUE(
        std::any_of(
          after.bound_writer_gid.cbegin(),
          after.bound_writer_gid.cend(),
          [](std::uint8_t value) {return value != 0U;}));
    } else {
      EXPECT_TRUE(after.motion_inhibited);
      EXPECT_FALSE(after.authority_live);
      EXPECT_FALSE(after.candidate_fresh);
      EXPECT_FALSE(after.writer_bound);
      EXPECT_TRUE(selected.is_zero());
    }

    if (after.state != State::Armed || !after.candidate_fresh) {
      EXPECT_TRUE(selected.is_zero());
    }
    if (
      after.state == State::Inhibited ||
      after.state == State::Faulted)
    {
      EXPECT_TRUE(after.lease_id.empty());
      EXPECT_TRUE(after.candidate_topic.empty());
    }
    if (after.state == State::Prepared) {
      EXPECT_FALSE(after.lease_id.empty());
      EXPECT_FALSE(after.candidate_topic.empty());
      EXPECT_TRUE(selected.is_zero());
    }

    if (fault_was_seen || before.state == State::Faulted) {
      EXPECT_EQ(after.state, State::Faulted);
    }
    fault_was_seen =
      fault_was_seen || after.state == State::Faulted;
  }

  EXPECT_TRUE(fault_was_seen);
  EXPECT_EQ(gate.snapshot().state, State::Faulted);
  EXPECT_TRUE(gate.selected_command().is_zero());
}
}  // namespace
}  // namespace voice_nav_mission
