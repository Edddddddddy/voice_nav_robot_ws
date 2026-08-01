import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "scripts" / "check_motion_gate_contract.py"

CONTROL_INTERFACE = """\
uint8 PREPARE=1
uint8 OPEN=2
uint8 RENEW=3
uint8 INHIBIT=4

uint8 operation
string<=36 request_id
string<=36 gate_instance_id
uint64 expected_control_seq
string<=36 lease_id
---
uint16 APPLIED=0
uint16 DUPLICATE=1
uint16 REJECTED=2
uint16 FAULTED=3

uint16 NONE=0
uint16 INVALID_REQUEST=1
uint16 STALE_GATE=2
uint16 STALE_SEQUENCE=3
uint16 INVALID_STATE=4
uint16 STALE_LEASE=5
uint16 REQUEST_ID_COLLISION=6
uint16 PREPARE_EXPIRED=7
uint16 AUTHORITY_EXPIRED=8
uint16 CANDIDATE_EXPIRED=9
uint16 WRITER_UNAVAILABLE=10
uint16 WRITER_AMBIGUOUS=11
uint16 WRITER_MISMATCH=12
uint16 WRITER_STILL_PRESENT=13
uint16 INVALID_CANDIDATE=14
uint16 SEQUENCE_EXHAUSTED=15
uint16 CONFIGURATION_INVALID=16
uint16 PUBLISH_FAILED=17
uint16 INTERNAL_FAILURE=18
uint16 WRITER_METADATA_PENDING=19

uint16 code
uint16 reason
string<=36 gate_instance_id
uint64 control_seq
uint8 state
string<=36 lease_id
string<=128 candidate_topic
uint8[16] bound_writer_gid
bool motion_inhibited
bool authority_live
bool candidate_fresh
bool writer_bound
bool zero_selected
bool zero_published
uint64 output_publish_seq
uint64 zero_publish_seq
string<=160 detail
"""

STATE_INTERFACE = """\
uint8 INHIBITED=0
uint8 PREPARED=1
uint8 ARMED=2
uint8 FAULTED=3

uint16 NONE=0
uint16 INVALID_REQUEST=1
uint16 STALE_GATE=2
uint16 STALE_SEQUENCE=3
uint16 INVALID_STATE=4
uint16 STALE_LEASE=5
uint16 REQUEST_ID_COLLISION=6
uint16 PREPARE_EXPIRED=7
uint16 AUTHORITY_EXPIRED=8
uint16 CANDIDATE_EXPIRED=9
uint16 WRITER_UNAVAILABLE=10
uint16 WRITER_AMBIGUOUS=11
uint16 WRITER_MISMATCH=12
uint16 WRITER_STILL_PRESENT=13
uint16 INVALID_CANDIDATE=14
uint16 SEQUENCE_EXHAUSTED=15
uint16 CONFIGURATION_INVALID=16
uint16 PUBLISH_FAILED=17
uint16 INTERNAL_FAILURE=18
uint16 WRITER_METADATA_PENDING=19

string<=36 gate_instance_id
uint64 state_seq
uint64 control_seq
uint8 state
string<=36 lease_id
string<=128 candidate_topic
uint8[16] bound_writer_gid
bool motion_inhibited
bool authority_live
bool candidate_fresh
bool writer_bound
bool zero_selected
uint64 output_publish_seq
uint64 zero_publish_seq
uint16 reason
string<=160 detail
"""

CORE_HEADER = """\
#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace voice_nav_mission
{
enum class State {Inhibited, Prepared, Armed, Faulted};
struct ControlRequest;
struct ControlResult;
struct OpenBinding;
struct OpenBindingProvider;
struct Candidate;
struct Command;

class MotionGateCore
{
public:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;
  ControlResult prepare(const ControlRequest &, SteadyTimePoint);
  ControlResult open(
    const ControlRequest &,
    SteadyTimePoint,
    const OpenBindingProvider &);
  ControlResult renew(const ControlRequest &, SteadyTimePoint);
  ControlResult inhibit(const ControlRequest &, SteadyTimePoint);
  ControlResult accept_candidate(const Candidate &, SteadyTimePoint);
  Command tick(SteadyTimePoint);

private:
  State state_{State::Inhibited};
  std::unordered_map<std::string, ControlResult> request_id_cache_;
  std::uint64_t control_seq_{0};
};
}  // namespace voice_nav_mission
"""

CORE_SOURCE = """\
#include "voice_nav_mission/motion_gate_core.hpp"

#include <algorithm>
#include <cmath>

namespace voice_nav_mission
{
ControlResult MotionGateCore::prepare(
  const ControlRequest & request,
  SteadyTimePoint now)
{
  (void)now;
  if (state_ != State::Inhibited ||
    request.expected_control_seq != control_seq_)
  {
    return stale_request();
  }
  const auto cached = request_id_cache_.find(request.request_id);
  if (cached != request_id_cache_.end()) {
    if (cached->second.request_fingerprint != request_fingerprint(request)) {
      return conflicting_request_id();
    }
    return duplicate_request(cached->second);
  }
  current_lease_id_ = make_lease_id();
  ++control_seq_;
  state_ = State::Prepared;
  return applied_result();
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
    return stale_request();
  }
  if (request.expected_control_seq != control_seq_) {
    return stale_request();
  }
  if (request.lease_id != lease_id_) {
    return stale_request();
  }
  if (now >= prepare_deadline_) {
    return stale_request();
  }
  if (!binding_provider) {
    return stale_request();
  }
  const auto binding = binding_provider();
  if (!binding.ready) {
    return stale_request();
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
    return stale_request();
  }
  bound_writer_gid_ = binding.writer_gid;
  writer_bound_ = true;
  authority_deadline_ = now + authority_lease_;
  candidate_deadline_ = now + candidate_freshness_;
  ++control_seq_;
  state_ = State::Armed;
  return applied_result();
}

ControlResult MotionGateCore::renew(
  const ControlRequest & request,
  SteadyTimePoint now)
{
  if (state_ != State::Armed ||
    request.expected_control_seq != control_seq_ ||
    now >= authority_deadline_)
  {
    return stale_request();
  }
  ++control_seq_;
  authority_deadline_ = now + authority_lease_;
  return applied_result();
}

ControlResult MotionGateCore::inhibit(
  const ControlRequest & request,
  SteadyTimePoint now)
{
  (void)now;
  const auto cached = request_id_cache_.find(request.request_id);
  if (cached != request_id_cache_.end()) {
    return duplicate_request(cached->second);
  }
  return retire_lease(request);
}

ControlResult MotionGateCore::accept_candidate(
  const Candidate & candidate,
  SteadyTimePoint now)
{
  const bool finite =
    std::isfinite(candidate.linear_x) &&
    std::isfinite(candidate.linear_y) &&
    std::isfinite(candidate.linear_z) &&
    std::isfinite(candidate.angular_x) &&
    std::isfinite(candidate.angular_y) &&
    std::isfinite(candidate.angular_z);
  if (!finite ||
    candidate.linear_y != 0.0 ||
    candidate.linear_z != 0.0 ||
    candidate.angular_x != 0.0 ||
    candidate.angular_y != 0.0)
  {
    return retire_lease(candidate);
  }
  selected_.linear_x =
    std::clamp(candidate.linear_x, linear_x_min_, linear_x_max_);
  selected_.angular_z =
    std::clamp(candidate.angular_z, angular_z_min_, angular_z_max_);
  candidate_deadline_ = now + candidate_freshness_;
  return applied_result();
}

Command MotionGateCore::tick(SteadyTimePoint now)
{
  if (now >= authority_deadline_ || now >= candidate_deadline_) {
    retire_lease(timeout_reason());
    return zero_command();
  }
  return selected_;
}
}  // namespace voice_nav_mission
"""

WRITER_OBSERVATION_HEADER = """\
#pragma once

#include "voice_nav_mission/motion_gate_core.hpp"
#include <rmw/types.h>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace voice_nav_mission
{
struct WriterEndpointObservation
{
  std::string topic_type;
  std::string node_name;
  std::string node_namespace;
  rmw_endpoint_type_t endpoint_type;
  rmw_qos_profile_t qos;
  WriterGid writer_gid;
};

struct WriterObservationPolicy
{
  std::string expected_topic_type;
  std::string expected_writer_fqn;
};

class WriterObservationSession
{
public:
  explicit WriterObservationSession(WriterObservationPolicy policy);
  OpenBinding observe(
    const std::vector<WriterEndpointObservation> & endpoints,
    std::chrono::milliseconds elapsed);
  void reset() noexcept;

private:
  WriterObservationPolicy policy_;
  std::optional<WriterGid> pinned_writer_gid_;
  bool identity_confirmed_{false};
  bool terminal_mismatch_{false};
  std::string terminal_detail_;
};
}  // namespace voice_nav_mission
"""

WRITER_OBSERVATION_SOURCE = """\
#include "writer_observation.hpp"

#include <algorithm>

namespace voice_nav_mission
{
namespace
{
constexpr std::size_t kMaximumDetailLength = 160U;
constexpr char kUnknownNodeName[] = "_NODE_NAME_UNKNOWN_";
constexpr char kUnknownNodeNamespace[] = "_NODE_NAMESPACE_UNKNOWN_";

std::string bounded_detail(std::string detail)
{
  if (detail.size() > kMaximumDetailLength) {
    detail.resize(kMaximumDetailLength);
  }
  return detail;
}

bool gid_is_zero(const WriterGid & gid)
{
  return std::all_of(
    gid.cbegin(), gid.cend(),
    [](std::uint8_t value) {return value == 0U;});
}

bool candidate_qos_is_compatible(const rmw_qos_profile_t & qos)
{
  return qos.depth == 1U;
}

std::string normalized_namespace(std::string node_namespace)
{
  return node_namespace.empty() ? "/" : node_namespace;
}

bool node_name_is_unresolved(const std::string & node_name)
{
  return node_name.empty() || node_name == kUnknownNodeName;
}

bool node_namespace_is_unresolved(const std::string & node_namespace)
{
  return node_namespace == kUnknownNodeNamespace;
}

std::string endpoint_fqn(const WriterEndpointObservation & endpoint)
{
  return normalized_namespace(endpoint.node_namespace) + endpoint.node_name;
}

std::string fqn_namespace(const std::string & fqn)
{
  return fqn.substr(0U, fqn.rfind('/') + 1U);
}

std::string fqn_name(const std::string & fqn)
{
  return fqn.substr(fqn.rfind('/') + 1U);
}

std::string observation_summary(
  const WriterEndpointObservation & endpoint,
  std::chrono::milliseconds elapsed)
{
  return
    "n=1 k=" + std::to_string(endpoint.endpoint_type) +
    " id=" + endpoint.node_name +
    " q=" + std::to_string(endpoint.qos.depth) +
    " g=" + std::to_string(endpoint.writer_gid.back()) +
    " ms=" + std::to_string(elapsed.count()) +
    " t=" + endpoint.topic_type;
}

OpenBinding mismatch(std::string detail)
{
  return {false, Reason::WriterMismatch, {}, bounded_detail(detail)};
}
}  // namespace

OpenBinding WriterObservationSession::observe(
  const std::vector<WriterEndpointObservation> & endpoints,
  std::chrono::milliseconds elapsed)
{
  if (terminal_mismatch_) {
    return mismatch(terminal_detail_);
  }
  const auto reject_mismatch = [this](std::string detail) {
      if (pinned_writer_gid_) {
        terminal_mismatch_ = true;
        terminal_detail_ = detail;
      }
      return mismatch(detail);
    };
  if (endpoints.empty()) {
    return {false, Reason::WriterUnavailable, {}, "no writer"};
  }
  if (endpoints.size() != 1U) {
    return {false, Reason::WriterAmbiguous, {}, "ambiguous"};
  }
  const auto & endpoint = endpoints.front();
  const auto summary = observation_summary(endpoint, elapsed);
  if (endpoint.endpoint_type != RMW_ENDPOINT_PUBLISHER) {
    return reject_mismatch("wrong endpoint kind");
  }
  if (endpoint.topic_type != policy_.expected_topic_type) {
    return reject_mismatch("wrong type");
  }
  if (!candidate_qos_is_compatible(endpoint.qos)) {
    return reject_mismatch("wrong qos");
  }
  if (gid_is_zero(endpoint.writer_gid)) {
    return reject_mismatch("zero gid");
  }
  if (pinned_writer_gid_ &&
    *pinned_writer_gid_ != endpoint.writer_gid)
  {
    return reject_mismatch("replacement writer");
  }
  const bool name_unresolved =
    node_name_is_unresolved(endpoint.node_name);
  const bool namespace_unresolved =
    node_namespace_is_unresolved(endpoint.node_namespace);
  const auto expected_namespace =
    fqn_namespace(policy_.expected_writer_fqn);
  const auto expected_name =
    fqn_name(policy_.expected_writer_fqn);
  if (!name_unresolved && endpoint.node_name != expected_name) {
    return reject_mismatch("partial node name mismatch; " + summary);
  }
  if (!namespace_unresolved) {
    const auto observed_namespace =
      normalized_namespace(endpoint.node_namespace);
    if (observed_namespace != expected_namespace) {
      return reject_mismatch("partial identity mismatch; " + summary);
    }
  }
  if (name_unresolved || namespace_unresolved) {
    if (identity_confirmed_) {
      return {true, Reason::None, endpoint.writer_gid, "retained"};
    }
    if (!pinned_writer_gid_) {
      pinned_writer_gid_ = endpoint.writer_gid;
    }
    return {
      false,
      Reason::WriterMetadataPending,
      endpoint.writer_gid,
      bounded_detail("identity unresolved; " + summary)};
  }
  const auto observed_fqn = endpoint_fqn(endpoint);
  if (observed_fqn != policy_.expected_writer_fqn) {
    return reject_mismatch("wrong fqn");
  }
  if (!pinned_writer_gid_) {
    pinned_writer_gid_ = endpoint.writer_gid;
  }
  identity_confirmed_ = true;
  return {true, Reason::None, endpoint.writer_gid, "ready"};
}

void WriterObservationSession::reset() noexcept
{
  pinned_writer_gid_.reset();
  identity_confirmed_ = false;
  terminal_mismatch_ = false;
  terminal_detail_.clear();
}
}  // namespace voice_nav_mission
"""

WRITER_OBSERVATION_TEST = """\
#include "writer_observation.hpp"
#include <gtest/gtest.h>

TEST(WriterObservationSession, PinsUnresolvedIdentityUntilTheSameWriterResolves)
{
  EXPECT_EQ(pending.reason, Reason::WriterMetadataPending);
  EXPECT_LE(pending.detail.size(), 160U);
  for (const auto * field : {"n=1", "t=", "id=", "q=", "g=", "ms=7"}) {
    EXPECT_NE(pending.detail.find(field), std::string::npos);
  }
}

TEST(WriterObservationSession, ReplacementPoisonsPinnedGenerationUntilReset)
{
  EXPECT_EQ(replacement.reason, Reason::WriterMismatch);
  session.reset();
}

TEST(WriterObservationSession, ConfirmedSameGidSurvivesIdentityOnlyGraphRegression)
{
  EXPECT_TRUE(regressed.ready);
}

TEST(WriterObservationSession, KnownWrongNamespaceCannotEnterPending)
{
  EXPECT_EQ(result.reason, Reason::WriterMismatch);
}

TEST(WriterObservationSession, ExactUnknownIdentityMarkersConvergeForPinnedGid)
{
  const auto name = "_NODE_NAME_UNKNOWN_";
  const auto ns = "_NODE_NAMESPACE_UNKNOWN_";
  EXPECT_EQ(pending.reason, Reason::WriterMetadataPending);
}

TEST(WriterObservationSession, KnownPartialIdentityMustAgreeBeforePending)
{
  EXPECT_EQ(contradiction.reason, Reason::WriterMismatch);
}
"""

NODE_SOURCE = """\
#include <chrono>
#include <mutex>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "writer_observation.hpp"

using namespace std::chrono_literals;
static_assert(RMW_GID_STORAGE_SIZE == 16u);

class MotionGateNode : public rclcpp::Node
{
public:
  MotionGateNode()
  : Node("motion_gate_node"),
    writer_observation_session_({
      "geometry_msgs/msg/TwistStamped",
      "/collision_monitor"})
  {
    auto candidate_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort().durability_volatile();
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable().transient_local();
    rcl_interfaces::msg::ParameterDescriptor safety_descriptor;
    safety_descriptor.read_only = true;
    (void)candidate_qos;
    (void)state_qos;
    (void)safety_descriptor;
    (void)"/motion_gate/internal/control";
    (void)"/motion_gate/internal/state";
    (void)"/voice_nav_internal/motion_gate/candidate/lease_";
    (void)"/diff_drive_controller/cmd_vel";
    use_sim_time_guard_ = add_on_set_parameters_callback(
      [](const auto &) {
        SetParametersResult result;
        result.successful = false;
        result.reason =
          "MotionGate use_sim_time is immutable after startup";
        return result;
      });
    final_command_publisher_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(
      final_command_topic_, rclcpp::SystemDefaultsQoS());
    state_publisher_ =
      create_publisher<StateMessage>(
      "/motion_gate/internal/state", state_qos);
    output_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() {
        const auto now = std::chrono::steady_clock::now();
        publish_serialized(core_.tick(now));
      });
  }

  OpenBinding
  discover_unique_writer_gid_on_topic(const std::string & topic)
  {
    const auto endpoints = get_publishers_info_by_topic(topic);
    std::vector<WriterEndpointObservation> observations;
    observations.reserve(endpoints.size());
    for (const auto & endpoint : endpoints) {
      WriterGid writer_gid{};
      const auto & endpoint_gid = endpoint.endpoint_gid();
      std::copy(
        endpoint_gid.cbegin(), endpoint_gid.cend(), writer_gid.begin());
      observations.push_back(WriterEndpointObservation{
        endpoint.topic_type(),
        endpoint.node_name(),
        endpoint.node_namespace(),
        static_cast<rmw_endpoint_type_t>(endpoint.endpoint_type()),
        endpoint.qos_profile().get_rmw_qos_profile(),
        writer_gid});
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - writer_observation_started_at_);
    return writer_observation_session_.observe(observations, elapsed);
  }

  std::optional<std::string> final_controller_health_error() const
  {
    return std::nullopt;
  }

  void open_candidate_reader(
    const ControlRequest & request,
    ControlResponse & response)
  {
    OpenBinding expected_binding;
    const auto result = core_.open(
      request,
      std::chrono::steady_clock::now(),
      [this, &request, &expected_binding]() {
        if (const auto error = final_controller_health_error()) {
          return OpenBinding{
            false, Reason::WriterUnavailable, {}, *error};
        }
        const auto first =
          discover_unique_writer_gid_on_topic(candidate_topic_);
        if (!first.ready) {
          return first;
        }
        candidate_subscription_.reset();
        candidate_subscription_ =
          create_candidate_subscription(
          candidate_topic_, request.lease_id, true);
        const auto second =
          discover_unique_writer_gid_on_topic(candidate_topic_);
        if (second.writer_gid != first.writer_gid) {
          return OpenBinding{};
        }
        expected_binding = first;
        return first;
      });
    if (result.code != ResultCode::Applied) {
      return;
    }
    candidate_subscription_.reset();
    candidate_subscription_ =
      create_candidate_subscription(
      candidate_topic_, request.lease_id, false);
    const auto third =
      discover_unique_writer_gid_on_topic(candidate_topic_);
    (void)third;
    (void)expected_binding;
    response.code = response.APPLIED;
  }

  ControlResult handle_prepare(
    const ControlRequest & request,
    SteadyTimePoint now)
  {
    auto result = core_.prepare(request, now);
    if (result.code == ResultCode::Applied) {
      writer_observation_session_.reset();
      writer_observation_started_at_ = now;
      candidate_subscription_ = create_candidate_subscription(
        result.candidate_topic, result.lease_id, true);
    }
    return result;
  }

  void on_candidate(
    const geometry_msgs::msg::TwistStamped & message,
    const rclcpp::MessageInfo & message_info)
  {
    const auto & publisher_gid =
      message_info.get_rmw_message_info().publisher_gid;
    core_.accept_candidate(
      to_candidate(message, publisher_gid),
      std::chrono::steady_clock::now());
  }

  void publish_serialized(Command command)
  {
    std::scoped_lock lock(publication_mutex_);
    if (!get_parameter("use_sim_time").as_bool() ||
      !get_clock()->ros_time_is_active())
    {
      core_.force_fault(
        Reason::ConfigurationInvalid,
        "use_sim_time runtime invariant was violated");
      command = Command{};
      command.header.stamp.sec = 0;
    } else {
      command.header.stamp = get_clock()->now();
    }
    final_command_publisher_->publish(command);
  }

  ControlResult handle_inhibit(
    const ControlRequest & request,
    SteadyTimePoint now)
  {
    const auto before = core_.snapshot();
    const auto result = core_.inhibit(request, now);
    const auto after = core_.snapshot();
    reconcile_adapter_transition(before, after);
    return result;
  }

  void fill_response(
    const ControlResult & result,
    ControlResponse & response,
    bool zero_published)
  {
    response.motion_inhibited = result.motion_inhibited;
    response.zero_published = zero_published;
    response.output_publish_seq = output_publish_seq_;
    response.zero_publish_seq = zero_publish_seq_;
  }

  void on_control(
    const ControlRequest & request,
    ControlResponse & response)
  {
    ControlResult result;
    switch (request.operation) {
      case Operation::Inhibit:
        result = handle_inhibit(
          request, std::chrono::steady_clock::now());
        break;
    }
    const auto command =
      core_.tick(std::chrono::steady_clock::now());
    publish_serialized(command);
    publish_state();
    fill_response(result, response, command.is_zero());
  }

private:
  std::mutex publication_mutex_;
  WriterObservationSession writer_observation_session_;
  MotionGateCore::SteadyTimePoint writer_observation_started_at_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(std::make_shared<MotionGateNode>());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
"""

MISSION_PACKAGE = """\
<?xml version="1.0"?>
<package format="3">
  <name>voice_nav_mission</name>
  <version>0.1.0</version>
  <description>MotionGate contract fixture</description>
  <maintainer email="test@example.com">Test Maintainer</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>geometry_msgs</depend>
  <depend>rcl_interfaces</depend>
  <depend>rclcpp</depend>
  <depend>rmw</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <exec_depend>rmw_fastrtps_cpp</exec_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_cmake_ros</test_depend>
  <test_depend>launch_ros</test_depend>
  <test_depend>launch_testing</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
"""

MISSION_CMAKE = """\
cmake_minimum_required(VERSION 3.22)
project(voice_nav_mission)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InternalMotionGateState.msg"
  "srv/InternalMotionGateControl.srv"
)

add_library(motion_gate_core STATIC src/motion_gate_core.cpp)
add_executable(
  motion_gate_node
  src/motion_gate_node.cpp
  src/writer_observation.cpp
)
rosidl_get_typesupport_target(
  motion_gate_typesupport
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)
target_link_libraries(motion_gate_node motion_gate_core "${motion_gate_typesupport}")

if(BUILD_TESTING)
  ament_add_gtest(motion_gate_core_test test/motion_gate_core_test.cpp)
  ament_add_gtest(
    writer_observation_test
    test/writer_observation_test.cpp
    src/writer_observation.cpp
  )
  add_launch_test(
    test/test_motion_gate_node.py
    TIMEOUT 60
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  set_tests_properties(
    test_test_motion_gate_node.py
    PROPERTIES
      RUN_SERIAL TRUE
  )
endif()

install(
  TARGETS motion_gate_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
"""

GATE_CONFIG = """\
motion_gate_node:
  ros__parameters:
    use_sim_time: true
    output_frequency_hz: 50.0
    authority_lease_ms: 250
    candidate_freshness_ms: 150
    prepare_timeout_ms: 1000
    writer_graph_timeout_ms: 1000
    candidate_qos_depth: 1
    expected_candidate_writer_fqn: /collision_monitor
    request_cache_size: 64
    linear_x_min: -0.20
    linear_x_max: 0.40
    angular_z_min: -1.20
    angular_z_max: 1.20
"""

PRODUCT_LAUNCH = """\
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    locked_rmw = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_fastrtps_cpp',
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('voice_nav_sim'),
                'launch',
                'simulation.launch.py',
            ])
        )
    )
    gate_config = PathJoinSubstitution([
        FindPackageShare('voice_nav_bringup'),
        'config',
        'motion_gate.yaml',
    ])
    motion_gate = Node(
        package='voice_nav_mission',
        executable='motion_gate_node',
        name='motion_gate_node',
        parameters=[gate_config],
    )
    return LaunchDescription([locked_rmw, simulation, motion_gate])
"""

BRINGUP_PACKAGE = """\
<?xml version="1.0"?>
<package format="3">
  <name>voice_nav_bringup</name>
  <version>0.1.0</version>
  <description>MotionGate composition fixture</description>
  <maintainer email="test@example.com">Test Maintainer</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>rmw_fastrtps_cpp</exec_depend>
  <exec_depend>voice_nav_mission</exec_depend>
  <exec_depend>voice_nav_sim</exec_depend>
</package>
"""

BRINGUP_CMAKE = """\
cmake_minimum_required(VERSION 3.22)
project(voice_nav_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
if(BUILD_TESTING)
  add_launch_test(
    test/test_motion_gate_product.py
    TIMEOUT 180
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  set_tests_properties(
    test_test_motion_gate_product.py
    PROPERTIES
      RUN_SERIAL TRUE
  )
endif()
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
"""

OPEN_CONVERGENCE = """\
def converge_open(
    *,
    expected,
    protocol,
    attempt,
    new_request_id,
    deadline,
    now,
    sleep,
    backoff_seconds,
):
    last_response = None
    attempts = 0
    seen_request_ids = set()
    while True:
        remaining = deadline - now()
        if remaining <= 0.0:
            raise OpenConvergenceTimeout(last_response, attempts)
        request_id = new_request_id()
        seen_request_ids.add(request_id)
        response = attempt(request_id, remaining)
        last_response = response
        attempts += 1
        remaining = deadline - now()
        if remaining <= 0.0:
            raise OpenConvergenceTimeout(last_response, attempts)
        if not _is_writer_discovery_pending(response, protocol):
            return response
        _validate_pending_snapshot(response, expected, protocol)
        remaining = deadline - now()
        if remaining <= 0.0:
            raise OpenConvergenceTimeout(last_response, attempts)
        sleep(min(backoff_seconds[0], remaining))
"""

CONTROLLERS = """\
controller_manager:
  ros__parameters:
    update_rate: 100

diff_drive_controller:
  ros__parameters:
    cmd_vel_timeout: 0.35
    linear.x.min_velocity: -0.20
    linear.x.max_velocity: 0.40
    angular.z.min_velocity: -1.20
    angular.z.max_velocity: 1.20
"""

FIXTURE_FILES = {
    "src/voice_nav_mission/srv/InternalMotionGateControl.srv": (
        CONTROL_INTERFACE
    ),
    "src/voice_nav_mission/msg/InternalMotionGateState.msg": (
        STATE_INTERFACE
    ),
    (
        "src/voice_nav_mission/include/voice_nav_mission/"
        "motion_gate_core.hpp"
    ): CORE_HEADER,
    "src/voice_nav_mission/src/motion_gate_core.cpp": CORE_SOURCE,
    (
        "src/voice_nav_mission/src/writer_observation.hpp"
    ): WRITER_OBSERVATION_HEADER,
    (
        "src/voice_nav_mission/src/writer_observation.cpp"
    ): WRITER_OBSERVATION_SOURCE,
    (
        "src/voice_nav_mission/test/writer_observation_test.cpp"
    ): WRITER_OBSERVATION_TEST,
    "src/voice_nav_mission/src/motion_gate_node.cpp": NODE_SOURCE,
    "src/voice_nav_mission/package.xml": MISSION_PACKAGE,
    "src/voice_nav_mission/CMakeLists.txt": MISSION_CMAKE,
    "src/voice_nav_bringup/config/motion_gate.yaml": GATE_CONFIG,
    (
        "src/voice_nav_bringup/launch/product_sim.launch.py"
    ): PRODUCT_LAUNCH,
    "src/voice_nav_bringup/package.xml": BRINGUP_PACKAGE,
    "src/voice_nav_bringup/CMakeLists.txt": BRINGUP_CMAKE,
    (
        "src/voice_nav_bringup/test/"
        "motion_gate_open_convergence.py"
    ): OPEN_CONVERGENCE,
    "src/voice_nav_sim/config/controllers.yaml": CONTROLLERS,
}


class MotionGateContractTest(unittest.TestCase):
    def create_fixture(self, root: Path) -> None:
        for relative_path, contents in FIXTURE_FILES.items():
            path = root / relative_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                textwrap.dedent(contents),
                encoding="utf-8",
            )

    def run_checker(
        self,
        mutation=None,
    ) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.create_fixture(root)
            if mutation is not None:
                mutation(root)
            return subprocess.run(
                [
                    sys.executable,
                    str(CHECKER),
                    "--root",
                    str(root),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

    @staticmethod
    def replace(
        root: Path,
        relative_path: str,
        old: str,
        new: str,
    ) -> None:
        path = root / relative_path
        source = path.read_text(encoding="utf-8")
        if old not in source:
            raise AssertionError(
                f"fixture mutation source not found in {relative_path}: {old}"
            )
        path.write_text(source.replace(old, new, 1), encoding="utf-8")

    def test_synthetic_valid_contract_passes(self) -> None:
        completed = self.run_checker()

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("MotionGate contract passed", completed.stdout)

    def test_repository_motion_gate_contract_passes(self) -> None:
        completed = subprocess.run(
            [
                sys.executable,
                str(CHECKER),
                "--root",
                str(REPOSITORY_ROOT),
            ],
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_node_must_delegate_gate_local_snapshot_to_observation_session(
        self,
    ) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_node.cpp",
                (
                    "return writer_observation_session_.observe("
                    "observations, elapsed);"
                ),
                "return OpenBinding{};",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("Gate-local TopicEndpointInfo adapter", completed.stderr)

    def test_pending_identity_must_pin_the_nonzero_writer_gid(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/writer_observation.cpp",
                "pinned_writer_gid_ = endpoint.writer_gid;",
                "// unresolved identity is not pinned",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("unresolved node-identity branch", completed.stderr)

    def test_writer_mismatch_cannot_be_broadened_into_typed_pending(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/writer_observation.cpp",
                "Reason::WriterMismatch",
                "Reason::WriterMetadataPending",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("WriterObservationSession implementation", completed.stderr)

    def test_open_attempt_deadline_precedes_all_response_classification(
        self,
    ) -> None:
        deadline_block = (
            "        remaining = deadline - now()\n"
            "        if remaining <= 0.0:\n"
            "            raise OpenConvergenceTimeout("
            "last_response, attempts)\n"
        )
        transition = (
            "        attempts += 1\n"
            + deadline_block
            + "        if not _is_writer_discovery_pending("
            "response, protocol):\n"
            "            return response\n"
            "        _validate_pending_snapshot("
            "response, expected, protocol)"
        )
        mutations = (
            (
                transition,
                (
                    "        attempts += 1\n"
                    "        if not _is_writer_discovery_pending("
                    "response, protocol):\n"
                    "            return response\n"
                    "        _validate_pending_snapshot("
                    "response, expected, protocol)"
                ),
            ),
            (
                transition,
                (
                    "        attempts += 1\n"
                    "        if not _is_writer_discovery_pending("
                    "response, protocol):\n"
                    "            return response\n"
                    + deadline_block
                    + "        _validate_pending_snapshot("
                    "response, expected, protocol)"
                ),
            ),
        )
        for old, new in mutations:
            with self.subTest(mutation=new):
                def mutation(
                    root: Path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(
                        root,
                        (
                            "src/voice_nav_bringup/test/"
                            "motion_gate_open_convergence.py"
                        ),
                        old_value,
                        new_value,
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    "immediate post-attempt deadline",
                    completed.stderr,
                )

    def test_core_target_is_static_and_never_installed_or_exported(self) -> None:
        mutations = (
            (
                "add_library(motion_gate_core STATIC src/motion_gate_core.cpp)",
                "add_library(motion_gate_core SHARED src/motion_gate_core.cpp)",
                "motion_gate_core must be one internal STATIC target",
            ),
            (
                "TARGETS motion_gate_node",
                "TARGETS motion_gate_core motion_gate_node",
                "motion_gate_core must not be installed",
            ),
            (
                "ament_package()",
                (
                    "ament_export_targets(export_motion_gate_core "
                    "HAS_LIBRARY_TARGET)\n"
                    "ament_package()"
                ),
                "motion_gate_core must not be exported",
            ),
            (
                "ament_package()",
                (
                    "install(FILES "
                    "include/voice_nav_mission/motion_gate_core.hpp "
                    "DESTINATION include/voice_nav_mission)\n"
                    "ament_package()"
                ),
                "motion_gate_core.hpp must not be installed",
            ),
        )
        for old, new, diagnostic in mutations:
            with self.subTest(mutation=diagnostic):
                def mutation(
                    root: Path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(
                        root,
                        "src/voice_nav_mission/CMakeLists.txt",
                        old_value,
                        new_value,
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(diagnostic, completed.stderr)

    def test_state_diagnostic_gid_width_is_pinned_to_jazzy_rmw(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/msg/InternalMotionGateState.msg",
                "uint8[16] bound_writer_gid",
                "uint8[24] bound_writer_gid",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("uint8[16] bound_writer_gid", completed.stderr)

    def test_control_request_cannot_carry_cross_process_gid(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/srv/InternalMotionGateControl.srv",
                "uint64 expected_control_seq",
                (
                    "uint64 expected_control_seq\n"
                    "uint8[16] writer_gid"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must not carry a cross-process writer GID", completed.stderr)

    def test_internal_strings_must_be_bounded(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/srv/InternalMotionGateControl.srv",
                "string<=36 request_id",
                "string request_id",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("unbounded strings", completed.stderr)

    def test_private_idl_fields_are_closed_on_every_side(self) -> None:
        mutations = (
            (
                "src/voice_nav_mission/srv/InternalMotionGateControl.srv",
                "uint64 expected_control_seq",
                "uint64 expected_control_seq\nbool rogue_request_field",
                "request",
            ),
            (
                "src/voice_nav_mission/srv/InternalMotionGateControl.srv",
                "string<=160 detail",
                "string<=160 detail\nbool rogue_response_field",
                "response",
            ),
            (
                "src/voice_nav_mission/msg/InternalMotionGateState.msg",
                "string<=160 detail",
                "string<=160 detail\nbool rogue_state_field",
                "InternalMotionGateState.msg",
            ),
        )
        for relative_path, old, new, diagnostic in mutations:
            with self.subTest(section=diagnostic):
                def mutation(
                    root: Path,
                    path: str = relative_path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(root, path, old_value, new_value)

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn("closed private protocol", completed.stderr)
                self.assertIn(diagnostic, completed.stderr)

    def test_private_idl_rejects_unbounded_sequences(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/msg/InternalMotionGateState.msg",
                "uint8[16] bound_writer_gid",
                "uint8[] bound_writer_gid",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("unbounded sequences", completed.stderr)

    def test_internal_idl_cannot_leak_into_public_package(self) -> None:
        def mutation(root: Path) -> None:
            leaked = (
                root
                / "src"
                / "voice_nav_interfaces"
                / "srv"
                / "InternalMotionGateControl.srv"
            )
            leaked.parent.mkdir(parents=True)
            leaked.write_text(CONTROL_INTERFACE, encoding="utf-8")

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must not be duplicated", completed.stderr)

    def test_exact_output_and_deadman_periods_are_enforced(self) -> None:
        mutations = (
            ("use_sim_time: true", "use_sim_time: false"),
            ("output_frequency_hz: 50.0", "output_frequency_hz: 20.0"),
            ("authority_lease_ms: 250", "authority_lease_ms: 350"),
            (
                "candidate_freshness_ms: 150",
                "candidate_freshness_ms: 250",
            ),
        )
        for old, new in mutations:
            with self.subTest(parameter=old.split(":", maxsplit=1)[0]):
                def mutation(
                    root: Path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(
                        root,
                        "src/voice_nav_bringup/config/motion_gate.yaml",
                        old_value,
                        new_value,
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn("MotionGate parameter", completed.stderr)

    def test_use_sim_time_is_immutable_and_fail_closed_at_publication(self) -> None:
        mutations = (
            (
                "add_on_set_parameters_callback",
                "add_post_set_parameters_callback",
                "add_on_set_parameters_callback",
            ),
            (
                "MotionGate use_sim_time is immutable after startup",
                "MotionGate clock policy changed",
                "MotionGate use_sim_time is immutable after startup",
            ),
            (
                "use_sim_time runtime invariant was violated",
                "clock invariant ignored",
                "use_sim_time runtime invariant was violated",
            ),
            (
                "ros_time_is_active()",
                "system_time_is_active()",
                "ros_time_is_active()",
            ),
            (
                "command.header.stamp.sec = 0",
                "command.header.stamp.sec = 1",
                "command.header.stamp.sec = 0",
            ),
        )
        for old, new, diagnostic in mutations:
            with self.subTest(contract=diagnostic):
                def mutation(
                    root: Path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(
                        root,
                        "src/voice_nav_mission/src/motion_gate_node.cpp",
                        old_value,
                        new_value,
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(diagnostic, completed.stderr)

    def test_candidate_and_final_command_qos_are_explicit(self) -> None:
        mutations = (
            (
                ".best_effort().durability_volatile()",
                ".reliable().durability_volatile()",
                ".best_effort()",
            ),
            (
                "rclcpp::SystemDefaultsQoS()",
                "rclcpp::QoS(1)",
                "rclcpp::SystemDefaultsQoS()",
            ),
        )
        for old, new, diagnostic in mutations:
            with self.subTest(contract=diagnostic):
                def mutation(
                    root: Path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(
                        root,
                        "src/voice_nav_mission/src/motion_gate_node.cpp",
                        old_value,
                        new_value,
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(diagnostic, completed.stderr)

    def test_state_qos_cannot_be_satisfied_by_candidate_tokens(self) -> None:
        def mutation(root: Path) -> None:
            path = (
                root
                / "src"
                / "voice_nav_mission"
                / "src"
                / "motion_gate_node.cpp"
            )
            source = path.read_text(encoding="utf-8")
            source = source.replace(
                ".reliable().transient_local();",
                ".best_effort().durability_volatile();",
                1,
            )
            source = source.replace(
                ".best_effort().durability_volatile();",
                (
                    ".best_effort().durability_volatile()"
                    ".reliable().transient_local();"
                ),
                1,
            )
            path.write_text(source, encoding="utf-8")

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "MotionGate state publisher construction",
            completed.stderr,
        )

    def test_package_dependencies_pin_test_and_runtime_owners(self) -> None:
        mutations = (
            (
                "src/voice_nav_mission/package.xml",
                "<test_depend>launch_ros</test_depend>",
                "<exec_depend>launch_ros</exec_depend>",
                "launch_ros",
            ),
            (
                "src/voice_nav_mission/package.xml",
                "<exec_depend>rmw_fastrtps_cpp</exec_depend>",
                "<depend>rmw_fastrtps_cpp</depend>",
                "node runtime-checks",
            ),
            (
                "src/voice_nav_bringup/package.xml",
                "<exec_depend>rmw_fastrtps_cpp</exec_depend>",
                "<depend>rmw_fastrtps_cpp</depend>",
                "product_sim.launch.py selects",
            ),
        )
        for relative_path, old, new, diagnostic in mutations:
            with self.subTest(path=relative_path, dependency=old):
                def mutation(
                    root: Path,
                    path: str = relative_path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(root, path, old_value, new_value)

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(diagnostic, completed.stderr)

    def test_each_package_registers_one_serial_launch_test(self) -> None:
        specifications = (
            (
                "voice_nav_mission",
                "src/voice_nav_mission/CMakeLists.txt",
                "test/test_motion_gate_node.py",
                "60",
                "test_test_motion_gate_node.py",
            ),
            (
                "voice_nav_bringup",
                "src/voice_nav_bringup/CMakeLists.txt",
                "test/test_motion_gate_product.py",
                "180",
                "test_test_motion_gate_product.py",
            ),
        )
        for package, relative_path, test_path, timeout, test_name in (
            specifications
        ):
            mutations = (
                (
                    f"TIMEOUT {timeout}",
                    f"TIMEOUT {int(timeout) + 1}",
                    "TIMEOUT",
                ),
                (
                    "RUN_SERIAL TRUE",
                    "RUN_SERIAL FALSE",
                    "RUN_SERIAL TRUE",
                ),
                (
                    "      RUN_SERIAL TRUE\n  )",
                    (
                        "      RUN_SERIAL TRUE\n  )\n"
                        "  set_tests_properties(\n"
                        f"    {test_name}\n"
                        "    PROPERTIES\n"
                        "      DISABLED TRUE\n"
                        "  )"
                    ),
                    "must remain enabled",
                ),
                (
                    "${ament_cmake_ros_DIR}/run_test_isolated.py",
                    "${ament_cmake_ros_DIR}/run_test.py",
                    "isolated RUNNER",
                ),
                (
                    (
                        "add_launch_test(\n"
                        f"    {test_path}\n"
                        f"    TIMEOUT {timeout}\n"
                        "    RUNNER "
                        '"${ament_cmake_ros_DIR}/run_test_isolated.py"\n'
                        "  )"
                    ),
                    (
                        "add_launch_test(\n"
                        f"    {test_path}\n"
                        f"    TIMEOUT {timeout}\n"
                        "    RUNNER "
                        '"${ament_cmake_ros_DIR}/run_test_isolated.py"\n'
                        "  )\n"
                        "  add_launch_test(\n"
                        f"    {test_path}\n"
                        f"    TIMEOUT {timeout}\n"
                        "    RUNNER "
                        '"${ament_cmake_ros_DIR}/run_test_isolated.py"\n'
                        "  )"
                    ),
                    "exactly one add_launch_test",
                ),
            )
            for old, new, diagnostic in mutations:
                with self.subTest(package=package, mutation=diagnostic):
                    def mutation(
                        root: Path,
                        path: str = relative_path,
                        old_value: str = old,
                        new_value: str = new,
                    ) -> None:
                        self.replace(root, path, old_value, new_value)

                    completed = self.run_checker(mutation)

                    self.assertNotEqual(completed.returncode, 0)
                    self.assertIn(diagnostic, completed.stderr)

    def test_gate_limits_cannot_exceed_controller_limits(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/config/controllers.yaml",
                "linear.x.max_velocity: 0.40",
                "linear.x.max_velocity: 0.30",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("wider than controller", completed.stderr)

    def test_same_package_rosidl_typesupport_link_is_required(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/CMakeLists.txt",
                "rosidl_get_typesupport_target(",
                "rosidl_target_interfaces(",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("rosidl_get_typesupport_target", completed.stderr)

    def test_core_deadlines_cannot_use_ros_time(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_core.cpp",
                '#include "voice_nav_mission/motion_gate_core.hpp"',
                (
                    '#include "voice_nav_mission/motion_gate_core.hpp"\n'
                    "// rclcpp::Clock must not drive core deadlines"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("injected steady time", completed.stderr)

    def test_prepare_requires_cas_and_conflicting_id_detection(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_core.cpp",
                "cached->second.request_fingerprint",
                "cached->second.unchecked_request_body",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("request_fingerprint", completed.stderr)

    def test_candidate_data_cannot_renew_authority(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_core.cpp",
                (
                    "candidate_deadline_ = now + candidate_freshness_;\n"
                    "  return applied_result();\n"
                    "}\n\n"
                    "Command MotionGateCore::tick"
                ),
                (
                    "candidate_deadline_ = now + candidate_freshness_;\n"
                    "  authority_deadline_ = now + authority_lease_;\n"
                    "  return applied_result();\n"
                    "}\n\n"
                    "Command MotionGateCore::tick"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must never renew", completed.stderr)

    def test_open_must_recreate_reader_before_binding_and_arming(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_node.cpp",
                "candidate_subscription_.reset();",
                "// stale PREPARE queue remains",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("OPEN queue barrier", completed.stderr)

    def test_open_must_bind_one_writer_in_gate_graph_context(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_node.cpp",
                "discover_unique_writer_gid_on_topic",
                "trust_cross_process_writer_gid",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "discover_unique_writer_gid_on_topic",
            completed.stderr,
        )

    def test_core_open_validates_before_invoking_graph_provider(self) -> None:
        def mutation(root: Path) -> None:
            path = (
                root
                / "src"
                / "voice_nav_mission"
                / "src"
                / "motion_gate_core.cpp"
            )
            source = path.read_text(encoding="utf-8")
            source = source.replace(
                "  const auto binding = binding_provider();",
                "  const auto binding = OpenBinding{};",
                1,
            )
            source = source.replace(
                "  ControlResult rejection;",
                (
                    "  const auto premature_binding = binding_provider();\n"
                    "  (void)premature_binding;\n"
                    "  ControlResult rejection;"
                ),
                1,
            )
            path.write_text(source, encoding="utf-8")

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "pure validation before graph provider",
            completed.stderr,
        )

    def test_core_open_faults_and_remembers_contradictory_ready_binding(
        self,
    ) -> None:
        contradictory_binding_block = (
            "  if (binding.reason != Reason::None) {\n"
            "    force_fault(\n"
            "      Reason::InternalFailure,\n"
            "      \"writer binding provider returned ready with a "
            "non-NONE reason\");\n"
            "    auto fault = result_from_snapshot(\n"
            "      ResultCode::Faulted, reason_, detail_);\n"
            "    remember(request, fault);\n"
            "    return fault;\n"
            "  }"
        )
        mutations = (
            (
                contradictory_binding_block,
                "  // contradictory ready binding is ignored",
            ),
            (
                "if (binding.reason != Reason::None)",
                "if (binding.reason == Reason::None)",
            ),
        )
        for old, new in mutations:
            with self.subTest(mutation=new):
                def mutation(
                    root: Path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(
                        root,
                        "src/voice_nav_mission/src/motion_gate_core.cpp",
                        old_value,
                        new_value,
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    "binding.reason != Reason::None",
                    completed.stderr,
                )

    def test_open_rejection_path_cannot_touch_graph_before_core(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_node.cpp",
                (
                    "OpenBinding expected_binding;\n"
                    "    const auto result = core_.open("
                ),
                (
                    "OpenBinding expected_binding;\n"
                    "    const auto unsafe_snapshot =\n"
                    "      discover_unique_writer_gid_on_topic("
                    "candidate_topic_);\n"
                    "    (void)unsafe_snapshot;\n"
                    "    const auto result = core_.open("
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("before touching the DDS graph", completed.stderr)

    def test_accepting_reader_requires_applied_and_third_snapshot(self) -> None:
        mutations = (
            (
                "if (result.code != ResultCode::Applied)",
                "if (false)",
                "result.code != ResultCode::Applied",
            ),
            (
                "candidate_topic_, request.lease_id, false",
                "candidate_topic_, request.lease_id, true",
                "accepting reader only after APPLIED",
            ),
            (
                (
                    "const auto third =\n"
                    "      discover_unique_writer_gid_on_topic("
                    "candidate_topic_);"
                ),
                "const auto third = expected_binding;",
                "exactly three",
            ),
        )
        for old, new, diagnostic in mutations:
            with self.subTest(mutation=diagnostic):
                def mutation(
                    root: Path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(
                        root,
                        (
                            "src/voice_nav_mission/src/"
                            "motion_gate_node.cpp"
                        ),
                        old_value,
                        new_value,
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(diagnostic, completed.stderr)

    def test_final_publication_must_be_serialized(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_node.cpp",
                "std::scoped_lock lock(publication_mutex_);",
                "final_command_publisher_->publish(command);",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("serialized publication", completed.stderr)

    def test_inhibit_must_publish_zero_before_acknowledgement(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_node.cpp",
                (
                    "publish_serialized(command);\n"
                    "    publish_state();\n"
                    "    fill_response("
                    "result, response, command.is_zero());"
                ),
                (
                    "fill_response("
                    "result, response, command.is_zero());\n"
                    "    publish_serialized(command);\n"
                    "    publish_state();"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("zero-before-response", completed.stderr)

    def test_product_launch_must_have_one_gate_and_no_mux(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_bringup/launch/product_sim.launch.py"
                ),
                (
                    "return LaunchDescription("
                    "[locked_rmw, simulation, motion_gate])"
                ),
                (
                    "twist_mux = Node(package='twist_mux', "
                    "executable='twist_mux')\n"
                    "    return LaunchDescription([locked_rmw, simulation, "
                    "motion_gate, twist_mux])"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("bypass", completed.stderr)

    def test_product_launch_must_not_hide_gate_death(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_bringup/launch/product_sim.launch.py",
                "parameters=[gate_config],",
                (
                    "parameters=[gate_config],\n"
                    "        on_exit=Shutdown(reason='MotionGate exited.'),"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("consumer deadman", completed.stderr)

    def test_product_launch_returns_simulation_gate_and_rmw_lock(self) -> None:
        mutations = (
            (
                "[locked_rmw, simulation, motion_gate]",
                "[locked_rmw, motion_gate]",
                "simulation action",
            ),
            (
                "[locked_rmw, simulation, motion_gate]",
                "[locked_rmw, simulation]",
                "motion_gate action",
            ),
            (
                "[locked_rmw, simulation, motion_gate]",
                "[simulation, motion_gate]",
                "locked RMW action",
            ),
            (
                "value='rmw_fastrtps_cpp'",
                "value='rmw_cyclonedds_cpp'",
                "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
            ),
            (
                "[locked_rmw, simulation, motion_gate]",
                "[simulation, locked_rmw, motion_gate]",
                "execute before",
            ),
        )
        for old, new, diagnostic in mutations:
            with self.subTest(mutation=diagnostic):
                def mutation(
                    root: Path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(
                        root,
                        (
                            "src/voice_nav_bringup/launch/"
                            "product_sim.launch.py"
                        ),
                        old_value,
                        new_value,
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(diagnostic, completed.stderr)

    def test_product_gate_uses_only_trusted_config_without_remaps(self) -> None:
        mutations = (
            (
                "parameters=[gate_config],",
                "parameters=[{'use_sim_time': True}],",
                "exactly [gate_config]",
            ),
            (
                "'motion_gate.yaml',",
                (
                    "('unsafe.yaml' if True else "
                    "'motion_gate.yaml'),"
                ),
                "installed trusted",
            ),
            (
                "parameters=[gate_config],",
                (
                    "parameters=[gate_config],\n"
                    "        remappings=[('/unsafe', '/bypass')],"
                ),
                "must not accept endpoint remappings",
            ),
        )
        for old, new, diagnostic in mutations:
            with self.subTest(mutation=diagnostic):
                def mutation(
                    root: Path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(
                        root,
                        (
                            "src/voice_nav_bringup/launch/"
                            "product_sim.launch.py"
                        ),
                        old_value,
                        new_value,
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(diagnostic, completed.stderr)

    def test_returned_simulation_uses_the_installed_launch(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_bringup/launch/"
                    "product_sim.launch.py"
                ),
                "'simulation.launch.py',",
                (
                    "('unsafe.launch.py' if True else "
                    "'simulation.launch.py'),"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "installed voice_nav_sim/launch/simulation.launch.py",
            completed.stderr,
        )

    def test_no_second_production_source_may_name_final_endpoint(self) -> None:
        def mutation(root: Path) -> None:
            rogue = root / "src" / "rogue_control" / "src" / "rogue.cpp"
            rogue.parent.mkdir(parents=True)
            rogue.write_text(
                'const char * topic = "/diff_drive_controller/cmd_vel";\n',
                encoding="utf-8",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("only motion_gate_node may name", completed.stderr)


if __name__ == "__main__":
    unittest.main()
