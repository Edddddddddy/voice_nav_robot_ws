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

#include <rmw/qos_profiles.h>
#include <rmw/rmw.h>
#include <rmw/types.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "voice_nav_mission/motion_gate_core.hpp"
#include "voice_nav_mission/msg/internal_motion_gate_state.hpp"
#include "voice_nav_mission/srv/internal_motion_gate_control.hpp"
#include "writer_observation.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;
using ControlService =
  voice_nav_mission::srv::InternalMotionGateControl;
using StateMessage =
  voice_nav_mission::msg::InternalMotionGateState;
using TwistStamped = geometry_msgs::msg::TwistStamped;

static_assert(RMW_GID_STORAGE_SIZE == 16, "MotionGate requires a 16-byte RMW GID");
static_assert(kWriterGidSize == RMW_GID_STORAGE_SIZE, "Core and RMW GID sizes differ");

constexpr char kControlService[] = "/motion_gate/internal/control";
constexpr char kStateTopic[] = "/motion_gate/internal/state";
constexpr char kCandidateTopicPrefix[] =
  "/voice_nav_internal/motion_gate/candidate/lease_";
constexpr char kFinalCommandTopic[] = "/diff_drive_controller/cmd_vel";
constexpr char kFinalControllerFqn[] = "/diff_drive_controller";
constexpr char kCandidateType[] = "geometry_msgs/msg/TwistStamped";
constexpr char kSupportedRmwImplementation[] = "rmw_fastrtps_cpp";
constexpr std::size_t kMaximumDetailLength = 160U;

WriterGid message_writer_gid(const rclcpp::MessageInfo & message_info)
{
  WriterGid publisher_gid{};
  const auto & raw_gid =
    message_info.get_rmw_message_info().publisher_gid;
  std::copy_n(
    raw_gid.data,
    RMW_GID_STORAGE_SIZE,
    publisher_gid.begin());
  return publisher_gid;
}

bool gid_is_zero(const WriterGid & gid)
{
  return std::all_of(
    gid.cbegin(),
    gid.cend(),
    [](std::uint8_t value) {return value == 0U;});
}

std::string bounded_detail(std::string detail)
{
  if (detail.size() > kMaximumDetailLength) {
    detail.resize(kMaximumDetailLength);
  }
  return detail;
}

std::string endpoint_fqn(
  const rclcpp::TopicEndpointInfo & endpoint)
{
  std::string node_namespace = endpoint.node_namespace();
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + endpoint.node_name();
  }
  if (node_namespace.front() != '/') {
    node_namespace.insert(node_namespace.begin(), '/');
  }
  if (node_namespace.back() == '/') {
    node_namespace.pop_back();
  }
  return node_namespace + "/" + endpoint.node_name();
}

std::string make_gate_instance_id()
{
  std::array<std::uint8_t, 16> bytes{};
  std::random_device random;
  for (auto & byte : bytes) {
    byte = static_cast<std::uint8_t>(random());
  }

  // Mix in steady time so a deterministic random_device implementation does
  // not repeat an instance ID across rapid process restarts.
  auto ticks = static_cast<std::uint64_t>(
    std::chrono::steady_clock::now().time_since_epoch().count());
  for (std::size_t index = 0; index < sizeof(ticks); ++index) {
    bytes[index] ^= static_cast<std::uint8_t>(ticks & 0xffU);
    ticks >>= 8U;
  }

  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : bytes) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

Command make_zero_command() noexcept
{
  return {};
}

}  // namespace

class MotionGateNode final : public rclcpp::Node
{
public:
  explicit MotionGateNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("motion_gate_node", options),
    callback_group_(
      create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive)),
    node_config_(load_node_config()),
    core_(node_config_.core, make_gate_instance_id()),
    writer_observation_session_({
      kCandidateType,
      node_config_.expected_candidate_writer_fqn})
  {
    const auto * rmw_identifier = rmw_get_implementation_identifier();
    if (
      rmw_identifier == nullptr ||
      std::string{rmw_identifier} != kSupportedRmwImplementation)
    {
      throw std::runtime_error(
              "motion_gate_node supports only rmw_fastrtps_cpp");
    }
    if (get_fully_qualified_name() != std::string{"/motion_gate_node"}) {
      throw std::runtime_error(
              "motion_gate_node must run at the exact FQN /motion_gate_node");
    }
    if (!get_parameter("use_sim_time").as_bool()) {
      throw std::runtime_error(
              "motion_gate_node requires use_sim_time=true");
    }
    use_sim_time_guard_ =
      add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "use_sim_time") {
            result.successful = false;
            result.reason =
            "MotionGate use_sim_time is immutable after startup";
            break;
          }
        }
        return result;
      });

    rclcpp::PublisherOptions publisher_options;
    publisher_options.callback_group = callback_group_;
    publisher_options.use_intra_process_comm =
      rclcpp::IntraProcessSetting::Disable;
    final_command_publisher_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(
      kFinalCommandTopic,
      rclcpp::SystemDefaultsQoS(),
      publisher_options);

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    state_qos.reliable().transient_local();
    state_publisher_ =
      create_publisher<StateMessage>(
      kStateTopic,
      state_qos,
      publisher_options);

    control_service_ =
      create_service<ControlService>(
      kControlService,
      [this](
        const std::shared_ptr<ControlService::Request> request,
        std::shared_ptr<ControlService::Response> response)
      {
        on_control(*request, response);
      },
      rclcpp::ServicesQoS(),
      callback_group_);

    output_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() {on_output_timer();},
      callback_group_);

    const bool zero_published =
      publish_serialized(make_zero_command());
    if (zero_published) {
      publish_state_or_stop();
    }

    RCLCPP_INFO(
      get_logger(),
      "MotionGate %s started inhibited; steady authority=%ld ms, "
      "candidate freshness=%ld ms",
      core_.snapshot().gate_instance_id.c_str(),
      node_config_.core.authority_lease.count(),
      node_config_.core.candidate_freshness.count());
  }

private:
  struct NodeConfig
  {
    MotionGateConfig core;
    std::chrono::milliseconds writer_graph_timeout{1000};
    std::string expected_candidate_writer_fqn{"/collision_monitor"};
  };

  template<typename ValueT>
  ValueT declare_read_only_parameter(
    const std::string & name,
    const ValueT & default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
      "Trusted MotionGate safety policy; immutable after startup";
    descriptor.read_only = true;
    return declare_parameter<ValueT>(
      name,
      default_value,
      descriptor);
  }

  NodeConfig load_node_config()
  {
    NodeConfig config;
    const auto output_frequency_hz =
      declare_read_only_parameter<double>(
      "output_frequency_hz", 50.0);
    const auto authority_lease_ms =
      declare_read_only_parameter<std::int64_t>(
      "authority_lease_ms", 250);
    const auto candidate_freshness_ms =
      declare_read_only_parameter<std::int64_t>(
      "candidate_freshness_ms", 150);
    const auto prepare_timeout_ms =
      declare_read_only_parameter<std::int64_t>(
      "prepare_timeout_ms", 6000);
    const auto writer_graph_timeout_ms =
      declare_read_only_parameter<std::int64_t>(
      "writer_graph_timeout_ms", 1000);
    const auto candidate_qos_depth =
      declare_read_only_parameter<std::int64_t>(
      "candidate_qos_depth", 1);
    config.expected_candidate_writer_fqn =
      declare_read_only_parameter<std::string>(
      "expected_candidate_writer_fqn", "/collision_monitor");
    const auto request_cache_size =
      declare_read_only_parameter<std::int64_t>(
      "request_cache_size", 64);
    config.core.linear_x_min =
      declare_read_only_parameter<double>(
      "linear_x_min", -0.20);
    config.core.linear_x_max =
      declare_read_only_parameter<double>(
      "linear_x_max", 0.40);
    config.core.angular_z_min =
      declare_read_only_parameter<double>(
      "angular_z_min", -1.20);
    config.core.angular_z_max =
      declare_read_only_parameter<double>(
      "angular_z_max", 1.20);

    if (
      !std::isfinite(output_frequency_hz) ||
      std::abs(output_frequency_hz - 50.0) >
      std::numeric_limits<double>::epsilon())
    {
      throw std::invalid_argument(
              "output_frequency_hz must be exactly 50.0");
    }
    if (candidate_qos_depth != 1) {
      throw std::invalid_argument(
              "candidate_qos_depth must be exactly 1");
    }
    if (
      authority_lease_ms <= 0 ||
      candidate_freshness_ms <= 0 ||
      prepare_timeout_ms <= 0 ||
      writer_graph_timeout_ms <= 0 ||
      request_cache_size <= 0 ||
      prepare_timeout_ms > 10000 ||
      writer_graph_timeout_ms > 10000 ||
      request_cache_size > 1024)
    {
      throw std::invalid_argument(
              "MotionGate duration and cache parameters must be positive");
    }
    if (
      candidate_freshness_ms >= authority_lease_ms ||
      authority_lease_ms >= 350)
    {
      throw std::invalid_argument(
              "MotionGate requires freshness < authority < controller timeout");
    }
    if (
      config.expected_candidate_writer_fqn.empty() ||
      config.expected_candidate_writer_fqn.front() != '/')
    {
      throw std::invalid_argument(
              "expected_candidate_writer_fqn must be absolute");
    }
    if (
      !std::isfinite(config.core.linear_x_min) ||
      !std::isfinite(config.core.linear_x_max) ||
      !std::isfinite(config.core.angular_z_min) ||
      !std::isfinite(config.core.angular_z_max) ||
      config.core.linear_x_min < -0.20 ||
      config.core.linear_x_max > 0.40 ||
      config.core.angular_z_min < -1.20 ||
      config.core.angular_z_max > 1.20)
    {
      throw std::invalid_argument(
              "MotionGate limits must be finite and no wider than controller limits");
    }

    config.core.authority_lease =
      std::chrono::milliseconds(authority_lease_ms);
    config.core.candidate_freshness =
      std::chrono::milliseconds(candidate_freshness_ms);
    config.core.prepare_timeout =
      std::chrono::milliseconds(prepare_timeout_ms);
    config.writer_graph_timeout =
      std::chrono::milliseconds(writer_graph_timeout_ms);
    config.core.request_cache_size =
      static_cast<std::size_t>(request_cache_size);
    return config;
  }

  rclcpp::QoS candidate_qos() const
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort().durability_volatile();
    return qos;
  }

  rclcpp::Subscription<TwistStamped>::SharedPtr
  create_candidate_subscription(
    const std::string & topic,
    const std::string & lease_id,
    bool discard)
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    options.use_intra_process_comm =
      rclcpp::IntraProcessSetting::Disable;
    return create_subscription<TwistStamped>(
      topic,
      candidate_qos(),
      [this, lease_id, discard](
        const TwistStamped::ConstSharedPtr message,
        const rclcpp::MessageInfo & message_info)
      {
        if (!discard) {
          on_candidate(*message, message_info, lease_id);
        }
      },
      options);
  }

  OpenBinding discover_unique_writer_gid_on_topic(
    const std::string & topic)
  {
    const auto endpoints = get_publishers_info_by_topic(topic);
    std::vector<WriterEndpointObservation> observations;
    observations.reserve(endpoints.size());
    for (const auto & endpoint : endpoints) {
      WriterGid writer_gid{};
      const auto & endpoint_gid = endpoint.endpoint_gid();
      std::copy(
        endpoint_gid.cbegin(),
        endpoint_gid.cend(),
        writer_gid.begin());
      observations.push_back(WriterEndpointObservation{
          endpoint.topic_type(),
          endpoint.node_name(),
          endpoint.node_namespace(),
          static_cast<rmw_endpoint_type_t>(endpoint.endpoint_type()),
          endpoint.qos_profile().get_rmw_qos_profile(),
          writer_gid});
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - writer_observation_started_at_);
    if (elapsed < 0ms) {
      elapsed = 0ms;
    }
    return writer_observation_session_.observe(observations, elapsed);
  }

  OpenBinding wait_for_unique_writer_gid_on_topic(
    const std::string & topic)
  {
    const auto deadline =
      writer_observation_started_at_ + node_config_.writer_graph_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto observation = discover_unique_writer_gid_on_topic(topic);
      if (observation.ready || observation.reason == Reason::WriterMismatch) {
        return observation;
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
      if (remaining <= 0ms) {
        break;
      }
      std::this_thread::sleep_for(std::min(5ms, remaining));
    }
    return discover_unique_writer_gid_on_topic(topic);
  }

  std::optional<std::string> final_controller_health_error() const
  {
    const auto endpoints =
      get_subscriptions_info_by_topic(kFinalCommandTopic);
    const auto publisher_qos =
      final_command_publisher_->get_actual_qos().get_rmw_qos_profile();
    std::string incompatible_reason;

    for (const auto & endpoint : endpoints) {
      if (
        endpoint.topic_type() != kCandidateType ||
        endpoint_fqn(endpoint) != kFinalControllerFqn)
      {
        continue;
      }

      rmw_qos_compatibility_type_t compatibility =
        RMW_QOS_COMPATIBILITY_ERROR;
      std::array<char, 256> reason{};
      const auto return_code =
        rmw_qos_profile_check_compatible(
        publisher_qos,
        endpoint.qos_profile().get_rmw_qos_profile(),
        &compatibility,
        reason.data(),
        reason.size());
      if (
        return_code == RMW_RET_OK &&
        compatibility != RMW_QOS_COMPATIBILITY_ERROR)
      {
        return std::nullopt;
      }
      incompatible_reason =
        return_code == RMW_RET_OK ?
        std::string{reason.data()} :
      "RMW QoS compatibility check failed";
    }

    if (!incompatible_reason.empty()) {
      return "final controller QoS is incompatible: " +
             incompatible_reason;
    }
    return "final controller command endpoint is unavailable";
  }

  ControlResult open_candidate_reader(
    const ControlRequest & request,
    MotionGateCore::SteadyTimePoint now)
  {
    const auto before = core_.snapshot();
    OpenBinding expected_binding;

    auto result =
      core_.open(
      request,
      now,
      [this, &request, &expected_binding]() {
        if (const auto error = final_controller_health_error()) {
          return OpenBinding{
          false,
          Reason::WriterUnavailable,
          {},
          *error};
        }

        const auto first =
        wait_for_unique_writer_gid_on_topic(
          core_.snapshot().candidate_topic);
        if (!first.ready) {
          return first;
        }

        // Reader B is still discard-only. Destroying reader A and then B
        // flushes every sample queued before Core enters ARMED.
        candidate_subscription_.reset();
        candidate_subscription_ =
        create_candidate_subscription(
          core_.snapshot().candidate_topic,
          request.lease_id,
          true);

        const auto second =
        wait_for_unique_writer_gid_on_topic(
          core_.snapshot().candidate_topic);
        if (!second.ready) {
          return second;
        }
        if (second.writer_gid != first.writer_gid) {
          return OpenBinding{
          false,
          Reason::WriterMismatch,
          {},
          "candidate writer changed across the discard-reader barrier"};
        }
        expected_binding = first;
        return first;
      });

    auto after = core_.snapshot();
    reconcile_adapter_transition(before, after);
    if (result.code != ResultCode::Applied) {
      if (after.state == State::Faulted) {
        return fault_from_snapshot();
      }
      return result;
    }

    const auto armed = after;
    try {
      // Reader C is the first accepting reader, and it is created only after
      // Core has atomically entered ARMED with zero selected.
      candidate_subscription_.reset();
      candidate_subscription_ =
        create_candidate_subscription(
        armed.candidate_topic,
        armed.lease_id,
        false);

      const auto third =
        wait_for_unique_writer_gid_on_topic(armed.candidate_topic);
      const auto controller_error = final_controller_health_error();
      if (
        !expected_binding.ready ||
        !third.ready ||
        third.writer_gid != expected_binding.writer_gid ||
        controller_error.has_value())
      {
        core_.force_fault(
          controller_error.has_value() ?
          Reason::WriterUnavailable : Reason::WriterMismatch,
          controller_error.has_value() ?
          *controller_error :
          "candidate writer changed before the accepting reader was ready");
      }
    } catch (const std::exception & error) {
      core_.force_fault(
        Reason::InternalFailure,
        std::string{"accepting candidate reader failed: "} + error.what());
    } catch (...) {
      core_.force_fault(
        Reason::InternalFailure,
        "accepting candidate reader failed with an unknown exception");
    }

    after = core_.snapshot();
    reconcile_adapter_transition(armed, after);
    if (after.state == State::Faulted) {
      candidate_subscription_.reset();
      return fault_from_snapshot();
    }
    return result;
  }

  void on_candidate(
    const TwistStamped & message,
    const rclcpp::MessageInfo & message_info,
    const std::string & lease_id)
  {
    const auto publisher_gid = message_writer_gid(message_info);
    Candidate candidate;
    candidate.lease_id = lease_id;
    candidate.writer_gid = publisher_gid;
    candidate.from_intra_process = gid_is_zero(publisher_gid);
    candidate.linear_x = message.twist.linear.x;
    candidate.linear_y = message.twist.linear.y;
    candidate.linear_z = message.twist.linear.z;
    candidate.angular_x = message.twist.angular.x;
    candidate.angular_y = message.twist.angular.y;
    candidate.angular_z = message.twist.angular.z;

    const auto now = std::chrono::steady_clock::now();
    const auto before = core_.snapshot();
    (void)core_.accept_candidate(candidate, now);
    auto after = core_.snapshot();
    reconcile_adapter_transition(before, after);

    const auto before_tick = after;
    const auto command = core_.tick(now);
    after = core_.snapshot();
    reconcile_adapter_transition(before_tick, after);
    const auto before_publish = after;
    if (!publish_serialized(command)) {
      (void)publish_serialized(make_zero_command());
    }
    after = core_.snapshot();
    reconcile_adapter_transition(before_publish, after);
    publish_state_or_stop();
  }

  void on_output_timer()
  {
    const auto before = core_.snapshot();
    const auto command =
      core_.tick(std::chrono::steady_clock::now());
    auto after = core_.snapshot();
    reconcile_adapter_transition(before, after);
    const auto before_publish = after;
    if (!publish_serialized(command)) {
      (void)publish_serialized(make_zero_command());
    }
    after = core_.snapshot();
    reconcile_adapter_transition(before_publish, after);
    publish_state_or_stop();
  }

  bool publish_serialized(const Command & selected)
  {
    std::scoped_lock lock(publication_mutex_);
    auto command_to_publish = selected;
    bool use_sim_time_locked_true = false;
    try {
      use_sim_time_locked_true =
        get_parameter("use_sim_time").as_bool() &&
        get_clock()->ros_time_is_active();
    } catch (const std::exception &) {
      use_sim_time_locked_true = false;
    }
    if (!use_sim_time_locked_true) {
      core_.force_fault(
        Reason::ConfigurationInvalid,
        "use_sim_time runtime invariant was violated");
      command_to_publish = make_zero_command();
    }

    const bool sequence_exhausted =
      output_publish_seq_ == std::numeric_limits<std::uint64_t>::max();
    if (sequence_exhausted) {
      core_.force_fault(
        Reason::SequenceExhausted,
        "output publication sequence exhausted");
      command_to_publish = make_zero_command();
    }

    TwistStamped command;
    if (use_sim_time_locked_true) {
      command.header.stamp = get_clock()->now();
    } else {
      // Never emit a system-time stamp on the simulation command endpoint.
      // A zero ROS stamp plus a zero command remains fail-closed even if a
      // future rclcpp regression bypasses the immutable-parameter callback.
      command.header.stamp.sec = 0;
      command.header.stamp.nanosec = 0U;
    }
    command.twist.linear.x = command_to_publish.linear_x;
    command.twist.angular.z = command_to_publish.angular_z;

    try {
      final_command_publisher_->publish(command);
    } catch (const std::exception & error) {
      core_.force_fault(
        Reason::PublishFailed,
        std::string{"final command publication failed: "} + error.what());
      RCLCPP_ERROR(
        get_logger(),
        "MotionGate final publication failed: %s",
        error.what());
      return false;
    } catch (...) {
      core_.force_fault(
        Reason::PublishFailed,
        "final command publication failed with an unknown exception");
      RCLCPP_ERROR(
        get_logger(),
        "MotionGate final publication failed with an unknown exception");
      return false;
    }

    last_publication_was_zero_ = command_to_publish.is_zero();
    if (sequence_exhausted) {
      zero_publish_seq_ = output_publish_seq_;
      return true;
    }
    ++output_publish_seq_;
    if (command_to_publish.is_zero()) {
      zero_publish_seq_ = output_publish_seq_;
    }
    return true;
  }

  bool publish_state()
  {
    const auto snapshot = core_.snapshot();
    StateMessage message;
    message.gate_instance_id = snapshot.gate_instance_id;
    message.state_seq = snapshot.state_seq;
    message.control_seq = snapshot.control_seq;
    message.state = static_cast<std::uint8_t>(snapshot.state);
    message.lease_id = snapshot.lease_id;
    message.candidate_topic = snapshot.candidate_topic;
    std::copy(
      snapshot.bound_writer_gid.cbegin(),
      snapshot.bound_writer_gid.cend(),
      message.bound_writer_gid.begin());
    message.motion_inhibited = snapshot.motion_inhibited;
    message.authority_live = snapshot.authority_live;
    message.candidate_fresh = snapshot.candidate_fresh;
    message.writer_bound = snapshot.writer_bound;
    message.zero_selected = snapshot.zero_selected;
    message.output_publish_seq = output_publish_seq_;
    message.zero_publish_seq = zero_publish_seq_;
    message.reason = static_cast<std::uint16_t>(snapshot.reason);
    message.detail = bounded_detail(snapshot.detail);
    try {
      state_publisher_->publish(message);
    } catch (const std::exception & error) {
      core_.force_fault(
        Reason::PublishFailed,
        std::string{"state publication failed: "} + error.what());
      RCLCPP_ERROR(
        get_logger(),
        "MotionGate state publication failed: %s",
        error.what());
      return false;
    } catch (...) {
      core_.force_fault(
        Reason::PublishFailed,
        "state publication failed with an unknown exception");
      RCLCPP_ERROR(
        get_logger(),
        "MotionGate state publication failed with an unknown exception");
      return false;
    }
    return true;
  }

  void publish_state_or_stop()
  {
    const auto before = core_.snapshot();
    if (publish_state()) {
      return;
    }
    const auto after = core_.snapshot();
    reconcile_adapter_transition(before, after);
    (void)publish_serialized(make_zero_command());
  }

  void remember_retired_writer(const Snapshot & snapshot)
  {
    if (
      !snapshot.writer_bound ||
      snapshot.candidate_topic.empty() ||
      gid_is_zero(snapshot.bound_writer_gid))
    {
      return;
    }
    retired_candidate_topic_ = snapshot.candidate_topic;
    retired_writer_gid_ = snapshot.bound_writer_gid;
  }

  void reconcile_adapter_transition(
    const Snapshot & before,
    const Snapshot & after)
  {
    const bool armed_generation_ended =
      before.state == State::Armed &&
      (
      after.state != State::Armed ||
      after.lease_id != before.lease_id);
    if (armed_generation_ended) {
      remember_retired_writer(before);
      candidate_subscription_.reset();
      return;
    }

    const bool prepared_generation_ended =
      before.state == State::Prepared &&
      after.state != State::Prepared &&
      after.state != State::Armed;
    if (prepared_generation_ended) {
      candidate_subscription_.reset();
    }
  }

  bool retired_writer_is_present() const
  {
    if (
      retired_candidate_topic_.empty() ||
      gid_is_zero(retired_writer_gid_))
    {
      return false;
    }
    const auto endpoints =
      get_publishers_info_by_topic(retired_candidate_topic_);
    return std::any_of(
      endpoints.cbegin(),
      endpoints.cend(),
      [this](const rclcpp::TopicEndpointInfo & endpoint)
      {
        WriterGid gid{};
        const auto & endpoint_gid = endpoint.endpoint_gid();
        std::copy(endpoint_gid.cbegin(), endpoint_gid.cend(), gid.begin());
        return gid == retired_writer_gid_;
      });
  }

  bool wait_for_retired_writer_to_disappear()
  {
    if (!retired_writer_is_present()) {
      retired_candidate_topic_.clear();
      retired_writer_gid_.fill(0U);
      return true;
    }

    const auto deadline =
      std::chrono::steady_clock::now() +
      node_config_.writer_graph_timeout;
    auto next_zero = std::chrono::steady_clock::now();
    while (
      rclcpp::ok() &&
      std::chrono::steady_clock::now() < deadline)
    {
      if (!retired_writer_is_present()) {
        retired_candidate_topic_.clear();
        retired_writer_gid_.fill(0U);
        return true;
      }
      if (std::chrono::steady_clock::now() >= next_zero) {
        const bool zero_published =
          publish_serialized(make_zero_command());
        if (
          !zero_published ||
          core_.snapshot().state == State::Faulted)
        {
          throw std::runtime_error(
                  "failed to maintain zero while waiting for retired writer");
        }
        next_zero += 20ms;
      }
      std::this_thread::sleep_for(1ms);
    }
    if (!retired_writer_is_present()) {
      retired_candidate_topic_.clear();
      retired_writer_gid_.fill(0U);
      return true;
    }
    return false;
  }

  ControlResult result_from_current_snapshot(
    ResultCode code,
    Reason reason,
    std::string detail) const
  {
    const auto snapshot = core_.snapshot();
    ControlResult result;
    result.code = code;
    result.reason = reason;
    result.gate_instance_id = snapshot.gate_instance_id;
    result.control_seq = snapshot.control_seq;
    result.state = snapshot.state;
    result.lease_id = snapshot.lease_id;
    result.candidate_topic = snapshot.candidate_topic;
    result.bound_writer_gid = snapshot.bound_writer_gid;
    result.motion_inhibited = snapshot.motion_inhibited;
    result.authority_live = snapshot.authority_live;
    result.candidate_fresh = snapshot.candidate_fresh;
    result.writer_bound = snapshot.writer_bound;
    result.zero_selected = snapshot.zero_selected;
    result.detail = bounded_detail(std::move(detail));
    return result;
  }

  ControlResult fault_from_snapshot() const
  {
    const auto snapshot = core_.snapshot();
    return result_from_current_snapshot(
      ResultCode::Faulted,
      snapshot.reason,
      snapshot.detail);
  }

  ControlResult handle_prepare(
    const ControlRequest & request,
    MotionGateCore::SteadyTimePoint now)
  {
    const auto before = core_.snapshot();
    auto result =
      core_.prepare(
      request,
      now,
      [this]() {
        const bool admitted =
        wait_for_retired_writer_to_disappear();
        return PrepareAdmission{
        admitted,
        admitted ? Reason::None : Reason::WriterStillPresent,
        admitted ? "retired writer absent" :
        "retired candidate writer remained visible past the graph timeout"};
      });
    auto after = core_.snapshot();
    reconcile_adapter_transition(before, after);
    if (result.code == ResultCode::Applied) {
      const auto prepared = after;
      writer_observation_session_.reset();
      writer_observation_started_at_ = now;
      try {
        candidate_subscription_ =
          create_candidate_subscription(
          result.candidate_topic,
          result.lease_id,
          true);
      } catch (const std::exception & error) {
        core_.force_fault(
          Reason::InternalFailure,
          std::string{"discard candidate reader failed: "} + error.what());
      } catch (...) {
        core_.force_fault(
          Reason::InternalFailure,
          "discard candidate reader failed with an unknown exception");
      }
      after = core_.snapshot();
      reconcile_adapter_transition(prepared, after);
      if (after.state == State::Faulted) {
        return fault_from_snapshot();
      }
    }
    return result;
  }

  ControlResult handle_renew(
    const ControlRequest & request,
    MotionGateCore::SteadyTimePoint now)
  {
    const auto before = core_.snapshot();
    auto result = core_.renew(request, now);
    const auto after = core_.snapshot();
    reconcile_adapter_transition(before, after);
    return result;
  }

  ControlResult handle_inhibit(
    const ControlRequest & request,
    MotionGateCore::SteadyTimePoint now)
  {
    const auto before = core_.snapshot();
    auto result = core_.inhibit(request, now);
    const auto after = core_.snapshot();
    reconcile_adapter_transition(before, after);
    return result;
  }

  ControlRequest to_core_request(
    const ControlService::Request & request) const
  {
    ControlRequest converted;
    converted.operation =
      static_cast<Operation>(request.operation);
    converted.request_id = request.request_id;
    converted.gate_instance_id = request.gate_instance_id;
    converted.expected_control_seq =
      request.expected_control_seq;
    converted.lease_id = request.lease_id;
    return converted;
  }

  void fill_response(
    const ControlResult & result,
    ControlService::Response & response,
    bool zero_published) const
  {
    response.code = static_cast<std::uint16_t>(result.code);
    response.reason = static_cast<std::uint16_t>(result.reason);
    response.gate_instance_id = result.gate_instance_id;
    response.control_seq = result.control_seq;
    response.state = static_cast<std::uint8_t>(result.state);
    response.lease_id = result.lease_id;
    response.candidate_topic = result.candidate_topic;
    std::copy(
      result.bound_writer_gid.cbegin(),
      result.bound_writer_gid.cend(),
      response.bound_writer_gid.begin());
    response.motion_inhibited = result.motion_inhibited;
    response.authority_live = result.authority_live;
    response.candidate_fresh = result.candidate_fresh;
    response.writer_bound = result.writer_bound;
    response.zero_selected = result.zero_selected;
    response.zero_published = zero_published;
    response.output_publish_seq = output_publish_seq_;
    response.zero_publish_seq = zero_publish_seq_;
    response.detail = bounded_detail(result.detail);
  }

  void on_control(
    const ControlService::Request & request_message,
    const std::shared_ptr<ControlService::Response> & response)
  {
    const auto request = to_core_request(request_message);
    const auto now = std::chrono::steady_clock::now();
    ControlResult result;

    switch (request.operation) {
      case Operation::Prepare:
        result = handle_prepare(request, now);
        break;
      case Operation::Open:
        result = open_candidate_reader(request, now);
        break;
      case Operation::Renew:
        result = handle_renew(request, now);
        break;
      case Operation::Inhibit:
        result = handle_inhibit(request, now);
        break;
      default:
        result = handle_prepare(request, now);
        break;
    }

    auto before_tick = core_.snapshot();
    const auto command = core_.tick(now);
    auto after = core_.snapshot();
    reconcile_adapter_transition(before_tick, after);

    const auto before_publish = after;
    const bool command_published = publish_serialized(command);
    const bool emergency_zero_published =
      !command_published && publish_serialized(make_zero_command());
    after = core_.snapshot();
    reconcile_adapter_transition(before_publish, after);
    bool zero_published =
      (command_published || emergency_zero_published) &&
      last_publication_was_zero_;

    const auto before_state = after;
    const bool state_published = publish_state();
    after = core_.snapshot();
    reconcile_adapter_transition(before_state, after);
    if (!state_published) {
      const bool state_failure_zero_published =
        publish_serialized(make_zero_command());
      zero_published =
        zero_published ||
        (state_failure_zero_published && last_publication_was_zero_);
      after = core_.snapshot();
    }

    if (after.state == State::Faulted) {
      result = fault_from_snapshot();
    } else if (result.code == ResultCode::Duplicate) {
      result = result_from_current_snapshot(
        ResultCode::Duplicate,
        result.reason,
        result.detail);
    }
    fill_response(result, *response, zero_published);
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    use_sim_time_guard_;
  NodeConfig node_config_;
  MotionGateCore core_;
  WriterObservationSession writer_observation_session_;
  MotionGateCore::SteadyTimePoint writer_observation_started_at_{};
  rclcpp::Publisher<TwistStamped>::SharedPtr
    final_command_publisher_;
  rclcpp::Publisher<StateMessage>::SharedPtr state_publisher_;
  rclcpp::Service<ControlService>::SharedPtr control_service_;
  rclcpp::Subscription<TwistStamped>::SharedPtr
    candidate_subscription_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  std::mutex publication_mutex_;
  std::uint64_t output_publish_seq_{0U};
  std::uint64_t zero_publish_seq_{0U};
  bool last_publication_was_zero_{true};
  std::string retired_candidate_topic_;
  WriterGid retired_writer_gid_{};
};

}  // namespace voice_nav_mission

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node =
      std::make_shared<voice_nav_mission::MotionGateNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("motion_gate_node"),
      "MotionGate startup failed: %s",
      error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
