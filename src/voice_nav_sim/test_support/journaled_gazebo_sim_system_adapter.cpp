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

#include "journaled_gazebo_sim_system_adapter.hpp"

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <pluginlib/class_list_macros.hpp>

#include <cstring>
#include <utility>

namespace voice_nav_sim
{

namespace
{

constexpr char kLeftWheelJoint[] = "left_wheel_joint";
constexpr char kRightWheelJoint[] = "right_wheel_joint";

std::uint64_t double_bits(double value) noexcept
{
  static_assert(sizeof(value) == sizeof(std::uint64_t));
  std::uint64_t bits{0U};
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

}  // namespace

JournaledGazeboSimSystemAdapter::JournaledGazeboSimSystemAdapter()
: upstream_loader_(
    "gz_ros2_control",
    "gz_ros2_control::GazeboSimSystemInterface"),
  upstream_(upstream_loader_.createSharedInstance(
      "gz_ros2_control/GazeboSimSystem"))
{
}

JournaledGazeboSimSystemAdapter::JournaledGazeboSimSystemAdapter(
  std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> upstream)
: JournaledGazeboSimSystemAdapter(
    std::move(upstream), std::shared_ptr<HardwareWriteJournal>{})
{
}

JournaledGazeboSimSystemAdapter::JournaledGazeboSimSystemAdapter(
  std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> upstream,
  std::shared_ptr<HardwareWriteJournal> write_journal)
: upstream_loader_(
    "gz_ros2_control",
    "gz_ros2_control::GazeboSimSystemInterface"),
  upstream_(std::move(upstream)),
  write_journal_(std::move(write_journal))
{
}

JournaledGazeboSimSystemAdapter::JournaledGazeboSimSystemAdapter(
  std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> upstream,
  std::shared_ptr<HardwareWriteJournalAttachment> write_journal_attachment)
: upstream_loader_(
    "gz_ros2_control",
    "gz_ros2_control::GazeboSimSystemInterface"),
  upstream_(std::move(upstream)),
  write_journal_attachment_(std::move(write_journal_attachment))
{
}

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
hardware_interface::CallbackReturn JournaledGazeboSimSystemAdapter::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  // gz_ros2_control 1.2.19 overrides this legacy overload; parity requires it.
  return upstream_->on_init(hardware_info);
}
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

hardware_interface::CallbackReturn
JournaledGazeboSimSystemAdapter::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  return upstream_->on_configure(previous_state);
}

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
std::vector<hardware_interface::StateInterface>
JournaledGazeboSimSystemAdapter::export_state_interfaces()
{
  // gz_ros2_control 1.2.19 exports through these legacy overrides.
  return upstream_->export_state_interfaces();
}

std::vector<hardware_interface::CommandInterface>
JournaledGazeboSimSystemAdapter::export_command_interfaces()
{
  return upstream_->export_command_interfaces();
}
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

hardware_interface::CallbackReturn
JournaledGazeboSimSystemAdapter::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  return upstream_->on_activate(previous_state);
}

hardware_interface::CallbackReturn
JournaledGazeboSimSystemAdapter::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  return upstream_->on_deactivate(previous_state);
}

hardware_interface::return_type
JournaledGazeboSimSystemAdapter::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return upstream_->prepare_command_mode_switch(
    start_interfaces, stop_interfaces);
}

hardware_interface::return_type
JournaledGazeboSimSystemAdapter::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return upstream_->perform_command_mode_switch(
    start_interfaces, stop_interfaces);
}

bool JournaledGazeboSimSystemAdapter::initSim(
  rclcpp::Node::SharedPtr & model_node,
  std::map<std::string, gz::sim::Entity> & joints,
  const hardware_interface::HardwareInfo & hardware_info,
  gz::sim::EntityComponentManager & entity_component_manager,
  unsigned int update_rate)
{
  entity_component_manager_ = nullptr;
  left_wheel_entity_ = gz::sim::kNullEntity;
  right_wheel_entity_ = gz::sim::kNullEntity;
  const bool initialized = upstream_->initSim(
    model_node,
    joints,
    hardware_info,
    entity_component_manager,
    update_rate);
  if (!initialized) {
    return false;
  }
  if (write_journal_attachment_ != nullptr) {
    const auto journal_name =
      hardware_info.hardware_parameters.find("journal_name");
    const auto journal_nonce =
      hardware_info.hardware_parameters.find("journal_nonce");
    if (
      journal_name == hardware_info.hardware_parameters.end() ||
      journal_nonce == hardware_info.hardware_parameters.end() ||
      journal_name->second.empty() || journal_nonce->second.empty())
    {
      return false;
    }
    if (write_journal_ == nullptr) {
      try {
        auto attached = write_journal_attachment_->attach(
          journal_name->second, journal_nonce->second);
        if (attached == nullptr) {
          return false;
        }
        write_journal_ = std::move(attached);
        attached_journal_name_ = journal_name->second;
        attached_journal_nonce_ = journal_nonce->second;
      } catch (...) {
        return false;
      }
    } else if (
      journal_name->second != attached_journal_name_ ||
      journal_nonce->second != attached_journal_nonce_)
    {
      return false;
    }
  }
  if (initialized && write_journal_ != nullptr) {
    const auto left_joint = joints.find(kLeftWheelJoint);
    const auto right_joint = joints.find(kRightWheelJoint);
    if (left_joint != joints.end() && right_joint != joints.end()) {
      entity_component_manager_ = &entity_component_manager;
      left_wheel_entity_ = left_joint->second;
      right_wheel_entity_ = right_joint->second;
    }
  }
  return true;
}

hardware_interface::return_type JournaledGazeboSimSystemAdapter::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  return upstream_->read(time, period);
}

hardware_interface::return_type JournaledGazeboSimSystemAdapter::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  HardwareWriteTicket ticket{};
  const bool journal_enabled = write_journal_ != nullptr;
  if (journal_enabled) {
    ticket = write_journal_->begin_write(time.nanoseconds());
  }
  auto delegated_result = hardware_interface::return_type::ERROR;
  try {
    delegated_result = upstream_->write(time, period);
  } catch (...) {
    if (journal_enabled) {
      write_journal_->finish_write(
        ticket,
        kHardwareWriteDelegatedException,
        observe_wheel_commands());
    }
    throw;
  }
  if (journal_enabled) {
    write_journal_->finish_write(
      ticket,
      static_cast<std::uint64_t>(delegated_result),
      observe_wheel_commands());
  }
  return delegated_result;
}

HardwareWriteWheelObservation
JournaledGazeboSimSystemAdapter::observe_wheel_commands() const noexcept
{
  try {
    if (
      entity_component_manager_ == nullptr ||
      left_wheel_entity_ == gz::sim::kNullEntity ||
      right_wheel_entity_ == gz::sim::kNullEntity ||
      !entity_component_manager_->HasEntity(left_wheel_entity_) ||
      !entity_component_manager_->HasEntity(right_wheel_entity_))
    {
      return HardwareWriteWheelObservation{
        HardwareWriteObservationStatus::kMissingEntity, 0U, 0U};
    }
    const auto * left_command =
      entity_component_manager_->Component<
      gz::sim::components::JointVelocityCmd>(left_wheel_entity_);
    const auto * right_command =
      entity_component_manager_->Component<
      gz::sim::components::JointVelocityCmd>(right_wheel_entity_);
    if (left_command == nullptr || right_command == nullptr) {
      return HardwareWriteWheelObservation{
        HardwareWriteObservationStatus::kMissingComponent, 0U, 0U};
    }
    if (left_command->Data().empty() || right_command->Data().empty()) {
      return HardwareWriteWheelObservation{
        HardwareWriteObservationStatus::kEmptyComponent, 0U, 0U};
    }

    return HardwareWriteWheelObservation{
      HardwareWriteObservationStatus::kValid,
      double_bits(left_command->Data()[0]),
      double_bits(right_command->Data()[0])};
  } catch (...) {
    return HardwareWriteWheelObservation{
      HardwareWriteObservationStatus::kInspectionFailure, 0U, 0U};
  }
}

}  // namespace voice_nav_sim

PLUGINLIB_EXPORT_CLASS(
  voice_nav_sim::JournaledGazeboSimSystemAdapter,
  gz_ros2_control::GazeboSimSystemInterface)
