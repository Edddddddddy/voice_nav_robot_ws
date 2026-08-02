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

#ifndef VOICE_NAV_SIM__JOURNALED_GAZEBO_SIM_SYSTEM_ADAPTER_HPP_
#define VOICE_NAV_SIM__JOURNALED_GAZEBO_SIM_SYSTEM_ADAPTER_HPP_

#include <gz_ros2_control/gz_system_interface.hpp>
#include <pluginlib/class_loader.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_write_ledger_writer.hpp"

namespace voice_nav_sim
{

class HardwareWriteJournalAttachment
{
public:
  virtual ~HardwareWriteJournalAttachment() = default;

  [[nodiscard]] virtual std::shared_ptr<HardwareWriteJournal> attach(
    const std::string & journal_name,
    const std::string & journal_nonce) = 0;
};

class JournaledGazeboSimSystemAdapter final
  : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  JournaledGazeboSimSystemAdapter();
  explicit JournaledGazeboSimSystemAdapter(
    std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> upstream);
  JournaledGazeboSimSystemAdapter(
    std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> upstream,
    std::shared_ptr<HardwareWriteJournal> write_journal);
  JournaledGazeboSimSystemAdapter(
    std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> upstream,
    std::shared_ptr<HardwareWriteJournalAttachment> write_journal_attachment);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
  override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  bool initSim(
    rclcpp::Node::SharedPtr & model_node,
    std::map<std::string, gz::sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    gz::sim::EntityComponentManager & entity_component_manager,
    unsigned int update_rate) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  [[nodiscard]] HardwareWriteWheelObservation observe_wheel_commands()
  const noexcept;

  pluginlib::ClassLoader<
    gz_ros2_control::GazeboSimSystemInterface> upstream_loader_;
  std::shared_ptr<
    gz_ros2_control::GazeboSimSystemInterface> upstream_;
  std::shared_ptr<HardwareWriteJournalAttachment> write_journal_attachment_;
  std::shared_ptr<HardwareWriteJournal> write_journal_;
  std::string attached_journal_name_;
  std::string attached_journal_nonce_;
  gz::sim::EntityComponentManager * entity_component_manager_{nullptr};
  gz::sim::Entity left_wheel_entity_{gz::sim::kNullEntity};
  gz::sim::Entity right_wheel_entity_{gz::sim::kNullEntity};
};

}  // namespace voice_nav_sim

#endif  // VOICE_NAV_SIM__JOURNALED_GAZEBO_SIM_SYSTEM_ADAPTER_HPP_
