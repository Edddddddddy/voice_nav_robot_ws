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

#include <map>
#include <memory>
#include <string>

namespace voice_nav_sim
{

class JournaledGazeboSimSystemAdapter final
  : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  JournaledGazeboSimSystemAdapter();
  explicit JournaledGazeboSimSystemAdapter(
    std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> upstream);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

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
  pluginlib::ClassLoader<
    gz_ros2_control::GazeboSimSystemInterface> upstream_loader_;
  std::shared_ptr<
    gz_ros2_control::GazeboSimSystemInterface> upstream_;
};

}  // namespace voice_nav_sim

#endif  // VOICE_NAV_SIM__JOURNALED_GAZEBO_SIM_SYSTEM_ADAPTER_HPP_
