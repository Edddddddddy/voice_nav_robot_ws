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

#include <pluginlib/class_list_macros.hpp>

#include <utility>

namespace voice_nav_sim
{

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
: upstream_loader_(
    "gz_ros2_control",
    "gz_ros2_control::GazeboSimSystemInterface"),
  upstream_(std::move(upstream))
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

bool JournaledGazeboSimSystemAdapter::initSim(
  rclcpp::Node::SharedPtr & model_node,
  std::map<std::string, gz::sim::Entity> & joints,
  const hardware_interface::HardwareInfo & hardware_info,
  gz::sim::EntityComponentManager & entity_component_manager,
  unsigned int update_rate)
{
  return upstream_->initSim(
    model_node,
    joints,
    hardware_info,
    entity_component_manager,
    update_rate);
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
  return upstream_->write(time, period);
}

}  // namespace voice_nav_sim

PLUGINLIB_EXPORT_CLASS(
  voice_nav_sim::JournaledGazeboSimSystemAdapter,
  gz_ros2_control::GazeboSimSystemInterface)
