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

#include "journaled_gazebo_sim_system_adapter.hpp"

#include <gz_ros2_control/gz_system_interface.hpp>
#include <pluginlib/class_loader.hpp>

#include <memory>

namespace
{

using GazeboSystemInterface = gz_ros2_control::GazeboSimSystemInterface;

class RecordingGazeboSystem final : public GazeboSystemInterface
{
public:
  bool initSim(
    rclcpp::Node::SharedPtr &,
    std::map<std::string, gz::sim::Entity> &,
    const hardware_interface::HardwareInfo &,
    gz::sim::EntityComponentManager &,
    unsigned int) override
  {
    return false;
  }

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override
  {
    on_init_hardware_info = &hardware_info;
    return hardware_interface::CallbackReturn::ERROR;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &,
    const rclcpp::Duration &) override
  {
    return hardware_interface::return_type::ERROR;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override
  {
    return hardware_interface::return_type::ERROR;
  }

  const hardware_interface::HardwareInfo * on_init_hardware_info{nullptr};
};

TEST(
  JournaledGazeboSimSystemAdapter,
  LoadsExportedAdapterAndItsPinnedUpstream)
{
  pluginlib::ClassLoader<GazeboSystemInterface> loader(
    "gz_ros2_control",
    "gz_ros2_control::GazeboSimSystemInterface");

  std::shared_ptr<GazeboSystemInterface> adapter;
  ASSERT_NO_THROW(
    adapter = loader.createSharedInstance(
      "voice_nav_sim/JournaledGazeboSimSystemAdapter"));
  ASSERT_NE(adapter, nullptr);

  adapter.reset();
}

TEST(JournaledGazeboSimSystemAdapter, ForwardsOnInitArgumentAndResult)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream);
  const hardware_interface::HardwareInfo hardware_info{};

  const auto result = adapter.on_init(hardware_info);

  EXPECT_EQ(upstream->on_init_hardware_info, &hardware_info);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

}  // namespace
