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

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    on_configure_previous_state = &previous_state;
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
    override
  {
    ++export_state_interfaces_calls;
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.emplace_back("recording_joint", "state", &state_value);
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override
  {
    ++export_command_interfaces_calls;
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.emplace_back("recording_joint", "command", &command_value);
    return interfaces;
  }

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    on_activate_previous_state = &previous_state;
    return hardware_interface::CallbackReturn::ERROR;
  }

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    on_deactivate_previous_state = &previous_state;
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
  const rclcpp_lifecycle::State * on_configure_previous_state{nullptr};
  std::size_t export_state_interfaces_calls{0U};
  std::size_t export_command_interfaces_calls{0U};
  const rclcpp_lifecycle::State * on_activate_previous_state{nullptr};
  const rclcpp_lifecycle::State * on_deactivate_previous_state{nullptr};
  double state_value{1.0};
  double command_value{2.0};
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

TEST(JournaledGazeboSimSystemAdapter, ForwardsOnConfigureArgumentAndResult)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream);
  const rclcpp_lifecycle::State previous_state;

  const auto result = adapter.on_configure(previous_state);

  EXPECT_EQ(upstream->on_configure_previous_state, &previous_state);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST(JournaledGazeboSimSystemAdapter, ForwardsExportedInterfaceCollections)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream);

  auto state_interfaces = adapter.export_state_interfaces();
  auto command_interfaces = adapter.export_command_interfaces();

  ASSERT_EQ(upstream->export_state_interfaces_calls, 1U);
  ASSERT_EQ(state_interfaces.size(), 1U);
  EXPECT_EQ(state_interfaces.front().get_name(), "recording_joint/state");
  ASSERT_EQ(upstream->export_command_interfaces_calls, 1U);
  ASSERT_EQ(command_interfaces.size(), 1U);
  EXPECT_EQ(command_interfaces.front().get_name(), "recording_joint/command");
}

TEST(JournaledGazeboSimSystemAdapter, ForwardsActivationTransitions)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream);
  const rclcpp_lifecycle::State activation_previous_state;
  const rclcpp_lifecycle::State deactivation_previous_state;

  const auto activation_result =
    adapter.on_activate(activation_previous_state);
  const auto deactivation_result =
    adapter.on_deactivate(deactivation_previous_state);

  EXPECT_EQ(
    upstream->on_activate_previous_state, &activation_previous_state);
  EXPECT_EQ(activation_result, hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(
    upstream->on_deactivate_previous_state, &deactivation_previous_state);
  EXPECT_EQ(deactivation_result, hardware_interface::CallbackReturn::ERROR);
}

}  // namespace
