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

#include <gz/sim/EntityComponentManager.hh>
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
    rclcpp::Node::SharedPtr & model_node,
    std::map<std::string, gz::sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    gz::sim::EntityComponentManager & entity_component_manager,
    unsigned int update_rate) override
  {
    init_model_node = &model_node;
    init_joints = &joints;
    init_hardware_info = &hardware_info;
    init_entity_component_manager = &entity_component_manager;
    init_update_rate = update_rate;
    return true;
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

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override
  {
    prepare_start_interfaces = &start_interfaces;
    prepare_stop_interfaces = &stop_interfaces;
    return hardware_interface::return_type::ERROR;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override
  {
    perform_start_interfaces = &start_interfaces;
    perform_stop_interfaces = &stop_interfaces;
    return hardware_interface::return_type::ERROR;
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

  rclcpp::Node::SharedPtr * init_model_node{nullptr};
  std::map<std::string, gz::sim::Entity> * init_joints{nullptr};
  const hardware_interface::HardwareInfo * init_hardware_info{nullptr};
  gz::sim::EntityComponentManager * init_entity_component_manager{nullptr};
  unsigned int init_update_rate{0U};
  const hardware_interface::HardwareInfo * on_init_hardware_info{nullptr};
  const rclcpp_lifecycle::State * on_configure_previous_state{nullptr};
  std::size_t export_state_interfaces_calls{0U};
  std::size_t export_command_interfaces_calls{0U};
  const rclcpp_lifecycle::State * on_activate_previous_state{nullptr};
  const rclcpp_lifecycle::State * on_deactivate_previous_state{nullptr};
  const std::vector<std::string> * prepare_start_interfaces{nullptr};
  const std::vector<std::string> * prepare_stop_interfaces{nullptr};
  const std::vector<std::string> * perform_start_interfaces{nullptr};
  const std::vector<std::string> * perform_stop_interfaces{nullptr};
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

TEST(JournaledGazeboSimSystemAdapter, ForwardsInitSimArgumentsAndResult)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream);
  rclcpp::Node::SharedPtr model_node;
  std::map<std::string, gz::sim::Entity> joints;
  const hardware_interface::HardwareInfo hardware_info{};
  gz::sim::EntityComponentManager entity_component_manager;
  constexpr unsigned int update_rate = 73U;

  const bool result = adapter.initSim(
    model_node,
    joints,
    hardware_info,
    entity_component_manager,
    update_rate);

  EXPECT_EQ(upstream->init_model_node, &model_node);
  EXPECT_EQ(upstream->init_joints, &joints);
  EXPECT_EQ(upstream->init_hardware_info, &hardware_info);
  EXPECT_EQ(
    upstream->init_entity_component_manager, &entity_component_manager);
  EXPECT_EQ(upstream->init_update_rate, update_rate);
  EXPECT_TRUE(result);
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

TEST(JournaledGazeboSimSystemAdapter, ForwardsCommandModeSwitches)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream);
  const std::vector<std::string> prepare_start{"prepare_start"};
  const std::vector<std::string> prepare_stop{"prepare_stop"};
  const std::vector<std::string> perform_start{"perform_start"};
  const std::vector<std::string> perform_stop{"perform_stop"};

  const auto prepare_result =
    adapter.prepare_command_mode_switch(prepare_start, prepare_stop);
  const auto perform_result =
    adapter.perform_command_mode_switch(perform_start, perform_stop);

  EXPECT_EQ(upstream->prepare_start_interfaces, &prepare_start);
  EXPECT_EQ(upstream->prepare_stop_interfaces, &prepare_stop);
  EXPECT_EQ(prepare_result, hardware_interface::return_type::ERROR);
  EXPECT_EQ(upstream->perform_start_interfaces, &perform_start);
  EXPECT_EQ(upstream->perform_stop_interfaces, &perform_stop);
  EXPECT_EQ(perform_result, hardware_interface::return_type::ERROR);
}

}  // namespace
