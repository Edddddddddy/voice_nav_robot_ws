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
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz_ros2_control/gz_system_interface.hpp>
#include <pluginlib/class_loader.hpp>

#include <cstring>
#include <memory>
#include <optional>

namespace
{

using GazeboSystemInterface = gz_ros2_control::GazeboSimSystemInterface;

std::uint64_t double_bits(double value)
{
  static_assert(sizeof(value) == sizeof(std::uint64_t));
  std::uint64_t bits{0U};
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

class RecordingHardwareWriteSink final
  : public voice_nav_sim::HardwareWriteSink
{
public:
  bool append(
    const voice_nav_sim::HardwareWriteRecord & record) noexcept override
  {
    last_record = record;
    return true;
  }

  std::optional<voice_nav_sim::HardwareWriteRecord> last_record;
};

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
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override
  {
    read_time = &time;
    read_period = &period;
    return read_result;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override
  {
    write_time = &time;
    write_period = &period;
    if (write_entity_component_manager != nullptr) {
      auto * left_command =
        write_entity_component_manager->Component<
        gz::sim::components::JointVelocityCmd>(write_left_entity);
      auto * right_command =
        write_entity_component_manager->Component<
        gz::sim::components::JointVelocityCmd>(write_right_entity);
      if (left_command == nullptr) {
        (void)write_entity_component_manager->CreateComponent(
          write_left_entity,
          gz::sim::components::JointVelocityCmd({delegated_left_command}));
      } else if (!left_command->Data().empty()) {
        left_command->Data()[0] = delegated_left_command;
      }
      if (right_command == nullptr) {
        (void)write_entity_component_manager->CreateComponent(
          write_right_entity,
          gz::sim::components::JointVelocityCmd({delegated_right_command}));
      } else if (!right_command->Data().empty()) {
        right_command->Data()[0] = delegated_right_command;
      }
    }
    return write_result;
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
  const rclcpp::Time * read_time{nullptr};
  const rclcpp::Duration * read_period{nullptr};
  const rclcpp::Time * write_time{nullptr};
  const rclcpp::Duration * write_period{nullptr};
  gz::sim::EntityComponentManager * write_entity_component_manager{nullptr};
  gz::sim::Entity write_left_entity{gz::sim::kNullEntity};
  gz::sim::Entity write_right_entity{gz::sim::kNullEntity};
  double delegated_left_command{0.0};
  double delegated_right_command{0.0};
  hardware_interface::return_type read_result{
    hardware_interface::return_type::ERROR};
  hardware_interface::return_type write_result{
    hardware_interface::return_type::OK};
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

TEST(JournaledGazeboSimSystemAdapter, ForwardsReadAndWriteCycles)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream);
  const rclcpp::Time read_time(11, 12, RCL_ROS_TIME);
  const rclcpp::Duration read_period(13, 14);
  const rclcpp::Time write_time(21, 22, RCL_ROS_TIME);
  const rclcpp::Duration write_period(23, 24);

  const auto read_result = adapter.read(read_time, read_period);
  const auto write_result = adapter.write(write_time, write_period);

  EXPECT_EQ(upstream->read_time, &read_time);
  EXPECT_EQ(upstream->read_period, &read_period);
  EXPECT_EQ(read_result, hardware_interface::return_type::ERROR);
  EXPECT_EQ(upstream->write_time, &write_time);
  EXPECT_EQ(upstream->write_period, &write_period);
  EXPECT_EQ(write_result, hardware_interface::return_type::OK);
}

TEST(
  JournaledGazeboSimSystemAdapter,
  ObservesActualWheelCommandsAfterDelegatedWrite)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  auto sink = std::make_shared<RecordingHardwareWriteSink>();
  constexpr std::uint64_t generation = 41U;
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(
    upstream, sink, generation);
  rclcpp::Node::SharedPtr model_node;
  gz::sim::EntityComponentManager entity_component_manager;
  const auto left_entity = entity_component_manager.CreateEntity();
  const auto right_entity = entity_component_manager.CreateEntity();
  std::map<std::string, gz::sim::Entity> joints{
    {"left_wheel_joint", left_entity},
    {"right_wheel_joint", right_entity}};
  const hardware_interface::HardwareInfo hardware_info{};
  ASSERT_TRUE(
    adapter.initSim(
      model_node,
      joints,
      hardware_info,
      entity_component_manager,
      50U));
  upstream->write_entity_component_manager = &entity_component_manager;
  upstream->write_left_entity = left_entity;
  upstream->write_right_entity = right_entity;
  upstream->delegated_left_command = -1.25;
  upstream->delegated_right_command = 2.5;
  upstream->write_result = hardware_interface::return_type::ERROR;
  const rclcpp::Time time(123, 456, RCL_ROS_TIME);
  const rclcpp::Duration period(0, 20'000'000);

  const auto result = adapter.write(time, period);

  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
  ASSERT_TRUE(sink->last_record.has_value());
  EXPECT_EQ(sink->last_record->generation, generation);
  EXPECT_EQ(sink->last_record->write_seq, 1U);
  EXPECT_EQ(sink->last_record->sim_stamp_ns, time.nanoseconds());
  EXPECT_EQ(
    sink->last_record->delegated_result,
    static_cast<std::uint8_t>(hardware_interface::return_type::ERROR));
  EXPECT_EQ(
    sink->last_record->left_command_bits,
    double_bits(upstream->delegated_left_command));
  EXPECT_EQ(
    sink->last_record->right_command_bits,
    double_bits(upstream->delegated_right_command));
}

}  // namespace
