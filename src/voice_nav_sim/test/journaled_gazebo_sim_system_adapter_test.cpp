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
#include <stdexcept>

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

class RecordingHardwareWriteJournal final
  : public voice_nav_sim::HardwareWriteJournal
{
public:
  voice_nav_sim::HardwareWriteTicket begin_write(
    std::int64_t sim_stamp_ns) noexcept override
  {
    ++begin_calls;
    begun_sim_stamp_ns = sim_stamp_ns;
    ticket.sim_stamp_ns = sim_stamp_ns;
    return ticket;
  }

  void finish_write(
    voice_nav_sim::HardwareWriteTicket finished,
    std::uint64_t delegated_result_value,
    voice_nav_sim::HardwareWriteWheelObservation observation_value)
  noexcept override
  {
    ++finish_calls;
    finished_ticket = finished;
    delegated_result = delegated_result_value;
    observation = observation_value;
  }

  std::size_t begin_calls{0U};
  std::size_t finish_calls{0U};
  std::int64_t begun_sim_stamp_ns{0};
  voice_nav_sim::HardwareWriteTicket ticket{37U, 0, 1U, 2U, true};
  voice_nav_sim::HardwareWriteTicket finished_ticket{};
  std::uint64_t delegated_result{0U};
  voice_nav_sim::HardwareWriteWheelObservation observation{};
};

class RecordingHardwareWriteJournalAttachment final
  : public voice_nav_sim::HardwareWriteJournalAttachment
{
public:
  std::shared_ptr<voice_nav_sim::HardwareWriteJournal> attach(
    const std::string & journal_name,
    const std::string & journal_nonce) override
  {
    ++attach_calls;
    attached_name = journal_name;
    attached_nonce = journal_nonce;
    if (throw_on_attach) {
      throw std::invalid_argument("attachment rejected");
    }
    return journal;
  }

  std::size_t attach_calls{0U};
  std::string attached_name;
  std::string attached_nonce;
  bool throw_on_attach{false};
  std::shared_ptr<RecordingHardwareWriteJournal> journal{
    std::make_shared<RecordingHardwareWriteJournal>()};
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
    return init_result;
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
    if (write_journal != nullptr) {
      journal_began_before_upstream_write =
        write_journal->begin_calls == 1U &&
        write_journal->finish_calls == 0U;
    }
    if (throw_on_write) {
      throw std::runtime_error("delegated write failed");
    }
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
  bool init_result{true};
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
  RecordingHardwareWriteJournal * write_journal{nullptr};
  bool journal_began_before_upstream_write{false};
  bool throw_on_write{false};
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
  auto journal = std::make_shared<RecordingHardwareWriteJournal>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(
    upstream, journal);
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
  upstream->write_journal = journal.get();
  upstream->write_left_entity = left_entity;
  upstream->write_right_entity = right_entity;
  upstream->delegated_left_command = -1.25;
  upstream->delegated_right_command = 2.5;
  upstream->write_result = hardware_interface::return_type::ERROR;
  const rclcpp::Time time(123, 456, RCL_ROS_TIME);
  const rclcpp::Duration period(0, 20'000'000);

  const auto result = adapter.write(time, period);

  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
  EXPECT_TRUE(upstream->journal_began_before_upstream_write);
  EXPECT_EQ(journal->begin_calls, 1U);
  EXPECT_EQ(journal->finish_calls, 1U);
  EXPECT_EQ(journal->begun_sim_stamp_ns, time.nanoseconds());
  EXPECT_EQ(journal->finished_ticket.write_seq, 37U);
  EXPECT_EQ(journal->finished_ticket.sim_stamp_ns, time.nanoseconds());
  EXPECT_EQ(
    journal->delegated_result,
    static_cast<std::uint64_t>(hardware_interface::return_type::ERROR));
  EXPECT_EQ(
    journal->observation.status,
    voice_nav_sim::HardwareWriteObservationStatus::kValid);
  EXPECT_EQ(
    journal->observation.left_command_bits,
    double_bits(upstream->delegated_left_command));
  EXPECT_EQ(
    journal->observation.right_command_bits,
    double_bits(upstream->delegated_right_command));
}

TEST(
  JournaledGazeboSimSystemAdapter,
  AttachesJournalIdentityBeforeFirstWrite)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  auto attachment =
    std::make_shared<RecordingHardwareWriteJournalAttachment>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(
    upstream, attachment);
  rclcpp::Node::SharedPtr model_node;
  gz::sim::EntityComponentManager entity_component_manager;
  const auto left_entity = entity_component_manager.CreateEntity();
  const auto right_entity = entity_component_manager.CreateEntity();
  std::map<std::string, gz::sim::Entity> joints{
    {"left_wheel_joint", left_entity},
    {"right_wheel_joint", right_entity}};
  hardware_interface::HardwareInfo hardware_info{};
  hardware_info.hardware_parameters = {
    {"journal_name", "/voice_nav_hardware_0011223344556677"},
    {"journal_nonce", "00112233445566778899aabbccddeeff"}};

  ASSERT_TRUE(
    adapter.initSim(
      model_node,
      joints,
      hardware_info,
      entity_component_manager,
      50U));
  EXPECT_EQ(attachment->attach_calls, 1U);
  EXPECT_EQ(
    attachment->attached_name,
    "/voice_nav_hardware_0011223344556677");
  EXPECT_EQ(
    attachment->attached_nonce,
    "00112233445566778899aabbccddeeff");

  (void)adapter.write(
    rclcpp::Time(6, 0, RCL_ROS_TIME), rclcpp::Duration(0, 20'000'000));

  EXPECT_EQ(attachment->journal->begin_calls, 1U);
  EXPECT_EQ(attachment->journal->finish_calls, 1U);
}

TEST(
  JournaledGazeboSimSystemAdapter,
  RejectsIncompleteJournalIdentityWithoutAttaching)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  auto attachment =
    std::make_shared<RecordingHardwareWriteJournalAttachment>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(
    upstream, attachment);
  rclcpp::Node::SharedPtr model_node;
  gz::sim::EntityComponentManager entity_component_manager;
  std::map<std::string, gz::sim::Entity> joints;
  hardware_interface::HardwareInfo hardware_info{};
  hardware_info.hardware_parameters = {
    {"journal_name", "/voice_nav_hardware_0011223344556677"}};

  EXPECT_FALSE(
    adapter.initSim(
      model_node,
      joints,
      hardware_info,
      entity_component_manager,
      50U));
  EXPECT_EQ(attachment->attach_calls, 0U);
}

TEST(
  JournaledGazeboSimSystemAdapter,
  RejectsJournalAttachmentFailure)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  auto attachment =
    std::make_shared<RecordingHardwareWriteJournalAttachment>();
  attachment->throw_on_attach = true;
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(
    upstream, attachment);
  rclcpp::Node::SharedPtr model_node;
  gz::sim::EntityComponentManager entity_component_manager;
  std::map<std::string, gz::sim::Entity> joints;
  hardware_interface::HardwareInfo hardware_info{};
  hardware_info.hardware_parameters = {
    {"journal_name", "/voice_nav_hardware_0011223344556677"},
    {"journal_nonce", "00112233445566778899aabbccddeeff"}};

  EXPECT_FALSE(
    adapter.initSim(
      model_node,
      joints,
      hardware_info,
      entity_component_manager,
      50U));
  EXPECT_EQ(attachment->attach_calls, 1U);
}

TEST(
  JournaledGazeboSimSystemAdapter,
  ReportsMissingEntityAfterFailedReinitialization)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  auto journal = std::make_shared<RecordingHardwareWriteJournal>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream, journal);
  rclcpp::Node::SharedPtr model_node;
  const hardware_interface::HardwareInfo hardware_info{};
  gz::sim::EntityComponentManager first_entity_component_manager;
  const auto first_left = first_entity_component_manager.CreateEntity();
  const auto first_right = first_entity_component_manager.CreateEntity();
  std::map<std::string, gz::sim::Entity> first_joints{
    {"left_wheel_joint", first_left},
    {"right_wheel_joint", first_right}};
  ASSERT_TRUE(
    adapter.initSim(
      model_node,
      first_joints,
      hardware_info,
      first_entity_component_manager,
      50U));

  upstream->init_result = false;
  gz::sim::EntityComponentManager second_entity_component_manager;
  std::map<std::string, gz::sim::Entity> second_joints;
  ASSERT_FALSE(
    adapter.initSim(
      model_node,
      second_joints,
      hardware_info,
      second_entity_component_manager,
      50U));
  upstream->write_entity_component_manager =
    &first_entity_component_manager;
  upstream->write_left_entity = first_left;
  upstream->write_right_entity = first_right;

  const auto result = adapter.write(
    rclcpp::Time(2, 0, RCL_ROS_TIME), rclcpp::Duration(0, 20'000'000));

  EXPECT_EQ(result, hardware_interface::return_type::OK);
  EXPECT_EQ(journal->begin_calls, 1U);
  EXPECT_EQ(journal->finish_calls, 1U);
  EXPECT_EQ(
    journal->observation.status,
    voice_nav_sim::HardwareWriteObservationStatus::kMissingEntity);
  EXPECT_EQ(journal->observation.left_command_bits, 0U);
  EXPECT_EQ(journal->observation.right_command_bits, 0U);
}

TEST(
  JournaledGazeboSimSystemAdapter,
  ReportsMissingWheelCommandComponent)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  auto journal = std::make_shared<RecordingHardwareWriteJournal>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream, journal);
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

  (void)adapter.write(
    rclcpp::Time(3, 0, RCL_ROS_TIME), rclcpp::Duration(0, 20'000'000));

  EXPECT_EQ(journal->begin_calls, 1U);
  EXPECT_EQ(journal->finish_calls, 1U);
  EXPECT_EQ(
    journal->observation.status,
    voice_nav_sim::HardwareWriteObservationStatus::kMissingComponent);
  EXPECT_EQ(journal->observation.left_command_bits, 0U);
  EXPECT_EQ(journal->observation.right_command_bits, 0U);
}

TEST(
  JournaledGazeboSimSystemAdapter,
  ReportsRemovedWheelEntity)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  auto journal = std::make_shared<RecordingHardwareWriteJournal>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream, journal);
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
  entity_component_manager.RequestRemoveEntity(left_entity);
  entity_component_manager.ProcessRemoveEntityRequests();
  ASSERT_FALSE(entity_component_manager.HasEntity(left_entity));

  (void)adapter.write(
    rclcpp::Time(4, 0, RCL_ROS_TIME), rclcpp::Duration(0, 20'000'000));

  EXPECT_EQ(journal->begin_calls, 1U);
  EXPECT_EQ(journal->finish_calls, 1U);
  EXPECT_EQ(
    journal->observation.status,
    voice_nav_sim::HardwareWriteObservationStatus::kMissingEntity);
}

TEST(
  JournaledGazeboSimSystemAdapter,
  ReportsEmptyWheelCommandComponent)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  auto journal = std::make_shared<RecordingHardwareWriteJournal>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream, journal);
  rclcpp::Node::SharedPtr model_node;
  gz::sim::EntityComponentManager entity_component_manager;
  const auto left_entity = entity_component_manager.CreateEntity();
  const auto right_entity = entity_component_manager.CreateEntity();
  (void)entity_component_manager.CreateComponent(
    left_entity,
    gz::sim::components::JointVelocityCmd(std::vector<double>{}));
  (void)entity_component_manager.CreateComponent(
    right_entity,
    gz::sim::components::JointVelocityCmd(std::vector<double>{}));
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

  (void)adapter.write(
    rclcpp::Time(4, 0, RCL_ROS_TIME), rclcpp::Duration(0, 20'000'000));

  EXPECT_EQ(journal->begin_calls, 1U);
  EXPECT_EQ(journal->finish_calls, 1U);
  EXPECT_EQ(
    journal->observation.status,
    voice_nav_sim::HardwareWriteObservationStatus::kEmptyComponent);
  EXPECT_EQ(journal->observation.left_command_bits, 0U);
  EXPECT_EQ(journal->observation.right_command_bits, 0U);
}

TEST(
  JournaledGazeboSimSystemAdapter,
  FinishesJournalCycleWhenDelegatedWriteThrows)
{
  auto upstream = std::make_shared<RecordingGazeboSystem>();
  auto journal = std::make_shared<RecordingHardwareWriteJournal>();
  voice_nav_sim::JournaledGazeboSimSystemAdapter adapter(upstream, journal);
  upstream->write_journal = journal.get();
  upstream->throw_on_write = true;

  EXPECT_THROW(
    adapter.write(
      rclcpp::Time(5, 0, RCL_ROS_TIME),
      rclcpp::Duration(0, 20'000'000)),
    std::runtime_error);

  EXPECT_TRUE(upstream->journal_began_before_upstream_write);
  EXPECT_EQ(journal->begin_calls, 1U);
  EXPECT_EQ(journal->finish_calls, 1U);
  EXPECT_EQ(
    journal->delegated_result,
    voice_nav_sim::kHardwareWriteDelegatedException);
  EXPECT_EQ(
    journal->observation.status,
    voice_nav_sim::HardwareWriteObservationStatus::kMissingEntity);
}

}  // namespace
