import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "scripts" / "check_crash_stop_contract.py"


ADAPTER_HEADER = """\
#pragma once

#include <memory>

#include <gz_ros2_control/gz_system_interface.hpp>
#include <pluginlib/class_loader.hpp>

#include "hardware_write_ledger_writer.hpp"

namespace voice_nav_sim
{
class HardwareWriteJournalAttachment
{
public:
  virtual std::shared_ptr<HardwareWriteJournal> attach(
    const std::string & name, const std::string & nonce) = 0;
};

class JournaledGazeboSimSystemAdapter final : public
  gz_ros2_control::GazeboSimSystemInterface
{
public:
  bool initSim();
  CallbackReturn on_init();
  CallbackReturn on_configure();
  CallbackReturn on_activate();
  CallbackReturn on_deactivate();
  StateInterfaces export_state_interfaces();
  CommandInterfaces export_command_interfaces();
  Return prepare_command_mode_switch();
  Return perform_command_mode_switch();
  Return read();
  Return write();

private:
  pluginlib::ClassLoader<
    gz_ros2_control::GazeboSimSystemInterface> upstream_loader_;
  std::shared_ptr<
    gz_ros2_control::GazeboSimSystemInterface> upstream_;
  std::shared_ptr<HardwareWriteJournalAttachment> write_journal_attachment_;
  std::shared_ptr<HardwareWriteJournal> write_journal_;
};
}  // namespace voice_nav_sim
"""


WRITE_JOURNAL_HEADER = """\
#pragma once

#include <cstdint>

namespace voice_nav_sim
{
struct HardwareWriteTicket
{
  std::uint64_t write_seq;
  std::int64_t sim_stamp_ns;
  std::uint64_t bank_index;
  std::uint64_t bank_epoch;
  bool included;
};

struct HardwareWriteWheelObservation
{
  std::uint64_t status;
  std::uint64_t left_command_bits;
  std::uint64_t right_command_bits;
};

class HardwareWriteJournal
{
public:
  virtual HardwareWriteTicket begin_write(
    std::int64_t sim_stamp_ns) noexcept = 0;
  virtual void finish_write(
    HardwareWriteTicket ticket,
    std::uint64_t delegated_result,
    HardwareWriteWheelObservation observation) noexcept = 0;
};
}  // namespace voice_nav_sim
"""


ADAPTER_SOURCE = """\
#include "attached_hardware_write_ledger.hpp"
#include "journaled_gazebo_sim_system_adapter.hpp"

namespace voice_nav_sim
{
class PosixHardwareWriteJournalAttachment final
  : public HardwareWriteJournalAttachment
{
public:
  std::shared_ptr<HardwareWriteJournal> attach(
    const std::string & name, const std::string & nonce) override
  {
    return std::make_shared<AttachedHardwareWriteLedger>(
      HardwareWriteLedgerDiscoveryConfig{name, nonce});
  }
};

JournaledGazeboSimSystemAdapter::JournaledGazeboSimSystemAdapter()
: upstream_loader_(
    "gz_ros2_control",
    "gz_ros2_control::GazeboSimSystemInterface"),
  upstream_(upstream_loader_.createSharedInstance(
      "gz_ros2_control/GazeboSimSystem")),
  write_journal_attachment_(
    std::make_shared<PosixHardwareWriteJournalAttachment>())
{}

bool JournaledGazeboSimSystemAdapter::initSim()
{
  return upstream_->initSim(model, hardware_info, ecm, update_rate);
}

CallbackReturn JournaledGazeboSimSystemAdapter::on_init()
{
  return upstream_->on_init(info);
}

CallbackReturn JournaledGazeboSimSystemAdapter::on_configure()
{
  return upstream_->on_configure(previous_state);
}

CallbackReturn JournaledGazeboSimSystemAdapter::on_activate()
{
  return upstream_->on_activate(previous_state);
}

CallbackReturn JournaledGazeboSimSystemAdapter::on_deactivate()
{
  return upstream_->on_deactivate(previous_state);
}

StateInterfaces JournaledGazeboSimSystemAdapter::export_state_interfaces()
{
  return upstream_->export_state_interfaces();
}

CommandInterfaces JournaledGazeboSimSystemAdapter::export_command_interfaces()
{
  return upstream_->export_command_interfaces();
}

Return JournaledGazeboSimSystemAdapter::prepare_command_mode_switch()
{
  return upstream_->prepare_command_mode_switch(start, stop);
}

Return JournaledGazeboSimSystemAdapter::perform_command_mode_switch()
{
  return upstream_->perform_command_mode_switch(start, stop);
}

Return JournaledGazeboSimSystemAdapter::read()
{
  return upstream_->read(time, period);
}

Return JournaledGazeboSimSystemAdapter::write()
{
  const auto ticket = write_journal_->begin_write(time.nanoseconds());
  const auto delegated_result = upstream_->write(time, period);
  write_journal_->finish_write(
    ticket, static_cast<std::uint64_t>(delegated_result), observation);
  return delegated_result;
}
}  // namespace voice_nav_sim
"""


PLUGIN_DESCRIPTION = """\
<library path="voice_nav_sim_journaled_gazebo_sim_system_adapter">
  <class
    name="voice_nav_sim/JournaledGazeboSimSystemAdapter"
    type="voice_nav_sim::JournaledGazeboSimSystemAdapter"
    base_class_type="gz_ros2_control::GazeboSimSystemInterface">
    <description>Default-off crash evidence Adapter.</description>
  </class>
</library>
"""


ADAPTER_BEHAVIOR_TEST = """\
#include <gtest/gtest.h>

TEST(
  JournaledGazeboSimSystemAdapter,
  LoadsExportedAdapterAndItsPinnedUpstream)
{}

TEST(JournaledGazeboSimSystemAdapter, ForwardsInitSimArgumentsAndResult) {}
TEST(JournaledGazeboSimSystemAdapter, ForwardsOnInitArgumentAndResult) {}
TEST(JournaledGazeboSimSystemAdapter, ForwardsOnConfigureArgumentAndResult) {}
TEST(JournaledGazeboSimSystemAdapter, ForwardsExportedInterfaceCollections) {}
TEST(JournaledGazeboSimSystemAdapter, ForwardsActivationTransitions) {}
TEST(JournaledGazeboSimSystemAdapter, ForwardsCommandModeSwitches) {}
TEST(JournaledGazeboSimSystemAdapter, ForwardsReadAndWriteCycles) {}
TEST(
  JournaledGazeboSimSystemAdapter,
  ObservesActualWheelCommandsAfterDelegatedWrite)
{}
TEST(
  JournaledGazeboSimSystemAdapter,
  ReportsMissingEntityAfterFailedReinitialization)
{}
TEST(
  JournaledGazeboSimSystemAdapter,
  ReportsMissingWheelCommandComponent)
{}
TEST(JournaledGazeboSimSystemAdapter, ReportsRemovedWheelEntity) {}
TEST(JournaledGazeboSimSystemAdapter, ReportsEmptyWheelCommandComponent) {}
TEST(
  JournaledGazeboSimSystemAdapter,
  FinishesJournalCycleWhenDelegatedWriteThrows)
{}
TEST(
  JournaledGazeboSimSystemAdapter,
  AttachesJournalIdentityBeforeFirstWrite)
{}
TEST(
  JournaledGazeboSimSystemAdapter,
  RejectsIncompleteJournalIdentityWithoutAttaching)
{}
TEST(
  JournaledGazeboSimSystemAdapter,
  RejectsJournalAttachmentFailure)
{}
"""


RUNTIME_ADAPTER_TEST = """\
import math
import pytest
import struct
import unittest


def double_from_bits(bits):
    return struct.unpack('<d', struct.pack('<Q', bits))[0]


def expanded_journaled_robot_description():
    product_urdf = xacro.process_file('voice_nav_robot.urdf.xacro').toxml()
    return transform_product_urdf(
        product_urdf,
        LEDGER_OWNER.name,
        LEDGER_OWNER.nonce,
    )


TEST_PARTITION = claim_unique_test_partition('l0010_hw_write')


@pytest.mark.launch_test
def generate_test_description():
    robot_description = expanded_journaled_robot_description()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )
    return LaunchDescription([robot_state_publisher])


class JournaledGazeboHardwareWriteTest(unittest.TestCase):
    def setUp(self, proc_info):
        self.addCleanup(
            gazebo_shutdown.structured_stop_gazebo,
            proc_info,
            expected_partition=TEST_PARTITION,
        )

    def test_real_gazebo_writer_records_nonzero_wheel_commands(
        self,
        proc_info,
        gazebo_action,
        ledger_owner,
    ):
        launched_pid = gazebo_action.process_details['pid']
        self.assertEqual(
            ledger_owner.wait_for_writer(launched_pid),
            launched_pid,
        )
        arm_ticket = ledger_owner.post_arm(
            interval_id=1,
            segment_budget=64,
            invocation_budget=64,
            require_zero_commands=False,
        )
        arm_response = ledger_owner.wait_response(arm_ticket)
        self.assertEqual(arm_response[9], CONTROL_RESPONSE_OK)
        seal_ticket = ledger_owner.post_seal(
            interval_id=1,
            bank_index=arm_response[10],
            bank_epoch=arm_response[11],
            not_before_sim_stamp_ns=0,
            require_exact_stamp=False,
        )
        seal_response = ledger_owner.wait_response(seal_ticket)
        self.assertEqual(seal_response[9], CONTROL_RESPONSE_OK)
        self.assertEqual(seal_response[10], arm_response[10])
        self.assertEqual(seal_response[11], arm_response[11])
        self.assertGreater(seal_response[12], arm_response[12])
        snapshot = ledger_owner.read_sealed_interval(
            interval_id=1,
            bank_index=seal_response[10],
            bank_epoch=seal_response[11],
            seal_fence_write_seq=seal_response[12],
        )
        self.assertEqual(snapshot.terminal_state, BANK_STATE_SEALED_OK)
        self.assertEqual(snapshot.oracle_faults, 0)
        self.assertEqual(
            ledger_owner.load_header_word(GLOBAL_ORACLE_FAULTS_WORD),
            0,
        )
        segments = tuple(
            segment for page in snapshot.pages for segment in page.segments
        )
        self.assertTrue(segments)
        self.assertEqual(segments[0][1], arm_response[12] + 1)
        self.assertEqual(segments[-1][2], seal_response[12])
        self.assertTrue(
            all(
                following[1] == previous[2] + 1
                for previous, following in zip(segments, segments[1:])
            )
        )
        self.assertTrue(all(segment[5] == 0 for segment in segments))
        wheel_commands = tuple(
            (double_from_bits(segment[6]), double_from_bits(segment[7]))
            for segment in segments
        )
        self.assertTrue(
            all(
                math.isfinite(left) and math.isfinite(right)
                for left, right in wheel_commands
            )
        )
        self.assertTrue(
            any(
                abs(left) > 1e-3 and abs(right) > 1e-3
                for left, right in wheel_commands
            )
        )
        self.assertTrue(ledger_owner.acknowledge(snapshot))
"""


SIM_CMAKE = """\
cmake_minimum_required(VERSION 3.22)
project(voice_nav_sim)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(gz_sim_vendor REQUIRED)
  find_package(gz-sim8 REQUIRED)
  find_package(gz_ros2_control REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

  add_library(
    voice_nav_sim_hardware_write_ledger_posix STATIC
    test_support/attached_hardware_write_ledger.cpp
    test_support/hardware_write_ledger_writer.cpp
  )

  add_library(
    voice_nav_sim_journaled_gazebo_sim_system_adapter SHARED
    test_support/journaled_gazebo_sim_system_adapter.cpp
  )
  ament_target_dependencies(
    voice_nav_sim_journaled_gazebo_sim_system_adapter
    gz_ros2_control
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
  target_link_libraries(
    voice_nav_sim_journaled_gazebo_sim_system_adapter PRIVATE
    gz-sim8::gz-sim8
    voice_nav_sim_hardware_write_ledger_posix
  )
  pluginlib_export_plugin_description_file(
    gz_ros2_control
    test_support/journaled_gazebo_sim_system_adapter_plugins.xml
  )
  install(
    TARGETS voice_nav_sim_journaled_gazebo_sim_system_adapter
    DESTINATION lib
  )
  ament_add_gtest(
    journaled_gazebo_sim_system_adapter_test
    test/journaled_gazebo_sim_system_adapter_test.cpp
  )
  target_link_libraries(
    journaled_gazebo_sim_system_adapter_test PRIVATE
    voice_nav_sim_journaled_gazebo_sim_system_adapter
  )
  add_launch_test(
    test/test_journaled_gazebo_hardware_write.py
    TIMEOUT 180
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  set_tests_properties(
    test_test_journaled_gazebo_hardware_write.py
    PROPERTIES RUN_SERIAL TRUE
  )
endif()

ament_package()
"""


SIM_PACKAGE = """\
<?xml version="1.0"?>
<package format="3">
  <name>voice_nav_sim</name>
  <version>0.1.0</version>
  <description>Contract fixture</description>
  <maintainer email="fixture@example.com">Fixture</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <test_depend>gz_ros2_control</test_depend>
  <test_depend>gz_sim_vendor</test_depend>
  <test_depend>hardware_interface</test_depend>
  <test_depend>pal_statistics_msgs</test_depend>
  <test_depend>pluginlib</test_depend>
  <test_depend>rclcpp</test_depend>
  <test_depend>rclcpp_lifecycle</test_depend>
</package>
"""


EVIDENCE_POLICY = """\
PRODUCER_REQUIRE_UNIQUE_FINAL_MARKER = False
GATE_REQUIRE_UNIQUE_FINAL_MARKER = True
GATE_FINAL_MARKER_MAX_COMMITS = 1
GATE_ACK_DEADLINE_OUTPUT_PERIODS = 1

JOURNAL_INSTRUMENTATION_ALLOCATION_FREE = True
UPSTREAM_GAZEBO_WRITE_ALLOCATION_FREE = False


def validate_gate_final_marker(
    marker_commit_count,
    ack_output_seq,
    next_output_seq,
):
    if marker_commit_count != GATE_FINAL_MARKER_MAX_COMMITS:
        raise ValueError('final marker was not committed exactly once')
    if ack_output_seq >= next_output_seq:
        raise ValueError('controller ACK arrived after a repeat')
"""


ROBOT_TRANSFORMER = """\
import copy
import xml.etree.ElementTree as element_tree


PRODUCT_HARDWARE_PLUGIN = 'gz_ros2_control/GazeboSimSystem'
TEST_HARDWARE_PLUGIN = 'voice_nav_sim/JournaledGazeboSimSystemAdapter'


def transform_product_urdf(
    canonical_product_urdf,
    journal_name,
    journal_nonce,
):
    root = element_tree.fromstring(canonical_product_urdf)
    transformed = copy.deepcopy(root)
    hardware_plugins = transformed.findall('./ros2_control/hardware/plugin')
    if len(hardware_plugins) != 1:
        raise ValueError('expected exactly one hardware plugin')
    if hardware_plugins[0].text != PRODUCT_HARDWARE_PLUGIN:
        raise ValueError('canonical hardware plugin changed')
    hardware_plugins[0].text = TEST_HARDWARE_PLUGIN
    hardware = transformed.find('./ros2_control/hardware')
    element_tree.SubElement(
        hardware, 'param', {'name': 'journal_name'}
    ).text = journal_name
    element_tree.SubElement(
        hardware, 'param', {'name': 'journal_nonce'}
    ).text = journal_nonce
    return element_tree.tostring(transformed, encoding='unicode')
"""


PRODUCT_XACRO = """\
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"
       name="voice_nav_robot">
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
  </ros2_control>
</robot>
"""


PRODUCT_LAUNCH = """\
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motion_gate = Node(
        package='voice_nav_mission',
        executable='motion_gate_node',
        parameters=['motion_gate.yaml'],
    )
    return LaunchDescription([motion_gate])
"""


SIMULATION_LAUNCH = """\
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
"""


PRODUCT_GATE_YAML = """\
motion_gate_node:
  ros__parameters:
    use_sim_time: true
    output_frequency_hz: 50.0
"""


FIXTURE_FILES = {
    (
        "src/voice_nav_sim/test_support/hardware_write_ledger_writer.hpp"
    ): WRITE_JOURNAL_HEADER,
    (
        "src/voice_nav_sim/test_support/"
        "journaled_gazebo_sim_system_adapter.hpp"
    ): ADAPTER_HEADER,
    (
        "src/voice_nav_sim/test_support/"
        "journaled_gazebo_sim_system_adapter.cpp"
    ): ADAPTER_SOURCE,
    (
        "src/voice_nav_sim/test_support/"
        "journaled_gazebo_sim_system_adapter_plugins.xml"
    ): PLUGIN_DESCRIPTION,
    (
        "src/voice_nav_sim/test/"
        "journaled_gazebo_sim_system_adapter_test.cpp"
    ): ADAPTER_BEHAVIOR_TEST,
    (
        "src/voice_nav_sim/test/"
        "test_journaled_gazebo_hardware_write.py"
    ): RUNTIME_ADAPTER_TEST,
    (
        "src/voice_nav_sim/test_support/crash_stop_policy.py"
    ): EVIDENCE_POLICY,
    (
        "src/voice_nav_sim/test_support/crash_robot_description.py"
    ): ROBOT_TRANSFORMER,
    (
        "src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro"
    ): PRODUCT_XACRO,
    (
        "src/voice_nav_bringup/launch/product_sim.launch.py"
    ): PRODUCT_LAUNCH,
    (
        "src/voice_nav_sim/launch/simulation.launch.py"
    ): SIMULATION_LAUNCH,
    "src/voice_nav_sim/CMakeLists.txt": SIM_CMAKE,
    "src/voice_nav_sim/package.xml": SIM_PACKAGE,
    (
        "src/voice_nav_bringup/config/motion_gate.yaml"
    ): PRODUCT_GATE_YAML,
}


class CrashStopContractTest(unittest.TestCase):
    def create_fixture(self, root: Path) -> None:
        for relative_path, contents in FIXTURE_FILES.items():
            path = root / relative_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(textwrap.dedent(contents), encoding="utf-8")

    def run_checker(
        self,
        mutation=None,
    ) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.create_fixture(root)
            if mutation is not None:
                mutation(root)
            return subprocess.run(
                [
                    sys.executable,
                    str(CHECKER),
                    "--root",
                    str(root),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

    def run_repository_snapshot_checker(
        self,
        mutation,
    ) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for relative_path in FIXTURE_FILES:
                source_path = REPOSITORY_ROOT / relative_path
                destination = root / relative_path
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_text(
                    source_path.read_text(encoding="utf-8"),
                    encoding="utf-8",
                )
            mutation(root)
            return subprocess.run(
                [
                    sys.executable,
                    str(CHECKER),
                    "--root",
                    str(root),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

    @staticmethod
    def replace(
        root: Path,
        relative_path: str,
        old: str,
        new: str,
    ) -> None:
        path = root / relative_path
        source = path.read_text(encoding="utf-8")
        if old not in source:
            raise AssertionError(
                f"fixture mutation source not found in {relative_path}: "
                f"{old}"
            )
        path.write_text(source.replace(old, new, 1), encoding="utf-8")

    def test_synthetic_valid_contract_passes(self) -> None:
        completed = self.run_checker()

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("Crash-stop contract passed", completed.stdout)

    def test_repository_crash_stop_contract_passes(self) -> None:
        completed = subprocess.run(
            [
                sys.executable,
                str(CHECKER),
                "--root",
                str(REPOSITORY_ROOT),
            ],
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_direct_concrete_hardware_subclass_is_rejected(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "journaled_gazebo_sim_system_adapter.hpp"
                ),
                (
                    "class JournaledGazeboSimSystemAdapter final : public\n"
                    "  gz_ros2_control::GazeboSimSystemInterface"
                ),
                (
                    "class JournaledGazeboSimSystemAdapter final : public\n"
                    "  gz_ros2_control::GazeboSimSystem"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must not directly subclass", completed.stderr)

    def test_plugin_description_must_name_exact_adapter_type(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "journaled_gazebo_sim_system_adapter_plugins.xml"
                ),
                'type="voice_nav_sim::JournaledGazeboSimSystemAdapter"',
                'type="voice_nav_sim::WrongAdapter"',
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("exact owned Adapter type", completed.stderr)

    def test_adapter_cannot_own_write_sequence(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "journaled_gazebo_sim_system_adapter.hpp"
                ),
                "  std::shared_ptr<HardwareWriteJournal> write_journal_;",
                (
                    "  std::shared_ptr<HardwareWriteJournal> write_journal_;\n"
                    "  std::uint64_t next_write_seq_;"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must not own Writer sequence", completed.stderr)

    def test_adapter_cannot_construct_legacy_write_records(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "journaled_gazebo_sim_system_adapter.cpp"
                ),
                "  const auto ticket =",
                "  HardwareWriteRecord record{};\n  const auto ticket =",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must not use legacy sink or record", completed.stderr)

    def test_journal_ticket_cannot_omit_write_sequence(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "hardware_write_ledger_writer.hpp"
                ),
                "  std::uint64_t write_seq;\n",
                "",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("ticket is missing Writer-owned facts: write_seq", completed.stderr)

    def test_journal_interface_requires_finish_write(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "hardware_write_ledger_writer.hpp"
                ),
                "virtual void finish_write(",
                "virtual void disabled_finish_write(",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must define begin_write and finish_write", completed.stderr)

    def test_adapter_must_begin_before_delegated_write(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "journaled_gazebo_sim_system_adapter.cpp"
                ),
                "write_journal_->begin_write(",
                "write_journal_->disabled_begin_write(",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("begin before the delegated upstream write", completed.stderr)

    def test_adapter_must_finish_after_delegated_write(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "journaled_gazebo_sim_system_adapter.cpp"
                ),
                "write_journal_->finish_write(",
                "write_journal_->disabled_finish_write(",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("finish after the delegated upstream write", completed.stderr)

    def test_default_adapter_requires_posix_attachment(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "journaled_gazebo_sim_system_adapter.cpp"
                ),
                "std::make_shared<PosixHardwareWriteJournalAttachment>()",
                "std::shared_ptr<HardwareWriteJournalAttachment>{}",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("default Adapter must construct", completed.stderr)

    def test_runtime_evidence_requires_exact_gazebo_writer_pid(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                "ledger_owner.wait_for_writer(launched_pid)",
                "ledger_owner.disabled_wait_for_writer(launched_pid)",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("exact Gazebo Writer PID", completed.stderr)

    def test_runtime_evidence_rejects_derived_writer_pid(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                "gazebo_action.process_details['pid']",
                "gazebo_action.process_details['pid'] + 1",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("exact Gazebo Writer PID", completed.stderr)

    def test_runtime_evidence_requires_complete_ledger_interval(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                "ledger_owner.post_seal(",
                "ledger_owner.disabled_post_seal(",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "ARM, SEAL, immutable snapshot, and ACK",
            completed.stderr,
        )

    def test_runtime_evidence_requires_clean_nonzero_wheel_writes(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                "segment[5] == 0",
                "segment[5] >= 0",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "VALID upstream-OK non-zero wheel evidence",
            completed.stderr,
        )

    def test_runtime_segments_must_come_from_snapshot_pages(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                (
                    "segment for page in snapshot.pages "
                    "for segment in page.segments"
                ),
                (
                    "(0, arm_response[12] + 1, seal_response[12], "
                    "0, 0, 0, 0x3FF0000000000000, "
                    "0x3FF0000000000000) for _ in (snapshot,)"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "VALID upstream-OK non-zero wheel evidence",
            completed.stderr,
        )

    def test_real_runtime_terminal_tokens_outside_assertion_fail(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                (
                    "        self.assertEqual(\n"
                    "            snapshot.terminal_state,\n"
                    "            BANK_STATE_SEALED_OK,\n"
                ),
                (
                    "        retained_terminal_tokens = (\n"
                    "            snapshot.terminal_state,\n"
                    "            BANK_STATE_SEALED_OK,\n"
                ),
            )

        completed = self.run_repository_snapshot_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "fault-free terminal ledger",
            completed.stderr,
        )

    def test_runtime_terminal_assertion_cannot_reverse_polarity(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                (
                    "self.assertEqual("
                    "snapshot.terminal_state, BANK_STATE_SEALED_OK)"
                ),
                (
                    "self.assertNotEqual("
                    "snapshot.terminal_state, BANK_STATE_SEALED_OK)"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("fault-free terminal ledger", completed.stderr)

    def test_runtime_seal_cannot_swap_arm_bank_and_epoch(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                "bank_index=arm_response[10]",
                "bank_index=arm_response[11]",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "ARM, SEAL, immutable snapshot, and ACK",
            completed.stderr,
        )

    def test_runtime_evidence_cannot_return_before_proof(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                "        launched_pid = gazebo_action.process_details['pid']",
                (
                    "        return\n"
                    "        launched_pid = gazebo_action.process_details['pid']"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must remain executable", completed.stderr)

    def test_runtime_evidence_cannot_skip_before_proof(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                "        launched_pid = gazebo_action.process_details['pid']",
                (
                    "        self.skipTest('disabled')\n"
                    "        launched_pid = gazebo_action.process_details['pid']"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must remain executable", completed.stderr)

    def test_runtime_evidence_cannot_be_decorated_as_skipped(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                (
                    "    def test_real_gazebo_writer_records_nonzero_"
                    "wheel_commands("
                ),
                (
                    "    @unittest.skip('disabled')\n"
                    "    def test_real_gazebo_writer_records_nonzero_"
                    "wheel_commands("
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must remain executable", completed.stderr)

    def test_runtime_evidence_class_must_be_collected(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                (
                    "class JournaledGazeboHardwareWriteTest("
                    "unittest.TestCase):"
                ),
                "class JournaledGazeboHardwareWriteTest:",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must remain executable", completed.stderr)

    def test_runtime_assertion_methods_cannot_be_replaced(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                "        launched_pid = gazebo_action.process_details['pid']",
                (
                    "        self.assertEqual = lambda *args, **kwargs: None\n"
                    "        self.assertGreater = lambda *args, **kwargs: None\n"
                    "        self.assertTrue = lambda *args, **kwargs: None\n"
                    "        launched_pid = gazebo_action.process_details['pid']"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("unmodified unittest assertions", completed.stderr)

    def test_runtime_evidence_requires_canonical_test_transform(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                "transform_product_urdf(",
                "disabled_transform_product_urdf(",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("canonical product Xacro", completed.stderr)

    def test_runtime_transform_result_must_reach_the_launch_graph(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                (
                    "    return transform_product_urdf(\n"
                    "        product_urdf,\n"
                    "        LEDGER_OWNER.name,\n"
                    "        LEDGER_OWNER.nonce,\n"
                    "    )"
                ),
                (
                    "    transform_product_urdf(\n"
                    "        product_urdf,\n"
                    "        LEDGER_OWNER.name,\n"
                    "        LEDGER_OWNER.nonce,\n"
                    "    )\n"
                    "    return product_urdf"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("canonical product Xacro", completed.stderr)

    def test_runtime_expansion_result_cannot_be_discarded(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                (
                    "    robot_description = "
                    "expanded_journaled_robot_description()"
                ),
                (
                    "    expanded_journaled_robot_description()\n"
                    "    robot_description = 'untransformed'"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("canonical product Xacro", completed.stderr)

    def test_runtime_transform_must_feed_robot_state_publisher(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                "'robot_description': robot_description,",
                (
                    "'robot_description': '<robot name=\"not_canonical\"/>',\n"
                    "                'proof_only_robot_description': "
                    "robot_description,"
                ),
            )

        completed = self.run_repository_snapshot_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("canonical product Xacro", completed.stderr)

    def test_runtime_evidence_requires_partition_scoped_cleanup(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test/"
                    "test_journaled_gazebo_hardware_write.py"
                ),
                "gazebo_shutdown.structured_stop_gazebo,",
                "gazebo_shutdown.disabled_structured_stop_gazebo,",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "partition-scoped structured Gazebo stop",
            completed.stderr,
        )

    def test_runtime_evidence_must_be_registered_as_isolated(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "    test/test_journaled_gazebo_hardware_write.py\n",
                "    test/disabled_journaled_gazebo_hardware_write.py\n",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "register the isolated runtime Adapter test",
            completed.stderr,
        )

    def test_runtime_evidence_must_be_serialized(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "    PROPERTIES RUN_SERIAL TRUE\n",
                "    PROPERTIES RUN_SERIAL FALSE\n",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("serialize its Gazebo process", completed.stderr)

    def test_runtime_evidence_requires_runner_keyword(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "    RUNNER \"${ament_cmake_ros_DIR}/run_test_isolated.py\"\n",
                "    ARGS \"${ament_cmake_ros_DIR}/run_test_isolated.py\"\n",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("register the isolated runtime Adapter test", completed.stderr)

    def test_runtime_runner_text_inside_labels_is_not_a_runner(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                (
                    "    TIMEOUT 180\n"
                    "    RUNNER \"${ament_cmake_ros_DIR}/run_test_isolated.py\"\n"
                ),
                (
                    "    TIMEOUT 180\n"
                    "    LABELS launch_test RUNNER "
                    "\"${ament_cmake_ros_DIR}/run_test_isolated.py\"\n"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("register the isolated runtime Adapter test", completed.stderr)

    def test_runtime_target_cannot_redirect_the_serial_property(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "    TIMEOUT 180\n",
                "    TARGET unowned_runtime_target\n    TIMEOUT 180\n",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("register the isolated runtime Adapter test", completed.stderr)

    def test_runtime_registration_must_be_reachable_when_testing(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "if(BUILD_TESTING)",
                "if(BUILD_TESTING AND FALSE)",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("register the isolated runtime Adapter test", completed.stderr)

    def test_runtime_registration_command_cannot_be_shadowed(self) -> None:
        def mutation(root: Path) -> None:
            path = root / "src/voice_nav_sim/CMakeLists.txt"
            source = path.read_text(encoding="utf-8")
            path.write_text(
                (
                    "macro(add_launch_test test_path)\n"
                    "  get_filename_component(test_name \"${test_path}\" NAME)\n"
                    "  add_test(\n"
                    "    NAME \"test_${test_name}\"\n"
                    "    COMMAND \"${CMAKE_COMMAND}\" -E true\n"
                    "  )\n"
                    "endmacro()\n"
                )
                + source,
                encoding="utf-8",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("register the isolated runtime Adapter test", completed.stderr)

    def test_runtime_serial_text_inside_label_is_not_a_property(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "    PROPERTIES RUN_SERIAL TRUE\n",
                '    PROPERTIES LABELS "RUN_SERIAL TRUE"\n',
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("serialize its Gazebo process", completed.stderr)

    def test_runtime_serial_property_must_target_the_runtime_test(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                (
                    "    test_test_journaled_gazebo_hardware_write.py\n"
                    "    PROPERTIES RUN_SERIAL TRUE\n"
                ),
                (
                    "    journaled_gazebo_sim_system_adapter_test\n"
                    "    PROPERTIES LABELS "
                    "test_test_journaled_gazebo_hardware_write.py "
                    "RUN_SERIAL TRUE\n"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("serialize its Gazebo process", completed.stderr)

    def test_later_runtime_serial_assignment_cannot_disable_it(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "endif()\n\nament_package()",
                (
                    "  set_tests_properties(\n"
                    "    test_test_journaled_gazebo_hardware_write.py\n"
                    "    PROPERTIES RUN_SERIAL FALSE\n"
                    "  )\n"
                    "endif()\n\nament_package()"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("serialize its Gazebo process", completed.stderr)

    def test_variable_target_cannot_override_runtime_serialization(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "endif()\n\nament_package()",
                (
                    "  set(\n"
                    "    runtime_test_name\n"
                    "    test_test_journaled_gazebo_hardware_write.py\n"
                    "  )\n"
                    "  set_tests_properties(\n"
                    "    ${runtime_test_name}\n"
                    "    PROPERTIES RUN_SERIAL FALSE\n"
                    "  )\n"
                    "endif()\n\nament_package()"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("serialize its Gazebo process", completed.stderr)

    def test_explicit_cmake_truthy_serial_values_are_supported(self) -> None:
        for truthy in ("TRUE", "true", "ON", "YES", "Y", "1"):
            with self.subTest(truthy=truthy):
                def mutation(root: Path) -> None:
                    self.replace(
                        root,
                        "src/voice_nav_sim/CMakeLists.txt",
                        "    PROPERTIES RUN_SERIAL TRUE\n",
                        f"    PROPERTIES RUN_SERIAL {truthy}\n",
                    )

                completed = self.run_checker(mutation)

                self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_runtime_method_cannot_be_replaced_after_definition(self) -> None:
        def mutation(root: Path) -> None:
            path = (
                root
                / "src/voice_nav_sim/test/"
                "test_journaled_gazebo_hardware_write.py"
            )
            source = path.read_text(encoding="utf-8")
            path.write_text(
                source
                + (
                    "\nJournaledGazeboHardwareWriteTest."
                    "test_real_gazebo_writer_records_nonzero_wheel_commands = (\n"
                    "    lambda self, proc_info, gazebo_action, ledger_owner: "
                    "None\n"
                    ")\n"
                ),
                encoding="utf-8",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must remain executable", completed.stderr)

    def test_runtime_assertions_must_use_self_receiver(self) -> None:
        def mutation(root: Path) -> None:
            path = (
                root
                / "src/voice_nav_sim/test/"
                "test_journaled_gazebo_hardware_write.py"
            )
            source = path.read_text(encoding="utf-8")
            source = source.replace(
                "import unittest\n",
                "import unittest\nfrom unittest.mock import Mock\n",
                1,
            )
            source = source.replace(
                "TEST_PARTITION = ",
                "NOOP_ASSERTIONS = Mock()\n\nTEST_PARTITION = ",
                1,
            )
            for method in ("assertEqual", "assertGreater", "assertTrue"):
                source = source.replace(
                    f"self.{method}(",
                    f"NOOP_ASSERTIONS.{method}(",
                )
            path.write_text(source, encoding="utf-8")

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("unmodified unittest assertions", completed.stderr)

    def test_runtime_proof_builtins_cannot_be_rebound(self) -> None:
        def mutation(root: Path) -> None:
            path = (
                root
                / "src/voice_nav_sim/test/"
                "test_journaled_gazebo_hardware_write.py"
            )
            source = path.read_text(encoding="utf-8")
            path.write_text(
                "any = lambda *_args, **_kwargs: True\n" + source,
                encoding="utf-8",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("proof primitives", completed.stderr)

    def test_runtime_publisher_must_be_direct_launch_action(self) -> None:
        def mutation(root: Path) -> None:
            path = (
                root
                / "src/voice_nav_sim/test/"
                "test_journaled_gazebo_hardware_write.py"
            )
            source = path.read_text(encoding="utf-8")
            source = source.replace(
                "from launch.actions import ",
                "from launch.actions import OpaqueFunction, ",
                1,
            )
            source = source.replace(
                "                robot_state_publisher,\n",
                (
                    "                OpaqueFunction(\n"
                    "                    function=lambda _context: (\n"
                    "                        [] if robot_state_publisher else []\n"
                    "                    ),\n"
                    "                ),\n"
                ),
                1,
            )
            path.write_text(source, encoding="utf-8")

        completed = self.run_repository_snapshot_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("canonical product Xacro", completed.stderr)

    def test_runtime_registration_rejects_build_testing_rebind(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "if(BUILD_TESTING)",
                "set(BUILD_TESTING FALSE)\nif(BUILD_TESTING)",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("register the isolated runtime Adapter test", completed.stderr)

    def test_runtime_registration_rejects_python_override(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "    TIMEOUT 180\n",
                "    TIMEOUT 180\n    PYTHON_EXECUTABLE /bin/true\n",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("register the isolated runtime Adapter test", completed.stderr)

    def test_runtime_registration_rejects_skip_test(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "    RUNNER \"${ament_cmake_ros_DIR}/run_test_isolated.py\"\n",
                (
                    "    RUNNER "
                    "\"${ament_cmake_ros_DIR}/run_test_isolated.py\"\n"
                    "    SKIP_TEST\n"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("register the isolated runtime Adapter test", completed.stderr)

    def test_dynamic_property_name_cannot_disable_serialization(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "endif()\n\nament_package()",
                (
                    "  set(serial_property RUN_SERIAL)\n"
                    "  set_tests_properties(\n"
                    "    test_test_journaled_gazebo_hardware_write.py\n"
                    "    PROPERTIES ${serial_property} FALSE\n"
                    "  )\n"
                    "endif()\n\nament_package()"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("serialize its Gazebo process", completed.stderr)

    def test_runtime_evidence_cannot_sweep_processes(self) -> None:
        def mutation(root: Path) -> None:
            path = (
                root
                / "src/voice_nav_sim/test/"
                "test_journaled_gazebo_hardware_write.py"
            )
            source = path.read_text(encoding="utf-8")
            path.write_text(source + "\nos.kill(1, 9)\n", encoding="utf-8")

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "without broad process discovery or signals",
            completed.stderr,
        )

    def test_adapter_target_requires_posix_ledger_link(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/CMakeLists.txt",
                "    gz-sim8::gz-sim8\n"
                "    voice_nav_sim_hardware_write_ledger_posix\n",
                "    gz-sim8::gz-sim8\n",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must link the POSIX hardware ledger", completed.stderr)

    def test_adapter_cannot_drop_upstream_lifecycle_delegation(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "journaled_gazebo_sim_system_adapter.cpp"
                ),
                "  return upstream_->on_activate(previous_state);",
                "  return CallbackReturn::SUCCESS;",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("missing: on_activate", completed.stderr)

    def test_product_xacro_cannot_select_the_test_adapter(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/urdf/"
                    "voice_nav_robot.urdf.xacro"
                ),
                "gz_ros2_control/GazeboSimSystem",
                "voice_nav_sim/JournaledGazeboSimSystemAdapter",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("product Xacro must not expose", completed.stderr)

    def test_product_xacro_cannot_contain_second_hardware_plugin(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/urdf/"
                    "voice_nav_robot.urdf.xacro"
                ),
                "    </hardware>",
                (
                    "      <plugin>gz_ros2_control/GazeboSimSystem</plugin>\n"
                    "    </hardware>"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("exactly one unchanged upstream", completed.stderr)

    def test_transformer_cannot_modify_unrelated_product_xml(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/test_support/crash_robot_description.py",
                "    transformed = copy.deepcopy(root)",
                (
                    "    transformed = copy.deepcopy(root)\n"
                    "    transformed.set('name', 'changed')"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("changed XML outside", completed.stderr)

    def test_transformer_license_url_is_not_a_machine_path(self) -> None:
        def mutation(root: Path) -> None:
            path = (
                root
                / "src/voice_nav_sim/test_support/"
                "crash_robot_description.py"
            )
            source = path.read_text(encoding="utf-8")
            path.write_text(
                "# See http://www.apache.org/licenses/LICENSE-2.0\n"
                + source,
                encoding="utf-8",
            )

        completed = self.run_checker(mutation)

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("Crash-stop contract passed", completed.stdout)

    def test_transformer_machine_specific_paths_are_rejected(self) -> None:
        machine_paths = (
            "C:/Users/alice/robot.urdf",
            "C:\\Users\\alice\\robot.urdf",
            "/home/alice/robot.urdf",
            "/mnt/c/Users/alice/robot.urdf",
        )

        for machine_path in machine_paths:
            with self.subTest(machine_path=machine_path):
                def mutation(root: Path) -> None:
                    path = (
                        root
                        / "src/voice_nav_sim/test_support/"
                        "crash_robot_description.py"
                    )
                    source = path.read_text(encoding="utf-8")
                    path.write_text(
                        f"# machine path: {machine_path}\n" + source,
                        encoding="utf-8",
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    "must not contain a machine-specific absolute path",
                    completed.stderr,
                )

    def test_adapter_requires_direct_statistics_dependency(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/package.xml",
                "  <test_depend>pal_statistics_msgs</test_depend>\n",
                "",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "missing direct test dependencies: pal_statistics_msgs",
            completed.stderr,
        )

    def test_product_launch_cannot_enable_gate_event_journal_pair(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_bringup/launch/"
                    "product_sim.launch.py"
                ),
                "parameters=['motion_gate.yaml'],",
                (
                    "parameters=['motion_gate.yaml', {"
                    "'test_gate_event_journal_name': "
                    "'/voice_nav_gate_00112233445566778899aabbccddeeff', "
                    "'test_gate_event_journal_descriptor': "
                    "'v1:1000:7:16:123456789abcdef00fedcba987654321'"
                    "}],"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("product launch must not expose", completed.stderr)
        self.assertIn("test_gate_event_journal_name", completed.stderr)
        self.assertIn("test_gate_event_journal_descriptor", completed.stderr)

    def test_product_simulation_launch_cannot_select_test_adapter(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/launch/simulation.launch.py",
                "    return LaunchDescription([])",
                (
                    "    test_hardware = "
                    "'voice_nav_sim/JournaledGazeboSimSystemAdapter'\n"
                    "    return LaunchDescription([test_hardware])"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "product simulation launch must not expose",
            completed.stderr,
        )

    def test_product_yaml_cannot_enable_gate_event_journal_pair(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_bringup/config/motion_gate.yaml"
                ),
                "    output_frequency_hz: 50.0",
                (
                    "    output_frequency_hz: 50.0\n"
                    "    test_gate_event_journal_name: "
                    "/voice_nav_gate_00112233445566778899aabbccddeeff\n"
                    "    test_gate_event_journal_descriptor: "
                    "v1:1000:7:16:123456789abcdef00fedcba987654321"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("product MotionGate YAML must not expose", completed.stderr)
        self.assertIn("test_gate_event_journal_name", completed.stderr)
        self.assertIn("test_gate_event_journal_descriptor", completed.stderr)

    def test_producer_barrier_does_not_require_unique_marker(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/crash_stop_policy.py"
                ),
                "PRODUCER_REQUIRE_UNIQUE_FINAL_MARKER = False",
                "PRODUCER_REQUIRE_UNIQUE_FINAL_MARKER = True",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "producer arming barriers must not require a unique final marker",
            completed.stderr,
        )

    def test_gate_final_marker_cannot_be_committed_twice(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/crash_stop_policy.py"
                ),
                "GATE_FINAL_MARKER_MAX_COMMITS = 1",
                "GATE_FINAL_MARKER_MAX_COMMITS = 2",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("exactly one COMMITTED Gate output", completed.stderr)

    def test_gate_controller_ack_cannot_arrive_after_repeat(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/crash_stop_policy.py"
                ),
                "GATE_ACK_DEADLINE_OUTPUT_PERIODS = 1",
                "GATE_ACK_DEADLINE_OUTPUT_PERIODS = 2",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("ACK must arrive before the next Gate output", completed.stderr)

    def test_allocation_free_claim_cannot_include_upstream_write(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/crash_stop_policy.py"
                ),
                "UPSTREAM_GAZEBO_WRITE_ALLOCATION_FREE = False",
                "UPSTREAM_GAZEBO_WRITE_ALLOCATION_FREE = True",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "allocation-free claim must be scoped to added preallocated",
            completed.stderr,
        )


if __name__ == "__main__":
    unittest.main()
