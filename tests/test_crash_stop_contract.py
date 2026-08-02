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

#include "hardware_write_sink.hpp"

namespace voice_nav_sim
{
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
};
}  // namespace voice_nav_sim
"""


WRITE_SINK_HEADER = """\
#pragma once

#include <cstdint>

namespace voice_nav_sim
{
struct HardwareWriteRecord
{
  std::uint64_t generation;
  std::uint64_t write_seq;
  std::int64_t sim_stamp_ns;
  std::uint8_t delegated_result;
  std::uint64_t left_command_bits;
  std::uint64_t right_command_bits;
};
}  // namespace voice_nav_sim
"""


ADAPTER_SOURCE = """\
#include "journaled_gazebo_sim_system_adapter.hpp"

namespace voice_nav_sim
{
JournaledGazeboSimSystemAdapter::JournaledGazeboSimSystemAdapter()
: upstream_loader_(
    "gz_ros2_control",
    "gz_ros2_control::GazeboSimSystemInterface"),
  upstream_(upstream_loader_.createSharedInstance(
      "gz_ros2_control/GazeboSimSystem"))
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
  const auto delegated_result = upstream_->write(time, period);
  journal_after_delegated_write(
    time, delegated_result, left_joint, right_joint);
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
  ForwardsLifecycleArgumentsAndResults)
{}

TEST(
  JournaledGazeboSimSystemAdapter,
  ForwardsInterfacesModeSwitchAndIoArgumentsAndResults)
{}

TEST(JournaledGazeboSimSystemAdapter, LoadsExportedAdapterPlugin)
{}
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
        "src/voice_nav_sim/test_support/hardware_write_sink.hpp"
    ): WRITE_SINK_HEADER,
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

    def test_hardware_record_cannot_invent_per_write_iteration(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "hardware_write_sink.hpp"
                ),
                "  std::uint64_t write_seq;",
                (
                    "  std::uint64_t write_seq;\n"
                    "  std::uint64_t iteration;"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must not claim per-write Gazebo iteration", completed.stderr)

    def test_hardware_record_cannot_omit_test_generation(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_sim/test_support/"
                    "hardware_write_sink.hpp"
                ),
                "  std::uint64_t generation;\n",
                "",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("missing observable write facts: generation", completed.stderr)

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
