import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "scripts" / "check_motion_gate_contract.py"

CONTROL_INTERFACE = """\
uint8 PREPARE=1
uint8 OPEN=2
uint8 RENEW=3
uint8 INHIBIT=4

uint8 operation
string<=36 request_id
string<=36 gate_instance_id
string<=36 lease_id
uint64 expected_control_seq
---
uint16 APPLIED=0
uint16 DUPLICATE=1
uint16 REJECTED=2
uint16 FAULTED=3

uint16 code
uint16 reason
uint64 control_seq
string<=36 gate_instance_id
string<=36 lease_id
uint8 state
bool writer_bound
bool zero_selected
bool motion_inhibited
bool zero_published
uint64 output_publish_seq
uint64 zero_publish_seq
uint8[16] bound_writer_gid
string<=128 candidate_topic
string<=160 detail
"""

STATE_INTERFACE = """\
uint8 INHIBITED=0
uint8 PREPARED=1
uint8 ARMED=2
uint8 FAULTED=3

string<=36 gate_instance_id
uint64 control_seq
uint64 state_seq
uint8 state
string<=36 lease_id
bool authority_live
bool candidate_fresh
bool writer_bound
bool zero_selected
bool motion_inhibited
uint64 output_publish_seq
uint64 zero_publish_seq
uint8[16] bound_writer_gid
string<=128 candidate_topic
uint16 reason
string<=160 detail
"""

CORE_HEADER = """\
#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace voice_nav_mission
{
enum class State {Inhibited, Prepared, Armed, Faulted};
struct ControlRequest;
struct ControlResult;
struct Candidate;
struct Command;

class MotionGateCore
{
public:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;
  ControlResult prepare(const ControlRequest &, SteadyTimePoint);
  ControlResult open(const ControlRequest &, SteadyTimePoint);
  ControlResult renew(const ControlRequest &, SteadyTimePoint);
  ControlResult inhibit(const ControlRequest &, SteadyTimePoint);
  ControlResult accept_candidate(const Candidate &, SteadyTimePoint);
  Command tick(SteadyTimePoint);

private:
  State state_{State::Inhibited};
  std::unordered_map<std::string, ControlResult> request_id_cache_;
  std::uint64_t control_seq_{0};
};
}  // namespace voice_nav_mission
"""

CORE_SOURCE = """\
#include "voice_nav_mission/motion_gate_core.hpp"

#include <algorithm>
#include <cmath>

namespace voice_nav_mission
{
ControlResult MotionGateCore::prepare(
  const ControlRequest & request,
  SteadyTimePoint now)
{
  (void)now;
  if (state_ != State::Inhibited ||
    request.expected_control_seq != control_seq_)
  {
    return stale_request();
  }
  const auto cached = request_id_cache_.find(request.request_id);
  if (cached != request_id_cache_.end()) {
    if (cached->second.request_fingerprint != request_fingerprint(request)) {
      return conflicting_request_id();
    }
    return duplicate_request(cached->second);
  }
  current_lease_id_ = make_lease_id();
  ++control_seq_;
  state_ = State::Prepared;
  return applied_result();
}

ControlResult MotionGateCore::open(
  const ControlRequest & request,
  SteadyTimePoint now)
{
  if (state_ != State::Prepared ||
    request.expected_control_seq != control_seq_ ||
    !writer_bound_)
  {
    return stale_request();
  }
  authority_deadline_ = now + authority_lease_;
  candidate_deadline_ = now + candidate_freshness_;
  ++control_seq_;
  state_ = State::Armed;
  return applied_result();
}

ControlResult MotionGateCore::renew(
  const ControlRequest & request,
  SteadyTimePoint now)
{
  if (state_ != State::Armed ||
    request.expected_control_seq != control_seq_ ||
    now >= authority_deadline_)
  {
    return stale_request();
  }
  ++control_seq_;
  authority_deadline_ = now + authority_lease_;
  return applied_result();
}

ControlResult MotionGateCore::inhibit(
  const ControlRequest & request,
  SteadyTimePoint now)
{
  (void)now;
  const auto cached = request_id_cache_.find(request.request_id);
  if (cached != request_id_cache_.end()) {
    return duplicate_request(cached->second);
  }
  return retire_lease(request);
}

ControlResult MotionGateCore::accept_candidate(
  const Candidate & candidate,
  SteadyTimePoint now)
{
  const bool finite =
    std::isfinite(candidate.linear_x) &&
    std::isfinite(candidate.linear_y) &&
    std::isfinite(candidate.linear_z) &&
    std::isfinite(candidate.angular_x) &&
    std::isfinite(candidate.angular_y) &&
    std::isfinite(candidate.angular_z);
  if (!finite ||
    candidate.linear_y != 0.0 ||
    candidate.linear_z != 0.0 ||
    candidate.angular_x != 0.0 ||
    candidate.angular_y != 0.0)
  {
    return retire_lease(candidate);
  }
  selected_.linear_x =
    std::clamp(candidate.linear_x, linear_x_min_, linear_x_max_);
  selected_.angular_z =
    std::clamp(candidate.angular_z, angular_z_min_, angular_z_max_);
  candidate_deadline_ = now + candidate_freshness_;
  return applied_result();
}

Command MotionGateCore::tick(SteadyTimePoint now)
{
  if (now >= authority_deadline_ || now >= candidate_deadline_) {
    retire_lease(timeout_reason());
    return zero_command();
  }
  return selected_;
}
}  // namespace voice_nav_mission
"""

NODE_SOURCE = """\
#include <chrono>
#include <mutex>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"

using namespace std::chrono_literals;
static_assert(RMW_GID_STORAGE_SIZE == 16u);

class MotionGateNode : public rclcpp::Node
{
public:
  MotionGateNode()
  : Node("motion_gate_node")
  {
    auto candidate_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort().durability_volatile();
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable().transient_local();
    rcl_interfaces::msg::ParameterDescriptor safety_descriptor;
    safety_descriptor.read_only = true;
    (void)candidate_qos;
    (void)state_qos;
    (void)safety_descriptor;
    (void)"/motion_gate/internal/control";
    (void)"/motion_gate/internal/state";
    (void)"/voice_nav_internal/motion_gate/candidate/lease_";
    (void)"/diff_drive_controller/cmd_vel";
    final_command_publisher_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(
      final_command_topic_, rclcpp::SystemDefaultsQoS());
    output_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() {
        const auto now = std::chrono::steady_clock::now();
        publish_serialized(core_.tick(now));
      });
  }

  std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>
  discover_unique_writer_gid_on_topic(const std::string & topic)
  {
    const auto endpoints = get_publishers_info_by_topic(topic);
    if (endpoints.size() != 1) {
      throw std::runtime_error("candidate topic must have one writer");
    }
    return endpoints.front().endpoint_gid();
  }

  void open_candidate_reader(
    const ControlRequest & request,
    ControlResponse & response)
  {
    const auto bound_writer_gid =
      discover_unique_writer_gid_on_topic(candidate_topic_);
    candidate_subscription_.reset();
    candidate_subscription_ =
      create_candidate_subscription(candidate_topic_);
    bind_writer_gid(bound_writer_gid);
    core_.open(request, std::chrono::steady_clock::now());
    response.code = response.APPLIED;
  }

  void on_candidate(
    const geometry_msgs::msg::TwistStamped & message,
    const rclcpp::MessageInfo & message_info)
  {
    const auto & publisher_gid =
      message_info.get_rmw_message_info().publisher_gid;
    core_.accept_candidate(
      to_candidate(message, publisher_gid),
      std::chrono::steady_clock::now());
  }

  void publish_serialized(Command command)
  {
    std::scoped_lock lock(publication_mutex_);
    command.header.stamp = get_clock()->now();
    final_command_publisher_->publish(command);
  }

  void handle_inhibit(
    const ControlRequest & request,
    ControlResponse * response)
  {
    core_.inhibit(request, std::chrono::steady_clock::now());
    publish_serialized(make_zero_command());
    response->motion_inhibited = true;
    response->zero_published = true;
  }

private:
  std::mutex publication_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(std::make_shared<MotionGateNode>());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
"""

MISSION_PACKAGE = """\
<?xml version="1.0"?>
<package format="3">
  <name>voice_nav_mission</name>
  <version>0.1.0</version>
  <description>MotionGate contract fixture</description>
  <maintainer email="test@example.com">Test Maintainer</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>geometry_msgs</depend>
  <depend>rclcpp</depend>
  <depend>rmw</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>launch_testing</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
"""

MISSION_CMAKE = """\
cmake_minimum_required(VERSION 3.8)
project(voice_nav_mission)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InternalMotionGateState.msg"
  "srv/InternalMotionGateControl.srv"
)

add_library(motion_gate_core src/motion_gate_core.cpp)
add_executable(motion_gate_node src/motion_gate_node.cpp)
rosidl_get_typesupport_target(
  motion_gate_typesupport
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)
target_link_libraries(motion_gate_node motion_gate_core "${motion_gate_typesupport}")

if(BUILD_TESTING)
  ament_add_gtest(motion_gate_core_test test/motion_gate_core_test.cpp)
endif()

install(
  TARGETS motion_gate_core motion_gate_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
"""

GATE_CONFIG = """\
motion_gate_node:
  ros__parameters:
    use_sim_time: true
    output_frequency_hz: 50.0
    authority_lease_ms: 250
    candidate_freshness_ms: 150
    prepare_timeout_ms: 1000
    writer_graph_timeout_ms: 1000
    candidate_qos_depth: 1
    expected_candidate_writer_fqn: /collision_monitor
    request_cache_size: 64
    linear_x_min: -0.20
    linear_x_max: 0.40
    angular_z_min: -1.20
    angular_z_max: 1.20
"""

PRODUCT_LAUNCH = """\
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('voice_nav_sim'),
                'launch',
                'simulation.launch.py',
            ])
        )
    )
    gate_config = PathJoinSubstitution([
        FindPackageShare('voice_nav_bringup'),
        'config',
        'motion_gate.yaml',
    ])
    motion_gate = Node(
        package='voice_nav_mission',
        executable='motion_gate_node',
        name='motion_gate_node',
        parameters=[gate_config],
    )
    return LaunchDescription([simulation, motion_gate])
"""

BRINGUP_PACKAGE = """\
<?xml version="1.0"?>
<package format="3">
  <name>voice_nav_bringup</name>
  <version>0.1.0</version>
  <description>MotionGate composition fixture</description>
  <maintainer email="test@example.com">Test Maintainer</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>voice_nav_mission</exec_depend>
  <exec_depend>voice_nav_sim</exec_depend>
</package>
"""

BRINGUP_CMAKE = """\
cmake_minimum_required(VERSION 3.8)
project(voice_nav_bringup)

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
"""

CONTROLLERS = """\
controller_manager:
  ros__parameters:
    update_rate: 100

diff_drive_controller:
  ros__parameters:
    cmd_vel_timeout: 0.35
    linear.x.min_velocity: -0.20
    linear.x.max_velocity: 0.40
    angular.z.min_velocity: -1.20
    angular.z.max_velocity: 1.20
"""

FIXTURE_FILES = {
    "src/voice_nav_mission/srv/InternalMotionGateControl.srv": (
        CONTROL_INTERFACE
    ),
    "src/voice_nav_mission/msg/InternalMotionGateState.msg": (
        STATE_INTERFACE
    ),
    (
        "src/voice_nav_mission/include/voice_nav_mission/"
        "motion_gate_core.hpp"
    ): CORE_HEADER,
    "src/voice_nav_mission/src/motion_gate_core.cpp": CORE_SOURCE,
    "src/voice_nav_mission/src/motion_gate_node.cpp": NODE_SOURCE,
    "src/voice_nav_mission/package.xml": MISSION_PACKAGE,
    "src/voice_nav_mission/CMakeLists.txt": MISSION_CMAKE,
    "src/voice_nav_bringup/config/motion_gate.yaml": GATE_CONFIG,
    (
        "src/voice_nav_bringup/launch/product_sim.launch.py"
    ): PRODUCT_LAUNCH,
    "src/voice_nav_bringup/package.xml": BRINGUP_PACKAGE,
    "src/voice_nav_bringup/CMakeLists.txt": BRINGUP_CMAKE,
    "src/voice_nav_sim/config/controllers.yaml": CONTROLLERS,
}


class MotionGateContractTest(unittest.TestCase):
    def create_fixture(self, root: Path) -> None:
        for relative_path, contents in FIXTURE_FILES.items():
            path = root / relative_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                textwrap.dedent(contents),
                encoding="utf-8",
            )

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
                f"fixture mutation source not found in {relative_path}: {old}"
            )
        path.write_text(source.replace(old, new, 1), encoding="utf-8")

    def test_synthetic_valid_contract_passes(self) -> None:
        completed = self.run_checker()

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("MotionGate contract passed", completed.stdout)

    def test_repository_motion_gate_contract_passes(self) -> None:
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

    def test_state_diagnostic_gid_width_is_pinned_to_jazzy_rmw(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/msg/InternalMotionGateState.msg",
                "uint8[16] bound_writer_gid",
                "uint8[24] bound_writer_gid",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("uint8[16] bound_writer_gid", completed.stderr)

    def test_control_request_cannot_carry_cross_process_gid(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/srv/InternalMotionGateControl.srv",
                "uint64 expected_control_seq",
                (
                    "uint64 expected_control_seq\n"
                    "uint8[16] writer_gid"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must not carry a cross-process writer GID", completed.stderr)

    def test_internal_strings_must_be_bounded(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/srv/InternalMotionGateControl.srv",
                "string<=36 request_id",
                "string request_id",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("bounded contract", completed.stderr)

    def test_internal_idl_cannot_leak_into_public_package(self) -> None:
        def mutation(root: Path) -> None:
            leaked = (
                root
                / "src"
                / "voice_nav_interfaces"
                / "srv"
                / "InternalMotionGateControl.srv"
            )
            leaked.parent.mkdir(parents=True)
            leaked.write_text(CONTROL_INTERFACE, encoding="utf-8")

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must not be duplicated", completed.stderr)

    def test_exact_output_and_deadman_periods_are_enforced(self) -> None:
        mutations = (
            ("use_sim_time: true", "use_sim_time: false"),
            ("output_frequency_hz: 50.0", "output_frequency_hz: 20.0"),
            ("authority_lease_ms: 250", "authority_lease_ms: 350"),
            (
                "candidate_freshness_ms: 150",
                "candidate_freshness_ms: 250",
            ),
        )
        for old, new in mutations:
            with self.subTest(parameter=old.split(":", maxsplit=1)[0]):
                def mutation(
                    root: Path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(
                        root,
                        "src/voice_nav_bringup/config/motion_gate.yaml",
                        old_value,
                        new_value,
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn("MotionGate parameter", completed.stderr)

    def test_candidate_and_final_command_qos_are_explicit(self) -> None:
        mutations = (
            (
                ".best_effort().durability_volatile()",
                ".reliable().durability_volatile()",
                ".best_effort()",
            ),
            (
                "rclcpp::SystemDefaultsQoS()",
                "rclcpp::QoS(1)",
                "rclcpp::SystemDefaultsQoS()",
            ),
        )
        for old, new, diagnostic in mutations:
            with self.subTest(contract=diagnostic):
                def mutation(
                    root: Path,
                    old_value: str = old,
                    new_value: str = new,
                ) -> None:
                    self.replace(
                        root,
                        "src/voice_nav_mission/src/motion_gate_node.cpp",
                        old_value,
                        new_value,
                    )

                completed = self.run_checker(mutation)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(diagnostic, completed.stderr)

    def test_gate_limits_cannot_exceed_controller_limits(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_sim/config/controllers.yaml",
                "linear.x.max_velocity: 0.40",
                "linear.x.max_velocity: 0.30",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("wider than controller", completed.stderr)

    def test_same_package_rosidl_typesupport_link_is_required(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/CMakeLists.txt",
                "rosidl_get_typesupport_target(",
                "rosidl_target_interfaces(",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("rosidl_get_typesupport_target", completed.stderr)

    def test_core_deadlines_cannot_use_ros_time(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_core.cpp",
                '#include "voice_nav_mission/motion_gate_core.hpp"',
                (
                    '#include "voice_nav_mission/motion_gate_core.hpp"\n'
                    "// rclcpp::Clock must not drive core deadlines"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("injected steady time", completed.stderr)

    def test_prepare_requires_cas_and_conflicting_id_detection(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_core.cpp",
                "cached->second.request_fingerprint",
                "cached->second.unchecked_request_body",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("request_fingerprint", completed.stderr)

    def test_candidate_data_cannot_renew_authority(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_core.cpp",
                (
                    "candidate_deadline_ = now + candidate_freshness_;\n"
                    "  return applied_result();\n"
                    "}\n\n"
                    "Command MotionGateCore::tick"
                ),
                (
                    "candidate_deadline_ = now + candidate_freshness_;\n"
                    "  authority_deadline_ = now + authority_lease_;\n"
                    "  return applied_result();\n"
                    "}\n\n"
                    "Command MotionGateCore::tick"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must never renew", completed.stderr)

    def test_open_must_recreate_reader_before_binding_and_arming(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_node.cpp",
                "candidate_subscription_.reset();",
                "// stale PREPARE queue remains",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("OPEN queue barrier", completed.stderr)

    def test_open_must_bind_one_writer_in_gate_graph_context(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_node.cpp",
                "discover_unique_writer_gid_on_topic",
                "trust_cross_process_writer_gid",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "discover_unique_writer_gid_on_topic",
            completed.stderr,
        )

    def test_final_publication_must_be_serialized(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_node.cpp",
                "std::scoped_lock lock(publication_mutex_);",
                "final_command_publisher_->publish(command);",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("serialized publication", completed.stderr)

    def test_inhibit_must_publish_zero_before_acknowledgement(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_mission/src/motion_gate_node.cpp",
                (
                    "publish_serialized(make_zero_command());\n"
                    "    response->motion_inhibited = true;\n"
                    "    response->zero_published = true;"
                ),
                (
                    "response->zero_published = true;\n"
                    "    publish_serialized(make_zero_command());"
                    "\n    response->motion_inhibited = true;"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("INHIBIT acknowledgement", completed.stderr)

    def test_product_launch_must_have_one_gate_and_no_mux(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                (
                    "src/voice_nav_bringup/launch/product_sim.launch.py"
                ),
                "return LaunchDescription([simulation, motion_gate])",
                (
                    "twist_mux = Node(package='twist_mux', "
                    "executable='twist_mux')\n"
                    "    return LaunchDescription([simulation, motion_gate, "
                    "twist_mux])"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("bypass", completed.stderr)

    def test_product_launch_must_not_hide_gate_death(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                "src/voice_nav_bringup/launch/product_sim.launch.py",
                "parameters=[gate_config],",
                (
                    "parameters=[gate_config],\n"
                    "        on_exit=Shutdown(reason='MotionGate exited.'),"
                ),
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("consumer deadman", completed.stderr)

    def test_no_second_production_source_may_name_final_endpoint(self) -> None:
        def mutation(root: Path) -> None:
            rogue = root / "src" / "rogue_control" / "src" / "rogue.cpp"
            rogue.parent.mkdir(parents=True)
            rogue.write_text(
                'const char * topic = "/diff_drive_controller/cmd_vel";\n',
                encoding="utf-8",
            )

        completed = self.run_checker(mutation)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("only motion_gate_node may name", completed.stderr)


if __name__ == "__main__":
    unittest.main()
