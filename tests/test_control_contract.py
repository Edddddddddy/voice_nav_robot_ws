import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "scripts" / "check_control_contract.py"

VALID_ROBOT = """\
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="voice_nav_robot">
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-20.0</param>
        <param name="max">20.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-20.0</param>
        <param name="max">20.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <gazebo>
    <plugin filename="libgz_ros2_control-system.so"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(arg controllers_file)</parameters>
      <hold_joints>true</hold_joints>
    </plugin>
  </gazebo>
</robot>
"""

VALID_CONTROLLERS = """\
controller_manager:
  ros__parameters:
    use_sim_time: true
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    use_sim_time: true
    left_wheel_names: [left_wheel_joint]
    right_wheel_names: [right_wheel_joint]
    wheel_separation: 0.40
    wheel_radius: 0.035
    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0
    tf_frame_prefix_enable: false
    odom_frame_id: odom
    base_frame_id: base_footprint
    enable_odom_tf: true
    open_loop: false
    position_feedback: true
    publish_rate: 50.0
    cmd_vel_timeout: 0.35
    publish_limited_velocity: true
    linear.x.has_velocity_limits: true
    linear.x.max_velocity: 0.40
    linear.x.min_velocity: -0.20
    angular.z.has_velocity_limits: true
    angular.z.max_velocity: 1.20
    angular.z.min_velocity: -1.20
"""

VALID_PACKAGE = """\
<?xml version="1.0"?>
<package format="3">
  <name>voice_nav_sim</name>
  <version>0.1.0</version>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>diff_drive_controller</exec_depend>
  <exec_depend>gz_tools_vendor</exec_depend>
  <exec_depend>gz_ros2_control</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>ros2controlcli</exec_depend>
  <exec_depend>rosgraph_msgs</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ruby</exec_depend>
  <exec_depend>xacro</exec_depend>
  <test_depend>ament_index_python</test_depend>
  <test_depend>controller_manager_msgs</test_depend>
  <test_depend>geometry_msgs</test_depend>
  <test_depend>launch_testing</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>nav_msgs</test_depend>
  <test_depend>python3-pytest</test_depend>
  <test_depend>rclpy</test_depend>
  <test_depend>sensor_msgs</test_depend>
  <test_depend>tf2_msgs</test_depend>
</package>
"""

VALID_CMAKE = """\
install(
  DIRECTORY
    config
    launch
    urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
"""

VALID_LAUNCH = """\
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def start_after_success(next_action, stage):
    def handle_exit(event, _context):
        if event.returncode == 0:
            return [] if next_action is None else [next_action]
        return [Shutdown(reason=f'{stage} failed')]
    return handle_exit


def generate_launch_description():
    package_share = FindPackageShare('voice_nav_sim')
    PathJoinSubstitution([package_share, 'config', 'controllers.yaml'])
    gazebo = ExecuteProcess(
        cmd=[
            FindExecutable(name='ruby'),
            FindExecutable(name='gz'),
            'sim',
        ],
        on_exit=Shutdown(),
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        on_exit=Shutdown(),
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        on_exit=Shutdown(),
    )
    spawn_robot = Node(package='ros_gz_sim', executable='create')
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
    )
    first = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=start_after_success(
                joint_state_broadcaster_spawner,
                'Robot spawn',
            ),
        )
    )
    second = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=start_after_success(
                diff_drive_controller_spawner,
                'Joint-state broadcaster startup',
            ),
        )
    )
    third = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=start_after_success(
                None,
                'Differential-drive controller startup',
            ),
        )
    )
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        clock_bridge,
        first,
        second,
        third,
    ])
"""


class ControlContractTest(unittest.TestCase):
    def run_checker(
        self,
        robot: str = VALID_ROBOT,
        controllers: str = VALID_CONTROLLERS,
        package: str = VALID_PACKAGE,
        cmake: str = VALID_CMAKE,
        launch: str = VALID_LAUNCH,
    ) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            files = {
                "robot.xacro": robot,
                "controllers.yaml": controllers,
                "package.xml": package,
                "CMakeLists.txt": cmake,
                "simulation.launch.py": launch,
            }
            for filename, contents in files.items():
                (root / filename).write_text(
                    textwrap.dedent(contents),
                    encoding="utf-8",
                )
            return subprocess.run(
                [
                    sys.executable,
                    str(CHECKER),
                    "--robot-description",
                    str(root / "robot.xacro"),
                    "--controllers",
                    str(root / "controllers.yaml"),
                    "--package",
                    str(root / "package.xml"),
                    "--cmake",
                    str(root / "CMakeLists.txt"),
                    "--launch",
                    str(root / "simulation.launch.py"),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

    def test_valid_contract_passes(self) -> None:
        completed = self.run_checker()

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("Control contract passed", completed.stdout)

    def test_repository_control_contract_passes(self) -> None:
        completed = subprocess.run(
            [
                sys.executable,
                str(CHECKER),
                "--robot-description",
                str(
                    REPOSITORY_ROOT
                    / "src"
                    / "voice_nav_sim"
                    / "urdf"
                    / "voice_nav_robot.urdf.xacro"
                ),
                "--controllers",
                str(
                    REPOSITORY_ROOT
                    / "src"
                    / "voice_nav_sim"
                    / "config"
                    / "controllers.yaml"
                ),
                "--package",
                str(
                    REPOSITORY_ROOT
                    / "src"
                    / "voice_nav_sim"
                    / "package.xml"
                ),
                "--cmake",
                str(
                    REPOSITORY_ROOT
                    / "src"
                    / "voice_nav_sim"
                    / "CMakeLists.txt"
                ),
                "--launch",
                str(
                    REPOSITORY_ROOT
                    / "src"
                    / "voice_nav_sim"
                    / "launch"
                    / "simulation.launch.py"
                ),
            ],
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_native_diff_drive_plugin_is_rejected(self) -> None:
        robot = VALID_ROBOT.replace(
            "</robot>",
            '<gazebo><plugin filename="gz-sim-diff-drive-system" '
            'name="gz::sim::systems::DiffDrive"/></gazebo>\n</robot>',
        )

        completed = self.run_checker(robot=robot)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("native Gazebo DiffDrive", completed.stderr)

    def test_missing_wheel_is_rejected(self) -> None:
        start = VALID_ROBOT.index('    <joint name="right_wheel_joint">')
        end = VALID_ROBOT.index("    </joint>", start) + len("    </joint>\n")
        robot = VALID_ROBOT[:start] + VALID_ROBOT[end:]

        completed = self.run_checker(robot=robot)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("ros2_control joints must be exactly", completed.stderr)

    def test_wrong_joint_interface_is_rejected(self) -> None:
        robot = VALID_ROBOT.replace(
            '<command_interface name="velocity">',
            '<command_interface name="effort">',
            1,
        )

        completed = self.run_checker(robot=robot)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("command interface must be velocity", completed.stderr)

    def test_wrong_timeout_is_rejected(self) -> None:
        controllers = VALID_CONTROLLERS.replace(
            "cmd_vel_timeout: 0.35",
            "cmd_vel_timeout: 3.5",
        )

        completed = self.run_checker(controllers=controllers)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("cmd_vel_timeout", completed.stderr)

    def test_wrong_base_frame_is_rejected(self) -> None:
        controllers = VALID_CONTROLLERS.replace(
            "base_frame_id: base_footprint",
            "base_frame_id: base_link",
        )

        completed = self.run_checker(controllers=controllers)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("base_frame_id", completed.stderr)

    def test_wrong_wheel_separation_is_rejected(self) -> None:
        controllers = VALID_CONTROLLERS.replace(
            "wheel_separation: 0.40",
            "wheel_separation: 0.20",
        )

        completed = self.run_checker(controllers=controllers)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("wheel_separation", completed.stderr)

    def test_missing_direct_runtime_dependency_is_rejected(self) -> None:
        package = VALID_PACKAGE.replace(
            "  <exec_depend>gz_tools_vendor</exec_depend>\n",
            "",
        )

        completed = self.run_checker(package=package)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("runtime dependencies", completed.stderr)
        self.assertIn("gz_tools_vendor", completed.stderr)

    def test_missing_direct_test_dependency_is_rejected(self) -> None:
        package = VALID_PACKAGE.replace(
            "  <test_depend>ament_index_python</test_depend>\n",
            "",
        )

        completed = self.run_checker(package=package)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("test dependencies", completed.stderr)
        self.assertIn("ament_index_python", completed.stderr)

    def test_fake_stamped_velocity_switch_is_rejected(self) -> None:
        controllers = VALID_CONTROLLERS + (
            "    enable_stamped_cmd_vel: true\n"
        )

        completed = self.run_checker(controllers=controllers)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("Jazzy uses TwistStamped intrinsically", completed.stderr)

    def test_controller_acceleration_limiter_is_rejected(self) -> None:
        controllers = VALID_CONTROLLERS + (
            "    linear.x.max_acceleration: 0.50\n"
        )

        completed = self.run_checker(controllers=controllers)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("would delay the consumer deadman zero", completed.stderr)

    def test_velocity_limit_disable_switch_is_rejected(self) -> None:
        controllers = VALID_CONTROLLERS.replace(
            "linear.x.has_velocity_limits: true",
            "linear.x.has_velocity_limits: false",
        )

        completed = self.run_checker(controllers=controllers)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("has_velocity_limits", completed.stderr)

    def test_shell_owned_gazebo_is_rejected(self) -> None:
        launch = VALID_LAUNCH.replace(
            "on_exit=Shutdown(),",
            "shell=True,\n        on_exit=Shutdown(),",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("shell execution must stay disabled", completed.stderr)

    def test_sleep_ordered_controller_startup_is_rejected(self) -> None:
        launch = VALID_LAUNCH.replace(
            "return LaunchDescription",
            "TimerAction(period=5.0, actions=[])\n    return LaunchDescription",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("process events, not TimerAction", completed.stderr)

    def test_multiple_bridge_processes_are_rejected(self) -> None:
        launch = VALID_LAUNCH.replace(
            "    clock_bridge = Node(",
            (
                "    duplicate_bridge = Node(\n"
                "        package='ros_gz_bridge',\n"
                "        executable='parameter_bridge',\n"
                "        on_exit=Shutdown(),\n"
                "    )\n"
                "    clock_bridge = Node("
            ),
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "exactly one ros_gz_bridge node",
            completed.stderr,
        )

    def test_unconditional_controller_startup_is_rejected(self) -> None:
        launch = VALID_LAUNCH.replace(
            (
                "on_exit=start_after_success(\n"
                "                joint_state_broadcaster_spawner,\n"
                "                'Robot spawn',\n"
                "            ),"
            ),
            "on_exit=[joint_state_broadcaster_spawner],",
        ).replace(
            (
                "on_exit=start_after_success(\n"
                "                diff_drive_controller_spawner,\n"
                "                'Joint-state broadcaster startup',\n"
                "            ),"
            ),
            "on_exit=[diff_drive_controller_spawner],",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "all three startup transitions must be return-code guarded",
            completed.stderr,
        )

if __name__ == "__main__":
    unittest.main()
