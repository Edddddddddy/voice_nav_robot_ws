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
    linear.x.max_velocity: 0.40
    linear.x.min_velocity: -0.20
    linear.x.max_acceleration: 0.50
    linear.x.max_deceleration: -0.50
    linear.x.max_acceleration_reverse: -0.50
    linear.x.max_deceleration_reverse: 0.50
    angular.z.max_velocity: 1.20
    angular.z.min_velocity: -1.20
    angular.z.max_acceleration: 1.50
    angular.z.max_deceleration: -1.50
    angular.z.max_acceleration_reverse: -1.50
    angular.z.max_deceleration_reverse: 1.50
"""

VALID_PACKAGE = """\
<?xml version="1.0"?>
<package format="3">
  <name>voice_nav_sim</name>
  <version>0.1.0</version>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>diff_drive_controller</exec_depend>
  <exec_depend>gz_ros2_control</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>xacro</exec_depend>
</package>
"""

VALID_CMAKE = """\
install(
  DIRECTORY
    config
    launch
    urdf
  DESTINATION share/${PROJECT_NAME}
)
"""


class ControlContractTest(unittest.TestCase):
    def run_checker(
        self,
        robot: str = VALID_ROBOT,
        controllers: str = VALID_CONTROLLERS,
        package: str = VALID_PACKAGE,
        cmake: str = VALID_CMAKE,
    ) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            files = {
                "robot.xacro": robot,
                "controllers.yaml": controllers,
                "package.xml": package,
                "CMakeLists.txt": cmake,
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

    def test_fake_stamped_velocity_switch_is_rejected(self) -> None:
        controllers = VALID_CONTROLLERS + (
            "    enable_stamped_cmd_vel: true\n"
        )

        completed = self.run_checker(controllers=controllers)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("Jazzy uses TwistStamped intrinsically", completed.stderr)


if __name__ == "__main__":
    unittest.main()
