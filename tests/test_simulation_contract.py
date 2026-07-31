import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "scripts" / "check_simulation_contract.py"

VALID_WORLD = """\
<?xml version="1.0"?>
<sdf version="1.10">
  <world name="voice_nav_test_world">
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>
    <model name="test_obstacle">
      <static>true</static>
      <pose>2.0 0 0.5 0 0 0</pose>
      <link name="obstacle_link">
        <collision name="obstacle_collision">
          <geometry>
            <box>
              <size>0.5 1.0 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="obstacle_visual">
          <geometry>
            <box>
              <size>0.5 1.0 1.0</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""

VALID_ROBOT = """\
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="voice_nav_robot">
  <link name="laser_link"/>
  <gazebo reference="laser_link">
    <sensor name="front_lidar" type="gpu_lidar">
      <topic>/scan</topic>
      <gz_frame_id>laser_link</gz_frame_id>
      <update_rate>10</update_rate>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.141592653589793</min_angle>
            <max_angle>3.141592653589793</max_angle>
          </horizontal>
          <vertical>
            <samples>1</samples>
            <resolution>1</resolution>
            <min_angle>0</min_angle>
            <max_angle>0</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.05</min>
          <max>8.0</max>
          <resolution>0.01</resolution>
        </range>
      </lidar>
    </sensor>
  </gazebo>
</robot>
"""

VALID_BRIDGE = """\
- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
  qos_profile: CLOCK
- ros_topic_name: /scan
  gz_topic_name: /scan
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS
  qos_profile: SENSOR_DATA
"""

VALID_PACKAGE = """\
<?xml version="1.0"?>
<package format="3">
  <name>voice_nav_sim</name>
  <version>0.1.0</version>
  <description>Simulation contract fixture</description>
  <maintainer email="test@example.com">Test Maintainer</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>gz_tools_vendor</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>rosgraph_msgs</exec_depend>
  <exec_depend>ruby</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>xacro</exec_depend>
</package>
"""

VALID_CMAKE = """\
install(
  DIRECTORY
    config
    launch
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
"""

VALID_LAUNCH = """\
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('voice_nav_sim')
    world_file = PathJoinSubstitution(
        [package_share, 'worlds', 'voice_nav_test_world.sdf']
    )
    bridge_file = PathJoinSubstitution(
        [package_share, 'config', 'bridge.yaml']
    )
    gazebo = ExecuteProcess(
        cmd=[
            FindExecutable(name='gz'),
            'sim',
            '-r',
            '-s',
            '--headless-rendering',
            world_file,
        ],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_file}],
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '--world',
            'voice_nav_test_world',
            '--topic',
            'robot_description',
        ],
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-ros-args',
            '--ros-args --remap ~/odom:=/odom',
        ],
    )
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        bridge,
        spawn_robot,
        diff_drive_controller_spawner,
    ])
"""


class SimulationContractTest(unittest.TestCase):
    def run_checker(
        self,
        *,
        launch: str = VALID_LAUNCH,
        world: str = VALID_WORLD,
        robot: str = VALID_ROBOT,
        bridge: str = VALID_BRIDGE,
        package: str = VALID_PACKAGE,
        cmake: str = VALID_CMAKE,
    ) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            paths = {
                "launch": root / "launch" / "simulation.launch.py",
                "world": root / "worlds" / "voice_nav_test_world.sdf",
                "robot": root / "urdf" / "voice_nav_robot.urdf.xacro",
                "bridge": root / "config" / "bridge.yaml",
                "package": root / "package.xml",
                "cmake": root / "CMakeLists.txt",
            }
            contents = {
                "launch": launch,
                "world": world,
                "robot": robot,
                "bridge": bridge,
                "package": package,
                "cmake": cmake,
            }
            for name, path in paths.items():
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(
                    textwrap.dedent(contents[name]),
                    encoding="utf-8",
                )
            return subprocess.run(
                [
                    sys.executable,
                    str(CHECKER),
                    "--launch",
                    str(paths["launch"]),
                    "--world",
                    str(paths["world"]),
                    "--robot-description",
                    str(paths["robot"]),
                    "--bridge",
                    str(paths["bridge"]),
                    "--package",
                    str(paths["package"]),
                    "--cmake",
                    str(paths["cmake"]),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

    def test_synthetic_valid_contract_passes(self) -> None:
        completed = self.run_checker()

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("Simulation contract passed", completed.stdout)

    def test_repository_simulation_contract_passes(self) -> None:
        simulation_package = (
            REPOSITORY_ROOT / "src" / "voice_nav_sim"
        )
        completed = subprocess.run(
            [
                sys.executable,
                str(CHECKER),
                "--launch",
                str(
                    simulation_package
                    / "launch"
                    / "simulation.launch.py"
                ),
                "--world",
                str(
                    simulation_package
                    / "worlds"
                    / "voice_nav_test_world.sdf"
                ),
                "--robot-description",
                str(
                    simulation_package
                    / "urdf"
                    / "voice_nav_robot.urdf.xacro"
                ),
                "--bridge",
                str(
                    simulation_package
                    / "config"
                    / "bridge.yaml"
                ),
                "--package",
                str(simulation_package / "package.xml"),
                "--cmake",
                str(simulation_package / "CMakeLists.txt"),
            ],
            check=False,
            capture_output=True,
            text=True,
        )

        if completed.returncode != 0:
            self.assertEqual(
                completed.stderr.splitlines()[0],
                (
                    "Simulation contract failed: simulation launch must load "
                    "the packaged non-empty test world; found built-in "
                    "empty.sdf"
                ),
            )
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_builtin_empty_world_is_rejected_first(self) -> None:
        launch = VALID_LAUNCH.replace(
            "world_file = PathJoinSubstitution(\n"
            "        [package_share, 'worlds', "
            "'voice_nav_test_world.sdf']\n"
            "    )",
            "world_file = 'empty.sdf'",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "packaged non-empty test world; found built-in empty.sdf",
            completed.stderr,
        )

    def test_unused_world_path_string_does_not_satisfy_launch(self) -> None:
        launch = VALID_LAUNCH.replace(
            "cmd=[FindExecutable(name='gz'), 'sim', '-r', world_file]",
            "cmd=[FindExecutable(name='gz'), 'sim', '-r', '/tmp/world.sdf']",
        ).replace(
            "world_file = PathJoinSubstitution(",
            "unused_world_file = PathJoinSubstitution(",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "through FindPackageShare",
            completed.stderr,
        )

    def test_headless_gazebo_requires_headless_rendering(self) -> None:
        launch = VALID_LAUNCH.replace(
            "            '--headless-rendering',\n",
            "",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "headless Gazebo server command must include "
            "--headless-rendering",
            completed.stderr,
        )

    def test_wrong_world_name_is_rejected(self) -> None:
        completed = self.run_checker(
            world=VALID_WORLD.replace(
                'world name="voice_nav_test_world"',
                'world name="another_world"',
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "world name must be voice_nav_test_world",
            completed.stderr,
        )

    def test_missing_direct_runtime_dependency_is_rejected(self) -> None:
        mutations = (
            (
                "  <exec_depend>sensor_msgs</exec_depend>\n",
                "  <test_depend>sensor_msgs</test_depend>\n",
                "sensor_msgs",
            ),
            (
                "  <exec_depend>ros_gz_bridge</exec_depend>\n",
                "",
                "ros_gz_bridge",
            ),
        )
        for old, new, dependency in mutations:
            with self.subTest(dependency=dependency):
                completed = self.run_checker(
                    package=VALID_PACKAGE.replace(old, new)
                )

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    "missing direct runtime dependencies",
                    completed.stderr,
                )
                self.assertIn(dependency, completed.stderr)

    def test_cmake_must_install_worlds(self) -> None:
        completed = self.run_checker(
            cmake=VALID_CMAKE.replace("    worlds\n", "")
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "CMake install is missing directories: worlds",
            completed.stderr,
        )

    def test_external_world_uri_is_rejected(self) -> None:
        for uri in (
            "https://fuel.gazebosim.org/1.0/openrobotics/models/Ground",
            "fuel://openrobotics/models/Ground",
            "model://ground_plane",
        ):
            with self.subTest(uri=uri):
                world = VALID_WORLD.replace(
                    '<model name="ground_plane">',
                    f"<include><uri>{uri}</uri></include>\n"
                    '    <model name="ground_plane">',
                )

                completed = self.run_checker(world=world)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    "must not reference external URI",
                    completed.stderr,
                )

    def test_local_world_resource_uri_is_rejected(self) -> None:
        for uri in (
            "/home/developer/cache/hidden.dae",
            "../meshes/hidden.dae",
            "meshes/hidden.dae",
        ):
            with self.subTest(uri=uri):
                world = VALID_WORLD.replace(
                    '<model name="ground_plane">',
                    f"<include><uri>{uri}</uri></include>\n"
                    '    <model name="ground_plane">',
                )

                completed = self.run_checker(world=world)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    "must not reference resource URI",
                    completed.stderr,
                )

    def test_inline_ground_collision_is_required(self) -> None:
        ground_start = VALID_WORLD.index(
            '    <model name="ground_plane">'
        )
        ground_end = (
            VALID_WORLD.index(
                "    </model>",
                ground_start,
            )
            + len("    </model>\n")
        )
        world = (
            VALID_WORLD[:ground_start]
            + VALID_WORLD[ground_end:]
        )

        completed = self.run_checker(world=world)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "must contain an inline static ground plane collision",
            completed.stderr,
        )

    def test_each_required_gazebo_world_system_is_enforced(self) -> None:
        systems = (
            (
                "gz::sim::systems::Physics",
                '<plugin filename="gz-sim-physics-system"\n'
                '            name="gz::sim::systems::Physics"/>\n',
            ),
            (
                "gz::sim::systems::UserCommands",
                '<plugin filename="gz-sim-user-commands-system"\n'
                '            name="gz::sim::systems::UserCommands"/>\n',
            ),
            (
                "gz::sim::systems::SceneBroadcaster",
                '<plugin filename="gz-sim-scene-broadcaster-system"\n'
                '            name="gz::sim::systems::SceneBroadcaster"/>\n',
            ),
            (
                "gz::sim::systems::Sensors",
                '<plugin filename="gz-sim-sensors-system"\n'
                '            name="gz::sim::systems::Sensors">\n'
                "      <render_engine>ogre2</render_engine>\n"
                "    </plugin>\n",
            ),
        )
        for system_name, element in systems:
            with self.subTest(system=system_name):
                completed = self.run_checker(
                    world=VALID_WORLD.replace("    " + element, "")
                )

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(system_name, completed.stderr)

    def test_wrong_gazebo_world_system_filename_is_rejected(self) -> None:
        completed = self.run_checker(
            world=VALID_WORLD.replace(
                "gz-sim-physics-system",
                "wrong-physics-system",
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "Physics filename must be gz-sim-physics-system",
            completed.stderr,
        )

    def test_sensors_system_must_use_ogre2(self) -> None:
        completed = self.run_checker(
            world=VALID_WORLD.replace(
                "<render_engine>ogre2</render_engine>",
                "<render_engine>ogre</render_engine>",
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("render_engine must be ogre2", completed.stderr)

    def test_test_obstacle_center_is_enforced(self) -> None:
        completed = self.run_checker(
            world=VALID_WORLD.replace(
                "<pose>2.0 0 0.5 0 0 0</pose>",
                "<pose>1.0 0 0.5 0 0 0</pose>",
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("centered at (2.0, 0, 0.5)", completed.stderr)

    def test_test_obstacle_collision_size_is_enforced(self) -> None:
        completed = self.run_checker(
            world=VALID_WORLD.replace(
                "<size>0.5 1.0 1.0</size>",
                "<size>0.4 1.0 1.0</size>",
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("size (0.5, 1.0, 1.0)", completed.stderr)

    def test_lidar_must_be_owned_by_laser_link(self) -> None:
        robot = VALID_ROBOT.replace(
            '<gazebo reference="laser_link">',
            '<gazebo reference="other_link">',
        ).replace(
            "</robot>",
            "<!-- laser_link gpu_lidar /scan -->\n</robot>",
        )

        completed = self.run_checker(robot=robot)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "laser_link must own exactly one Gazebo LiDAR sensor",
            completed.stderr,
        )

    def test_second_sensor_on_another_link_is_rejected(self) -> None:
        robot = VALID_ROBOT.replace(
            "</robot>",
            '  <gazebo reference="base_link">\n'
            '    <sensor name="rogue_lidar" type="gpu_lidar"/>\n'
            "  </gazebo>\n"
            "</robot>",
        )

        completed = self.run_checker(robot=robot)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "exactly one Gazebo sensor",
            completed.stderr,
        )

    def test_unexpanded_xacro_lidar_does_not_satisfy_contract(self) -> None:
        macro_robot = VALID_ROBOT.replace(
            '  <link name="laser_link"/>',
            '  <xacro:macro name="unused_lidar">\n'
            '    <link name="laser_link"/>',
        ).replace(
            "  </gazebo>\n</robot>",
            "  </gazebo>\n"
            "  </xacro:macro>\n"
            "</robot>",
        )
        conditional_robot = VALID_ROBOT.replace(
            '  <gazebo reference="laser_link">',
            '  <xacro:if value="${false}">\n'
            '    <gazebo reference="laser_link">',
        ).replace(
            "  </gazebo>\n</robot>",
            "  </gazebo>\n"
            "  </xacro:if>\n"
            "</robot>",
        )
        for construct, robot in (
            ("macro", macro_robot),
            ("conditional", conditional_robot),
        ):
            with self.subTest(construct=construct):
                completed = self.run_checker(robot=robot)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn("robot root must directly", completed.stderr)

    def test_lidar_sensor_type_is_enforced(self) -> None:
        completed = self.run_checker(
            robot=VALID_ROBOT.replace(
                'type="gpu_lidar"',
                'type="ray"',
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("sensor type must be gpu_lidar", completed.stderr)

    def test_lidar_topic_is_enforced(self) -> None:
        completed = self.run_checker(
            robot=VALID_ROBOT.replace(
                "<topic>/scan</topic>",
                "<topic>/model/robot/scan</topic>",
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("LiDAR topic must be /scan", completed.stderr)

    def test_lidar_frame_is_enforced(self) -> None:
        completed = self.run_checker(
            robot=VALID_ROBOT.replace(
                "<gz_frame_id>laser_link</gz_frame_id>",
                "<gz_frame_id>base_link</gz_frame_id>",
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "gz_frame_id must be laser_link",
            completed.stderr,
        )

    def test_lidar_update_rate_is_enforced(self) -> None:
        completed = self.run_checker(
            robot=VALID_ROBOT.replace(
                "<update_rate>10</update_rate>",
                "<update_rate>5</update_rate>",
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("update_rate must be 10.0", completed.stderr)

    def test_lidar_pose_must_be_link_local_zero(self) -> None:
        completed = self.run_checker(
            robot=VALID_ROBOT.replace(
                "<update_rate>10</update_rate>",
                "<update_rate>10</update_rate>\n"
                "      <pose>0.1 0 0 0 0 0</pose>",
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "LiDAR pose must be link-local zero",
            completed.stderr,
        )

    def test_lidar_horizontal_geometry_is_enforced(self) -> None:
        mutations = (
            ("<samples>360</samples>", "<samples>180</samples>", "samples"),
            (
                "<resolution>1</resolution>",
                "<resolution>0.5</resolution>",
                "resolution",
            ),
            (
                "<min_angle>-3.141592653589793</min_angle>",
                "<min_angle>-1.57</min_angle>",
                "min_angle",
            ),
            (
                "<max_angle>3.141592653589793</max_angle>",
                "<max_angle>1.57</max_angle>",
                "max_angle",
            ),
        )
        for old, new, field in mutations:
            with self.subTest(field=field):
                completed = self.run_checker(
                    robot=VALID_ROBOT.replace(old, new, 1)
                )

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    f"horizontal scan {field} must be",
                    completed.stderr,
                )

    def test_lidar_vertical_geometry_is_enforced(self) -> None:
        vertical = """\
          <vertical>
            <samples>1</samples>
            <resolution>1</resolution>
            <min_angle>0</min_angle>
            <max_angle>0</max_angle>
          </vertical>
"""
        mutations = (
            ("<samples>1</samples>", "<samples>2</samples>", "samples"),
            (
                "<resolution>1</resolution>",
                "<resolution>2</resolution>",
                "resolution",
            ),
            (
                "<min_angle>0</min_angle>",
                "<min_angle>-0.1</min_angle>",
                "min_angle",
            ),
            (
                "<max_angle>0</max_angle>",
                "<max_angle>0.1</max_angle>",
                "max_angle",
            ),
        )
        for old, new, field in mutations:
            with self.subTest(field=field):
                mutated_vertical = vertical.replace(old, new)
                completed = self.run_checker(
                    robot=VALID_ROBOT.replace(
                        vertical,
                        mutated_vertical,
                    )
                )

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    f"vertical scan {field} must be",
                    completed.stderr,
                )

    def test_lidar_range_geometry_is_enforced(self) -> None:
        mutations = (
            ("<min>0.05</min>", "<min>0.1</min>", "min"),
            ("<max>8.0</max>", "<max>4.0</max>", "max"),
            (
                "<resolution>0.01</resolution>",
                "<resolution>0.1</resolution>",
                "resolution",
            ),
        )
        for old, new, field in mutations:
            with self.subTest(field=field):
                completed = self.run_checker(
                    robot=VALID_ROBOT.replace(old, new)
                )

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    f"LiDAR range {field} must be",
                    completed.stderr,
                )

    def test_lidar_noise_is_rejected(self) -> None:
        robot = VALID_ROBOT.replace(
            "</range>",
            "</range>\n"
            "        <noise><type>gaussian</type></noise>",
        )

        completed = self.run_checker(robot=robot)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "must not configure synthetic noise",
            completed.stderr,
        )

    def test_bridge_config_must_be_loaded_from_package_share(self) -> None:
        launch = VALID_LAUNCH.replace(
            "parameters=[{'config_file': bridge_file}]",
            "parameters=[{'config_file': '/tmp/bridge.yaml'}]",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "config_file must resolve to "
            "voice_nav_sim/config/bridge.yaml",
            completed.stderr,
        )

    def test_bridge_node_rejects_parameter_overrides(self) -> None:
        mutations = (
            (
                "parameters=[{'config_file': bridge_file}]",
                "parameters=[{\n"
                "            'config_file': bridge_file,\n"
                "            'override_timestamps_with_wall_time': True,\n"
                "        }]",
            ),
            (
                "parameters=[{'config_file': bridge_file}]",
                "parameters=[\n"
                "            {'config_file': bridge_file},\n"
                "            {'use_sim_time': True},\n"
                "        ]",
            ),
        )
        for case, (old, new) in enumerate(mutations):
            with self.subTest(case=case):
                completed = self.run_checker(
                    launch=VALID_LAUNCH.replace(old, new)
                )

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    "parameters must be exactly one config_file mapping",
                    completed.stderr,
                )

    def test_unused_bridge_path_string_does_not_satisfy_launch(self) -> None:
        launch = VALID_LAUNCH.replace(
            "parameters=[{'config_file': bridge_file}]",
            "parameters=[{'config_file': '/tmp/bridge.yaml'}]",
        ).replace(
            "bridge_file = PathJoinSubstitution(",
            "unused_bridge_file = PathJoinSubstitution(",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "through FindPackageShare",
            completed.stderr,
        )

    def test_inline_bridge_arguments_are_rejected(self) -> None:
        launch = VALID_LAUNCH.replace(
            "parameters=[{'config_file': bridge_file}],",
            "parameters=[{'config_file': bridge_file}],\n"
            "        arguments=[\n"
            "            '/clock@rosgraph_msgs/msg/Clock"
            "[gz.msgs.Clock',\n"
            "        ],",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "topics must come only from config/bridge.yaml",
            completed.stderr,
        )

    def test_robot_spawn_targets_the_packaged_world(self) -> None:
        launch = VALID_LAUNCH.replace(
            "'voice_nav_test_world',\n"
            "            '--topic',",
            "'empty',\n"
            "            '--topic',",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "spawn --world must be voice_nav_test_world",
            completed.stderr,
        )

    def test_bridge_forbidden_topic_is_rejected(self) -> None:
        bridge = VALID_BRIDGE + """\
- ros_topic_name: /odom
  gz_topic_name: /model/voice_nav_robot/odometry
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS
  qos_profile: SENSOR_DATA
"""

        completed = self.run_checker(bridge=bridge)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "bridge allowlist must contain only /clock and /scan",
            completed.stderr,
        )
        self.assertIn("/odom", completed.stderr)

    def test_bridge_direction_is_gazebo_to_ros(self) -> None:
        for topic in ("/clock", "/scan"):
            with self.subTest(topic=topic):
                marker = (
                    f"- ros_topic_name: {topic}\n"
                    f"  gz_topic_name: {topic}\n"
                )
                mutated_marker = marker.replace(
                    "  gz_topic_name:",
                    "  direction: ROS_TO_GZ\n"
                    "  gz_topic_name:",
                )
                bridge = VALID_BRIDGE.replace(
                    marker,
                    mutated_marker,
                ).replace(
                    "  direction: GZ_TO_ROS\n",
                    "",
                    1 if topic == "/clock" else 2,
                )
                if topic == "/scan":
                    # The count-based replace above removes /clock first.
                    bridge = VALID_BRIDGE.replace(
                        marker,
                        mutated_marker,
                    )
                    scan_start = bridge.index(
                        "- ros_topic_name: /scan"
                    )
                    direction_start = bridge.index(
                        "  direction: GZ_TO_ROS\n",
                        scan_start,
                    )
                    bridge = (
                        bridge[:direction_start]
                        + bridge[
                            direction_start
                            + len("  direction: GZ_TO_ROS\n") :
                        ]
                    )

                completed = self.run_checker(bridge=bridge)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    f"{topic} bridge direction must be GZ_TO_ROS",
                    completed.stderr,
                )

    def test_bridge_types_are_exact(self) -> None:
        mutations = (
            (
                "rosgraph_msgs/msg/Clock",
                "builtin_interfaces/msg/Time",
                "/clock bridge ros_type_name",
            ),
            (
                "gz.msgs.Clock",
                "gz.msgs.Time",
                "/clock bridge gz_type_name",
            ),
            (
                "sensor_msgs/msg/LaserScan",
                "sensor_msgs/msg/PointCloud2",
                "/scan bridge ros_type_name",
            ),
            (
                "gz.msgs.LaserScan",
                "gz.msgs.PointCloudPacked",
                "/scan bridge gz_type_name",
            ),
        )
        for old, new, message in mutations:
            with self.subTest(type_name=old):
                completed = self.run_checker(
                    bridge=VALID_BRIDGE.replace(old, new)
                )

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(message, completed.stderr)

    def test_bridge_qos_profiles_are_exact(self) -> None:
        mutations = (
            ("qos_profile: CLOCK", "qos_profile: SENSOR_DATA", "/clock"),
            ("qos_profile: SENSOR_DATA", "qos_profile: CLOCK", "/scan"),
        )
        for old, new, topic in mutations:
            with self.subTest(topic=topic):
                completed = self.run_checker(
                    bridge=VALID_BRIDGE.replace(old, new)
                )

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    f"{topic} bridge qos_profile",
                    completed.stderr,
                )

    def test_controller_odom_remap_must_be_direct(self) -> None:
        launch = VALID_LAUNCH.replace(
            "            '--controller-ros-args',\n"
            "            '--ros-args --remap ~/odom:=/odom',\n",
            "",
        ).replace(
            "arguments=[\n"
            "            'diff_drive_controller',",
            "remappings=[('~/odom', '/odom')],\n"
            "        arguments=[\n"
            "            'diff_drive_controller',",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "must directly remap ~/odom to /odom",
            completed.stderr,
        )

    def test_unreturned_required_action_does_not_satisfy_launch(self) -> None:
        launch = VALID_LAUNCH.replace(
            "        bridge,\n",
            "",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "must start exactly one ros_gz_bridge parameter_bridge",
            completed.stderr,
        )

    def test_unreturned_forbidden_nodes_do_not_fail(self) -> None:
        launch = VALID_LAUNCH.replace(
            "    return LaunchDescription([",
            "    dead_relay = Node(\n"
            "        package='topic_tools',\n"
            "        executable='relay',\n"
            "    )\n"
            "    dead_tf = Node(\n"
            "        package='tf2_ros',\n"
            "        executable='static_transform_publisher',\n"
            "    )\n"
            "    return LaunchDescription([",
        )

        completed = self.run_checker(launch=launch)

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_unrelated_relay_executable_is_allowed(self) -> None:
        launch = VALID_LAUNCH.replace(
            "    return LaunchDescription([",
            "    application_node = Node(\n"
            "        package='example_tools',\n"
            "        executable='relay',\n"
            "    )\n"
            "    return LaunchDescription([\n"
            "        application_node,",
        )

        completed = self.run_checker(launch=launch)

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_nonpublishing_tf2_ros_node_is_allowed(self) -> None:
        launch = VALID_LAUNCH.replace(
            "    return LaunchDescription([",
            "    tf_diagnostic = Node(\n"
            "        package='tf2_ros',\n"
            "        executable='tf2_echo',\n"
            "    )\n"
            "    return LaunchDescription([\n"
            "        tf_diagnostic,",
        )

        completed = self.run_checker(launch=launch)

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_odometry_relay_is_rejected(self) -> None:
        launch = VALID_LAUNCH.replace(
            "    return LaunchDescription([",
            "    odom_relay = Node(\n"
            "        package='topic_tools',\n"
            "        executable='relay',\n"
            "        arguments=['/diff_drive_controller/odom', '/odom'],\n"
            "    )\n"
            "    return LaunchDescription([\n"
            "        odom_relay,",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "must not add an odometry topic relay",
            completed.stderr,
        )

    def test_extra_tf_publisher_is_rejected(self) -> None:
        launch = VALID_LAUNCH.replace(
            "    return LaunchDescription([",
            "    extra_tf = Node(\n"
            "        package='tf2_ros',\n"
            "        executable='static_transform_publisher',\n"
            "    )\n"
            "    return LaunchDescription([\n"
            "        extra_tf,",
        )

        completed = self.run_checker(launch=launch)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "must not add another TF publisher",
            completed.stderr,
        )


if __name__ == "__main__":
    unittest.main()
