import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def start_after_success(next_action, stage):
    def handle_exit(event, _context):
        if event.returncode == 0:
            return [] if next_action is None else [next_action]
        return [
            Shutdown(
                reason=(
                    f'{stage} failed with exit code {event.returncode}; '
                    'aborting simulation startup.'
                )
            )
        ]

    return handle_exit


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    shutdown_on_gazebo_exit = LaunchConfiguration(
        'shutdown_on_gazebo_exit'
    )
    package_share = FindPackageShare('voice_nav_sim')
    xacro_file = PathJoinSubstitution(
        [
            package_share,
            'urdf',
            'voice_nav_robot.urdf.xacro',
        ]
    )
    controllers_file = PathJoinSubstitution(
        [
            package_share,
            'config',
            'controllers.yaml',
        ]
    )
    bridge_file = PathJoinSubstitution(
        [
            package_share,
            'config',
            'bridge.yaml',
        ]
    )
    world_file = PathJoinSubstitution(
        [
            package_share,
            'worlds',
            'voice_nav_test_world.sdf',
        ]
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'),
                ' ',
                xacro_file,
                ' controllers_file:=',
                controllers_file,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
        on_exit=Shutdown(reason='Robot state publisher exited.'),
    )

    gazebo_environment = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join(
            filter(
                None,
                (
                    os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH'),
                    os.environ.get('LD_LIBRARY_PATH'),
                ),
            )
        ),
        'GZ_SIM_RESOURCE_PATH': os.environ.get(
            'GZ_SIM_RESOURCE_PATH',
            '',
        ),
    }
    gazebo_common_arguments = [
        FindExecutable(name='ruby'),
        FindExecutable(name='gz'),
        'sim',
        '-r',
        '-v',
        '2',
        world_file,
        '--force-version',
        '8',
    ]
    gazebo_server = ExecuteProcess(
        cmd=gazebo_common_arguments[:4]
        + ['-s', '--headless-rendering']
        + gazebo_common_arguments[4:],
        name='gazebo',
        output='screen',
        additional_env=gazebo_environment,
        condition=IfCondition(headless),
        sigterm_timeout='10',
        sigkill_timeout='5',
    )
    gazebo_with_gui = ExecuteProcess(
        cmd=gazebo_common_arguments,
        name='gazebo',
        output='screen',
        additional_env=gazebo_environment,
        condition=UnlessCondition(headless),
        sigterm_timeout='10',
        sigkill_timeout='5',
    )
    stop_after_gazebo_server_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_server,
            on_exit=[Shutdown(reason='Gazebo server exited.')],
        ),
        condition=IfCondition(shutdown_on_gazebo_exit),
    )
    stop_after_gazebo_gui_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_with_gui,
            on_exit=[Shutdown(reason='Gazebo exited.')],
        ),
        condition=IfCondition(shutdown_on_gazebo_exit),
    )

    simulation_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='simulation_bridge',
        output='screen',
        parameters=[{'config_file': bridge_file}],
        on_exit=Shutdown(reason='Simulation bridge exited.'),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_voice_nav_robot',
        output='screen',
        arguments=[
            '--world',
            'voice_nav_test_world',
            '--topic',
            'robot_description',
            '--name',
            'voice_nav_robot',
            '--z',
            '0.03',
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_state_broadcaster',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30',
            '--switch-timeout',
            '10',
            '--service-call-timeout',
            '10',
        ],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_diff_drive_controller',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30',
            '--switch-timeout',
            '10',
            '--service-call-timeout',
            '10',
            '--controller-ros-args',
            '--ros-args --remap ~/odom:=/odom',
        ],
    )

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=start_after_success(
                joint_state_broadcaster_spawner,
                'Robot spawn',
            ),
        )
    )
    start_diff_drive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=start_after_success(
                diff_drive_controller_spawner,
                'Joint-state broadcaster startup',
            ),
        )
    )
    stop_after_diff_drive_failure = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=start_after_success(
                None,
                'Differential-drive controller startup',
            ),
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'headless',
                default_value='true',
                description='Run Gazebo server-only when true.',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument(
                'shutdown_on_gazebo_exit',
                default_value='true',
                description=(
                    'Shut down the whole launch when Gazebo exits. Tests '
                    'disable this while joining Gazebo explicitly.'
                ),
                choices=['true', 'false'],
            ),
            robot_state_publisher,
            gazebo_server,
            gazebo_with_gui,
            stop_after_gazebo_server_exit,
            stop_after_gazebo_gui_exit,
            simulation_bridge,
            spawn_robot,
            start_joint_state_broadcaster,
            start_diff_drive_controller,
            stop_after_diff_drive_failure,
        ]
    )
