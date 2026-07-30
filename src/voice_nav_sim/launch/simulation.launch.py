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
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare('voice_nav_sim'),
            'urdf',
            'voice_nav_robot.urdf.xacro',
        ]
    )
    controllers_file = PathJoinSubstitution(
        [
            FindPackageShare('voice_nav_sim'),
            'config',
            'controllers.yaml',
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
        'empty.sdf',
        '--force-version',
        '8',
    ]
    gazebo_server = ExecuteProcess(
        cmd=gazebo_common_arguments[:4]
        + ['-s']
        + gazebo_common_arguments[4:],
        name='gazebo',
        output='screen',
        additional_env=gazebo_environment,
        condition=IfCondition(headless),
        on_exit=Shutdown(reason='Gazebo server exited.'),
        sigterm_timeout='10',
        sigkill_timeout='5',
    )
    gazebo_with_gui = ExecuteProcess(
        cmd=gazebo_common_arguments,
        name='gazebo',
        output='screen',
        additional_env=gazebo_environment,
        condition=UnlessCondition(headless),
        on_exit=Shutdown(reason='Gazebo exited.'),
        sigterm_timeout='10',
        sigkill_timeout='5',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        on_exit=Shutdown(reason='Clock bridge exited.'),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_voice_nav_robot',
        output='screen',
        arguments=[
            '--world',
            'empty',
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
            robot_state_publisher,
            gazebo_server,
            gazebo_with_gui,
            clock_bridge,
            spawn_robot,
            start_joint_state_broadcaster,
            start_diff_drive_controller,
            stop_after_diff_drive_failure,
        ]
    )
