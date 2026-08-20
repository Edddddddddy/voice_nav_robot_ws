# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Navigation MVP composition using the saved map package."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from voice_nav_sim._scenario_spec import resolve_scenario


# Let Fast DDS discover every lifecycle service before the managers issue
# their first transition.  The app's shared readiness deadline still bounds
# the complete startup.
_LIFECYCLE_DISCOVERY_SETTLE_S = 5.0
_NAVIGATION_LIFECYCLE_SETTLE_S = 12.0


def generate_launch_description():
    """Start simulation, localization, Nav2, and the navigation Runtime."""
    scenario_spec = resolve_scenario('navigation')
    headless = LaunchConfiguration('headless')
    shutdown_on_gazebo_exit = LaunchConfiguration('shutdown_on_gazebo_exit')
    map_id = LaunchConfiguration('map_id')
    map_root = LaunchConfiguration('map_root')
    bringup_share = FindPackageShare('voice_nav_bringup')
    runtime_config = PathJoinSubstitution([
        bringup_share, 'config', 'mission_navigation.yaml'
    ])
    nav2_params = PathJoinSubstitution([
        bringup_share, 'config', 'nav2_navigation_mvp.yaml'
    ])
    map_file = PathJoinSubstitution([map_root, map_id, 'map.yaml'])

    product = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            bringup_share, 'launch', 'product_sim.launch.py'
        ])),
        launch_arguments={
            'scenario': 'navigation',
            'headless': headless,
            'shutdown_on_gazebo_exit': shutdown_on_gazebo_exit,
            'runtime_config': runtime_config,
            'runtime_start_delay': '30.0',
            'map_id': map_id,
        }.items(),
    )

    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params, {
            'use_sim_time': True,
            'yaml_filename': map_file,
        }],
        remappings=tf_remappings,
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name=scenario_spec.map_odom_owner.removeprefix('/'),
        output='screen',
        parameters=[nav2_params],
        remappings=tf_remappings,
    )
    localization_manager = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_util', 'lifecycle_bringup',
            'map_server', 'amcl',
        ],
        output='screen',
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=tf_remappings,
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
        remappings=tf_remappings,
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
        remappings=tf_remappings,
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
        remappings=tf_remappings,
    )
    navigation_manager = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_util', 'lifecycle_bringup',
            'controller_server', 'planner_server',
            'behavior_server', 'bt_navigator',
        ],
        output='screen',
    )
    navigation = GroupAction([
        SetRemap('/cmd_vel', '/voice_nav/nav2_cmd_vel'),
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
    ])
    localization_autostart = TimerAction(
        period=_LIFECYCLE_DISCOVERY_SETTLE_S,
        actions=[localization_manager],
    )
    navigation_autostart = TimerAction(
        period=_NAVIGATION_LIFECYCLE_SETTLE_S,
        actions=[navigation_manager],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='voice_nav_navigation_rviz',
        output='screen',
        condition=UnlessCondition(headless),
        arguments=['-d', PathJoinSubstitution([
            bringup_share, 'config', 'voice_nav_navigation.rviz'
        ])],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': 'true'},
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        DeclareLaunchArgument(
            'headless', default_value='true', choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'shutdown_on_gazebo_exit',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'map_id',
            default_value='voice_mvp',
            description='Map package ID published by the Mapping MVP.',
        ),
        DeclareLaunchArgument(
            'map_root',
            default_value=os.path.join(
                os.environ.get(
                    'XDG_DATA_HOME',
                    os.path.join(os.path.expanduser('~'), '.local', 'share'),
                ),
                'voice_nav',
                'maps',
            ),
            description=(
                'Trusted map package root; it is not supplied by Mission '
                'payloads.'
            ),
        ),
        product,
        map_server,
        amcl,
        navigation,
        localization_autostart,
        navigation_autostart,
        rviz,
    ])
