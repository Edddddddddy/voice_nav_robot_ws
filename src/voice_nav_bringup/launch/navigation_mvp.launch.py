# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Thin Navigation MVP composition for the fixed study place."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start simulation, localization, Nav2, and the navigation Runtime."""
    headless = LaunchConfiguration('headless')
    shutdown_on_gazebo_exit = LaunchConfiguration('shutdown_on_gazebo_exit')
    bringup_share = FindPackageShare('voice_nav_bringup')
    nav2_share = FindPackageShare('nav2_bringup')
    runtime_config = PathJoinSubstitution([
        bringup_share, 'config', 'mission_navigation.yaml'
    ])
    nav2_params = PathJoinSubstitution([
        bringup_share, 'config', 'nav2_navigation_mvp.yaml'
    ])
    map_file = PathJoinSubstitution([
        bringup_share, 'config', 'voice_nav_study_map.yaml'
    ])

    product = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            bringup_share, 'launch', 'product_sim.launch.py'
        ])),
        launch_arguments={
            'headless': headless,
            'shutdown_on_gazebo_exit': shutdown_on_gazebo_exit,
            'world_name': 'voice_nav_house_world',
            'laser_update_rate': '20',
            'runtime_config': runtime_config,
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
        name='amcl',
        output='screen',
        parameters=[nav2_params],
        remappings=tf_remappings,
    )
    localization_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
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
    navigation_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
    )
    navigation = GroupAction([
        SetRemap('/cmd_vel', '/voice_nav/nav2_cmd_vel'),
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        navigation_manager,
    ])

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
        product,
        map_server,
        amcl,
        localization_manager,
        navigation,
    ])
