# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Fast local Nav2 demo for the fixed house map.

This launch intentionally routes Nav2 directly to the simulated controller.
It is a rapid-development path only and does not claim MotionGate protection.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the fixed map Nav2 demo and shared rapid Agent stack."""
    headless = LaunchConfiguration('headless')
    bringup_share = FindPackageShare('voice_nav_bringup')
    product = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'product_sim.launch.py'])
        ),
        launch_arguments={
            'headless': headless,
            'shutdown_on_gazebo_exit': 'true',
            'world': 'voice_nav_house_world.sdf',
            'world_name': 'voice_nav_house_world',
            'runtime_enabled': 'false',
            'safety_chain_enabled': 'false',
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
            )
        ),
        launch_arguments={
            'map': PathJoinSubstitution(
                [bringup_share, 'maps', 'house_demo.yaml']
            ),
            'use_sim_time': 'true',
            'autostart': 'true',
            'slam': 'False',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )
    initial_pose = Node(
        package='voice_nav_agent', executable='rapid_initial_pose', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    rapid_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [bringup_share, 'launch', 'rapid_agent_stack.launch.py']
            )
        ),
        launch_arguments={'mode': 'navigation'}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='true', choices=['true', 'false'],
            description='Run Gazebo server-only when true.',
        ),
        product,
        GroupAction([
            SetRemap(src='cmd_vel', dst='/diff_drive_controller/cmd_vel'),
            nav2,
        ]),
        initial_pose,
        rapid_stack,
    ])
