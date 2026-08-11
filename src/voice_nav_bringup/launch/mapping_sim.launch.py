# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Fast local Mapping Mode for the fixed VoiceNav house simulation."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch house simulation, SLAM Toolbox, and the rapid Agent stack."""
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
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
            )
        ),
        launch_arguments={
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
            'use_sim_time': 'true',
            'slam_params_file': PathJoinSubstitution(
                [bringup_share, 'config', 'slam_mapping.yaml']
            ),
        }.items(),
    )
    rapid_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [bringup_share, 'launch', 'rapid_agent_stack.launch.py']
            )
        ),
        launch_arguments={'mode': 'mapping'}.items(),
    )
    return LaunchDescription([
        SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_fastrtps_cpp'),
        DeclareLaunchArgument(
            'headless', default_value='true', choices=['true', 'false'],
            description='Run Gazebo server-only when true.',
        ),
        product,
        slam,
        rapid_stack,
    ])
