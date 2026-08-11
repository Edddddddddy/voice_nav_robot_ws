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
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


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
    nav2_share = FindPackageShare('nav2_bringup')
    rapid_nav2_params = RewrittenYaml(
        source_file=PathJoinSubstitution(
            [nav2_share, 'params', 'nav2_params.yaml']
        ),
        param_rewrites={
            'controller_server.ros__parameters.FollowPath.plugin':
                'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            'controller_server.ros__parameters.enable_stamped_cmd_vel': 'true',
            'controller_server.ros__parameters.FollowPath.desired_linear_vel': '0.25',
            'controller_server.ros__parameters.FollowPath.lookahead_dist': '0.5',
            'controller_server.ros__parameters.FollowPath.use_velocity_scaled_lookahead_dist':
                'true',
            'bt_navigator.ros__parameters.default_server_timeout': '2000',
        },
        convert_types=True,
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [nav2_share, 'launch', 'bringup_launch.py']
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
            'params_file': rapid_nav2_params,
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
        launch_arguments={
            'mode': 'navigation',
            'llm_enabled': LaunchConfiguration('llm_enabled'),
            'named_places_file': PathJoinSubstitution(
                [bringup_share, 'maps', 'house_demo_places.yaml']
            ),
        }.items(),
    )
    direct_cmd_vel = Node(
        package='voice_nav_agent',
        executable='rapid_cmd_vel_relay',
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='true', choices=['true', 'false'],
            description='Run Gazebo server-only when true.',
        ),
        DeclareLaunchArgument(
            'llm_enabled', default_value='true', choices=['true', 'false'],
        ),
        product,
        TimerAction(period=12.0, actions=[nav2]),
        direct_cmd_vel,
        TimerAction(period=14.0, actions=[initial_pose]),
        rapid_stack,
    ])
