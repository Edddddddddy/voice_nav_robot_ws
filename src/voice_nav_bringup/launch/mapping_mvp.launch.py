# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Mapping MVP composition with optional live RViz visualization."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import UnlessCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from voice_nav_sim._scenario_spec import resolve_scenario


def generate_launch_description():
    """Start the fixed world, product Runtime, and one active SLAM owner."""
    scenario_spec = resolve_scenario('mapping')
    headless = LaunchConfiguration('headless')
    shutdown_on_gazebo_exit = LaunchConfiguration('shutdown_on_gazebo_exit')
    map_id = LaunchConfiguration('map_id')
    product = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('voice_nav_bringup'),
                'launch',
                'product_sim.launch.py',
            ])
        ),
        launch_arguments={
            'scenario': 'mapping',
            'headless': headless,
            'shutdown_on_gazebo_exit': shutdown_on_gazebo_exit,
            'map_id': map_id,
        }.items(),
    )
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='',
        name=scenario_spec.map_odom_owner.removeprefix('/'),
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('voice_nav_bringup'),
                'config',
                'slam_toolbox_mapping.yaml',
            ]),
            {'use_lifecycle_manager': False, 'use_sim_time': True},
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='voice_nav_mapping_rviz',
        output='screen',
        condition=UnlessCondition(headless),
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('voice_nav_bringup'),
            'config',
            'voice_nav_mapping.rviz',
        ])],
        parameters=[{'use_sim_time': True}],
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[MappingMvp] slam_toolbox is activating.'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            choices=['true', 'false'],
            description='Run the Mapping MVP without the Gazebo GUI.',
        ),
        DeclareLaunchArgument(
            'shutdown_on_gazebo_exit',
            default_value='true',
            choices=['true', 'false'],
            description='Shut down the Mapping MVP when Gazebo exits.',
        ),
        DeclareLaunchArgument(
            'map_id',
            default_value='voice_mvp',
            description='Map package ID used for the Mapping MVP.',
        ),
        product,
        slam,
        rviz,
        configure,
        activate,
    ])
