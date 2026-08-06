# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    shutdown_on_gazebo_exit = LaunchConfiguration(
        'shutdown_on_gazebo_exit'
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('voice_nav_sim'),
                    'launch',
                    'simulation.launch.py',
                ]
            )
        ),
        launch_arguments={
            'headless': headless,
            'shutdown_on_gazebo_exit': shutdown_on_gazebo_exit,
        }.items(),
    )
    gate_config = PathJoinSubstitution(
        [
            FindPackageShare('voice_nav_bringup'),
            'config',
            'motion_gate.yaml',
        ]
    )
    motion_gate = Node(
        package='voice_nav_mission',
        executable='motion_gate_node',
        name='motion_gate_node',
        output='screen',
        parameters=[gate_config],
    )
    motion_conditioning_container = Node(
        package='rclcpp_components',
        executable='component_container_mt',
        name='motion_conditioning_container',
        output='screen',
    )
    runtime_config = PathJoinSubstitution(
        [
            FindPackageShare('voice_nav_bringup'),
            'config',
            'mission_runtime.yaml',
        ]
    )
    mission_runtime = Node(
        package='voice_nav_mission',
        executable='mission_runtime_node',
        name='mission_runtime_node',
        output='screen',
        parameters=[runtime_config],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name='RMW_IMPLEMENTATION',
                value='rmw_fastrtps_cpp',
            ),
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
                    'Shut down product bringup when Gazebo exits. Tests '
                    'disable this while joining Gazebo explicitly.'
                ),
                choices=['true', 'false'],
            ),
            simulation,
            motion_conditioning_container,
            motion_gate,
            mission_runtime,
        ]
    )
