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
    world_name = LaunchConfiguration('world_name')
    laser_update_rate = LaunchConfiguration('laser_update_rate')
    shutdown_on_gazebo_exit = LaunchConfiguration(
        'shutdown_on_gazebo_exit'
    )
    runtime_config = LaunchConfiguration('runtime_config')
    map_id = LaunchConfiguration('map_id')
    trusted_named_places_file = LaunchConfiguration('trusted_named_places_file')
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
            'world_name': world_name,
            'laser_update_rate': laser_update_rate,
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
    mission_runtime = Node(
        package='voice_nav_mission',
        executable='mission_runtime_node',
        name='mission_runtime_node',
        output='screen',
        parameters=[runtime_config, {
            'map_id': map_id,
            'trusted_named_places_file': trusted_named_places_file,
        }],
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
                'world_name',
                default_value='voice_nav_test_world',
                description='Select one trusted VoiceNav simulation world.',
                choices=['voice_nav_test_world', 'voice_nav_house_world'],
            ),
            DeclareLaunchArgument(
                'laser_update_rate',
                default_value='10',
                description='Select one trusted LiDAR sampling profile.',
                choices=['10', '20'],
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
            DeclareLaunchArgument(
                'runtime_config',
                default_value=PathJoinSubstitution([
                    FindPackageShare('voice_nav_bringup'),
                    'config',
                    'mission_runtime.yaml',
                ]),
                description='Trusted Mission Runtime configuration.',
            ),
            DeclareLaunchArgument(
                'map_id',
                default_value='voice_mvp',
                description='Trusted map package ID used by Runtime.',
            ),
            DeclareLaunchArgument(
                'trusted_named_places_file',
                default_value=PathJoinSubstitution([
                    FindPackageShare('voice_nav_bringup'),
                    'config',
                    'named_places.yaml',
                ]),
                description='Trusted house named-place fixture.',
            ),
            simulation,
            motion_conditioning_container,
            motion_gate,
            mission_runtime,
        ]
    )
