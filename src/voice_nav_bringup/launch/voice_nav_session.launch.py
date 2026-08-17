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

"""Persistent, simulation-only VoiceNav session entrypoint."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Select one product mode and keep one parameter-driven voice session."""
    mode = LaunchConfiguration('mode')
    headless = LaunchConfiguration('headless')
    shutdown_on_gazebo_exit = LaunchConfiguration('shutdown_on_gazebo_exit')
    common_arguments = {
        'headless': headless,
        'shutdown_on_gazebo_exit': shutdown_on_gazebo_exit,
    }

    def mode_condition(value):
        return IfCondition(EqualsSubstitution(mode, value))

    motion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('voice_nav_bringup'),
            'launch',
            'product_sim.launch.py',
        ])),
        launch_arguments=common_arguments.items(),
        condition=mode_condition('motion'),
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('voice_nav_bringup'),
            'launch',
            'mapping_mvp.launch.py',
        ])),
        launch_arguments=common_arguments.items(),
        condition=mode_condition('mapping'),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('voice_nav_bringup'),
            'launch',
            'navigation_mvp.launch.py',
        ])),
        launch_arguments=common_arguments.items(),
        condition=mode_condition('navigation'),
    )
    agent = Node(
        package='voice_nav_agent',
        executable='agent_node',
        name='agent_node',
        output='screen',
    )
    session = Node(
        package='voice_nav_audio',
        executable='scripted_voice_demo',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scenario': 'session',
            'command_text': '',
        }],
        arguments=[
            '--ros-args',
            '-r',
            'scripted_voice_demo_configuration:__node:=voice_nav_command_gateway',
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='motion',
            choices=['motion', 'mapping', 'navigation'],
            description='Select exactly one VoiceNav product mode.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            choices=['true', 'false'],
            description='Run the simulation session without the Gazebo GUI.',
        ),
        DeclareLaunchArgument(
            'shutdown_on_gazebo_exit',
            default_value='true',
            choices=['true', 'false'],
            description='Fail closed when the required Gazebo simulation exits.',
        ),
        motion,
        mapping,
        navigation,
        agent,
        session,
    ])
