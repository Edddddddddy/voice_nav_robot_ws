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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Reuse the product graph and keep one parameter-driven voice session."""
    headless = LaunchConfiguration('headless')
    shutdown_on_gazebo_exit = LaunchConfiguration('shutdown_on_gazebo_exit')
    product = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('voice_nav_bringup'),
            'launch',
            'product_sim.launch.py',
        ])),
        launch_arguments={
            'headless': headless,
            'shutdown_on_gazebo_exit': shutdown_on_gazebo_exit,
        }.items(),
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
        product,
        agent,
        session,
    ])
