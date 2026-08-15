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

"""Installed, simulation-only entrypoint for one caller-provided Voice Turn."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def _scripted_demo_launch_module():
    launch_path = Path(__file__).with_name('scripted_voice_demo.launch.py')
    specification = importlib.util.spec_from_file_location(
        'voice_nav_scripted_voice_demo_launch', launch_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load the shared VoicePipeline demo seam')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def create_voice_nav_demo(**kwargs):
    """Expose the shared readiness, observation, and evidence launch seam."""
    return _scripted_demo_launch_module().create_voice_nav_demo(**kwargs)


def _stop_loopback(server, worker):
    """Stop the shared local provider fixture used by the observation seam."""
    _scripted_demo_launch_module()._stop_loopback(server, worker)


def generate_launch_description():
    """Launch the existing product graph with exactly one command input."""
    command_text = LaunchConfiguration('command_text')
    headless = LaunchConfiguration('headless')
    shutdown_on_gazebo_exit = LaunchConfiguration('shutdown_on_gazebo_exit')
    actions, _ = create_voice_nav_demo(
        command_text=command_text,
        headless=headless,
        shutdown_on_gazebo_exit=shutdown_on_gazebo_exit,
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'command_text',
            default_value='',
            description='One bounded final command to submit through VoicePipeline.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            choices=['true', 'false'],
            description='Run the simulation demo without the Gazebo GUI.',
        ),
        DeclareLaunchArgument(
            'shutdown_on_gazebo_exit',
            default_value='true',
            choices=['true', 'false'],
            description='Fail closed when the required Gazebo simulation exits.',
        ),
        *actions,
    ])
