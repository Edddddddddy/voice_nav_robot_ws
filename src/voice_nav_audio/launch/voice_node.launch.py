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

"""Start the installed real-ASR voice root with the existing Agent."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _environment(name: str) -> str:
    return os.environ.get(name, '')


def generate_launch_description() -> LaunchDescription:
    """Run one bounded SenseVoice WAV turn and stop when the root exits."""
    input_wav = LaunchConfiguration('input_wav')
    result_path = LaunchConfiguration('result_path')
    silero_vad_model = LaunchConfiguration('silero_vad_model')
    sensevoice_model = LaunchConfiguration('sensevoice_model')
    sensevoice_tokens = LaunchConfiguration('sensevoice_tokens')
    exact_head = LaunchConfiguration('exact_head')

    voice_node = Node(
        package='voice_nav_audio',
        executable='voice_node',
        name='voice_node',
        output='screen',
        parameters=[{
            'input_profile': 'sensevoice_wav',
            'input_wav': input_wav,
            'silero_vad_model': silero_vad_model,
            'sensevoice_model': sensevoice_model,
            'sensevoice_tokens': sensevoice_tokens,
            'result_path': result_path,
            'exact_head': exact_head,
        }],
    )
    agent_node = Node(
        package='voice_nav_agent',
        executable='agent_node',
        name='agent_node',
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('input_wav', default_value=_environment('VOICE_NAV_SENSEVOICE_WAV')),
        DeclareLaunchArgument(
            'silero_vad_model',
            default_value=_environment('VOICE_NAV_SENSEVOICE_VAD_MODEL'),
        ),
        DeclareLaunchArgument(
            'sensevoice_model',
            default_value=_environment('VOICE_NAV_SENSEVOICE_MODEL'),
        ),
        DeclareLaunchArgument(
            'sensevoice_tokens',
            default_value=_environment('VOICE_NAV_SENSEVOICE_TOKENS'),
        ),
        DeclareLaunchArgument('result_path', default_value='voice_node.json'),
        DeclareLaunchArgument('exact_head', default_value=_environment('VOICE_NAV_REAL_GATE_HEAD') or 'unknown'),
        agent_node,
        voice_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=voice_node,
                on_exit=[Shutdown(reason='installed voice_node completed')],
            )
        ),
    ])
