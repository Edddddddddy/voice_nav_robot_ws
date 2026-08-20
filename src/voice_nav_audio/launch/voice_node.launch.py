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

"""Start the installed continuous full-duplex voice root with the Agent."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _environment(name: str) -> str:
    return os.environ.get(name, '')


def generate_launch_description() -> LaunchDescription:
    """Run the continuous full-duplex voice_node profile."""
    input_profile = LaunchConfiguration('input_profile')
    chaowen_tts_root = LaunchConfiguration('chaowen_tts_root')
    keyword_model_root = LaunchConfiguration('keyword_model_root')
    silero_vad_model = LaunchConfiguration('silero_vad_model')
    sensevoice_model = LaunchConfiguration('sensevoice_model')
    sensevoice_tokens = LaunchConfiguration('sensevoice_tokens')
    include_agent = LaunchConfiguration('include_agent')

    voice_node = Node(
        package='voice_nav_audio',
        executable='voice_node',
        name='voice_node',
        output='screen',
        parameters=[{
            'input_profile': input_profile,
            'chaowen_tts_root': chaowen_tts_root,
            'keyword_model_root': keyword_model_root,
            'silero_vad_model': silero_vad_model,
            'sensevoice_model': sensevoice_model,
            'sensevoice_tokens': sensevoice_tokens,
        }],
    )
    agent_node = Node(
        package='voice_nav_agent',
        executable='agent_node',
        name='agent_node',
        output='screen',
        condition=IfCondition(include_agent),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'include_agent',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Include the Agent for the standalone voice_node entrypoint.'
            ),
        ),
        DeclareLaunchArgument(
            'input_profile',
            default_value='vad_auto',
            choices=['vad_auto'],
            description='Use the continuous full-duplex VAD/ASR session.',
        ),
        DeclareLaunchArgument(
            'chaowen_tts_root',
            default_value=_environment('VOICE_NAV_CHAOWEN_TTS_ROOT'),
        ),
        DeclareLaunchArgument(
            'keyword_model_root',
            default_value=_environment('VOICE_NAV_KWS_ROOT'),
        ),
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
        agent_node,
        voice_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=voice_node,
                on_exit=[Shutdown(reason='installed voice_node completed')],
            )
        ),
    ])
