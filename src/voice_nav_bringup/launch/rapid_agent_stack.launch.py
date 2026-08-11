# Copyright 2026 Edddddddddy
# Licensed under the Apache License, Version 2.0

"""Reusable rapid Agent, local model, voice, and Mission bridge stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from voice_nav_agent.rapid_map_package import load_places


def _runtime_node(context):
    """Create the public Runtime with mode-specific rapid child ports."""
    mode = LaunchConfiguration('mode').perform(context)
    places_file = LaunchConfiguration('named_places_file').perform(context)
    runtime_overrides = {
        'operating_mode': mode,
        'rapid_delegate_action': '/rapid/mission/execute',
    }
    if mode == 'navigation':
        runtime_overrides['named_place_ids'] = sorted(
            load_places(places_file)
        )
    return [Node(
        package='voice_nav_mission',
        executable='mission_runtime_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('voice_nav_bringup'),
                'config',
                'mission_runtime.yaml',
            ]),
            runtime_overrides,
        ],
    )]


def generate_launch_description():
    """Launch the shared rapid model, Agent, voice, and execution adapters."""
    mode = LaunchConfiguration('mode')
    llm_bundle = LaunchConfiguration('llm_bundle')
    llm_enabled = LaunchConfiguration('llm_enabled')
    voice_root = LaunchConfiguration('voice_deps_root')
    capture_fifo = LaunchConfiguration('capture_fifo')
    playback_fifo = LaunchConfiguration('playback_fifo')
    mission = Node(
        package='voice_nav_agent',
        executable='rapid_mission_bridge',
        output='screen',
        parameters=[{
            'mode': mode,
            'map_output_root': LaunchConfiguration('map_output_root'),
            'named_places_file': LaunchConfiguration('named_places_file'),
            'action_name': '/rapid/mission/execute',
            'state_topic': '/rapid/mission/state',
            'stop_service': '/rapid/mission/stop',
            'enforce_runtime_token': False,
            'use_sim_time': True,
        }],
    )
    agent = Node(
        package='voice_nav_agent', executable='agent_node', output='screen'
    )
    voice = Node(
        package='voice_nav_agent',
        executable='rapid_voice_node',
        output='screen',
        parameters=[{
            'piper_path': PathJoinSubstitution(
                [voice_root, 'bin', 'piper']
            ),
            'piper_model': PathJoinSubstitution(
                [voice_root, 'models', 'zh_CN-huayan-medium.onnx']
            ),
            'vosk_python': PathJoinSubstitution(
                [voice_root, 'bin', 'python']
            ),
            'vosk_model': PathJoinSubstitution(
                [voice_root, 'models', 'vosk-model-small-cn-0.22']
            ),
            'kws_model': PathJoinSubstitution([
                voice_root, 'models',
                'sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20',
            ]),
            'vad_model': PathJoinSubstitution(
                [voice_root, 'models', 'silero_vad.int8.onnx']
            ),
            'asr_model': PathJoinSubstitution([
                voice_root, 'models',
                'sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30',
            ]),
            'keywords_file': PathJoinSubstitution(
                [voice_root, 'models', 'rapid_keywords.txt']
            ),
            'pcm_fifo': capture_fifo,
            'playback_fifo': playback_fifo,
        }],
    )
    audio = Node(
        package='voice_nav_audio',
        executable='audio_engine_node',
        output='screen',
        parameters=[{
            'capture_fifo': capture_fifo,
            'playback_fifo': playback_fifo,
        }],
    )
    llm = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([llm_bundle, 'bin', 'llama-server']),
            '--model',
            PathJoinSubstitution(
                [llm_bundle, 'models', 'Qwen3-0.6B-Q8_0.gguf']
            ),
            '--host', '127.0.0.1',
            '--port', '8080',
            '--ctx-size', '2048',
            '--n-predict', '256',
            '--parallel', '1',
        ],
        output='screen',
        condition=IfCondition(llm_enabled),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='navigation',
            choices=['mapping', 'navigation'],
        ),
        DeclareLaunchArgument(
            'llm_enabled', default_value='true', choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'llm_bundle',
            default_value=EnvironmentVariable(
                'VOICE_NAV_LLM_BUNDLE',
                default_value=(
                    '/home/ubuntu/.local/share/voice-nav-llm/bundles/'
                    '8bd4d412079a162ecca022943900d47bb17e5c6f363bbe65a8e7c6909936fdfc'
                ),
            ),
        ),
        DeclareLaunchArgument(
            'voice_deps_root',
            default_value=EnvironmentVariable(
                'VOICE_NAV_VOICE_ROOT',
                default_value=(
                    '/mnt/c/Users/lcy/code/ros2/'
                    'voice_nav_robot_ws_rapid/.deps/voice'
                ),
            ),
        ),
        DeclareLaunchArgument(
            'map_output_root', default_value='/tmp/voice_nav_rapid_maps'
        ),
        DeclareLaunchArgument('named_places_file', default_value=''),
        DeclareLaunchArgument(
            'capture_fifo', default_value='/tmp/voice_nav_rapid_capture.pcm'
        ),
        DeclareLaunchArgument(
            'playback_fifo', default_value='/tmp/voice_nav_rapid_playback.pcm'
        ),
        llm,
        audio,
        mission,
        OpaqueFunction(function=_runtime_node),
        agent,
        voice,
    ])
