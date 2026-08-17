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

"""Package-private staged SenseVoice behavior for the installed app."""

from __future__ import annotations

import os
import subprocess
import wave


_MAX_ERROR_REASON_LENGTH = 160
_INPUT_SAMPLE_RATE_HZ = 16000
_INPUT_CHANNELS = 1
_INPUT_SAMPLE_WIDTH_BYTES = 2
# Keep the frontend bound aligned with SenseVoiceProviderConfig's existing
# 15-second utterance queue.
_MAX_INPUT_SECONDS = 15
_MAX_INPUT_FRAMES = _INPUT_SAMPLE_RATE_HZ * _MAX_INPUT_SECONDS
_MAX_INPUT_WAV_BYTES = 44 + (
    _MAX_INPUT_FRAMES * _INPUT_CHANNELS * _INPUT_SAMPLE_WIDTH_BYTES
)


def _clock_now(clock) -> float:
    """Read the injected monotonic clock."""
    return clock() if callable(clock) else clock.monotonic()


def _bounded_reason(error) -> str:
    """Keep provider diagnostics short and single-line for stable JSON."""
    if not error:
        return ''
    return str(error).replace('\r', ' ').replace('\n', ' ')[
        :_MAX_ERROR_REASON_LENGTH
    ]


def validate_input_wav(input_wav: str) -> dict[str, str]:
    """Validate the product's supported, bounded PCM WAV input."""
    try:
        if not os.path.isabs(input_wav) or not os.path.isfile(input_wav):
            return {
                'status': 'unavailable',
                'reason': 'input_wav_must_be_absolute_regular_file',
            }
        file_size = os.path.getsize(input_wav)
        if file_size == 0:
            return {'status': 'unavailable', 'reason': 'input_wav_empty'}
        if file_size > _MAX_INPUT_WAV_BYTES:
            return {'status': 'unavailable', 'reason': 'input_wav_too_large'}
        with wave.open(input_wav, 'rb') as stream:
            frame_count = stream.getnframes()
            if frame_count == 0:
                return {'status': 'unavailable', 'reason': 'input_wav_empty'}
            if frame_count > _MAX_INPUT_FRAMES:
                return {'status': 'unavailable', 'reason': 'input_wav_too_large'}
            if (
                stream.getframerate() != _INPUT_SAMPLE_RATE_HZ
                or stream.getnchannels() != _INPUT_CHANNELS
                or stream.getsampwidth() != _INPUT_SAMPLE_WIDTH_BYTES
                or stream.getcomptype() != 'NONE'
            ):
                return {
                    'status': 'unavailable',
                    'reason': 'input_wav_unsupported_format',
                }
            if len(stream.readframes(frame_count)) != (
                frame_count * _INPUT_CHANNELS * _INPUT_SAMPLE_WIDTH_BYTES
            ):
                return {
                    'status': 'unavailable',
                    'reason': 'input_wav_unsupported_format',
                }
    except (OSError, EOFError, wave.Error):
        return {
            'status': 'unavailable',
            'reason': 'input_wav_unsupported_format',
        }
    return {'status': 'ready', 'reason': ''}


def build_frontend_command(
    input_wav: str,
    output_wav: str | None = None,
    chaowen_tts_root: str | None = None,
) -> tuple[str, ...]:
    """Build the frontend command with only trusted path parameters."""
    command = (
        'ros2',
        'launch',
        'voice_nav_audio',
        'voice_node.launch.py',
        'input_profile:=sensevoice_wav',
        f'input_wav:={input_wav}',
    )
    if output_wav is not None:
        command += (f'output_wav:={output_wav}',)
        if chaowen_tts_root is not None:
            command += (f'chaowen_tts_root:={chaowen_tts_root}',)
    return command + ('include_agent:=false',)


def wait_for_input_sink_readiness(
    timeout_s: float,
    clock,
    stable_result,
) -> dict[str, str]:
    """Wait for the long-lived Agent subscriber before provider spawn."""
    if timeout_s <= 0:
        return stable_result('unavailable', 'invalid_readiness_timeout')

    rclpy = None
    node = None
    owns_context = False
    try:
        import rclpy

        owns_context = not rclpy.ok()
        if owns_context:
            rclpy.init(args=None)
        node = rclpy.create_node('voice_nav_app_input_sink_readiness')
    except Exception as error:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy is not None and owns_context and rclpy.ok():
            rclpy.shutdown()
        return stable_result(
            'unavailable',
            f'readiness_start_failed:{_bounded_reason(error)}',
        )

    deadline = _clock_now(clock) + timeout_s
    try:
        while True:
            subscribers = node.get_subscriptions_info_by_topic('/voice/turn')
            agent_ready = any(
                endpoint.node_name == 'agent_node'
                and endpoint.node_namespace in ('', '/')
                for endpoint in subscribers
            )
            if agent_ready:
                return stable_result('ready')
            remaining = deadline - _clock_now(clock)
            if remaining <= 0:
                return stable_result(
                    'unavailable', 'sensevoice_wav_input_sink_timeout',
                )
            rclpy.spin_once(
                node, timeout_sec=min(0.1, max(0.0, remaining)),
            )
    except Exception as error:
        return stable_result(
            'unavailable',
            f'readiness_wait_failed:{_bounded_reason(error)}',
        )
    finally:
        try:
            node.destroy_node()
        finally:
            if owns_context and rclpy.ok():
                rclpy.shutdown()


def wait_for_completion(process, deadline, clock, poll) -> tuple[int, str]:
    """Wait for the provider and return an exit code plus stable reason."""
    remaining = max(0.0, deadline - _clock_now(clock))
    if remaining <= 0:
        return 1, 'sensevoice_wav_completion_timeout'
    try:
        process.wait(timeout=remaining)
    except (subprocess.TimeoutExpired, TimeoutError):
        if poll(process) is None:
            return 1, 'sensevoice_wav_completion_timeout'
    except Exception as error:
        return 1, (
            'sensevoice_wav_completion_failed:'
            f'{_bounded_reason(error)}'
        )

    returncode = poll(process)
    if returncode == 0:
        return 0, ''
    return 1, f'sensevoice_wav_failed:{returncode}'
