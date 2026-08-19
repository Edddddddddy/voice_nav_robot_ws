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

"""Package-private readiness helpers for the continuous voice frontend."""

from __future__ import annotations


def _clock_now(clock) -> float:
    return clock() if callable(clock) else clock.monotonic()


def _bounded_reason(error) -> str:
    return (
        str(error).replace('\r', ' ').replace('\n', ' ')[:160]
        if error else ''
    )


def build_vad_auto_command() -> tuple[str, ...]:
    """Build the only installed continuous VoiceNav input command."""
    return (
        'ros2', 'run', 'voice_nav_audio', 'voice_node',
        '--ros-args', '-p', 'input_profile:=vad_auto',
    )


def wait_for_input_sink_readiness(timeout_s: float, clock, stable_result):
    """Wait for the Agent subscription before starting continuous input."""
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
            node.destroy_node()
        if rclpy is not None and owns_context and rclpy.ok():
            rclpy.shutdown()
        return stable_result(
            'unavailable', f'readiness_start_failed:{_bounded_reason(error)}',
        )

    deadline = _clock_now(clock) + timeout_s
    try:
        while True:
            subscribers = node.get_subscriptions_info_by_topic('/voice/turn')
            if any(
                endpoint.node_name == 'agent_node'
                and endpoint.node_namespace in ('', '/')
                for endpoint in subscribers
            ):
                return stable_result('ready')
            remaining = deadline - _clock_now(clock)
            if remaining <= 0:
                return stable_result(
                    'unavailable', 'continuous_input_sink_timeout',
                )
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, remaining)))
    except Exception as error:
        return stable_result(
            'unavailable', f'readiness_wait_failed:{_bounded_reason(error)}',
        )
    finally:
        node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()


def wait_for_frontend_readiness(
    process, timeout_s: float, clock, poll, stable_result,
):
    """Wait for the continuous frontend publisher to be observable twice."""
    if timeout_s <= 0:
        return stable_result(
            'unavailable', 'invalid_frontend_readiness_timeout',
        )

    rclpy = None
    node = None
    owns_context = False
    try:
        import rclpy

        owns_context = not rclpy.ok()
        if owns_context:
            rclpy.init(args=None)
        node = rclpy.create_node('voice_nav_app_frontend_readiness')
    except Exception as error:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and owns_context and rclpy.ok():
            rclpy.shutdown()
        return stable_result(
            'unavailable',
            f'frontend_readiness_start_failed:{_bounded_reason(error)}',
        )

    deadline = _clock_now(clock) + timeout_s
    observed = False
    try:
        while True:
            if poll(process) is not None:
                return stable_result(
                    'unavailable', 'continuous_frontend_exited_before_ready',
                )
            publishers = node.get_publishers_info_by_topic('/voice/turn')
            ready = _has_voice_frontend_publisher(publishers)
            if ready and observed:
                return stable_result('ready')
            observed = ready
            remaining = deadline - _clock_now(clock)
            if remaining <= 0:
                return stable_result(
                    'unavailable', 'continuous_frontend_readiness_timeout',
                )
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, remaining)))
    except Exception as error:
        return stable_result(
            'unavailable',
            f'frontend_readiness_wait_failed:{_bounded_reason(error)}',
        )
    finally:
        node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()


def _has_voice_frontend_publisher(publishers) -> bool:
    """Match the continuous SpeechInputNode graph identity exactly."""
    return any(
        endpoint.node_name == 'voice_speech_input'
        and endpoint.node_namespace in ('', '/')
        for endpoint in publishers
    )
