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

"""Package-private mode readiness state for the installed VoiceNav app."""

from __future__ import annotations


PRIMARY_STATE_ACTIVE = 3

_MODE_LIFECYCLE_NODES = {
    'mapping': ('slam_toolbox',),
    'navigation': (
        'map_server',
        'amcl',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ),
}


def _valid_map(message) -> bool:
    try:
        width = int(message.info.width)
        height = int(message.info.height)
        data = list(message.data)
        return (
            width > 0
            and height > 0
            and len(data) == width * height
            and any(value >= 0 for value in data)
        )
    except (AttributeError, TypeError, ValueError):
        return False


def _frame_name(value) -> str:
    return str(value).lstrip('/')


def _unique_map_odom(message) -> bool:
    try:
        transforms = message.transforms
        matches = [
            transform for transform in transforms
            if _frame_name(transform.header.frame_id) == 'map'
            and _frame_name(transform.child_frame_id) == 'odom'
        ]
    except AttributeError:
        return False
    except (TypeError, ValueError):
        return False
    return len(matches) == 1


class _ModeReadinessState:
    """Small callback state machine containing all mode evidence."""

    def __init__(
        self,
        mode: str,
        active_state_id: int = PRIMARY_STATE_ACTIVE,
    ):
        if mode not in _MODE_LIFECYCLE_NODES:
            raise ValueError(f'unsupported mode: {mode}')
        self.mode = mode
        self._active_state_id = active_state_id
        self._required_nodes = _MODE_LIFECYCLE_NODES[mode]
        self._active_nodes = set()
        self._map_seen = False
        self._map_odom_seen = False

    def observe_lifecycle(self, node_name: str, state_id: int) -> None:
        if (
            node_name in self._required_nodes
            and state_id == self._active_state_id
        ):
            self._active_nodes.add(node_name)

    def observe_map(self, message) -> None:
        self._map_seen = self._map_seen or _valid_map(message)

    def observe_tf(self, message) -> None:
        self._map_odom_seen = self._map_odom_seen or _unique_map_odom(message)

    def needs_lifecycle(self, node_name: str) -> bool:
        return node_name not in self._active_nodes

    def is_ready(self) -> bool:
        return (
            len(self._active_nodes) == len(self._required_nodes)
            and self._map_seen
            and self._map_odom_seen
        )

    def failure_stage(self) -> str:
        for node_name in self._required_nodes:
            if node_name not in self._active_nodes:
                return f'{node_name}_lifecycle'
        if not self._map_seen:
            return 'map'
        if not self._map_odom_seen:
            return 'map_odom_tf'
        return ''


_LIFECYCLE_SERVICES = {
    'mapping': {
        'slam_toolbox': '/slam_toolbox/get_state',
    },
    'navigation': {
        'map_server': '/map_server/get_state',
        'amcl': '/amcl/get_state',
        'controller_server': '/controller_server/get_state',
        'planner_server': '/planner_server/get_state',
        'behavior_server': '/behavior_server/get_state',
        'bt_navigator': '/bt_navigator/get_state',
    },
}
_MAP_TOPIC = '/map'
_TF_TOPIC = '/tf'


def _clock_now(clock) -> float:
    return clock() if callable(clock) else clock.monotonic()


def _ready_result() -> dict[str, str]:
    return {'status': 'ready', 'reason': ''}


def _unavailable_result(
    mode: str,
    stage: str,
    reason: str = 'mode_readiness_timeout',
) -> dict[str, str]:
    return {
        'status': 'unavailable',
        'reason': reason,
        'mode': mode,
        'stage': stage,
    }


def wait_for_mode_readiness(session_spec, deadline, clock):
    """Wait for typed lifecycle, map, and TF evidence for one mode."""
    mode = getattr(session_spec, 'mode', '')
    if mode == 'motion':
        return _ready_result()
    if mode not in _LIFECYCLE_SERVICES:
        return _unavailable_result(mode or 'unknown', 'setup')
    if _clock_now(clock) >= deadline:
        return _unavailable_result(mode, 'deadline')

    try:
        import rclpy
        from lifecycle_msgs.msg import State
        from lifecycle_msgs.srv import GetState
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from tf2_msgs.msg import TFMessage
    except Exception:
        return _unavailable_result(mode, 'setup', 'mode_readiness_failed')

    if not rclpy.ok():
        return _unavailable_result(mode, 'context', 'mode_readiness_failed')

    node = None
    state = _ModeReadinessState(mode, State.PRIMARY_STATE_ACTIVE)
    pending = {}
    try:
        node = rclpy.create_node('voice_nav_app_mode_readiness')

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        def on_map(message: OccupancyGrid) -> None:
            state.observe_map(message)

        def on_tf(message: TFMessage) -> None:
            state.observe_tf(message)

        node.create_subscription(
            OccupancyGrid, _MAP_TOPIC, on_map, map_qos,
        )
        node.create_subscription(TFMessage, _TF_TOPIC, on_tf, tf_qos)
        clients = {
            node_name: node.create_client(GetState, service_name)
            for node_name, service_name in _LIFECYCLE_SERVICES[mode].items()
        }

        while True:
            if state.is_ready():
                return _ready_result()
            remaining = deadline - _clock_now(clock)
            if remaining <= 0:
                return _unavailable_result(mode, state.failure_stage())

            for node_name, client in clients.items():
                if not state.needs_lifecycle(node_name):
                    continue
                if node_name in pending:
                    continue
                try:
                    if client.service_is_ready():
                        pending[node_name] = client.call_async(
                            GetState.Request()
                        )
                except Exception:
                    continue

            rclpy.spin_once(
                node, timeout_sec=min(0.1, max(0.0, remaining)),
            )
            for node_name, future in list(pending.items()):
                if not future.done():
                    continue
                try:
                    response = future.result()
                    state.observe_lifecycle(
                        node_name, response.current_state.id,
                    )
                except Exception:
                    pass
                del pending[node_name]
    except Exception:
        return _unavailable_result(mode, 'setup', 'mode_readiness_failed')
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
