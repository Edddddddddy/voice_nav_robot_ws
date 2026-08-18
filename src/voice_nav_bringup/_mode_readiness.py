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
RUNTIME_MODE_MAPPING = 1
RUNTIME_MODE_NAVIGATION = 2
RUNTIME_AVAILABLE = 1
GATE_INHIBITED = 0
MOTION_GATE_INHIBITED = 0
NO_ACTIVE_STEP = 0xFFFFFFFF
MOTION_SAFE_STATIONARY_HOLD_S = 2.0
_MOTION_ODOMETRY_FRESHNESS_S = 0.1

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
        require_runtime: bool = False,
    ):
        if mode not in _MODE_LIFECYCLE_NODES:
            raise ValueError(f'unsupported mode: {mode}')
        self.mode = mode
        self._active_state_id = active_state_id
        self._required_nodes = _MODE_LIFECYCLE_NODES[mode]
        self._runtime_ready = not require_runtime
        self._active_nodes = set()
        self._map_seen = False
        self._map_odom_seen = False

    def observe_lifecycle(self, node_name: str, state_id: int) -> None:
        if (
            node_name in self._required_nodes
            and state_id == self._active_state_id
        ):
            self._active_nodes.add(node_name)

    def observe_runtime(self, message) -> None:
        """Accept only the Runtime snapshot for this selected mode."""
        expected_mode = (
            RUNTIME_MODE_MAPPING
            if self.mode == 'mapping'
            else RUNTIME_MODE_NAVIGATION
        )
        self._runtime_ready = (
            int(message.operating_mode) == expected_mode
            and int(message.availability) == RUNTIME_AVAILABLE
            and int(message.gate_state) == GATE_INHIBITED
        )

    def observe_map(self, message) -> None:
        self._map_seen = self._map_seen or _valid_map(message)

    def observe_tf(self, message) -> None:
        self._map_odom_seen = self._map_odom_seen or _unique_map_odom(message)

    def needs_lifecycle(self, node_name: str) -> bool:
        return node_name not in self._active_nodes

    def is_ready(self) -> bool:
        return (
            self._runtime_ready
            and len(self._active_nodes) == len(self._required_nodes)
            and self._map_seen
            and self._map_odom_seen
        )

    def failure_stage(self) -> str:
        if not self._runtime_ready:
            return 'runtime_mode_snapshot'
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


def _is_stationary_odometry(message) -> bool:
    try:
        twist = message.twist.twist
        return (
            abs(twist.linear.x) <= 0.01
            and abs(twist.angular.z) <= 0.02
        )
    except (AttributeError, TypeError, ValueError):
        return False


class _MotionReadinessState:
    """Observe the typed Motion safe-stationary barrier without sleeping."""

    def __init__(self, clock, hold_s: float = MOTION_SAFE_STATIONARY_HOLD_S):
        if hold_s <= 0:
            raise ValueError('motion readiness hold must be positive')
        self._clock = clock
        self._hold_s = hold_s
        self._runtime_available = False
        self._runtime_gate_inhibited = False
        self._motion_gate_inhibited = False
        self._no_active_step = False
        self._controller_active = False
        self._controller_observed = False
        self._controller_zero = False
        self._odometry_received_at = None
        self._odometry_stationary = False
        self._safe_since = None

    def observe_runtime(self, message) -> None:
        self._runtime_available = message.availability == RUNTIME_AVAILABLE
        self._runtime_gate_inhibited = message.gate_state == GATE_INHIBITED
        self._no_active_step = message.active_step == NO_ACTIVE_STEP
        self._refresh_barrier()

    def observe_motion_gate(self, message) -> None:
        self._motion_gate_inhibited = (
            message.state == MOTION_GATE_INHIBITED
            and message.motion_inhibited
        )
        self._controller_observed = message.zero_publish_seq > 0
        self._controller_zero = (
            message.zero_selected and message.zero_publish_seq > 0
        )
        self._refresh_barrier()

    def observe_controller_active(self, active: bool) -> None:
        self._controller_active = bool(active)
        self._refresh_barrier()

    def observe_odometry(self, message) -> None:
        received_at = _clock_now(self._clock)
        if (
            self._odometry_received_at is not None
            and received_at - self._odometry_received_at
            > _MOTION_ODOMETRY_FRESHNESS_S
        ):
            self._safe_since = received_at
        self._odometry_received_at = received_at
        self._odometry_stationary = _is_stationary_odometry(message)
        self._refresh_barrier()

    def _odometry_is_fresh(self) -> bool:
        return (
            self._odometry_received_at is not None
            and _clock_now(self._clock) - self._odometry_received_at
            <= _MOTION_ODOMETRY_FRESHNESS_S
        )

    def _safe(self) -> bool:
        return (
            self._runtime_available
            and self._runtime_gate_inhibited
            and self._motion_gate_inhibited
            and self._no_active_step
            and self._controller_active
            and self._controller_observed
            and self._controller_zero
            and self._odometry_received_at is not None
            and self._odometry_stationary
            and self._odometry_is_fresh()
        )

    def _refresh_barrier(self) -> None:
        if not self._safe():
            self._safe_since = None
        elif self._safe_since is None:
            self._safe_since = _clock_now(self._clock)

    def is_ready(self) -> bool:
        self._refresh_barrier()
        return (
            self._safe_since is not None
            and _clock_now(self._clock) - self._safe_since >= self._hold_s
        )

    def failure_stage(self) -> str:
        if not self._runtime_available:
            return 'runtime_available'
        if not self._runtime_gate_inhibited:
            return 'gate_inhibited'
        if not self._motion_gate_inhibited:
            return 'gate_inhibited'
        if not self._no_active_step:
            return 'runtime_safe_stationary'
        if not self._controller_active:
            return 'controller_lifecycle'
        if not self._controller_observed:
            return 'controller_zero'
        if not self._controller_zero:
            return 'controller_zero'
        if self._odometry_received_at is None:
            return 'odometry_stationary'
        if not self._odometry_stationary:
            return 'odometry_stationary'
        return 'safe_stationary_hold'


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


def _wait_for_motion_readiness(session_spec, deadline, clock):
    """Wait for Runtime, Gate, controller, and stationary odometry evidence."""
    del session_spec
    if _clock_now(clock) >= deadline:
        return _unavailable_result('motion', 'deadline')

    try:
        import rclpy
        from controller_manager_msgs.srv import ListControllers
        from nav_msgs.msg import Odometry
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            qos_profile_sensor_data,
            QoSProfile,
            ReliabilityPolicy,
        )
        from voice_nav_interfaces.msg import MissionState
        from voice_nav_mission.msg import InternalMotionGateState
    except Exception:
        return _unavailable_result('motion', 'setup', 'mode_readiness_failed')

    if not rclpy.ok():
        return _unavailable_result(
            'motion', 'context', 'mode_readiness_failed',
        )

    state = _MotionReadinessState(clock)
    node = None
    pending_controller = None
    try:
        node = rclpy.create_node('voice_nav_app_motion_readiness')
        runtime_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(
            MissionState,
            '/mission/state',
            state.observe_runtime,
            runtime_qos,
        )
        node.create_subscription(
            InternalMotionGateState,
            '/motion_gate/internal/state',
            state.observe_motion_gate,
            runtime_qos,
        )
        node.create_subscription(
            Odometry,
            '/odom',
            state.observe_odometry,
            qos_profile_sensor_data,
        )
        controller_client = node.create_client(
            ListControllers, '/controller_manager/list_controllers',
        )

        while True:
            if state.is_ready():
                return _ready_result()
            remaining = deadline - _clock_now(clock)
            if remaining <= 0:
                return _unavailable_result('motion', state.failure_stage())
            if pending_controller is None:
                try:
                    if controller_client.service_is_ready():
                        pending_controller = controller_client.call_async(
                            ListControllers.Request(),
                        )
                except Exception:
                    pending_controller = None
            rclpy.spin_once(
                node, timeout_sec=min(0.1, max(0.0, remaining)),
            )
            if pending_controller is not None and pending_controller.done():
                try:
                    response = pending_controller.result()
                    state.observe_controller_active(
                        any(
                            controller.name == 'diff_drive_controller'
                            and controller.state == 'active'
                            for controller in response.controller
                        ),
                    )
                except Exception:
                    pass
                pending_controller = None
    except Exception:
        return _unavailable_result('motion', 'setup', 'mode_readiness_failed')
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass


def wait_for_mode_readiness(session_spec, deadline, clock):
    """Wait for typed lifecycle, map, and TF evidence for one mode."""
    mode = getattr(session_spec, 'mode', '')
    if mode == 'motion':
        return _wait_for_motion_readiness(session_spec, deadline, clock)
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
        from voice_nav_interfaces.msg import MissionState
        from tf2_msgs.msg import TFMessage
    except Exception:
        return _unavailable_result(mode, 'setup', 'mode_readiness_failed')

    if not rclpy.ok():
        return _unavailable_result(mode, 'context', 'mode_readiness_failed')

    node = None
    state = _ModeReadinessState(
        mode, State.PRIMARY_STATE_ACTIVE, require_runtime=True,
    )
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
        node.create_subscription(
            MissionState, '/mission/state', state.observe_runtime, map_qos,
        )
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
