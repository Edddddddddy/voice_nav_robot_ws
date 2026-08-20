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

"""Thin ROS Adapter for Agent planning and delivery deep modules."""

from __future__ import annotations

import re
import secrets
import threading
from typing import Any, Callable, Optional
import unicodedata

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from voice_nav_interfaces.action import ExecuteMission, Speak
from voice_nav_interfaces.msg import (
    MissionState as MissionStateMessage,
    VoiceTurn as VoiceTurnMessage,
)
from voice_nav_interfaces.srv import StopMission

from ._agent_delivery_session import AgentDeliverySession
from ._agent_engine import AgentEngine, AgentOutcome
from ._ros_delivery_port import RosDeliveryPort
from .core import MissionState, VoiceTurn


VOICE_TURN_TOPIC = '/voice/turn'
MISSION_STATE_TOPIC = '/mission/state'
MISSION_EXECUTE_ACTION = '/mission/execute'
MISSION_STOP_SERVICE = '/mission/stop'
VOICE_SPEAK_ACTION = '/voice/speak'
_UNSET = object()

_CONTROL_SEPARATOR_RE = re.compile(r'[，,。；;！!?、\s]+')


class _SerialSeam:
    """Serialize short Agent Adapter operations across callback sources."""

    def __init__(self) -> None:
        self._lock = threading.RLock()

    def invoke(self, callback: Callable[..., Any], *args: Any) -> Any:
        with self._lock:
            return callback(*args)


class AgentNode(Node):
    """Adapt ROS messages to AgentEngine and AgentDeliverySession."""

    def __init__(
        self,
        *,
        agent_instance_id: Optional[str] = None,
    ) -> None:
        instance_id = agent_instance_id or _new_agent_instance_id()
        if not re.fullmatch(r'[0-9a-f]{32}', instance_id):
            raise RuntimeError('agent source instance ID is not a CSPRNG ID')
        super().__init__('agent_node')

        self._seam = _SerialSeam()
        self._agent_instance_id = instance_id
        self._callback_group = MutuallyExclusiveCallbackGroup()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.declare_parameter('llm_endpoint', 'http://127.0.0.1:8080')
        self._engine = AgentEngine(
            instance_id,
            planner_endpoint=self.get_parameter('llm_endpoint').value,
            on_outcome=self._on_engine_outcome,
        )

        self._latest_state: Optional[MissionState] = None
        self._state_sample_signature: Optional[tuple[Any, ...]] = None
        self._state_subscription_generation = 0
        self._state_subscription_epoch_gid: Any = None
        self._state_rebuild_scheduled = False
        self._state_rebuild_guard = self.create_guard_condition(
            self._on_state_rebuild_event,
            callback_group=self._callback_group,
        )
        self._state_subscription = None
        self._install_state_subscription()
        self._turn_subscription = self.create_subscription(
            VoiceTurnMessage,
            VOICE_TURN_TOPIC,
            self._on_turn_message,
            _voice_turn_qos(),
            callback_group=self._callback_group,
        )
        self._mission_client = ActionClient(
            self,
            ExecuteMission,
            MISSION_EXECUTE_ACTION,
            callback_group=self._callback_group,
        )
        self._stop_client = self.create_client(
            StopMission,
            MISSION_STOP_SERVICE,
            callback_group=self._callback_group,
        )
        self._speak_client = ActionClient(
            self,
            Speak,
            VOICE_SPEAK_ACTION,
            callback_group=self._callback_group,
        )
        self._delivery_port = RosDeliveryPort(
            mission_client=self._mission_client,
            stop_client=self._stop_client,
            speak_client=self._speak_client,
            timer_factory=self._one_shot_timer,
            steady_time=self._steady_time_seconds,
            invoke=self._seam.invoke,
            logger=self.get_logger(),
        )
        self._delivery = AgentDeliverySession(
            instance_id,
            self._engine,
            self._delivery_port,
        )

    @property
    def agent_source_instance_id(self) -> str:
        """Return the CSPRNG-backed Agent source instance identity."""
        return self._agent_instance_id

    def _on_state_message(
        self,
        message: MissionStateMessage,
        message_info: Any,
        generation: Optional[int] = None,
        epoch_gid: Any = _UNSET,
    ) -> None:
        if generation is None:
            generation = self._state_subscription_generation
        if epoch_gid is _UNSET:
            epoch_gid = self._state_subscription_epoch_gid
        self._seam.invoke(
            self._observe_state,
            message,
            message_info,
            generation,
            epoch_gid,
        )

    def _install_state_subscription(self) -> None:
        self._state_subscription_generation += 1
        generation = self._state_subscription_generation
        epoch_gid = self._capture_state_epoch_gid()
        self._state_subscription_epoch_gid = epoch_gid

        def callback(message: MissionStateMessage, message_info: Any) -> None:
            self._on_state_message(
                message,
                message_info,
                generation,
                epoch_gid,
            )

        self._state_subscription = self.create_subscription(
            MissionStateMessage,
            MISSION_STATE_TOPIC,
            callback,
            _state_qos(),
            callback_group=self._callback_group,
        )

    def _on_state_rebuild_event(self) -> None:
        self._seam.invoke(self._rebuild_state_subscription)

    def _schedule_state_subscription_rebuild(self) -> None:
        if self._state_rebuild_scheduled:
            return
        self._state_rebuild_scheduled = True
        try:
            self._state_rebuild_guard.trigger()
        except (AttributeError, RuntimeError):
            self._state_rebuild_scheduled = False

    def _rebuild_state_subscription(self) -> None:
        self._state_rebuild_scheduled = False
        self._latest_state = None
        self._state_sample_signature = None
        self._engine.invalidate('runtime_state_subscription_rebuild')
        if self._state_subscription is not None:
            self.destroy_subscription(self._state_subscription)
        self._install_state_subscription()

    def _capture_state_epoch_gid(self) -> Any:
        publishers = self._compatible_state_publishers()
        if len(publishers) != 1:
            return None
        gid = _gid_key(getattr(publishers[0], 'endpoint_gid', None))
        return gid if _gid_is_present(gid) else None

    def _observe_state(
        self,
        message: MissionStateMessage,
        message_info: Any,
        generation: int,
        epoch_gid: Any,
    ) -> None:
        if generation != self._state_subscription_generation:
            return
        epoch_gid = _gid_key(epoch_gid)
        current_epoch_gid = _gid_key(self._state_subscription_epoch_gid)
        publishers = self._compatible_state_publishers()
        current_gid = None
        if len(publishers) == 1:
            current_gid = _gid_key(getattr(publishers[0], 'endpoint_gid', None))
        message_gid = _message_publisher_gid(message_info)
        proof_valid = (
            _gid_is_present(epoch_gid)
            and _gid_is_present(current_gid)
            and epoch_gid == current_epoch_gid
            and current_gid == epoch_gid
            and (message_gid is None or message_gid == current_gid)
        )
        if not proof_valid:
            self._latest_state = None
            self._state_sample_signature = None
            self._schedule_state_subscription_rebuild()
            return
        self._latest_state = MissionState(
            runtime_instance_id=message.runtime_instance_id,
            admission_epoch=int(message.admission_epoch),
            operating_mode=int(message.operating_mode),
            availability=int(message.availability),
            gate_state=int(message.gate_state),
            active_step=int(message.active_step),
            supported_step_mask=int(message.supported_step_mask),
            max_steps=int(message.max_steps),
            named_place_ids=tuple(message.named_place_ids),
        )
        self._engine.observe_runtime_snapshot(self._latest_state)
        self._state_sample_signature = _publisher_signature(publishers[0])

    def _on_turn_message(self, message: VoiceTurnMessage) -> None:
        self._seam.invoke(self._handle_turn_message, message)

    def _handle_turn_message(self, message: VoiceTurnMessage) -> None:
        envelope = VoiceTurn(
            voice_instance_id=message.voice_instance_id,
            voice_seq=int(message.voice_seq),
            session_id=message.session_id,
            turn_id=message.turn_id,
            kind=int(message.kind),
            text=message.text,
            confidence=float(message.confidence),
            during_playback=bool(message.during_playback),
        )
        snapshot = self._planning_snapshot(
            require_execute_ready=not _is_control_turn(envelope)
        )
        self._engine.handle_turn(envelope, snapshot)

    def _on_engine_outcome(self, outcome: AgentOutcome, turn: VoiceTurn) -> None:
        self._seam.invoke(self._delivery.accept, outcome, turn)

    def destroy_node(self) -> bool:
        """Close delivery admission before stopping the Planner worker."""
        self._seam.invoke(self._delivery.shutdown)
        self._seam.invoke(self._engine.invalidate, 'destroy_node')
        self._engine.shutdown()
        return super().destroy_node()

    def _planning_snapshot(
        self, *, require_execute_ready: bool
    ) -> Optional[MissionState]:
        publishers = self._compatible_state_publishers()
        if self._latest_state is None:
            self._reconcile_empty_state_epoch(publishers)
            return None
        current_gid = None
        if len(publishers) == 1:
            current_gid = _gid_key(getattr(publishers[0], 'endpoint_gid', None))
        current_epoch_gid = _gid_key(self._state_subscription_epoch_gid)
        if (
            not _gid_is_present(current_gid)
            or current_gid != current_epoch_gid
            or self._state_sample_signature is None
        ):
            self._latest_state = None
            self._state_sample_signature = None
            self._schedule_state_subscription_rebuild()
            return None
        if _publisher_signature(publishers[0]) != self._state_sample_signature:
            self._latest_state = None
            self._state_sample_signature = None
            self._schedule_state_subscription_rebuild()
            return None
        if require_execute_ready and not _action_server_ready(
            self._mission_client
        ):
            return None
        return self._latest_state

    def _reconcile_empty_state_epoch(self, publishers: list[Any]) -> None:
        """Reconcile a unique publisher before a fail-closed empty Turn."""
        if len(publishers) != 1:
            return
        current_gid = _gid_key(getattr(publishers[0], 'endpoint_gid', None))
        current_epoch_gid = _gid_key(self._state_subscription_epoch_gid)
        if not _gid_is_present(current_gid) or current_gid != current_epoch_gid:
            self._schedule_state_subscription_rebuild()

    def _compatible_state_publishers(self) -> list[Any]:
        try:
            infos = self.get_publishers_info_by_topic(MISSION_STATE_TOPIC)
        except (AttributeError, RuntimeError, TypeError):
            return []
        return [info for info in infos if _compatible_state_publisher(info)]

    def _one_shot_timer(
        self, seconds: float, callback: Callable[[], None]
    ) -> Any:
        holder: dict[str, Any] = {}

        def fire() -> None:
            timer = holder.get('timer')
            if timer is not None:
                timer.cancel()
            self._seam.invoke(callback)

        timer = self.create_timer(
            seconds,
            fire,
            callback_group=self._callback_group,
            clock=self._steady_clock,
        )
        holder['timer'] = timer
        return timer

    def _steady_time_seconds(self) -> float:
        return self._steady_clock.now().nanoseconds / 1_000_000_000.0


def _new_agent_instance_id() -> str:
    """Generate one lower-case 32-hex Agent source identity."""
    value = secrets.token_hex(16)
    if not re.fullmatch(r'[0-9a-f]{32}', value):
        raise RuntimeError('OS CSPRNG returned an invalid Agent identity')
    return value


def _voice_turn_qos() -> QoSProfile:
    """Return the approved final VoiceTurn subscription QoS."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _state_qos() -> QoSProfile:
    """Return the approved transient MissionState subscription QoS."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _publisher_signature(info: Any) -> tuple[Any, ...]:
    """Create a stable identity for one discovered state publisher."""
    gid = _gid_key(getattr(info, 'endpoint_gid', None))
    return (
        gid,
        getattr(info, 'node_name', None),
        getattr(info, 'node_namespace', None),
        getattr(info, 'topic_type', None),
    )


def _gid_key(gid: Any) -> Any:
    """Normalize ROS GID containers for exact metadata comparison."""
    if gid is None:
        return None
    if isinstance(gid, dict):
        return _gid_key(gid.get('data'))
    if hasattr(gid, 'data'):
        return _gid_key(getattr(gid, 'data'))
    if isinstance(gid, bytes):
        return gid
    try:
        return bytes(gid)
    except (TypeError, ValueError):
        try:
            return tuple(gid)
        except TypeError:
            return gid


def _gid_is_present(gid: Any) -> bool:
    """Return whether a normalized endpoint GID can prove an epoch."""
    if gid is None:
        return False
    try:
        return len(gid) > 0
    except TypeError:
        return True


def _message_publisher_gid(message_info: Any) -> Any:
    """Read an optional publisher GID from current or future ROS metadata."""
    if isinstance(message_info, dict):
        raw_message_gid = message_info.get('publisher_gid')
    else:
        raw_message_gid = getattr(message_info, 'publisher_gid', None)
    return _gid_key(raw_message_gid)


def _publisher_gid_matches(message_info: Any, publisher_info: Any) -> bool:
    """Accept a state sample only from the endpoint that published it."""
    message_gid = _message_publisher_gid(message_info)
    endpoint_gid = _gid_key(getattr(publisher_info, 'endpoint_gid', None))
    return message_gid is not None and message_gid == endpoint_gid


def _compatible_state_publisher(info: Any) -> bool:
    """Check the state publisher against the frozen subscriber contract."""
    topic_type = getattr(info, 'topic_type', None)
    if topic_type not in (None, 'voice_nav_interfaces/msg/MissionState'):
        return False
    qos = getattr(info, 'qos_profile', None)
    if qos is None:
        return True
    try:
        depth = int(getattr(qos, 'depth', 0))
        history = getattr(qos, 'history', None)
    except (TypeError, ValueError):
        return False
    history_unknown = history is not None and int(history) == int(
        HistoryPolicy.UNKNOWN
    )
    history_ok = history == HistoryPolicy.KEEP_LAST or history_unknown
    depth_ok = depth == 1 or (history_unknown and depth == 0)
    return (
        history_ok
        and depth_ok
        and getattr(qos, 'reliability', None) == ReliabilityPolicy.RELIABLE
        and getattr(qos, 'durability', None) == DurabilityPolicy.TRANSIENT_LOCAL
    )


def _action_server_ready(client: Any) -> bool:
    """Check an Action server once without turning discovery into a wait."""
    try:
        return bool(client.server_is_ready())
    except (AttributeError, RuntimeError):
        return False


def _is_control_turn(turn: VoiceTurn) -> bool:
    """Keep STOP and local CANCEL independent of planning readiness."""
    if turn.kind == VoiceTurn.STOP:
        return True
    if not isinstance(turn.text, str):
        return False
    normalized = unicodedata.normalize('NFKC', turn.text).strip()
    clauses = [clause.strip() for clause in _CONTROL_SEPARATOR_RE.split(normalized)]
    return any(
        bool(
            re.fullmatch(
                r'(?:小智\s*)?(?:请\s*)?(?:取消任务|停止|紧急停止)',
                clause,
            )
        )
        for clause in clauses
        if clause
    )


def main(args: Optional[list[str]] = None) -> None:
    """Run the production `agent_node` process."""
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
