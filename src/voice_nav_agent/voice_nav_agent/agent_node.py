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

"""ROS Adapter for the bounded VoiceNav Agent Core."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import re
import secrets
import threading
from typing import Any, Callable, Optional

from action_msgs.srv import CancelGoal
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
    MissionStep as MissionStepMessage,
    VoiceTurn as VoiceTurnMessage,
)
from voice_nav_interfaces.srv import StopMission

from ._agent_engine import AgentEngine, AgentOutcome
from ._console_trace import format_event
from .core import (
    is_control_text,
    MissionState,
    PlanningToken,
    StopDecision,
    VoiceTurn,
)


VOICE_TURN_TOPIC = '/voice/turn'
MISSION_STATE_TOPIC = '/mission/state'
MISSION_EXECUTE_ACTION = '/mission/execute'
MISSION_STOP_SERVICE = '/mission/stop'
VOICE_SPEAK_ACTION = '/voice/speak'
RESPONSE_DEADLINE_SECONDS = 1.0
SPEAK_DISCOVERY_RECHECK_SECONDS = 0.05
MAX_UINT64 = (1 << 64) - 1
_UNSET = object()


def _runtime_allows_patrol_renewal(token: Any, state: MissionState) -> bool:
    """Accept the current safe Runtime epoch, but never a stale/replaced one."""
    return (
        state.runtime_instance_id == token.runtime_instance_id
        and state.admission_epoch >= token.admission_epoch
    )


class MissionSlotState(str, Enum):
    """States of the one Mission Goal owned by this Agent instance."""

    SEND_PENDING = 'SEND_PENDING'
    ACTIVE = 'ACTIVE'
    CANCEL_PENDING = 'CANCEL_PENDING'


@dataclass(slots=True)
class _MissionSlot:
    """Fenced state for one locally submitted Mission Goal."""

    generation: int
    turn_generation: int
    token: PlanningToken
    session_id: str
    turn_id: str
    terminal_turn_generation: int
    terminal_session_id: str
    terminal_turn_id: str
    state: MissionSlotState = MissionSlotState.SEND_PENDING
    goal_handle: Any = None
    send_timer: Any = None
    cancel_timer: Any = None
    cancel_requested: bool = False
    cancel_started: bool = False
    cancel_failure_spoken: bool = False
    terminal_speech_spoken: bool = False
    success_text: str = '任务已完成。'


@dataclass(slots=True)
class _MissionContinuation:
    """One Mission admitted only after a confirmed Operational Stop."""

    outcome: AgentOutcome
    mission: Any
    turn: VoiceTurn
    turn_generation: int
    success_text: str


@dataclass(slots=True)
class _MappingPatrol:
    """One user-owned patrol renewed only at safe Runtime boundaries."""

    outcome: AgentOutcome
    turn: VoiceTurn
    turn_generation: int


@dataclass(slots=True)
class _SpeakOperation:
    """Fenced state for one Speak Goal and its acceptance window."""

    generation: int
    turn_generation: int
    source_seq: int
    session_id: str
    turn_id: str
    text: str
    priority: int
    discovery_deadline_seconds: float = 0.0
    goal_handle: Any = None
    send_timer: Any = None
    cancel_on_accept: bool = False
    cancel_started: bool = False
    stale: bool = False
    send_started: bool = False


@dataclass(slots=True)
class _StopOperation:
    """Fenced state for one Operational Stop service request."""

    generation: int
    turn_generation: int
    decision: StopDecision
    session_id: str
    turn_id: str
    response_timer: Any = None
    continuation: Optional[_MissionContinuation] = None


class _SerialSeam:
    """Serialize Core calls and ownership transitions across ROS callbacks."""

    def __init__(self) -> None:
        """Create the short critical section used by all adapter callbacks."""
        self._lock = threading.RLock()

    def invoke(self, callback: Callable[..., Any], *args: Any) -> Any:
        """Run one adapter operation without concurrent state mutation."""
        with self._lock:
            return callback(*args)


class AgentNode(Node):
    """The sole ROS process that adapts Voice Turns to Agent Core Ports."""

    def __init__(
        self,
        *,
        agent_instance_id: Optional[str] = None,
    ) -> None:
        """Create the node with the frozen VoiceNav ROS graph."""
        instance_id = agent_instance_id or _new_agent_instance_id()
        if not re.fullmatch(r'[0-9a-f]{32}', instance_id):
            raise RuntimeError('agent source instance ID is not a CSPRNG ID')
        super().__init__('agent_node')

        self._seam = _SerialSeam()
        self._agent_instance_id = instance_id
        self._callback_group = MutuallyExclusiveCallbackGroup()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self._turn_generation = 0
        self._mission_generation = 0
        self._speak_generation = 0
        self._speak_seq = 0
        self._stop_generation = 0
        self._mission_slot: Optional[_MissionSlot] = None
        self._speak_operation: Optional[_SpeakOperation] = None
        self._stop_operation: Optional[_StopOperation] = None
        self._mission_continuation: Optional[_MissionContinuation] = None
        self._mapping_patrol: Optional[_MappingPatrol] = None
        self.declare_parameter('llm_endpoint', 'http://127.0.0.1:8080')
        self._engine = AgentEngine(
            self._agent_instance_id,
            planner_endpoint=self.get_parameter('llm_endpoint').value,
            on_outcome=self._on_engine_outcome,
        )

        self._latest_state: Optional[MissionState] = None
        self._traced_state: Optional[tuple[Any, ...]] = None
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

    @property
    def agent_source_instance_id(self) -> str:
        """Return the CSPRNG-backed Agent source instance identity."""
        return self._agent_instance_id

    def _trace(self, event: str, **fields: Any) -> None:
        """Emit one compact product event to the app console."""
        self.get_logger().info(format_event(event, **fields))

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
        self._traced_state = None
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
        state = MissionState(
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
        self._latest_state = state
        trace_state = (
            state.runtime_instance_id,
            state.admission_epoch,
            state.operating_mode,
            state.availability,
            state.gate_state,
            state.active_step,
        )
        if trace_state != self._traced_state:
            self._traced_state = trace_state
            self._trace(
                'runtime_state',
                runtime_instance_id=state.runtime_instance_id,
                admission_epoch=state.admission_epoch,
                mode=state.operating_mode,
                availability=state.availability,
                gate_state=state.gate_state,
                active_step=state.active_step,
            )
        self._engine.observe_runtime_snapshot(state)
        self._state_sample_signature = _publisher_signature(
            publishers[0]
        )
        self._resume_mapping_patrol()

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
        self._trace(
            'voice_turn',
            voice_seq=envelope.voice_seq,
            kind=envelope.kind,
            text=envelope.text,
            confidence=round(envelope.confidence, 4),
            during_playback=envelope.during_playback,
            runtime_mode=(
                int(snapshot.operating_mode) if snapshot is not None else None
            ),
        )
        self._engine.handle_turn(envelope, snapshot)

    def _on_engine_outcome(self, outcome: AgentOutcome, turn: VoiceTurn) -> None:
        """Map only closed Engine outcomes onto ROS Actions/Services."""
        self._seam.invoke(self._handle_engine_outcome, outcome, turn)

    def _handle_engine_outcome(self, outcome: AgentOutcome, turn: VoiceTurn) -> None:
        turn_generation = outcome.generation or (self._turn_generation + 1)
        if not self._engine.consume_delivery_lease(outcome.delivery_lease):
            return
        self._trace(
            'agent_decision',
            voice_seq=turn.voice_seq,
            kind=outcome.kind,
            reason=outcome.reason,
            text=outcome.text,
            generation=turn_generation,
        )
        self._turn_generation = max(self._turn_generation, turn_generation)
        self._stop_operation = None
        self._mission_continuation = None
        self._mapping_patrol = None
        self._retire_speak_operation()
        if outcome.kind == 'mission' and outcome.mission is not None:
            success_text = '任务已完成。'
            patrol = None
            if outcome.reason == 'mapping_patrol':
                success_text = ''
                patrol = _MappingPatrol(
                    outcome,
                    turn,
                    turn_generation,
                )
            slot = self._handle_mission(
                outcome.mission,
                turn,
                turn_generation,
                success_text,
            )
            if slot is not None:
                self._mapping_patrol = patrol
                self._engine.record_owned_mission(outcome, slot)
        elif outcome.kind == 'cancel':
            self._handle_cancel(outcome, turn, turn_generation)
        elif outcome.kind == 'stop':
            decision = StopDecision(
                request_id=outcome.turn_id,
                source_instance_id=outcome.source_instance_id,
                source_seq=outcome.source_seq,
                reason=outcome.reason or 'voice_stop',
            )
            self._handle_stop(decision, turn, turn_generation)
        elif outcome.kind == 'stop_and_save' and outcome.mission is not None:
            decision = StopDecision(
                request_id=outcome.turn_id,
                source_instance_id=outcome.source_instance_id,
                source_seq=outcome.source_seq,
                reason=outcome.reason or 'voice_stop_and_save',
            )
            self._handle_stop(
                decision,
                turn,
                turn_generation,
                _MissionContinuation(
                    outcome=outcome,
                    mission=outcome.mission,
                    turn=turn,
                    turn_generation=turn_generation,
                    success_text='地图已保存。',
                ),
            )
        elif outcome.kind in ('clarify', 'reply', 'rejected'):
            if outcome.text:
                self._speak_text(
                    outcome.text,
                    Speak.Goal.NORMAL,
                    turn.session_id,
                    turn.turn_id,
                    turn_generation,
                )

    def destroy_node(self) -> bool:
        """Stop the bounded Planner worker before destroying ROS resources."""
        self._retire_speak_operation()
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
        if _publisher_signature(publishers[0]) != (
            self._state_sample_signature
        ):
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
        if (
            not _gid_is_present(current_gid)
            or current_gid != current_epoch_gid
        ):
            self._schedule_state_subscription_rebuild()

    def _compatible_state_publishers(self) -> list[Any]:
        try:
            infos = self.get_publishers_info_by_topic(MISSION_STATE_TOPIC)
        except (AttributeError, RuntimeError, TypeError):
            return []
        return [info for info in infos if _compatible_state_publisher(info)]

    def _handle_mission(
        self,
        mission: Any,
        turn: VoiceTurn,
        turn_generation: int,
        success_text: str = '任务已完成。',
    ) -> Optional[_MissionSlot]:
        if self._mission_slot is not None:
            self._speak_text(
                '本地任务正在处理。',
                Speak.Goal.NORMAL,
                turn.session_id,
                turn.turn_id,
                turn_generation,
            )
            return None

        self._mission_generation += 1
        slot = _MissionSlot(
            generation=self._mission_generation,
            turn_generation=turn_generation,
            token=mission.token,
            session_id=turn.session_id,
            turn_id=turn.turn_id,
            terminal_turn_generation=turn_generation,
            terminal_session_id=turn.session_id,
            terminal_turn_id=turn.turn_id,
            success_text=success_text,
        )
        self._mission_slot = slot
        goal = self._mission_goal(mission)
        self._trace(
            'mission_submit',
            turn_id=turn.turn_id,
            generation=turn_generation,
            steps=[
                {
                    'kind': int(step.kind),
                    'distance_m': round(float(step.distance_m), 4),
                    'angle_rad': round(float(step.angle_rad), 4),
                    'target_id': str(step.target_id),
                }
                for step in mission.steps
            ],
        )
        try:
            future = self._mission_client.send_goal_async(goal)
        except Exception as error:
            self.get_logger().warning(
                f'Mission Goal transport failed before submission: {error}'
            )
            self._mission_send_failure(slot, 'mission_transport_error')
            return None
        slot.send_timer = self._one_shot_timer(
            RESPONSE_DEADLINE_SECONDS,
            lambda: self._mission_send_timeout(slot),
        )
        future.add_done_callback(
            lambda completed: self._seam.invoke(
                self._mission_goal_response, slot, completed
            )
        )
        return slot

    def _mission_goal(self, mission: Any) -> ExecuteMission.Goal:
        goal = ExecuteMission.Goal()
        goal.source_instance_id = mission.token.source_instance_id
        goal.source_seq = mission.token.source_seq
        goal.runtime_instance_id = mission.token.runtime_instance_id
        goal.admission_epoch = mission.token.admission_epoch
        for step in mission.steps:
            message = MissionStepMessage()
            message.kind = int(step.kind)
            message.distance_m = float(step.distance_m)
            message.angle_rad = float(step.angle_rad)
            message.target_id = step.target_id
            goal.steps.append(message)
        return goal

    def _mission_send_timeout(self, slot: _MissionSlot) -> None:
        if not self._mission_matches(slot) or slot.state != (
            MissionSlotState.SEND_PENDING
        ):
            return
        slot.send_timer = None
        self._clear_mission_slot(slot)
        if slot.terminal_turn_generation != self._turn_generation:
            return
        text = (
            '取消任务未确认。'
            if slot.cancel_requested
            else '任务提交未确认。'
        )
        self._speak_text(
            text,
            Speak.Goal.NORMAL,
            slot.terminal_session_id,
            slot.terminal_turn_id,
            slot.terminal_turn_generation,
        )

    def _mission_goal_response(self, slot: _MissionSlot, future: Any) -> None:
        handle = None
        error: Optional[Exception] = None
        try:
            handle = future.result()
        except Exception as caught:
            error = caught
        if error is not None:
            if self._mission_matches(slot):
                self._mission_send_failure(slot, 'mission_transport_error')
            return
        if not getattr(handle, 'accepted', False):
            if self._mission_matches(slot):
                self._mission_send_failure(slot, 'mission_goal_rejected')
            return
        if not self._mission_matches(slot):
            self._best_effort_cancel_mission(handle)
            return

        self._cancel_timer(slot.send_timer)
        slot.send_timer = None
        slot.goal_handle = handle
        self._trace(
            'mission_accepted',
            turn_id=slot.turn_id,
            generation=slot.turn_generation,
        )
        if slot.cancel_requested:
            slot.state = MissionSlotState.CANCEL_PENDING
            self._request_mission_cancel(slot)
        else:
            slot.state = MissionSlotState.ACTIVE
        try:
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda completed: self._seam.invoke(
                    self._mission_result, slot, completed
                )
            )
        except Exception as caught:
            self.get_logger().warning(
                f'Mission result callback setup failed: {caught}'
            )

    def _mission_send_failure(self, slot: _MissionSlot, reason: str) -> None:
        if not self._mission_matches(slot):
            return
        self._cancel_timer(slot.send_timer)
        self._cancel_timer(slot.cancel_timer)
        self._clear_mission_slot(slot)
        if slot.terminal_turn_generation != self._turn_generation:
            return
        text = (
            '取消任务未确认。'
            if slot.cancel_requested
            else '任务提交失败。'
        )
        self.get_logger().warning(f'Mission submission failed: {reason}')
        self._speak_text(
            text,
            Speak.Goal.NORMAL,
            slot.terminal_session_id,
            slot.terminal_turn_id,
            slot.terminal_turn_generation,
        )

    def _handle_cancel(
        self,
        outcome: AgentOutcome,
        turn: VoiceTurn,
        turn_generation: int,
    ) -> None:
        slot = self._mission_slot
        if slot is None:
            self._speak_text(
                '没有可取消的本地任务。',
                Speak.Goal.NORMAL,
                turn.session_id,
                turn.turn_id,
                turn_generation,
            )
            return
        if outcome.identity is not slot:
            self._speak_text(
                '没有可取消的本地任务。',
                Speak.Goal.NORMAL,
                turn.session_id,
                turn.turn_id,
                turn_generation,
            )
            return
        if slot.cancel_requested:
            slot.terminal_turn_generation = turn_generation
            slot.terminal_session_id = turn.session_id
            slot.terminal_turn_id = turn.turn_id
            return
        slot.cancel_requested = True
        slot.state = MissionSlotState.CANCEL_PENDING
        slot.terminal_turn_generation = turn_generation
        slot.terminal_session_id = turn.session_id
        slot.terminal_turn_id = turn.turn_id
        if slot.goal_handle is not None:
            self._request_mission_cancel(slot)

    def _request_mission_cancel(self, slot: _MissionSlot) -> None:
        if (
            not self._mission_matches(slot)
            or slot.goal_handle is None
            or slot.cancel_started
        ):
            return
        slot.cancel_started = True
        try:
            future = slot.goal_handle.cancel_goal_async()
        except Exception as error:
            self._mission_cancel_failure(slot, 'cancel_transport_error')
            self.get_logger().warning(
                f'Mission cancel transport failed: {error}'
            )
            return
        slot.cancel_timer = self._one_shot_timer(
            RESPONSE_DEADLINE_SECONDS,
            lambda: self._mission_cancel_timeout(slot),
        )
        future.add_done_callback(
            lambda completed: self._seam.invoke(
                self._mission_cancel_response, slot, completed
            )
        )

    def _mission_cancel_response(
        self, slot: _MissionSlot, future: Any
    ) -> None:
        if not self._mission_matches(slot):
            return
        try:
            response = future.result()
            return_code = int(response.return_code)
        except Exception:
            self._mission_cancel_failure(slot, 'cancel_response_error')
            return
        if return_code != CancelGoal.Response.ERROR_NONE:
            self._mission_cancel_failure(slot, 'cancel_rejected')
            return
        self._cancel_timer(slot.cancel_timer)
        slot.cancel_timer = None
        self._engine.record_cancelled_mission(slot)

    def _mission_cancel_timeout(self, slot: _MissionSlot) -> None:
        if not self._mission_matches(slot):
            return
        slot.cancel_timer = None
        self._mission_cancel_failure(slot, 'cancel_response_timeout')

    def _mission_cancel_failure(self, slot: _MissionSlot, reason: str) -> None:
        if not self._mission_matches(slot):
            return
        self._cancel_timer(slot.cancel_timer)
        slot.cancel_timer = None
        slot.state = MissionSlotState.ACTIVE
        if slot.cancel_failure_spoken:
            return
        slot.cancel_failure_spoken = True
        self.get_logger().warning(
            f'Mission cancel was not confirmed: {reason}'
        )
        if slot.terminal_turn_generation != self._turn_generation:
            return
        self._speak_text(
            '取消请求未确认。',
            Speak.Goal.NORMAL,
            slot.terminal_session_id,
            slot.terminal_turn_id,
            slot.terminal_turn_generation,
        )

    def _mission_result(self, slot: _MissionSlot, future: Any) -> None:
        if not self._mission_matches(slot):
            return
        try:
            wrapped = future.result()
            result = wrapped.result
            code = int(result.code)
            detail = str(result.detail)
        except Exception as error:
            self.get_logger().warning(
                f'Mission result was malformed: {error}'
            )
            code = ExecuteMission.Result.INTERNAL_ERROR
            detail = 'malformed_result'
        self._trace(
            'mission_result',
            turn_id=slot.turn_id,
            generation=slot.turn_generation,
            code=code,
            detail=detail[:256],
        )
        self._engine.record_cancelled_mission(slot)
        self._clear_mission_slot(slot)
        self._resume_mission_continuation()
        patrol = self._mapping_patrol
        if (
            code == ExecuteMission.Result.SUCCEEDED
            and patrol is not None
            and patrol.turn_generation == slot.turn_generation
            and patrol.turn_generation == self._turn_generation
        ):
            self._resume_mapping_patrol()
            return
        if patrol is not None and patrol.turn_generation == slot.turn_generation:
            self._mapping_patrol = None
        if slot.terminal_speech_spoken:
            return
        if slot.terminal_turn_generation != self._turn_generation:
            return
        slot.terminal_speech_spoken = True
        if code == ExecuteMission.Result.SUCCEEDED:
            text = slot.success_text
        elif code == ExecuteMission.Result.CANCELED:
            text = '任务已取消。'
        elif code == ExecuteMission.Result.STOPPED:
            text = '任务已停止。'
        elif code == ExecuteMission.Result.SAFETY_FAULT:
            text = '任务遇到安全故障。'
        else:
            text = '任务执行失败。'
        priority = (
            Speak.Goal.URGENT
            if code == ExecuteMission.Result.SAFETY_FAULT
            else Speak.Goal.NORMAL
        )
        self._speak_text(
            text,
            priority,
            slot.terminal_session_id,
            slot.terminal_turn_id,
            slot.terminal_turn_generation,
        )

    def _handle_stop(
        self,
        decision: StopDecision,
        turn: VoiceTurn,
        turn_generation: int,
        continuation: Optional[_MissionContinuation] = None,
    ) -> None:
        self._stop_generation += 1
        operation = _StopOperation(
            generation=self._stop_generation,
            turn_generation=turn_generation,
            decision=decision,
            session_id=turn.session_id,
            turn_id=turn.turn_id,
            continuation=continuation,
        )
        self._stop_operation = operation
        if not _service_ready(self._stop_client):
            self._stop_failure(operation, 'stop_service_unavailable')
            return
        request = StopMission.Request()
        request.request_id = decision.request_id
        request.source_instance_id = decision.source_instance_id
        request.source_seq = decision.source_seq
        request.reason = decision.reason
        self._trace(
            'stop_request',
            turn_id=turn.turn_id,
            generation=turn_generation,
            reason=decision.reason,
            save_after_stop=continuation is not None,
        )
        try:
            future = self._stop_client.call_async(request)
        except Exception as error:
            self.get_logger().warning(f'STOP service call failed: {error}')
            self._stop_failure(operation, 'stop_transport_error')
            return
        operation.response_timer = self._one_shot_timer(
            RESPONSE_DEADLINE_SECONDS,
            lambda: self._stop_timeout(operation),
        )
        future.add_done_callback(
            lambda completed: self._seam.invoke(
                self._stop_response, operation, completed
            )
        )

    def _stop_response(self, operation: _StopOperation, future: Any) -> None:
        if self._stop_operation is not operation:
            return
        try:
            response = future.result()
            code = int(response.code)
            inhibited = bool(response.motion_inhibited)
        except Exception:
            self._stop_failure(operation, 'stop_response_error')
            return
        self._cancel_timer(operation.response_timer)
        operation.response_timer = None
        self._stop_operation = None
        self._trace(
            'stop_result',
            turn_id=operation.turn_id,
            generation=operation.turn_generation,
            code=code,
            motion_inhibited=inhibited,
        )
        if (
            code in (StopMission.Response.APPLIED, StopMission.Response.DUPLICATE)
            and inhibited
        ):
            if operation.continuation is not None:
                runtime_instance_id = str(response.runtime_instance_id)
                admission_epoch = int(response.admission_epoch)
                if not runtime_instance_id or admission_epoch <= 0:
                    self._speak_text(
                        '已停止，但地图未保存。',
                        Speak.Goal.URGENT,
                        operation.session_id,
                        operation.turn_id,
                        operation.turn_generation,
                    )
                    return
                continuation = operation.continuation
                token = replace(
                    continuation.mission.token,
                    runtime_instance_id=runtime_instance_id,
                    admission_epoch=admission_epoch,
                )
                continuation.mission = replace(
                    continuation.mission,
                    token=token,
                )
                continuation.outcome = replace(
                    continuation.outcome,
                    mission=continuation.mission,
                    token=token,
                )
                self._mission_continuation = continuation
                self._resume_mission_continuation()
                return
            self._speak_text(
                '已停止。',
                Speak.Goal.URGENT,
                operation.session_id,
                operation.turn_id,
                operation.turn_generation,
            )
        else:
            self._speak_text(
                '停止请求未确认。',
                Speak.Goal.URGENT,
                operation.session_id,
                operation.turn_id,
                operation.turn_generation,
            )

    def _stop_timeout(self, operation: _StopOperation) -> None:
        if self._stop_operation is not operation:
            return
        operation.response_timer = None
        self._stop_failure(operation, 'stop_response_timeout')

    def _stop_failure(self, operation: _StopOperation, reason: str) -> None:
        if self._stop_operation is not operation:
            return
        self._cancel_timer(operation.response_timer)
        operation.response_timer = None
        self._stop_operation = None
        self.get_logger().warning(f'STOP was not confirmed: {reason}')
        self._trace(
            'stop_result',
            turn_id=operation.turn_id,
            generation=operation.turn_generation,
            code='failed',
            reason=reason,
        )
        self._speak_text(
            '停止请求未确认。',
            Speak.Goal.URGENT,
            operation.session_id,
            operation.turn_id,
            operation.turn_generation,
        )

    def _resume_mission_continuation(self) -> None:
        continuation = self._mission_continuation
        if continuation is None or self._mission_slot is not None:
            return
        self._mission_continuation = None
        if continuation.turn_generation != self._turn_generation:
            return
        slot = self._handle_mission(
            continuation.mission,
            continuation.turn,
            continuation.turn_generation,
            continuation.success_text,
        )
        if slot is not None:
            self._engine.record_owned_mission(continuation.outcome, slot)

    def _resume_mapping_patrol(self) -> None:
        patrol = self._mapping_patrol
        if (
            patrol is None
            or self._mission_slot is not None
            or patrol.turn_generation != self._turn_generation
        ):
            return
        state = self._planning_snapshot(require_execute_ready=True)
        if (
            state is None
            or state.operating_mode != MissionState.MAPPING
            or state.availability != MissionState.AVAILABLE
            or state.gate_state != MissionState.GATE_INHIBITED
            or patrol.outcome.token is None
            or not _runtime_allows_patrol_renewal(
                patrol.outcome.token, state
            )
        ):
            return
        outcome = self._engine.renew_mapping_patrol(
            patrol.outcome,
            state,
        )
        if outcome is None or outcome.mission is None:
            return
        patrol.outcome = outcome
        slot = self._handle_mission(
            outcome.mission,
            patrol.turn,
            patrol.turn_generation,
            '',
        )
        if slot is not None:
            self._engine.record_owned_mission(outcome, slot)

    def _retire_speak_operation(self) -> None:
        operation = self._speak_operation
        if operation is None:
            return
        operation.stale = True
        operation.cancel_on_accept = True
        self._speak_operation = None
        self._cancel_timer(operation.send_timer)
        operation.send_timer = None
        if operation.goal_handle is not None:
            self._best_effort_cancel_speak(operation)

    def _speak_text(
        self,
        text: str,
        priority: int,
        session_id: str,
        turn_id: str,
        turn_generation: int,
    ) -> None:
        if self._speak_seq >= MAX_UINT64:
            self.get_logger().error('Speak sequence overflow; speech disabled')
            return
        self._retire_speak_operation()
        self._speak_seq += 1
        self._speak_generation += 1
        operation = _SpeakOperation(
            generation=self._speak_generation,
            turn_generation=turn_generation,
            source_seq=self._speak_seq,
            session_id=session_id,
            turn_id=turn_id,
            text=text[:512],
            priority=priority,
        )
        self._speak_operation = operation
        if not _action_server_ready(self._speak_client):
            operation.discovery_deadline_seconds = (
                self._steady_time_seconds() + RESPONSE_DEADLINE_SECONDS
            )
            self._schedule_speak_discovery_recheck(operation)
            return
        self._send_speak_goal(operation)

    def _speak_discovery_recheck(self, operation: _SpeakOperation) -> None:
        """Retry discovery until the existing response deadline is reached."""
        if (
            self._speak_operation is not operation
            or operation.stale
            or operation.send_started
        ):
            return
        operation.send_timer = None
        if self._steady_time_seconds() >= operation.discovery_deadline_seconds:
            self._speak_discovery_timeout(operation)
            return
        if _action_server_ready(self._speak_client):
            self._send_speak_goal(operation)
            return
        self._schedule_speak_discovery_recheck(operation)

    def _schedule_speak_discovery_recheck(
        self, operation: _SpeakOperation
    ) -> None:
        if (
            self._speak_operation is not operation
            or operation.stale
            or operation.send_started
        ):
            return
        remaining = operation.discovery_deadline_seconds - (
            self._steady_time_seconds()
        )
        if remaining <= 0.0:
            self._speak_discovery_timeout(operation)
            return
        operation.send_timer = self._one_shot_timer(
            min(SPEAK_DISCOVERY_RECHECK_SECONDS, remaining),
            lambda: self._speak_discovery_recheck(operation),
        )

    def _speak_discovery_timeout(self, operation: _SpeakOperation) -> None:
        if self._speak_operation is not operation or operation.stale:
            return
        self._cancel_timer(operation.send_timer)
        operation.send_timer = None
        operation.stale = True
        operation.cancel_on_accept = True
        self._speak_operation = None
        self.get_logger().warning('Speak server unavailable; dropping speech')

    def _send_speak_goal(self, operation: _SpeakOperation) -> None:
        """Submit exactly one already-admitted Speak action goal."""
        if (
            self._speak_operation is not operation
            or operation.stale
            or operation.send_started
        ):
            return
        operation.send_started = True
        goal = Speak.Goal()
        goal.source_instance_id = self._agent_instance_id
        goal.source_seq = operation.source_seq
        goal.session_id = operation.session_id
        goal.turn_id = operation.turn_id
        goal.priority = operation.priority
        goal.text = operation.text
        goal.allow_barge_in = True
        try:
            future = self._speak_client.send_goal_async(goal)
        except Exception as error:
            self.get_logger().warning(
                f'Speak Goal transport failed: {error}'
            )
            self._speak_send_failure(operation)
            return
        operation.send_timer = self._one_shot_timer(
            RESPONSE_DEADLINE_SECONDS,
            lambda: self._speak_send_timeout(operation),
        )
        future.add_done_callback(
            lambda completed: self._seam.invoke(
                self._speak_goal_response, operation, completed
            )
        )

    def _speak_goal_response(
        self, operation: _SpeakOperation, future: Any
    ) -> None:
        try:
            handle = future.result()
        except Exception:
            if self._speak_operation is operation:
                self._speak_send_failure(operation)
            return
        if not getattr(handle, 'accepted', False):
            if self._speak_operation is operation:
                self._speak_send_failure(operation)
            return
        if self._speak_operation is not operation or operation.stale:
            self._best_effort_cancel_speak_handle(operation, handle)
            return
        self._cancel_timer(operation.send_timer)
        operation.send_timer = None
        operation.goal_handle = handle
        if operation.cancel_on_accept:
            self._best_effort_cancel_speak(operation)
            return
        try:
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda completed: self._seam.invoke(
                    self._speak_result, operation, completed
                )
            )
        except Exception as error:
            self.get_logger().warning(
                f'Speak result callback setup failed: {error}'
            )

    def _speak_send_timeout(self, operation: _SpeakOperation) -> None:
        if self._speak_operation is not operation:
            return
        operation.send_timer = None
        operation.stale = True
        self._speak_operation = None
        self.get_logger().warning('Speak Goal acceptance was not confirmed')

    def _speak_send_failure(self, operation: _SpeakOperation) -> None:
        if self._speak_operation is not operation:
            return
        self._cancel_timer(operation.send_timer)
        operation.send_timer = None
        self._speak_operation = None
        self.get_logger().warning('Speak Goal was not accepted')

    def _speak_result(self, operation: _SpeakOperation, future: Any) -> None:
        if self._speak_operation is not operation:
            return
        try:
            result = future.result().result
            code = int(result.code)
        except Exception:
            code = Speak.Result.FAILED
        self.get_logger().debug(f'Speak Goal finished with code {code}')
        self._speak_operation = None

    def _best_effort_cancel_speak(self, operation: _SpeakOperation) -> None:
        if operation.goal_handle is None or operation.cancel_started:
            return
        self._best_effort_cancel_speak_handle(operation, operation.goal_handle)

    @staticmethod
    def _best_effort_cancel_speak_handle(
        operation: _SpeakOperation, handle: Any
    ) -> None:
        if operation.cancel_started:
            return
        operation.cancel_started = True
        try:
            handle.cancel_goal_async()
        except Exception:
            pass

    @staticmethod
    def _best_effort_cancel_mission(handle: Any) -> None:
        try:
            handle.cancel_goal_async()
        except Exception:
            pass

    def _mission_matches(self, slot: _MissionSlot) -> bool:
        return self._mission_slot is slot and slot.generation == (
            self._mission_slot.generation if self._mission_slot else -1
        )

    def _clear_mission_slot(self, slot: _MissionSlot) -> None:
        if self._mission_slot is not slot:
            return
        self._cancel_timer(slot.send_timer)
        self._cancel_timer(slot.cancel_timer)
        slot.send_timer = None
        slot.cancel_timer = None
        self._mission_slot = None

    @staticmethod
    def _cancel_timer(timer: Any) -> None:
        """Cancel a one-shot deadline timer when its operation completes."""
        if timer is None:
            return
        try:
            timer.cancel()
        except (AttributeError, RuntimeError):
            pass

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
        """Return the injected steady-clock time used by bounded discovery."""
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
    """Accept a state sample only from the endpoint that actually published it."""
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


def _service_ready(client: Any) -> bool:
    """Check a Service server once without blocking the adapter."""
    try:
        return bool(client.service_is_ready())
    except (AttributeError, RuntimeError):
        return False


def _is_control_turn(turn: VoiceTurn) -> bool:
    """Keep STOP and local CANCEL independent of planning readiness."""
    if turn.kind == VoiceTurn.STOP:
        return True
    if not isinstance(turn.text, str):
        return False
    return is_control_text(turn.text)


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
