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

"""Own the bounded Mission, STOP and speech delivery lifetime for Agent."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import threading
from typing import Callable, Optional, Protocol

from ._agent_engine import AgentOutcome
from .core import VoiceTurn


NORMAL_SPEECH = 1
URGENT_SPEECH = 2
MAX_UINT64 = (1 << 64) - 1


class MissionTerminal(str, Enum):
    """Transport-neutral terminal states for an admitted Mission."""

    SUCCEEDED = 'succeeded'
    CANCELED = 'canceled'
    STOPPED = 'stopped'
    SAFETY_FAULT = 'safety_fault'
    SUBMIT_TIMEOUT = 'submit_timeout'
    SUBMIT_FAILED = 'submit_failed'
    FAILED = 'failed'


@dataclass(frozen=True, slots=True)
class SpeakRequest:
    """One bounded speech request handed to the ROS delivery Adapter."""

    source_instance_id: str
    source_seq: int
    session_id: str
    turn_id: str
    text: str
    priority: int


@dataclass(frozen=True, slots=True)
class StopRequest:
    """One bounded Operational Stop request handed to the ROS Adapter."""

    request_id: str
    source_instance_id: str
    source_seq: int
    reason: str


class DeliveryPort(Protocol):
    """Asynchronous I/O owned outside the delivery state machine."""

    def submit_mission(
        self,
        identity: object,
        mission: object,
        callback: Callable[[MissionTerminal], None],
    ) -> bool: ...

    def cancel_mission(
        self, identity: object, callback: Callable[[bool], None]
    ) -> bool: ...

    def submit_stop(
        self,
        identity: object,
        request: StopRequest,
        callback: Callable[[bool], None],
    ) -> bool: ...

    def submit_speak(
        self,
        identity: object,
        request: SpeakRequest,
        callback: Callable[[], None],
    ) -> bool: ...

    def retire(self, identity: object) -> None: ...

    def shutdown(self) -> None: ...


class AgentEnginePort(Protocol):
    """Small ownership seam consumed from the existing AgentEngine."""

    def consume_delivery_lease(self, lease: object) -> bool: ...

    def record_owned_mission(
        self, outcome: AgentOutcome, identity: object
    ) -> bool: ...

    def record_cancelled_mission(self, identity: object) -> bool: ...


@dataclass(slots=True, eq=False)
class _MissionOperation:
    generation: int
    session_id: str
    turn_id: str


@dataclass(slots=True, eq=False)
class _Operation:
    generation: int


class AgentDeliverySession:
    """Single owner of Agent delivery state and stale-callback fencing."""

    def __init__(
        self,
        source_instance_id: str,
        engine: AgentEnginePort,
        port: DeliveryPort,
    ) -> None:
        self._source_instance_id = source_instance_id
        self._engine = engine
        self._port = port
        self._lock = threading.RLock()
        self._generation = 0
        self._speak_seq = 0
        self._mission: Optional[_MissionOperation] = None
        self._speak: Optional[_Operation] = None
        self._stop: Optional[_Operation] = None
        self._closed = False

    def accept(self, outcome: AgentOutcome, turn: VoiceTurn) -> bool:
        """Accept one current Engine outcome and start at most one delivery."""
        with self._lock:
            if self._closed or not self._engine.consume_delivery_lease(
                outcome.delivery_lease
            ):
                return False
            generation = outcome.generation or (self._generation + 1)
            if generation <= self._generation:
                return False
            self._generation = generation
            self._retire_speak()
            self._retire_stop()

            if outcome.kind == 'mission' and outcome.mission is not None:
                self._submit_mission(outcome, turn)
            elif outcome.kind == 'cancel':
                self._cancel_mission(outcome, turn)
            elif outcome.kind == 'stop':
                self._submit_stop(outcome, turn)
            elif outcome.kind in ('clarify', 'reply', 'rejected'):
                if outcome.text:
                    self._speak_text(
                        outcome.text,
                        NORMAL_SPEECH,
                        turn.session_id,
                        turn.turn_id,
                    )
            return True

    def shutdown(self) -> None:
        """Close admission and revoke every operation before transport teardown."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            mission = self._mission
            self._mission = None
            if mission is not None:
                self._port.retire(mission)
                self._engine.record_cancelled_mission(mission)
            self._retire_speak()
            self._retire_stop()
            self._port.shutdown()

    def _submit_mission(
        self, outcome: AgentOutcome, turn: VoiceTurn
    ) -> None:
        if self._mission is not None:
            self._speak_text(
                '本地任务正在处理。',
                NORMAL_SPEECH,
                turn.session_id,
                turn.turn_id,
            )
            return
        operation = _MissionOperation(
            self._generation,
            turn.session_id,
            turn.turn_id,
        )
        self._mission = operation
        if not self._engine.record_owned_mission(outcome, operation):
            self._mission = None
            return
        accepted = self._port.submit_mission(
            operation,
            outcome.mission,
            lambda terminal: self._invoke(
                self._mission_terminal, operation, terminal
            ),
        )
        if accepted:
            return
        self._engine.record_cancelled_mission(operation)
        self._mission = None
        self._speak_text(
            '任务提交失败。',
            NORMAL_SPEECH,
            turn.session_id,
            turn.turn_id,
        )

    def _cancel_mission(
        self, outcome: AgentOutcome, turn: VoiceTurn
    ) -> None:
        operation = self._mission
        if operation is None or outcome.identity is not operation:
            self._speak_text(
                '没有可取消的本地任务。',
                NORMAL_SPEECH,
                turn.session_id,
                turn.turn_id,
            )
            return
        operation.generation = self._generation
        operation.session_id = turn.session_id
        operation.turn_id = turn.turn_id
        accepted = self._port.cancel_mission(
            operation,
            lambda confirmed: self._invoke(
                self._cancel_result, operation, confirmed
            ),
        )
        if not accepted:
            self._cancel_result(operation, False)

    def _cancel_result(
        self, operation: _MissionOperation, confirmed: bool
    ) -> None:
        if self._mission is not operation or self._closed:
            return
        if confirmed:
            self._engine.record_cancelled_mission(operation)
            return
        if operation.generation == self._generation:
            self._speak_text(
                '取消请求未确认。',
                NORMAL_SPEECH,
                operation.session_id,
                operation.turn_id,
            )

    def _mission_terminal(
        self, operation: _MissionOperation, terminal: MissionTerminal
    ) -> None:
        if self._mission is not operation:
            return
        self._mission = None
        self._engine.record_cancelled_mission(operation)
        if self._closed or operation.generation != self._generation:
            return
        text = {
            MissionTerminal.SUCCEEDED: '任务已完成。',
            MissionTerminal.CANCELED: '任务已取消。',
            MissionTerminal.STOPPED: '任务已停止。',
            MissionTerminal.SAFETY_FAULT: '任务遇到安全故障。',
            MissionTerminal.SUBMIT_TIMEOUT: '任务提交未确认。',
            MissionTerminal.SUBMIT_FAILED: '任务提交失败。',
        }.get(terminal, '任务执行失败。')
        priority = (
            URGENT_SPEECH
            if terminal is MissionTerminal.SAFETY_FAULT
            else NORMAL_SPEECH
        )
        self._speak_text(
            text,
            priority,
            operation.session_id,
            operation.turn_id,
        )

    def _submit_stop(self, outcome: AgentOutcome, turn: VoiceTurn) -> None:
        operation = _Operation(self._generation)
        self._stop = operation
        request = StopRequest(
            request_id=outcome.turn_id,
            source_instance_id=outcome.source_instance_id,
            source_seq=outcome.source_seq,
            reason=outcome.reason or 'voice_stop',
        )
        accepted = self._port.submit_stop(
            operation,
            request,
            lambda confirmed: self._invoke(
                self._stop_result,
                operation,
                confirmed,
                turn.session_id,
                turn.turn_id,
            ),
        )
        if not accepted:
            self._stop_result(
                operation,
                False,
                turn.session_id,
                turn.turn_id,
            )

    def _stop_result(
        self,
        operation: _Operation,
        confirmed: bool,
        session_id: str,
        turn_id: str,
    ) -> None:
        if self._stop is not operation:
            return
        self._stop = None
        if self._closed or operation.generation != self._generation:
            return
        self._speak_text(
            '已停止。' if confirmed else '停止请求未确认。',
            URGENT_SPEECH,
            session_id,
            turn_id,
        )

    def _speak_text(
        self,
        text: str,
        priority: int,
        session_id: str,
        turn_id: str,
    ) -> None:
        if self._closed or self._speak_seq >= MAX_UINT64:
            return
        self._retire_speak()
        self._speak_seq += 1
        operation = _Operation(self._generation)
        self._speak = operation
        request = SpeakRequest(
            source_instance_id=self._source_instance_id,
            source_seq=self._speak_seq,
            session_id=session_id,
            turn_id=turn_id,
            text=text[:512],
            priority=priority,
        )
        accepted = self._port.submit_speak(
            operation,
            request,
            lambda: self._invoke(self._speak_done, operation),
        )
        if not accepted and self._speak is operation:
            self._speak = None

    def _speak_done(self, operation: _Operation) -> None:
        if self._speak is operation:
            self._speak = None

    def _retire_speak(self) -> None:
        operation = self._speak
        if operation is None:
            return
        self._speak = None
        self._port.retire(operation)

    def _retire_stop(self) -> None:
        operation = self._stop
        if operation is None:
            return
        self._stop = None
        self._port.retire(operation)

    def _invoke(self, callback: Callable[..., None], *args: object) -> None:
        with self._lock:
            callback(*args)
