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

"""Package-private bounded Response/session orchestration primitives."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
from typing import Callable, Optional

from .core import (
    _bounded_id,
    _finite,
    _freeze_runtime_snapshot,
    _uint64,
    MAX_RETIRED_VOICE_INSTANCES,
    MissionProposal,
    PlanningToken,
    SemanticValidator,
    VoiceTurn,
)
from .llm_fallback import _decode_steps, _load_schema


@dataclass(frozen=True, slots=True)
class _ToolDefinition:
    """One closed semantic tool definition exposed to a provider."""

    name: str
    parameters: dict[str, object]


_TOOL_REGISTRY = (
    _ToolDefinition(
        'read_runtime_snapshot',
        {'type': 'object', 'properties': {}, 'additionalProperties': False},
    ),
    _ToolDefinition(
        'propose_mission',
        _load_schema()['oneOf'][0],
    ),
    _ToolDefinition(
        'cancel_owned_mission',
        {'type': 'object', 'properties': {}, 'additionalProperties': False},
    ),
)


@dataclass(frozen=True, slots=True)
class _ToolCall:
    """One untrusted provider tool invocation."""

    name: object
    arguments: object


@dataclass(frozen=True, slots=True)
class _ProviderResponse:
    """One bounded provider result for a pending Response."""

    kind: object
    text: object = ''
    tool_calls: object = ()


@dataclass(frozen=True, slots=True)
class _ResponseEvent:
    """One package-private text outcome for a completed Response."""

    kind: str
    session_id: str
    turn_id: str
    text: str


@dataclass(frozen=True, slots=True)
class _ToolOutput:
    """One bounded semantic tool result retained for the provider seam."""

    name: str
    value: tuple[tuple[str, object], ...]


@dataclass(frozen=True, slots=True)
class _ResponseRequest:
    """One provider work item with its immutable planning snapshot."""

    turn: VoiceTurn
    token: PlanningToken
    runtime_snapshot: object
    clarification: Optional[str]
    agent_generation: int


@dataclass(frozen=True, slots=True)
class _OwnedMission:
    """The exact active Mission identity owned by this Agent instance."""

    identity: object
    agent_generation: int


@dataclass(frozen=True, slots=True)
class _ProposeAction:
    """One parsed Mission awaiting a fenced external two-phase commit."""

    request: _ResponseRequest
    mission: object


@dataclass(frozen=True, slots=True)
class _CancelAction:
    """One parsed owned cancellation awaiting a fenced external commit."""

    request: _ResponseRequest
    owned: _OwnedMission


@dataclass(frozen=True, slots=True)
class _RetainedState:
    """Bounded state retained by one package-private Response session."""

    events: int
    tool_outputs: int
    retired_voice_instances: int
    active: bool
    pending: bool
    owned_mission: bool
    voice_fencing_latched: bool


class _ResponseSession:
    """Own one bounded, deterministic provider Response session."""

    def __init__(
        self,
        agent_instance_id: str,
        provider: object,
        runtime_snapshot: Callable[[], object],
        mission_port: object,
        validator: Optional[SemanticValidator] = None,
    ) -> None:
        self._agent_instance_id = agent_instance_id
        self._provider = provider
        self._runtime_snapshot = runtime_snapshot
        self._mission_port = mission_port
        self._validator = validator or SemanticValidator()
        self._lock = threading.RLock()
        self._source_seq = 0
        self._generation = 0
        self._agent_generation = 1
        self._active: Optional[_ResponseRequest] = None
        self._pending: Optional[_ResponseRequest] = None
        self._clarification: Optional[str] = None
        self._events: deque[_ResponseEvent] = deque(maxlen=1)
        self._tool_outputs: deque[_ToolOutput] = deque(maxlen=1)
        self._owned_mission: Optional[_OwnedMission] = None
        self._voice_instance_id: Optional[str] = None
        self._last_voice_seq = -1
        self._retired_voice_instances: set[str] = set()
        self._voice_fencing_latched = False
        self._session_id: Optional[str] = None

    @property
    def tool_registry(self) -> tuple[_ToolDefinition, ...]:
        """Return the exact closed semantic tool set."""
        return _TOOL_REGISTRY

    @property
    def events(self) -> tuple[_ResponseEvent, ...]:
        """Return text outcomes without connecting a product Speak adapter."""
        with self._lock:
            return tuple(self._events)

    @property
    def tool_outputs(self) -> tuple[_ToolOutput, ...]:
        """Return bounded output without exposing Runtime transport data."""
        with self._lock:
            return tuple(self._tool_outputs)

    @property
    def retained_state(self) -> _RetainedState:
        """Describe all bounded session-retained state for behavior tests."""
        with self._lock:
            return _RetainedState(
                events=len(self._events),
                tool_outputs=len(self._tool_outputs),
                retired_voice_instances=len(self._retired_voice_instances),
                active=self._active is not None,
                pending=self._pending is not None,
                owned_mission=self._owned_mission is not None,
                voice_fencing_latched=self._voice_fencing_latched,
            )

    def consume_events(self) -> tuple[_ResponseEvent, ...]:
        """Return and clear the current bounded text delivery."""
        with self._lock:
            events = tuple(self._events)
            self._events.clear()
            return events

    def consume_tool_outputs(self) -> tuple[_ToolOutput, ...]:
        """Return and clear the current bounded semantic tool delivery."""
        with self._lock:
            outputs = tuple(self._tool_outputs)
            self._tool_outputs.clear()
            return outputs

    def accept_turn(self, turn: object) -> None:
        """Start one Response against a frozen trusted Runtime snapshot."""
        request: Optional[_ResponseRequest] = None
        with self._lock:
            if not self._observe_newer_turn(turn):
                return
            self._invalidate_response()
            if not self._valid_turn(turn):
                return
            assert isinstance(turn, VoiceTurn)
            if self._session_id is None:
                self._session_id = turn.session_id
            elif turn.session_id != self._session_id:
                return
            if turn.kind != VoiceTurn.COMMAND:
                return
            snapshot, _reason = _freeze_runtime_snapshot(
                self._runtime_snapshot()
            )
            if snapshot is None:
                return
            self._source_seq += 1
            token = PlanningToken(
                source_instance_id=self._agent_instance_id,
                source_seq=self._source_seq,
                voice_instance_id=turn.voice_instance_id,
                voice_seq=turn.voice_seq,
                session_id=turn.session_id,
                turn_id=turn.turn_id,
                local_generation=self._generation,
                runtime_instance_id=snapshot.runtime_instance_id,
                admission_epoch=snapshot.admission_epoch,
                operating_mode=snapshot.operating_mode,
                supported_step_mask=snapshot.supported_step_mask,
                max_steps=snapshot.max_steps,
                named_place_ids=snapshot.named_place_ids,
                availability=snapshot.availability,
                gate_state=snapshot.gate_state,
            )
            request = _ResponseRequest(
                turn,
                token,
                snapshot,
                self._clarification,
                self._agent_generation,
            )
            self._clarification = None
            if self._active is None:
                self._active = request
            else:
                self._pending = request
                request = None
        if request is not None:
            self._provider.submit(request)

    def invalidate_agent_generation(self) -> None:
        """Discard all Response state after an Agent generation transition."""
        with self._lock:
            self._agent_generation += 1
            self._invalidate_response()
            self._active = None
            self._owned_mission = None

    def restart(self, agent_instance_id: str) -> None:
        """Clear dialogue state and begin a fresh Agent instance lifetime."""
        if not _bounded_id(agent_instance_id, 36):
            raise ValueError('agent_instance_id must be a bounded ID')
        with self._lock:
            self._agent_generation += 1
            self._invalidate_response()
            self._active = None
            self._owned_mission = None
            self._agent_instance_id = agent_instance_id
            self._source_seq = 0
            self._voice_instance_id = None
            self._last_voice_seq = -1
            self._retired_voice_instances.clear()
            self._voice_fencing_latched = False
            self._session_id = None
            self._events.clear()
            self._tool_outputs.clear()

    def _observe_newer_turn(self, turn: object) -> bool:
        if not isinstance(turn, VoiceTurn):
            return False
        if not _bounded_id(turn.voice_instance_id, 36) or not _uint64(
            turn.voice_seq
        ):
            return False
        if self._voice_fencing_latched:
            return turn.kind == VoiceTurn.STOP
        if self._voice_instance_id is None:
            self._voice_instance_id = turn.voice_instance_id
            self._last_voice_seq = -1
        elif turn.voice_instance_id != self._voice_instance_id:
            if turn.voice_instance_id in self._retired_voice_instances:
                return False
            if len(self._retired_voice_instances) >= (
                MAX_RETIRED_VOICE_INSTANCES
            ):
                self._voice_fencing_latched = True
                return False
            self._retired_voice_instances.add(self._voice_instance_id)
            self._voice_instance_id = turn.voice_instance_id
            self._last_voice_seq = -1
            if len(self._retired_voice_instances) >= (
                MAX_RETIRED_VOICE_INSTANCES
            ):
                self._voice_fencing_latched = True
        if turn.voice_seq <= self._last_voice_seq:
            return False
        self._last_voice_seq = turn.voice_seq
        return True

    def _invalidate_response(self) -> None:
        self._generation += 1
        self._pending = None
        self._clarification = None

    @staticmethod
    def _valid_turn(turn: object) -> bool:
        if not isinstance(turn, VoiceTurn):
            return False
        return (
            _bounded_id(turn.voice_instance_id, 36)
            and _uint64(turn.voice_seq)
            and _bounded_id(turn.session_id, 36)
            and _bounded_id(turn.turn_id, 36)
            and turn.kind in (VoiceTurn.COMMAND, VoiceTurn.STOP)
            and isinstance(turn.text, str)
            and len(turn.text) <= 512
            and _finite(turn.confidence)
            and 0.0 <= turn.confidence <= 1.0
        )

    def complete(self, request: object, response: object) -> None:
        """Apply a provider result only while its Response is current."""
        action: Optional[object] = None
        next_request: Optional[_ResponseRequest] = None
        with self._lock:
            if request is not self._active or not isinstance(
                response, _ProviderResponse
            ):
                return
            assert isinstance(request, _ResponseRequest)
            self._active = None
            if self._request_is_current_locked(request):
                action = self._apply_current_response_locked(request, response)
            next_request = self._take_latest_pending_locked()
        if next_request is not None:
            self._provider.submit(next_request)
        if isinstance(action, _ProposeAction):
            self._commit_propose(action)
        elif isinstance(action, _CancelAction):
            self._commit_cancel(action)

    def _request_is_current_locked(self, request: _ResponseRequest) -> bool:
        if (
            request.token.local_generation != self._generation
            or request.agent_generation != self._agent_generation
        ):
            return False
        current, _reason = _freeze_runtime_snapshot(self._runtime_snapshot())
        return current == request.runtime_snapshot

    def _apply_current_response_locked(
        self, request: _ResponseRequest, response: _ProviderResponse
    ) -> Optional[object]:
        if response.kind == 'clarify' and self._valid_text(response.text):
            self._clarification = response.text
            self._events.append(
                _ResponseEvent(
                    'clarify', request.turn.session_id, request.turn.turn_id,
                    response.text,
                )
            )
            return None
        if response.kind != 'tool' or not isinstance(
            response.tool_calls, tuple
        ):
            return None
        if len(response.tool_calls) != 1:
            return None
        call = response.tool_calls[0]
        if not isinstance(call, _ToolCall):
            return None
        if call.name == 'read_runtime_snapshot':
            if type(call.arguments) is not dict or call.arguments != {}:
                return None
            snapshot = request.runtime_snapshot
            self._tool_outputs.append(
                _ToolOutput(
                    'read_runtime_snapshot',
                    (
                        ('runtime_instance_id', snapshot.runtime_instance_id),
                        ('admission_epoch', snapshot.admission_epoch),
                        ('operating_mode', snapshot.operating_mode),
                        ('availability', snapshot.availability),
                        ('gate_state', snapshot.gate_state),
                        ('supported_step_mask', snapshot.supported_step_mask),
                        ('max_steps', snapshot.max_steps),
                        ('named_place_ids', snapshot.named_place_ids),
                    ),
                )
            )
            return None
        if call.name == 'cancel_owned_mission':
            if type(call.arguments) is not dict or call.arguments != {}:
                return None
            owned = self._owned_mission
            if (
                owned is None
                or owned.agent_generation != self._agent_generation
            ):
                return None
            return _CancelAction(request, owned)
        if call.name != 'propose_mission' or type(call.arguments) is not dict:
            return None
        if set(call.arguments) != {'kind', 'steps'}:
            return None
        if call.arguments['kind'] != 'mission':
            return None
        steps = _decode_steps({'steps': call.arguments['steps']})
        if steps is None:
            return None
        proposal = MissionProposal(tuple(steps), request.token)
        validation = self._validator.validate(proposal, request.token)
        if validation.accepted:
            assert validation.mission is not None
            return _ProposeAction(request, validation.mission)
        return None

    def _take_latest_pending_locked(self) -> Optional[_ResponseRequest]:
        pending = self._pending
        self._pending = None
        if (
            pending is None
            or pending.token.local_generation != self._generation
        ):
            return None
        self._active = pending
        return pending

    def _commit_propose(self, action: _ProposeAction) -> None:
        """Prepare outside the lock and linearize Mission submission inside it."""
        prepared = self._mission_port.prepare_mission(action.mission)
        with self._lock:
            if not self._request_is_current_locked(action.request):
                return
            identity = self._mission_port.commit_mission(prepared)
            if identity is not None:
                self._owned_mission = _OwnedMission(
                    identity, self._agent_generation
                )

    def _commit_cancel(self, action: _CancelAction) -> None:
        """Prepare outside the lock and linearize exact owned cancellation."""
        prepared = self._mission_port.prepare_cancel(action.owned.identity)
        with self._lock:
            if (
                not self._request_is_current_locked(action.request)
                or self._owned_mission is not action.owned
                or action.owned.agent_generation != self._agent_generation
            ):
                return
            if not self._mission_port.is_active(action.owned.identity):
                self._owned_mission = None
                return
            self._mission_port.commit_cancel(prepared)
            self._owned_mission = None

    @staticmethod
    def _valid_text(value: object) -> bool:
        return isinstance(value, str) and 0 < len(value) <= 512
