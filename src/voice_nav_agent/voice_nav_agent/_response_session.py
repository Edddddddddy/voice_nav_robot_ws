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

_UNSET = object()


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
    adapter_generation: int = 0
    runtime_snapshot: object = None


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
    response_generation: int
    adapter_generation: int = 0
    round: int = 1  # noqa: A003 - frozen provider protocol field
    snapshot_output: Optional['_ToolOutput'] = None


@dataclass(frozen=True, slots=True)
class _OwnedMission:
    """The exact active Mission identity owned by this Agent instance."""

    identity: object
    agent_generation: int
    session_id: str


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
class _SnapshotContinuation:
    """One second and final provider round after a frozen snapshot read."""

    request: _ResponseRequest


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

    def accept_turn(
        self,
        turn: object,
        token: Optional[PlanningToken] = None,
        adapter_generation: int = 0,
    ) -> None:
        """Start one Response against a frozen trusted Runtime snapshot."""
        request: Optional[_ResponseRequest] = None
        snapshot = self._capture_runtime_snapshot()
        with self._lock:
            if not self._observe_newer_turn(turn):
                return
            self._invalidate_response(
                clear_clarification=(
                    not isinstance(turn, VoiceTurn)
                    or turn.kind != VoiceTurn.COMMAND
                )
            )
            if not self._valid_turn(turn):
                return
            assert isinstance(turn, VoiceTurn)
            if self._session_id is None:
                self._session_id = turn.session_id
            elif turn.session_id != self._session_id:
                self._session_id = turn.session_id
                self._clarification = None
                self._events.clear()
                self._tool_outputs.clear()
            if turn.kind != VoiceTurn.COMMAND:
                return
            if snapshot is None:
                return
            if token is None:
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
            elif not self._token_matches(turn, token, snapshot):
                return
            if not isinstance(adapter_generation, int) or adapter_generation < 0:
                return
            request = _ResponseRequest(
                turn,
                token,
                snapshot,
                self._clarification,
                self._agent_generation,
                self._generation,
                adapter_generation,
            )
            self._clarification = None
            if self._active is None:
                self._active = request
            else:
                self._pending = request
                request = None
        if request is not None:
            self._submit_provider(request)

    def invalidate(self, *, clear_clarification: bool = True) -> None:
        """Fence active provider work without cancelling a committed Mission."""
        with self._lock:
            self._invalidate_response(clear_clarification=clear_clarification)
            # A provider may not yet have admitted this active Session work
            # (notably a continuation).  Detach it before the next accepted
            # turn so that the latest request can submit immediately.
            self._active = None
            self._events.clear()

    def observe_invalid_turn(self, turn: object) -> bool:
        """Advance only the Response high-watermark for an invalid envelope."""
        with self._lock:
            if not self._observe_newer_turn(turn):
                return False
            self._invalidate_response()
            self._active = None
            self._events.clear()
            return True

    def provider_capacity_ready(self) -> None:
        """Retry only the current Response after the adapter becomes idle."""
        request: Optional[_ResponseRequest] = None
        with self._lock:
            active = self._active
            if (
                active is not None
                and active.response_generation == self._generation
                and active.agent_generation == self._agent_generation
            ):
                request = active
            else:
                self._active = None
                request = self._take_latest_pending_locked()
        if request is not None:
            self._submit_provider(request)

    def fail(self, request: object) -> None:
        """Convert one current provider transport/protocol failure into text."""
        self.complete(request, _ProviderResponse(kind='failure'))

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

    def _invalidate_response(self, *, clear_clarification: bool = True) -> None:
        self._generation += 1
        self._pending = None
        if clear_clarification:
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

    def complete(
        self,
        request: object,
        response: object,
        current_snapshot: object = _UNSET,
        *,
        skip_runtime_check: bool = False,
    ) -> None:
        """Apply a provider result only while its Response is current."""
        if current_snapshot is _UNSET:
            current_snapshot = self._capture_runtime_snapshot()
        action: Optional[object] = None
        next_request: Optional[_ResponseRequest] = None
        with self._lock:
            if request is not self._active or not isinstance(
                response, _ProviderResponse
            ):
                return
            assert isinstance(request, _ResponseRequest)
            self._active = None
            if self._request_is_current_locked(
                request, current_snapshot, skip_runtime_check
            ):
                action = self._apply_current_response_locked(request, response)
            if isinstance(action, _SnapshotContinuation):
                next_request = action.request
                self._active = next_request
            else:
                next_request = self._take_latest_pending_locked()
        if next_request is not None:
            self._submit_provider(next_request)
        if isinstance(action, _ProposeAction):
            self._commit_propose(action)
        elif isinstance(action, _CancelAction):
            self._commit_cancel(action)

    def _capture_runtime_snapshot(self) -> object:
        snapshot, _reason = _freeze_runtime_snapshot(self._runtime_snapshot())
        return snapshot

    def _request_is_current_locked(
        self,
        request: _ResponseRequest,
        current_snapshot: object,
        skip_runtime_check: bool = False,
    ) -> bool:
        if (
            request.response_generation != self._generation
            or request.agent_generation != self._agent_generation
        ):
            return False
        return skip_runtime_check or current_snapshot == request.runtime_snapshot

    def _apply_current_response_locked(
        self, request: _ResponseRequest, response: _ProviderResponse
    ) -> Optional[object]:
        if response.kind == 'clarify' and self._valid_text(response.text):
            self._clarification = response.text
            self._events.append(
                _ResponseEvent(
                    'clarify', request.turn.session_id, request.turn.turn_id,
                    response.text, request.adapter_generation,
                    request.runtime_snapshot,
                )
            )
            return None
        if response.kind == 'reply' and self._valid_text(response.text):
            self._events.append(
                _ResponseEvent(
                    'reply', request.turn.session_id, request.turn.turn_id,
                    response.text, request.adapter_generation,
                    request.runtime_snapshot,
                )
            )
            return None
        if response.kind == 'failure':
            return self._failure_event_locked(request)
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
            if request.round != 1:
                return self._failure_event_locked(request)
            snapshot = request.runtime_snapshot
            output = _ToolOutput(
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
            self._tool_outputs.append(output)
            return _SnapshotContinuation(
                _ResponseRequest(
                    request.turn,
                    request.token,
                    request.runtime_snapshot,
                    request.clarification,
                    request.agent_generation,
                    request.response_generation,
                    request.adapter_generation,
                    round=2,
                    snapshot_output=output,
                )
            )
        if call.name == 'cancel_owned_mission':
            if type(call.arguments) is not dict or call.arguments != {}:
                return None
            owned = self._owned_mission
            if (
                owned is None
                or owned.agent_generation != self._agent_generation
                or owned.session_id != request.turn.session_id
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

    def _failure_event_locked(self, request: _ResponseRequest) -> None:
        self._events.append(
            _ResponseEvent(
                'failure',
                request.turn.session_id,
                request.turn.turn_id,
                '当前无法处理该导航请求。',
                request.adapter_generation,
                request.runtime_snapshot,
            )
        )

    def _take_latest_pending_locked(self) -> Optional[_ResponseRequest]:
        pending = self._pending
        self._pending = None
        if (
            pending is None
            or pending.response_generation != self._generation
        ):
            return None
        self._active = pending
        return pending

    def _submit_provider(self, request: _ResponseRequest) -> None:
        """Submit only an active request; a full adapter wakes us when idle."""
        with self._lock:
            if (
                self._active is not request
                or request.response_generation != self._generation
                or request.agent_generation != self._agent_generation
            ):
                return
        accepted = self._provider.submit(request)
        if accepted is not False:
            return

    def _token_matches(
        self, turn: VoiceTurn, token: object, snapshot: object
    ) -> bool:
        if not isinstance(token, PlanningToken):
            return False
        return (
            token.source_instance_id == self._agent_instance_id
            and token.voice_instance_id == turn.voice_instance_id
            and token.voice_seq == turn.voice_seq
            and token.session_id == turn.session_id
            and token.turn_id == turn.turn_id
            and token.runtime_instance_id == snapshot.runtime_instance_id
            and token.admission_epoch == snapshot.admission_epoch
            and token.operating_mode == snapshot.operating_mode
            and token.supported_step_mask == snapshot.supported_step_mask
            and token.max_steps == snapshot.max_steps
            and token.named_place_ids == snapshot.named_place_ids
            and token.availability == snapshot.availability
            and token.gate_state == snapshot.gate_state
        )

    def _commit_propose(self, action: _ProposeAction) -> None:
        """Prepare outside the lock and linearize Mission submission inside it."""
        prepared = self._mission_port.prepare_mission(action.mission)
        guarded_commit = getattr(
            self._mission_port, 'commit_mission_if_current', None
        )
        if callable(guarded_commit):
            guarded_commit(prepared, action.request, None)
        else:
            snapshot = self._capture_runtime_snapshot()
            self.commit_prepared_mission_if_current(
                action.request,
                snapshot,
                lambda: self._mission_port.commit_mission(prepared),
            )

    def _commit_cancel(self, action: _CancelAction) -> None:
        """Prepare outside the lock and linearize exact owned cancellation."""
        prepared = self._mission_port.prepare_cancel(action.owned.identity)
        guarded_commit = getattr(
            self._mission_port, 'commit_cancel_if_current', None
        )
        if callable(guarded_commit):
            guarded_commit(prepared, action.request, action.owned, None)
        else:
            snapshot = self._capture_runtime_snapshot()
            active = self._mission_port.is_active(action.owned.identity)
            self.commit_prepared_cancel_if_current(
                action.request,
                action.owned,
                snapshot,
                (
                    (lambda: self._mission_port.commit_cancel(prepared) or True)
                    if active else lambda: False
                ),
            )

    def commit_prepared_mission_if_current(
        self,
        request: object,
        current_snapshot: object,
        commit: Callable[[], object],
    ) -> object:
        """Atomically fence a prepared Mission and retain its exact identity."""
        with self._lock:
            if not isinstance(request, _ResponseRequest) or not (
                self._request_is_current_locked(request, current_snapshot)
            ):
                return None
            identity = commit()
            if identity is not None:
                self._owned_mission = _OwnedMission(
                    identity, self._agent_generation, request.turn.session_id
                )
            return identity

    def commit_prepared_cancel_if_current(
        self,
        request: object,
        owned: object,
        current_snapshot: object,
        commit: Callable[[], bool],
    ) -> bool:
        """Atomically fence and clear only the exact owned Mission identity."""
        with self._lock:
            if (
                not isinstance(request, _ResponseRequest)
                or not isinstance(owned, _OwnedMission)
                or not self._request_is_current_locked(request, current_snapshot)
                or self._owned_mission is not owned
                or owned.agent_generation != self._agent_generation
                or owned.session_id != request.turn.session_id
            ):
                return False
            committed = commit()
            if committed:
                self._owned_mission = None
            return committed

    @staticmethod
    def _valid_text(value: object) -> bool:
        return isinstance(value, str) and 0 < len(value) <= 512
