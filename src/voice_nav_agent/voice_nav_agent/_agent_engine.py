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

"""One package-private deep module owning Agent planning state and fencing."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import threading
from typing import Callable, Optional

from ._planner import (
    LoopbackPlanner,
    PlannerPort,
    PlannerRequest,
    PlannerResponse,
)
from .core import (
    AgentCore,
    AgentPolicy,
    CancelDecision,
    ClarifyDecision,
    DecisionKind,
    IgnoreDecision,
    Mission,
    MissionDecision,
    MissionProposal,
    MissionState,
    PlanningToken,
    ReplyDecision,
    StopDecision,
    VoiceTurn,
)
from .planner_schema import (
    PLANNER_FAILURE_REASONS,
    SAFE_REPLY,
)


OUTCOME_KINDS = frozenset(
    {'reply', 'clarify', 'mission', 'cancel', 'stop', 'rejected'}
)


# Keep this deny list deliberately small: an action must be paired with a
# restricted capability, so ordinary mentions such as ``网络地图`` remain
# planner inputs.
_DENIED_CAPABILITY_PAIRS = (
    ('访问', ('网络', '互联网', '网页')),
    ('执行', ('shell', '命令', '终端', '脚本')),
    ('运行', ('shell', '命令', '终端', '脚本')),
)


def _is_denied_capability(turn: object) -> bool:
    """Return whether a COMMAND explicitly requests a forbidden capability."""
    if not isinstance(turn, VoiceTurn) or turn.kind != VoiceTurn.COMMAND:
        return False
    text = turn.text
    if not isinstance(text, str):
        return False
    normalized = ''.join(text.split()).lower()
    return any(
        action + target in normalized
        for action, targets in _DENIED_CAPABILITY_PAIRS
        for target in targets
    )


class _DeliveryLease:
    """Opaque identity handed to the ROS Adapter for one side-effect turn."""

    __slots__ = ()


class OutcomeKind(str, Enum):
    """Closed lowercase outcome vocabulary crossing the Engine boundary."""

    REPLY = 'reply'
    CLARIFY = 'clarify'
    MISSION = 'mission'
    CANCEL = 'cancel'
    STOP = 'stop'
    REJECTED = 'rejected'


@dataclass(frozen=True, slots=True)
class AgentOutcome:
    """Closed AgentEngine outcome crossing the ROS Adapter boundary."""

    kind: str
    text: str = ''
    mission: Optional[Mission] = None
    token: Optional[PlanningToken] = None
    source_instance_id: str = ''
    source_seq: int = 0
    session_id: str = ''
    turn_id: str = ''
    identity: object = None
    reason: str = ''
    generation: int = 0
    delivery_lease: Optional[_DeliveryLease] = None

    def __post_init__(self) -> None:
        if self.kind not in OUTCOME_KINDS:
            raise ValueError(f'unknown Agent outcome: {self.kind!r}')

    @classmethod
    def rejected(
        cls,
        reason: str,
        *,
        text: str = SAFE_REPLY,
        turn: Optional[VoiceTurn] = None,
        generation: int = 0,
    ) -> 'AgentOutcome':
        return cls(
            'rejected',
            text=text,
            session_id=turn.session_id if turn is not None else '',
            turn_id=turn.turn_id if turn is not None else '',
            reason=reason,
            generation=generation,
        )


@dataclass(frozen=True, slots=True)
class _OwnedMission:
    """Exact Mission identity retained by this Engine instance."""

    identity: object
    token: PlanningToken
    session_id: str


class AgentEngine:
    """Own all Agent generation, latest-wins, epoch and Mission fences."""

    def __init__(
        self,
        agent_instance_id: str,
        *,
        planner: Optional[PlannerPort] = None,
        planner_endpoint: str = 'http://127.0.0.1:8080',
        policy: Optional[AgentPolicy] = None,
        clock: Optional[Callable[[], float]] = None,
        on_outcome: Optional[Callable[[AgentOutcome, VoiceTurn], None]] = None,
    ) -> None:
        self._agent_instance_id = agent_instance_id
        self._policy = policy or AgentPolicy()
        self._core = AgentCore(
            agent_instance_id,
            policy=self._policy,
            clock=clock,
        )
        self._planner: PlannerPort = planner or LoopbackPlanner(planner_endpoint)
        self._on_outcome = on_outcome
        self._lock = threading.RLock()
        self._generation = 0
        self._agent_generation = 1
        self._runtime_snapshot: Optional[MissionState] = None
        self._clarification: Optional[str] = None
        self._active: Optional[PlannerRequest] = None
        self._owned_mission: Optional[_OwnedMission] = None
        self._delivery_leases: dict[
            _DeliveryLease, tuple[int, int]
        ] = {}
        self._closed = False

    def handle_turn(
        self,
        turn: VoiceTurn,
        runtime_snapshot: object,
    ) -> Optional[AgentOutcome]:
        """Classify a turn and either return an outcome or admit bounded planning."""
        decision = self._core.handle_turn(turn, runtime_snapshot)
        if not isinstance(decision, (IgnoreDecision, StopDecision)):
            self._fence_runtime_epoch(runtime_snapshot)
        if decision.kind is not DecisionKind.LLM_NEEDED:
            with self._lock:
                self._clarification = None

        if isinstance(decision, IgnoreDecision):
            generation = 0
            if self._is_newer_invalid_turn(turn):
                generation = self.invalidate('invalid_turn')
            outcome = AgentOutcome.rejected(
                decision.reason,
                turn=turn if isinstance(turn, VoiceTurn) else None,
                generation=generation,
            )
            return self._commit_immediate_outcome(outcome)

        if _is_denied_capability(turn):
            with self._lock:
                self._clarification = None
            generation = self.invalidate('denied_capability')
            outcome = AgentOutcome.rejected(
                'denied_capability', turn=turn, generation=generation
            )
            return self._emit_immediate_outcome(outcome, turn)

        generation = self._next_generation()
        self._planner.invalidate(generation)
        if isinstance(decision, StopDecision):
            outcome = AgentOutcome(
                'stop',
                source_instance_id=decision.source_instance_id,
                source_seq=decision.source_seq,
                session_id=getattr(turn, 'session_id', ''),
                turn_id=getattr(turn, 'turn_id', ''),
                reason=decision.reason,
                generation=generation,
            )
            return self._emit_immediate_outcome(outcome, turn)

        if isinstance(decision, MissionDecision):
            outcome = AgentOutcome(
                'mission',
                mission=decision.mission,
                token=decision.token,
                source_instance_id=decision.token.source_instance_id,
                source_seq=decision.token.source_seq,
                session_id=decision.token.session_id,
                turn_id=decision.token.turn_id,
                reason=decision.reason,
                generation=generation,
            )
            return self._emit_immediate_outcome(outcome, turn)

        if isinstance(decision, CancelDecision):
            with self._lock:
                owned = self._owned_mission
                valid_owned = (
                    owned is not None
                    and owned.session_id == decision.session_id
                )
            if not valid_owned:
                outcome = AgentOutcome.rejected(
                    'no_owned_mission', turn=turn
                )
            else:
                outcome = AgentOutcome(
                    'cancel',
                    source_instance_id=decision.source_instance_id,
                    source_seq=decision.source_seq,
                    session_id=decision.session_id,
                    turn_id=decision.turn_id,
                    identity=owned.identity,
                    reason=decision.reason,
                    generation=generation,
                )
            return self._emit_immediate_outcome(outcome, turn)

        if isinstance(decision, ClarifyDecision):
            outcome = AgentOutcome(
                'clarify',
                text=decision.prompt,
                source_instance_id=self._agent_instance_id,
                source_seq=0,
                session_id=decision.session_id,
                turn_id=decision.turn_id,
                reason=decision.reason,
                generation=generation,
            )
            return self._emit_immediate_outcome(outcome, turn)

        if isinstance(decision, ReplyDecision):
            outcome = AgentOutcome(
                'reply',
                text=decision.text,
                session_id=turn.session_id,
                turn_id=turn.turn_id,
                reason=decision.reason,
                generation=generation,
            )
            return self._emit_immediate_outcome(outcome, turn)

        if decision.kind is DecisionKind.LLM_NEEDED:
            with self._lock:
                clarification = self._clarification
            request = PlannerRequest(
                turn=turn,
                token=decision.token,
                runtime_snapshot=self._snapshot(runtime_snapshot),
                generation=generation,
                agent_generation=self._agent_generation,
                clarification=clarification,
            )
            with self._lock:
                self._clarification = None
            accepted = self._admit(request)
            if accepted:
                return None
            outcome = AgentOutcome.rejected(
                'planner_unavailable', turn=turn, generation=generation
            )
            return self._emit_immediate_outcome(outcome, turn)

        outcome = AgentOutcome.rejected('unsupported_decision', turn=turn)
        return self._emit_immediate_outcome(outcome, turn)

    def invalidate(self, reason: str = '') -> int:
        """Fence all planner work while retaining a committed Mission identity."""
        generation = self._next_generation()
        with self._lock:
            self._active = None
        self._planner.invalidate(generation)
        return generation

    def consume_delivery_lease(self, lease: object) -> bool:
        """Consume one current outcome lease without invoking external code."""
        if not isinstance(lease, _DeliveryLease):
            return False
        with self._lock:
            binding = self._delivery_leases.pop(lease, None)
            if binding is None:
                return False
            return (
                not self._closed
                and binding == (self._agent_generation, self._generation)
            )

    def observe_runtime_snapshot(self, runtime_snapshot: object) -> bool:
        """Fence planner work when the ROS adapter observes a new Runtime epoch."""
        snapshot = self._try_snapshot(runtime_snapshot)
        if snapshot is None:
            return False
        with self._lock:
            changed = (
                self._runtime_snapshot is not None
                and snapshot != self._runtime_snapshot
            )
            self._runtime_snapshot = snapshot
        if changed:
            self.invalidate('runtime_epoch_changed')
        return changed

    def restart(self, agent_instance_id: str) -> None:
        """Start a new Agent lifetime and revoke old planner/Mission ownership."""
        self.invalidate('restart')
        with self._lock:
            self._agent_generation += 1
            self._agent_instance_id = agent_instance_id
            self._owned_mission = None
            self._runtime_snapshot = None
            self._core = AgentCore(
                agent_instance_id,
                policy=self._policy,
            )

    def shutdown(self) -> None:
        """Close the planner once and prevent late outcomes from crossing the seam."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._active = None
            self._owned_mission = None
        generation = self._next_generation()
        self._planner.invalidate(generation)
        self._planner.shutdown()

    def record_owned_mission(
        self, outcome: AgentOutcome, identity: object
    ) -> bool:
        """Commit one external Mission handle only for the current outcome token."""
        with self._lock:
            if (
                self._closed
                or outcome.kind != 'mission'
                or outcome.mission is None
                or outcome.token is None
                or outcome.token.local_generation <= 0
                or outcome.generation != self._generation
            ):
                return False
            self._owned_mission = _OwnedMission(
                identity,
                outcome.token,
                outcome.session_id,
            )
            return True

    def record_cancelled_mission(self, identity: object) -> bool:
        """Clear ownership only for the exact opaque Mission identity."""
        with self._lock:
            if self._owned_mission is None or self._owned_mission.identity is not identity:
                return False
            self._owned_mission = None
            return True

    def _admit(self, request: PlannerRequest) -> bool:
        with self._lock:
            if self._closed:
                return False
            self._active = request
        accepted = self._planner.submit(request, self._on_planner_done)
        if accepted:
            return True
        with self._lock:
            if self._active is request:
                self._active = None
        return False

    def _commit_immediate_outcome(self, outcome: AgentOutcome) -> AgentOutcome:
        with self._lock:
            return replace(
                outcome,
                delivery_lease=self._new_delivery_lease_locked(),
            )

    def _emit_immediate_outcome(
        self, outcome: AgentOutcome, turn: VoiceTurn
    ) -> AgentOutcome:
        committed = self._commit_immediate_outcome(outcome)
        self._emit(committed, turn)
        return committed

    def _on_planner_done(
        self, request: PlannerRequest, response: PlannerResponse
    ) -> None:
        outcome = self._outcome_from_planner(request, response)
        if outcome is None:
            return
        committed = self._commit_outcome(request, outcome)
        if committed is None:
            return
        self._emit(committed, request.turn)

    def _commit_outcome(
        self, request: PlannerRequest, outcome: AgentOutcome
    ) -> Optional[AgentOutcome]:
        """Linearize freshness and callback handoff without invoking it under lock."""
        with self._lock:
            current = (
                not self._closed
                and self._active is request
                and request.generation == self._generation
                and request.agent_generation == self._agent_generation
                and self._runtime_snapshot == request.runtime_snapshot
            )
            if not current:
                return None
            self._active = None
            if outcome.kind == 'clarify':
                self._clarification = outcome.text
            return replace(
                outcome,
                delivery_lease=self._new_delivery_lease_locked(),
            )

    def _outcome_from_planner(
        self, request: PlannerRequest, response: PlannerResponse
    ) -> Optional[AgentOutcome]:
        if not isinstance(response, PlannerResponse):
            return AgentOutcome.rejected(
                'planner_response_type',
                turn=request.turn,
                generation=request.generation,
            )
        if response.kind == 'tool':
            return self._handle_tool(request, response)
        if response.kind in ('reply', 'clarify') and response.text:
            return AgentOutcome(
                response.kind,
                text=response.text,
                session_id=request.turn.session_id,
                turn_id=request.turn.turn_id,
                reason='planner',
                generation=request.generation,
            )
        if response.kind != 'mission':
            reason = (
                response.reason
                if response.reason in PLANNER_FAILURE_REASONS
                else 'transport'
            )
            return AgentOutcome.rejected(
                f'planner_invalid_{reason}',
                turn=request.turn,
                generation=request.generation,
            )
        proposal = MissionProposal(response.steps, request.token)
        validation = self._core.semantic_validator.validate(proposal, request.token)
        if not validation.accepted:
            return AgentOutcome.rejected(
                'planner_semantic_invalid',
                turn=request.turn,
                generation=request.generation,
            )
        assert validation.mission is not None
        return AgentOutcome(
            'mission',
            mission=validation.mission,
            token=request.token,
            source_instance_id=request.token.source_instance_id,
            source_seq=request.token.source_seq,
            session_id=request.turn.session_id,
            turn_id=request.turn.turn_id,
            reason='planner',
            generation=request.generation,
        )

    def _handle_tool(
        self, request: PlannerRequest, response: PlannerResponse
    ) -> Optional[AgentOutcome]:
        arguments = dict(response.tool_arguments)
        if response.tool_name == 'read_runtime_snapshot' and request.round == 1:
            continuation = PlannerRequest(
                turn=request.turn,
                token=request.token,
                runtime_snapshot=request.runtime_snapshot,
                generation=request.generation,
                agent_generation=request.agent_generation,
                clarification=request.clarification,
                round=2,
                snapshot_output=self._snapshot_value(request.runtime_snapshot),
            )
            with self._lock:
                if self._closed or request.generation != self._generation:
                    return None
                self._active = continuation
            if not self._planner.submit(continuation, self._on_planner_done):
                with self._lock:
                    if self._active is continuation:
                        self._active = request
                return AgentOutcome.rejected(
                    'planner_unavailable',
                    turn=request.turn,
                    generation=request.generation,
                )
            return None
        if response.tool_name == 'propose_mission':
            steps = arguments.get('steps')
            if arguments.get('kind') != 'mission' or not isinstance(steps, tuple):
                return AgentOutcome.rejected(
                    'planner_schema_invalid',
                    turn=request.turn,
                    generation=request.generation,
                )
            return self._outcome_from_planner(
                request, PlannerResponse.mission(steps)
            )
        if response.tool_name == 'cancel_owned_mission' and arguments == {}:
            with self._lock:
                owned = self._owned_mission
                valid = owned is not None and owned.session_id == request.turn.session_id
            if not valid:
                return AgentOutcome.rejected(
                    'no_owned_mission',
                    turn=request.turn,
                    generation=request.generation,
                )
            assert owned is not None
            return AgentOutcome(
                'cancel',
                identity=owned.identity,
                session_id=request.turn.session_id,
                turn_id=request.turn.turn_id,
                reason='planner',
                generation=request.generation,
            )
        return AgentOutcome.rejected(
            'planner_tool_invalid',
            turn=request.turn,
            generation=request.generation,
        )

    def _fence_runtime_epoch(self, runtime_snapshot: object) -> None:
        self.observe_runtime_snapshot(runtime_snapshot)

    def _snapshot(self, runtime_snapshot: object) -> MissionState:
        snapshot = self._try_snapshot(runtime_snapshot)
        if snapshot is None:
            raise ValueError('planner request requires a valid Runtime snapshot')
        with self._lock:
            self._runtime_snapshot = snapshot
        return snapshot

    @staticmethod
    def _try_snapshot(value: object) -> Optional[MissionState]:
        return value if isinstance(value, MissionState) else None

    @staticmethod
    def _snapshot_value(snapshot: MissionState) -> dict[str, object]:
        return {
            'runtime_instance_id': snapshot.runtime_instance_id,
            'admission_epoch': snapshot.admission_epoch,
            'operating_mode': snapshot.operating_mode,
            'availability': snapshot.availability,
            'gate_state': snapshot.gate_state,
            'active_step': snapshot.active_step,
            'supported_step_mask': snapshot.supported_step_mask,
            'max_steps': snapshot.max_steps,
            'named_place_ids': snapshot.named_place_ids,
        }

    def _next_generation(self) -> int:
        with self._lock:
            self._generation += 1
            self._delivery_leases.clear()
            return self._generation

    def _new_delivery_lease_locked(self) -> _DeliveryLease:
        lease = _DeliveryLease()
        self._delivery_leases[lease] = (
            self._agent_generation,
            self._generation,
        )
        return lease

    def _is_newer_invalid_turn(self, turn: object) -> bool:
        # AgentCore performs the authoritative Voice fence.  Invalid envelopes
        # are still fenced locally so an older planner callback cannot deliver.
        return isinstance(turn, VoiceTurn)

    def _emit(self, outcome: AgentOutcome, turn: VoiceTurn) -> None:
        callback = self._on_outcome
        if callback is not None:
            callback(outcome, turn)


# Package-private aliases make the intended boundary obvious to tests without
# exposing an additional public orchestration facade.
_AgentEngine = AgentEngine
_AgentOutcome = AgentOutcome
