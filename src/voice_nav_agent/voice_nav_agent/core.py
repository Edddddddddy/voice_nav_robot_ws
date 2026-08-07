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

"""Deterministic, side-effect-free VoiceNav Agent Core."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from enum import Enum, IntEnum
import math
import re
import struct
import time
from typing import Callable, ClassVar, Mapping, Optional, Union
import unicodedata


MAX_UINT64 = (1 << 64) - 1
MAX_UINT32 = (1 << 32) - 1
MAX_RETIRED_VOICE_INSTANCES = 64
PLACE_ID_PATTERN = re.compile(r'^[a-z][a-z0-9_-]{0,31}$')
_RUNTIME_ROTATE_ANGLE_MAX_RAD = struct.unpack(
    '<f', struct.pack('<f', 6.283185)
)[0]
_CLAUSE_BOUNDARY = re.compile(
    r'(?:[，,；;。！!?、]\s*(?:然后|再)?|(?:然后|再))'
)
_SENTENCE_TERMINATORS = frozenset('。！!?')


class DecisionKind(str, Enum):
    """Closed set of outcomes produced by :class:`AgentCore`."""

    MISSION = 'MISSION'
    CANCEL = 'CANCEL'
    STOP = 'STOP'
    CLARIFY = 'CLARIFY'
    REPLY = 'REPLY'
    LLM_NEEDED = 'LLM_NEEDED'
    IGNORE = 'IGNORE'


class OperatingMode(IntEnum):
    """Operating modes copied from the stable MissionState Interface."""

    MAPPING = 1
    NAVIGATION = 2


class Availability(IntEnum):
    """Runtime availability values copied from MissionState."""

    UNAVAILABLE = 0
    AVAILABLE = 1
    BUSY = 2
    FAULTED = 3


class GateState(IntEnum):
    """Motion Gate states copied from MissionState."""

    GATE_INHIBITED = 0
    GATE_ARMED = 1
    GATE_FAULTED = 2


@dataclass(frozen=True, slots=True)
class VoiceTurn:
    """Immutable Core copy of a bounded formal VoiceTurn message."""

    COMMAND: ClassVar[int] = 1
    STOP: ClassVar[int] = 2

    voice_instance_id: str = ''
    voice_seq: int = 0
    session_id: str = ''
    turn_id: str = ''
    kind: int = COMMAND
    text: str = ''
    confidence: float = 0.0
    during_playback: bool = False


@dataclass(frozen=True, slots=True)
class MissionState:
    """Immutable Core copy of a Mission Runtime state snapshot."""

    MAPPING: ClassVar[int] = int(OperatingMode.MAPPING)
    NAVIGATION: ClassVar[int] = int(OperatingMode.NAVIGATION)
    UNAVAILABLE: ClassVar[int] = int(Availability.UNAVAILABLE)
    AVAILABLE: ClassVar[int] = int(Availability.AVAILABLE)
    BUSY: ClassVar[int] = int(Availability.BUSY)
    FAULTED: ClassVar[int] = int(Availability.FAULTED)
    GATE_INHIBITED: ClassVar[int] = int(GateState.GATE_INHIBITED)
    GATE_ARMED: ClassVar[int] = int(GateState.GATE_ARMED)
    GATE_FAULTED: ClassVar[int] = int(GateState.GATE_FAULTED)

    runtime_instance_id: str = ''
    admission_epoch: int = 0
    operating_mode: int = int(OperatingMode.MAPPING)
    availability: int = int(Availability.UNAVAILABLE)
    gate_state: int = int(GateState.GATE_FAULTED)
    active_step: int = MAX_UINT32
    supported_step_mask: int = 0
    max_steps: int = 3
    named_place_ids: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        """Copy mutable sequence inputs into the immutable snapshot."""
        if isinstance(self.named_place_ids, (list, tuple)):
            object.__setattr__(self, 'named_place_ids', tuple(self.named_place_ids))


@dataclass(frozen=True, slots=True)
class MissionStep:
    """Closed Mission step union used by proposals and validated Missions."""

    MOVE_DISTANCE: ClassVar[int] = 1
    ROTATE_ANGLE: ClassVar[int] = 2
    NAVIGATE_TO: ClassVar[int] = 3
    SAVE_MAP: ClassVar[int] = 4

    kind: int
    distance_m: float = 0.0
    angle_rad: float = 0.0
    target_id: str = ''


@dataclass(frozen=True, slots=True)
class AgentPolicy:
    """Trusted semantic policy mirrored from Mission Runtime."""

    max_steps: int = 3
    move_distance_min_m: float = 0.05
    move_distance_max_m: float = 2.0
    rotate_angle_min_rad: float = 0.05
    rotate_angle_max_rad: float = _RUNTIME_ROTATE_ANGLE_MAX_RAD
    clarification_timeout_s: float = 15.0
    clarification_capacity: int = 64


@dataclass(frozen=True, slots=True)
class PlanningToken:
    """Immutable identity and Runtime snapshot carried by a plan."""

    source_instance_id: str
    source_seq: int
    voice_instance_id: str
    voice_seq: int
    session_id: str
    turn_id: str
    local_generation: int
    runtime_instance_id: str
    admission_epoch: int
    operating_mode: int
    supported_step_mask: int
    max_steps: int
    named_place_ids: tuple[str, ...]
    availability: int = int(Availability.AVAILABLE)
    gate_state: int = int(GateState.GATE_INHIBITED)

    def __post_init__(self) -> None:
        """Copy mutable place collections into the token."""
        if isinstance(self.named_place_ids, (list, tuple)):
            object.__setattr__(self, 'named_place_ids', tuple(self.named_place_ids))

    @property
    def runtime_id(self) -> str:
        """Return the Runtime identity using the short contract name."""
        return self.runtime_instance_id

    @property
    def epoch(self) -> int:
        """Return the admission epoch using the short contract name."""
        return self.admission_epoch

    @property
    def mode(self) -> int:
        """Return the operating mode using the short contract name."""
        return self.operating_mode

    @property
    def capabilities(self) -> int:
        """Return the supported Mission step mask."""
        return self.supported_step_mask

    @property
    def named_places(self) -> tuple[str, ...]:
        """Return the immutable Named Place IDs."""
        return self.named_place_ids


ImmutablePlanningContext = PlanningToken
PlanningContext = PlanningToken


@dataclass(frozen=True, slots=True)
class MissionProposal:
    """Typed Mission proposal awaiting semantic validation."""

    steps: tuple[MissionStep, ...]
    token: PlanningToken

    def __post_init__(self) -> None:
        """Copy mutable step collections into the proposal."""
        if isinstance(self.steps, (list, tuple)):
            object.__setattr__(self, 'steps', tuple(self.steps))

    @property
    def planning_token(self) -> PlanningToken:
        """Return the proposal's immutable planning token."""
        return self.token


@dataclass(frozen=True, slots=True)
class Mission:
    """Semantically validated typed Mission."""

    steps: tuple[MissionStep, ...]
    token: PlanningToken

    def __post_init__(self) -> None:
        """Copy mutable step collections into the validated Mission."""
        if isinstance(self.steps, (list, tuple)):
            object.__setattr__(self, 'steps', tuple(self.steps))

    @property
    def planning_token(self) -> PlanningToken:
        """Return the Mission's immutable planning token."""
        return self.token


@dataclass(frozen=True, slots=True)
class ValidationRejection:
    """Structured semantic rejection for an untrusted proposal."""

    reason: str
    detail: str


@dataclass(frozen=True, slots=True)
class ValidationResult:
    """Closed result of semantic validation."""

    mission: Optional[Mission] = None
    rejection: Optional[ValidationRejection] = None

    @property
    def accepted(self) -> bool:
        """Return whether a validated Mission was produced."""
        return self.mission is not None and self.rejection is None

    @property
    def ok(self) -> bool:
        """Return the accepted status under a concise test-friendly name."""
        return self.accepted


class SemanticValidator:
    """Validate deterministic and future LLM proposals through one seam."""

    def __init__(self, policy: Optional[AgentPolicy] = None):
        """Create a validator using the trusted Runtime-mirrored policy."""
        self._policy = policy or AgentPolicy()

    def validate(
        self,
        proposal: MissionProposal,
        immutable_planning_context: PlanningToken,
    ) -> ValidationResult:
        """Return a validated Mission or a structured semantic rejection."""
        if not isinstance(proposal, MissionProposal):
            return self._reject('invalid_proposal', 'proposal_type')
        if not isinstance(proposal.token, PlanningToken):
            return self._reject('invalid_planning_token', 'token_type')
        if not isinstance(immutable_planning_context, PlanningToken):
            return self._reject('invalid_planning_context', 'context_type')
        if proposal.token != immutable_planning_context:
            return self._reject(
                'planning_context_mismatch', 'token_is_not_current_context'
            )

        token_reason = self._validate_token(proposal.token)
        if token_reason is not None:
            return self._reject(token_reason, token_reason)
        if not isinstance(proposal.steps, tuple):
            return self._reject('invalid_steps', 'steps_must_be_immutable')
        if not 1 <= len(proposal.steps) <= 3:
            return self._reject('invalid_step_count', 'steps_must_be_one_to_three')
        if len(proposal.steps) > proposal.token.max_steps:
            return self._reject('step_count_exceeds_snapshot', 'max_steps')
        if len(proposal.steps) > self._policy.max_steps:
            return self._reject('step_count_exceeds_policy', 'max_steps')

        for step in proposal.steps:
            reason = self._validate_step(step, proposal.token)
            if reason is not None:
                return self._reject(reason, reason)

        return ValidationResult(mission=Mission(proposal.steps, proposal.token))

    def _validate_token(self, token: PlanningToken) -> Optional[str]:
        if not _bounded_id(token.source_instance_id, 36):
            return 'invalid_source_instance_id'
        if not _uint64(token.source_seq) or token.source_seq == 0:
            return 'invalid_source_sequence'
        if not _bounded_id(token.voice_instance_id, 36):
            return 'invalid_voice_instance_id'
        if not _uint64(token.voice_seq):
            return 'invalid_voice_sequence'
        if not _bounded_id(token.session_id, 36):
            return 'invalid_session_id'
        if not _bounded_id(token.turn_id, 36):
            return 'invalid_turn_id'
        if not _uint64(token.local_generation) or token.local_generation == 0:
            return 'invalid_local_generation'
        if not _bounded_id(token.runtime_instance_id, 36):
            return 'invalid_runtime_instance_id'
        if not _uint64(token.admission_epoch) or token.admission_epoch == 0:
            return 'invalid_admission_epoch'
        if token.operating_mode not in (
            OperatingMode.MAPPING,
            OperatingMode.NAVIGATION,
        ):
            return 'invalid_operating_mode'
        if not _uint8(token.supported_step_mask) or token.supported_step_mask > 0x0F:
            return 'invalid_capability_mask'
        if not _uint8(token.max_steps) or not 1 <= token.max_steps <= 3:
            return 'invalid_max_steps'
        if token.availability != Availability.AVAILABLE:
            return 'runtime_not_available'
        if token.gate_state != GateState.GATE_INHIBITED:
            return 'gate_not_inhibited'
        if not isinstance(token.named_place_ids, tuple):
            return 'invalid_named_places'
        if len(token.named_place_ids) > 32:
            return 'too_many_named_places'
        if any(not isinstance(place, str) for place in token.named_place_ids):
            return 'invalid_named_place_id'
        if len(set(token.named_place_ids)) != len(token.named_place_ids):
            return 'duplicate_named_place'
        if any(not _valid_logical_id(place) for place in token.named_place_ids):
            return 'invalid_named_place_id'
        return None

    def _validate_step(
        self, step: MissionStep, token: PlanningToken
    ) -> Optional[str]:
        if not isinstance(step, MissionStep):
            return 'invalid_step'
        if not _uint8(step.kind):
            return 'unknown_step_kind'
        if step.kind not in (
            MissionStep.MOVE_DISTANCE,
            MissionStep.ROTATE_ANGLE,
            MissionStep.NAVIGATE_TO,
            MissionStep.SAVE_MAP,
        ):
            return 'unknown_step_kind'
        if not _finite(step.distance_m) or not _finite(step.angle_rad):
            return 'non_finite_step'
        if not isinstance(step.target_id, str):
            return 'invalid_target_id'
        if step.kind == MissionStep.MOVE_DISTANCE:
            if step.angle_rad != 0.0 or step.target_id != '':
                return 'invalid_union'
            if not _within_signed_range(
                step.distance_m,
                self._policy.move_distance_min_m,
                self._policy.move_distance_max_m,
            ):
                return 'distance_out_of_range'
        elif step.kind == MissionStep.ROTATE_ANGLE:
            if step.distance_m != 0.0 or step.target_id != '':
                return 'invalid_union'
            if not _within_angle_wire_range(
                step.angle_rad,
                self._policy.rotate_angle_min_rad,
                self._policy.rotate_angle_max_rad,
            ):
                return 'angle_out_of_range'
        elif step.kind == MissionStep.NAVIGATE_TO:
            if step.distance_m != 0.0 or step.angle_rad != 0.0:
                return 'invalid_union'
            if token.operating_mode != OperatingMode.NAVIGATION:
                return 'mode_mismatch'
            if not _valid_logical_id(step.target_id):
                return 'invalid_place_id'
            if step.target_id not in token.named_place_ids:
                return 'unknown_place'
        else:
            if step.distance_m != 0.0 or step.angle_rad != 0.0:
                return 'invalid_union'
            if token.operating_mode != OperatingMode.MAPPING:
                return 'mode_mismatch'
            if not _valid_logical_id(step.target_id):
                return 'invalid_map_id'

        capability = 1 << (int(step.kind) - 1)
        if token.supported_step_mask & capability == 0:
            return 'unsupported_step'
        return None

    @staticmethod
    def _reject(reason: str, detail: str) -> ValidationResult:
        return ValidationResult(rejection=ValidationRejection(reason, detail))


AgentSemanticValidator = SemanticValidator


@dataclass(frozen=True, slots=True)
class MissionDecision:
    """Decision carrying a semantically validated Mission."""

    kind: ClassVar[DecisionKind] = DecisionKind.MISSION
    mission: Mission
    reason: str = 'rule'

    @property
    def proposal(self) -> Mission:
        """Return the validated Mission under the proposal vocabulary."""
        return self.mission

    @property
    def steps(self) -> tuple[MissionStep, ...]:
        """Expose validated steps at the Decision boundary."""
        return self.mission.steps

    @property
    def token(self) -> PlanningToken:
        """Expose the Mission planning token at the Decision boundary."""
        return self.mission.token


@dataclass(frozen=True, slots=True)
class CancelDecision:
    """Decision to cancel the current local Agent Mission handle."""

    kind: ClassVar[DecisionKind] = DecisionKind.CANCEL
    source_instance_id: str
    source_seq: int
    session_id: str
    turn_id: str
    local_generation: int
    reason: str = 'local_cancel'


@dataclass(frozen=True, slots=True)
class StopDecision:
    """Operational Stop decision using Voice producer identity."""

    kind: ClassVar[DecisionKind] = DecisionKind.STOP
    request_id: str
    source_instance_id: str
    source_seq: int
    reason: str = 'voice_stop'


@dataclass(frozen=True, slots=True)
class ClarifyDecision:
    """Bounded clarification request for one missing parameter."""

    kind: ClassVar[DecisionKind] = DecisionKind.CLARIFY
    session_id: str
    turn_id: str
    local_generation: int
    reason: str
    prompt: str

    @property
    def text(self) -> str:
        """Return the clarification prompt as reply-compatible text."""
        return self.prompt


@dataclass(frozen=True, slots=True)
class ReplyDecision:
    """Structured safe reply with no Mission side effect."""

    kind: ClassVar[DecisionKind] = DecisionKind.REPLY
    reason: str
    text: str

    @property
    def detail(self) -> str:
        """Return the safe reply text under the Runtime vocabulary."""
        return self.text


@dataclass(frozen=True, slots=True)
class LLMNeededDecision:
    """Decision requesting the future bounded local LLM path."""

    kind: ClassVar[DecisionKind] = DecisionKind.LLM_NEEDED
    token: PlanningToken
    normalized_text: str
    reason: str = 'unsupported_expression'

    @property
    def planning_token(self) -> PlanningToken:
        """Return the immutable token that fences a future LLM result."""
        return self.token

    @property
    def text(self) -> str:
        """Return normalized text under a concise future-LLM vocabulary."""
        return self.normalized_text


@dataclass(frozen=True, slots=True)
class IgnoreDecision:
    """No-op decision for bad envelopes and replayed Voice commands."""

    kind: ClassVar[DecisionKind] = DecisionKind.IGNORE
    reason: str


Decision = Union[
    MissionDecision,
    CancelDecision,
    StopDecision,
    ClarifyDecision,
    ReplyDecision,
    LLMNeededDecision,
    IgnoreDecision,
]


@dataclass(frozen=True, slots=True)
class _PendingIntent:
    session_id: str
    parameter: str
    operation: str
    sign: int = 1
    prefix_steps: tuple[MissionStep, ...] = ()
    suffix_steps: tuple[MissionStep, ...] = ()
    created_at: float = 0.0


@dataclass(frozen=True, slots=True)
class _ClauseResult:
    kind: str
    step: Optional[MissionStep] = None
    pending: Optional[_PendingIntent] = None
    reason: str = ''


@dataclass(frozen=True, slots=True)
class _ParseResult:
    kind: str
    steps: tuple[MissionStep, ...] = ()
    pending: Optional[_PendingIntent] = None
    reason: str = ''


@dataclass(frozen=True, slots=True)
class _Envelope:
    voice_instance_id: str
    voice_seq: int
    session_id: str
    turn_id: str
    kind: int
    text: str
    confidence: float


class AgentCore:
    """Pure Agent Core that maps formal Voice Turns to typed Decisions."""

    def __init__(
        self,
        agent_instance_id: str,
        policy: Optional[AgentPolicy] = None,
        clock: Optional[Union[Callable[[], float], object]] = None,
        validator: Optional[SemanticValidator] = None,
        *,
        trusted_policy: Optional[AgentPolicy] = None,
        steady_clock: Optional[Union[Callable[[], float], object]] = None,
    ):
        """Create a Core with trusted policy and an injectable steady clock."""
        if not _bounded_id(agent_instance_id, 36):
            raise ValueError('agent_instance_id must be a non-empty bounded ID')
        if trusted_policy is not None:
            if policy is not None and policy != trusted_policy:
                raise ValueError('policy and trusted_policy disagree')
            policy = trusted_policy
        if steady_clock is not None:
            if clock is not None and clock != steady_clock:
                raise ValueError('clock and steady_clock disagree')
            clock = steady_clock
        self._agent_instance_id = agent_instance_id
        self._policy = policy or AgentPolicy()
        self._validator = validator or SemanticValidator(self._policy)
        self._clock = clock or time.monotonic
        self._source_seq = 0
        self._local_generation = 0
        self._current_voice_instance: Optional[str] = None
        self._last_voice_seq = -1
        self._retired_voice_instances: set[str] = set()
        self._voice_fencing_latched = False
        self._pending: dict[str, _PendingIntent] = {}

    @property
    def validator(self) -> SemanticValidator:
        """Return the shared semantic Validator used by deterministic rules."""
        return self._validator

    @property
    def semantic_validator(self) -> SemanticValidator:
        """Return the shared Validator under its full contract name."""
        return self._validator

    def handle_turn(
        self,
        turn: object,
        runtime_snapshot_or_none: object = None,
    ) -> Decision:
        """Handle one Voice Turn against one immutable planning snapshot."""
        self._expire_pending()
        envelope = self._read_envelope(turn)
        if envelope is None:
            return IgnoreDecision('invalid_envelope')

        normalized = _normalize(envelope.text)
        if envelope.kind == VoiceTurn.STOP or _contains_stop_clause(normalized):
            self._observe_stop_voice(envelope)
            return StopDecision(
                request_id=envelope.turn_id,
                source_instance_id=envelope.voice_instance_id,
                source_seq=envelope.voice_seq,
            )

        fence_decision = self._fence_command(envelope)
        if fence_decision is not None:
            return fence_decision

        allocation = self._allocate_command()
        if allocation is None:
            return ReplyDecision(
                'source_sequence_exhausted', '请重启 Agent 后再试。'
            )
        source_seq, local_generation = allocation
        snapshot, snapshot_reason = _freeze_runtime_snapshot(
            runtime_snapshot_or_none
        )
        pending = self._pending.pop(envelope.session_id, None)
        plain_text = _strip_invocation(normalized)

        if plain_text == '取消任务':
            return CancelDecision(
                source_instance_id=self._agent_instance_id,
                source_seq=source_seq,
                session_id=envelope.session_id,
                turn_id=envelope.turn_id,
                local_generation=local_generation,
            )

        if snapshot is None:
            return ReplyDecision(snapshot_reason, _reply_text(snapshot_reason))

        token = PlanningToken(
            source_instance_id=self._agent_instance_id,
            source_seq=source_seq,
            voice_instance_id=envelope.voice_instance_id,
            voice_seq=envelope.voice_seq,
            session_id=envelope.session_id,
            turn_id=envelope.turn_id,
            local_generation=local_generation,
            runtime_instance_id=snapshot.runtime_instance_id,
            admission_epoch=snapshot.admission_epoch,
            operating_mode=snapshot.operating_mode,
            supported_step_mask=snapshot.supported_step_mask,
            max_steps=snapshot.max_steps,
            named_place_ids=snapshot.named_place_ids,
            availability=snapshot.availability,
            gate_state=snapshot.gate_state,
        )
        parsed = self._parse_text(plain_text, snapshot)

        if parsed.kind == 'unknown' and pending is not None:
            answer = self._answer_pending(plain_text, pending, snapshot)
            if answer.kind == 'complete':
                parsed = _ParseResult('complete', steps=answer.steps)
            elif answer.kind == 'clarify':
                pending = answer.pending
                if not self._store_pending(pending):
                    return ReplyDecision(
                        'clarification_capacity_exhausted',
                        _reply_text('clarification_capacity_exhausted'),
                    )
                return self._clarification(
                    envelope, local_generation, pending.parameter
                )

        if parsed.kind == 'cancel':
            return CancelDecision(
                source_instance_id=self._agent_instance_id,
                source_seq=source_seq,
                session_id=envelope.session_id,
                turn_id=envelope.turn_id,
                local_generation=local_generation,
            )
        if parsed.kind == 'missing':
            pending = parsed.pending
            if pending is not None:
                pending = _PendingIntent(
                    session_id=envelope.session_id,
                    parameter=pending.parameter,
                    operation=pending.operation,
                    sign=pending.sign,
                    prefix_steps=pending.prefix_steps,
                    suffix_steps=pending.suffix_steps,
                )
            if pending is None or not self._store_pending(pending):
                return ReplyDecision(
                    'clarification_capacity_exhausted',
                    _reply_text('clarification_capacity_exhausted'),
                )
            return self._clarification(envelope, local_generation, pending.parameter)
        if parsed.kind == 'invalid':
            return ReplyDecision(parsed.reason, _reply_text(parsed.reason))
        if parsed.kind == 'unknown':
            return LLMNeededDecision(token, _normalize(envelope.text))

        proposal = MissionProposal(parsed.steps, token)
        validation = self._validator.validate(proposal, token)
        if not validation.accepted:
            assert validation.rejection is not None
            return ReplyDecision(
                validation.rejection.reason,
                _reply_text(validation.rejection.reason),
            )
        assert validation.mission is not None
        return MissionDecision(validation.mission)

    def _read_envelope(self, turn: object) -> Optional[_Envelope]:
        try:
            envelope = _Envelope(
                voice_instance_id=_read_value(turn, 'voice_instance_id'),
                voice_seq=_read_value(turn, 'voice_seq'),
                session_id=_read_value(turn, 'session_id'),
                turn_id=_read_value(turn, 'turn_id'),
                kind=_read_value(turn, 'kind'),
                text=_read_value(turn, 'text'),
                confidence=_read_value(turn, 'confidence'),
            )
        except (AttributeError, KeyError, TypeError):
            return None
        if not _bounded_id(envelope.voice_instance_id, 36):
            return None
        if not _uint64(envelope.voice_seq):
            return None
        if not _bounded_id(envelope.session_id, 36):
            return None
        if not _bounded_id(envelope.turn_id, 36):
            return None
        if envelope.kind not in (VoiceTurn.COMMAND, VoiceTurn.STOP):
            return None
        if not isinstance(envelope.text, str) or len(envelope.text) > 512:
            return None
        if not _finite(envelope.confidence) or not 0.0 <= envelope.confidence <= 1.0:
            return None
        return envelope

    def _fence_command(self, envelope: _Envelope) -> Optional[Decision]:
        if self._voice_fencing_latched:
            return ReplyDecision(
                'voice_instance_capacity_exhausted',
                _reply_text('voice_instance_capacity_exhausted'),
            )
        instance = envelope.voice_instance_id
        if self._current_voice_instance is None:
            self._current_voice_instance = instance
            self._last_voice_seq = envelope.voice_seq
            return None
        if instance == self._current_voice_instance:
            if envelope.voice_seq <= self._last_voice_seq:
                return IgnoreDecision('duplicate_or_replayed_command')
            self._last_voice_seq = envelope.voice_seq
            return None
        if instance in self._retired_voice_instances:
            return IgnoreDecision('retired_voice_instance')
        if len(self._retired_voice_instances) >= MAX_RETIRED_VOICE_INSTANCES:
            self._voice_fencing_latched = True
            return ReplyDecision(
                'voice_instance_capacity_exhausted',
                _reply_text('voice_instance_capacity_exhausted'),
            )
        self._retired_voice_instances.add(self._current_voice_instance)
        self._current_voice_instance = instance
        self._last_voice_seq = envelope.voice_seq
        self._pending.clear()
        self._rotate_generation()
        if len(self._retired_voice_instances) >= MAX_RETIRED_VOICE_INSTANCES:
            self._voice_fencing_latched = True
        return None

    def _observe_stop_voice(self, envelope: _Envelope) -> None:
        if self._voice_fencing_latched:
            self._pending.clear()
            self._rotate_generation()
            return
        instance = envelope.voice_instance_id
        if self._current_voice_instance is None:
            self._current_voice_instance = instance
            self._last_voice_seq = envelope.voice_seq
        elif instance == self._current_voice_instance:
            self._last_voice_seq = max(self._last_voice_seq, envelope.voice_seq)
        elif instance not in self._retired_voice_instances:
            if len(self._retired_voice_instances) < MAX_RETIRED_VOICE_INSTANCES:
                self._retired_voice_instances.add(self._current_voice_instance)
                self._current_voice_instance = instance
                self._last_voice_seq = envelope.voice_seq
                if len(self._retired_voice_instances) >= MAX_RETIRED_VOICE_INSTANCES:
                    self._voice_fencing_latched = True
        self._pending.clear()
        self._rotate_generation()

    def _allocate_command(self) -> Optional[tuple[int, int]]:
        if self._source_seq >= MAX_UINT64:
            return None
        if self._local_generation >= MAX_UINT64:
            return None
        self._source_seq += 1
        self._local_generation += 1
        return self._source_seq, self._local_generation

    def _rotate_generation(self) -> None:
        if self._local_generation < MAX_UINT64:
            self._local_generation += 1

    def _parse_text(self, text: str, state: MissionState) -> _ParseResult:
        if not text:
            return _ParseResult('invalid', reason='empty_command')
        if text == '取消任务':
            return _ParseResult('cancel')
        clauses = [clause.strip() for clause in _split_clauses(text)]
        if clauses and not clauses[-1] and _has_single_terminal_ending(text):
            clauses.pop()
        if len(clauses) > 3:
            return _ParseResult('invalid', reason='too_many_steps')
        if any(not clause for clause in clauses):
            return _ParseResult('invalid', reason='empty_clause')

        clause_results: list[_ClauseResult] = []
        for clause in clauses:
            result = self._parse_clause(clause, state)
            clause_results.append(result)
        unknowns = [result for result in clause_results if result.kind == 'unknown']
        if unknowns and len(clause_results) > 1:
            return _ParseResult('invalid', reason='mixed_unknown_rule')

        invalids = [result for result in clause_results if result.kind == 'invalid']
        if invalids:
            return _ParseResult('invalid', reason=invalids[0].reason)

        if any(result.kind == 'cancel' for result in clause_results):
            return _ParseResult('invalid', reason='mixed_cancel')

        if unknowns:
            return _ParseResult('unknown')

        missing = [result.pending for result in clause_results
                   if result.kind == 'missing']
        missing = [pending for pending in missing if pending is not None]

        if missing:
            if len(missing) > 1:
                return _ParseResult('invalid', reason='multiple_missing_parameters')
            index = next(
                index
                for index, result in enumerate(clause_results)
                if result.kind == 'missing'
            )
            pending = missing[0]
            pending = _PendingIntent(
                session_id='',
                parameter=pending.parameter,
                operation=pending.operation,
                sign=pending.sign,
                prefix_steps=tuple(
                    result.step
                    for result in clause_results[:index]
                    if result.step is not None
                ),
                suffix_steps=tuple(
                    result.step
                    for result in clause_results[index + 1:]
                    if result.step is not None
                ),
            )
            return _ParseResult('missing', pending=pending)
        steps = tuple(
            result.step
            for result in clause_results
            if result.step is not None
        )
        return _ParseResult('complete', steps=steps)

    def _parse_clause(
        self, clause: str, state: MissionState
    ) -> _ClauseResult:
        if clause == '取消任务':
            return _ClauseResult('cancel')

        movement = re.fullmatch(
            r'(前进|向前走|后退)\s*(\S+)\s*米', clause
        )
        if movement:
            value = _parse_number(movement.group(2))
            if value is None:
                return _ClauseResult('invalid', reason='invalid_number')
            sign = -1 if movement.group(1) == '后退' else 1
            return _ClauseResult(
                'step',
                step=MissionStep(
                    MissionStep.MOVE_DISTANCE,
                    distance_m=sign * value,
                ),
            )
        if any(clause.startswith(prefix) for prefix in ('前进', '向前走', '后退')):
            prefix = next(
                prefix
                for prefix in ('前进', '向前走', '后退')
                if clause.startswith(prefix)
            )
            remainder = clause[len(prefix):].strip()
            if not remainder or remainder == '米':
                sign = -1 if prefix == '后退' else 1
                return _ClauseResult(
                    'missing',
                    pending=_PendingIntent(
                        session_id='',
                        parameter='missing_distance',
                        operation='move',
                        sign=sign,
                    ),
                )
            value = _parse_number(remainder)
            if value is not None:
                if _within_signed_range(
                    value,
                    self._policy.move_distance_min_m,
                    self._policy.move_distance_max_m,
                ):
                    sign = -1 if prefix == '后退' else 1
                    return _ClauseResult(
                        'missing',
                        pending=_PendingIntent(
                            session_id='',
                            parameter='missing_distance',
                            operation='move',
                            sign=sign,
                        ),
                    )
                return _ClauseResult('invalid', reason='distance_out_of_range')
            return _ClauseResult('invalid', reason='invalid_distance_unit')

        rotation = re.fullmatch(r'(左转|右转)\s*(\S+)\s*度', clause)
        if rotation:
            value = _parse_number(rotation.group(2))
            if value is None:
                return _ClauseResult('invalid', reason='invalid_number')
            sign = -1 if rotation.group(1) == '右转' else 1
            angle_rad = _angle_from_degrees(sign, value)
            if angle_rad is None:
                return _ClauseResult('invalid', reason='angle_out_of_range')
            return _ClauseResult(
                'step',
                step=MissionStep(
                    MissionStep.ROTATE_ANGLE,
                    angle_rad=angle_rad,
                ),
            )
        if clause.startswith('左转') or clause.startswith('右转'):
            prefix = '右转' if clause.startswith('右转') else '左转'
            remainder = clause[len(prefix):].strip()
            if not remainder or remainder == '度':
                sign = -1 if prefix == '右转' else 1
                return _ClauseResult(
                    'missing',
                    pending=_PendingIntent(
                        session_id='',
                        parameter='missing_angle',
                        operation='rotate',
                        sign=sign,
                    ),
                )
            value = _parse_number(remainder)
            if value is not None:
                sign = -1 if prefix == '右转' else 1
                angle_rad = _angle_from_degrees(sign, value)
                if angle_rad is not None and _within_angle_wire_range(
                    angle_rad,
                    self._policy.rotate_angle_min_rad,
                    self._policy.rotate_angle_max_rad,
                ):
                    return _ClauseResult(
                        'missing',
                        pending=_PendingIntent(
                            session_id='',
                            parameter='missing_angle',
                            operation='rotate',
                            sign=sign,
                        ),
                    )
                return _ClauseResult('invalid', reason='angle_out_of_range')
            return _ClauseResult('invalid', reason='invalid_angle_unit')

        place = re.fullmatch(r'(去|前往)\s*(.*)', clause)
        if place:
            target = place.group(2)
            if not target:
                return _ClauseResult(
                    'missing',
                    pending=_PendingIntent(
                        session_id='',
                        parameter='missing_place',
                        operation='place',
                    ),
                )
            if not _valid_logical_id(target):
                return _ClauseResult('invalid', reason='invalid_place_id')
            if target not in state.named_place_ids:
                return _ClauseResult('invalid', reason='unknown_place')
            return _ClauseResult(
                'step', step=MissionStep(MissionStep.NAVIGATE_TO, target_id=target)
            )
        if clause.startswith('去') or clause.startswith('前往'):
            return _ClauseResult('invalid', reason='invalid_place_id')

        save_map = re.fullmatch(r'保存地图为\s*(.*)', clause)
        if save_map:
            target = save_map.group(1)
            if not target:
                return _ClauseResult(
                    'missing',
                    pending=_PendingIntent(
                        session_id='',
                        parameter='missing_map',
                        operation='map',
                    ),
                )
            if not _valid_logical_id(target):
                return _ClauseResult('invalid', reason='invalid_map_id')
            return _ClauseResult(
                'step', step=MissionStep(MissionStep.SAVE_MAP, target_id=target)
            )
        if clause.startswith('保存地图'):
            if clause == '保存地图':
                return _ClauseResult(
                    'missing',
                    pending=_PendingIntent(
                        session_id='',
                        parameter='missing_map',
                        operation='map',
                    ),
                )
            return _ClauseResult('invalid', reason='invalid_map_id')
        return _ClauseResult('unknown')

    def _answer_pending(
        self,
        text: str,
        pending: _PendingIntent,
        state: MissionState,
    ) -> _ParseResult:
        value: Optional[float] = None
        target: Optional[str] = None
        if pending.parameter == 'missing_distance':
            match = re.fullmatch(r'(\S+)\s*米', text)
            if match:
                value = _parse_number(match.group(1))
        elif pending.parameter == 'missing_angle':
            match = re.fullmatch(r'(\S+)\s*度', text)
            if match:
                value = _parse_number(match.group(1))
        elif pending.parameter in ('missing_place', 'missing_map'):
            target = text
            if ' ' in target:
                target = ''

        if pending.parameter == 'missing_distance' and value is not None:
            step = MissionStep(
                MissionStep.MOVE_DISTANCE,
                distance_m=pending.sign * value,
            )
        elif pending.parameter == 'missing_angle' and value is not None:
            angle_rad = _angle_from_degrees(pending.sign, value)
            if angle_rad is None:
                return self._repeat_clarification(pending)
            step = MissionStep(
                MissionStep.ROTATE_ANGLE,
                angle_rad=angle_rad,
            )
        elif pending.parameter == 'missing_place' and target is not None:
            if not _valid_logical_id(target):
                return self._repeat_clarification(pending)
            if target not in state.named_place_ids:
                return self._repeat_clarification(pending)
            return self._pending_target_result(pending, target, False)
        elif pending.parameter == 'missing_map' and target is not None:
            if not _valid_logical_id(target):
                return self._repeat_clarification(pending)
            return self._pending_target_result(pending, target, True)
        else:
            return self._repeat_clarification(pending)

        if step is None:
            return self._repeat_clarification(pending)
        return _ParseResult(
            'complete',
            steps=pending.prefix_steps + (step,) + pending.suffix_steps,
        )

    @staticmethod
    def _pending_target_result(
        pending: _PendingIntent, target: str, is_map: bool
    ) -> _ParseResult:
        kind = MissionStep.SAVE_MAP if is_map else MissionStep.NAVIGATE_TO
        step = MissionStep(kind, target_id=target)
        return _ParseResult(
            'complete',
            steps=pending.prefix_steps + (step,) + pending.suffix_steps,
        )

    @staticmethod
    def _repeat_clarification(pending: _PendingIntent) -> _ParseResult:
        return _ParseResult(
            'clarify',
            pending=_PendingIntent(
                session_id=pending.session_id,
                parameter=pending.parameter,
                operation=pending.operation,
                sign=pending.sign,
                prefix_steps=pending.prefix_steps,
                suffix_steps=pending.suffix_steps,
            ),
        )

    def _store_pending(self, pending: Optional[_PendingIntent]) -> bool:
        if pending is None:
            return False
        now = self._now()
        session_id = pending.session_id
        if (
            session_id not in self._pending
            and len(self._pending) >= self._policy.clarification_capacity
        ):
            return False
        self._pending[session_id] = _PendingIntent(
            session_id=session_id,
            parameter=pending.parameter,
            operation=pending.operation,
            sign=pending.sign,
            prefix_steps=pending.prefix_steps,
            suffix_steps=pending.suffix_steps,
            created_at=now,
        )
        return True

    def _clarification(
        self, envelope: _Envelope, local_generation: int, parameter: str
    ) -> ClarifyDecision:
        return ClarifyDecision(
            session_id=envelope.session_id,
            turn_id=envelope.turn_id,
            local_generation=local_generation,
            reason=parameter,
            prompt=_clarification_text(parameter),
        )

    def _expire_pending(self) -> None:
        now = self._now()
        expired = [
            session
            for session, pending in self._pending.items()
            if now >= pending.created_at + self._policy.clarification_timeout_s
        ]
        for session in expired:
            del self._pending[session]

    def _now(self) -> float:
        clock = self._clock
        if callable(clock):
            return float(clock())
        return float(clock.now())  # type: ignore[attr-defined]


def _read_value(source: object, name: str) -> object:
    if isinstance(source, Mapping):
        return source.get(name)
    return getattr(source, name)


def _bounded_id(value: object, maximum: int) -> bool:
    return isinstance(value, str) and 0 < len(value) <= maximum


def _uint8(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= 255


def _uint64(value: object) -> bool:
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and 0 <= value <= MAX_UINT64
    )


def _finite(value: object) -> bool:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(float(value))
    except (OverflowError, TypeError, ValueError):
        return False


def _valid_logical_id(value: object) -> bool:
    return isinstance(value, str) and PLACE_ID_PATTERN.fullmatch(value) is not None


def _within_signed_range(value: float, minimum: float, maximum: float) -> bool:
    if not _finite(value):
        return False
    numeric = float(value)
    return minimum <= abs(numeric) <= maximum and numeric != 0.0


def _within_angle_wire_range(
    value: object, minimum: float, configured_maximum: float
) -> bool:
    wire_value = _binary32(value)
    wire_minimum = _binary32(minimum)
    wire_configured_maximum = _binary32(configured_maximum)
    if (
        wire_value is None
        or wire_minimum is None
        or wire_configured_maximum is None
        or not math.isfinite(wire_value)
        or not math.isfinite(wire_minimum)
        or not math.isfinite(wire_configured_maximum)
    ):
        return False
    wire_maximum = min(_RUNTIME_ROTATE_ANGLE_MAX_RAD, wire_configured_maximum)
    return (
        wire_minimum <= abs(wire_value) <= wire_maximum
        and wire_value != 0.0
    )


def _binary32(value: object) -> Optional[float]:
    try:
        return struct.unpack('<f', struct.pack('<f', float(value)))[0]
    except (OverflowError, TypeError, ValueError, struct.error):
        return None


def _normalize(text: str) -> str:
    normalized = unicodedata.normalize('NFKC', text)
    normalized = normalized.translate(
        str.maketrans({chr(code): chr(code + 32) for code in range(65, 91)})
    )
    return ' '.join(normalized.split())


def _strip_invocation(text: str) -> str:
    result = text.strip()
    if result.startswith('小智'):
        result = _consume_invocation_prefix(result[2:])
    if result.startswith('请'):
        result = _consume_invocation_prefix(result[1:])
    return result


def _consume_invocation_prefix(text: str) -> str:
    result = text.lstrip()
    boundary = _CLAUSE_BOUNDARY.match(result)
    if boundary is not None:
        result = result[boundary.end():].lstrip()
    return result


def _split_clauses(text: str) -> list[str]:
    return _CLAUSE_BOUNDARY.split(text)


def _has_single_terminal_ending(text: str) -> bool:
    if not text or text[-1] not in _SENTENCE_TERMINATORS:
        return False
    prefix = text[:-1].rstrip()
    if not prefix or prefix[-1] in _SENTENCE_TERMINATORS:
        return False
    if prefix[-1] in '，,；;、' or prefix.endswith(('然后', '再')):
        return False
    return True


def _contains_stop_clause(text: str) -> bool:
    for clause in _split_clauses(text):
        if _strip_invocation(clause).replace(' ', '') in {'停止', '紧急停止'}:
            return True
    return False


def _parse_number(token: str) -> Optional[float]:
    if token == '半':
        return 0.5
    try:
        if re.fullmatch(r'[0-9]+(?:\.[0-9]+)?', token):
            value = float(Decimal(token))
        elif re.fullmatch(r'[零〇一二两三四五六七八九十]+', token):
            value = float(_parse_chinese_integer(token))
        else:
            decimal_match = re.fullmatch(
                r'([零〇一二两三四五六七八九十]+)点([零〇一二三四五六七八九]+)',
                token,
            )
            if decimal_match is None:
                return None
            integer = _parse_chinese_integer(decimal_match.group(1))
            digits = ''.join(
                str(_chinese_digit(character))
                for character in decimal_match.group(2)
            )
            value = float(f'{integer}.{digits}')
    except (InvalidOperation, OverflowError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _angle_from_degrees(sign: int, value: float) -> Optional[float]:
    if not _finite(value) or value < 0.0 or value > 360.0:
        return None
    if value == 360.0:
        return sign * _RUNTIME_ROTATE_ANGLE_MAX_RAD
    return _binary32(sign * math.radians(value))


def _parse_chinese_integer(token: str) -> int:
    values = {
        '零': 0,
        '〇': 0,
        '一': 1,
        '二': 2,
        '两': 2,
        '三': 3,
        '四': 4,
        '五': 5,
        '六': 6,
        '七': 7,
        '八': 8,
        '九': 9,
        '十': 10,
    }
    if token == '十':
        return 10
    if len(token) == 1 and token in values:
        return values[token]
    raise ValueError('only zero through ten are supported')


def _chinese_digit(character: str) -> int:
    return {
        '零': 0,
        '〇': 0,
        '一': 1,
        '二': 2,
        '三': 3,
        '四': 4,
        '五': 5,
        '六': 6,
        '七': 7,
        '八': 8,
        '九': 9,
    }[character]


def _freeze_runtime_snapshot(
    raw: object,
) -> tuple[Optional[MissionState], str]:
    if raw is None:
        return None, 'runtime_snapshot_missing'
    try:
        state = MissionState(
            runtime_instance_id=_read_value(raw, 'runtime_instance_id'),
            admission_epoch=_read_value(raw, 'admission_epoch'),
            operating_mode=_read_value(raw, 'operating_mode'),
            availability=_read_value(raw, 'availability'),
            gate_state=_read_value(raw, 'gate_state'),
            active_step=_read_value(raw, 'active_step'),
            supported_step_mask=_read_value(raw, 'supported_step_mask'),
            max_steps=_read_value(raw, 'max_steps'),
            named_place_ids=_read_value(raw, 'named_place_ids'),
        )
    except (AttributeError, KeyError, TypeError):
        return None, 'runtime_snapshot_malformed'
    reason = _validate_runtime_state(state)
    if reason is not None:
        return None, reason
    return state, ''


def _validate_runtime_state(state: MissionState) -> Optional[str]:
    if not _bounded_id(state.runtime_instance_id, 36):
        return 'invalid_runtime_instance_id'
    if not _uint64(state.admission_epoch) or state.admission_epoch == 0:
        return 'invalid_admission_epoch'
    if state.operating_mode not in (
        OperatingMode.MAPPING,
        OperatingMode.NAVIGATION,
    ):
        return 'invalid_operating_mode'
    if state.availability != Availability.AVAILABLE:
        return 'runtime_not_available'
    if state.gate_state != GateState.GATE_INHIBITED:
        return 'gate_not_inhibited'
    if not _uint8(state.max_steps) or not 1 <= state.max_steps <= 3:
        return 'invalid_max_steps'
    if not _uint32(state.active_step):
        return 'invalid_active_step'
    if not _uint8(state.supported_step_mask) or state.supported_step_mask > 0x0F:
        return 'invalid_capability_mask'
    if not isinstance(state.named_place_ids, tuple):
        return 'invalid_named_places'
    if len(state.named_place_ids) > 32:
        return 'too_many_named_places'
    if any(not isinstance(place, str) for place in state.named_place_ids):
        return 'invalid_named_place_id'
    if len(set(state.named_place_ids)) != len(state.named_place_ids):
        return 'duplicate_named_place'
    if any(not _valid_logical_id(place) for place in state.named_place_ids):
        return 'invalid_named_place_id'
    return None


def _uint32(value: object) -> bool:
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and 0 <= value <= MAX_UINT32
    )


def _clarification_text(reason: str) -> str:
    return {
        'missing_distance': '请提供距离（米）。',
        'missing_angle': '请提供角度（度）。',
        'missing_place': '请提供当前地图中的 Place ID。',
        'missing_map': '请提供合法的 Map ID。',
    }.get(reason, '请补充一个参数。')


def _reply_text(reason: str) -> str:
    return {
        'runtime_snapshot_missing': '当前没有可用的 Runtime 状态。',
        'runtime_snapshot_malformed': 'Runtime 状态快照无效。',
        'runtime_not_available': 'Runtime 当前不可用。',
        'gate_not_inhibited': '运动安全门当前未处于禁止状态。',
        'source_sequence_exhausted': '请重启 Agent 后再试。',
        'voice_instance_capacity_exhausted': 'Voice 实例已超过安全容量，请重启 Agent。',
        'clarification_capacity_exhausted': '澄清状态已满，请稍后重试。',
        'mode_mismatch': '当前模式不支持该任务。',
        'unsupported_step': '当前能力不支持该任务。',
        'unknown_place': '当前地图中没有这个 Place ID。',
        'invalid_place_id': 'Place ID 无效。',
        'invalid_map_id': 'Map ID 无效。',
        'distance_out_of_range': '移动距离超出安全范围。',
        'angle_out_of_range': '旋转角度超出安全范围。',
        'invalid_union': 'Mission step 参数组合无效。',
        'too_many_steps': '一次最多只能执行三步。',
        'multiple_missing_parameters': '一次只能补充一个任务参数。',
    }.get(reason, '这条指令不能形成安全任务。')
