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

"""Behavior tests for the package-private Response session state machine."""

from dataclasses import replace
import threading

import pytest

from voice_nav_agent._response_session import (
    _ProviderResponse,
    _ResponseSession,
    _ToolCall,
)
from voice_nav_agent.core import (
    Availability,
    GateState,
    MAX_RETIRED_VOICE_INSTANCES,
    MissionState,
    OperatingMode,
    VoiceTurn,
)


class _FakeProvider:
    def __init__(self):
        self.requests = []

    def submit(self, request):
        self.requests.append(request)


class _FakeMissionPort:
    def __init__(self):
        self.missions = []
        self.cancelled = []
        self.active = set()

    def submit(self, mission):
        self.missions.append(mission)
        identity = object()
        self.active.add(identity)
        return identity

    def prepare_mission(self, mission):
        return mission

    def commit_mission(self, mission):
        return self.submit(mission)

    def is_active(self, identity):
        return identity in self.active

    def cancel(self, identity):
        self.cancelled.append(identity)
        self.active.discard(identity)

    def prepare_cancel(self, identity):
        return identity

    def commit_cancel(self, identity):
        self.cancel(identity)


class _BlockingMissionPort(_FakeMissionPort):
    def __init__(self):
        super().__init__()
        self.block_propose = False
        self.block_cancel = False
        self.propose_entered = threading.Event()
        self.cancel_entered = threading.Event()
        self.release_propose = threading.Event()
        self.release_cancel = threading.Event()

    def submit(self, mission):
        if self.block_propose:
            self.propose_entered.set()
            assert self.release_propose.wait(1.0)
        return super().submit(mission)

    def prepare_mission(self, mission):
        if self.block_propose:
            self.propose_entered.set()
            assert self.release_propose.wait(1.0)
        return mission

    def commit_mission(self, mission):
        return super().submit(mission)

    def prepare_cancel(self, identity):
        if self.block_cancel:
            self.cancel_entered.set()
            assert self.release_cancel.wait(1.0)
        return identity

    def commit_cancel(self, identity):
        super().cancel(identity)

    def cancel(self, identity):
        if self.block_cancel:
            self.cancel_entered.set()
            assert self.release_cancel.wait(1.0)
        super().cancel(identity)


def _state():
    return MissionState(
        runtime_instance_id='runtime-a',
        admission_epoch=7,
        operating_mode=OperatingMode.NAVIGATION,
        availability=Availability.AVAILABLE,
        gate_state=GateState.GATE_INHIBITED,
        active_step=4294967295,
        supported_step_mask=0b1111,
        max_steps=3,
        named_place_ids=('lobby',),
    )


def _turn(text, sequence, voice_instance_id='voice-a'):
    return VoiceTurn(
        voice_instance_id=voice_instance_id,
        voice_seq=sequence,
        session_id='session-a',
        turn_id=f'turn-{sequence}',
        kind=VoiceTurn.COMMAND,
        text=text,
        confidence=1.0,
    )


def _mission_call(target_id='lobby'):
    return _ProviderResponse(
        kind='tool',
        tool_calls=(
            _ToolCall(
                'propose_mission',
                {
                    'kind': 'mission',
                    'steps': [
                        {'kind': 'navigate_to', 'target_id': target_id},
                    ],
                },
            ),
        ),
    )


def _invalidate_with_stop_or_new_turn(session, sequence, invalidation):
    if invalidation == 'stop':
        session.accept_turn(replace(_turn('stop', sequence), kind=VoiceTurn.STOP))
    else:
        session.accept_turn(_turn('ordinary new turn', sequence))


def test_clarify_then_next_turn_proposes_one_valid_mission():
    """A clarification can lead to one valid Mission on the next turn."""
    provider = _FakeProvider()
    mission_port = _FakeMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    assert [tool.name for tool in session.tool_registry] == [
        'read_runtime_snapshot',
        'propose_mission',
        'cancel_owned_mission',
    ]
    proposal_schema = session.tool_registry[1].parameters
    assert proposal_schema['additionalProperties'] is False
    assert set(proposal_schema['properties']) == {'kind', 'steps'}
    assert proposal_schema['properties']['kind'] == {'const': 'mission'}

    session.accept_turn(_turn('带我去一个安静的地方', 1))
    first = provider.requests.pop()
    session.complete(
        first,
        _ProviderResponse(kind='clarify', text='请告诉我目的地。'),
    )

    assert [event.kind for event in session.events] == ['clarify']

    session.accept_turn(_turn('去大厅', 2))
    second = provider.requests.pop()
    session.complete(second, _mission_call())

    assert len(mission_port.missions) == 1
    assert mission_port.missions[0].token.turn_id == 'turn-2'
    assert mission_port.cancelled == []


def test_one_active_response_retains_only_the_latest_pending_turn():
    """A stale active Response cannot displace the latest pending turn."""
    provider = _FakeProvider()
    mission_port = _FakeMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    session.accept_turn(_turn('first', 1))
    first = provider.requests[-1]
    session.accept_turn(_turn('middle', 2))
    session.accept_turn(_turn('latest', 3))

    assert provider.requests == [first]

    session.complete(first, _mission_call())

    assert mission_port.missions == []
    assert [request.turn.turn_id for request in provider.requests] == [
        'turn-1',
        'turn-3',
    ]

    session.complete(provider.requests[-1], _mission_call())

    assert [mission.token.turn_id for mission in mission_port.missions] == [
        'turn-3'
    ]


def test_read_runtime_snapshot_returns_only_the_frozen_bounded_projection():
    """Snapshot reads return only the bounded planning-time projection."""
    provider = _FakeProvider()
    mission_port = _FakeMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    session.accept_turn(_turn('状态如何', 1))
    session.complete(
        provider.requests[-1],
        _ProviderResponse(
            kind='tool',
            tool_calls=(_ToolCall('read_runtime_snapshot', {}),),
        ),
    )

    assert session.tool_outputs[-1].name == 'read_runtime_snapshot'
    assert dict(session.tool_outputs[-1].value) == {
        'runtime_instance_id': 'runtime-a',
        'admission_epoch': 7,
        'operating_mode': OperatingMode.NAVIGATION,
        'availability': Availability.AVAILABLE,
        'gate_state': GateState.GATE_INHIBITED,
        'supported_step_mask': 0b1111,
        'max_steps': 3,
        'named_place_ids': ('lobby',),
    }
    assert mission_port.missions == []
    assert mission_port.cancelled == []


def test_malformed_newer_turn_sets_high_watermark_and_invalidates_old_work():
    """A malformed newer turn fences the old result and its own sequence."""
    provider = _FakeProvider()
    mission_port = _FakeMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    session.accept_turn(_turn('first', 1))
    first = provider.requests[-1]
    session.accept_turn(replace(_turn('bad', 2), text=object()))
    session.accept_turn(_turn('same-sequence', 2))

    assert provider.requests == [first]

    session.complete(first, _mission_call())
    session.accept_turn(_turn('fresh', 3))
    session.complete(provider.requests[-1], _mission_call())

    assert [mission.token.turn_id for mission in mission_port.missions] == [
        'turn-3'
    ]


@pytest.mark.parametrize(
    'response',
    [
        _ProviderResponse(
            kind='tool', tool_calls=(_ToolCall('unknown_tool', {}),)
        ),
        _ProviderResponse(
            kind='tool',
            tool_calls=(
                _ToolCall(
                    'propose_mission',
                    {
                        'kind': 'mission',
                        'steps': [],
                        'forged_token': 'never accepted',
                    },
                ),
            ),
        ),
        _ProviderResponse(
            kind='tool',
            tool_calls=(
                _ToolCall('propose_mission', {'kind': 'mission', 'steps': {}}),
            ),
        ),
        _ProviderResponse(
            kind='tool',
            tool_calls=(
                _ToolCall('read_runtime_snapshot', {}),
                _ToolCall('propose_mission', {'kind': 'mission', 'steps': []}),
            ),
        ),
    ],
)
def test_unknown_extra_wrong_type_or_duplicate_tool_has_no_side_effect(
    response,
):
    """Untrusted tool shapes never submit or cancel a Mission."""
    provider = _FakeProvider()
    mission_port = _FakeMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    session.accept_turn(_turn('untrusted', 1))
    session.complete(provider.requests[-1], response)

    assert mission_port.missions == []
    assert mission_port.cancelled == []


def test_normal_new_turn_preserves_committed_mission_until_owned_cancel():
    """Only the explicit owned-cancel tool can cancel a committed Mission."""
    provider = _FakeProvider()
    mission_port = _FakeMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    session.accept_turn(_turn('commit', 1))
    session.complete(provider.requests[-1], _mission_call())
    committed_identity = next(iter(mission_port.active))

    session.accept_turn(_turn('ordinary new turn', 2))

    assert mission_port.cancelled == []

    session.complete(
        provider.requests[-1],
        _ProviderResponse(
            kind='tool',
            tool_calls=(_ToolCall('cancel_owned_mission', {}),),
        ),
    )

    assert mission_port.cancelled == [committed_identity]
    assert len(mission_port.missions) == 1


def test_agent_generation_invalidation_drops_late_provider_result():
    """An Agent generation transition drops a late provider result."""
    provider = _FakeProvider()
    mission_port = _FakeMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    session.accept_turn(_turn('late provider', 1))
    request = provider.requests[-1]
    session.invalidate_agent_generation()
    session.complete(request, _mission_call())

    assert mission_port.missions == []
    assert mission_port.cancelled == []


def test_restart_clears_dialogue_state_without_replaying_old_work():
    """Restart clears clarification state and ignores prior provider work."""
    provider = _FakeProvider()
    mission_port = _FakeMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    session.accept_turn(_turn('before restart', 1))
    old_request = provider.requests[-1]
    session.complete(
        old_request,
        _ProviderResponse(kind='clarify', text='请补充目的地。'),
    )
    assert [event.kind for event in session.events] == ['clarify']

    session.restart('agent-b')
    session.complete(old_request, _mission_call())
    assert session.events == ()
    session.accept_turn(_turn('after restart', 1))
    session.complete(provider.requests[-1], _mission_call())

    assert [
        mission.token.source_instance_id for mission in mission_port.missions
    ] == ['agent-b']
    assert mission_port.cancelled == []


@pytest.mark.parametrize(
    'changes',
    [
        {'runtime_instance_id': 'runtime-b'},
        {'admission_epoch': 8},
    ],
)
def test_runtime_identity_or_epoch_change_drops_late_tool_without_a_mission(
    changes,
):
    """A Runtime identity or epoch change drops a late tool result."""
    provider = _FakeProvider()
    mission_port = _FakeMissionPort()
    runtime = [_state()]
    session = _ResponseSession(
        'agent-a', provider, lambda: runtime[0], mission_port
    )

    session.accept_turn(_turn('late tool', 1))
    request = provider.requests[-1]
    runtime[0] = replace(runtime[0], **changes)
    session.complete(request, _mission_call())

    assert mission_port.missions == []
    assert mission_port.cancelled == []


def test_stop_control_invalidates_blocked_response_without_entering_tool_loop(
):
    """A STOP control turn fences blocked work without starting a tool loop."""
    provider = _FakeProvider()
    mission_port = _FakeMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    session.accept_turn(_turn('provider still blocked', 1))
    request = provider.requests[-1]
    session.accept_turn(replace(_turn('stop', 2), kind=VoiceTurn.STOP))

    assert provider.requests == [request]

    session.complete(request, _mission_call())

    assert mission_port.missions == []
    assert mission_port.cancelled == []


def test_cancel_owned_mission_rejects_foreign_stale_and_no_active_identity():
    """Foreign, stale, and absent identities cannot be cancelled."""
    no_active_provider = _FakeProvider()
    no_active_port = _FakeMissionPort()
    no_active_session = _ResponseSession(
        'agent-a', no_active_provider, _state, no_active_port
    )
    no_active_session.accept_turn(_turn('no active', 1))
    no_active_session.complete(
        no_active_provider.requests[-1],
        _ProviderResponse(
            kind='tool',
            tool_calls=(_ToolCall('cancel_owned_mission', {}),),
        ),
    )

    foreign_provider = _FakeProvider()
    foreign_port = _FakeMissionPort()
    foreign_port.active.add(object())
    foreign_session = _ResponseSession(
        'agent-a', foreign_provider, _state, foreign_port
    )
    foreign_session.accept_turn(_turn('foreign active', 1))
    foreign_session.complete(
        foreign_provider.requests[-1],
        _ProviderResponse(
            kind='tool',
            tool_calls=(_ToolCall('cancel_owned_mission', {}),),
        ),
    )

    stale_provider = _FakeProvider()
    stale_port = _FakeMissionPort()
    stale_session = _ResponseSession(
        'agent-a', stale_provider, _state, stale_port
    )
    stale_session.accept_turn(_turn('commit', 1))
    stale_session.complete(stale_provider.requests[-1], _mission_call())
    stale_port.active.clear()
    stale_session.accept_turn(_turn('cancel stale', 2))
    stale_session.complete(
        stale_provider.requests[-1],
        _ProviderResponse(
            kind='tool',
            tool_calls=(_ToolCall('cancel_owned_mission', {}),),
        ),
    )

    assert no_active_port.cancelled == []
    assert foreign_port.cancelled == []
    assert stale_port.cancelled == []


def test_thousand_turn_burst_keeps_one_active_and_one_latest_pending_request(
):
    """A 1000-turn burst runs the active request and latest replacement."""
    provider = _FakeProvider()
    mission_port = _FakeMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    session.accept_turn(_turn('turn 1', 1))
    first = provider.requests[-1]
    for sequence in range(2, 1001):
        session.accept_turn(_turn(f'turn {sequence}', sequence))

    assert provider.requests == [first]

    session.complete(first, _mission_call())

    latest = provider.requests[-1]
    assert [request.turn.turn_id for request in provider.requests] == [
        'turn-1',
        'turn-1000',
    ]

    session.complete(latest, _mission_call())

    assert [mission.token.turn_id for mission in mission_port.missions] == [
        'turn-1000'
    ]


def test_sequential_clarifications_retain_one_consumable_delivery():
    """Sequential clarification results retain only the current delivery."""
    provider = _FakeProvider()
    session = _ResponseSession('agent-a', provider, _state, _FakeMissionPort())

    for sequence in range(1, 1001):
        session.accept_turn(_turn(f'clarify {sequence}', sequence))
        session.complete(
            provider.requests.pop(),
            _ProviderResponse(kind='clarify', text=f'prompt {sequence}'),
        )

    retained = session.retained_state

    assert retained.events == 1
    assert retained.tool_outputs == 0
    assert retained.retired_voice_instances == 0
    assert not retained.active
    assert not retained.pending
    assert not retained.owned_mission
    assert session.consume_events()[-1].text == 'prompt 1000'
    assert session.events == ()


def test_sequential_snapshot_reads_retain_one_consumable_delivery():
    """Sequential snapshot reads retain only the current tool delivery."""
    provider = _FakeProvider()
    session = _ResponseSession('agent-a', provider, _state, _FakeMissionPort())

    for sequence in range(1, 1001):
        session.accept_turn(_turn(f'read {sequence}', sequence))
        session.complete(
            provider.requests.pop(),
            _ProviderResponse(
                kind='tool',
                tool_calls=(_ToolCall('read_runtime_snapshot', {}),),
            ),
        )

    retained = session.retained_state

    assert retained.events == 0
    assert retained.tool_outputs == 1
    assert retained.retired_voice_instances == 0
    assert not retained.active
    assert not retained.pending
    assert not retained.owned_mission
    assert session.consume_tool_outputs()[-1].name == 'read_runtime_snapshot'
    assert session.tool_outputs == ()


def test_voice_instance_rotation_is_bounded_and_latches_fail_closed():
    """Rotating Voice instances cannot grow retained fencing state forever."""
    provider = _FakeProvider()
    session = _ResponseSession('agent-a', provider, _state, _FakeMissionPort())

    for sequence in range(1, 1001):
        before = len(provider.requests)
        session.accept_turn(
            _turn(f'voice {sequence}', 1, f'voice-{sequence}')
        )
        if len(provider.requests) != before:
            session.complete(
                provider.requests.pop(), _ProviderResponse(kind='reply')
            )

    before_old_instance = len(provider.requests)
    session.accept_turn(_turn('old instance', 2, 'voice-1'))
    retained = session.retained_state

    assert len(provider.requests) == before_old_instance
    assert retained.events == 0
    assert retained.tool_outputs == 0
    assert retained.retired_voice_instances == MAX_RETIRED_VOICE_INSTANCES
    assert retained.voice_fencing_latched
    assert not retained.active
    assert not retained.pending
    assert not retained.owned_mission


@pytest.mark.parametrize('invalidation', ['stop', 'new_turn'])
def test_invalidation_drops_blocked_propose_before_side_effect_commit(
    invalidation,
):
    """STOP and a new turn fence a blocked Mission proposal before commit."""
    provider = _FakeProvider()
    mission_port = _BlockingMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    session.accept_turn(_turn('propose', 1))
    request = provider.requests.pop()
    mission_port.block_propose = True
    worker = threading.Thread(
        target=lambda: session.complete(request, _mission_call()), daemon=True
    )
    worker.start()
    assert mission_port.propose_entered.wait(1.0)

    _invalidate_with_stop_or_new_turn(session, 2, invalidation)
    mission_port.release_propose.set()
    worker.join(1.0)

    assert not worker.is_alive()
    assert mission_port.missions == []
    assert mission_port.cancelled == []
    assert not session.retained_state.owned_mission


@pytest.mark.parametrize('invalidation', ['stop', 'new_turn'])
def test_invalidation_drops_blocked_owned_cancel_before_side_effect_commit(
    invalidation,
):
    """STOP and a new turn fence a blocked owned-cancel before commit."""
    provider = _FakeProvider()
    mission_port = _BlockingMissionPort()
    session = _ResponseSession('agent-a', provider, _state, mission_port)

    session.accept_turn(_turn('commit', 1))
    session.complete(provider.requests.pop(), _mission_call())
    mission_port.block_cancel = True
    session.accept_turn(_turn('cancel', 2))
    request = provider.requests.pop()
    cancel_response = _ProviderResponse(
        kind='tool',
        tool_calls=(_ToolCall('cancel_owned_mission', {}),),
    )
    worker = threading.Thread(
        target=lambda: session.complete(request, cancel_response), daemon=True
    )
    worker.start()
    assert mission_port.cancel_entered.wait(1.0)

    _invalidate_with_stop_or_new_turn(session, 3, invalidation)
    mission_port.release_cancel.set()
    worker.join(1.0)

    assert not worker.is_alive()
    assert len(mission_port.missions) == 1
    assert mission_port.cancelled == []
