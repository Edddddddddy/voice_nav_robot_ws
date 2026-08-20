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

from types import SimpleNamespace

import pytest

from voice_nav_agent._agent_delivery_session import (
    AgentDeliverySession,
    MissionTerminal,
)
from voice_nav_agent._agent_engine import AgentOutcome
from voice_nav_agent.core import VoiceTurn


class FakeEngine:
    def __init__(self):
        self.owned = None

    def consume_delivery_lease(self, _lease):
        return True

    def record_owned_mission(self, _outcome, identity):
        self.owned = identity
        return True

    def record_cancelled_mission(self, identity):
        if self.owned is not identity:
            return False
        self.owned = None
        return True


class FakeDeliveryPort:
    def __init__(self):
        self.missions = []
        self.cancels = []
        self.stops = []
        self.speeches = []
        self.retired = []
        self.closed = False

    def submit_mission(self, identity, mission, callback):
        self.missions.append((identity, mission, callback))
        return True

    def cancel_mission(self, identity, callback):
        self.cancels.append((identity, callback))
        return True

    def submit_stop(self, identity, request, callback):
        self.stops.append((identity, request, callback))
        return True

    def submit_speak(self, identity, request, callback):
        self.speeches.append((identity, request, callback))
        return True

    def retire(self, identity):
        self.retired.append(identity)

    def shutdown(self):
        self.closed = True


def make_turn(sequence=1, *, kind=VoiceTurn.COMMAND):
    return VoiceTurn(
        voice_instance_id='voice-a',
        voice_seq=sequence,
        session_id='session-a',
        turn_id=f'turn-{sequence}',
        kind=kind,
        text='测试',
        confidence=1.0,
    )


def make_mission_outcome(generation=1):
    mission = SimpleNamespace(token=SimpleNamespace(), steps=())
    return AgentOutcome(
        'mission',
        mission=mission,
        token=mission.token,
        generation=generation,
        delivery_lease=SimpleNamespace(),
    )


def make_outcome(kind, generation, **changes):
    values = {
        'kind': kind,
        'generation': generation,
        'delivery_lease': SimpleNamespace(),
    }
    values.update(changes)
    return AgentOutcome(**values)


@pytest.mark.parametrize(
    ('terminal', 'expected_text'),
    [
        (MissionTerminal.SUCCEEDED, '任务已完成。'),
        (MissionTerminal.FAILED, '任务执行失败。'),
        (MissionTerminal.SAFETY_FAULT, '任务遇到安全故障。'),
    ],
)
def test_mission_terminal_produces_exactly_one_current_speech(
    terminal, expected_text
):
    engine = FakeEngine()
    port = FakeDeliveryPort()
    session = AgentDeliverySession('agent-a', engine, port)

    assert session.accept(make_mission_outcome(), make_turn())
    mission_identity, _mission, complete = port.missions[0]
    assert engine.owned is mission_identity

    complete(terminal)
    complete(terminal)

    assert engine.owned is None
    assert [entry[1].text for entry in port.speeches] == [expected_text]


def test_cancel_targets_only_the_exact_owned_mission():
    engine = FakeEngine()
    port = FakeDeliveryPort()
    session = AgentDeliverySession('agent-a', engine, port)
    session.accept(make_mission_outcome(), make_turn())
    mission_identity = engine.owned

    cancel = make_outcome('cancel', 2, identity=mission_identity)
    assert session.accept(cancel, make_turn(2))
    assert port.cancels[0][0] is mission_identity

    port.cancels[0][1](True)
    port.missions[0][2](MissionTerminal.CANCELED)

    assert engine.owned is None
    assert port.speeches[-1][1].text == '任务已取消。'


@pytest.mark.parametrize(
    ('confirmed', 'expected_text'),
    [(True, '已停止。'), (False, '停止请求未确认。')],
)
def test_stop_has_one_bounded_terminal_reply(confirmed, expected_text):
    engine = FakeEngine()
    port = FakeDeliveryPort()
    session = AgentDeliverySession('agent-a', engine, port)
    outcome = make_outcome(
        'stop',
        1,
        source_instance_id='voice-a',
        source_seq=1,
        turn_id='turn-1',
        reason='voice_stop',
    )

    assert session.accept(outcome, make_turn(kind=VoiceTurn.STOP))
    port.stops[0][2](confirmed)
    port.stops[0][2](confirmed)

    assert [entry[1].text for entry in port.speeches] == [expected_text]


def test_newer_outcome_retires_old_speech_and_late_completion_is_inert():
    engine = FakeEngine()
    port = FakeDeliveryPort()
    session = AgentDeliverySession('agent-a', engine, port)

    session.accept(
        make_outcome('reply', 1, text='旧回复'),
        make_turn(1),
    )
    old_identity, _request, old_done = port.speeches[-1]
    session.accept(
        make_outcome('reply', 2, text='新回复'),
        make_turn(2),
    )

    assert port.retired == [old_identity]
    old_done()
    assert [entry[1].text for entry in port.speeches] == ['旧回复', '新回复']


def test_shutdown_closes_admission_and_revokes_late_callbacks():
    engine = FakeEngine()
    port = FakeDeliveryPort()
    session = AgentDeliverySession('agent-a', engine, port)
    session.accept(make_mission_outcome(), make_turn())
    complete = port.missions[0][2]

    session.shutdown()
    complete(MissionTerminal.SUCCEEDED)

    assert port.closed
    assert not session.accept(
        make_outcome('reply', 2, text='late'), make_turn(2)
    )
    assert port.speeches == []
