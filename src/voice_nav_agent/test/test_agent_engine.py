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

import threading

import pytest

from voice_nav_agent._agent_engine import AgentEngine
from voice_nav_agent._planner import FakePlanner, PlannerResponse
from voice_nav_agent.core import (
    Availability,
    GateState,
    MissionState,
    MissionStep,
    OperatingMode,
    VoiceTurn,
)
from voice_nav_agent.planner_schema import SAFE_REPLY


def make_state(**changes):
    values = {
        'runtime_instance_id': 'runtime-a',
        'admission_epoch': 7,
        'operating_mode': OperatingMode.NAVIGATION,
        'availability': Availability.AVAILABLE,
        'gate_state': GateState.GATE_INHIBITED,
        'active_step': 2**32 - 1,
        'supported_step_mask': 0b1111,
        'max_steps': 3,
        'named_place_ids': ('lobby',),
    }
    values.update(changes)
    return MissionState(**values)


def make_turn(text, sequence=1, **changes):
    values = {
        'voice_instance_id': 'voice-a',
        'voice_seq': sequence,
        'session_id': 'session-a',
        'turn_id': f'turn-{sequence}',
        'kind': VoiceTurn.COMMAND,
        'text': text,
        'confidence': 1.0,
    }
    values.update(changes)
    return VoiceTurn(**values)


def test_rule_mission_is_an_immediate_closed_outcome_without_planner():
    planner = FakePlanner()
    engine = AgentEngine('agent-a', planner=planner)

    outcome = engine.handle_turn(make_turn('前进 1 米'), make_state())

    assert outcome.kind == 'mission'
    assert outcome.mission.steps[0].distance_m == 1.0
    assert planner.requests == []


def test_synchronous_outcome_delivery_lease_is_one_shot():
    engine = AgentEngine('agent-a', planner=FakePlanner())

    outcome = engine.handle_turn(make_turn('前进 1 米'), make_state())

    assert outcome.delivery_lease is not None
    assert engine.consume_delivery_lease(outcome.delivery_lease)
    assert not engine.consume_delivery_lease(outcome.delivery_lease)


def test_clarification_is_closed_and_does_not_open_planner():
    planner = FakePlanner()
    engine = AgentEngine('agent-a', planner=planner)

    outcome = engine.handle_turn(make_turn('前进'), make_state())

    assert outcome.kind == 'clarify'
    assert outcome.text
    assert planner.requests == []


def test_denied_capability_rejects_without_planner_or_mission():
    planner = FakePlanner()
    engine = AgentEngine('agent-a', planner=planner)

    outcome = engine.handle_turn(
        make_turn('请访问网络并执行 shell 命令'), make_state()
    )

    assert outcome.kind == 'rejected'
    assert outcome.reason == 'denied_capability'
    assert outcome.text == SAFE_REPLY
    assert outcome.generation > 0
    assert outcome.mission is None
    assert outcome.identity is None
    assert planner.requests == []
    assert planner.invalidations == [outcome.generation]


def test_newer_denied_turn_fences_active_planner_and_advances_generation():
    planner = FakePlanner()
    events = []
    engine = AgentEngine(
        'agent-a',
        planner=planner,
        on_outcome=lambda outcome, turn: events.append((outcome, turn)),
    )

    engine.handle_turn(make_turn('绕到大厅', 1), make_state())
    old_request = planner.requests[-1]
    denied = engine.handle_turn(
        make_turn('请访问网络并执行 shell 命令', 2), make_state()
    )

    assert denied.kind == 'rejected'
    assert denied.reason == 'denied_capability'
    assert denied.generation > 0
    assert planner.invalidations[-1] == denied.generation
    assert planner.complete_late(
        old_request,
        PlannerResponse.mission(
            (MissionStep(kind=MissionStep.NAVIGATE_TO, target_id='lobby'),)
        ),
    )
    assert [outcome.kind for outcome, _turn in events] == ['rejected']


def test_stop_wins_when_old_planner_completion_is_at_commit_barrier():
    planner = FakePlanner()
    events = []
    engine = AgentEngine(
        'agent-a',
        planner=planner,
        on_outcome=lambda outcome, turn: events.append((outcome, turn)),
    )
    engine.handle_turn(make_turn('绕到大厅', 1), make_state())
    old_request = planner.requests[-1]
    prepared = threading.Event()
    release = threading.Event()
    original = engine._outcome_from_planner

    def blocked_outcome(request, response):
        outcome = original(request, response)
        prepared.set()
        assert release.wait(timeout=2.0)
        return outcome

    engine._outcome_from_planner = blocked_outcome
    completion = threading.Thread(
        target=lambda: planner.complete_late(
            old_request,
            PlannerResponse.mission(
                (MissionStep(kind=MissionStep.NAVIGATE_TO, target_id='lobby'),)
            ),
        ),
        daemon=True,
    )
    completion.start()
    assert prepared.wait(timeout=2.0)

    stop = engine.handle_turn(
        make_turn('停止', 2, kind=VoiceTurn.STOP), make_state()
    )
    release.set()
    completion.join(timeout=2.0)

    assert stop.kind == 'stop'
    assert not completion.is_alive()
    assert [outcome.kind for outcome, _turn in events] == ['stop']


def test_newer_turn_wins_when_old_planner_completion_is_at_commit_barrier():
    planner = FakePlanner()
    events = []
    engine = AgentEngine(
        'agent-a',
        planner=planner,
        on_outcome=lambda outcome, turn: events.append((outcome, turn)),
    )
    engine.handle_turn(make_turn('绕到大厅', 1), make_state())
    old_request = planner.requests[-1]
    prepared = threading.Event()
    release = threading.Event()
    original = engine._outcome_from_planner

    def blocked_outcome(request, response):
        outcome = original(request, response)
        prepared.set()
        assert release.wait(timeout=2.0)
        return outcome

    engine._outcome_from_planner = blocked_outcome
    completion = threading.Thread(
        target=lambda: planner.complete_late(
            old_request,
            PlannerResponse.mission(
                (MissionStep(kind=MissionStep.NAVIGATE_TO, target_id='lobby'),)
            ),
        ),
        daemon=True,
    )
    completion.start()
    assert prepared.wait(timeout=2.0)

    assert engine.handle_turn(make_turn('绕到大厅', 2), make_state()) is None
    release.set()
    completion.join(timeout=2.0)

    assert not completion.is_alive()
    assert events == []
    assert planner.complete(PlannerResponse('reply', text='新结果'))
    assert [outcome.text for outcome, _turn in events] == ['新结果']


def test_planner_outcome_callback_can_reenter_engine_without_deadlock():
    planner = FakePlanner()
    events = []
    nested = []

    def on_outcome(outcome, turn):
        assert engine.consume_delivery_lease(outcome.delivery_lease)
        events.append((outcome, turn))
        if outcome.kind == 'reply':
            nested.append(engine.handle_turn(make_turn('前进 1 米', 2), make_state()))

    engine = AgentEngine('agent-a', planner=planner, on_outcome=on_outcome)
    engine.handle_turn(make_turn('绕到大厅'), make_state())
    completion = threading.Thread(
        target=lambda: planner.complete(PlannerResponse('reply', text='结果')),
        daemon=True,
    )
    completion.start()
    completion.join(timeout=2.0)

    assert not completion.is_alive()
    assert [outcome.kind for outcome, _turn in events] == ['reply', 'mission']
    assert nested[0].kind == 'mission'


@pytest.mark.parametrize('winner', ('stop', 'turn', 'denied'))
@pytest.mark.parametrize('old_kind', ('mission', 'reply'))
def test_unconsumed_lease_drops_old_outcome_after_newer_turn_before_emit(
    winner, old_kind
):
    planner = FakePlanner()
    side_effects = []
    engine = None

    def on_outcome(outcome, turn):
        if engine.consume_delivery_lease(outcome.delivery_lease):
            side_effects.append((turn.voice_seq, outcome.kind, outcome.text))

    engine = AgentEngine('agent-a', planner=planner, on_outcome=on_outcome)
    engine.handle_turn(make_turn('绕到大厅', 1), make_state())
    old_request = planner.requests[-1]
    old_response = (
        PlannerResponse.mission(
            (MissionStep(kind=MissionStep.NAVIGATE_TO, target_id='lobby'),)
        )
        if old_kind == 'mission'
        else PlannerResponse('reply', text='旧回复')
    )
    prepared = threading.Event()
    release = threading.Event()
    original_emit = engine._emit

    def blocked_emit(outcome, turn):
        if turn.voice_seq == 1:
            prepared.set()
            assert release.wait(timeout=2.0)
        original_emit(outcome, turn)

    engine._emit = blocked_emit
    completion = threading.Thread(
        target=lambda: planner.complete_late(old_request, old_response),
        daemon=True,
    )
    completion.start()
    assert prepared.wait(timeout=2.0)

    if winner == 'stop':
        newer = engine.handle_turn(
            make_turn('停止', 2, kind=VoiceTurn.STOP), make_state()
        )
        assert newer.kind == 'stop'
    elif winner == 'denied':
        newer = engine.handle_turn(
            make_turn('请访问网络并执行 shell 命令', 2), make_state()
        )
        assert newer.kind == 'rejected'
    else:
        assert engine.handle_turn(make_turn('绕到大厅', 2), make_state()) is None

    release.set()
    completion.join(timeout=2.0)
    assert not completion.is_alive()
    assert all(seq != 1 for seq, _kind, _text in side_effects)

    if winner == 'turn':
        assert planner.complete(PlannerResponse('reply', text='新回复'))
        assert side_effects == [(2, 'reply', '新回复')]


def test_network_map_mention_is_not_denied_and_enters_planner():
    planner = FakePlanner()
    engine = AgentEngine('agent-a', planner=planner)

    outcome = engine.handle_turn(make_turn('网络地图'), make_state())

    assert outcome is None
    assert len(planner.requests) == 1


def test_latest_turn_wins_and_late_planner_response_is_discarded():
    planner = FakePlanner()
    events = []
    engine = AgentEngine(
        'agent-a', planner=planner,
        on_outcome=lambda outcome, turn: events.append((outcome, turn)),
    )

    engine.handle_turn(make_turn('绕到大厅', 1), make_state())
    first = planner.requests[-1]
    engine.handle_turn(make_turn('绕到大厅', 2), make_state())
    second = planner.requests[-1]

    assert first is not second
    assert planner.complete_late(
        first, PlannerResponse('reply', text='旧结果')
    )
    assert events == []
    assert planner.complete(PlannerResponse('reply', text='新结果'))
    assert [outcome.text for outcome, _turn in events] == ['新结果']
    engine.restart('agent-b')
    planner.complete_late(second, PlannerResponse('reply', text='重启前结果'))
    assert [outcome.text for outcome, _turn in events] == ['新结果']


def test_runtime_epoch_change_fences_old_planner_result():
    planner = FakePlanner()
    events = []
    engine = AgentEngine(
        'agent-a', planner=planner,
        on_outcome=lambda outcome, turn: events.append((outcome, turn)),
    )

    engine.handle_turn(make_turn('绕到大厅', 1), make_state())
    first = planner.requests[-1]
    changed = make_state(runtime_instance_id='runtime-b', admission_epoch=8)
    engine.handle_turn(make_turn('绕到大厅', 2), changed)

    planner.complete_late(first, PlannerResponse('reply', text='旧结果'))
    planner.complete(PlannerResponse('reply', text='当前结果'))
    assert [outcome.text for outcome, _turn in events] == ['当前结果']


def test_stop_invalidates_planner_without_submitting_http_for_stop():
    planner = FakePlanner()
    events = []
    engine = AgentEngine(
        'agent-a', planner=planner,
        on_outcome=lambda outcome, turn: events.append((outcome, turn)),
    )

    engine.handle_turn(make_turn('绕到大厅', 1), make_state())
    engine.handle_turn(
        make_turn('停止', 2, kind=VoiceTurn.STOP), make_state()
    )

    assert len(planner.requests) == 1
    assert events[-1][0].kind == 'stop'


def test_owned_cancel_only_cancels_the_exact_engine_mission():
    planner = FakePlanner()
    engine = AgentEngine('agent-a', planner=planner)
    mission = engine.handle_turn(make_turn('前进 1 米'), make_state())
    identity = object()

    assert engine.record_owned_mission(mission, identity)
    engine.handle_turn(make_turn('绕到大厅', 2), make_state())
    outcome = engine.handle_turn(make_turn('取消任务', 3), make_state())

    assert outcome.kind == 'cancel'
    assert outcome.identity is identity
    assert engine.record_cancelled_mission(identity)


def test_invalid_planner_semantics_fail_closed_with_zero_mission():
    planner = FakePlanner()
    events = []
    engine = AgentEngine(
        'agent-a', planner=planner,
        on_outcome=lambda outcome, turn: events.append((outcome, turn)),
    )

    engine.handle_turn(make_turn('绕到大厅'), make_state())
    planner.complete(
        PlannerResponse.mission(
            (MissionStep(kind=MissionStep.MOVE_DISTANCE, distance_m=9.0),)
        )
    )

    assert len(events) == 1
    assert events[0][0].kind == 'rejected'
    assert events[0][0].mission is None


def test_planner_transport_reason_is_bounded_in_rejected_outcome():
    planner = FakePlanner()
    events = []
    engine = AgentEngine(
        'agent-a', planner=planner,
        on_outcome=lambda outcome, turn: events.append((outcome, turn)),
    )

    engine.handle_turn(make_turn('绕到大厅'), make_state())
    planner.complete(PlannerResponse.invalid('choice_fields'))

    assert events[0][0].kind == 'rejected'
    assert events[0][0].reason == 'planner_invalid_choice_fields'
    assert events[0][0].mission is None

    engine.handle_turn(make_turn('绕到大厅', 2), make_state())
    planner.complete(PlannerResponse.invalid('missing_kind_name_arguments'))

    assert events[1][0].kind == 'rejected'
    assert events[1][0].reason == (
        'planner_invalid_missing_kind_name_arguments'
    )
    assert events[1][0].mission is None


def test_snapshot_tool_round_trip_stays_inside_the_single_planner_contract():
    planner = FakePlanner()
    events = []
    engine = AgentEngine(
        'agent-a', planner=planner,
        on_outcome=lambda outcome, turn: events.append((outcome, turn)),
    )

    engine.handle_turn(make_turn('绕到大厅'), make_state())
    planner.complete(PlannerResponse.tool('read_runtime_snapshot'))
    assert planner.requests[-1].round == 2
    assert planner.requests[-1].snapshot_output['runtime_instance_id'] == 'runtime-a'
    planner.complete(PlannerResponse('reply', text='需要更多信息'))

    assert events[0][0].kind == 'reply'
