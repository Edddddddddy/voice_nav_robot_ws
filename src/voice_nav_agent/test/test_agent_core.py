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

from dataclasses import replace
import math
import struct
import time

import pytest

from voice_nav_agent.core import (
    AgentCore,
    AgentPolicy,
    Availability,
    DecisionKind,
    GateState,
    MissionProposal,
    MissionState,
    MissionStep,
    OperatingMode,
    SemanticValidator,
    VoiceTurn,
)


class ManualClock:
    def __init__(self):
        self.value = 0.0

    def now(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


def make_state(**changes):
    values = {
        'runtime_instance_id': 'runtime-a',
        'admission_epoch': 7,
        'operating_mode': OperatingMode.NAVIGATION,
        'availability': Availability.AVAILABLE,
        'gate_state': GateState.GATE_INHIBITED,
        'active_step': 4294967295,
        'supported_step_mask': 0b1111,
        'max_steps': 3,
        'named_place_ids': ('lobby', 'charging'),
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
        'during_playback': False,
    }
    values.update(changes)
    return VoiceTurn(**values)


def make_core(clock=None, **policy_changes):
    return AgentCore(
        'agent-a',
        policy=AgentPolicy(**policy_changes),
        clock=clock or ManualClock().now,
    )


def _binary32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def test_rule_mission_normalizes_text_and_captures_immutable_snapshot():
    clock = ManualClock()
    core = make_core(clock=clock)
    state = make_state()

    decision = core.handle_turn(
        make_turn('  小智，  向前走 0.5 米！ '), state
    )

    assert decision.kind is DecisionKind.MISSION
    assert decision.mission.steps == (
        MissionStep(kind=MissionStep.MOVE_DISTANCE, distance_m=0.5),
    )
    assert decision.mission.token.source_instance_id == 'agent-a'
    assert decision.mission.token.source_seq == 1
    assert decision.mission.token.runtime_instance_id == 'runtime-a'
    assert decision.mission.token.admission_epoch == 7
    assert decision.mission.token.named_place_ids == ('lobby', 'charging')

    changed = make_state(runtime_instance_id='runtime-new', admission_epoch=8)
    assert decision.mission.token.runtime_instance_id == 'runtime-a'
    assert decision.mission.token.admission_epoch == 7
    assert changed != state


@pytest.mark.parametrize(
    ('text', 'kind', 'distance', 'angle'),
    [
        ('后退半米', MissionStep.MOVE_DISTANCE, -0.5, 0.0),
        ('前进一米', MissionStep.MOVE_DISTANCE, 1.0, 0.0),
        ('前进一点五米', MissionStep.MOVE_DISTANCE, 1.5, 0.0),
        ('左转90度', MissionStep.ROTATE_ANGLE, 0.0, math.pi / 2),
        ('右转 5 度', MissionStep.ROTATE_ANGLE, 0.0, -math.radians(5)),
    ],
)
def test_rule_mission_supports_bounded_numbers_and_signed_steps(
    text, kind, distance, angle
):
    core = make_core()

    decision = core.handle_turn(make_turn(text), make_state())

    assert decision.kind is DecisionKind.MISSION
    assert decision.mission.steps[0].kind == kind
    assert decision.mission.steps[0].distance_m == pytest.approx(distance)
    assert decision.mission.steps[0].angle_rad == pytest.approx(angle)


def test_rule_mission_preserves_one_to_three_step_order_and_place_id():
    core = make_core()

    decision = core.handle_turn(
        make_turn('前进 1 米然后左转 90 度再去 lobby'), make_state()
    )

    assert decision.kind is DecisionKind.MISSION
    assert [step.kind for step in decision.mission.steps] == [
        MissionStep.MOVE_DISTANCE,
        MissionStep.ROTATE_ANGLE,
        MissionStep.NAVIGATE_TO,
    ]
    assert decision.mission.steps[2].target_id == 'lobby'


@pytest.mark.parametrize(
    'text',
    [
        '前进 1 米 然后 左转 90 度',
        '前进 1 米，然后左转 90 度',
        '前进 1 米；左转 90 度',
        '前进 1 米,左转 90 度',
        '前进 1 米;左转 90 度',
    ],
)
def test_clause_separators_are_trimmed_and_equivalent(text):
    core = make_core()

    decision = core.handle_turn(make_turn(text), make_state())

    assert decision.kind is DecisionKind.MISSION
    assert [step.kind for step in decision.steps] == [
        MissionStep.MOVE_DISTANCE,
        MissionStep.ROTATE_ANGLE,
    ]


@pytest.mark.parametrize(
    'text',
    [
        '前进 1 米；',
        '前进 1 米；；左转 90 度',
    ],
)
def test_empty_clause_fails_closed_after_separator_split(text):
    decision = make_core().handle_turn(make_turn(text), make_state())

    assert decision.kind is DecisionKind.REPLY
    assert decision.reason == 'empty_clause'


@pytest.mark.parametrize(
    'separator',
    ['，', ',', '；', ';', '。', '！', '!', '？', '?', '、'],
)
def test_every_approved_punctuation_preserves_a_fixed_stop_clause(separator):
    decision = make_core().handle_turn(
        make_turn(f'前进 1 米{separator}紧急停止'), make_state()
    )

    assert decision.kind is DecisionKind.STOP
    assert decision.reason == 'voice_stop'


@pytest.mark.parametrize('terminator', ['。', '！', '!', '？', '?'])
def test_one_sentence_terminator_may_end_the_last_non_empty_clause(terminator):
    decision = make_core().handle_turn(
        make_turn(f'前进 1 米{terminator}'), make_state()
    )

    assert decision.kind is DecisionKind.MISSION
    assert decision.steps[0].distance_m == 1.0


@pytest.mark.parametrize(
    'text',
    [
        '前进 1 米。。左转 90 度',
        '前进 1 米，，左转 90 度',
        '前进 1 米然后然后左转 90 度',
        '小智然后然后前进 1 米',
        '小智，，前进 1 米',
    ],
)
def test_repeated_internal_separator_or_connector_fails_closed(text):
    decision = make_core().handle_turn(make_turn(text), make_state())

    assert decision.kind is DecisionKind.REPLY
    assert decision.reason == 'empty_clause'


def test_invocation_may_consume_one_composite_boundary_but_not_two():
    decision = make_core().handle_turn(
        make_turn('小智，然后前进 1 米'), make_state()
    )

    assert decision.kind is DecisionKind.MISSION
    assert decision.steps[0].distance_m == 1.0


@pytest.mark.parametrize(
    ('text', 'reason'),
    [
        ('前进 1 米然后左转 90 度再去 lobby然后保存地图为 map_a', 'too_many_steps'),
        ('前进', 'missing_distance'),
        ('前进 3 米', 'distance_out_of_range'),
        ('去 unknown', 'unknown_place'),
        ('保存地图为 ../map', 'invalid_map_id'),
    ],
)
def test_invalid_or_incomplete_closed_rules_never_make_a_mission(text, reason):
    core = make_core()

    decision = core.handle_turn(make_turn(text), make_state())

    assert decision.kind in (DecisionKind.CLARIFY, DecisionKind.REPLY)
    assert decision.reason == reason


@pytest.mark.parametrize(
    ('text', 'reason'),
    [
        ('前进米', 'missing_distance'),
        ('前进 1', 'missing_distance'),
        ('左转度', 'missing_angle'),
        ('左转 90', 'missing_angle'),
        ('保存地图', 'missing_map'),
        ('保存地图为', 'missing_map'),
    ],
)
def test_single_missing_parameter_enters_clarification(text, reason):
    decision = make_core().handle_turn(make_turn(text), make_state())

    assert decision.kind is DecisionKind.CLARIFY
    assert decision.reason == reason


@pytest.mark.parametrize(
    ('text', 'reason'),
    [
        ('前进 0', 'distance_out_of_range'),
        ('后退 0', 'distance_out_of_range'),
        ('前进 3', 'distance_out_of_range'),
        ('左转 0', 'angle_out_of_range'),
        ('左转 361', 'angle_out_of_range'),
    ],
)
def test_out_of_range_missing_unit_value_is_rejected_before_clarification(
    text, reason
):
    core = make_core()

    decision = core.handle_turn(make_turn(text), make_state())
    answer_after_rejection = core.handle_turn(
        make_turn('1 米', sequence=2), make_state()
    )

    assert decision.kind is DecisionKind.REPLY
    assert decision.reason == reason
    assert answer_after_rejection.kind is DecisionKind.LLM_NEEDED


@pytest.mark.parametrize(
    ('text', 'reason', 'state_changes', 'answer'),
    [
        ('前进 3 米然后左转', 'distance_out_of_range', {}, '90 度'),
        ('左转然后前进 3 米', 'distance_out_of_range', {}, '90 度'),
        ('左转 1 度然后前进', 'angle_out_of_range', {}, '1 米'),
        (
            '保存地图为 map_a 然后前进',
            'mode_mismatch',
            {'operating_mode': OperatingMode.NAVIGATION},
            '1 米',
        ),
        (
            '左转 90 度然后前进',
            'unsupported_step',
            {'supported_step_mask': 0b0001},
            '1 米',
        ),
        ('去 unknown 然后前进', 'unknown_place', {}, '1 米'),
    ],
)
def test_invalid_complete_sibling_rejects_before_pending_is_stored(
    text, reason, state_changes, answer
):
    core = make_core()

    first = core.handle_turn(make_turn(text), make_state(**state_changes))
    after_failure = core.handle_turn(
        make_turn(answer, sequence=2), make_state(**state_changes)
    )

    assert first.kind is DecisionKind.REPLY
    assert first.reason == reason
    assert after_failure.kind is DecisionKind.LLM_NEEDED


@pytest.mark.parametrize(
    ('text', 'answer', 'expected_kinds'),
    [
        (
            '前进 1 米然后左转',
            '90 度',
            [MissionStep.MOVE_DISTANCE, MissionStep.ROTATE_ANGLE],
        ),
        (
            '左转然后前进 1 米',
            '90 度',
            [MissionStep.ROTATE_ANGLE, MissionStep.MOVE_DISTANCE],
        ),
    ],
)
def test_valid_complete_sibling_and_single_missing_parameter_keep_order(
    text, answer, expected_kinds
):
    core = make_core()

    clarify = core.handle_turn(make_turn(text), make_state())
    mission = core.handle_turn(
        make_turn(answer, sequence=2), make_state()
    )

    assert clarify.kind is DecisionKind.CLARIFY
    assert mission.kind is DecisionKind.MISSION
    assert [step.kind for step in mission.steps] == expected_kinds


def test_mode_and_capability_rejections_are_structured_replies():
    core = make_core()

    wrong_mode = core.handle_turn(
        make_turn('保存地图为 map_a'),
        make_state(operating_mode=OperatingMode.NAVIGATION),
    )
    no_capability = core.handle_turn(
        make_turn('前进 1 米', sequence=2),
        make_state(supported_step_mask=0),
    )
    too_many_for_snapshot = core.handle_turn(
        make_turn('前进 1 米然后左转 90 度再去 lobby', sequence=3),
        make_state(max_steps=2),
    )

    assert wrong_mode.kind is DecisionKind.REPLY
    assert wrong_mode.reason == 'mode_mismatch'
    assert no_capability.kind is DecisionKind.REPLY
    assert no_capability.reason == 'unsupported_step'
    assert too_many_for_snapshot.kind is DecisionKind.REPLY
    assert too_many_for_snapshot.reason == 'step_count_exceeds_snapshot'


def test_save_map_is_mapping_only_and_uses_a_logical_map_id():
    core = make_core()

    decision = core.handle_turn(
        make_turn('保存地图为 map_a'),
        make_state(operating_mode=OperatingMode.MAPPING),
    )

    assert decision.kind is DecisionKind.MISSION
    assert decision.steps == (
        MissionStep(MissionStep.SAVE_MAP, target_id='map_a'),
    )


@pytest.mark.parametrize(
    'changes',
    [
        {'runtime_instance_id': ''},
        {'admission_epoch': 0},
        {'availability': Availability.BUSY},
        {'gate_state': GateState.GATE_ARMED},
        {'max_steps': 4},
        {'supported_step_mask': 0x10},
        {'named_place_ids': ('lobby', 'lobby')},
        {'named_place_ids': ('大厅',)},
    ],
)
def test_malformed_or_unusable_runtime_snapshot_fails_closed(changes):
    core = make_core()

    decision = core.handle_turn(make_turn('前进 1 米'), make_state(**changes))

    assert decision.kind is DecisionKind.REPLY


def test_clarification_answers_reject_complete_unknown_place_and_end_pending():
    clock = ManualClock()
    core = make_core(clock=clock)

    missing_place = core.handle_turn(
        make_turn('去'), make_state()
    )
    bad_answer = core.handle_turn(
        make_turn('unknown', sequence=2), make_state()
    )
    good_answer = core.handle_turn(
        make_turn('lobby', sequence=3), make_state()
    )

    assert missing_place.kind is DecisionKind.CLARIFY
    assert missing_place.reason == 'missing_place'
    assert bad_answer.kind is DecisionKind.REPLY
    assert bad_answer.reason == 'unknown_place'
    assert good_answer.kind is DecisionKind.LLM_NEEDED


def test_complete_out_of_range_angle_answer_rejects_and_ends_pending():
    core = make_core()

    clarify = core.handle_turn(make_turn('左转'), make_state())
    illegal = core.handle_turn(
        make_turn('361 度', sequence=2), make_state()
    )
    bare_answer = core.handle_turn(
        make_turn('90 度', sequence=3), make_state()
    )

    assert clarify.kind is DecisionKind.CLARIFY
    assert illegal.kind is DecisionKind.REPLY
    assert illegal.reason == 'angle_out_of_range'
    assert bare_answer.kind is DecisionKind.LLM_NEEDED


def test_reclarification_validates_siblings_before_overwrite():
    core = make_core()

    clarify = core.handle_turn(
        make_turn('前进 1 米然后左转'), make_state()
    )
    rejected = core.handle_turn(
        make_turn('1 厘米', sequence=2),
        make_state(supported_step_mask=0b0010),
    )
    bare_answer = core.handle_turn(
        make_turn('90 度', sequence=3), make_state()
    )

    assert clarify.kind is DecisionKind.CLARIFY
    assert rejected.kind is DecisionKind.REPLY
    assert rejected.reason == 'unsupported_step'
    assert bare_answer.kind is DecisionKind.LLM_NEEDED


@pytest.mark.parametrize(
    ('text', 'initial_changes', 'changed_changes', 'reason'),
    [
        (
            '去 lobby 然后前进',
            {'operating_mode': OperatingMode.NAVIGATION},
            {
                'operating_mode': OperatingMode.NAVIGATION,
                'named_place_ids': ('charging',),
            },
            'unknown_place',
        ),
        (
            '前进 1 米然后左转',
            {'max_steps': 2},
            {'max_steps': 1},
            'step_count_exceeds_snapshot',
        ),
        (
            '保存地图为 map_a 然后前进',
            {'operating_mode': OperatingMode.MAPPING},
            {'operating_mode': OperatingMode.NAVIGATION},
            'mode_mismatch',
        ),
    ],
)
def test_reclarification_pending_commit_rejects_changed_snapshot_siblings(
    text, initial_changes, changed_changes, reason
):
    core = make_core()

    clarify = core.handle_turn(
        make_turn(text), make_state(**initial_changes)
    )
    rejected = core.handle_turn(
        make_turn('1 厘米', sequence=2), make_state(**changed_changes)
    )
    bare_answer = core.handle_turn(
        make_turn('1 米', sequence=3), make_state(**initial_changes)
    )

    assert clarify.kind is DecisionKind.CLARIFY
    assert rejected.kind is DecisionKind.REPLY
    assert rejected.reason == reason
    assert bare_answer.kind is DecisionKind.LLM_NEEDED


def test_valid_wrong_unit_reclarifies_and_then_accepts_a_typed_answer():
    core = make_core()

    clarify = core.handle_turn(make_turn('左转'), make_state())
    repeated = core.handle_turn(
        make_turn('90 弧度', sequence=2), make_state()
    )
    mission = core.handle_turn(
        make_turn('90 度', sequence=3), make_state()
    )

    assert clarify.kind is DecisionKind.CLARIFY
    assert repeated.kind is DecisionKind.CLARIFY
    assert repeated.reason == 'missing_angle'
    assert mission.kind is DecisionKind.MISSION
    assert mission.steps[0].kind == MissionStep.ROTATE_ANGLE


@pytest.mark.parametrize(
    'answer', ['0 度', '1 度', '361 度', '360.0001 度']
)
def test_complete_illegal_angle_answers_end_pending(answer):
    core = make_core()

    core.handle_turn(make_turn('左转'), make_state())
    rejected = core.handle_turn(
        make_turn(answer, sequence=2), make_state()
    )
    bare_answer = core.handle_turn(
        make_turn('90 度', sequence=3), make_state()
    )

    assert rejected.kind is DecisionKind.REPLY
    assert rejected.reason == 'angle_out_of_range'
    assert bare_answer.kind is DecisionKind.LLM_NEEDED


@pytest.mark.parametrize('answer', ['0 米', '3 米'])
def test_complete_illegal_distance_answers_end_pending(answer):
    core = make_core()

    core.handle_turn(make_turn('前进'), make_state())
    rejected = core.handle_turn(
        make_turn(answer, sequence=2), make_state()
    )
    bare_answer = core.handle_turn(
        make_turn('1 米', sequence=3), make_state()
    )

    assert rejected.kind is DecisionKind.REPLY
    assert rejected.reason == 'distance_out_of_range'
    assert bare_answer.kind is DecisionKind.LLM_NEEDED


@pytest.mark.parametrize(
    ('command', 'answer', 'reason', 'state_changes'),
    [
        ('去', '../place', 'invalid_place_id', {}),
        (
            '保存地图',
            '../map',
            'invalid_map_id',
            {'operating_mode': OperatingMode.MAPPING},
        ),
    ],
)
def test_complete_illegal_id_answers_end_pending(
    command, answer, reason, state_changes
):
    core = make_core()
    state = make_state(**state_changes)

    core.handle_turn(make_turn(command), state)
    rejected = core.handle_turn(make_turn(answer, sequence=2), state)
    bare_answer = core.handle_turn(
        make_turn('lobby' if command == '去' else 'map_a', sequence=3),
        state,
    )

    assert rejected.kind is DecisionKind.REPLY
    assert rejected.reason == reason
    assert bare_answer.kind is DecisionKind.LLM_NEEDED


@pytest.mark.parametrize(
    ('command', 'answer'),
    [('前进', 'n/a 米'), ('左转', 'n/a 度')],
)
def test_complete_typed_unit_with_invalid_number_ends_pending(command, answer):
    core = make_core()

    core.handle_turn(make_turn(command), make_state())
    rejected = core.handle_turn(make_turn(answer, sequence=2), make_state())
    bare_answer = core.handle_turn(
        make_turn('1 米' if command == '前进' else '90 度', sequence=3),
        make_state(),
    )

    assert rejected.kind is DecisionKind.REPLY
    assert rejected.reason == 'invalid_number'
    assert bare_answer.kind is DecisionKind.LLM_NEEDED


def test_clarification_is_session_scoped_and_stop_clears_pending_state():
    core = make_core()

    clarify = core.handle_turn(
        make_turn('前进', session_id='session-a'), make_state()
    )
    other_session = core.handle_turn(
        make_turn('1 米', sequence=2, session_id='session-b'), make_state()
    )
    stop = core.handle_turn(
        make_turn(
            '停止',
            sequence=3,
            turn_id='stop-turn',
            kind=VoiceTurn.STOP,
        ),
        runtime_snapshot_or_none=None,
    )
    after_stop = core.handle_turn(
        make_turn('1 米', sequence=4, session_id='session-a'), make_state()
    )

    assert clarify.kind is DecisionKind.CLARIFY
    assert other_session.kind is DecisionKind.LLM_NEEDED
    assert stop.kind is DecisionKind.STOP
    assert after_stop.kind is DecisionKind.LLM_NEEDED


def test_clarification_capacity_is_bounded_and_expired_entries_are_reclaimed():
    clock = ManualClock()
    core = make_core(clock=clock, clarification_capacity=1)

    first = core.handle_turn(
        make_turn('前进', session_id='session-a'), make_state()
    )
    full = core.handle_turn(
        make_turn('左转', sequence=2, session_id='session-b'), make_state()
    )
    clock.advance(15.0)
    reclaimed = core.handle_turn(
        make_turn('左转', sequence=3, session_id='session-b'), make_state()
    )

    assert first.kind is DecisionKind.CLARIFY
    assert full.kind is DecisionKind.REPLY
    assert full.reason == 'clarification_capacity_exhausted'
    assert reclaimed.kind is DecisionKind.CLARIFY


def test_new_complete_command_replaces_old_pending_intent():
    core = make_core()

    first = core.handle_turn(make_turn('前进'), make_state())
    replacement = core.handle_turn(
        make_turn('左转 90 度', sequence=2), make_state()
    )
    answer_to_old = core.handle_turn(
        make_turn('1 米', sequence=3), make_state()
    )

    assert first.kind is DecisionKind.CLARIFY
    assert replacement.kind is DecisionKind.MISSION
    assert answer_to_old.kind is DecisionKind.LLM_NEEDED


def test_retired_voice_instance_set_fails_closed_without_silent_eviction():
    core = make_core()
    core.handle_turn(make_turn('前进 1 米'), make_state())

    for sequence in range(1, 65):
        decision = core.handle_turn(
            make_turn(
                '前进 1 米',
                sequence=1,
                voice_instance_id=f'voice-{sequence}',
                turn_id=f'turn-{sequence}',
            ),
            make_state(),
        )
        assert decision.kind is DecisionKind.MISSION

    capacity = core.handle_turn(
        make_turn(
            '前进 1 米',
            sequence=1,
            voice_instance_id='voice-65',
            turn_id='turn-65',
        ),
        make_state(),
    )
    old = core.handle_turn(
        make_turn(
            '前进 1 米',
            sequence=2,
            voice_instance_id='voice-1',
            turn_id='old-turn',
        ),
        make_state(),
    )
    any_new_command = core.handle_turn(
        make_turn(
            '前进 1 米',
            sequence=1,
            voice_instance_id='voice-66',
            turn_id='turn-66',
        ),
        make_state(),
    )
    stop = core.handle_turn(
        make_turn(
            '停止',
            sequence=3,
            voice_instance_id='voice-66',
            turn_id='stop-after-capacity',
            kind=VoiceTurn.STOP,
        ),
        runtime_snapshot_or_none=None,
    )

    assert capacity.kind is DecisionKind.REPLY
    assert capacity.reason == 'voice_instance_capacity_exhausted'
    assert old.kind is DecisionKind.REPLY
    assert old.reason == 'voice_instance_capacity_exhausted'
    assert any_new_command.kind is DecisionKind.REPLY
    assert any_new_command.reason == 'voice_instance_capacity_exhausted'
    assert stop.kind is DecisionKind.STOP

    fresh = make_core().handle_turn(
        make_turn('前进 1 米'), make_state()
    )
    assert fresh.kind is DecisionKind.MISSION


def test_unknown_well_formed_expression_becomes_llm_needed_with_same_token():
    core = make_core()

    decision = core.handle_turn(make_turn('绕到大厅'), make_state())

    assert decision.kind is DecisionKind.LLM_NEEDED
    assert decision.normalized_text == '绕到大厅'
    assert decision.token.source_seq == 1
    assert decision.token.runtime_instance_id == 'runtime-a'


def test_issue136_scripted_turn_texts_both_take_llm_needed_path():
    """The product scenario must not bypass its provider dialogue locally."""
    core = make_core()

    first = core.handle_turn(make_turn('绕到大厅'), make_state())
    second = core.handle_turn(make_turn('半米', sequence=2), make_state())

    assert first.kind is DecisionKind.LLM_NEEDED
    assert second.kind is DecisionKind.LLM_NEEDED
    assert first.normalized_text == '绕到大厅'
    assert second.normalized_text == '半米'


@pytest.mark.parametrize(
    'text',
    [
        '绕到大厅然后前进',
        '前进然后绕到大厅',
        '绕到大厅然后前进 1 米',
        '前进 1 米然后绕到大厅',
    ],
)
def test_unknown_mixed_with_rule_or_missing_is_order_independent_rejection(text):
    core = make_core()

    decision = core.handle_turn(make_turn(text), make_state())
    answer_after_rejection = core.handle_turn(
        make_turn('1 米', sequence=2), make_state()
    )

    assert decision.kind is DecisionKind.REPLY
    assert decision.reason == 'mixed_unknown_rule'
    assert answer_after_rejection.kind is DecisionKind.LLM_NEEDED


def test_cancel_is_local_and_does_not_require_runtime_snapshot():
    clock = ManualClock()
    core = make_core(clock=clock)

    core.handle_turn(make_turn('前进', sequence=1), make_state())
    decision = core.handle_turn(
        make_turn('小智取消任务', sequence=2), runtime_snapshot_or_none=None
    )

    assert decision.kind is DecisionKind.CANCEL
    assert decision.source_instance_id == 'agent-a'
    assert decision.source_seq == 2


def test_stop_uses_final_voice_identity_rule_and_bypasses_command_fencing():
    clock = ManualClock()
    core = make_core(clock=clock)

    core.handle_turn(make_turn('前进', sequence=1), make_state())
    first = core.handle_turn(
        make_turn(
            '停止',
            sequence=9,
            turn_id='stop-turn',
            kind=VoiceTurn.STOP,
        ),
        runtime_snapshot_or_none=None,
    )
    retry = core.handle_turn(
        make_turn(
            '任意文本',
            sequence=9,
            turn_id='stop-turn',
            kind=VoiceTurn.STOP,
        ),
        runtime_snapshot_or_none=None,
    )

    assert first.kind is DecisionKind.STOP
    assert first.request_id == 'stop-turn'
    assert first.source_instance_id == 'voice-a'
    assert first.source_seq == 9
    assert first.reason == 'voice_stop'
    assert retry == first

    restarted = make_core()
    after_restart = restarted.handle_turn(
        make_turn(
            '任意文本',
            sequence=9,
            turn_id='stop-turn',
            kind=VoiceTurn.STOP,
        ),
        runtime_snapshot_or_none=None,
    )
    collision = restarted.handle_turn(
        make_turn(
            '任意文本',
            sequence=10,
            voice_instance_id='voice-b',
            turn_id='stop-turn',
            kind=VoiceTurn.STOP,
        ),
        runtime_snapshot_or_none=None,
    )

    assert after_restart == first
    assert collision != first


def test_duplicate_and_retired_commands_are_ignored_without_new_effect():
    core = make_core()

    first = core.handle_turn(make_turn('前进 1 米', sequence=4), make_state())
    duplicate = core.handle_turn(make_turn('前进 2 米', sequence=4), make_state())
    old_instance = core.handle_turn(
        make_turn(
            '前进 2 米',
            sequence=1,
            voice_instance_id='voice-old',
            turn_id='old-turn',
        ),
        make_state(),
    )

    assert first.kind is DecisionKind.MISSION
    assert duplicate.kind is DecisionKind.IGNORE
    assert old_instance.kind is DecisionKind.MISSION

    late_old = core.handle_turn(
        make_turn(
            '前进 1 米',
            sequence=5,
            voice_instance_id='voice-a',
            turn_id='late-turn',
        ),
        make_state(),
    )
    assert late_old.kind is DecisionKind.IGNORE


def test_clarification_uses_current_snapshot_and_expires_on_steady_clock():
    clock = ManualClock()
    core = make_core(clock=clock)

    clarify = core.handle_turn(make_turn('前进'), make_state())
    completed = core.handle_turn(
        make_turn('1 米', sequence=2),
        make_state(runtime_instance_id='runtime-b', admission_epoch=8),
    )

    assert clarify.kind is DecisionKind.CLARIFY
    assert clarify.reason == 'missing_distance'
    assert completed.kind is DecisionKind.MISSION
    assert completed.mission.steps[0].distance_m == 1.0
    assert completed.mission.token.runtime_instance_id == 'runtime-b'
    assert completed.mission.token.admission_epoch == 8

    expired = AgentCore('agent-b', clock=clock.now)
    expired.handle_turn(make_turn('前进'), make_state())
    clock.advance(15.0)
    no_longer_pending = expired.handle_turn(
        make_turn('1 米', sequence=2), make_state()
    )
    assert no_longer_pending.kind is DecisionKind.LLM_NEEDED


def test_voice_instance_change_clears_clarification_and_old_instance_is_retired():
    core = make_core()

    core.handle_turn(make_turn('前进'), make_state())
    changed = core.handle_turn(
        make_turn('前进 1 米', sequence=1, voice_instance_id='voice-b'),
        make_state(),
    )
    late = core.handle_turn(
        make_turn('1 米', sequence=2, voice_instance_id='voice-a'),
        make_state(),
    )

    assert changed.kind is DecisionKind.MISSION
    assert late.kind is DecisionKind.IGNORE


def test_bad_envelope_is_ignored_and_stop_phrase_wins_over_motion():
    core = make_core()

    bad = core.handle_turn(
        make_turn('前进 1 米', confidence=float('nan')), make_state()
    )
    stop = core.handle_turn(
        make_turn('紧急停止然后前进 2 米', sequence=2), make_state()
    )
    punctuation_stop = core.handle_turn(
        make_turn('前进 1 米；紧急停止', sequence=3), make_state()
    )

    assert bad.kind is DecisionKind.IGNORE
    assert bad.reason == 'invalid_envelope'
    assert stop.kind is DecisionKind.STOP
    assert stop.reason == 'voice_stop'
    assert punctuation_stop.kind is DecisionKind.STOP
    assert punctuation_stop.reason == 'voice_stop'


def test_runtime_snapshot_is_required_and_malformed_union_is_rejected_by_validator():
    core = make_core()

    missing = core.handle_turn(make_turn('前进 1 米'), None)
    assert missing.kind is DecisionKind.REPLY
    assert missing.reason == 'runtime_snapshot_missing'

    state = make_state()
    token = core.handle_turn(
        make_turn('前进 1 米', sequence=2), state
    ).mission.token
    invalid = MissionProposal(
        token=token,
        steps=(MissionStep(MissionStep.MOVE_DISTANCE, distance_m=1.0, angle_rad=1.0),),
    )
    result = SemanticValidator().validate(invalid, token)

    assert not result.accepted
    assert result.rejection.reason == 'invalid_union'


def test_validator_requires_original_planning_context_and_rejects_mismatch():
    core = make_core()
    decision = core.handle_turn(make_turn('前进 1 米'), make_state())
    token = decision.mission.token
    proposal = MissionProposal(
        token=token,
        steps=(MissionStep(MissionStep.MOVE_DISTANCE, distance_m=1.0),),
    )
    validator = SemanticValidator()

    with pytest.raises(TypeError):
        validator.validate(proposal)

    changed_token = replace(token, admission_epoch=token.admission_epoch + 1)
    mismatch = validator.validate(
        MissionProposal(proposal.steps, changed_token), token
    )

    assert not mismatch.accepted
    assert mismatch.rejection.reason == 'planning_context_mismatch'


def test_angle_uses_the_runtime_binary32_wire_boundary_for_exact_360_degrees():
    core = make_core()
    wire_max = _binary32(6.283185)

    left = core.handle_turn(make_turn('左转 360 度'), make_state())
    right = core.handle_turn(
        make_turn('右转 360 度', sequence=2), make_state()
    )

    assert AgentPolicy().rotate_angle_max_rad == wire_max
    assert left.kind is DecisionKind.MISSION
    assert right.kind is DecisionKind.MISSION
    assert left.steps[0].angle_rad == wire_max
    assert right.steps[0].angle_rad == -wire_max


def test_validator_rejects_angle_above_runtime_binary32_wire_limit():
    core = make_core()
    token = core.handle_turn(
        make_turn('前进 1 米'), make_state()
    ).mission.token
    wire_max = _binary32(6.283185)
    over_wire = math.nextafter(math.tau, -math.inf)
    proposal = MissionProposal(
        token=token,
        steps=(MissionStep(MissionStep.ROTATE_ANGLE, angle_rad=over_wire),),
    )

    assert over_wire < math.tau
    assert _binary32(over_wire) > wire_max
    result = core.validator.validate(proposal, token)

    assert not result.accepted
    assert result.rejection.reason == 'angle_out_of_range'


@pytest.mark.parametrize('value', [10**400, math.nan, math.inf, -math.inf])
def test_validator_structurally_rejects_non_finite_or_overflowing_numbers(value):
    core = make_core()
    token = core.handle_turn(
        make_turn('前进 1 米'), make_state()
    ).mission.token
    proposal = MissionProposal(
        token=token,
        steps=(MissionStep(MissionStep.MOVE_DISTANCE, distance_m=value),),
    )

    result = core.validator.validate(proposal, token)

    assert not result.accepted
    assert result.rejection.reason == 'non_finite_step'


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_validator_rejects_non_finite_angle_values(value):
    core = make_core()
    token = core.handle_turn(
        make_turn('前进 1 米'), make_state()
    ).mission.token
    proposal = MissionProposal(
        token=token,
        steps=(MissionStep(MissionStep.ROTATE_ANGLE, angle_rad=value),),
    )

    result = core.validator.validate(proposal, token)

    assert not result.accepted
    assert result.rejection.reason == 'non_finite_step'


def test_warm_rule_decision_p95_stays_within_fifty_milliseconds():
    core = make_core()
    state = make_state()
    samples = []

    for sequence in range(1, 1001):
        started = time.perf_counter()
        decision = core.handle_turn(
            make_turn('前进 1 米', sequence=sequence), state
        )
        samples.append(time.perf_counter() - started)
        assert decision.kind is DecisionKind.MISSION

    samples.sort()
    p95 = samples[int(len(samples) * 0.95) - 1]
    assert p95 <= 0.05
