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

import copy
import json
import math

import pytest

from voice_nav_demo_command_replay import evidence_from_log, validate


def _command_evidence():
    return {
        'schema_version': 4,
        'head': 'a' * 40,
        'scenario': 'command',
        'simulation_only': True,
        'voice': {
            'turns': [{
                'voice_instance_id': 'voice-a',
                'voice_seq': 1,
                'session_id': 'session-a',
                'turn_id': 'turn-a',
                'kind': 1,
                'text': '右转九十度',
            }],
            'speak_completed_count': 1,
            'speak_texts': ['任务已完成。'],
        },
        'provider': {'llm_http_request_count': 0},
        'missions': {
            'unique_goal_count': 1,
            'successful_goal_count': 1,
            'steps': [{'kind': 2, 'angle_rad': -math.pi / 2}],
        },
        'motion': {
            'displacement_m': 0.03,
            'yaw_delta_rad': -math.pi / 2,
            'controller_nonzero_observed': True,
            'final_gate_inhibited': True,
            'final_command_is_zero': True,
            'final_odometry_is_stationary': True,
            'stationary_hold_ms': 200,
        },
        'teardown': 'bounded_clean_exit',
        'REAL_AUDIO_MODELS': 'NOT_RUN',
        'REAL_LLM_CORPUS': 'NOT_RUN',
    }


def test_command_evidence_replay_accepts_the_frozen_causal_record():
    assert validate(_command_evidence()) is True


def test_command_evidence_replay_reads_one_launch_prefixed_record():
    evidence = _command_evidence()
    raw_log = '[scripted_voice_demo-9] EVIDENCE scripted_voice_demo ' + json.dumps(
        evidence, ensure_ascii=False,
    )

    assert evidence_from_log(raw_log) == evidence


@pytest.mark.parametrize(
    ('path', 'value'),
    [
        (('provider', 'llm_http_request_count'), 1),
        (('voice', 'turns'), []),
        (('missions', 'steps'), [{'kind': 1, 'angle_rad': 0.0}]),
        (('missions', 'steps'), [{'kind': 2, 'angle_rad': -math.pi / 2 + 0.13}]),
        (('motion', 'displacement_m'), 0.11),
        (('motion', 'yaw_delta_rad'), -math.pi / 2 - 0.13),
        (('motion', 'stationary_hold_ms'), 199),
    ],
)
def test_command_evidence_replay_rejects_causal_mutations(path, value):
    evidence = copy.deepcopy(_command_evidence())
    target = evidence
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value

    with pytest.raises(AssertionError):
        validate(evidence)
