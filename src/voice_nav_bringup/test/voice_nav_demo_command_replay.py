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

"""Replay the schema-versioned evidence emitted by the one-command demo."""

import argparse
import json
import math
from pathlib import Path
import re


EVIDENCE_PREFIX = 'EVIDENCE scripted_voice_demo '
COMMAND_TEXT = '右转九十度'


def validate(evidence):
    """Reject command evidence whose causal records or safety bounds changed."""
    assert set(evidence) == {
        'schema_version', 'head', 'scenario', 'simulation_only', 'voice',
        'provider', 'missions', 'motion', 'teardown', 'REAL_AUDIO_MODELS',
        'REAL_LLM_CORPUS',
    }
    assert evidence['schema_version'] == 4
    assert isinstance(evidence['head'], str)
    assert re.fullmatch(r'[0-9a-f]{40}', evidence['head'])
    assert evidence['scenario'] == 'command'
    assert evidence['simulation_only'] is True
    assert evidence['teardown'] == 'bounded_clean_exit'
    assert evidence['REAL_AUDIO_MODELS'] == 'NOT_RUN'
    assert evidence['REAL_LLM_CORPUS'] == 'NOT_RUN'

    voice = evidence['voice']
    assert set(voice) == {'turns', 'speak_completed_count', 'speak_texts'}
    assert voice['speak_completed_count'] == 1
    assert voice['speak_texts'] == ['任务已完成。']
    assert isinstance(voice['turns'], list) and len(voice['turns']) == 1
    turn = voice['turns'][0]
    assert set(turn) == {
        'voice_instance_id', 'voice_seq', 'session_id', 'turn_id', 'kind', 'text',
    }
    assert all(isinstance(turn[key], str) and turn[key] for key in (
        'voice_instance_id', 'session_id', 'turn_id',
    ))
    assert turn['voice_seq'] == 1
    assert turn['kind'] == 1
    assert turn['text'] == COMMAND_TEXT

    assert evidence['provider'] == {'llm_http_request_count': 0}

    missions = evidence['missions']
    assert set(missions) == {'unique_goal_count', 'successful_goal_count', 'steps'}
    assert missions['unique_goal_count'] == 1
    assert missions['successful_goal_count'] == 1
    assert isinstance(missions['steps'], list) and len(missions['steps']) == 1
    step = missions['steps'][0]
    assert set(step) == {'kind', 'angle_rad'}
    assert step['kind'] == 2
    assert abs(step['angle_rad'] + math.pi / 2) <= 0.12

    motion = evidence['motion']
    assert set(motion) == {
        'displacement_m', 'yaw_delta_rad', 'controller_nonzero_observed',
        'final_gate_inhibited', 'final_command_is_zero',
        'final_odometry_is_stationary', 'stationary_hold_ms',
    }
    assert abs(motion['displacement_m']) <= 0.10
    assert abs(motion['yaw_delta_rad'] + math.pi / 2) <= 0.12
    assert motion['controller_nonzero_observed'] is True
    assert motion['final_gate_inhibited'] is True
    assert motion['final_command_is_zero'] is True
    assert motion['final_odometry_is_stationary'] is True
    assert motion['stationary_hold_ms'] >= 200
    return True


def evidence_from_log(raw_log):
    """Load exactly one command record from the launch's unmodified stdout."""
    records = [
        json.loads(line.split(EVIDENCE_PREFIX, 1)[1])
        for line in raw_log.splitlines()
        if EVIDENCE_PREFIX in line
    ]
    assert len(records) == 1
    return records[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('raw_log', type=Path)
    arguments = parser.parse_args()
    validate(evidence_from_log(arguments.raw_log.read_text(encoding='utf-8')))


if __name__ == '__main__':
    main()
