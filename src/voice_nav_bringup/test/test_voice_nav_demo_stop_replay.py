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
import hashlib
import json

import pytest

from voice_nav_demo_stop_replay import evidence_from_log, validate


def _stop_evidence():
    return {
        'schema_version': 2,
        'head': 'a' * 40,
        'scenario': 'stop',
        'simulation_only': True,
        'voice': {
            'turns': [
                {
                    'voice_instance_id': 'voice-a',
                    'voice_seq': 1,
                    'session_id': 'session-a',
                    'turn_id': 'turn-a',
                    'kind': 1,
                    'text': '前进 2 米',
                },
                {
                    'voice_instance_id': 'voice-a',
                    'voice_seq': 2,
                    'session_id': 'session-a',
                    'turn_id': 'turn-b',
                    'kind': 2,
                    'text': '小智停止',
                },
            ],
        },
        'missions': {
            'unique_goal_count': 1,
            'terminal_non_success_goal_count': 1,
        },
        'stop': {
            'turn_count': 1,
            'controller_nonzero_before_stop': True,
            'post_stop_nonzero_command_observed': False,
        },
        'motion': {
            'displacement_m': 1.5,
            'final_gate_inhibited': True,
            'final_zero_stationary': True,
            'stationary_hold_ms': 200,
        },
        'speak_completed_count': 1,
        'direct_stop_request_count': 1,
        'speak_barged_in_count': 1,
        'audio_fence': {
            'generation_before': 1,
            'generation_after': 2,
            'stale_pcm_after': 0,
        },
        'REAL_AUDIO_MODELS': 'NOT_RUN',
        'REAL_LLM_CORPUS': 'NOT_RUN',
    }


def _product_json(product=None):
    return json.dumps(
        _stop_evidence() if product is None else product,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    )


def _stop_envelope(product_json=None):
    product_json = _product_json() if product_json is None else product_json
    return {
        'schema_version': 4,
        'product_sha256': hashlib.sha256(product_json.encode('utf-8')).hexdigest(),
        'head': 'a' * 40,
        'scenario': 'stop',
        'provider_measurement': {
            'transport': '127.0.0.1',
            'request_count': 0,
        },
        'audio_fence': {
            'generation_before': 1,
            'generation_after': 2,
            'stale_pcm_after': 0,
        },
        'post_exit': {
            'exit_code': 0,
            'product_descendants_empty': True,
            'product_owners_empty': True,
            'start_count': 1,
            'restart_count': 0,
            'session': {
                'id': 4242,
                'process_group_id': 4242,
                'start_identity': {
                    'pid': 4242,
                    'starttime_ticks': 123,
                    'executable': '/usr/lib/voice_nav/scripted_voice_demo',
                    'cmdline': [
                        '/usr/lib/voice_nav/scripted_voice_demo',
                        '--ros-args',
                    ],
                    'owner_uid': 1000,
                },
            },
            'remaining_members': [],
        },
    }


def _raw_log(product_json=None, envelope=None):
    product_json = _product_json() if product_json is None else product_json
    envelope = _stop_envelope(product_json) if envelope is None else envelope
    return '\n'.join((
        '[scripted_voice_demo-9] EVIDENCE scripted_voice_demo ' + product_json,
        '[stop_harness] EVIDENCE voice_nav_stop_post_exit ' + json.dumps(
            envelope, sort_keys=True, separators=(',', ':')
        ),
    ))


def test_stop_evidence_replay_accepts_the_frozen_v3_envelope():
    assert validate(_stop_envelope()) is True


def test_stop_evidence_replay_reads_product_and_one_post_exit_envelope():
    envelope = _stop_envelope()
    assert evidence_from_log(_raw_log(envelope=envelope)) == envelope


def test_stop_replay_rejects_an_isolated_handwritten_product_record():
    product_json = _product_json()
    with pytest.raises(AssertionError):
        evidence_from_log(
            '[scripted_voice_demo-9] EVIDENCE scripted_voice_demo ' + product_json
        )


def test_stop_replay_rejects_a_provider_count_without_loopback_measurement():
    envelope = _stop_envelope()
    envelope['provider_measurement']['transport'] = 'derived_from_stop_turn'
    with pytest.raises(AssertionError):
        validate(envelope)


def test_stop_replay_rejects_an_envelope_without_post_exit():
    envelope = _stop_envelope()
    del envelope['post_exit']
    with pytest.raises(AssertionError):
        validate(envelope)


def test_stop_replay_rejects_a_member_outside_the_frozen_session():
    envelope = _stop_envelope()
    envelope['post_exit']['remaining_members'] = [{
        'pid': 99,
        'ppid': 1,
        'state': 'S',
        'process_group_id': 99,
        'session_id': 4243,
        'starttime_ticks': 123,
        'executable': '/usr/bin/helper',
        'cmdline': ['/usr/bin/helper', '--different-argv'],
        'owner_uid': 1000,
    }]
    with pytest.raises(AssertionError):
        validate(envelope)


@pytest.mark.parametrize(
    ('path', 'value'),
    [
        (('provider_measurement', 'request_count'), 1),
        (('provider_measurement', 'transport'), 'handwritten_schema'),
        (('audio_fence', 'stale_pcm_after'), 1),
        (('post_exit', 'product_descendants_empty'), False),
        (('post_exit', 'restart_count'), 1),
    ],
)
def test_stop_evidence_replay_rejects_provenance_mutations(path, value):
    evidence = copy.deepcopy(_stop_envelope())
    target = evidence
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value

    with pytest.raises(AssertionError):
        validate(evidence)


def test_stop_evidence_replay_rejects_product_hash_or_head_mismatch():
    product = _stop_evidence()
    product['head'] = 'b' * 40
    product_json = _product_json(product)
    envelope = _stop_envelope()
    with pytest.raises(AssertionError):
        evidence_from_log(_raw_log(product_json=product_json, envelope=envelope))
