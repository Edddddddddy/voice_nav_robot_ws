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

"""Replay the post-exit, schema-v4 scripted STOP evidence envelope."""

import argparse
import hashlib
import json
from pathlib import Path
import re


PRODUCT_EVIDENCE_PREFIX = 'EVIDENCE scripted_voice_demo '
EVIDENCE_PREFIX = 'EVIDENCE voice_nav_stop_post_exit '
STOP_TEXTS = frozenset(('小智停止', '紧急停止'))
_SHA256 = re.compile(r'[0-9a-f]{64}')


def _assert_integer(value):
    assert isinstance(value, int) and not isinstance(value, bool)


def _validate_product(product):
    assert set(product) == {
        'schema_version', 'head', 'scenario', 'simulation_only', 'voice',
        'missions', 'stop', 'motion', 'speak_completed_count',
        'direct_stop_request_count', 'speak_barged_in_count',
        'audio_fence',
        'REAL_AUDIO_MODELS', 'REAL_LLM_CORPUS',
    }
    assert product['schema_version'] == 2
    assert isinstance(product['head'], str)
    assert re.fullmatch(r'[0-9a-f]{40}', product['head'])
    assert product['scenario'] == 'stop'
    assert product['simulation_only'] is True
    assert product['REAL_AUDIO_MODELS'] == 'NOT_RUN'
    assert product['REAL_LLM_CORPUS'] == 'NOT_RUN'

    voice = product['voice']
    assert set(voice) == {'turns'}
    assert isinstance(voice['turns'], list) and len(voice['turns']) == 2
    command, stop = voice['turns']
    assert all(set(turn) == {
        'voice_instance_id', 'voice_seq', 'session_id', 'turn_id', 'kind', 'text',
    } for turn in voice['turns'])
    assert command['voice_instance_id'] == stop['voice_instance_id']
    assert command['session_id'] == stop['session_id']
    assert command['voice_seq'] == 1
    assert stop['voice_seq'] == 2
    assert command['kind'] == 1
    assert stop['kind'] == 2
    assert command['text']
    assert stop['text'] in STOP_TEXTS
    assert command['turn_id'] != stop['turn_id']

    assert product['missions'] == {
        'unique_goal_count': 1,
        'terminal_non_success_goal_count': 1,
    }
    assert product['stop'] == {
        'turn_count': 1,
        'controller_nonzero_before_stop': True,
        'post_stop_nonzero_command_observed': False,
    }
    motion = product['motion']
    assert set(motion) == {
        'displacement_m', 'final_gate_inhibited', 'final_zero_stationary',
        'stationary_hold_ms',
    }
    assert isinstance(motion['displacement_m'], (int, float))
    assert motion['final_gate_inhibited'] is True
    assert motion['final_zero_stationary'] is True
    assert motion['stationary_hold_ms'] >= 200

    assert product['speak_completed_count'] == 1
    assert product['direct_stop_request_count'] == 1
    assert product['speak_barged_in_count'] == 1
    _validate_audio_fence(product['audio_fence'])


def _validate_provider_measurement(measurement):
    assert set(measurement) == {'transport', 'request_count'}
    assert measurement['transport'] == '127.0.0.1'
    _assert_integer(measurement['request_count'])
    assert measurement['request_count'] == 0


def _validate_audio_fence(fence):
    assert set(fence) == {
        'generation_before', 'generation_after', 'stale_pcm_after',
    }
    for value in fence.values():
        _assert_integer(value)
        assert value >= 0
    assert fence['generation_before'] > 0
    assert fence['generation_after'] > fence['generation_before']
    assert fence['stale_pcm_after'] == 0


def _validate_process_identity(identity):
    assert set(identity) == {
        'pid', 'starttime_ticks', 'executable', 'cmdline', 'owner_uid',
    }
    _assert_integer(identity['pid'])
    assert identity['pid'] > 0
    _assert_integer(identity['starttime_ticks'])
    assert identity['starttime_ticks'] >= 0
    assert isinstance(identity['executable'], str)
    assert identity['executable']
    assert isinstance(identity['cmdline'], list)
    assert identity['cmdline'] and all(
        isinstance(argument, str) and argument for argument in identity['cmdline']
    )
    _assert_integer(identity['owner_uid'])
    assert identity['owner_uid'] >= 0


def _validate_session(session):
    assert set(session) == {'id', 'process_group_id', 'start_identity'}
    _assert_integer(session['id'])
    assert session['id'] > 0
    _assert_integer(session['process_group_id'])
    assert session['process_group_id'] == session['id']
    _validate_process_identity(session['start_identity'])
    assert session['start_identity']['pid'] == session['id']


def _validate_remaining_member(member, session_id):
    assert set(member) == {
        'pid', 'ppid', 'state', 'process_group_id', 'session_id',
        'starttime_ticks', 'executable', 'cmdline', 'owner_uid',
    }
    _assert_integer(member['pid'])
    assert member['pid'] > 0
    _assert_integer(member['ppid'])
    assert member['ppid'] >= 0
    assert isinstance(member['state'], str) and len(member['state']) == 1
    _assert_integer(member['process_group_id'])
    assert member['process_group_id'] > 0
    _assert_integer(member['session_id'])
    assert member['session_id'] == session_id
    _assert_integer(member['starttime_ticks'])
    assert member['starttime_ticks'] >= 0
    assert isinstance(member['executable'], str) and member['executable']
    assert isinstance(member['cmdline'], list)
    assert member['cmdline'] and all(
        isinstance(argument, str) and argument for argument in member['cmdline']
    )
    _assert_integer(member['owner_uid'])
    assert member['owner_uid'] >= 0


def _validate_post_exit(post_exit):
    assert set(post_exit) == {
        'exit_code', 'product_descendants_empty', 'product_owners_empty',
        'start_count', 'restart_count', 'session', 'remaining_members',
    }
    _assert_integer(post_exit['exit_code'])
    assert post_exit['exit_code'] in (0, -2)
    _assert_integer(post_exit['start_count'])
    assert post_exit['start_count'] == 1
    _validate_session(post_exit['session'])
    assert isinstance(post_exit['remaining_members'], list)
    for member in post_exit['remaining_members']:
        _validate_remaining_member(member, post_exit['session']['id'])
    assert post_exit['product_descendants_empty'] is True
    assert post_exit['product_descendants_empty'] is (
        not post_exit['remaining_members']
    )
    assert post_exit['product_owners_empty'] is True
    assert post_exit['product_owners_empty'] is not any(
        member['owner_uid'] == post_exit['session']['start_identity']['owner_uid']
        for member in post_exit['remaining_members']
    )
    _assert_integer(post_exit['restart_count'])
    assert post_exit['restart_count'] == post_exit['start_count'] - 1 == 0


def validate(evidence, product=None, product_json=None):
    """Validate the v4 envelope and its optional raw product association."""
    assert set(evidence) == {
        'schema_version', 'product_sha256', 'head', 'scenario',
        'provider_measurement', 'audio_fence', 'post_exit',
    }
    assert evidence['schema_version'] == 4
    assert isinstance(evidence['product_sha256'], str)
    assert _SHA256.fullmatch(evidence['product_sha256'])
    assert isinstance(evidence['head'], str)
    assert re.fullmatch(r'[0-9a-f]{40}', evidence['head'])
    assert evidence['scenario'] == 'stop'
    _validate_provider_measurement(evidence['provider_measurement'])
    _validate_audio_fence(evidence['audio_fence'])
    _validate_post_exit(evidence['post_exit'])
    if product is not None:
        _validate_product(product)
        assert product['head'] == evidence['head']
        assert product['scenario'] == evidence['scenario']
        assert product['audio_fence'] == evidence['audio_fence']
    if product_json is not None:
        assert isinstance(product_json, str)
        assert hashlib.sha256(product_json.encode('utf-8')).hexdigest() == evidence[
            'product_sha256'
        ]
    return True


def _records_from_log(raw_log):
    product_records = []
    product_jsons = []
    envelope_records = []
    for line in raw_log.splitlines():
        if PRODUCT_EVIDENCE_PREFIX in line:
            product_json = line.split(PRODUCT_EVIDENCE_PREFIX, 1)[1]
            product_jsons.append(product_json)
            product_records.append(json.loads(product_json))
        if EVIDENCE_PREFIX in line:
            envelope_records.append(json.loads(line.split(EVIDENCE_PREFIX, 1)[1]))
    assert len(product_records) == 1
    assert len(envelope_records) == 1
    return product_records[0], product_jsons[0], envelope_records[0]


def evidence_from_log(raw_log):
    """Load exactly one product and one final post-exit envelope."""
    product, product_json, envelope = _records_from_log(raw_log)
    validate(envelope, product=product, product_json=product_json)
    return envelope


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('raw_log', type=Path)
    arguments = parser.parse_args()
    evidence_from_log(arguments.raw_log.read_text(encoding='utf-8'))


if __name__ == '__main__':
    main()
