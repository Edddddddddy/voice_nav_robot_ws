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

"""Replay and preflight contracts for the real-audio Motion smoke."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import pytest
import threading
import signal
from types import SimpleNamespace


def _load_entry_module():
    source = Path(__file__).resolve().parents[1] / 'voice_nav_motion_smoke.py'
    specification = importlib.util.spec_from_file_location(
        'voice_nav_motion_smoke_replay', source,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def test_runtime_inputs_and_replay_require_the_complete_motion_contract(tmp_path):
    """Do not turn a successful app exit into a false end-to-end PASS."""
    module = _load_entry_module()
    paths = {
        'vad': tmp_path / 'silero_vad.int8.onnx',
        'model': tmp_path / 'model.int8.onnx',
        'tokens': tmp_path / 'tokens.txt',
        'chaowen': tmp_path / 'chaowen',
        'prefix': tmp_path / 'sherpa-tts-on',
    }
    for name, path in paths.items():
        if name in ('chaowen', 'prefix'):
            path.mkdir()
        else:
            path.write_bytes(b'fixture')

    missing_head = module.validate_runtime_inputs(
        exact_head='not-an-exact-head', **paths,
    )
    assert missing_head['ok'] is False
    assert missing_head['reason'] == 'exact_head_must_be_40_hex'

    expected_assets = {
        name: {
            'size': path.stat().st_size,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        }
        for name, path in (
            ('vad', paths['vad']),
            ('model', paths['model']),
            ('tokens', paths['tokens']),
        )
    }
    ready = module.validate_runtime_inputs(
        exact_head='d0eb8fd2064e4c710dc19aa3ae520b476005ac42',
        expected_assets=expected_assets,
        **paths,
    )
    assert ready['ok'] is True
    assert ready['paths']['prefix'] == str(paths['prefix'].resolve())
    assert ready['asset_provenance']['model']['sha256'] == expected_assets['model']['sha256']

    evidence = {
        'command': list(module.build_app_command()),
        'status': 'passed',
        'voice_turn_count': 1,
        'command_count': 1,
        'mission_count': 1,
        'llm_calls': 0,
        'controller_nonzero': True,
        'final_zero': True,
        'final_gate_inhibited': True,
        'final_stationary': True,
        'stationary_ms': 200,
        'speak_status_completed_count': 0,
        'speak_completed_count': 1,
        'speak_completion_proof': 'microphone_once_frontend_exit_0',
        'cleanup': {
            'status': 'graceful',
            'app_returncode': 0,
            'app_alive': False,
            'owned_session_alive': False,
            'owned_session_groups': [],
        },
    }
    assert module.validate_smoke_evidence(evidence) == {'ok': True, 'reason': ''}
    evidence['cleanup'] = {'status': 'graceful'}
    assert module.validate_smoke_evidence(evidence)['reason'] == (
        'cleanup_returncode_must_be_zero'
    )
    evidence['cleanup'] = {'status': 'failed', 'app_returncode': 0}
    assert module.validate_smoke_evidence(evidence)['reason'] == (
        'cleanup_must_be_graceful'
    )
    evidence['cleanup'] = {'status': 'graceful', 'app_returncode': 0}
    evidence['speak_completed_count'] = 0
    assert module.validate_smoke_evidence(evidence)['reason'] == (
        'speak_completed_count_must_be_one'
    )


def test_microphone_once_frontend_exit_proves_one_speak_when_status_is_not_observed():
    """The strict microphone-once frontend contract is authoritative when status is 0."""
    module = _load_entry_module()

    result = module._authoritative_speak_completion(
        command=module.build_app_command(),
        app_returncode=0,
        speak_status_completed_count=0,
    )

    assert result == {
        'ok': True,
        'speak_completed_count': 1,
        'speak_completion_proof': 'microphone_once_frontend_exit_0',
        'speak_status_completed_count': 0,
    }


def test_observer_keeps_external_speak_status_count_as_raw_diagnostic():
    """External action status remains diagnostic and separate from frontend proof."""
    observer_source = Path(__file__).resolve().parents[1] / '_motion_smoke_observer.py'
    observer_namespace = {}
    exec(observer_source.read_text(encoding='utf-8'), observer_namespace)

    class FakeVoiceTurn:
        COMMAND = 1
        STOP = 2

    class FakeLedger:
        voice_turn_events = ()
        command_events = ()
        mission_status_events = ()
        odometry_events = ()
        speak_success_events = ((1, 'goal'),)
        mission_goal_ids = frozenset()
        successful_mission_ids = frozenset()
        latest_safety_sample = SimpleNamespace(mission_gate_inhibited=True)

        @property
        def controller_nonzero_observed(self):
            return False

        @property
        def latest_controller_zero(self):
            return True

        @property
        def latest_odom_stationary(self):
            return True

    observer = observer_namespace['MotionSmokeObserver'].__new__(
        observer_namespace['MotionSmokeObserver'],
    )
    observer._lock = threading.RLock()
    observer._ledger = FakeLedger()
    observer._voice_turn = FakeVoiceTurn

    snapshot = observer.snapshot()

    assert snapshot['speak_status_completed_count'] == 1
    assert 'speak_completed_count' not in snapshot


def test_smoke_evidence_requires_the_authoritative_microphone_once_proof():
    """A final count of one is valid only with the fixed frontend proof."""
    module = _load_entry_module()
    evidence = {
        'command': list(module.build_app_command()),
        'voice_turn_count': 1,
        'command_count': 1,
        'mission_count': 1,
        'llm_calls': 0,
        'speak_status_completed_count': 0,
        'speak_completed_count': 1,
        'speak_completion_proof': 'microphone_once_frontend_exit_0',
        'controller_nonzero': True,
        'final_zero': True,
        'final_gate_inhibited': True,
        'final_stationary': True,
        'stationary_ms': 200,
        'cleanup': {
            'status': 'graceful',
            'app_returncode': 0,
            'app_alive': False,
            'owned_session_alive': False,
        },
    }
    assert module.validate_smoke_evidence(evidence) == {'ok': True, 'reason': ''}

    evidence['speak_completion_proof'] = 'unavailable'
    assert module.validate_smoke_evidence(evidence)['reason'] == (
        'speak_completion_proof_unavailable'
    )


def test_nonzero_microphone_once_frontend_cannot_promote_speak_completion():
    """A nonzero frontend return code remains a bounded Speak failure."""
    module = _load_entry_module()

    result = module._authoritative_speak_completion(
        command=module.build_app_command(),
        app_returncode=1,
        speak_status_completed_count=0,
    )

    assert result['ok'] is False
    assert result['reason'] == 'speak_completion_app_returncode_nonzero'
    assert result['speak_completed_count'] == 0


def test_observed_single_speak_status_passes_and_remains_a_raw_source():
    """One external completed status is retained while frontend proof stays authoritative."""
    module = _load_entry_module()

    result = module._authoritative_speak_completion(
        command=module.build_app_command(),
        app_returncode=0,
        speak_status_completed_count=1,
    )

    assert result == {
        'ok': True,
        'speak_completed_count': 1,
        'speak_completion_proof': 'microphone_once_frontend_exit_0',
        'speak_status_completed_count': 1,
    }


def test_duplicate_speak_statuses_fail_closed():
    """More than one external completed Speak status cannot be collapsed to one."""
    module = _load_entry_module()

    result = module._authoritative_speak_completion(
        command=module.build_app_command(),
        app_returncode=0,
        speak_status_completed_count=2,
    )

    assert result['ok'] is False
    assert result['reason'] == 'speak_status_completed_count_multiple'
    assert result['speak_completed_count'] == 0


def test_unknown_app_composition_fails_closed_even_with_zero_status():
    """The frontend proof is not portable to another app composition."""
    module = _load_entry_module()

    result = module._authoritative_speak_completion(
        command=module.build_app_command()[:-1] + ('console',),
        app_returncode=0,
        speak_status_completed_count=0,
    )

    assert result['ok'] is False
    assert result['reason'] == 'speak_completion_unknown_composition'
    assert result['speak_completed_count'] == 0


def test_persisted_motion_artifact_replays_without_rewriting_its_exact_head():
    """Replay the supplied product artifact while preserving its persisted provenance."""
    artifact_value = os.environ.get('VOICE_NAV_ISSUE174_REPLAY_ARTIFACT')
    if not artifact_value:
        pytest.skip('VOICE_NAV_ISSUE174_REPLAY_ARTIFACT is not set')
    artifact = json.loads(Path(artifact_value).read_text(encoding='utf-8'))
    module = _load_entry_module()
    persisted_head = artifact['exact_head']
    raw_count = artifact.get(
        'speak_status_completed_count',
        artifact.get('speak_completed_count'),
    )

    proof = module._authoritative_speak_completion(
        command=tuple(artifact['command']),
        app_returncode=artifact['cleanup']['app_returncode'],
        speak_status_completed_count=raw_count,
    )
    replayed = {**artifact, **proof}

    assert replayed['exact_head'] == persisted_head
    assert replayed['speak_status_completed_count'] == 0
    assert module.validate_smoke_evidence(replayed) == {'ok': True, 'reason': ''}


def test_stationarity_accepts_only_the_trailing_continuous_hold():
    """A longer middle pause must not satisfy the final stationarity hold."""
    observer_source = Path(__file__).resolve().parents[1] / '_motion_smoke_observer.py'
    observer_namespace = {}
    exec(observer_source.read_text(encoding='utf-8'), observer_namespace)
    events = (
        (0, None, False),
        (100_000_000, None, True),
        (400_000_000, None, False),
        (500_000_000, None, True),
        (650_000_000, None, False),
        (700_000_000, None, True),
        (850_000_000, None, True),
    )
    assert observer_namespace['MotionSmokeObserver']._stationary_hold_ms(events, 0) == 150


def test_observer_snapshot_reports_missing_safety_sample_as_bounded_failure():
    """Missing safety data is evidence failure, not an AttributeError."""
    observer_source = Path(__file__).resolve().parents[1] / '_motion_smoke_observer.py'
    observer_namespace = {}
    exec(observer_source.read_text(encoding='utf-8'), observer_namespace)

    class FakeVoiceTurn:
        COMMAND = 1
        STOP = 2

    class FakeLedger:
        voice_turn_events = ()
        command_events = ()
        mission_status_events = ()
        odometry_events = ()
        speak_success_events = ()
        mission_goal_ids = set()
        successful_mission_ids = set()
        latest_safety_sample = None

        @property
        def controller_nonzero_observed(self):
            return False

        @property
        def latest_controller_zero(self):
            return False

        @property
        def latest_odom_stationary(self):
            return False

    observer = observer_namespace['MotionSmokeObserver'].__new__(
        observer_namespace['MotionSmokeObserver'],
    )
    observer._lock = threading.RLock()
    observer._ledger = FakeLedger()
    observer._voice_turn = FakeVoiceTurn
    assert observer.snapshot() == {
        'observer_error': 'latest_safety_sample_unavailable',
    }


def test_runtime_inputs_reject_unlocked_sensevoice_identity(tmp_path):
    """The installed microphone path must verify the locked file identity."""
    module = _load_entry_module()
    paths = {
        'vad': tmp_path / 'silero_vad.int8.onnx',
        'model': tmp_path / 'model.int8.onnx',
        'tokens': tmp_path / 'tokens.txt',
        'chaowen': tmp_path / 'chaowen',
        'prefix': tmp_path / 'sherpa-tts-on',
    }
    for name, path in paths.items():
        if name in ('chaowen', 'prefix'):
            path.mkdir()
        else:
            path.write_bytes((name + '-fixture').encode('utf-8'))

    result = module.validate_runtime_inputs(
        exact_head='d0eb8fd2064e4c710dc19aa3ae520b476005ac42',
        **paths,
    )
    assert result['ok'] is False
    assert result['reason'] == 'vad_size_mismatch'
    assert result['asset_provenance']['vad']['path'] == str(paths['vad'].resolve())


def test_timeout_cleanup_interrupts_app_then_proves_owned_session_exit():
    """Timeout cleanup triggers the app finally before any escalation."""
    module = _load_entry_module()
    calls = []
    app_alive = [True]
    session_alive = [False]

    def send_app_signal(_process, signum):
        calls.append(('app', signum))
        if signum == signal.SIGKILL:
            app_alive[0] = False
        return True

    def send_session_signal(group_id, signum):
        calls.append((group_id, signum))
        return True

    def wait_for_exit(_process, _timeout_s):
        if calls[-1] == ('app', signal.SIGTERM):
            app_alive[0] = False
        return not app_alive[0]

    result = module._cleanup_timed_out_app('app', session_groups=(4242,),
        send_app_signal=send_app_signal,
        send_session_signal=send_session_signal,
        wait_for_exit=wait_for_exit,
        session_alive=lambda _groups: session_alive[0],
    )

    assert calls == [('app', signal.SIGINT), ('app', signal.SIGTERM)]
    assert result == {
        'status': 'timeout_terminated',
        'app_alive': False,
        'owned_session_alive': False,
        'owned_session_groups': [4242],
    }


def test_timeout_cleanup_never_reports_graceful_when_app_needs_kill():
    """A forced timeout remains a failure even when all processes are gone."""
    module = _load_entry_module()
    calls = []
    app_alive = [True]

    def send_app_signal(_process, signum):
        calls.append(('app', signum))
        if signum == signal.SIGKILL:
            app_alive[0] = False
        return True

    def wait_for_exit(_process, _timeout_s):
        return calls[-1] == ('app', signal.SIGKILL)

    def send_session_signal(group_id, signum):
        calls.append((group_id, signum))
        app_alive[0] = False

    result = module._cleanup_timed_out_app('app', session_groups=(),
        send_app_signal=send_app_signal,
        send_session_signal=send_session_signal,
        wait_for_exit=wait_for_exit,
        session_alive=lambda _groups: False,
    )

    assert calls == [('app', signal.SIGINT), ('app', signal.SIGTERM),
        ('app', signal.SIGKILL)]
    assert result['status'] == 'timeout_killed'
    assert result['app_alive'] is False
    assert result['owned_session_alive'] is False
    assert result['status'] != 'graceful'


def test_observer_teardown_uses_bounded_supported_executor_shutdown():
    """Observer teardown must use the available bounded executor API."""
    module = _load_entry_module()
    calls = []

    class FakeExecutor:
        def remove_node(self, node):
            calls.append(('remove_node', node))

        def shutdown(self, *, timeout_sec):
            calls.append(('shutdown', timeout_sec))

    class FakeThread:
        def join(self, *, timeout):
            calls.append(('join', timeout))

    class FakeObserver:
        def close(self):
            calls.append(('observer.close',))

    class FakeNode:
        def destroy_node(self):
            calls.append(('node.destroy',))

    class FakeRclpy:
        @staticmethod
        def ok():
            return True

        @staticmethod
        def shutdown():
            calls.append(('rclpy.shutdown',))

    module._shutdown_observer_runtime(
        stop_spin=lambda: calls.append(('stop_spin',)),
        executor=FakeExecutor(),
        spin_thread=FakeThread(),
        observer=FakeObserver(),
        node=FakeNode(),
        rclpy=FakeRclpy(),
    )

    assert [call[0] for call in calls] == [
        'stop_spin',
        'join',
        'observer.close',
        'remove_node',
        'node.destroy',
        'shutdown',
        'rclpy.shutdown',
    ]
    assert calls[1] == ('join', 5)
    assert calls[5] == ('shutdown', 5.0)


def test_observer_teardown_error_still_writes_bounded_failure_artifact(tmp_path):
    """Cleanup exceptions remain observable without losing the smoke artifact."""
    module = _load_entry_module()

    class FakeExecutor:
        def remove_node(self, _node):
            return None

        def shutdown(self, *, timeout_sec):
            assert timeout_sec == 5.0
            raise RuntimeError('executor shutdown fixture failure')

    class FakeThread:
        def join(self, *, timeout):
            assert timeout == 5

    class FakeObserver:
        def close(self):
            return None

    class FakeNode:
        def destroy_node(self):
            return None

    class FakeRclpy:
        @staticmethod
        def ok():
            return False

    cleanup_errors = module._shutdown_observer_runtime(
        stop_spin=lambda: None,
        executor=FakeExecutor(),
        spin_thread=FakeThread(),
        observer=FakeObserver(),
        node=FakeNode(),
        rclpy=FakeRclpy(),
    )
    assert cleanup_errors == [
        'executor_shutdown:executor shutdown fixture failure',
    ]

    artifact = tmp_path / 'bounded-failure.json'
    payload = module._result(
        status='unavailable',
        reason='observer_error:executor shutdown fixture failure',
        head='d0eb8fd2064e4c710dc19aa3ae520b476005ac42',
        artifact=artifact,
        returncode=1,
        snapshot={'observer_error': 'executor shutdown fixture failure'},
        cleanup={
            'status': 'cleanup_failed',
            'app_alive': False,
            'owned_session_alive': False,
            'owned_session_groups': [],
            'cleanup_errors': cleanup_errors,
        },
    )
    assert module._write_and_print(artifact, payload) == 1
    written = json.loads(artifact.read_text(encoding='utf-8'))
    assert written['status'] == 'unavailable'
    assert written['cleanup']['status'] == 'cleanup_failed'
    assert written['cleanup']['app_returncode'] == 1
    assert written['cleanup']['cleanup_errors'] == cleanup_errors
