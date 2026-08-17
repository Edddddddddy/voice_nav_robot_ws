#!/usr/bin/env python3
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

"""Installed one-shot composition for the real microphone Motion smoke."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import runpy
import signal
import subprocess
import time


def _contract_module():
    return runpy.run_path(str(Path(__file__).with_name('_real_audio_motion_smoke.py')))


def _sensevoice_asset_module():
    return runpy.run_path(
        str(Path(__file__).with_name('_sensevoice_asset_verifier.py'))
    )


def build_app_command() -> tuple[str, ...]:
    """Return the existing app composition without forwarding child options."""
    return _contract_module()['build_motion_smoke_command']()


def _is_fixed_microphone_once_command(command: object) -> bool:
    try:
        return tuple(command) == build_app_command()
    except TypeError:
        return False


def _authoritative_speak_completion(
    *,
    command: tuple[str, ...] | list[str],
    app_returncode: int | None,
    speak_status_completed_count: object,
) -> dict[str, object]:
    """Promote only the fixed microphone-once frontend's strict Speak proof."""
    raw_count = speak_status_completed_count
    base = {
        'speak_completed_count': 0,
        'speak_completion_proof': 'unavailable',
        'speak_status_completed_count': raw_count,
    }
    if not _is_fixed_microphone_once_command(command):
        return {
            **base,
            'ok': False,
            'reason': 'speak_completion_unknown_composition',
        }
    if (
        not isinstance(app_returncode, int)
        or isinstance(app_returncode, bool)
        or app_returncode != 0
    ):
        return {
            **base,
            'ok': False,
            'reason': 'speak_completion_app_returncode_nonzero',
        }
    if (
        not isinstance(raw_count, int)
        or isinstance(raw_count, bool)
        or raw_count < 0
    ):
        return {
            **base,
            'ok': False,
            'reason': 'speak_status_completed_count_invalid',
        }
    if raw_count > 1:
        return {
            **base,
            'ok': False,
            'reason': 'speak_status_completed_count_multiple',
        }
    return {
        'ok': True,
        'speak_completed_count': 1,
        'speak_completion_proof': 'microphone_once_frontend_exit_0',
        'speak_status_completed_count': raw_count,
    }


def validate_runtime_inputs(
    *,
    exact_head: str,
    vad: Path,
    model: Path,
    tokens: Path,
    chaowen: Path,
    prefix: Path,
    expected_assets: dict[str, dict[str, object]] | None = None,
) -> dict[str, object]:
    """Require explicit absolute runtime paths before creating any process."""
    if re.fullmatch(r'[0-9a-f]{40}', exact_head) is None:
        return {'ok': False, 'reason': 'exact_head_must_be_40_hex'}
    path_values = {
        'vad': Path(vad),
        'model': Path(model),
        'tokens': Path(tokens),
        'chaowen': Path(chaowen),
        'prefix': Path(prefix),
    }
    for name, path in path_values.items():
        if not path.is_absolute():
            return {
                'ok': False,
                'reason': f'{name}_path_must_be_absolute',
            }
        if path.is_symlink():
            return {
                'ok': False,
                'reason': f'{name}_path_must_not_be_symlink',
            }
        expected_directory = name in ('chaowen', 'prefix')
        if expected_directory and not path.is_dir():
            return {
                'ok': False,
                'reason': f'{name}_directory_unavailable',
            }
        if not expected_directory and not path.is_file():
            return {
                'ok': False,
                'reason': f'{name}_file_unavailable',
            }
    asset_result = _sensevoice_asset_module()['verify_sensevoice_assets'](
        vad=path_values['vad'],
        model=path_values['model'],
        tokens=path_values['tokens'],
        expected_assets=expected_assets,
    )
    if not asset_result['ok']:
        return {
            'ok': False,
            'reason': str(asset_result['reason']),
            'asset_provenance': asset_result.get('asset_provenance', {}),
        }
    return {
        'ok': True,
        'reason': '',
        'paths': {name: str(path.resolve()) for name, path in path_values.items()},
        'asset_provenance': asset_result['asset_provenance'],
    }


def validate_smoke_evidence(evidence: dict[str, object]) -> dict[str, object]:
    """Replay the bounded acceptance contract without invoking ROS or audio."""
    observer_error = evidence.get('observer_error')
    if observer_error:
        return {'ok': False, 'reason': str(observer_error)}
    if not _is_fixed_microphone_once_command(evidence.get('command', ())):
        return {
            'ok': False,
            'reason': 'speak_completion_unknown_composition',
        }
    raw_count = evidence.get('speak_status_completed_count')
    if (
        not isinstance(raw_count, int)
        or isinstance(raw_count, bool)
        or raw_count < 0
    ):
        return {
            'ok': False,
            'reason': 'speak_status_completed_count_invalid',
        }
    if raw_count > 1:
        return {
            'ok': False,
            'reason': 'speak_status_completed_count_multiple',
        }
    if evidence.get('speak_completion_proof') != (
        'microphone_once_frontend_exit_0'
    ):
        return {
            'ok': False,
            'reason': 'speak_completion_proof_unavailable',
        }
    checks = (
        ('voice_turn_count', 1, 'voice_turn_count_must_be_one'),
        ('command_count', 1, 'command_count_must_be_one'),
        ('mission_count', 1, 'mission_count_must_be_one'),
        ('llm_calls', 0, 'llm_calls_must_be_zero'),
        ('speak_completed_count', 1, 'speak_completed_count_must_be_one'),
    )
    for field, expected, reason in checks:
        if evidence.get(field) != expected:
            return {'ok': False, 'reason': reason}
    if evidence.get('controller_nonzero') is not True:
        return {'ok': False, 'reason': 'controller_nonzero_not_observed'}
    if evidence.get('final_zero') is not True:
        return {'ok': False, 'reason': 'final_zero_not_observed'}
    if evidence.get('final_gate_inhibited') is not True:
        return {'ok': False, 'reason': 'final_gate_inhibited_not_observed'}
    if evidence.get('final_stationary') is not True:
        return {'ok': False, 'reason': 'final_stationary_not_observed'}
    stationary_ms = evidence.get('stationary_ms')
    if not isinstance(stationary_ms, (int, float)) or stationary_ms < 200:
        return {'ok': False, 'reason': 'stationarity_must_be_at_least_200ms'}
    cleanup = evidence.get('cleanup')
    if not isinstance(cleanup, dict):
        return {'ok': False, 'reason': 'cleanup_must_be_object'}
    cleanup_returncode = cleanup.get('app_returncode')
    if (
        not isinstance(cleanup_returncode, int)
        or isinstance(cleanup_returncode, bool)
        or cleanup_returncode != 0
    ):
        return {'ok': False, 'reason': 'cleanup_returncode_must_be_zero'}
    if cleanup.get('status') != 'graceful':
        return {'ok': False, 'reason': 'cleanup_must_be_graceful'}
    if cleanup.get('app_alive') is not False:
        return {'ok': False, 'reason': 'app_process_still_alive'}
    if cleanup.get('owned_session_alive') is not False:
        return {'ok': False, 'reason': 'owned_session_process_still_alive'}
    return {'ok': True, 'reason': ''}


def _write_artifact(path: Path, payload: dict[str, object]) -> None:
    path = Path(path)
    if not path.is_absolute():
        raise ValueError('smoke artifact path must be absolute')
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() or path.is_symlink():
        raise FileExistsError(f'smoke artifact already exists: {path}')
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            json.dump(payload, stream, ensure_ascii=False, sort_keys=True, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _result(
    *,
    status: str,
    reason: str,
    head: str,
    artifact: Path,
    contract: dict[str, object] | None = None,
    returncode: int | None = None,
    paths: dict[str, str] | None = None,
    asset_provenance: dict[str, object] | None = None,
    snapshot: dict[str, object] | None = None,
    cleanup: dict[str, object] | None = None,
) -> dict[str, object]:
    cleanup_payload = dict(cleanup or {})
    cleanup_payload.setdefault(
        'status',
        'graceful' if returncode is not None else 'not_started',
    )
    cleanup_payload.setdefault('app_returncode', returncode)
    return {
        'schema_version': 'voice_nav.real_audio_motion_smoke.v1',
        'status': status,
        'reason': reason,
        'exact_head': head,
        'command': list(build_app_command()),
        'artifact': str(artifact),
        'paths': paths or {},
        'asset_provenance': asset_provenance or {},
        'provenance': contract.get('provenance', {}) if contract else {},
        'device': contract.get('capability', {}) if contract else {},
        **(snapshot or {}),
        'cleanup': cleanup_payload,
    }


def _observer_module() -> dict[str, object]:
    return runpy.run_path(
        str(Path(__file__).with_name('_motion_smoke_observer.py'))
    )


_APP_INTERRUPT_TIMEOUT_S = 15.0
_APP_TERMINATE_TIMEOUT_S = 5.0
_APP_KILL_TIMEOUT_S = 5.0
_OBSERVER_JOIN_TIMEOUT_S = 5.0
_EXECUTOR_SHUTDOWN_TIMEOUT_S = 5.0


def _process_exited(process) -> bool:
    try:
        return process.poll() is not None
    except (AttributeError, OSError):
        return False


def _send_app_signal(process, signum: int) -> bool:
    """Signal only the outer app handle so its existing finally runs."""
    try:
        if os.name == 'nt' and signum == signal.SIGINT:
            process.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            process.send_signal(signum)
    except (OSError, ProcessLookupError, AttributeError):
        return False
    return True


def _send_session_signal(group_id: int, signum: int) -> bool:
    """Signal an observed session group, never the caller's process group."""
    if os.name == 'nt':
        return False
    try:
        os.killpg(group_id, signum)
    except ProcessLookupError:
        return True
    except OSError:
        return False
    return True


def _session_groups_alive(group_ids: tuple[int, ...] | list[int]) -> bool:
    if os.name == 'nt':
        return False
    for group_id in group_ids:
        try:
            os.killpg(int(group_id), 0)
        except ProcessLookupError:
            continue
        except OSError:
            return True
        else:
            return True
    return False


def _wait_for_session_groups(
    group_ids: tuple[int, ...] | list[int],
    timeout_s: float,
) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_s)
    while _session_groups_alive(group_ids) and time.monotonic() < deadline:
        time.sleep(0.05)
    return not _session_groups_alive(group_ids)


def _cleanup_timed_out_app(
    process,
    *,
    session_groups: tuple[int, ...] | list[int],
    send_app_signal=None,
    send_session_signal=None,
    wait_for_exit=None,
    session_alive=None,
) -> dict[str, object]:
    """Trigger app cleanup, escalate only after bounded app phases, then prove no leak."""
    send_app_signal = send_app_signal or _send_app_signal
    send_session_signal = send_session_signal or _send_session_signal
    wait_for_exit = wait_for_exit or (
        lambda child, timeout_s: _wait_for_process_exit(child, timeout_s)
    )
    session_alive = session_alive or _session_groups_alive
    groups = sorted({int(group) for group in session_groups})
    app_exited = _process_exited(process)
    status = 'timeout_interrupted'

    if not app_exited:
        if not send_app_signal(process, signal.SIGINT):
            status = 'timeout_cleanup_failed'
        elif wait_for_exit(process, _APP_INTERRUPT_TIMEOUT_S):
            app_exited = True
        else:
            status = 'timeout_terminated'
            if not send_app_signal(process, signal.SIGTERM):
                status = 'timeout_cleanup_failed'
            elif wait_for_exit(process, _APP_TERMINATE_TIMEOUT_S):
                app_exited = True
            else:
                status = 'timeout_killed'
                if not send_app_signal(process, signal.SIGKILL):
                    status = 'timeout_cleanup_failed'
                else:
                    app_exited = wait_for_exit(process, _APP_KILL_TIMEOUT_S)

    owned_session_alive = bool(session_alive(groups)) if groups else False
    if owned_session_alive:
        for group_id in groups:
            if not send_session_signal(group_id, signal.SIGTERM):
                status = 'timeout_cleanup_unverified'
        if wait_for_exit is not None:
            _wait_for_session_groups(groups, _APP_TERMINATE_TIMEOUT_S)
        owned_session_alive = bool(session_alive(groups))
        if owned_session_alive:
            status = 'timeout_cleanup_unverified'
            for group_id in groups:
                send_session_signal(group_id, signal.SIGKILL)
            _wait_for_session_groups(groups, _APP_KILL_TIMEOUT_S)
            owned_session_alive = bool(session_alive(groups))

    if not app_exited:
        app_exited = _process_exited(process)
    return {
        'status': status if app_exited and not owned_session_alive else 'timeout_cleanup_unverified',
        'app_alive': not app_exited,
        'owned_session_alive': owned_session_alive,
        'owned_session_groups': groups,
    }


def _wait_for_process_exit(process, timeout_s: float) -> bool:
    try:
        process.wait(timeout=max(0.0, timeout_s))
    except (subprocess.TimeoutExpired, TimeoutError):
        return _process_exited(process)
    except Exception:
        return _process_exited(process)
    return True


def _capture_owned_session_groups(process) -> tuple[int, ...]:
    """Capture child process groups created by the app, bounded to its tree."""
    if os.name == 'nt':
        return ()
    try:
        result = subprocess.run(
            ('ps', '-eo', 'pid=,ppid=,pgid='),
            check=False,
            capture_output=True,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.SubprocessError):
        return ()
    if result.returncode != 0:
        return ()
    rows: dict[int, tuple[int, int]] = {}
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) != 3:
            continue
        try:
            pid, parent, group_id = (int(value) for value in fields)
        except ValueError:
            continue
        rows[pid] = (parent, group_id)
    root_pid = int(getattr(process, 'pid', 0) or 0)
    if root_pid <= 0:
        return ()
    descendants = {root_pid}
    changed = True
    while changed:
        changed = False
        for pid, (parent, _group_id) in rows.items():
            if parent in descendants and pid not in descendants:
                descendants.add(pid)
                changed = True
    root_group = root_pid
    return tuple(sorted({
        group_id
        for pid, (_parent, group_id) in rows.items()
        if pid in descendants and pid != root_pid and group_id != root_group
    }))


def _shutdown_observer_runtime(
    *,
    stop_spin,
    executor,
    spin_thread,
    observer,
    node,
    rclpy,
) -> list[str]:
    """Close the observer in bounded order using the supported executor API."""
    errors: list[str] = []

    def attempt(label, action) -> None:
        try:
            action()
        except Exception as error:  # pragma: no cover - exercised by seam tests
            errors.append(f'{label}:{str(error)[:120]}')

    attempt('stop_spin', stop_spin)
    attempt(
        'spin_join',
        lambda: spin_thread.join(timeout=_OBSERVER_JOIN_TIMEOUT_S),
    )
    attempt('observer_close', observer.close)
    attempt('executor_remove_node', lambda: executor.remove_node(node))
    attempt('node_destroy', node.destroy_node)
    attempt(
        'executor_shutdown',
        lambda: executor.shutdown(timeout_sec=_EXECUTOR_SHUTDOWN_TIMEOUT_S),
    )
    attempt(
        'rclpy_shutdown',
        lambda: rclpy.shutdown() if rclpy.ok() else None,
    )
    return errors


def _run_product_once(
    environment: dict[str, str],
    *,
    timeout_s: float = 180.0,
) -> tuple[int, dict[str, object], dict[str, object]]:
    """Run the existing app once while a read-only typed observer spins."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init(args=None)
    node = rclpy.create_node('voice_nav_real_audio_motion_smoke_observer')
    observer = _observer_module()['MotionSmokeObserver'](node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinning = True

    def spin() -> None:
        while spinning and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    import threading

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    process = None
    cleanup: dict[str, object] = {
        'status': 'not_started',
        'app_alive': True,
        'owned_session_alive': False,
        'owned_session_groups': [],
    }
    owned_session_groups: set[int] = set()
    try:
        process = subprocess.Popen(
            build_app_command(),
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        time.sleep(0.25)
        owned_session_groups.update(_capture_owned_session_groups(process))
        try:
            process.communicate(timeout=timeout_s)
            owned_session_groups.update(_capture_owned_session_groups(process))
            cleanup = {
                'status': 'graceful',
                'app_alive': not _process_exited(process),
                'owned_session_alive': _session_groups_alive(
                    tuple(sorted(owned_session_groups)),
                ),
                'owned_session_groups': sorted(owned_session_groups),
            }
        except subprocess.TimeoutExpired:
            owned_session_groups.update(_capture_owned_session_groups(process))
            cleanup = _cleanup_timed_out_app(
                process,
                session_groups=tuple(sorted(owned_session_groups)),
            )
            process.communicate()
        time.sleep(0.25)
        return int(process.returncode), observer.snapshot(), cleanup
    except Exception as error:
        if process is not None and process.poll() is None:
            owned_session_groups.update(_capture_owned_session_groups(process))
            cleanup = _cleanup_timed_out_app(
                process,
                session_groups=tuple(sorted(owned_session_groups)),
            )
        return 1, {'observer_error': str(error)[:160]}, cleanup
    finally:
        def stop_spin() -> None:
            nonlocal spinning
            spinning = False

        cleanup_errors = _shutdown_observer_runtime(
            stop_spin=stop_spin,
            executor=executor,
            spin_thread=spin_thread,
            observer=observer,
            node=node,
            rclpy=rclpy,
        )
        if cleanup_errors:
            cleanup['status'] = 'cleanup_failed'
            cleanup['cleanup_errors'] = cleanup_errors


def _write_and_print(artifact: Path, payload: dict[str, object]) -> int:
    _write_artifact(artifact, payload)
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
    return 0 if payload.get('status') == 'passed' else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--head', default=os.environ.get('VOICE_NAV_MOTION_SMOKE_HEAD', ''))
    parser.add_argument('--prefix', default=os.environ.get('VOICE_NAV_SHERPA_ONNX_PREFIX', ''))
    parser.add_argument(
        '--vad',
        default=os.environ.get('VOICE_NAV_SENSEVOICE_VAD_MODEL', ''),
    )
    parser.add_argument(
        '--model',
        default=os.environ.get('VOICE_NAV_SENSEVOICE_MODEL', ''),
    )
    parser.add_argument(
        '--tokens',
        default=os.environ.get('VOICE_NAV_SENSEVOICE_TOKENS', ''),
    )
    parser.add_argument(
        '--chaowen',
        default=os.environ.get('VOICE_NAV_CHAOWEN_TTS_ROOT', ''),
    )
    parser.add_argument(
        '--artifact',
        default=os.environ.get(
            'VOICE_NAV_MOTION_SMOKE_ARTIFACT',
            '/tmp/voice_nav_real_audio_motion_smoke.json',
        ),
    )
    arguments = parser.parse_args(argv)
    artifact = Path(arguments.artifact).expanduser()
    head = arguments.head.strip()
    if not head:
        payload = _result(
            status='unavailable',
            reason='exact_head_required_as_parameter_or_environment',
            head=head,
            artifact=artifact,
        )
        return _write_and_print(artifact, payload)
    if not all((arguments.prefix, arguments.vad, arguments.model, arguments.tokens, arguments.chaowen)):
        payload = _result(
            status='unavailable',
            reason='absolute_audio_paths_are_required',
            head=head,
            artifact=artifact,
        )
        return _write_and_print(artifact, payload)

    input_result = validate_runtime_inputs(
        exact_head=head,
        vad=Path(arguments.vad).expanduser(),
        model=Path(arguments.model).expanduser(),
        tokens=Path(arguments.tokens).expanduser(),
        chaowen=Path(arguments.chaowen).expanduser(),
        prefix=Path(arguments.prefix).expanduser(),
    )
    if not input_result['ok']:
        payload = _result(
            status='unavailable',
            reason=str(input_result['reason']),
            head=head,
            artifact=artifact,
            paths=input_result.get('paths'),
            asset_provenance=input_result.get('asset_provenance'),
        )
        return _write_and_print(artifact, payload)

    paths = input_result['paths']
    contract = _contract_module()['inspect_runtime_contract'](
        exact_head=head,
        sherpa_prefix=Path(paths['prefix']),
    )
    if not contract['ok']:
        payload = _result(
            status='unavailable',
            reason=str(contract['reason']),
            head=head,
            artifact=artifact,
            contract=contract,
            paths=paths,
            asset_provenance=input_result.get('asset_provenance'),
        )
        return _write_and_print(artifact, payload)

    chaowen_result = runpy.run_path(
        str(Path(__file__).with_name('_chaowen_asset_verifier.py'))
    )['verify_chaowen_root'](paths['chaowen'])
    if chaowen_result.get('status') != 'ready':
        payload = _result(
            status='unavailable',
            reason=str(chaowen_result.get('reason', 'chaowen_assets_unavailable')),
            head=head,
            artifact=artifact,
            contract=contract,
            paths=paths,
            asset_provenance=input_result.get('asset_provenance'),
        )
        return _write_and_print(artifact, payload)

    environment = os.environ.copy()
    environment.update({
        'VOICE_NAV_MOTION_SMOKE_HEAD': head,
        'VOICE_NAV_SHERPA_ONNX_PREFIX': paths['prefix'],
        'VOICE_NAV_SENSEVOICE_VAD_MODEL': paths['vad'],
        'VOICE_NAV_SENSEVOICE_MODEL': paths['model'],
        'VOICE_NAV_SENSEVOICE_TOKENS': paths['tokens'],
        'VOICE_NAV_CHAOWEN_TTS_ROOT': paths['chaowen'],
    })
    returncode, snapshot, cleanup = _run_product_once(environment)
    snapshot = {
        **snapshot,
        **_authoritative_speak_completion(
            command=build_app_command(),
            app_returncode=returncode,
            speak_status_completed_count=snapshot.get(
                'speak_status_completed_count',
            ),
        ),
    }
    evidence = _result(
        status='unavailable',
        reason='voice_nav_app_failed' if returncode != 0 else '',
        head=head,
        artifact=artifact,
        contract=contract,
        returncode=returncode,
        paths=paths,
        asset_provenance=input_result.get('asset_provenance'),
        snapshot=snapshot,
        cleanup=cleanup,
    )
    replay = validate_smoke_evidence(evidence)
    if returncode == 0 and replay['ok'] and cleanup.get('status') == 'graceful':
        evidence['status'] = 'passed'
        evidence['reason'] = ''
    elif returncode == 0 and not replay['ok']:
        evidence['reason'] = str(replay['reason'])
    return _write_and_print(artifact, evidence)


if __name__ == '__main__':
    raise SystemExit(main())
