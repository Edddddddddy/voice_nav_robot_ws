#!/usr/bin/env python3
# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Installed, bounded non-Audio headless runner for Issue #164."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
from typing import Callable, Mapping


SCHEMA_VERSION = 1
SCHEMA_NAME = 'voice_nav.issue164.headless'
_TASK_ID_ENV = 'VOICE_NAV_ISSUE164_TASK_ID'
_WORKSPACE_ENV = 'VOICE_NAV_WORKSPACE_ROOT'
_GRACEFUL_CLEANUP_S = 10.0
_TERMINATE_CLEANUP_S = 5.0
_CTEST_INVENTORY_TIMEOUT_S = 30.0
_EXACT_HEAD_RE = re.compile(r'^[0-9a-fA-F]{40}$')
_CTEST_TEST_LINE_RE = re.compile(
    r'^\s*Test\s+#\d+:\s*(?P<name>\S(?:.*\S)?)\s*$'
)
NAVIGATION_STATIONARITY_PREFIX = 'EVIDENCE issue164_navigation_mvp '
NAVIGATION_STATIONARITY_HOLD_MS = 200
NAVIGATION_WHEEL_ENDPOINT_TOLERANCE = 0.02


@dataclass(frozen=True)
class PhaseSpec:
    """One exact product phase owned by this runner."""

    phase_id: str
    label: str
    package: str
    test_regex: str
    timeout_s: float
    required_tests: tuple[str, ...]
    proves: tuple[str, ...]
    summary: str


PHASES = (
    PhaseSpec(
        'B', 'move_stop', 'voice_nav_bringup',
        '^(scripted_voice_demo_launch_test|voice_nav_demo_stop_launch_test)$',
        300.0,
        (
            'scripted_voice_demo_launch_test',
            'voice_nav_demo_stop_launch_test',
        ),
        ('MOVE', 'STOP', 'final_zero', 'stationary>=200ms'),
        'scripted MOVE and STOP product checks with final zero and stationarity',
    ),
    PhaseSpec(
        'C', 'mapping_mvp', 'voice_nav_bringup',
        '^mapping_mvp_launch_test$', 300.0,
        ('mapping_mvp_launch_test',),
        (
            'slam_active', 'single_map_to_odom_owner',
            'scan_time_tf', 'formal_MOVE_ROTATE',
            'occupancy_nonempty_known_unknown', 'final_zero',
        ),
        'mapping MVP graph, formal motion, occupancy, owner, and final zero',
    ),
    PhaseSpec(
        'D', 'navigation_mvp', 'voice_nav_bringup',
        '^navigation_mvp_launch_test$', 420.0,
        ('navigation_mvp_launch_test',),
        (
            'nav2_active', 'single_map_to_odom_owner',
            'formal_NAVIGATE_TO_study', 'pose_error<=0.50m_0.50rad',
            'final_zero', 'stationary>=200ms',
        ),
        'navigation MVP study place, owner, pose tolerance, zero, and stationarity',
    ),
)


CommandRunner = Callable[..., Mapping[str, object]]


def _safe_task_id(value: str) -> str:
    value = re.sub(r'[^A-Za-z0-9_.-]+', '-', value).strip('-._')
    return value or 'task'


def make_task_id() -> str:
    """Return a collision-resistant, filesystem-safe local task id."""
    configured = os.environ.get(_TASK_ID_ENV)
    if configured:
        return _safe_task_id(configured)
    return f'task-{os.getpid()}-{time.time_ns()}'


def validate_exact_head(value: str) -> str:
    """Validate the immutable 40-hex repository head used by this run."""
    if not isinstance(value, str) or _EXACT_HEAD_RE.fullmatch(value) is None:
        raise ValueError('exact_head must be exactly 40 hexadecimal characters')
    return value


def _portable_git_path(value: str) -> Path:
    """Translate a Windows worktree gitdir for the WSL runner when needed."""
    normalized = value.strip().replace('\\', '/')
    if re.fullmatch(r'[A-Za-z]:/.*', normalized):
        return Path('/mnt') / normalized[0].lower() / normalized[3:]
    return Path(normalized)


def workspace_head(workspace_root: Path) -> str:
    """Read HEAD through Git, with a read-only WSL worktree fallback."""
    try:
        result = subprocess.run(
            ('git', '-C', str(workspace_root), 'rev-parse', 'HEAD'),
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
        return validate_exact_head(result.stdout.strip())
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        marker = workspace_root / '.git'
        if not marker.is_file():
            raise ValueError('workspace HEAD could not be read')
        marker_text = marker.read_text(encoding='utf-8').strip()
        if not marker_text.startswith('gitdir:'):
            raise ValueError('workspace gitdir marker is invalid')
        gitdir = _portable_git_path(marker_text.split(':', 1)[1].strip())
        head_text = (gitdir / 'HEAD').read_text(encoding='utf-8').strip()
        if head_text.startswith('ref: '):
            reference = head_text[5:]
            common_dir_file = gitdir / 'commondir'
            if common_dir_file.is_file():
                common_dir = (
                    gitdir / common_dir_file.read_text(encoding='utf-8').strip()
                ).resolve()
            else:
                common_dir = gitdir
            head_text = (common_dir / reference).read_text(
                encoding='utf-8'
            ).strip()
        return validate_exact_head(head_text)


def default_output_path(workspace_root: Path, task_id: str | None = None) -> Path:
    """Choose an ignored, task-local result path without reusing evidence."""
    task = _safe_task_id(task_id or make_task_id())
    return (
        workspace_root / 'build' / 'test-results' / 'issue164'
        / task / 'result.json'
    )


def phase_command(workspace_root: Path, phase: PhaseSpec) -> tuple[str, ...]:
    """Build the exact installed CTest invocation for one phase."""
    command = (
        'ctest',
        '--test-dir',
        str(workspace_root / 'build' / phase.package),
        '-R',
        phase.test_regex,
        '--no-tests=error',
        '--output-on-failure',
    )
    if phase.phase_id == 'D':
        return command + ('--verbose',)
    return command


def inventory_command(
    workspace_root: Path,
    phase: PhaseSpec,
) -> tuple[str, ...]:
    """Build the read-only CTest inventory command for one product phase."""
    return (
        'ctest',
        '--test-dir',
        str(workspace_root / 'build' / phase.package),
        '-N',
    )


def parse_ctest_inventory(output: str) -> tuple[str, ...]:
    """Return the exact test names listed by a plain ``ctest -N`` report."""
    return tuple(
        match.group('name')
        for line in output.splitlines()
        if (match := _CTEST_TEST_LINE_RE.match(line)) is not None
    )


def _runner_environment() -> dict[str, str]:
    environment = dict(os.environ)
    environment.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    environment.setdefault(
        'ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'
    )
    return environment


def _send_owned_signal(process: subprocess.Popen, signum: int) -> bool:
    """Signal only the process group created by this runner."""
    if not _owned_process_group_alive(process):
        return True
    try:
        if os.name == 'nt':
            if signum == signal.SIGINT:
                process.send_signal(signal.CTRL_BREAK_EVENT)
            elif signum == signal.SIGTERM:
                process.terminate()
            else:
                process.kill()
        else:
            os.killpg(process.pid, signum)
    except (OSError, ProcessLookupError):
        return process.poll() is not None
    return True


def _owned_process_group_alive(process: subprocess.Popen) -> bool:
    """Check the owned group, including descendants after parent exit."""
    if os.name == 'nt':
        return process.poll() is None
    try:
        os.killpg(process.pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _wait_for_exit(process: subprocess.Popen, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    try:
        process.wait(timeout=max(0.0, deadline - time.monotonic()))
    except (subprocess.TimeoutExpired, TimeoutError):
        pass
    while _owned_process_group_alive(process):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        time.sleep(min(0.05, remaining))
    return True


def _cleanup_owned_process(process: subprocess.Popen) -> str:
    """Stop the process group in bounded, escalating phases."""
    if not _owned_process_group_alive(process):
        return 'exited'
    if not _send_owned_signal(process, signal.SIGINT):
        return 'failed'
    if _wait_for_exit(process, _GRACEFUL_CLEANUP_S):
        return 'graceful'
    if not _send_owned_signal(process, signal.SIGTERM):
        return 'failed'
    if _wait_for_exit(process, _TERMINATE_CLEANUP_S):
        return 'terminated'
    if not _send_owned_signal(process, signal.SIGKILL):
        return 'failed'
    return 'killed' if _wait_for_exit(process, 1.0) else 'failed'


def _run_owned_command(
    command: tuple[str, ...],
    *,
    cwd: Path,
    env: Mapping[str, str],
    timeout_s: float,
    log_path: Path,
) -> Mapping[str, object]:
    """Run one phase in a process group owned exclusively by this runner."""
    options: dict[str, object] = {
        'cwd': str(cwd),
        'env': dict(env),
        'stdin': subprocess.DEVNULL,
    }
    if os.name == 'nt':
        options['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        options['start_new_session'] = True

    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic_ns()
    cleanup = 'failed'
    returncode: int | None = None
    with log_path.open('x', encoding='utf-8', newline='\n') as log:
        options['stdout'] = log
        options['stderr'] = subprocess.STDOUT
        process = subprocess.Popen(command, **options)
        try:
            try:
                returncode = process.wait(timeout=timeout_s)
                cleanup = 'exited'
            except subprocess.TimeoutExpired:
                cleanup = _cleanup_owned_process(process)
                returncode = 124
            except KeyboardInterrupt:
                cleanup = _cleanup_owned_process(process)
                raise
        finally:
            if _owned_process_group_alive(process):
                cleanup = _cleanup_owned_process(process)
            elif cleanup == 'running':
                cleanup = 'exited'
    owned_processes_remaining = int(_owned_process_group_alive(process))
    return {
        'returncode': returncode if returncode is not None else 1,
        'duration_ms': (time.monotonic_ns() - started) // 1_000_000,
        'cleanup_stage': cleanup,
        'owned_processes_remaining': owned_processes_remaining,
        'pid': process.pid,
        'log': str(log_path),
    }


def write_result_no_replace(path: Path, document: Mapping[str, object]) -> None:
    """Publish one JSON result and refuse to replace existing evidence."""
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(
        document,
        ensure_ascii=False,
        sort_keys=True,
        indent=2,
    ) + '\n'
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{path.name}.', suffix='.tmp', dir=os.fspath(path.parent)
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(
            descriptor, 'w', encoding='utf-8', newline='\n'
        ) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        # A hard link is an atomic publish that fails instead of replacing an
        # existing path.  The temporary file and result are on one directory.
        os.link(temporary_path, path)
    except Exception:
        try:
            temporary_path.unlink()
        except OSError:
            pass
        raise
    else:
        temporary_path.unlink()


def preflight_output_paths(output_path: Path) -> None:
    """Reject any reused task evidence before starting product phases."""
    paths = (
        output_path,
        *(output_path.parent / f'inventory-{phase.phase_id}.log'
          for phase in PHASES),
        *(output_path.parent / f'phase-{phase.phase_id}.log' for phase in PHASES),
    )
    existing = tuple(path for path in paths if path.exists())
    if existing:
        rendered = ', '.join(str(path) for path in existing)
        raise FileExistsError(f'task-local output already exists: {rendered}')


def _inventory_failure(
    phase: PhaseSpec,
    *,
    test: str,
    reason: str,
    detail: str,
) -> dict[str, object]:
    return {
        'phase': phase.phase_id,
        'test': test,
        'failure_kind': 'build_contract',
        'reason': reason,
        'detail': detail,
        'required_tests': list(phase.required_tests),
    }


def _inventory_check(
    phase: PhaseSpec,
    command: tuple[str, ...],
    log_path: Path,
    result: Mapping[str, object],
) -> tuple[dict[str, object], list[dict[str, object]]]:
    returncode = int(result.get('returncode', 1))
    cleanup_stage = str(result.get('cleanup_stage', 'unknown'))
    owned_remaining = int(result.get('owned_processes_remaining', 1))
    log_error = None
    try:
        registered_tests = parse_ctest_inventory(
            log_path.read_text(encoding='utf-8')
        )
    except (OSError, UnicodeError) as error:
        registered_tests = ()
        log_error = str(error)

    test_counts = {
        test: registered_tests.count(test)
        for test in phase.required_tests
    }
    failures: list[dict[str, object]] = []
    if returncode != 0 or cleanup_stage != 'exited' or owned_remaining != 0:
        failures.append(
            _inventory_failure(
                phase,
                test='<ctest -N inventory>',
                reason='ctest_inventory_command_failed',
                detail=(
                    f'phase {phase.phase_id} CTest build-contract inventory '
                    f'command failed: returncode={returncode}, '
                    f'cleanup_stage={cleanup_stage}, '
                    f'owned_processes_remaining={owned_remaining}'
                ),
            )
        )
    elif log_error is not None:
        failures.append(
            _inventory_failure(
                phase,
                test='<ctest -N inventory>',
                reason='inventory_log_unreadable',
                detail=(
                    f'phase {phase.phase_id} CTest build-contract inventory '
                    f'log could not be read: {log_error}'
                ),
            )
        )
    else:
        for test, count in test_counts.items():
            if count == 0:
                failures.append(
                    _inventory_failure(
                        phase,
                        test=test,
                        reason='missing_required_test',
                        detail=(
                            f'phase {phase.phase_id} requires exact test '
                            f'{test!r}; CTest build-contract inventory '
                            f'registered {count} instances; all product '
                            'phases are blocked'
                        ),
                    )
                )
            elif count > 1:
                failures.append(
                    _inventory_failure(
                        phase,
                        test=test,
                        reason='duplicate_required_test',
                        detail=(
                            f'phase {phase.phase_id} requires exact test '
                            f'{test!r} exactly once; CTest build-contract '
                            f'inventory registered {count} instances; all '
                            'product phases are blocked'
                        ),
                    )
                )

    check = {
        'phase': phase.phase_id,
        'label': phase.label,
        'package': phase.package,
        'required_tests': list(phase.required_tests),
        'registered_tests': list(registered_tests),
        'test_counts': test_counts,
        'command': list(command),
        'status': 'passed' if not failures else 'failed',
        'failure_kind': 'build_contract' if failures else None,
        'reason': failures[0]['reason'] if failures else None,
        'returncode': returncode,
        'cleanup_stage': cleanup_stage,
        'owned_processes_remaining': owned_remaining,
        'log': str(log_path),
    }
    return check, failures


def run_inventory_preflight(
    *,
    workspace_root: Path,
    output_path: Path,
    command_runner: CommandRunner,
) -> dict[str, object]:
    """Verify every required installed CTest name before starting product phases."""
    checks: list[dict[str, object]] = []
    failures: list[dict[str, object]] = []
    environment = _runner_environment()
    for phase in PHASES:
        command = inventory_command(workspace_root, phase)
        log_path = output_path.parent / f'inventory-{phase.phase_id}.log'
        try:
            result = command_runner(
                command,
                cwd=workspace_root,
                env=environment,
                timeout_s=_CTEST_INVENTORY_TIMEOUT_S,
                log_path=log_path,
            )
        except (OSError, subprocess.SubprocessError, TimeoutError) as error:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_path.write_text(
                f'CTest inventory command could not start: {error}\n',
                encoding='utf-8',
            )
            result = {
                'returncode': 127,
                'duration_ms': 0,
                'cleanup_stage': 'not_started',
                'owned_processes_remaining': 0,
            }
        check, check_failures = _inventory_check(
            phase, command, log_path, result
        )
        checks.append(check)
        failures.extend(check_failures)
    return {
        'status': 'passed' if not failures else 'failed',
        'checks': checks,
        'failures': failures,
    }


def parse_navigation_stationarity_marker(
    log_text: str,
) -> dict[str, object] | None:
    """Parse the one bounded JSON stationarity marker emitted by phase D."""
    matches = [
        line.split(NAVIGATION_STATIONARITY_PREFIX, 1)[1].strip()
        for line in log_text.splitlines()
        if NAVIGATION_STATIONARITY_PREFIX in line
    ]
    if not matches:
        return None
    if len(matches) != 1:
        raise ValueError(
            'phase D navigation stationarity marker must occur exactly once'
        )
    try:
        payload = json.loads(matches[0])
    except json.JSONDecodeError as error:
        raise ValueError(
            'phase D navigation stationarity marker is not JSON'
        ) from error
    if not isinstance(payload, dict):
        raise ValueError('phase D stationarity marker must be a JSON object')
    final_stationary = payload.get('final_stationary')
    if type(final_stationary) is not bool:
        raise ValueError('phase D final_stationary must be a JSON boolean')
    for field in (
        'stationary_ms', 'zero_sim_ns', 'zero_receipt_ns',
        'odom_receipt_ns', 'odom_stamp_ns', 'joint_receipt_ns',
        'joint_stamp_ns', 'stationary_end_sim_ns',
    ):
        value = payload.get(field)
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise ValueError(f'phase D {field} must be a non-negative integer')
    for field in ('joint_left_velocity', 'joint_right_velocity'):
        value = payload.get(field)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or abs(float(value)) > NAVIGATION_WHEEL_ENDPOINT_TOLERANCE
        ):
            raise ValueError(
                f'phase D {field} must be a finite bounded number'
            )
    if (
        final_stationary and
        payload['stationary_ms'] < NAVIGATION_STATIONARITY_HOLD_MS
    ):
        raise ValueError(
            'phase D stationary evidence is shorter than 200 ms'
        )
    if final_stationary:
        if payload['zero_receipt_ns'] >= payload['odom_receipt_ns']:
            raise ValueError('phase D odom receipt is not after final zero')
        if payload['zero_receipt_ns'] >= payload['joint_receipt_ns']:
            raise ValueError('phase D joint receipt is not after final zero')
        if payload['zero_sim_ns'] >= payload['odom_stamp_ns']:
            raise ValueError('phase D odom stamp is not after final zero')
        if payload['zero_sim_ns'] >= payload['joint_stamp_ns']:
            raise ValueError('phase D joint stamp is not after final zero')
        if payload['stationary_end_sim_ns'] <= payload['zero_sim_ns']:
            raise ValueError('phase D stationary window is before final zero')
        if payload['stationary_end_sim_ns'] > payload['odom_stamp_ns']:
            raise ValueError('phase D odom endpoint precedes stationary window')
        if payload['stationary_end_sim_ns'] > payload['joint_stamp_ns']:
            raise ValueError('phase D joint endpoint precedes stationary window')
    return payload


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _phase_record(
    phase: PhaseSpec,
    command: tuple[str, ...],
    log_path: Path,
    result: Mapping[str, object],
) -> dict[str, object]:
    returncode = int(result.get('returncode', 1))
    cleanup_stage = str(result.get('cleanup_stage', 'unknown'))
    owned_remaining = int(result.get('owned_processes_remaining', 1))
    passed = (
        returncode == 0
        and cleanup_stage == 'exited'
        and owned_remaining == 0
    )
    record = {
        'id': phase.phase_id,
        'label': phase.label,
        'package': phase.package,
        'test_regex': phase.test_regex,
        'proves': list(phase.proves),
        'summary': phase.summary,
        'command': list(command),
        'status': 'passed' if passed else 'failed',
        'returncode': returncode,
        'duration_ms': int(result.get('duration_ms', 0)),
        'log': str(result.get('log', log_path)),
        'cleanup_stage': cleanup_stage,
        'owned_processes_remaining': owned_remaining,
        'pid': result.get('pid'),
    }
    if phase.phase_id == 'D':
        stationarity = None
        stationarity_error = None
        try:
            stationarity = parse_navigation_stationarity_marker(
                log_path.read_text(encoding='utf-8')
            )
            if stationarity is None:
                stationarity_error = 'phase D stationarity marker is missing'
            elif stationarity['final_stationary'] is not True:
                stationarity_error = (
                    'phase D final_stationary evidence is false'
                )
        except (OSError, ValueError) as error:
            stationarity_error = str(error)
        if stationarity_error is not None:
            passed = False
            record['status'] = 'failed'
            record['stationarity_error'] = stationarity_error
        record['stationarity_evidence'] = stationarity
    return record


def _skipped_phase_record(
    phase: PhaseSpec,
    command: tuple[str, ...],
) -> dict[str, object]:
    return {
        'id': phase.phase_id,
        'label': phase.label,
        'package': phase.package,
        'test_regex': phase.test_regex,
        'proves': list(phase.proves),
        'summary': phase.summary,
        'command': list(command),
        'status': 'skipped',
        'reason': 'previous_phase_failed',
        'returncode': None,
        'duration_ms': 0,
        'log': None,
        'cleanup_stage': 'not_started',
        'owned_processes_remaining': 0,
        'pid': None,
    }


def run_pipeline(
    *,
    workspace_root: Path,
    output_path: Path,
    task_id: str,
    exact_head: str,
    command_runner: CommandRunner = _run_owned_command,
) -> int:
    """Run the non-Audio B-D phases once and publish structured evidence."""
    exact_head = validate_exact_head(exact_head)
    workspace_root = workspace_root.resolve()
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    preflight_output_paths(output_path)
    started_at = _utc_now()
    inventory_preflight = run_inventory_preflight(
        workspace_root=workspace_root,
        output_path=output_path,
        command_runner=command_runner,
    )
    if inventory_preflight['status'] != 'passed':
        document: dict[str, object] = {
            'schema_version': SCHEMA_VERSION,
            'schema': SCHEMA_NAME,
            'task_id': _safe_task_id(task_id),
            'exact_head': exact_head,
            'status': 'failed',
            'phase_count': 0,
            'completed_phase_count': 0,
            'started_at_utc': started_at,
            'finished_at_utc': _utc_now(),
            'workspace_root': str(workspace_root),
            'output_path': str(output_path),
            'owned_processes_remaining': sum(
                int(check['owned_processes_remaining'])
                for check in inventory_preflight['checks']
            ),
            'final_zero': None,
            'final_stationary': None,
            'tf_owner_overlap_observed': False,
            'inventory_preflight': inventory_preflight,
            'phases': [],
        }
        write_result_no_replace(output_path, document)
        return 1

    environment = _runner_environment()
    phase_records: list[dict[str, object]] = []
    failed = False

    for phase in PHASES:
        command = phase_command(workspace_root, phase)
        if failed:
            phase_records.append(_skipped_phase_record(phase, command))
            continue
        log_path = output_path.parent / f'phase-{phase.phase_id}.log'
        phase_environment = dict(environment)
        phase_environment['VOICE_NAV_EXACT_HEAD'] = exact_head
        result = command_runner(
            command,
            cwd=workspace_root,
            env=phase_environment,
            timeout_s=phase.timeout_s,
            log_path=log_path,
        )
        record = _phase_record(phase, command, log_path, result)
        phase_records.append(record)
        failed = record['status'] != 'passed'

    document: dict[str, object] = {
        'schema_version': SCHEMA_VERSION,
        'schema': SCHEMA_NAME,
        'task_id': _safe_task_id(task_id),
        'exact_head': exact_head,
        'status': 'failed' if failed else 'passed',
        'phase_count': len(PHASES),
        'completed_phase_count': sum(
            phase['status'] != 'skipped' for phase in phase_records
        ),
        'started_at_utc': started_at,
        'finished_at_utc': _utc_now(),
        'workspace_root': str(workspace_root),
        'output_path': str(output_path),
        'owned_processes_remaining': sum(
            int(phase['owned_processes_remaining'])
            for phase in phase_records
        ),
        'final_zero': True if not failed else None,
        'final_stationary': next(
            (
                phase['stationarity_evidence']['final_stationary']
                for phase in phase_records
                if (
                    phase.get('id') == 'D' and
                    isinstance(phase.get('stationarity_evidence'), dict)
                )
            ),
            None,
        ),
        'tf_owner_overlap_observed': False,
        'inventory_preflight': inventory_preflight,
        'phases': phase_records,
    }
    write_result_no_replace(output_path, document)
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Run the installed Issue #164 B-D headless product phases.'
    )
    parser.add_argument(
        '--workspace-root',
        type=Path,
        default=Path(os.environ.get(_WORKSPACE_ENV, Path.cwd())),
    )
    parser.add_argument('--exact-head', required=True)
    parser.add_argument('--task-id', default=None)
    parser.add_argument('--output', type=Path, default=None)
    arguments = parser.parse_args(argv)

    try:
        exact_head = validate_exact_head(arguments.exact_head)
        task_id = _safe_task_id(arguments.task_id or make_task_id())
        workspace_root = arguments.workspace_root.resolve()
        actual_head = workspace_head(workspace_root)
        if actual_head.lower() != exact_head.lower():
            raise ValueError(
                f'exact_head mismatch: expected {exact_head}, '
                f'workspace={actual_head}'
            )
        output_path = arguments.output
        if output_path is None:
            output_path = default_output_path(workspace_root, task_id)
        elif not output_path.is_absolute():
            output_path = workspace_root / output_path
        return run_pipeline(
            workspace_root=workspace_root,
            output_path=output_path,
            task_id=task_id,
            exact_head=exact_head,
        )
    except FileExistsError as error:
        print(f'error: refusing to overwrite result: {error}', file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError) as error:
        print(f'error: Issue #164 runner failed: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
