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

"""Failure-safe Gazebo process teardown for isolated launch tests."""

from collections.abc import Callable, Iterable, Mapping
import os
import re
import secrets
import shutil
import subprocess
from typing import Any

from launch.events.process import ProcessExited


SERVICE_TIMEOUT_MILLISECONDS = 5000
SUBPROCESS_TIMEOUT_SECONDS = 7.0
STOP_RPC_ATTEMPTS = 2
PROCESS_TIMEOUT_SECONDS = 10.0
POSITIVE_ACK = re.compile(r'\s*data:\s*true\s*')
TEST_PARTITION_SCOPE = re.compile(r'[a-z0-9][a-z0-9_]{0,31}')


def run_cleanup_steps(
    message: str,
    steps: Iterable[tuple[str, Callable[[], None]]],
) -> None:
    """Attempt every fixture-destruction step and report all failures."""
    errors = []
    for label, callback in steps:
        try:
            callback()
        except Exception as error:
            try:
                error.add_note(f'cleanup step failed: {label}')
            except Exception:
                # Exhausting the cleanup ladder is more important than an
                # optional diagnostic note supplied by an exception subtype.
                pass
            errors.append(error)

    if errors:
        raise ExceptionGroup(message, errors)


def join_started_thread(thread: Any, *, timeout_seconds: float) -> None:
    """Join a started thread without failing on partial fixture setup."""
    if thread.ident is None:
        return

    thread.join(timeout=timeout_seconds)
    if thread.is_alive():
        raise TimeoutError(
            f'fixture spin thread did not stop within '
            f'{timeout_seconds:.1f} seconds'
        )


def claim_unique_test_partition(scope: str) -> str:
    """Claim a fresh Gazebo partition for this launch-test process."""
    if TEST_PARTITION_SCOPE.fullmatch(scope) is None:
        raise ValueError(
            'Gazebo test partition scope must be 1-32 lowercase '
            'letters, digits, or underscores'
        )
    partition = (
        f'voice_nav_{scope}_{os.getpid()}_{secrets.token_hex(16)}'
    )
    os.environ['GZ_PARTITION'] = partition
    return partition


def structured_stop_gazebo(
    proc_info: Any,
    *,
    expected_partition: str,
    environment: Mapping[str, str] | None = None,
    executable_lookup: Callable[[str], str | None] = shutil.which,
    runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> None:
    """Stop and join the launch-managed Gazebo in one exact test partition."""
    active_environment = dict(
        os.environ if environment is None else environment
    )
    actual_partition = active_environment.get('GZ_PARTITION', '')
    if not expected_partition or actual_partition != expected_partition:
        raise AssertionError(
            'Refusing /server_control outside the isolated test partition: '
            f'expected={expected_partition!r}, actual={actual_partition!r}'
        )

    proc_info.assertWaitForStartup(
        process='gazebo',
        timeout=PROCESS_TIMEOUT_SECONDS,
    )
    if isinstance(proc_info['gazebo'], ProcessExited):
        raise AssertionError(
            'Gazebo already exited before the structured stop request'
        )

    ruby = executable_lookup('ruby')
    gz = executable_lookup('gz')
    if ruby is None or gz is None:
        raise AssertionError('ruby or gz executable is unavailable')

    arguments = [
        ruby,
        gz,
        'service',
        '-s',
        '/server_control',
        '--reqtype',
        'gz.msgs.ServerControl',
        '--reptype',
        'gz.msgs.Boolean',
        '--timeout',
        str(SERVICE_TIMEOUT_MILLISECONDS),
        '--req',
        'stop: true',
    ]
    last_timeout = None
    for _attempt in range(STOP_RPC_ATTEMPTS):
        try:
            completed = runner(
                arguments,
                capture_output=True,
                text=True,
                timeout=SUBPROCESS_TIMEOUT_SECONDS,
                check=False,
                shell=False,
                env=active_environment,
            )
        except subprocess.TimeoutExpired as error:
            last_timeout = error
            continue
        break
    else:
        raise AssertionError(
            'Gazebo structured stop RPC timed out after '
            f'{STOP_RPC_ATTEMPTS} attempts of '
            f'{SUBPROCESS_TIMEOUT_SECONDS:.1f} seconds'
        ) from last_timeout

    if completed.returncode != 0:
        raise AssertionError(
            'Gazebo structured stop RPC failed: '
            f'code={completed.returncode}, '
            f'stdout={completed.stdout!r}, '
            f'stderr={completed.stderr!r}'
        )
    if POSITIVE_ACK.fullmatch(completed.stdout) is None:
        raise AssertionError(
            'Gazebo structured stop did not return a positive ACK: '
            f'{completed.stdout!r}'
        )

    # The ACK means only that the request was accepted. This is the actual
    # launch-managed process completion barrier.
    proc_info.assertWaitForShutdown(
        process='gazebo',
        timeout=PROCESS_TIMEOUT_SECONDS,
    )
