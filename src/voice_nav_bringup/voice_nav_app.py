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

"""Installed simulation-only app wrapper for the existing command console."""

from __future__ import annotations

import argparse
import json
import os
import runpy
import signal
import subprocess
import sys
import time
from typing import Literal


SESSION_COMMAND = (
    'ros2',
    'launch',
    'voice_nav_bringup',
    'voice_nav_session.launch.py',
    'headless:=true',
    'shutdown_on_gazebo_exit:=true',
)
READINESS_TIMEOUT_S = 60.0
GRACEFUL_SHUTDOWN_TIMEOUT_S = 10.0
TERMINATE_SHUTDOWN_TIMEOUT_S = 5.0
COMMAND_GATEWAY_SERVICE = '/voice_nav_command_gateway/set_parameters'
_OWNED_RCLPY_CONTEXT = False
_CleanupStage = Literal['graceful', 'terminated', 'killed', 'failed']


def _clock_now(clock) -> float:
    """Read an injected monotonic clock."""
    return clock() if callable(clock) else clock.monotonic()


def _stable_result(status: str, reason: str = '') -> dict[str, str]:
    return {'status': status, 'reason': reason}


def _write_result(stdout, result: dict[str, str]) -> None:
    stdout.write(
        json.dumps(
            result,
            ensure_ascii=False,
            sort_keys=True,
            separators=(',', ':'),
        )
        + '\n'
    )
    stdout.flush()


def _reason(error) -> str:
    return str(error) if error else ''


def _spawn_session(command, *, stdout, stderr):
    """Start exactly one new process group for the fixed session command."""
    options = {
        'stdin': subprocess.DEVNULL,
        'stdout': stdout,
        'stderr': stderr,
    }
    if os.name == 'nt':
        options['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        options['start_new_session'] = True
    process = subprocess.Popen(command, **options)
    # start_new_session makes the child PID the process-group ID.  Keeping the
    # identity on the owned handle prevents cleanup from targeting a caller's
    # process group.
    process.process_group_id = process.pid
    return process


def _wait_for_command_gateway_readiness(
    timeout_s: float,
    clock=time.monotonic,
) -> dict[str, str]:
    """Wait for the fixed gateway service without submitting a command."""
    global _OWNED_RCLPY_CONTEXT

    if timeout_s <= 0:
        return _stable_result('unavailable', 'invalid_readiness_timeout')

    rclpy = None
    node = None
    owns_context = False
    try:
        import rclpy
        from rcl_interfaces.srv import SetParameters

        owns_context = not rclpy.ok()
        if owns_context:
            rclpy.init(args=None)
            _OWNED_RCLPY_CONTEXT = True
        node = rclpy.create_node('voice_nav_app_readiness')
        client = node.create_client(SetParameters, COMMAND_GATEWAY_SERVICE)
    except Exception as error:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy is not None and owns_context and rclpy.ok():
            rclpy.shutdown()
            _OWNED_RCLPY_CONTEXT = False
        return _stable_result(
            'unavailable', f'readiness_start_failed:{_reason(error)}',
        )

    deadline = _clock_now(clock) + timeout_s
    keep_context = False
    try:
        remaining = max(0.0, deadline - _clock_now(clock))
        if client.wait_for_service(timeout_sec=remaining):
            keep_context = owns_context
            return _stable_result('ready')
        return _stable_result(
            'unavailable', 'command_gateway_readiness_timeout',
        )
    except Exception as error:
        return _stable_result(
            'unavailable', f'readiness_wait_failed:{_reason(error)}',
        )
    finally:
        try:
            node.destroy_node()
        finally:
            if owns_context and not keep_context and rclpy.ok():
                rclpy.shutdown()
                _OWNED_RCLPY_CONTEXT = False


def _shutdown_owned_rclpy_context() -> None:
    """Release the context retained for the existing console."""
    global _OWNED_RCLPY_CONTEXT

    if not _OWNED_RCLPY_CONTEXT:
        return
    try:
        import rclpy

        if rclpy.ok():
            rclpy.shutdown()
    finally:
        _OWNED_RCLPY_CONTEXT = False


def _run_existing_console(*, stdin, stdout) -> int:
    """Enter the already-installed console without reimplementing it."""
    try:
        import voice_nav_console
    except ModuleNotFoundError as error:
        if error.name != 'voice_nav_console':
            raise
        directory = os.path.dirname(__file__)
        console_path = os.path.join(directory, 'voice_nav_console')
        if not os.path.isfile(console_path):
            console_path = os.path.join(directory, 'voice_nav_console.py')
        namespace = runpy.run_path(console_path)
        console_main = namespace['main']
        transport_factory = namespace['RosParameterTransport']
    else:
        console_main = voice_nav_console.main
        transport_factory = voice_nav_console.RosParameterTransport

    transport = transport_factory()
    try:
        client = getattr(transport, '_client', None)
        if client is not None:
            client.wait_for_service(timeout_sec=READINESS_TIMEOUT_S)
        return console_main(
            [], transport=transport, stdin=stdin, stdout=stdout,
        )
    finally:
        transport.close()


def _poll(process):
    poll = getattr(process, 'poll', None)
    if poll is None:
        return None
    try:
        return poll()
    except Exception:
        return None


def _send_group_signal(process, signum) -> bool:
    """Signal only the process group owned by this app."""
    group_signal = getattr(process, 'send_group_signal', None)
    if group_signal is not None:
        try:
            group_signal(signum)
            return True
        except Exception:
            return False

    group_id = getattr(process, 'process_group_id', None)
    if os.name != 'nt' and group_id is not None:
        try:
            os.killpg(group_id, signum)
        except ProcessLookupError:
            return True
        except OSError:
            return False
        return True

    send_signal = getattr(process, 'send_signal', None)
    if send_signal is None:
        return False
    try:
        if os.name == 'nt' and signum == signal.SIGINT:
            send_signal(signal.CTRL_BREAK_EVENT)
        else:
            send_signal(signum)
    except Exception:
        return False
    return True


def _wait_for_exit(process, timeout_s: float, clock) -> bool:
    """Wait once for a bounded phase, using the injected monotonic clock."""
    deadline = _clock_now(clock) + timeout_s
    remaining = max(0.0, deadline - _clock_now(clock))
    try:
        process.wait(timeout=remaining)
    except (subprocess.TimeoutExpired, TimeoutError):
        return _poll(process) is not None
    except Exception:
        return _poll(process) is not None
    return True


def _cleanup_owned_session(process, clock) -> _CleanupStage:
    """Stop only the owned process group and report the strongest stage."""
    if process is None or _poll(process) is not None:
        return 'graceful'

    if not _send_group_signal(process, signal.SIGINT):
        return 'failed'
    if _wait_for_exit(process, GRACEFUL_SHUTDOWN_TIMEOUT_S, clock):
        return 'graceful'
    if not _send_group_signal(process, signal.SIGTERM):
        return 'failed'
    if _wait_for_exit(process, TERMINATE_SHUTDOWN_TIMEOUT_S, clock):
        return 'terminated'
    if not _send_group_signal(process, signal.SIGKILL):
        return 'failed'
    try:
        process.wait()
    except Exception:
        return 'killed' if _poll(process) is not None else 'failed'
    return 'killed' if _poll(process) is not None else 'failed'


def _readiness_is_ready(result) -> bool:
    return result is True or (
        isinstance(result, dict) and result.get('status') == 'ready'
    )


def _run_started_session(
    process,
    readiness,
    console_main,
    clock,
    stdin,
    stdout,
) -> int:
    """Run readiness and console after the child has started."""
    if _poll(process) is not None:
        _write_result(
            stdout,
            _stable_result('unavailable', 'session_exited_on_start'),
        )
        return 1

    try:
        readiness_result = readiness(READINESS_TIMEOUT_S, clock)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        _write_result(
            stdout,
            _stable_result(
                'unavailable', f'readiness_failed:{_reason(error)}',
            ),
        )
        return 1
    if not _readiness_is_ready(readiness_result):
        reason = (
            readiness_result.get('reason', 'command_gateway_not_ready')
            if isinstance(readiness_result, dict)
            else 'command_gateway_not_ready'
        )
        _write_result(stdout, _stable_result('unavailable', reason))
        return 1
    if _poll(process) is not None:
        _write_result(
            stdout,
            _stable_result('unavailable', 'session_exited_before_ready'),
        )
        return 1

    _write_result(stdout, _stable_result('ready'))
    try:
        console_result = console_main(stdin=stdin, stdout=stdout)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        _write_result(
            stdout,
            _stable_result('unavailable', f'console_failed:{_reason(error)}'),
        )
        return 1

    if _poll(process) is not None:
        _write_result(
            stdout,
            _stable_result('unavailable', 'session_exited_during_console'),
        )
        return 1
    return 0 if console_result is None else int(console_result)


def run_app(
    process_factory,
    readiness,
    console_main,
    clock,
    stdout,
    stderr,
    stdin=None,
) -> int:
    """Run one fixed session and then the existing console."""
    process = None
    exit_code = 1
    try:
        if stdin is None:
            stdin = sys.stdin
        process = process_factory(
            SESSION_COMMAND,
            stdout=stderr,
            stderr=stderr,
        )
        if process is None:
            raise RuntimeError('process_factory returned no process')
        exit_code = _run_started_session(
            process,
            readiness,
            console_main,
            clock,
            stdin,
            stdout,
        )
    except KeyboardInterrupt:
        exit_code = 130
    except Exception as error:
        _write_result(
            stdout,
            _stable_result(
                'unavailable', f'session_start_failed:{_reason(error)}',
            ),
        )
        exit_code = 1
    finally:
        if process is not None:
            try:
                cleanup_stage = _cleanup_owned_session(process, clock)
            except Exception:
                cleanup_stage = 'failed'
            if exit_code == 0 and cleanup_stage != 'graceful':
                _write_result(
                    stdout,
                    {
                        **_stable_result(
                            'unavailable', f'cleanup_{cleanup_stage}',
                        ),
                        'stage': cleanup_stage,
                    },
                )
                exit_code = 1
        _shutdown_owned_rclpy_context()
    return exit_code


def main(
    argv: list[str] | None = None,
    *,
    process_factory=None,
    readiness=None,
    clock=time.monotonic,
    console_main=None,
    stdin=None,
    stdout=None,
    stderr=None,
) -> int:
    """Run the fixed simulation session; no child arguments are accepted."""
    parser = argparse.ArgumentParser()
    try:
        parser.parse_args(argv)
    except SystemExit as error:
        return int(error.code)

    if stdout is None:
        stdout = sys.stdout
    if stderr is None:
        stderr = sys.stderr
    if process_factory is None:
        process_factory = _spawn_session
    if readiness is None:
        readiness = _wait_for_command_gateway_readiness
    if console_main is None:
        console_main = _run_existing_console
    return run_app(
        process_factory,
        readiness,
        console_main,
        clock,
        stdout,
        stderr,
        stdin,
    )


if __name__ == '__main__':
    raise SystemExit(main())
