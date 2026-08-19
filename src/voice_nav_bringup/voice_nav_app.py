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

import argparse
from dataclasses import dataclass
import json
import os
import runpy
import subprocess
import sys
import time
from typing import Literal


Mode = Literal['motion', 'mapping', 'navigation']
Display = Literal['headless', 'gui']
InputProfile = Literal['console', 'vad-auto']
SESSION_LAUNCH_FILE = 'voice_nav_session.launch.py'


@dataclass(frozen=True)
class _InputSpec:
    """One immutable input frontend selected before process creation."""

    profile: InputProfile
    explicit: bool = False


class _SessionSpec:
    """One closed product composition selected before process creation."""

    __slots__ = ('mode', 'display', 'launch_file', 'command', '_frozen')

    def __init__(
        self,
        *,
        mode: Mode,
        display: Display,
        launch_file: str,
        command: tuple[str, ...],
    ) -> None:
        object.__setattr__(self, 'mode', mode)
        object.__setattr__(self, 'display', display)
        object.__setattr__(self, 'launch_file', launch_file)
        object.__setattr__(self, 'command', command)
        object.__setattr__(self, '_frozen', True)

    def __setattr__(self, name, value) -> None:
        if getattr(self, '_frozen', False):
            raise AttributeError('session spec is immutable')
        object.__setattr__(self, name, value)


@dataclass(frozen=True)
class _AppRun:
    """Dependencies and owned processes for one app invocation."""

    readiness: object
    mode_readiness: object
    frontend_factory: object
    frontend_readiness: object
    console_main: object
    clock: object
    stdin: object
    stdout: object
    stderr: object
    owned_processes: list


def _session_command(
    mode: Mode,
    display: Display,
    input_spec: _InputSpec | None = None,
) -> tuple[str, ...]:
    headless = 'true' if display == 'headless' else 'false'
    command = (
        'ros2',
        'launch',
        'voice_nav_bringup',
        SESSION_LAUNCH_FILE,
        f'mode:={mode}',
        f'headless:={headless}',
        'shutdown_on_gazebo_exit:=true',
    )
    if input_spec is None:
        return command
    session_input = (
        'console'
        if input_spec.profile == 'console'
        else 'none'
    )
    if input_spec.explicit or session_input == 'none':
        return command + (f'input:={session_input}',)
    return command


_SESSION_SPECS = {
    (mode, display): _SessionSpec(
        mode=mode,
        display=display,
        launch_file=launch_file,
        command=_session_command(mode, display),
    )
    for mode, launch_file in (
        ('motion', SESSION_LAUNCH_FILE),
        ('mapping', 'mapping_mvp.launch.py'),
        ('navigation', 'navigation_mvp.launch.py'),
    )
    for display in ('headless', 'gui')
}
SESSION_COMMAND = _SESSION_SPECS[('motion', 'headless')].command
READINESS_TIMEOUT_S = 60.0
COMMAND_GATEWAY_SERVICE = '/voice_nav_command_gateway/set_parameters'
_OWNED_RCLPY_CONTEXT = False


def _selected_session_spec(
    session_spec: _SessionSpec,
    input_spec: _InputSpec,
) -> _SessionSpec:
    """Bind one input frontend to the existing immutable mode selection."""
    return _SessionSpec(
        mode=session_spec.mode,
        display=session_spec.display,
        launch_file=session_spec.launch_file,
        command=_session_command(
            session_spec.mode, session_spec.display, input_spec,
        ),
    )


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


def _ensure_rclpy_context() -> None:
    """Provide the shared readiness context before mode observation begins."""
    global _OWNED_RCLPY_CONTEXT

    import rclpy

    if not rclpy.ok():
        rclpy.init(args=None)
        _OWNED_RCLPY_CONTEXT = True


def _spawn_session(command, *, stdin=subprocess.DEVNULL, stdout, stderr):
    """Start exactly one new process group for the fixed session command."""
    options = {
        'stdin': stdin,
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


def _spawn_voice_frontend(
    command, *, stdin=subprocess.DEVNULL, stdout, stderr,
):
    """Start the staged provider frontend in its own process group."""
    return _spawn_session(
        command, stdin=stdin, stdout=stdout, stderr=stderr,
    )


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


_MODE_READINESS = None
_SENSEVOICE_INPUT = None
_PROCESS_LIFECYCLE = None


def _sensevoice_input_module():
    """Load the installed package-private SenseVoice deep module."""
    global _SENSEVOICE_INPUT

    if _SENSEVOICE_INPUT is None:
        helper_path = os.path.join(
            os.path.dirname(__file__), '_sensevoice_input.py',
        )
        _SENSEVOICE_INPUT = runpy.run_path(helper_path)
    return _SENSEVOICE_INPUT


def _process_lifecycle_module():
    """Load the one installed process lifecycle implementation."""
    global _PROCESS_LIFECYCLE

    if _PROCESS_LIFECYCLE is None:
        helper_path = os.path.join(
            os.path.dirname(__file__), '_process_lifecycle.py',
        )
        _PROCESS_LIFECYCLE = runpy.run_path(helper_path)
    return _PROCESS_LIFECYCLE


def _wait_for_voice_input_sink_readiness(
    timeout_s: float,
    clock=time.monotonic,
) -> dict[str, str]:
    """Wait for the long-lived Agent input sink through its deep module."""
    return _sensevoice_input_module()['wait_for_input_sink_readiness'](
        timeout_s, clock, _stable_result,
    )


def _wait_for_mode_readiness(session_spec, deadline, clock):
    """Wait for the selected mode after the shared gateway deadline phase."""
    global _MODE_READINESS

    try:
        _ensure_rclpy_context()
    except Exception as error:
        return _stable_result(
            'unavailable',
            f'mode_readiness_context_failed:{_reason(error)}',
        )
    if _MODE_READINESS is None:
        helper_path = os.path.join(
            os.path.dirname(__file__), '_mode_readiness.py',
        )
        namespace = runpy.run_path(helper_path)
        _MODE_READINESS = namespace['wait_for_mode_readiness']
    return _MODE_READINESS(session_spec, deadline, clock)


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


_poll = _process_lifecycle_module()['poll_process']


def _readiness_is_ready(result) -> bool:
    return result is True or (
        isinstance(result, dict) and result.get('status') == 'ready'
    )


def _run_selected_input(
    process,
    owner_process,
    input_spec,
    console_main,
    clock,
    deadline,
    stdin,
    stdout,
) -> int:
    """Run exactly the chosen frontend after shared readiness succeeds."""
    if input_spec.profile == 'vad-auto':
        try:
            outcome = _process_lifecycle_module()[
                'wait_for_frontend_or_owner_exit'
            ](process, owner_process)
        except KeyboardInterrupt:
            return 130
        except Exception as error:
            _write_result(
                stdout,
                _stable_result(
                    'unavailable',
                    f'vad_auto_failed:{_reason(error)}',
                ),
            )
            return 1
        if outcome == 'owner_exited':
            _write_result(
                stdout,
                _stable_result('unavailable', 'session_exited_during_input'),
            )
            return 1
        return 0 if _poll(process) == 0 else 1
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
    return 0 if console_result is None else int(console_result)


def _run_started_session(
    process,
    session_spec,
    input_spec,
    run: _AppRun,
) -> int:
    """Run one mode, then stage exactly the selected input frontend."""
    if _poll(process) is not None:
        _write_result(
            run.stdout,
            _stable_result('unavailable', 'session_exited_on_start'),
        )
        return 1

    deadline = _clock_now(run.clock) + READINESS_TIMEOUT_S
    if input_spec.profile != 'console' or session_spec.mode != 'motion':
        try:
            mode_result = run.mode_readiness(session_spec, deadline, run.clock)
        except KeyboardInterrupt:
            return 130
        except Exception:
            mode_result = {
                **_stable_result('unavailable', 'mode_readiness_failed'),
                'mode': session_spec.mode,
                'stage': 'setup',
            }
    else:
        mode_result = _stable_result('ready')
    if not _readiness_is_ready(mode_result):
        if isinstance(mode_result, dict):
            mode_result = {
                **mode_result,
                'mode': mode_result.get('mode', session_spec.mode),
                'stage': mode_result.get('stage', 'unknown'),
            }
        else:
            mode_result = {
                **_stable_result('unavailable', 'mode_readiness_failed'),
                'mode': session_spec.mode,
                'stage': 'unknown',
            }
        _write_result(run.stdout, mode_result)
        return 1
    if _poll(process) is not None:
        _write_result(
            run.stdout,
            _stable_result('unavailable', 'session_exited_before_ready'),
        )
        return 1

    try:
        readiness_result = run.readiness(
            max(0.0, deadline - _clock_now(run.clock)), run.clock,
        )
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        _write_result(
            run.stdout,
            _stable_result(
                'unavailable', f'readiness_failed:{_reason(error)}',
            ),
        )
        return 1
    if not _readiness_is_ready(readiness_result):
        default_reason = (
            'command_gateway_not_ready'
            if input_spec.profile == 'console'
            else 'input_sink_not_ready'
        )
        reason = (
            readiness_result.get('reason', default_reason)
            if isinstance(readiness_result, dict)
            else default_reason
        )
        _write_result(run.stdout, _stable_result('unavailable', reason))
        return 1
    if _poll(process) is not None:
        reason = (
            'session_exited_before_ready'
            if input_spec.profile == 'console'
            else 'session_exited_before_input'
        )
        _write_result(run.stdout, _stable_result('unavailable', reason))
        return 1

    input_process = process
    if input_spec.profile == 'vad-auto':
        try:
            frontend_command = _sensevoice_input_module()[
                'build_vad_auto_command']()
            frontend_kwargs = {
                'stdout': run.stderr,
                'stderr': run.stderr,
            }
            input_process = run.frontend_factory(
                frontend_command, **frontend_kwargs,
            )
            if input_process is None:
                raise RuntimeError('frontend_factory returned no process')
            run.owned_processes.append(input_process)
        except Exception as error:
            _write_result(
                run.stdout,
                _stable_result(
                    'unavailable',
                    f'input_provider_start_failed:{_reason(error)}',
                ),
            )
            return 1

        frontend_result = run.frontend_readiness(
            input_process,
            max(0.0, deadline - _clock_now(run.clock)),
            run.clock,
            _poll,
            _stable_result,
        )
        if not _readiness_is_ready(frontend_result):
            reason = (
                frontend_result.get(
                    'reason', 'vad_auto_frontend_not_ready',
                )
                if isinstance(frontend_result, dict)
                else 'vad_auto_frontend_not_ready'
            )
            _write_result(
                run.stdout, _stable_result('unavailable', reason),
            )
            return 1

    if _poll(process) is not None:
        _write_result(
            run.stdout,
            _stable_result('unavailable', 'session_exited_before_input'),
        )
        return 1
    if _poll(input_process) is not None:
        if input_spec.profile == 'console':
            _write_result(
                run.stdout,
                _stable_result('unavailable', 'session_exited_before_ready'),
            )
            return 1

    _write_result(run.stdout, _stable_result('ready'))
    input_result = _run_selected_input(
        input_process,
        process,
        input_spec,
        run.console_main,
        run.clock,
        deadline,
        run.stdin,
        run.stdout,
    )

    if input_spec.profile == 'console' and _poll(process) is not None:
        _write_result(
            run.stdout,
            _stable_result('unavailable', 'session_exited_during_console'),
        )
        return 1
    return input_result


def run_app(
    process_factory,
    readiness,
    console_main,
    clock,
    stdout,
    stderr,
    stdin=None,
    session_spec: _SessionSpec | None = None,
    mode_readiness=None,
    input_spec: _InputSpec | None = None,
    frontend_factory=None,
    frontend_readiness=None,
) -> int:
    """Run one closed session spec and exactly one selected frontend."""
    owned_processes = []
    exit_code = 1
    try:
        if stdin is None:
            stdin = sys.stdin
        if session_spec is None:
            session_spec = _SESSION_SPECS[('motion', 'headless')]
        if input_spec is None:
            input_spec = _InputSpec(profile='console')
        if mode_readiness is None:
            mode_readiness = _wait_for_mode_readiness
        if frontend_factory is None:
            frontend_factory = _spawn_voice_frontend
        if frontend_readiness is None:
            frontend_readiness = (
                _sensevoice_input_module()['wait_for_frontend_readiness']
                if frontend_factory is _spawn_voice_frontend
                else lambda *_args: _stable_result('ready')
            )
        run = _AppRun(
            readiness=readiness,
            mode_readiness=mode_readiness,
            frontend_factory=frontend_factory,
            frontend_readiness=frontend_readiness,
            console_main=console_main,
            clock=clock,
            stdin=stdin,
            stdout=stdout,
            stderr=stderr,
            owned_processes=owned_processes,
        )
        process = process_factory(
            session_spec.command,
            stdout=stderr,
            stderr=stderr,
        )
        if process is None:
            raise RuntimeError('process_factory returned no process')
        owned_processes.append(process)
        exit_code = _run_started_session(
            process, session_spec, input_spec, run,
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
        cleanup_stage = 'graceful'
        lifecycle = _process_lifecycle_module()
        for process in reversed(owned_processes):
            try:
                process_stage = lifecycle['close_owned_process'](process)
            except Exception:
                process_stage = 'failed'
            if (
                cleanup_stage == 'graceful'
                and process_stage not in lifecycle['NORMAL_CLEANUP_STAGES']
            ):
                cleanup_stage = process_stage
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
    mode_readiness=None,
    frontend_factory=None,
    frontend_readiness=None,
) -> int:
    """Run one closed simulation session; no child arguments are accepted."""
    if stdout is None:
        stdout = sys.stdout
    if stderr is None:
        stderr = sys.stderr
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--mode',
        choices=('motion', 'mapping', 'navigation'),
        default='motion',
    )
    parser.add_argument(
        '--display',
        choices=('headless', 'gui'),
        default='headless',
    )
    parser.add_argument(
        '--input',
        choices=('console', 'vad-auto'),
        default=None,
    )
    try:
        arguments = parser.parse_args(argv)
    except SystemExit as error:
        return int(error.code)

    input_profile = arguments.input or 'console'
    input_spec = _InputSpec(
        profile=input_profile,
        explicit=arguments.input is not None,
    )

    base_session_spec = _SESSION_SPECS[(arguments.mode, arguments.display)]
    session_spec = _selected_session_spec(base_session_spec, input_spec)
    production_process_factory = process_factory is None
    if production_process_factory:
        process_factory = _spawn_session
    if readiness is None:
        readiness = (
            _wait_for_command_gateway_readiness
            if input_profile == 'console'
            else _wait_for_voice_input_sink_readiness
        )
    if console_main is None:
        console_main = _run_existing_console

    def invoke() -> int:
        return run_app(
            process_factory,
            readiness,
            console_main,
            clock,
            stdout,
            stderr,
            stdin,
            session_spec,
            mode_readiness,
            input_spec,
            frontend_factory,
            frontend_readiness,
        )

    if not production_process_factory:
        return invoke()
    lifecycle = _process_lifecycle_module()
    try:
        lease = lifecycle['acquire_session_lease']()
    except lifecycle['SessionBusyError']:
        _write_result(
            stdout, _stable_result('unavailable', 'session_already_running'),
        )
        return 1
    except lifecycle['SessionLeaseError'] as error:
        _write_result(
            stdout,
            _stable_result(
                'unavailable', f'session_lease_failed:{_reason(error)}',
            ),
        )
        return 1
    with lease:
        return invoke()


if __name__ == '__main__':
    raise SystemExit(main())
