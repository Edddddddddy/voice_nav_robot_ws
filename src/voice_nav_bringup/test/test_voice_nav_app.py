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

"""Behavioral tests for the installed simulation-only app wrapper."""

from __future__ import annotations

import ast
import builtins
import importlib.util
from io import StringIO
from pathlib import Path
import signal


def _load_app_module():
    source = Path(__file__).resolve().parents[1] / 'voice_nav_app.py'
    specification = importlib.util.spec_from_file_location(
        'voice_nav_app', source,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


class _FakeProcess:
    def __init__(self):
        self.returncode = None
        self.group_signals = []
        self.wait_timeouts = []

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        self.returncode = 0
        return self.returncode

    def send_group_signal(self, signum):
        self.group_signals.append(signum)


class _SlowFakeProcess(_FakeProcess):
    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        if timeout == 10.0:
            raise TimeoutError('graceful shutdown still running')
        self.returncode = 0
        return self.returncode


class _StubbornFakeProcess(_FakeProcess):
    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        if timeout is not None:
            raise TimeoutError('shutdown still running')
        self.returncode = -signal.SIGKILL
        return self.returncode


class _SignalFailureFakeProcess(_FakeProcess):
    def send_group_signal(self, signum):
        del signum
        raise OSError('group signal failed')


def test_app_starts_fixed_session_waits_ready_then_enters_existing_console():
    """Compose the fixed session and console through injected seams."""
    app = _load_app_module()
    process = _FakeProcess()
    starts = []
    readiness_timeouts = []
    console_calls = []
    stdout = StringIO()
    stderr = StringIO()

    def process_factory(command, **kwargs):
        starts.append((tuple(command), kwargs))
        return process

    def readiness(timeout_s, clock):
        del clock
        readiness_timeouts.append(timeout_s)
        return {'status': 'ready', 'reason': 'ignored'}

    def console_main(*, stdin, stdout):
        console_calls.append((stdin, stdout))
        return 0

    result = app.main(
        [],
        process_factory=process_factory,
        readiness=readiness,
        clock=lambda: 0.0,
        console_main=console_main,
        stdin=StringIO(),
        stdout=stdout,
        stderr=stderr,
    )

    assert result == 0
    assert [command for command, _kwargs in starts] == [
        (
            'ros2', 'launch', 'voice_nav_bringup',
            'voice_nav_session.launch.py',
            'headless:=true',
            'shutdown_on_gazebo_exit:=true',
        ),
    ]
    assert readiness_timeouts == [60.0]
    assert stdout.getvalue() == '{"reason":"","status":"ready"}\n'
    assert len(console_calls) == 1
    assert console_calls[0][0].getvalue() == ''
    assert console_calls[0][1] is stdout


def test_installed_console_fallback_loads_extensionless_existing_console(
    monkeypatch, tmp_path,
):
    """Load the existing console when its installed entry has no suffix."""
    app = _load_app_module()
    console = tmp_path / 'voice_nav_console'
    console.write_text(
        'class RosParameterTransport:\n'
        '    def close(self):\n'
        '        pass\n'
        'def main(_argv, *, transport, stdin, stdout):\n'
        '    del transport, stdin, stdout\n'
        '    return 0\n',
        encoding='utf-8',
    )
    original_import = builtins.__import__

    def import_without_console(name, *args, **kwargs):
        if name == 'voice_nav_console':
            raise ModuleNotFoundError(name=name)
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', import_without_console)
    monkeypatch.setattr(app, '__file__', str(tmp_path / 'voice_nav_app'))

    assert app._run_existing_console(
        stdin=StringIO(), stdout=StringIO(),
    ) == 0


def test_quit_cleans_owned_group_gracefully_then_terminates_after_budgets():
    """A :quit console exit uses the bounded two-phase group teardown."""
    app = _load_app_module()
    process = _SlowFakeProcess()
    stdout = StringIO()

    def process_factory(_command, **_kwargs):
        return process

    def readiness(_timeout_s, _clock):
        return {'status': 'ready', 'reason': ''}

    def quit_console(*, stdin, stdout):
        assert stdin.readline() == ':quit\n'
        del stdout
        return 0

    result = app.main(
        [],
        process_factory=process_factory,
        readiness=readiness,
        clock=lambda: 0.0,
        console_main=quit_console,
        stdin=StringIO(':quit\n'),
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert stdout.getvalue() == (
        '{"reason":"","status":"ready"}\n'
        '{"reason":"cleanup_terminated","stage":"terminated",'
        '"status":"unavailable"}\n'
    )
    assert process.group_signals == [signal.SIGINT, signal.SIGTERM]
    assert process.wait_timeouts == [10.0, 5.0]


def test_child_output_is_redirected_to_app_stderr_and_args_not_forwarded():
    """Keep child logs off JSON stdout and reject arbitrary app arguments."""
    app = _load_app_module()
    process = _FakeProcess()
    stderr = StringIO()
    starts = []

    def process_factory(command, **kwargs):
        starts.append((command, kwargs))
        return process

    def readiness(_timeout_s, _clock):
        return {'status': 'ready', 'reason': ''}

    assert app.main(
        ['headless:=false'],
        process_factory=process_factory,
        readiness=readiness,
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=stderr,
    ) != 0
    assert starts == []

    stdout = StringIO()
    result = app.main(
        [],
        process_factory=process_factory,
        readiness=readiness,
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=stderr,
    )

    assert result == 0
    assert starts[0][1]['stdout'] is stderr
    assert starts[0][1]['stderr'] is stderr
    assert stdout.getvalue() == '{"reason":"","status":"ready"}\n'


def test_startup_failure_is_structured_nonzero_and_does_not_enter_console():
    """Fail closed when the single fixed child cannot be started."""
    app = _load_app_module()
    stdout = StringIO()

    def process_factory(_command, **_kwargs):
        raise OSError('ros2 not found')

    console_calls = []
    result = app.main(
        [],
        process_factory=process_factory,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: console_calls.append(True),
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert console_calls == []
    assert stdout.getvalue() == (
        '{"reason":"session_start_failed:ros2 not found",'
        '"status":"unavailable"}\n'
    )


def test_readiness_failure_cleans_child_and_returns_nonzero():
    """Stop a started child when gateway readiness never becomes ready."""
    app = _load_app_module()
    process = _FakeProcess()
    stdout = StringIO()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {
            'status': 'unavailable',
            'reason': 'command_gateway_readiness_timeout',
        },
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert process.group_signals == [signal.SIGINT]
    assert stdout.getvalue() == (
        '{"reason":"command_gateway_readiness_timeout",'
        '"status":"unavailable"}\n'
    )


def test_session_exit_after_readiness_is_nonzero_without_console_or_signals():
    """Treat a session that exits before console entry as a failure."""
    app = _load_app_module()
    process = _FakeProcess()
    console_calls = []
    stdout = StringIO()

    def readiness(_timeout_s, _clock):
        process.returncode = 17
        return {'status': 'ready', 'reason': ''}

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=readiness,
        console_main=lambda **_kwargs: console_calls.append(True),
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert console_calls == []
    assert process.group_signals == []
    assert stdout.getvalue() == (
        '{"reason":"session_exited_before_ready",'
        '"status":"unavailable"}\n'
    )


def test_ctrl_c_returns_130_after_cleaning_the_owned_group():
    """Propagate Ctrl+C as 130 while still cleaning the session group."""
    app = _load_app_module()
    process = _FakeProcess()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=(
            lambda **_kwargs: (_ for _ in ()).throw(KeyboardInterrupt)
        ),
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 130
    assert process.group_signals == [signal.SIGINT]


def test_ctrl_c_keeps_130_when_cleanup_escalates_to_kill():
    """Keep the signal exit code even when bounded cleanup needs SIGKILL."""
    app = _load_app_module()
    process = _StubbornFakeProcess()
    stdout = StringIO()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=(
            lambda **_kwargs: (_ for _ in ()).throw(KeyboardInterrupt)
        ),
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 130
    assert stdout.getvalue() == '{"reason":"","status":"ready"}\n'
    assert process.group_signals == [
        signal.SIGINT,
        signal.SIGTERM,
        signal.SIGKILL,
    ]


def test_console_nonzero_survives_forced_cleanup():
    """Do not replace an existing console failure with cleanup status."""
    app = _load_app_module()
    process = _StubbornFakeProcess()
    stdout = StringIO()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: 23,
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 23
    assert stdout.getvalue() == '{"reason":"","status":"ready"}\n'
    assert process.group_signals == [
        signal.SIGINT,
        signal.SIGTERM,
        signal.SIGKILL,
    ]


def test_cleanup_signal_failure_is_nonzero_with_bounded_failure_reason():
    """Report a failed cleanup stage without claiming a clean exit."""
    app = _load_app_module()
    process = _SignalFailureFakeProcess()
    stdout = StringIO()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert stdout.getvalue() == (
        '{"reason":"","status":"ready"}\n'
        '{"reason":"cleanup_failed","stage":"failed",'
        '"status":"unavailable"}\n'
    )


def test_stubborn_group_is_forced_after_both_bounded_shutdown_phases():
    """Use SIGKILL only after graceful and terminate budgets expire."""
    app = _load_app_module()
    process = _StubbornFakeProcess()
    stdout = StringIO()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert stdout.getvalue() == (
        '{"reason":"","status":"ready"}\n'
        '{"reason":"cleanup_killed","stage":"killed",'
        '"status":"unavailable"}\n'
    )
    assert process.group_signals == [
        signal.SIGINT,
        signal.SIGTERM,
        signal.SIGKILL,
    ]
    assert process.wait_timeouts == [10.0, 5.0, None]


def test_app_static_authority_is_limited_to_process_and_gateway_readiness():
    """Keep ROS motion and voice authority out of the app wrapper."""
    source = Path(__file__).resolve().parents[1] / 'voice_nav_app.py'
    source_text = source.read_text(encoding='utf-8')
    tree = ast.parse(source_text)
    allowed_roots = {
        '__future__', 'argparse', 'json', 'os', 'rcl_interfaces', 'rclpy',
        'runpy', 'signal', 'subprocess', 'sys', 'time', 'typing',
        'voice_nav_console',
    }
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(
                alias.name.split('.')[0] for alias in node.names
            )
        elif isinstance(node, ast.ImportFrom) and node.module is not None:
            imported_roots.add(node.module.split('.')[0])

    assert imported_roots <= allowed_roots
    assert source_text.count("'ros2'") == 1
    assert source_text.count("'voice_nav_session.launch.py'") == 1
    assert 'READINESS_SETTLE_S' not in source_text
    assert 'time.sleep(' not in source_text
    for forbidden in (
        'VoiceTurn', 'Mission', 'StopMission', 'Twist', 'cmd_vel',
        'create_publisher', 'create_subscription', 'publish(',
    ):
        assert forbidden not in source_text
