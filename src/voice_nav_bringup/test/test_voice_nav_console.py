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

"""Behavioral tests for the installed simulation-only command console."""

from __future__ import annotations

import ast
import importlib.util
import json
from io import BytesIO, StringIO
from pathlib import Path


def _load_console_module():
    source = Path(__file__).resolve().parents[1] / 'voice_nav_console.py'
    specification = importlib.util.spec_from_file_location(
        'voice_nav_console', source,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


class _RecordingTransport:
    def __init__(self, response=None):
        self.submissions = []
        self.response = response or {'accepted': True, 'reason': ''}

    def submit(self, text, timeout_s):
        self.submissions.append((text, timeout_s))
        return self.response


class _TimeoutTransport:
    def __init__(self):
        self.submissions = []

    def submit(self, text, timeout_s):
        self.submissions.append((text, timeout_s))
        raise TimeoutError('gateway response timeout')


class _FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class _FakeFuture:
    def __init__(self, done):
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return object()


class _BudgetClient:
    def __init__(self, clock, future):
        self.clock = clock
        self.future = future
        self.discovery_timeouts = []
        self.submission_count = 0

    def wait_for_service(self, timeout_sec):
        self.discovery_timeouts.append(timeout_sec)
        self.clock.advance(1.25)
        return True

    def call_async(self, request):
        del request
        self.submission_count += 1
        self.clock.advance(0.25)
        return self.future


class _BudgetRclpy:
    def __init__(self, clock):
        self.clock = clock
        self.response_timeouts = []

    def spin_until_future_complete(self, node, future, timeout_sec):
        del node, future
        self.response_timeouts.append(timeout_sec)
        self.clock.advance(timeout_sec)


def test_service_discovery_and_response_share_one_two_second_budget():
    """Use one deadline for fake discovery and fake response wait."""
    console = _load_console_module()
    clock = _FakeClock()
    client = _BudgetClient(clock, _FakeFuture(done=True))
    rclpy = _BudgetRclpy(clock)

    response, reason = console._call_set_parameters_with_budget(
        client=client,
        node=object(),
        rclpy=rclpy,
        request=object(),
        timeout_s=2.0,
        clock=clock,
    )

    assert response is not None
    assert reason == ''
    assert client.discovery_timeouts == [2.0]
    assert rclpy.response_timeouts == [0.5]
    assert client.submission_count == 1
    assert clock.now == 2.0


def test_response_timeout_stops_at_remaining_shared_budget():
    """Stop the fake response wait at the remaining shared budget."""
    console = _load_console_module()
    clock = _FakeClock()
    client = _BudgetClient(clock, _FakeFuture(done=False))
    rclpy = _BudgetRclpy(clock)

    response, reason = console._call_set_parameters_with_budget(
        client=client,
        node=object(),
        rclpy=rclpy,
        request=object(),
        timeout_s=2.0,
        clock=clock,
    )

    assert response is None
    assert reason == (
        'voice_nav_command_gateway set_parameters response timeout'
    )
    assert rclpy.response_timeouts == [0.5]
    assert clock.now == 2.0


def test_cli_static_authority_allowlist_is_only_the_fixed_parameter_client():
    """Keep CLI authority limited to the fixed parameter-service client."""
    source = Path(__file__).resolve().parents[1] / 'voice_nav_console.py'
    source_text = source.read_text(encoding='utf-8')
    tree = ast.parse(source_text)
    allowed_roots = {
        '__future__', 'argparse', 'json', 'rcl_interfaces', 'rclpy', 'sys',
        'time',
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
    assert (
        "PARAMETER_SERVICE = '/voice_nav_command_gateway/set_parameters'"
        in source_text
    )
    assert 'SetParameters' in source_text
    for forbidden in (
        'VoiceTurn', 'Mission', 'StopMission', 'ActionClient', 'Twist',
        'cmd_vel', 'create_publisher', 'create_subscription', 'publish(',
    ):
        assert forbidden not in source_text


def test_blank_command_is_rejected_before_transport_is_called():
    """Reject whitespace before creating or calling a transport."""
    console = _load_console_module()
    transport = _RecordingTransport()
    output = StringIO()

    exit_code = console.main(
        ['--command', ' \t'],
        transport=transport,
        stdin=StringIO(),
        stdout=output,
    )

    assert exit_code == 2
    assert transport.submissions == []
    assert output.getvalue() == (
        '{"reason":"command_text must contain non-whitespace UTF-8 text '
        'of at most 512 bytes","status":"rejected"}\n'
    )


def test_command_over_512_utf8_bytes_is_rejected_before_transport_is_called():
    """Reject an oversized UTF-8 command before transport submission."""
    console = _load_console_module()
    transport = _RecordingTransport()
    output = StringIO()

    exit_code = console.main(
        ['--command', '你' * 171],
        transport=transport,
        stdin=StringIO(),
        stdout=output,
    )

    assert exit_code == 2
    assert transport.submissions == []
    assert json.loads(output.getvalue()) == {
        'status': 'rejected',
        'reason': (
            'command_text must contain non-whitespace UTF-8 text '
            'of at most 512 bytes'
        ),
    }


def test_quit_line_exits_without_submitting_a_command():
    """Treat the quit sentinel as local control with zero submissions."""
    console = _load_console_module()
    transport = _RecordingTransport()
    output = StringIO()

    exit_code = console.main(
        [],
        transport=transport,
        stdin=BytesIO(':quit\n右转九十度\n'.encode('utf-8')),
        stdout=output,
    )

    assert exit_code == 0
    assert transport.submissions == []
    assert output.getvalue() == ''


def test_invalid_utf8_line_is_rejected_without_a_transport_call():
    """Reject malformed UTF-8 without invoking the transport."""
    console = _load_console_module()
    transport = _RecordingTransport()
    output = StringIO()

    exit_code = console.main(
        [],
        transport=transport,
        stdin=BytesIO(b'\xff\n'),
        stdout=output,
    )

    assert exit_code == 0
    assert transport.submissions == []
    assert json.loads(output.getvalue()) == {
        'status': 'rejected',
        'reason': 'command_text must be valid UTF-8',
    }


def test_busy_rejection_is_reported_once_without_an_automatic_retry():
    """Preserve a busy response without retrying the command."""
    console = _load_console_module()
    transport = _RecordingTransport({
        'accepted': False,
        'reason': 'command_text is busy; wait for the safe stationary barrier',
    })
    output = StringIO()

    exit_code = console.main(
        [],
        transport=transport,
        stdin=StringIO('右转九十度\n'),
        stdout=output,
    )

    assert exit_code == 0
    assert transport.submissions == [('右转九十度', 2.0)]
    assert json.loads(output.getvalue()) == {
        'status': 'rejected',
        'reason': 'command_text is busy; wait for the safe stationary barrier',
    }


def test_stop_phrases_are_each_submitted_once_without_console_interpretation():
    """Submit each exact stop phrase once without local interpretation."""
    console = _load_console_module()
    transport = _RecordingTransport()
    output = StringIO()
    phrases = ('停止', '小智停止', '紧急停止')

    exit_code = console.main(
        [],
        transport=transport,
        stdin=StringIO(''.join(f'{phrase}\n' for phrase in phrases)),
        stdout=output,
    )

    assert exit_code == 0
    assert transport.submissions == [(phrase, 2.0) for phrase in phrases]
    assert output.getvalue() == ''.join(
        '{"reason":"","status":"accepted"}\n' for _ in phrases
    )


def test_transport_timeout_is_unavailable_and_nonzero_without_a_retry():
    """Report one timeout as unavailable without retrying."""
    console = _load_console_module()
    transport = _TimeoutTransport()
    output = StringIO()

    exit_code = console.main(
        ['--command', '右转九十度'],
        transport=transport,
        stdin=StringIO(),
        stdout=output,
    )

    assert exit_code == 1
    assert transport.submissions == [('右转九十度', 2.0)]
    assert json.loads(output.getvalue()) == {
        'status': 'unavailable',
        'reason': 'gateway response timeout',
    }


def test_command_mode_submits_one_exact_text_and_reports_accepted_json():
    """Submit one command-mode text and emit stable accepted JSON."""
    console = _load_console_module()
    transport = _RecordingTransport()
    output = StringIO()

    exit_code = console.main(
        ['--command', '右转九十度'],
        transport=transport,
        stdin=StringIO(),
        stdout=output,
    )

    assert exit_code == 0
    assert transport.submissions == [('右转九十度', 2.0)]
    assert output.getvalue() == '{"reason":"","status":"accepted"}\n'
    assert json.loads(output.getvalue()) == {
        'status': 'accepted',
        'reason': '',
    }
