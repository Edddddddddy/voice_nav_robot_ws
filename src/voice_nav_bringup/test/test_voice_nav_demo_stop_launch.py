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

"""One-shot, package-private STOP product evidence harness."""

import atexit
import hashlib
import importlib.util
import json
import os
import select
import shlex
import tempfile
from pathlib import Path
import re
import signal
import subprocess
import sys
import unittest
from urllib.parse import urlparse
from xml.sax.saxutils import escape

import launch_testing
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
import pytest

from launch_testing.asserts import assertExitCodes

# launch_testing's isolated runner executes this file without adding its
# sibling test directory to sys.path; the replay module remains package-private.
_TEST_DIRECTORY = str(Path(__file__).resolve().parent)
if _TEST_DIRECTORY not in sys.path:
    sys.path.insert(0, _TEST_DIRECTORY)

from voice_nav_demo_stop_replay import (
    EVIDENCE_PREFIX,
    PRODUCT_EVIDENCE_PREFIX,
    evidence_from_log,
)
from process_identity import read_process_snapshot


_SESSION_EXEC_HELPER = Path(__file__).with_name('session_exec.py')
_SESSION_READY = b'VOICE_NAV_SESSION_READY\n'


def _load_scripted_voice_demo_launch():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch' / 'scripted_voice_demo.launch.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_stop_scripted_demo_launch', launch_path,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _source_checkout_head():
    # The caller freezes the exact checkout HEAD before launch_testing starts.
    injected_head = os.environ.get('VOICE_NAV_EXACT_HEAD')
    assert injected_head is not None
    assert re.fullmatch(r'[0-9a-f]{40}', injected_head)
    return injected_head


def _read_process_member(entry):
    stat_text = (entry / 'stat').read_text(encoding='utf-8')
    closing_parenthesis = stat_text.rfind(')')
    if closing_parenthesis < 0:
        raise ValueError('malformed process stat')
    fields_after_comm = stat_text[closing_parenthesis + 2:].split()
    if len(fields_after_comm) <= 19:
        raise ValueError('short process stat')
    command = tuple(
        item.decode('utf-8', errors='strict')
        for item in (entry / 'cmdline').read_bytes().split(b'\0')
        if item
    )
    if not command:
        raise ValueError('empty process command')
    return {
        'pid': int(entry.name),
        'ppid': int(fields_after_comm[1]),
        'state': fields_after_comm[0],
        'process_group_id': int(fields_after_comm[2]),
        'session_id': int(fields_after_comm[3]),
        'starttime_ticks': int(fields_after_comm[19]),
        'executable': os.path.realpath(os.readlink(entry / 'exe')),
        'cmdline': list(command),
        'owner_uid': entry.stat().st_uid,
    }


def _product_processes_alive(start_event):
    session_id = start_event.get('session_id')
    process_group_id = start_event.get('process_group_id')
    identity = start_event.get('identity')
    if (
        not isinstance(session_id, int)
        or session_id <= 0
        or not isinstance(process_group_id, int)
        or process_group_id <= 0
        or session_id != start_event['pid']
        or process_group_id != start_event['pid']
        or not isinstance(identity, dict)
    ):
        raise AssertionError('product process boundary is not an exclusive session')

    members = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            member = _read_process_member(entry)
            if member['state'] == 'Z' or member['session_id'] != session_id:
                continue
            if member['pid'] == identity['pid'] and (
                member['starttime_ticks'] != identity['starttime_ticks']
            ):
                raise AssertionError('product process PID identity was reused')
            members.append(member)
        except (FileNotFoundError, OSError, UnicodeError):
            continue
        except (ValueError, IndexError):
            continue
    return sorted(members, key=lambda member: member['pid'])


_REPARENT_PARENT_HELPER = r'''
import os
import subprocess
import sys

(
    control_fd,
    ready_read_fd,
    ready_write_fd,
    release_fd,
    exit_write_fd,
    status_write_fd,
) = (
    int(value) for value in sys.argv[1:7]
)
child_command = 'printf R; IFS= read -r _'
child = subprocess.Popen(
    ['/bin/sh', '-c', child_command, 'voice-nav-different-argv'],
    stdin=control_fd,
    stdout=ready_write_fd,
    pass_fds=(exit_write_fd,),
    stderr=subprocess.DEVNULL,
)
os.close(control_fd)
os.close(ready_write_fd)
os.close(exit_write_fd)
if os.read(ready_read_fd, 1) != b'R':
    os._exit(2)
os.close(ready_read_fd)
print(f'PARENT_READY child_pid={child.pid}', flush=True)
os.write(
    status_write_fd,
    f'PARENT_READY child_pid={child.pid}\n'.encode('ascii'),
)
os.close(status_write_fd)
os.read(release_fd, 1)
os._exit(0)
'''


def _exact_command_processes_alive(start_event):
    """Retain the pre-fix command match as a mutation-test oracle."""
    expected_command = tuple(start_event['command'])
    expected_executable = os.path.realpath(expected_command[0])
    members = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            proc_root = entry
            executable = os.path.realpath(os.readlink(proc_root / 'exe'))
            command = tuple(
                item.decode('utf-8', errors='strict')
                for item in (proc_root / 'cmdline').read_bytes().split(b'\0')
                if item
            )
            if executable != expected_executable or command != expected_command:
                continue
            members.append({'pid': int(entry.name)})
        except (FileNotFoundError, OSError, UnicodeError):
            continue
    return members


def _terminate_exact_process(pid, starttime_ticks, exit_read_fd):
    snapshot = read_process_snapshot(pid)
    assert snapshot.starttime_ticks == starttime_ticks
    os.kill(pid, signal.SIGTERM)
    readable, _, _ = select.select([exit_read_fd], [], [], 5.0)
    assert exit_read_fd in readable
    assert os.read(exit_read_fd, 1) == b''


def test_session_boundary_finds_reparented_different_command_member():
    """A reparented different executable must remain inside the frozen boundary."""
    control_read_fd, control_write_fd = os.pipe()
    ready_read_fd, ready_write_fd = os.pipe()
    release_read_fd, release_write_fd = os.pipe()
    exit_read_fd, exit_write_fd = os.pipe()
    status_read_fd, status_write_fd = os.pipe()
    helper = subprocess.Popen(
        [
            sys.executable,
            '-c',
            _REPARENT_PARENT_HELPER,
            str(control_read_fd),
            str(ready_read_fd),
            str(ready_write_fd),
            str(release_read_fd),
            str(exit_write_fd),
            str(status_write_fd),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
        pass_fds=(
            control_read_fd,
            ready_read_fd,
            ready_write_fd,
            release_read_fd,
            exit_write_fd,
            status_write_fd,
        ),
    )
    for descriptor in (
        control_read_fd,
        ready_read_fd,
        ready_write_fd,
        release_read_fd,
        exit_write_fd,
        status_write_fd,
    ):
        os.close(descriptor)

    child_pid = None
    try:
        readable, _, _ = select.select([status_read_fd], [], [], 5.0)
        assert status_read_fd in readable
        ready_line = os.read(status_read_fd, 128).decode('utf-8').strip()
        match = re.fullmatch(r'PARENT_READY child_pid=(\d+)', ready_line)
        assert match is not None, ready_line
        child_pid = int(match.group(1))

        snapshot = read_process_snapshot(helper.pid)
        start_event = {
            'pid': helper.pid,
            'command': list(snapshot.cmdline),
            'identity': {
                'pid': snapshot.pid,
                'starttime_ticks': snapshot.starttime_ticks,
                'executable': snapshot.executable,
                'cmdline': list(snapshot.cmdline),
            },
            'session_id': os.getsid(helper.pid),
            'process_group_id': os.getpgid(helper.pid),
            'owner_uid': (Path('/proc') / str(helper.pid)).stat().st_uid,
        }
        assert start_event['session_id'] == helper.pid
        assert start_event['process_group_id'] == helper.pid

        os.write(release_write_fd, b'X')
        helper.wait(timeout=5.0)

        child_snapshot = read_process_snapshot(child_pid)
        assert child_snapshot.executable != snapshot.executable
        assert os.getsid(child_pid) == start_event['session_id']
        assert os.getpgid(child_pid) == start_event['process_group_id']
        child_stat = (Path('/proc') / str(child_pid) / 'stat').read_text(
            encoding='utf-8',
        )
        child_stat_suffix = child_stat[child_stat.rfind(')') + 2:].split()
        assert int(child_stat_suffix[1]) != helper.pid

        assert _exact_command_processes_alive(start_event) == []
        remaining = _product_processes_alive(start_event)
        assert any(member['pid'] == child_pid for member in remaining)
    finally:
        if helper.poll() is None:
            try:
                os.write(release_write_fd, b'X')
            except OSError:
                pass
            helper.wait(timeout=5.0)
        if child_pid is not None:
            try:
                child_snapshot = read_process_snapshot(child_pid)
            except Exception:
                child_snapshot = None
            if child_snapshot is not None:
                _terminate_exact_process(
                    child_pid,
                    child_snapshot.starttime_ticks,
                    exit_read_fd,
                )
        os.close(control_write_fd)
        os.close(release_write_fd)
        os.close(exit_read_fd)
        os.close(status_read_fd)
        helper.stdout.close()
        helper.stderr.close()


def _capture_product_json(proc_output, speech_driver):
    stdout = ''.join(
        event.text.decode('utf-8', errors='strict')
        for event in proc_output[speech_driver]
        if event.from_stdout
    )
    product_lines = [
        line.split(PRODUCT_EVIDENCE_PREFIX, 1)[1]
        for line in stdout.splitlines()
        if PRODUCT_EVIDENCE_PREFIX in line
    ]
    assert len(product_lines) == 1
    product_json = product_lines[0]
    json.loads(product_json)
    return product_json


def _write_artifacts(raw_text, post_exit_facts):
    artifact_dir = Path(os.environ.get(
        'VOICE_NAV_STOP_ARTIFACT_DIR',
        Path.cwd() / 'voice_nav_stop_artifacts',
    )).resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    raw_path = artifact_dir / 'voice_nav_stop.raw.log'
    schema_source = Path(__file__).with_name('voice_nav_demo_stop_evidence.schema.json')
    schema_path = artifact_dir / 'voice_nav_stop.schema.json'
    xunit_path = artifact_dir / 'voice_nav_stop.xunit.xml'
    sha_path = artifact_dir / 'SHA256SUMS.txt'

    raw_path.write_text(raw_text, encoding='utf-8')
    schema_path.write_text(schema_source.read_text(encoding='utf-8'), encoding='utf-8')
    xunit_payload = escape(json.dumps(post_exit_facts, sort_keys=True))
    xunit_path.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<testsuite name="voice_nav_stop_product_smoke" tests="1" '
        'failures="0" errors="0">\n'
        '  <testcase classname="voice_nav_stop" '
        'name="post_exit_envelope" time="0">\n'
        f'    <system-out>{xunit_payload}</system-out>\n'
        '  </testcase>\n'
        '</testsuite>\n',
        encoding='utf-8',
    )
    artifact_files = (raw_path, xunit_path, schema_path)
    sha_path.write_text(
        ''.join(
            f'{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n'
            for path in artifact_files
        ),
        encoding='utf-8',
    )
    return artifact_dir, raw_path, xunit_path, schema_path, sha_path


def _create_session_ready_fifo():
    fifo_directory = Path(tempfile.mkdtemp(prefix='voice_nav_session_'))
    fifo_path = fifo_directory / 'ready'
    os.mkfifo(fifo_path)

    def cleanup():
        try:
            fifo_path.unlink()
        except FileNotFoundError:
            pass
        try:
            fifo_directory.rmdir()
        except OSError:
            pass

    atexit.register(cleanup)
    return fifo_path, cleanup


@pytest.mark.launch_test
def generate_test_description():
    """Launch exactly one installed scripted STOP product command."""
    launch_module = _load_scripted_voice_demo_launch()
    session_ready_fifo, cleanup_session_ready_fifo = _create_session_ready_fifo()
    session_prefix = shlex.join((
        sys.executable,
        str(_SESSION_EXEC_HELPER),
        '--ready-fifo',
        str(session_ready_fifo),
    ))
    actions, fixtures = launch_module.create_scripted_voice_demo(
        headless='true',
        shutdown_on_gazebo_exit='false',
        shutdown_when_demo_exits=True,
        scenario='stop',
        speech_driver_prefix=session_prefix,
    )
    speech_driver = fixtures['speech_driver']
    harness_state = {
        'start_events': [],
        'product_json': None,
        'cleanup_session_ready_fifo': cleanup_session_ready_fifo,
    }

    def record_product_start(event, _context):
        if event.action is not speech_driver:
            return
        ready_fd = os.open(session_ready_fifo, os.O_RDONLY | os.O_NONBLOCK)
        try:
            readable, _, _ = select.select([ready_fd], [], [], 10.0)
            assert ready_fd in readable
            assert os.read(ready_fd, len(_SESSION_READY)) == _SESSION_READY
        finally:
            os.close(ready_fd)
        snapshot = read_process_snapshot(event.pid)
        session_id = os.getsid(event.pid)
        process_group_id = os.getpgid(event.pid)
        owner_uid = (Path('/proc') / str(event.pid)).stat().st_uid
        assert session_id == event.pid
        assert process_group_id == event.pid
        identity = {
            'pid': snapshot.pid,
            'starttime_ticks': snapshot.starttime_ticks,
            'executable': snapshot.executable,
            'cmdline': list(snapshot.cmdline),
            'owner_uid': owner_uid,
        }
        start_event = {
            'pid': event.pid,
            'name': event.process_name,
            'process_group_id': process_group_id,
            'session_id': session_id,
            'owner_uid': owner_uid,
            'command': list(event.cmd),
            'identity': identity,
        }
        assert any(
            member['pid'] == event.pid
            and member['starttime_ticks'] == snapshot.starttime_ticks
            for member in _product_processes_alive(start_event)
        )
        harness_state['start_events'].append({
            **start_event,
        })

    return LaunchDescription([
        RegisterEventHandler(OnProcessStart(
            target_action=speech_driver,
            on_start=record_product_start,
        )),
        *actions,
        launch_testing.actions.ReadyToTest(),
    ]), {
        **fixtures,
        'harness_state': harness_state,
    }


class StopProductSmokeTest(unittest.TestCase):
    def test_product_record_is_emitted_once(
        self, proc_output, speech_driver, harness_state,
    ):
        proc_output.assertWaitFor(
            expected_output=PRODUCT_EVIDENCE_PREFIX,
            process=speech_driver,
            timeout=180.0,
            stream='stdout',
        )
        assert len(harness_state['start_events']) == 1
        harness_state['product_json'] = _capture_product_json(
            proc_output, speech_driver,
        )


@launch_testing.post_shutdown_test()
class StopProductSmokePostExitTest(unittest.TestCase):
    def test_builds_and_replays_post_exit_envelope(
        self,
        proc_info,
        speech_driver,
        llm_server,
        harness_state,
    ):
        assertExitCodes(proc_info, process=speech_driver, allowable_exit_codes=[0, -2])
        assert len(harness_state['start_events']) == 1
        assert harness_state['product_json'] is not None

        start = harness_state['start_events'][0]
        exit_event = proc_info[speech_driver]
        assert exit_event.pid == start['pid']
        assert not (Path('/proc') / str(start['pid'])).exists()
        remaining_products = _product_processes_alive(start)
        assert remaining_products == []

        with llm_server.lock:
            request_count = len(llm_server.requests)
        assert urlparse(llm_server.endpoint).hostname == '127.0.0.1'

        product_json = harness_state['product_json']
        product = json.loads(product_json)
        exact_head = _source_checkout_head()
        assert product['head'] == exact_head
        session = {
            'id': start['session_id'],
            'process_group_id': start['process_group_id'],
            'start_identity': start['identity'],
        }
        start_count = len(harness_state['start_events'])
        restart_count = start_count - 1
        envelope = {
            'schema_version': 4,
            'product_sha256': hashlib.sha256(
                product_json.encode('utf-8')
            ).hexdigest(),
            'head': exact_head,
            'scenario': 'stop',
            'provider_measurement': {
                'transport': urlparse(llm_server.endpoint).hostname,
                'request_count': request_count,
            },
            'audio_fence': product['audio_fence'],
            'post_exit': {
                'exit_code': exit_event.returncode,
                'product_descendants_empty': not remaining_products,
                'product_owners_empty': not any(
                    member['owner_uid'] == start['owner_uid']
                    for member in remaining_products
                ),
                'start_count': start_count,
                'restart_count': restart_count,
                'session': session,
                'remaining_members': remaining_products,
            },
        }
        raw_text = '\n'.join((
            PRODUCT_EVIDENCE_PREFIX + product_json,
            EVIDENCE_PREFIX + json.dumps(
                envelope, sort_keys=True, separators=(',', ':'),
            ),
            '',
        ))
        assert evidence_from_log(raw_text) == envelope
        artifact_dir, raw_path, xunit_path, schema_path, sha_path = _write_artifacts(
            raw_text,
            {
                'head': exact_head,
                'pid': start['pid'],
                'name': start['name'],
                'process_group_id': start['process_group_id'],
                'owner_uid': start['owner_uid'],
                'exit_code': exit_event.returncode,
                'start_count': start_count,
                'restart_count': restart_count,
                'session': session,
                'remaining_members': remaining_products,
            },
        )
        print(
            EVIDENCE_PREFIX + json.dumps(
                envelope, sort_keys=True, separators=(',', ':'),
            ),
            flush=True,
        )
        print(
            'VOICE_NAV_STOP_ARTIFACTS '
            + json.dumps({
                'directory': str(artifact_dir),
                'raw': str(raw_path),
                'xunit': str(xunit_path),
                'schema': str(schema_path),
                'sha256': str(sha_path),
            }, sort_keys=True),
            flush=True,
        )
        harness_state['cleanup_session_ready_fifo']()
