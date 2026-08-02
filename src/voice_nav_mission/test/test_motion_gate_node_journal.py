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

"""Launch acceptance for MotionGate's test-only journal attachment seam."""

import atexit
import importlib.util
import os
from pathlib import Path
import signal
import time
import unittest

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
import pytest


def load_journal_support():
    """Load the sibling test helper without relying on the CTest cwd."""
    support_path = Path(__file__).with_name(
        'gate_event_journal_test_support.py',
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_gate_event_journal_test_support',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load Gate event journal test support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


journal_support = load_journal_support()


CLAIMED_SLOTS_HEADER_WORD = 11
WRITER_PID_HEADER_WORD = 13
JOURNAL_CAPACITY = 512
WAIT_TIMEOUT_SECONDS = 5.0
POLL_SECONDS = 0.005
PARTIAL_CONFIGURATION_ERROR = (
    'Gate event journal test parameters must be all-or-none'
)


class JournalFixtures:
    """Own the three independent journal generations used by this launch."""

    def __init__(self):
        """Create every owner atomically from the test's point of view."""
        self._owners = []
        try:
            self.valid = self._create(generation=101)
            self.name_only = self._create(generation=102)
            self.descriptor_only = self._create(generation=103)
        except BaseException:
            self.cleanup()
            raise

    def _create(self, *, generation):
        owner = journal_support.GateEventJournalOwner(
            capacity=JOURNAL_CAPACITY,
            generation=generation,
        )
        self._owners.append(owner)
        return owner

    def cleanup(self):
        """Idempotently release all parent-owned names and mappings."""
        first_error = None
        while self._owners:
            try:
                self._owners.pop().cleanup()
            except BaseException as error:
                if first_error is None:
                    first_error = error
        if first_error is not None:
            raise first_error


def motion_gate_action(*, name='', descriptor=''):
    """Build one exact-FQN MotionGate process with journal test overrides."""
    parameters = {'use_sim_time': True}
    if name:
        parameters['test_gate_event_journal_name'] = name
    if descriptor:
        parameters['test_gate_event_journal_descriptor'] = descriptor
    return Node(
        package='voice_nav_mission',
        executable='motion_gate_node',
        name='motion_gate_node',
        output='screen',
        parameters=[parameters],
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch one valid and two partial journal configurations."""
    fixtures = JournalFixtures()
    atexit.register(fixtures.cleanup)

    valid_gate = motion_gate_action(
        name=fixtures.valid.name,
        descriptor=fixtures.valid.descriptor,
    )
    name_only_gate = motion_gate_action(name=fixtures.name_only.name)
    descriptor_only_gate = motion_gate_action(
        descriptor=fixtures.descriptor_only.descriptor,
    )

    def cleanup_fixtures(event, context):
        del event, context
        fixtures.cleanup()
        return []

    return (
        LaunchDescription(
            [
                RegisterEventHandler(
                    OnShutdown(on_shutdown=cleanup_fixtures),
                ),
                valid_gate,
                name_only_gate,
                descriptor_only_gate,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            'fixtures': fixtures,
            'valid_gate': valid_gate,
            'name_only_gate': name_only_gate,
            'descriptor_only_gate': descriptor_only_gate,
        },
    )


class MotionGateNodeJournalTest(unittest.TestCase):
    """Verify the node process owns exactly one requested journal mapping."""

    def test_partial_configuration_exits_without_writer_claim(
        self,
        proc_info,
        proc_output,
        fixtures,
        name_only_gate,
        descriptor_only_gate,
    ):
        """Reject either partial parameter pair before claiming its region."""
        cases = (
            (name_only_gate, fixtures.name_only),
            (descriptor_only_gate, fixtures.descriptor_only),
        )
        for process, owner in cases:
            with self.subTest(process=process):
                proc_output.assertWaitFor(
                    PARTIAL_CONFIGURATION_ERROR,
                    process=process,
                    timeout=WAIT_TIMEOUT_SECONDS,
                )
                proc_info.assertWaitForShutdown(
                    process,
                    timeout=WAIT_TIMEOUT_SECONDS,
                )
                assertExitCodes(
                    proc_info,
                    process=process,
                    allowable_exit_codes=[1],
                )
                self.assertEqual(
                    owner.load_header_word(WRITER_PID_HEADER_WORD),
                    0,
                )
                self.assertEqual(
                    owner.load_header_word(CLAIMED_SLOTS_HEADER_WORD),
                    0,
                )

    def test_full_configuration_claims_exact_pid_and_mapping_survives_exit(
        self,
        proc_info,
        proc_output,
        fixtures,
        valid_gate,
    ):
        """Claim the launch PID and preserve the parent mapping after exit."""
        proc_info.assertWaitForStartup(
            valid_gate,
            timeout=WAIT_TIMEOUT_SECONDS,
        )
        proc_output.assertWaitFor(
            'started inhibited',
            process=valid_gate,
            timeout=WAIT_TIMEOUT_SECONDS,
        )
        process_details = valid_gate.process_details
        self.assertIsNotNone(process_details)
        launched_pid = process_details['pid']
        self.assertGreater(launched_pid, 0)

        deadline = time.monotonic() + WAIT_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            writer_pid = fixtures.valid.load_header_word(
                WRITER_PID_HEADER_WORD,
            )
            if writer_pid == launched_pid:
                break
            if writer_pid != 0:
                self.fail(
                    'journal was claimed by an unexpected process: '
                    f'expected={launched_pid}, actual={writer_pid}',
                )
            time.sleep(POLL_SECONDS)
        else:
            self.fail(
                'timed out waiting for the valid MotionGate writer claim',
            )

        os.kill(launched_pid, signal.SIGINT)
        proc_info.assertWaitForShutdown(
            valid_gate,
            timeout=WAIT_TIMEOUT_SECONDS,
        )
        assertExitCodes(
            proc_info,
            process=valid_gate,
            allowable_exit_codes=[0],
        )
        self.assertEqual(proc_info[valid_gate].pid, launched_pid)

        descriptor = fixtures.valid.open_existing()
        os.close(descriptor)
        self.assertFalse(fixtures.valid.region.closed)
        self.assertEqual(
            fixtures.valid.load_header_word(WRITER_PID_HEADER_WORD),
            launched_pid,
        )
        header, _ = fixtures.valid.snapshot()
        self.assertEqual(header[0], journal_support.MAGIC)
        self.assertEqual(header[7], fixtures.valid.generation)
        self.assertEqual(header[13], launched_pid)


@launch_testing.post_shutdown_test()
class MotionGateNodeJournalShutdownTest(unittest.TestCase):
    """Preserve exact expected exit statuses in the xUnit evidence."""

    def test_exit_codes_match_configuration_contract(
        self,
        proc_info,
        valid_gate,
        name_only_gate,
        descriptor_only_gate,
    ):
        """Require graceful valid exit and deterministic startup rejection."""
        assertExitCodes(
            proc_info,
            process=valid_gate,
            allowable_exit_codes=[0],
        )
        for process in (name_only_gate, descriptor_only_gate):
            assertExitCodes(
                proc_info,
                process=process,
                allowable_exit_codes=[1],
            )
