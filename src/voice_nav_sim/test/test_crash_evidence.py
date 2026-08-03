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

import importlib.util
from pathlib import Path
import signal
import unittest

from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.events.process import ProcessExited
from launch.events.process import SignalProcess


def load_crash_evidence_support():
    support_path = (
        Path(__file__).resolve().parents[1]
        / 'test_support'
        / 'crash_evidence.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_crash_evidence_test_support',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load crash evidence support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


crash_evidence = load_crash_evidence_support()


def load_launch_crash_adapter_support():
    support_path = (
        Path(__file__).resolve().parents[1]
        / 'test_support'
        / 'launch_crash_adapter.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_launch_crash_adapter_test_support',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load launch crash adapter support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


launch_crash_adapter = load_launch_crash_adapter_support()


class EqualButDistinctAction:
    """Expose accidental equality-keyed process accounting."""

    def __init__(self, name):
        self.name = name

    def __eq__(self, _other):
        return True

    def __hash__(self):
        return 1


class OrderingCrashLedger(crash_evidence.CrashLedger):
    """Expose collaborator ordering without weakening ledger semantics."""

    def __init__(self, operations):
        super().__init__()
        self.operations = operations

    def arm_sigkill(self, action, *, signal_intent_monotonic_ns):
        self.operations.append('arm')
        return super().arm_sigkill(
            action,
            signal_intent_monotonic_ns=signal_intent_monotonic_ns,
        )

    def record_exit(
        self,
        action,
        returncode,
        *,
        observed_monotonic_ns=None,
    ):
        self.operations.append('record')
        return super().record_exit(
            action,
            returncode,
            observed_monotonic_ns=observed_monotonic_ns,
        )


class RecordingLaunchService:
    """Synchronously expose the fastest possible process-exit race."""

    def __init__(self, handler, exit_event, operations):
        self.emitted_events = []
        self.handler = handler
        self.exit_event = exit_event
        self.operations = operations

    def emit_event(self, event):
        self.operations.append('emit')
        self.emitted_events.append(event)
        if event.process_matcher(self.exit_event.action):
            self.handler.handle(self.exit_event, LaunchContext())


def process_exit_event(action, returncode):
    return ProcessExited(
        action=action,
        name='fixture',
        cmd=['fixture'],
        cwd=None,
        env={},
        pid=12345,
        returncode=returncode,
    )


class CrashLedgerTest(unittest.TestCase):
    def test_launch_adapter_records_clean_process_exit(self):
        candidate = ExecuteProcess(cmd=['/bin/true'])
        ledger = crash_evidence.CrashLedger()
        adapter = launch_crash_adapter.LaunchCrashAdapter(ledger)
        registration = adapter.expect_clean(candidate, 'candidate')

        registration.event_handler.handle(
            process_exit_event(candidate, 0),
            LaunchContext(),
        )

        label, returncode, observed_ns = adapter.exit_observation(candidate)
        self.assertEqual(label, 'candidate')
        self.assertEqual(returncode, 0)
        self.assertGreater(observed_ns, 0)
        self.assertEqual(
            adapter.assert_complete(),
            (('candidate', 0),),
        )

    def test_launch_adapter_uses_exact_sigkill_and_process_exit_time(self):
        authority = ExecuteProcess(cmd=['/bin/true'])
        equal_role_but_distinct = ExecuteProcess(cmd=['/bin/true'])
        operations = []
        clock_values = iter((100, 125))

        def ordered_monotonic_ns():
            operations.append('clock')
            return next(clock_values)

        original_monotonic_ns = (
            launch_crash_adapter.time.monotonic_ns
        )
        self.addCleanup(
            setattr,
            launch_crash_adapter.time,
            'monotonic_ns',
            original_monotonic_ns,
        )
        launch_crash_adapter.time.monotonic_ns = ordered_monotonic_ns

        ledger = OrderingCrashLedger(operations)
        adapter = launch_crash_adapter.LaunchCrashAdapter(ledger)
        registration = adapter.expect_sigkill(authority, 'authority')
        handler = registration.event_handler

        authority_exit = process_exit_event(authority, -signal.SIGKILL)
        other_exit = process_exit_event(
            equal_role_but_distinct,
            -signal.SIGKILL,
        )
        self.assertTrue(handler.matches(authority_exit))
        self.assertFalse(handler.matches(other_exit))

        launch_service = RecordingLaunchService(
            handler,
            authority_exit,
            operations,
        )
        signal_intent_ns = adapter.request_sigkill(
            launch_service,
            authority,
        )
        self.assertEqual(signal_intent_ns, 100)
        self.assertEqual(
            operations,
            ['clock', 'arm', 'emit', 'clock', 'record'],
        )
        self.assertEqual(len(launch_service.emitted_events), 1)
        signal_event = launch_service.emitted_events[0]
        self.assertIsInstance(signal_event, SignalProcess)
        self.assertEqual(signal_event.signal, signal.SIGKILL)
        self.assertTrue(signal_event.process_matcher(authority))
        self.assertFalse(
            signal_event.process_matcher(equal_role_but_distinct),
        )

        label, returncode, observed_ns = adapter.exit_observation(
            authority,
        )
        self.assertEqual(label, 'authority')
        self.assertEqual(returncode, -signal.SIGKILL)
        self.assertEqual(observed_ns, 125)
        self.assertGreaterEqual(observed_ns, signal_intent_ns)
        self.assertEqual(
            adapter.assert_complete(),
            (('authority', -signal.SIGKILL),),
        )

    def test_exact_action_exit_accounting_is_closed_and_exhaustive(self):
        authority = EqualButDistinctAction('authority')
        candidate = EqualButDistinctAction('candidate')
        gazebo = EqualButDistinctAction('gazebo')
        unknown_equal_action = EqualButDistinctAction('unknown')

        ledger = crash_evidence.CrashLedger()
        ledger.expect_sigkill(authority, 'authority')
        ledger.expect_clean(candidate, 'candidate')
        ledger.expect_clean(gazebo, 'gazebo')
        ledger.arm_sigkill(authority, signal_intent_monotonic_ns=1)

        with self.assertRaisesRegex(
            crash_evidence.CrashEvidenceError,
            'unknown exact action',
        ):
            ledger.record_exit(unknown_equal_action, -signal.SIGKILL)

        ledger.record_exit(authority, -signal.SIGKILL)
        with self.assertRaisesRegex(
            crash_evidence.CrashEvidenceError,
            'duplicate exit',
        ):
            ledger.record_exit(authority, -signal.SIGKILL)

        with self.assertRaisesRegex(
            crash_evidence.CrashEvidenceError,
            'candidate.*expected 0.*observed 137',
        ):
            ledger.record_exit(candidate, 137)
        ledger.record_exit(candidate, 0)
        ledger.record_exit(gazebo, 0)

        self.assertEqual(
            ledger.assert_complete(),
            (
                ('authority', -signal.SIGKILL),
                ('candidate', 0),
                ('gazebo', 0),
            ),
        )

        incomplete = crash_evidence.CrashLedger()
        incomplete.expect_clean(gazebo, 'gazebo')
        with self.assertRaisesRegex(
            crash_evidence.CrashEvidenceError,
            'missing exits: gazebo',
        ):
            incomplete.assert_complete()

        duplicate_label = crash_evidence.CrashLedger()
        duplicate_label.expect_clean(authority, 'authority')
        with self.assertRaisesRegex(
            crash_evidence.CrashEvidenceError,
            'duplicate action label',
        ):
            duplicate_label.expect_clean(candidate, 'authority')

        duplicate_action = crash_evidence.CrashLedger()
        duplicate_action.expect_clean(authority, 'authority')
        with self.assertRaisesRegex(
            crash_evidence.CrashEvidenceError,
            'exact action already declared',
        ):
            duplicate_action.expect_sigkill(authority, 'authority_kill')

    def test_signal_intent_is_not_exit_and_event_time_is_retained(self):
        authority = EqualButDistinctAction('authority')
        clean_process = EqualButDistinctAction('clean_process')
        ledger = crash_evidence.CrashLedger()
        ledger.expect_sigkill(authority, 'authority')
        ledger.expect_clean(clean_process, 'clean_process')

        ledger.arm_sigkill(authority, signal_intent_monotonic_ns=100)
        with self.assertRaisesRegex(
            crash_evidence.CrashEvidenceError,
            'missing exits: authority, clean_process',
        ):
            ledger.assert_complete()

        ledger.record_exit(
            authority,
            -signal.SIGKILL,
            observed_monotonic_ns=125,
        )
        ledger.record_exit(
            clean_process,
            0,
            observed_monotonic_ns=130,
        )
        self.assertEqual(
            ledger.exit_observation(authority),
            ('authority', -signal.SIGKILL, 125),
        )

        unarmed = crash_evidence.CrashLedger()
        unarmed.expect_sigkill(authority, 'authority')
        with self.assertRaisesRegex(
            crash_evidence.CrashEvidenceError,
            'SIGKILL action was not armed',
        ):
            unarmed.record_exit(
                authority,
                -signal.SIGKILL,
                observed_monotonic_ns=125,
            )

        invalid_order = crash_evidence.CrashLedger()
        invalid_order.expect_sigkill(authority, 'authority')
        invalid_order.arm_sigkill(
            authority,
            signal_intent_monotonic_ns=126,
        )
        with self.assertRaisesRegex(
            crash_evidence.CrashEvidenceError,
            'exit observation precedes signal intent',
        ):
            invalid_order.record_exit(
                authority,
                -signal.SIGKILL,
                observed_monotonic_ns=125,
            )

        with self.assertRaisesRegex(
            crash_evidence.CrashEvidenceError,
            'cannot arm clean action',
        ):
            ledger.arm_sigkill(
                clean_process,
                signal_intent_monotonic_ns=140,
            )

        with self.assertRaisesRegex(
            crash_evidence.CrashEvidenceError,
            'no declared actions',
        ):
            crash_evidence.CrashLedger().assert_complete()


if __name__ == '__main__':
    unittest.main()
