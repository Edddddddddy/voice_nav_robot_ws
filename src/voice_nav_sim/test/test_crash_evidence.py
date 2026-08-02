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


class EqualButDistinctAction:
    """Expose accidental equality-keyed process accounting."""

    def __init__(self, name):
        self.name = name

    def __eq__(self, _other):
        return True

    def __hash__(self):
        return 1


class CrashLedgerTest(unittest.TestCase):
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
