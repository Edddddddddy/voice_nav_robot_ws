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


if __name__ == '__main__':
    unittest.main()
