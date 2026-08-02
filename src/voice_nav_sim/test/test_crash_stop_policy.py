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

"""Behavior tests for the pure VN-0011A crash-stop evidence policy."""

import importlib.util
from pathlib import Path
import unittest


def load_crash_stop_policy():
    """Load the package-private policy without importing a ROS package."""
    support_path = (
        Path(__file__).resolve().parents[1]
        / 'test_support'
        / 'crash_stop_policy.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_crash_stop_policy_test_support',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load crash-stop policy support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


crash_stop_policy = load_crash_stop_policy()


class CrashStopPolicyTest(unittest.TestCase):
    """Keep producer and MotionGate-death evidence rules distinct."""

    def test_policy_constants_scope_marker_and_allocation_claims(self):
        self.assertIs(
            crash_stop_policy.PRODUCER_REQUIRE_UNIQUE_FINAL_MARKER,
            False,
        )
        self.assertIs(
            crash_stop_policy.GATE_REQUIRE_UNIQUE_FINAL_MARKER,
            True,
        )
        self.assertEqual(
            crash_stop_policy.GATE_FINAL_MARKER_MAX_COMMITS,
            1,
        )
        self.assertEqual(
            crash_stop_policy.GATE_ACK_DEADLINE_OUTPUT_PERIODS,
            1,
        )
        self.assertIs(
            crash_stop_policy.JOURNAL_INSTRUMENTATION_ALLOCATION_FREE,
            True,
        )
        self.assertIs(
            crash_stop_policy.UPSTREAM_GAZEBO_WRITE_ALLOCATION_FREE,
            False,
        )

    def test_exact_marker_commit_and_pre_repeat_ack_are_accepted(self):
        self.assertIsNone(
            crash_stop_policy.validate_gate_final_marker(
                marker_commit_count=1,
                ack_output_seq=41,
                next_output_seq=42,
            ),
        )

    def test_marker_commit_count_is_one_exact_uint64_value(self):
        for invalid_count in (0, 2, -1, True, 1.0, '1'):
            with self.subTest(invalid_count=invalid_count):
                with self.assertRaisesRegex(
                    crash_stop_policy.CrashStopPolicyError,
                    'committed exactly once',
                ):
                    crash_stop_policy.validate_gate_final_marker(
                        marker_commit_count=invalid_count,
                        ack_output_seq=41,
                        next_output_seq=42,
                    )

    def test_ack_and_next_output_are_positive_uint64_sequences(self):
        invalid_pairs = (
            (0, 1),
            (-1, 1),
            (True, 2),
            (1.0, 2),
            (1, 0),
            (1, -1),
            (1, False),
            (1, 1.0),
            ((1 << 64), (1 << 64) + 1),
            (1, (1 << 64)),
        )
        for ack_output_seq, next_output_seq in invalid_pairs:
            with self.subTest(
                ack_output_seq=ack_output_seq,
                next_output_seq=next_output_seq,
            ):
                with self.assertRaisesRegex(
                    crash_stop_policy.CrashStopPolicyError,
                    'positive uint64',
                ):
                    crash_stop_policy.validate_gate_final_marker(
                        marker_commit_count=1,
                        ack_output_seq=ack_output_seq,
                        next_output_seq=next_output_seq,
                    )

    def test_controller_ack_at_or_after_repeat_is_rejected(self):
        for ack_output_seq, next_output_seq in ((42, 42), (43, 42)):
            with self.subTest(
                ack_output_seq=ack_output_seq,
                next_output_seq=next_output_seq,
            ):
                with self.assertRaisesRegex(
                    crash_stop_policy.CrashStopPolicyError,
                    'before the next Gate output',
                ):
                    crash_stop_policy.validate_gate_final_marker(
                        marker_commit_count=1,
                        ack_output_seq=ack_output_seq,
                        next_output_seq=next_output_seq,
                    )


if __name__ == '__main__':
    unittest.main()
