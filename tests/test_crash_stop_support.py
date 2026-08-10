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

"""Behavior tests for crash-stop observation helpers."""

from __future__ import annotations

from collections import deque
import importlib.util
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock


def _load_support():
    support_path = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'voice_nav_bringup'
        / 'test'
        / 'crash_stop_support.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_crash_stop_support_unit', support_path
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load crash-stop support')
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


support = _load_support()


def command(stamp_ns: int, linear_x: float):
    stamp = SimpleNamespace(
        sec=stamp_ns // 1_000_000_000,
        nanosec=stamp_ns % 1_000_000_000,
    )
    vector = SimpleNamespace(x=linear_x, y=0.0, z=0.0)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        twist=SimpleNamespace(linear=vector, angular=vector),
    )


class ConsumerTimeoutAnchorTest(unittest.TestCase):

    def test_selects_latest_nonzero_final_even_when_observer_delivers_it_late(self):
        samples = (
            (10, command(3_994_000_000, 0.01)),
            (20, command(4_098_000_000, 0.02)),
            (31, command(4_120_000_000, 0.03)),
        )

        receipt_ns, message = support.select_consumer_timeout_anchor(
            samples,
            signal_boundary_sim_ns=3_994_000_000,
        )

        self.assertEqual(receipt_ns, 31)
        self.assertEqual(support._stamp_ns(message), 4_120_000_000)

    def test_recomputes_frozen_timeout_from_actual_last_final(self):
        result = support.consumer_timeout_result(
            (
                (10, command(3_994_000_000, 0.01)),
                (20, command(4_098_000_000, 0.02)),
            ),
            signal_boundary_sim_ns=3_994_000_000,
            zero_sim_ns=4_449_000_000,
            zero_receipt_ns=30,
        )

        self.assertEqual(result['last_nonzero_sim_ns'], 4_098_000_000)
        self.assertEqual(result['last_nonzero_receipt_ns'], 20)
        self.assertEqual(result['delta_ns'], 351_000_000)

    def test_late_final_that_breaks_timeout_is_not_ignored(self):
        with self.assertRaisesRegex(AssertionError, '0.329000 s'):
            support.consumer_timeout_result(
                (
                    (10, command(3_994_000_000, 0.01)),
                    (31, command(4_120_000_000, 0.03)),
                ),
                signal_boundary_sim_ns=3_994_000_000,
                zero_sim_ns=4_449_000_000,
                zero_receipt_ns=30,
            )

    def test_quiescence_fence_observes_final_added_after_first_snapshot(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe.final_commands = deque(
            ((10, command(3_994_000_000, 0.01)),),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 4_500_000_000),), maxlen=20)
        probe.publisher_count = mock.Mock(
            side_effect=AssertionError('stale graph endpoint is not liveness')
        )

        class FakeTime:

            def __init__(self):
                self.now_ns = 0
                self.sleep_count = 0

            def monotonic(self):
                return self.now_ns / 1_000_000_000

            def monotonic_ns(self):
                return self.now_ns

            def sleep(self, seconds):
                self.now_ns += 100_000_000
                with probe.lock:
                    if self.sleep_count == 0:
                        probe.final_commands.append(
                            (50, command(4_120_000_000, 0.03))
                        )
                    probe.clock_samples.append(
                        (self.now_ns, 4_500_000_000 + self.now_ns)
                    )
                self.sleep_count += 1

        with mock.patch.object(support, 'time', FakeTime()):
            with self.assertRaisesRegex(AssertionError, '0.329000 s'):
                probe.wait_confirm_consumer_timeout(
                    3_994_000_000,
                    {
                        'zero_sim_ns': 4_449_000_000,
                        'zero_receipt_ns': 30,
                    },
                    timeout=1.0,
                )
        probe.publisher_count.assert_not_called()

    def test_rejects_missing_eligible_nonzero_final(self):
        samples = (
            (10, command(3_994_000_000, 0.0)),
            (31, command(3_900_000_000, 0.02)),
        )

        with self.assertRaisesRegex(
            AssertionError,
            'no eligible non-zero final command',
        ):
            support.select_consumer_timeout_anchor(
                samples,
                signal_boundary_sim_ns=3_994_000_000,
            )


if __name__ == '__main__':
    unittest.main()
