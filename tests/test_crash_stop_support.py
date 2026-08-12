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

    def test_probe_isolates_safety_observers_from_high_rate_sensors(self):
        callback_groups = {}

        class FakeNode:

            def create_subscription(
                self,
                _message_type,
                topic,
                _callback,
                _qos,
                *,
                callback_group=None,
            ):
                callback_groups[topic] = callback_group
                return object()

            def create_client(self, *_args, **_kwargs):
                return object()

        class FakeExecutor:

            def __init__(self, *_args, **_kwargs):
                pass

            def add_node(self, _node):
                pass

            def spin(self):
                pass

        class FakeThread:

            def __init__(self, *_args, **_kwargs):
                pass

            def start(self):
                pass

        interfaces = support.WorkspaceInterfaceTypes(
            execute_mission=object(),
            mission_state=object(),
            mission_step=object(),
            gate_state=object(),
            gate_control=object(),
        )
        with (
            mock.patch.object(
                support,
                '_get_workspace_interface_types',
                return_value=interfaces,
            ),
            mock.patch.object(
                support.rclpy,
                'create_node',
                return_value=FakeNode(),
            ),
            mock.patch.object(
                support,
                'Parameter',
                side_effect=lambda *_args, **_kwargs: object(),
            ),
            mock.patch.object(
                support,
                'MultiThreadedExecutor',
                FakeExecutor,
            ),
            mock.patch.object(
                support,
                'ActionClient',
                return_value=object(),
            ),
            mock.patch.object(support.threading, 'Thread', FakeThread),
        ):
            support.CrashStopProbe()

        safety_group = callback_groups[support.GATE_STATE_TOPIC]
        self.assertIsNotNone(safety_group)
        self.assertIs(
            safety_group,
            callback_groups[support.FINAL_COMMAND_TOPIC],
        )
        self.assertIs(
            safety_group,
            callback_groups[support.LIMITED_COMMAND_TOPIC],
        )
        self.assertIsNot(
            safety_group,
            callback_groups[support.CLOCK_TOPIC],
        )

    def test_pure_helpers_load_before_workspace_interfaces_are_built(self):
        original_import = __import__

        def reject_workspace_interfaces(name, *args, **kwargs):
            if name == 'voice_nav_interfaces' or name.startswith(
                'voice_nav_interfaces.'
            ) or name == 'voice_nav_mission' or name.startswith(
                'voice_nav_mission.'
            ):
                raise ModuleNotFoundError(name)
            return original_import(name, *args, **kwargs)

        with mock.patch(
            'builtins.__import__', side_effect=reject_workspace_interfaces
        ):
            isolated_support = _load_support()

        result = isolated_support.consumer_timeout_result(
            ((10, command(3_994_000_000, 0.01)),),
            (
                (20, command(4_000_000_000, 0.01)),
                (30, command(4_345_000_000, 0.0)),
            ),
            source_anchor=(10, command(3_994_000_000, 0.01)),
            zero_sim_ns=4_345_000_000,
            zero_receipt_ns=30,
        )
        self.assertEqual(result['delta_ns'], 351_000_000)

        with mock.patch(
            'builtins.__import__', side_effect=reject_workspace_interfaces
        ):
            with self.assertRaises(ModuleNotFoundError):
                isolated_support._load_workspace_interface_types()

    def test_workspace_interface_api_is_loaded_lazily_and_cached(self):
        isolated_support = _load_support()
        execute_mission = object()
        interfaces = isolated_support.WorkspaceInterfaceTypes(
            execute_mission=execute_mission,
            mission_state=object(),
            mission_step=object(),
            gate_state=object(),
            gate_control=object(),
        )

        with mock.patch.object(
            isolated_support,
            '_load_workspace_interface_types',
            return_value=interfaces,
        ) as loader:
            self.assertIs(isolated_support.ExecuteMission, execute_mission)
            self.assertIs(isolated_support.ExecuteMission, execute_mission)

        loader.assert_called_once_with()

    def test_gazebo_shutdown_api_is_resolved_only_on_first_access(self):
        isolated_support = _load_support()
        structured_stop = object()
        loaded = SimpleNamespace(structured_stop_gazebo=structured_stop)

        with mock.patch.object(
            isolated_support,
            '_load_gazebo_shutdown',
            return_value=loaded,
        ) as loader:
            self.assertIs(
                isolated_support.gazebo_shutdown.structured_stop_gazebo,
                structured_stop,
            )
            self.assertIs(
                isolated_support.gazebo_shutdown.structured_stop_gazebo,
                structured_stop,
            )

        loader.assert_called_once_with()

    def test_unique_trace_returns_authoritative_source_and_controller_stamps(self):
        source_anchor = (20, command(4_000_000_000, 0.02))

        result = support.consumer_timeout_result(
            (
                (10, command(3_990_000_000, 0.01)),
                source_anchor,
            ),
            (
                (15, command(3_995_000_000, 0.01)),
                (25, command(4_010_000_000, 0.02)),
                (30, command(4_351_000_000, 0.0)),
            ),
            source_anchor=source_anchor,
            zero_sim_ns=4_351_000_000,
            zero_receipt_ns=30,
        )

        self.assertEqual(
            result['authoritative_source_stamp_ns'], 4_000_000_000
        )
        self.assertEqual(
            result['controller_last_nonzero_update_ns'], 4_010_000_000
        )
        self.assertEqual(result['controller_zero_update_ns'], 4_351_000_000)
        self.assertEqual(result['delta_ns'], 351_000_000)

    def test_timeout_window_remains_strict_at_both_boundaries(self):
        source_anchor = (20, command(4_000_000_000, 0.02))
        final_samples = ((10, command(3_990_000_000, 0.01)), source_anchor)

        for zero_sim_ns, should_pass in (
            (4_350_000_000, False),
            (4_351_000_000, True),
            (4_360_000_000, True),
            (4_361_000_000, False),
        ):
            with self.subTest(zero_sim_ns=zero_sim_ns):
                limited_samples = (
                    (15, command(3_995_000_000, 0.01)),
                    (25, command(4_010_000_000, 0.02)),
                    (30, command(zero_sim_ns, 0.0)),
                )
                if should_pass:
                    result = support.consumer_timeout_result(
                        final_samples,
                        limited_samples,
                        source_anchor=source_anchor,
                        zero_sim_ns=zero_sim_ns,
                        zero_receipt_ns=30,
                    )
                    self.assertEqual(
                        result['delta_ns'], zero_sim_ns - 4_000_000_000
                    )
                else:
                    with self.assertRaisesRegex(
                        AssertionError, 'controller consumer timeout outside'
                    ):
                        support.consumer_timeout_result(
                            final_samples,
                            limited_samples,
                            source_anchor=source_anchor,
                            zero_sim_ns=zero_sim_ns,
                            zero_receipt_ns=30,
                        )

    def test_out_of_window_unique_trace_retains_launch_failure_evidence(self):
        source_anchor = (20, command(4_000_000_000, 0.02))

        for zero_sim_ns in (4_361_000_000, 4_374_000_000):
            with self.subTest(zero_sim_ns=zero_sim_ns):
                with self.assertRaises(AssertionError) as context:
                    support.consumer_timeout_result(
                        (source_anchor,),
                        (
                            (25, command(4_010_000_000, 0.02)),
                            (30, command(zero_sim_ns, 0.0)),
                        ),
                        source_anchor=source_anchor,
                        zero_sim_ns=zero_sim_ns,
                        zero_receipt_ns=30,
                    )

                evidence = getattr(context.exception, 'evidence', None)
                self.assertIsNotNone(evidence)
                self.assertEqual(
                    evidence['authoritative_source_stamp_ns'], 4_000_000_000
                )
                self.assertEqual(
                    evidence['controller_nonzero_update_stamp_ns'], 4_010_000_000
                )
                self.assertEqual(
                    evidence['controller_zero_update_stamp_ns'], zero_sim_ns)
                self.assertEqual(
                    evidence['delta_ns'], zero_sim_ns - 4_000_000_000
                )
                self.assertEqual(
                    evidence['accepted_window_ns'], [350_000_000, 360_000_000]
                )
                self.assertEqual(
                    evidence['association_basis'],
                    'unique ordered source/controller trace',
                )
                self.assertEqual(
                    evidence['reason'],
                    'controller consumer timeout outside accepted window',
                )

    def test_rejects_late_source_delivery_after_frozen_anchor(self):
        source_anchor = (20, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'late source delivery after frozen anchor',
        ) as context:
            support.consumer_timeout_result(
                (
                    source_anchor,
                    (40, command(4_020_000_000, 0.03)),
                ),
                (
                    (25, command(4_010_000_000, 0.02)),
                    (30, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=30,
            )

        self.assertEqual(
            context.exception.evidence['late_source_header_stamp_ns'],
            4_020_000_000,
        )

    def test_rejects_source_observer_receipt_late_for_controller_update(self):
        source_anchor = (50, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'late source observer receipt',
        ):
            support.consumer_timeout_result(
                (source_anchor,),
                (
                    (25, command(4_010_000_000, 0.02)),
                    (60, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=60,
            )

    def test_rejects_missing_controller_nonzero_before_first_zero(self):
        source_anchor = (20, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'missing non-zero controller observation before first zero',
        ):
            support.consumer_timeout_result(
                (source_anchor,),
                ((30, command(4_351_000_000, 0.0)),),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=30,
            )

    def test_rejects_same_valued_duplicate_source_commands(self):
        source_anchor = (20, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'same-valued duplicate source commands',
        ) as context:
            support.consumer_timeout_result(
                (
                    (10, command(3_990_000_000, 0.02)),
                    source_anchor,
                ),
                (
                    (25, command(4_010_000_000, 0.02)),
                    (30, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=30,
            )

        self.assertEqual(context.exception.evidence['matching_source_count'], 2)

    def test_rejects_out_of_order_source_trace(self):
        source_anchor = (20, command(3_999_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'out-of-order topic header stamp',
        ):
            support.consumer_timeout_result(
                (
                    (10, command(4_000_000_000, 0.01)),
                    source_anchor,
                ),
                (
                    (25, command(4_010_000_000, 0.02)),
                    (30, command(4_350_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_350_000_000,
                zero_receipt_ns=30,
            )

    def test_rejects_unassociated_controller_nonzero_before_first_zero(self):
        source_anchor = (20, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'unassociated non-zero controller output before first zero',
        ):
            support.consumer_timeout_result(
                (source_anchor,),
                (
                    (25, command(4_010_000_000, 0.02)),
                    (26, command(4_020_000_000, 0.03)),
                    (30, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=30,
            )

    def test_rejects_unassociated_controller_nonzero_before_matching_sample(self):
        source_anchor = (20, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'unassociated non-zero controller output before first zero',
        ) as context:
            support.consumer_timeout_result(
                (source_anchor,),
                (
                    (25, command(4_005_000_000, 0.03)),
                    (26, command(4_010_000_000, 0.02)),
                    (30, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=30,
            )

        self.assertEqual(
            context.exception.evidence['controller_update_stamp_ns'],
            4_005_000_000,
        )

    def test_rejects_unassociated_controller_nonzero_after_first_zero(self):
        source_anchor = (20, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'unassociated non-zero controller output after first zero',
        ):
            support.consumer_timeout_result(
                (source_anchor,),
                (
                    (25, command(4_010_000_000, 0.02)),
                    (30, command(4_351_000_000, 0.0)),
                    (40, command(4_360_000_000, 0.03)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=30,
            )

    def test_quiescence_fence_observes_final_added_after_first_snapshot(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        source_anchor = (10, command(3_994_000_000, 0.01))
        probe.final_commands = deque((source_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                (20, command(4_000_000_000, 0.01)),
                (30, command(4_449_000_000, 0.0)),
            ),
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
            with self.assertRaisesRegex(
                support.ConsumerTraceAmbiguous,
                'late source delivery after frozen anchor',
            ):
                probe.wait_confirm_consumer_timeout(
                    source_anchor,
                    {
                        'zero_sim_ns': 4_449_000_000,
                        'zero_receipt_ns': 30,
                    },
                    timeout=1.0,
                )
        probe.publisher_count.assert_not_called()

    def test_quiescence_fence_waits_for_delayed_limited_sample(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        source_anchor = (10, command(3_994_000_000, 0.01))
        probe.final_commands = deque((source_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                (20, command(4_000_000_000, 0.01)),
                (30, command(4_345_000_000, 0.0)),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 4_500_000_000),), maxlen=20)

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
                    if self.sleep_count == 1:
                        probe.limited_commands.append(
                            (40, command(4_346_000_000, 0.0))
                        )
                    probe.clock_samples.append(
                        (self.now_ns, 4_500_000_000 + self.now_ns)
                    )
                self.sleep_count += 1

        fake_time = FakeTime()
        with mock.patch.object(support, 'time', fake_time):
            result = probe.wait_confirm_consumer_timeout(
                source_anchor,
                {
                    'zero_sim_ns': 4_345_000_000,
                    'zero_receipt_ns': 30,
                },
                timeout=1.0,
            )

        self.assertEqual(result['delta_ns'], 351_000_000)
        self.assertGreaterEqual(fake_time.now_ns, 400_000_000)

if __name__ == '__main__':
    unittest.main()
