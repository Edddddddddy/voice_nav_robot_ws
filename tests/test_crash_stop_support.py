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


class LimitedObservation:
    """Test-only limited sample carrying the Jazzy publication sequence."""

    def __init__(self, receipt_ns, message, publication_sequence_number):
        self.receipt_ns = receipt_ns
        self.message = message
        self.publication_sequence_number = publication_sequence_number

    def __iter__(self):
        return iter((self.receipt_ns, self.message))

    def __getitem__(self, index):
        return (self.receipt_ns, self.message)[index]


def limited_endpoint_fence(
    *, endpoint_gid='gid-controller', limited_receipt_fence_ns=20
):
    """Build the pre-SIGKILL endpoint/receipt boundary."""
    return {
        'endpoint_gid': endpoint_gid,
        'limited_receipt_fence_ns': limited_receipt_fence_ns,
    }


def fenced_consumer_zero(
    zero_sim_ns,
    zero_receipt_ns,
    *,
    endpoint_gid='gid-controller',
    limited_receipt_fence_ns=20,
):
    """Build the endpoint-bound first-zero proof used by confirmation."""
    return {
        'zero_sim_ns': zero_sim_ns,
        'zero_receipt_ns': zero_receipt_ns,
        **limited_endpoint_fence(
            endpoint_gid=endpoint_gid,
            limited_receipt_fence_ns=limited_receipt_fence_ns,
        ),
    }


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
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        source_anchor = (10, command(3_994_000_000, 0.01))
        probe.final_commands = deque((source_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                LimitedObservation(30, command(4_449_000_000, 0.0), 9),
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

        fake_time = FakeTime()
        with mock.patch.object(support, 'time', fake_time):
            with self.assertRaisesRegex(
                support.ConsumerTraceAmbiguous,
                'late source delivery after frozen anchor',
            ):
                probe.wait_confirm_consumer_timeout(
                    source_anchor,
                    fenced_consumer_zero(4_449_000_000, 30),
                    timeout=1.0,
                )
        probe.publisher_count.assert_not_called()
        self.assertGreaterEqual(fake_time.now_ns, 300_000_000)

    def test_quiescence_fence_waits_for_delayed_limited_sample(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        source_anchor = (10, command(3_994_000_000, 0.01))
        probe.final_commands = deque((source_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                LimitedObservation(30, command(4_345_000_000, 0.0), 9),
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
                            LimitedObservation(
                                40,
                                command(4_545_000_000, 0.0),
                                10,
                            )
                        )
                    probe.clock_samples.append(
                        (self.now_ns, 4_500_000_000 + self.now_ns)
                    )
                self.sleep_count += 1

        fake_time = FakeTime()
        with mock.patch.object(support, 'time', fake_time):
            result = probe.wait_confirm_consumer_timeout(
                source_anchor,
                fenced_consumer_zero(4_345_000_000, 30),
                timeout=1.0,
            )

        self.assertEqual(result['delta_ns'], 351_000_000)
        self.assertEqual(
            result['limited_zero_watermark']['watermark_stamp_ns'],
            4_545_000_000,
        )
        self.assertGreaterEqual(fake_time.now_ns, 200_000_000)

    def test_quiescence_fence_freezes_continuous_limited_zero_prefix(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        source_anchor = (10, command(3_994_000_000, 0.01))
        probe.final_commands = deque((source_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                LimitedObservation(30, command(4_345_000_000, 0.0), 9),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 4_500_000_000),), maxlen=20)

        class FakeTime:

            def __init__(self):
                self.now_ns = 0

            def monotonic(self):
                return self.now_ns / 1_000_000_000

            def monotonic_ns(self):
                return self.now_ns

            def sleep(self, seconds):
                self.now_ns += 100_000_000
                with probe.lock:
                    probe.limited_commands.append(
                        LimitedObservation(
                            30 + self.now_ns,
                            command(4_345_000_000 + self.now_ns, 0.0),
                            9 + self.now_ns // 100_000_000,
                        )
                    )
                    probe.clock_samples.append(
                        (self.now_ns, 4_500_000_000 + self.now_ns)
                    )

        fake_time = FakeTime()
        with mock.patch.object(support, 'time', fake_time):
            result = probe.wait_confirm_consumer_timeout(
                source_anchor,
                fenced_consumer_zero(4_345_000_000, 30),
                timeout=1.0,
            )

        self.assertEqual(result['delta_ns'], 351_000_000)
        self.assertEqual(
            result['limited_zero_watermark']['watermark_stamp_ns'],
            4_545_000_000,
        )
        self.assertEqual(
            result['endpoint_continuity']['endpoint_gid'], 'gid-controller'
        )

    def test_quiescence_fence_reports_limited_watermark_timeout(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        source_anchor = (10, command(3_994_000_000, 0.01))
        probe.final_commands = deque((source_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                LimitedObservation(30, command(4_345_000_000, 0.0), 9),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 4_500_000_000),), maxlen=20)

        class FakeTime:

            def __init__(self):
                self.now_ns = 0

            def monotonic(self):
                return self.now_ns / 1_000_000_000

            def monotonic_ns(self):
                return self.now_ns

            def sleep(self, seconds):
                self.now_ns += 100_000_000
                with probe.lock:
                    probe.clock_samples.append(
                        (self.now_ns, 4_500_000_000 + self.now_ns)
                    )

        with mock.patch.object(support, 'time', FakeTime()):
            with self.assertRaisesRegex(
                support.ConsumerTraceWatermarkTimeout,
                'limited zero watermark did not advance',
            ) as context:
                probe.wait_confirm_consumer_timeout(
                    source_anchor,
                    fenced_consumer_zero(4_345_000_000, 30),
                    timeout=0.5,
                )

        self.assertEqual(context.exception.evidence['endpoint_gid'], 'gid-controller')
        self.assertEqual(
            context.exception.evidence['first_zero_stamp_ns'], 4_345_000_000
        )
        self.assertGreaterEqual(
            context.exception.evidence['final_quiescence']['wall_elapsed_ns'],
            200_000_000,
        )

    def test_quiescence_fence_rejects_endpoint_change(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        endpoint_gids = iter((
            ('gid-controller',),
            ('gid-controller',),
            ('gid-replacement',),
        ))
        probe._limited_endpoint_snapshot_provider = lambda: next(endpoint_gids)
        source_anchor = (10, command(3_994_000_000, 0.01))
        probe.final_commands = deque((source_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                LimitedObservation(30, command(4_345_000_000, 0.0), 9),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 4_500_000_000),), maxlen=20)

        class FakeTime:

            now_ns = 0

            @classmethod
            def monotonic(cls):
                return cls.now_ns / 1_000_000_000

            @classmethod
            def monotonic_ns(cls):
                return cls.now_ns

            @classmethod
            def sleep(cls, seconds):
                cls.now_ns += 100_000_000
                with probe.lock:
                    probe.clock_samples.append(
                        (cls.now_ns, 4_500_000_000 + cls.now_ns)
                    )

        with mock.patch.object(support, 'time', FakeTime):
            with self.assertRaisesRegex(
                support.ConsumerTraceAmbiguous,
                'limited endpoint changed during continuity fence',
            ) as context:
                probe.wait_confirm_consumer_timeout(
                    source_anchor,
                    fenced_consumer_zero(4_345_000_000, 30),
                    timeout=1.0,
                )

        self.assertEqual(
            context.exception.evidence['expected_endpoint_gid'], 'gid-controller'
        )
        self.assertEqual(
            context.exception.evidence['observed_endpoint_gid'], 'gid-replacement'
        )

    def test_quiescence_fence_requires_one_nonempty_endpoint_gid(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: (
            'gid-controller',
            'gid-shadow',
        )
        probe.final_commands = deque(
            ((10, command(3_994_000_000, 0.01)),), maxlen=20
        )
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                LimitedObservation(30, command(4_345_000_000, 0.0), 9),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 4_500_000_000),), maxlen=20)

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'limited endpoint set was not singleton',
        ) as context:
            probe.wait_confirm_consumer_timeout(
                probe.final_commands[0],
                fenced_consumer_zero(4_345_000_000, 30),
                timeout=1.0,
            )

        self.assertEqual(context.exception.evidence['endpoint_count'], 2)
        self.assertEqual(context.exception.evidence['checkpoint'], 'observation')

    def test_quiescence_fence_rejects_all_zero_endpoint_gid(self):
        for endpoint_gid in ('', '0' * 32):
            with self.subTest(endpoint_gid=endpoint_gid):
                probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
                probe.lock = threading.Lock()
                probe._limited_endpoint_snapshot_provider = (
                    lambda: (endpoint_gid,)
                )
                probe.final_commands = deque(
                    ((10, command(3_994_000_000, 0.01)),), maxlen=20
                )
                probe.limited_commands = deque(
                    (
                        LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                        LimitedObservation(30, command(4_345_000_000, 0.0), 9),
                    ),
                    maxlen=20,
                )
                probe.clock_samples = deque(
                    ((0, 4_500_000_000),), maxlen=20
                )

                with self.assertRaisesRegex(
                    support.ConsumerTraceAmbiguous,
                    'limited endpoint identity unavailable',
                ) as context:
                    probe.wait_confirm_consumer_timeout(
                        probe.final_commands[0],
                        fenced_consumer_zero(4_345_000_000, 30),
                        timeout=1.0,
                    )

                self.assertEqual(
                    context.exception.evidence['endpoint_gid'], endpoint_gid
                )

    def test_signal_boundary_motion_proof_returns_atomic_endpoint_receipt_fence(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._gate_state_type = SimpleNamespace(ARMED=1)
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        probe.gate_states = deque(
            ((900, SimpleNamespace(
                state=1,
                authority_live=True,
                candidate_fresh=True,
                motion_inhibited=False,
                zero_selected=False,
            )),),
            maxlen=20,
        )
        probe.final_commands = deque(
            ((901, command(4_000_000_000, 0.01)),), maxlen=20
        )
        probe.limited_commands = deque(
            (LimitedObservation(902, command(4_001_000_000, 0.01), 8),),
            maxlen=20,
        )
        probe.clock_samples = deque(
            ((800, 4_000_000_000), (903, 4_001_000_000)), maxlen=20
        )

        class FakeTime:

            @staticmethod
            def monotonic_ns():
                return 1_000

        with mock.patch.object(support, 'time', FakeTime):
            boundary = probe.capture_motion_at_signal_boundary()

        self.assertEqual(boundary['limited_endpoint_fence'], {
            'endpoint_gid': 'gid-controller',
            'limited_receipt_fence_ns': 902,
        })
        self.assertEqual(boundary['limited'][0], 902)

    def test_consumer_zero_rejects_endpoint_replacement_before_or_at_first_zero(self):
        for timing in ('before-first-zero', 'at-first-zero'):
            with self.subTest(timing=timing):
                probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
                probe.lock = threading.Lock()
                endpoint = {'gid': 'gid-original'}
                probe._limited_endpoint_snapshot_provider = (
                    lambda: (endpoint['gid'],)
                )
                probe.limited_commands = deque(
                    (
                        LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                    ),
                    maxlen=20,
                )
                probe.clock_samples = deque(
                    ((0, 4_500_000_000),), maxlen=20
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
                                endpoint['gid'] = 'gid-replacement'
                                if timing == 'at-first-zero':
                                    probe.limited_commands.append(
                                        LimitedObservation(
                                            30,
                                            command(4_345_000_000, 0.0),
                                            9,
                                        )
                                    )
                            if (
                                self.sleep_count == 1
                                and timing == 'before-first-zero'
                            ):
                                probe.limited_commands.append(
                                    LimitedObservation(
                                        30,
                                        command(4_345_000_000, 0.0),
                                        9,
                                    )
                                )
                            probe.clock_samples.append(
                                (
                                    self.now_ns,
                                    4_500_000_000 + self.now_ns,
                                )
                            )
                        self.sleep_count += 1

                with mock.patch.object(support, 'time', FakeTime()):
                    with self.assertRaisesRegex(
                        support.ConsumerTraceAmbiguous,
                        'limited endpoint changed during continuity fence',
                    ) as context:
                        probe.wait_consumer_zero(
                            0,
                            3_999_000_000,
                            limited_endpoint_fence(
                                endpoint_gid='gid-original',
                            ),
                        )

                self.assertEqual(
                    context.exception.evidence['expected_endpoint_gid'],
                    'gid-original',
                )
                self.assertEqual(
                    context.exception.evidence['observed_endpoint_gid'],
                    'gid-replacement',
                )

    def test_consumer_zero_never_skips_a_known_first_zero_for_later_zero(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                LimitedObservation(30, command(4_345_000_000, 0.0), 9),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 4_500_000_000),), maxlen=20)

        class FakeTime:

            def __init__(self):
                self.now_ns = 0

            def monotonic(self):
                return self.now_ns / 1_000_000_000

            def monotonic_ns(self):
                return self.now_ns

            def sleep(self, seconds):
                self.now_ns += 1_000_000_000
                with probe.lock:
                    if self.now_ns == 1_000_000_000:
                        probe.limited_commands.append(
                            LimitedObservation(
                                40,
                                command(4_355_000_000, 0.0),
                                10,
                            )
                        )
                    probe.clock_samples.append(
                        (self.now_ns, 4_500_000_000 + self.now_ns)
                    )

        with mock.patch.object(support, 'time', FakeTime()):
            with self.assertRaisesRegex(
                support.ConsumerTraceAmbiguous,
                'controller first zero predates signal-boundary receipt fence',
            ):
                probe.wait_consumer_zero(
                    0,
                    3_999_000_000,
                    limited_endpoint_fence(limited_receipt_fence_ns=35),
                )

    def test_consumer_zero_selects_known_first_zero_after_pre_signal_fence(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                LimitedObservation(30, command(4_345_000_000, 0.0), 9),
                LimitedObservation(40, command(4_355_000_000, 0.0), 10),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 4_500_000_000),), maxlen=20)

        class FakeTime:

            @staticmethod
            def monotonic():
                return 0.0

            @staticmethod
            def monotonic_ns():
                return 0

        with mock.patch.object(support, 'time', FakeTime):
            consumer_zero = probe.wait_consumer_zero(
                0,
                3_999_000_000,
                limited_endpoint_fence(limited_receipt_fence_ns=20),
            )

        self.assertEqual(consumer_zero['zero_receipt_ns'], 30)
        self.assertEqual(consumer_zero['zero_sim_ns'], 4_345_000_000)

    def test_consumer_zero_rejects_adjacent_sequence_after_first_zero_endpoint_replacement(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        endpoint = {'gid': 'gid-original'}
        probe._limited_endpoint_snapshot_provider = lambda: (endpoint['gid'],)
        source_anchor = (10, command(3_994_000_000, 0.01))
        probe.final_commands = deque((source_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(4_000_000_000, 0.01), 8),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 4_500_000_000),), maxlen=20)

        class FirstZeroTime:

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
                        probe.limited_commands.append(
                            LimitedObservation(
                                30,
                                command(4_345_000_000, 0.0),
                                9,
                            )
                        )
                    probe.clock_samples.append(
                        (self.now_ns, 4_500_000_000 + self.now_ns)
                    )
                self.sleep_count += 1

        with mock.patch.object(support, 'time', FirstZeroTime()):
            consumer_zero = probe.wait_consumer_zero(
                0,
                3_999_000_000,
                limited_endpoint_fence(endpoint_gid='gid-original'),
            )

        self.assertEqual(consumer_zero['endpoint_gid'], 'gid-original')
        self.assertEqual(consumer_zero['limited_receipt_fence_ns'], 20)

        endpoint['gid'] = 'gid-replacement'
        with probe.lock:
            probe.limited_commands.append(
                LimitedObservation(40, command(4_545_000_000, 0.0), 10)
            )

        class ConfirmTime:

            now_ns = 0

            @classmethod
            def monotonic(cls):
                return cls.now_ns / 1_000_000_000

            @classmethod
            def monotonic_ns(cls):
                return cls.now_ns

            @classmethod
            def sleep(cls, seconds):
                cls.now_ns += 100_000_000
                with probe.lock:
                    probe.clock_samples.append(
                        (cls.now_ns, 4_500_000_000 + cls.now_ns)
                    )

        with mock.patch.object(support, 'time', ConfirmTime):
            with self.assertRaisesRegex(
                support.ConsumerTraceAmbiguous,
                'limited endpoint changed during continuity fence',
            ) as context:
                probe.wait_confirm_consumer_timeout(
                    source_anchor,
                    consumer_zero,
                    timeout=1.0,
                )

        self.assertEqual(
            context.exception.evidence['expected_endpoint_gid'], 'gid-original'
        )
        self.assertEqual(
            context.exception.evidence['observed_endpoint_gid'], 'gid-replacement'
        )

    def test_limited_endpoint_snapshot_counts_empty_and_all_zero_raw_endpoints(self):
        for invalid_gid in (b'', b'\0' * 16):
            with self.subTest(invalid_gid=invalid_gid):
                probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
                probe.node = SimpleNamespace(
                    get_publishers_info_by_topic=lambda _topic: (
                        SimpleNamespace(endpoint_gid=b'\x01'),
                        SimpleNamespace(endpoint_gid=invalid_gid),
                    )
                )

                with self.assertRaisesRegex(
                    support.ConsumerTraceAmbiguous,
                    'limited endpoint set was not singleton',
                ) as context:
                    probe._require_limited_endpoint_continuity(
                        expected_gid=None,
                        checkpoint='endpoint-count',
                    )

                self.assertEqual(context.exception.evidence['endpoint_count'], 2)

    def test_quiescence_fence_rechecks_delayed_final_at_deadline(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        source_anchor = (10, command(3_994_000_000, 0.01))
        probe.final_commands = deque((source_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                LimitedObservation(30, command(4_345_000_000, 0.0), 9),
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
                    if self.sleep_count == 2:
                        probe.final_commands.append(
                            (40, command(4_120_000_000, 0.03))
                        )
                    probe.clock_samples.append(
                        (self.now_ns, 4_500_000_000 + self.now_ns)
                    )
                self.sleep_count += 1

        with mock.patch.object(support, 'time', FakeTime()):
            with self.assertRaises(AssertionError) as context:
                probe.wait_confirm_consumer_timeout(
                    source_anchor,
                    fenced_consumer_zero(4_345_000_000, 30),
                    timeout=0.25,
                )

        self.assertNotIsInstance(
            context.exception, support.ConsumerTraceWatermarkTimeout
        )
        self.assertIn(
            'final command observer did not quiesce', str(context.exception)
        )

    def test_limited_callback_preserves_jazzy_publication_sequence(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe.limited_commands = deque(maxlen=20)

        probe._append_limited(
            command(4_345_000_000, 0.0),
            {'publication_sequence_number': 17},
        )
        probe._append_limited(
            command(4_355_000_000, 0.0),
            {'publication_sequence_number': None},
        )

        first, second = probe.limited_commands
        self.assertEqual(first.publication_sequence_number, 17)
        self.assertIsNone(second.publication_sequence_number)

    def test_limited_watermark_rejects_invalid_publication_sequences(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        consumer_zero = {
            'zero_sim_ns': 4_345_000_000,
            'zero_receipt_ns': 30,
        }
        for sequence, reason in (
            (None, 'limited publication sequence unavailable'),
            (9, 'duplicate limited publication sequence'),
            (8, 'limited publication sequence regressed'),
        ):
            with self.subTest(sequence=sequence):
                limited = (
                    LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                    LimitedObservation(30, command(4_345_000_000, 0.0), 9),
                    LimitedObservation(40, command(4_545_000_000, 0.0), sequence),
                )
                with self.assertRaisesRegex(
                    support.ConsumerTraceAmbiguous,
                    reason,
                ):
                    probe._limited_zero_watermark(limited, consumer_zero)

    def test_limited_watermark_rejects_late_nonzero_after_first_zero(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        limited = (
            LimitedObservation(20, command(4_000_000_000, 0.01), 8),
            LimitedObservation(30, command(4_345_000_000, 0.0), 9),
            LimitedObservation(40, command(4_545_000_000, 0.01), 10),
        )

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'non-zero controller output after first zero',
        ):
            probe._limited_zero_watermark(
                limited,
                {
                    'zero_sim_ns': 4_345_000_000,
                    'zero_receipt_ns': 30,
                },
            )

    def test_limited_watermark_rejects_out_of_order_controller_stamp(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        limited = (
            LimitedObservation(20, command(4_000_000_000, 0.01), 8),
            LimitedObservation(30, command(4_345_000_000, 0.0), 9),
            LimitedObservation(40, command(4_344_000_000, 0.0), 10),
        )

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'out-of-order topic header stamp',
        ):
            probe._limited_zero_watermark(
                limited,
                {
                    'zero_sim_ns': 4_345_000_000,
                    'zero_receipt_ns': 30,
                },
            )

    def test_limited_watermark_rejects_publication_sequence_gap(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'limited publication sequence gap',
        ) as context:
            probe._limited_zero_watermark(
                (
                    LimitedObservation(20, command(4_000_000_000, 0.01), 8),
                    LimitedObservation(30, command(4_345_000_000, 0.0), 9),
                    LimitedObservation(40, command(4_545_000_000, 0.0), 12),
                ),
                {
                    'zero_sim_ns': 4_345_000_000,
                    'zero_receipt_ns': 30,
                },
            )

        self.assertEqual(context.exception.evidence['expected_sequence'], 10)
        self.assertEqual(
            context.exception.evidence['missing_sequence_count'], 2
        )

if __name__ == '__main__':
    unittest.main()
