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
from copy import deepcopy
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


class FinalObservation:
    """Test-only final sample carrying Jazzy callback metadata."""

    def __init__(
        self,
        receipt_ns,
        message,
        publication_sequence_number,
        *,
        source_timestamp_ns=1_000,
        received_timestamp_ns=2_000,
        reception_sequence_number=None,
        final_subscription_identity='final-subscription',
    ):
        self.receipt_ns = receipt_ns
        self.message = message
        self.source_timestamp_ns = source_timestamp_ns
        self.received_timestamp_ns = received_timestamp_ns
        self.publication_sequence_number = publication_sequence_number
        self.reception_sequence_number = reception_sequence_number
        self.final_subscription_identity = final_subscription_identity

    def __iter__(self):
        return iter((self.receipt_ns, self.message))

    def __getitem__(self, index):
        return (self.receipt_ns, self.message)[index]


def jazzy_message_info(
    publication_sequence_number,
    *,
    source_timestamp=1_000,
    received_timestamp=2_000,
    reception_sequence_number=None,
):
    """The real Jazzy callback mapping observed through rclpy 7.1.11."""
    return {
        'source_timestamp': source_timestamp,
        'received_timestamp': received_timestamp,
        'publication_sequence_number': publication_sequence_number,
        'reception_sequence_number': reception_sequence_number,
    }


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


def writer_retirement_certificate(*, endpoint_gid='gid-gate'):
    """Build the exact-death evidence required before graph stale handling."""
    return {
        'identity': {
            'pid': 120,
            'starttime_ticks': 240,
            'executable': '/opt/voice_nav/lib/motion_gate_node',
            'cmdline': ['motion_gate_node', '--ros-args'],
        },
        'signal': {
            'name': 'SIGKILL',
            'call_start_monotonic_ns': 63_374_470_300_000,
            'ack_monotonic_ns': 63_374_470_335_330,
        },
        'pidfd_exit': {
            'ready_monotonic_ns': 63_374_470_400_000,
        },
        'launch_process_exited': {
            'action_matches_captured': True,
            'returncode': -9,
            'observed_monotonic_ns': 63_374_470_320_000,
        },
        'final_endpoint_gid': endpoint_gid,
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

    def test_accepts_same_gate_drain_delivery_after_motion_proof_anchor(self):
        """The d8c44e0 late sample is the final consumer source, not ambiguity."""
        motion_proof_anchor = FinalObservation(
            32_206_293_538_241,
            command(1_636_000_000, 0.02),
            8,
        )
        drain_delivery = FinalObservation(
            32_206_308_127_382,
            command(1_651_000_000, 0.02),
            9,
            source_timestamp_ns=1_001,
            received_timestamp_ns=2_001,
        )
        consumer_source_anchor, source_finalization = (
            support.finalize_consumer_source_anchor(
                (motion_proof_anchor, drain_delivery),
                motion_proof_anchor=motion_proof_anchor,
                final_endpoint_fence={
                    'endpoint_gid': 'gid-gate',
                    'final_subscription_identity': 'final-subscription',
                    'final_receipt_fence_ns': 32_206_293_538_241,
                    'final_header_stamp_ns': 1_636_000_000,
                    'final_publication_sequence_number': 8,
                    'final_source_timestamp_ns': 1_000,
                    'final_received_timestamp_ns': 2_000,
                    'final_reception_sequence_number': None,
                },
            )
        )

        result = support.consumer_timeout_result(
            (motion_proof_anchor, drain_delivery),
            (
                LimitedObservation(
                    32_206_308_127_400,
                    command(1_660_000_000, 0.02),
                    1,
                ),
                LimitedObservation(
                    32_206_308_127_500,
                    command(2_010_000_000, 0.0),
                    2,
                ),
            ),
            source_anchor=consumer_source_anchor,
            zero_sim_ns=2_010_000_000,
            zero_receipt_ns=32_206_308_127_500,
            source_finalization=source_finalization,
        )

        self.assertEqual(
            result['authoritative_source_stamp_ns'], 1_651_000_000
        )
        self.assertEqual(
            result['delta_ns'], 359_000_000)

    def test_final_source_finalization_rejects_invalid_subscription_trace(self):
        motion_proof_anchor = FinalObservation(
            10, command(1_636_000_000, 0.02), 8
        )
        final_endpoint_fence = {
            'endpoint_gid': 'gid-gate',
            'final_subscription_identity': 'final-subscription',
            'final_receipt_fence_ns': 10,
            'final_header_stamp_ns': 1_636_000_000,
            'final_publication_sequence_number': 8,
            'final_source_timestamp_ns': 1_000,
            'final_received_timestamp_ns': 2_000,
            'final_reception_sequence_number': None,
        }
        cases = (
            (
                (
                    motion_proof_anchor,
                    FinalObservation(
                        20,
                        command(1_651_000_000, 0.02),
                        9,
                        final_subscription_identity='foreign-subscription',
                    ),
                ),
                'final subscription identity changed during endpoint continuity',
            ),
            (
                (
                    motion_proof_anchor,
                    FinalObservation(20, command(1_652_000_000, 0.0), 9),
                ),
                'finalized consumer source anchor was zero',
            ),
            (
                (
                    motion_proof_anchor,
                    FinalObservation(20, command(0, 0.02), 9),
                ),
                'non-positive topic header stamp in relevant trace phase',
            ),
            (
                (
                    motion_proof_anchor,
                    FinalObservation(
                        20, command(1_635_000_000, 0.02), 9
                    ),
                ),
                'out-of-order topic header stamp',
            ),
            (
                (
                    motion_proof_anchor,
                    FinalObservation(20, command(1_651_000_000, 0.02), 8),
                ),
                'duplicate final publication sequence',
            ),
            (
                (
                    motion_proof_anchor,
                    FinalObservation(20, command(1_651_000_000, 0.02), 7),
                ),
                'final publication sequence regressed',
            ),
            (
                (
                    motion_proof_anchor,
                    FinalObservation(20, command(1_651_000_000, 0.02), 10),
                ),
                'final publication sequence gap',
            ),
        )

        for final_samples, reason in cases:
            with self.subTest(reason=reason):
                with self.assertRaisesRegex(
                    support.ConsumerTraceAmbiguous, reason
                ):
                    support.finalize_consumer_source_anchor(
                        final_samples,
                        motion_proof_anchor=motion_proof_anchor,
                        final_endpoint_fence=final_endpoint_fence,
                    )

    def test_final_source_requires_controller_match_after_finalized_source(self):
        motion_proof_anchor = FinalObservation(
            10, command(1_636_000_000, 0.02), 8
        )
        drain_delivery = FinalObservation(
            20,
            command(1_651_000_000, 0.02),
            9,
            source_timestamp_ns=1_001,
            received_timestamp_ns=2_001,
        )
        consumer_source_anchor, source_finalization = (
            support.finalize_consumer_source_anchor(
                (motion_proof_anchor, drain_delivery),
                motion_proof_anchor=motion_proof_anchor,
                final_endpoint_fence={
                    'endpoint_gid': 'gid-gate',
                    'final_subscription_identity': 'final-subscription',
                    'final_receipt_fence_ns': 10,
                    'final_header_stamp_ns': 1_636_000_000,
                    'final_publication_sequence_number': 8,
                    'final_source_timestamp_ns': 1_000,
                    'final_received_timestamp_ns': 2_000,
                    'final_reception_sequence_number': None,
                },
            )
        )

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'missing non-zero controller observation before first zero',
        ):
            support.consumer_timeout_result(
                (motion_proof_anchor, drain_delivery),
                (
                    LimitedObservation(15, command(1_640_000_000, 0.02), 1),
                    LimitedObservation(30, command(2_010_000_000, 0.0), 2),
                ),
                source_anchor=consumer_source_anchor,
                zero_sim_ns=2_010_000_000,
                zero_receipt_ns=30,
                source_finalization=source_finalization,
            )

    def test_final_observation_callback_records_real_jazzy_metadata_without_gid(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe.final_commands = deque(maxlen=20)

        probe._append_final(
            command(1_651_000_000, 0.02),
            jazzy_message_info(
                8,
                source_timestamp=1_700,
                received_timestamp=1_800,
                reception_sequence_number=None,
            ),
        )

        (first,) = probe.final_commands
        self.assertEqual(first.publication_sequence_number, 8)
        self.assertEqual(first.source_timestamp_ns, 1_700)
        self.assertEqual(first.received_timestamp_ns, 1_800)
        self.assertIsNone(first.reception_sequence_number)
        self.assertEqual(first.header_stamp_ns, 1_651_000_000)
        self.assertEqual(first.twist_signature[0], 0.02)
        self.assertFalse(hasattr(first, 'publisher_gid'))

    def test_excludes_pre_anchor_same_signature_source_before_unique_anchor(self):
        source_anchor = (30, command(4_000_000_000, 0.02))

        result = support.consumer_timeout_result(
            (
                (1, command(0, 0.0)),
                (2, command(0, 0.0)),
                (3, command(3_990_000_000, 0.02)),
                source_anchor,
            ),
            (
                (4, command(0, 0.0)),
                (5, command(0, 0.0)),
                (15, command(3_995_000_000, 0.01)),
                (35, command(4_010_000_000, 0.02)),
                (40, command(4_351_000_000, 0.0)),
            ),
            source_anchor=source_anchor,
            zero_sim_ns=4_351_000_000,
            zero_receipt_ns=40,
        )

        association = result['association']
        self.assertEqual(association['source_header_stamp_ns'], 4_000_000_000)
        self.assertEqual(association['controller_zero_update_stamp_ns'], 4_351_000_000)
        self.assertEqual(result['delta_ns'], 351_000_000)
        self.assertEqual(
            association['phase_isolation']['final']['excluded_prefix_count'],
            3,
        )
        self.assertEqual(
            association['phase_isolation']['limited']['excluded_prefix_count'],
            3,
        )

    def test_rejects_duplicate_anchor_identity_after_phase_isolation(self):
        source_anchor = (30, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'frozen source anchor was not unique in final trace',
        ) as context:
            support.consumer_timeout_result(
                (
                    (1, command(0, 0.0)),
                    source_anchor,
                    source_anchor,
                ),
                (
                    (35, command(4_010_000_000, 0.02)),
                    (40, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=40,
            )

        self.assertEqual(
            context.exception.evidence['matching_source_count'], 2
        )

    def test_rejects_missing_anchor_identity_after_startup_prefix(self):
        source_anchor = (30, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'frozen source anchor was missing from final trace',
        ):
            support.consumer_timeout_result(
                (
                    (1, command(0, 0.0)),
                    (2, command(0, 0.0)),
                    (3, command(3_990_000_000, 0.01)),
                ),
                (
                    (35, command(4_010_000_000, 0.02)),
                    (40, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=40,
            )

    def test_rejects_zero_stamp_delivered_after_anchor_receipt(self):
        source_anchor = (30, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'late source delivery after frozen anchor',
        ):
            support.consumer_timeout_result(
                (
                    (1, command(0, 0.0)),
                    source_anchor,
                    (40, command(0, 0.0)),
                ),
                (
                    (35, command(4_010_000_000, 0.02)),
                    (45, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=45,
            )

    def test_rejects_controller_receipt_stamp_phase_conflict(self):
        source_anchor = (30, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'controller header stamp precedes source anchor phase',
        ) as context:
            support.consumer_timeout_result(
                (
                    (1, command(0, 0.0)),
                    source_anchor,
                ),
                (
                    (4, command(0, 0.0)),
                    (35, command(3_995_000_000, 0.01)),
                    (40, command(4_010_000_000, 0.02)),
                    (45, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=45,
            )

        self.assertEqual(
            context.exception.evidence['controller_receipt_ns'], 35
        )

    def test_rejects_zero_stamp_controller_sample_after_anchor_receipt(self):
        source_anchor = (30, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'non-positive topic header stamp in relevant trace phase',
        ) as context:
            support.consumer_timeout_result(
                (
                    (1, command(0, 0.0)),
                    source_anchor,
                ),
                (
                    (4, command(0, 0.0)),
                    (35, command(0, 0.0)),
                    (40, command(4_010_000_000, 0.02)),
                    (45, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=45,
            )

        self.assertEqual(context.exception.evidence['observed_receipt_ns'], 35)
        self.assertEqual(
            context.exception.evidence['phase_isolation'][
                'excluded_prefix_count'
            ],
            1,
        )

    def test_rejects_first_relevant_zero_with_early_receipt(self):
        source_anchor = (30, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'first relevant controller zero conflicts with source anchor',
        ) as context:
            support.consumer_timeout_result(
                (source_anchor,),
                (
                    (25, command(4_005_000_000, 0.0)),
                    (35, command(4_010_000_000, 0.02)),
                    (40, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=40,
            )

        self.assertEqual(
            context.exception.evidence['first_relevant_zero_receipt_ns'], 25
        )
        self.assertEqual(
            context.exception.evidence['first_relevant_zero_stamp_ns'],
            4_005_000_000,
        )
        self.assertTrue(context.exception.evidence['receipt_conflict'])
        self.assertFalse(context.exception.evidence['stamp_conflict'])

    def test_watermark_rejects_first_relevant_zero_with_anchor_stamp(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        source_anchor = (30, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'first relevant controller zero conflicts with source anchor',
        ) as context:
            probe._limited_zero_watermark(
                (
                    LimitedObservation(31, command(4_000_000_000, 0.0), 4),
                    LimitedObservation(35, command(4_010_000_000, 0.02), 5),
                    LimitedObservation(40, command(4_351_000_000, 0.0), 6),
                    LimitedObservation(50, command(4_551_000_000, 0.0), 7),
                ),
                {
                    'zero_sim_ns': 4_351_000_000,
                    'zero_receipt_ns': 40,
                },
                source_anchor=source_anchor,
            )

        self.assertEqual(
            context.exception.evidence['first_relevant_zero_receipt_ns'], 31
        )
        self.assertEqual(
            context.exception.evidence['first_relevant_zero_stamp_ns'],
            4_000_000_000,
        )
        self.assertFalse(context.exception.evidence['receipt_conflict'])
        self.assertTrue(context.exception.evidence['stamp_conflict'])

    def test_rejects_relevant_controller_duplicate_stamp_after_prefix(self):
        source_anchor = (30, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'out-of-order topic header stamp',
        ) as context:
            support.consumer_timeout_result(
                (
                    (1, command(0, 0.0)),
                    source_anchor,
                ),
                (
                    (4, command(0, 0.0)),
                    (5, command(0, 0.0)),
                    (15, command(3_995_000_000, 0.01)),
                    (35, command(4_010_000_000, 0.02)),
                    (40, command(4_351_000_000, 0.0)),
                    (45, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=40,
            )

        self.assertEqual(
            context.exception.evidence['phase_isolation'][
                'excluded_prefix_count'
            ],
            3,
        )

    def test_phase_does_not_skip_first_unproven_final_sample(self):
        source_anchor = (30, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'out-of-order observer receipt',
        ) as context:
            support.consumer_timeout_result(
                (
                    (1, command(0, 0.0)),
                    (35, command(3_995_000_000, 0.01)),
                    source_anchor,
                ),
                (
                    (40, command(4_010_000_000, 0.02)),
                    (45, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=45,
            )

        phase = context.exception.evidence['phase_isolation']
        self.assertEqual(phase['excluded_prefix_count'], 1)
        self.assertEqual(phase['relevant_trace_start']['receipt_ns'], 35)

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

    def test_rejects_same_signature_source_after_anchor_receipt(self):
        source_anchor = (20, command(4_000_000_000, 0.02))

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'late source delivery after frozen anchor',
        ):
            support.consumer_timeout_result(
                (
                    source_anchor,
                    (40, command(4_020_000_000, 0.02)),
                ),
                (
                    (25, command(4_010_000_000, 0.02)),
                    (30, command(4_351_000_000, 0.0)),
                ),
                source_anchor=source_anchor,
                zero_sim_ns=4_351_000_000,
                zero_receipt_ns=30,
            )

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
        probe._final_endpoint_snapshot_provider = lambda: ()
        source_anchor = FinalObservation(
            10, command(4_000_000_000, 0.01), 8
        )
        probe.final_commands = deque((source_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                LimitedObservation(60, command(4_110_000_000, 0.03), 8),
                LimitedObservation(70, command(4_451_000_000, 0.0), 9),
                LimitedObservation(80, command(4_651_000_000, 0.0), 10),
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
                    if self.sleep_count == 0:
                        probe.final_commands.append(
                            FinalObservation(
                                50,
                                command(4_100_000_000, 0.03),
                                9,
                                source_timestamp_ns=1_001,
                                received_timestamp_ns=2_001,
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
                fenced_consumer_zero(4_451_000_000, 70),
                final_endpoint_fence={
                    'endpoint_gid': 'gid-gate',
                    'final_subscription_identity': 'final-subscription',
                    'final_receipt_fence_ns': 10,
                    'final_header_stamp_ns': 4_000_000_000,
                    'final_publication_sequence_number': 8,
                    'final_source_timestamp_ns': 1_000,
                    'final_received_timestamp_ns': 2_000,
                    'final_reception_sequence_number': None,
                },
                writer_retirement_certificate=writer_retirement_certificate(),
                timeout=1.0,
            )
        self.assertEqual(result['last_nonzero_sim_ns'], 4_100_000_000)
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

    def test_quiescence_fence_isolates_startup_prefix_for_zero_watermark(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        source_anchor = (30, command(4_000_000_000, 0.02))
        probe.final_commands = deque(
            (
                (1, command(0, 0.0)),
                (2, command(0, 0.0)),
                (3, command(3_990_000_000, 0.01)),
                source_anchor,
            ),
            maxlen=20,
        )
        probe.limited_commands = deque(
            (
                LimitedObservation(4, command(0, 0.0), 1),
                LimitedObservation(5, command(0, 0.0), 2),
                LimitedObservation(15, command(3_995_000_000, 0.01), 3),
                LimitedObservation(35, command(4_010_000_000, 0.02), 4),
                LimitedObservation(40, command(4_351_000_000, 0.0), 5),
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
                            40 + self.now_ns,
                            command(4_351_000_000 + self.now_ns, 0.0),
                            5 + self.now_ns // 100_000_000,
                        )
                    )
                    probe.clock_samples.append(
                        (self.now_ns, 4_500_000_000 + self.now_ns)
                    )

        with mock.patch.object(support, 'time', FakeTime()):
            result = probe.wait_confirm_consumer_timeout(
                source_anchor,
                fenced_consumer_zero(4_351_000_000, 40),
                timeout=1.0,
            )

        self.assertEqual(result['delta_ns'], 351_000_000)
        self.assertEqual(
            result['association']['phase_isolation']['limited'][
                'excluded_prefix_count'
            ],
            3,
        )
        self.assertEqual(
            result['limited_zero_watermark']['watermark_stamp_ns'],
            4_551_000_000,
        )

    def test_quiescence_fence_reports_limited_watermark_timeout(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        source_anchor = (30, command(4_000_000_000, 0.02))
        probe.final_commands = deque(
            (
                (1, command(0, 0.0)),
                (2, command(0, 0.0)),
                (3, command(3_990_000_000, 0.01)),
                source_anchor,
            ),
            maxlen=20,
        )
        probe.limited_commands = deque(
            (
                LimitedObservation(4, command(0, 0.0), 1),
                LimitedObservation(5, command(0, 0.0), 2),
                LimitedObservation(15, command(3_995_000_000, 0.01), 3),
                LimitedObservation(35, command(4_010_000_000, 0.02), 4),
                LimitedObservation(40, command(4_351_000_000, 0.0), 5),
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
                    fenced_consumer_zero(4_351_000_000, 40),
                    timeout=0.5,
                )

        self.assertEqual(context.exception.evidence['endpoint_gid'], 'gid-controller')
        self.assertEqual(
            context.exception.evidence['first_zero_stamp_ns'], 4_351_000_000
        )
        self.assertEqual(
            context.exception.evidence['limited_phase_isolation'][
                'excluded_prefix_count'
            ],
            3,
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
        probe.node = SimpleNamespace(
            get_publishers_info_by_topic=lambda _topic: (
                SimpleNamespace(
                    node_name=support.GATE_NODE,
                    node_namespace='/',
                    endpoint_gid=b'\x01',
                ),
            )
        )
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
            (FinalObservation(901, command(4_000_000_000, 0.01), 8),),
            maxlen=20,
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
        self.assertEqual(boundary['final_endpoint_fence'], {
            'endpoint_gid': '01',
            'final_subscription_identity': 'final-subscription',
            'final_receipt_fence_ns': 901,
            'final_header_stamp_ns': 4_000_000_000,
            'final_publication_sequence_number': 8,
            'final_source_timestamp_ns': 1_000,
            'final_received_timestamp_ns': 2_000,
            'final_reception_sequence_number': None,
        })
        self.assertEqual(boundary['limited'][0], 902)
        self.assertEqual(boundary['final'].publication_sequence_number, 8)

        consumer_source_anchor, source_finalization = (
            support.finalize_consumer_source_anchor(
                (
                    boundary['final'],
                    FinalObservation(
                        904,
                        command(4_015_000_000, 0.01),
                        9,
                        source_timestamp_ns=1_001,
                        received_timestamp_ns=2_001,
                    ),
                ),
                motion_proof_anchor=boundary['final'],
                final_endpoint_fence=boundary['final_endpoint_fence'],
            )
        )
        self.assertEqual(support._stamp_ns(consumer_source_anchor[1]), 4_015_000_000)
        self.assertEqual(
            source_finalization['consumer_source_anchor'][
                'final_subscription_identity'
            ],
            'final-subscription',
        )

    def test_final_endpoint_disappearance_rejects_remaining_or_replacement_owner(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        final_fence = {
            'endpoint_gid': 'gid-gate',
            'final_receipt_fence_ns': 902,
        }

        for endpoint_gids, reason in (
            (('gid-foreign',), 'foreign final endpoint appeared'),
            (('gid-gate', 'gid-duplicate'), 'duplicate final endpoints remained'),
        ):
            with self.subTest(endpoint_gids=endpoint_gids):
                probe._final_endpoint_snapshot_provider = lambda: endpoint_gids
                with self.assertRaisesRegex(
                    support.ConsumerTraceAmbiguous, reason
                ):
                    probe._require_final_endpoint_disappearance(
                        final_fence,
                        checkpoint='unit',
                        writer_retirement_certificate=(
                            writer_retirement_certificate()
                        ),
                    )

        probe._final_endpoint_snapshot_provider = lambda: ()
        self.assertEqual(
            probe._require_final_endpoint_disappearance(
                final_fence,
                checkpoint='unit',
                writer_retirement_certificate=writer_retirement_certificate(),
            )['remaining_endpoint_gids'],
            [],
        )

    def test_00bee9e_replay_accepts_only_stale_exact_retired_gid_after_certificate(self):
        """Replay #110's exact stale-GID trace without Gazebo.

        The final receipt stopped at 63374475559097 ns, while Linux pidfd and
        launch had already proved the exact Gate exited.  Jazzy graph discovery
        still returned only the captured GID.  The old protocol waited for the
        graph to become empty and timed out; this is the intended RED loop.
        """
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe._final_endpoint_snapshot_provider = lambda: (
            '010f0c266499c44d0000000000001503',
        )
        certificate = writer_retirement_certificate(
            endpoint_gid='010f0c266499c44d0000000000001503'
        )

        result = probe.wait_final_endpoint_disappearance(
            {
                'endpoint_gid': '010f0c266499c44d0000000000001503',
                'final_receipt_fence_ns': 63_374_475_559_097,
            },
            writer_retirement_certificate=certificate,
        )

        self.assertEqual(result['graph_state'], 'stale_exact_retired_gid')
        self.assertEqual(
            result['remaining_endpoint_gids'],
            ['010f0c266499c44d0000000000001503'],
        )

    def test_retirement_certificate_fails_closed_for_missing_or_mismatched_axis(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe._final_endpoint_snapshot_provider = lambda: ('gid-gate',)
        final_fence = {
            'endpoint_gid': 'gid-gate',
            'final_receipt_fence_ns': 63_374_475_559_097,
        }
        cases = (
            (
                'signal',
                lambda certificate: certificate.pop('signal'),
                'signal evidence was incomplete',
            ),
            (
                'signal-call-start',
                lambda certificate: certificate['signal'].pop(
                    'call_start_monotonic_ns'
                ),
                'signal evidence was incomplete',
            ),
            (
                'pidfd-exit',
                lambda certificate: certificate.pop('pidfd_exit'),
                'pidfd exit evidence was incomplete',
            ),
            (
                'launch-action',
                lambda certificate: certificate['launch_process_exited'].update(
                    action_matches_captured=False
                ),
                'launch exit evidence was incomplete',
            ),
            (
                'launch-before-signal-call',
                lambda certificate: certificate['launch_process_exited'].update(
                    observed_monotonic_ns=(
                        certificate['signal']['call_start_monotonic_ns'] - 1
                    )
                ),
                'launch exit evidence was incomplete',
            ),
            (
                'launch-returncode',
                lambda certificate: certificate['launch_process_exited'].update(
                    returncode=0
                ),
                'launch exit evidence was incomplete',
            ),
            (
                'identity',
                lambda certificate: certificate['identity'].pop('starttime_ticks'),
                'identity was incomplete',
            ),
            (
                'endpoint',
                lambda certificate: certificate.update(
                    final_endpoint_gid='gid-replacement'
                ),
                'endpoint did not match signal boundary',
            ),
        )
        for name, mutate, reason in cases:
            with self.subTest(name=name):
                certificate = deepcopy(writer_retirement_certificate())
                mutate(certificate)
                with self.assertRaisesRegex(
                    support.ConsumerTraceAmbiguous,
                    reason,
                ):
                    probe.wait_final_endpoint_disappearance(
                        final_fence,
                        writer_retirement_certificate=certificate,
                    )

    def test_final_endpoint_disappearance_times_out_when_exact_gate_remains(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe._final_endpoint_snapshot_provider = lambda: ('gid-gate',)

        class FakeTime:

            now_ns = 0

            @classmethod
            def monotonic(cls):
                return cls.now_ns / 1_000_000_000

            @classmethod
            def monotonic_ns(cls):
                return cls.now_ns

            @classmethod
            def sleep(cls, _seconds):
                cls.now_ns += 100_000_000

        with mock.patch.object(support, 'time', FakeTime):
            with self.assertRaisesRegex(
                support.ConsumerTraceAmbiguous,
                'writer retirement certificate was unavailable',
            ):
                probe.wait_final_endpoint_disappearance(
                    {
                        'endpoint_gid': 'gid-gate',
                        'final_receipt_fence_ns': 902,
                    },
                    writer_retirement_certificate=None,
                    timeout=0.2,
                )

    def test_confirmation_freezes_same_gate_drain_after_endpoint_disappears(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        probe._final_endpoint_snapshot_provider = lambda: ('gid-gate',)
        motion_proof_anchor = FinalObservation(
            10, command(1_636_000_000, 0.02), 8
        )
        probe.final_commands = deque(
            (
                motion_proof_anchor,
                FinalObservation(20, command(1_651_000_000, 0.02), 9),
            ),
            maxlen=20,
        )
        probe.limited_commands = deque(
            (
                LimitedObservation(30, command(1_660_000_000, 0.02), 1),
                LimitedObservation(40, command(2_010_000_000, 0.0), 2),
                LimitedObservation(50, command(2_210_000_000, 0.0), 3),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 2_010_000_000),), maxlen=20)

        class FakeTime:

            now_ns = 0

            @classmethod
            def monotonic(cls):
                return cls.now_ns / 1_000_000_000

            @classmethod
            def monotonic_ns(cls):
                return cls.now_ns

            @classmethod
            def sleep(cls, _seconds):
                cls.now_ns += 100_000_000
                with probe.lock:
                    probe.clock_samples.append(
                        (cls.now_ns, 2_010_000_000 + cls.now_ns)
                    )

        with mock.patch.object(support, 'time', FakeTime):
            result = probe.wait_confirm_consumer_timeout(
                motion_proof_anchor,
                fenced_consumer_zero(2_010_000_000, 40),
                final_endpoint_fence={
                    'endpoint_gid': 'gid-gate',
                    'final_subscription_identity': 'final-subscription',
                    'final_receipt_fence_ns': 10,
                    'final_header_stamp_ns': 1_636_000_000,
                    'final_publication_sequence_number': 8,
                    'final_source_timestamp_ns': 1_000,
                    'final_received_timestamp_ns': 2_000,
                    'final_reception_sequence_number': None,
                },
                writer_retirement_certificate=writer_retirement_certificate(),
                timeout=1.0,
            )

        self.assertEqual(result['last_nonzero_sim_ns'], 1_651_000_000)
        self.assertEqual(
            result['source_finalization']['consumer_source_anchor'][
                'final_subscription_identity'
            ],
            'final-subscription',
        )
        self.assertEqual(
            result['final_endpoint_disappearance']['remaining_endpoint_gids'],
            ['gid-gate'],
        )
        self.assertEqual(
            result['final_endpoint_disappearance']['graph_state'],
            'stale_exact_retired_gid',
        )

    def test_confirmation_rejects_callback_entering_after_last_clean_check(self):
        """No passed checkpoint may hide a frozen old-epoch callback."""
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        probe._final_endpoint_snapshot_provider = lambda: ()
        motion_proof_anchor = FinalObservation(
            10, command(1_636_000_000, 0.02), 8
        )
        probe.final_commands = deque(
            (
                motion_proof_anchor,
                FinalObservation(20, command(1_651_000_000, 0.02), 9),
            ),
            maxlen=20,
        )
        probe.limited_commands = deque(
            (
                LimitedObservation(30, command(1_660_000_000, 0.02), 1),
                LimitedObservation(40, command(2_010_000_000, 0.0), 2),
                LimitedObservation(50, command(2_210_000_000, 0.0), 3),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 2_010_000_000),), maxlen=20)

        class FakeTime:

            now_ns = 0

            @classmethod
            def monotonic(cls):
                return cls.now_ns / 1_000_000_000

            @classmethod
            def monotonic_ns(cls):
                return cls.now_ns

            @classmethod
            def sleep(cls, _seconds):
                cls.now_ns += 100_000_000
                with probe.lock:
                    probe.clock_samples.append(
                        (cls.now_ns, 2_010_000_000 + cls.now_ns)
                    )

        original_clean_check = probe._assert_final_callback_freeze_clean
        clean_check_count = 0
        callback_entered = threading.Event()
        callback_threads = []

        def inject_old_callback_after_last_clean_check():
            nonlocal clean_check_count
            original_clean_check()
            clean_check_count += 1
            if clean_check_count != 2:
                return

            def old_callback():
                callback_entered.set()
                probe._append_final(
                    command(1_652_000_000, 0.02),
                    jazzy_message_info(10),
                )

            callback_thread = threading.Thread(
                target=old_callback,
                name='post-clean-old-final-callback',
            )
            callback_threads.append(callback_thread)
            callback_thread.start()
            self.assertTrue(callback_entered.wait(timeout=1.0))
            callback_thread.join(timeout=1.0)
            self.assertFalse(callback_thread.is_alive())

        with mock.patch.object(support, 'time', FakeTime):
            with (
                mock.patch.object(
                    probe,
                    '_assert_final_callback_freeze_clean',
                    side_effect=inject_old_callback_after_last_clean_check,
                ),
                self.assertRaisesRegex(
                    support.ConsumerTraceAmbiguous,
                    'final callback entered after trace freeze',
                ),
            ):
                probe.wait_confirm_consumer_timeout(
                    motion_proof_anchor,
                    fenced_consumer_zero(2_010_000_000, 40),
                    final_endpoint_fence={
                        'endpoint_gid': 'gid-gate',
                        'final_subscription_identity': 'final-subscription',
                        'final_receipt_fence_ns': 10,
                        'final_header_stamp_ns': 1_636_000_000,
                        'final_publication_sequence_number': 8,
                        'final_source_timestamp_ns': 1_000,
                        'final_received_timestamp_ns': 2_000,
                        'final_reception_sequence_number': None,
                    },
                    writer_retirement_certificate=(
                        writer_retirement_certificate()
                    ),
                    timeout=1.0,
                )

        self.assertEqual(clean_check_count, 2)
        self.assertEqual(len(callback_threads), 1)
        self.assertIsNone(probe._finalized_final_trace)

    def test_replacement_rejects_old_callback_after_finalization_returns(self):
        """A frozen old callback blocks replacement before subscription swap."""
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        probe._final_endpoint_snapshot_provider = lambda: ()
        motion_proof_anchor = FinalObservation(
            10, command(1_636_000_000, 0.02), 8
        )
        probe.final_commands = deque(
            (
                motion_proof_anchor,
                FinalObservation(20, command(1_651_000_000, 0.02), 9),
            ),
            maxlen=20,
        )
        probe.limited_commands = deque(
            (
                LimitedObservation(30, command(1_660_000_000, 0.02), 1),
                LimitedObservation(40, command(2_010_000_000, 0.0), 2),
                LimitedObservation(50, command(2_210_000_000, 0.0), 3),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 2_010_000_000),), maxlen=20)

        class FakeTime:

            now_ns = 0

            @classmethod
            def monotonic(cls):
                return cls.now_ns / 1_000_000_000

            @classmethod
            def monotonic_ns(cls):
                return cls.now_ns

            @classmethod
            def sleep(cls, _seconds):
                cls.now_ns += 100_000_000
                with probe.lock:
                    probe.clock_samples.append(
                        (cls.now_ns, 2_010_000_000 + cls.now_ns)
                    )

        with mock.patch.object(support, 'time', FakeTime):
            probe.wait_confirm_consumer_timeout(
                motion_proof_anchor,
                fenced_consumer_zero(2_010_000_000, 40),
                final_endpoint_fence={
                    'endpoint_gid': 'gid-gate',
                    'final_subscription_identity': 'final-subscription',
                    'final_receipt_fence_ns': 10,
                    'final_header_stamp_ns': 1_636_000_000,
                    'final_publication_sequence_number': 8,
                    'final_source_timestamp_ns': 1_000,
                    'final_received_timestamp_ns': 2_000,
                    'final_reception_sequence_number': None,
                },
                writer_retirement_certificate=writer_retirement_certificate(),
                timeout=1.0,
            )
            old_epoch = probe._final_observation_epoch
            old_identity = probe._final_subscription_identity
            callback_entered = threading.Event()

            def old_callback():
                callback_entered.set()
                probe._append_final(
                    command(1_652_000_000, 0.02),
                    jazzy_message_info(10),
                    observation_epoch=old_epoch,
                    final_subscription_identity=old_identity,
                )

            callback_thread = threading.Thread(
                target=old_callback,
                name='after-finalization-old-final-callback',
            )
            callback_thread.start()
            self.assertTrue(callback_entered.wait(timeout=1.0))
            callback_thread.join(timeout=1.0)
            self.assertFalse(callback_thread.is_alive())

            old_subscription = object()

            class FakeNode:

                def __init__(self):
                    self.destroyed = []

                def destroy_subscription(self, subscription):
                    self.destroyed.append(subscription)

                def create_subscription(self, *_args, **_kwargs):
                    return object()

            node = FakeNode()
            probe.node = node
            probe.final_subscription = old_subscription
            probe.subscriptions = [old_subscription]
            probe.safety_observation_group = object()

            with self.assertRaisesRegex(
                support.ConsumerTraceAmbiguous,
                'old callback entered after finalized trace freeze',
            ):
                probe.begin_replacement_final_observation_epoch()

        self.assertEqual(node.destroyed, [])
        self.assertTrue(probe._final_trace_frozen)
        self.assertEqual(probe._final_observation_epoch, old_epoch)
        self.assertEqual(probe._final_subscription_identity, old_identity)
        self.assertEqual(probe._final_post_freeze_ingress_count, 1)

    def test_replacement_subscription_failure_remains_frozen(self):
        """A failed observer swap cannot advance or unfreeze an old epoch."""
        for failure_stage in ('destroy', 'create'):
            with self.subTest(failure_stage=failure_stage):
                probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
                probe.lock = threading.Lock()
                probe._final_callback_condition = threading.Condition()
                probe._final_callbacks_in_flight = 0
                probe._final_trace_frozen = True
                probe._final_post_freeze_ingress_count = 0
                probe._final_stale_epoch_ingress_count = 0
                probe._final_observation_epoch = 7
                probe._final_trace_epoch_state = 'finalized'
                probe._final_subscription_identity = 'old-subscription'
                probe._finalized_final_trace = (
                    FinalObservation(10, command(1_636_000_000, 0.02), 8),
                )
                probe._finalized_trace_fence = {
                    'observation_epoch': 7,
                    'final_subscription_identity': 'old-subscription',
                    'trace_length': 1,
                }
                old_subscription = object()
                probe.final_subscription = old_subscription
                probe.subscriptions = [old_subscription]
                probe.safety_observation_group = object()

                class FailingNode:

                    def destroy_subscription(self, _subscription):
                        if failure_stage == 'destroy':
                            raise RuntimeError('destroy failed')
                        return True

                    def create_subscription(self, *_args, **_kwargs):
                        raise RuntimeError('create failed')

                probe.node = FailingNode()

                with self.assertRaisesRegex(
                    support.ConsumerTraceAmbiguous,
                    'replacement final observer transition failed',
                ):
                    probe.begin_replacement_final_observation_epoch()

                self.assertTrue(probe._final_trace_frozen)
                self.assertEqual(probe._final_trace_epoch_state, 'failed')
                self.assertEqual(probe._final_observation_epoch, 7)
                self.assertEqual(
                    probe._final_subscription_identity, 'old-subscription'
                )
                self.assertIs(probe.final_subscription, old_subscription)
                self.assertEqual(probe._final_post_freeze_ingress_count, 0)

    def test_replacement_rejects_false_destroy_without_creating_observer(self):
        """A false rclpy destroy result keeps the old epoch frozen."""
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._final_callback_condition = threading.Condition()
        probe._final_callbacks_in_flight = 0
        probe._final_trace_frozen = True
        probe._final_post_freeze_ingress_count = 0
        probe._final_stale_epoch_ingress_count = 0
        probe._final_observation_epoch = 7
        probe._final_trace_epoch_state = 'finalized'
        probe._final_subscription_identity = 'old-subscription'
        probe._finalized_final_trace = (
            FinalObservation(10, command(1_636_000_000, 0.02), 8),
        )
        probe._finalized_trace_fence = {
            'observation_epoch': 7,
            'final_subscription_identity': 'old-subscription',
            'trace_length': 1,
        }
        old_subscription = object()
        probe.final_subscription = old_subscription
        probe.subscriptions = [old_subscription]
        probe.safety_observation_group = object()

        class FalseDestroyNode:

            def __init__(self):
                self.create_calls = 0

            def destroy_subscription(self, _subscription):
                return False

            def create_subscription(self, *_args, **_kwargs):
                self.create_calls += 1
                return object()

        node = FalseDestroyNode()
        probe.node = node

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'replacement final observer transition failed',
        ):
            probe.begin_replacement_final_observation_epoch()

        self.assertEqual(node.create_calls, 0)
        self.assertTrue(probe._final_trace_frozen)
        self.assertEqual(probe._final_trace_epoch_state, 'failed')
        self.assertEqual(probe._final_observation_epoch, 7)
        self.assertEqual(probe._final_subscription_identity, 'old-subscription')
        self.assertIs(probe.final_subscription, old_subscription)

    def test_finalized_trace_opens_isolated_replacement_epoch_for_recovery(self):
        """A replacement Gate must not inherit either trace or callbacks."""
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        probe._final_endpoint_snapshot_provider = lambda: ()
        motion_proof_anchor = FinalObservation(
            10, command(1_636_000_000, 0.02), 8
        )
        probe.final_commands = deque(
            (
                motion_proof_anchor,
                FinalObservation(20, command(1_651_000_000, 0.02), 9),
            ),
            maxlen=20,
        )
        probe.limited_commands = deque(
            (
                LimitedObservation(30, command(1_660_000_000, 0.02), 1),
                LimitedObservation(40, command(2_010_000_000, 0.0), 2),
                LimitedObservation(50, command(2_210_000_000, 0.0), 3),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 2_010_000_000),), maxlen=20)

        class FakeTime:

            now_ns = 0

            @classmethod
            def monotonic(cls):
                return cls.now_ns / 1_000_000_000

            @classmethod
            def monotonic_ns(cls):
                return cls.now_ns

            @classmethod
            def sleep(cls, _seconds):
                cls.now_ns += 100_000_000
                with probe.lock:
                    probe.clock_samples.append(
                        (cls.now_ns, 2_010_000_000 + cls.now_ns)
                    )

        with mock.patch.object(support, 'time', FakeTime):
            probe.wait_confirm_consumer_timeout(
                motion_proof_anchor,
                fenced_consumer_zero(2_010_000_000, 40),
                final_endpoint_fence={
                    'endpoint_gid': 'gid-gate',
                    'final_subscription_identity': 'final-subscription',
                    'final_receipt_fence_ns': 10,
                    'final_header_stamp_ns': 1_636_000_000,
                    'final_publication_sequence_number': 8,
                    'final_source_timestamp_ns': 1_000,
                    'final_received_timestamp_ns': 2_000,
                    'final_reception_sequence_number': None,
                },
                writer_retirement_certificate=writer_retirement_certificate(),
                timeout=1.0,
            )
            finalized_trace = tuple(probe.final_commands)
            previous_observation_epoch = probe._final_observation_epoch
            previous_subscription_identity = (
                probe._final_subscription_identity
            )
            old_subscription = object()

            class FakeNode:

                def __init__(self):
                    self.destroyed = []
                    self.final_callback = None

                def destroy_subscription(self, subscription):
                    self.destroyed.append(subscription)
                    return True

                def create_subscription(
                    self,
                    _message_type,
                    topic,
                    callback,
                    _qos,
                    *,
                    callback_group,
                ):
                    self.topic = topic
                    self.callback_group = callback_group
                    self.final_callback = callback
                    return object()

            node = FakeNode()
            probe.node = node
            probe.final_subscription = old_subscription
            probe.subscriptions = [old_subscription]
            probe.safety_observation_group = object()
            replacement = probe.begin_replacement_final_observation_epoch()
            self.assertEqual(node.destroyed, [old_subscription])
            self.assertEqual(node.topic, support.FINAL_COMMAND_TOPIC)
            self.assertIs(node.callback_group, probe.safety_observation_group)
            self.assertEqual(
                replacement['observation_epoch'],
                previous_observation_epoch + 1,
            )
            self.assertNotEqual(
                replacement['final_subscription_identity'],
                previous_subscription_identity,
            )
            node.final_callback(
                command(2_300_000_000, 0.0),
                jazzy_message_info(1),
            )
            probe._append_final(
                command(2_301_000_000, 0.02),
                jazzy_message_info(10),
                observation_epoch=previous_observation_epoch,
                final_subscription_identity=previous_subscription_identity,
            )
            node.final_callback(
                command(2_400_000_000, 0.02),
                jazzy_message_info(2),
            )

        self.assertEqual(
            [support._stamp_ns(sample.message) for sample in probe._finalized_final_trace],
            [support._stamp_ns(sample.message) for sample in finalized_trace],
        )
        self.assertEqual(
            replacement['finalized_trace_fence']['trace_length'],
            len(finalized_trace),
        )
        replacement_samples = [
            observation
            for observation in probe.final_commands
            if getattr(observation, 'observation_epoch', None)
            == replacement['observation_epoch']
        ]
        self.assertEqual(
            [support._stamp_ns(sample.message) for sample in replacement_samples],
            [2_300_000_000, 2_400_000_000],
        )
        self.assertEqual(probe._final_stale_epoch_ingress_count, 1)
        self.assertFalse(probe._final_trace_frozen)

    def test_confirmation_fail_closed_when_final_quiescence_clock_stops(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        probe._final_endpoint_snapshot_provider = lambda: ()
        motion_proof_anchor = FinalObservation(
            10, command(1_636_000_000, 0.02), 8
        )
        probe.final_commands = deque((motion_proof_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(1_660_000_000, 0.02), 1),
                LimitedObservation(30, command(2_010_000_000, 0.0), 2),
                LimitedObservation(40, command(2_210_000_000, 0.0), 3),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 2_010_000_000),), maxlen=20)

        class FakeTime:

            now_ns = 0

            @classmethod
            def monotonic(cls):
                return cls.now_ns / 1_000_000_000

            @classmethod
            def monotonic_ns(cls):
                return cls.now_ns

            @classmethod
            def sleep(cls, _seconds):
                cls.now_ns += 100_000_000

        with mock.patch.object(support, 'time', FakeTime):
            with self.assertRaisesRegex(
                support.ConsumerTraceAmbiguous,
                'final command observer did not quiesce with advancing clock',
            ):
                probe.wait_confirm_consumer_timeout(
                    motion_proof_anchor,
                    fenced_consumer_zero(2_010_000_000, 30),
                    final_endpoint_fence={
                        'endpoint_gid': 'gid-gate',
                        'final_subscription_identity': 'final-subscription',
                        'final_receipt_fence_ns': 10,
                        'final_header_stamp_ns': 1_636_000_000,
                        'final_publication_sequence_number': 8,
                        'final_source_timestamp_ns': 1_000,
                        'final_received_timestamp_ns': 2_000,
                        'final_reception_sequence_number': None,
                    },
                    writer_retirement_certificate=(
                        writer_retirement_certificate()
                    ),
                    timeout=0.2,
                )

    def test_confirmation_waits_for_queued_final_callback_before_freezing(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        class CallbackBlockedLock:

            def __init__(self):
                self._lock = threading.Lock()
                self.callback_waiting = threading.Event()
                self.release_callback = threading.Event()

            def __enter__(self):
                if threading.current_thread().name == 'queued-final-callback':
                    self.callback_waiting.set()
                    if not self.release_callback.wait(timeout=1.0):
                        raise RuntimeError('test callback was not released')
                self._lock.acquire()
                return self

            def __exit__(self, exc_type, exc_value, traceback):
                self._lock.release()

        callback_lock = CallbackBlockedLock()
        probe.lock = callback_lock
        probe._final_callback_condition = threading.Condition()
        probe._final_callbacks_in_flight = 0
        probe._final_trace_frozen = False
        probe._final_post_freeze_ingress_count = 0
        probe._final_subscription_identity = 'final-subscription'
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        motion_proof_anchor = FinalObservation(
            10, command(1_636_000_000, 0.02), 8
        )
        probe.final_commands = deque((motion_proof_anchor,), maxlen=20)
        probe.limited_commands = deque(
            (
                LimitedObservation(20, command(1_650_000_000, 0.02), 1),
                LimitedObservation(30, command(1_987_000_000, 0.0), 2),
                LimitedObservation(40, command(2_187_000_000, 0.0), 3),
            ),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 1_987_000_000),), maxlen=20)

        probe._final_endpoint_snapshot_provider = lambda: ()

        class FakeTime:

            now_ns = 0

            @classmethod
            def monotonic(cls):
                return cls.now_ns / 1_000_000_000

            @classmethod
            def monotonic_ns(cls):
                return cls.now_ns

            @classmethod
            def sleep(cls, _seconds):
                cls.now_ns += 100_000_000
                with probe.lock:
                    probe.clock_samples.append(
                        (cls.now_ns, 1_987_000_000 + cls.now_ns)
                    )

        callback_thread = threading.Thread(
            target=probe._append_final,
            args=(
                command(1_651_000_000, 0.02),
                jazzy_message_info(
                    9,
                    source_timestamp=1_001,
                    received_timestamp=2_001,
                ),
            ),
            name='queued-final-callback',
        )
        callback_thread.start()
        self.assertTrue(callback_lock.callback_waiting.wait(timeout=1.0))
        release_timer = threading.Timer(
            0.02,
            callback_lock.release_callback.set,
        )
        release_timer.start()
        try:
            with mock.patch.object(support, 'time', FakeTime):
                with self.assertRaisesRegex(
                    support.ConsumerTraceAmbiguous,
                    'final source appeared after quiescence freeze',
                ):
                    probe.wait_confirm_consumer_timeout(
                        motion_proof_anchor,
                        fenced_consumer_zero(1_987_000_000, 30),
                        final_endpoint_fence={
                            'endpoint_gid': 'gid-gate',
                            'final_subscription_identity': (
                                'final-subscription'
                            ),
                            'final_receipt_fence_ns': 10,
                            'final_header_stamp_ns': 1_636_000_000,
                            'final_publication_sequence_number': 8,
                            'final_source_timestamp_ns': 1_000,
                            'final_received_timestamp_ns': 2_000,
                            'final_reception_sequence_number': None,
                        },
                        writer_retirement_certificate=(
                            writer_retirement_certificate()
                        ),
                        timeout=1.0,
                    )
        finally:
            callback_lock.release_callback.set()
            release_timer.cancel()
            callback_thread.join(timeout=1.0)
        self.assertFalse(callback_thread.is_alive())
        self.assertEqual(len(probe.final_commands), 2)

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
        source_anchor = (30, command(4_000_000_000, 0.02))
        consumer_zero = {
            'zero_sim_ns': 4_351_000_000,
            'zero_receipt_ns': 40,
        }
        for sequence, reason in (
            (None, 'limited publication sequence unavailable'),
            (5, 'duplicate limited publication sequence'),
            (4, 'limited publication sequence regressed'),
        ):
            with self.subTest(sequence=sequence):
                limited = (
                    LimitedObservation(4, command(0, 0.0), 1),
                    LimitedObservation(5, command(0, 0.0), 2),
                    LimitedObservation(15, command(3_995_000_000, 0.01), 3),
                    LimitedObservation(35, command(4_010_000_000, 0.02), 4),
                    LimitedObservation(40, command(4_351_000_000, 0.0), 5),
                    LimitedObservation(50, command(4_551_000_000, 0.0), sequence),
                )
                with self.assertRaisesRegex(
                    support.ConsumerTraceAmbiguous,
                    reason,
                ):
                    probe._limited_zero_watermark(
                        limited,
                        consumer_zero,
                        source_anchor=source_anchor,
                    )

    def test_limited_watermark_rejects_late_nonzero_after_first_zero(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        source_anchor = (30, command(4_000_000_000, 0.02))
        limited = (
            LimitedObservation(4, command(0, 0.0), 1),
            LimitedObservation(5, command(0, 0.0), 2),
            LimitedObservation(15, command(3_995_000_000, 0.01), 3),
            LimitedObservation(35, command(4_010_000_000, 0.02), 4),
            LimitedObservation(40, command(4_351_000_000, 0.0), 5),
            LimitedObservation(50, command(4_551_000_000, 0.01), 6),
        )

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'non-zero controller output after first zero',
        ):
            probe._limited_zero_watermark(
                limited,
                {
                    'zero_sim_ns': 4_351_000_000,
                    'zero_receipt_ns': 40,
                },
                source_anchor=source_anchor,
            )

    def test_limited_watermark_rejects_out_of_order_controller_stamp(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        source_anchor = (30, command(4_000_000_000, 0.02))
        limited = (
            LimitedObservation(4, command(0, 0.0), 1),
            LimitedObservation(5, command(0, 0.0), 2),
            LimitedObservation(15, command(3_995_000_000, 0.01), 3),
            LimitedObservation(35, command(4_010_000_000, 0.02), 4),
            LimitedObservation(40, command(4_351_000_000, 0.0), 5),
            LimitedObservation(50, command(4_350_000_000, 0.0), 6),
        )

        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'out-of-order topic header stamp',
        ):
            probe._limited_zero_watermark(
                limited,
                {
                    'zero_sim_ns': 4_351_000_000,
                    'zero_receipt_ns': 40,
                },
                source_anchor=source_anchor,
            )

    def test_limited_watermark_rejects_publication_sequence_gap(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        source_anchor = (30, command(4_000_000_000, 0.02))
        with self.assertRaisesRegex(
            support.ConsumerTraceAmbiguous,
            'limited publication sequence gap',
        ) as context:
            probe._limited_zero_watermark(
                (
                    LimitedObservation(4, command(0, 0.0), 1),
                    LimitedObservation(5, command(0, 0.0), 2),
                    LimitedObservation(15, command(3_995_000_000, 0.01), 3),
                    LimitedObservation(35, command(4_010_000_000, 0.02), 4),
                    LimitedObservation(40, command(4_351_000_000, 0.0), 5),
                    LimitedObservation(50, command(4_551_000_000, 0.0), 8),
                ),
                {
                    'zero_sim_ns': 4_351_000_000,
                    'zero_receipt_ns': 40,
                },
                source_anchor=source_anchor,
            )

        self.assertEqual(context.exception.evidence['expected_sequence'], 6)
        self.assertEqual(
            context.exception.evidence['missing_sequence_count'], 2
        )

if __name__ == '__main__':
    unittest.main()
