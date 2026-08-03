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

"""Collect Layer-2 receipt evidence after exact authority-process death."""

import importlib.util
import math
from pathlib import Path
import signal
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from voice_nav_mission.msg import InternalMotionGateState


STATE_TOPIC = '/motion_gate/internal/state'
FINAL_TOPIC = '/diff_drive_controller/cmd_vel'
WAIT_TIMEOUT_SECONDS = 8.0
POLL_SECONDS = 0.002
ARMING_MAX_AGE_NS = 20_000_000
ARMING_WINDOW_NS = 40_000_000
AUTHORITY_STOP_DEADLINE_NS = 300_000_000
ZERO_HOLD_SECONDS = 0.12
ZERO_HOLD_MIN_SPAN_NS = 100_000_000
MINIMUM_ZERO_HOLD_SAMPLES = 5
UINT64_MAX = (1 << 64) - 1


def load_test_support(filename, module_name):
    """Load one installed package-owned support module by exact filename."""
    support_path = (
        Path(get_package_share_directory('voice_nav_sim'))
        / 'test_support'
        / filename
    )
    specification = importlib.util.spec_from_file_location(
        module_name,
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError(f'could not load test support: {filename}')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


crash_evidence = load_test_support(
    'crash_evidence.py',
    'voice_nav_authority_crash_evidence',
)
launch_crash_adapter = load_test_support(
    'launch_crash_adapter.py',
    'voice_nav_authority_launch_crash_adapter',
)
fault_producer_actions = load_test_support(
    'fault_producer_actions.py',
    'voice_nav_authority_fault_producer_actions',
)


def state_qos():
    """Match MotionGate's transient-local state snapshot contract."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def candidate_qos():
    """Match the candidate writer's BEST_EFFORT volatile stream."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def motion_gate_action():
    """Create the exact no-Gazebo Gate used by the authority-death tracer."""
    return Node(
        package='voice_nav_mission',
        executable='motion_gate_node',
        name='motion_gate_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'output_frequency_hz': 50.0,
                'authority_lease_ms': 250,
                'candidate_freshness_ms': 150,
                'prepare_timeout_ms': 1000,
                'writer_graph_timeout_ms': 1000,
                'candidate_qos_depth': 1,
                'expected_candidate_writer_fqn': '/collision_monitor',
                'request_cache_size': 64,
                'linear_x_min': -0.20,
                'linear_x_max': 0.40,
                'angular_z_min': -1.20,
                'angular_z_max': 1.20,
            }
        ],
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Register exact exits before launching three independently owned PIDs."""
    ledger = crash_evidence.CrashLedger()
    crash_adapter = launch_crash_adapter.LaunchCrashAdapter(ledger)
    motion_gate = motion_gate_action()
    producers = fault_producer_actions.make_fault_producers(
        'authority_death',
    )
    exit_registrations = (
        crash_adapter.expect_clean(motion_gate, 'motion_gate'),
        crash_adapter.expect_sigkill(producers.authority, 'authority'),
        crash_adapter.expect_clean(producers.candidate, 'candidate'),
    )
    return (
        LaunchDescription(
            [
                *exit_registrations,
                motion_gate,
                *producers.actions,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            'crash_adapter': crash_adapter,
            'motion_gate': motion_gate,
            'authority': producers.authority,
            'candidate': producers.candidate,
        },
    )


class AuthorityProcessDeathTest(unittest.TestCase):
    """Observe exact process death without owning a Gate control surface."""

    @classmethod
    def setUpClass(cls):
        """Start one process-local ROS context for the observer only."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Release the observer context after behavior evidence completes."""
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Provide the exact final consumer and collect state/output receipts."""
        self.node = rclpy.create_node('diff_drive_controller')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.lock = threading.Lock()
        self.states = []
        self.outputs = []
        self.candidate_messages = []
        self.candidate_subscription = None
        self.state_subscription = self.node.create_subscription(
            InternalMotionGateState,
            STATE_TOPIC,
            self.on_state,
            state_qos(),
        )
        self.output_subscription = self.node.create_subscription(
            TwistStamped,
            FINAL_TOPIC,
            self.on_output,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()

    def tearDown(self):
        """Stop only the observer; launch owns every process lifecycle."""
        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        self.assertFalse(
            self.spin_thread.is_alive(),
            'observer executor did not stop within its teardown budget',
        )
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    def on_state(self, message):
        """Capture package-private Gate state with a steady receipt fence."""
        receipt_ns = time.monotonic_ns()
        with self.lock:
            self.states.append((receipt_ns, message))

    def on_output(self, message):
        """Capture final Gate output with an independent steady fence."""
        receipt_ns = time.monotonic_ns()
        with self.lock:
            self.outputs.append((receipt_ns, message))

    def on_candidate(self, message):
        """Capture raw candidate traffic independently of final Gate output."""
        receipt_ns = time.monotonic_ns()
        with self.lock:
            self.candidate_messages.append((receipt_ns, message))

    def snapshot(self):
        """Return immutable observer collections under one lock acquisition."""
        with self.lock:
            return tuple(self.states), tuple(self.outputs)

    def candidate_snapshot(self):
        """Return immutable raw candidate receipts."""
        with self.lock:
            return tuple(self.candidate_messages)

    def bind_candidate_observer(self, topic, writer_gid):
        """Bind the armed topic and prove the exact writer is already live."""
        self.assertIsNone(self.candidate_subscription)
        self.candidate_subscription = self.node.create_subscription(
            TwistStamped,
            topic,
            self.on_candidate,
            candidate_qos(),
        )
        expected_gid = bytes(writer_gid)
        deadline = time.monotonic() + WAIT_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            compatible = [
                endpoint
                for endpoint in self.node.get_publishers_info_by_topic(topic)
                if (
                    endpoint.node_name == 'collision_monitor'
                    and endpoint.node_namespace == '/'
                    and endpoint.topic_type
                    == 'geometry_msgs/msg/TwistStamped'
                )
            ]
            if (
                len(compatible) == 1
                and bytes(compatible[0].endpoint_gid) == expected_gid
                and self.candidate_snapshot()
            ):
                return
            time.sleep(POLL_SECONDS)
        self.fail('timed out binding the exact candidate writer GID')

    @staticmethod
    def is_zero(message):
        """Accept only exact all-axis zero output."""
        twist = message.twist
        return all(
            value == 0.0
            for value in (
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            )
        )

    @classmethod
    def is_finite_nonzero(cls, message):
        """Require a finite planar command with every illegal axis at zero."""
        twist = message.twist
        planar = (twist.linear.x, twist.angular.z)
        return (
            all(math.isfinite(value) for value in planar)
            and abs(planar[0]) + abs(planar[1]) > 0.0
            and twist.linear.y == 0.0
            and twist.linear.z == 0.0
            and twist.angular.x == 0.0
            and twist.angular.y == 0.0
        )

    @staticmethod
    def is_live_armed(state):
        """Recognize the complete live Gate generation invariant."""
        return (
            state.state == InternalMotionGateState.ARMED
            and state.authority_live
            and state.candidate_fresh
            and state.writer_bound
            and not state.motion_inhibited
            and not state.zero_selected
            and bool(state.lease_id)
            and bool(state.candidate_topic)
            and any(state.bound_writer_gid)
        )

    def wait_for_initial_armed(self):
        """Return one live state and finite non-zero final output."""
        deadline = time.monotonic() + WAIT_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            states, outputs = self.snapshot()
            if states and outputs and self.is_live_armed(states[-1][1]):
                nonzero = next(
                    (
                        sample
                        for sample in reversed(outputs)
                        if self.is_finite_nonzero(sample[1])
                    ),
                    None,
                )
                if nonzero is not None:
                    return states[-1], nonzero
            time.sleep(POLL_SECONDS)
        self.fail('timed out waiting for the independent producers to arm')

    def wait_for_fresh_arming_barrier(self, baseline_state):
        """Require a new RENEW plus recent non-zero output before SIGKILL."""
        started_ns = time.monotonic_ns()
        deadline = time.monotonic() + WAIT_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            now_ns = time.monotonic_ns()
            states, outputs = self.snapshot()
            renewed = next(
                (
                    sample
                    for sample in reversed(states)
                    if (
                        sample[0] >= started_ns
                        and sample[1].control_seq
                        > baseline_state.control_seq
                        and self.is_live_armed(sample[1])
                        and sample[1].gate_instance_id
                        == baseline_state.gate_instance_id
                        and sample[1].lease_id == baseline_state.lease_id
                    )
                ),
                None,
            )
            nonzero = next(
                (
                    sample
                    for sample in reversed(outputs)
                    if (
                        sample[0] >= started_ns
                        and self.is_finite_nonzero(sample[1])
                    )
                ),
                None,
            )
            if (
                renewed is not None
                and nonzero is not None
                and now_ns - renewed[0] <= ARMING_MAX_AGE_NS
                and now_ns - nonzero[0] <= ARMING_MAX_AGE_NS
            ):
                return min(renewed[0], nonzero[0]), renewed, nonzero
            time.sleep(POLL_SECONDS)
        self.fail('timed out waiting for a fresh authority arming barrier')

    def wait_for_exact_exit(self, crash_adapter, authority):
        """Poll only the exact ProcessExited observation recorded by Adapter."""
        deadline = time.monotonic() + WAIT_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            try:
                return crash_adapter.exit_observation(authority)
            except crash_evidence.CrashEvidenceError:
                time.sleep(POLL_SECONDS)
        self.fail('timed out waiting for exact authority ProcessExited')

    def wait_for_terminal_evidence(self, exit_ns):
        """Return first matching terminal state and zero after ProcessExited."""
        deadline_ns = exit_ns + AUTHORITY_STOP_DEADLINE_NS
        while True:
            states, outputs = self.snapshot()
            terminal = next(
                (
                    sample
                    for sample in states
                    if (
                        sample[0] >= exit_ns
                        and sample[0] <= deadline_ns
                        and sample[1].state
                        == InternalMotionGateState.INHIBITED
                        and sample[1].reason
                        == InternalMotionGateState.AUTHORITY_EXPIRED
                    )
                ),
                None,
            )
            zero = next(
                (
                    sample
                    for sample in outputs
                    if (
                        sample[0] >= exit_ns
                        and sample[0] <= deadline_ns
                        and self.is_zero(sample[1])
                    )
                ),
                None,
            )
            if terminal is not None and zero is not None:
                return terminal, zero
            if time.monotonic_ns() > deadline_ns:
                break
            time.sleep(POLL_SECONDS)
        self.fail('authority death did not close Gate output within 300 ms')

    def assert_no_preexit_retirement(
        self,
        barrier_started_ns,
        exit_ns,
        armed_state,
    ):
        """Reject a Gate that had already retired before exact ProcessExited."""
        states, outputs = self.snapshot()
        preexit_states = [
            state
            for receipt, state in states
            if barrier_started_ns <= receipt < exit_ns
        ]
        preexit_outputs = [
            output
            for receipt, output in outputs
            if barrier_started_ns <= receipt < exit_ns
        ]
        self.assertTrue(preexit_states)
        self.assertTrue(preexit_outputs)
        self.assertTrue(
            all(
                self.is_live_armed(state)
                and state.gate_instance_id == armed_state.gate_instance_id
                and state.lease_id == armed_state.lease_id
                for state in preexit_states
            ),
            'Gate retired before exact authority ProcessExited',
        )
        self.assertTrue(
            all(self.is_finite_nonzero(output) for output in preexit_outputs),
            'zero output preceded exact authority ProcessExited',
        )

    def assert_terminal_state(self, terminal, armed):
        """Require complete lease retirement and monotonic output sequences."""
        self.assertEqual(terminal.gate_instance_id, armed.gate_instance_id)
        self.assertGreater(terminal.state_seq, armed.state_seq)
        self.assertGreater(terminal.control_seq, armed.control_seq)
        self.assertEqual(terminal.state, InternalMotionGateState.INHIBITED)
        self.assertEqual(
            terminal.reason,
            InternalMotionGateState.AUTHORITY_EXPIRED,
        )
        self.assertEqual(terminal.detail, 'authority lease expired')
        self.assertEqual(terminal.lease_id, '')
        self.assertEqual(terminal.candidate_topic, '')
        self.assertFalse(any(terminal.bound_writer_gid))
        self.assertTrue(terminal.motion_inhibited)
        self.assertFalse(terminal.authority_live)
        self.assertFalse(terminal.candidate_fresh)
        self.assertFalse(terminal.writer_bound)
        self.assertTrue(terminal.zero_selected)
        self.assertGreater(
            terminal.output_publish_seq,
            armed.output_publish_seq,
        )
        self.assertEqual(
            terminal.zero_publish_seq,
            terminal.output_publish_seq,
        )
        self.assertGreater(
            terminal.zero_publish_seq,
            armed.zero_publish_seq,
        )
        self.assertLess(terminal.control_seq, UINT64_MAX)
        self.assertLess(terminal.output_publish_seq, UINT64_MAX)

    def assert_zero_hold_and_candidate_counter_evidence(
        self,
        zero_receipt_ns,
        candidate_topic,
        expected_writer_gid,
        exit_ns,
    ):
        """Keep observing Gate zero while the exact candidate writer survives."""
        hold_deadline = time.monotonic() + ZERO_HOLD_SECONDS
        while time.monotonic() < hold_deadline:
            time.sleep(POLL_SECONDS)
        _states, outputs = self.snapshot()
        held = [
            (receipt, output)
            for receipt, output in outputs
            if receipt >= zero_receipt_ns
        ]
        self.assertGreaterEqual(len(held), MINIMUM_ZERO_HOLD_SAMPLES)
        self.assertGreaterEqual(
            held[-1][0] - held[0][0],
            ZERO_HOLD_MIN_SPAN_NS,
        )
        self.assertTrue(
            all(self.is_zero(output) for _receipt, output in held),
            'Gate emitted non-zero output after authority expiry',
        )
        expected_gid = bytes(expected_writer_gid)
        compatible_writers = [
            endpoint
            for endpoint in self.node.get_publishers_info_by_topic(
                candidate_topic,
            )
            if (
                endpoint.node_name == 'collision_monitor'
                and endpoint.node_namespace == '/'
                and endpoint.topic_type
                == 'geometry_msgs/msg/TwistStamped'
            )
        ]
        self.assertEqual(
            len(compatible_writers),
            1,
            'candidate counter-evidence disappeared with authority',
        )
        self.assertEqual(
            bytes(compatible_writers[0].endpoint_gid),
            expected_gid,
            'candidate writer GID changed after authority exit',
        )
        candidate_after_exit = [
            (receipt, message)
            for receipt, message in self.candidate_snapshot()
            if receipt >= exit_ns
        ]
        self.assertGreaterEqual(
            len(candidate_after_exit),
            MINIMUM_ZERO_HOLD_SAMPLES,
        )
        self.assertGreaterEqual(
            candidate_after_exit[-1][0] - candidate_after_exit[0][0],
            ZERO_HOLD_MIN_SPAN_NS,
        )
        self.assertTrue(
            all(
                self.is_finite_nonzero(message)
                for _receipt, message in candidate_after_exit
            ),
            'exact candidate stopped non-zero traffic after authority exit',
        )
        return len(held), len(candidate_after_exit)

    def test_exact_authority_sigkill_expires_gate_to_zero(
        self,
        proc_info,
        launch_service,
        crash_adapter,
        motion_gate,
        authority,
        candidate,
    ):
        """Measure Gate state/output from exact ProcessExited, never signal."""
        for action in (motion_gate, authority, candidate):
            proc_info.assertWaitForStartup(action, timeout=5.0)
        pids = {
            action.process_details['pid']
            for action in (motion_gate, authority, candidate)
        }
        self.assertEqual(len(pids), 3)
        self.assertTrue(all(pid > 0 for pid in pids))

        (_initial_receipt, initial), _initial_output = (
            self.wait_for_initial_armed()
        )
        self.bind_candidate_observer(
            initial.candidate_topic,
            initial.bound_writer_gid,
        )
        barrier_started, (state_receipt, armed), (output_receipt, _output) = (
            self.wait_for_fresh_arming_barrier(initial)
        )
        signal_intent_ns = crash_adapter.request_sigkill(
            launch_service,
            authority,
        )
        self.assertLessEqual(
            signal_intent_ns - barrier_started,
            ARMING_WINDOW_NS,
        )
        self.assertLessEqual(
            signal_intent_ns - state_receipt,
            ARMING_MAX_AGE_NS,
        )
        self.assertLessEqual(
            signal_intent_ns - output_receipt,
            ARMING_MAX_AGE_NS,
        )

        label, returncode, exit_ns = self.wait_for_exact_exit(
            crash_adapter,
            authority,
        )
        self.assertEqual(label, 'authority')
        self.assertEqual(returncode, -signal.SIGKILL)
        self.assertGreaterEqual(exit_ns, signal_intent_ns)
        proc_info.assertWaitForShutdown(authority, timeout=2.0)
        self.assert_no_preexit_retirement(
            barrier_started,
            exit_ns,
            armed,
        )

        (terminal_receipt, terminal), (zero_receipt, zero) = (
            self.wait_for_terminal_evidence(exit_ns)
        )
        self.assertLessEqual(
            terminal_receipt - exit_ns,
            AUTHORITY_STOP_DEADLINE_NS,
        )
        self.assertLessEqual(
            zero_receipt - exit_ns,
            AUTHORITY_STOP_DEADLINE_NS,
        )
        self.assert_terminal_state(terminal, armed)
        self.assertTrue(self.is_zero(zero))
        zero_samples, candidate_samples = (
            self.assert_zero_hold_and_candidate_counter_evidence(
                zero_receipt,
                armed.candidate_topic,
                armed.bound_writer_gid,
                exit_ns,
            )
        )
        print(
            'AUTHORITY_DEATH_EVIDENCE '
            f'exit_to_state_ms={(terminal_receipt - exit_ns) / 1e6:.3f} '
            f'exit_to_zero_ms={(zero_receipt - exit_ns) / 1e6:.3f} '
            f'control_seq_delta={terminal.control_seq - armed.control_seq} '
            f'zero_samples={zero_samples} '
            f'candidate_samples={candidate_samples}',
            flush=True,
        )


@launch_testing.post_shutdown_test()
class AuthorityProcessDeathShutdownTest(unittest.TestCase):
    """Require one killed authority and two exact clean process exits."""

    def test_exact_exit_ledger_is_complete(
        self,
        proc_info,
        crash_adapter,
        motion_gate,
        authority,
        candidate,
    ):
        """Reject broad exit-code allowlists and hidden collateral exits."""
        self.assertEqual(
            crash_adapter.assert_complete(),
            (
                ('motion_gate', 0),
                ('authority', -signal.SIGKILL),
                ('candidate', 0),
            ),
        )
        assertExitCodes(
            proc_info,
            process=motion_gate,
            allowable_exit_codes=[0],
        )
        assertExitCodes(
            proc_info,
            process=authority,
            allowable_exit_codes=[-signal.SIGKILL],
        )
        assertExitCodes(
            proc_info,
            process=candidate,
            allowable_exit_codes=[0],
        )
