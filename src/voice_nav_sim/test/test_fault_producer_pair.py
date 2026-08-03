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

"""Prove two independent helper processes autonomously arm MotionGate."""

import importlib.util
import math
from pathlib import Path
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
POLL_SECONDS = 0.005
SUSTAINED_LIVE_SECONDS = 0.55
CANDIDATE_PREFIX = '/voice_nav_internal/motion_gate/candidate/lease_'


def load_fault_producer_actions():
    """Load the installed factory through its package-owned test seam."""
    support_path = (
        Path(get_package_share_directory('voice_nav_sim'))
        / 'test_support'
        / 'fault_producer_actions.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_fault_producer_actions_test_support',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load fault producer action support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


fault_producer_actions = load_fault_producer_actions()


def state_qos():
    """Match MotionGate's transient-local state snapshot contract."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch a real Gate and two separately crashable producer processes."""
    motion_gate = Node(
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
    producers = fault_producer_actions.make_fault_producers('authority_arm')
    return (
        LaunchDescription(
            [
                motion_gate,
                *producers.actions,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            'motion_gate': motion_gate,
            'authority': producers.authority,
            'candidate': producers.candidate,
        },
    )


class FaultProducerPairTest(unittest.TestCase):
    """Observe the Gate without owning any control or candidate endpoint."""

    @classmethod
    def setUpClass(cls):
        """Start one process-local ROS context for the observer only."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Release the observer context after launch behavior completes."""
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Subscribe to output evidence without creating a control client."""
        # The real Gate refuses to arm unless the final controller consumer is
        # present.  This test-local node is that exact health endpoint and also
        # records output; it deliberately owns no Gate control client.
        self.node = rclpy.create_node('diff_drive_controller')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.lock = threading.Lock()
        self.states = []
        self.outputs = []
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
        """Stop only the process-local observer; launch owns all processes."""
        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        self.assertFalse(
            self.spin_thread.is_alive(),
            'observer executor did not stop within its bounded teardown',
        )
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    def on_state(self, message):
        """Capture state with a steady observer receipt time."""
        with self.lock:
            self.states.append((time.monotonic_ns(), message))

    def on_output(self, message):
        """Capture final Gate output independently of internal state."""
        with self.lock:
            self.outputs.append((time.monotonic_ns(), message))

    def armed_evidence(self):
        """Return one fully live state plus one observed non-zero output."""
        with self.lock:
            states = tuple(self.states)
            outputs = tuple(self.outputs)
        if not states or not outputs:
            return None
        state_receipt, state = states[-1]
        nonzero = next(
            (
                (receipt, message)
                for receipt, message in reversed(outputs)
                if abs(message.twist.linear.x) +
                abs(message.twist.angular.z) > 0.0
            ),
            None,
        )
        if (
            nonzero is None or
            state.state != InternalMotionGateState.ARMED or
            not state.authority_live or
            not state.candidate_fresh or
            not state.writer_bound or
            state.motion_inhibited or
            state.zero_selected
        ):
            return None
        return state_receipt, state, nonzero

    def wait_for_armed_evidence(self):
        """Wait for autonomous PREPARE/OPEN/RENEW and candidate traffic."""
        deadline = time.monotonic() + WAIT_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            evidence = self.armed_evidence()
            if evidence is not None:
                return evidence
            time.sleep(POLL_SECONDS)
        self.fail('timed out waiting for autonomous fault producers to arm')

    def require_sustained_renewal_and_candidate_traffic(self, first_state):
        """Keep the test open beyond both leases and reject one-shot helpers."""
        started_at = time.monotonic_ns()
        deadline = time.monotonic() + SUSTAINED_LIVE_SECONDS
        while time.monotonic() < deadline:
            self.assertIsNotNone(
                self.armed_evidence(),
                'producer pair stopped sustaining a live armed Gate',
            )
            time.sleep(POLL_SECONDS)

        with self.lock:
            states = tuple(
                message
                for receipt, message in self.states
                if receipt >= started_at
            )
            outputs = tuple(
                message
                for receipt, message in self.outputs
                if receipt >= started_at
            )
        self.assertTrue(states)
        self.assertTrue(outputs)
        self.assertGreaterEqual(
            max(state.control_seq for state in states),
            first_state.control_seq + 3,
            'authority did not autonomously renew across the live window',
        )
        self.assertTrue(
            all(
                state.state == InternalMotionGateState.ARMED and
                state.authority_live and
                state.candidate_fresh and
                state.writer_bound and
                not state.motion_inhibited
                for state in states
            ),
            'Gate left its live armed invariants during the sustain window',
        )
        self.assertTrue(
            any(
                abs(message.twist.linear.x) +
                abs(message.twist.angular.z) > 0.0
                for message in outputs
            ),
            'candidate did not sustain non-zero traffic',
        )

    def test_independent_helpers_arm_gate_without_parent_control(
        self,
        proc_info,
        motion_gate,
        authority,
        candidate,
    ):
        """Require distinct live processes and a bounded non-zero command."""
        for action in (motion_gate, authority, candidate):
            proc_info.assertWaitForStartup(action, timeout=5.0)
        pids = {
            action.process_details['pid']
            for action in (motion_gate, authority, candidate)
        }
        self.assertEqual(len(pids), 3)
        self.assertTrue(all(pid > 0 for pid in pids))

        _state_receipt, state, (_output_receipt, output) = (
            self.wait_for_armed_evidence()
        )
        self.assertEqual(len(state.gate_instance_id), 32)
        self.assertEqual(len(state.lease_id), 32)
        self.assertTrue(state.candidate_topic.startswith(CANDIDATE_PREFIX))
        self.assertTrue(any(state.bound_writer_gid))
        self.assertTrue(math.isfinite(output.twist.linear.x))
        self.assertTrue(math.isfinite(output.twist.angular.z))
        self.assertGreater(
            abs(output.twist.linear.x) + abs(output.twist.angular.z),
            0.0,
        )
        self.assertEqual(output.twist.linear.y, 0.0)
        self.assertEqual(output.twist.linear.z, 0.0)
        self.assertEqual(output.twist.angular.x, 0.0)
        self.assertEqual(output.twist.angular.y, 0.0)
        self.require_sustained_renewal_and_candidate_traffic(state)


@launch_testing.post_shutdown_test()
class FaultProducerPairShutdownTest(unittest.TestCase):
    """Require deterministic clean teardown for all three exact actions."""

    def test_all_fixture_processes_exit_cleanly(
        self,
        proc_info,
        motion_gate,
        authority,
        candidate,
    ):
        """Reject helper crashes hidden by a broad exit allowlist."""
        for action in (motion_gate, authority, candidate):
            assertExitCodes(
                proc_info,
                process=action,
                allowable_exit_codes=[0],
            )
