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

from collections import deque
import signal
import time
import unittest

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState


STATE_TOPIC = '/mission/state'
ACTION_NAME = '/mission/execute'
RUNTIME_NODE = 'mission_runtime_node'


def generate_test_description():
    runtime = Node(
        package='voice_nav_mission',
        executable='mission_runtime_node',
        name=RUNTIME_NODE,
        output='screen',
        respawn=True,
        respawn_delay=0.0,
        parameters=[
            {
                'operating_mode': 'mapping',
                'use_sim_time': True,
                'mission_deadline_ms': 30000,
                'gate_discovery_deadline_ms': 2000,
                'control_response_deadline_ms': 100,
                'stop_barrier_ms': 250,
                'cancel_grace_ms': 250,
                'source_cache_size': 64,
                'stop_cache_size': 64,
                'max_steps': 3,
            }
        ],
    )
    return (
        LaunchDescription([runtime, launch_testing.actions.ReadyToTest()]),
        {'runtime': runtime},
    )


class MissionRuntimeRestartTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('mission_runtime_restart_client')
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.states = deque(maxlen=20)
        self.runtime_ids = set()
        self.state_subscription = self.node.create_subscription(
            MissionState,
            STATE_TOPIC,
            self.on_state,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.action_client = ActionClient(
            self.node, ExecuteMission, ACTION_NAME
        )
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.action_client.destroy()
        self.node.destroy_node()

    def on_state(self, message):
        self.states.append(message)
        self.runtime_ids.add(message.runtime_instance_id)

    def spin_until(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)
            value = predicate()
            if value is not None and value is not False:
                return value
        self.fail('等待 Runtime identity 超时')

    def test_restart_rotates_identity_in_independent_process_fixture(
        self, launch_service, proc_info, runtime
    ):
        self.assertTrue(self.action_client.wait_for_server(timeout_sec=10.0))
        first = self.spin_until(lambda: self.states[-1] if self.states else None)
        first_id = first.runtime_instance_id
        self.states.clear()
        launch_service.emit_event(
            launch.events.process.SignalProcess(
                signal_number=signal.SIGINT,
                process_matcher=launch.events.matches_action(runtime),
            )
        )
        second = self.spin_until(
            lambda: next(
                (
                    state for state in self.states
                    if state.runtime_instance_id != first_id
                ),
                None,
            ),
            timeout=15.0,
        )
        self.assertNotEqual(second.runtime_instance_id, first_id)
        self.assertEqual(second.admission_epoch, 1)
        self.assertEqual(second.availability, MissionState.UNAVAILABLE)
        self.assertEqual(second.gate_state, MissionState.GATE_FAULTED)
        self.assertEqual(self.runtime_ids, {first_id, second.runtime_instance_id})
        proc_info.assertWaitForStartup(runtime, timeout=10.0)


@launch_testing.post_shutdown_test()
class MissionRuntimeShutdownTest(unittest.TestCase):

    def test_runtime_exits_cleanly(self, proc_info, runtime):
        assertExitCodes(proc_info, process=runtime, allowable_exit_codes=[0, -2])
