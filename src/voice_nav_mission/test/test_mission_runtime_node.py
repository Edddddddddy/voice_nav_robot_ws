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
import time
import unittest

from action_msgs.msg import GoalStatus
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState, MissionStep
from voice_nav_interfaces.srv import StopMission


STATE_TOPIC = '/mission/state'
ACTION_NAME = '/mission/execute'
STOP_NAME = '/mission/stop'
RUNTIME_NODE = 'mission_runtime_node'


def generate_test_description():
    runtime = Node(
        package='voice_nav_mission',
        executable='mission_runtime_node',
        name=RUNTIME_NODE,
        output='screen',
        parameters=[
            {
                'operating_mode': 'mapping',
                'mission_deadline_ms': 30000,
                'gate_discovery_deadline_ms': 2000,
                'control_response_deadline_ms': 100,
                'stop_barrier_ms': 250,
                'cancel_grace_ms': 250,
                'source_cache_size': 64,
                'stop_cache_size': 64,
                'max_steps': 3,
                'named_place_ids': [],
            }
        ],
    )
    return LaunchDescription(
        [runtime, launch_testing.actions.ReadyToTest()]
    )


class MissionRuntimeNodeTest(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('mission_runtime_contract_client')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.states = deque(maxlen=10)
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_subscription = self.node.create_subscription(
            MissionState,
            STATE_TOPIC,
            self.states.append,
            state_qos,
        )
        self.action_client = ActionClient(
            self.node, ExecuteMission, ACTION_NAME
        )
        self.stop_client = self.node.create_client(StopMission, STOP_NAME)
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.action_client.destroy()
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)
            value = predicate()
            if value is not None and value is not False:
                return value
        self.fail('等待 Runtime Interface 超时')

    def test_late_state_and_business_rejection_without_gate(self):
        state = self.spin_until(lambda: self.states[-1] if self.states else None)
        self.assertRegex(state.runtime_instance_id, r'^[0-9a-f]{32}$')
        self.assertEqual(state.admission_epoch, 1)
        self.assertEqual(state.operating_mode, MissionState.MAPPING)
        self.assertEqual(state.availability, MissionState.UNAVAILABLE)
        self.assertEqual(
            state.active_step, 2**32 - 1
        )
        self.assertEqual(state.supported_step_mask, 3)
        self.assertEqual(state.max_steps, 3)

        self.assertTrue(self.action_client.wait_for_server(timeout_sec=5.0))
        goal = ExecuteMission.Goal()
        goal.source_instance_id = 'source-test'
        goal.source_seq = 1
        goal.runtime_instance_id = state.runtime_instance_id
        goal.admission_epoch = state.admission_epoch
        move = MissionStep()
        move.kind = MissionStep.MOVE_DISTANCE
        move.distance_m = 0.5
        goal.steps.append(move)
        send_future = self.action_client.send_goal_async(goal)
        goal_handle = self.spin_until(lambda: send_future.result())
        self.assertTrue(goal_handle)
        result_future = goal_handle.get_result_async()
        wrapped = self.spin_until(lambda: result_future.result())
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            wrapped.result.code,
            ExecuteMission.Result.DEPENDENCY_UNAVAILABLE,
        )
        self.assertEqual(wrapped.result.failed_step, -1)

    def test_invalid_goal_is_aborted_with_structured_result(self):
        state = self.spin_until(lambda: self.states[-1] if self.states else None)
        self.assertTrue(self.action_client.wait_for_server(timeout_sec=5.0))
        goal = ExecuteMission.Goal()
        goal.source_instance_id = 'source-invalid'
        goal.source_seq = 1
        goal.runtime_instance_id = state.runtime_instance_id
        goal.admission_epoch = state.admission_epoch
        invalid = MissionStep()
        invalid.kind = MissionStep.MOVE_DISTANCE
        invalid.distance_m = 0.0
        goal.steps.append(invalid)
        send_future = self.action_client.send_goal_async(goal)
        goal_handle = self.spin_until(lambda: send_future.result())
        self.assertTrue(goal_handle)
        result_future = goal_handle.get_result_async()
        wrapped = self.spin_until(lambda: result_future.result())
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            wrapped.result.code, ExecuteMission.Result.INVALID_PLAN
        )
        self.assertEqual(wrapped.result.failed_step, -1)


@launch_testing.post_shutdown_test()
class MissionRuntimeShutdownTest(unittest.TestCase):

    def test_runtime_exits_cleanly(self, proc_info):
        assertExitCodes(proc_info)
