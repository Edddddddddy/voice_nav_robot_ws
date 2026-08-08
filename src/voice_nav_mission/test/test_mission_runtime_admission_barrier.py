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

import signal
import threading
import unittest

from action_msgs.msg import GoalStatus
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState, MissionStep


ACTION_NAME = '/mission/execute'
STATE_TOPIC = '/mission/state'
RUNTIME_NODE = 'mission_runtime_node'
ADMISSION_MARKER = 'R73_TEST_ACTION_ADMISSION_PENDING'


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
                'test_pause_action_admission': True,
            }
        ],
    )
    return (
        LaunchDescription([runtime, launch_testing.actions.ReadyToTest()]),
        {'runtime': runtime},
    )


class MissionRuntimeAdmissionBarrierTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('mission_runtime_admission_client')
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.start()
        self.state_ready = threading.Event()
        self.state = None
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_subscription = self.node.create_subscription(
            MissionState,
            STATE_TOPIC,
            self.on_state,
            state_qos,
        )
        self.action_client = ActionClient(
            self.node, ExecuteMission, ACTION_NAME
        )
        self.addCleanup(self.cleanup)

    def on_state(self, message):
        self.state = message
        self.state_ready.set()

    def cleanup(self):
        self.executor.shutdown(timeout_sec=5.0)
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=5.0)
        self.action_client.destroy()
        self.node.destroy_node()

    def test_goal_acceptance_survives_pre_shutdown_before_handoff(
        self, launch_service, proc_info, proc_output, runtime
    ):
        self.assertTrue(self.state_ready.wait(timeout=10.0))
        self.assertTrue(self.action_client.wait_for_server(timeout_sec=10.0))

        goal = ExecuteMission.Goal()
        goal.source_instance_id = 'admission-barrier-test'
        goal.source_seq = 1
        goal.runtime_instance_id = self.state.runtime_instance_id
        goal.admission_epoch = self.state.admission_epoch
        step = MissionStep()
        step.kind = MissionStep.MOVE_DISTANCE
        step.distance_m = 0.5
        goal.steps.append(step)

        send_future = self.action_client.send_goal_async(goal)

        goal_done = threading.Event()
        result_request_started = threading.Event()
        result_done = threading.Event()
        goal_handle_holder = {}
        result_response_holder = {}

        def remember_result(result_future):
            result_response_holder['value'] = result_future.result()
            result_done.set()

        def request_result(goal_response):
            goal_handle = goal_response.result()
            goal_handle_holder['value'] = goal_handle
            goal_done.set()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                result_request_started.set()
                result_future.add_done_callback(remember_result)

        # Register the client continuation before waiting for the server's
        # on_accepted barrier.  Once on_goal returns, the real Action Client
        # receives the accepted handle and requests its result before SIGINT.
        send_future.add_done_callback(request_result)

        runtime_proxy = launch_testing.tools.ProcessProxy(
            runtime, proc_info, proc_output
        )
        self.assertTrue(
            runtime_proxy.wait_for_output(
                lambda output: ADMISSION_MARKER in output,
                timeout=10.0,
            )
        )
        self.assertTrue(goal_done.wait(timeout=5.0))
        self.assertTrue(result_request_started.wait(timeout=5.0))

        # Release quiesce only after the client has the accepted handle and its
        # result request is in flight.  The server may publish the terminal
        # boundary immediately after on_accepted returns.
        launch_service.emit_event(
            launch.events.process.SignalProcess(
                signal_number=signal.SIGINT,
                process_matcher=launch.events.matches_action(runtime),
            )
        )

        goal_handle = goal_handle_holder['value']
        self.assertTrue(goal_handle.accepted)

        self.assertTrue(result_done.wait(timeout=15.0))
        wrapped = result_response_holder['value']
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            wrapped.result.code,
            ExecuteMission.Result.SAFETY_FAULT,
        )
        self.assertEqual(wrapped.result.failed_step, -1)

        # A restarted process proves the pre-shutdown barrier did not leave a
        # pending UUID or in-flight Action callback holding the old server.
        proc_info.assertWaitForStartup(runtime, timeout=10.0)


@launch_testing.post_shutdown_test()
class MissionRuntimeAdmissionBarrierShutdownTest(unittest.TestCase):

    def test_runtime_exits_cleanly(self, proc_info, runtime):
        assertExitCodes(
            proc_info, process=runtime, allowable_exit_codes=[0, -2]
        )
