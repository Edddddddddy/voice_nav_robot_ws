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
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState, MissionStep
from voice_nav_interfaces.srv import StopMission
from voice_nav_mission.msg import InternalMotionGateState
from voice_nav_mission.srv import InternalMotionGateControl


STATE_TOPIC = '/mission/state'
ACTION_NAME = '/mission/execute'
STOP_NAME = '/mission/stop'
RUNTIME_NODE = 'mission_runtime_node'


class ActiveShutdownDependencies:

    def __init__(self):
        self.node = rclpy.create_node('mission_runtime_shutdown_dependencies')
        self.lock = threading.Lock()
        self.prepare_entered = threading.Event()
        self.release_prepare = threading.Event()
        self.operations = []
        self.control_seq = 0
        self.state = InternalMotionGateState.INHIBITED
        self.lease_id = ''
        self.candidate_topic = ''
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_publisher = self.node.create_publisher(
            InternalMotionGateState,
            '/motion_gate/internal/state',
            state_qos,
        )
        self.service = self.node.create_service(
            InternalMotionGateControl,
            '/motion_gate/internal/control',
            self.on_control,
            callback_group=ReentrantCallbackGroup(),
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_publisher = self.node.create_publisher(
            Odometry, '/odom', sensor_qos
        )
        self.scan_publisher = self.node.create_publisher(
            LaserScan, '/scan', sensor_qos
        )
        self.clock_publisher = self.node.create_publisher(
            Clock, '/clock', sensor_qos
        )
        self.clock_tick = 0
        self.timer = self.node.create_timer(0.02, self.publish_sources)
        self.publish_sources()

    def publish_sources(self):
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        self.odom_publisher.publish(odom)
        self.scan_publisher.publish(LaserScan())
        clock = Clock()
        self.clock_tick += 1
        clock.clock.sec = 1000 + self.clock_tick
        self.clock_publisher.publish(clock)
        self.publish_state()

    def publish_state(self):
        with self.lock:
            message = InternalMotionGateState()
            message.gate_instance_id = 'f' * 32
            message.state_seq = self.clock_tick
            message.control_seq = self.control_seq
            message.state = self.state
            message.lease_id = self.lease_id
            message.candidate_topic = self.candidate_topic
            message.motion_inhibited = self.state != InternalMotionGateState.ARMED
            message.authority_live = self.state == InternalMotionGateState.ARMED
            message.candidate_fresh = self.state == InternalMotionGateState.ARMED
            message.writer_bound = self.state == InternalMotionGateState.ARMED
            message.zero_selected = message.motion_inhibited
            message.output_publish_seq = 1
            message.zero_publish_seq = 1 if message.motion_inhibited else 0
        self.state_publisher.publish(message)

    def fill_response(self, response, detail):
        response.code = InternalMotionGateControl.Response.APPLIED
        response.reason = InternalMotionGateControl.Response.NONE
        response.gate_instance_id = 'f' * 32
        response.control_seq = self.control_seq
        response.state = self.state
        response.lease_id = self.lease_id
        response.candidate_topic = self.candidate_topic
        response.bound_writer_gid = [0] * 16
        response.motion_inhibited = self.state != InternalMotionGateState.ARMED
        response.authority_live = self.state == InternalMotionGateState.ARMED
        response.candidate_fresh = self.state == InternalMotionGateState.ARMED
        response.writer_bound = self.state == InternalMotionGateState.ARMED
        response.zero_selected = response.motion_inhibited
        response.output_publish_seq = 1
        response.zero_publish_seq = 1 if response.motion_inhibited else 0
        response.detail = detail

    def on_control(self, request, response):
        with self.lock:
            self.operations.append(request.operation)
        if request.operation == InternalMotionGateControl.Request.PREPARE:
            self.prepare_entered.set()
            self.release_prepare.wait(timeout=10.0)
            with self.lock:
                self.control_seq += 1
                self.state = InternalMotionGateState.PREPARED
                self.lease_id = 'e' * 32
                self.candidate_topic = '/candidate/shutdown-test'
                self.fill_response(response, 'prepared for shutdown barrier')
        elif request.operation == InternalMotionGateControl.Request.INHIBIT:
            self.release_prepare.set()
            with self.lock:
                self.control_seq += 1
                self.state = InternalMotionGateState.INHIBITED
                self.lease_id = ''
                self.candidate_topic = ''
                self.fill_response(response, 'inhibited for shutdown barrier')
        else:
            with self.lock:
                self.fill_response(response, 'unexpected operation')
        self.publish_state()
        return response

    def close(self):
        self.release_prepare.set()
        self.timer.cancel()
        self.node.destroy_node()


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


class MissionRuntimeNodeTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('mission_runtime_contract_client')
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.states = deque(maxlen=10)
        self.runtime_ids = set()
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
        self.stop_client = self.node.create_client(StopMission, STOP_NAME)
        self.addCleanup(self.cleanup)

    def cleanup(self):
        if hasattr(self, 'active_dependencies'):
            self.executor.remove_node(self.active_dependencies.node)
            self.active_dependencies.close()
            del self.active_dependencies
        self.action_client.destroy()
        self.node.destroy_node()

    def on_state(self, message):
        self.states.append(message)
        self.runtime_ids.add(message.runtime_instance_id)

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)
            value = predicate()
            if value is not None and value is not False:
                return value
        self.fail('等待 Runtime Interface 超时')

    def fresh_runtime_state(self):
        self.assertTrue(self.action_client.wait_for_server(timeout_sec=10.0))
        self.states.clear()
        return self.spin_until(lambda: self.states[-1] if self.states else None)

    def test_late_state_and_business_rejection_without_gate(self):
        state = self.fresh_runtime_state()
        self.assertRegex(state.runtime_instance_id, r'^[0-9a-f]{32}$')
        self.assertEqual(state.admission_epoch, 1)
        self.assertEqual(state.operating_mode, MissionState.MAPPING)
        self.assertEqual(state.availability, MissionState.UNAVAILABLE)
        self.assertEqual(
            state.active_step, 2**32 - 1
        )
        self.assertEqual(state.supported_step_mask, 3)
        self.assertEqual(state.max_steps, 3)

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
        state = self.fresh_runtime_state()
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

    def test_active_goal_gets_one_result_before_quiesced_node_restarts(
        self, launch_service, proc_info, runtime
    ):
        self.active_dependencies = ActiveShutdownDependencies()
        self.executor.add_node(self.active_dependencies.node)
        try:
            self.assertTrue(self.action_client.wait_for_server(timeout_sec=10.0))
            self.states.clear()
            state = self.spin_until(
                lambda: next(
                    (
                        sample for sample in reversed(self.states)
                        if sample.availability == MissionState.AVAILABLE and
                        sample.gate_state == MissionState.GATE_INHIBITED
                    ),
                    None,
                ),
                timeout=10.0,
            )
        except AssertionError:
            samples = [
                (sample.availability, sample.gate_state,
                 sample.admission_epoch)
                for sample in self.states
            ]
            self.fail(f'未获得可用 Gate 状态，已观测={samples}')
        goal = ExecuteMission.Goal()
        goal.source_instance_id = 'source-shutdown-active'
        goal.source_seq = 1
        goal.runtime_instance_id = state.runtime_instance_id
        goal.admission_epoch = state.admission_epoch
        move = MissionStep()
        move.kind = MissionStep.MOVE_DISTANCE
        move.distance_m = 0.5
        goal.steps.append(move)
        send_future = self.action_client.send_goal_async(goal)
        goal_handle = self.spin_until(
            lambda: send_future.result(), timeout=5.0
        )
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()
        self.spin_until(
            lambda: True if self.active_dependencies.prepare_entered.is_set()
            else None,
            timeout=5.0,
        )

        launch_service.emit_event(
            launch.events.process.SignalProcess(
                signal_number=signal.SIGINT,
                process_matcher=launch.events.matches_action(runtime),
            )
        )
        wrapped = self.spin_until(
            lambda: result_future.result(), timeout=15.0
        )
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertIn(
            wrapped.result.code,
            (
                ExecuteMission.Result.EXECUTION_FAILED,
                ExecuteMission.Result.SAFETY_FAULT,
                ExecuteMission.Result.TIMEOUT,
            ),
        )
        with self.active_dependencies.lock:
            operations = list(self.active_dependencies.operations)
        # The single conditioning PREPARE may issue bounded control-RPC
        # retries while the fake service is held; it must never cross into
        # OPEN during quiesce.
        self.assertGreaterEqual(
            operations.count(InternalMotionGateControl.Request.PREPARE), 1
        )
        self.assertNotIn(
            InternalMotionGateControl.Request.OPEN, operations
        )
        proc_info.assertWaitForStartup(runtime, timeout=10.0)

    def test_runtime_restart_rotates_identity_and_restarts_at_epoch_one(
        self, launch_service, proc_info, runtime
    ):
        first = self.fresh_runtime_state()
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
            timeout=10.0,
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
