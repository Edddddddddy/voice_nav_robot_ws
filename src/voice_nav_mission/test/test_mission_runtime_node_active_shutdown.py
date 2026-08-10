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
from composition_interfaces.srv import ListNodes
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
from voice_nav_mission.msg import InternalMotionGateState
from voice_nav_mission.srv import InternalMotionGateControl


STATE_TOPIC = '/mission/state'
ACTION_NAME = '/mission/execute'
RUNTIME_NODE = 'mission_runtime_node'


class ActiveShutdownDependencies:

    def __init__(self):
        self.node = rclpy.create_node('mission_runtime_active_shutdown_dependencies')
        self.lock = threading.Lock()
        self.prepare_entered = threading.Event()
        self.release_prepare = threading.Event()
        self.operations = []
        self.list_nodes_call_count = 0
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
        self.list_nodes_service = self.node.create_service(
            ListNodes,
            '/motion_conditioning_container/_container/list_nodes',
            self.on_list_nodes,
            callback_group=ReentrantCallbackGroup(),
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
            message.candidate_fresh = message.authority_live
            message.writer_bound = message.authority_live
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
        response.candidate_fresh = response.authority_live
        response.writer_bound = response.authority_live
        response.zero_selected = response.motion_inhibited
        response.output_publish_seq = 1
        response.zero_publish_seq = 1 if response.motion_inhibited else 0
        response.zero_published = response.motion_inhibited
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

    def on_list_nodes(self, request, response):
        del request
        with self.lock:
            self.list_nodes_call_count += 1
        response.full_node_names = []
        response.unique_ids = []
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


class MissionRuntimeActiveShutdownTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('mission_runtime_active_shutdown_client')
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.active_dependencies = ActiveShutdownDependencies()
        self.executor.add_node(self.active_dependencies.node)
        self.states = deque(maxlen=10)
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

    def on_state(self, message):
        self.states.append(message)

    def cleanup(self):
        self.executor.remove_node(self.active_dependencies.node)
        self.active_dependencies.close()
        self.action_client.destroy()
        self.node.destroy_node()

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)
            value = predicate()
            if value is not None and value is not False:
                return value
        self.fail('等待 Runtime Interface 超时')

    def test_active_goal_gets_one_result_before_quiesced_process_exits(
        self, launch_service, proc_info, runtime
    ):
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
        with self.active_dependencies.lock:
            operations = list(self.active_dependencies.operations)
            list_nodes_call_count = self.active_dependencies.list_nodes_call_count
        self.assertGreaterEqual(list_nodes_call_count, 2)
        self.assertGreaterEqual(
            operations.count(InternalMotionGateControl.Request.INHIBIT), 2
        )
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
        self.assertGreaterEqual(
            operations.count(InternalMotionGateControl.Request.PREPARE), 1
        )
        self.assertNotIn(InternalMotionGateControl.Request.OPEN, operations)
        proc_info.assertWaitForStartup(runtime, timeout=10.0)


@launch_testing.post_shutdown_test()
class MissionRuntimeShutdownTest(unittest.TestCase):

    def test_runtime_exits_cleanly(self, proc_info, runtime):
        assertExitCodes(proc_info, process=runtime, allowable_exit_codes=[0, -2])
