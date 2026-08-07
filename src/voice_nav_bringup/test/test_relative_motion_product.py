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
import importlib.util
import math
from pathlib import Path
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_system_default,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState, MissionStep
from voice_nav_interfaces.srv import StopMission
from voice_nav_mission.msg import InternalMotionGateState


ACTION_NAME = '/mission/execute'
STOP_NAME = '/mission/stop'
MISSION_STATE_TOPIC = '/mission/state'
GATE_STATE_TOPIC = '/motion_gate/internal/state'
ODOMETRY_TOPIC = '/odom'
FINAL_COMMAND_TOPIC = '/diff_drive_controller/cmd_vel'
RAW_CLOCK_TOPIC = '/clock'
RAW_SCAN_TOPIC = '/scan'
ZERO_EPSILON = 1.0e-6
STATIONARY_LINEAR_TOLERANCE = 0.01
STATIONARY_ANGULAR_TOLERANCE = 0.02
RAW_SCAN_SOURCE_AGE_LIMIT_SECONDS = 0.300


def load_gazebo_shutdown_support():
    support_path = (
        Path(get_package_share_directory('voice_nav_sim'))
        / 'test_support'
        / 'gazebo_shutdown.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_relative_motion_product_gazebo_shutdown',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load Gazebo shutdown support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


gazebo_shutdown = load_gazebo_shutdown_support()
PRODUCT_TEST_PARTITION = gazebo_shutdown.claim_unique_test_partition(
    'l0010_relative_motion_product'
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    package_share = get_package_share_directory('voice_nav_bringup')
    product = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{package_share}/launch/product_sim.launch.py'
        ),
        launch_arguments={
            'headless': 'true',
            'shutdown_on_gazebo_exit': 'false',
        }.items(),
    )
    return LaunchDescription([
        product,
        launch_testing.actions.ReadyToTest(),
    ])


def yaw_from_odom(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    numerator = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    denominator = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(numerator, denominator)


def wrapped_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def is_zero(message: TwistStamped) -> bool:
    twist = message.twist
    return all(
        abs(value) <= ZERO_EPSILON
        for value in (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
    )


class RelativeMotionProductTest(unittest.TestCase):

    def setUp(self, proc_info):
        self.addCleanup(self.destroy_ros_fixture)
        self.addCleanup(
            gazebo_shutdown.structured_stop_gazebo,
            proc_info,
            expected_partition=PRODUCT_TEST_PARTITION,
        )
        rclpy.init()
        self.node = rclpy.create_node(
            'relative_motion_contract_client',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.fixture_started_at = time.monotonic()
        self.samples_lock = threading.Lock()
        self.mission_states = deque(maxlen=2000)
        self.gate_states = deque(maxlen=4000)
        self.odometry = deque(maxlen=4000)
        self.final_commands = deque(maxlen=4000)
        self.clock_samples = deque(maxlen=4000)
        self.scan_samples = deque(maxlen=4000)
        self.raw_scan_age_samples = deque(maxlen=8000)
        self.latest_clock_ns = None
        self.latest_scan_stamp_ns = None
        self.raw_scan_age_max_seconds = 0.0
        self.raw_scan_first_over_limit = None
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        raw_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscriptions = [
            self.node.create_subscription(
                MissionState,
                MISSION_STATE_TOPIC,
                lambda message: self.append_sample(
                    self.mission_states, message
                ),
                state_qos,
            ),
            self.node.create_subscription(
                InternalMotionGateState,
                GATE_STATE_TOPIC,
                lambda message: self.append_sample(
                    self.gate_states, message
                ),
                state_qos,
            ),
            self.node.create_subscription(
                Odometry,
                ODOMETRY_TOPIC,
                lambda message: self.append_sample(self.odometry, message),
                100,
            ),
            self.node.create_subscription(
                TwistStamped,
                FINAL_COMMAND_TOPIC,
                lambda message: self.append_sample(
                    self.final_commands, message
                ),
                qos_profile_system_default,
            ),
            self.node.create_subscription(
                Clock,
                RAW_CLOCK_TOPIC,
                self.append_clock_sample,
                raw_sensor_qos,
            ),
            self.node.create_subscription(
                LaserScan,
                RAW_SCAN_TOPIC,
                self.append_scan_sample,
                raw_sensor_qos,
            ),
        ]
        self.action_client = ActionClient(
            self.node, ExecuteMission, ACTION_NAME
        )
        self.stop_client = self.node.create_client(StopMission, STOP_NAME)
        self.source_seq = 0

    def destroy_ros_fixture(self):
        executor = getattr(self, 'executor', None)
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        node = getattr(self, 'node', None)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def append_sample(self, samples, message):
        with self.samples_lock:
            samples.append((time.monotonic(), message))

    @staticmethod
    def clock_stamp_ns(message: Clock) -> int:
        return message.clock.sec * 1_000_000_000 + message.clock.nanosec

    @staticmethod
    def scan_stamp_ns(message: LaserScan) -> int:
        return message.header.stamp.sec * 1_000_000_000 + (
            message.header.stamp.nanosec
        )

    def record_raw_scan_age_locked(self):
        if self.latest_clock_ns is None or self.latest_scan_stamp_ns is None:
            return
        age_ns = self.latest_clock_ns - self.latest_scan_stamp_ns
        if age_ns < 0:
            return
        age_seconds = age_ns / 1_000_000_000.0
        observed_at = time.monotonic()
        self.raw_scan_age_samples.append((observed_at, age_seconds))
        self.raw_scan_age_max_seconds = max(
            self.raw_scan_age_max_seconds, age_seconds
        )
        if (
            age_seconds >= RAW_SCAN_SOURCE_AGE_LIMIT_SECONDS
            and self.raw_scan_first_over_limit is None
        ):
            self.raw_scan_first_over_limit = (
                observed_at,
                age_seconds,
                self.latest_clock_ns,
                self.latest_scan_stamp_ns,
            )

    def append_clock_sample(self, message: Clock):
        with self.samples_lock:
            self.clock_samples.append((time.monotonic(), message))
            self.latest_clock_ns = self.clock_stamp_ns(message)
            self.record_raw_scan_age_locked()

    def append_scan_sample(self, message: LaserScan):
        with self.samples_lock:
            self.scan_samples.append((time.monotonic(), message))
            self.latest_scan_stamp_ns = self.scan_stamp_ns(message)
            self.record_raw_scan_age_locked()

    def assert_raw_scan_age_under_limit(self):
        with self.samples_lock:
            maximum = self.raw_scan_age_max_seconds
            first_over = self.raw_scan_first_over_limit
            clock_count = len(self.clock_samples)
            scan_count = len(self.scan_samples)
        self.assertGreater(
            clock_count, 0, 'no raw /clock samples were observed'
        )
        self.assertGreater(scan_count, 0, 'no raw /scan samples were observed')
        self.assertLess(
            maximum,
            RAW_SCAN_SOURCE_AGE_LIMIT_SECONDS,
            msg=(
                'raw /scan ROS stamp age reached the fail-closed limit: '
                f'max={maximum:.6f}s first_over={first_over!r}'
            ),
        )

    def latest_sample(self, samples):
        with self.samples_lock:
            return samples[-1] if samples else None

    def sample_snapshot(self, samples):
        with self.samples_lock:
            return tuple(samples)

    def wait_until(self, predicate, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            value = predicate()
            if value is not None and value is not False:
                return value
        self.fail(f'timed out waiting for {description}')

    def wait_for_ready(self):
        return self.wait_until(
            lambda: (
                sample[1]
                if (
                    (sample := self.latest_sample(self.mission_states))
                    and sample[0] >= self.fixture_started_at
                    and (
                        (odom_sample := self.latest_sample(self.odometry))
                        and odom_sample[0] >= self.fixture_started_at
                    )
                    and sample[1].availability == MissionState.AVAILABLE
                    and sample[1].gate_state == MissionState.GATE_INHIBITED
                )
                else None
            ),
            30.0,
            'RelativeMotion Runtime availability',
        )

    def wait_for_stationary_zero(self):
        def safe_snapshot():
            now = time.monotonic()
            odom = self.sample_snapshot(self.odometry)
            commands = self.sample_snapshot(self.final_commands)
            gate = self.latest_sample(self.gate_states)
            recent_odom = tuple(
                sample for sample in odom if now - sample[0] <= 0.35
            )
            recent_commands = tuple(
                sample for sample in commands if now - sample[0] <= 0.35
            )
            stationary_since = (
                recent_odom[0][0]
                if recent_odom and now - recent_odom[0][0] >= 0.20
                else None
            )
            stationary = bool(stationary_since) and all(
                abs(sample[1].twist.twist.linear.x)
                <= STATIONARY_LINEAR_TOLERANCE
                and abs(sample[1].twist.twist.angular.z)
                <= STATIONARY_ANGULAR_TOLERANCE
                for sample in recent_odom
                if sample[0] >= stationary_since
            )
            commands_zero = bool(recent_commands) and all(
                is_zero(sample[1]) for sample in recent_commands
            )
            gate_zero = bool(gate) and gate[1].motion_inhibited and (
                gate[1].zero_selected and gate[1].zero_publish_seq > 0
            )
            return stationary and commands_zero and gate_zero

        self.wait_until(
            safe_snapshot,
            5.0,
            'Gate zero and 200 ms stationary odometry',
        )

    def make_goal(self, steps):
        state = self.wait_for_ready()
        goal = ExecuteMission.Goal()
        self.source_seq += 1
        goal.source_instance_id = 'issue64-relative-motion'
        goal.source_seq = self.source_seq
        goal.runtime_instance_id = state.runtime_instance_id
        goal.admission_epoch = state.admission_epoch
        goal.steps.extend(steps)
        return goal

    def execute_goal(self, steps, feedback_callback=None):
        goal = self.make_goal(steps)
        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        goal_handle = self.wait_until(
            lambda: send_future.result() if send_future.done() else None,
            30.0,
            'Mission goal admission',
        )
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()
        wrapped = self.wait_until(
            lambda: result_future.result() if result_future.done() else None,
            30.0,
            'Mission result',
        )
        self.wait_for_stationary_zero()
        return goal_handle, wrapped

    @staticmethod
    def move_step(distance: float) -> MissionStep:
        step = MissionStep()
        step.kind = MissionStep.MOVE_DISTANCE
        step.distance_m = distance
        return step

    @staticmethod
    def rotate_step(angle: float) -> MissionStep:
        step = MissionStep()
        step.kind = MissionStep.ROTATE_ANGLE
        step.angle_rad = angle
        return step

    def assert_move(self, distance: float):
        start = self.wait_until(
            lambda: self.latest_sample(self.odometry),
            30.0,
            'fresh starting odometry',
        )[1]
        _, wrapped = self.execute_goal([self.move_step(distance)])
        self.assertEqual(
            wrapped.status,
            GoalStatus.STATUS_SUCCEEDED,
            msg=(
                f'code={wrapped.result.code} '
                f'detail={wrapped.result.detail}'
            ),
        )
        self.assertEqual(wrapped.result.code, ExecuteMission.Result.SUCCEEDED)
        final = self.latest_sample(self.odometry)[1]
        start_yaw = yaw_from_odom(start)
        projection = (
            (final.pose.pose.position.x - start.pose.pose.position.x)
            * math.cos(start_yaw)
            + (final.pose.pose.position.y - start.pose.pose.position.y)
            * math.sin(start_yaw)
        )
        self.assertAlmostEqual(projection, distance, delta=0.05)

    def assert_rotate(self, angle: float):
        start = self.latest_sample(self.odometry)[1]
        _, wrapped = self.execute_goal([self.rotate_step(angle)])
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.code, ExecuteMission.Result.SUCCEEDED)
        final = self.latest_sample(self.odometry)[1]
        self.assertAlmostEqual(
            wrapped_angle(yaw_from_odom(final) - yaw_from_odom(start)),
            angle,
            delta=0.08,
        )

    def test_headless_move_rotate_multistep_cancel_stop_and_obstacle(self):
        self.assert_move(0.50)
        self.assert_move(-0.50)
        self.assert_rotate(1.5708)
        self.assert_rotate(-1.5708)

        leases_before = {
            sample[1].lease_id
            for sample in self.sample_snapshot(self.gate_states)
            if sample[1].state == InternalMotionGateState.ARMED
            and sample[1].lease_id
        }
        multi_steps = [
            self.move_step(0.25),
            self.rotate_step(0.50),
            self.rotate_step(-0.50),
        ]
        _, multi_result = self.execute_goal(multi_steps)
        self.assertEqual(multi_result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(
            multi_result.result.code, ExecuteMission.Result.SUCCEEDED
        )
        self.assert_raw_scan_age_under_limit()
        leases_after = {
            sample[1].lease_id
            for sample in self.sample_snapshot(self.gate_states)
            if sample[1].state == InternalMotionGateState.ARMED
            and sample[1].lease_id
        }
        self.assertGreaterEqual(len(leases_after - leases_before), 3)

        self.assert_rotate(3.0)
        self.assert_rotate(0.60)
        self.assert_rotate(-3.60)

        cancel_feedback = []
        cancel_goal = self.make_goal([self.move_step(1.0)])
        cancel_send = self.action_client.send_goal_async(
            cancel_goal,
            feedback_callback=lambda message: cancel_feedback.append(
                message.feedback
            ),
        )
        cancel_handle = self.wait_until(
            lambda: cancel_send.result() if cancel_send.done() else None,
            30.0,
            'cancel goal admission',
        )
        self.wait_until(
            lambda: cancel_feedback,
            5.0,
            'relative-motion feedback before cancel',
        )
        cancel_future = cancel_handle.cancel_goal_async()
        self.wait_until(
            lambda: cancel_future.result() if cancel_future.done() else None,
            5.0,
            'Action cancel acknowledgement',
        )
        canceled_result = cancel_handle.get_result_async()
        canceled = self.wait_until(
            lambda: (
                canceled_result.result() if canceled_result.done() else None
            ),
            15.0,
            'canceled Mission result',
        )
        self.assertEqual(canceled.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(canceled.result.code, ExecuteMission.Result.CANCELED)
        self.wait_for_stationary_zero()

        stop_goal = self.make_goal([self.move_step(1.0)])
        stop_send = self.action_client.send_goal_async(stop_goal)
        stop_handle = self.wait_until(
            lambda: stop_send.result() if stop_send.done() else None,
            30.0,
            'STOP goal admission',
        )
        self.assertTrue(stop_handle.accepted)
        self.wait_until(
            lambda: self.latest_sample(self.mission_states),
            5.0,
            'busy Mission state before STOP',
        )
        self.assertTrue(self.stop_client.wait_for_service(timeout_sec=5.0))
        stop_request = StopMission.Request()
        stop_request.request_id = '00000000000000000000000000000064'
        stop_request.source_instance_id = 'issue64-stop'
        stop_request.source_seq = 1
        stop_request.reason = 'Issue 64 headless STOP proof'
        stop_future = self.stop_client.call_async(stop_request)
        stop_response = self.wait_until(
            lambda: stop_future.result() if stop_future.done() else None,
            10.0,
            'STOP response',
        )
        self.assertEqual(stop_response.code, 0)
        self.assertTrue(stop_response.motion_inhibited)
        stopped_result = stop_handle.get_result_async()
        stopped = self.wait_until(
            lambda: stopped_result.result() if stopped_result.done() else None,
            15.0,
            'STOP Mission result',
        )
        self.assertEqual(stopped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(stopped.result.code, ExecuteMission.Result.STOPPED)
        self.wait_for_stationary_zero()

        obstacle_handle, obstacle = self.execute_goal([self.move_step(2.0)])
        self.assertEqual(obstacle_handle.accepted, True)
        self.assertEqual(obstacle.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            obstacle.result.code, ExecuteMission.Result.EXECUTION_FAILED
        )
        self.wait_for_stationary_zero()


@launch_testing.post_shutdown_test()
class RelativeMotionProductShutdownTest(unittest.TestCase):

    def test_all_launch_managed_processes_exit_cleanly(self, proc_info):
        assertExitCodes(proc_info, allowable_exit_codes=[
            0,
            -2,
        ])
