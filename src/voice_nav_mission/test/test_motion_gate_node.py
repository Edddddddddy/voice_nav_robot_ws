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
import math
import threading
import time
import unittest
import uuid

from geometry_msgs.msg import TwistStamped, Vector3
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.utilities import get_rmw_implementation_identifier
from voice_nav_mission.msg import InternalMotionGateState
from voice_nav_mission.srv import InternalMotionGateControl


CONTROL_SERVICE = '/motion_gate/internal/control'
STATE_TOPIC = '/motion_gate/internal/state'
FINAL_TOPIC = '/diff_drive_controller/cmd_vel'
CANDIDATE_PREFIX = '/voice_nav_internal/motion_gate/candidate/lease_'
WAIT_STEP_SECONDS = 0.005


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
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
    return (
        LaunchDescription(
            [
                motion_gate,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {'motion_gate': motion_gate},
    )


def candidate_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def state_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def request_id(prefix: str) -> str:
    # The IDL permits up to 36 characters; the locked private protocol
    # intentionally narrows this to one lowercase 32-hex representation.
    del prefix
    return uuid.uuid4().hex


def command(
    linear_x: float = 0.0,
    angular_z: float = 0.0,
    *,
    linear_y: float = 0.0,
) -> TwistStamped:
    message = TwistStamped()
    # Use a sentinel untrusted stamp. With frozen /clock, a correct Gate rewrite
    # produces zero rather than forwarding this value.
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.twist.linear.x = linear_x
    message.twist.linear.y = linear_y
    message.twist.angular.z = angular_z
    return message


class MotionGateNodeTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.authority = rclpy.create_node(
            'motion_gate_test_authority',
            parameter_overrides=[
                Parameter('use_sim_time', value=True),
            ],
            automatically_declare_parameters_from_overrides=True,
        )
        self.writer = rclpy.create_node('collision_monitor')
        self.controller = rclpy.create_node('diff_drive_controller')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.authority)
        self.executor.add_node(self.writer)
        self.executor.add_node(self.controller)
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )

        self.lock = threading.Lock()
        self.final_samples = deque(maxlen=10000)
        self.state_samples = deque(maxlen=1000)
        self.final_subscription = self.controller.create_subscription(
            TwistStamped,
            FINAL_TOPIC,
            self.on_final,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self.state_subscription = self.authority.create_subscription(
            InternalMotionGateState,
            STATE_TOPIC,
            self.on_state,
            state_qos(),
        )
        self.client = self.authority.create_client(
            InternalMotionGateControl,
            CONTROL_SERVICE,
        )
        self.parameter_client = AsyncParameterClient(
            self.authority,
            '/motion_gate_node',
        )
        self.candidate_publisher = None
        self.extra_nodes = []
        self.spin_thread.start()
        self.assertTrue(
            self.client.wait_for_service(timeout_sec=10.0),
            'MotionGate control service did not become available',
        )
        self.assertTrue(
            self.parameter_client.wait_for_services(timeout_sec=10.0),
            'MotionGate parameter services did not become available',
        )
        self.wait_until(
            lambda: self.latest_state(),
            5.0,
            'initial transient-local MotionGate state',
        )

    def tearDown(self):
        if self.candidate_publisher is not None:
            self.writer.destroy_publisher(self.candidate_publisher)
            self.candidate_publisher = None
        for node in self.extra_nodes:
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        for node in (self.authority, self.writer, self.controller):
            self.executor.remove_node(node)
            node.destroy_node()

    def on_final(self, message):
        with self.lock:
            self.final_samples.append((time.monotonic(), message))

    def on_state(self, message):
        with self.lock:
            self.state_samples.append((time.monotonic(), message))

    def latest_state(self):
        with self.lock:
            if not self.state_samples:
                return None
            return self.state_samples[-1][1]

    def final_snapshot(self):
        with self.lock:
            return tuple(self.final_samples)

    def wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(WAIT_STEP_SECONDS)
        self.fail(f'timed out waiting for {description}')

    def make_request(
        self,
        operation,
        state,
        *,
        lease_id='',
        request_id_value=None,
    ):
        request = InternalMotionGateControl.Request()
        request.operation = operation
        request.request_id = request_id_value or request_id('ctl')
        request.gate_instance_id = state.gate_instance_id
        request.expected_control_seq = state.control_seq
        request.lease_id = lease_id
        return request

    def call_request(self, request, timeout=5.0):
        future = self.client.call_async(request)
        self.wait_until(
            future.done,
            timeout,
            f'control operation {request.operation}',
        )
        self.assertIsNone(future.exception())
        return future.result()

    def call(self, operation, state, *, lease_id=''):
        request = self.make_request(
            operation,
            state,
            lease_id=lease_id,
        )
        return self.call_request(request)

    def wait_for_writer_graph(self, topic, expected_count):
        def count_matches():
            endpoints = self.writer.get_publishers_info_by_topic(topic)
            return len(endpoints) == expected_count

        self.wait_until(
            count_matches,
            3.0,
            f'{expected_count} candidate writer(s) on {topic}',
        )

    def wait_for_final_controller_graph(self, expected_count):
        def count_matches():
            endpoints = self.controller.get_subscriptions_info_by_topic(
                FINAL_TOPIC,
            )
            matching = [
                endpoint
                for endpoint in endpoints
                if (
                    endpoint.node_name == 'diff_drive_controller'
                    and endpoint.node_namespace == '/'
                )
            ]
            return len(matching) == expected_count

        self.wait_until(
            count_matches,
            3.0,
            f'{expected_count} final controller subscription(s)',
        )

    def prepare(self):
        state = self.latest_state()
        response = self.call(
            InternalMotionGateControl.Request.PREPARE,
            state,
        )
        self.assertEqual(
            response.code,
            InternalMotionGateControl.Response.APPLIED,
        )
        self.assertEqual(
            response.state,
            InternalMotionGateState.PREPARED,
        )
        self.assertTrue(response.motion_inhibited)
        self.assertTrue(response.zero_selected)
        self.assertTrue(response.zero_published)
        self.assertTrue(response.candidate_topic.startswith(CANDIDATE_PREFIX))
        self.assertLessEqual(len(response.lease_id), 36)
        self.assertLessEqual(len(response.candidate_topic), 128)
        return response

    def create_writer_and_open(self, prepared):
        self.candidate_publisher = self.writer.create_publisher(
            TwistStamped,
            prepared.candidate_topic,
            candidate_qos(),
        )
        self.wait_for_writer_graph(prepared.candidate_topic, 1)
        opened = self.call(
            InternalMotionGateControl.Request.OPEN,
            prepared,
            lease_id=prepared.lease_id,
        )
        self.assertEqual(
            opened.code,
            InternalMotionGateControl.Response.APPLIED,
        )
        self.assertEqual(opened.state, InternalMotionGateState.ARMED)
        self.assertTrue(opened.writer_bound)
        self.assertTrue(any(opened.bound_writer_gid))
        self.assertFalse(opened.motion_inhibited)
        self.assertTrue(opened.zero_selected)
        return opened

    def destroy_candidate_writer(self, topic):
        self.writer.destroy_publisher(self.candidate_publisher)
        self.candidate_publisher = None
        self.wait_for_writer_graph(topic, 0)

    def publish_for(self, message, duration, period=0.005):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.candidate_publisher.publish(message)
            time.sleep(period)

    def first_final_after(self, since, predicate):
        for observed, message in self.final_snapshot():
            if observed >= since and predicate(message):
                return observed, message
        return None

    def assert_only_zero_after(self, since, duration):
        deadline = time.monotonic() + duration
        self.wait_until(
            lambda: any(
                observed >= since
                for observed, _ in self.final_snapshot()
            ),
            min(duration, 1.0),
            'the first Gate output in the observation window',
        )
        while time.monotonic() < deadline:
            samples = [
                message
                for observed, message in self.final_snapshot()
                if observed >= since
            ]
            self.assertTrue(samples, 'expected continuing 50 Hz Gate output')
            for message in samples:
                self.assertEqual(message.twist.linear.x, 0.0)
                self.assertEqual(message.twist.angular.z, 0.0)
            time.sleep(0.01)

    def assert_only_nonzero_after(self, since, duration):
        deadline = time.monotonic() + duration
        self.wait_until(
            lambda: any(
                observed >= since
                for observed, _ in self.final_snapshot()
            ),
            min(duration, 1.0),
            'the first non-zero Gate output observation window',
        )
        while time.monotonic() < deadline:
            samples = [
                message
                for observed, message in self.final_snapshot()
                if observed >= since
            ]
            self.assertTrue(samples, 'expected continuing 50 Hz Gate output')
            for message in samples:
                self.assertGreater(
                    abs(message.twist.linear.x)
                    + abs(message.twist.angular.z),
                    0.0,
                )
            time.sleep(0.01)

    def add_extra_node(self, name):
        node = rclpy.create_node(name)
        self.executor.add_node(node)
        self.extra_nodes.append(node)
        return node

    def test_steady_fail_closed_protocol_without_clock(self):
        self.assertEqual(
            get_rmw_implementation_identifier(),
            'rmw_fastrtps_cpp',
        )

        # ROS time is intentionally frozen: this process launches no /clock
        # publisher, while Gate output and expiry must still advance.
        self.assertEqual(
            self.authority.count_publishers('/clock'),
            0,
        )
        initial = self.latest_state()
        self.assertEqual(initial.state, InternalMotionGateState.INHIBITED)
        self.assertTrue(initial.motion_inhibited)
        self.assertTrue(initial.zero_selected)

        # Invalid OPEN is rejected before the adapter queries an empty topic or
        # disturbs any reader.
        invalid_open = self.call(
            InternalMotionGateControl.Request.OPEN,
            initial,
            lease_id=request_id('fake-lease'),
        )
        self.assertEqual(
            invalid_open.code,
            InternalMotionGateControl.Response.REJECTED,
        )
        self.assertEqual(
            invalid_open.reason,
            InternalMotionGateControl.Response.INVALID_STATE,
        )
        self.assertEqual(invalid_open.control_seq, initial.control_seq)
        self.assertEqual(invalid_open.state, InternalMotionGateState.INHIBITED)

        start = time.monotonic()
        self.assert_only_zero_after(start, 0.08)
        zero_samples = [
            message
            for observed, message in self.final_snapshot()
            if observed >= start
        ]
        self.assertGreaterEqual(len(zero_samples), 3)
        self.assertTrue(
            all(
                message.header.stamp.sec == 0
                and message.header.stamp.nanosec == 0
                for message in zero_samples
            ),
            'without /clock, rewritten ROS stamps should remain frozen',
        )

        # A PREPARED generation is immutable across the complete negative graph
        # matrix: zero, wrong FQN, wrong QoS, wrong type, and two writers.
        prepared = self.prepare()
        prepared_identity = (
            prepared.control_seq,
            prepared.lease_id,
            prepared.candidate_topic,
        )

        def assert_open_rejected_without_mutation(response, reason):
            print(
                'Gate writer observation: '
                f'expected_reason={reason} actual_reason={response.reason} '
                f'detail={response.detail!r}',
                flush=True,
            )
            self.assertEqual(
                response.code,
                InternalMotionGateControl.Response.REJECTED,
            )
            self.assertEqual(response.reason, reason)
            self.assertEqual(response.state, InternalMotionGateState.PREPARED)
            self.assertEqual(
                (
                    response.control_seq,
                    response.lease_id,
                    response.candidate_topic,
                ),
                prepared_identity,
            )
            self.assertTrue(response.motion_inhibited)
            self.assertTrue(response.zero_selected)

        zero_writer = self.call(
            InternalMotionGateControl.Request.OPEN,
            prepared,
            lease_id=prepared.lease_id,
        )
        assert_open_rejected_without_mutation(
            zero_writer,
            InternalMotionGateControl.Response.WRITER_UNAVAILABLE,
        )

        wrong_fqn_node = self.add_extra_node('unexpected_candidate_writer')
        wrong_fqn_publisher = wrong_fqn_node.create_publisher(
            TwistStamped,
            prepared.candidate_topic,
            candidate_qos(),
        )
        self.wait_for_writer_graph(prepared.candidate_topic, 1)
        wrong_fqn = self.call(
            InternalMotionGateControl.Request.OPEN,
            prepared,
            lease_id=prepared.lease_id,
        )
        assert_open_rejected_without_mutation(
            wrong_fqn,
            InternalMotionGateControl.Response.WRITER_MISMATCH,
        )
        wrong_fqn_node.destroy_publisher(wrong_fqn_publisher)
        self.wait_for_writer_graph(prepared.candidate_topic, 0)

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        wrong_qos_publisher = self.writer.create_publisher(
            TwistStamped,
            prepared.candidate_topic,
            reliable_qos,
        )
        self.wait_for_writer_graph(prepared.candidate_topic, 1)
        wrong_qos = self.call(
            InternalMotionGateControl.Request.OPEN,
            prepared,
            lease_id=prepared.lease_id,
        )
        assert_open_rejected_without_mutation(
            wrong_qos,
            InternalMotionGateControl.Response.WRITER_MISMATCH,
        )
        self.writer.destroy_publisher(wrong_qos_publisher)
        self.wait_for_writer_graph(prepared.candidate_topic, 0)

        retained_qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        retained_qos_publisher = self.writer.create_publisher(
            TwistStamped,
            prepared.candidate_topic,
            retained_qos_profile,
        )
        self.wait_for_writer_graph(prepared.candidate_topic, 1)
        retained_qos_publisher.publish(command(0.30))
        retained_qos = self.call(
            InternalMotionGateControl.Request.OPEN,
            prepared,
            lease_id=prepared.lease_id,
        )
        assert_open_rejected_without_mutation(
            retained_qos,
            InternalMotionGateControl.Response.WRITER_MISMATCH,
        )
        self.writer.destroy_publisher(retained_qos_publisher)
        self.wait_for_writer_graph(prepared.candidate_topic, 0)

        wrong_type_publisher = self.writer.create_publisher(
            Vector3,
            prepared.candidate_topic,
            candidate_qos(),
        )
        self.wait_for_writer_graph(prepared.candidate_topic, 1)
        wrong_type = self.call(
            InternalMotionGateControl.Request.OPEN,
            prepared,
            lease_id=prepared.lease_id,
        )
        assert_open_rejected_without_mutation(
            wrong_type,
            InternalMotionGateControl.Response.WRITER_MISMATCH,
        )
        self.writer.destroy_publisher(wrong_type_publisher)
        self.wait_for_writer_graph(prepared.candidate_topic, 0)

        self.candidate_publisher = self.writer.create_publisher(
            TwistStamped,
            prepared.candidate_topic,
            candidate_qos(),
        )
        self.wait_for_writer_graph(prepared.candidate_topic, 1)

        # OPEN also requires the exact final controller endpoint and compatible
        # QoS; a valid candidate writer alone cannot arm the Gate.
        self.controller.destroy_subscription(self.final_subscription)
        self.final_subscription = None
        self.wait_for_final_controller_graph(0)
        controller_absent = self.call(
            InternalMotionGateControl.Request.OPEN,
            prepared,
            lease_id=prepared.lease_id,
        )
        assert_open_rejected_without_mutation(
            controller_absent,
            InternalMotionGateControl.Response.WRITER_UNAVAILABLE,
        )

        incompatible_controller_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            deadline=Duration(seconds=1.0),
        )
        self.final_subscription = self.controller.create_subscription(
            TwistStamped,
            FINAL_TOPIC,
            self.on_final,
            incompatible_controller_qos,
        )
        self.wait_for_final_controller_graph(1)
        controller_incompatible = self.call(
            InternalMotionGateControl.Request.OPEN,
            prepared,
            lease_id=prepared.lease_id,
        )
        assert_open_rejected_without_mutation(
            controller_incompatible,
            InternalMotionGateControl.Response.WRITER_UNAVAILABLE,
        )
        self.controller.destroy_subscription(self.final_subscription)
        self.final_subscription = self.controller.create_subscription(
            TwistStamped,
            FINAL_TOPIC,
            self.on_final,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self.wait_for_final_controller_graph(1)

        second_writer_node = self.add_extra_node('second_candidate_writer')
        second_writer_publisher = second_writer_node.create_publisher(
            TwistStamped,
            prepared.candidate_topic,
            candidate_qos(),
        )
        self.wait_for_writer_graph(prepared.candidate_topic, 2)
        ambiguous = self.call(
            InternalMotionGateControl.Request.OPEN,
            prepared,
            lease_id=prepared.lease_id,
        )
        assert_open_rejected_without_mutation(
            ambiguous,
            InternalMotionGateControl.Response.WRITER_AMBIGUOUS,
        )
        second_writer_node.destroy_publisher(second_writer_publisher)
        self.wait_for_writer_graph(prepared.candidate_topic, 1)

        # PREPARE owns a provisional discarding reader. Samples queued before
        # OPEN cannot leak through the destroyed/recreated queue.
        self.wait_until(
            lambda: self.candidate_publisher.get_subscription_count() == 1,
            3.0,
            'candidate writer matched to provisional discard reader',
        )
        self.publish_for(command(0.25), 0.05)
        open_started = time.monotonic()
        opened = self.call(
            InternalMotionGateControl.Request.OPEN,
            prepared,
            lease_id=prepared.lease_id,
        )
        self.assertEqual(
            opened.state,
            InternalMotionGateState.ARMED,
            (
                f'OPEN failed: code={opened.code} reason={opened.reason} '
                f'detail={opened.detail!r}'
            ),
        )
        self.assertTrue(any(opened.bound_writer_gid))
        self.assert_only_zero_after(open_started, 0.08)

        # The bound Gate-local graph GID now admits fresh samples. Finite
        # supported axes are clamped, and the outgoing stamp is Gate ROS time.
        clamp_started = time.monotonic()
        self.publish_for(command(2.0, -2.0), 0.06)
        clamped = self.wait_until(
            lambda: self.first_final_after(
                clamp_started,
                lambda message: (
                    math.isclose(message.twist.linear.x, 0.40)
                    and math.isclose(message.twist.angular.z, -1.20)
                ),
            ),
            1.0,
            'clamped candidate on the final command endpoint',
        )
        self.assertEqual(clamped[1].header.stamp.sec, 0)
        self.assertEqual(clamped[1].header.stamp.nanosec, 0)
        bound_state = self.wait_until(
            lambda: (
                self.latest_state()
                if (
                    self.latest_state().state
                    == InternalMotionGateState.ARMED
                    and self.latest_state().candidate_fresh
                    and self.latest_state().writer_bound
                )
                else None
            ),
            1.0,
            'fresh bound writer state',
        )
        self.assertEqual(
            bytes(bound_state.bound_writer_gid),
            bytes(opened.bound_writer_gid),
        )

        # Invalid unsupported axes retire the lease immediately.
        invalid_started = time.monotonic()
        self.candidate_publisher.publish(command(0.2, linear_y=0.01))
        invalid_state = self.wait_until(
            lambda: (
                self.latest_state()
                if (
                    self.latest_state().state
                    == InternalMotionGateState.INHIBITED
                    and self.latest_state().reason
                    == InternalMotionGateState.INVALID_CANDIDATE
                )
                else None
            ),
            1.0,
            'invalid-axis retirement',
        )
        self.assertTrue(invalid_state.motion_inhibited)
        self.wait_until(
            lambda: self.first_final_after(
                invalid_started,
                lambda message: (
                    message.twist.linear.x == 0.0
                    and message.twist.angular.z == 0.0
                ),
            ),
            0.3,
            'zero after invalid candidate',
        )
        retired_topic = prepared.candidate_topic
        self.destroy_candidate_writer(retired_topic)

        # A second publisher GID on the accepting reader retires the current
        # generation. The old bound writer then blocks PREPARE until it leaves
        # the graph; the cached retry must not repeat the one-second barrier.
        prepared = self.prepare()
        opened = self.create_writer_and_open(prepared)
        second_gid_started = time.monotonic()
        self.publish_for(command(0.20), 0.04)
        self.wait_until(
            lambda: self.first_final_after(
                second_gid_started,
                lambda message: message.twist.linear.x > 0.0,
            ),
            1.0,
            'motion before second-GID injection',
        )
        same_fqn_node = self.add_extra_node('collision_monitor')
        second_gid_publisher = same_fqn_node.create_publisher(
            TwistStamped,
            prepared.candidate_topic,
            candidate_qos(),
        )
        self.wait_for_writer_graph(prepared.candidate_topic, 2)
        for _ in range(10):
            second_gid_publisher.publish(command(0.30))
            time.sleep(0.005)
        mismatch_state = self.wait_until(
            lambda: (
                self.latest_state()
                if (
                    self.latest_state().state
                    == InternalMotionGateState.INHIBITED
                    and self.latest_state().reason
                    == InternalMotionGateState.WRITER_MISMATCH
                )
                else None
            ),
            1.0,
            'second-GID fail-closed retirement',
        )
        mismatch_zero_started = time.monotonic()
        self.assert_only_zero_after(mismatch_zero_started, 0.06)

        blocked_prepare_request = self.make_request(
            InternalMotionGateControl.Request.PREPARE,
            mismatch_state,
        )
        blocked_started = time.monotonic()
        blocked_prepare = self.call_request(
            blocked_prepare_request,
            timeout=3.0,
        )
        blocked_elapsed = time.monotonic() - blocked_started
        self.assertGreaterEqual(blocked_elapsed, 0.80)
        self.assertEqual(
            blocked_prepare.code,
            InternalMotionGateControl.Response.REJECTED,
        )
        self.assertEqual(
            blocked_prepare.reason,
            InternalMotionGateControl.Response.WRITER_STILL_PRESENT,
        )
        duplicate_started = time.monotonic()
        blocked_duplicate = self.call_request(blocked_prepare_request)
        self.assertLess(time.monotonic() - duplicate_started, 0.30)
        self.assertEqual(
            blocked_duplicate.code,
            InternalMotionGateControl.Response.DUPLICATE,
        )
        self.assertEqual(
            blocked_duplicate.reason,
            InternalMotionGateControl.Response.WRITER_STILL_PRESENT,
        )

        self.writer.destroy_publisher(self.candidate_publisher)
        self.candidate_publisher = None
        self.wait_for_writer_graph(prepared.candidate_topic, 1)
        same_fqn_node.destroy_publisher(second_gid_publisher)
        self.wait_for_writer_graph(prepared.candidate_topic, 0)

        prepared = self.prepare()
        cross_operation_collision = self.make_request(
            InternalMotionGateControl.Request.OPEN,
            prepared,
            lease_id=prepared.lease_id,
            request_id_value=blocked_prepare_request.request_id,
        )
        collision = self.call_request(cross_operation_collision)
        self.assertEqual(
            collision.reason,
            InternalMotionGateControl.Response.REQUEST_ID_COLLISION,
        )
        self.assertEqual(collision.state, InternalMotionGateState.PREPARED)
        self.assertEqual(collision.lease_id, prepared.lease_id)

        # A valid current-tuple INHIBIT is a publication barrier. An old
        # idempotent retry later must report the current B generation and must
        # not insert a zero pulse into B.
        opened = self.create_writer_and_open(prepared)
        moving_a_started = time.monotonic()
        self.publish_for(command(0.25), 0.04)
        self.wait_until(
            lambda: self.first_final_after(
                moving_a_started,
                lambda message: message.twist.linear.x > 0.0,
            ),
            1.0,
            'lease A motion before INHIBIT',
        )
        old_inhibit_request = self.make_request(
            InternalMotionGateControl.Request.INHIBIT,
            opened,
            lease_id=opened.lease_id,
        )
        inhibited = self.call_request(old_inhibit_request)
        acknowledged = time.monotonic()
        self.assertEqual(
            inhibited.code,
            InternalMotionGateControl.Response.APPLIED,
        )
        self.assertTrue(inhibited.motion_inhibited)
        self.assertTrue(inhibited.zero_published)
        self.assertEqual(
            inhibited.zero_publish_seq,
            inhibited.output_publish_seq,
        )
        self.assert_only_zero_after(acknowledged, 0.06)
        retired_topic = prepared.candidate_topic
        self.destroy_candidate_writer(retired_topic)

        prepared_b = self.prepare()
        opened_b = self.create_writer_and_open(prepared_b)
        flood_stop = threading.Event()

        def flood():
            while not flood_stop.is_set():
                self.candidate_publisher.publish(command(0.25))
                time.sleep(0.001)

        flood_thread = threading.Thread(target=flood, daemon=True)
        flood_thread.start()
        try:
            moving_b = self.wait_until(
                lambda: self.first_final_after(
                    time.monotonic() - 1.0,
                    lambda message: message.twist.linear.x > 0.0,
                ),
                1.0,
                'lease B motion before old retry',
            )
            del moving_b

            old_retry_started = time.monotonic()
            old_retry = self.call_request(old_inhibit_request)
            self.assertEqual(
                old_retry.code,
                InternalMotionGateControl.Response.DUPLICATE,
            )
            self.assertEqual(old_retry.state, InternalMotionGateState.ARMED)
            self.assertEqual(old_retry.lease_id, opened_b.lease_id)
            self.assertFalse(old_retry.motion_inhibited)
            self.assertFalse(old_retry.zero_selected)
            self.assertFalse(old_retry.zero_published)
            self.assert_only_nonzero_after(old_retry_started, 0.04)

            stale_prepare = self.make_request(
                InternalMotionGateControl.Request.PREPARE,
                self.latest_state(),
            )
            stale_prepare.expected_control_seq -= 1
            stale_prepare_started = time.monotonic()
            stale_prepare_response = self.call_request(stale_prepare)
            self.assertEqual(
                stale_prepare_response.reason,
                InternalMotionGateControl.Response.STALE_SEQUENCE,
            )
            self.assertEqual(
                stale_prepare_response.lease_id,
                opened_b.lease_id,
            )
            self.assert_only_nonzero_after(stale_prepare_started, 0.04)

            invalid_open_request = self.make_request(
                InternalMotionGateControl.Request.OPEN,
                self.latest_state(),
                lease_id=opened_b.lease_id,
            )
            invalid_open_started = time.monotonic()
            invalid_open_response = self.call_request(invalid_open_request)
            self.assertEqual(
                invalid_open_response.reason,
                InternalMotionGateControl.Response.INVALID_STATE,
            )
            self.assertEqual(
                invalid_open_response.lease_id,
                opened_b.lease_id,
            )
            self.assert_only_nonzero_after(invalid_open_started, 0.04)

            # Fresh request IDs with stale CAS fields must be side-effect free
            # at the Node adapter boundary. In particular, rejected INHIBIT
            # must not insert a transient zero into active generation B.
            stale_control_cases = (
                (
                    InternalMotionGateControl.Request.RENEW,
                    'gate',
                    InternalMotionGateControl.Response.STALE_GATE,
                ),
                (
                    InternalMotionGateControl.Request.RENEW,
                    'sequence',
                    InternalMotionGateControl.Response.STALE_SEQUENCE,
                ),
                (
                    InternalMotionGateControl.Request.RENEW,
                    'lease',
                    InternalMotionGateControl.Response.STALE_LEASE,
                ),
                (
                    InternalMotionGateControl.Request.INHIBIT,
                    'gate',
                    InternalMotionGateControl.Response.STALE_GATE,
                ),
                (
                    InternalMotionGateControl.Request.INHIBIT,
                    'sequence',
                    InternalMotionGateControl.Response.STALE_SEQUENCE,
                ),
                (
                    InternalMotionGateControl.Request.INHIBIT,
                    'lease',
                    InternalMotionGateControl.Response.STALE_LEASE,
                ),
            )
            for operation, stale_field, expected_reason in stale_control_cases:
                with self.subTest(
                    operation=operation,
                    stale_field=stale_field,
                ):
                    current_b = self.latest_state()
                    stale_request = self.make_request(
                        operation,
                        current_b,
                        lease_id=current_b.lease_id,
                    )
                    if stale_field == 'gate':
                        stale_request.gate_instance_id = request_id('gate')
                    elif stale_field == 'sequence':
                        stale_request.expected_control_seq -= 1
                    else:
                        stale_request.lease_id = request_id('lease')

                    stale_started = time.monotonic()
                    stale_response = self.call_request(stale_request)
                    self.assertEqual(stale_response.reason, expected_reason)
                    self.assertEqual(
                        stale_response.lease_id,
                        opened_b.lease_id,
                    )
                    self.assertFalse(stale_response.zero_published)
                    self.assert_only_nonzero_after(stale_started, 0.025)

                    # Candidate traffic never renews authority. Refresh the
                    # independent authority deadline between matrix rows.
                    renewed = self.call(
                        InternalMotionGateControl.Request.RENEW,
                        self.latest_state(),
                        lease_id=opened_b.lease_id,
                    )
                    self.assertEqual(
                        renewed.code,
                        InternalMotionGateControl.Response.APPLIED,
                    )

            # use_sim_time is a safety-relevant runtime invariant. A mutation
            # while moving must be rejected before rclcpp switches the clock;
            # otherwise the controller could receive future system timestamps
            # and its consumer-side timeout would no longer be dependable.
            parameter_change_started = time.monotonic()
            set_future = self.parameter_client.set_parameters(
                [Parameter('use_sim_time', value=False)],
            )
            self.wait_until(
                set_future.done,
                2.0,
                'use_sim_time mutation rejection',
            )
            self.assertIsNone(set_future.exception())
            self.assertEqual(len(set_future.result().results), 1)
            self.assertFalse(set_future.result().results[0].successful)
            self.assertIn(
                'immutable',
                set_future.result().results[0].reason,
            )
            get_future = self.parameter_client.get_parameters(
                ['use_sim_time'],
            )
            self.wait_until(
                get_future.done,
                2.0,
                'use_sim_time value after rejected mutation',
            )
            self.assertIsNone(get_future.exception())
            self.assertTrue(get_future.result().values[0].bool_value)
            self.assert_only_nonzero_after(
                parameter_change_started,
                0.04,
            )
            post_mutation_samples = [
                message
                for observed, message in self.final_snapshot()
                if observed >= parameter_change_started
            ]
            self.assertTrue(post_mutation_samples)
            self.assertTrue(
                all(
                    message.header.stamp.sec == 0
                    and message.header.stamp.nanosec == 0
                    for message in post_mutation_samples
                ),
                'rejected mutation must preserve frozen ROS-time stamps',
            )

            current_b = self.latest_state()
            current_inhibit = self.call(
                InternalMotionGateControl.Request.INHIBIT,
                current_b,
                lease_id=current_b.lease_id,
            )
            final_acknowledged = time.monotonic()
            self.assertTrue(current_inhibit.motion_inhibited)
            self.assertTrue(current_inhibit.zero_published)
            self.assert_only_zero_after(final_acknowledged, 0.06)
        finally:
            flood_stop.set()
            flood_thread.join(timeout=1.0)
        retired_topic = prepared_b.candidate_topic
        self.destroy_candidate_writer(retired_topic)

        # Authority expiry is driven by steady time. Valid candidates continue
        # arriving, but without RENEW the Gate must reach zero within 300 ms.
        prepared = self.prepare()
        opened = self.create_writer_and_open(prepared)
        opened_at = time.monotonic()
        expiry_stop = threading.Event()

        def keep_candidates_alive():
            while not expiry_stop.is_set():
                self.candidate_publisher.publish(command(0.20))
                time.sleep(0.005)

        expiry_thread = threading.Thread(
            target=keep_candidates_alive,
            daemon=True,
        )
        expiry_thread.start()
        try:
            first_nonzero = self.wait_until(
                lambda: self.first_final_after(
                    opened_at,
                    lambda message: message.twist.linear.x > 0.0,
                ),
                1.0,
                'non-zero command before authority expiry',
            )
            expired_state = self.wait_until(
                lambda: (
                    self.latest_state()
                    if (
                        self.latest_state().state
                        == InternalMotionGateState.INHIBITED
                        and self.latest_state().reason
                        == InternalMotionGateState.AUTHORITY_EXPIRED
                    )
                    else None
                ),
                0.35,
                'steady authority expiry despite candidate flood',
            )
            zero_observation = self.wait_until(
                lambda: self.first_final_after(
                    first_nonzero[0],
                    lambda message: (
                        message.twist.linear.x == 0.0
                        and message.twist.angular.z == 0.0
                    ),
                ),
                0.35,
                'zero selected by steady authority expiry',
            )
            self.assertLessEqual(
                zero_observation[0] - opened_at,
                0.30,
            )
            self.assertTrue(expired_state.motion_inhibited)
        finally:
            expiry_stop.set()
            expiry_thread.join(timeout=1.0)


@launch_testing.post_shutdown_test()
class MotionGateNodeShutdownTest(unittest.TestCase):

    def test_motion_gate_exits_cleanly(self, proc_info, motion_gate):
        assertExitCodes(proc_info, process=motion_gate)
