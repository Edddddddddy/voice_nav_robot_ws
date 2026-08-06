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
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_system_default,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.utilities import get_rmw_implementation_identifier
from voice_nav_mission.msg import InternalMotionGateState
from voice_nav_mission.srv import InternalMotionGateControl


CONTROL_SERVICE = '/motion_gate/internal/control'
STATE_TOPIC = '/motion_gate/internal/state'
FINAL_COMMAND_TOPIC = '/diff_drive_controller/cmd_vel'
LIMITED_COMMAND_TOPIC = '/diff_drive_controller/cmd_vel_out'
ODOMETRY_TOPIC = '/odom'

COMMAND_EPSILON = 1e-6
STATIONARY_LINEAR_TOLERANCE = 0.02
STATIONARY_ANGULAR_TOLERANCE = 0.02
MOVING_LINEAR_TOLERANCE = 0.03
MOVING_ANGULAR_TOLERANCE = 0.08
OPEN_CONVERGENCE_BUDGET_SECONDS = 1.0


def load_open_convergence_support():
    support_path = Path(__file__).with_name(
        'motion_gate_open_convergence.py'
    )
    specification = importlib.util.spec_from_file_location(
        'motion_gate_open_convergence',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load MotionGate OPEN convergence support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def load_gazebo_shutdown_support():
    support_path = (
        Path(get_package_share_directory('voice_nav_sim'))
        / 'test_support'
        / 'gazebo_shutdown.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_motion_gate_product_gazebo_shutdown',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load Gazebo shutdown test support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


open_convergence = load_open_convergence_support()
gazebo_shutdown = load_gazebo_shutdown_support()
PRODUCT_TEST_PARTITION = (
    gazebo_shutdown.claim_unique_test_partition(
        'l0009_motion_gate_product'
    )
)
OPEN_PROTOCOL = open_convergence.ProtocolValues(
    applied=InternalMotionGateControl.Response.APPLIED,
    rejected=InternalMotionGateControl.Response.REJECTED,
    writer_unavailable=(
        InternalMotionGateControl.Response.WRITER_UNAVAILABLE
    ),
    writer_metadata_pending=(
        InternalMotionGateControl.Response.WRITER_METADATA_PENDING
    ),
    prepared=InternalMotionGateState.PREPARED,
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
    return LaunchDescription(
        [
            product,
            launch_testing.actions.ReadyToTest(),
        ]
    )


def is_zero(message: TwistStamped) -> bool:
    twist = message.twist
    return all(
        abs(value) <= COMMAND_EPSILON
        for value in (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
    )


class MotionGateProductTest(unittest.TestCase):
    def setUp(self, proc_info):
        self.addCleanup(self.destroy_ros_fixture)
        self.addCleanup(
            gazebo_shutdown.structured_stop_gazebo,
            proc_info,
            expected_partition=PRODUCT_TEST_PARTITION,
        )
        self.addCleanup(self.inhibit_for_cleanup)
        rclpy.init()
        self.node = rclpy.create_node(
            'collision_monitor',
            parameter_overrides=[
                Parameter('use_sim_time', value=True),
            ],
            automatically_declare_parameters_from_overrides=True,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.samples_lock = threading.Lock()
        self.states = deque(maxlen=1000)
        self.final_commands = deque(maxlen=4000)
        self.limited_commands = deque(maxlen=4000)
        self.odometry = deque(maxlen=4000)
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscriptions = [
            self.node.create_subscription(
                InternalMotionGateState,
                STATE_TOPIC,
                lambda message: self.append_sample(
                    self.states,
                    message,
                ),
                state_qos,
            ),
            self.node.create_subscription(
                TwistStamped,
                FINAL_COMMAND_TOPIC,
                lambda message: self.append_sample(
                    self.final_commands,
                    message,
                ),
                qos_profile_system_default,
            ),
            self.node.create_subscription(
                TwistStamped,
                LIMITED_COMMAND_TOPIC,
                lambda message: self.append_sample(
                    self.limited_commands,
                    message,
                ),
                100,
            ),
            self.node.create_subscription(
                Odometry,
                ODOMETRY_TOPIC,
                lambda message: self.append_sample(
                    self.odometry,
                    message,
                ),
                100,
            ),
        ]
        self.control_client = self.node.create_client(
            InternalMotionGateControl,
            CONTROL_SERVICE,
        )
        self.candidate_publishers = []
        self.request_counter = 0
        self.current_lease_id = ''
        self.current_candidate_topic = ''
        self.current_control_seq = 0
        self.current_gate_instance_id = ''
        self.open_convergence_deadline = 0.0
        self.spin_thread.start()

    def destroy_ros_fixture(self):
        node = getattr(self, 'node', None)
        steps = []
        for index, publisher in enumerate(tuple(
            getattr(self, 'candidate_publishers', ())
        )):
            if node is not None:
                steps.append(
                    (
                        f'candidate publisher {index} destroy',
                        lambda publisher=publisher: (
                            node.destroy_publisher(publisher)
                        ),
                    )
                )
        executor = getattr(self, 'executor', None)
        if executor is not None:
            steps.append(
                (
                    'executor shutdown',
                    lambda: executor.shutdown(timeout_sec=2.0),
                )
            )
        spin_thread = getattr(self, 'spin_thread', None)
        if spin_thread is not None:
            steps.append(
                (
                    'spin thread join',
                    lambda: gazebo_shutdown.join_started_thread(
                        spin_thread,
                        timeout_seconds=2.0,
                    ),
                )
            )
        if node is not None:
            steps.append(('node destroy', node.destroy_node))

        def shutdown_rclpy():
            if rclpy.ok():
                rclpy.shutdown()

        steps.append(('rclpy shutdown', shutdown_rclpy))
        gazebo_shutdown.run_cleanup_steps(
            'MotionGate product ROS fixture destruction failed',
            steps,
        )

    def append_sample(self, samples, message):
        with self.samples_lock:
            samples.append((time.monotonic(), message))

    def sample_snapshot(self, samples):
        with self.samples_lock:
            return tuple(samples)

    def latest_sample(self, samples):
        with self.samples_lock:
            return samples[-1] if samples else None

    def clear_samples(self, samples):
        with self.samples_lock:
            samples.clear()

    def wait_until(self, predicate, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.01)
        self.fail(f'timed out waiting for {description}')

    def call_control(self, request, timeout: float = 3.0):
        future = self.control_client.call_async(request)
        response = self.wait_until(
            lambda: future.result() if future.done() else None,
            timeout,
            'MotionGate control response',
        )
        return response

    def next_request_id(self, _prefix: str) -> str:
        self.request_counter += 1
        return f'{self.request_counter:032x}'

    def make_request(
        self,
        operation: int,
        prefix: str,
        *,
        request_id: str | None = None,
    ):
        request = InternalMotionGateControl.Request()
        request.operation = operation
        request.request_id = request_id or self.next_request_id(prefix)
        request.gate_instance_id = self.current_gate_instance_id
        request.expected_control_seq = self.current_control_seq
        request.lease_id = self.current_lease_id
        return request

    def update_authority(self, response):
        self.current_gate_instance_id = response.gate_instance_id
        self.current_control_seq = response.control_seq
        self.current_lease_id = response.lease_id
        self.current_candidate_topic = response.candidate_topic

    def prepare(self):
        state = self.wait_until(
            lambda: (
                sample[1]
                if (
                    (sample := self.latest_sample(self.states))
                    and sample[1].state
                    == InternalMotionGateState.INHIBITED
                )
                else None
            ),
            3.0,
            'inhibited MotionGate state',
        )
        self.current_gate_instance_id = state.gate_instance_id
        self.current_control_seq = state.control_seq
        self.current_lease_id = ''
        request = self.make_request(
            InternalMotionGateControl.Request.PREPARE,
            'prepare',
        )
        response = self.call_control(request)
        self.assertEqual(
            response.code,
            InternalMotionGateControl.Response.APPLIED,
            response.detail,
        )
        self.assertEqual(
            response.state,
            InternalMotionGateState.PREPARED,
        )
        self.assertTrue(response.motion_inhibited)
        self.assertTrue(response.zero_selected)
        self.assertTrue(response.lease_id)
        self.assertTrue(
            response.candidate_topic.startswith(
                '/voice_nav_internal/motion_gate/candidate/lease_'
            )
        )
        self.update_authority(response)
        return response

    def wait_for_candidate_writer_graph(self, topic: str):
        def observed_writer():
            endpoints = [
                endpoint
                for endpoint in self.node.get_publishers_info_by_topic(topic)
                if (
                    endpoint.node_name == 'collision_monitor'
                    and endpoint.node_namespace == '/'
                    and endpoint.topic_type == 'geometry_msgs/msg/TwistStamped'
                    and any(endpoint.endpoint_gid)
                )
            ]
            return endpoints[0] if len(endpoints) == 1 else None

        return self.wait_until(
            observed_writer,
            OPEN_CONVERGENCE_BUDGET_SECONDS,
            'candidate writer graph identity',
        )

    def open_gate(self):
        self.open_convergence_deadline = (
            time.monotonic() + OPEN_CONVERGENCE_BUDGET_SECONDS
        )
        expected = open_convergence.PreparedIdentity(
            gate_instance_id=self.current_gate_instance_id,
            control_seq=self.current_control_seq,
            lease_id=self.current_lease_id,
            candidate_topic=self.current_candidate_topic,
        )

        def attempt_open(request_id, remaining_seconds):
            request = self.make_request(
                InternalMotionGateControl.Request.OPEN,
                'open',
                request_id=request_id,
            )
            return self.call_control(request, timeout=remaining_seconds)

        response = open_convergence.converge_open(
            expected=expected,
            protocol=OPEN_PROTOCOL,
            attempt=attempt_open,
            new_request_id=lambda: self.next_request_id('open'),
            deadline=self.open_convergence_deadline,
        )
        self.assertEqual(
            response.code,
            InternalMotionGateControl.Response.APPLIED,
            response.detail,
        )
        self.assertEqual(response.state, InternalMotionGateState.ARMED)
        self.assertEqual(response.gate_instance_id, expected.gate_instance_id)
        self.assertEqual(response.control_seq, expected.control_seq + 1)
        self.assertEqual(response.lease_id, expected.lease_id)
        self.assertEqual(response.candidate_topic, expected.candidate_topic)
        self.assertFalse(response.motion_inhibited)
        self.assertTrue(response.authority_live)
        self.assertFalse(response.candidate_fresh)
        self.assertTrue(response.writer_bound)
        self.assertTrue(response.zero_selected)
        self.assertTrue(response.zero_published)
        self.assertTrue(any(response.bound_writer_gid))
        self.update_authority(response)
        return response

    def renew(self):
        request = self.make_request(
            InternalMotionGateControl.Request.RENEW,
            'renew',
        )
        response = self.call_control(request)
        self.assertEqual(
            response.code,
            InternalMotionGateControl.Response.APPLIED,
            response.detail,
        )
        self.assertEqual(response.state, InternalMotionGateState.ARMED)
        self.update_authority(response)
        return response

    def inhibit(self):
        request = self.make_request(
            InternalMotionGateControl.Request.INHIBIT,
            'inhibit',
        )
        started = time.monotonic()
        response = self.call_control(request)
        completed = time.monotonic()
        self.assertEqual(
            response.code,
            InternalMotionGateControl.Response.APPLIED,
            response.detail,
        )
        self.assertEqual(
            response.state,
            InternalMotionGateState.INHIBITED,
        )
        self.assertTrue(response.motion_inhibited)
        self.assertTrue(response.zero_selected)
        self.assertTrue(response.zero_published)
        self.update_authority(response)
        return started, completed, response

    def inhibit_for_cleanup(self):
        control_client = getattr(self, 'control_client', None)
        if (
            not rclpy.ok()
            or control_client is None
            or not control_client.service_is_ready()
            or not getattr(self, 'current_lease_id', '')
        ):
            return
        self.inhibit()

    def create_candidate_publisher(self, topic: str):
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        publisher = self.node.create_publisher(
            TwistStamped,
            topic,
            qos,
        )
        self.candidate_publishers.append(publisher)
        self.wait_until(
            lambda: publisher.get_subscription_count() == 1,
            3.0,
            'one MotionGate candidate reader',
        )
        self.wait_for_candidate_writer_graph(topic)
        return publisher

    def destroy_candidate_publisher(self, publisher, topic: str):
        self.node.destroy_publisher(publisher)
        self.candidate_publishers.remove(publisher)
        self.wait_until(
            lambda: not self.node.get_publishers_info_by_topic(topic),
            3.0,
            'old candidate writer to disappear from the graph',
        )

    def publish_candidate(
        self,
        publisher,
        linear_x: float,
        angular_z: float,
    ):
        message = TwistStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.twist.linear.x = linear_x
        message.twist.angular.z = angular_z
        publisher.publish(message)

    def publish_with_renewals(
        self,
        publisher,
        linear_x: float,
        angular_z: float,
        duration: float,
    ):
        deadline = time.monotonic() + duration
        next_renewal = time.monotonic()
        last_publish_at = 0.0
        while time.monotonic() < deadline:
            last_publish_at = time.monotonic()
            self.publish_candidate(
                publisher,
                linear_x,
                angular_z,
            )
            if time.monotonic() >= next_renewal:
                self.renew()
                next_renewal = time.monotonic() + 0.075
            time.sleep(0.02)
        return last_publish_at

    def wait_for_command(
        self,
        samples,
        predicate,
        after: float,
        timeout: float,
        description: str,
    ):
        return self.wait_until(
            lambda: next(
                (
                    sample
                    for sample in self.sample_snapshot(samples)
                    if sample[0] >= after and predicate(sample[1])
                ),
                None,
            ),
            timeout,
            description,
        )

    def wait_for_stationary_window(
        self,
        after: float,
        duration: float,
        timeout: float = 2.0,
    ):
        deadline = time.monotonic() + timeout
        stationary_since = None
        stationary_samples = 0
        last_sample_time = None
        while time.monotonic() < deadline:
            candidates = [
                sample
                for sample in self.sample_snapshot(self.odometry)
                if sample[0] >= after
                and (
                    last_sample_time is None
                    or sample[0] > last_sample_time
                )
            ]
            for receipt_time, message in candidates:
                if (
                    last_sample_time is not None
                    and receipt_time - last_sample_time > 0.10
                ):
                    stationary_since = None
                    stationary_samples = 0
                last_sample_time = receipt_time
                twist = message.twist.twist
                stationary = (
                    abs(twist.linear.x)
                    < STATIONARY_LINEAR_TOLERANCE
                    and abs(twist.angular.z)
                    < STATIONARY_ANGULAR_TOLERANCE
                )
                if stationary:
                    if stationary_since is None:
                        stationary_since = receipt_time
                        stationary_samples = 1
                    else:
                        stationary_samples += 1
                    if (
                        receipt_time - stationary_since >= duration
                        and stationary_samples >= 5
                    ):
                        return stationary_since, receipt_time
                else:
                    stationary_since = None
                    stationary_samples = 0
            time.sleep(0.01)
        self.fail(
            f'odometry did not remain stationary for {duration:.3f}s'
        )

    def wait_for_inhibited_state(
        self,
        after_state_seq: int,
        reason: int,
    ):
        return self.wait_until(
            lambda: next(
                (
                    sample
                    for sample in reversed(
                        self.sample_snapshot(self.states)
                    )
                    if sample[1].state
                    == InternalMotionGateState.INHIBITED
                    and sample[1].state_seq > after_state_seq
                    and sample[1].reason == reason
                    and sample[1].zero_selected
                ),
                None,
            ),
            2.0,
            'automatic inhibited state transition',
        )

    def wait_for_accepted_candidate(
        self,
        lease_id: str,
        after_state_seq: int,
        after: float,
    ):
        return self.wait_until(
            lambda: next(
                (
                    sample
                    for sample in reversed(
                        self.sample_snapshot(self.states)
                    )
                    if sample[0] >= after
                    and sample[1].state_seq > after_state_seq
                    and sample[1].state
                    == InternalMotionGateState.ARMED
                    and sample[1].lease_id == lease_id
                    and sample[1].authority_live
                    and sample[1].candidate_fresh
                    and not sample[1].zero_selected
                ),
                None,
            ),
            1.0,
            'accepted candidate state_seq handshake',
        )

    def wait_for_motion_evidence(self, after: float):
        final_nonzero = self.wait_for_command(
            self.final_commands,
            lambda message: not is_zero(message),
            after,
            1.0,
            'non-zero Gate output',
        )
        controller_nonzero = self.wait_for_command(
            self.limited_commands,
            lambda message: not is_zero(message),
            after,
            1.0,
            'non-zero controller output',
        )
        moving_odometry = self.wait_until(
            lambda: next(
                (
                    sample
                    for sample in self.sample_snapshot(self.odometry)
                    if sample[0] >= after
                    and (
                        abs(sample[1].twist.twist.linear.x)
                        >= MOVING_LINEAR_TOLERANCE
                        or abs(sample[1].twist.twist.angular.z)
                        >= MOVING_ANGULAR_TOLERANCE
                    )
                ),
                None,
            ),
            1.0,
            'moving odometry',
        )
        return final_nonzero, controller_nonzero, moving_odometry

    def assert_automatic_stop_evidence(
        self,
        *,
        measurement_started: float,
        accepted_state_seq: int,
        final_nonzero_at: float,
        controller_nonzero_at: float,
        reason: int,
        gate_zero_limit: float,
    ):
        _, expired_state = self.wait_for_inhibited_state(
            accepted_state_seq,
            reason,
        )
        self.assertTrue(expired_state.motion_inhibited)
        self.assertTrue(expired_state.zero_selected)
        self.assertFalse(expired_state.authority_live)
        self.assertFalse(expired_state.candidate_fresh)

        gate_zero = self.wait_for_command(
            self.final_commands,
            is_zero,
            final_nonzero_at,
            1.0,
            'Gate final zero after automatic inhibition',
        )
        controller_zero = self.wait_for_command(
            self.limited_commands,
            is_zero,
            controller_nonzero_at,
            1.0,
            'controller zero after automatic inhibition',
        )
        self.assertGreater(gate_zero[0], final_nonzero_at)
        self.assertGreater(controller_zero[0], controller_nonzero_at)
        self.assertLessEqual(
            gate_zero[0] - measurement_started,
            gate_zero_limit,
        )
        self.assertLessEqual(
            controller_zero[0] - gate_zero[0],
            0.20,
        )
        stationary_started, stationary_completed = (
            self.wait_for_stationary_window(
                gate_zero[0],
                0.20,
                timeout=2.0,
            )
        )
        self.assertLessEqual(
            stationary_started - gate_zero[0],
            1.20,
        )
        self.assertGreaterEqual(
            stationary_completed - stationary_started,
            0.20,
        )
        reason_name = (
            'AUTHORITY_EXPIRED'
            if reason == InternalMotionGateState.AUTHORITY_EXPIRED
            else 'CANDIDATE_EXPIRED'
        )
        print(
            'EVIDENCE automatic_stop '
            f'reason={reason_name} '
            f'gate_zero_ms='
            f'{(gate_zero[0] - measurement_started) * 1000.0:.3f} '
            f'controller_after_gate_ms='
            f'{(controller_zero[0] - gate_zero[0]) * 1000.0:.3f} '
            f'stationary_after_gate_ms='
            f'{(stationary_started - gate_zero[0]) * 1000.0:.3f} '
            f'stationary_hold_ms='
            f'{(stationary_completed - stationary_started) * 1000.0:.3f}',
            flush=True,
        )
        return expired_state

    def adopt_state_authority(self, state):
        self.current_gate_instance_id = state.gate_instance_id
        self.current_control_seq = state.control_seq
        self.current_lease_id = state.lease_id

    def assert_final_owner_and_qos(self):
        publishers = self.wait_until(
            lambda: (
                endpoints
                if (
                    len(
                        endpoints := self.node.get_publishers_info_by_topic(
                            FINAL_COMMAND_TOPIC
                        )
                    )
                    == 1
                )
                else None
            ),
            15.0,
            'one final command publisher',
        )
        publisher = publishers[0]
        self.assertEqual(publisher.node_name, 'motion_gate_node')
        self.assertEqual(publisher.node_namespace, '/')
        publisher_gid = bytes(publisher.endpoint_gid)
        self.assertEqual(len(publisher_gid), 16)

        state_publishers = self.wait_until(
            lambda: (
                endpoints
                if len(
                    endpoints := self.node.get_publishers_info_by_topic(
                        STATE_TOPIC
                    )
                )
                == 1
                else None
            ),
            5.0,
            'one transient MotionGate state publisher',
        )
        state_qos = state_publishers[0].qos_profile
        self.assertEqual(
            state_qos.reliability,
            ReliabilityPolicy.RELIABLE,
        )
        self.assertEqual(
            state_qos.durability,
            DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Fast DDS graph introspection reports UNKNOWN/0 for this pair even
        # though the publisher was created as KEEP_LAST(1).  The late-joining
        # transient subscription above proves the operational history
        # contract; reliability and durability remain exact graph checks.
        self.assertIn(
            (state_qos.history, state_qos.depth),
            (
                (HistoryPolicy.KEEP_LAST, 1),
                (HistoryPolicy.UNKNOWN, 0),
            ),
        )
        return publisher_gid

    def test_motion_gate_product_contract(self):
        self.assertEqual(
            get_rmw_implementation_identifier(),
            'rmw_fastrtps_cpp',
            'MotionGate GID binding is locked to rmw_fastrtps_cpp',
        )
        self.assertTrue(
            self.control_client.wait_for_service(timeout_sec=30.0),
            'MotionGate control service did not become available',
        )
        self.wait_until(
            lambda: (
                sample
                if (
                    (sample := self.latest_sample(self.odometry))
                    and self.node.get_clock().now().nanoseconds > 0
                )
                else None
            ),
            60.0,
            'advancing simulation and odometry',
        )
        initial_publisher_gid = self.assert_final_owner_and_qos()
        print(
            'EVIDENCE product_identity '
            f'rmw={get_rmw_implementation_identifier()} '
            f'final_owner=/motion_gate_node '
            f'final_gid={initial_publisher_gid.hex()}',
            flush=True,
        )
        initial_state = self.wait_until(
            lambda: (
                sample[1]
                if (
                    (sample := self.latest_sample(self.states))
                    and sample[1].state
                    == InternalMotionGateState.INHIBITED
                )
                else None
            ),
            5.0,
            'default inhibited state',
        )
        self.assertTrue(initial_state.motion_inhibited)
        self.assertTrue(initial_state.zero_selected)
        self.wait_for_command(
            self.final_commands,
            is_zero,
            time.monotonic() - 0.2,
            1.0,
            'default periodic Gate zero',
        )

        prepared = self.prepare()
        publisher = self.create_candidate_publisher(
            prepared.candidate_topic
        )
        pre_open_position = self.latest_sample(self.odometry)[1].pose.pose
        for _ in range(4):
            self.publish_candidate(publisher, 0.20, 0.0)
            time.sleep(0.02)
        open_response = self.open_gate()
        self.assertEqual(
            bytes(open_response.bound_writer_gid),
            bytes(
                self.node.get_publishers_info_by_topic(
                    prepared.candidate_topic
                )[0].endpoint_gid
            ),
        )
        print(
            'EVIDENCE lease_binding '
            f'gate={open_response.gate_instance_id} '
            f'lease={open_response.lease_id} '
            f'topic={prepared.candidate_topic} '
            f'writer_gid={bytes(open_response.bound_writer_gid).hex()}',
            flush=True,
        )
        opened_at = time.monotonic()
        self.renew()
        time.sleep(0.08)
        self.assertTrue(
            all(
                is_zero(message)
                for receipt_time, message in self.sample_snapshot(
                    self.final_commands
                )
                if receipt_time >= opened_at
            ),
            'a sample queued before OPEN escaped the reader barrier',
        )
        post_open_position = self.latest_sample(self.odometry)[1].pose.pose
        self.assertLess(
            math.hypot(
                post_open_position.position.x
                - pre_open_position.position.x,
                post_open_position.position.y
                - pre_open_position.position.y,
            ),
            0.01,
        )

        bounded_start = self.latest_sample(self.odometry)[1].pose.pose.position
        self.publish_with_renewals(
            publisher,
            0.15,
            0.0,
            0.65,
        )
        self.wait_for_command(
            self.final_commands,
            lambda message: message.twist.linear.x > 0.10,
            opened_at,
            2.0,
            'bounded forward Gate output',
        )
        bounded_end = self.wait_until(
            lambda: (
                sample[1].pose.pose.position
                if (
                    (sample := self.latest_sample(self.odometry))
                    and math.hypot(
                        sample[1].pose.pose.position.x
                        - bounded_start.x,
                        sample[1].pose.pose.position.y
                        - bounded_start.y,
                    )
                    > 0.03
                )
                else None
            ),
            3.0,
            'bounded forward odometry',
        )
        self.assertGreater(
            math.hypot(
                bounded_end.x - bounded_start.x,
                bounded_end.y - bounded_start.y,
            ),
            0.03,
        )
        print(
            'EVIDENCE bounded_motion '
            f'distance_m='
            f'{math.hypot(bounded_end.x - bounded_start.x, bounded_end.y - bounded_start.y):.6f}',
            flush=True,
        )

        clamp_started = time.monotonic()
        self.publish_with_renewals(
            publisher,
            2.0,
            2.0,
            0.12,
        )
        clamped = self.wait_for_command(
            self.final_commands,
            lambda message: (
                abs(message.twist.linear.x - 0.40) <= COMMAND_EPSILON
                and abs(message.twist.angular.z - 1.20)
                <= COMMAND_EPSILON
            ),
            clamp_started,
            2.0,
            'Gate clamp at trusted bounds',
        )
        self.assertAlmostEqual(clamped[1].twist.linear.x, 0.40)
        self.assertAlmostEqual(clamped[1].twist.angular.z, 1.20)
        controller_clamped = self.wait_for_command(
            self.limited_commands,
            lambda message: (
                abs(message.twist.linear.x - 0.40) <= COMMAND_EPSILON
                and abs(message.twist.angular.z - 1.20)
                <= COMMAND_EPSILON
            ),
            clamp_started,
            2.0,
            'controller output at trusted clamp bounds',
        )
        self.assertAlmostEqual(
            controller_clamped[1].twist.linear.x,
            0.40,
        )
        self.assertAlmostEqual(
            controller_clamped[1].twist.angular.z,
            1.20,
        )
        print(
            'EVIDENCE clamp '
            f'gate_linear_x={clamped[1].twist.linear.x:.3f} '
            f'gate_angular_z={clamped[1].twist.angular.z:.3f} '
            f'controller_linear_x='
            f'{controller_clamped[1].twist.linear.x:.3f} '
            f'controller_angular_z='
            f'{controller_clamped[1].twist.angular.z:.3f}',
            flush=True,
        )

        authority_measurement_started = time.monotonic()
        self.renew()
        renewed_state = self.wait_until(
            lambda: next(
                (
                    sample[1]
                    for sample in reversed(
                        self.sample_snapshot(self.states)
                    )
                    if sample[1].state
                    == InternalMotionGateState.ARMED
                    and sample[1].lease_id == self.current_lease_id
                    and sample[1].control_seq
                    == self.current_control_seq
                ),
                None,
            ),
            1.0,
            'renewed authority state',
        )
        authority_flood_stop = threading.Event()

        def flood_candidates_until_authority_expires():
            while not authority_flood_stop.is_set():
                self.publish_candidate(publisher, 0.12, 0.0)
                time.sleep(0.015)

        authority_flood = threading.Thread(
            target=flood_candidates_until_authority_expires,
            daemon=True,
        )
        authority_flood.start()
        try:
            accepted_at, accepted_state = (
                self.wait_for_accepted_candidate(
                    self.current_lease_id,
                    renewed_state.state_seq,
                    authority_measurement_started,
                )
            )
            final_motion, controller_motion, _ = (
                self.wait_for_motion_evidence(accepted_at)
            )
            expired_state = self.assert_automatic_stop_evidence(
                measurement_started=authority_measurement_started,
                accepted_state_seq=accepted_state.state_seq,
                final_nonzero_at=final_motion[0],
                controller_nonzero_at=controller_motion[0],
                reason=InternalMotionGateState.AUTHORITY_EXPIRED,
                gate_zero_limit=0.300,
            )
        finally:
            authority_flood_stop.set()
            authority_flood.join(timeout=1.0)
        self.assertFalse(authority_flood.is_alive())
        self.adopt_state_authority(expired_state)
        old_topic = prepared.candidate_topic
        self.destroy_candidate_publisher(publisher, old_topic)

        prepared = self.prepare()
        publisher = self.create_candidate_publisher(
            prepared.candidate_topic
        )
        self.open_gate()
        self.publish_with_renewals(
            publisher,
            0.12,
            0.0,
            0.35,
        )
        self.renew()
        renewed_state = self.wait_until(
            lambda: next(
                (
                    sample[1]
                    for sample in reversed(
                        self.sample_snapshot(self.states)
                    )
                    if sample[1].state
                    == InternalMotionGateState.ARMED
                    and sample[1].lease_id == self.current_lease_id
                    and sample[1].control_seq
                    == self.current_control_seq
                ),
                None,
            ),
            1.0,
            'freshness-case renewed authority state',
        )
        freshness_measurement_started = time.monotonic()
        self.publish_candidate(publisher, 0.12, 0.0)
        accepted_at, accepted_state = self.wait_for_accepted_candidate(
            self.current_lease_id,
            renewed_state.state_seq,
            freshness_measurement_started,
        )
        # Renew authority well after the final candidate, then renew it again
        # before the candidate deadline. Neither operation may refresh
        # candidate freshness.
        for renewal_offset in (0.085, 0.115):
            remaining = accepted_at + renewal_offset - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            renewal = self.renew()
            self.assertTrue(renewal.candidate_fresh)
        final_motion, controller_motion, _ = self.wait_for_motion_evidence(
            accepted_at
        )
        expired_state = self.assert_automatic_stop_evidence(
            measurement_started=freshness_measurement_started,
            accepted_state_seq=accepted_state.state_seq,
            final_nonzero_at=final_motion[0],
            controller_nonzero_at=controller_motion[0],
            reason=InternalMotionGateState.CANDIDATE_EXPIRED,
            gate_zero_limit=0.200,
        )
        self.adopt_state_authority(expired_state)
        self.destroy_candidate_publisher(
            publisher,
            prepared.candidate_topic,
        )

        # Freshness expiry already retired the preceding lease.  Establish a
        # new live authority so the next assertion measures explicit INHIBIT,
        # rather than accidentally issuing a stale request to an inhibited
        # Gate.
        prepared = self.prepare()
        publisher = self.create_candidate_publisher(
            prepared.candidate_topic
        )
        self.open_gate()
        explicit_inhibit_motion_started = time.monotonic()
        self.publish_with_renewals(
            publisher,
            0.20,
            0.0,
            0.35,
        )
        final_motion, controller_motion, moving_odometry = (
            self.wait_for_motion_evidence(
                explicit_inhibit_motion_started
            )
        )
        self.assertFalse(is_zero(final_motion[1]))
        self.assertFalse(is_zero(controller_motion[1]))
        self.assertTrue(
            abs(moving_odometry[1].twist.twist.linear.x)
            >= MOVING_LINEAR_TOLERANCE
            or abs(moving_odometry[1].twist.twist.angular.z)
            >= MOVING_ANGULAR_TOLERANCE
        )

        self.clear_samples(self.final_commands)
        self.clear_samples(self.limited_commands)
        inhibit_started, inhibit_completed, _ = self.inhibit()
        gate_zero = self.wait_for_command(
            self.final_commands,
            is_zero,
            inhibit_started,
            1.0,
            'Gate zero published before/with INHIBIT acknowledgement',
        )
        controller_zero = self.wait_for_command(
            self.limited_commands,
            is_zero,
            inhibit_started,
            1.0,
            'controller limited-output zero after INHIBIT',
        )
        self.assertLessEqual(gate_zero[0], inhibit_completed + 0.10)
        self.assertLessEqual(controller_zero[0], inhibit_completed + 0.15)

        post_inhibit_guard = time.monotonic() + 0.03
        guard_deadline = post_inhibit_guard + 0.22
        while time.monotonic() < guard_deadline:
            self.publish_candidate(publisher, 0.20, 0.0)
            time.sleep(0.015)
        self.assertTrue(
            all(
                is_zero(message)
                for receipt_time, message in self.sample_snapshot(
                    self.final_commands
                )
                if receipt_time >= post_inhibit_guard
            ),
            'old-topic candidate produced non-zero after INHIBIT',
        )
        stationary_started, stationary_completed = (
            self.wait_for_stationary_window(
                gate_zero[0],
                0.20,
                timeout=2.0,
            )
        )
        self.assertLessEqual(
            stationary_started - gate_zero[0],
            1.20,
        )
        self.assertGreaterEqual(
            stationary_completed - stationary_started,
            0.20,
        )
        print(
            'EVIDENCE explicit_inhibit '
            f'ack_ms={(inhibit_completed - inhibit_started) * 1000.0:.3f} '
            f'gate_zero_ms={(gate_zero[0] - inhibit_started) * 1000.0:.3f} '
            f'controller_zero_ms='
            f'{(controller_zero[0] - inhibit_started) * 1000.0:.3f} '
            f'moving_linear_x='
            f'{moving_odometry[1].twist.twist.linear.x:.6f} '
            f'moving_angular_z='
            f'{moving_odometry[1].twist.twist.angular.z:.6f} '
            f'stationary_after_gate_ms='
            f'{(stationary_started - gate_zero[0]) * 1000.0:.3f} '
            f'stationary_hold_ms='
            f'{(stationary_completed - stationary_started) * 1000.0:.3f}',
            flush=True,
        )
        self.destroy_candidate_publisher(
            publisher,
            prepared.candidate_topic,
        )

        final_publishers = self.assert_final_owner_and_qos()
        self.assertEqual(final_publishers, initial_publisher_gid)
        print(
            'EVIDENCE final_owner_stable '
            f'final_gid={final_publishers.hex()}',
            flush=True,
        )


@launch_testing.post_shutdown_test()
class MotionGateProductShutdownTest(unittest.TestCase):

    def test_all_launch_managed_processes_exit_cleanly(self, proc_info):
        assertExitCodes(proc_info)
