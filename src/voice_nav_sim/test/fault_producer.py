#!/usr/bin/env python3
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

"""Run one test-only process role for independent MotionGate producers."""

import time
import uuid

from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Empty
from voice_nav_mission.msg import InternalMotionGateState
from voice_nav_mission.srv import InternalMotionGateControl


CONTROL_SERVICE = '/motion_gate/internal/control'
STATE_TOPIC = '/motion_gate/internal/state'
FINAL_TOPIC = '/diff_drive_controller/cmd_vel'
READY_TOPIC_PREFIX = '/voice_nav_test/fault_producer'
PUBLISH_PERIOD_SECONDS = 0.01
RENEW_PERIOD_SECONDS = 0.075
WAIT_STEP_SECONDS = 0.005
STARTUP_TIMEOUT_SECONDS = 5.0
CONTROL_TIMEOUT_SECONDS = 0.5
OPEN_CONVERGENCE_SECONDS = 1.0
OPEN_BACKOFF_SECONDS = 0.01
NO_WRITER_PENDING_DETAIL = 'candidate topic has no writer'


def state_qos():
    """Match MotionGate's transient-local state snapshot contract."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def candidate_qos():
    """Match the pinned candidate writer QoS accepted by MotionGate."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def ready_qos():
    """Retain candidate readiness until the authority reader is matched."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def is_retryable_open_response(response):
    """Classify only the two locked fail-closed discovery observations."""
    if response.code != InternalMotionGateControl.Response.REJECTED:
        return False
    return (
        response.reason ==
        InternalMotionGateControl.Response.WRITER_METADATA_PENDING or
        (
            response.reason ==
            InternalMotionGateControl.Response.WRITER_UNAVAILABLE and
            response.detail == NO_WRITER_PENDING_DETAIL
        )
    )


def require_unchanged_prepared_response(response, prepared):
    """Reject any pending response that changed the prepared generation."""
    expected = {
        'gate_instance_id': prepared.gate_instance_id,
        'control_seq': prepared.control_seq,
        'state': InternalMotionGateState.PREPARED,
        'lease_id': prepared.lease_id,
        'candidate_topic': prepared.candidate_topic,
        'motion_inhibited': True,
        'authority_live': False,
        'candidate_fresh': False,
        'writer_bound': False,
        'zero_selected': True,
        'zero_published': True,
    }
    mismatches = [
        field
        for field, expected_value in expected.items()
        if getattr(response, field) != expected_value
    ]
    if any(response.bound_writer_gid):
        mismatches.append('bound_writer_gid')
    if mismatches:
        raise RuntimeError(
            'retryable OPEN response changed prepared invariants: '
            + ', '.join(mismatches)
        )


class FaultProducerNode(Node):
    """Own one closed producer role in its own OS process."""

    def __init__(self):
        """Validate immutable fixture identity before exposing behavior."""
        super().__init__('fault_producer_helper')
        immutable = ParameterDescriptor(read_only=True)
        self.role = self.declare_parameter(
            'role',
            '',
            immutable,
        ).value
        self.case_id = self.declare_parameter(
            'case_id',
            '',
            immutable,
        ).value
        if self.role not in ('authority', 'candidate'):
            raise RuntimeError('fault producer role must be authority or candidate')
        if not self.case_id:
            raise RuntimeError('fault producer case_id must be non-empty')
        if (
            self.role == 'candidate' and
            self.get_fully_qualified_name() != '/collision_monitor'
        ):
            raise RuntimeError('candidate FQN must be /collision_monitor')
        self.get_logger().info(
            f'fault producer role={self.role} case={self.case_id} started'
        )

        self.ready_topic = f'{READY_TOPIC_PREFIX}/{self.case_id}/ready'
        self.candidate_ready = False
        self.ready_subscription = None
        self.ready_publisher = None
        self.latest_state = None
        self.state_subscription = self.create_subscription(
            InternalMotionGateState,
            STATE_TOPIC,
            self.on_state,
            state_qos(),
        )
        self.control_client = None
        self.candidate_publisher = None
        self.candidate_topic = ''
        if self.role == 'authority':
            self.control_client = self.create_client(
                InternalMotionGateControl,
                CONTROL_SERVICE,
            )
            self.ready_subscription = self.create_subscription(
                Empty,
                self.ready_topic,
                self.on_candidate_ready,
                ready_qos(),
            )
        else:
            self.ready_publisher = self.create_publisher(
                Empty,
                self.ready_topic,
                ready_qos(),
            )

    def on_state(self, message):
        """Retain the newest transient state for one serialized role loop."""
        self.latest_state = message

    def on_candidate_ready(self, _message):
        """Record that the candidate has consumed a real Gate snapshot."""
        self.candidate_ready = True

    def wait_for_state(self, executor, predicate, description):
        """Wait with a steady deadline while continuing ROS progress."""
        deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS
        while rclpy.ok() and time.monotonic() < deadline:
            state = self.latest_state
            if state is not None and predicate(state):
                return state
            executor.spin_once(timeout_sec=WAIT_STEP_SECONDS)
        if not rclpy.ok():
            return None
        raise RuntimeError(f'timed out waiting for {description}')

    def request_control(
        self,
        executor,
        operation,
        view,
        *,
        lease_id='',
        timeout_seconds=CONTROL_TIMEOUT_SECONDS,
    ):
        """Issue one fresh-ID control request and return its exact response."""
        request = InternalMotionGateControl.Request()
        request.operation = operation
        request.request_id = uuid.uuid4().hex
        request.gate_instance_id = view.gate_instance_id
        request.expected_control_seq = view.control_seq
        request.lease_id = lease_id
        future = self.control_client.call_async(request)
        deadline = time.monotonic() + min(
            CONTROL_TIMEOUT_SECONDS,
            timeout_seconds,
        )
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=WAIT_STEP_SECONDS)
        if not rclpy.ok():
            return None
        if not future.done():
            raise RuntimeError(
                f'control operation {operation} exceeded its steady deadline'
            )
        if future.exception() is not None:
            raise RuntimeError(
                f'control operation {operation} failed: {future.exception()}'
            )
        return future.result()

    def call_control(self, executor, operation, view, *, lease_id=''):
        """Require one serialized authority operation to be applied."""
        response = self.request_control(
            executor,
            operation,
            view,
            lease_id=lease_id,
        )
        if response is None:
            return None
        if response.code != InternalMotionGateControl.Response.APPLIED:
            raise RuntimeError(
                f'control operation {operation} rejected: '
                f'code={response.code} reason={response.reason} '
                f'detail={response.detail}'
            )
        return response

    def wait_for_candidate_writer(self, executor, topic, deadline):
        """Require the one exact-FQN publisher before requesting OPEN."""
        while rclpy.ok() and time.monotonic() < deadline:
            endpoints = self.get_publishers_info_by_topic(topic)
            matching = [
                endpoint
                for endpoint in endpoints
                if (
                    endpoint.node_name == 'collision_monitor' and
                    endpoint.node_namespace == '/'
                )
            ]
            if len(endpoints) == 1 and len(matching) == 1:
                return
            executor.spin_once(timeout_sec=WAIT_STEP_SECONDS)
        if rclpy.ok():
            raise RuntimeError(
                'timed out waiting for the unique /collision_monitor writer'
            )

    def open_with_convergence(self, executor, prepared, deadline):
        """Retry only locked discovery-pending responses before one deadline."""
        attempts = 0
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(
                    f'OPEN convergence expired after {attempts} attempt(s)'
                )
            response = self.request_control(
                executor,
                InternalMotionGateControl.Request.OPEN,
                prepared,
                lease_id=prepared.lease_id,
                timeout_seconds=remaining,
            )
            if response is None:
                return None
            attempts += 1
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f'OPEN convergence expired after {attempts} attempt(s)'
                )
            if response.code == InternalMotionGateControl.Response.APPLIED:
                return response
            if not is_retryable_open_response(response):
                raise RuntimeError(
                    'control operation OPEN rejected: '
                    f'code={response.code} reason={response.reason} '
                    f'detail={response.detail}'
                )
            require_unchanged_prepared_response(response, prepared)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                continue
            executor.spin_once(
                timeout_sec=min(OPEN_BACKOFF_SECONDS, remaining)
            )
        return None

    def wait_for_candidate_state_reader(self, executor):
        """Establish the candidate reader before creating a short generation."""
        deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS
        while rclpy.ok() and time.monotonic() < deadline:
            endpoints = self.get_subscriptions_info_by_topic(STATE_TOPIC)
            matching = [
                endpoint
                for endpoint in endpoints
                if (
                    endpoint.node_name == 'collision_monitor' and
                    endpoint.node_namespace == '/'
                )
            ]
            if len(matching) == 1:
                return
            executor.spin_once(timeout_sec=WAIT_STEP_SECONDS)
        if rclpy.ok():
            raise RuntimeError(
                'timed out waiting for the /collision_monitor state reader'
            )

    def wait_for_candidate_ready(self, executor):
        """Wait until the candidate proves its Gate subscription is live."""
        deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS
        while rclpy.ok() and time.monotonic() < deadline:
            endpoints = self.get_publishers_info_by_topic(self.ready_topic)
            matching = [
                endpoint
                for endpoint in endpoints
                if (
                    endpoint.node_name == 'collision_monitor' and
                    endpoint.node_namespace == '/'
                )
            ]
            if self.candidate_ready and len(matching) == 1:
                return
            executor.spin_once(timeout_sec=WAIT_STEP_SECONDS)
        if rclpy.ok():
            raise RuntimeError(
                'timed out waiting for candidate Gate-state readiness'
            )

    def wait_for_final_controller_reader(self, executor):
        """Require the exact final consumer before creating a generation."""
        deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS
        while rclpy.ok() and time.monotonic() < deadline:
            endpoints = self.get_subscriptions_info_by_topic(FINAL_TOPIC)
            matching = [
                endpoint
                for endpoint in endpoints
                if (
                    endpoint.node_name == 'diff_drive_controller' and
                    endpoint.node_namespace == '/' and
                    endpoint.topic_type == 'geometry_msgs/msg/TwistStamped'
                )
            ]
            if len(matching) == 1:
                return
            executor.spin_once(timeout_sec=WAIT_STEP_SECONDS)
        if rclpy.ok():
            raise RuntimeError(
                'timed out waiting for the /diff_drive_controller reader'
            )

    def run_authority(self, executor):
        """Own PREPARE, OPEN, and every authority RENEW until process death."""
        deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS
        while (
            rclpy.ok() and
            not self.control_client.wait_for_service(timeout_sec=0.05)
        ):
            if time.monotonic() >= deadline:
                raise RuntimeError('MotionGate control service unavailable')
            executor.spin_once(timeout_sec=WAIT_STEP_SECONDS)
        initial = self.wait_for_state(
            executor,
            lambda state: state.state == InternalMotionGateState.INHIBITED,
            'initial inhibited Gate state',
        )
        if initial is None:
            return
        # PREPARED is deliberately short-lived.  Do not create its generation
        # until the independently launched candidate can receive the snapshot.
        self.wait_for_candidate_state_reader(executor)
        if not rclpy.ok():
            return
        self.wait_for_candidate_ready(executor)
        if not rclpy.ok():
            return
        self.wait_for_final_controller_reader(executor)
        if not rclpy.ok():
            return
        open_deadline = time.monotonic() + OPEN_CONVERGENCE_SECONDS
        prepared = self.call_control(
            executor,
            InternalMotionGateControl.Request.PREPARE,
            initial,
        )
        if prepared is None:
            return
        if prepared.state != InternalMotionGateState.PREPARED:
            raise RuntimeError('PREPARE did not establish the prepared state')
        self.wait_for_candidate_writer(
            executor,
            prepared.candidate_topic,
            open_deadline,
        )
        if not rclpy.ok():
            return
        opened = self.open_with_convergence(
            executor,
            prepared,
            open_deadline,
        )
        if opened is None:
            return
        if opened.state != InternalMotionGateState.ARMED:
            raise RuntimeError('OPEN did not establish the armed state')

        current = opened
        next_renew = time.monotonic() + RENEW_PERIOD_SECONDS
        while rclpy.ok():
            remaining = next_renew - time.monotonic()
            if remaining > 0.0:
                executor.spin_once(
                    timeout_sec=min(WAIT_STEP_SECONDS, remaining)
                )
                continue
            renewed = self.call_control(
                executor,
                InternalMotionGateControl.Request.RENEW,
                current,
                lease_id=current.lease_id,
            )
            if renewed is None:
                return
            if renewed.state != InternalMotionGateState.ARMED:
                raise RuntimeError('RENEW did not preserve the armed state')
            current = renewed
            next_renew = time.monotonic() + RENEW_PERIOD_SECONDS

    def bind_candidate_topic(self, topic):
        """Replace only the stale generation publisher with the new topic."""
        if topic == self.candidate_topic and self.candidate_publisher is not None:
            return
        if self.candidate_publisher is not None:
            self.destroy_publisher(self.candidate_publisher)
        self.candidate_publisher = self.create_publisher(
            TwistStamped,
            topic,
            candidate_qos(),
        )
        self.candidate_topic = topic

    def run_candidate(self, executor):
        """Discover the prepared topic and publish one bounded marker."""
        initial = self.wait_for_state(
            executor,
            lambda _state: True,
            'initial Gate snapshot',
        )
        if initial is None:
            return
        self.ready_publisher.publish(Empty())
        next_publish = time.monotonic()
        while rclpy.ok():
            executor.spin_once(timeout_sec=WAIT_STEP_SECONDS)
            state = self.latest_state
            if (
                state is not None and
                state.state in (
                    InternalMotionGateState.PREPARED,
                    InternalMotionGateState.ARMED,
                ) and
                state.candidate_topic
            ):
                self.bind_candidate_topic(state.candidate_topic)
            if (
                self.candidate_publisher is None or
                time.monotonic() < next_publish
            ):
                continue
            command = TwistStamped()
            command.header.stamp = self.get_clock().now().to_msg()
            command.twist.linear.x = 0.18
            command.twist.angular.z = 0.30
            self.candidate_publisher.publish(command)
            next_publish = time.monotonic() + PUBLISH_PERIOD_SECONDS

    def run(self):
        """Execute only the role selected by the immutable launch fixture."""
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            if self.role == 'authority':
                self.run_authority(executor)
            else:
                self.run_candidate(executor)
        finally:
            executor.remove_node(self)
            executor.shutdown(timeout_sec=1.0)


def main(args=None):
    """Run one helper until launch performs deterministic teardown."""
    rclpy.init(args=args)
    node = None
    try:
        node = FaultProducerNode()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
