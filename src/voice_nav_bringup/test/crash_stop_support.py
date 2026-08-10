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

"""Deep test support for the two real crash-stop acceptance scenarios."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import importlib.util
import json
from pathlib import Path
import secrets
import sys
import threading
import time
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import launch_testing.actions
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState, MissionStep
from voice_nav_mission.msg import InternalMotionGateState
from voice_nav_mission.srv import InternalMotionGateControl


FINAL_COMMAND_TOPIC = '/diff_drive_controller/cmd_vel'
LIMITED_COMMAND_TOPIC = '/diff_drive_controller/cmd_vel_out'
ODOMETRY_TOPIC = '/odom'
SCAN_TOPIC = '/scan'
JOINT_STATE_TOPIC = '/joint_states'
CLOCK_TOPIC = '/clock'
MISSION_STATE_TOPIC = '/mission/state'
GATE_STATE_TOPIC = '/motion_gate/internal/state'
GATE_CONTROL_SERVICE = '/motion_gate/internal/control'
ACTION_NAME = '/mission/execute'

RUNTIME_NODE = 'mission_runtime_node'
GATE_NODE = 'motion_gate_node'
ZERO_LINEAR_TOLERANCE = 0.01
ZERO_ANGULAR_TOLERANCE = 0.02
ZERO_WHEEL_TOLERANCE = 0.02
GATE_ZERO_DEADLINE_NS = 350_000_000
CONSUMER_ZERO_MIN_NS = 350_000_000
CONSUMER_ZERO_MAX_NS = 360_000_000
STATIONARY_DEADLINE_NS = 1_200_000_000
STATIONARY_HOLD_NS = 200_000_000
NO_GOAL_WINDOW_NS = 1_000_000_000
WALL_WATCHDOG_SECONDS = 5.0
DEPENDENCY_FRESHNESS_NS = 200_000_000
FINAL_STREAM_QUIESCENCE_NS = 200_000_000


def _load_gazebo_shutdown():
    support_path = (
        Path(get_package_share_directory('voice_nav_sim'))
        / 'test_support'
        / 'gazebo_shutdown.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_crash_stop_gazebo_shutdown',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load Gazebo shutdown support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _load_product_launch():
    launch_path = (
        Path(get_package_share_directory('voice_nav_bringup'))
        / 'launch'
        / 'product_sim.launch.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_crash_stop_product_launch',
        launch_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load product launch')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


gazebo_shutdown = _load_gazebo_shutdown()


def _load_process_identity():
    support_path = Path(__file__).with_name('process_identity.py')
    specification = importlib.util.spec_from_file_location(
        'voice_nav_crash_stop_process_identity',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load process identity support')
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


process_identity = _load_process_identity()
ExactPidfdProcess = process_identity.ExactPidfdProcess
ProcessStartedCapture = process_identity.ProcessStartedCapture


def _load_state_observation():
    support_path = Path(__file__).with_name('state_observation.py')
    specification = importlib.util.spec_from_file_location(
        'voice_nav_crash_stop_state_observation', support_path
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load state observation support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


state_observation = _load_state_observation()


@dataclass
class RestartRecord:
    """One explicitly restarted launch action and its exact pidfd capture."""

    action: Node
    capture: ProcessStartedCapture


class RestartRegistry:
    """Owns only actions explicitly created by this test."""

    def __init__(self) -> None:
        self.records: list[RestartRecord] = []

    def append(self, record: RestartRecord) -> None:
        self.records.append(record)

    def close(self) -> None:
        for record in self.records:
            record.capture.close()


def generate_product_test_description(scope: str):
    """Return product launch actions plus exact handles for both nodes."""
    partition = gazebo_shutdown.claim_unique_test_partition(scope)
    product_launch = _load_product_launch().generate_launch_description()
    node_actions = {
        str(action.node_executable): action
        for action in product_launch.entities
        if isinstance(action, Node)
        and action.node_package == 'voice_nav_mission'
    }
    try:
        runtime = node_actions[RUNTIME_NODE]
        gate = node_actions[GATE_NODE]
    except KeyError as error:
        raise RuntimeError(
            'product launch does not expose the required Runtime/Gate actions'
        ) from error

    runtime_capture = ProcessStartedCapture(
        action=runtime,
        expected_executable=RUNTIME_NODE,
        expected_node_name=RUNTIME_NODE,
    )
    gate_capture = ProcessStartedCapture(
        action=gate,
        expected_executable=GATE_NODE,
        expected_node_name=GATE_NODE,
    )
    product_launch.add_action(
        RegisterEventHandler(
            OnProcessStart(on_start=runtime_capture.on_start)
        )
    )
    product_launch.add_action(
        RegisterEventHandler(OnProcessStart(on_start=gate_capture.on_start))
    )
    product_launch.add_action(launch_testing.actions.ReadyToTest())
    return product_launch, {
        'runtime': runtime,
        'gate': gate,
        'runtime_capture': runtime_capture,
        'gate_capture': gate_capture,
        'partition': partition,
        'restarts': RestartRegistry(),
    }


def restart_product_node(
    launch_service: Any,
    restarts: RestartRegistry,
    *,
    executable: str,
    name: str,
    config_filename: str,
) -> RestartRecord:
    """Start exactly one replacement node using the trusted product YAML."""
    config_path = (
        Path(get_package_share_directory('voice_nav_bringup'))
        / 'config'
        / config_filename
    )
    action = Node(
        package='voice_nav_mission',
        executable=executable,
        name=name,
        output='screen',
        parameters=[str(config_path)],
    )
    capture = ProcessStartedCapture(
        action=action,
        expected_executable=executable,
        expected_node_name=name,
    )
    launch_service.include_launch_description(
        LaunchDescription(
            [
                RegisterEventHandler(
                    OnProcessStart(on_start=capture.on_start)
                ),
                action,
            ]
        )
    )
    record = RestartRecord(action, capture)
    restarts.append(record)
    return record


def process_startup_summary(
    role: str,
    process: ExactPidfdProcess,
    capture: ProcessStartedCapture,
) -> dict[str, Any]:
    """Summarize one launch-owned process start without selecting a process."""
    return {
        'role': role,
        'process_started_monotonic_ns': capture.started_monotonic_ns,
        'expected_node_name': process.expected_node_name,
        'expected_executable': process.expected_executable,
        'expected_executable_path': process.expected_executable_path,
        'event_cmd': list(process.event_command),
        'pid': process.snapshot.pid,
        'starttime_ticks': process.snapshot.starttime_ticks,
        'executable': process.snapshot.executable,
        'cmdline': list(process.snapshot.cmdline),
    }


def _stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _clock_ns(message: Clock) -> int:
    return int(message.clock.sec) * 1_000_000_000 + int(
        message.clock.nanosec
    )


def is_zero(message: TwistStamped) -> bool:
    values = (
        message.twist.linear.x,
        message.twist.linear.y,
        message.twist.linear.z,
        message.twist.angular.x,
        message.twist.angular.y,
        message.twist.angular.z,
    )
    return all(value == 0.0 for value in values)


def select_consumer_timeout_anchor(
    final_samples: tuple[tuple[int, Any], ...],
    *,
    signal_boundary_sim_ns: int,
):
    """Select the latest killed-Gate final command by simulation stamp."""
    eligible = []
    for receipt_ns, message in final_samples:
        stamp_ns = _stamp_ns(message)
        if (
            stamp_ns >= signal_boundary_sim_ns
            and not is_zero(message)
        ):
            eligible.append((stamp_ns, receipt_ns, message))
    if not eligible:
        raise AssertionError(
            'no eligible non-zero final command before consumer zero'
        )
    _, receipt_ns, message = max(eligible, key=lambda sample: sample[:2])
    return receipt_ns, deepcopy(message)


def consumer_timeout_result(
    final_samples: tuple[tuple[int, Any], ...],
    *,
    signal_boundary_sim_ns: int,
    zero_sim_ns: int,
    zero_receipt_ns: int,
):
    """Apply the frozen controller timeout to the drained final stream."""
    last_nonzero_receipt_ns, last_nonzero = (
        select_consumer_timeout_anchor(
            final_samples,
            signal_boundary_sim_ns=signal_boundary_sim_ns,
        )
    )
    last_nonzero_sim_ns = _stamp_ns(last_nonzero)
    delta_ns = zero_sim_ns - last_nonzero_sim_ns
    if not (CONSUMER_ZERO_MIN_NS < delta_ns <= CONSUMER_ZERO_MAX_NS):
        raise AssertionError(
            'controller consumer timeout outside '
            f'(0.35, 0.36] s: {delta_ns / 1_000_000_000:.6f} s; '
            f'last_nonzero_sim_ns={last_nonzero_sim_ns}; '
            f'zero_sim_ns={zero_sim_ns}'
        )
    return {
        'last_nonzero_sim_ns': last_nonzero_sim_ns,
        'last_nonzero_receipt_ns': last_nonzero_receipt_ns,
        'zero_sim_ns': zero_sim_ns,
        'delta_ns': delta_ns,
        'zero_receipt_ns': zero_receipt_ns,
    }


def gate_zero_proven(message: InternalMotionGateState) -> bool:
    return (
        message.state == InternalMotionGateState.INHIBITED
        and message.motion_inhibited
        and message.zero_selected
        and message.zero_publish_seq > 0
        and message.zero_publish_seq >= message.output_publish_seq
    )


class CrashStopProbe:
    """One ROS observation Interface for both public crash-stop scenarios."""

    def __init__(self) -> None:
        self.node = rclpy.create_node(
            'voice_nav_crash_stop_probe',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            name='voice-nav-crash-stop-probe',
            daemon=True,
        )
        self.lock = threading.Lock()
        self.mission_states = deque(maxlen=4000)
        self.gate_states = deque(maxlen=4000)
        self.final_commands = deque(maxlen=8000)
        self.limited_commands = deque(maxlen=8000)
        self.odometry = deque(maxlen=4000)
        self.scan_samples = deque(maxlen=4000)
        self.joint_states = deque(maxlen=4000)
        self.clock_samples = deque(maxlen=4000)
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscriptions = [
            self.node.create_subscription(
                MissionState,
                MISSION_STATE_TOPIC,
                lambda message: self._append(
                    self.mission_states, message
                ),
                state_qos,
            ),
            self.node.create_subscription(
                InternalMotionGateState,
                GATE_STATE_TOPIC,
                lambda message: self._append(self.gate_states, message),
                state_qos,
            ),
            self.node.create_subscription(
                TwistStamped,
                FINAL_COMMAND_TOPIC,
                lambda message: self._append(
                    self.final_commands, message
                ),
                100,
            ),
            self.node.create_subscription(
                TwistStamped,
                LIMITED_COMMAND_TOPIC,
                lambda message: self._append(
                    self.limited_commands, message
                ),
                100,
            ),
            self.node.create_subscription(
                Odometry,
                ODOMETRY_TOPIC,
                lambda message: self._append(self.odometry, message),
                100,
            ),
            self.node.create_subscription(
                LaserScan,
                SCAN_TOPIC,
                lambda message: self._append(self.scan_samples, message),
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                JointState,
                JOINT_STATE_TOPIC,
                lambda message: self._append(
                    self.joint_states, message
                ),
                100,
            ),
            self.node.create_subscription(
                Clock,
                CLOCK_TOPIC,
                lambda message: self._append(
                    self.clock_samples, _clock_ns(message)
                ),
                qos_profile_sensor_data,
            ),
        ]
        self.action_client = ActionClient(
            self.node, ExecuteMission, ACTION_NAME
        )
        self.gate_client = self.node.create_client(
            InternalMotionGateControl,
            GATE_CONTROL_SERVICE,
        )
        self.candidate_publishers = []
        self.spin_thread.start()

    def _append(self, collection: deque, message: Any) -> None:
        with self.lock:
            collection.append((time.monotonic_ns(), message))

    def _snapshot(self, collection: deque):
        with self.lock:
            return tuple(collection)

    def latest(self, collection: deque):
        with self.lock:
            return collection[-1] if collection else None

    def wait_until(
        self,
        predicate: Callable[[], Any],
        timeout: float,
        description: str,
    ):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value is not None and value is not False:
                return value
            time.sleep(0.01)
        raise AssertionError(f'timed out waiting for {description}')

    def wait_process_capture(
        self, capture: ProcessStartedCapture, timeout: float = 30.0
    ) -> ExactPidfdProcess:
        return capture.wait(timeout)

    def count_fqn(self, expected_fqn: str) -> int:
        matches = []
        for name, namespace in self.node.get_node_names_and_namespaces():
            prefix = namespace.rstrip('/')
            fqn = f'{prefix}/{name}' if prefix else f'/{name}'
            if fqn == expected_fqn:
                matches.append(fqn)
        return len(matches)

    def wait_runtime_ready(
        self,
        previous_runtime_id: str | None = None,
        *,
        after_monotonic_ns: int | None = None,
        timeout: float = 45.0,
    ):
        def ready():
            for receipt_ns, message in reversed(
                self._snapshot(self.mission_states)
            ):
                if (
                    after_monotonic_ns is not None
                    and receipt_ns < after_monotonic_ns
                ):
                    continue
                if (
                    message.availability == MissionState.AVAILABLE
                    and message.gate_state == MissionState.GATE_INHIBITED
                    and message.runtime_instance_id
                    and (
                        previous_runtime_id is None
                        or message.runtime_instance_id != previous_runtime_id
                    )
                ):
                    return deepcopy(message)
            return None

        return self.wait_until(ready, timeout, 'Runtime AVAILABLE/inhibited')

    def wait_runtime_fault(
        self,
        *,
        after_monotonic_ns: int | None = None,
        timeout: float = 10.0,
    ):
        """
        Observe Runtime FAULTED without requiring a fresh Gate state.

        After the Gate process is killed, MissionState may legitimately retain
        its last valid Gate snapshot (for example GATE_ARMED).  The crash-stop
        test proves the independent zero, consumer-deadman, and stationarity
        barriers before reaching this wait; here we only require Runtime's
        FAULTED transition and reject malformed Gate enum values.
        """
        def faulted():
            for receipt_ns, message in reversed(
                self._snapshot(self.mission_states)
            ):
                if (
                    after_monotonic_ns is not None
                    and receipt_ns < after_monotonic_ns
                ):
                    continue
                if (
                    message.availability == MissionState.FAULTED
                    and message.gate_state in (
                        MissionState.GATE_INHIBITED,
                        MissionState.GATE_ARMED,
                        MissionState.GATE_FAULTED,
                    )
                ):
                    return deepcopy(message)
            return None

        return self.wait_until(faulted, timeout, 'Runtime SAFETY_FAULT')

    def wait_gate_instance(
        self, previous_instance: str | None = None, timeout: float = 20.0
    ):
        def ready():
            for _, message in reversed(self._snapshot(self.gate_states)):
                if (
                    message.gate_instance_id
                    and (
                        previous_instance is None
                        or message.gate_instance_id != previous_instance
                    )
                ):
                    return deepcopy(message)
            return None

        return self.wait_until(ready, timeout, 'MotionGate instance')

    def wait_clock(self, timeout: float = 15.0) -> int:
        return self.wait_until(
            lambda: next(
                (
                    value
                    for _, value in reversed(self._snapshot(self.clock_samples))
                    if value > 0
                ),
                None,
            ),
            timeout,
            'advancing simulation clock',
        )

    def _wait_future(self, future: Any, timeout: float, description: str):
        return self.wait_until(
            lambda: future.result() if future.done() else None,
            timeout,
            description,
        )

    def send_goal(
        self,
        state: MissionState,
        *,
        source_instance_id: str,
        source_seq: int,
        distance_m: float,
    ):
        self.wait_until(
            lambda: self.action_client.wait_for_server(timeout_sec=0.2),
            20.0,
            'Mission Action server',
        )
        goal = ExecuteMission.Goal()
        goal.source_instance_id = source_instance_id
        goal.source_seq = source_seq
        goal.runtime_instance_id = state.runtime_instance_id
        goal.admission_epoch = state.admission_epoch
        step = MissionStep()
        step.kind = MissionStep.MOVE_DISTANCE
        step.distance_m = distance_m
        goal.steps.append(step)
        handle = self._wait_future(
            self.action_client.send_goal_async(goal),
            10.0,
            'Mission Goal response',
        )
        if not handle.accepted:
            raise AssertionError('Mission Goal was rejected by the server')
        return handle

    def wait_goal_result(self, handle: Any, timeout: float = 30.0):
        wrapped = self._wait_future(
            handle.get_result_async(),
            timeout,
            'Mission Result',
        )
        return wrapped.status, wrapped.result

    def _latest_nonzero(self, collection: deque):
        for receipt_ns, message in reversed(self._snapshot(collection)):
            if not is_zero(message):
                return receipt_ns, deepcopy(message)
        return None

    def wait_for_armed_motion(self, timeout: float = 15.0):
        stable_since = None
        stable_clock_ns = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state_sample = self.latest(self.gate_states)
            final_sample = self.latest(self.final_commands)
            limited_sample = self.latest(self.limited_commands)
            clock_sample = self.latest(self.clock_samples)
            good = (
                state_sample is not None
                and state_sample[1].state == InternalMotionGateState.ARMED
                and state_sample[1].authority_live
                and state_sample[1].candidate_fresh
                and final_sample is not None
                and not is_zero(final_sample[1])
                and limited_sample is not None
                and not is_zero(limited_sample[1])
                and clock_sample is not None
                and clock_sample[1] > 0
            )
            if good:
                if stable_since is None:
                    stable_since = time.monotonic_ns()
                    stable_clock_ns = clock_sample[1]
                if (
                    time.monotonic_ns() - stable_since >= STATIONARY_HOLD_NS
                    and stable_clock_ns is not None
                    and clock_sample[1] - stable_clock_ns >= STATIONARY_HOLD_NS
                ):
                    final_nonzero = self._latest_nonzero(self.final_commands)
                    limited_nonzero = self._latest_nonzero(
                        self.limited_commands
                    )
                    if final_nonzero is None or limited_nonzero is None:
                        raise AssertionError(
                            'non-zero motion samples disappeared before kill'
                        )
                    return {
                        'gate': deepcopy(state_sample[1]),
                        'final': final_nonzero,
                        'limited': limited_nonzero,
                        'clock_ns': clock_sample[1],
                        'observed_ns': time.monotonic_ns(),
                    }
            else:
                stable_since = None
                stable_clock_ns = None
            time.sleep(0.01)
        raise AssertionError(
            'did not observe 200 ms of armed, moving Runtime/Gate pipeline'
        )

    def capture_motion_at_signal_boundary(self):
        """Freeze the moving proof immediately before pidfd SIGKILL."""
        with self.lock:
            state_sample = (
                self.gate_states[-1] if self.gate_states else None
            )
            final_sample = (
                self.final_commands[-1] if self.final_commands else None
            )
            limited_sample = (
                self.limited_commands[-1]
                if self.limited_commands else None
            )
            clock_sample = (
                self.clock_samples[-1] if self.clock_samples else None
            )
            previous_clock_sample = (
                self.clock_samples[-2]
                if len(self.clock_samples) >= 2
                else None
            )
            observed_ns = time.monotonic_ns()
            samples = (
                state_sample,
                final_sample,
                limited_sample,
                clock_sample,
                previous_clock_sample,
            )
            if any(sample is None for sample in samples):
                raise AssertionError(
                    'motion proof was incomplete at the pidfd signal boundary'
                )
            state = state_sample[1]
            if not (
                state.state == InternalMotionGateState.ARMED
                and state.authority_live
                and state.candidate_fresh
                and not state.motion_inhibited
                and not state.zero_selected
                and not is_zero(final_sample[1])
                and not is_zero(limited_sample[1])
                and clock_sample[1] > 0
                and previous_clock_sample is not None
                and clock_sample[1] > previous_clock_sample[1]
            ):
                raise AssertionError(
                    'target was not armed and moving at the pidfd signal boundary'
                )
            if any(
                observed_ns - sample[0] > DEPENDENCY_FRESHNESS_NS
                for sample in samples
            ):
                raise AssertionError(
                    'motion proof was stale at the pidfd signal boundary'
                )
            return {
                'gate': deepcopy(state),
                'final': (final_sample[0], deepcopy(final_sample[1])),
                'limited': (
                    limited_sample[0],
                    deepcopy(limited_sample[1]),
                ),
                'clock': clock_sample,
                'observed_ns': observed_ns,
            }

    def wait_runtime_zero(
        self,
        kill_ack_ns: int,
        prekill_zero_publish_seq: int,
    ):
        gate_zero = None
        final_zero = None
        limited_zero = None
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if gate_zero is None:
                for receipt_ns, message in self._snapshot(self.gate_states):
                    if (
                        receipt_ns >= kill_ack_ns
                        and message.zero_publish_seq
                        > prekill_zero_publish_seq
                        and gate_zero_proven(message)
                    ):
                        gate_zero = (receipt_ns, deepcopy(message))
                        break
            if final_zero is None:
                for receipt_ns, message in self._snapshot(self.final_commands):
                    if receipt_ns >= kill_ack_ns and is_zero(message):
                        final_zero = (receipt_ns, deepcopy(message))
                        break
            if limited_zero is None:
                for receipt_ns, message in self._snapshot(self.limited_commands):
                    if receipt_ns >= kill_ack_ns and is_zero(message):
                        limited_zero = (receipt_ns, deepcopy(message))
                        break
            if gate_zero and final_zero and limited_zero:
                zero_ns = max(
                    gate_zero[0], final_zero[0], limited_zero[0]
                )
                latency_ns = zero_ns - kill_ack_ns
                if latency_ns > GATE_ZERO_DEADLINE_NS:
                    raise AssertionError(
                        f'Runtime crash zero latency exceeded 350 ms: '
                        f'{latency_ns / 1_000_000:.3f} ms'
                    )
                return {
                    'gate_zero_ns': gate_zero[0],
                    'final_zero_ns': final_zero[0],
                    'limited_zero_ns': limited_zero[0],
                    'zero_ns': zero_ns,
                    'latency_ns': latency_ns,
                    'zero_sim_ns': _stamp_ns(final_zero[1]),
                }
            time.sleep(0.01)
        raise AssertionError(
            'Runtime SIGKILL did not produce Gate/final/controller zero proof'
        )

    def wait_zero_after(self, start_receipt_ns: int):
        gate_zero = None
        final_zero = None
        limited_zero = None
        deadline = time.monotonic() + WALL_WATCHDOG_SECONDS
        while time.monotonic() < deadline:
            if gate_zero is None:
                for receipt_ns, message in self._snapshot(self.gate_states):
                    if receipt_ns >= start_receipt_ns and gate_zero_proven(
                        message
                    ):
                        gate_zero = (receipt_ns, deepcopy(message))
                        break
            if final_zero is None:
                for receipt_ns, message in self._snapshot(self.final_commands):
                    if receipt_ns >= start_receipt_ns and is_zero(message):
                        final_zero = (receipt_ns, deepcopy(message))
                        break
            if limited_zero is None:
                for receipt_ns, message in self._snapshot(self.limited_commands):
                    if receipt_ns >= start_receipt_ns and is_zero(message):
                        limited_zero = (receipt_ns, deepcopy(message))
                        break
            if gate_zero and final_zero and limited_zero:
                zero_receipt_ns = max(
                    gate_zero[0], final_zero[0], limited_zero[0]
                )
                final_after_zero = next(
                    (
                        (receipt_ns, deepcopy(message))
                        for receipt_ns, message in self._snapshot(
                            self.final_commands
                        )
                        if receipt_ns >= zero_receipt_ns and is_zero(message)
                    ),
                    final_zero,
                )
                return {
                    'gate_zero_ns': gate_zero[0],
                    'final_zero_ns': final_zero[0],
                    'limited_zero_ns': limited_zero[0],
                    'zero_ns': zero_receipt_ns,
                    'zero_sim_ns': _stamp_ns(final_after_zero[1]),
                }
            time.sleep(0.01)
        raise AssertionError('no bounded final/controller zero proof observed')

    def wait_consumer_zero(
        self, kill_ack_ns: int, signal_boundary_sim_ns: int
    ):
        deadline = time.monotonic() + WALL_WATCHDOG_SECONDS
        previous_clock = self.latest(self.clock_samples)
        while time.monotonic() < deadline:
            current_clock = self.latest(self.clock_samples)
            if (
                previous_clock is not None
                and current_clock is not None
                and current_clock[1] <= previous_clock[1]
                and time.monotonic_ns() - kill_ack_ns > 1_000_000_000
            ):
                raise AssertionError(
                    'simulation clock stopped during consumer-timeout measurement'
                )
            if current_clock is not None:
                previous_clock = current_clock
            candidate = None
            for receipt_ns, message in self._snapshot(self.limited_commands):
                stamp_ns = _stamp_ns(message)
                if (
                    receipt_ns >= kill_ack_ns
                    and stamp_ns > signal_boundary_sim_ns
                    and is_zero(message)
                ):
                    candidate = (receipt_ns, deepcopy(message))
                    break
            if candidate is not None:
                return {
                    'zero_sim_ns': _stamp_ns(candidate[1]),
                    'zero_receipt_ns': candidate[0],
                }
            time.sleep(0.01)
        raise AssertionError(
            'controller cmd_vel_out did not select zero before the watchdog'
        )

    def wait_confirm_consumer_timeout(
        self,
        signal_boundary_sim_ns: int,
        consumer_zero: dict[str, int],
        *,
        timeout: float = WALL_WATCHDOG_SECONDS,
    ):
        deadline = time.monotonic() + timeout
        stable_key = None
        stable_since_ns = None
        stable_clock_ns = None
        while time.monotonic() < deadline:
            now_ns = time.monotonic_ns()
            with self.lock:
                final_snapshot = tuple(self.final_commands)
                clock_sample = (
                    self.clock_samples[-1]
                    if self.clock_samples
                    else None
                )
            if not final_snapshot or clock_sample is None:
                stable_key = None
                stable_since_ns = None
                stable_clock_ns = None
                time.sleep(0.01)
                continue

            last_receipt_ns, last_message = final_snapshot[-1]
            current_key = (
                len(final_snapshot),
                last_receipt_ns,
                _stamp_ns(last_message),
            )
            if current_key != stable_key:
                stable_key = current_key
                stable_since_ns = now_ns
                stable_clock_ns = clock_sample[1]
            elif (
                stable_since_ns is not None
                and stable_clock_ns is not None
                and now_ns - stable_since_ns >= FINAL_STREAM_QUIESCENCE_NS
                and clock_sample[1] - stable_clock_ns
                >= FINAL_STREAM_QUIESCENCE_NS
            ):
                with self.lock:
                    confirmed_snapshot = tuple(self.final_commands)
                    confirmed_clock = (
                        self.clock_samples[-1]
                        if self.clock_samples
                        else None
                    )
                    confirmed_last = (
                        confirmed_snapshot[-1]
                        if confirmed_snapshot
                        else None
                    )
                    confirmed_key = (
                        len(confirmed_snapshot),
                        confirmed_last[0],
                        _stamp_ns(confirmed_last[1]),
                    ) if confirmed_last is not None else None
                    if (
                        confirmed_key == stable_key
                        and confirmed_clock is not None
                        and confirmed_clock[1] - stable_clock_ns
                        >= FINAL_STREAM_QUIESCENCE_NS
                    ):
                        return consumer_timeout_result(
                            confirmed_snapshot,
                            signal_boundary_sim_ns=(
                                signal_boundary_sim_ns
                            ),
                            zero_sim_ns=consumer_zero['zero_sim_ns'],
                            zero_receipt_ns=(
                                consumer_zero['zero_receipt_ns']
                            ),
                        )
                stable_key = None
                stable_since_ns = None
                stable_clock_ns = None
            time.sleep(0.01)
        raise AssertionError(
            'final command observer did not quiesce with advancing clock'
        )

    @staticmethod
    def _wheels_stationary(message: JointState) -> bool:
        velocities = dict(zip(message.name, message.velocity))
        return all(
            name in velocities
            and abs(velocities[name]) <= ZERO_WHEEL_TOLERANCE
            for name in ('left_wheel_joint', 'right_wheel_joint')
        )

    @staticmethod
    def _odom_stationary(message: Odometry) -> bool:
        twist = message.twist.twist
        return (
            abs(twist.linear.x) <= ZERO_LINEAR_TOLERANCE
            and abs(twist.angular.z) <= ZERO_ANGULAR_TOLERANCE
        )

    def wait_stationary(
        self,
        zero_sim_ns: int,
        zero_receipt_ns: int,
        *,
        timeout: float = WALL_WATCHDOG_SECONDS,
    ):
        deadline = time.monotonic() + timeout
        hold_start = None
        last_sim_ns = zero_sim_ns
        last_receipt_ns = zero_receipt_ns
        while time.monotonic() < deadline:
            odom_samples = self._snapshot(self.odometry)
            joint = self.latest(self.joint_states)
            for receipt_ns, odom in odom_samples:
                if receipt_ns <= last_receipt_ns:
                    continue
                stamp_ns = _stamp_ns(odom)
                if receipt_ns < zero_receipt_ns or stamp_ns <= zero_sim_ns:
                    continue
                if stamp_ns < last_sim_ns:
                    raise AssertionError('odom simulation stamp regressed')
                if stamp_ns - zero_sim_ns > STATIONARY_DEADLINE_NS:
                    raise AssertionError(
                        'odom did not reach the stationary tolerance within '
                        '1.2 s simulation time'
                    )
                stationary = (
                    self._odom_stationary(odom)
                    and joint is not None
                    and self._wheels_stationary(joint[1])
                )
                if stationary:
                    if hold_start is None:
                        hold_start = stamp_ns
                    if stamp_ns - hold_start >= STATIONARY_HOLD_NS:
                        return {
                            'zero_sim_ns': zero_sim_ns,
                            'stationary_start_sim_ns': hold_start,
                            'stationary_end_sim_ns': stamp_ns,
                            'settle_ns': hold_start - zero_sim_ns,
                            'hold_ns': stamp_ns - hold_start,
                        }
                else:
                    hold_start = None
                last_sim_ns = stamp_ns
                last_receipt_ns = receipt_ns
            time.sleep(0.01)
        raise AssertionError(
            'stationarity watchdog expired before a 200 ms simulation hold'
        )

    def wait_new_gate_zero(self, previous_instance: str):
        return self.wait_until(
            lambda: next(
                (
                    (receipt_ns, deepcopy(message))
                    for receipt_ns, message in self._snapshot(
                        self.gate_states
                    )
                    if (
                        receipt_ns >= 0
                        and message.gate_instance_id != previous_instance
                        and gate_zero_proven(message)
                    )
                ),
                None,
            ),
            20.0,
            'new Gate inhibited zero proof',
        )

    def assert_no_goal_zero_window(self, start_receipt_ns: int):
        start_clock = self.wait_clock()
        deadline = time.monotonic() + WALL_WATCHDOG_SECONDS
        while time.monotonic() < deadline:
            for receipt_ns, message in self._snapshot(self.final_commands):
                if receipt_ns >= start_receipt_ns and not is_zero(message):
                    raise AssertionError(
                        'non-zero final command appeared without a new Goal'
                    )
            for receipt_ns, message in self._snapshot(self.limited_commands):
                if receipt_ns >= start_receipt_ns and not is_zero(message):
                    raise AssertionError(
                        'non-zero controller command appeared without a new Goal'
                    )
            for receipt_ns, message in self._snapshot(self.gate_states):
                if receipt_ns >= start_receipt_ns and not gate_zero_proven(
                    message
                ):
                    raise AssertionError(
                        'Gate left its inhibited zero state without a new Goal'
                    )
            for receipt_ns, message in self._snapshot(self.odometry):
                if receipt_ns >= start_receipt_ns and not self._odom_stationary(
                    message
                ):
                    raise AssertionError(
                        'non-stationary odometry appeared without a new Goal'
                    )
            for receipt_ns, message in self._snapshot(self.joint_states):
                if receipt_ns >= start_receipt_ns and not self._wheels_stationary(
                    message
                ):
                    raise AssertionError(
                        'non-zero wheel velocity appeared without a new Goal'
                    )
            current_clock = self.latest(self.clock_samples)
            if current_clock and current_clock[1] >= start_clock + NO_GOAL_WINDOW_NS:
                return {
                    'start_sim_ns': start_clock,
                    'end_sim_ns': current_clock[1],
                }
            time.sleep(0.01)
        raise AssertionError(
            'no-Goal safety window did not complete with advancing simulation'
        )

    def assert_unique_final_owner(self) -> str:
        endpoints = self.node.get_publishers_info_by_topic(
            FINAL_COMMAND_TOPIC
        )
        matching = [
            endpoint
            for endpoint in endpoints
            if endpoint.node_name == GATE_NODE
            and endpoint.node_namespace == '/'
        ]
        if len(endpoints) != 1 or len(matching) != 1:
            raise AssertionError(
                'MotionGate is not the unique final-command publisher'
            )
        return bytes(matching[0].endpoint_gid).hex()

    def publisher_count(self, topic: str) -> int:
        return len(self.node.get_publishers_info_by_topic(topic))

    def wait_no_publishers(self, topic: str, timeout: float = 10.0) -> None:
        self.wait_until(
            lambda: self.publisher_count(topic) == 0,
            timeout,
            f'no publishers on stale topic {topic}',
        )

    def assert_stale_gate_tuple(self, old_state: InternalMotionGateState):
        self.wait_until(
            lambda: self.gate_client.wait_for_service(timeout_sec=0.2),
            10.0,
            'new Gate control service',
        )
        before = self.latest(self.gate_states)
        request = InternalMotionGateControl.Request()
        request.operation = InternalMotionGateControl.Request.RENEW
        request.request_id = secrets.token_hex(16)
        request.gate_instance_id = old_state.gate_instance_id
        request.expected_control_seq = old_state.control_seq
        request.lease_id = old_state.lease_id
        request_start_ns = time.monotonic_ns()
        response = self._wait_future(
            self.gate_client.call_async(request),
            5.0,
            'stale Gate tuple response',
        )
        if response.code != InternalMotionGateControl.Response.REJECTED:
            raise AssertionError(
                'old Gate tuple was accepted: ' + response.detail
            )
        if response.reason not in (
            InternalMotionGateControl.Response.STALE_GATE,
            InternalMotionGateControl.Response.STALE_SEQUENCE,
            InternalMotionGateControl.Response.STALE_LEASE,
        ):
            raise AssertionError(
                f'old Gate tuple returned unexpected reason={response.reason}'
            )
        time.sleep(0.1)
        after = self.latest(self.gate_states)
        if before is None or after is None:
            raise AssertionError('Gate snapshot missing around stale replay')
        stable_fields = (
            'gate_instance_id',
            'control_seq',
            'state',
            'lease_id',
            'candidate_topic',
            'bound_writer_gid',
            'motion_inhibited',
            'authority_live',
            'candidate_fresh',
            'writer_bound',
            'zero_selected',
            'reason',
        )
        for field in stable_fields:
            if not state_observation.values_equal(
                getattr(after[1], field), getattr(before[1], field)
            ):
                raise AssertionError(
                    f'stale replay changed Gate field {field}'
                )
        observed_gate = False
        for receipt_ns, message in self._snapshot(self.gate_states):
            if receipt_ns < request_start_ns:
                continue
            observed_gate = True
            if not gate_zero_proven(message):
                raise AssertionError(
                    'stale replay interrupted the continuous Gate zero proof'
                )
        if not observed_gate:
            raise AssertionError('no Gate state observed during stale replay')
        observed_final = False
        for receipt_ns, message in self._snapshot(self.final_commands):
            if receipt_ns < request_start_ns:
                continue
            observed_final = True
            if not is_zero(message):
                raise AssertionError(
                    'stale replay published a non-zero final command'
                )
        if not observed_final:
            raise AssertionError('no final command observed during stale replay')
        return {
            'code': int(response.code),
            'reason': int(response.reason),
            'detail': response.detail,
        }

    def publish_old_candidate(self, topic: str, duration: float = 0.5):
        publisher = self.node.create_publisher(
            TwistStamped,
            topic,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.candidate_publishers.append(publisher)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            message = TwistStamped()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.twist.linear.x = 0.25
            publisher.publish(message)
            time.sleep(0.02)
        self.node.destroy_publisher(publisher)
        self.candidate_publishers.remove(publisher)

    def inhibit_for_cleanup(self):
        if not rclpy.ok() or not self.gate_client.service_is_ready():
            return
        sample = self.latest(self.gate_states)
        if sample is None or not sample[1].lease_id:
            return
        message = sample[1]
        request = InternalMotionGateControl.Request()
        request.operation = InternalMotionGateControl.Request.INHIBIT
        request.request_id = secrets.token_hex(16)
        request.gate_instance_id = message.gate_instance_id
        request.expected_control_seq = message.control_seq
        request.lease_id = message.lease_id
        future = self.gate_client.call_async(request)
        self._wait_future(future, 3.0, 'cleanup Gate inhibit')

    def destroy(self):
        for publisher in tuple(self.candidate_publishers):
            self.node.destroy_publisher(publisher)
        self.candidate_publishers.clear()
        self.action_client.destroy()
        self.executor.shutdown(timeout_sec=2.0)
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=2.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def evidence(self, scenario: str, **values: Any) -> None:
        payload = {'scenario': scenario, **values}
        print(
            'EVIDENCE '
            + scenario
            + ' '
            + json.dumps(payload, sort_keys=True, separators=(',', ':')),
            flush=True,
        )

    def diagnostic(self) -> dict[str, Any]:
        def message_summary(message: Any):
            if isinstance(message, InternalMotionGateState):
                return {
                    'instance': message.gate_instance_id,
                    'state': int(message.state),
                    'control_seq': int(message.control_seq),
                    'lease': message.lease_id,
                    'inhibited': bool(message.motion_inhibited),
                    'zero_selected': bool(message.zero_selected),
                    'output_seq': int(message.output_publish_seq),
                    'zero_seq': int(message.zero_publish_seq),
                }
            if isinstance(message, MissionState):
                return {
                    'runtime': message.runtime_instance_id,
                    'epoch': int(message.admission_epoch),
                    'operating_mode': int(message.operating_mode),
                    'availability': int(message.availability),
                    'gate_state': int(message.gate_state),
                    'active_step': int(message.active_step),
                    'supported_step_mask': int(message.supported_step_mask),
                    'max_steps': int(message.max_steps),
                    'named_place_ids': list(message.named_place_ids),
                }
            if isinstance(message, TwistStamped):
                return {
                    'stamp_ns': _stamp_ns(message),
                    'linear_x': float(message.twist.linear.x),
                    'angular_z': float(message.twist.angular.z),
                }
            if isinstance(message, LaserScan):
                return {
                    'stamp_ns': _stamp_ns(message),
                    'frame_id': message.header.frame_id,
                    'range_count': len(message.ranges),
                    'range_min': float(message.range_min),
                    'range_max': float(message.range_max),
                }
            if isinstance(message, Odometry):
                return {
                    'stamp_ns': _stamp_ns(message),
                    'linear_x': float(message.twist.twist.linear.x),
                    'angular_z': float(message.twist.twist.angular.z),
                }
            return message

        def sample_summary(collection: deque):
            return [
                {
                    **message_summary(message),
                    'observer_receipt_monotonic_ns': receipt_ns,
                }
                for receipt_ns, message in self._snapshot(collection)[-5:]
            ]

        def stream_freshness(
            collection: deque,
            stamp_getter: Callable[[Any], int],
        ):
            sample = self.latest(collection)
            if sample is None:
                return {
                    'observed': False,
                    'receipt_monotonic_ns': None,
                    'age_monotonic_ns': None,
                    'fresh_steady_200ms': False,
                    'stamp_ns': None,
                }
            receipt_ns, message = sample
            age_ns = max(0, time.monotonic_ns() - receipt_ns)
            try:
                stamp_ns = int(stamp_getter(message))
            except Exception:
                stamp_ns = None
            return {
                'observed': True,
                'receipt_monotonic_ns': receipt_ns,
                'age_monotonic_ns': age_ns,
                'fresh_steady_200ms': age_ns <= DEPENDENCY_FRESHNESS_NS,
                'stamp_ns': stamp_ns,
            }

        mission_health_transitions = []
        previous_health = None
        for receipt_ns, message in self._snapshot(self.mission_states):
            current_health = {
                'availability': int(message.availability),
                'gate_state': int(message.gate_state),
                'epoch': int(message.admission_epoch),
            }
            if current_health != previous_health:
                mission_health_transitions.append({
                    'receipt_monotonic_ns': receipt_ns,
                    'from': previous_health,
                    'to': current_health,
                })
            previous_health = current_health

        clock_snapshot = self._snapshot(self.clock_samples)
        clock_advanced = (
            len(clock_snapshot) >= 2
            and clock_snapshot[-1][1] > clock_snapshot[-2][1]
        )
        try:
            ros_time_active = bool(self.node.get_clock().ros_time_is_active())
            ros_time_now_ns = int(self.node.get_clock().now().nanoseconds)
        except Exception:
            ros_time_active = False
            ros_time_now_ns = 0

        return {
            'gate': sample_summary(self.gate_states),
            'mission': sample_summary(self.mission_states),
            'final': sample_summary(self.final_commands),
            'limited': sample_summary(self.limited_commands),
            'scan': sample_summary(self.scan_samples),
            'odom': sample_summary(self.odometry),
            'clock': [
                value for _, value in self._snapshot(self.clock_samples)[-5:]
            ],
            'clock_samples': [
                {
                    'stamp_ns': value,
                    'observer_receipt_monotonic_ns': receipt_ns,
                }
                for receipt_ns, value in self._snapshot(
                    self.clock_samples
                )[-5:]
            ],
            'freshness': {
                'scan': stream_freshness(self.scan_samples, _stamp_ns),
                'odom': stream_freshness(self.odometry, _stamp_ns),
                'clock': stream_freshness(
                    self.clock_samples, lambda value: value
                ),
                'ros_time_active': ros_time_active,
                'ros_time_now_ns': ros_time_now_ns,
                'clock_advanced': clock_advanced,
            },
            'mission_health_transitions': mission_health_transitions[-8:],
        }


def assert_action_result(
    status: int,
    result: ExecuteMission.Result,
    expected_code: int,
) -> None:
    if expected_code == ExecuteMission.Result.SUCCEEDED:
        expected_status = GoalStatus.STATUS_SUCCEEDED
    else:
        expected_status = GoalStatus.STATUS_ABORTED
    if status != expected_status:
        raise AssertionError(f'unexpected Action status: {status}')
    if result.code != expected_code:
        raise AssertionError(
            f'unexpected Mission result code={result.code}, detail={result.detail}'
        )
