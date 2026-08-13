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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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


@dataclass(frozen=True)
class WorkspaceInterfaceTypes:
    """Generated workspace types required only by a running ROS probe."""

    execute_mission: Any
    mission_state: Any
    mission_step: Any
    gate_state: Any
    gate_control: Any


@dataclass(frozen=True)
class CommandObservation:
    """One observed command with optional Jazzy publication metadata."""

    receipt_ns: int
    message: Any
    publication_sequence_number: int | None = None
    source_timestamp_ns: int | None = None
    received_timestamp_ns: int | None = None
    reception_sequence_number: int | None = None
    final_subscription_identity: str | None = None
    observation_epoch: int | None = None
    header_stamp_ns: int | None = None
    twist_signature: tuple[float, ...] | None = None

    def __iter__(self):
        return iter((self.receipt_ns, self.message))

    def __getitem__(self, index: int):
        return (self.receipt_ns, self.message)[index]


def _load_workspace_interface_types() -> WorkspaceInterfaceTypes:
    from voice_nav_interfaces.action import ExecuteMission
    from voice_nav_interfaces.msg import MissionState, MissionStep
    from voice_nav_mission.msg import InternalMotionGateState
    from voice_nav_mission.srv import InternalMotionGateControl

    return WorkspaceInterfaceTypes(
        execute_mission=ExecuteMission,
        mission_state=MissionState,
        mission_step=MissionStep,
        gate_state=InternalMotionGateState,
        gate_control=InternalMotionGateControl,
    )


_workspace_interface_types: WorkspaceInterfaceTypes | None = None
_workspace_interface_types_lock = threading.Lock()


def _get_workspace_interface_types() -> WorkspaceInterfaceTypes:
    global _workspace_interface_types
    interfaces = _workspace_interface_types
    if interfaces is None:
        with _workspace_interface_types_lock:
            interfaces = _workspace_interface_types
            if interfaces is None:
                interfaces = _load_workspace_interface_types()
                _workspace_interface_types = interfaces
    return interfaces


def __getattr__(name: str) -> Any:
    interface_name = {
        'ExecuteMission': 'execute_mission',
        'MissionState': 'mission_state',
        'MissionStep': 'mission_step',
        'InternalMotionGateState': 'gate_state',
        'InternalMotionGateControl': 'gate_control',
    }.get(name)
    if interface_name is None:
        raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
    return getattr(_get_workspace_interface_types(), interface_name)


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


class _LazyGazeboShutdown:
    """Preserve the launch-test API without resolving package share on import."""

    def __init__(self) -> None:
        self._module = None
        self._lock = threading.Lock()

    def __getattr__(self, name: str) -> Any:
        module = self._module
        if module is None:
            with self._lock:
                module = self._module
                if module is None:
                    module = _load_gazebo_shutdown()
                    self._module = module
        return getattr(module, name)


gazebo_shutdown = _LazyGazeboShutdown()


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


class ConsumerTraceAmbiguous(AssertionError):
    """Fail closed when observer topics cannot prove controller consumption."""

    def __init__(self, reason: str, **details: Any) -> None:
        self.evidence = {
            'status': 'ambiguous',
            'reason': reason,
            **details,
        }
        super().__init__(
            'controller consumer evidence ambiguous: '
            + json.dumps(self.evidence, sort_keys=True)
        )


class ConsumerTraceWatermarkTimeout(AssertionError):
    """Fail closed when the controller zero stream cannot reach its watermark."""

    def __init__(self, **details: Any) -> None:
        self.evidence = {
            'status': 'timeout',
            'reason': 'limited zero watermark did not advance',
            **details,
        }
        super().__init__(
            'controller consumer evidence incomplete: '
            + json.dumps(self.evidence, sort_keys=True)
        )


class ConsumerTimeoutOutOfBounds(AssertionError):
    """Preserve a proved controller association when its timeout is unsafe."""

    def __init__(
        self,
        trace: dict[str, Any],
        *,
        delta_ns: int,
    ) -> None:
        self.evidence = {
            'status': 'failed',
            'reason': 'controller consumer timeout outside accepted window',
            'authoritative_source_stamp_ns': trace['source_header_stamp_ns'],
            'controller_nonzero_update_stamp_ns': (
                trace['controller_update_stamp_ns']
            ),
            'controller_zero_update_stamp_ns': (
                trace['controller_zero_update_stamp_ns']
            ),
            'delta_ns': delta_ns,
            'accepted_window_ns': [
                CONSUMER_ZERO_MIN_NS,
                CONSUMER_ZERO_MAX_NS,
            ],
            'association_basis': trace['association_basis'],
            'association': trace,
        }
        super().__init__(
            'controller consumer timeout outside '
            f'(0.35, 0.36] s: {delta_ns / 1_000_000_000:.6f} s; '
            f'last_nonzero_sim_ns={trace["source_header_stamp_ns"]}; '
            f'zero_sim_ns={trace["controller_zero_update_stamp_ns"]}'
        )


def _twist_signature(message: TwistStamped) -> tuple[float, ...]:
    twist = message.twist
    return (
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    )


def _final_observation_metadata(
    observation: Any,
    *,
    checkpoint: str,
) -> dict[str, int | str | None]:
    """Return the Jazzy-observable identity and sequence for one final sample."""
    subscription_identity = getattr(
        observation, 'final_subscription_identity', None
    )
    if not isinstance(subscription_identity, str) or not subscription_identity:
        raise ConsumerTraceAmbiguous(
            'final subscription identity unavailable',
            checkpoint=checkpoint,
            receipt_ns=observation[0],
            header_stamp_ns=_stamp_ns(observation[1]),
        )
    publication_sequence_number = getattr(
        observation, 'publication_sequence_number', None
    )
    if (
        not isinstance(publication_sequence_number, int)
        or isinstance(publication_sequence_number, bool)
        or publication_sequence_number < 0
    ):
        raise ConsumerTraceAmbiguous(
            'final publication sequence unavailable',
            checkpoint=checkpoint,
            receipt_ns=observation[0],
            header_stamp_ns=_stamp_ns(observation[1]),
        )

    def optional_jazzy_timestamp(name: str) -> int | None:
        value = getattr(observation, name, None)
        if value is None:
            return None
        if not isinstance(value, int) or isinstance(value, bool):
            raise ConsumerTraceAmbiguous(
                'final Jazzy metadata was malformed',
                checkpoint=checkpoint,
                field=name,
                receipt_ns=observation[0],
                header_stamp_ns=_stamp_ns(observation[1]),
            )
        return value

    return {
        'final_subscription_identity': subscription_identity,
        'publication_sequence_number': publication_sequence_number,
        'source_timestamp_ns': optional_jazzy_timestamp('source_timestamp_ns'),
        'received_timestamp_ns': optional_jazzy_timestamp(
            'received_timestamp_ns'
        ),
        'reception_sequence_number': optional_jazzy_timestamp(
            'reception_sequence_number'
        ),
    }


def _final_endpoint_continuity_fence(
    final_endpoint_fence: dict[str, Any],
) -> dict[str, int | str | None]:
    """Validate the graph and subscription boundary frozen before SIGKILL."""
    endpoint_gid = final_endpoint_fence.get('endpoint_gid')
    if (
        not isinstance(endpoint_gid, str)
        or not endpoint_gid
        or all(character == '0' for character in endpoint_gid)
    ):
        raise ConsumerTraceAmbiguous(
            'final endpoint identity unavailable',
            checkpoint='signal-boundary',
            endpoint_gid=(endpoint_gid if isinstance(endpoint_gid, str) else ''),
        )
    subscription_identity = final_endpoint_fence.get(
        'final_subscription_identity'
    )
    if not isinstance(subscription_identity, str) or not subscription_identity:
        raise ConsumerTraceAmbiguous(
            'final subscription identity unavailable',
            checkpoint='signal-boundary',
        )
    fields = {
        'final_receipt_fence_ns': final_endpoint_fence.get(
            'final_receipt_fence_ns'
        ),
        'final_header_stamp_ns': final_endpoint_fence.get(
            'final_header_stamp_ns'
        ),
        'final_publication_sequence_number': final_endpoint_fence.get(
            'final_publication_sequence_number'
        ),
    }
    for name, value in fields.items():
        if not isinstance(value, int) or isinstance(value, bool):
            raise ConsumerTraceAmbiguous(
                'final endpoint continuity fence was incomplete',
                checkpoint='signal-boundary',
                field=name,
            )
    optional_fields = {
        'final_source_timestamp_ns': final_endpoint_fence.get(
            'final_source_timestamp_ns'
        ),
        'final_received_timestamp_ns': final_endpoint_fence.get(
            'final_received_timestamp_ns'
        ),
        'final_reception_sequence_number': final_endpoint_fence.get(
            'final_reception_sequence_number'
        ),
    }
    for name, value in optional_fields.items():
        if value is not None and (
            not isinstance(value, int) or isinstance(value, bool)
        ):
            raise ConsumerTraceAmbiguous(
                'final endpoint continuity fence metadata was malformed',
                checkpoint='signal-boundary',
                field=name,
            )
    return {
        'endpoint_gid': endpoint_gid,
        'final_subscription_identity': subscription_identity,
        **fields,
        **optional_fields,
    }


def finalize_consumer_source_anchor(
    final_samples: tuple[tuple[int, Any], ...],
    *,
    motion_proof_anchor: tuple[int, Any],
    final_endpoint_fence: dict[str, Any],
) -> tuple[tuple[int, Any], dict[str, Any]]:
    """Freeze a drained final source without claiming per-message publisher GID.

    Jazzy rclpy exposes timestamps and sequence numbers, but not an RMW
    publisher GID in the subscription callback.  The graph endpoint is frozen
    before SIGKILL and proved absent afterwards; this helper proves that the
    finalized trace is one ordered delivery stream from the same local final
    subscription.  It deliberately does not infer a per-message GID.
    """
    fence = _final_endpoint_continuity_fence(final_endpoint_fence)

    motion_receipt_ns, motion_message = motion_proof_anchor
    motion_stamp_ns = _stamp_ns(motion_message)
    motion_metadata = _final_observation_metadata(
        motion_proof_anchor,
        checkpoint='motion-proof',
    )
    if (
        motion_metadata['final_subscription_identity']
        != fence['final_subscription_identity']
        or motion_metadata['publication_sequence_number']
        != fence['final_publication_sequence_number']
        or motion_receipt_ns != fence['final_receipt_fence_ns']
        or motion_stamp_ns != fence['final_header_stamp_ns']
        or motion_metadata['source_timestamp_ns']
        != fence['final_source_timestamp_ns']
        or motion_metadata['received_timestamp_ns']
        != fence['final_received_timestamp_ns']
        or motion_metadata['reception_sequence_number']
        != fence['final_reception_sequence_number']
    ):
        raise ConsumerTraceAmbiguous(
            'motion proof final observation did not match endpoint continuity fence',
            motion_proof_receipt_ns=motion_receipt_ns,
            motion_proof_stamp_ns=motion_stamp_ns,
            motion_proof_metadata=motion_metadata,
            final_endpoint_fence=fence,
        )

    relevant_final, final_phase = _relevant_trace_phase(
        final_samples,
        source_anchor=motion_proof_anchor,
        topic=FINAL_COMMAND_TOPIC,
    )
    if not relevant_final:
        raise ConsumerTraceAmbiguous(
            'motion proof anchor was missing from final trace',
            motion_proof_receipt_ns=motion_receipt_ns,
            motion_proof_stamp_ns=motion_stamp_ns,
        )

    motion_indexes = [
        index
        for index, observation in enumerate(relevant_final)
        if (
            observation[0] == motion_receipt_ns
            and _stamp_ns(observation[1]) == motion_stamp_ns
            and _twist_signature(observation[1])
            == _twist_signature(motion_message)
            and _final_observation_metadata(
                observation,
                checkpoint='motion-proof-trace',
            )
            == motion_metadata
        )
    ]
    if len(motion_indexes) != 1:
        raise ConsumerTraceAmbiguous(
            'motion proof anchor was not uniquely observed in final trace',
            motion_proof_receipt_ns=motion_receipt_ns,
            motion_proof_stamp_ns=motion_stamp_ns,
            matching_motion_proof_count=len(motion_indexes),
            final_phase_isolation=final_phase,
        )

    previous_sequence = None
    for observation in relevant_final:
        receipt_ns, message = observation
        metadata = _final_observation_metadata(
            observation,
            checkpoint='finalized-trace',
        )
        if (
            metadata['final_subscription_identity']
            != fence['final_subscription_identity']
        ):
            raise ConsumerTraceAmbiguous(
                'final subscription identity changed during endpoint continuity',
                expected_final_subscription_identity=(
                    fence['final_subscription_identity']
                ),
                observed_final_subscription_identity=(
                    metadata['final_subscription_identity']
                ),
                observed_receipt_ns=receipt_ns,
                observed_stamp_ns=_stamp_ns(message),
                final_phase_isolation=final_phase,
            )
        sequence = metadata['publication_sequence_number']
        if previous_sequence is not None:
            if sequence == previous_sequence:
                raise ConsumerTraceAmbiguous(
                    'duplicate final publication sequence',
                    previous_sequence=previous_sequence,
                    publication_sequence_number=sequence,
                    final_phase_isolation=final_phase,
                )
            if sequence < previous_sequence:
                raise ConsumerTraceAmbiguous(
                    'final publication sequence regressed',
                    previous_sequence=previous_sequence,
                    publication_sequence_number=sequence,
                    final_phase_isolation=final_phase,
                )
            if sequence > previous_sequence + 1:
                raise ConsumerTraceAmbiguous(
                    'final publication sequence gap',
                    expected_sequence=previous_sequence + 1,
                    publication_sequence_number=sequence,
                    final_phase_isolation=final_phase,
                )
        previous_sequence = sequence

    source_anchor = relevant_final[-1]
    source_receipt_ns, source_message = source_anchor
    source_stamp_ns = _stamp_ns(source_message)
    if is_zero(source_message):
        raise ConsumerTraceAmbiguous(
            'finalized consumer source anchor was zero',
            final_endpoint_gid=fence['endpoint_gid'],
            source_receipt_ns=source_receipt_ns,
            source_header_stamp_ns=source_stamp_ns,
            final_phase_isolation=final_phase,
        )
    source_metadata = _final_observation_metadata(
        source_anchor,
        checkpoint='consumer-source',
    )
    return source_anchor, {
        'motion_proof_anchor': {
            'receipt_ns': motion_receipt_ns,
            'header_stamp_ns': motion_stamp_ns,
            **motion_metadata,
        },
        'consumer_source_anchor': {
            'receipt_ns': source_receipt_ns,
            'header_stamp_ns': source_stamp_ns,
            'twist_signature': list(_twist_signature(source_message)),
            **source_metadata,
        },
        'endpoint_continuity': {
            'graph_endpoint_gid': fence['endpoint_gid'],
            'final_subscription_identity': fence[
                'final_subscription_identity'
            ],
            'signal_boundary_final_receipt_ns': fence[
                'final_receipt_fence_ns'
            ],
            'signal_boundary_final_header_stamp_ns': fence[
                'final_header_stamp_ns'
            ],
            'signal_boundary_final_publication_sequence_number': fence[
                'final_publication_sequence_number'
            ],
            'finalized_publication_sequence_number': source_metadata[
                'publication_sequence_number'
            ],
        },
        'final_phase_isolation': final_phase,
    }


def _assert_strict_trace_order(
    samples: tuple[tuple[int, Any], ...],
    *,
    topic: str,
) -> None:
    previous_receipt_ns = None
    previous_stamp_ns = None
    for receipt_ns, message in samples:
        stamp_ns = _stamp_ns(message)
        if (
            previous_receipt_ns is not None
            and receipt_ns <= previous_receipt_ns
        ):
            raise ConsumerTraceAmbiguous(
                'out-of-order observer receipt',
                topic=topic,
                previous_receipt_ns=previous_receipt_ns,
                receipt_ns=receipt_ns,
            )
        if previous_stamp_ns is not None and stamp_ns <= previous_stamp_ns:
            raise ConsumerTraceAmbiguous(
                'out-of-order topic header stamp',
                topic=topic,
                previous_stamp_ns=previous_stamp_ns,
                stamp_ns=stamp_ns,
            )
        previous_receipt_ns = receipt_ns
        previous_stamp_ns = stamp_ns


def _relevant_trace_phase(
    samples: tuple[tuple[int, Any], ...],
    *,
    source_anchor: tuple[int, Any],
    topic: str,
) -> tuple[tuple[tuple[int, Any], ...], dict[str, Any]]:
    """Keep only the trace phase that cannot predate the source anchor."""
    source_receipt_ns, source_message = source_anchor
    source_stamp_ns = _stamp_ns(source_message)
    excluded_prefix_count = 0
    for receipt_ns, message in samples:
        if (
            receipt_ns < source_receipt_ns
            and _stamp_ns(message) < source_stamp_ns
        ):
            excluded_prefix_count += 1
            continue
        break

    excluded_prefix = samples[:excluded_prefix_count]
    relevant_samples = samples[excluded_prefix_count:]
    phase_evidence = {
        'topic': topic,
        'excluded_prefix_count': excluded_prefix_count,
        'excluded_prefix_boundary': (
            {
                'receipt_ns': excluded_prefix[-1][0],
                'header_stamp_ns': _stamp_ns(excluded_prefix[-1][1]),
            }
            if excluded_prefix
            else None
        ),
        'relevant_trace_start': (
            {
                'receipt_ns': relevant_samples[0][0],
                'header_stamp_ns': _stamp_ns(relevant_samples[0][1]),
            }
            if relevant_samples
            else None
        ),
    }
    for receipt_ns, message in relevant_samples:
        stamp_ns = _stamp_ns(message)
        if stamp_ns <= 0:
            raise ConsumerTraceAmbiguous(
                'non-positive topic header stamp in relevant trace phase',
                topic=topic,
                source_header_stamp_ns=source_stamp_ns,
                source_receipt_ns=source_receipt_ns,
                observed_header_stamp_ns=stamp_ns,
                observed_receipt_ns=receipt_ns,
                phase_isolation=phase_evidence,
            )
    try:
        _assert_strict_trace_order(relevant_samples, topic=topic)
    except ConsumerTraceAmbiguous as error:
        error.evidence['phase_isolation'] = phase_evidence
        raise
    return relevant_samples, phase_evidence


def _first_relevant_controller_zero(
    relevant_samples: tuple[tuple[int, Any], ...],
    *,
    source_anchor: tuple[int, Any],
    zero_sim_ns: int,
    zero_receipt_ns: int,
    phase_isolation: dict[str, Any],
) -> tuple[int, tuple[int, Any]]:
    """Return the first relevant controller zero or fail closed."""
    source_receipt_ns, source_message = source_anchor
    source_stamp_ns = _stamp_ns(source_message)
    for index, (receipt_ns, message) in enumerate(relevant_samples):
        if not is_zero(message):
            continue
        stamp_ns = _stamp_ns(message)
        receipt_conflict = receipt_ns < source_receipt_ns
        stamp_conflict = stamp_ns <= source_stamp_ns
        if receipt_conflict or stamp_conflict:
            raise ConsumerTraceAmbiguous(
                'first relevant controller zero conflicts with source anchor',
                source_header_stamp_ns=source_stamp_ns,
                source_receipt_ns=source_receipt_ns,
                first_relevant_zero_receipt_ns=receipt_ns,
                first_relevant_zero_stamp_ns=stamp_ns,
                receipt_conflict=receipt_conflict,
                stamp_conflict=stamp_conflict,
                phase_isolation=phase_isolation,
            )
        if receipt_ns != zero_receipt_ns or stamp_ns != zero_sim_ns:
            raise ConsumerTraceAmbiguous(
                'provided controller zero was not the first relevant zero observation',
                first_relevant_zero_receipt_ns=receipt_ns,
                first_relevant_zero_stamp_ns=stamp_ns,
                zero_receipt_ns=zero_receipt_ns,
                zero_sim_ns=zero_sim_ns,
                phase_isolation=phase_isolation,
            )
        return index, (receipt_ns, message)
    raise ConsumerTraceAmbiguous(
        'missing controller first zero observation',
        source_header_stamp_ns=source_stamp_ns,
        source_receipt_ns=source_receipt_ns,
        phase_isolation=phase_isolation,
    )


def controller_consumed_command_trace(
    final_samples: tuple[tuple[int, Any], ...],
    limited_samples: tuple[tuple[int, Any], ...],
    *,
    source_anchor: tuple[int, Any],
    zero_sim_ns: int,
    zero_receipt_ns: int,
) -> dict[str, Any]:
    """Prove the source command which the controller held before its first zero."""
    source_receipt_ns, source_message = source_anchor
    source_stamp_ns = _stamp_ns(source_message)
    source_signature = _twist_signature(source_message)
    if is_zero(source_message):
        raise ConsumerTraceAmbiguous(
            'frozen source anchor was zero',
            source_header_stamp_ns=source_stamp_ns,
            source_receipt_ns=source_receipt_ns,
        )

    source_indexes = [
        index
        for index, (receipt_ns, message) in enumerate(final_samples)
        if (
            receipt_ns == source_receipt_ns
            and _stamp_ns(message) == source_stamp_ns
            and _twist_signature(message) == source_signature
        )
    ]
    if not source_indexes:
        raise ConsumerTraceAmbiguous(
            'frozen source anchor was missing from final trace',
            source_header_stamp_ns=source_stamp_ns,
            source_receipt_ns=source_receipt_ns,
        )
    if len(source_indexes) != 1:
        raise ConsumerTraceAmbiguous(
            'frozen source anchor was not unique in final trace',
            source_header_stamp_ns=source_stamp_ns,
            source_receipt_ns=source_receipt_ns,
            matching_source_count=len(source_indexes),
        )
    source_index = source_indexes[0]
    if source_index != len(final_samples) - 1:
        late_receipt_ns, late_message = final_samples[source_index + 1]
        raise ConsumerTraceAmbiguous(
            'late source delivery after frozen anchor',
            source_header_stamp_ns=source_stamp_ns,
            source_receipt_ns=source_receipt_ns,
            late_source_header_stamp_ns=_stamp_ns(late_message),
            late_source_receipt_ns=late_receipt_ns,
        )
    relevant_final_samples, final_phase = _relevant_trace_phase(
        final_samples,
        source_anchor=source_anchor,
        topic=FINAL_COMMAND_TOPIC,
    )
    relevant_limited_samples, limited_phase = _relevant_trace_phase(
        limited_samples,
        source_anchor=source_anchor,
        topic=LIMITED_COMMAND_TOPIC,
    )
    phase_isolation = {
        'source_anchor': {
            'receipt_ns': source_receipt_ns,
            'header_stamp_ns': source_stamp_ns,
            'twist_signature': list(source_signature),
        },
        'final': final_phase,
        'limited': limited_phase,
    }
    for controller_receipt_ns, controller_message in relevant_limited_samples:
        controller_stamp_ns = _stamp_ns(controller_message)
        if controller_stamp_ns < source_stamp_ns:
            raise ConsumerTraceAmbiguous(
                'controller header stamp precedes source anchor phase',
                source_header_stamp_ns=source_stamp_ns,
                source_receipt_ns=source_receipt_ns,
                controller_update_stamp_ns=controller_stamp_ns,
                controller_receipt_ns=controller_receipt_ns,
                phase_isolation=phase_isolation,
            )

    zero_index, (observed_zero_receipt_ns, observed_zero) = (
        _first_relevant_controller_zero(
            relevant_limited_samples,
            source_anchor=source_anchor,
            zero_sim_ns=zero_sim_ns,
            zero_receipt_ns=zero_receipt_ns,
            phase_isolation=phase_isolation,
        )
    )
    observed_zero_sim_ns = _stamp_ns(observed_zero)
    if any(
        not is_zero(message)
        for _, message in relevant_limited_samples[zero_index + 1 :]
    ):
        raise ConsumerTraceAmbiguous(
            'unassociated non-zero controller output after first zero',
            zero_sim_ns=zero_sim_ns,
            zero_receipt_ns=zero_receipt_ns,
        )

    pre_zero_nonzero = [
        sample
        for sample in relevant_limited_samples[:zero_index]
        if (
            not is_zero(sample[1])
            and _stamp_ns(sample[1]) > source_stamp_ns
        )
    ]
    if not pre_zero_nonzero:
        raise ConsumerTraceAmbiguous(
            'missing non-zero controller observation before first zero',
            source_header_stamp_ns=source_stamp_ns,
            source_receipt_ns=source_receipt_ns,
        )
    for controller_receipt_ns, controller_message in pre_zero_nonzero:
        controller_stamp_ns = _stamp_ns(controller_message)
        if controller_receipt_ns <= source_receipt_ns:
            raise ConsumerTraceAmbiguous(
                'late source observer receipt',
                source_header_stamp_ns=source_stamp_ns,
                source_receipt_ns=source_receipt_ns,
                controller_update_stamp_ns=controller_stamp_ns,
                controller_receipt_ns=controller_receipt_ns,
            )
        if source_stamp_ns >= controller_stamp_ns:
            raise ConsumerTraceAmbiguous(
                'source header stamp does not precede controller update',
                source_header_stamp_ns=source_stamp_ns,
                controller_update_stamp_ns=controller_stamp_ns,
            )
        if controller_stamp_ns >= zero_sim_ns:
            raise ConsumerTraceAmbiguous(
                'controller non-zero update does not precede first zero',
                controller_update_stamp_ns=controller_stamp_ns,
                controller_zero_update_stamp_ns=zero_sim_ns,
            )
        if _twist_signature(controller_message) != source_signature:
            raise ConsumerTraceAmbiguous(
                'unassociated non-zero controller output before first zero',
                source_header_stamp_ns=source_stamp_ns,
                controller_update_stamp_ns=controller_stamp_ns,
            )

        matching_sources = [
            (receipt_ns, message)
            for receipt_ns, message in relevant_final_samples
            if (
                receipt_ns <= controller_receipt_ns
                and _stamp_ns(message) <= controller_stamp_ns
                and not is_zero(message)
                and _twist_signature(message) == source_signature
            )
        ]
        if len(matching_sources) != 1:
            raise ConsumerTraceAmbiguous(
                'same-valued duplicate source commands',
                source_header_stamp_ns=source_stamp_ns,
                controller_update_stamp_ns=controller_stamp_ns,
                matching_source_count=len(matching_sources),
            )
        if matching_sources[0][0] != source_receipt_ns:
            raise ConsumerTraceAmbiguous(
                'controller output was not uniquely associated with frozen source',
                source_header_stamp_ns=source_stamp_ns,
                controller_update_stamp_ns=controller_stamp_ns,
            )

    return {
        'association_basis': 'unique ordered source/controller trace',
        'source_header_stamp_ns': source_stamp_ns,
        'source_observer_receipt_ns': source_receipt_ns,
        'controller_update_stamp_ns': controller_stamp_ns,
        'controller_observer_receipt_ns': controller_receipt_ns,
        'controller_zero_update_stamp_ns': zero_sim_ns,
        'controller_zero_observer_receipt_ns': zero_receipt_ns,
        'phase_isolation': phase_isolation,
    }


def consumer_timeout_result(
    final_samples: tuple[tuple[int, Any], ...],
    limited_samples: tuple[tuple[int, Any], ...],
    *,
    source_anchor: tuple[int, Any],
    zero_sim_ns: int,
    zero_receipt_ns: int,
    source_finalization: dict[str, Any] | None = None,
):
    """Apply the controller timeout to the finalized exact-Gate source."""
    finalized_source_evidence = source_finalization
    trace = controller_consumed_command_trace(
        final_samples,
        limited_samples,
        source_anchor=source_anchor,
        zero_sim_ns=zero_sim_ns,
        zero_receipt_ns=zero_receipt_ns,
    )
    last_nonzero_sim_ns = trace['source_header_stamp_ns']
    delta_ns = zero_sim_ns - last_nonzero_sim_ns
    if not (CONSUMER_ZERO_MIN_NS < delta_ns <= CONSUMER_ZERO_MAX_NS):
        raise ConsumerTimeoutOutOfBounds(
            trace,
            delta_ns=delta_ns,
        )
    result = {
        'authoritative_source_stamp_ns': last_nonzero_sim_ns,
        'authoritative_source_receipt_ns': (
            trace['source_observer_receipt_ns']
        ),
        'controller_last_nonzero_update_ns': (
            trace['controller_update_stamp_ns']
        ),
        'controller_zero_update_ns': zero_sim_ns,
        'association': trace,
        'last_nonzero_sim_ns': last_nonzero_sim_ns,
        'last_nonzero_receipt_ns': trace['source_observer_receipt_ns'],
        'zero_sim_ns': zero_sim_ns,
        'delta_ns': delta_ns,
        'zero_receipt_ns': zero_receipt_ns,
    }
    if finalized_source_evidence is not None:
        result['source_finalization'] = finalized_source_evidence
    return result


def gate_zero_proven(message: Any) -> bool:
    return (
        message.state == type(message).INHIBITED
        and message.motion_inhibited
        and message.zero_selected
        and message.zero_publish_seq > 0
        and message.zero_publish_seq >= message.output_publish_seq
    )


class CrashStopProbe:
    """One ROS observation Interface for both public crash-stop scenarios."""

    def __init__(self) -> None:
        interfaces = _get_workspace_interface_types()
        self._execute_mission_type = interfaces.execute_mission
        self._mission_state_type = interfaces.mission_state
        self._mission_step_type = interfaces.mission_step
        self._gate_state_type = interfaces.gate_state
        self._gate_control_type = interfaces.gate_control
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
        self._final_callback_condition = threading.Condition()
        self._final_callbacks_in_flight = 0
        self._final_trace_frozen = False
        self._final_post_freeze_ingress_count = 0
        self._final_stale_epoch_ingress_count = 0
        self._final_observation_epoch = 0
        self._final_trace_epoch_state = 'observing'
        self._final_subscription_identity = (
            'voice-nav-final-subscription-' + secrets.token_hex(16)
        )
        self._finalized_final_trace: tuple[CommandObservation, ...] | None = (
            None
        )
        self._finalized_trace_fence: dict[str, Any] | None = None
        self.mission_states = deque(maxlen=4000)
        self.gate_states = deque(maxlen=4000)
        self.final_commands = deque(maxlen=8000)
        self.limited_commands = deque(maxlen=8000)
        self.odometry = deque(maxlen=4000)
        self.scan_samples = deque(maxlen=4000)
        self.joint_states = deque(maxlen=4000)
        self.clock_samples = deque(maxlen=4000)
        # Keep the 1 kHz /clock and sensor diagnostics from serializing the
        # safety-observation callbacks behind the Node's default mutually
        # exclusive group.  Cross-topic receipt latency is part of the
        # crash-stop proof, so Gate/final/limited commands share their own
        # ordered lane while MissionState and high-rate sensors use separate
        # lanes on the existing four-thread executor.
        self.state_observation_group = MutuallyExclusiveCallbackGroup()
        self.safety_observation_group = MutuallyExclusiveCallbackGroup()
        self.sensor_observation_group = MutuallyExclusiveCallbackGroup()
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscriptions = [
            self.node.create_subscription(
                interfaces.mission_state,
                MISSION_STATE_TOPIC,
                lambda message: self._append(
                    self.mission_states, message
                ),
                state_qos,
                callback_group=self.state_observation_group,
            ),
            self.node.create_subscription(
                interfaces.gate_state,
                GATE_STATE_TOPIC,
                lambda message: self._append(self.gate_states, message),
                state_qos,
                callback_group=self.safety_observation_group,
            ),
            self._create_final_subscription(
                observation_epoch=self._final_observation_epoch,
                final_subscription_identity=(
                    self._final_subscription_identity
                ),
            ),
            self.node.create_subscription(
                TwistStamped,
                LIMITED_COMMAND_TOPIC,
                self._append_limited,
                100,
                callback_group=self.safety_observation_group,
            ),
            self.node.create_subscription(
                Odometry,
                ODOMETRY_TOPIC,
                lambda message: self._append(self.odometry, message),
                100,
                callback_group=self.sensor_observation_group,
            ),
            self.node.create_subscription(
                LaserScan,
                SCAN_TOPIC,
                lambda message: self._append(self.scan_samples, message),
                qos_profile_sensor_data,
                callback_group=self.sensor_observation_group,
            ),
            self.node.create_subscription(
                JointState,
                JOINT_STATE_TOPIC,
                lambda message: self._append(
                    self.joint_states, message
                ),
                100,
                callback_group=self.sensor_observation_group,
            ),
            self.node.create_subscription(
                Clock,
                CLOCK_TOPIC,
                lambda message: self._append(
                    self.clock_samples, _clock_ns(message)
                ),
                qos_profile_sensor_data,
                callback_group=self.sensor_observation_group,
            ),
        ]
        self.final_subscription = self.subscriptions[2]
        self.action_client = ActionClient(
            self.node, self._execute_mission_type, ACTION_NAME
        )
        self.gate_client = self.node.create_client(
            self._gate_control_type,
            GATE_CONTROL_SERVICE,
        )
        self.candidate_publishers = []
        self.spin_thread.start()

    def _append(self, collection: deque, message: Any) -> None:
        with self.lock:
            collection.append((time.monotonic_ns(), message))

    def _append_limited(self, message: Any, message_info: Any) -> None:
        if isinstance(message_info, dict):
            sequence = message_info.get('publication_sequence_number')
        else:
            sequence = getattr(message_info, 'publication_sequence_number', None)
        if not isinstance(sequence, int) or isinstance(sequence, bool):
            sequence = None
        with self.lock:
            self.limited_commands.append(
                CommandObservation(
                    receipt_ns=time.monotonic_ns(),
                    message=message,
                    publication_sequence_number=sequence,
                )
            )

    def _ensure_final_callback_state(self) -> None:
        """Initialize the package-private final callback barrier for unit probes."""
        if not hasattr(self, '_final_callback_condition'):
            self._final_callback_condition = threading.Condition()
        if not hasattr(self, '_final_callbacks_in_flight'):
            self._final_callbacks_in_flight = 0
        if not hasattr(self, '_final_trace_frozen'):
            self._final_trace_frozen = False
        if not hasattr(self, '_final_post_freeze_ingress_count'):
            self._final_post_freeze_ingress_count = 0
        if not hasattr(self, '_final_stale_epoch_ingress_count'):
            self._final_stale_epoch_ingress_count = 0
        if not hasattr(self, '_final_observation_epoch'):
            self._final_observation_epoch = 0
        if not hasattr(self, '_final_trace_epoch_state'):
            self._final_trace_epoch_state = 'observing'
        if not hasattr(self, '_final_subscription_identity'):
            self._final_subscription_identity = (
                'voice-nav-final-subscription-' + secrets.token_hex(16)
            )
        if not hasattr(self, '_finalized_final_trace'):
            self._finalized_final_trace = None
        if not hasattr(self, '_finalized_trace_fence'):
            self._finalized_trace_fence = None

    @staticmethod
    def _jazzy_message_info_int(message_info: Any, field: str) -> int | None:
        if isinstance(message_info, dict):
            value = message_info.get(field)
        else:
            value = getattr(message_info, field, None)
        if not isinstance(value, int) or isinstance(value, bool):
            return None
        return value

    def _create_final_subscription(
        self,
        *,
        observation_epoch: int,
        final_subscription_identity: str,
    ):
        """Create one epoch-bound final-command observer subscription."""
        return self.node.create_subscription(
            TwistStamped,
            FINAL_COMMAND_TOPIC,
            lambda message, message_info: self._append_final(
                message,
                message_info,
                observation_epoch=observation_epoch,
                final_subscription_identity=final_subscription_identity,
            ),
            100,
            callback_group=self.safety_observation_group,
        )

    def _append_final(
        self,
        message: Any,
        message_info: Any,
        *,
        observation_epoch: int | None = None,
        final_subscription_identity: str | None = None,
    ) -> None:
        """Record Jazzy metadata and mark callback ingress before waiting on lock."""
        self._ensure_final_callback_state()
        with self._final_callback_condition:
            if observation_epoch is None:
                observation_epoch = self._final_observation_epoch
            if final_subscription_identity is None:
                final_subscription_identity = self._final_subscription_identity
            if (
                observation_epoch != self._final_observation_epoch
                or final_subscription_identity
                != self._final_subscription_identity
            ):
                self._final_stale_epoch_ingress_count += 1
                self._final_callback_condition.notify_all()
                return
            if self._final_trace_frozen:
                self._final_post_freeze_ingress_count += 1
                self._final_callback_condition.notify_all()
                return
            self._final_callbacks_in_flight += 1
        try:
            with self.lock:
                self.final_commands.append(
                    CommandObservation(
                        receipt_ns=time.monotonic_ns(),
                        message=message,
                        publication_sequence_number=(
                            self._jazzy_message_info_int(
                                message_info,
                                'publication_sequence_number',
                            )
                        ),
                        source_timestamp_ns=self._jazzy_message_info_int(
                            message_info,
                            'source_timestamp',
                        ),
                        received_timestamp_ns=self._jazzy_message_info_int(
                            message_info,
                            'received_timestamp',
                        ),
                        reception_sequence_number=(
                            self._jazzy_message_info_int(
                                message_info,
                                'reception_sequence_number',
                            )
                        ),
                        final_subscription_identity=(
                            final_subscription_identity
                        ),
                        observation_epoch=observation_epoch,
                        header_stamp_ns=_stamp_ns(message),
                        twist_signature=_twist_signature(message),
                    )
                )
        finally:
            with self._final_callback_condition:
                self._final_callbacks_in_flight -= 1
                self._final_callback_condition.notify_all()

    def begin_replacement_final_observation_epoch(self) -> dict[str, Any]:
        """Fence an immutable final trace before observing a replacement Gate."""
        self._ensure_final_callback_state()
        with self._final_callback_condition:
            if (
                not self._final_trace_frozen
                or self._final_trace_epoch_state != 'finalized'
            ):
                raise ConsumerTraceAmbiguous(
                    'replacement final observation requires a frozen trace'
                )
            if (
                self._final_callbacks_in_flight
                or self._final_post_freeze_ingress_count
            ):
                self._final_trace_epoch_state = 'failed'
                raise ConsumerTraceAmbiguous(
                    'old callback entered after finalized trace freeze',
                    in_flight_callbacks=self._final_callbacks_in_flight,
                    post_freeze_ingress_count=(
                        self._final_post_freeze_ingress_count
                    ),
                )
            if self._finalized_final_trace is None:
                raise ConsumerTraceAmbiguous(
                    'replacement final observation requires finalized trace'
                )
            if self._finalized_trace_fence is None:
                raise ConsumerTraceAmbiguous(
                    'replacement final observation requires finalized trace fence'
                )
            previous_epoch = self._final_observation_epoch
            previous_identity = self._final_subscription_identity
            previous_trace = self._finalized_final_trace
            previous_post_freeze_ingress_count = (
                self._final_post_freeze_ingress_count
            )
            replacement_epoch = previous_epoch + 1
            replacement_identity = (
                'voice-nav-final-subscription-' + secrets.token_hex(16)
            )
            self._final_trace_epoch_state = 'transitioning'
            self._final_callback_condition.notify_all()

        old_subscription = getattr(self, 'final_subscription', None)
        subscriptions = getattr(self, 'subscriptions', None)
        subscription_index = None
        if old_subscription is not None and subscriptions is not None:
            try:
                subscription_index = subscriptions.index(old_subscription)
            except ValueError as error:
                self._fail_replacement_final_observation_transition(
                    'replacement final subscription was not registered',
                    error=error,
                )

        replacement_subscription = None
        try:
            if old_subscription is not None:
                if self.node.destroy_subscription(old_subscription) is not True:
                    raise RuntimeError(
                        'replacement final subscription was not destroyed'
                    )
                replacement_subscription = self._create_final_subscription(
                    observation_epoch=replacement_epoch,
                    final_subscription_identity=replacement_identity,
                )
                if replacement_subscription is None:
                    raise RuntimeError(
                        'replacement final subscription was not created'
                    )
        except Exception as error:
            self._fail_replacement_final_observation_transition(
                'replacement final observer transition failed',
                error=error,
            )

        with self._final_callback_condition:
            if (
                self._final_trace_epoch_state != 'transitioning'
                or not self._final_trace_frozen
                or self._final_callbacks_in_flight
                or self._final_post_freeze_ingress_count
            ):
                self._final_trace_epoch_state = 'failed'
                self._final_callback_condition.notify_all()
                failure = ConsumerTraceAmbiguous(
                    'old callback entered during replacement final transition',
                    in_flight_callbacks=self._final_callbacks_in_flight,
                    post_freeze_ingress_count=(
                        self._final_post_freeze_ingress_count
                    ),
                )
            else:
                if subscription_index is not None:
                    subscriptions[subscription_index] = replacement_subscription
                if old_subscription is not None:
                    self.final_subscription = replacement_subscription
                self._final_observation_epoch = replacement_epoch
                self._final_subscription_identity = replacement_identity
                self._final_trace_frozen = False
                self._final_post_freeze_ingress_count = 0
                self._final_trace_epoch_state = 'replacement'
                replacement = {
                    'finalized_trace_fence': deepcopy(
                        self._finalized_trace_fence
                    ),
                    'previous_observation_epoch': previous_epoch,
                    'previous_final_subscription_identity': previous_identity,
                    'previous_final_trace_length': len(previous_trace),
                    'previous_post_freeze_ingress_count': (
                        previous_post_freeze_ingress_count
                    ),
                    'observation_epoch': self._final_observation_epoch,
                    'final_subscription_identity': (
                        self._final_subscription_identity
                    ),
                }
                self._final_callback_condition.notify_all()
                return replacement

        if replacement_subscription is not None:
            try:
                self.node.destroy_subscription(replacement_subscription)
            except Exception:
                pass
        raise failure

    def _fail_replacement_final_observation_transition(
        self,
        reason: str,
        *,
        error: Exception,
    ) -> None:
        """Keep an incomplete subscription swap frozen and fail closed."""
        with self._final_callback_condition:
            self._final_trace_frozen = True
            self._final_trace_epoch_state = 'failed'
            self._final_callback_condition.notify_all()
        raise ConsumerTraceAmbiguous(
            reason,
            error_type=type(error).__name__,
            error_detail=str(error),
        ) from error

    def _freeze_final_callback_ingress(
        self,
        *,
        deadline: float,
    ) -> dict[str, int]:
        """Block a finalization until every callback already in ingress drains."""
        self._ensure_final_callback_state()
        with self._final_callback_condition:
            self._final_trace_frozen = True
            while self._final_callbacks_in_flight:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._final_trace_frozen = False
                    self._final_callback_condition.notify_all()
                    raise ConsumerTraceAmbiguous(
                        'final callback ingress did not drain before freeze',
                        in_flight_callbacks=self._final_callbacks_in_flight,
                        post_freeze_ingress_count=(
                            self._final_post_freeze_ingress_count
                        ),
                    )
                self._final_callback_condition.wait(timeout=min(remaining, 0.05))
            if self._final_post_freeze_ingress_count:
                self._final_trace_frozen = False
                self._final_callback_condition.notify_all()
                raise ConsumerTraceAmbiguous(
                    'final callback entered after trace freeze',
                    post_freeze_ingress_count=(
                        self._final_post_freeze_ingress_count
                    ),
                )
            self._final_trace_epoch_state = 'frozen'
            return {
                'in_flight_callbacks': self._final_callbacks_in_flight,
                'post_freeze_ingress_count': (
                    self._final_post_freeze_ingress_count
                ),
            }

    def _unfreeze_final_callback_ingress(self) -> None:
        """Permit diagnostic collection again after a failed finalization."""
        self._ensure_final_callback_state()
        with self._final_callback_condition:
            self._final_trace_frozen = False
            self._final_post_freeze_ingress_count = 0
            self._final_trace_epoch_state = 'observing'
            self._final_callback_condition.notify_all()

    def _assert_final_callback_freeze_clean(self) -> None:
        """Reject a callback that entered after the finalized trace freeze."""
        self._ensure_final_callback_state()
        with self._final_callback_condition:
            if self._final_post_freeze_ingress_count:
                raise ConsumerTraceAmbiguous(
                    'final callback entered after trace freeze',
                    post_freeze_ingress_count=(
                        self._final_post_freeze_ingress_count
                    ),
                )

    def _commit_finalized_trace_checkpoint(
        self,
        rechecked_snapshot: tuple[CommandObservation, ...],
    ) -> dict[str, Any]:
        """Atomically freeze the immutable old epoch and its switch fence."""
        self._ensure_final_callback_state()
        with self._final_callback_condition:
            if (
                not self._final_trace_frozen
                or self._final_trace_epoch_state != 'frozen'
            ):
                raise ConsumerTraceAmbiguous(
                    'finalized trace checkpoint was not frozen'
                )
            if (
                self._final_callbacks_in_flight
                or self._final_post_freeze_ingress_count
            ):
                raise ConsumerTraceAmbiguous(
                    'final callback entered after trace freeze',
                    in_flight_callbacks=self._final_callbacks_in_flight,
                    post_freeze_ingress_count=(
                        self._final_post_freeze_ingress_count
                    ),
                )
            self._finalized_final_trace = tuple(deepcopy(rechecked_snapshot))
            self._finalized_trace_fence = {
                'observation_epoch': self._final_observation_epoch,
                'final_subscription_identity': (
                    self._final_subscription_identity
                ),
                'trace_length': len(self._finalized_final_trace),
            }
            self._final_trace_epoch_state = 'finalized'
            return deepcopy(self._finalized_trace_fence)

    def _final_endpoint_snapshot(self) -> tuple[str, ...]:
        provider = getattr(self, '_final_endpoint_snapshot_provider', None)
        if provider is not None:
            return tuple(provider())
        return tuple(
            sorted(
                bytes(endpoint.endpoint_gid).hex()
                for endpoint in self.node.get_publishers_info_by_topic(
                    FINAL_COMMAND_TOPIC
                )
            )
        )

    def _limited_endpoint_snapshot(self) -> tuple[str, ...]:
        provider = getattr(self, '_limited_endpoint_snapshot_provider', None)
        if provider is not None:
            return tuple(provider())
        return tuple(
            sorted(
                bytes(endpoint.endpoint_gid).hex()
                for endpoint in self.node.get_publishers_info_by_topic(
                    LIMITED_COMMAND_TOPIC
                )
            )
        )

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
                    message.availability == self._mission_state_type.AVAILABLE
                    and message.gate_state == self._mission_state_type.GATE_INHIBITED
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
                    message.availability == self._mission_state_type.FAULTED
                    and message.gate_state in (
                        self._mission_state_type.GATE_INHIBITED,
                        self._mission_state_type.GATE_ARMED,
                        self._mission_state_type.GATE_FAULTED,
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
        state: Any,
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
        goal = self._execute_mission_type.Goal()
        goal.source_instance_id = source_instance_id
        goal.source_seq = source_seq
        goal.runtime_instance_id = state.runtime_instance_id
        goal.admission_epoch = state.admission_epoch
        step = self._mission_step_type()
        step.kind = self._mission_step_type.MOVE_DISTANCE
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
                and state_sample[1].state == self._gate_state_type.ARMED
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
                state.state == self._gate_state_type.ARMED
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
            final_endpoint_gid = self.assert_unique_final_owner()
            final_metadata = _final_observation_metadata(
                final_sample,
                checkpoint='signal-boundary',
            )
            endpoint_gid = self._require_limited_endpoint_continuity(
                expected_gid=None,
                checkpoint='signal-boundary',
            )
            return {
                'gate': deepcopy(state),
                'final': deepcopy(final_sample),
                'limited': (
                    limited_sample[0],
                    deepcopy(limited_sample[1]),
                ),
                'clock': clock_sample,
                'observed_ns': observed_ns,
                'limited_endpoint_fence': {
                    'endpoint_gid': endpoint_gid,
                    'limited_receipt_fence_ns': limited_sample[0],
                },
                'final_endpoint_fence': {
                    'endpoint_gid': final_endpoint_gid,
                    'final_subscription_identity': final_metadata[
                        'final_subscription_identity'
                    ],
                    'final_receipt_fence_ns': final_sample[0],
                    'final_header_stamp_ns': _stamp_ns(final_sample[1]),
                    'final_publication_sequence_number': final_metadata[
                        'publication_sequence_number'
                    ],
                    'final_source_timestamp_ns': final_metadata[
                        'source_timestamp_ns'
                    ],
                    'final_received_timestamp_ns': final_metadata[
                        'received_timestamp_ns'
                    ],
                    'final_reception_sequence_number': final_metadata[
                        'reception_sequence_number'
                    ],
                },
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
        self,
        kill_ack_ns: int,
        signal_boundary_sim_ns: int,
        limited_endpoint_fence: dict[str, Any],
    ):
        deadline = time.monotonic() + WALL_WATCHDOG_SECONDS
        endpoint_gid, limited_receipt_fence_ns = (
            self._limited_endpoint_fence(limited_endpoint_fence)
        )
        previous_clock = self.latest(self.clock_samples)
        while time.monotonic() < deadline:
            with self.lock:
                current_clock = (
                    self.clock_samples[-1] if self.clock_samples else None
                )
                limited_snapshot = tuple(self.limited_commands)
                self._require_limited_endpoint_continuity(
                    expected_gid=endpoint_gid,
                    checkpoint='first-zero',
                )
                candidate = next(
                    (
                        (receipt_ns, deepcopy(message))
                        for receipt_ns, message in limited_snapshot
                        if (
                            receipt_ns >= kill_ack_ns
                            and _stamp_ns(message) > signal_boundary_sim_ns
                            and is_zero(message)
                        )
                    ),
                    None,
                )
                if (
                    candidate is not None
                    and candidate[0] <= limited_receipt_fence_ns
                ):
                    raise ConsumerTraceAmbiguous(
                        'controller first zero predates signal-boundary '
                        'receipt fence',
                        endpoint_gid=endpoint_gid,
                        limited_receipt_fence_ns=limited_receipt_fence_ns,
                        zero_receipt_ns=candidate[0],
                        zero_sim_ns=_stamp_ns(candidate[1]),
                    )
            if (
                previous_clock is not None
                and current_clock is not None
                and current_clock[1] <= previous_clock[1]
                and time.monotonic_ns() - kill_ack_ns > 1_000_000_000
            ):
                raise ConsumerTraceAmbiguous(
                    'simulation clock stopped during consumer-timeout measurement',
                    previous_clock_ns=previous_clock[1],
                    observed_clock_ns=current_clock[1],
                    kill_ack_ns=kill_ack_ns,
                )
            if current_clock is not None:
                previous_clock = current_clock
            if candidate is not None:
                return {
                    'zero_sim_ns': _stamp_ns(candidate[1]),
                    'zero_receipt_ns': candidate[0],
                    'endpoint_gid': endpoint_gid,
                    'limited_receipt_fence_ns': limited_receipt_fence_ns,
                }
            time.sleep(0.01)
        raise AssertionError(
            'controller cmd_vel_out did not select zero before the watchdog'
        )

    @staticmethod
    def _final_endpoint_fence(
        final_endpoint_fence: dict[str, Any],
    ) -> tuple[str, int]:
        endpoint_gid = final_endpoint_fence.get('endpoint_gid')
        if (
            not isinstance(endpoint_gid, str)
            or not endpoint_gid
            or all(character == '0' for character in endpoint_gid)
        ):
            raise ConsumerTraceAmbiguous(
                'final endpoint identity unavailable',
                checkpoint='signal-boundary',
                endpoint_gid=(endpoint_gid if isinstance(endpoint_gid, str) else ''),
            )
        receipt_fence_ns = final_endpoint_fence.get('final_receipt_fence_ns')
        if (
            not isinstance(receipt_fence_ns, int)
            or isinstance(receipt_fence_ns, bool)
        ):
            raise ConsumerTraceAmbiguous(
                'final endpoint receipt fence unavailable',
                checkpoint='signal-boundary',
            )
        return endpoint_gid, receipt_fence_ns

    def _require_final_endpoint_disappearance(
        self,
        final_endpoint_fence: dict[str, Any],
        *,
        checkpoint: str,
    ) -> dict[str, Any] | None:
        endpoint_gid, receipt_fence_ns = self._final_endpoint_fence(
            final_endpoint_fence
        )
        endpoint_gids = self._final_endpoint_snapshot()
        if not endpoint_gids:
            return {
                'endpoint_gid': endpoint_gid,
                'final_receipt_fence_ns': receipt_fence_ns,
                'checkpoint': checkpoint,
                'remaining_endpoint_gids': [],
            }
        if endpoint_gid not in endpoint_gids:
            raise ConsumerTraceAmbiguous(
                'foreign final endpoint appeared after exact Gate SIGKILL',
                checkpoint=checkpoint,
                expected_endpoint_gid=endpoint_gid,
                observed_endpoint_gids=list(endpoint_gids),
            )
        if len(endpoint_gids) != 1:
            raise ConsumerTraceAmbiguous(
                'duplicate final endpoints remained after exact Gate SIGKILL',
                checkpoint=checkpoint,
                expected_endpoint_gid=endpoint_gid,
                observed_endpoint_gids=list(endpoint_gids),
            )
        return None

    def wait_final_endpoint_disappearance(
        self,
        final_endpoint_fence: dict[str, Any],
        *,
        timeout: float = WALL_WATCHDOG_SECONDS,
    ) -> dict[str, Any]:
        """Wait for the signal-boundary Gate endpoint to leave the graph."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            disappeared = self._require_final_endpoint_disappearance(
                final_endpoint_fence,
                checkpoint='post-pidfd',
            )
            if disappeared is not None:
                disappeared['observed_ns'] = time.monotonic_ns()
                return disappeared
            time.sleep(0.01)
        endpoint_gid, receipt_fence_ns = self._final_endpoint_fence(
            final_endpoint_fence
        )
        raise ConsumerTraceAmbiguous(
            'exact Gate final endpoint did not disappear after pidfd acknowledgement',
            endpoint_gid=endpoint_gid,
            final_receipt_fence_ns=receipt_fence_ns,
        )

    def wait_confirm_consumer_timeout(
        self,
        motion_proof_anchor: tuple[int, Any],
        consumer_zero: dict[str, int],
        *,
        final_endpoint_fence: dict[str, Any] | None = None,
        timeout: float = WALL_WATCHDOG_SECONDS,
    ):
        deadline = time.monotonic() + timeout
        endpoint_gid, limited_receipt_fence_ns = (
            self._consumer_zero_endpoint_fence(consumer_zero)
        )
        final_stable_key = None
        final_stable_since_ns = None
        final_stable_clock_ns = None
        while time.monotonic() < deadline:
            now_ns = time.monotonic_ns()
            with self.lock:
                final_snapshot = tuple(self.final_commands)
                limited_snapshot = tuple(self.limited_commands)
                clock_sample = (
                    self.clock_samples[-1]
                    if self.clock_samples
                    else None
                )
                self._require_limited_endpoint_continuity(
                    expected_gid=endpoint_gid,
                    checkpoint='observation',
                )
            if (
                not final_snapshot
                or not limited_snapshot
                or clock_sample is None
            ):
                final_stable_key = None
                final_stable_since_ns = None
                final_stable_clock_ns = None
                time.sleep(0.01)
                continue

            last_receipt_ns, last_message = final_snapshot[-1]
            current_final_key = (
                len(final_snapshot),
                last_receipt_ns,
                _stamp_ns(last_message),
            )
            if current_final_key != final_stable_key:
                final_stable_key = current_final_key
                final_stable_since_ns = now_ns
                final_stable_clock_ns = clock_sample[1]
            elif (
                final_stable_since_ns is not None
                and final_stable_clock_ns is not None
                and now_ns - final_stable_since_ns >= FINAL_STREAM_QUIESCENCE_NS
                and clock_sample[1] - final_stable_clock_ns
                >= FINAL_STREAM_QUIESCENCE_NS
            ):
                with self.lock:
                    confirmed_snapshot = tuple(self.final_commands)
                    confirmed_limited = tuple(self.limited_commands)
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
                    confirmed_final_key = (
                        len(confirmed_snapshot),
                        confirmed_last[0],
                        _stamp_ns(confirmed_last[1]),
                    ) if confirmed_last is not None else None
                    confirmed_endpoint_gid = (
                        self._require_limited_endpoint_continuity(
                            expected_gid=endpoint_gid,
                            checkpoint='final',
                        )
                    )
                    if confirmed_final_key != final_stable_key:
                        raise ConsumerTraceAmbiguous(
                            'final source appeared after quiescence freeze',
                            stable_final_key=list(final_stable_key),
                            observed_final_key=(
                                list(confirmed_final_key)
                                if confirmed_final_key is not None
                                else None
                            ),
                        )
                if (
                    confirmed_clock is None
                    or confirmed_clock[1] - final_stable_clock_ns
                    < FINAL_STREAM_QUIESCENCE_NS
                ):
                    time.sleep(0.01)
                    continue

                final_callback_ingress = None
                try:
                    if final_endpoint_fence is not None:
                        # This is separate from self.lock: a callback can have
                        # entered rclpy ingress already and still be waiting for
                        # self.lock.  Drain it before the final graph/trace
                        # checkpoint so a queued source cannot appear after a
                        # successful return.
                        final_callback_ingress = (
                            self._freeze_final_callback_ingress(deadline=deadline)
                        )
                    with self.lock:
                        rechecked_snapshot = tuple(self.final_commands)
                        rechecked_limited = tuple(self.limited_commands)
                        rechecked_clock = (
                            self.clock_samples[-1]
                            if self.clock_samples
                            else None
                        )
                        rechecked_last = (
                            rechecked_snapshot[-1]
                            if rechecked_snapshot
                            else None
                        )
                        rechecked_final_key = (
                            len(rechecked_snapshot),
                            rechecked_last[0],
                            _stamp_ns(rechecked_last[1]),
                        ) if rechecked_last is not None else None
                        rechecked_endpoint_gid = (
                            self._require_limited_endpoint_continuity(
                                expected_gid=endpoint_gid,
                                checkpoint='finalized-trace',
                            )
                        )
                    if rechecked_final_key != confirmed_final_key:
                        raise ConsumerTraceAmbiguous(
                            'final source appeared after quiescence freeze',
                            stable_final_key=list(final_stable_key),
                            confirmed_final_key=(
                                list(confirmed_final_key)
                                if confirmed_final_key is not None
                                else None
                            ),
                            rechecked_final_key=(
                                list(rechecked_final_key)
                                if rechecked_final_key is not None
                                else None
                            ),
                        )
                    if (
                        rechecked_clock is None
                        or rechecked_clock[1] - final_stable_clock_ns
                        < FINAL_STREAM_QUIESCENCE_NS
                    ):
                        if final_endpoint_fence is not None:
                            self._unfreeze_final_callback_ingress()
                        continue

                    consumer_source_anchor = motion_proof_anchor
                    source_finalization = None
                    final_endpoint_disappearance = None
                    if final_endpoint_fence is not None:
                        final_endpoint_disappearance = (
                            self._require_final_endpoint_disappearance(
                                final_endpoint_fence,
                                checkpoint='finalized-trace',
                            )
                        )
                        if final_endpoint_disappearance is None:
                            self._unfreeze_final_callback_ingress()
                            continue
                        consumer_source_anchor, source_finalization = (
                            finalize_consumer_source_anchor(
                                rechecked_snapshot,
                                motion_proof_anchor=motion_proof_anchor,
                                final_endpoint_fence=final_endpoint_fence,
                            )
                        )
                        self._assert_final_callback_freeze_clean()
                    controller_consumed_command_trace(
                        rechecked_snapshot,
                        rechecked_limited,
                        source_anchor=consumer_source_anchor,
                        zero_sim_ns=consumer_zero['zero_sim_ns'],
                        zero_receipt_ns=consumer_zero['zero_receipt_ns'],
                    )
                    watermark = self._limited_zero_watermark(
                        rechecked_limited,
                        consumer_zero,
                        source_anchor=consumer_source_anchor,
                    )
                    if watermark is None:
                        if final_endpoint_fence is not None:
                            self._unfreeze_final_callback_ingress()
                        time.sleep(0.01)
                        continue
                    result = consumer_timeout_result(
                        rechecked_snapshot,
                        rechecked_limited,
                        source_anchor=consumer_source_anchor,
                        zero_sim_ns=consumer_zero['zero_sim_ns'],
                        zero_receipt_ns=consumer_zero['zero_receipt_ns'],
                        source_finalization=source_finalization,
                    )
                    result['final_quiescence'] = {
                        'stable_key': list(final_stable_key),
                        'wall_elapsed_ns': now_ns - final_stable_since_ns,
                        'sim_elapsed_ns': (
                            rechecked_clock[1] - final_stable_clock_ns
                        ),
                    }
                    result['limited_zero_watermark'] = watermark
                    result['endpoint_continuity'] = {
                        'endpoint_gid': rechecked_endpoint_gid,
                        'signal_boundary_checkpoint': 'singleton',
                        'first_zero_checkpoint': 'singleton',
                        'observation_checkpoint': 'singleton',
                        'final_checkpoint': 'singleton',
                        'limited_receipt_fence_ns': limited_receipt_fence_ns,
                    }
                    if final_callback_ingress is not None:
                        result['final_callback_ingress'] = (
                            final_callback_ingress
                        )
                    if final_endpoint_disappearance is not None:
                        result['final_endpoint_disappearance'] = (
                            final_endpoint_disappearance
                        )
                    if final_endpoint_fence is not None:
                        self._assert_final_callback_freeze_clean()
                        result['finalized_trace_checkpoint'] = (
                            self._commit_finalized_trace_checkpoint(
                                rechecked_snapshot
                            )
                        )
                    return result
                except Exception:
                    if final_endpoint_fence is not None:
                        self._unfreeze_final_callback_ingress()
                    raise
            time.sleep(0.01)
        with self.lock:
            deadline_now_ns = time.monotonic_ns()
            final_snapshot = tuple(self.final_commands)
            limited_snapshot = tuple(self.limited_commands)
            clock_sample = (
                self.clock_samples[-1] if self.clock_samples else None
            )
            self._require_limited_endpoint_continuity(
                expected_gid=endpoint_gid,
                checkpoint='deadline',
            )
            if (
                final_snapshot
                and limited_snapshot
                and clock_sample is not None
            ):
                deadline_last = final_snapshot[-1]
                deadline_final_key = (
                    len(final_snapshot),
                    deadline_last[0],
                    _stamp_ns(deadline_last[1]),
                )
                deadline_watermark = self._limited_zero_watermark(
                    limited_snapshot,
                    consumer_zero,
                    source_anchor=motion_proof_anchor,
                )
                if (
                    deadline_final_key == final_stable_key
                    and final_stable_since_ns is not None
                    and final_stable_clock_ns is not None
                    and deadline_now_ns - final_stable_since_ns
                    >= FINAL_STREAM_QUIESCENCE_NS
                    and clock_sample[1] - final_stable_clock_ns
                    >= FINAL_STREAM_QUIESCENCE_NS
                    and deadline_watermark is None
                ):
                    raise ConsumerTraceWatermarkTimeout(
                        endpoint_gid=endpoint_gid,
                        final_quiescence={
                            'stable_key': list(final_stable_key),
                            'wall_elapsed_ns': (
                                deadline_now_ns - final_stable_since_ns
                            ),
                            'sim_elapsed_ns': (
                                clock_sample[1] - final_stable_clock_ns
                            ),
                        },
                        **self._limited_zero_watermark_evidence(
                            limited_snapshot,
                            consumer_zero,
                            source_anchor=motion_proof_anchor,
                        ),
                    )
        raise ConsumerTraceAmbiguous(
            'final command observer did not quiesce with advancing clock',
            final_sample_count=len(final_snapshot),
            final_last_receipt_ns=(
                final_snapshot[-1][0] if final_snapshot else None
            ),
            final_last_stamp_ns=(
                _stamp_ns(final_snapshot[-1][1]) if final_snapshot else None
            ),
            clock_ns=(clock_sample[1] if clock_sample is not None else None),
        )

    @staticmethod
    def _consumer_zero_endpoint_fence(
        consumer_zero: dict[str, Any],
    ) -> tuple[str, int]:
        return CrashStopProbe._limited_endpoint_fence(consumer_zero)

    @staticmethod
    def _limited_endpoint_fence(
        limited_endpoint_fence: dict[str, Any],
    ) -> tuple[str, int]:
        endpoint_gid = limited_endpoint_fence.get('endpoint_gid')
        if (
            not isinstance(endpoint_gid, str)
            or not endpoint_gid
            or all(character == '0' for character in endpoint_gid)
        ):
            raise ConsumerTraceAmbiguous(
                'limited endpoint identity unavailable',
                checkpoint='consumer-zero',
                endpoint_gid=(
                    endpoint_gid if isinstance(endpoint_gid, str) else ''
                ),
            )
        limited_receipt_fence_ns = limited_endpoint_fence.get(
            'limited_receipt_fence_ns'
        )
        if (
            not isinstance(limited_receipt_fence_ns, int)
            or isinstance(limited_receipt_fence_ns, bool)
        ):
            raise ConsumerTraceAmbiguous(
                'limited endpoint fence boundary unavailable',
                checkpoint='signal-boundary',
            )
        return endpoint_gid, limited_receipt_fence_ns

    def _require_limited_endpoint_continuity(
        self,
        *,
        expected_gid: str | None,
        checkpoint: str,
    ) -> str:
        endpoint_gids = self._limited_endpoint_snapshot()
        if len(endpoint_gids) != 1:
            raise ConsumerTraceAmbiguous(
                'limited endpoint set was not singleton',
                checkpoint=checkpoint,
                endpoint_count=len(endpoint_gids),
            )
        endpoint_gid = endpoint_gids[0]
        if (
            not isinstance(endpoint_gid, str)
            or not endpoint_gid
            or all(character == '0' for character in endpoint_gid)
        ):
            raise ConsumerTraceAmbiguous(
                'limited endpoint identity unavailable',
                checkpoint=checkpoint,
                endpoint_gid=(
                    endpoint_gid if isinstance(endpoint_gid, str) else ''
                ),
            )
        if expected_gid is not None and endpoint_gid != expected_gid:
            raise ConsumerTraceAmbiguous(
                'limited endpoint changed during continuity fence',
                checkpoint=checkpoint,
                expected_endpoint_gid=expected_gid,
                observed_endpoint_gid=endpoint_gid,
            )
        return endpoint_gid

    def _limited_zero_watermark(
        self,
        limited_snapshot: tuple[Any, ...],
        consumer_zero: dict[str, int],
        *,
        source_anchor: tuple[int, Any],
    ) -> dict[str, Any] | None:
        relevant_limited, limited_phase = _relevant_trace_phase(
            limited_snapshot,
            source_anchor=source_anchor,
            topic=LIMITED_COMMAND_TOPIC,
        )
        self._assert_limited_publication_sequence_continuity(
            relevant_limited
        )
        first_zero_index, _ = _first_relevant_controller_zero(
            relevant_limited,
            source_anchor=source_anchor,
            zero_sim_ns=consumer_zero['zero_sim_ns'],
            zero_receipt_ns=consumer_zero['zero_receipt_ns'],
            phase_isolation=limited_phase,
        )
        zero_indexes = [
            index
            for index, sample in enumerate(relevant_limited)
            if (
                sample[0] == consumer_zero['zero_receipt_ns']
                and _stamp_ns(sample[1]) == consumer_zero['zero_sim_ns']
                and is_zero(sample[1])
            )
        ]
        if len(zero_indexes) != 1:
            raise ConsumerTraceAmbiguous(
                'provided controller zero was not uniquely observed',
                matching_zero_count=len(zero_indexes),
                zero_receipt_ns=consumer_zero['zero_receipt_ns'],
                zero_sim_ns=consumer_zero['zero_sim_ns'],
            )
        if zero_indexes[0] != first_zero_index:
            raise ConsumerTraceAmbiguous(
                'provided controller zero disagrees with first relevant zero',
                first_relevant_zero_index=first_zero_index,
                provided_zero_index=zero_indexes[0],
                phase_isolation=limited_phase,
            )
        limited_receipt_fence_ns = consumer_zero.get(
            'limited_receipt_fence_ns'
        )
        if (
            limited_receipt_fence_ns is not None
            and consumer_zero['zero_receipt_ns']
            <= limited_receipt_fence_ns
        ):
            raise ConsumerTraceAmbiguous(
                'provided controller zero predates signal-boundary receipt '
                'fence',
                limited_receipt_fence_ns=limited_receipt_fence_ns,
                zero_receipt_ns=consumer_zero['zero_receipt_ns'],
            )
        zero_prefix = relevant_limited[first_zero_index:]
        first_zero = zero_prefix[0]
        first_zero_stamp_ns = _stamp_ns(first_zero[1])
        first_zero_sequence = self._limited_publication_sequence(
            first_zero,
            checkpoint='first-zero',
        )
        for observation in zero_prefix:
            if not is_zero(observation[1]):
                raise ConsumerTraceAmbiguous(
                    'non-zero controller output after first zero',
                    first_zero_receipt_ns=first_zero[0],
                    first_zero_stamp_ns=first_zero_stamp_ns,
                    observed_receipt_ns=observation[0],
                    observed_stamp_ns=_stamp_ns(observation[1]),
                )
        watermark = zero_prefix[-1]
        watermark_stamp_ns = _stamp_ns(watermark[1])
        if (
            watermark_stamp_ns - first_zero_stamp_ns
            < FINAL_STREAM_QUIESCENCE_NS
        ):
            return None
        return {
            'first_zero_stamp_ns': first_zero_stamp_ns,
            'first_zero_receipt_ns': first_zero[0],
            'first_zero_publication_sequence_number': first_zero_sequence,
            'watermark_stamp_ns': watermark_stamp_ns,
            'watermark_receipt_ns': watermark[0],
            'watermark_publication_sequence_number': (
                self._limited_publication_sequence(
                    watermark,
                    checkpoint='watermark',
                )
            ),
        }

    def _assert_limited_publication_sequence_continuity(
        self,
        limited_snapshot: tuple[Any, ...],
    ) -> None:
        previous_sequence = None
        for observation in limited_snapshot:
            sequence = self._limited_publication_sequence(
                observation,
                checkpoint='observation',
            )
            if previous_sequence is not None:
                if sequence == previous_sequence:
                    raise ConsumerTraceAmbiguous(
                        'duplicate limited publication sequence',
                        previous_sequence=previous_sequence,
                        publication_sequence_number=sequence,
                    )
                if sequence < previous_sequence:
                    raise ConsumerTraceAmbiguous(
                        'limited publication sequence regressed',
                        previous_sequence=previous_sequence,
                        publication_sequence_number=sequence,
                    )
                if sequence > previous_sequence + 1:
                    raise ConsumerTraceAmbiguous(
                        'limited publication sequence gap',
                        expected_sequence=previous_sequence + 1,
                        publication_sequence_number=sequence,
                        missing_sequence_count=(
                            sequence - previous_sequence - 1
                        ),
                    )
            previous_sequence = sequence

    @staticmethod
    def _limited_publication_sequence(
        observation: Any,
        *,
        checkpoint: str,
    ) -> int:
        sequence = getattr(observation, 'publication_sequence_number', None)
        if not isinstance(sequence, int) or isinstance(sequence, bool):
            raise ConsumerTraceAmbiguous(
                'limited publication sequence unavailable',
                checkpoint=checkpoint,
                receipt_ns=observation[0],
                stamp_ns=_stamp_ns(observation[1]),
            )
        return sequence

    @staticmethod
    def _limited_zero_watermark_evidence(
        limited_snapshot: tuple[Any, ...],
        consumer_zero: dict[str, int],
        *,
        source_anchor: tuple[int, Any],
    ) -> dict[str, Any]:
        relevant_limited, phase_isolation = _relevant_trace_phase(
            limited_snapshot,
            source_anchor=source_anchor,
            topic=LIMITED_COMMAND_TOPIC,
        )
        _, first_zero = _first_relevant_controller_zero(
            relevant_limited,
            source_anchor=source_anchor,
            zero_sim_ns=consumer_zero['zero_sim_ns'],
            zero_receipt_ns=consumer_zero['zero_receipt_ns'],
            phase_isolation=phase_isolation,
        )
        last_sample = relevant_limited[-1]
        return {
            'first_zero_receipt_ns': first_zero[0],
            'first_zero_stamp_ns': _stamp_ns(first_zero[1]),
            'last_limited_receipt_ns': last_sample[0],
            'last_limited_stamp_ns': _stamp_ns(last_sample[1]),
            'required_progress_ns': FINAL_STREAM_QUIESCENCE_NS,
            'limited_phase_isolation': phase_isolation,
        }

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
        try:
            endpoint_gid = bytes(matching[0].endpoint_gid)
        except (AttributeError, TypeError, ValueError) as error:
            raise AssertionError(
                'final-command endpoint GID is unavailable at signal boundary'
            ) from error
        if not endpoint_gid:
            raise AssertionError(
                'final-command endpoint GID is unavailable at signal boundary'
            )
        if not any(endpoint_gid):
            raise AssertionError(
                'final-command endpoint GID is invalid at signal boundary'
            )
        return endpoint_gid.hex()

    def publisher_count(self, topic: str) -> int:
        return len(self.node.get_publishers_info_by_topic(topic))

    def wait_no_publishers(self, topic: str, timeout: float = 10.0) -> None:
        self.wait_until(
            lambda: self.publisher_count(topic) == 0,
            timeout,
            f'no publishers on stale topic {topic}',
        )

    def assert_stale_gate_tuple(self, old_state: Any):
        self.wait_until(
            lambda: self.gate_client.wait_for_service(timeout_sec=0.2),
            10.0,
            'new Gate control service',
        )
        before = self.latest(self.gate_states)
        request = self._gate_control_type.Request()
        request.operation = self._gate_control_type.Request.RENEW
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
        if response.code != self._gate_control_type.Response.REJECTED:
            raise AssertionError(
                'old Gate tuple was accepted: ' + response.detail
            )
        if response.reason not in (
            self._gate_control_type.Response.STALE_GATE,
            self._gate_control_type.Response.STALE_SEQUENCE,
            self._gate_control_type.Response.STALE_LEASE,
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
        request = self._gate_control_type.Request()
        request.operation = self._gate_control_type.Request.INHIBIT
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
            if isinstance(message, self._gate_state_type):
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
            if isinstance(message, self._mission_state_type):
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
    result: Any,
    expected_code: int,
) -> None:
    if expected_code == type(result).SUCCEEDED:
        expected_status = GoalStatus.STATUS_SUCCEEDED
    else:
        expected_status = GoalStatus.STATUS_ABORTED
    if status != expected_status:
        raise AssertionError(f'unexpected Action status: {status}')
    if result.code != expected_code:
        raise AssertionError(
            f'unexpected Mission result code={result.code}, detail={result.detail}'
        )
