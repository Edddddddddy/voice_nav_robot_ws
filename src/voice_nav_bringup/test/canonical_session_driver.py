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

"""Package-private readiness seam for the installed VoiceNav session driver."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import threading
import time
from typing import Callable

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from voice_nav_interfaces.msg import MissionState, VoiceTurn


@dataclass(frozen=True)
class TypedSafetySample:
    """One callback-derived sample used by the initial admission barrier."""

    mission_gate_inhibited: bool
    active_step: int
    controller_zero: bool
    odom_stationary: bool

    @property
    def safe(self) -> bool:
        return (
            self.mission_gate_inhibited
            and self.active_step == 0xFFFFFFFF
            and self.controller_zero
            and self.odom_stationary
        )


class InitialSafeStationaryBarrier:
    """Require continuous typed safety samples for a monotonic hold window."""

    def __init__(self, *, now_ns: Callable[[], int], required_hold_ns: int):
        if required_hold_ns <= 0:
            raise ValueError('required_hold_ns must be positive')
        self._now_ns = now_ns
        self._required_hold_ns = required_hold_ns
        self._safe_since_ns: int | None = None

    def observe(self, sample: TypedSafetySample) -> bool:
        if not sample.safe:
            self._safe_since_ns = None
            return False
        if self._safe_since_ns is None:
            self._safe_since_ns = self._now_ns()
            return False
        return self.is_stable()

    def is_stable(self) -> bool:
        return (
            self._safe_since_ns is not None
            and self._now_ns() - self._safe_since_ns >= self._required_hold_ns
        )

    @property
    def safe_since_ns(self) -> int | None:
        return self._safe_since_ns


def _is_zero_command(message: TwistStamped) -> bool:
    twist = message.twist
    return all(
        abs(value) <= 1.0e-6
        for value in (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
    )


def _is_stationary_odometry(message: Odometry) -> bool:
    twist = message.twist.twist
    return (
        abs(twist.linear.x) <= 0.01
        and abs(twist.angular.z) <= 0.02
    )


class TypedObservationLedger:
    """Typed callback state used by every canonical driver predicate."""

    def __init__(self):
        self._mission_gate_inhibited = False
        self._active_step = 0xFFFFFFFF
        self._gate_armed_observed = False
        self._controller_nonzero_observed = False
        self._latest_controller_zero = False
        self._latest_odom_stationary = False
        self._stop_turn_observed = False
        self._clock_samples: list[tuple[int, int]] = []
        self._mission_state_events: list[tuple[int, MissionState]] = []
        self._command_events: list[tuple[int, TwistStamped, bool]] = []
        self._odometry_events: list[tuple[int, Odometry, bool]] = []
        self._voice_turn_events: list[tuple[int, VoiceTurn]] = []
        self._mission_status_events: list[tuple[int, str, int]] = []
        self._speak_success_events: list[tuple[int, str]] = []
        self._mission_goal_ids: set[str] = set()
        self._successful_mission_ids: set[str] = set()
        self._terminal_non_success_ids: set[str] = set()
        self._initial_odometry: Odometry | None = None
        self._latest_odometry: Odometry | None = None

    @staticmethod
    def _at_ns(at_ns: int | None) -> int:
        return time.monotonic_ns() if at_ns is None else at_ns

    @staticmethod
    def _goal_id(status) -> str:
        return bytes(status.goal_info.goal_id.uuid).hex()

    def on_clock(self, message: Clock, at_ns: int | None = None) -> None:
        clock_ns = int(message.clock.sec) * 1_000_000_000
        clock_ns += int(message.clock.nanosec)
        self._clock_samples.append((self._at_ns(at_ns), clock_ns))

    def on_mission_state(self, message: MissionState, at_ns: int | None = None) -> None:
        received_ns = self._at_ns(at_ns)
        self._mission_state_events.append((received_ns, message))
        self._gate_armed_observed |= message.gate_state == MissionState.GATE_ARMED
        self._mission_gate_inhibited = (
            message.gate_state == MissionState.GATE_INHIBITED
        )
        self._active_step = int(message.active_step)

    def on_controller_command(
        self,
        message: TwistStamped,
        at_ns: int | None = None,
    ) -> None:
        received_ns = self._at_ns(at_ns)
        is_zero = _is_zero_command(message)
        self._command_events.append((received_ns, message, is_zero))
        self._controller_nonzero_observed |= not is_zero
        self._latest_controller_zero = is_zero

    def on_odometry(self, message: Odometry, at_ns: int | None = None) -> None:
        received_ns = self._at_ns(at_ns)
        stationary = _is_stationary_odometry(message)
        self._odometry_events.append((received_ns, message, stationary))
        if self._initial_odometry is None:
            self._initial_odometry = message
        self._latest_odometry = message
        self._latest_odom_stationary = stationary

    def on_voice_turn(self, message: VoiceTurn, at_ns: int | None = None) -> None:
        self._voice_turn_events.append((self._at_ns(at_ns), message))
        self._stop_turn_observed |= message.kind == VoiceTurn.STOP

    def on_mission_status(
        self,
        message: GoalStatusArray,
        at_ns: int | None = None,
    ) -> None:
        received_ns = self._at_ns(at_ns)
        for status in message.status_list:
            goal_id = self._goal_id(status)
            status_code = int(status.status)
            self._mission_goal_ids.add(goal_id)
            self._mission_status_events.append(
                (received_ns, goal_id, status_code),
            )
            if status_code == GoalStatus.STATUS_SUCCEEDED:
                self._successful_mission_ids.add(goal_id)
            if status_code in (
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            ):
                self._terminal_non_success_ids.add(goal_id)

    def on_speak_status(
        self,
        message: GoalStatusArray,
        at_ns: int | None = None,
    ) -> None:
        received_ns = self._at_ns(at_ns)
        for status in message.status_list:
            if int(status.status) == GoalStatus.STATUS_SUCCEEDED:
                self._speak_success_events.append(
                    (received_ns, self._goal_id(status)),
                )

    @property
    def controller_nonzero_observed(self) -> bool:
        return self._controller_nonzero_observed

    @property
    def latest_controller_zero(self) -> bool:
        return self._latest_controller_zero

    @property
    def latest_odom_stationary(self) -> bool:
        return self._latest_odom_stationary

    @property
    def gate_armed_observed(self) -> bool:
        return self._gate_armed_observed

    @property
    def latest_safety_sample(self) -> TypedSafetySample:
        return TypedSafetySample(
            mission_gate_inhibited=self._mission_gate_inhibited,
            active_step=self._active_step,
            controller_zero=self._latest_controller_zero,
            odom_stationary=self._latest_odom_stationary,
        )

    @property
    def stop_turn_observed(self) -> bool:
        return self._stop_turn_observed

    @property
    def clock_samples(self) -> tuple[tuple[int, int], ...]:
        return tuple(self._clock_samples)

    @property
    def mission_goal_ids(self) -> frozenset[str]:
        return frozenset(self._mission_goal_ids)

    @property
    def successful_mission_ids(self) -> frozenset[str]:
        return frozenset(self._successful_mission_ids)

    @property
    def terminal_non_success_ids(self) -> frozenset[str]:
        return frozenset(self._terminal_non_success_ids)

    @property
    def initial_odometry(self) -> Odometry | None:
        return self._initial_odometry

    @property
    def latest_odometry(self) -> Odometry | None:
        return self._latest_odometry

    @property
    def voice_turn_events(self) -> tuple[tuple[int, VoiceTurn], ...]:
        return tuple(self._voice_turn_events)

    @property
    def speak_success_events(self) -> tuple[tuple[int, str], ...]:
        return tuple(self._speak_success_events)

    @property
    def mission_status_events(self) -> tuple[tuple[int, str, int], ...]:
        return tuple(self._mission_status_events)

    @property
    def command_events(self) -> tuple[tuple[int, TwistStamped, bool], ...]:
        return tuple(self._command_events)

    @property
    def odometry_events(self) -> tuple[tuple[int, Odometry, bool], ...]:
        return tuple(self._odometry_events)

    def has_clock_sample(self) -> bool:
        return any(clock_ns > 0 for _, clock_ns in self._clock_samples)

    def has_nonzero_command_after(self, after_ns: int) -> bool:
        return any(
            received_ns >= after_ns and not is_zero
            for received_ns, _, is_zero in self._command_events
        )

    def post_stop_controller_watermark(
        self,
        stop_request_ns: int,
        *,
        stop_phase_end_ns: int | None = None,
    ) -> dict[str, object]:
        """Return a typed receipt-order zero watermark within an optional bound."""
        def in_stop_phase(received_ns: int) -> bool:
            return (
                received_ns >= stop_request_ns
                and (
                    stop_phase_end_ns is None
                    or received_ns < stop_phase_end_ns
                )
            )

        first_zero_index: int | None = None
        first_zero_ns: int | None = None
        for index, (received_ns, _, is_zero) in enumerate(self._command_events):
            if in_stop_phase(received_ns) and is_zero:
                first_zero_index = index
                first_zero_ns = received_ns
                break

        result: dict[str, object]
        if first_zero_index is None:
            result = {
                'stop_request_ns': stop_request_ns,
                'first_post_stop_zero_ns': None,
                'post_stop_zero_watermark_nonzero_count': 0,
                'post_stop_zero_watermark_nonzero': False,
            }
        else:
            nonzero_count = sum(
                not is_zero
                for received_ns, _, is_zero in self._command_events[first_zero_index + 1:]
                if in_stop_phase(received_ns)
            )
            result = {
                'stop_request_ns': stop_request_ns,
                'first_post_stop_zero_ns': first_zero_ns,
                'post_stop_zero_watermark_nonzero_count': nonzero_count,
                'post_stop_zero_watermark_nonzero': nonzero_count > 0,
            }
        if stop_phase_end_ns is not None:
            result['stop_phase_end_ns'] = stop_phase_end_ns
        return result

    def has_angular_command_after(self, after_ns: int) -> bool:
        return any(
            received_ns >= after_ns and abs(message.twist.angular.z) > 1.0e-6
            for received_ns, message, _ in self._command_events
        )

    def has_zero_command_after(self, after_ns: int) -> bool:
        return any(
            received_ns >= after_ns and is_zero
            for received_ns, _, is_zero in self._command_events
        )

    def has_stationary_odom_after(self, after_ns: int) -> bool:
        return any(
            received_ns >= after_ns and stationary
            for received_ns, _, stationary in self._odometry_events
        )

    def has_stop_turn_after(self, after_ns: int) -> bool:
        return any(
            received_ns >= after_ns and message.kind == VoiceTurn.STOP
            for received_ns, message in self._voice_turn_events
        )

    def has_command_turn_after(self, after_ns: int) -> bool:
        return any(
            received_ns >= after_ns and message.kind == VoiceTurn.COMMAND
            for received_ns, message in self._voice_turn_events
        )

    def has_terminal_non_success_after(self, after_ns: int) -> bool:
        return any(
            received_ns >= after_ns
            and status_code in (
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            )
            for received_ns, _, status_code in self._mission_status_events
        )

    def has_successful_mission_after(self, after_ns: int) -> bool:
        return any(
            received_ns >= after_ns
            and status_code == GoalStatus.STATUS_SUCCEEDED
            for received_ns, _, status_code in self._mission_status_events
        )

    def has_speak_success_after(self, after_ns: int) -> bool:
        return any(received_ns >= after_ns for received_ns, _ in self._speak_success_events)


class StopPhaseFinalizer:
    """Freeze STOP evidence under the same condition used by typed callbacks."""

    def __init__(self, condition: threading.Condition):
        self._condition = condition
        self._frozen_snapshot: dict[str, object] | None = None

    def _finalize_locked(
        self,
        ledger: TypedObservationLedger,
        *,
        stop_request_ns: int,
        stop_phase_end_ns: int,
        stationary_hold_ms: int | None = None,
    ) -> dict[str, object]:
        if self._frozen_snapshot is not None:
            return dict(self._frozen_snapshot)

        watermark = ledger.post_stop_controller_watermark(
            stop_request_ns,
            stop_phase_end_ns=stop_phase_end_ns,
        )
        recovery = bool(watermark['post_stop_zero_watermark_nonzero'])
        if recovery:
            status = 'blocked'
            reason = 'post_stop_nonzero_before_rotate_admission'
        elif watermark['first_post_stop_zero_ns'] is None:
            status = 'blocked'
            reason = 'post_stop_zero_watermark_not_observed'
        else:
            status = 'ready'
            reason = 'stop_phase_frozen_before_rotate_admission'
        snapshot = dict(watermark)
        snapshot.update({
            'status': status,
            'reason': reason,
            'recovery': recovery,
        })
        if stationary_hold_ms is not None:
            snapshot['stationary_hold_ms'] = stationary_hold_ms
        self._frozen_snapshot = snapshot
        return dict(snapshot)

    def finalize(
        self,
        ledger: TypedObservationLedger,
        *,
        stop_request_ns: int,
        stop_phase_end_ns: int,
        stationary_hold_ms: int | None = None,
    ) -> dict[str, object]:
        with self._condition:
            return self._finalize_locked(
                ledger,
                stop_request_ns=stop_request_ns,
                stop_phase_end_ns=stop_phase_end_ns,
                stationary_hold_ms=stationary_hold_ms,
            )

    def frozen_snapshot(self) -> dict[str, object] | None:
        with self._condition:
            if self._frozen_snapshot is None:
                return None
            return dict(self._frozen_snapshot)


def parameter_admission_result(
    response: SetParametersResult,
) -> dict[str, object]:
    """Preserve the typed SetParameters success bit and reason verbatim."""
    return {
        'status': 'accepted' if response.successful else 'rejected',
        'successful': bool(response.successful),
        'reason': response.reason,
    }


STOP_COMMANDS = frozenset(('停止', '小智停止', '紧急停止'))


def sequence_parameter_admission(
    text: str,
    *,
    attempt: Callable[[str, float], dict[str, object]],
    generation: Callable[[], int],
    wait_for_new_safe_sample: Callable[[int, float], bool],
    timeout_s: float,
) -> dict[str, object]:
    """Bound ordinary admission retries to new typed safety generations.

    The service result is the only admission authority. A busy ordinary
    command may be retried only after the supplied waiter observes a strictly
    newer typed safety generation. STOP phrases are deliberately single-shot
    so a late ordinary result cannot duplicate or reset the stop turn.
    """
    deadline_ns = time.monotonic_ns() + int(timeout_s * 1_000_000_000)
    attempts: list[dict[str, object]] = []

    def finish(result: dict[str, object]) -> dict[str, object]:
        finished = dict(result)
        finished['attempts'] = [dict(item) for item in attempts]
        return finished

    def timeout(reason: str) -> dict[str, object]:
        return {
            'status': 'timeout',
            'successful': False,
            'reason': reason,
            'text': text,
            'attempts': [dict(item) for item in attempts],
        }

    while True:
        remaining_ns = deadline_ns - time.monotonic_ns()
        if remaining_ns <= 0:
            return timeout('ordinary_parameter_admission_retry_timeout')

        result = dict(attempt(text, remaining_ns / 1_000_000_000))
        attempts.append(dict(result))
        if result.get('successful') is True or text in STOP_COMMANDS:
            return finish(result)

        reason = str(result.get('reason', ''))
        if not reason.startswith('command_text is busy'):
            return finish(result)

        observed_generation = generation()
        remaining_ns = deadline_ns - time.monotonic_ns()
        if remaining_ns <= 0:
            return timeout('ordinary_parameter_admission_retry_timeout')
        if not wait_for_new_safe_sample(
            observed_generation,
            remaining_ns / 1_000_000_000,
        ):
            return timeout('new_typed_safe_sample_for_parameter_retry')
        if generation() <= observed_generation:
            return timeout('typed_safe_sample_generation_not_advanced')


def _yaw_from_odometry(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def _wrapped_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _state_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class TypedCanonicalSessionDriver:
    """One event-driven typed driver for readiness and the product sequence."""

    INITIAL_HOLD_NS = 2_000_000_000
    FINAL_HOLD_NS = 200_000_000
    ACTIVE_STEP_NONE = 0xFFFFFFFF

    def __init__(self, node, *, command_node: str = 'voice_nav_command_gateway'):
        self.node = node
        self.command_node = command_node
        self.condition = threading.Condition()
        self.ledger = TypedObservationLedger()
        self.stop_phase_finalizer = StopPhaseFinalizer(self.condition)
        self.initial_barrier = InitialSafeStationaryBarrier(
            now_ns=time.monotonic_ns,
            required_hold_ns=self.INITIAL_HOLD_NS,
        )
        self.stop_started_ns: int | None = None
        self.stop_safe_since_ns: int | None = None
        self.stop_phase_end_ns: int | None = None
        self.rotate_started_ns: int | None = None
        self.rotate_safe_since_ns: int | None = None
        self.gate_inhibited_after_motion = False
        self._observation_generation = 0
        self._subscriptions = self._create_subscriptions()
        self._parameter_client = node.create_client(
            SetParameters,
            f'/{command_node}/set_parameters',
        )

    def _create_subscriptions(self):
        state_qos = _state_qos()
        reliable_qos = _reliable_qos()
        return [
            self.node.create_subscription(
                Clock,
                '/clock',
                self._on_clock,
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                MissionState,
                '/mission/state',
                self._on_mission_state,
                state_qos,
            ),
            self.node.create_subscription(
                TwistStamped,
                '/diff_drive_controller/cmd_vel',
                self._on_controller_command,
                reliable_qos,
            ),
            self.node.create_subscription(
                Odometry,
                '/odom',
                self._on_odometry,
                reliable_qos,
            ),
            self.node.create_subscription(
                VoiceTurn,
                '/voice/turn',
                self._on_voice_turn,
                reliable_qos,
            ),
            self.node.create_subscription(
                GoalStatusArray,
                '/mission/execute/_action/status',
                self._on_mission_status,
                reliable_qos,
            ),
            self.node.create_subscription(
                GoalStatusArray,
                '/voice/speak/_action/status',
                self._on_speak_status,
                reliable_qos,
            ),
        ]

    def _notify(self) -> None:
        self.condition.notify_all()

    def _update_safety_windows_locked(self, now_ns: int) -> None:
        sample = self.ledger.latest_safety_sample
        self.initial_barrier.observe(sample)
        if self.stop_started_ns is not None:
            if sample.safe:
                if self.stop_safe_since_ns is None:
                    self.stop_safe_since_ns = now_ns
            else:
                self.stop_safe_since_ns = None
            self.gate_inhibited_after_motion = (
                self.gate_inhibited_after_motion
                or (
                    self.ledger.gate_armed_observed
                    and sample.mission_gate_inhibited
                )
            )
        if self.rotate_started_ns is not None:
            if sample.safe:
                if self.rotate_safe_since_ns is None:
                    self.rotate_safe_since_ns = now_ns
            else:
                self.rotate_safe_since_ns = None

    def _on_clock(self, message: Clock) -> None:
        with self.condition:
            self.ledger.on_clock(message)
            self._observation_generation += 1
            self._notify()

    def _on_mission_state(self, message: MissionState) -> None:
        with self.condition:
            now_ns = time.monotonic_ns()
            self.ledger.on_mission_state(message, now_ns)
            self._update_safety_windows_locked(now_ns)
            self._observation_generation += 1
            self._notify()

    def _on_controller_command(self, message: TwistStamped) -> None:
        with self.condition:
            now_ns = time.monotonic_ns()
            self.ledger.on_controller_command(message, now_ns)
            self._update_safety_windows_locked(now_ns)
            self._observation_generation += 1
            self._notify()

    def _on_odometry(self, message: Odometry) -> None:
        with self.condition:
            now_ns = time.monotonic_ns()
            self.ledger.on_odometry(message, now_ns)
            self._update_safety_windows_locked(now_ns)
            self._observation_generation += 1
            self._notify()

    def _on_voice_turn(self, message: VoiceTurn) -> None:
        with self.condition:
            self.ledger.on_voice_turn(message)
            self._observation_generation += 1
            self._notify()

    def _on_mission_status(self, message: GoalStatusArray) -> None:
        with self.condition:
            self.ledger.on_mission_status(message)
            self._observation_generation += 1
            self._notify()

    def _on_speak_status(self, message: GoalStatusArray) -> None:
        with self.condition:
            self.ledger.on_speak_status(message)
            self._observation_generation += 1
            self._notify()

    def _wait_until(
        self,
        predicate: Callable[[], bool],
        *,
        timeout_s: float,
        reason: str,
    ) -> dict[str, object]:
        deadline_ns = time.monotonic_ns() + int(timeout_s * 1_000_000_000)
        with self.condition:
            while not predicate():
                remaining_ns = deadline_ns - time.monotonic_ns()
                if remaining_ns <= 0:
                    return {
                        'status': 'timeout',
                        'reason': reason,
                        'timeout_s': timeout_s,
                    }
                self.condition.wait(timeout=remaining_ns / 1_000_000_000)
        return {'status': 'ready', 'reason': reason}

    def wait_for_clock(self, timeout_s: float) -> dict[str, object]:
        return self._wait_until(
            self.ledger.has_clock_sample,
            timeout_s=timeout_s,
            reason='first_real_clock_sample',
        )

    def wait_for_parameter_service(self, timeout_s: float) -> dict[str, object]:
        if self._parameter_client.wait_for_service(timeout_sec=timeout_s):
            return {
                'status': 'ready',
                'reason': 'voice_command_gateway_set_parameters_service',
            }
        return {
            'status': 'timeout',
            'reason': 'voice_command_gateway_set_parameters_service_timeout',
            'timeout_s': timeout_s,
        }

    def wait_for_initial_safe_stationary(self, timeout_s: float) -> dict[str, object]:
        def ready() -> bool:
            return (
                self.ledger.has_clock_sample()
                and self.ledger.latest_safety_sample.safe
                and self.initial_barrier.is_stable()
            )

        result = self._wait_until(
            ready,
            timeout_s=timeout_s,
            reason='initial_safe_stationary_barrier_2s',
        )
        if result['status'] == 'ready':
            result.update({
                'hold_ns': self.INITIAL_HOLD_NS,
                'safe_since_ns': self.initial_barrier.safe_since_ns,
            })
        return result

    def set_command_text(self, text: str, timeout_s: float = 30.0) -> dict[str, object]:
        if not self._parameter_client.wait_for_service(timeout_sec=timeout_s):
            return {
                'status': 'timeout',
                'successful': False,
                'reason': 'set_parameters_service_timeout',
            }
        request = SetParameters.Request()
        request.parameters = [
            Parameter('command_text', value=text).to_parameter_msg(),
        ]
        future = self._parameter_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout=timeout_s):
            return {
                'status': 'timeout',
                'successful': False,
                'reason': 'set_parameters_response_timeout',
            }
        try:
            response = future.result()
        except Exception as error:
            return {
                'status': 'error',
                'successful': False,
                'reason': f'set_parameters_call_failed:{error}',
            }
        if not response.results:
            return {
                'status': 'error',
                'successful': False,
                'reason': 'set_parameters_response_empty',
            }
        result = parameter_admission_result(response.results[0])
        result['text'] = text
        return result

    def _ordinary_retry_window_ready_locked(self, text: str) -> bool:
        if not self.ledger.latest_safety_sample.safe:
            return False
        if text == '前进 2 米':
            return self.initial_barrier.is_stable()
        return (
            self.stop_safe_since_ns is not None
            and time.monotonic_ns() - self.stop_safe_since_ns >= self.FINAL_HOLD_NS
        )

    def set_command_text_until_accepted(
        self,
        text: str,
        *,
        timeout_s: float = 60.0,
    ) -> dict[str, object]:
        """Retry only after a new typed safe sample, preserving actual reasons."""
        def current_generation() -> int:
            with self.condition:
                return self._observation_generation

        def wait_for_new_safe_sample(
            after_generation: int,
            wait_timeout_s: float,
        ) -> bool:
            result = self._wait_until(
                lambda: (
                    self._observation_generation > after_generation
                    and self._ordinary_retry_window_ready_locked(text)
                ),
                timeout_s=wait_timeout_s,
                reason='new_typed_safe_sample_for_parameter_retry',
            )
            return result['status'] == 'ready'

        return sequence_parameter_admission(
            text,
            attempt=lambda command, attempt_timeout_s: self.set_command_text(
                command,
                timeout_s=min(30.0, attempt_timeout_s),
            ),
            generation=current_generation,
            wait_for_new_safe_sample=wait_for_new_safe_sample,
            timeout_s=timeout_s,
        )

    def _mark_stop_start(self) -> int:
        with self.condition:
            self.stop_started_ns = time.monotonic_ns()
            self.stop_safe_since_ns = None
            self.stop_phase_end_ns = None
            self.gate_inhibited_after_motion = False
            self._notify()
            return self.stop_started_ns

    def _mark_rotate_start(self) -> int:
        with self.condition:
            self.rotate_started_ns = time.monotonic_ns()
            self.rotate_safe_since_ns = None
            self._notify()
            return self.rotate_started_ns

    def _finalize_stop_phase_before_rotate(
        self,
        stop_request_ns: int,
    ) -> dict[str, object]:
        """Freeze STOP and open the ROTATE boundary while holding condition."""
        with self.condition:
            stop_phase_end_ns = time.monotonic_ns()
            stationary_hold_ms = (
                (stop_phase_end_ns - self.stop_safe_since_ns) // 1_000_000
                if self.stop_safe_since_ns is not None else 0
            )
            snapshot = self.stop_phase_finalizer._finalize_locked(
                self.ledger,
                stop_request_ns=stop_request_ns,
                stop_phase_end_ns=stop_phase_end_ns,
                stationary_hold_ms=int(stationary_hold_ms),
            )
            self.stop_phase_end_ns = stop_phase_end_ns
            if snapshot['status'] == 'ready':
                self.rotate_started_ns = stop_phase_end_ns
                self.rotate_safe_since_ns = None
                self._notify()
            return snapshot

    def wait_for_move(self, after_ns: int, timeout_s: float) -> dict[str, object]:
        return self._wait_until(
            lambda: (
                self.ledger.has_nonzero_command_after(after_ns)
                and len(self.ledger.mission_goal_ids) >= 1
            ),
            timeout_s=timeout_s,
            reason='move_controller_nonzero',
        )

    def wait_for_stop(self, after_ns: int, timeout_s: float) -> dict[str, object]:
        def stopped() -> bool:
            hold_ready = (
                self.stop_safe_since_ns is not None
                and time.monotonic_ns() - self.stop_safe_since_ns >= self.FINAL_HOLD_NS
            )
            watermark = self.ledger.post_stop_controller_watermark(after_ns)
            return (
                self.ledger.has_stop_turn_after(after_ns)
                and self.ledger.has_terminal_non_success_after(after_ns)
                and self.ledger.has_speak_success_after(after_ns)
                and self.gate_inhibited_after_motion
                and self.ledger.has_zero_command_after(after_ns)
                and self.ledger.has_stationary_odom_after(after_ns)
                and watermark['first_post_stop_zero_ns'] is not None
                and not watermark['post_stop_zero_watermark_nonzero']
                and hold_ready
            )

        return self._wait_until(
            stopped,
            timeout_s=timeout_s,
            reason='formal_stop_and_safe_stationary_200ms',
        )

    def wait_for_rotate(self, after_ns: int, timeout_s: float) -> dict[str, object]:
        def rotated() -> bool:
            hold_ready = (
                self.rotate_safe_since_ns is not None
                and time.monotonic_ns() - self.rotate_safe_since_ns >= self.FINAL_HOLD_NS
            )
            return (
                self.ledger.has_command_turn_after(after_ns)
                and self.ledger.has_angular_command_after(after_ns)
                and self.ledger.has_successful_mission_after(after_ns)
                and self.ledger.has_speak_success_after(after_ns)
                and hold_ready
            )

        return self._wait_until(
            rotated,
            timeout_s=timeout_s,
            reason='rotate_success_and_final_stationary_200ms',
        )

    def _failure(
        self,
        reason: str,
        phases: dict[str, object],
    ) -> dict[str, object]:
        return {
            'status': 'blocked',
            'failure': reason,
            'phases': phases,
            'evidence': self.evidence(),
        }

    def run(self, timeout_s: float = 180.0) -> dict[str, object]:
        return self.run_with_readiness(timeout_s=timeout_s)

    def run_with_readiness(
        self,
        *,
        timeout_s: float = 180.0,
        readiness_fifo: str | None = None,
    ) -> dict[str, object]:
        phases: dict[str, object] = {}
        service = self.wait_for_parameter_service(min(timeout_s, 60.0))
        phases['parameter_service'] = service
        if service['status'] != 'ready':
            return self._failure('voice_command_gateway_not_ready', phases)
        readiness = self.wait_for_clock(min(timeout_s, 90.0))
        phases['clock'] = readiness
        if readiness['status'] != 'ready':
            return self._failure('first_real_clock_sample_not_observed', phases)
        safe = self.wait_for_initial_safe_stationary(min(timeout_s, 30.0))
        phases['initial_safe_stationary'] = safe
        if safe['status'] != 'ready':
            return self._failure('initial_safe_stationary_barrier_not_ready', phases)
        if readiness_fifo is not None:
            with open(readiness_fifo, 'w', encoding='utf-8') as stream:
                stream.write(json.dumps(safe, sort_keys=True) + '\n')
                stream.flush()

        move_start = time.monotonic_ns()
        move_admission = self.set_command_text_until_accepted('前进 2 米')
        phases['move_admission'] = move_admission
        if move_admission.get('successful') is not True:
            return self._failure('move_parameter_admission_rejected', phases)
        phases['move_observation'] = self.wait_for_move(move_start, 60.0)
        if phases['move_observation']['status'] != 'ready':
            return self._failure('move_nonzero_not_observed', phases)

        stop_start = self._mark_stop_start()
        stop_admission = self.set_command_text('停止')
        phases['stop_admission'] = stop_admission
        if stop_admission.get('successful') is not True:
            return self._failure('stop_parameter_admission_rejected', phases)
        phases['stop_observation'] = self.wait_for_stop(stop_start, 60.0)
        if phases['stop_observation']['status'] != 'ready':
            return self._failure('formal_stop_or_safe_stationary_not_observed', phases)

        stop_phase = self._finalize_stop_phase_before_rotate(stop_start)
        phases['stop_phase'] = stop_phase
        if stop_phase['status'] != 'ready':
            return self._failure('stop_phase_finalization_failed', phases)

        rotate_start = self.rotate_started_ns
        if rotate_start is None:
            return self._failure('rotate_admission_boundary_not_set', phases)
        rotate_admission = self.set_command_text_until_accepted('右转九十度')
        phases['rotate_admission'] = rotate_admission
        if rotate_admission.get('successful') is not True:
            return self._failure('rotate_parameter_admission_rejected', phases)
        phases['rotate_observation'] = self.wait_for_rotate(rotate_start, 60.0)
        if phases['rotate_observation']['status'] != 'ready':
            return self._failure('rotate_or_final_stationary_not_observed', phases)

        return {
            'status': 'ready',
            'phases': phases,
            'evidence': self.evidence(),
        }

    def evidence(self) -> dict[str, object]:
        with self.condition:
            turns = [
                {
                    'voice_instance_id': message.voice_instance_id,
                    'voice_seq': int(message.voice_seq),
                    'session_id': message.session_id,
                    'turn_id': message.turn_id,
                    'kind': int(message.kind),
                    'text': message.text,
                }
                for _, message in self.ledger.voice_turn_events
            ]
            initial = self.ledger.initial_odometry
            latest = self.ledger.latest_odometry
            displacement = None
            yaw_delta = None
            if initial is not None and latest is not None:
                initial_yaw = _yaw_from_odometry(initial)
                latest_yaw = _yaw_from_odometry(latest)
                displacement = (
                    (latest.pose.pose.position.x - initial.pose.pose.position.x)
                    * math.cos(initial_yaw)
                    + (latest.pose.pose.position.y - initial.pose.pose.position.y)
                    * math.sin(initial_yaw)
                )
                yaw_delta = _wrapped_angle(latest_yaw - initial_yaw)
            now_ns = time.monotonic_ns()
            stop_snapshot = self.stop_phase_finalizer.frozen_snapshot()
            if stop_snapshot is None:
                stop_snapshot = {
                    'status': 'not_finalized',
                    'reason': 'stop_phase_not_frozen',
                    'stop_request_ns': self.stop_started_ns,
                    'first_post_stop_zero_ns': None,
                    'stop_phase_end_ns': self.stop_phase_end_ns,
                    'post_stop_zero_watermark_nonzero_count': None,
                    'post_stop_zero_watermark_nonzero': None,
                    'recovery': None,
                    'stationary_hold_ms': 0,
                }
            rotate_hold_ms = (
                (now_ns - self.rotate_safe_since_ns) // 1_000_000
                if self.rotate_safe_since_ns is not None else 0
            )
            rotate_nonzero = (
                self.ledger.has_angular_command_after(self.rotate_started_ns)
                if self.rotate_started_ns is not None else False
            )
            return {
                'voice': {
                    'turns': turns,
                    'speak_completed_count': len(self.ledger.speak_success_events),
                },
                'missions': {
                    'unique_goal_count': len(self.ledger.mission_goal_ids),
                    'successful_goal_count': len(self.ledger.successful_mission_ids),
                    'terminal_non_success_goal_count': len(
                        self.ledger.terminal_non_success_ids
                    ),
                },
                'motion': {
                    'displacement_m': displacement,
                    'yaw_delta_rad': yaw_delta,
                    'controller_nonzero_observed': (
                        self.ledger.controller_nonzero_observed
                    ),
                    'post_stop_nonzero_command_observed': (
                        stop_snapshot['post_stop_zero_watermark_nonzero']
                    ),
                    'stop_request_ns': stop_snapshot['stop_request_ns'],
                    'first_post_stop_zero_ns': (
                        stop_snapshot['first_post_stop_zero_ns']
                    ),
                    'post_stop_zero_watermark_nonzero_count': (
                        stop_snapshot['post_stop_zero_watermark_nonzero_count']
                    ),
                    'post_stop_zero_watermark_nonzero': (
                        stop_snapshot['post_stop_zero_watermark_nonzero']
                    ),
                    'stop_phase_end_ns': stop_snapshot['stop_phase_end_ns'],
                    'gate_inhibited_after_motion': self.gate_inhibited_after_motion,
                    'final_gate_inhibited': self.ledger.latest_safety_sample.mission_gate_inhibited,
                    'final_command_is_zero': self.ledger.latest_controller_zero,
                    'final_odometry_is_stationary': self.ledger.latest_odom_stationary,
                    'stop_stationary_hold_ms': int(stop_snapshot['stationary_hold_ms']),
                    'rotate_stationary_hold_ms': int(rotate_hold_ms),
                    'rotate_admission_start_ns': self.rotate_started_ns,
                    'rotate_nonzero_command_observed': rotate_nonzero,
                },
                'stop_phase': stop_snapshot,
                'rotate_motion': {
                    'admission_start_ns': self.rotate_started_ns,
                    'controller_nonzero_observed': rotate_nonzero,
                },
                'readiness': {
                    'clock_sample_count': len(self.ledger.clock_samples),
                    'initial_safe_stationary_hold_ns': self.INITIAL_HOLD_NS,
                },
            }


def run_canonical_session(
    *,
    timeout_s: float = 180.0,
    command_node: str = 'voice_nav_command_gateway',
    readiness_fifo: str | None = None,
) -> dict[str, object]:
    """Run the canonical sequence with one typed rclpy observer process."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    node = rclpy.create_node('voice_nav_canonical_session_driver')
    executor = SingleThreadedExecutor()
    driver = TypedCanonicalSessionDriver(node, command_node=command_node)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        return driver.run_with_readiness(
            timeout_s=timeout_s,
            readiness_fifo=readiness_fifo,
        )
    except Exception as error:
        return {
            'status': 'blocked',
            'failure': f'driver_exception:{error}',
            'phases': {},
            'evidence': driver.evidence(),
        }
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


def wait_for_first_real_clock_sample(
    wait_for_sample: Callable[[Callable[[int], None], float], bool],
    *,
    timeout_s: float,
) -> dict[str, object]:
    """Wait for an actual positive simulation-clock sample, fail-closed.

    ``wait_for_sample`` owns the event-driven wait (the ROS adapter below uses
    ``rclpy.spin_until_future_complete``).  This function only accepts a
    callback-delivered sample, so a missing topic/graph cannot be mistaken for
    readiness by a single immediate graph query or a fixed sleep.
    """
    if timeout_s <= 0:
        return {
            'status': 'timeout',
            'reason': 'invalid_clock_sample_timeout',
            'clock_ns': None,
            'sample_count': 0,
        }

    samples: list[int] = []

    def on_sample(clock_ns: int) -> None:
        if isinstance(clock_ns, bool) or not isinstance(clock_ns, int):
            return
        if clock_ns > 0:
            samples.append(clock_ns)

    try:
        wait_completed = wait_for_sample(on_sample, timeout_s)
    except Exception:
        return {
            'status': 'error',
            'reason': 'clock_sample_wait_failed',
            'clock_ns': None,
            'sample_count': len(samples),
        }

    if wait_completed and samples:
        return {
            'status': 'ready',
            'reason': 'first_real_clock_sample',
            'clock_ns': samples[0],
            'sample_count': len(samples),
        }
    return {
        'status': 'timeout',
        'reason': 'clock_sample_timeout',
        'clock_ns': None,
        'sample_count': len(samples),
    }


def _wait_for_ros_sample(node, on_sample, timeout_s: float) -> bool:
    """Bridge one ROS Clock callback to a Future-backed event wait."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.task import Future
    from rosgraph_msgs.msg import Clock

    future = Future()

    def on_clock(message: Clock) -> None:
        clock_ns = int(message.clock.sec) * 1_000_000_000
        clock_ns += int(message.clock.nanosec)
        on_sample(clock_ns)
        if clock_ns > 0 and not future.done():
            future.set_result(True)

    subscription = node.create_subscription(
        Clock,
        '/clock',
        on_clock,
        qos_profile_sensor_data,
    )
    try:
        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=timeout_s,
        )
        return bool(future.done())
    finally:
        node.destroy_subscription(subscription)


def wait_for_first_real_ros_clock_sample(timeout_s: float) -> dict[str, object]:
    """Observe the first real installed-session /clock sample."""
    import rclpy

    rclpy.init()
    node = rclpy.create_node('voice_nav_canonical_clock_driver')
    try:
        return wait_for_first_real_clock_sample(
            lambda on_sample, wait_timeout_s: _wait_for_ros_sample(
                node,
                on_sample,
                wait_timeout_s,
            ),
            timeout_s=timeout_s,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    """Run either the clock probe or the full typed canonical session."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--canonical-session', action='store_true')
    parser.add_argument('--wait-clock', action='store_true')
    parser.add_argument('--timeout-s', type=float, required=True)
    parser.add_argument(
        '--command-node',
        default='voice_nav_command_gateway',
    )
    parser.add_argument('--readiness-fifo')
    arguments = parser.parse_args(argv)
    if arguments.canonical_session:
        result = run_canonical_session(
            timeout_s=arguments.timeout_s,
            command_node=arguments.command_node,
            readiness_fifo=arguments.readiness_fifo,
        )
        print(json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)
        return 0 if result['status'] == 'ready' else 1
    if not arguments.wait_clock:
        parser.error('--wait-clock or --canonical-session is required')
    result = wait_for_first_real_ros_clock_sample(arguments.timeout_s)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)
    return 0 if result['status'] == 'ready' else 1


if __name__ == '__main__':
    raise SystemExit(main())
