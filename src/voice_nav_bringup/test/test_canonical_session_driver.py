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

"""Behavioral tests for the package-private installed-session driver seam."""

from __future__ import annotations

from threading import Condition, Event, Thread

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from voice_nav_interfaces.msg import MissionState, VoiceTurn

from canonical_session_driver import (
    InitialSafeStationaryBarrier,
    StopPhaseFinalizer,
    TypedObservationLedger,
    TypedSafetySample,
    parameter_admission_result,
    sequence_parameter_admission,
    wait_for_first_real_clock_sample,
)


class _DeferredClockSampleSource:
    """Represent a launch whose /clock graph/sample arrives after startup."""

    def __init__(self):
        self.wait_started = Event()
        self.sample_available = Event()
        self._on_sample = None

    def wait_for_sample(self, on_sample, timeout_s):
        self._on_sample = on_sample
        self.wait_started.set()
        if self.sample_available.wait(timeout=timeout_s):
            return True
        return False

    def publish(self, clock_ns):
        self._on_sample(clock_ns)
        self.sample_available.set()


class _TimedOutClockSampleSource:
    """Represent a graph that never delivers a real clock sample."""

    def __init__(self):
        self.wait_started = Event()

    def wait_for_sample(self, _on_sample, _timeout_s):
        self.wait_started.set()
        return False


class _FakeMonotonic:
    def __init__(self):
        self.value_ns = 0

    def now_ns(self):
        return self.value_ns

    def advance(self, delta_ns):
        self.value_ns += delta_ns


def _safe_sample():
    return TypedSafetySample(
        mission_gate_inhibited=True,
        active_step=0xFFFFFFFF,
        controller_zero=True,
        odom_stationary=True,
    )


def _unsafe_sample():
    return TypedSafetySample(
        mission_gate_inhibited=False,
        active_step=0,
        controller_zero=False,
        odom_stationary=False,
    )


def test_initial_barrier_requires_two_seconds_of_continuous_typed_safety():
    """One safe MissionState is insufficient and unsafe input resets the hold."""
    monotonic = _FakeMonotonic()
    barrier = InitialSafeStationaryBarrier(
        now_ns=monotonic.now_ns,
        required_hold_ns=2_000_000_000,
    )

    assert barrier.observe(_safe_sample()) is False
    monotonic.advance(1_999_999_999)
    assert barrier.observe(_safe_sample()) is False

    monotonic.advance(1)
    assert barrier.observe(_safe_sample()) is True

    monotonic.advance(1)
    assert barrier.observe(_unsafe_sample()) is False
    assert barrier.observe(_safe_sample()) is False
    monotonic.advance(1_999_999_999)
    assert barrier.observe(_safe_sample()) is False
    monotonic.advance(1)
    assert barrier.observe(_safe_sample()) is True


def test_observer_predicates_use_typed_callbacks_for_motion_gate_odom_and_stop():
    """All canonical safety predicates consume typed ROS callback messages."""
    observer = TypedObservationLedger()

    state = MissionState()
    state.gate_state = MissionState.GATE_INHIBITED
    state.active_step = 0xFFFFFFFF
    observer.on_mission_state(state)

    nonzero = TwistStamped()
    nonzero.twist.linear.x = 0.2
    observer.on_controller_command(nonzero)

    zero = TwistStamped()
    observer.on_controller_command(zero)

    odometry = Odometry()
    observer.on_odometry(odometry)

    stop = VoiceTurn()
    stop.kind = VoiceTurn.STOP
    observer.on_voice_turn(stop)

    assert observer.controller_nonzero_observed is True
    assert observer.latest_controller_zero is True
    assert observer.latest_odom_stationary is True
    assert observer.latest_safety_sample == _safe_sample()
    assert observer.stop_turn_observed is True


def test_stop_watermark_ignores_old_move_nonzero_before_first_post_stop_zero():
    """A queued MOVE sample before the first STOP zero is not recovery."""
    observer = TypedObservationLedger()
    stop_request_ns = 100

    old_move_nonzero = TwistStamped()
    old_move_nonzero.twist.linear.x = 0.2
    observer.on_controller_command(old_move_nonzero, at_ns=110)

    first_zero = TwistStamped()
    observer.on_controller_command(first_zero, at_ns=120)

    watermark = observer.post_stop_controller_watermark(stop_request_ns)

    assert watermark == {
        'stop_request_ns': 100,
        'first_post_stop_zero_ns': 120,
        'post_stop_zero_watermark_nonzero_count': 0,
        'post_stop_zero_watermark_nonzero': False,
    }


def test_stop_watermark_fails_closed_for_nonzero_after_first_post_stop_zero():
    """Any typed nonzero receipt after the zero watermark is a violation."""
    observer = TypedObservationLedger()
    stop_request_ns = 100

    first_zero = TwistStamped()
    observer.on_controller_command(first_zero, at_ns=120)

    resumed_nonzero = TwistStamped()
    resumed_nonzero.twist.linear.x = 0.2
    observer.on_controller_command(resumed_nonzero, at_ns=130)

    watermark = observer.post_stop_controller_watermark(stop_request_ns)

    assert watermark == {
        'stop_request_ns': 100,
        'first_post_stop_zero_ns': 120,
        'post_stop_zero_watermark_nonzero_count': 1,
        'post_stop_zero_watermark_nonzero': True,
    }


def test_stop_finalize_blocks_nonzero_after_zero_before_rotate_admission():
    """A late typed nonzero before rotate admission blocks STOP finalization."""
    observer = TypedObservationLedger()
    finalizer = StopPhaseFinalizer(Condition())

    first_zero = TwistStamped()
    observer.on_controller_command(first_zero, at_ns=120)
    assert observer.post_stop_controller_watermark(
        100,
        stop_phase_end_ns=121,
    )['post_stop_zero_watermark_nonzero'] is False

    late_nonzero = TwistStamped()
    late_nonzero.twist.linear.x = 0.2
    observer.on_controller_command(late_nonzero, at_ns=125)

    result = finalizer.finalize(
        observer,
        stop_request_ns=100,
        stop_phase_end_ns=130,
    )

    assert result['status'] == 'blocked'
    assert result['stop_phase_end_ns'] == 130
    assert result['post_stop_zero_watermark_nonzero_count'] == 1
    assert result['recovery'] is True


def test_stop_finalize_freezes_evidence_before_legal_rotate_motion():
    """Accepted ROTATE motion cannot mutate the frozen STOP evidence."""
    observer = TypedObservationLedger()
    finalizer = StopPhaseFinalizer(Condition())

    first_zero = TwistStamped()
    observer.on_controller_command(first_zero, at_ns=120)
    frozen = finalizer.finalize(
        observer,
        stop_request_ns=100,
        stop_phase_end_ns=130,
    )

    rotate_nonzero = TwistStamped()
    rotate_nonzero.twist.angular.z = 0.4
    observer.on_controller_command(rotate_nonzero, at_ns=140)

    assert frozen['status'] == 'ready'
    assert frozen['post_stop_zero_watermark_nonzero_count'] == 0
    assert frozen['recovery'] is False
    assert finalizer.frozen_snapshot() == frozen
    assert observer.post_stop_controller_watermark(
        100,
        stop_phase_end_ns=150,
    )['post_stop_zero_watermark_nonzero_count'] == 1


def test_parameter_admission_uses_typed_service_result_not_process_exit():
    """A rejected SetParameters response remains a structured rejection."""
    response = SetParametersResult()
    response.successful = False
    response.reason = 'runtime is not at the safe stationary barrier'

    assert parameter_admission_result(response) == {
        'status': 'rejected',
        'successful': False,
        'reason': 'runtime is not at the safe stationary barrier',
    }


def test_ordinary_admission_retries_only_after_updated_typed_safe_generation():
    """Rejected ordinary input has no effect and retries only after new safety."""
    generation = 0
    attempts = []
    wait_generations = []
    side_effects = []

    def attempt(text, _timeout_s):
        attempts.append(text)
        if len(attempts) == 1:
            return {
                'status': 'rejected',
                'successful': False,
                'reason': 'command_text is busy; wait for the safe stationary barrier',
            }
        side_effects.append(text)
        return {'status': 'accepted', 'successful': True, 'reason': ''}

    def wait_for_new_safe_sample(after_generation, _timeout_s):
        nonlocal generation
        wait_generations.append(after_generation)
        assert generation == after_generation
        generation += 1
        return True

    result = sequence_parameter_admission(
        '右转九十度',
        attempt=attempt,
        generation=lambda: generation,
        wait_for_new_safe_sample=wait_for_new_safe_sample,
        timeout_s=1.0,
    )

    assert attempts == ['右转九十度', '右转九十度']
    assert side_effects == ['右转九十度']
    assert wait_generations == [0]
    assert result['successful'] is True
    assert len(result['attempts']) == 2
    assert result['attempts'][0]['successful'] is False
    assert result['attempts'][1]['successful'] is True


def test_stop_admission_is_single_attempt_and_never_enters_ordinary_retry():
    """STOP is accepted/rejected once; it cannot resend or reset Voice state."""
    attempts = []
    wait_calls = []

    def attempt(text, _timeout_s):
        attempts.append(text)
        return {
            'status': 'accepted',
            'successful': True,
            'reason': '',
        }

    result = sequence_parameter_admission(
        '停止',
        attempt=attempt,
        generation=lambda: 4,
        wait_for_new_safe_sample=lambda *_args: wait_calls.append(True),
        timeout_s=1.0,
    )

    assert attempts == ['停止']
    assert wait_calls == []
    assert result['successful'] is True
    assert result['attempts'] == [
        {
            'status': 'accepted',
            'successful': True,
            'reason': '',
        },
    ]


def test_driver_waits_for_first_real_clock_sample_after_graph_startup():
    """A missing initial /clock graph must not fail a single immediate read."""
    source = _DeferredClockSampleSource()
    result = {}

    worker = Thread(
        target=lambda: result.setdefault(
            'readiness',
            wait_for_first_real_clock_sample(
                source.wait_for_sample,
                timeout_s=0.5,
            ),
        ),
    )
    worker.start()

    assert source.wait_started.wait(timeout=0.5)
    assert 'readiness' not in result

    source.publish(42)
    worker.join(timeout=0.5)

    assert result['readiness'] == {
        'status': 'ready',
        'reason': 'first_real_clock_sample',
        'clock_ns': 42,
        'sample_count': 1,
    }


def test_driver_fails_closed_with_structured_timeout_when_clock_never_arrives():
    """A missing real sample is a bounded structured failure, not readiness."""
    source = _TimedOutClockSampleSource()

    readiness = wait_for_first_real_clock_sample(
        source.wait_for_sample,
        timeout_s=0.5,
    )

    assert source.wait_started.is_set()
    assert readiness == {
        'status': 'timeout',
        'reason': 'clock_sample_timeout',
        'clock_ns': None,
        'sample_count': 0,
    }
