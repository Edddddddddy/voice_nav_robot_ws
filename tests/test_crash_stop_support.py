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

"""Behavior tests for crash-stop observation helpers."""

from __future__ import annotations

from collections import deque
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

from launch.events.process import ProcessStarted


def _load_support():
    support_path = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'voice_nav_bringup'
        / 'test'
        / 'crash_stop_support.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_crash_stop_support_unit', support_path
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load crash-stop support')
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


support = _load_support()


def command(stamp_ns: int, linear_x: float):
    stamp = SimpleNamespace(
        sec=stamp_ns // 1_000_000_000,
        nanosec=stamp_ns % 1_000_000_000,
    )
    vector = SimpleNamespace(x=linear_x, y=0.0, z=0.0)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        twist=SimpleNamespace(linear=vector, angular=vector),
    )


def gate_state(
    *,
    state: int = 2,
    authority_live: bool = True,
    candidate_fresh: bool = True,
    motion_inhibited: bool = False,
    zero_selected: bool = False,
):
    return SimpleNamespace(
        state=state,
        authority_live=authority_live,
        candidate_fresh=candidate_fresh,
        motion_inhibited=motion_inhibited,
        zero_selected=zero_selected,
    )


class ConsumerTimeoutAnchorTest(unittest.TestCase):

    def test_probe_isolates_safety_observers_from_high_rate_sensors(self):
        callback_groups = {}

        class FakeNode:

            def create_subscription(
                self,
                _message_type,
                topic,
                _callback,
                _qos,
                *,
                callback_group=None,
            ):
                callback_groups[topic] = callback_group
                return object()

            def create_client(self, *_args, **_kwargs):
                return object()

        class FakeExecutor:

            def __init__(self, *_args, **_kwargs):
                pass

            def add_node(self, _node):
                pass

            def spin(self):
                pass

        class FakeThread:

            def __init__(self, *_args, **_kwargs):
                pass

            def start(self):
                pass

        interfaces = support.WorkspaceInterfaceTypes(
            execute_mission=object(),
            mission_state=object(),
            mission_step=object(),
            gate_state=object(),
            gate_control=object(),
        )
        with (
            mock.patch.object(
                support,
                '_get_workspace_interface_types',
                return_value=interfaces,
            ),
            mock.patch.object(
                support.rclpy,
                'create_node',
                return_value=FakeNode(),
            ),
            mock.patch.object(
                support,
                'Parameter',
                side_effect=lambda *_args, **_kwargs: object(),
            ),
            mock.patch.object(
                support,
                'MultiThreadedExecutor',
                FakeExecutor,
            ),
            mock.patch.object(
                support,
                'ActionClient',
                return_value=object(),
            ),
            mock.patch.object(support.threading, 'Thread', FakeThread),
        ):
            support.CrashStopProbe()

        safety_group = callback_groups[support.GATE_STATE_TOPIC]
        self.assertIsNotNone(safety_group)
        self.assertIs(
            safety_group,
            callback_groups[support.FINAL_COMMAND_TOPIC],
        )
        self.assertIs(
            safety_group,
            callback_groups[support.LIMITED_COMMAND_TOPIC],
        )
        self.assertIsNot(
            safety_group,
            callback_groups[support.CLOCK_TOPIC],
        )

    def test_pure_helpers_load_before_workspace_interfaces_are_built(self):
        original_import = __import__

        def reject_workspace_interfaces(name, *args, **kwargs):
            if name == 'voice_nav_interfaces' or name.startswith(
                'voice_nav_interfaces.'
            ) or name == 'voice_nav_mission' or name.startswith(
                'voice_nav_mission.'
            ):
                raise ModuleNotFoundError(name)
            return original_import(name, *args, **kwargs)

        with mock.patch(
            'builtins.__import__', side_effect=reject_workspace_interfaces
        ):
            isolated_support = _load_support()

        _, message = isolated_support.select_consumer_timeout_anchor(
            ((10, command(3_994_000_000, 0.01)),),
            signal_boundary_sim_ns=3_994_000_000,
        )
        self.assertEqual(isolated_support._stamp_ns(message), 3_994_000_000)

        with mock.patch(
            'builtins.__import__', side_effect=reject_workspace_interfaces
        ):
            with self.assertRaises(ModuleNotFoundError):
                isolated_support._load_workspace_interface_types()

    def test_workspace_interface_api_is_loaded_lazily_and_cached(self):
        isolated_support = _load_support()
        execute_mission = object()
        interfaces = isolated_support.WorkspaceInterfaceTypes(
            execute_mission=execute_mission,
            mission_state=object(),
            mission_step=object(),
            gate_state=object(),
            gate_control=object(),
        )

        with mock.patch.object(
            isolated_support,
            '_load_workspace_interface_types',
            return_value=interfaces,
        ) as loader:
            self.assertIs(isolated_support.ExecuteMission, execute_mission)
            self.assertIs(isolated_support.ExecuteMission, execute_mission)

        loader.assert_called_once_with()

    def test_gazebo_shutdown_api_is_resolved_only_on_first_access(self):
        isolated_support = _load_support()
        structured_stop = object()
        loaded = SimpleNamespace(structured_stop_gazebo=structured_stop)

        with mock.patch.object(
            isolated_support,
            '_load_gazebo_shutdown',
            return_value=loaded,
        ) as loader:
            self.assertIs(
                isolated_support.gazebo_shutdown.structured_stop_gazebo,
                structured_stop,
            )
            self.assertIs(
                isolated_support.gazebo_shutdown.structured_stop_gazebo,
                structured_stop,
            )

        loader.assert_called_once_with()

    def test_selects_latest_nonzero_final_even_when_observer_delivers_it_late(self):
        samples = (
            (10, command(3_994_000_000, 0.01)),
            (20, command(4_098_000_000, 0.02)),
            (31, command(4_120_000_000, 0.03)),
        )

        receipt_ns, message = support.select_consumer_timeout_anchor(
            samples,
            signal_boundary_sim_ns=3_994_000_000,
        )

        self.assertEqual(receipt_ns, 31)
        self.assertEqual(support._stamp_ns(message), 4_120_000_000)

    def test_recomputes_frozen_timeout_from_actual_last_final(self):
        result = support.consumer_timeout_result(
            (
                (10, command(3_994_000_000, 0.01)),
                (20, command(4_098_000_000, 0.02)),
            ),
            signal_boundary_sim_ns=3_994_000_000,
            zero_sim_ns=4_449_000_000,
            zero_receipt_ns=30,
        )

        self.assertEqual(result['last_nonzero_sim_ns'], 4_098_000_000)
        self.assertEqual(result['last_nonzero_receipt_ns'], 20)
        self.assertEqual(result['delta_ns'], 351_000_000)

    def test_late_final_that_breaks_timeout_is_not_ignored(self):
        with self.assertRaisesRegex(AssertionError, '0.329000 s'):
            support.consumer_timeout_result(
                (
                    (10, command(3_994_000_000, 0.01)),
                    (31, command(4_120_000_000, 0.03)),
                ),
                signal_boundary_sim_ns=3_994_000_000,
                zero_sim_ns=4_449_000_000,
                zero_receipt_ns=30,
            )

    def test_quiescence_fence_observes_final_added_after_first_snapshot(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe.observation_changed = threading.Condition(probe.lock)
        probe.final_commands = deque(
            ((10, command(3_994_000_000, 0.01)),),
            maxlen=20,
        )
        probe.clock_samples = deque(((0, 4_500_000_000),), maxlen=20)
        probe.publisher_count = mock.Mock(
            side_effect=AssertionError('stale graph endpoint is not liveness')
        )

        class FakeTime:

            def __init__(self):
                self.now_ns = 0
                self.sleep_count = 0

            def monotonic(self):
                return self.now_ns / 1_000_000_000

            def monotonic_ns(self):
                return self.now_ns

            def sleep(self, seconds):
                self.now_ns += 100_000_000
                with probe.lock:
                    if self.sleep_count == 0:
                        probe.final_commands.append(
                            (50, command(4_120_000_000, 0.03))
                        )
                    probe.clock_samples.append(
                        (self.now_ns, 4_500_000_000 + self.now_ns)
                    )
                self.sleep_count += 1

        with mock.patch.object(support, 'time', FakeTime()):
            with self.assertRaisesRegex(AssertionError, '0.329000 s'):
                probe.wait_confirm_consumer_timeout(
                    3_994_000_000,
                    {
                        'zero_sim_ns': 4_449_000_000,
                        'zero_receipt_ns': 30,
                    },
                    timeout=1.0,
                )
        probe.publisher_count.assert_not_called()

    def test_rejects_missing_eligible_nonzero_final(self):
        samples = (
            (10, command(3_994_000_000, 0.0)),
            (31, command(3_900_000_000, 0.02)),
        )

        with self.assertRaisesRegex(
            AssertionError,
            'no eligible non-zero final command',
        ):
            support.select_consumer_timeout_anchor(
                samples,
                signal_boundary_sim_ns=3_994_000_000,
            )


class SignalBoundaryMotionProofTest(unittest.TestCase):

    def make_probe(self):
        probe = support.CrashStopProbe.__new__(support.CrashStopProbe)
        probe.lock = threading.Lock()
        probe.observation_changed = threading.Condition(probe.lock)
        probe._gate_state_type = SimpleNamespace(
            ARMED=2,
            INHIBITED=0,
            FAULTED=3,
        )
        return probe

    def fresh_motion_probe(self, **gate_fields):
        """Build a current proof whose only invalid axis is Gate state."""
        probe = self.make_probe()
        probe.gate_states = deque(
            ((1_000, gate_state(**gate_fields)),), maxlen=20
        )
        probe.final_commands = deque(
            ((1_000, command(1_000, 0.1)),), maxlen=20
        )
        probe.limited_commands = deque(
            ((1_000, command(1_000, 0.1)),), maxlen=20
        )
        probe.clock_samples = deque(
            ((900, 900), (1_000, 1_000)), maxlen=20
        )
        return probe

    def assert_fresh_invalid_gate_times_out(self, **gate_fields):
        probe = self.fresh_motion_probe(**gate_fields)
        active_goal = SimpleNamespace(status=support.GoalStatus.STATUS_EXECUTING)

        class FakeTime:

            def __init__(self):
                self.now_ns = 1_000

            def monotonic(self):
                return self.now_ns / 1_000_000_000

            def monotonic_ns(self):
                return self.now_ns

        fake_time = FakeTime()

        class FakeCondition:

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def wait(self, timeout=None):
                fake_time.now_ns += int(timeout * 1_000_000_000)

        probe.observation_changed = FakeCondition()
        with mock.patch.object(support, 'time', fake_time):
            with self.assertRaisesRegex(
                AssertionError,
                'timed out waiting for current armed, moving proof',
            ):
                probe.capture_motion_at_signal_boundary(
                    goal_handle=active_goal,
                    timeout=0.1,
                )

    def test_current_fresh_nonzero_proof_rejects_each_invalid_gate_axis(self):
        cases = (
            (
                'faulted',
                {'state': self.make_probe()._gate_state_type.FAULTED},
            ),
            ('motion_inhibited', {'motion_inhibited': True}),
            ('authority_dead', {'authority_live': False}),
            ('candidate_stale', {'candidate_fresh': False}),
            ('zero_selected', {'zero_selected': True}),
        )

        for name, gate_fields in cases:
            with self.subTest(name=name):
                self.assert_fresh_invalid_gate_times_out(**gate_fields)

    @unittest.skipUnless(
        hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'),
        'pidfd is a Linux test requirement',
    )
    def test_faulted_gate_boundary_callback_does_not_pidfd_kill_child(self):
        probe = self.fresh_motion_probe(
            state=self.make_probe()._gate_state_type.FAULTED
        )
        active_goal = SimpleNamespace(status=support.GoalStatus.STATUS_EXECUTING)
        command_line = [
            sys.executable,
            '-c',
            'import time; time.sleep(60)',
            '--ros-args',
            '-r',
            '__node:=signal_boundary_faulted_child',
        ]
        action = object()
        child = subprocess.Popen(command_line)
        event = ProcessStarted(
            action=action,
            name='signal_boundary_faulted_child',
            cmd=command_line,
            cwd=os.getcwd(),
            env=dict(os.environ),
            pid=child.pid,
        )
        process = None
        try:
            process = support.ExactPidfdProcess.from_process_started(
                action=action,
                event=event,
                expected_executable=Path(sys.executable).name,
                expected_node_name='signal_boundary_faulted_child',
            )
            with self.assertRaisesRegex(
                AssertionError,
                'timed out waiting for current armed, moving proof',
            ):
                process.kill(
                    lambda: 1,
                    before_signal=lambda: (
                        probe.capture_motion_at_signal_boundary(
                            goal_handle=active_goal,
                            timeout=0.01,
                        )
                    ),
                )
            self.assertFalse(process.sigkill_sent)
            self.assertIsNone(child.poll())
        finally:
            if process is not None:
                process.close()
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=2)

    def test_boundary_waits_for_fresh_motion_after_transient_zero(self):
        probe = self.make_probe()
        active_goal = SimpleNamespace(status=support.GoalStatus.STATUS_EXECUTING)
        probe.gate_states = deque(((900, gate_state()),), maxlen=20)
        probe.final_commands = deque(
            ((900, command(1_000, 0.0)),), maxlen=20
        )
        probe.limited_commands = deque(
            ((900, command(1_000, 0.0)),), maxlen=20
        )
        probe.clock_samples = deque(
            ((900, 900), (901, 1_000)), maxlen=20
        )

        class FakeTime:

            def __init__(self):
                self.now_ns = 1_000
                self.slept = False

            def monotonic(self):
                return self.now_ns / 1_000_000_000

            def monotonic_ns(self):
                return self.now_ns

        fake_time = FakeTime()

        class FakeCondition:

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def wait(self, timeout=None):
                fake_time.now_ns += 10
                if fake_time.slept:
                    return
                probe.gate_states.append(
                    (fake_time.now_ns, gate_state())
                )
                probe.final_commands.append(
                    (fake_time.now_ns, command(1_010, 0.1))
                )
                probe.limited_commands.append(
                    (fake_time.now_ns, command(1_010, 0.1))
                )
                probe.clock_samples.append((fake_time.now_ns, 1_010))
                fake_time.slept = True

        probe.observation_changed = FakeCondition()
        with mock.patch.object(support, 'time', fake_time):
            proof = probe.capture_motion_at_signal_boundary(
                goal_handle=active_goal,
                timeout=0.1,
            )

        self.assertEqual(proof['observed_ns'], 1_010)
        self.assertFalse(support.is_zero(proof['final'][1]))
        self.assertFalse(support.is_zero(proof['limited'][1]))

    def test_expired_motion_proof_fails_closed_before_signal(self):
        probe = self.make_probe()
        active_goal = SimpleNamespace(status=support.GoalStatus.STATUS_EXECUTING)
        probe.gate_states = deque(((0, gate_state()),), maxlen=20)
        probe.final_commands = deque(
            ((0, command(1_000, 0.1)),), maxlen=20
        )
        probe.limited_commands = deque(
            ((0, command(1_000, 0.1)),), maxlen=20
        )
        probe.clock_samples = deque(((0, 900), (0, 1_000)), maxlen=20)

        class FakeTime:

            def __init__(self):
                self.now_ns = support.DEPENDENCY_FRESHNESS_NS + 1

            def monotonic(self):
                return self.now_ns / 1_000_000_000

            def monotonic_ns(self):
                return self.now_ns

        fake_time = FakeTime()

        class FakeCondition:

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def wait(self, timeout=None):
                fake_time.now_ns += int(timeout * 1_000_000_000)

        probe.observation_changed = FakeCondition()
        with mock.patch.object(support, 'time', fake_time):
            with self.assertRaisesRegex(
                AssertionError,
                'timed out waiting for current armed, moving proof',
            ):
                probe.capture_motion_at_signal_boundary(
                    goal_handle=active_goal,
                    timeout=0.1,
                )

    def test_ended_goal_cannot_reuse_current_motion_proof(self):
        probe = self.make_probe()
        probe.gate_states = deque(((1_000, gate_state()),), maxlen=20)
        probe.final_commands = deque(
            ((1_000, command(1_000, 0.1)),), maxlen=20
        )
        probe.limited_commands = deque(
            ((1_000, command(1_000, 0.1)),), maxlen=20
        )
        probe.clock_samples = deque(
            ((900, 900), (1_000, 1_000)), maxlen=20
        )
        active_goal = SimpleNamespace(
            status=support.GoalStatus.STATUS_SUCCEEDED
        )

        class FakeTime:

            def __init__(self):
                self.now_ns = 1_000

            def monotonic(self):
                return self.now_ns / 1_000_000_000

            def monotonic_ns(self):
                return self.now_ns

        fake_time = FakeTime()

        class FakeCondition:

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def wait(self, timeout=None):
                fake_time.now_ns += int(timeout * 1_000_000_000)

        probe.observation_changed = FakeCondition()
        with mock.patch.object(support, 'time', fake_time):
            with self.assertRaisesRegex(
                AssertionError,
                'timed out waiting for current armed, moving proof',
            ):
                probe.capture_motion_at_signal_boundary(
                    goal_handle=active_goal,
                    timeout=0.1,
                )


if __name__ == '__main__':
    unittest.main()
