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

"""Tests for the exact pidfd process-identity Interface."""

from __future__ import annotations

from collections import deque
from dataclasses import replace
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
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
        / 'process_identity.py'
    )
    import importlib.util

    specification = importlib.util.spec_from_file_location(
        'voice_nav_process_identity', support_path
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load process identity support')
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


support = _load_support()


def _load_crash_stop_support():
    support_path = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'voice_nav_bringup'
        / 'test'
        / 'crash_stop_support.py'
    )
    import importlib.util

    specification = importlib.util.spec_from_file_location(
        'voice_nav_crash_stop_support_pidfd_unit', support_path
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load crash-stop support')
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


crash_stop_support = _load_crash_stop_support()


@unittest.skipUnless(
    hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'),
    'pidfd is a Linux test requirement',
)
class ProcessIdentityTest(unittest.TestCase):
    def make_child(self):
        action = object()
        command = [
            sys.executable,
            '-c',
            'import time; time.sleep(60)',
            '--ros-args',
            '-r',
            '__node:=pidfd_test_child',
        ]
        child = subprocess.Popen(command)
        event = ProcessStarted(
            action=action,
            name='pidfd_test_child',
            cmd=command,
            cwd=os.getcwd(),
            env=dict(os.environ),
            pid=child.pid,
        )
        return child, action, event

    @staticmethod
    def _command(stamp_ns, linear_x):
        stamp = SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        )
        vector = SimpleNamespace(x=linear_x, y=0.0, z=0.0)
        return SimpleNamespace(
            header=SimpleNamespace(stamp=stamp),
            twist=SimpleNamespace(linear=vector, angular=vector),
        )

    def _signal_boundary_probe(self, endpoint_gids):
        probe = crash_stop_support.CrashStopProbe.__new__(
            crash_stop_support.CrashStopProbe
        )
        probe.lock = threading.Lock()
        probe._gate_state_type = SimpleNamespace(ARMED=1)
        probe._limited_endpoint_snapshot_provider = lambda: ('gid-controller',)
        probe.node = SimpleNamespace(
            get_publishers_info_by_topic=lambda _topic: tuple(
                SimpleNamespace(
                    node_name=crash_stop_support.GATE_NODE,
                    node_namespace='/',
                    endpoint_gid=endpoint_gid,
                )
                for endpoint_gid in endpoint_gids
            )
        )
        probe.gate_states = deque(
            ((900, SimpleNamespace(
                state=1,
                authority_live=True,
                candidate_fresh=True,
                motion_inhibited=False,
                zero_selected=False,
            )),),
            maxlen=20,
        )
        probe.final_commands = deque(
            (crash_stop_support.CommandObservation(
                receipt_ns=901,
                message=self._command(4_000_000_000, 0.01),
                publication_sequence_number=8,
                source_timestamp_ns=1_000,
                received_timestamp_ns=2_000,
                final_subscription_identity='signal-boundary-subscription',
            ),),
            maxlen=20,
        )
        probe.limited_commands = deque(
            ((902, self._command(4_001_000_000, 0.01)),), maxlen=20
        )
        probe.clock_samples = deque(
            ((800, 4_000_000_000), (903, 4_001_000_000)), maxlen=20
        )
        return probe

    def test_event_absolute_executable_is_used_when_node_is_not_on_path(self):
        temporary_directory = tempfile.TemporaryDirectory()
        executable = Path(temporary_directory.name) / 'voice_nav_unlisted_node'
        executable.symlink_to(sys.executable)
        action = object()
        command = [
            str(executable),
            '-c',
            'import time; time.sleep(60)',
            '--ros-args',
            '-r',
            '__node:=pidfd_test_child',
        ]
        child = subprocess.Popen(command)
        event = ProcessStarted(
            action=action,
            name='pidfd_test_child',
            cmd=command,
            cwd=os.getcwd(),
            env=dict(os.environ),
            pid=child.pid,
        )
        guard = None
        try:
            guard = support.ExactPidfdProcess.from_process_started(
                action=action,
                event=event,
                expected_executable=executable.name,
                expected_node_name='pidfd_test_child',
            )
            acknowledged = guard.kill(lambda: 1)
            self.assertGreater(acknowledged, 0)
            self.assertNotEqual(child.wait(timeout=2), 0)
        finally:
            if guard is not None:
                guard.close()
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=2)
            temporary_directory.cleanup()

    def test_resolved_wrong_elf_is_rejected_before_signal(self):
        child, action, event = self.make_child()
        temporary_directory = tempfile.TemporaryDirectory()
        executable = Path(temporary_directory.name) / 'voice_nav_wrong_node'
        executable.symlink_to('/bin/sh')
        command = [str(executable), *event.cmd[1:]]
        wrong_event = ProcessStarted(
            action=action,
            name=event.name,
            cmd=command,
            cwd=event.cwd,
            env=event.env,
            pid=event.pid,
        )
        try:
            with self.assertRaises(support.ProcessIdentityError):
                support.ExactPidfdProcess.from_process_started(
                    action=action,
                    event=wrong_event,
                    expected_executable=executable.name,
                    expected_node_name='pidfd_test_child',
                )
        finally:
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=2)
            temporary_directory.cleanup()

    def test_duplicate_executable_candidates_are_rejected(self):
        child, action, event = self.make_child()
        duplicate_command = [*event.cmd, event.cmd[0]]
        duplicate_event = ProcessStarted(
            action=action,
            name=event.name,
            cmd=duplicate_command,
            cwd=event.cwd,
            env=event.env,
            pid=event.pid,
        )
        try:
            with self.assertRaises(support.ProcessIdentityError):
                support.ExactPidfdProcess.from_process_started(
                    action=action,
                    event=duplicate_event,
                    expected_executable=Path(sys.executable).name,
                    expected_node_name='pidfd_test_child',
                )
        finally:
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=2)

    def test_wrong_node_name_is_rejected_before_signal(self):
        child, action, event = self.make_child()
        try:
            with self.assertRaises(support.ProcessIdentityError):
                support.ExactPidfdProcess.from_process_started(
                    action=action,
                    event=event,
                    expected_executable=Path(sys.executable).name,
                    expected_node_name='wrong_node_name',
                )
        finally:
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=2)

    def test_exact_child_can_be_killed_and_reaped_through_pidfd(self):
        child, action, event = self.make_child()
        guard = None
        try:
            guard = support.ExactPidfdProcess.from_process_started(
                action=action,
                event=event,
                expected_executable=Path(sys.executable).name,
                expected_node_name='pidfd_test_child',
            )
            acknowledged = guard.kill(lambda: 1)
            self.assertGreater(acknowledged, 0)
            self.assertNotEqual(child.wait(timeout=2), 0)
            with self.assertRaises(support.ProcessIdentityError):
                guard.validate(lambda: 1)
        finally:
            if guard is not None:
                guard.close()
            if child.poll() is None:
                child.wait(timeout=2)

    def test_signal_boundary_proof_and_receipt_fence_run_before_sigkill(self):
        child, action, event = self.make_child()
        guard = None
        calls = []
        original_pidfd_send_signal = signal.pidfd_send_signal

        def traced_pidfd_send_signal(pidfd, sig):
            if sig == signal.SIGKILL:
                calls.append('sigkill')
            return original_pidfd_send_signal(pidfd, sig)

        try:
            guard = support.ExactPidfdProcess.from_process_started(
                action=action,
                event=event,
                expected_executable=Path(sys.executable).name,
                expected_node_name='pidfd_test_child',
            )
            with mock.patch.object(
                support.signal,
                'pidfd_send_signal',
                side_effect=traced_pidfd_send_signal,
            ):
                def capture_signal_boundary_evidence():
                    calls.append('motion-proof')
                    calls.append('endpoint-receipt-fence')

                guard.kill(
                    lambda: calls.append('graph') or 1,
                    before_signal=capture_signal_boundary_evidence,
                )
            self.assertEqual(
                calls,
                [
                    'graph',
                    'motion-proof',
                    'endpoint-receipt-fence',
                    'sigkill',
                ],
            )
            self.assertTrue(guard.sigkill_sent)
            self.assertNotEqual(child.wait(timeout=2), 0)
        finally:
            if guard is not None:
                guard.close()
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=2)

    def test_failed_signal_boundary_precondition_does_not_kill(self):
        child, action, event = self.make_child()
        guard = None
        calls = []
        try:
            guard = support.ExactPidfdProcess.from_process_started(
                action=action,
                event=event,
                expected_executable=Path(sys.executable).name,
                expected_node_name='pidfd_test_child',
            )

            def count_unique_graph_owner():
                calls.append('graph')
                return 1

            def reject_signal_boundary():
                calls.append('motion')
                raise AssertionError('target stopped moving before SIGKILL')

            with self.assertRaisesRegex(
                AssertionError,
                'target stopped moving before SIGKILL',
            ):
                guard.kill(
                    count_unique_graph_owner,
                    before_signal=reject_signal_boundary,
                )
            self.assertEqual(calls, ['graph', 'motion'])
            self.assertFalse(guard.sigkill_sent)
            self.assertIsNone(child.poll())
        finally:
            if guard is not None:
                guard.close()
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=2)

    def test_invalid_final_endpoint_gid_does_not_kill_before_signal(self):
        cases = (
            ('missing', (None,), 'final-command endpoint GID is unavailable'),
            ('empty', (b'',), 'final-command endpoint GID is unavailable'),
            ('all-zero', (b'\x00' * 16,), 'final-command endpoint GID is invalid'),
            ('duplicate', (b'\x01', b'\x02'), 'unique final-command publisher'),
        )
        for case, endpoint_gids, reason in cases:
            with self.subTest(case=case):
                child, action, event = self.make_child()
                guard = None
                try:
                    guard = support.ExactPidfdProcess.from_process_started(
                        action=action,
                        event=event,
                        expected_executable=Path(sys.executable).name,
                        expected_node_name='pidfd_test_child',
                    )
                    probe = self._signal_boundary_probe(endpoint_gids)
                    with (
                        mock.patch.object(
                            crash_stop_support.time,
                            'monotonic_ns',
                            return_value=1_000,
                        ),
                        self.assertRaisesRegex(AssertionError, reason),
                    ):
                        guard.kill(
                            lambda: 1,
                            before_signal=(
                                probe.capture_motion_at_signal_boundary
                            ),
                        )
                    self.assertFalse(guard.sigkill_sent)
                    self.assertIsNone(child.poll())
                finally:
                    if guard is not None:
                        guard.close()
                    if child.poll() is None:
                        child.terminate()
                        child.wait(timeout=2)

    def test_identity_change_after_motion_boundary_does_not_kill(self):
        child, action, event = self.make_child()
        guard = None
        callbacks = []
        try:
            guard = support.ExactPidfdProcess.from_process_started(
                action=action,
                event=event,
                expected_executable=Path(sys.executable).name,
                expected_node_name='pidfd_test_child',
            )

            def count_unique_graph_owner():
                callbacks.append('graph')
                return 1

            def invalidate_recorded_identity_after_motion_proof():
                callbacks.append('motion')
                guard.snapshot = replace(
                    guard.snapshot,
                    starttime_ticks=guard.snapshot.starttime_ticks + 1,
                )
            try:
                guard.kill(
                    count_unique_graph_owner,
                    before_signal=(
                        invalidate_recorded_identity_after_motion_proof
                    ),
                )
            except support.ProcessIdentityError:
                pass
            else:
                self.fail(
                    'recorded identity change after motion proof sent '
                    f'SIGKILL; child exit={child.wait(timeout=2)}'
                )
            self.assertEqual(callbacks, ['graph', 'motion'])
            self.assertIsNone(child.poll())
            self.assertFalse(guard.sigkill_sent)
        finally:
            if guard is not None:
                if child.poll() is None:
                    guard.snapshot = support.read_process_snapshot(child.pid)
                guard.close()
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=2)

    def test_wrong_executable_is_rejected_before_signal(self):
        child, action, event = self.make_child()
        guard = None
        try:
            with self.assertRaises(support.ProcessIdentityError):
                support.ExactPidfdProcess.from_process_started(
                    action=action,
                    event=event,
                    expected_executable='not-the-child',
                    expected_node_name='pidfd_test_child',
                )
        finally:
            guard = support.ExactPidfdProcess.from_process_started(
                action=action,
                event=event,
                expected_executable=Path(sys.executable).name,
                expected_node_name='pidfd_test_child',
            )
            guard.kill(lambda: 1)
            guard.close()
            child.wait(timeout=2)

    def test_changed_starttime_is_rejected(self):
        child, action, event = self.make_child()
        guard = None
        try:
            guard = support.ExactPidfdProcess.from_process_started(
                action=action,
                event=event,
                expected_executable=Path(sys.executable).name,
                expected_node_name='pidfd_test_child',
            )
            guard.snapshot = replace(
                guard.snapshot,
                starttime_ticks=guard.snapshot.starttime_ticks + 1,
            )
            with self.assertRaises(support.ProcessIdentityError):
                guard.validate(lambda: 1)
        finally:
            if guard is not None:
                guard.snapshot = support.read_process_snapshot(child.pid)
                guard.kill(lambda: 1)
                guard.close()
            child.wait(timeout=2)


if __name__ == '__main__':
    unittest.main()
