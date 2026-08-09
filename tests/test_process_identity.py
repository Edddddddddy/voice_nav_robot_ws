"""Tests for the exact pidfd process-identity Interface."""

from __future__ import annotations

from dataclasses import replace
import os
from pathlib import Path
import signal
import subprocess
import sys
import unittest

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
