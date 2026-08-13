# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Behavioral contract for ROS-domain Mapping/Navigation exclusion."""

import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest


MODE_LOCK_MODULE = Path(__file__).parents[1] / 'launch_support' / 'mode_lock.py'


def load_mode_lock_module():
    specification = importlib.util.spec_from_file_location(
        'voice_nav_mode_lock', MODE_LOCK_MODULE
    )
    if specification is None or specification.loader is None:
        raise AssertionError('could not load Mapping mode-lock support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


@unittest.skipUnless(os.name == 'posix', 'requires POSIX flock support')
class MappingModeLockTest(unittest.TestCase):
    def test_same_domain_is_exclusive_until_the_owner_releases(self):
        mode_lock = load_mode_lock_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            runtime_directory = Path(temporary_directory)
            runtime_directory.chmod(0o700)
            environment = {
                'ROS_DOMAIN_ID': '17',
                'XDG_RUNTIME_DIR': str(runtime_directory),
            }
            owner = mode_lock.acquire_mode_lock(
                mode='mapping', environment=environment
            )
            try:
                with self.assertRaises(mode_lock.ModeLockConflict):
                    mode_lock.acquire_mode_lock(
                        mode='navigation', environment=environment
                    )
            finally:
                owner.close()

            successor = mode_lock.acquire_mode_lock(
                mode='mapping', environment=environment
            )
            successor.close()

    def test_shutdown_keeps_lock_until_process_and_tf_owner_are_gone(self):
        mode_lock = load_mode_lock_module()

        class Owner:
            closed = 0

            def close(self):
                self.closed += 1

        owner = Owner()
        gate = mode_lock.ModeLockShutdownGate(owner)
        gate.request_shutdown()
        self.assertEqual(owner.closed, 0)
        gate.observe_slam_process_exit()
        self.assertEqual(owner.closed, 0)
        gate.observe_tf_owner_disappearance()
        self.assertEqual(owner.closed, 1)

    def test_unsafe_xdg_runtime_directory_uses_uid_scoped_fallback(self):
        mode_lock = load_mode_lock_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            fixture = Path(temporary_directory)
            invalid_xdg = fixture / 'xdg'
            invalid_xdg.mkdir(mode=0o755)
            invalid_xdg.chmod(0o755)
            fallback_parent = fixture / 'fallback'
            fallback_parent.mkdir(mode=0o700)

            owner = mode_lock.acquire_mode_lock(
                mode='mapping',
                environment={
                    'ROS_DOMAIN_ID': '21',
                    'XDG_RUNTIME_DIR': str(invalid_xdg),
                },
                fallback_parent=fallback_parent,
            )
            try:
                self.assertEqual(
                    owner.path.parent,
                    fallback_parent / f'voice_nav-{os.geteuid()}',
                )
            finally:
                owner.close()

    def test_stale_lock_is_reused_and_records_current_owner_diagnostics(self):
        mode_lock = load_mode_lock_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            runtime_directory = Path(temporary_directory)
            runtime_directory.chmod(0o700)
            lock_root = runtime_directory / 'voice_nav'
            lock_root.mkdir(mode=0o700)
            lock_path = lock_root / 'mode-ros-domain-20.lock'
            lock_path.write_text('stale\n', encoding='utf-8')
            lock_path.chmod(0o600)

            owner = mode_lock.acquire_mode_lock(
                mode='navigation',
                environment={
                    'ROS_DOMAIN_ID': '20',
                    'XDG_RUNTIME_DIR': str(runtime_directory),
                },
            )
            try:
                diagnostics = json.loads(lock_path.read_text(encoding='utf-8'))
                self.assertEqual(diagnostics['domain'], 20)
                self.assertEqual(diagnostics['mode'], 'navigation')
                self.assertEqual(diagnostics['pid'], os.getpid())
                self.assertGreater(diagnostics['starttime'], 0)
                self.assertRegex(diagnostics['launch_nonce'], r'^[0-9a-f]{32}$')
            finally:
                owner.close()

    def test_non_private_existing_lock_file_fails_closed(self):
        mode_lock = load_mode_lock_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            runtime_directory = Path(temporary_directory)
            runtime_directory.chmod(0o700)
            lock_root = runtime_directory / 'voice_nav'
            lock_root.mkdir(mode=0o700)
            lock_path = lock_root / 'mode-ros-domain-24.lock'
            lock_path.write_text('untrusted\n', encoding='utf-8')
            lock_path.chmod(0o644)

            with self.assertRaises(mode_lock.ModeLockError):
                mode_lock.acquire_mode_lock(
                    mode='mapping',
                    environment={
                        'ROS_DOMAIN_ID': '24',
                        'XDG_RUNTIME_DIR': str(runtime_directory),
                    },
                )

    def test_symlink_lock_path_fails_as_a_mode_lock_error(self):
        mode_lock = load_mode_lock_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            runtime_directory = Path(temporary_directory)
            runtime_directory.chmod(0o700)
            lock_root = runtime_directory / 'voice_nav'
            lock_root.mkdir(mode=0o700)
            target = runtime_directory / 'target'
            target.write_text('untrusted\n', encoding='utf-8')
            (lock_root / 'mode-ros-domain-25.lock').symlink_to(target)

            with self.assertRaises(mode_lock.ModeLockError):
                mode_lock.acquire_mode_lock(
                    mode='mapping',
                    environment={
                        'ROS_DOMAIN_ID': '25',
                        'XDG_RUNTIME_DIR': str(runtime_directory),
                    },
                )

    def test_context_exit_releases_the_different_domain_lock(self):
        mode_lock = load_mode_lock_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            runtime_directory = Path(temporary_directory)
            runtime_directory.chmod(0o700)
            environment = {
                'ROS_DOMAIN_ID': '29',
                'XDG_RUNTIME_DIR': str(runtime_directory),
            }
            with mode_lock.acquire_mode_lock(
                mode='navigation', environment=environment
            ):
                with self.assertRaises(mode_lock.ModeLockConflict):
                    mode_lock.acquire_mode_lock(
                        mode='mapping', environment=environment
                    )
            owner = mode_lock.acquire_mode_lock(
                mode='mapping', environment=environment
            )
            owner.close()


if __name__ == '__main__':
    unittest.main()
