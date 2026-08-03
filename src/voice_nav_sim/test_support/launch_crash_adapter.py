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

"""Launch-process adapter for exact VN-0011A crash evidence."""

import signal
import time

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import SignalProcess


class LaunchCrashAdapter:
    """Bind one crash ledger to launch actions and process events."""

    def __init__(self, ledger):
        """Retain the closed exact-action ledger owned by the harness."""
        self._ledger = ledger

    def expect_sigkill(self, action, label):
        """Declare and observe one exact action killed by SIGKILL."""
        self._ledger.expect_sigkill(action, label)
        return self._exit_registration(action)

    def expect_clean(self, action, label):
        """Declare and observe one exact action that must exit zero."""
        self._ledger.expect_clean(action, label)
        return self._exit_registration(action)

    def request_sigkill(self, launch_service, action):
        """Arm and dispatch SIGKILL through the launch event loop."""
        signal_intent_monotonic_ns = time.monotonic_ns()
        self._ledger.arm_sigkill(
            action,
            signal_intent_monotonic_ns=signal_intent_monotonic_ns,
        )
        launch_service.emit_event(
            SignalProcess(
                signal_number=signal.SIGKILL,
                process_matcher=matches_action(action),
            )
        )
        return signal_intent_monotonic_ns

    def exit_observation(self, action):
        """Return one exact ProcessExited observation."""
        return self._ledger.exit_observation(action)

    def assert_complete(self):
        """Return the exhaustive process ledger after every exit."""
        return self._ledger.assert_complete()

    def _exit_registration(self, action):
        def record_exit(event, _context):
            observed_monotonic_ns = time.monotonic_ns()
            self._ledger.record_exit(
                action,
                event.returncode,
                observed_monotonic_ns=observed_monotonic_ns,
            )
            return []

        return RegisterEventHandler(
            OnProcessExit(
                target_action=action,
                on_exit=record_exit,
            )
        )
