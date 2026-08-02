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

"""Pure VN-0011A crash-evidence primitives shared by launch tests."""

from __future__ import annotations

import signal
import time
from typing import Any


class CrashEvidenceError(AssertionError):
    """The observed process-exit evidence violates its closed contract."""


class CrashLedger:
    """Track exact launch actions and their one allowed terminal exit."""

    def __init__(self) -> None:
        self._entries: list[_ExpectedExit] = []
        self._recording_started = False

    def expect_sigkill(self, action: Any, label: str) -> None:
        """Declare the one exact action intentionally killed by SIGKILL."""
        self._declare(action, label, -signal.SIGKILL)

    def expect_clean(self, action: Any, label: str) -> None:
        """Declare a launch-managed action that must exit successfully."""
        self._declare(action, label, 0)

    def arm_sigkill(
        self,
        action: Any,
        *,
        signal_intent_monotonic_ns: int,
    ) -> None:
        """Arm one declared kill without treating signal intent as death."""
        self._recording_started = True
        entry = self._entry_for_exact_action(action)
        if entry is None:
            raise CrashEvidenceError(
                'cannot arm an unknown exact action'
            )
        if entry.expected_returncode != -signal.SIGKILL:
            raise CrashEvidenceError(
                f'cannot arm clean action: {entry.label}'
            )
        if entry.signal_intent_monotonic_ns is not None:
            raise CrashEvidenceError(
                f'SIGKILL action already armed: {entry.label}'
            )
        entry.signal_intent_monotonic_ns = self._valid_monotonic_ns(
            signal_intent_monotonic_ns,
            context='signal intent',
        )

    def record_exit(
        self,
        action: Any,
        returncode: int,
        *,
        observed_monotonic_ns: int | None = None,
    ) -> None:
        """Record one observed exit without accepting equality aliases."""
        self._recording_started = True
        entry = self._entry_for_exact_action(action)
        if entry is None:
            raise CrashEvidenceError(
                'exit belongs to an unknown exact action'
            )
        if entry.observed_returncode is not None:
            raise CrashEvidenceError(
                f'duplicate exit for {entry.label}'
            )
        if isinstance(returncode, bool) or not isinstance(returncode, int):
            raise CrashEvidenceError(
                f'{entry.label} exit code must be an integer'
            )
        if (
            entry.expected_returncode == -signal.SIGKILL
            and entry.signal_intent_monotonic_ns is None
        ):
            raise CrashEvidenceError(
                f'SIGKILL action was not armed: {entry.label}'
            )
        if returncode != entry.expected_returncode:
            raise CrashEvidenceError(
                f'{entry.label} expected {entry.expected_returncode}, '
                f'observed {returncode}'
            )
        event_time_ns = self._valid_monotonic_ns(
            time.monotonic_ns()
            if observed_monotonic_ns is None
            else observed_monotonic_ns,
            context='exit observation',
        )
        if (
            entry.signal_intent_monotonic_ns is not None
            and event_time_ns < entry.signal_intent_monotonic_ns
        ):
            raise CrashEvidenceError(
                f'{entry.label} exit observation precedes signal intent'
            )
        entry.observed_returncode = returncode
        entry.observed_monotonic_ns = event_time_ns

    def exit_observation(
        self,
        action: Any,
    ) -> tuple[str, int, int]:
        """Return one exact action's observed event, never signal time."""
        entry = self._entry_for_exact_action(action)
        if entry is None:
            raise CrashEvidenceError(
                'observation belongs to an unknown exact action'
            )
        if (
            entry.observed_returncode is None
            or entry.observed_monotonic_ns is None
        ):
            raise CrashEvidenceError(
                f'exit has not been observed: {entry.label}'
            )
        return (
            entry.label,
            entry.observed_returncode,
            entry.observed_monotonic_ns,
        )

    def assert_complete(self) -> tuple[tuple[str, int], ...]:
        """Return the closed ledger only when every action exited once."""
        self._recording_started = True
        if not self._entries:
            raise CrashEvidenceError('no declared actions')
        missing = [
            entry.label
            for entry in self._entries
            if entry.observed_returncode is None
        ]
        if missing:
            raise CrashEvidenceError(
                'missing exits: ' + ', '.join(missing)
            )
        return tuple(
            (entry.label, entry.observed_returncode)
            for entry in self._entries
            if entry.observed_returncode is not None
        )

    def _declare(
        self,
        action: Any,
        label: str,
        expected_returncode: int,
    ) -> None:
        if self._recording_started:
            raise CrashEvidenceError(
                'action declarations are closed after exit recording starts'
            )
        if action is None:
            raise CrashEvidenceError('exact action must not be None')
        if not isinstance(label, str) or not label.strip():
            raise CrashEvidenceError('action label must be non-empty')
        if any(entry.label == label for entry in self._entries):
            raise CrashEvidenceError(f'duplicate action label: {label}')
        if self._entry_for_exact_action(action) is not None:
            raise CrashEvidenceError(
                f'exact action already declared: {label}'
            )
        self._entries.append(
            _ExpectedExit(
                action=action,
                label=label,
                expected_returncode=expected_returncode,
            )
        )

    @staticmethod
    def _valid_monotonic_ns(value: int, *, context: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise CrashEvidenceError(
                f'{context} must be a positive integer nanosecond value'
            )
        return value

    def _entry_for_exact_action(
        self,
        action: Any,
    ) -> _ExpectedExit | None:
        return next(
            (
                entry
                for entry in self._entries
                if entry.action is action
            ),
            None,
        )


class _ExpectedExit:
    __slots__ = (
        'action',
        'label',
        'expected_returncode',
        'signal_intent_monotonic_ns',
        'observed_returncode',
        'observed_monotonic_ns',
    )

    def __init__(
        self,
        *,
        action: Any,
        label: str,
        expected_returncode: int,
    ) -> None:
        self.action = action
        self.label = label
        self.expected_returncode = expected_returncode
        self.signal_intent_monotonic_ns: int | None = None
        self.observed_returncode: int | None = None
        self.observed_monotonic_ns: int | None = None
