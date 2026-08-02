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

    def record_exit(self, action: Any, returncode: int) -> None:
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
        if returncode != entry.expected_returncode:
            raise CrashEvidenceError(
                f'{entry.label} expected {entry.expected_returncode}, '
                f'observed {returncode}'
            )
        entry.observed_returncode = returncode

    def assert_complete(self) -> tuple[tuple[str, int], ...]:
        """Return the closed ledger only when every action exited once."""
        self._recording_started = True
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
        'observed_returncode',
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
        self.observed_returncode: int | None = None
