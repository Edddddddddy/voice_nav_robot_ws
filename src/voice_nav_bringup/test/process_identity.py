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

"""Exact launch-process identity and pidfd fault-injection support."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import signal
import threading
import time
from typing import Callable, Iterable

from launch.events.process import ProcessStarted


class ProcessIdentityError(RuntimeError):
    """The launch process no longer matches its recorded identity."""


@dataclass(frozen=True)
class ProcessSnapshot:
    """The kernel identity recorded for one exact process lifetime."""

    pid: int
    starttime_ticks: int
    executable: str
    cmdline: tuple[str, ...]


def read_process_snapshot(pid: int) -> ProcessSnapshot:
    """Read one Linux process identity without scanning other processes."""
    if pid <= 0:
        raise ProcessIdentityError(f'invalid process pid: {pid}')
    proc_root = Path('/proc') / str(pid)
    try:
        stat_text = (proc_root / 'stat').read_text(encoding='utf-8')
        executable = os.readlink(proc_root / 'exe')
        raw_cmdline = (proc_root / 'cmdline').read_bytes()
    except (FileNotFoundError, OSError, UnicodeError) as error:
        raise ProcessIdentityError(
            f'could not read process identity for pid={pid}'
        ) from error

    closing_parenthesis = stat_text.rfind(')')
    if closing_parenthesis < 0:
        raise ProcessIdentityError(
            f'malformed /proc/{pid}/stat process identity'
        )
    fields_after_comm = stat_text[closing_parenthesis + 2:].split()
    # The first field after the comm field is stat field 3.  starttime is
    # field 22, hence offset 19 in this suffix.
    if len(fields_after_comm) <= 19:
        raise ProcessIdentityError(
            f'short /proc/{pid}/stat process identity'
        )
    try:
        starttime_ticks = int(fields_after_comm[19])
    except ValueError as error:
        raise ProcessIdentityError(
            f'invalid /proc/{pid}/stat starttime'
        ) from error
    cmdline = tuple(
        item.decode('utf-8', errors='strict')
        for item in raw_cmdline.split(b'\0')
        if item
    )
    if not cmdline:
        raise ProcessIdentityError(f'empty /proc/{pid}/cmdline')
    return ProcessSnapshot(
        pid=pid,
        starttime_ticks=starttime_ticks,
        executable=executable,
        cmdline=cmdline,
    )


def _command_has_node_name(
    command: Iterable[str],
    expected_node_name: str,
) -> bool:
    marker = f'__node:={expected_node_name}'
    return any(argument == marker for argument in command)


def _command_has_executable(
    command: Iterable[str],
    expected_executable: str,
) -> bool:
    executable_name = Path(expected_executable).name
    return any(Path(argument).name == executable_name for argument in command)


def _resolve_launch_executable(
    command: Iterable[str],
    expected_executable: str,
) -> str:
    executable_name = Path(expected_executable).name
    candidates = [
        Path(argument)
        for argument in command
        if Path(argument).name == executable_name
    ]
    if len(candidates) != 1:
        raise ProcessIdentityError(
            'launch command executable candidate is not unique'
        )
    candidate = candidates[0]
    if not candidate.is_absolute():
        raise ProcessIdentityError(
            'launch command executable is not an absolute path'
        )
    try:
        resolved = candidate.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise ProcessIdentityError(
            'launch command executable path cannot be resolved'
        ) from error
    if not resolved.is_file() or not os.access(resolved, os.X_OK):
        raise ProcessIdentityError(
            'launch command executable path is not executable'
        )
    return os.path.realpath(resolved)


def _executable_matches(
    executable: str,
    expected_path: str,
) -> bool:
    return os.path.realpath(executable) == os.path.realpath(expected_path)


class ExactPidfdProcess:
    """A small Interface for one launch-owned, identity-checked process."""

    def __init__(
        self,
        *,
        action: object,
        event: ProcessStarted,
        expected_executable: str,
        expected_node_name: str,
    ) -> None:
        if event.action is not action:
            raise ProcessIdentityError(
                'ProcessStarted action is not the requested launch action'
            )
        if not _command_has_node_name(event.cmd, expected_node_name):
            raise ProcessIdentityError(
                'launch command node name does not match the expected FQN'
            )
        expected_executable_path = _resolve_launch_executable(
            event.cmd, expected_executable
        )
        self.action = action
        self.expected_executable = expected_executable
        self.expected_executable_path = expected_executable_path
        self.expected_node_name = expected_node_name
        self.event_command = tuple(event.cmd)
        try:
            self.pidfd = os.pidfd_open(event.pid)
        except (AttributeError, OSError) as error:
            raise ProcessIdentityError(
                'Linux pidfd_open is required for crash injection'
            ) from error
        try:
            self.snapshot = read_process_snapshot(event.pid)
        except Exception:
            os.close(self.pidfd)
            raise
        if not _executable_matches(
            self.snapshot.executable, expected_executable_path
        ):
            os.close(self.pidfd)
            raise ProcessIdentityError(
                'launch process executable does not match the expected node'
            )
        self._closed = False

    @classmethod
    def from_process_started(
        cls,
        *,
        action: object,
        event: ProcessStarted,
        expected_executable: str,
        expected_node_name: str,
    ) -> 'ExactPidfdProcess':
        """Capture a pidfd immediately from a launch ProcessStarted event."""
        return cls(
            action=action,
            event=event,
            expected_executable=expected_executable,
            expected_node_name=expected_node_name,
        )

    def validate(self, graph_count: Callable[[], int]) -> ProcessSnapshot:
        """Revalidate lifetime, executable, command, pidfd, and ROS graph."""
        if self._closed:
            raise ProcessIdentityError('pidfd was already closed')
        current = read_process_snapshot(self.snapshot.pid)
        if current != self.snapshot:
            raise ProcessIdentityError(
                'process PID/starttime/executable/cmdline changed'
            )
        if not _command_has_executable(
            current.cmdline, self.expected_executable
        ) or not _command_has_node_name(
            current.cmdline, self.expected_node_name
        ) or not _executable_matches(
            current.executable, self.expected_executable_path
        ):
            raise ProcessIdentityError(
                'current process command no longer matches launch identity'
            )
        try:
            signal.pidfd_send_signal(self.pidfd, 0)
        except (AttributeError, OSError) as error:
            raise ProcessIdentityError(
                'recorded pidfd is no longer live'
            ) from error
        try:
            count = graph_count()
        except Exception as error:
            raise ProcessIdentityError(
                'ROS graph identity could not be checked'
            ) from error
        if count != 1:
            raise ProcessIdentityError(
                f'expected one ROS node /{self.expected_node_name}, got {count}'
            )
        return current

    def kill(self, graph_count: Callable[[], int]) -> int:
        """Send SIGKILL only through the validated pidfd and return ACK time."""
        self.validate(graph_count)
        try:
            signal.pidfd_send_signal(self.pidfd, signal.SIGKILL)
        except (AttributeError, OSError) as error:
            raise ProcessIdentityError(
                'pidfd SIGKILL injection failed'
            ) from error
        return time.monotonic_ns()

    def close(self) -> None:
        """Close the one captured pidfd after injection or teardown."""
        if not self._closed:
            os.close(self.pidfd)
            self._closed = True


class ProcessStartedCapture:
    """Capture one exact ProcessStarted event and its pidfd."""

    def __init__(
        self,
        *,
        action: object,
        expected_executable: str,
        expected_node_name: str,
    ) -> None:
        self.action = action
        self.expected_executable = expected_executable
        self.expected_node_name = expected_node_name
        self._ready = threading.Event()
        self._process: ExactPidfdProcess | None = None
        self.started_monotonic_ns: int | None = None

    def on_start(self, event: ProcessStarted, _context: object) -> list[object]:
        if event.action is self.action:
            self.started_monotonic_ns = time.monotonic_ns()
            self._process = ExactPidfdProcess.from_process_started(
                action=self.action,
                event=event,
                expected_executable=self.expected_executable,
                expected_node_name=self.expected_node_name,
            )
            self._ready.set()
        return []

    def wait(self, timeout: float) -> ExactPidfdProcess:
        """Wait for the exact ProcessStarted event without a second launch."""
        if not self._ready.wait(timeout=timeout):
            raise ProcessIdentityError(
                'timed out waiting for the expected ProcessStarted event'
            )
        return self.process

    @property
    def process(self) -> ExactPidfdProcess:
        if self._process is None or not self._ready.is_set():
            raise ProcessIdentityError('ProcessStarted identity is not ready')
        return self._process

    def close(self) -> None:
        if self._process is not None:
            self._process.close()
