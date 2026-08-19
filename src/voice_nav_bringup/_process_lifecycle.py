# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Owned process-group lifecycle and one-session lease for VoiceNav apps."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import time
from typing import Literal


CleanupStage = Literal['exited', 'graceful', 'terminated', 'killed', 'failed']
NORMAL_CLEANUP_STAGES = frozenset(('exited', 'graceful'))
GRACEFUL_TIMEOUT_S = 10.0
TERMINATE_TIMEOUT_S = 5.0
KILL_TIMEOUT_S = 1.0
PROCESS_OBSERVATION_INTERVAL_S = 0.05


class SessionLeaseError(RuntimeError):
    """Base error for a session lease that cannot be safely acquired."""


class SessionBusyError(SessionLeaseError):
    """The current user already owns a VoiceNav session in this ROS domain."""


class SessionLease:
    """One non-blocking Linux flock held for an app invocation."""

    def __init__(self, descriptor: int, path: Path) -> None:
        self._descriptor = descriptor
        self.path = path

    def close(self) -> None:
        if self._descriptor < 0:
            return
        descriptor = self._descriptor
        self._descriptor = -1
        try:
            import fcntl

            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)

    def __enter__(self) -> 'SessionLease':
        return self

    def __exit__(self, _type, _value, _traceback) -> None:
        self.close()


def _lease_root(root: Path | None) -> Path:
    if root is not None:
        selected = Path(root)
    else:
        runtime = os.environ.get('XDG_RUNTIME_DIR')
        selected = (
            Path(runtime) / 'voice-nav'
            if runtime
            else Path('/tmp') / f'voice-nav-{os.getuid()}'
        )
    if not selected.is_absolute() or selected.is_symlink():
        raise SessionLeaseError('session lease root must be absolute and not a symlink')
    selected.mkdir(mode=0o700, parents=True, exist_ok=True)
    metadata = selected.stat()
    if not stat.S_ISDIR(metadata.st_mode):
        raise SessionLeaseError('session lease root is not a directory')
    if metadata.st_uid != os.getuid():
        raise SessionLeaseError('session lease root is not owned by the current user')
    return selected


def _domain_id(value: str | None) -> int:
    raw = os.environ.get('ROS_DOMAIN_ID', '0') if value is None else value
    try:
        domain_id = int(raw, 10)
    except (TypeError, ValueError) as error:
        raise SessionLeaseError('ROS_DOMAIN_ID must be an integer') from error
    if domain_id < 0 or domain_id > 232:
        raise SessionLeaseError('ROS_DOMAIN_ID must be between 0 and 232')
    return domain_id


def acquire_session_lease(
    *, domain_id: str | None = None, root: Path | None = None,
) -> SessionLease:
    """Acquire one per-user, per-domain lease without killing another owner."""
    if os.name != 'posix':
        raise SessionLeaseError('session lease requires Linux flock')
    import fcntl

    selected_root = _lease_root(root)
    selected_domain = _domain_id(domain_id)
    path = selected_root / f'session-domain-{selected_domain}.lock'
    flags = os.O_RDWR | os.O_CREAT | getattr(os, 'O_CLOEXEC', 0)
    flags |= getattr(os, 'O_NOFOLLOW', 0)
    descriptor = os.open(path, flags, 0o600)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != os.getuid():
            raise SessionLeaseError('session lease file identity is invalid')
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise SessionBusyError(
                f'VoiceNav session already runs in ROS domain {selected_domain}'
            ) from error
        payload = json.dumps(
            {'pid': os.getpid(), 'ros_domain_id': selected_domain},
            sort_keys=True,
            separators=(',', ':'),
        ).encode('ascii')
        os.ftruncate(descriptor, 0)
        os.write(descriptor, payload)
        os.fsync(descriptor)
        return SessionLease(descriptor, path)
    except BaseException:
        os.close(descriptor)
        raise


def poll_process(process):
    poll = getattr(process, 'poll', None)
    if poll is None:
        return None
    try:
        return poll()
    except Exception:
        return None


def _posix_process_group_alive(process) -> bool | None:
    """Probe the exact owned process group instead of only its leader."""
    if os.name == 'nt':
        return None
    group_id = getattr(process, 'process_group_id', None)
    if group_id is None:
        return None
    try:
        group_id = int(group_id)
        if group_id <= 0:
            return True
        os.killpg(group_id, 0)
    except ProcessLookupError:
        return False
    except (OSError, ValueError):
        # An owned group that cannot be probed is kept live until cleanup
        # reports a failed signal, rather than being mistaken for an exit.
        return True
    return True


def is_process_live(process) -> bool:
    if process is None:
        return False
    try:
        group_probe = _posix_process_group_alive(process)
        if group_probe is not None:
            return group_probe
        group_alive = getattr(process, 'group_alive', None)
        if group_alive is not None:
            return bool(group_alive())
        return poll_process(process) is None
    except Exception:
        return True


def _signal_owned_group(process, signum: int) -> bool:
    group_signal = getattr(process, 'send_group_signal', None)
    if group_signal is not None:
        try:
            return group_signal(signum) is not False
        except ProcessLookupError:
            return True
        except Exception:
            return False

    group_id = getattr(process, 'process_group_id', None)
    if os.name != 'nt' and group_id is not None:
        try:
            os.killpg(int(group_id), signum)
        except ProcessLookupError:
            return True
        except OSError:
            return False
        return True

    sender = getattr(process, 'send_signal', None)
    if sender is None:
        return False
    try:
        if os.name == 'nt' and signum == signal.SIGINT:
            sender(signal.CTRL_BREAK_EVENT)
        else:
            sender(signum)
    except Exception:
        return False
    return True


def _wait_for_exit(process, timeout_s: float) -> bool:
    watch_group = (
        os.name != 'nt' and getattr(process, 'process_group_id', None) is not None
    )
    if not watch_group:
        try:
            process.wait(timeout=timeout_s)
        except (subprocess.TimeoutExpired, TimeoutError):
            pass
        except Exception:
            pass
        return not is_process_live(process)

    deadline = time.monotonic() + max(0.0, float(timeout_s))
    try:
        while is_process_live(process):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return not is_process_live(process)
            try:
                process.wait(timeout=remaining)
            except (subprocess.TimeoutExpired, TimeoutError):
                pass
            except Exception:
                pass
            if is_process_live(process):
                time.sleep(min(PROCESS_OBSERVATION_INTERVAL_S, remaining))
    except Exception:
        return False
    return True


def _close_streams(process) -> None:
    closer = getattr(process, 'close_streams', None)
    if closer is not None:
        try:
            closer()
        except Exception:
            pass


def close_owned_process(
    process,
    *,
    graceful_timeout_s: float = GRACEFUL_TIMEOUT_S,
    terminate_timeout_s: float = TERMINATE_TIMEOUT_S,
    kill_timeout_s: float = KILL_TIMEOUT_S,
) -> CleanupStage:
    """Close one exactly-owned process group with bounded escalation."""
    if process is None or not is_process_live(process):
        _close_streams(process)
        return 'exited'
    for signum, timeout_s, stage in (
        (signal.SIGINT, graceful_timeout_s, 'graceful'),
        (signal.SIGTERM, terminate_timeout_s, 'terminated'),
        (signal.SIGKILL, kill_timeout_s, 'killed'),
    ):
        if not _signal_owned_group(process, signum):
            _close_streams(process)
            return 'failed'
        if _wait_for_exit(process, timeout_s):
            _close_streams(process)
            return stage
    _close_streams(process)
    return 'failed'


def wait_for_frontend_or_owner_exit(
    frontend,
    owner,
    *,
    interval_s: float = PROCESS_OBSERVATION_INTERVAL_S,
) -> Literal['frontend_exited', 'owner_exited']:
    """Wait without sleeping while retaining observation of the owner session."""
    while True:
        if poll_process(owner) is not None:
            return 'owner_exited'
        if poll_process(frontend) is not None:
            return 'frontend_exited'
        try:
            frontend.wait(timeout=interval_s)
        except (subprocess.TimeoutExpired, TimeoutError):
            continue
        if poll_process(owner) is not None:
            return 'owner_exited'
        return 'frontend_exited'
