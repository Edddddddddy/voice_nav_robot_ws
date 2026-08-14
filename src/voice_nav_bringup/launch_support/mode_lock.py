# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Process-owned Mapping/Navigation exclusion scoped by ROS domain."""

import errno
import fcntl
import json
import os
from pathlib import Path
import secrets
import stat


class ModeLockError(RuntimeError):
    """The requested mode lock cannot be established safely."""


class ModeLockConflict(ModeLockError):
    """Another Mapping or Navigation launch owns this ROS domain."""


class ModeLock:
    """Own one advisory lock until the launch process shuts down."""

    def __init__(self, *, descriptor, path):
        self._descriptor = descriptor
        self.path = path

    def close(self):
        descriptor = self._descriptor
        if descriptor is None:
            return
        self._descriptor = None
        try:
            owned = os.fstat(descriptor)
            try:
                current = os.stat(self.path, follow_symlinks=False)
            except FileNotFoundError:
                current = None
            if (
                current is not None
                and stat.S_ISREG(current.st_mode)
                and current.st_dev == owned.st_dev
                and current.st_ino == owned.st_ino
            ):
                os.unlink(self.path)
        finally:
            try:
                fcntl.flock(descriptor, fcntl.LOCK_UN)
            finally:
                os.close(descriptor)

    def __enter__(self):
        return self

    def __exit__(self, _exception_type, _exception, _traceback):
        self.close()


class ModeLockShutdownGate:
    """Release a mode lock only after Mapping ownership has disappeared."""

    def __init__(self, owner):
        self._owner = owner
        self._shutdown_requested = False
        self._slam_process_exited = False
        self._tf_owner_disappeared = False
        self._released = False

    def request_shutdown(self):
        self._shutdown_requested = True
        self._release_if_ready()

    def observe_slam_process_exit(self):
        self._slam_process_exited = True
        self._release_if_ready()

    def observe_tf_owner_disappearance(self):
        self._tf_owner_disappeared = True
        self._release_if_ready()

    def _release_if_ready(self):
        if (
            not self._released
            and self._shutdown_requested
            and self._slam_process_exited
            and self._tf_owner_disappeared
        ):
            self._released = True
            self._owner.close()


def _is_secure_directory(path, *, uid):
    try:
        status = os.stat(path, follow_symlinks=False)
    except OSError:
        return False
    return (
        stat.S_ISDIR(status.st_mode)
        and status.st_uid == uid
        and stat.S_IMODE(status.st_mode) == 0o700
    )


def _ensure_lock_directory(path, *, uid):
    try:
        path.mkdir(mode=0o700)
    except FileExistsError:
        pass
    except OSError as error:
        raise ModeLockError(
            f'could not create mode-lock directory: {path}'
        ) from error
    if not _is_secure_directory(path, uid=uid):
        raise ModeLockError('mode-lock root must be UID-owned with mode 0700')
    return path


def _select_lock_root(environment, *, uid, fallback_parent):
    runtime = environment.get('XDG_RUNTIME_DIR')
    if runtime:
        runtime_path = Path(runtime)
        if runtime_path.is_absolute() and _is_secure_directory(
            runtime_path, uid=uid
        ):
            return _ensure_lock_directory(
                runtime_path / 'voice_nav', uid=uid
            )
    return _ensure_lock_directory(
        Path(fallback_parent) / f'voice_nav-{uid}', uid=uid
    )


def _process_starttime():
    try:
        process_stat = Path('/proc/self/stat').read_text(encoding='ascii')
        closing_parenthesis = process_stat.rfind(')')
        fields = process_stat[closing_parenthesis + 2:].split()
        return int(fields[19])
    except (OSError, UnicodeError, ValueError, IndexError) as error:
        raise ModeLockError('could not read launch process starttime') from error


def _validate_open_lock(descriptor, *, uid):
    status = os.fstat(descriptor)
    if not stat.S_ISREG(status.st_mode):
        raise ModeLockError('mode lock must be a regular file')
    if status.st_uid != uid:
        raise ModeLockError('mode lock must be owned by the effective UID')
    if stat.S_IMODE(status.st_mode) != 0o600:
        raise ModeLockError('mode lock file mode must be 0600')


def acquire_mode_lock(*, mode, environment=None, fallback_parent=Path('/tmp')):
    """Acquire a domain-scoped mode lock before owner processes start."""
    if mode not in ('mapping', 'navigation'):
        raise ModeLockError('mode must be mapping or navigation')
    environment = os.environ if environment is None else environment
    domain = environment.get('ROS_DOMAIN_ID', '0')
    if not domain.isdecimal() or not 0 <= int(domain) <= 232:
        raise ModeLockError('ROS_DOMAIN_ID must be in [0, 232]')

    root = _select_lock_root(
        environment,
        uid=os.geteuid(),
        fallback_parent=fallback_parent,
    )
    path = root / f'mode-ros-domain-{int(domain)}.lock'
    try:
        descriptor = os.open(
            path,
            os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW,
            0o600,
        )
    except OSError as error:
        raise ModeLockError(f'could not open mode lock: {path}') from error
    try:
        _validate_open_lock(descriptor, uid=os.geteuid())
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as error:
        os.close(descriptor)
        if error.errno in (errno.EACCES, errno.EAGAIN):
            raise ModeLockConflict(
                f'ROS domain {domain} already has a mode owner'
            ) from error
        raise ModeLockError('could not acquire mode lock') from error
    try:
        diagnostics = {
            'domain': int(domain),
            'launch_nonce': secrets.token_hex(16),
            'mode': mode,
            'pid': os.getpid(),
            'starttime': _process_starttime(),
        }
        payload = (json.dumps(diagnostics, sort_keys=True) + '\n').encode()
        os.ftruncate(descriptor, 0)
        os.write(descriptor, payload)
        os.fsync(descriptor)
    except BaseException:
        fcntl.flock(descriptor, fcntl.LOCK_UN)
        os.close(descriptor)
        raise
    return ModeLock(descriptor=descriptor, path=path)
