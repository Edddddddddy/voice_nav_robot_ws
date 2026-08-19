#!/usr/bin/env python3
# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Bounded Mapping-to-Navigation handoff for the installed bringup entry.

The supervisor deliberately owns no ROS, Gazebo, SLAM, Nav2, Runtime, Agent,
or audio implementation.  Those systems are supplied through two narrow
seams: a process factory and a phase observer.  This keeps the orchestration
testable without starting a real robot stack while preserving the production
ordering and ownership rules.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import runpy
import signal
import subprocess
import sys
import threading
import time
from typing import Callable, Literal, Protocol


TRUSTED_MAP_ID = 'voice_mvp'
NAVIGATION_PHRASE = '去书房'
_MAP_ROOT_PARTS = ('voice_nav', 'maps')
_DEFAULT_PHASE_TIMEOUT_S = 300.0
_MAPPING_MODE = 'mapping'
_NAVIGATION_MODE = 'navigation'
_EXPECTED_PACKAGE_FILES = (
    'map.yaml',
    'map.pgm',
    'map.posegraph',
    'map.data',
    'named_places.yaml',
    'manifest.yaml',
)
_HASHED_PACKAGE_FILES = _EXPECTED_PACKAGE_FILES[:-1]
_EXPECTED_VERSIONS = {
    'slam_toolbox': '2.8.5',
    'navigation2': '1.3.12',
}

Mode = Literal['mapping', 'navigation']
Display = Literal['headless', 'gui']
Clock = Callable[[], float]

_PROCESS_LIFECYCLE = runpy.run_path(
    str(Path(__file__).with_name('_process_lifecycle.py')),
)
close_owned_process = _PROCESS_LIFECYCLE['close_owned_process']
is_process_live = _PROCESS_LIFECYCLE['is_process_live']
NORMAL_CLEANUP_STAGES = _PROCESS_LIFECYCLE['NORMAL_CLEANUP_STAGES']


class RoundtripError(RuntimeError):
    """A controlled failure in the Mapping-to-Navigation handoff."""


@dataclass(frozen=True)
class MapPackageDescriptor:
    """The only map paths that a Navigation process may receive."""

    map_id: str
    package_root: Path
    map_yaml: Path
    named_places_yaml: Path


@dataclass(frozen=True)
class RoundtripResult:
    """Stable result returned by one bounded roundtrip attempt."""

    status: Literal['completed', 'failed']
    map_id: str
    package_root: Path
    map_yaml: Path | None = None
    named_places_yaml: Path | None = None
    reason: str = ''


class OwnedProcess(Protocol):
    """Process handle whose factory guarantees an isolated process group."""

    def poll(self) -> int | None:
        ...

    def wait(self, timeout: float | None = None) -> int | None:
        ...

    def group_alive(self) -> bool:
        ...


class RoundtripObserver(Protocol):
    """External readiness and semantic evidence needed by the supervisor."""

    def wait_for_mapping_save(
        self, *, map_root: Path, map_id: str, deadline: float,
    ) -> None:
        ...

    def wait_for_speak(self, *, map_id: str, deadline: float) -> None:
        ...

    def wait_for_map_odom_owner_gone(
        self, *, map_id: str, deadline: float,
    ) -> None:
        ...

    def wait_for_navigation_ready(
        self, *, descriptor: MapPackageDescriptor, deadline: float,
    ) -> None:
        ...

    def wait_for_vad_navigation(
        self, *, phrase: str, deadline: float,
    ) -> None:
        ...

    def wait_for_navigation_goal(
        self,
        *,
        descriptor: MapPackageDescriptor,
        target_id: str,
        deadline: float,
    ) -> None:
        ...

    def wait_for_navigation_speak(
        self, *, target_id: str, deadline: float,
    ) -> None:
        ...


def map_package_root(
    *, xdg_data_home: Path | str | None = None,
    environment: dict[str, str] | None = None,
) -> Path:
    """Resolve the trusted ``XDG_DATA_HOME/voice_nav/maps`` root.

    ``xdg_data_home`` is an injected environment value for tests, not an
    arbitrary package root.  Callers cannot provide a path containing a map
    ID; the supervisor appends the fixed VoiceNav directory and ID itself.
    """
    if xdg_data_home is None:
        environment = os.environ if environment is None else environment
        configured = environment.get('XDG_DATA_HOME')
        base = (
            Path(configured)
            if configured
            else Path.home() / '.local' / 'share'
        )
    else:
        base = Path(xdg_data_home)
    if not base.is_absolute():
        raise RoundtripError('XDG_DATA_HOME must be absolute')
    if base.is_symlink():
        raise RoundtripError('XDG_DATA_HOME must not be a symlink')
    return base.joinpath(*_MAP_ROOT_PARTS)


def _clock_now(clock: Clock | object) -> float:
    value = clock() if callable(clock) else clock.monotonic()
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise RoundtripError('clock must return a finite number')
    return float(value)


def _regular_nonempty(path: Path) -> bool:
    try:
        return (
            path.is_file()
            and not path.is_symlink()
            and path.stat().st_size > 0
        )
    except OSError:
        return False


def _validate_map_yaml(package_root: Path, map_yaml: Path) -> None:
    """Reject an unsafe image reference while keeping YAML parsing downstream."""
    try:
        text = map_yaml.read_text(encoding='utf-8')
    except (OSError, UnicodeError) as error:
        raise RoundtripError(f'map.yaml cannot be read: {error}') from error
    image_value: str | None = None
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        if stripped.startswith('image:'):
            image_value = stripped.split(':', 1)[1].strip().strip("'\"")
            break
    if image_value is None:
        raise RoundtripError('map.yaml has no image reference')
    image_path = Path(image_value)
    if image_path.is_absolute() or '..' in image_path.parts:
        raise RoundtripError('map.yaml image reference must stay in package')
    if image_path.name != 'map.pgm' or not _regular_nonempty(
        package_root / image_path
    ):
        raise RoundtripError('map.yaml must reference a non-empty map.pgm')


def _yaml_scalar(text: str, key: str) -> str | None:
    match = re.search(
        rf'(?m)^[ \t]*{re.escape(key)}[ \t]*:[ \t]*([^#\s][^#\r\n]*)',
        text,
    )
    return match.group(1).strip().strip("'\"") if match else None


def _yaml_section_scalars(text: str, section: str) -> dict[str, str]:
    """Read one shallow, trusted mapping without turning YAML into an API."""
    lines = text.splitlines()
    start = None
    section_indent = 0
    for index, line in enumerate(lines):
        match = re.match(r'^([ \t]*)([^:#]+):[ \t]*$', line)
        if match and match.group(2).strip() == section:
            start = index + 1
            section_indent = len(match.group(1).expandtabs(2))
            break
    if start is None:
        return {}
    values: dict[str, str] = {}
    for line in lines[start:]:
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        indent = len(line) - len(line.lstrip(' \t'))
        if indent <= section_indent:
            break
        match = re.match(r'^[ \t]+([^:#]+):[ \t]*([^#\s][^#\r\n]*)', line)
        if match:
            values[match.group(1).strip()] = (
                match.group(2).strip().strip("'\"")
            )
    return values


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    except OSError as error:
        raise RoundtripError(f'cannot hash {path.name}: {error}') from error
    return digest.hexdigest()


def _validate_named_places(named_places_yaml: Path) -> None:
    try:
        text = named_places_yaml.read_text(encoding='utf-8')
    except (OSError, UnicodeError) as error:
        raise RoundtripError(
            f'named_places.yaml cannot be read: {error}'
        ) from error
    if _yaml_scalar(text, 'schema_version') != '1':
        raise RoundtripError('named_places.yaml schema_version must be 1')
    if _yaml_scalar(text, 'map_id') != TRUSTED_MAP_ID:
        raise RoundtripError(
            'named_places.yaml map_id must be the trusted VoiceNav map'
        )
    if re.search(r'(?m)^\s*study\s*:\s*(?:#.*)?$', text) is None:
        raise RoundtripError('named_places.yaml must contain study')
    for coordinate in ('x', 'y', 'yaw'):
        value = _yaml_scalar(text, coordinate)
        try:
            finite = value is not None and math.isfinite(float(value))
        except (TypeError, ValueError):
            finite = False
        if not finite:
            raise RoundtripError(
                f'named_places.yaml study {coordinate} must be finite'
            )


def _validate_manifest(package_root: Path, manifest: Path) -> None:
    try:
        text = manifest.read_text(encoding='utf-8')
    except (OSError, UnicodeError) as error:
        raise RoundtripError(f'manifest.yaml cannot be read: {error}') from error
    if _yaml_scalar(text, 'schema_version') != '1':
        raise RoundtripError('manifest.yaml schema_version must be 1')
    if _yaml_scalar(text, 'map_id') != TRUSTED_MAP_ID:
        raise RoundtripError(
            'manifest.yaml map_id must be the trusted VoiceNav map'
        )
    versions = _yaml_section_scalars(text, 'versions')
    if versions != _EXPECTED_VERSIONS:
        raise RoundtripError(
            'manifest.yaml dependency versions are not locked to the contract'
        )
    for filename in _EXPECTED_PACKAGE_FILES:
        if not re.search(
            rf'(?m)^[ \t]*(?:-[ \t]*)?{re.escape(filename)}[ \t]*$', text,
        ):
            raise RoundtripError(
                f'manifest.yaml is missing relative file {filename}'
            )
    hashes = _yaml_section_scalars(text, 'sha256')
    if set(hashes) != set(_HASHED_PACKAGE_FILES):
        raise RoundtripError(
            'manifest.yaml must contain exactly five artifact SHA-256 entries'
        )
    for filename in _HASHED_PACKAGE_FILES:
        expected = hashes[filename]
        if re.fullmatch(r'[0-9a-f]{64}', expected) is None:
            raise RoundtripError(f'invalid SHA-256 for {filename}')
        if _sha256(package_root / filename) != expected:
            raise RoundtripError(f'SHA-256 mismatch for {filename}')


def _load_descriptor(package_root: Path, map_id: str) -> MapPackageDescriptor:
    """Load only the fixed package files needed by Navigation."""
    if map_id != TRUSTED_MAP_ID:
        raise RoundtripError('map_id is not the trusted VoiceNav map')
    if package_root.is_symlink() or not package_root.is_dir():
        raise RoundtripError('map package directory is unavailable')
    map_yaml = package_root / 'map.yaml'
    named_places_yaml = package_root / 'named_places.yaml'
    if not _regular_nonempty(map_yaml):
        raise RoundtripError('map.yaml is not a non-empty regular file')
    if not _regular_nonempty(named_places_yaml):
        raise RoundtripError(
            'named_places.yaml is not a non-empty regular file'
        )
    for filename in _EXPECTED_PACKAGE_FILES:
        path = package_root / filename
        if not _regular_nonempty(path):
            raise RoundtripError(
                f'{filename} is not a non-empty regular file'
            )
    _validate_map_yaml(package_root, map_yaml)
    _validate_named_places(named_places_yaml)
    _validate_manifest(package_root, package_root / 'manifest.yaml')
    return MapPackageDescriptor(
        map_id=map_id,
        package_root=package_root,
        map_yaml=map_yaml,
        named_places_yaml=named_places_yaml,
    )


def _observer_call(observer: RoundtripObserver, name: str, **kwargs) -> None:
    callback = getattr(observer, name, None)
    if callback is None:
        raise RoundtripError(f'observer is missing {name}')
    try:
        callback(**kwargs)
    except RoundtripError:
        raise
    except Exception as error:
        raise RoundtripError(f'{name} failed: {error}') from error


class MapRoundtripSupervisor:
    """Own one bounded Mapping process, then one Navigation process.

    The process factory receives ``mode``, the trusted maps root, and either
    ``None`` (Mapping) or a controlled :class:`MapPackageDescriptor`
    (Navigation).  The observer receives deadlines rather than being polled
    by this module, so tests and production adapters can choose their own
    event source without exposing ROS details here.
    """

    def __init__(
        self,
        *,
        process_factory: Callable[..., OwnedProcess],
        observer: RoundtripObserver,
        xdg_data_home: Path | str | None = None,
        map_id: str = TRUSTED_MAP_ID,
        phase_timeout_s: float = _DEFAULT_PHASE_TIMEOUT_S,
        clock: Clock | object = time.monotonic,
    ) -> None:
        if map_id != TRUSTED_MAP_ID:
            raise RoundtripError('only map_id=voice_mvp is allowed')
        if not isinstance(phase_timeout_s, (int, float)) or not math.isfinite(
            phase_timeout_s
        ) or phase_timeout_s <= 0:
            raise RoundtripError('phase timeout must be finite and positive')
        self._process_factory = process_factory
        self._observer = observer
        self._map_id = map_id
        self._phase_timeout_s = float(phase_timeout_s)
        self._clock = clock
        self._map_root = map_package_root(xdg_data_home=xdg_data_home)

    @property
    def map_root(self) -> Path:
        """Return the fixed root without exposing a caller-selected path."""
        return self._map_root

    def _deadline(self) -> float:
        return _clock_now(self._clock) + self._phase_timeout_s

    def _start(self, mode: Mode, descriptor: MapPackageDescriptor | None):
        try:
            process = self._process_factory(
                mode=mode,
                map_root=self._map_root,
                descriptor=descriptor,
            )
        except Exception as error:
            raise RoundtripError(f'{mode} process failed to start: {error}') from error
        if process is None:
            raise RoundtripError(f'{mode} process factory returned no process')
        if not is_process_live(process):
            raise RoundtripError(f'{mode} process exited on start')
        return process

    def run(self) -> RoundtripResult:
        """Run the fixed phase sequence and return a stable result."""
        package_root = self._map_root / self._map_id
        mapping_process: OwnedProcess | None = None
        navigation_process: OwnedProcess | None = None
        descriptor: MapPackageDescriptor | None = None
        failure: str | None = None

        if package_root.exists() or package_root.is_symlink():
            return RoundtripResult(
                status='failed',
                map_id=self._map_id,
                package_root=package_root,
                reason='map_package_exists_overwrite_rejected',
            )
        try:
            self._map_root.mkdir(parents=True, exist_ok=True)
            mapping_process = self._start(_MAPPING_MODE, None)
            _observer_call(
                self._observer,
                'wait_for_mapping_save',
                map_root=self._map_root,
                map_id=self._map_id,
                deadline=self._deadline(),
            )
            _observer_call(
                self._observer,
                'wait_for_speak',
                map_id=self._map_id,
                deadline=self._deadline(),
            )
            descriptor = _load_descriptor(package_root, self._map_id)
        except RoundtripError as error:
            failure = str(error)
        except BaseException:
            try:
                if mapping_process is not None:
                    close_owned_process(mapping_process)
            finally:
                raise

        cleanup_error: str | None = None
        if mapping_process is not None:
            try:
                cleanup_stage = close_owned_process(mapping_process)
                if cleanup_stage not in NORMAL_CLEANUP_STAGES:
                    cleanup_error = f'mapping_process_cleanup_{cleanup_stage}'
            except RoundtripError as error:
                cleanup_error = str(error)
            try:
                _observer_call(
                    self._observer,
                    'wait_for_map_odom_owner_gone',
                    map_id=self._map_id,
                    deadline=self._deadline(),
                )
            except RoundtripError as error:
                if failure is None:
                    failure = str(error)

        if cleanup_error is not None and failure is None:
            failure = cleanup_error
        if failure is not None or descriptor is None:
            return RoundtripResult(
                status='failed',
                map_id=self._map_id,
                package_root=package_root,
                map_yaml=descriptor.map_yaml if descriptor else None,
                named_places_yaml=(
                    descriptor.named_places_yaml if descriptor else None
                ),
                reason=failure or 'mapping_roundtrip_failed',
            )

        try:
            navigation_process = self._start(_NAVIGATION_MODE, descriptor)
            _observer_call(
                self._observer,
                'wait_for_navigation_ready',
                descriptor=descriptor,
                deadline=self._deadline(),
            )
            _observer_call(
                self._observer,
                'wait_for_vad_navigation',
                phrase=NAVIGATION_PHRASE,
                deadline=self._deadline(),
            )
            _observer_call(
                self._observer,
                'wait_for_navigation_goal',
                descriptor=descriptor,
                target_id='study',
                deadline=self._deadline(),
            )
            _observer_call(
                self._observer,
                'wait_for_navigation_speak',
                target_id='study',
                deadline=self._deadline(),
            )
        except RoundtripError as error:
            failure = str(error)
        finally:
            if navigation_process is not None:
                try:
                    cleanup_stage = close_owned_process(navigation_process)
                    if cleanup_stage not in NORMAL_CLEANUP_STAGES:
                        failure = failure or (
                            f'navigation_process_cleanup_{cleanup_stage}'
                        )
                except RoundtripError as error:
                    failure = failure or str(error)

        if failure is not None:
            return RoundtripResult(
                status='failed',
                map_id=self._map_id,
                package_root=package_root,
                map_yaml=descriptor.map_yaml,
                named_places_yaml=descriptor.named_places_yaml,
                reason=failure,
            )
        return RoundtripResult(
            status='completed',
            map_id=self._map_id,
            package_root=package_root,
            map_yaml=descriptor.map_yaml,
            named_places_yaml=descriptor.named_places_yaml,
        )


# Keep the shorter name available to callers that treat this as a generic
# roundtrip seam while retaining the map-specific public name in documentation.
RoundtripSupervisor = MapRoundtripSupervisor


def build_production_command(
    mode: Mode,
    display: Display = 'headless',
) -> tuple[str, ...]:
    """Return the fixed installed app command for one immutable mode."""
    if mode not in (_MAPPING_MODE, _NAVIGATION_MODE):
        raise RoundtripError(f'unsupported roundtrip mode: {mode}')
    if display not in ('headless', 'gui'):
        raise RoundtripError(f'unsupported roundtrip display: {display}')
    return (
        'ros2',
        'run',
        'voice_nav_bringup',
        'voice_nav_app',
        '--mode',
        mode,
        '--display',
        display,
        '--input',
        'vad-auto',
    )


class ObservedProcess:
    """Owned app process with bounded, continuously drained output."""

    _RECENT_LINE_LIMIT = 256

    def __init__(self, process: subprocess.Popen[str], mode: Mode) -> None:
        self._process = process
        self.mode = mode
        self.process_group_id = process.pid
        self._recent_lines: deque[str] = deque(
            maxlen=self._RECENT_LINE_LIMIT,
        )
        self._lock = threading.Lock()
        self._changed = threading.Event()
        self._reader_threads: list[threading.Thread] = []
        for stream_name in ('stdout', 'stderr'):
            stream = getattr(process, stream_name, None)
            if stream is None:
                continue
            reader = threading.Thread(
                target=self._drain,
                args=(stream_name, stream),
                name=f'voice-nav-roundtrip-{mode}-{stream_name}',
                daemon=True,
            )
            self._reader_threads.append(reader)
            reader.start()

    def _drain(self, stream_name: str, stream) -> None:
        try:
            try:
                for line in iter(stream.readline, ''):
                    rendered = f'{self.mode}/{stream_name}: {line.rstrip()}'
                    with self._lock:
                        self._recent_lines.append(rendered)
                    sys.stderr.write(rendered + '\n')
                    sys.stderr.flush()
                    self._changed.set()
            except (OSError, ValueError):
                pass
        finally:
            self._changed.set()

    def poll(self) -> int | None:
        return self._process.poll()

    def wait(self, timeout: float | None = None) -> int | None:
        return self._process.wait(timeout=timeout)

    def group_alive(self) -> bool:
        if os.name == 'nt':
            return self._process.poll() is None
        try:
            os.killpg(self.process_group_id, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def send_group_signal(self, signum: int) -> None:
        if os.name == 'nt':
            if signum == signal.SIGINT:
                self._process.send_signal(signal.CTRL_BREAK_EVENT)
            elif signum == signal.SIGTERM:
                self._process.terminate()
            else:
                self._process.kill()
            return
        os.killpg(self.process_group_id, signum)

    def recent_lines(self) -> tuple[str, ...]:
        with self._lock:
            return tuple(self._recent_lines)

    def wait_for_log(
        self,
        predicate: Callable[[tuple[str, ...]], bool],
        *,
        deadline: float,
        clock: Clock | object,
        description: str,
    ) -> None:
        while True:
            if predicate(self.recent_lines()):
                return
            returncode = self.poll()
            if returncode is not None:
                recent = self.recent_lines()[-8:]
                raise RoundtripError(
                    f'{self.mode} process exited before {description}: '
                    f'returncode={returncode}; recent={recent}'
                )
            remaining = deadline - _clock_now(clock)
            if remaining <= 0:
                raise RoundtripError(
                    f'{self.mode} log evidence timeout: {description}'
                )
            self._changed.wait(timeout=min(0.05, remaining))
            self._changed.clear()

    def close_streams(self) -> None:
        for stream_name in ('stdout', 'stderr'):
            stream = getattr(self._process, stream_name, None)
            if stream is not None:
                try:
                    stream.close()
                except (OSError, ValueError):
                    pass
        for reader in self._reader_threads:
            reader.join(timeout=0.2)


class ProductionProcessFactory:
    """Create fixed app commands and retain their observed process handles."""

    def __init__(self, display: Display = 'headless') -> None:
        if display not in ('headless', 'gui'):
            raise RoundtripError(f'unsupported roundtrip display: {display}')
        self._display = display
        self._processes: dict[Mode, ObservedProcess] = {}

    def __call__(
        self,
        *,
        mode: Mode,
        map_root: Path,
        descriptor: MapPackageDescriptor | None,
    ) -> ObservedProcess:
        environment = dict(os.environ)
        xdg_data_home = map_root.parent.parent
        environment.update({
            'XDG_DATA_HOME': str(xdg_data_home),
            'VOICE_NAV_MAP_ID': TRUSTED_MAP_ID,
            'VOICE_NAV_MAP_ROOT': str(map_root),
            'VOICE_NAV_ROUNDTRIP': '1',
        })
        if descriptor is not None:
            environment.update({
                'VOICE_NAV_MAP_YAML': str(descriptor.map_yaml),
                'VOICE_NAV_NAMED_PLACES_YAML': str(
                    descriptor.named_places_yaml
                ),
            })
        options: dict[str, object] = {
            'env': environment,
            'stdin': subprocess.DEVNULL,
            'stdout': subprocess.PIPE,
            'stderr': subprocess.PIPE,
            'text': True,
            'encoding': 'utf-8',
            'errors': 'replace',
            'bufsize': 1,
        }
        if os.name == 'nt':
            options['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            options['start_new_session'] = True
        process = subprocess.Popen(
            build_production_command(mode, self._display), **options,
        )
        observed = ObservedProcess(process, mode)
        self._processes[mode] = observed
        return observed

    def process(self, mode: Mode) -> ObservedProcess:
        try:
            return self._processes[mode]
        except KeyError as error:
            raise RoundtripError(f'{mode} process has not started') from error


def production_process_factory(
    *,
    mode: Mode,
    map_root: Path,
    descriptor: MapPackageDescriptor | None,
) -> OwnedProcess:
    """Compatibility helper for callers that need one fixed app process."""
    return ProductionProcessFactory()(
        mode=mode, map_root=map_root, descriptor=descriptor,
    )


def _has_ready_json(lines: tuple[str, ...]) -> bool:
    for line in lines:
        try:
            json_start = line.find('{')
            payload = json.loads(line[json_start:]) if json_start >= 0 else None
        except (json.JSONDecodeError, TypeError):
            continue
        if isinstance(payload, dict) and payload.get('status') == 'ready':
            return True
    return any('app ready' in line.lower() for line in lines)


def _has_log_text(*needles: str) -> Callable[[tuple[str, ...]], bool]:
    lowered = tuple(needle.lower() for needle in needles)
    return lambda lines: all(
        any(needle in line.lower() for line in lines)
        for needle in lowered
    )


class ProductionRoundtripObserver:
    """Observe package, app logs, and the post-cleanup ROS graph."""

    def __init__(
        self,
        *,
        process_factory: ProductionProcessFactory,
        clock: Clock | object = time.monotonic,
    ) -> None:
        self._process_factory = process_factory
        self._clock = clock

    def _process(self, mode: Mode) -> ObservedProcess:
        return self._process_factory.process(mode)

    def wait_for_mapping_save(
        self, *, map_root: Path, map_id: str, deadline: float,
    ) -> None:
        process = self._process('mapping')
        package_root = map_root / map_id

        def package_and_ready_evidence(lines: tuple[str, ...]) -> bool:
            if not _has_ready_json(lines):
                return False
            try:
                _load_descriptor(package_root, map_id)
            except RoundtripError:
                return False
            return True

        process.wait_for_log(
            package_and_ready_evidence,
            deadline=deadline,
            clock=self._clock,
            description='app ready and complete SAVE_MAP package',
        )

    def wait_for_speak(self, *, map_id: str, deadline: float) -> None:
        del map_id
        self._process('mapping').wait_for_log(
            _has_log_text('speak_result', 'completed'),
            deadline=deadline,
            clock=self._clock,
            description='mapping Speak completed',
        )

    def wait_for_map_odom_owner_gone(
        self, *, map_id: str, deadline: float,
    ) -> None:
        del map_id
        if is_process_live(self._process('mapping')):
            raise RoundtripError('mapping process group is still alive')
        _confirm_slam_owner_gone(deadline=deadline, clock=self._clock)

    def wait_for_navigation_ready(
        self, *, descriptor: MapPackageDescriptor, deadline: float,
    ) -> None:
        self._process('navigation').wait_for_log(
            _has_ready_json,
            deadline=deadline,
            clock=self._clock,
            description='navigation app ready',
        )
        if descriptor.map_yaml.is_symlink() or not descriptor.map_yaml.is_file():
            raise RoundtripError('navigation map.yaml is unavailable')

    def wait_for_vad_navigation(self, *, phrase: str, deadline: float) -> None:
        if phrase != NAVIGATION_PHRASE:
            raise RoundtripError('unsupported continuous VAD phrase')
        self._process('navigation').wait_for_log(
            _has_log_text(phrase),
            deadline=deadline,
            clock=self._clock,
            description='VoiceTurn 去书房',
        )

    def wait_for_navigation_goal(
        self,
        *,
        descriptor: MapPackageDescriptor,
        target_id: str,
        deadline: float,
    ) -> None:
        if target_id != 'study' or descriptor.map_id != TRUSTED_MAP_ID:
            raise RoundtripError('only NAVIGATE_TO(study) is allowed')
        self._process('navigation').wait_for_log(
            _has_log_text('Reached the goal!', 'Goal succeeded'),
            deadline=deadline,
            clock=self._clock,
            description='NAVIGATE_TO(study) success',
        )

    def wait_for_navigation_speak(
        self, *, target_id: str, deadline: float,
    ) -> None:
        if target_id != 'study':
            raise RoundtripError('navigation Speak target must be study')
        self._process('navigation').wait_for_log(
            _has_log_text('speak_result', 'completed'),
            deadline=deadline,
            clock=self._clock,
            description='navigation Speak completed',
        )


def _confirm_slam_owner_gone(*, deadline: float, clock: Clock | object) -> None:
    """Use the existing ROS graph CLI as a read-only owner probe."""
    remaining = max(0.01, deadline - _clock_now(clock))
    try:
        result = subprocess.run(
            ('ros2', 'node', 'list', '--no-daemon'),
            capture_output=True,
            text=True,
            timeout=remaining,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise RoundtripError(f'ROS graph owner probe failed: {error}') from error
    if result.returncode != 0:
        raise RoundtripError('ROS graph owner probe returned non-zero')
    names = {
        line.strip().rstrip('/')
        for line in result.stdout.splitlines()
        if line.strip()
    }
    if '/slam_toolbox' in names or 'slam_toolbox' in names:
        raise RoundtripError('slam_toolbox owner remained after Mapping cleanup')


def _write_result(stdout, result: RoundtripResult | dict[str, str]) -> None:
    if isinstance(result, RoundtripResult):
        payload = {
            'status': result.status,
            'map_id': result.map_id,
            'package_root': str(result.package_root),
            'map_yaml': str(result.map_yaml) if result.map_yaml else None,
            'named_places_yaml': (
                str(result.named_places_yaml)
                if result.named_places_yaml else None
            ),
            'reason': result.reason,
        }
    else:
        payload = result
    stdout.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + '\n')
    stdout.flush()


def main(
    argv: list[str] | None = None,
    *,
    process_factory: Callable[..., OwnedProcess] | None = None,
    observer: RoundtripObserver | None = None,
    clock: Clock | object = time.monotonic,
    stdout=None,
) -> int:
    """Run an injected roundtrip or the fixed installed composition."""
    if stdout is None:
        stdout = sys.stdout
    parser = argparse.ArgumentParser(
        description='Run the bounded VoiceNav Mapping-to-Navigation handoff.',
    )
    parser.add_argument(
        '--timeout-s',
        type=float,
        default=_DEFAULT_PHASE_TIMEOUT_S,
        help='Bound each injected phase (seconds).',
    )
    parser.add_argument(
        '--display',
        choices=('headless', 'gui'),
        default='headless',
        help='Run both Mapping and Navigation with or without Gazebo GUI.',
    )
    try:
        arguments = parser.parse_args(argv)
    except SystemExit as error:
        return int(error.code)
    if process_factory is None and observer is None:
        production_factory = ProductionProcessFactory(arguments.display)
        process_factory = production_factory
        observer = ProductionRoundtripObserver(
            process_factory=production_factory,
            clock=clock,
        )
    elif process_factory is None or observer is None:
        _write_result(
            stdout,
            {
                'status': 'unavailable',
                'reason': 'process_factory_and_observer_must_be_paired',
            },
        )
        return 2
    try:
        result = MapRoundtripSupervisor(
            process_factory=process_factory,
            observer=observer,
            phase_timeout_s=arguments.timeout_s,
            clock=clock,
        ).run()
    except RoundtripError as error:
        _write_result(
            stdout,
            {'status': 'unavailable', 'reason': str(error)},
        )
        return 2
    _write_result(stdout, result)
    return 0 if result.status == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
