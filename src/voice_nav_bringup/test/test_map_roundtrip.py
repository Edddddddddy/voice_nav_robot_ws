# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Behavior tests for the bounded Mapping-to-Navigation roundtrip seam."""

from __future__ import annotations

import importlib.util
import hashlib
import io
from pathlib import Path
import pytest
import sys


def _load_roundtrip_module():
    package_root = Path(__file__).resolve().parents[1]
    module_path = package_root / 'voice_nav_map_roundtrip.py'
    specification = importlib.util.spec_from_file_location(
        'voice_nav_map_roundtrip', module_path,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


class _ObservedPopen:
    def __init__(self, *, stdout, stderr, returncode=None):
        self.pid = 12001
        self.stdout = stdout
        self.stderr = stderr
        self._returncode = returncode

    def poll(self):
        return self._returncode


class _FakeProcess:
    def __init__(self, events: list[str], name: str) -> None:
        self._events = events
        self._name = name
        self._returncode: int | None = None

    def poll(self) -> int | None:
        return self._returncode

    def send_group_signal(self, signum: int) -> None:
        del signum
        self._events.append(f'{self._name}_group_stop')
        self._returncode = 0

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        self._returncode = 0
        return self._returncode


def _write_fake_package(package: Path, *, tamper: bool = False) -> None:
    package.mkdir(parents=True)
    (package / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\0')
    (package / 'map.posegraph').write_text('posegraph\n')
    (package / 'map.data').write_text('serialized\n')
    (package / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.05\n',
    )
    (package / 'named_places.yaml').write_text(
        'schema_version: 1\nmap_id: voice_mvp\nplaces:\n  study:\n'
        '    x: 1.0\n    y: 2.0\n    yaw: 0.0\n',
    )
    hashes = {
        filename: hashlib.sha256((package / filename).read_bytes()).hexdigest()
        for filename in (
            'map.yaml', 'map.pgm', 'map.posegraph', 'map.data',
            'named_places.yaml',
        )
    }
    if tamper:
        hashes['map.data'] = '0' * 64
    (package / 'manifest.yaml').write_text(
        'schema_version: 1\nmap_id: voice_mvp\nversions:\n'
        '  slam_toolbox: 2.8.5\n  navigation2: 1.3.12\nfiles:\n'
        '  - map.yaml\n  - map.pgm\n  - map.posegraph\n'
        '  - map.data\n  - named_places.yaml\n  - manifest.yaml\n'
        'sha256:\n'
        + ''.join(
            f'  {filename}: {digest}\n'
            for filename, digest in hashes.items()
        ),
    )


def test_roundtrip_publishes_mapping_before_navigation_and_preserves_paths(
    tmp_path: Path,
):
    module = _load_roundtrip_module()
    events: list[str] = []
    processes: dict[str, _FakeProcess] = {}

    def process_factory(*, mode, map_root, descriptor):
        events.append(f'{mode}_start')
        assert map_root == tmp_path / 'voice_nav' / 'maps'
        if mode == 'mapping':
            assert descriptor is None
        else:
            assert descriptor.map_id == module.TRUSTED_MAP_ID
            assert descriptor.map_yaml == (
                map_root / module.TRUSTED_MAP_ID / 'map.yaml'
            )
            assert descriptor.named_places_yaml == (
                map_root / module.TRUSTED_MAP_ID / 'named_places.yaml'
            )
        process = _FakeProcess(events, mode)
        processes[mode] = process
        return process

    class _FakeObserver:
        def wait_for_mapping_save(self, *, map_root, map_id, deadline):
            del deadline
            events.append('mapping_save_success')
            package = map_root / map_id
            _write_fake_package(package)

        def wait_for_speak(self, *, map_id, deadline):
            del map_id, deadline
            events.append('mapping_speak_success')

        def wait_for_map_odom_owner_gone(self, *, map_id, deadline):
            del map_id, deadline
            events.append('mapping_owner_gone')

        def wait_for_navigation_ready(self, *, descriptor, deadline):
            del deadline
            events.append('navigation_ready')
            assert descriptor.map_id == module.TRUSTED_MAP_ID

        def wait_for_vad_navigation(self, *, phrase, deadline):
            del deadline
            events.append(f'vad:{phrase}')
            assert phrase == '去书房'

        def wait_for_navigation_goal(
            self, *, descriptor, target_id, deadline,
        ):
            del deadline
            events.append('navigation_goal_succeeded')
            assert descriptor.map_id == module.TRUSTED_MAP_ID
            assert target_id == 'study'

        def wait_for_navigation_speak(self, *, target_id, deadline):
            del deadline
            events.append('navigation_speak_success')
            assert target_id == 'study'

    supervisor = module.MapRoundtripSupervisor(
        process_factory=process_factory,
        observer=_FakeObserver(),
        xdg_data_home=tmp_path,
        phase_timeout_s=1.0,
    )

    result = supervisor.run()

    assert result.status == 'completed', result.reason
    assert result.map_id == module.TRUSTED_MAP_ID
    assert result.map_yaml.is_file()
    assert result.named_places_yaml.is_file()
    assert events == [
        'mapping_start',
        'mapping_save_success',
        'mapping_speak_success',
        'mapping_group_stop',
        'mapping_owner_gone',
        'navigation_start',
        'navigation_ready',
        'vad:去书房',
        'navigation_goal_succeeded',
        'navigation_speak_success',
        'navigation_group_stop',
    ]
    assert processes['mapping'].poll() == 0
    assert processes['navigation'].poll() == 0


def test_mapping_keyboard_interrupt_cleans_owned_process_and_reraises(
    tmp_path: Path,
):
    module = _load_roundtrip_module()
    events: list[str] = []
    processes: list[_FakeProcess] = []

    def process_factory(*, mode, map_root, descriptor):
        del map_root, descriptor
        process = _FakeProcess(events, mode)
        processes.append(process)
        return process

    class _SimulatedKeyboardInterrupt(KeyboardInterrupt):
        pass

    class _InterruptedObserver:
        def wait_for_mapping_save(self, *, map_root, map_id, deadline):
            del map_root, map_id, deadline
            raise _SimulatedKeyboardInterrupt()

    with pytest.raises(_SimulatedKeyboardInterrupt):
        module.MapRoundtripSupervisor(
            process_factory=process_factory,
            observer=_InterruptedObserver(),
            xdg_data_home=tmp_path,
            phase_timeout_s=1.0,
        ).run()

    assert len(processes) == 1
    assert processes[0].poll() == 0
    assert events == ['mapping_group_stop']


def test_tampered_artifact_hash_fails_before_navigation_start(tmp_path: Path):
    module = _load_roundtrip_module()
    starts: list[str] = []

    def process_factory(*, mode, map_root, descriptor):
        del map_root, descriptor
        starts.append(mode)
        return _FakeProcess([], mode)

    class _Observer:
        def wait_for_mapping_save(self, *, map_root, map_id, deadline):
            del deadline
            _write_fake_package(map_root / map_id, tamper=True)

        def wait_for_speak(self, *, map_id, deadline):
            del map_id, deadline

        def wait_for_map_odom_owner_gone(self, *, map_id, deadline):
            del map_id, deadline

    result = module.MapRoundtripSupervisor(
        process_factory=process_factory,
        observer=_Observer(),
        xdg_data_home=tmp_path,
        phase_timeout_s=1.0,
    ).run()

    assert result.status == 'failed'
    assert 'SHA-256 mismatch for map.data' in result.reason
    assert starts == ['mapping']


def test_trusted_id_and_overwrite_guard_run_before_process_start(tmp_path: Path):
    module = _load_roundtrip_module()
    with pytest.raises(module.RoundtripError, match='only map_id=voice_mvp'):
        module.MapRoundtripSupervisor(
            process_factory=lambda **_: None,
            observer=object(),
            xdg_data_home=tmp_path,
            map_id='../escape',
        )

    package = tmp_path / 'voice_nav' / 'maps' / module.TRUSTED_MAP_ID
    package.mkdir(parents=True)
    starts: list[str] = []
    result = module.MapRoundtripSupervisor(
        process_factory=lambda **kwargs: starts.append(kwargs['mode']),
        observer=object(),
        xdg_data_home=tmp_path,
    ).run()
    assert result.status == 'failed'
    assert result.reason == 'map_package_exists_overwrite_rejected'
    assert starts == []


def test_production_command_is_fixed_and_has_no_caller_mode_arguments():
    module = _load_roundtrip_module()
    assert module.build_production_command('mapping') == (
        'ros2', 'run', 'voice_nav_bringup', 'voice_nav_app',
        '--mode', 'mapping', '--display', 'headless', '--input', 'vad-auto',
    )
    with pytest.raises(module.RoundtripError):
        module.build_production_command('motion')


def test_observed_process_forwards_prefixed_output_and_bounds_recent_ring(
    monkeypatch,
):
    module = _load_roundtrip_module()
    parent_stderr = io.StringIO()
    monkeypatch.setattr(module.sys, 'stderr', parent_stderr)
    process = _ObservedPopen(
        stdout=io.StringIO(''.join(f'line-{index}\n' for index in range(300))),
        stderr=io.StringIO('warning\n'),
    )
    observed = module.ObservedProcess(process, 'mapping')

    class _Clock:
        def __init__(self):
            self.calls = 0

        def __call__(self):
            self.calls += 1
            return 0.0 if self.calls == 1 else 2.0

    try:
        observed.wait_for_log(
            lambda lines: 'mapping/stdout: line-299' in lines,
            deadline=1.0,
            clock=_Clock(),
            description='child output',
        )
        recent = observed.recent_lines()
    finally:
        observed.close_streams()

    forwarded = parent_stderr.getvalue()
    assert 'mapping/stdout: line-299\n' in forwarded
    assert 'mapping/stderr: warning\n' in forwarded
    assert len(recent) == 256
    assert 'mapping/stdout: line-299' in recent


def test_wait_for_log_fails_fast_with_bounded_exit_reason():
    module = _load_roundtrip_module()
    process = _ObservedPopen(
        stdout=io.StringIO(),
        stderr=io.StringIO(),
        returncode=23,
    )
    observed = module.ObservedProcess(process, 'mapping')

    class _Clock:
        def __init__(self):
            self.calls = 0

        def __call__(self):
            self.calls += 1
            return 0.0 if self.calls == 1 else 2000.0

    clock = _Clock()
    try:
        with pytest.raises(module.RoundtripError) as raised:
            observed.wait_for_log(
                lambda lines: False,
                deadline=1000.0,
                clock=clock,
                description='mapping ready',
            )
    finally:
        observed.close_streams()

    message = str(raised.value)
    assert 'returncode=23' in message
    assert 'recent=' in message
    assert clock.calls <= 1


class _LogProcess:
    def __init__(self, lines: tuple[str, ...]) -> None:
        self.lines = lines

    def wait_for_log(self, predicate, **kwargs):
        del kwargs
        if not predicate(self.lines):
            raise RuntimeError('expected log predicate was not satisfied')


class _ProcessRegistry:
    def __init__(self, process) -> None:
        self._process = process

    def process(self, mode):
        assert mode == 'mapping'
        return self._process


def test_production_mapping_save_uses_ready_json_and_complete_package(
    tmp_path: Path,
):
    module = _load_roundtrip_module()
    package = tmp_path / 'voice_nav' / 'maps' / module.TRUSTED_MAP_ID
    _write_fake_package(package)
    observer = module.ProductionRoundtripObserver(
        process_factory=_ProcessRegistry(
            _LogProcess(('stdout: {"status":"ready"}',)),
        ),
        clock=lambda: 0.0,
    )

    observer.wait_for_mapping_save(
        map_root=tmp_path / 'voice_nav' / 'maps',
        map_id=module.TRUSTED_MAP_ID,
        deadline=1.0,
    )


def test_production_mapping_save_rejects_log_without_package(tmp_path: Path):
    module = _load_roundtrip_module()
    observer = module.ProductionRoundtripObserver(
        process_factory=_ProcessRegistry(
            _LogProcess(('stdout: {"status":"ready"}', 'SAVE_MAP done')),
        ),
        clock=lambda: 0.0,
    )

    with pytest.raises(RuntimeError, match='expected log predicate'):
        observer.wait_for_mapping_save(
            map_root=tmp_path / 'voice_nav' / 'maps',
            map_id=module.TRUSTED_MAP_ID,
            deadline=1.0,
        )


def test_cleanup_signals_descendants_when_leader_already_exited():
    module = _load_roundtrip_module()

    class _GroupProcess:
        def __init__(self):
            self.signals: list[int] = []
            self.descendants_alive = True

        def poll(self):
            return 0

        def group_alive(self):
            return self.descendants_alive

        def send_group_signal(self, signum):
            self.signals.append(signum)
            self.descendants_alive = False

        def wait(self, timeout=None):
            del timeout
            return 0

    process = _GroupProcess()
    assert module._close_owned_process(process) == 'graceful'
    assert process.signals == [module.signal.SIGINT]
    assert process.group_alive() is False


def test_cleanup_escalation_uses_bounded_product_budgets_without_sleep():
    module = _load_roundtrip_module()

    class _EscalatingProcess:
        def __init__(self):
            self.signals: list[int] = []
            self.wait_timeouts: list[float | None] = []
            self.descendants_alive = True

        def poll(self):
            return 0

        def group_alive(self):
            return self.descendants_alive

        def send_group_signal(self, signum):
            self.signals.append(signum)
            if signum == module.signal.SIGKILL:
                self.descendants_alive = False

        def wait(self, timeout=None):
            self.wait_timeouts.append(timeout)
            return 0

    process = _EscalatingProcess()
    assert module._close_owned_process(process) == 'killed'
    assert process.signals == [
        module.signal.SIGINT,
        module.signal.SIGTERM,
        module.signal.SIGKILL,
    ]
    assert process.wait_timeouts == [10.0, 5.0, 1.0]
    assert module._DEFAULT_PHASE_TIMEOUT_S == 300.0
