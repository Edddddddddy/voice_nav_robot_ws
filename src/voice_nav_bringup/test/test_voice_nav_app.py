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

"""Behavioral tests for the installed simulation-only app wrapper."""

from __future__ import annotations

import ast
import builtins
import hashlib
import importlib.util
import signal
import wave
from io import StringIO
from pathlib import Path
from types import SimpleNamespace


def _load_app_module():
    source = Path(__file__).resolve().parents[1] / 'voice_nav_app.py'
    specification = importlib.util.spec_from_file_location(
        'voice_nav_app', source,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _load_mode_readiness_module():
    source = Path(__file__).resolve().parents[1] / '_mode_readiness.py'
    specification = importlib.util.spec_from_file_location(
        'voice_nav_mode_readiness', source,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _load_sensevoice_input_module():
    source = Path(__file__).resolve().parents[1] / '_sensevoice_input.py'
    specification = importlib.util.spec_from_file_location(
        'voice_nav_sensevoice_input', source,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _load_chaowen_asset_verifier_module():
    source = (
        Path(__file__).resolve().parents[1] / '_chaowen_asset_verifier.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_chaowen_asset_verifier', source,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _write_pcm_wav(path: Path, *, frames: int = 1600) -> None:
    with wave.open(str(path), 'wb') as stream:
        stream.setnchannels(1)
        stream.setsampwidth(2)
        stream.setframerate(16000)
        stream.writeframes(b'\x00\x00' * frames)


def _configured_motion_readiness(readiness, now):
    state = readiness._MotionReadinessState(lambda: now[0])
    state.observe_runtime(SimpleNamespace(
        availability=readiness.RUNTIME_AVAILABLE,
        gate_state=readiness.GATE_INHIBITED,
        active_step=readiness.NO_ACTIVE_STEP,
    ))
    state.observe_motion_gate(SimpleNamespace(
        state=readiness.MOTION_GATE_INHIBITED,
        motion_inhibited=True,
        zero_selected=True,
        zero_publish_seq=1,
    ))
    state.observe_controller_active(True)
    return state


def _odometry_sample(linear_x=0.0):
    return SimpleNamespace(
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=linear_x),
                angular=SimpleNamespace(z=0.0),
            ),
        ),
    )


class _FakeProcess:
    def __init__(self):
        self.returncode = None
        self.group_signals = []
        self.wait_timeouts = []

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        self.returncode = 0
        return self.returncode

    def send_group_signal(self, signum):
        self.group_signals.append(signum)


class _InteractiveFakeProcess(_FakeProcess):
    """Model a frontend that owns stdin until the user exits it."""

    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        if timeout is not None:
            raise TimeoutError('interactive frontend must not be timed')
        self.returncode = 0
        return self.returncode


class _InterruptingInteractiveFakeProcess(_FakeProcess):
    """Raise Ctrl+C only from the interactive wait, then cleanly exit."""

    def __init__(self):
        super().__init__()
        self.interrupted = False

    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        if not self.interrupted:
            self.interrupted = True
            raise KeyboardInterrupt
        self.returncode = 0
        return self.returncode


def test_mapping_state_requires_active_slam_valid_map_and_unique_map_odom_tf():
    """Keep typed mode evidence behind a small callback state interface."""
    readiness = _load_mode_readiness_module()
    state = readiness._ModeReadinessState('mapping')

    assert state.failure_stage() == 'slam_toolbox_lifecycle'
    state.observe_lifecycle('slam_toolbox', readiness.PRIMARY_STATE_ACTIVE)
    assert state.failure_stage() == 'map'

    invalid_map = SimpleNamespace(
        info=SimpleNamespace(width=2, height=2), data=[-1, 0],
    )
    state.observe_map(invalid_map)
    assert state.failure_stage() == 'map'

    non_matching_tf = SimpleNamespace(
        transforms=[SimpleNamespace(
            header=SimpleNamespace(frame_id='odom'),
            child_frame_id='base_footprint',
        )],
    )
    state.observe_tf(non_matching_tf)
    assert state.failure_stage() == 'map'

    valid_map = SimpleNamespace(
        info=SimpleNamespace(width=2, height=1), data=[-1, 0],
    )
    state.observe_map(valid_map)
    assert state.failure_stage() == 'map_odom_tf'

    duplicate_tf = SimpleNamespace(
        transforms=[
            SimpleNamespace(
                header=SimpleNamespace(frame_id='map'),
                child_frame_id='odom',
            ),
            SimpleNamespace(
                header=SimpleNamespace(frame_id='map'),
                child_frame_id='odom',
            ),
        ],
    )
    state.observe_tf(duplicate_tf)
    assert state.failure_stage() == 'map_odom_tf'

    unique_tf = SimpleNamespace(
        transforms=[SimpleNamespace(
            header=SimpleNamespace(frame_id='map'),
            child_frame_id='odom',
        )],
    )
    state.observe_tf(unique_tf)
    assert state.is_ready()


def test_navigation_state_requires_all_lifecycle_nodes_before_map_and_tf():
    """Require every approved Navigation lifecycle dependency."""
    readiness = _load_mode_readiness_module()
    state = readiness._ModeReadinessState('navigation')

    for node_name in readiness._MODE_LIFECYCLE_NODES['navigation']:
        assert state.failure_stage() == f'{node_name}_lifecycle'
        state.observe_lifecycle(node_name, readiness.PRIMARY_STATE_ACTIVE)

    assert state.failure_stage() == 'map'
    state.observe_map(SimpleNamespace(
        info=SimpleNamespace(width=1, height=1), data=[0],
    ))
    assert state.failure_stage() == 'map_odom_tf'
    state.observe_tf(SimpleNamespace(
        transforms=[SimpleNamespace(
            header=SimpleNamespace(frame_id='map'),
            child_frame_id='odom',
        )],
    ))
    assert state.is_ready()


def test_mode_readiness_can_require_a_matching_runtime_mode_snapshot():
    """Do not release a voice frontend before Runtime exposes its mode."""
    readiness = _load_mode_readiness_module()
    state = readiness._ModeReadinessState('navigation', require_runtime=True)

    for node_name in readiness._MODE_LIFECYCLE_NODES['navigation']:
        state.observe_lifecycle(node_name, readiness.PRIMARY_STATE_ACTIVE)
    state.observe_map(SimpleNamespace(
        info=SimpleNamespace(width=1, height=1), data=[0],
    ))
    state.observe_tf(SimpleNamespace(
        transforms=[SimpleNamespace(
            header=SimpleNamespace(frame_id='map'),
            child_frame_id='odom',
        )],
    ))

    assert not state.is_ready()
    state.observe_runtime(SimpleNamespace(
        operating_mode=readiness.RUNTIME_MODE_MAPPING,
        availability=readiness.RUNTIME_AVAILABLE,
        gate_state=readiness.GATE_INHIBITED,
    ))
    assert not state.is_ready()
    state.observe_runtime(SimpleNamespace(
        operating_mode=readiness.RUNTIME_MODE_NAVIGATION,
        availability=readiness.RUNTIME_AVAILABLE,
        gate_state=readiness.GATE_INHIBITED,
    ))
    assert state.is_ready()


def test_motion_mode_readiness_respects_an_expired_shared_deadline():
    """Fail closed before creating a readiness observer after the deadline."""
    readiness = _load_mode_readiness_module()
    result = readiness.wait_for_mode_readiness(
        SimpleNamespace(mode='motion'), 0.0, lambda: 0.0,
    )

    assert result == {
        'status': 'unavailable',
        'reason': 'mode_readiness_timeout',
        'mode': 'motion',
        'stage': 'deadline',
    }


def test_product_frontend_accepts_a_bounded_noncanonical_pcm_wav(tmp_path):
    """Accept a supported bounded WAV without requiring the locked fixture."""
    sensevoice_input = _load_sensevoice_input_module()
    wav = tmp_path / 'noncanonical.wav'
    _write_pcm_wav(wav, frames=1600)

    assert sensevoice_input.validate_input_wav(str(wav)) == {
        'status': 'ready',
        'reason': '',
    }


def test_vad_auto_frontend_readiness_uses_existing_publisher_identity():
    """Recognize the real SpeechInputNode and no look-alike graph owner."""
    sensevoice_input = _load_sensevoice_input_module()

    assert sensevoice_input._has_voice_frontend_publisher([
        SimpleNamespace(
            node_name='voice_speech_input', node_namespace='/',
        ),
    ])
    assert not sensevoice_input._has_voice_frontend_publisher([
        SimpleNamespace(node_name='speech_input_node', node_namespace='/'),
    ])


def test_frontend_readiness_accepts_a_publisher_seen_on_the_first_poll(monkeypatch):
    """Do not require a second poll when the frontend is already ready."""
    sensevoice_input = _load_sensevoice_input_module()
    events = []

    class FakeNode:
        def get_publishers_info_by_topic(self, topic):
            assert topic == '/voice/turn'
            return [SimpleNamespace(
                node_name='voice_speech_input', node_namespace='/',
            )]

        def destroy_node(self):
            events.append('destroy')

    class FakeRclpy:
        def __init__(self):
            self._ok = False

        def ok(self):
            return self._ok

        def init(self, args=None):
            del args
            self._ok = True
            events.append('init')

        def create_node(self, name):
            assert name == 'voice_nav_app_frontend_readiness'
            return FakeNode()

        def spin_once(self, *args, **kwargs):
            del args, kwargs
            events.append('spin')

        def shutdown(self):
            self._ok = False
            events.append('shutdown')

    monkeypatch.setitem(__import__('sys').modules, 'rclpy', FakeRclpy())
    result = sensevoice_input.wait_for_frontend_readiness(
        SimpleNamespace(poll=lambda: None),
        1.0,
        lambda: 0.0,
        lambda process: process.poll(),
        lambda status, reason='': {'status': status, 'reason': reason},
    )

    assert result == {'status': 'ready', 'reason': ''}
    assert events == ['init', 'spin', 'destroy', 'shutdown']


def test_product_frontend_rejects_empty_malformed_and_oversized_wav(tmp_path):
    """Reject invalid WAVs before any session or provider process starts."""
    sensevoice_input = _load_sensevoice_input_module()

    empty = tmp_path / 'empty.wav'
    empty.write_bytes(b'')
    assert sensevoice_input.validate_input_wav(str(empty))['reason'] == (
        'input_wav_empty'
    )

    malformed = tmp_path / 'malformed.wav'
    _write_pcm_wav(malformed, frames=1600)
    malformed.write_bytes(malformed.read_bytes()[:-2])
    assert sensevoice_input.validate_input_wav(str(malformed))['reason'] == (
        'input_wav_unsupported_format'
    )

    oversized = tmp_path / 'oversized.wav'
    _write_pcm_wav(oversized, frames=240001)
    assert sensevoice_input.validate_input_wav(str(oversized))['reason'] == (
        'input_wav_too_large'
    )


def test_motion_readiness_requires_typed_safe_stationary_barrier():
    """Require all motion evidence and the existing safe hold window."""
    readiness = _load_mode_readiness_module()
    now = [0.0]
    state = _configured_motion_readiness(readiness, now)
    state.observe_odometry(_odometry_sample())

    assert not state.is_ready()
    now[0] = readiness.MOTION_SAFE_STATIONARY_HOLD_S
    assert not state.is_ready()


def test_motion_readiness_accepts_continuous_fresh_stationary_odometry():
    """Require fresh stationary odometry throughout the hold window."""
    readiness = _load_mode_readiness_module()
    now = [0.0]
    state = _configured_motion_readiness(readiness, now)
    stationary = _odometry_sample()
    state.observe_odometry(stationary)

    for sample_index in range(1, 101):
        now[0] = sample_index * 0.02
        state.observe_odometry(stationary)

    assert state.is_ready()


def test_motion_readiness_resets_after_stale_odometry_gap():
    """Do not carry a stationary window across a stale odometry gap."""
    readiness = _load_mode_readiness_module()
    now = [0.0]
    state = _configured_motion_readiness(readiness, now)
    stationary = _odometry_sample()
    state.observe_odometry(stationary)

    now[0] = readiness.MOTION_SAFE_STATIONARY_HOLD_S + (
        readiness._MOTION_ODOMETRY_FRESHNESS_S + 0.01
    )
    state.observe_odometry(stationary)

    assert not state.is_ready()


def test_motion_readiness_resets_after_nonstationary_odometry_sample():
    """Reset the hold window when a fresh odometry sample is moving."""
    readiness = _load_mode_readiness_module()
    now = [0.0]
    state = _configured_motion_readiness(readiness, now)
    stationary = _odometry_sample()
    state.observe_odometry(stationary)
    for sample_index in range(1, 51):
        now[0] = sample_index * 0.02
        state.observe_odometry(stationary)

    moving = _odometry_sample(0.2)
    state.observe_odometry(moving)
    for sample_index in range(51, 150):
        now[0] = sample_index * 0.02
        state.observe_odometry(stationary)

    assert not state.is_ready()


def test_mode_readiness_timeout_is_bounded_with_mode_and_stage():
    """Expose only stable mode/stage diagnostics when the budget is gone."""
    readiness = _load_mode_readiness_module()
    result = readiness.wait_for_mode_readiness(
        SimpleNamespace(mode='mapping'), 0.0, lambda: 0.0,
    )

    assert result == {
        'status': 'unavailable',
        'reason': 'mode_readiness_timeout',
        'mode': 'mapping',
        'stage': 'deadline',
    }


class _SlowFakeProcess(_FakeProcess):
    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        if timeout == 10.0:
            raise TimeoutError('graceful shutdown still running')
        self.returncode = 0
        return self.returncode


class _StubbornFakeProcess(_FakeProcess):
    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        if timeout is not None:
            raise TimeoutError('shutdown still running')
        self.returncode = -signal.SIGKILL
        return self.returncode


class _SignalFailureFakeProcess(_FakeProcess):
    def send_group_signal(self, signum):
        del signum
        raise OSError('group signal failed')


def test_app_starts_fixed_session_waits_ready_then_enters_existing_console():
    """Compose the fixed session and console through injected seams."""
    app = _load_app_module()
    process = _FakeProcess()
    starts = []
    readiness_timeouts = []
    console_calls = []
    stdout = StringIO()
    stderr = StringIO()

    def process_factory(command, **kwargs):
        starts.append((tuple(command), kwargs))
        return process

    def readiness(timeout_s, clock):
        del clock
        readiness_timeouts.append(timeout_s)
        return {'status': 'ready', 'reason': 'ignored'}

    def console_main(*, stdin, stdout):
        console_calls.append((stdin, stdout))
        return 0

    result = app.main(
        [],
        process_factory=process_factory,
        readiness=readiness,
        clock=lambda: 0.0,
        console_main=console_main,
        stdin=StringIO(),
        stdout=stdout,
        stderr=stderr,
    )

    assert result == 0
    assert [command for command, _kwargs in starts] == [
        (
            'ros2', 'launch', 'voice_nav_bringup',
            'voice_nav_session.launch.py',
            'mode:=motion',
            'headless:=true',
            'shutdown_on_gazebo_exit:=true',
        ),
    ]
    assert readiness_timeouts == [60.0]
    assert stdout.getvalue() == '{"reason":"","status":"ready"}\n'
    assert len(console_calls) == 1
    assert console_calls[0][0].getvalue() == ''
    assert console_calls[0][1] is stdout


def test_motion_console_keeps_existing_gateway_only_startup_path():
    """Do not put the SenseVoice motion barrier in front of console input."""
    app = _load_app_module()
    process = _FakeProcess()

    def unexpected_mode_readiness(*_args):
        raise AssertionError(
            'motion console must not use SenseVoice readiness'
        )

    assert app.main(
        [],
        process_factory=lambda *_args, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        mode_readiness=unexpected_mode_readiness,
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    ) == 0


def test_navigation_headless_uses_one_closed_session_spec_before_console():
    """Map navigation CLI input to one fixed launch command before spawn."""
    app = _load_app_module()
    process = _FakeProcess()
    starts = []

    def process_factory(command, **kwargs):
        starts.append((tuple(command), kwargs))
        return process

    result = app.main(
        ['--mode', 'navigation', '--display', 'headless'],
        process_factory=process_factory,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        mode_readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 0
    assert [command for command, _kwargs in starts] == [
        (
            'ros2', 'launch', 'voice_nav_bringup',
            'voice_nav_session.launch.py',
            'mode:=navigation',
            'headless:=true',
            'shutdown_on_gazebo_exit:=true',
        ),
    ]


def test_mapping_headless_uses_the_same_session_composition_root():
    """Select Mapping through the closed session mode, not a raw launch."""
    app = _load_app_module()
    process = _FakeProcess()
    starts = []

    def process_factory(command, **kwargs):
        starts.append((tuple(command), kwargs))
        return process

    result = app.main(
        ['--mode', 'mapping', '--display', 'headless'],
        process_factory=process_factory,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        mode_readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 0
    assert [command for command, _kwargs in starts] == [
        (
            'ros2', 'launch', 'voice_nav_bringup',
            'voice_nav_session.launch.py',
            'mode:=mapping',
            'headless:=true',
            'shutdown_on_gazebo_exit:=true',
        ),
    ]


def test_mapping_dependency_failure_is_bounded_without_ready_or_console():
    """Do not enter Mapping console when gateway alone is ready."""
    app = _load_app_module()
    process = _FakeProcess()
    console_calls = []
    stdout = StringIO()

    result = app.main(
        ['--mode', 'mapping'],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        mode_readiness=lambda spec, _deadline, _clock: {
            'status': 'unavailable',
            'reason': 'mode_readiness_timeout',
            'mode': spec.mode,
            'stage': 'map',
        },
        console_main=lambda **_kwargs: console_calls.append(True),
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert console_calls == []
    assert process.group_signals == [signal.SIGINT]
    assert stdout.getvalue() == (
        '{"mode":"mapping","reason":"mode_readiness_timeout",'
        '"stage":"map","status":"unavailable"}\n'
    )


def test_navigation_dependency_failure_is_bounded_without_ready_or_console():
    """Do not enter Navigation console when gateway alone is ready."""
    app = _load_app_module()
    process = _FakeProcess()
    console_calls = []
    stdout = StringIO()

    result = app.main(
        ['--mode', 'navigation'],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        mode_readiness=lambda spec, _deadline, _clock: {
            'status': 'unavailable',
            'reason': 'mode_readiness_timeout',
            'mode': spec.mode,
            'stage': 'controller_server_lifecycle',
        },
        console_main=lambda **_kwargs: console_calls.append(True),
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert console_calls == []
    assert process.group_signals == [signal.SIGINT]
    assert stdout.getvalue() == (
        '{"mode":"navigation","reason":"mode_readiness_timeout",'
        '"stage":"controller_server_lifecycle",'
        '"status":"unavailable"}\n'
    )


def test_gateway_and_mode_readiness_share_one_total_deadline():
    """Pass one bounded budget through both readiness stages."""
    app = _load_app_module()
    process = _FakeProcess()
    gateway_deadlines = []
    mode_deadlines = []

    def gateway_readiness(timeout_s, _clock):
        gateway_deadlines.append(timeout_s)
        return {'status': 'ready', 'reason': ''}

    def mode_readiness(spec, deadline, _clock):
        assert spec.mode == 'navigation'
        mode_deadlines.append(deadline)
        return {'status': 'ready', 'reason': ''}

    result = app.main(
        ['--mode', 'navigation'],
        process_factory=lambda _command, **_kwargs: process,
        readiness=gateway_readiness,
        mode_readiness=mode_readiness,
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 0
    assert gateway_deadlines == [60.0]
    assert mode_deadlines == [60.0]


def test_gui_display_maps_to_one_fixed_non_headless_session_argument():
    """Map GUI only to the approved launch display argument."""
    app = _load_app_module()
    process = _FakeProcess()
    starts = []

    def process_factory(command, **kwargs):
        starts.append((tuple(command), kwargs))
        return process

    result = app.main(
        ['--mode', 'mapping', '--display', 'gui'],
        process_factory=process_factory,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        mode_readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 0
    assert [command for command, _kwargs in starts] == [
        (
            'ros2', 'launch', 'voice_nav_bringup',
            'voice_nav_session.launch.py',
            'mode:=mapping',
            'headless:=false',
            'shutdown_on_gazebo_exit:=true',
        ),
    ]


def test_invalid_mode_or_display_is_rejected_before_process_spawn():
    """Keep the CLI enums closed before creating an owned process group."""
    app = _load_app_module()

    for argv in (
        ['--mode', 'teleoperation'],
        ['--display', 'tui'],
    ):
        starts = []
        result = app.main(
            argv,
            process_factory=(
                lambda *args, **kwargs: starts.append((args, kwargs))
            ),
            stdout=StringIO(),
            stderr=StringIO(),
        )

        assert result == 2
        assert starts == []


def test_input_matrix_rejects_invalid_values_before_process_spawn(tmp_path):
    """Reject the closed input/path matrix before creating an owned process."""
    app = _load_app_module()
    wav = tmp_path / 'input.wav'
    _write_pcm_wav(wav)
    directory = tmp_path / 'wav-directory'
    directory.mkdir()
    invalid_argv = (
        ['--input', 'microphone'],
        ['--input', 'none'],
        ['--input', 'console', '--input-wav', str(wav.resolve())],
        ['--input', 'sensevoice-wav'],
        ['--input', 'sensevoice-wav', '--input-wav', 'relative.wav'],
        ['--input', 'sensevoice-wav', '--input-wav', str(directory.resolve())],
        [
            '--input', 'sensevoice-wav',
            '--input-wav', str(tmp_path / 'missing.wav'),
        ],
    )

    for argv in invalid_argv:
        starts = []
        result = app.main(
            argv,
            process_factory=(
                lambda *args, **kwargs: starts.append((args, kwargs))
            ),
            stdout=StringIO(),
            stderr=StringIO(),
        )

        assert result != 0
        assert starts == []


def test_input_matrix_keeps_console_default_and_selects_one_wav_frontend(
    tmp_path,
):
    """Select console by default or one bounded WAV frontend."""
    app = _load_app_module()
    wav = tmp_path / 'input.wav'
    _write_pcm_wav(wav)
    starts = []

    def process_factory(command, **kwargs):
        starts.append((tuple(command), kwargs))
        return _FakeProcess()

    def readiness(*_args):
        return {'status': 'ready', 'reason': ''}

    assert app.main(
        [],
        process_factory=process_factory,
        readiness=readiness,
        mode_readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    ) == 0
    assert starts[-1][0] == (
        'ros2', 'launch', 'voice_nav_bringup',
        'voice_nav_session.launch.py',
        'mode:=motion', 'headless:=true',
        'shutdown_on_gazebo_exit:=true',
    )

    assert app.main(
        [
            '--mode', 'motion', '--display', 'headless',
            '--input', 'sensevoice-wav', '--input-wav', str(wav.resolve()),
        ],
        process_factory=process_factory,
        readiness=readiness,
        mode_readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        frontend_factory=(
            lambda command, **kwargs: process_factory(command, **kwargs)
        ),
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    ) == 0
    assert starts[-2][0] == (
        'ros2', 'launch', 'voice_nav_bringup',
        'voice_nav_session.launch.py',
        'mode:=motion', 'headless:=true',
        'shutdown_on_gazebo_exit:=true',
        'input:=none',
    )
    assert starts[-1][0] == (
        'ros2', 'launch', 'voice_nav_audio', 'voice_node.launch.py',
        'input_profile:=sensevoice_wav',
        f'input_wav:={wav.resolve()}', 'include_agent:=false',
    )
    assert not any(
        'model' in argument or 'vad' in argument
        for command, _kwargs in starts[-2:]
        for argument in command
    )


def test_microphone_once_uses_one_closed_voice_composition_without_forwarding(
):
    """Stage one bounded microphone frontend without child tuning arguments."""
    app = _load_app_module()
    session_process = _FakeProcess()
    frontend_process = _FakeProcess()
    starts = []

    def process_factory(command, **kwargs):
        starts.append((tuple(command), kwargs))
        return session_process

    def frontend_factory(command, **kwargs):
        starts.append((tuple(command), kwargs))
        return frontend_process

    result = app.main(
        [
            '--mode', 'motion', '--display', 'headless',
            '--input', 'microphone-once',
        ],
        process_factory=process_factory,
        frontend_factory=frontend_factory,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        mode_readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 0
    assert starts[0][0] == (
        'ros2', 'launch', 'voice_nav_bringup',
        'voice_nav_session.launch.py',
        'mode:=motion', 'headless:=true',
        'shutdown_on_gazebo_exit:=true', 'input:=none',
    )
    assert starts[1][0] == (
        'ros2', 'launch', 'voice_nav_audio', 'voice_node.launch.py',
        'input_profile:=microphone_once',
        'include_agent:=false',
    )
    assert starts[1][0].count('include_agent:=false') == 1
    assert not any(
        any(
            token.startswith(prefix)
            for prefix in (
                'input_wav:=', 'output_wav:=', 'silero_vad_model:=',
                'sensevoice_model:=', 'sensevoice_tokens:=',
                'threshold:=', 'device:=', 'ros__parameters:=',
            )
        )
        for command, _kwargs in starts
        for token in command
    )


def test_vad_auto_uses_dedicated_frontend_and_never_console():
    """Keep continuous VAD closed and prevent stdin text passthrough."""
    app = _load_app_module()
    session_process = _FakeProcess()
    frontend_process = _FakeProcess()
    starts = []
    readiness_checks = []
    console_calls = []
    stdin = StringIO('\n普通文本\n:quit\n')

    def process_factory(command, **kwargs):
        starts.append((tuple(command), kwargs))
        return session_process

    def frontend_factory(command, **kwargs):
        starts.append((tuple(command), kwargs))
        return frontend_process

    result = app.main(
        [
            '--mode', 'motion', '--display', 'headless',
            '--input', 'vad-auto',
        ],
        process_factory=process_factory,
        frontend_factory=frontend_factory,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        frontend_readiness=lambda process, *_args: (
            readiness_checks.append(process)
            or {'status': 'ready', 'reason': ''}
        ),
        mode_readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: console_calls.append(True),
        stdin=stdin,
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 0
    assert console_calls == []
    assert readiness_checks == [frontend_process]
    assert starts[0][0][-1] == 'input:=none'
    assert starts[1][0] == (
        'ros2', 'run', 'voice_nav_audio', 'voice_node',
        '--ros-args', '-p', 'input_profile:=vad_auto',
    )
    assert 'stdin' not in starts[1][1]


def test_vad_auto_selects_the_same_frontend_after_each_product_mode_is_ready():
    """Keep Mapping and Navigation on one ordered VAD composition path."""
    app = _load_app_module()

    for mode in ('mapping', 'navigation'):
        with_mode_starts = []
        session_process = _FakeProcess()
        frontend_process = _FakeProcess()

        def process_factory(command, **kwargs):
            with_mode_starts.append((tuple(command), kwargs))
            return session_process

        def frontend_factory(command, **kwargs):
            with_mode_starts.append((tuple(command), kwargs))
            return frontend_process

        assert app.main(
            ['--mode', mode, '--display', 'headless', '--input', 'vad-auto'],
            process_factory=process_factory,
            frontend_factory=frontend_factory,
            readiness=lambda *_args: {'status': 'ready', 'reason': ''},
            frontend_readiness=lambda *_args: {
                'status': 'ready', 'reason': '',
            },
            mode_readiness=lambda spec, *_args: (
                {'status': 'ready', 'reason': ''}
                if spec.mode == mode else {'status': 'unavailable'}
            ),
            clock=lambda: 0.0,
            stdout=StringIO(),
            stderr=StringIO(),
        ) == 0

        assert with_mode_starts[0][0] == (
            'ros2', 'launch', 'voice_nav_bringup',
            'voice_nav_session.launch.py', f'mode:={mode}',
            'headless:=true', 'shutdown_on_gazebo_exit:=true', 'input:=none',
        )
        assert with_mode_starts[1][0] == (
            'ros2', 'run', 'voice_nav_audio', 'voice_node',
            '--ros-args', '-p', 'input_profile:=vad_auto',
        )


def test_vad_auto_frontend_failure_is_unavailable_before_ready():
    """Do not invite speech until the continuous publisher is observable."""
    app = _load_app_module()
    session_process = _FakeProcess()
    frontend_process = _FakeProcess()
    stdout = StringIO()

    result = app.main(
        [
            '--mode', 'motion', '--display', 'headless',
            '--input', 'vad-auto',
        ],
        process_factory=lambda *_args, **_kwargs: session_process,
        frontend_factory=lambda *_args, **_kwargs: frontend_process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        frontend_readiness=lambda *_args: {
            'status': 'unavailable',
            'reason': 'vad_auto_frontend_readiness_timeout',
        },
        mode_readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert '"status":"ready"' not in stdout.getvalue()
    assert stdout.getvalue() == (
        '{"reason":"vad_auto_frontend_readiness_timeout",'
        '"status":"unavailable"}\n'
    )
    assert frontend_process.group_signals == [signal.SIGINT]
    assert session_process.group_signals == [signal.SIGINT]


def test_vad_auto_waits_for_child_after_startup_deadline():
    """Do not apply the readiness deadline to the continuous VAD child."""
    app = _load_app_module()
    session_process = _FakeProcess()
    frontend_process = _InteractiveFakeProcess()
    now = [0.0]

    def readiness(*_args):
        now[0] = 61.0
        return {'status': 'ready', 'reason': ''}

    result = app.main(
        [
            '--mode', 'motion', '--display', 'headless',
            '--input', 'vad-auto',
        ],
        process_factory=lambda *_args, **_kwargs: session_process,
        frontend_factory=lambda *_args, **_kwargs: frontend_process,
        readiness=readiness,
        mode_readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        clock=lambda: now[0],
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 0
    assert frontend_process.wait_timeouts == [None]


def test_vad_auto_ctrl_c_returns_130_from_child_wait():
    """Propagate Ctrl+C while retaining bounded owned-process cleanup."""
    app = _load_app_module()
    session_process = _FakeProcess()
    frontend_process = _InterruptingInteractiveFakeProcess()

    result = app.main(
        [
            '--mode', 'motion', '--display', 'headless',
            '--input', 'vad-auto',
        ],
        process_factory=lambda *_args, **_kwargs: session_process,
        frontend_factory=lambda *_args, **_kwargs: frontend_process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        mode_readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 130
    assert frontend_process.wait_timeouts[0] is None
    assert frontend_process.group_signals == [signal.SIGINT]


def test_sensevoice_output_wav_forwards_only_output_and_locked_tts_root(
    monkeypatch, tmp_path,
):
    """Pass the output contract and locked TTS root without audio details."""
    app = _load_app_module()
    wav = tmp_path / 'input.wav'
    output = tmp_path / 'reply.wav'
    tts_root = tmp_path / 'chaowen'
    tts_root.mkdir()
    _write_pcm_wav(wav)
    monkeypatch.setenv('VOICE_NAV_CHAOWEN_TTS_ROOT', str(tts_root.resolve()))
    monkeypatch.setattr(
        app,
        '_chaowen_asset_verifier_module',
        lambda: {
            'verify_chaowen_root': lambda _root: {
                'status': 'ready', 'reason': '',
            },
        },
    )
    starts = []

    def process_factory(command, **kwargs):
        starts.append((tuple(command), kwargs))
        return _FakeProcess()

    assert app.main(
        [
            '--input', 'sensevoice-wav', '--input-wav', str(wav.resolve()),
            '--output-wav', str(output.resolve()),
        ],
        process_factory=process_factory,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        mode_readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        frontend_factory=lambda command, **kwargs: process_factory(command, **kwargs),
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    ) == 0

    assert starts[-1][0] == (
        'ros2', 'launch', 'voice_nav_audio', 'voice_node.launch.py',
        'input_profile:=sensevoice_wav', f'input_wav:={wav.resolve()}',
        f'output_wav:={output.resolve()}',
        f'chaowen_tts_root:={tts_root.resolve()}', 'include_agent:=false',
    )


def test_tampered_chaowen_asset_is_rejected_before_process_spawn(
    monkeypatch, tmp_path,
):
    """Reject a same-sized changed runtime file before creating a child."""
    verifier = _load_chaowen_asset_verifier_module()
    tts_root = tmp_path / 'chaowen'
    tts_root.mkdir()
    asset_names = (
        'model.onnx', 'lexicon.txt', 'tokens.txt', 'phone.fst', 'date.fst',
        'number.fst',
    )
    for name in asset_names:
        (tts_root / name).write_bytes(b'safe')
    monkeypatch.setattr(
        verifier,
        '_EXPECTED_FILES',
        tuple(
            (name, 4, hashlib.sha256(b'safe').hexdigest())
            for name in asset_names
        ),
    )
    asset = tts_root / 'model.onnx'
    asset.write_bytes(b'tamp')

    app = _load_app_module()
    monkeypatch.setenv('VOICE_NAV_CHAOWEN_TTS_ROOT', str(tts_root.resolve()))
    wav = tmp_path / 'input.wav'
    _write_pcm_wav(wav)
    monkeypatch.setattr(
        app,
        '_chaowen_asset_verifier_module',
        lambda: verifier.__dict__,
    )
    starts = []
    stdout = StringIO()

    result = app.main(
        [
            '--input', 'sensevoice-wav', '--input-wav', str(wav.resolve()),
            '--output-wav', str((tmp_path / 'reply.wav').resolve()),
        ],
        process_factory=lambda *args, **kwargs: starts.append((args, kwargs)),
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 2
    assert starts == []
    assert 'chaowen_tts_asset_sha256_mismatch:model.onnx' in stdout.getvalue()


def test_output_wav_path_is_rejected_before_process_spawn(tmp_path):
    """Reject output paths that could create partial or ambiguous artifacts."""
    app = _load_app_module()
    wav = tmp_path / 'input.wav'
    _write_pcm_wav(wav)
    existing = tmp_path / 'existing.wav'
    existing.write_bytes(b'keep')
    starts = []

    for output in (
        'relative.wav',
        str(existing.resolve()),
        str((tmp_path / 'missing-parent' / 'reply.wav').resolve()),
    ):
        result = app.main(
            [
                '--input', 'sensevoice-wav', '--input-wav', str(wav.resolve()),
                '--output-wav', output,
            ],
            process_factory=lambda *args, **kwargs: starts.append((args, kwargs)),
            stdout=StringIO(),
            stderr=StringIO(),
        )
        assert result == 2

    assert starts == []
    assert existing.read_bytes() == b'keep'


def test_sensevoice_provider_is_not_started_when_mode_readiness_is_blocked(
    tmp_path,
):
    """Stage the provider behind mode readiness and clean the session."""
    app = _load_app_module()
    wav = tmp_path / 'input.wav'
    _write_pcm_wav(wav)
    session_process = _FakeProcess()
    frontend_starts = []

    def frontend_factory(*args, **kwargs):
        frontend_starts.append((args, kwargs))
        return _FakeProcess()

    result = app.main(
        [
            '--input', 'sensevoice-wav', '--input-wav', str(wav.resolve()),
        ],
        process_factory=lambda *_args, **_kwargs: session_process,
        frontend_factory=frontend_factory,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        mode_readiness=lambda *_args: {
            'status': 'unavailable',
            'reason': 'mode_readiness_timeout',
            'mode': 'motion',
            'stage': 'deadline',
        },
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 1
    assert frontend_starts == []
    assert session_process.group_signals == [signal.SIGINT]


def test_sensevoice_provider_is_not_started_until_agent_input_sink_is_ready(
    tmp_path,
):
    """Block the one-shot provider until the long-lived Agent sink is ready."""
    app = _load_app_module()
    wav = tmp_path / 'input.wav'
    _write_pcm_wav(wav)
    session_process = _FakeProcess()
    frontend_starts = []
    events = []

    def mode_readiness(*_args):
        events.append('mode-ready')
        return {'status': 'ready', 'reason': ''}

    def input_sink_readiness(timeout_s, _clock):
        assert timeout_s == 60.0
        events.append('input-sink-check')
        return {
            'status': 'unavailable',
            'reason': 'sensevoice_wav_input_sink_timeout',
        }

    result = app.main(
        [
            '--input', 'sensevoice-wav', '--input-wav', str(wav.resolve()),
        ],
        process_factory=lambda *_args, **_kwargs: session_process,
        frontend_factory=lambda *args, **kwargs: frontend_starts.append(
            (args, kwargs)
        ),
        readiness=input_sink_readiness,
        mode_readiness=mode_readiness,
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 1
    assert events == ['mode-ready', 'input-sink-check']
    assert frontend_starts == []
    assert session_process.group_signals == [signal.SIGINT]


def test_sensevoice_provider_starts_once_only_after_input_sink_readiness(
    tmp_path,
):
    """Wait for the Agent sink before starting one provider."""
    app = _load_app_module()
    wav = tmp_path / 'input.wav'
    _write_pcm_wav(wav)
    session_process = _FakeProcess()
    frontend_process = _FakeProcess()
    events = []

    def mode_readiness(*_args):
        events.append('mode-ready')
        return {'status': 'ready', 'reason': ''}

    def frontend_factory(command, **kwargs):
        events.append(('frontend-start', tuple(command)))
        return frontend_process

    def input_sink_readiness(*_args):
        events.append('input-sink-ready')
        return {'status': 'ready', 'reason': ''}

    result = app.main(
        [
            '--input', 'sensevoice-wav', '--input-wav', str(wav.resolve()),
        ],
        process_factory=lambda *_args, **_kwargs: session_process,
        frontend_factory=frontend_factory,
        readiness=input_sink_readiness,
        mode_readiness=mode_readiness,
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 0
    assert events[0] == 'mode-ready'
    assert events[1] == 'input-sink-ready'
    assert events[2][0] == 'frontend-start'


def test_installed_console_fallback_loads_extensionless_existing_console(
    monkeypatch, tmp_path,
):
    """Load the existing console when its installed entry has no suffix."""
    app = _load_app_module()
    console = tmp_path / 'voice_nav_console'
    console.write_text(
        'class RosParameterTransport:\n'
        '    def close(self):\n'
        '        pass\n'
        'def main(_argv, *, transport, stdin, stdout):\n'
        '    del transport, stdin, stdout\n'
        '    return 0\n',
        encoding='utf-8',
    )
    original_import = builtins.__import__

    def import_without_console(name, *args, **kwargs):
        if name == 'voice_nav_console':
            raise ModuleNotFoundError(name=name)
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', import_without_console)
    monkeypatch.setattr(app, '__file__', str(tmp_path / 'voice_nav_app'))

    assert app._run_existing_console(
        stdin=StringIO(), stdout=StringIO(),
    ) == 0


def test_quit_cleans_owned_group_gracefully_then_terminates_after_budgets():
    """A :quit console exit uses the bounded two-phase group teardown."""
    app = _load_app_module()
    process = _SlowFakeProcess()
    stdout = StringIO()

    def process_factory(_command, **_kwargs):
        return process

    def readiness(_timeout_s, _clock):
        return {'status': 'ready', 'reason': ''}

    def quit_console(*, stdin, stdout):
        assert stdin.readline() == ':quit\n'
        del stdout
        return 0

    result = app.main(
        [],
        process_factory=process_factory,
        readiness=readiness,
        clock=lambda: 0.0,
        console_main=quit_console,
        stdin=StringIO(':quit\n'),
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert stdout.getvalue() == (
        '{"reason":"","status":"ready"}\n'
        '{"reason":"cleanup_terminated","stage":"terminated",'
        '"status":"unavailable"}\n'
    )
    assert process.group_signals == [signal.SIGINT, signal.SIGTERM]
    assert process.wait_timeouts == [10.0, 5.0]


def test_child_output_is_redirected_to_app_stderr_and_args_not_forwarded():
    """Keep child logs off JSON stdout and reject arbitrary app arguments."""
    app = _load_app_module()
    process = _FakeProcess()
    stderr = StringIO()
    starts = []

    def process_factory(command, **kwargs):
        starts.append((command, kwargs))
        return process

    def readiness(_timeout_s, _clock):
        return {'status': 'ready', 'reason': ''}

    assert app.main(
        ['headless:=false'],
        process_factory=process_factory,
        readiness=readiness,
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=stderr,
    ) != 0
    assert starts == []

    stdout = StringIO()
    result = app.main(
        [],
        process_factory=process_factory,
        readiness=readiness,
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=stderr,
    )

    assert result == 0
    assert starts[0][1]['stdout'] is stderr
    assert starts[0][1]['stderr'] is stderr
    assert stdout.getvalue() == '{"reason":"","status":"ready"}\n'


def test_startup_failure_is_structured_nonzero_and_does_not_enter_console():
    """Fail closed when the single fixed child cannot be started."""
    app = _load_app_module()
    stdout = StringIO()

    def process_factory(_command, **_kwargs):
        raise OSError('ros2 not found')

    console_calls = []
    result = app.main(
        [],
        process_factory=process_factory,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: console_calls.append(True),
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert console_calls == []
    assert stdout.getvalue() == (
        '{"reason":"session_start_failed:ros2 not found",'
        '"status":"unavailable"}\n'
    )


def test_readiness_failure_cleans_child_and_returns_nonzero():
    """Stop a started child when gateway readiness never becomes ready."""
    app = _load_app_module()
    process = _FakeProcess()
    stdout = StringIO()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {
            'status': 'unavailable',
            'reason': 'command_gateway_readiness_timeout',
        },
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert process.group_signals == [signal.SIGINT]
    assert stdout.getvalue() == (
        '{"reason":"command_gateway_readiness_timeout",'
        '"status":"unavailable"}\n'
    )


def test_session_exit_after_readiness_is_nonzero_without_console_or_signals():
    """Treat a session that exits before console entry as a failure."""
    app = _load_app_module()
    process = _FakeProcess()
    console_calls = []
    stdout = StringIO()

    def readiness(_timeout_s, _clock):
        process.returncode = 17
        return {'status': 'ready', 'reason': ''}

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=readiness,
        console_main=lambda **_kwargs: console_calls.append(True),
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert console_calls == []
    assert process.group_signals == []
    assert stdout.getvalue() == (
        '{"reason":"session_exited_before_ready",'
        '"status":"unavailable"}\n'
    )


def test_ctrl_c_returns_130_after_cleaning_the_owned_group():
    """Propagate Ctrl+C as 130 while still cleaning the session group."""
    app = _load_app_module()
    process = _FakeProcess()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=(
            lambda **_kwargs: (_ for _ in ()).throw(KeyboardInterrupt)
        ),
        clock=lambda: 0.0,
        stdout=StringIO(),
        stderr=StringIO(),
    )

    assert result == 130
    assert process.group_signals == [signal.SIGINT]


def test_ctrl_c_keeps_130_when_cleanup_escalates_to_kill():
    """Keep the signal exit code even when bounded cleanup needs SIGKILL."""
    app = _load_app_module()
    process = _StubbornFakeProcess()
    stdout = StringIO()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=(
            lambda **_kwargs: (_ for _ in ()).throw(KeyboardInterrupt)
        ),
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 130
    assert stdout.getvalue() == '{"reason":"","status":"ready"}\n'
    assert process.group_signals == [
        signal.SIGINT,
        signal.SIGTERM,
        signal.SIGKILL,
    ]


def test_console_nonzero_survives_forced_cleanup():
    """Do not replace an existing console failure with cleanup status."""
    app = _load_app_module()
    process = _StubbornFakeProcess()
    stdout = StringIO()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: 23,
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 23
    assert stdout.getvalue() == '{"reason":"","status":"ready"}\n'
    assert process.group_signals == [
        signal.SIGINT,
        signal.SIGTERM,
        signal.SIGKILL,
    ]


def test_cleanup_signal_failure_is_nonzero_with_bounded_failure_reason():
    """Report a failed cleanup stage without claiming a clean exit."""
    app = _load_app_module()
    process = _SignalFailureFakeProcess()
    stdout = StringIO()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert stdout.getvalue() == (
        '{"reason":"","status":"ready"}\n'
        '{"reason":"cleanup_failed","stage":"failed",'
        '"status":"unavailable"}\n'
    )


def test_stubborn_group_is_forced_after_both_bounded_shutdown_phases():
    """Use SIGKILL only after graceful and terminate budgets expire."""
    app = _load_app_module()
    process = _StubbornFakeProcess()
    stdout = StringIO()

    result = app.main(
        [],
        process_factory=lambda _command, **_kwargs: process,
        readiness=lambda *_args: {'status': 'ready', 'reason': ''},
        console_main=lambda **_kwargs: 0,
        clock=lambda: 0.0,
        stdout=stdout,
        stderr=StringIO(),
    )

    assert result == 1
    assert stdout.getvalue() == (
        '{"reason":"","status":"ready"}\n'
        '{"reason":"cleanup_killed","stage":"killed",'
        '"status":"unavailable"}\n'
    )
    assert process.group_signals == [
        signal.SIGINT,
        signal.SIGTERM,
        signal.SIGKILL,
    ]
    assert process.wait_timeouts == [10.0, 5.0, None]


def test_app_static_authority_is_limited_to_process_and_gateway_readiness():
    """Keep ROS motion and voice authority out of the app wrapper."""
    source = Path(__file__).resolve().parents[1] / 'voice_nav_app.py'
    source_text = source.read_text(encoding='utf-8')
    tree = ast.parse(source_text)
    allowed_roots = {
        '__future__', 'argparse', 'dataclasses', 'json', 'os',
        'rcl_interfaces', 'rclpy',
        'runpy', 'signal', 'subprocess', 'sys', 'time', 'typing',
        'voice_nav_console',
    }
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(
                alias.name.split('.')[0] for alias in node.names
            )
        elif isinstance(node, ast.ImportFrom) and node.module is not None:
            imported_roots.add(node.module.split('.')[0])

    assert imported_roots <= allowed_roots
    assert source_text.count("'ros2'") == 1
    assert source_text.count("'voice_nav_session.launch.py'") == 1
    assert 'READINESS_SETTLE_S' not in source_text
    assert 'time.sleep(' not in source_text
    for forbidden in (
        'VoiceTurn', 'Mission', 'StopMission', 'Twist', 'cmd_vel',
        'create_publisher', 'create_subscription', 'publish(',
    ):
        assert forbidden not in source_text
