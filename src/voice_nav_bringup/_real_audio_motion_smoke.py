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

"""Package-private preflight contracts for the real-audio Motion smoke."""

from __future__ import annotations

import hashlib
from pathlib import Path
import re
import subprocess
from typing import Callable, Sequence


_EXACT_TTS_ON_RECEIPT = """schema_version=2
id=sherpa-onnx
version=v1.13.4
revision=142807252687d81b40d6315f23470a1512a00de3
source_sha256=f0dc7c9b41b8691313daee671e826eb23946fa1320559a8d37e84f8774af76b2
onnxruntime_mode=shared
onnxruntime_version=1.27.0
onnxruntime_url=https://github.com/csukuangfj/onnxruntime-libs/releases/download/v1.27.0/onnxruntime-linux-x64-glibc2_17-Release-1.27.0.zip
onnxruntime_zip_size=8509524
onnxruntime_zip_sha256=9f0c0a6998f1b94c399eeddcb443beb4a922c9a4fd431fdc9cd6de67a1935d00
onnxruntime_git_commit=8f0278c77bf44b0cc83c098c6c722b92a36ac4b5
onnxruntime_license=MIT
onnxruntime_soname=libonnxruntime.so
onnxruntime_library=lib/libonnxruntime.so
onnxruntime_library_size=26403889
onnxruntime_library_sha256=026c7d5c609323fb16506dbc3cce801bcdffdd7566fdba49a50727e2e1e881ca
build_system=CMake
cxx_compiler=GNU 13.3.0
BUILD_SHARED_LIBS=OFF
SHERPA_ONNX_ENABLE_C_API=ON
SHERPA_ONNX_ENABLE_TESTS=OFF
SHERPA_ONNX_ENABLE_PORTAUDIO=OFF
SHERPA_ONNX_ENABLE_WEBSOCKET=OFF
SHERPA_ONNX_ENABLE_TTS=ON
SHERPA_ONNX_ENABLE_SPEAKER_DIARIZATION=OFF
SHERPA_ONNX_ENABLE_BINARY=OFF
SHERPA_ONNX_USE_PRE_INSTALLED_ONNXRUNTIME_IF_AVAILABLE=ON
c_api_header=include/sherpa-onnx/c-api/c-api.h
c_api_library=lib/libsherpa-onnx-c-api.a
core_library=lib/libsherpa-onnx-core.a
"""

_RECEIPT_RELATIVE_PATH = Path(
    'share', 'voice_nav', 'sherpa-onnx-provenance.receipt'
)
_REQUIRED_TARGETS = (
    Path('include', 'sherpa-onnx', 'c-api', 'c-api.h'),
    Path('lib', 'libsherpa-onnx-c-api.a'),
    Path('lib', 'libsherpa-onnx-core.a'),
    Path('lib', 'libonnxruntime.so'),
)
_ORT_LIBRARY_SIZE = 26403889
_ORT_LIBRARY_SHA256 = (
    '026c7d5c609323fb16506dbc3cce801bcdffdd7566fdba49a50727e2e1e881ca'
)
_HASH_CHUNK_BYTES = 1024 * 1024

MOTION_SMOKE_COMMAND = (
    'ros2',
    'run',
    'voice_nav_bringup',
    'voice_nav_app',
    '--mode',
    'motion',
    '--display',
    'headless',
    '--input',
    'microphone-once',
)



def _default_run_command(command: Sequence[str]):
    return subprocess.run(
        tuple(command),
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )


def _text(value: object) -> str:
    if isinstance(value, bytes):
        return value.decode('utf-8', errors='replace')
    return str(value or '')


def _is_portaudio_source_output_ready(
    source_outputs: str,
    *,
    source_index: int | str,
) -> bool:
    """Match the observable PulseAudio fields for the PortAudio input stream."""
    expected_source = str(source_index).strip()
    if not expected_source.isdigit():
        return False
    blocks = re.split(
        r'(?m)^[ \t]*Source Output #[0-9]+[ \t]*$',
        _text(source_outputs),
    )
    for block in blocks[1:]:
        source_match = re.search(
            r'(?m)^[ \t]*Source:[ \t]*([0-9]+)[ \t]*$',
            block,
        )
        corked_match = re.search(
            r'(?m)^[ \t]*Corked:[ \t]*(yes|no)[ \t]*$',
            block,
        )
        application_match = re.search(
            r'(?m)^[ \t]*application\.name[ \t]*=[ \t]*'
            r'"ALSA plug-in \[voice_node\]"[ \t]*$',
            block,
        )
        process_match = re.search(
            r'(?m)^[ \t]*application\.process\.binary[ \t]*='
            r'[ \t]*"voice_node"[ \t]*$',
            block,
        )
        if (
            source_match is not None
            and source_match.group(1) == expected_source
            and corked_match is not None
            and corked_match.group(1) == 'no'
            and (application_match is not None or process_match is not None)
        ):
            return True
    return False


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(_HASH_CHUNK_BYTES), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _key_values(receipt: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in receipt.splitlines():
        key, separator, value = line.partition('=')
        if not separator:
            key, separator, value = line.partition(':')
        if separator:
            values[key.strip()] = value.strip()
    return values


def _command_output(
    run_command: Callable[[Sequence[str]], object],
    command: Sequence[str],
) -> tuple[bool, str, str]:
    try:
        result = run_command(command)
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired) as error:
        return False, '', str(error)
    stdout = _text(getattr(result, 'stdout', ''))
    stderr = _text(getattr(result, 'stderr', ''))
    if getattr(result, 'returncode', 1) != 0:
        return False, stdout, stderr.strip() or 'command failed'
    return True, stdout, stderr


def probe_pulseaudio(
    *,
    run_command: Callable[[Sequence[str]], object] = _default_run_command,
) -> dict[str, object]:
    """Probe the WSLg PulseAudio contract without ALSA device heuristics."""
    commands = (
        ('pactl', 'info'),
        ('pactl', 'list', 'short', 'sources'),
        ('pactl', 'list', 'short', 'sinks'),
    )
    outputs: list[str] = []
    for command in commands:
        ok, stdout, error = _command_output(run_command, command)
        if not ok:
            return {
                'ok': False,
                'reason': 'pulseaudio_capability_unavailable',
                'error': error,
            }
        outputs.append(stdout)

    info = _key_values(outputs[0])
    server = info.get('Server String', '')
    default_source = info.get('Default Source', '')
    default_sink = info.get('Default Sink', '')
    source_lines = tuple(line for line in outputs[1].splitlines() if line.strip())
    sink_lines = tuple(line for line in outputs[2].splitlines() if line.strip())
    if (
        not server
        or not default_source
        or not default_sink
        or not any(default_source in line for line in source_lines)
        or not any(default_sink in line for line in sink_lines)
    ):
        return {
            'ok': False,
            'reason': 'pulseaudio_capability_unavailable',
            'error': 'PulseAudio server/default source/sink was incomplete',
        }
    return {
        'ok': True,
        'reason': '',
        'server': server,
        'default_source': default_source,
        'default_sink': default_sink,
        'sources': list(source_lines),
        'sinks': list(sink_lines),
    }


def build_motion_smoke_command() -> tuple[str, ...]:
    """Return the one approved app composition without child passthrough."""
    return MOTION_SMOKE_COMMAND


def verify_sherpa_tts_prefix(
    prefix: Path,
    *,
    _expected_ort_size: int = _ORT_LIBRARY_SIZE,
    _expected_ort_sha256: str = _ORT_LIBRARY_SHA256,
) -> dict[str, object]:
    """Verify the immutable receipt and its installed target paths."""
    prefix = Path(prefix)
    receipt_path = prefix / _RECEIPT_RELATIVE_PATH
    if not receipt_path.is_file():
        return {
            'ok': False,
            'reason': 'sherpa_tts_receipt_missing',
            'provenance': {},
        }
    try:
        receipt = receipt_path.read_text(encoding='utf-8')
    except (OSError, UnicodeError) as error:
        return {
            'ok': False,
            'reason': 'sherpa_tts_receipt_unreadable',
            'error': str(error),
            'provenance': {},
        }
    values = _key_values(receipt)
    provenance = {
        'version': values.get('version', ''),
        'onnxruntime_version': values.get('onnxruntime_version', ''),
        'tts': values.get('SHERPA_ONNX_ENABLE_TTS', ''),
        'onnxruntime_mode': values.get('onnxruntime_mode', ''),
        'onnxruntime_library_size': values.get('onnxruntime_library_size', ''),
        'onnxruntime_library_sha256': values.get('onnxruntime_library_sha256', ''),
        'receipt': str(receipt_path),
    }
    if receipt != _EXACT_TTS_ON_RECEIPT:
        return {
            'ok': False,
            'reason': 'sherpa_tts_receipt_mismatch',
            'provenance': provenance,
        }
    missing = [str(path) for path in _REQUIRED_TARGETS if not (prefix / path).is_file()]
    if missing:
        return {
            'ok': False,
            'reason': 'sherpa_tts_targets_missing',
            'missing_targets': missing,
            'provenance': provenance,
        }
    ort_library = prefix / 'lib' / 'libonnxruntime.so'
    if ort_library.is_symlink():
        return {
            'ok': False,
            'reason': 'sherpa_tts_onnxruntime_must_be_regular_file',
            'provenance': provenance,
        }
    try:
        actual_size = ort_library.stat().st_size
        actual_sha256 = _sha256(ort_library)
    except OSError as error:
        return {
            'ok': False,
            'reason': 'sherpa_tts_onnxruntime_unreadable',
            'error': str(error),
            'provenance': provenance,
        }
    if actual_size != _expected_ort_size:
        return {
            'ok': False,
            'reason': 'sherpa_tts_onnxruntime_size_mismatch',
            'provenance': {**provenance, 'actual_onnxruntime_library_size': actual_size},
        }
    if actual_sha256 != _expected_ort_sha256:
        return {
            'ok': False,
            'reason': 'sherpa_tts_onnxruntime_sha256_mismatch',
            'provenance': {
                **provenance,
                'actual_onnxruntime_library_sha256': actual_sha256,
            },
        }
    provenance['verified_onnxruntime_library_size'] = actual_size
    provenance['verified_onnxruntime_library_sha256'] = actual_sha256
    return {
        'ok': True,
        'reason': '',
        'provenance': provenance,
    }


def inspect_runtime_contract(
    *,
    exact_head: str,
    sherpa_prefix: Path,
    run_command: Callable[[Sequence[str]], object] = _default_run_command,
    _expected_ort_identity: tuple[int, str] | None = None,
) -> dict[str, object]:
    """Return bounded startup capability and dependency provenance."""
    capability = probe_pulseaudio(run_command=run_command)
    expected_size, expected_sha256 = (
        _expected_ort_identity
        if _expected_ort_identity is not None
        else (_ORT_LIBRARY_SIZE, _ORT_LIBRARY_SHA256)
    )
    sherpa = verify_sherpa_tts_prefix(
        sherpa_prefix,
        _expected_ort_size=expected_size,
        _expected_ort_sha256=expected_sha256,
    )
    provenance: dict[str, object] = {
        'exact_head': exact_head,
        'sherpa': sherpa.get('provenance', {}),
    }
    if not capability['ok']:
        return {
            'ok': False,
            'reason': 'pulseaudio_capability_unavailable',
            'capability': capability,
            'provenance': provenance,
        }
    if not sherpa['ok']:
        return {
            'ok': False,
            'reason': str(sherpa['reason']),
            'capability': capability,
            'provenance': provenance,
        }
    return {
        'ok': True,
        'reason': '',
        'capability': capability,
        'provenance': provenance,
    }
