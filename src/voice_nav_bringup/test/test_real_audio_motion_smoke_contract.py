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

"""Behavior contract for the real-audio Motion smoke runtime preflight."""

from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
from types import SimpleNamespace




def _load_contract_module():
    source = Path(__file__).resolve().parents[1] / '_real_audio_motion_smoke.py'
    specification = importlib.util.spec_from_file_location(
        'voice_nav_real_audio_motion_smoke', source,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _write_prefix(module, root: Path, ort_bytes: bytes) -> Path:
    prefix = root / 'sherpa-prefix'
    (prefix / 'share' / 'voice_nav').mkdir(parents=True)
    (prefix / 'include' / 'sherpa-onnx' / 'c-api').mkdir(parents=True)
    (prefix / 'lib').mkdir()
    (prefix / 'share' / 'voice_nav' / 'sherpa-onnx-provenance.receipt').write_text(
        module._EXACT_TTS_ON_RECEIPT, encoding='utf-8',
    )
    for target in (
        'include/sherpa-onnx/c-api/c-api.h',
        'lib/libsherpa-onnx-c-api.a',
        'lib/libsherpa-onnx-core.a',
    ):
        (prefix / target).write_bytes(b'contract fixture')
    (prefix / 'lib' / 'libonnxruntime.so').write_bytes(ort_bytes)
    return prefix


def test_runtime_contract_requires_pulse_and_exact_shared_tts_prefix(tmp_path):
    """Return bounded provenance only when both local runtime contracts pass."""
    module = _load_contract_module()
    ort_bytes = b'portable ort fixture'
    prefix = _write_prefix(module, tmp_path, ort_bytes)
    expected_identity = (
        len(ort_bytes),
        hashlib.sha256(ort_bytes).hexdigest(),
    )
    receipt = prefix / 'share' / 'voice_nav' / 'sherpa-onnx-provenance.receipt'

    def fake_run(command):
        outputs = {
            ('pactl', 'info'): (
                'Server String: unix:/mnt/wslg/PulseServer\n'
                'Default Sink: RDPSink\n'
                'Default Source: RDPSource\n'
            ),
            ('pactl', 'list', 'short', 'sources'):
                '2 RDPSource module-alsa-source.c s16le 1ch 44100Hz SUSPENDED\n',
            ('pactl', 'list', 'short', 'sinks'):
                '1 RDPSink module-alsa-sink.c s16le 2ch 44100Hz SUSPENDED\n',
        }
        stdout = outputs[tuple(command)]
        return SimpleNamespace(returncode=0, stdout=stdout, stderr='')

    receipt.write_text(
        module._EXACT_TTS_ON_RECEIPT.replace(
            'SHERPA_ONNX_ENABLE_TTS=ON', 'SHERPA_ONNX_ENABLE_TTS=OFF',
        ),
        encoding='utf-8',
    )

    result = module.inspect_runtime_contract(
        exact_head='d0eb8fd2064e4c710dc19aa3ae520b476005ac42',
        sherpa_prefix=prefix,
        run_command=fake_run,
        _expected_ort_identity=expected_identity,
    )

    assert result['ok'] is False
    assert result['reason'] == 'sherpa_tts_receipt_mismatch'
    assert result['capability']['server'] == 'unix:/mnt/wslg/PulseServer'
    assert result['capability']['default_source'] == 'RDPSource'
    assert result['capability']['default_sink'] == 'RDPSink'
    assert result['provenance']['exact_head'] == (
        'd0eb8fd2064e4c710dc19aa3ae520b476005ac42'
    )

    receipt.write_text(module._EXACT_TTS_ON_RECEIPT, encoding='utf-8')
    result = module.inspect_runtime_contract(
        exact_head='d0eb8fd2064e4c710dc19aa3ae520b476005ac42',
        sherpa_prefix=prefix,
        run_command=fake_run,
        _expected_ort_identity=expected_identity,
    )

    assert result['ok'] is True
    assert result['reason'] == ''
    assert result['provenance']['sherpa']['version'] == 'v1.13.4'
    assert result['provenance']['sherpa']['onnxruntime_version'] == '1.27.0'
    assert result['provenance']['sherpa']['tts'] == 'ON'
    assert result['provenance']['sherpa']['verified_onnxruntime_library_size'] == len(
        ort_bytes,
    )


def test_sherpa_prefix_verifier_rejects_tampered_fixture(tmp_path):
    """The package-private identity seam still detects fixture tampering."""
    module = _load_contract_module()
    ort_bytes = b'portable ort fixture'
    prefix = _write_prefix(module, tmp_path, ort_bytes)
    expected_size = len(ort_bytes)
    expected_sha256 = hashlib.sha256(ort_bytes).hexdigest()

    result = module.verify_sherpa_tts_prefix(
        prefix,
        _expected_ort_size=expected_size,
        _expected_ort_sha256=expected_sha256,
    )
    assert result['ok'] is True

    (prefix / 'lib' / 'libonnxruntime.so').write_bytes(b'tampered')
    result = module.verify_sherpa_tts_prefix(
        prefix,
        _expected_ort_size=expected_size,
        _expected_ort_sha256=expected_sha256,
    )
    assert result['ok'] is False
    assert result['reason'] == 'sherpa_tts_onnxruntime_size_mismatch'


def test_sherpa_prefix_verifier_defaults_to_production_identity(tmp_path):
    """Production calls retain the locked ORT size/SHA defaults."""
    module = _load_contract_module()
    prefix = _write_prefix(module, tmp_path, b'portable ort fixture')
    result = module.verify_sherpa_tts_prefix(prefix)
    assert result['ok'] is False
    assert result['reason'] == 'sherpa_tts_onnxruntime_size_mismatch'


def test_portaudio_source_output_ready_uses_observed_pulse_fields():
    """Readiness is source-bound and uncorked without inventing State output."""
    module = _load_contract_module()
    source_output = '''
Source Output #12
\tSource: 1
\tCorked: no
\tProperties:
\t\tapplication.name = "ALSA plug-in [voice_node]"
\t\tapplication.process.binary = "voice_node"
'''

    assert module._is_portaudio_source_output_ready(
        source_output,
        source_index=1,
    ) is True
    assert module._is_portaudio_source_output_ready(
        source_output.replace('\tSource: 1', '\tSource: 2'),
        source_index=1,
    ) is False
    assert module._is_portaudio_source_output_ready(
        source_output.replace('\tCorked: no', '\tCorked: yes'),
        source_index=1,
    ) is False
    assert module._is_portaudio_source_output_ready(
        source_output.replace('application.process.binary = "voice_node"',
                             'application.process.binary = "other-process"'),
        source_index=1,
    ) is True
    assert module._is_portaudio_source_output_ready(
        source_output.replace('ALSA plug-in [voice_node]', 'other-app'),
        source_index=1,
    ) is True
    assert module._is_portaudio_source_output_ready(
        source_output
        .replace('ALSA plug-in [voice_node]', 'other-app')
        .replace('application.process.binary = "voice_node"',
                 'application.process.binary = "other-process"'),
        source_index=1,
    ) is False
