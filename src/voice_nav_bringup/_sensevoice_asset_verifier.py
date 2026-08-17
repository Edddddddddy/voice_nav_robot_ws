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

"""Package-private verifier for the frozen SenseVoice/Silero runtime files."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import re
from typing import Mapping


_MANIFEST_FILENAME = '_sensevoice_runtime_asset_manifest.def'
_MANIFEST_ENTRY = re.compile(
    r'VOICE_NAV_SENSEVOICE_ASSET\(\s*'
    r'(SenseVoiceModel|Tokens|Vad)\s*,\s*'
    r'(\d+)U?\s*,\s*"([0-9a-f]{64})"\s*\)',
    re.MULTILINE,
)
_NAME_MAP = {
    'SenseVoiceModel': 'model',
    'Tokens': 'tokens',
    'Vad': 'vad',
}
_HASH_CHUNK_BYTES = 1024 * 1024


def _manifest_path() -> Path:
    installed_path = Path(__file__).with_name(_MANIFEST_FILENAME)
    if installed_path.is_file():
        return installed_path
    return (
        Path(__file__).resolve().parents[1]
        / 'voice_nav_audio'
        / 'src'
        / 'sensevoice_runtime_asset_manifest.def'
    )


def _load_expected_assets() -> dict[str, dict[str, object]]:
    try:
        entries = _MANIFEST_ENTRY.findall(
            _manifest_path().read_text(encoding='utf-8'),
        )
    except (OSError, UnicodeError):
        return {}
    if len(entries) != len(_NAME_MAP):
        return {}
    result = {
        _NAME_MAP[name]: {'size': int(size), 'sha256': digest}
        for name, size, digest in entries
    }
    return result if set(result) == set(_NAME_MAP.values()) else {}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(_HASH_CHUNK_BYTES), b''):
            digest.update(chunk)
    return digest.hexdigest()


def verify_sensevoice_assets(
    *,
    vad: Path,
    model: Path,
    tokens: Path,
    expected_assets: Mapping[str, Mapping[str, object]] | None = None,
) -> dict[str, object]:
    """Return exact size/SHA provenance for all three runtime files."""
    expected = dict(expected_assets) if expected_assets is not None else _load_expected_assets()
    if set(expected) != {'vad', 'model', 'tokens'}:
        return {
            'ok': False,
            'reason': 'sensevoice_asset_manifest_unavailable',
            'asset_provenance': {},
        }

    paths = {'vad': Path(vad), 'model': Path(model), 'tokens': Path(tokens)}
    provenance: dict[str, dict[str, object]] = {}
    for name, path in paths.items():
        entry = expected[name]
        expected_size = entry.get('size')
        expected_sha256 = entry.get('sha256')
        if (
            not isinstance(expected_size, int)
            or isinstance(expected_size, bool)
            or not isinstance(expected_sha256, str)
            or re.fullmatch(r'[0-9a-f]{64}', expected_sha256) is None
        ):
            return {
                'ok': False,
                'reason': 'sensevoice_asset_manifest_invalid',
                'asset_provenance': provenance,
            }
        if not path.is_absolute() or path.is_symlink() or not path.is_file():
            return {
                'ok': False,
                'reason': f'{name}_file_unavailable',
                'asset_provenance': provenance,
            }
        try:
            actual_size = path.stat().st_size
            actual_sha256 = _sha256(path)
        except OSError:
            return {
                'ok': False,
                'reason': f'{name}_file_unreadable',
                'asset_provenance': provenance,
            }
        provenance[name] = {
            'path': str(path.resolve()),
            'size': actual_size,
            'sha256': actual_sha256,
            'expected_size': expected_size,
            'expected_sha256': expected_sha256,
        }
        if actual_size != expected_size:
            return {
                'ok': False,
                'reason': f'{name}_size_mismatch',
                'asset_provenance': provenance,
            }
        if actual_sha256 != expected_sha256:
            return {
                'ok': False,
                'reason': f'{name}_sha256_mismatch',
                'asset_provenance': provenance,
            }
    return {'ok': True, 'reason': '', 'asset_provenance': provenance}
