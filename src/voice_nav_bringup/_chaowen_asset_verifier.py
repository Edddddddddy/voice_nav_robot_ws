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

"""Package-private verifier for the pinned Chaowen runtime files."""

from __future__ import annotations

import hashlib
import os
import re
from pathlib import Path


_HASH_CHUNK_BYTES = 1024 * 1024
_MANIFEST_FILENAME = 'chaowen_tts_asset_manifest.def'
_MANIFEST_ENTRY = re.compile(
    r'CHAOWEN_TTS_ASSET\("([^"]+)",\s*(\d+)U?,\s*"([0-9a-f]{64})"\)',
)


def _manifest_path() -> Path:
    installed_path = Path(__file__).with_name(_MANIFEST_FILENAME)
    if installed_path.is_file():
        return installed_path
    return (
        Path(__file__).resolve().parents[1]
        / 'voice_nav_audio' / 'src' / _MANIFEST_FILENAME
    )


def _load_expected_files() -> tuple[tuple[str, int, str], ...]:
    try:
        entries = tuple(
            (name, int(size), digest)
            for name, size, digest in _MANIFEST_ENTRY.findall(
                _manifest_path().read_text(encoding='utf-8'),
            )
        )
    except (OSError, UnicodeError, ValueError):
        return ()
    return entries if len(entries) == 6 else ()


_EXPECTED_FILES = _load_expected_files()


def _unavailable(reason: str) -> dict[str, str]:
    return {'status': 'unavailable', 'reason': reason}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(
            lambda: stream.read(_HASH_CHUNK_BYTES),
            b'',
        ):
            digest.update(chunk)
    return digest.hexdigest()


def verify_chaowen_root(root: str) -> dict[str, str]:
    """Return ready only when every pinned runtime file matches exactly."""
    if len(_EXPECTED_FILES) != 6:
        return _unavailable('chaowen_tts_asset_manifest_unavailable')
    if (
        not os.path.isabs(root)
        or os.path.islink(root)
        or not os.path.isdir(root)
    ):
        return _unavailable('chaowen_tts_root_not_verified')

    root_path = Path(root)
    for filename, expected_size, expected_hash in _EXPECTED_FILES:
        path = root_path / filename
        if path.is_symlink() or not path.is_file():
            return _unavailable(f'chaowen_tts_asset_missing:{filename}')
        try:
            if path.stat().st_size != expected_size:
                return _unavailable(f'chaowen_tts_asset_size_mismatch:{filename}')
            if _sha256(path) != expected_hash:
                return _unavailable(
                    f'chaowen_tts_asset_sha256_mismatch:{filename}',
                )
        except OSError:
            return _unavailable(f'chaowen_tts_asset_unreadable:{filename}')

    return {'status': 'ready', 'reason': ''}
