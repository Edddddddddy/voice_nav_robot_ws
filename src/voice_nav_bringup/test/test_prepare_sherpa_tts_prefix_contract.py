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

"""Behavior contracts for the maintenance-only sherpa prefix entry."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _run_prefix_script(
    prefix: Path,
    *,
    source_archive: Path | None = None,
    onnxruntime_zip: Path | None = None,
) -> subprocess.CompletedProcess[str]:
    repo = _repo_root()
    return subprocess.run(
        [
            'bash',
            str(repo / 'scripts' / 'prepare_sherpa_tts_prefix.sh'),
            '--prefix',
            str(prefix),
            *(('--source-archive', str(source_archive))
              if source_archive is not None else ()),
            *(('--onnxruntime-zip', str(onnxruntime_zip))
              if onnxruntime_zip is not None else ()),
            '--offline',
        ],
        cwd=repo,
        check=False,
        capture_output=True,
        text=True,
    )


def test_prepare_rejects_prefix_outside_ignored_dependency_root(tmp_path):
    result = _run_prefix_script(tmp_path / 'outside-prefix')
    assert result.returncode == 2
    assert 'below ignored .deps/voice-assets' in result.stderr


def test_prepare_rejects_raw_prefix_symlink(tmp_path):
    allowed_root = _repo_root() / '.deps' / 'voice-assets'
    allowed_root.mkdir(parents=True, exist_ok=True)
    link = allowed_root / f'boundary-prefix-link-{os.getpid()}'
    link_target = tmp_path / 'link-target'
    link.symlink_to(link_target, target_is_directory=True)
    try:
        result = _run_prefix_script(link)
    finally:
        link.unlink(missing_ok=True)
    assert result.returncode == 2
    assert 'symlink/reparse' in result.stderr


def test_prepare_requires_git_ignored_proof_for_valid_prefix(tmp_path):
    allowed_root = _repo_root() / '.deps' / 'voice-assets'
    prefix = allowed_root / f'boundary-prefix-{os.getpid()}'
    result = _run_prefix_script(
        prefix,
        source_archive=tmp_path / 'missing-source.tar.gz',
        onnxruntime_zip=tmp_path / 'missing-ort.zip',
    )
    assert result.returncode == 1
    assert 'missing locked cache' in result.stderr


def test_prepare_rejects_missing_or_mismatched_receipt(tmp_path, monkeypatch):
    allowed_root = _repo_root() / '.deps' / 'voice-assets'
    allowed_root.mkdir(parents=True, exist_ok=True)
    compiler = tmp_path / 'cxx-fixture'
    compiler.write_text("#!/usr/bin/env bash\nprintf '13.3.0\\n'\n", encoding='utf-8')
    compiler.chmod(0o755)
    monkeypatch.setenv('CXX', str(compiler))

    for label, receipt in (('missing', None), ('mismatched', 'not the exact receipt\n')):
        with_receipt = allowed_root / f'boundary-prefix-receipt-{os.getpid()}-{label}'
        (with_receipt / 'share' / 'voice_nav').mkdir(parents=True)
        if receipt is not None:
            (with_receipt / 'share' / 'voice_nav' / 'sherpa-onnx-provenance.receipt').write_text(
                receipt,
                encoding='utf-8',
            )
        try:
            result = _run_prefix_script(
                with_receipt,
                source_archive=tmp_path / f'{label}-source.tar.gz',
                onnxruntime_zip=tmp_path / f'{label}-ort.zip',
            )
        finally:
            shutil.rmtree(with_receipt)
        assert result.returncode == 1
        assert 'exact sherpa receipt mismatch' in result.stderr
        assert 'refusing to replace incomplete sherpa prefix' in result.stderr


def test_prepare_does_not_disable_cmake_tls_verification():
    script = (_repo_root() / 'scripts' / 'prepare_sherpa_tts_prefix.sh').read_text(
        encoding='utf-8',
    )
    assert 'CMAKE_TLS_VERIFY=OFF' not in script
