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

"""Behavioral tests for the bounded WSL VAD mode launcher."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess

import pytest


LAUNCHER = (
    Path(__file__).resolve().parents[3] / 'scripts' /
    'run_voice_nav_vad_mode.sh'
)


def _fake_ros2(tmp_path: Path) -> tuple[dict[str, str], Path]:
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    argv_file = tmp_path / 'ros2.argv'
    fake_ros2 = bin_dir / 'ros2'
    fake_ros2.write_text(
        '#!/usr/bin/env bash\n'
        'set -euo pipefail\n'
        'printf \'%s\\n\' "$@" > "${ROS2_ARGV_FILE:?}"\n',
        encoding='utf-8',
    )
    fake_ros2.chmod(0o755)
    environment = os.environ.copy()
    environment['PATH'] = f'{bin_dir}{os.pathsep}{environment["PATH"]}'
    environment['ROS2_ARGV_FILE'] = str(argv_file)
    return environment, argv_file


@pytest.mark.parametrize('mode', ('mapping', 'navigation'))
def test_launcher_execs_one_exact_vad_app_command(tmp_path, mode):
    environment, argv_file = _fake_ros2(tmp_path)

    result = subprocess.run(
        ['bash', str(LAUNCHER), mode],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert argv_file.read_text(encoding='utf-8').splitlines() == [
        'run',
        'voice_nav_bringup',
        'voice_nav_app',
        '--mode',
        mode,
        '--display',
        'headless',
        '--input',
        'vad-auto',
    ]


@pytest.mark.parametrize('arguments', (('invalid',), ('mapping', 'extra')))
def test_launcher_rejects_invalid_or_passthrough_arguments(tmp_path, arguments):
    environment, argv_file = _fake_ros2(tmp_path)

    result = subprocess.run(
        ['bash', str(LAUNCHER), *arguments],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert not argv_file.exists()
