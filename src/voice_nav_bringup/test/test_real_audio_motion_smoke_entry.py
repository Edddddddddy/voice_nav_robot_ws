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

"""Installed entry contract for the one-shot real-audio Motion smoke."""

from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_entry_module():
    source = Path(__file__).resolve().parents[1] / 'voice_nav_motion_smoke.py'
    specification = importlib.util.spec_from_file_location(
        'voice_nav_motion_smoke', source,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def test_entry_accepts_prepared_prefix_and_uses_existing_app_only():
    """Keep maintenance provisioning outside the installed runtime entry."""
    module = _load_entry_module()
    assert module.build_app_command() == (
        'ros2', 'run', 'voice_nav_bringup', 'voice_nav_app',
        '--mode', 'motion', '--display', 'headless',
        '--input', 'microphone-once',
    )
