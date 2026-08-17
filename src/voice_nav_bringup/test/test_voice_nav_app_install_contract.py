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

"""Install-space contract for the simulation-only app entrypoint."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def test_app_is_installed_as_the_voice_nav_bringup_executable():
    """Require the exact extensionless ros2-run executable name."""
    package = Path(__file__).resolve().parents[1]
    cmake = (package / 'CMakeLists.txt').read_text(encoding='utf-8')

    assert 'install(\n  PROGRAMS\n    voice_nav_app.py' in cmake
    assert '  RENAME voice_nav_app' in cmake
    assert 'DESTINATION lib/${PROJECT_NAME}' in cmake
    assert 'install(\n  FILES\n    _mode_readiness.py' in cmake
    assert '    _sensevoice_input.py' in cmake
    assert '    _chaowen_asset_verifier.py' in cmake
    assert 'chaowen_tts_asset_manifest.def' in cmake

    prefix = Path(get_package_prefix('voice_nav_bringup'))
    executable = prefix / 'lib' / 'voice_nav_bringup' / 'voice_nav_app'
    helper = prefix / 'lib' / 'voice_nav_bringup' / '_mode_readiness.py'
    sensevoice_helper = (
        prefix / 'lib' / 'voice_nav_bringup' / '_sensevoice_input.py'
    )
    chaowen_verifier = (
        prefix / 'lib' / 'voice_nav_bringup' / '_chaowen_asset_verifier.py'
    )
    chaowen_manifest = (
        prefix / 'lib' / 'voice_nav_bringup' / 'chaowen_tts_asset_manifest.def'
    )
    suffixed_executable = executable.with_name('voice_nav_app.py')
    assert executable.is_file()
    assert os.access(executable, os.X_OK)
    assert sensevoice_helper.is_file()
    assert chaowen_verifier.is_file()
    assert chaowen_manifest.is_file()
    assert not suffixed_executable.exists()
    assert helper.is_file()
