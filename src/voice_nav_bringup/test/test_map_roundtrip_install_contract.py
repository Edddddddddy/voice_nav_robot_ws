# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Install-space contract for the bounded map roundtrip entrypoint."""

import os
from pathlib import Path


def test_map_roundtrip_is_installed_as_extensionless_entrypoint():
    from ament_index_python.packages import get_package_prefix

    package = Path(__file__).resolve().parents[1]
    cmake = (package / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'voice_nav_map_roundtrip.py' in cmake
    assert 'RENAME voice_nav_map_roundtrip' in cmake
    assert 'DESTINATION lib/${PROJECT_NAME}' in cmake

    executable = (
        Path(get_package_prefix('voice_nav_bringup'))
        / 'lib' / 'voice_nav_bringup' / 'voice_nav_map_roundtrip'
    )
    assert executable.is_file()
    assert os.access(executable, os.X_OK)
    assert not executable.with_name('voice_nav_map_roundtrip.py').exists()
