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

"""Install-space contract for the explicit scripted simulation demo."""

import importlib.util
from pathlib import Path

import pytest

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)


def _load_launch_module(launch):
    spec = importlib.util.spec_from_file_location('scripted_voice_demo_launch', launch)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_scripted_voice_demo_route_is_a_closed_source_scenario():
    launch = Path(__file__).resolve().parents[1] / 'launch' / 'scripted_voice_demo.launch.py'
    module = _load_launch_module(launch)

    assert module.SCENARIOS == ('move', 'stop', 'route')
    with pytest.raises(ValueError, match='move\\|stop\\|route'):
        module.create_scripted_voice_demo(scenario='invalid')
    _actions, fixtures = module.create_scripted_voice_demo(
        scenario='route', shutdown_when_demo_exits=False,
    )
    try:
        assert fixtures['scenario'] == 'route'
    finally:
        module._stop_loopback(fixtures['llm_server'], fixtures['llm_thread'])


def test_scripted_voice_demo_is_available_without_a_test_environment():
    executable = Path(
        get_package_prefix('voice_nav_audio')
    ) / 'lib' / 'voice_nav_audio' / 'scripted_voice_demo'
    launch = Path(
        get_package_share_directory('voice_nav_bringup')
    ) / 'launch' / 'scripted_voice_demo.launch.py'

    assert executable.is_file()
    assert launch.is_file()


def test_scripted_voice_demo_keeps_pipeline_node_names_unremapped():
    """The multi-node executable must not receive a global __node remap."""
    launch = Path(
        get_package_share_directory('voice_nav_bringup')
    ) / 'launch' / 'scripted_voice_demo.launch.py'
    module = _load_launch_module(launch)
    _actions, fixtures = module.create_scripted_voice_demo(
        shutdown_when_demo_exits=False,
    )
    try:
        assert fixtures['speech_driver']._Node__node_name is None
    finally:
        module._stop_loopback(fixtures['llm_server'], fixtures['llm_thread'])


def test_scripted_voice_demo_route_is_a_closed_installed_scenario():
    launch = Path(
        get_package_share_directory('voice_nav_bringup')
    ) / 'launch' / 'scripted_voice_demo.launch.py'
    module = _load_launch_module(launch)

    assert module.SCENARIOS == ('move', 'stop', 'route')
    with pytest.raises(ValueError, match='move\\|stop\\|route'):
        module.create_scripted_voice_demo(scenario='invalid')
    _actions, fixtures = module.create_scripted_voice_demo(
        scenario='route', shutdown_when_demo_exits=False,
    )
    try:
        assert fixtures['scenario'] == 'route'
    finally:
        module._stop_loopback(fixtures['llm_server'], fixtures['llm_thread'])
