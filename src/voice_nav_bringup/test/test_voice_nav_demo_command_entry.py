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

"""Public contract for the caller-provided command VoiceNav demo entrypoint."""

import importlib.util
from pathlib import Path


def _load_launch_module(launch):
    specification = importlib.util.spec_from_file_location(
        'voice_nav_demo_launch', launch,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def test_voice_nav_demo_accepts_one_bounded_command_text():
    launch = Path(__file__).resolve().parents[1] / 'launch' / 'voice_nav_demo.launch.py'
    assert launch.is_file()
    module = _load_launch_module(launch)

    actions, fixtures = module.create_voice_nav_demo(
        command_text='右转九十度',
        shutdown_when_demo_exits=False,
    )
    try:
        assert fixtures['command_text'] == '右转九十度'
        assert actions
    finally:
        module._stop_loopback(fixtures['llm_server'], fixtures['llm_thread'])
