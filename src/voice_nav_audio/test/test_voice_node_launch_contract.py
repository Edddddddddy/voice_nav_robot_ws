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

"""Contract tests for the installed voice_node launch input profile choice."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument


def _launch_module():
    launch_path = Path(__file__).parents[1] / 'launch' / 'voice_node.launch.py'
    spec = importlib.util.spec_from_file_location(
        'voice_node_launch_under_test', launch_path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _input_profile_argument():
    description = _launch_module().generate_launch_description()
    return next(
        entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == 'input_profile'
    )


def test_launch_exposes_only_continuous_vad_auto():
    """The installed launch entrypoint has one continuous input profile."""
    argument = _input_profile_argument()
    assert argument.default_value[0].text == 'vad_auto'
    assert argument.choices == ['vad_auto']
