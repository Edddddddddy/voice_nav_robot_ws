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

"""Install-space contract for the persistent simulation session entrypoint."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def _load_session_launch():
    launch = Path(__file__).resolve().parents[1] / 'launch' / 'voice_nav_session.launch.py'
    specification = importlib.util.spec_from_file_location(
        'voice_nav_session_launch', launch,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def test_voice_nav_session_launch_source_is_installed():
    """The session launch must be present in the package share launch tree."""
    launch = Path(__file__).resolve().parents[1] / 'launch' / 'voice_nav_session.launch.py'
    assert launch.is_file()


def test_session_selects_one_mode_composition_and_one_agent_gateway():
    """Keep mode ownership in one session composition root."""
    description = _load_session_launch().generate_launch_description()
    includes = [
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert len(includes) == 3

    expected = {
        'motion': 'product_sim.launch.py',
        'mapping': 'mapping_mvp.launch.py',
        'navigation': 'navigation_mvp.launch.py',
    }
    for mode, launch_file in expected.items():
        context = LaunchContext()
        context.launch_configurations['mode'] = mode
        selected = [
            include for include in includes
            if include.condition.evaluate(context)
        ]
        assert len(selected) == 1
        assert launch_file in str(
            getattr(
                selected[0].launch_description_source,
                '_LaunchDescriptionSource__location',
            )
        )

    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    assert [
        (getattr(node, '_Node__package', None),
         getattr(node, '_Node__node_executable', None))
        for node in nodes
    ] == [
        ('voice_nav_agent', 'agent_node'),
        ('voice_nav_audio', 'scripted_voice_demo'),
    ]
