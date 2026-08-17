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
from launch.conditions import IfCondition

from launch_ros.actions import Node


def _load_session_launch():
    launch = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'voice_nav_session.launch.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_session_launch', launch,
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def test_voice_nav_session_launch_source_is_installed():
    """The session launch must be present in the package share launch tree."""
    launch = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'voice_nav_session.launch.py'
    )
    assert launch.is_file()


def test_session_selects_one_mode_composition_and_one_agent_gateway():
    """Keep mode ownership in one session composition root."""
    description = _load_session_launch().generate_launch_description()
    includes = [
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    mode_includes = [
        include for include in includes
        if any(
            launch_file in str(
                getattr(
                    include.launch_description_source,
                    '_LaunchDescriptionSource__location',
                )
            )
            for launch_file in (
                'product_sim.launch.py',
                'mapping_mvp.launch.py',
                'navigation_mvp.launch.py',
            )
        )
    ]
    assert len(mode_includes) == 3

    expected = {
        'motion': 'product_sim.launch.py',
        'mapping': 'mapping_mvp.launch.py',
        'navigation': 'navigation_mvp.launch.py',
    }
    for mode, launch_file in expected.items():
        context = LaunchContext()
        context.launch_configurations['mode'] = mode
        selected = [
            include for include in mode_includes
            if include.condition.evaluate(context)
        ]
        assert len(selected) == 1
        assert launch_file in str(
            getattr(
                selected[0].launch_description_source,
                '_LaunchDescriptionSource__location',
            )
        )

    nodes = [
        entity for entity in description.entities if isinstance(entity, Node)
    ]
    assert [
        (getattr(node, '_Node__package', None),
         getattr(node, '_Node__node_executable', None))
        for node in nodes
    ] == [
        ('voice_nav_agent', 'agent_node'),
        ('voice_nav_audio', 'scripted_voice_demo'),
    ]


def test_session_selects_one_command_gateway_and_one_agent():
    """Only the internal console gateway belongs in the session launch."""
    description = _load_session_launch().generate_launch_description()
    command_gateways = [
        entity for entity in description.entities
        if isinstance(entity, Node)
        and getattr(entity, '_Node__package', None) == 'voice_nav_audio'
        and getattr(
            entity, '_Node__node_executable', None,
        ) == 'scripted_voice_demo'
    ]
    assert len(command_gateways) == 1
    assert not any(
        'voice_node.launch.py' in str(
            getattr(
                entity.launch_description_source,
                '_LaunchDescriptionSource__location',
            )
        )
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    )
    assert sum(
        isinstance(entity, Node)
        and getattr(entity, '_Node__package', None) == 'voice_nav_agent'
        and getattr(entity, '_Node__node_executable', None) == 'agent_node'
        for entity in description.entities
    ) == 1
    assert isinstance(command_gateways[0].condition, IfCondition)

    def active_gateways(context):
        return [
            entity for entity in command_gateways
            if entity.condition.evaluate(context)
        ]

    console_context = LaunchContext()
    console_context.launch_configurations['input'] = 'console'
    assert active_gateways(console_context) == command_gateways

    none_context = LaunchContext()
    none_context.launch_configurations['input'] = 'none'
    assert active_gateways(none_context) == []
