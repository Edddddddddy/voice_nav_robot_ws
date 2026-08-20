# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Minimal source/install composition contract for the Mapping MVP."""

import importlib.util
from pathlib import Path
import sys

import yaml

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.utilities import perform_substitutions
from launch_ros.actions import LifecycleNode, Node


def _load_mapping_launch():
    source_python = (
        Path(__file__).resolve().parents[2] / 'voice_nav_sim' / 'python'
    )
    if str(source_python) not in sys.path:
        sys.path.insert(0, str(source_python))
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'mapping_mvp.launch.py'
    )
    specification = importlib.util.spec_from_file_location(
        'mapping_mvp_launch', launch_path
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _load_product_launch():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'product_sim.launch.py'
    )
    specification = importlib.util.spec_from_file_location(
        'product_sim_launch', launch_path
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def test_mapping_mvp_has_fixed_world_slam_config_and_product_composition():
    package_root = Path(__file__).resolve().parents[1]
    assert (package_root / 'config' / 'slam_toolbox_mapping.yaml').is_file()
    world = (
        package_root.parent
        / 'voice_nav_sim'
        / 'worlds'
        / 'voice_nav_house_world.sdf'
    )
    assert world.is_file()

    description = _load_mapping_launch().generate_launch_description()
    entities = description.entities
    slam_nodes = [
        entity for entity in entities
        if isinstance(entity, LifecycleNode)
        and getattr(entity, '_Node__node_name', None) == 'slam_toolbox'
    ]
    product_includes = [
        entity for entity in entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert len(slam_nodes) == 1
    assert len(product_includes) == 1
    rviz_nodes = [
        entity for entity in entities
        if isinstance(entity, Node)
        and getattr(entity, '_Node__package', None) == 'rviz2'
        and getattr(entity, '_Node__node_executable', None) == 'rviz2'
    ]
    assert len(rviz_nodes) == 1
    assert (package_root / 'config' / 'voice_nav_mapping.rviz').is_file()
    assert any(
        'product_sim.launch.py' in str(
            getattr(
                include.launch_description_source,
                '_LaunchDescriptionSource__location',
            )
        )
        for include in product_includes
    )


def test_mapping_rviz_enables_live_map_with_matching_qos():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'voice_nav_mapping.rviz'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    manager = config['Visualization Manager']
    displays = {display['Name']: display for display in manager['Displays']}
    live_map = displays['Live Map']

    assert manager['Enabled'] is True
    assert manager['Global Options']['Fixed Frame'] == 'map'
    assert manager['Views']['Current']['Scale'] == 60
    assert live_map['Enabled'] is True
    assert live_map['Topic'] == {
        'Depth': 5,
        'Durability Policy': 'Transient Local',
        'History Policy': 'Keep Last',
        'Reliability Policy': 'Reliable',
        'Value': '/map',
    }
    assert live_map['Update Topic']['Value'] == '/map_updates'


def test_mapping_rviz_uses_wsl_safe_software_rendering():
    description = _load_mapping_launch().generate_launch_description()
    rviz = next(
        entity for entity in description.entities
        if isinstance(entity, Node)
        and getattr(entity, '_Node__package', None) == 'rviz2'
    )
    executable = getattr(rviz, '_ExecuteLocal__process_description')
    context = LaunchContext()
    environment = {
        perform_substitutions(context, key):
            perform_substitutions(context, value)
        for key, value in getattr(executable, '_Executable__additional_env')
    }

    assert environment == {
        'LIBGL_ALWAYS_SOFTWARE': 'true',
    }


def test_product_runtime_starts_after_sim_controller_settles():
    description = _load_product_launch().generate_launch_description()
    timers = [
        entity for entity in description.entities
        if isinstance(entity, TimerAction)
    ]
    runtime_timers = [
        timer for timer in timers
        if any(
            isinstance(action, Node)
            and getattr(action, '_Node__node_name', None) ==
                'mission_runtime_node'
            for action in timer.actions
        )
    ]

    assert len(runtime_timers) == 1
    context = LaunchContext()
    context.launch_configurations['runtime_start_delay'] = '12.0'
    assert float(
        perform_substitutions(context, runtime_timers[0].period)
    ) >= 12.0
