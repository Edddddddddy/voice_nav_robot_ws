# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Minimal source/install composition contract for the Mapping MVP."""

import importlib.util
from pathlib import Path
import sys

from launch.actions import IncludeLaunchDescription
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
