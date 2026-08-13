# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Behavioral contracts for Mapping Mode launch admission."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from launch.actions import IncludeLaunchDescription
from launch_ros.actions import LifecycleNode


MAPPING_LAUNCH = Path(__file__).parents[1] / 'launch' / 'mapping_sim.launch.py'
PRODUCT_LAUNCH = Path(__file__).parents[1] / 'launch' / 'product_sim.launch.py'


def load_mapping_launch():
    specification = importlib.util.spec_from_file_location(
        'voice_nav_mapping_launch', MAPPING_LAUNCH
    )
    if specification is None or specification.loader is None:
        raise AssertionError('could not load Mapping launch support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def load_product_launch():
    specification = importlib.util.spec_from_file_location(
        'voice_nav_product_launch', PRODUCT_LAUNCH
    )
    if specification is None or specification.loader is None:
        raise AssertionError('could not load product launch support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


class MappingLaunchTest(unittest.TestCase):
    def test_mapping_quality_delegates_to_the_shared_oracle(self):
        specification = importlib.util.spec_from_file_location(
            'voice_nav_mapping_mode',
            Path(__file__).parents[0] / 'test_mapping_mode.py',
        )
        self.assertIsNotNone(specification)
        self.assertIsNotNone(specification.loader)
        module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(module)

        grid = SimpleNamespace(
            info=SimpleNamespace(
                resolution=0.05, width=1, height=1,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
            data=[100],
        )
        transform = SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=0.0, y=0.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
        )
        geometry = ({'name': 'wall', 'kind': 'box', 'x': 0.19, 'y': 0.025,
                     'size_x': 0.10, 'size_y': 0.05},)
        artifact = {
            'schema_version': 1,
            'policy': {
                'map_resolution': module.MAP_RESOLUTION,
                'floor_min': -2.95,
                'floor_max': 2.95,
                'minimum_route_clearance': module.MINIMUM_ROUTE_CLEARANCE,
                'occupied_threshold': module.OCCUPIED_THRESHOLD,
                'boundary_search_radius': module.BOUNDARY_SEARCH_RADIUS,
            },
            'geometry': geometry,
            'route': module.FROZEN_ROUTE,
            'grid': module._grid_record(grid),
            'map_from_odom': module._transform_record(transform),
        }
        self.assertEqual(
            module._mapping_quality(grid, transform, geometry),
            module.mapping_quality.evaluate_mapping_artifact(artifact),
        )

    def test_missing_taskset_fails_before_any_owner_can_start(self):
        mapping_launch = load_mapping_launch()
        with patch.object(mapping_launch.shutil, 'which', return_value=None):
            with self.assertRaises(mapping_launch.MappingPreflightError):
                mapping_launch._require_taskset()

    def test_missing_taskset_fails_before_lock_or_owner_actions(self):
        mapping_launch = load_mapping_launch()
        acquisitions = []

        def acquire_mode_lock(**arguments):
            acquisitions.append(arguments)
            raise AssertionError('lock must not be acquired after preflight')

        with self.assertRaises(mapping_launch.MappingPreflightError):
            mapping_launch._start_mapping(
                None,
                acquire_mode_lock=acquire_mode_lock,
                taskset_locator=lambda _name: None,
            )
        self.assertEqual(acquisitions, [])

    def test_mapping_composes_fixed_house_and_single_cpu_async_slam(self):
        mapping_launch = load_mapping_launch()

        class Owner:
            closed = False

            def close(self):
                self.closed = True

        owner = Owner()
        actions = mapping_launch._start_mapping(
            None, acquire_mode_lock=lambda **_arguments: owner
        )

        product = next(
            action
            for action in actions
            if isinstance(action, IncludeLaunchDescription)
        )
        arguments = dict(product.launch_arguments)
        self.assertEqual(arguments['world_name'], 'voice_nav_house_world')
        self.assertEqual(arguments['laser_update_rate'], '20')

        slam = next(
            action for action in actions if isinstance(action, LifecycleNode)
        )
        self.assertEqual(slam.node_package, 'slam_toolbox')
        self.assertEqual(slam.node_executable, 'async_slam_toolbox_node')
        self.assertEqual(
            slam.process_description.prefix[0].text, 'taskset --cpu-list 0'
        )
        self.assertFalse(owner.closed)

    def test_launch_registers_shutdown_and_slam_exit_gates(self):
        mapping_launch = load_mapping_launch()

        class Owner:
            closed = 0

            def close(self):
                self.closed += 1

        owner = Owner()
        actions = mapping_launch._start_mapping(
            None, acquire_mode_lock=lambda **_arguments: owner
        )
        shutdown_handler, process_exit_handler = actions[:2]
        self.assertIsNotNone(shutdown_handler.event_handler)
        self.assertIsNotNone(process_exit_handler.event_handler)
        self.assertEqual(owner.closed, 0)

    def test_product_forwards_only_trusted_world_and_lidar_selectors(self):
        product_launch = load_product_launch()
        description = product_launch.generate_launch_description()
        simulation = next(
            action
            for action in description.entities
            if isinstance(action, IncludeLaunchDescription)
            and 'simulation.launch.py'
            in action.launch_description_source.location
        )
        arguments = dict(simulation.launch_arguments)
        self.assertEqual(arguments['world_name'].variable_name[0].text, 'world_name')
        self.assertEqual(
            arguments['laser_update_rate'].variable_name[0].text,
            'laser_update_rate',
        )


if __name__ == '__main__':
    unittest.main()
