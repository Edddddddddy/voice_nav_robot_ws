# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Deployment-level behavior contracts for Mapping Mode."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


SLAM_CONFIG = (
    Path(__file__).parents[1]
    / 'src'
    / 'voice_nav_bringup'
    / 'config'
    / 'slam_toolbox_mapping.yaml'
)
HOUSE_WORLD = (
    Path(__file__).parents[1]
    / 'src'
    / 'voice_nav_sim'
    / 'worlds'
    / 'voice_nav_house_world.sdf'
)
BRINGUP_MANIFEST = (
    Path(__file__).parents[1] / 'src' / 'voice_nav_bringup' / 'package.xml'
)


class MappingContractTest(unittest.TestCase):
    def test_mapping_uses_the_frozen_async_slam_configuration(self):
        configuration = yaml.safe_load(SLAM_CONFIG.read_text(encoding='utf-8'))
        parameters = configuration['slam_toolbox']['ros__parameters']

        self.assertEqual(parameters['mode'], 'mapping')
        self.assertTrue(parameters['use_sim_time'])
        self.assertEqual(parameters['map_frame'], 'map')
        self.assertEqual(parameters['odom_frame'], 'odom')
        self.assertEqual(parameters['base_frame'], 'base_footprint')
        self.assertEqual(parameters['scan_topic'], '/scan')
        self.assertFalse(parameters['use_map_saver'])
        self.assertEqual(parameters['scan_queue_size'], 1)
        self.assertEqual(parameters['ceres_loss_function'], 'HuberLoss')
        self.assertEqual(parameters['resolution'], 0.05)

    def test_house_world_is_the_fixed_asymmetric_mapping_environment(self):
        world = ElementTree.parse(HOUSE_WORLD).getroot().find('world')
        self.assertIsNotNone(world)
        self.assertEqual(world.attrib['name'], 'voice_nav_house_world')
        models = {model.attrib['name']: model for model in world.findall('model')}

        self.assertEqual(models['south'].findtext('pose'), '0 -3 0.5 0 0 0')
        self.assertEqual(models['north'].findtext('pose'), '0 3 0.5 0 0 0')
        self.assertEqual(models['west'].findtext('pose'), '-3 0 0.5 0 0 0')
        self.assertEqual(models['east'].findtext('pose'), '3 0 0.5 0 0 0')
        self.assertEqual(
            models['lower_left_box'].findtext('pose'), '-1.9 -1.6 0.4 0 0 0'
        )
        self.assertEqual(
            models['upper_right_cylinder'].findtext('pose'), '2.0 1.9 0.4 0 0 0'
        )
        for name in (
            'south', 'north', 'west', 'east', 'inner_v_lower',
            'inner_v_middle', 'inner_v_upper', 'inner_h_west',
            'inner_h_east', 'lower_left_box', 'upper_right_cylinder',
        ):
            self.assertEqual(models[name].findtext('static'), 'true')
            self.assertIsNotNone(models[name].find('.//collision'))
            self.assertIsNotNone(models[name].find('.//visual'))

    def test_mapping_declares_the_slam_lifecycle_and_taskset_dependencies(self):
        package = ElementTree.parse(BRINGUP_MANIFEST).getroot()
        dependencies = {
            dependency.text for dependency in package.findall('exec_depend')
        }
        self.assertTrue(
            {'slam_toolbox', 'lifecycle_msgs', 'util-linux'} <= dependencies
        )


if __name__ == '__main__':
    unittest.main()
