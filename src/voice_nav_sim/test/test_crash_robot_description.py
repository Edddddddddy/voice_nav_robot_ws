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

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as element_tree


def load_crash_robot_description_support():
    support_path = (
        Path(__file__).resolve().parents[1]
        / 'test_support'
        / 'crash_robot_description.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_crash_robot_description_test_support',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load crash robot-description support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


crash_robot_description = load_crash_robot_description_support()


PRODUCT_URDF = '''\
<robot name="voice_nav_robot">
  <link name="base_link" />
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity" />
    </joint>
  </ros2_control>
  <gazebo reference="base_link"><gravity>true</gravity></gazebo>
</robot>
'''


def element_signature(element):
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or '').strip(),
        tuple(element_signature(child) for child in element),
    )


class CrashRobotDescriptionTest(unittest.TestCase):
    def test_valid_product_urdf_changes_only_owned_hardware_seam(self):
        transformed_text = crash_robot_description.transform_product_urdf(
            PRODUCT_URDF,
            '/voice_nav_hardware_0011223344556677',
            '00112233445566778899aabbccddeeff',
        )
        original = element_tree.fromstring(PRODUCT_URDF)
        transformed = element_tree.fromstring(transformed_text)
        hardware_nodes = transformed.findall('.//ros2_control/hardware')
        plugins = transformed.findall('.//ros2_control/hardware/plugin')

        self.assertEqual(len(hardware_nodes), 1)
        self.assertEqual(len(plugins), 1)
        self.assertEqual(
            (plugins[0].text or '').strip(),
            'voice_nav_sim/JournaledGazeboSimSystemAdapter',
        )
        hardware = hardware_nodes[0]
        journal_parameters = {
            child.get('name'): (child.text or '').strip()
            for child in list(hardware)
            if child.tag == 'param'
        }
        self.assertEqual(
            journal_parameters,
            {
                'journal_name': '/voice_nav_hardware_0011223344556677',
                'journal_nonce': '00112233445566778899aabbccddeeff',
            },
        )

        for child in list(hardware):
            if child.tag == 'param':
                hardware.remove(child)
        plugins[0].text = 'gz_ros2_control/GazeboSimSystem'
        self.assertEqual(
            element_signature(transformed),
            element_signature(original),
        )

    def test_rejects_preexisting_owned_journal_parameters(self):
        for parameter_name in ('journal_name', 'journal_nonce'):
            with self.subTest(parameter_name=parameter_name):
                contaminated_urdf = PRODUCT_URDF.replace(
                    '</hardware>',
                    '<param name="{}">existing</param></hardware>'.format(
                        parameter_name,
                    ),
                    1,
                )
                with self.assertRaisesRegex(
                    crash_robot_description.CrashRobotDescriptionError,
                    'must not already contain',
                ):
                    crash_robot_description.transform_product_urdf(
                        contaminated_urdf,
                        '/voice_nav_hardware_0011223344556677',
                        '00112233445566778899aabbccddeeff',
                    )

    def test_rejects_malformed_journal_identity(self):
        valid_name = '/voice_nav_contract_fixture'
        valid_nonce = '00112233445566778899aabbccddeeff'
        invalid_identities = (
            ('', valid_nonce, 'journal name'),
            ('voice_nav_missing_slash', valid_nonce, 'journal name'),
            ('/voice/nav', valid_nonce, 'journal name'),
            ('/voice nav', valid_nonce, 'journal name'),
            (valid_name, '', 'journal nonce'),
            (valid_name, '00112233445566778899AABBCCDDEEFF', 'journal nonce'),
            (valid_name, '00112233445566778899aabbccddeef', 'journal nonce'),
            (valid_name, 'g0112233445566778899aabbccddeeff', 'journal nonce'),
            (valid_name, '0' * 32, 'journal nonce'),
        )

        for journal_name, journal_nonce, diagnostic in invalid_identities:
            with self.subTest(
                journal_name=journal_name,
                journal_nonce=journal_nonce,
            ):
                with self.assertRaisesRegex(
                    crash_robot_description.CrashRobotDescriptionError,
                    diagnostic,
                ):
                    crash_robot_description.transform_product_urdf(
                        PRODUCT_URDF,
                        journal_name,
                        journal_nonce,
                    )

    def test_requires_one_unchanged_product_hardware_plugin(self):
        plugin_xml = (
            '<plugin>gz_ros2_control/GazeboSimSystem</plugin>'
        )
        invalid_descriptions = (
            PRODUCT_URDF.replace(plugin_xml, '', 1),
            PRODUCT_URDF.replace(plugin_xml, plugin_xml * 2, 1),
            PRODUCT_URDF.replace(
                'gz_ros2_control/GazeboSimSystem',
                'another_vendor/UnexpectedSystem',
                1,
            ),
            PRODUCT_URDF.replace(
                '</ros2_control>',
                '<ros2_control name="duplicate" type="system">'
                '<hardware>{}</hardware></ros2_control>'
                '</ros2_control>'.format(plugin_xml),
                1,
            ),
        )

        for description in invalid_descriptions:
            with self.subTest(description=description):
                with self.assertRaises(
                    crash_robot_description.CrashRobotDescriptionError,
                ):
                    crash_robot_description.transform_product_urdf(
                        description,
                        '/voice_nav_contract_fixture',
                        '00112233445566778899aabbccddeeff',
                    )


if __name__ == '__main__':
    unittest.main()
