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

"""Create the test-only crash robot from a canonical expanded product URDF."""

import re
import xml.etree.ElementTree as element_tree


PRODUCT_HARDWARE_PLUGIN = 'gz_ros2_control/GazeboSimSystem'
TEST_HARDWARE_PLUGIN = 'voice_nav_sim/JournaledGazeboSimSystemAdapter'


class CrashRobotDescriptionError(ValueError):
    """Report a robot description that cannot be transformed safely."""


def _validate_journal_identity(journal_name, journal_nonce):
    if not isinstance(journal_name, str) or re.fullmatch(
        r'/voice_nav_hardware_[0-9a-f]{16}',
        journal_name,
    ) is None:
        raise CrashRobotDescriptionError(
            'journal name must be /voice_nav_hardware_<16-lower-hex>',
        )
    if (
        not isinstance(journal_nonce, str)
        or re.fullmatch(r'[0-9a-f]{32}', journal_nonce) is None
        or journal_nonce == '0' * 32
    ):
        raise CrashRobotDescriptionError(
            'journal nonce must be 32 lowercase nonzero hex digits',
        )


def transform_product_urdf(product_urdf, journal_name, journal_nonce):
    """Replace only the owned hardware plugin seam and add journal identity."""
    _validate_journal_identity(journal_name, journal_nonce)
    try:
        root = element_tree.fromstring(product_urdf)
    except (element_tree.ParseError, TypeError) as error:
        raise CrashRobotDescriptionError(
            f'product URDF is not valid XML: {error}',
        ) from error

    hardware_nodes = root.findall('.//ros2_control/hardware')
    hardware_plugins = root.findall('.//ros2_control/hardware/plugin')
    if len(hardware_nodes) != 1 or len(hardware_plugins) != 1:
        raise CrashRobotDescriptionError(
            'expected exactly one hardware block and one hardware plugin',
        )
    hardware_plugin = hardware_plugins[0]
    if (hardware_plugin.text or '').strip() != PRODUCT_HARDWARE_PLUGIN:
        raise CrashRobotDescriptionError(
            'canonical product hardware plugin changed',
        )

    hardware = hardware_nodes[0]
    owned_parameter_names = {'journal_name', 'journal_nonce'}
    collisions = sorted(
        child.get('name')
        for child in list(hardware)
        if child.tag == 'param'
        and child.get('name') in owned_parameter_names
    )
    if collisions:
        raise CrashRobotDescriptionError(
            'canonical hardware must not already contain owned journal '
            'parameters: ' + ', '.join(collisions),
        )

    hardware_plugin.text = TEST_HARDWARE_PLUGIN
    element_tree.SubElement(
        hardware,
        'param',
        {'name': 'journal_name'},
    ).text = journal_name
    element_tree.SubElement(
        hardware,
        'param',
        {'name': 'journal_nonce'},
    ).text = journal_nonce
    return element_tree.tostring(root, encoding='unicode')
