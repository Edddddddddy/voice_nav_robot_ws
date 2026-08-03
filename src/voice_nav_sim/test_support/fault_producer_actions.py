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

"""Build the two exact launch actions used as crashable Gate producers."""

from dataclasses import dataclass
import re

from launch_ros.actions import Node


CASE_ID_PATTERN = re.compile(r'[a-z][a-z0-9_]{0,23}\Z')


@dataclass(frozen=True)
class FaultProducerActions:
    """Retain the exact authority and candidate launch identities."""

    authority: Node
    candidate: Node

    @property
    def actions(self):
        """Return the fixed launch order without manufacturing new actions."""
        return (self.authority, self.candidate)


def make_fault_producers(case_id):
    """Create one closed authority/candidate pair for a test generation."""
    if not isinstance(case_id, str) or not CASE_ID_PATTERN.fullmatch(case_id):
        raise ValueError('case_id must match [a-z][a-z0-9_]{0,23}')

    common_parameters = {
        'case_id': case_id,
        'use_sim_time': False,
    }
    authority = Node(
        package='voice_nav_sim',
        executable='fault_producer_helper',
        name=f'{case_id}_authority',
        output='screen',
        parameters=[{**common_parameters, 'role': 'authority'}],
    )
    candidate = Node(
        package='voice_nav_sim',
        executable='fault_producer_helper',
        name='collision_monitor',
        output='screen',
        parameters=[{**common_parameters, 'role': 'candidate'}],
    )
    return FaultProducerActions(authority=authority, candidate=candidate)
