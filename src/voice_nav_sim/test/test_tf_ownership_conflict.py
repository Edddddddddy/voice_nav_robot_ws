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

import sys
import textwrap
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import launch_testing
from launch_testing.asserts import assertExitCodes
import pytest


PARENT_FRAME = 'tf_audit_parent'
CHILD_FRAME = 'tf_audit_child'
DUPLICATE_NODE_NAME = 'duplicate_tf_owner'
FIRST_DISJOINT_PARENT = 'tf_disjoint_parent_one'
FIRST_DISJOINT_CHILD = 'tf_disjoint_child_one'
FIRST_DISJOINT_OWNER = 'disjoint_tf_owner_one'
SECOND_DISJOINT_PARENT = 'tf_disjoint_parent_two'
SECOND_DISJOINT_CHILD = 'tf_disjoint_child_two'
SECOND_DISJOINT_OWNER = 'disjoint_tf_owner_two'
DYNAMIC_PARENT = 'tf_dynamic_parent'
DYNAMIC_CHILD = 'tf_dynamic_child'
DYNAMIC_OWNER = 'dynamic_tf_owner'
WRONG_DYNAMIC_OWNER_FQN = '/wrong_dynamic_tf_owner'
AUDIT_TIMEOUT = '5.0'

DYNAMIC_TF_PUBLISHER = textwrap.dedent(
    f"""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node('{DYNAMIC_OWNER}')
    publisher = node.create_publisher(TFMessage, '/tf', 10)

    def publish_transform():
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = '{DYNAMIC_PARENT}'
        transform.child_frame_id = '{DYNAMIC_CHILD}'
        transform.transform.rotation.w = 1.0
        publisher.publish(TFMessage(transforms=[transform]))

    publish_transform()
    timer = node.create_timer(0.05, publish_transform)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    """
)


def static_transform(
    node_name,
    parent_frame,
    child_frame,
    translation,
):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=node_name,
        output='screen',
        arguments=[
            '--x',
            translation[0],
            '--y',
            translation[1],
            '--z',
            translation[2],
            '--roll',
            '0.0',
            '--pitch',
            '0.0',
            '--yaw',
            '0.0',
            '--frame-id',
            parent_frame,
            '--child-frame-id',
            child_frame,
        ],
    )


def node_fqn(node_name):
    return f'/{node_name}'


@pytest.mark.launch_test
def generate_test_description():
    first_owner = static_transform(
        DUPLICATE_NODE_NAME,
        PARENT_FRAME,
        CHILD_FRAME,
        ('0.10', '0.00', '0.00'),
    )
    second_owner = static_transform(
        DUPLICATE_NODE_NAME,
        PARENT_FRAME,
        CHILD_FRAME,
        ('0.20', '0.00', '0.00'),
    )
    first_disjoint_owner = static_transform(
        FIRST_DISJOINT_OWNER,
        FIRST_DISJOINT_PARENT,
        FIRST_DISJOINT_CHILD,
        ('0.00', '0.10', '0.00'),
    )
    second_disjoint_owner = static_transform(
        SECOND_DISJOINT_OWNER,
        SECOND_DISJOINT_PARENT,
        SECOND_DISJOINT_CHILD,
        ('0.00', '0.20', '0.00'),
    )
    dynamic_owner = ExecuteProcess(
        cmd=[sys.executable, '-c', DYNAMIC_TF_PUBLISHER],
        name='dynamic_tf_writer_process',
        output='screen',
    )
    enforcement_auditor = Node(
        package='voice_nav_sim',
        executable='tf_ownership_auditor',
        name='tf_ownership_enforcement_auditor',
        output='screen',
        arguments=[
            '--edge',
            '/tf_static',
            PARENT_FRAME,
            CHILD_FRAME,
            node_fqn(DUPLICATE_NODE_NAME),
            '--edge',
            '/tf_static',
            FIRST_DISJOINT_PARENT,
            FIRST_DISJOINT_CHILD,
            node_fqn(FIRST_DISJOINT_OWNER),
            '--edge',
            '/tf_static',
            SECOND_DISJOINT_PARENT,
            SECOND_DISJOINT_CHILD,
            node_fqn(SECOND_DISJOINT_OWNER),
            '--edge',
            '/tf',
            DYNAMIC_PARENT,
            DYNAMIC_CHILD,
            node_fqn(DYNAMIC_OWNER),
            '--reject-undeclared',
            '--timeout',
            AUDIT_TIMEOUT,
            '--stable-window',
            '0.75',
        ],
    )
    conflict_sentinel = Node(
        package='voice_nav_sim',
        executable='tf_ownership_auditor',
        name='tf_ownership_conflict_sentinel',
        output='screen',
        arguments=[
            '--expect-conflict',
            '/tf_static',
            PARENT_FRAME,
            CHILD_FRAME,
            node_fqn(DUPLICATE_NODE_NAME),
            '--timeout',
            AUDIT_TIMEOUT,
            '--stable-window',
            '0.75',
        ],
    )
    disjoint_auditor = Node(
        package='voice_nav_sim',
        executable='tf_ownership_auditor',
        name='tf_disjoint_ownership_auditor',
        output='screen',
        arguments=[
            '--edge',
            '/tf_static',
            FIRST_DISJOINT_PARENT,
            FIRST_DISJOINT_CHILD,
            node_fqn(FIRST_DISJOINT_OWNER),
            '--edge',
            '/tf_static',
            SECOND_DISJOINT_PARENT,
            SECOND_DISJOINT_CHILD,
            node_fqn(SECOND_DISJOINT_OWNER),
            '--timeout',
            AUDIT_TIMEOUT,
            '--stable-window',
            '0.75',
        ],
    )
    dynamic_auditor = Node(
        package='voice_nav_sim',
        executable='tf_ownership_auditor',
        name='tf_dynamic_ownership_auditor',
        output='screen',
        arguments=[
            '--edge',
            '/tf',
            DYNAMIC_PARENT,
            DYNAMIC_CHILD,
            node_fqn(DYNAMIC_OWNER),
            '--timeout',
            AUDIT_TIMEOUT,
            '--stable-window',
            '0.75',
        ],
    )
    wrong_owner_auditor = Node(
        package='voice_nav_sim',
        executable='tf_ownership_auditor',
        name='tf_wrong_owner_auditor',
        output='screen',
        arguments=[
            '--edge',
            '/tf',
            DYNAMIC_PARENT,
            DYNAMIC_CHILD,
            WRONG_DYNAMIC_OWNER_FQN,
            '--timeout',
            AUDIT_TIMEOUT,
            '--stable-window',
            '0.75',
        ],
    )

    return (
        LaunchDescription(
            [
                first_owner,
                second_owner,
                first_disjoint_owner,
                second_disjoint_owner,
                dynamic_owner,
                TimerAction(
                    period=1.0,
                    actions=[
                        enforcement_auditor,
                        conflict_sentinel,
                        disjoint_auditor,
                        dynamic_auditor,
                        wrong_owner_auditor,
                    ],
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            'enforcement_auditor': enforcement_auditor,
            'conflict_sentinel': conflict_sentinel,
            'disjoint_auditor': disjoint_auditor,
            'dynamic_auditor': dynamic_auditor,
            'wrong_owner_auditor': wrong_owner_auditor,
        },
    )


class TfOwnershipConflictTest(unittest.TestCase):

    def test_normal_audit_rejects_and_sentinel_proves_the_conflict(
        self,
        proc_info,
        proc_output,
        enforcement_auditor,
        conflict_sentinel,
        disjoint_auditor,
        dynamic_auditor,
        wrong_owner_auditor,
    ):
        proc_output.assertWaitFor(
            expected_output=(
                f'{PARENT_FRAME} -> {CHILD_FRAME} has '
                '2 publisher GID(s); expected 1'
            ),
            process=enforcement_auditor,
            timeout=10.0,
            stream='stderr',
        )
        proc_info.assertWaitForShutdown(
            process=enforcement_auditor,
            timeout=5.0,
        )
        assertExitCodes(
            proc_info,
            process=enforcement_auditor,
            allowable_exit_codes=[1],
        )

        proc_output.assertWaitFor(
            expected_output=(
                f'on /tf maps to {{{node_fqn(DYNAMIC_OWNER)}}}; '
                f'expected {WRONG_DYNAMIC_OWNER_FQN}'
            ),
            process=wrong_owner_auditor,
            timeout=10.0,
            stream='stderr',
        )
        proc_info.assertWaitForShutdown(
            process=wrong_owner_auditor,
            timeout=5.0,
        )
        assertExitCodes(
            proc_info,
            process=wrong_owner_auditor,
            allowable_exit_codes=[1],
        )

        proc_output.assertWaitFor(
            expected_output='TF ownership audit passed',
            process=conflict_sentinel,
            timeout=10.0,
            stream='stderr',
        )
        proc_info.assertWaitForShutdown(
            process=conflict_sentinel,
            timeout=15.0,
        )
        assertExitCodes(
            proc_info,
            process=conflict_sentinel,
            allowable_exit_codes=[0],
        )

        proc_output.assertWaitFor(
            expected_output='TF ownership audit passed',
            process=disjoint_auditor,
            timeout=10.0,
            stream='stderr',
        )
        proc_info.assertWaitForShutdown(
            process=disjoint_auditor,
            timeout=15.0,
        )
        assertExitCodes(
            proc_info,
            process=disjoint_auditor,
            allowable_exit_codes=[0],
        )

        proc_output.assertWaitFor(
            expected_output='TF ownership audit passed',
            process=dynamic_auditor,
            timeout=10.0,
            stream='stderr',
        )
        proc_info.assertWaitForShutdown(
            process=dynamic_auditor,
            timeout=15.0,
        )
        assertExitCodes(
            proc_info,
            process=dynamic_auditor,
            allowable_exit_codes=[0],
        )
