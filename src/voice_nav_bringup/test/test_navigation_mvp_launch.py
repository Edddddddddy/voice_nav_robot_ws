# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Small real-graph Navigation MVP probe for the fixed study place."""

import json
import math
from pathlib import Path
import sys
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
from tf2_ros import Buffer, TransformListener
import pytest
import rclpy
from rclpy.time import Time

_TEST_DIRECTORY = str(Path(__file__).resolve().parent)
if _TEST_DIRECTORY not in sys.path:
    sys.path.insert(0, _TEST_DIRECTORY)

import crash_stop_support as support


NAV2_STATE_SERVICES = {
    'controller_server': '/controller_server/get_state',
    'planner_server': '/planner_server/get_state',
    'behavior_server': '/behavior_server/get_state',
    'bt_navigator': '/bt_navigator/get_state',
}
MAP_FRAME = 'map'
BASE_FRAME = 'base_footprint'
TARGET_X = 0.5
TARGET_Y = 0.0
TARGET_YAW = 0.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    package_share = Path(get_package_share_directory('voice_nav_bringup'))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / 'navigation_mvp.launch.py')
        ),
        launch_arguments={
            'headless': 'true',
            'shutdown_on_gazebo_exit': 'false',
        }.items(),
    )
    tf_auditor = Node(
        package='voice_nav_sim',
        executable='tf_ownership_auditor',
        name='navigation_mvp_tf_ownership_auditor',
        output='screen',
        arguments=[
            '--edge', '/tf', 'map', 'odom', '/amcl',
            '--edge', '/tf', 'odom', 'base_footprint', '/diff_drive_controller',
            '--edge', '/tf', 'base_link', 'left_wheel', '/robot_state_publisher',
            '--edge', '/tf', 'base_link', 'right_wheel', '/robot_state_publisher',
            '--edge', '/tf_static', 'base_footprint', 'base_link',
            '/robot_state_publisher',
            '--edge', '/tf_static', 'base_link', 'caster_link',
            '/robot_state_publisher',
            '--edge', '/tf_static', 'base_link', 'laser_link',
            '/robot_state_publisher',
            '--reject-undeclared',
            '--timeout', '90',
            '--stable-window', '0.75',
        ],
    )
    return LaunchDescription([
        navigation,
        tf_auditor,
        launch_testing.actions.ReadyToTest(),
    ]), {'tf_auditor': tf_auditor}


def _yaw_from_quaternion(quaternion):
    numerator = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    denominator = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(numerator, denominator)


def _wrapped_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class NavigationMvpProductTest(unittest.TestCase):
    """Observe one formal NAVIGATE_TO and its real conditioning stop."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.probe = support.CrashStopProbe()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.probe.node,
            spin_thread=False,
        )
        self.nav2_state_clients = {
            node_name: self.probe.node.create_client(GetState, service_name)
            for node_name, service_name in NAV2_STATE_SERVICES.items()
        }
        self.addCleanup(self.probe.destroy)

    def _wait_nav2_active(self):
        self.probe.wait_until(
            lambda: all(
                client.wait_for_service(timeout_sec=0.2)
                for client in self.nav2_state_clients.values()
            ),
            90.0,
            'Nav2 lifecycle services',
        )

        def active_state():
            states = {}
            for node_name, client in self.nav2_state_clients.items():
                response = self.probe._wait_future(
                    client.call_async(GetState.Request()),
                    5.0,
                    f'Nav2 {node_name} lifecycle response',
                )
                states[node_name] = response.current_state
            if all(
                state.id == State.PRIMARY_STATE_ACTIVE
                for state in states.values()
            ):
                return states['controller_server']
            return None

        return self.probe.wait_until(
            active_state, 90.0, 'Nav2 managed nodes ACTIVE state'
        )

    def _send_study_goal(self, runtime):
        goal = self.probe._execute_mission_type.Goal()
        goal.source_instance_id = 'issue164-navigation-mvp'
        goal.source_seq = 1
        goal.runtime_instance_id = runtime.runtime_instance_id
        goal.admission_epoch = runtime.admission_epoch
        step = self.probe._mission_step_type()
        step.kind = self.probe._mission_step_type.NAVIGATE_TO
        step.target_id = 'study'
        goal.steps.append(step)
        handle = self.probe._wait_future(
            self.probe.action_client.send_goal_async(goal),
            20.0,
            'Navigation NAVIGATE_TO admission',
        )
        self.assertTrue(handle.accepted)
        return handle

    def _wait_final_pose(self):
        def pose():
            try:
                transform = self.tf_buffer.lookup_transform(
                    MAP_FRAME,
                    BASE_FRAME,
                    Time(),
                )
            except Exception:
                return None
            translation = transform.transform.translation
            distance = math.hypot(
                translation.x - TARGET_X,
                translation.y - TARGET_Y,
            )
            yaw_error = abs(_wrapped_angle(
                _yaw_from_quaternion(transform.transform.rotation) - TARGET_YAW
            ))
            if distance <= 0.50 and yaw_error <= 0.50:
                return distance, yaw_error
            return None

        return self.probe.wait_until(
            pose, 20.0, 'final map pose within study-place tolerance'
        )

    def _wait_nonzero_command(self, start_ns):
        return self.probe.wait_until(
            lambda: support.last_nonzero_command_after(
                self.probe._snapshot(self.probe.final_commands), start_ns
            ),
            30.0,
            'conditioned non-zero navigation command',
        )

    def test_navigation_mvp_executes_study_and_ends_zero(
        self,
        proc_info,
        proc_output,
        tf_auditor,
    ):
        self.probe.wait_clock(timeout=45.0)
        self.assertEqual(
            self._wait_nav2_active().id,
            State.PRIMARY_STATE_ACTIVE,
        )
        runtime = self.probe.wait_runtime_ready(timeout=90.0)
        self.assertEqual(
            runtime.operating_mode,
            self.probe._mission_state_type.NAVIGATION,
        )
        self.assertEqual(list(runtime.named_place_ids), ['study'])

        started_ns = time.monotonic_ns()
        handle = self._send_study_goal(runtime)
        status, result = self.probe.wait_goal_result(handle, timeout=120.0)
        support.assert_action_result(
            status,
            result,
            self.probe._execute_mission_type.Result.SUCCEEDED,
        )
        distance, yaw_error = self._wait_final_pose()
        nonzero_receipt_ns, _ = self._wait_nonzero_command(started_ns)
        zero = self.probe.wait_zero_after(nonzero_receipt_ns)
        stationarity = self.probe.wait_stationary(
            zero['zero_sim_ns'], zero['zero_ns']
        )
        final_stationary = (
            stationarity['hold_ns'] >= support.STATIONARY_HOLD_NS
        )
        self.assertTrue(final_stationary)

        proc_output.assertWaitFor(
            expected_output='TF ownership audit passed',
            process=tf_auditor,
            timeout=100.0,
            stream='stderr',
        )
        proc_info.assertWaitForShutdown(tf_auditor, timeout=10.0)
        assertExitCodes(
            proc_info,
            process=tf_auditor,
            allowable_exit_codes=[0],
        )
        print(
            'EVIDENCE issue164_navigation_mvp '
            + json.dumps({
                'nav2_state': 'ACTIVE',
                'map_odom_owner': '/amcl',
                'target': 'study',
                'position_error_m': distance,
                'yaw_error_rad': yaw_error,
                'final_zero': True,
                'final_stationary': final_stationary,
                'stationary_ms': stationarity['hold_ns'] // 1_000_000,
                'zero_sim_ns': zero['zero_sim_ns'],
                'zero_receipt_ns': zero['zero_ns'],
                'stationary_end_sim_ns': stationarity[
                    'stationary_end_sim_ns'
                ],
                'odom_receipt_ns': stationarity[
                    'odom_receipt_ns'
                ],
                'odom_stamp_ns': stationarity['odom_stamp_ns'],
                'joint_receipt_ns': stationarity['joint_receipt_ns'],
                'joint_stamp_ns': stationarity['joint_stamp_ns'],
                'joint_left_velocity': stationarity['joint_left_velocity'],
                'joint_right_velocity': stationarity['joint_right_velocity'],
            }, sort_keys=True, separators=(',', ':')),
            flush=True,
        )


@launch_testing.post_shutdown_test()
class NavigationMvpShutdownTest(unittest.TestCase):
    def test_navigation_mvp_auditor_exits_cleanly(self, proc_info, tf_auditor):
        assertExitCodes(
            proc_info,
            process=tf_auditor,
            allowable_exit_codes=[0, -2],
        )
