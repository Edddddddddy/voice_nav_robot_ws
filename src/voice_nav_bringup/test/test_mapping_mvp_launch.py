# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Thin Mapping MVP product probe for the one-day Issue #164 path."""

from collections import deque
from copy import deepcopy
import sys
import time
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_testing.asserts import assertExitCodes
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState, MissionStep
from voice_nav_interfaces.srv import StopMission

_TEST_DIRECTORY = str(Path(__file__).resolve().parent)
if _TEST_DIRECTORY not in sys.path:
    sys.path.insert(0, _TEST_DIRECTORY)

import crash_stop_support as support


MAP_TOPIC = '/map'
SLAM_STATE_SERVICE = '/slam_toolbox/get_state'
STOP_SERVICE = '/mission/stop'
SCAN_FRAME = 'laser_link'


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    share = Path(get_package_share_directory('voice_nav_bringup'))
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'mapping_mvp.launch.py')
        ),
        launch_arguments={
            'headless': 'true',
            'shutdown_on_gazebo_exit': 'false',
        }.items(),
    )
    tf_auditor = Node(
        package='voice_nav_sim',
        executable='tf_ownership_auditor',
        name='mapping_mvp_tf_ownership_auditor',
        output='screen',
        arguments=[
            '--edge', '/tf', 'map', 'odom', '/slam_toolbox',
            '--edge', '/tf', 'odom', 'base_footprint', '/diff_drive_controller',
            '--edge', '/tf', 'base_link', 'left_wheel', '/robot_state_publisher',
            '--edge', '/tf', 'base_link', 'right_wheel', '/robot_state_publisher',
            '--edge', '/tf_static', 'base_footprint', 'base_link',
            '/robot_state_publisher',
            '--edge', '/tf_static', 'base_link', 'caster_link',
            '/robot_state_publisher',
            '--edge', '/tf_static', 'base_link', SCAN_FRAME,
            '/robot_state_publisher',
            '--reject-undeclared',
            '--timeout', '60',
            '--stable-window', '0.75',
        ],
    )
    return LaunchDescription([
        mapping,
        tf_auditor,
        launch_testing.actions.ReadyToTest(),
    ]), {'tf_auditor': tf_auditor}


class MappingMvpProductTest(unittest.TestCase):
    """Observe one real Mapping mission and its safety stop boundary."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.probe = support.CrashStopProbe()
        self.maps = deque(maxlen=64)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscription = self.probe.node.create_subscription(
            OccupancyGrid,
            MAP_TOPIC,
            lambda message: self.probe._append(self.maps, message),
            map_qos,
        )
        self.slam_state_client = self.probe.node.create_client(
            GetState,
            SLAM_STATE_SERVICE,
        )
        self.stop_client = self.probe.node.create_client(
            StopMission,
            STOP_SERVICE,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.probe.node,
            spin_thread=False,
        )
        self.addCleanup(self.probe.destroy)

    def _wait_slam_active(self):
        self.probe.wait_until(
            lambda: self.slam_state_client.wait_for_service(timeout_sec=0.2),
            60.0,
            'slam_toolbox lifecycle state service',
        )

        def active_state():
            response = self.probe._wait_future(
                self.slam_state_client.call_async(GetState.Request()),
                5.0,
                'slam_toolbox lifecycle state response',
            )
            if response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                return response.current_state
            return None

        return self.probe.wait_until(
            active_state,
            60.0,
            'slam_toolbox ACTIVE state',
        )

    def _wait_map_with_known_and_unknown(self):
        def candidate():
            for _, message in reversed(self.probe._snapshot(self.maps)):
                values = list(message.data)
                if (
                    message.info.width > 0
                    and message.info.height > 0
                    and len(values) == message.info.width * message.info.height
                    and any(value == -1 for value in values)
                    and any(value >= 0 for value in values)
                ):
                    return deepcopy(message)
            return None

        return self.probe.wait_until(
            candidate,
            60.0,
            'non-empty OccupancyGrid with known and unknown cells',
        )

    def _wait_scan_time_tf(self):
        def candidate():
            for _, message in reversed(self.probe._snapshot(self.probe.scan_samples)):
                if message.header.frame_id != SCAN_FRAME:
                    continue
                if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
                    continue
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'base_footprint',
                        SCAN_FRAME,
                        Time.from_msg(message.header.stamp),
                        timeout=Duration(seconds=0.2),
                    )
                except Exception:
                    continue
                return message, transform
            return None

        return self.probe.wait_until(
            candidate,
            30.0,
            'scan-time base_footprint -> laser_link TF',
        )

    def _send_move_rotate(self, runtime):
        goal = ExecuteMission.Goal()
        goal.source_instance_id = 'issue164-mapping-mvp'
        goal.source_seq = 1
        goal.runtime_instance_id = runtime.runtime_instance_id
        goal.admission_epoch = runtime.admission_epoch
        move = MissionStep()
        move.kind = MissionStep.MOVE_DISTANCE
        move.distance_m = 0.25
        rotate = MissionStep()
        rotate.kind = MissionStep.ROTATE_ANGLE
        rotate.angle_rad = 0.5
        goal.steps.append(move)
        goal.steps.append(rotate)
        handle = self.probe._wait_future(
            self.probe.action_client.send_goal_async(goal),
            15.0,
            'Mapping MOVE/ROTATE ExecuteMission admission',
        )
        self.assertTrue(handle.accepted)
        return handle

    def _stop(self):
        self.assertTrue(self.stop_client.wait_for_service(timeout_sec=10.0))
        request = StopMission.Request()
        request.request_id = 'issue164-mapping-stop'
        request.source_instance_id = 'issue164-mapping-probe'
        request.source_seq = 1
        request.reason = 'Issue #164 Mapping MVP final STOP'
        started_ns = time.monotonic_ns()
        response = self.probe._wait_future(
            self.stop_client.call_async(request),
            15.0,
            'Mapping MVP STOP response',
        )
        self.assertEqual(response.code, StopMission.Response.APPLIED)
        self.assertTrue(response.motion_inhibited)
        return started_ns

    def test_mapping_mvp_observes_slam_map_tf_and_safe_stop(
        self,
        proc_info,
        proc_output,
        tf_auditor,
    ):
        self.probe.wait_clock(timeout=30.0)
        self.assertEqual(
            self._wait_slam_active().id,
            State.PRIMARY_STATE_ACTIVE,
        )
        runtime = self.probe.wait_runtime_ready(timeout=60.0)
        self.assertEqual(runtime.operating_mode, MissionState.MAPPING)

        handle = self._send_move_rotate(runtime)
        status, result = self.probe.wait_goal_result(handle, timeout=90.0)
        support.assert_action_result(
            status,
            result,
            ExecuteMission.Result.SUCCEEDED,
        )

        stop_started_ns = self._stop()
        zero = self.probe.wait_zero_after(stop_started_ns)
        stationary = self.probe.wait_stationary(
            zero['zero_sim_ns'],
            zero['zero_ns'],
            timeout=10.0,
        )
        grid = self._wait_map_with_known_and_unknown()
        scan, _ = self._wait_scan_time_tf()

        proc_output.assertWaitFor(
            expected_output='TF ownership audit passed',
            process=tf_auditor,
            timeout=75.0,
            stream='stderr',
        )
        proc_info.assertWaitForShutdown(tf_auditor, timeout=10.0)
        assertExitCodes(
            proc_info,
            process=tf_auditor,
            allowable_exit_codes=[0],
        )
        self.assertGreaterEqual(stationary['hold_ns'], 200_000_000)
        print(
            'EVIDENCE issue164_mapping_mvp '
            + str({
                'slam_state': 'ACTIVE',
                'map_cells': len(grid.data),
                'map_known_cells': sum(value >= 0 for value in grid.data),
                'map_unknown_cells': sum(value == -1 for value in grid.data),
                'scan_frame': scan.header.frame_id,
                'stop_code': StopMission.Response.APPLIED,
                'stationary_hold_ns': stationary['hold_ns'],
                'final_zero': True,
            }),
            flush=True,
        )


@launch_testing.post_shutdown_test()
class MappingMvpShutdownTest(unittest.TestCase):
    def test_mapping_mvp_auditor_exits_cleanly(self, proc_info, tf_auditor):
        assertExitCodes(
            proc_info,
            process=tf_auditor,
            allowable_exit_codes=[0, -2],
        )
