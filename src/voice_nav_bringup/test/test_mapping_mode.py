# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Headless Mapping Mode G1-G8 product acceptance."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import importlib.util
import math
import os
from pathlib import Path
import sys
import time
import unittest
import xml.etree.ElementTree as element_tree

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

MAP_TOPIC = '/map'
SLAM_NODE = '/slam_toolbox'
SLAM_POSE_TOPIC = '/pose'
SLAM_STATE_SERVICE = f'{SLAM_NODE}/get_state'
SAVE_MAP_SERVICE = f'{SLAM_NODE}/save_map'
TF_AUDIT_TIMEOUT_SECONDS = 60.0
STARTUP_TIMEOUT_SECONDS = 60.0
ROUTE_TIMEOUT_SECONDS = 240.0
MAP_RESOLUTION = 0.05
MINIMUM_ROUTE_CLEARANCE = 0.24
MINIMUM_KNOWN_FLOOR_RATIO = 0.80
MINIMUM_BOUNDARY_OCCUPANCY_RATIO = 0.70
OCCUPIED_THRESHOLD = 65
BOUNDARY_SEARCH_RADIUS = 0.10
FINAL_HOME_POSITION_TOLERANCE = 0.12
FINAL_HOME_YAW_TOLERANCE = 0.15
FROZEN_ROUTE = (
    (0.0, 0.0),
    (0.0, -1.0),
    (1.5, -1.0),
    (1.5, -1.8),
    (1.5, 1.2),
    (-1.4, 1.2),
    (-1.4, 1.8),
    (-2.0, 1.8),
    (-1.4, 1.8),
    (-1.4, 0.0),
    (0.0, 0.0),
)
FROZEN_GOALS = (
    ('G1', (('rotate', -1.570796), ('move', 1.00), ('rotate', 1.570796))),
    ('G2', (('move', 1.50), ('rotate', -1.570796), ('move', 0.80))),
    ('G3', (('rotate', 3.141593), ('move', 1.50), ('move', 1.50))),
    ('G4', (('rotate', 1.570796), ('move', 1.20), ('move', 1.70))),
    ('G5', (('rotate', -1.570796), ('move', 0.60), ('rotate', 1.570796))),
    ('G6', (('move', 0.60), ('rotate', 3.141593), ('move', 0.60))),
    ('G7', (('rotate', -1.570796), ('move', 1.80), ('rotate', 1.570796))),
    ('G8', (('move', 1.40), ('rotate', 6.283185))),
)


def _load_module(name, path):
    specification = importlib.util.spec_from_file_location(name, path)
    if specification is None or specification.loader is None:
        raise RuntimeError(f'could not load test support: {path}')
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


support = _load_module(
    'voice_nav_mapping_acceptance_support',
    Path(__file__).with_name('crash_stop_support.py'),
)
mode_lock = _load_module(
    'voice_nav_mapping_acceptance_mode_lock',
    Path(get_package_share_directory('voice_nav_bringup')) / 'launch_support' / 'mode_lock.py',
)
mapping_quality = _load_module(
    'voice_nav_mapping_quality_geometry',
    Path(__file__).parents[1] / 'launch_support' / 'mapping_quality.py',
)
mapping_scan_coverage = _load_module(
    'voice_nav_mapping_scan_coverage',
    Path(__file__).parents[1] / 'launch_support' / 'mapping_scan_coverage.py',
)


def _floats(value):
    return tuple(float(component) for component in value.split())


def _yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _wrapped_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _world_geometry():
    path = (
        Path(get_package_share_directory('voice_nav_sim')) / 'worlds' / 'voice_nav_house_world.sdf'
    )
    world = element_tree.parse(path).getroot().find('world')
    geometry = []
    for model in world.findall('model'):
        if model.get('name') == 'ground':
            continue
        pose = _floats(model.findtext('pose'))
        box_size = model.findtext('link/collision/geometry/box/size')
        if box_size is not None:
            size = _floats(box_size)
            geometry.append(
                {
                    'name': model.get('name'),
                    'kind': 'box',
                    'x': pose[0],
                    'y': pose[1],
                    'size_x': size[0],
                    'size_y': size[1],
                }
            )
        else:
            cylinder = model.find('link/collision/geometry/cylinder')
            geometry.append(
                {
                    'name': model.get('name'),
                    'kind': 'cylinder',
                    'x': pose[0],
                    'y': pose[1],
                    'radius': float(cylinder.findtext('radius')),
                }
            )
    return tuple(geometry)


def _clearance(shape, x, y):
    if shape['kind'] == 'box':
        return math.hypot(
            max(abs(x - shape['x']) - shape['size_x'] / 2.0, 0.0),
            max(abs(y - shape['y']) - shape['size_y'] / 2.0, 0.0),
        )
    return max(0.0, math.hypot(x - shape['x'], y - shape['y']) - shape['radius'])


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _pose_record(pose):
    return {
        'x': float(pose.position.x),
        'y': float(pose.position.y),
        'yaw': _yaw(pose.orientation),
    }


def _grid_record(grid):
    return {
        'resolution': float(grid.info.resolution),
        'width': int(grid.info.width),
        'height': int(grid.info.height),
        'origin': _pose_record(grid.info.origin),
        'data': [int(value) for value in grid.data],
    }


def _transform_record(transform):
    return {
        'x': float(transform.transform.translation.x),
        'y': float(transform.transform.translation.y),
        'yaw': _yaw(transform.transform.rotation),
    }


def _mapping_quality(grid, map_from_odom, geometry):
    return mapping_quality.evaluate_mapping_artifact(
        {
            'schema_version': 1,
            'policy': {
                'map_resolution': MAP_RESOLUTION,
                'floor_min': -2.95,
                'floor_max': 2.95,
                'minimum_route_clearance': MINIMUM_ROUTE_CLEARANCE,
                'occupied_threshold': OCCUPIED_THRESHOLD,
                'boundary_search_radius': BOUNDARY_SEARCH_RADIUS,
            },
            'geometry': deepcopy(geometry),
            'route': deepcopy(FROZEN_ROUTE),
            'grid': _grid_record(grid),
            'map_from_odom': _transform_record(map_from_odom),
        }
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    share = Path(get_package_share_directory('voice_nav_bringup'))
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'mapping_sim.launch.py')),
        launch_arguments={'headless': 'true'}.items(),
    )
    arguments = []
    for topic, parent, child, owner in (
        ('/tf', 'map', 'odom', SLAM_NODE),
        ('/tf', 'odom', 'base_footprint', '/diff_drive_controller'),
        ('/tf_static', 'base_footprint', 'base_link', '/robot_state_publisher'),
        ('/tf_static', 'base_link', 'laser_link', '/robot_state_publisher'),
    ):
        arguments.extend(['--edge', topic, parent, child, owner])
    arguments.extend(['--timeout', str(TF_AUDIT_TIMEOUT_SECONDS), '--stable-window', '5.0'])
    auditor = Node(
        package='voice_nav_sim',
        executable='tf_ownership_auditor',
        name='mapping_tf_ownership_auditor',
        output='screen',
        arguments=arguments,
    )
    return LaunchDescription([mapping, auditor, launch_testing.actions.ReadyToTest()]), {
        'tf_auditor': auditor
    }


class MappingModeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.probe = support.CrashStopProbe()
        self.maps = deque(maxlen=120)
        self.geometry = _world_geometry()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=300.0))
        self.tf_listener = TransformListener(self.tf_buffer, self.probe.node, spin_thread=False)
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
            callback_group=self.probe.sensor_observation_group,
        )
        self.slam_poses = deque(maxlen=5000)
        self.pose_subscription = self.probe.node.create_subscription(
            PoseWithCovarianceStamped,
            SLAM_POSE_TOPIC,
            lambda message: self.probe._append(self.slam_poses, message),
            100,
            callback_group=self.probe.sensor_observation_group,
        )
        self.slam_state_client = self.probe.node.create_client(GetState, SLAM_STATE_SERVICE)
        self.addCleanup(self.probe.destroy)

    def _wait_slam_active(self):
        self.probe.wait_until(
            lambda: self.slam_state_client.wait_for_service(timeout_sec=0.2),
            60.0,
            'slam_toolbox lifecycle state service',
        )

        def active():
            response = self.probe._wait_future(
                self.slam_state_client.call_async(GetState.Request()),
                5.0,
                'slam_toolbox lifecycle state response',
            )
            return (
                response.current_state
                if response.current_state.id == State.PRIMARY_STATE_ACTIVE
                else None
            )

        return self.probe.wait_until(active, 60.0, 'slam_toolbox ACTIVE state')

    def _wait_map(self):
        return self.probe.wait_until(
            lambda: next(
                (
                    deepcopy(message)
                    for _, message in reversed(self.probe._snapshot(self.maps))
                    if abs(message.info.resolution - MAP_RESOLUTION) <= 1.0e-9
                    and message.info.width
                    and message.info.height
                    and len(message.data) == message.info.width * message.info.height
                ),
                None,
            ),
            STARTUP_TIMEOUT_SECONDS,
            '0.05 m OccupancyGrid',
        )

    def _steps(self, specification):
        steps = []
        for kind, value in specification:
            step = support.MissionStep()
            if kind == 'move':
                step.kind, step.distance_m = support.MissionStep.MOVE_DISTANCE, value
            else:
                step.kind, step.angle_rad = support.MissionStep.ROTATE_ANGLE, value
            steps.append(step)
        return tuple(steps)

    def _route(self, runtime):
        started_wall_ns = time.monotonic_ns()
        started_ros_ns = self.probe.node.get_clock().now().nanoseconds
        deadline = time.monotonic() + ROUTE_TIMEOUT_SECONDS
        for source_seq, (name, specification) in enumerate(FROZEN_GOALS, 1):
            segment_started_ns = time.monotonic_ns()
            handle = self.probe.send_steps(
                runtime,
                source_instance_id='issue38-frozen-route',
                source_seq=source_seq,
                steps=self._steps(specification),
            )
            status, result = self.probe.wait_goal_result(
                handle, timeout=min(45.0, max(1.0, deadline - time.monotonic()))
            )
            try:
                support.assert_action_result(
                    status, result, support.ExecuteMission.Result.SUCCEEDED
                )
            except AssertionError as error:
                details = (
                    f'goal={name}, status={status}, code={result.code}, '
                    f'failed_step={result.failed_step}, detail={result.detail!r}, '
                    f'diagnostic={self.probe.diagnostic()}'
                )
                raise AssertionError(
                    f'G1-G8 route segment failed: {details}'
                ) from error
            self.probe.wait_zero_after(segment_started_ns)
            runtime = self.probe.wait_runtime_ready(
                after_monotonic_ns=segment_started_ns, timeout=20.0
            )
        return (
            started_wall_ns,
            started_ros_ns,
            self.probe.node.get_clock().now().nanoseconds,
            runtime,
        )

    def test_mapping_composition_and_frozen_route(self, proc_info, proc_output, tf_auditor):
        self.probe.wait_clock(timeout=30.0)
        self.assertEqual(self._wait_slam_active().id, State.PRIMARY_STATE_ACTIVE)
        runtime = self.probe.wait_runtime_ready(timeout=STARTUP_TIMEOUT_SECONDS)
        self.assertEqual(runtime.operating_mode, support.MissionState.MAPPING)
        self._wait_map()
        self.assertNotIn(SAVE_MAP_SERVICE, dict(self.probe.node.get_service_names_and_types()))
        with self.assertRaises(mode_lock.ModeLockConflict):
            mode_lock.acquire_mode_lock(mode='navigation')
        other_environment = dict(os.environ)
        other_environment['ROS_DOMAIN_ID'] = str(
            (int(os.environ.get('ROS_DOMAIN_ID', '0')) + 1) % 233
        )
        with mode_lock.acquire_mode_lock(mode='navigation', environment=other_environment):
            pass
        started_wall_ns, started_ros_ns, completed_ros_ns, runtime = self._route(runtime)
        zero = self.probe.wait_zero_after(started_wall_ns)
        stationarity = self.probe.wait_stationary(zero['zero_sim_ns'], zero['zero_ns'])
        odom = [
            (receipt, message)
            for receipt, message in self.probe._snapshot(self.probe.odometry)
            if receipt >= started_wall_ns
        ]
        self.assertGreater(len(odom), 40)
        clearance = min(
            _clearance(shape, message.pose.pose.position.x, message.pose.pose.position.y)
            for _, message in odom
            for shape in self.geometry
        )
        self.assertGreaterEqual(clearance, MINIMUM_ROUTE_CLEARANCE)
        final_pose = odom[-1][1].pose.pose
        self.assertLessEqual(
            math.hypot(final_pose.position.x, final_pose.position.y), FINAL_HOME_POSITION_TOLERANCE
        )
        self.assertLessEqual(
            abs(_wrapped_angle(_yaw(final_pose.orientation))), FINAL_HOME_YAW_TOLERANCE
        )
        scans = tuple(
            (_stamp_ns(message.header.stamp), deepcopy(message))
            for _, message in self.probe._snapshot(self.probe.scan_samples)
        )
        retained = mapping_scan_coverage.verify_route_scan_transform_coverage(
            scans,
            route_started_ns=started_ros_ns,
            route_completed_ns=completed_ros_ns,
            lookup_transform=lambda target, source, scan: self.tf_buffer.lookup_transform(
                target, source, Time.from_msg(scan.header.stamp), timeout=Duration(seconds=0.1)
            ),
        )
        self.assertGreaterEqual(len(retained), 40)
        route_map_receipt = self.probe.latest(self.maps)[0]
        quality = None
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            sample = self.probe.latest(self.maps)
            if sample is None or sample[0] <= route_map_receipt:
                time.sleep(0.05)
                continue
            grid = sample[1]
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 'odom', Time.from_msg(grid.header.stamp), timeout=Duration(seconds=2.0)
                )
            except Exception:
                continue
            quality = _mapping_quality(grid, transform, self.geometry)
            if (
                quality['known_floor_ratio'] >= MINIMUM_KNOWN_FLOOR_RATIO
                and quality['minimum_boundary_ratio'] >= MINIMUM_BOUNDARY_OCCUPANCY_RATIO
            ):
                break
        self.assertIsNotNone(quality)
        self.assertGreaterEqual(quality['known_floor_ratio'], MINIMUM_KNOWN_FLOOR_RATIO)
        self.assertGreaterEqual(
            quality['minimum_boundary_ratio'], MINIMUM_BOUNDARY_OCCUPANCY_RATIO
        )
        rotate = support.MissionStep()
        rotate.kind, rotate.angle_rad = support.MissionStep.ROTATE_ANGLE, 6.283185
        handle = self.probe.send_steps(
            runtime, source_instance_id='issue38-cancel-proof', source_seq=1, steps=(rotate,)
        )
        self.probe.wait_for_armed_motion(timeout=15.0)
        canceled_at = time.monotonic_ns()
        response = self.probe._wait_future(
            handle.cancel_goal_async(), 10.0, 'Mapping cancel response'
        )
        self.assertTrue(response.goals_canceling)
        status, result = self.probe.wait_goal_result(handle, timeout=20.0)
        self.assertEqual(status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(result.code, support.ExecuteMission.Result.CANCELED)
        cancel_zero = self.probe.wait_zero_after(canceled_at)
        cancel_stationarity = self.probe.wait_stationary(
            cancel_zero['zero_sim_ns'], cancel_zero['zero_ns']
        )
        self.assertEqual(self._wait_slam_active().id, State.PRIMARY_STATE_ACTIVE)
        proc_output.assertWaitFor(
            expected_output=(
                'TF ownership audit passed after full '
                f'{TF_AUDIT_TIMEOUT_SECONDS:.3f} s observation window'
            ),
            process=tf_auditor,
            timeout=75.0,
            stream='stderr',
        )
        proc_info.assertWaitForShutdown(tf_auditor, timeout=10.0)
        assertExitCodes(proc_info, process=tf_auditor, allowable_exit_codes=[0])
        evidence = (
            f'EVIDENCE issue38_mapping map={grid.info.width}x{grid.info.height}'
            f'@{grid.info.resolution:.3f} min_clearance={clearance:.3f} '
            f'known_floor_ratio={quality["known_floor_ratio"]:.3f} '
            f'min_boundary_ratio={quality["minimum_boundary_ratio"]:.3f} '
            f'scan_tf_bucket_count={len(retained)} '
            f'settle_ms={stationarity["settle_ns"] / 1_000_000:.3f} '
            f'cancel_settle_ms={cancel_stationarity["settle_ns"] / 1_000_000:.3f}'
        )
        print(evidence, flush=True)


@launch_testing.post_shutdown_test()
class MappingModeShutdownTest(unittest.TestCase):
    def test_all_launch_managed_processes_exit_cleanly(self, proc_info):
        assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
