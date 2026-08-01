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

from collections import deque
import importlib.util
import math
from pathlib import Path
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer
from tf2_ros import TransformListener


COMMAND_TOPIC = '/diff_drive_controller/cmd_vel'
ODOMETRY_TOPIC = '/odom'
LEGACY_ODOMETRY_TOPIC = '/diff_drive_controller/odom'
WORLD_BOX_FRONT_X = 1.75
WORLD_BOX_HALF_WIDTH_Y = 0.50
SCAN_COUNT = 360
TF_AUDIT_TIMEOUT_SECONDS = 35.0
SIMULATION_TEST_PARTITION = 'voice_nav_l0008_sim_test'

TF_EXPECTATIONS = (
    ('/tf', 'odom', 'base_footprint', '/diff_drive_controller'),
    (
        '/tf_static',
        'base_footprint',
        'base_link',
        '/robot_state_publisher',
    ),
    (
        '/tf_static',
        'base_link',
        'caster_link',
        '/robot_state_publisher',
    ),
    (
        '/tf_static',
        'base_link',
        'laser_link',
        '/robot_state_publisher',
    ),
    (
        '/tf',
        'base_link',
        'left_wheel',
        '/robot_state_publisher',
    ),
    (
        '/tf',
        'base_link',
        'right_wheel',
        '/robot_state_publisher',
    ),
)


def load_gazebo_shutdown_support():
    support_path = (
        Path(get_package_share_directory('voice_nav_sim'))
        / 'test_support'
        / 'gazebo_shutdown.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_simulation_interfaces_gazebo_shutdown',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load Gazebo shutdown test support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


gazebo_shutdown = load_gazebo_shutdown_support()


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    package_share = get_package_share_directory('voice_nav_sim')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{package_share}/launch/simulation.launch.py'
        ),
        launch_arguments={
            'headless': 'true',
            'shutdown_on_gazebo_exit': 'false',
        }.items(),
    )

    audit_arguments = []
    for topic, parent, child, owner in TF_EXPECTATIONS:
        audit_arguments.extend(
            ['--edge', topic, parent, child, owner]
        )
    audit_arguments.extend(
        [
            '--reject-undeclared',
            '--timeout',
            str(TF_AUDIT_TIMEOUT_SECONDS),
            '--stable-window',
            '3.0',
        ]
    )
    tf_ownership_auditor = Node(
        package='voice_nav_sim',
        executable='tf_ownership_auditor',
        name='lesson_0008_tf_ownership_auditor',
        output='screen',
        arguments=audit_arguments,
    )

    return (
        LaunchDescription(
            [
                simulation,
                tf_ownership_auditor,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {'tf_ownership_auditor': tf_ownership_auditor},
    )


def stamp_nanoseconds(message) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def angular_distance(first: float, second: float) -> float:
    return abs(
        math.atan2(
            math.sin(first - second),
            math.cos(first - second),
        )
    )


def endpoint_node_fqn(endpoint) -> str:
    namespace = endpoint.node_namespace.rstrip('/')
    if not namespace:
        return f'/{endpoint.node_name}'
    if not namespace.startswith('/'):
        namespace = f'/{namespace}'
    return f'{namespace}/{endpoint.node_name}'


def endpoint_node_identity_is_known(endpoint) -> bool:
    return (
        bool(endpoint.node_name)
        and bool(endpoint.node_namespace)
        and endpoint.node_name != '_NODE_NAME_UNKNOWN_'
        and endpoint.node_namespace != '_NODE_NAMESPACE_UNKNOWN_'
    )


def endpoint_identity(endpoint):
    return (
        endpoint_node_fqn(endpoint),
        bytes(int(value) & 0xFF for value in endpoint.endpoint_gid),
    )


class SimulationInterfacesTest(unittest.TestCase):

    def setUp(self, proc_info):
        self.addCleanup(self.cleanup_fixture, proc_info)
        rclpy.init()
        self.node = rclpy.create_node(
            'voice_nav_simulation_interfaces_test',
            parameter_overrides=[
                Parameter('use_sim_time', value=True),
            ],
            automatically_declare_parameters_from_overrides=True,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )

        self.samples_lock = threading.Lock()
        self.scans: deque[LaserScan] = deque(maxlen=300)
        self.odometry: deque[Odometry] = deque(maxlen=1000)
        self.subscriptions = [
            self.node.create_subscription(
                LaserScan,
                '/scan',
                lambda message: self.append_sample(self.scans, message),
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                Odometry,
                ODOMETRY_TOPIC,
                lambda message: self.append_sample(
                    self.odometry,
                    message,
                ),
                50,
            ),
        ]
        self.command_publisher = self.node.create_publisher(
            TwistStamped,
            COMMAND_TOPIC,
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.node,
            spin_thread=False,
        )
        self.spin_thread.start()

    def cleanup_fixture(self, proc_info):
        if (
            rclpy.ok()
            and getattr(self, 'node', None) is not None
            and getattr(self, 'command_publisher', None) is not None
        ):
            try:
                self.publish_command_for(0.0, 0.0, 0.25)
            except Exception:
                pass
        try:
            gazebo_shutdown.structured_stop_gazebo(
                proc_info,
                expected_partition=SIMULATION_TEST_PARTITION,
            )
        finally:
            self.destroy_ros_fixture()

    def destroy_ros_fixture(self):
        executor = getattr(self, 'executor', None)
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=2.0)
            except Exception:
                pass
        spin_thread = getattr(self, 'spin_thread', None)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        tf_listener = getattr(self, 'tf_listener', None)
        if tf_listener is not None:
            try:
                tf_listener.unregister()
            except Exception:
                pass
        node = getattr(self, 'node', None)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()

    def append_sample(self, samples, message):
        with self.samples_lock:
            samples.append(message)

    def sample_snapshot(self, samples):
        with self.samples_lock:
            return tuple(samples)

    def latest_sample(self, samples):
        with self.samples_lock:
            return samples[-1] if samples else None

    def wait_until(self, predicate, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.02)
        self.fail(f'timed out waiting for {description}')

    def ros_time(self, message) -> Time:
        return Time.from_msg(
            message.header.stamp,
            clock_type=self.node.get_clock().clock_type,
        )

    def transform_for_message(self, target, source, message):
        message_time = self.ros_time(message)
        if not self.tf_buffer.can_transform(
            target,
            source,
            message_time,
            timeout=Duration(seconds=0.0),
        ):
            return None
        return self.tf_buffer.lookup_transform(
            target,
            source,
            message_time,
            timeout=Duration(seconds=0.0),
        )

    def scans_after(self, stamp: int):
        return tuple(
            scan
            for scan in self.sample_snapshot(self.scans)
            if stamp_nanoseconds(scan) > stamp
        )

    def three_scans_with_exact_tf_after(self, stamp: int):
        candidates = self.scans_after(stamp)
        if len(candidates) < 3:
            return None
        selected = candidates[-3:]
        stamps = [stamp_nanoseconds(scan) for scan in selected]
        if any(
            current <= previous
            for previous, current in zip(stamps, stamps[1:])
        ):
            return None
        transforms = [
            self.transform_for_message('odom', 'laser_link', scan)
            for scan in selected
        ]
        if any(transform is None for transform in transforms):
            return None
        return selected, transforms

    def odometry_with_exact_tf(self, minimum_stamp: int = -1):
        for message in reversed(self.sample_snapshot(self.odometry)):
            if stamp_nanoseconds(message) < minimum_stamp:
                continue
            transform = self.transform_for_message(
                'odom',
                'base_footprint',
                message,
            )
            if transform is not None:
                return message, transform
        return None

    def publisher_endpoints(self, topic):
        return tuple(self.node.get_publishers_info_by_topic(topic))

    def single_scan_endpoint(self):
        endpoints = self.publisher_endpoints('/scan')
        if (
            len(endpoints) != 1
            or not endpoint_node_identity_is_known(endpoints[0])
        ):
            return None
        return endpoints[0]

    def odometry_endpoint_set(self):
        return frozenset(
            endpoint_identity(endpoint)
            for endpoint in self.publisher_endpoints(ODOMETRY_TOPIC)
        )

    def stable_product_endpoint_snapshot(
        self,
        expected_odometry,
        expected_scan,
    ):
        current_odometry = self.odometry_endpoint_set()
        legacy_odometry = self.publisher_endpoints(
            LEGACY_ODOMETRY_TOPIC
        )
        scan_endpoints = self.publisher_endpoints('/scan')
        if (
            current_odometry != expected_odometry
            or legacy_odometry
            or len(scan_endpoints) != 1
            or not endpoint_node_identity_is_known(scan_endpoints[0])
            or endpoint_identity(scan_endpoints[0]) != expected_scan
        ):
            return None
        return current_odometry, endpoint_identity(scan_endpoints[0])

    def expected_odometry_endpoints(self):
        endpoints = self.publisher_endpoints(ODOMETRY_TOPIC)
        if any(
            not endpoint_node_identity_is_known(endpoint)
            for endpoint in endpoints
        ):
            return None
        owners = {
            endpoint_node_fqn(endpoint)
            for endpoint in endpoints
        }
        if len(endpoints) != 1 or owners != {'/diff_drive_controller'}:
            return None
        if endpoints[0].topic_type != 'nav_msgs/msg/Odometry':
            return None
        if self.publisher_endpoints(LEGACY_ODOMETRY_TOPIC):
            return None
        identities = frozenset(
            endpoint_identity(endpoint)
            for endpoint in endpoints
        )
        if any(not any(gid) for _owner, gid in identities):
            return None
        return identities

    def publish_command_for(
        self,
        linear_x: float,
        angular_z: float,
        wall_seconds: float,
    ):
        deadline = time.monotonic() + wall_seconds
        while time.monotonic() < deadline:
            message = TwistStamped()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.twist.linear.x = linear_x
            message.twist.angular.z = angular_z
            self.command_publisher.publish(message)
            time.sleep(0.04)

    def assert_scan_geometry(self, scan: LaserScan):
        self.assertEqual(scan.header.frame_id, 'laser_link')
        self.assertEqual(len(scan.ranges), SCAN_COUNT)
        self.assertAlmostEqual(scan.angle_min, -math.pi, delta=1e-5)
        self.assertAlmostEqual(scan.angle_max, math.pi, delta=1e-5)
        self.assertAlmostEqual(
            scan.angle_increment,
            (scan.angle_max - scan.angle_min) / (SCAN_COUNT - 1),
            delta=1e-7,
        )
        self.assertAlmostEqual(scan.range_min, 0.05, delta=1e-6)
        self.assertAlmostEqual(scan.range_max, 8.0, delta=1e-6)

    def assert_box_front_range(self, scan, odom_to_laser):
        beam_index = min(
            range(len(scan.ranges)),
            key=lambda index: abs(
                scan.angle_min + index * scan.angle_increment
            ),
        )
        beam_angle = (
            scan.angle_min + beam_index * scan.angle_increment
        )
        transform = odom_to_laser.transform
        laser_yaw = yaw_from_quaternion(transform.rotation)
        world_ray_angle = laser_yaw + beam_angle
        world_ray_x = math.cos(world_ray_angle)
        self.assertGreater(
            world_ray_x,
            0.99,
            'nearest-zero laser beam must point toward the test box',
        )

        expected_range = (
            WORLD_BOX_FRONT_X - transform.translation.x
        ) / world_ray_x
        intersection_y = (
            transform.translation.y
            + expected_range * math.sin(world_ray_angle)
        )
        self.assertLessEqual(
            abs(intersection_y),
            WORLD_BOX_HALF_WIDTH_Y,
            'analytical ray misses the fixed test box front face',
        )
        self.assertTrue(math.isfinite(scan.ranges[beam_index]))
        # The configured range resolution is 0.01 m.  An additional
        # centimetre covers float conversion and GPU ray/surface tolerance.
        self.assertAlmostEqual(
            scan.ranges[beam_index],
            expected_range,
            delta=0.02,
        )
        self.node.get_logger().info(
            'analytic_box_range='
            f'beam_index:{beam_index},'
            f'beam_angle_rad:{beam_angle:.6f},'
            f'expected_m:{expected_range:.3f},'
            f'observed_m:{scan.ranges[beam_index]:.3f},'
            'tolerance_m:0.020'
        )

    def assert_odometry_matches_tf(self, odometry, transform):
        self.assertEqual(odometry.header.frame_id, 'odom')
        self.assertEqual(odometry.child_frame_id, 'base_footprint')
        self.assertEqual(
            stamp_nanoseconds(odometry),
            stamp_nanoseconds(transform),
        )
        pose = odometry.pose.pose
        tf_value = transform.transform
        self.assertAlmostEqual(
            pose.position.x,
            tf_value.translation.x,
            delta=1e-5,
        )
        self.assertAlmostEqual(
            pose.position.y,
            tf_value.translation.y,
            delta=1e-5,
        )
        self.assertAlmostEqual(
            pose.position.z,
            tf_value.translation.z,
            delta=1e-5,
        )
        self.assertLessEqual(
            angular_distance(
                yaw_from_quaternion(pose.orientation),
                yaw_from_quaternion(tf_value.rotation),
            ),
            1e-5,
        )

    def test_perception_odom_tf_and_ownership_contract(
        self,
        proc_info,
        proc_output,
        tf_ownership_auditor,
    ):
        self.wait_until(
            lambda: self.node.get_clock().now().nanoseconds > 0,
            30.0,
            'advancing simulation clock',
        )
        self.wait_until(
            lambda: self.command_publisher.get_subscription_count() == 1,
            45.0,
            'one differential-drive command subscriber',
        )
        scan_endpoint = self.wait_until(
            self.single_scan_endpoint,
            30.0,
            'exactly one /scan publisher endpoint',
        )
        self.assertEqual(
            endpoint_node_fqn(scan_endpoint),
            '/simulation_bridge',
        )
        self.assertEqual(
            scan_endpoint.topic_type,
            'sensor_msgs/msg/LaserScan',
        )
        self.assertEqual(
            scan_endpoint.qos_profile.reliability,
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.assertEqual(
            scan_endpoint.qos_profile.durability,
            DurabilityPolicy.VOLATILE,
        )
        initial_scan_identity = endpoint_identity(scan_endpoint)
        self.assertTrue(any(initial_scan_identity[1]))
        self.node.get_logger().info(
            'scan_endpoint='
            f'owner:{endpoint_node_fqn(scan_endpoint)},'
            f'type:{scan_endpoint.topic_type},'
            'reliability:BEST_EFFORT,durability:VOLATILE'
        )
        initial_owner_set = self.wait_until(
            self.expected_odometry_endpoints,
            45.0,
            (
                'one /odom publisher owned by /diff_drive_controller and '
                'no legacy private odometry publisher'
            ),
        )

        initial_scans, initial_scan_transforms = self.wait_until(
            lambda: self.three_scans_with_exact_tf_after(-1),
            45.0,
            'three increasing scans with exact-time odom to laser TF',
        )
        initial_stamps = [
            stamp_nanoseconds(scan)
            for scan in initial_scans
        ]
        self.assertTrue(
            all(
                current > previous
                for previous, current in zip(
                    initial_stamps,
                    initial_stamps[1:],
                )
            )
        )
        odom_owner, odom_gid = next(iter(initial_owner_set))
        self.node.get_logger().info(
            'odometry_endpoint='
            f'owner:{odom_owner},gid:{odom_gid.hex()},'
            f'legacy_publishers:{len(self.publisher_endpoints(LEGACY_ODOMETRY_TOPIC))}'
        )
        for scan in initial_scans:
            self.assert_scan_geometry(scan)
        self.node.get_logger().info(
            'initial_scan_stamps_ns='
            + ','.join(str(stamp) for stamp in initial_stamps)
        )
        self.assert_box_front_range(
            initial_scans[-1],
            initial_scan_transforms[-1],
        )

        initial_odom, initial_odom_tf = self.wait_until(
            self.odometry_with_exact_tf,
            20.0,
            'timestamp-matched odometry and odom to base TF',
        )
        self.assert_odometry_matches_tf(initial_odom, initial_odom_tf)
        initial_x = initial_odom.pose.pose.position.x
        last_initial_scan_stamp = initial_stamps[-1]
        self.node.get_logger().info(
            'matched_initial_odom_tf='
            f'stamp_ns:{stamp_nanoseconds(initial_odom)},'
            f'x_m:{initial_x:.3f},'
            f'y_m:{initial_odom.pose.pose.position.y:.3f},'
            f'yaw_rad:{yaw_from_quaternion(initial_odom.pose.pose.orientation):.3f}'
        )

        self.publish_command_for(0.12, 0.0, 0.80)
        self.publish_command_for(0.0, 0.0, 0.30)

        final_odom = self.wait_until(
            lambda: (
                message
                if (
                    (message := self.latest_sample(self.odometry))
                    and message.pose.pose.position.x > initial_x + 0.02
                    and abs(message.twist.twist.linear.x) < 0.01
                    and abs(message.twist.twist.angular.z) < 0.01
                )
                else None
            ),
            15.0,
            'bounded forward motion followed by explicit zero velocity',
        )
        travelled = final_odom.pose.pose.position.x - initial_x
        self.assertGreater(travelled, 0.02)
        self.assertLess(
            travelled,
            0.25,
            'bounded command moved farther than its integration envelope',
        )
        self.node.get_logger().info(
            f'bounded_forward_travel_m={travelled:.3f}'
        )

        final_odom_with_tf = self.wait_until(
            lambda: self.odometry_with_exact_tf(
                stamp_nanoseconds(final_odom)
            ),
            10.0,
            'post-motion timestamp-matched odometry and TF',
        )
        self.assert_odometry_matches_tf(*final_odom_with_tf)

        final_scans, final_scan_transforms = self.wait_until(
            lambda: self.three_scans_with_exact_tf_after(
                last_initial_scan_stamp
            ),
            15.0,
            'post-motion scans with exact-time odom to laser TF',
        )
        for scan in final_scans:
            self.assert_scan_geometry(scan)
        self.assertTrue(all(final_scan_transforms))
        self.node.get_logger().info(
            'post_motion_scan_stamps_ns='
            + ','.join(
                str(stamp_nanoseconds(scan))
                for scan in final_scans
            )
        )

        self.wait_until(
            lambda: self.stable_product_endpoint_snapshot(
                initial_owner_set,
                initial_scan_identity,
            ),
            5.0,
            'unchanged direct odometry and scan publisher endpoints',
        )

        proc_output.assertWaitFor(
            expected_output=(
                'TF ownership audit passed after full '
                f'{TF_AUDIT_TIMEOUT_SECONDS:.3f} s observation window'
            ),
            process=tf_ownership_auditor,
            timeout=55.0,
            stream='stderr',
        )
        proc_info.assertWaitForShutdown(
            process=tf_ownership_auditor,
            timeout=10.0,
        )
        assertExitCodes(
            proc_info,
            process=tf_ownership_auditor,
            allowable_exit_codes=[0],
        )


@launch_testing.post_shutdown_test()
class SimulationInterfacesShutdownTest(unittest.TestCase):

    def test_all_launch_managed_processes_exit_cleanly(self, proc_info):
        assertExitCodes(proc_info)
