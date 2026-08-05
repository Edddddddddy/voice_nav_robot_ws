from collections import deque
import importlib.util
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import ListHardwareInterfaces
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


COMMAND_TOPIC = '/diff_drive_controller/cmd_vel'
LIMITED_COMMAND_TOPIC = '/diff_drive_controller/cmd_vel_out'
ODOMETRY_TOPIC = '/odom'
GAZEBO_POSE_TOPIC = '/world/voice_nav_test_world/pose/info'
CONTROLLER_TIMEOUT_SECONDS = 0.35
CONTROL_PERIOD_SECONDS = 0.01
SIMULATION_STEP_EPSILON_SECONDS = 0.002
CONTROLLER_STARTUP_SERVICE_RESPONSE_TIMEOUT_SECONDS = 15.0


def load_gazebo_shutdown_support():
    support_path = (
        Path(get_package_share_directory('voice_nav_sim'))
        / 'test_support'
        / 'gazebo_shutdown.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_simulation_control_gazebo_shutdown',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load Gazebo shutdown test support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def load_gazebo_pose_support():
    support_path = (
        Path(get_package_share_directory('voice_nav_sim'))
        / 'test_support'
        / 'gazebo_pose.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_simulation_control_gazebo_pose',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load Gazebo pose test support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


gazebo_shutdown = load_gazebo_shutdown_support()
gazebo_pose_support = load_gazebo_pose_support()
SIMULATION_TEST_PARTITION = (
    gazebo_shutdown.claim_unique_test_partition(
        'l0008_sim_control'
    )
)


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
    return LaunchDescription(
        [
            simulation,
            launch_testing.actions.ReadyToTest(),
        ]
    )


def yaw_from_odometry(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def positive_angle_delta(start: float, end: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


class LaunchStartupPolicyTest(unittest.TestCase):
    def test_startup_handler_stops_after_failed_stage(self):
        package_share = get_package_share_directory('voice_nav_sim')
        launch_path = f'{package_share}/launch/simulation.launch.py'
        specification = importlib.util.spec_from_file_location(
            'voice_nav_simulation_launch',
            launch_path,
        )
        self.assertIsNotNone(specification)
        self.assertIsNotNone(specification.loader)
        launch_module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(launch_module)

        next_action = object()
        handler = launch_module.start_after_success(
            next_action,
            'Test stage',
        )
        self.assertEqual(
            handler(SimpleNamespace(returncode=0), None),
            [next_action],
        )
        self.assertEqual(
            launch_module.start_after_success(
                None,
                'Final stage',
            )(SimpleNamespace(returncode=0), None),
            [],
        )
        failed_actions = handler(
            SimpleNamespace(returncode=17),
            None,
        )
        self.assertEqual(len(failed_actions), 1)
        self.assertEqual(
            failed_actions[0].__class__.__name__,
            'Shutdown',
        )


class SimulationControlTest(unittest.TestCase):
    def setUp(self, proc_info):
        self.addCleanup(self.destroy_ros_fixture)
        self.addCleanup(
            gazebo_shutdown.structured_stop_gazebo,
            proc_info,
            expected_partition=SIMULATION_TEST_PARTITION,
        )
        self.addCleanup(self.publish_zero_for_cleanup)
        rclpy.init()
        self.node = rclpy.create_node(
            'voice_nav_simulation_control_test',
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
        self.joint_states: deque[JointState] = deque(maxlen=500)
        self.odometry: deque[Odometry] = deque(maxlen=500)
        self.transforms: deque[TFMessage] = deque(maxlen=500)
        self.limited_commands: deque[TwistStamped] = deque(maxlen=1000)

        self.subscriptions = [
            self.node.create_subscription(
                JointState,
                '/joint_states',
                lambda message: self.append_sample(
                    self.joint_states,
                    message,
                ),
                20,
            ),
            self.node.create_subscription(
                Odometry,
                ODOMETRY_TOPIC,
                lambda message: self.append_sample(
                    self.odometry,
                    message,
                ),
                20,
            ),
            self.node.create_subscription(
                TFMessage,
                '/tf',
                lambda message: self.append_sample(
                    self.transforms,
                    message,
                ),
                100,
            ),
            self.node.create_subscription(
                TwistStamped,
                LIMITED_COMMAND_TOPIC,
                lambda message: self.append_sample(
                    self.limited_commands,
                    message,
                ),
                100,
            ),
        ]
        self.command_publisher = self.node.create_publisher(
            TwistStamped,
            COMMAND_TOPIC,
            10,
        )
        self.list_controllers = self.node.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )
        self.list_hardware_interfaces = self.node.create_client(
            ListHardwareInterfaces,
            '/controller_manager/list_hardware_interfaces',
        )
        self.spin_thread.start()

    def publish_zero_for_cleanup(self):
        if (
            not rclpy.ok()
            or getattr(self, 'node', None) is None
            or getattr(self, 'command_publisher', None) is None
        ):
            return
        self.publish_for(0.0, 0.0, 0.15)

    def destroy_ros_fixture(self):
        steps = []
        executor = getattr(self, 'executor', None)
        if executor is not None:
            steps.append(
                (
                    'executor shutdown',
                    lambda: executor.shutdown(timeout_sec=2.0),
                )
            )
        spin_thread = getattr(self, 'spin_thread', None)
        if spin_thread is not None:
            steps.append(
                (
                    'spin thread join',
                    lambda: gazebo_shutdown.join_started_thread(
                        spin_thread,
                        timeout_seconds=2.0,
                    ),
                )
            )
        node = getattr(self, 'node', None)
        if node is not None:
            steps.append(('node destroy', node.destroy_node))

        def shutdown_rclpy():
            if rclpy.ok():
                rclpy.shutdown()

        steps.append(('rclpy shutdown', shutdown_rclpy))
        gazebo_shutdown.run_cleanup_steps(
            'simulation control ROS fixture destruction failed',
            steps,
        )

    def append_sample(self, samples, message):
        with self.samples_lock:
            samples.append(message)

    def latest_sample(self, samples):
        with self.samples_lock:
            return samples[-1] if samples else None

    def sample_snapshot(self, samples):
        with self.samples_lock:
            return tuple(samples)

    def clear_samples(self, samples):
        with self.samples_lock:
            samples.clear()

    def wait_until(self, predicate, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.02)
        self.fail(f'timed out waiting for {description}')

    def call_service(self, client, request, timeout: float = 5.0):
        future = client.call_async(request)
        self.wait_until(
            future.done,
            timeout,
            f'response from {client.srv_name}',
        )
        exception = future.exception()
        if exception is not None:
            raise exception
        return future.result()

    def simulation_nanoseconds(self) -> int:
        return self.node.get_clock().now().nanoseconds

    def gazebo_pose(self) -> tuple[float, float, float, float, float, float]:
        return gazebo_pose_support.read_model_pose(
            GAZEBO_POSE_TOPIC,
            'voice_nav_robot',
            expected_partition=SIMULATION_TEST_PARTITION,
        )

    def publish_for(
        self,
        linear_x: float,
        angular_z: float,
        wall_seconds: float,
        publisher=None,
    ) -> int:
        target_publisher = publisher or self.command_publisher
        deadline = time.monotonic() + wall_seconds
        last_stamp = 0
        while time.monotonic() < deadline:
            message = TwistStamped()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.twist.linear.x = linear_x
            message.twist.angular.z = angular_z
            target_publisher.publish(message)
            last_stamp = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            time.sleep(0.04)
        return last_stamp

    def controller_states(self, *, timeout: float):
        response = self.call_service(
            self.list_controllers,
            ListControllers.Request(),
            timeout=timeout,
        )
        return {
            controller.name: controller.state
            for controller in response.controller
        }

    def test_stamped_drive_odometry_tf_and_consumer_timeout(self):
        self.assertTrue(
            self.list_controllers.wait_for_service(timeout_sec=60.0),
            'controller manager did not become available',
        )
        self.assertTrue(
            self.list_hardware_interfaces.wait_for_service(timeout_sec=10.0),
            'controller-manager hardware Interface service is unavailable',
        )
        self.wait_until(
            lambda: self.simulation_nanoseconds() > 0,
            15.0,
            'advancing simulation clock',
        )
        states = self.wait_until(
            lambda: (
                current
                if (
                    current := self.controller_states(
                        timeout=(
                            CONTROLLER_STARTUP_SERVICE_RESPONSE_TIMEOUT_SECONDS
                        )
                    )
                ).get('joint_state_broadcaster')
                == 'active'
                and current.get('diff_drive_controller') == 'active'
                else None
            ),
            30.0,
            'both controllers to become active',
        )
        self.assertEqual(states['joint_state_broadcaster'], 'active')
        self.assertEqual(states['diff_drive_controller'], 'active')
        self.node.get_logger().info(
            'controller_states='
            'joint_state_broadcaster:active,'
            'diff_drive_controller:active'
        )

        hardware = self.call_service(
            self.list_hardware_interfaces,
            ListHardwareInterfaces.Request(),
        )
        command_interfaces = {
            interface.name: interface
            for interface in hardware.command_interfaces
        }
        state_interfaces = {
            interface.name: interface
            for interface in hardware.state_interfaces
        }
        for joint_name in ('left_wheel_joint', 'right_wheel_joint'):
            velocity_command = command_interfaces[f'{joint_name}/velocity']
            self.assertTrue(velocity_command.is_available)
            self.assertTrue(velocity_command.is_claimed)
            self.assertTrue(
                state_interfaces[f'{joint_name}/position'].is_available
            )
            self.assertTrue(
                state_interfaces[f'{joint_name}/velocity'].is_available
            )
        self.node.get_logger().info(
            'claimed_wheel_velocity_interfaces='
            'left_wheel_joint/velocity,right_wheel_joint/velocity'
        )

        topic_types = self.wait_until(
            lambda: dict(self.node.get_topic_names_and_types()).get(
                COMMAND_TOPIC
            ),
            10.0,
            'stamped controller command topic',
        )
        self.assertEqual(topic_types, ['geometry_msgs/msg/TwistStamped'])
        self.wait_until(
            lambda: self.command_publisher.get_subscription_count() == 1,
            10.0,
            'one controller command subscriber',
        )
        self.clear_samples(self.limited_commands)
        self.publish_for(2.0, 2.0, 0.2)
        limited_sample = self.wait_until(
            lambda: next(
                (
                    message
                    for message in reversed(
                        self.sample_snapshot(self.limited_commands)
                    )
                    if abs(message.twist.linear.x) > 0.0
                    or abs(message.twist.angular.z) > 0.0
                ),
                None,
            ),
            3.0,
            'hard-limited velocity output',
        )
        self.assertAlmostEqual(
            limited_sample.twist.linear.x,
            0.4,
            delta=1e-6,
        )
        self.assertAlmostEqual(
            limited_sample.twist.angular.z,
            1.2,
            delta=1e-6,
        )
        self.node.get_logger().info(
            'hard_velocity_limits=linear_x:0.400,angular_z:1.200'
        )
        self.publish_for(0.0, 0.0, 0.15)
        self.wait_until(
            lambda: (
                self.latest_sample(self.odometry)
                and self.latest_sample(self.joint_states)
            ),
            10.0,
            'initial odometry and joint state',
        )

        initial_odometry = self.latest_sample(self.odometry)
        initial_joint_state = self.latest_sample(self.joint_states)
        initial_gazebo_pose = self.gazebo_pose()
        initial_x = initial_odometry.pose.pose.position.x
        initial_positions = dict(
            zip(initial_joint_state.name, initial_joint_state.position)
        )
        self.publish_for(0.15, 0.0, 1.0)
        self.publish_for(0.0, 0.0, 0.15)
        moved_odometry = self.wait_until(
            lambda: (
                sample
                if (
                    sample := self.latest_sample(self.odometry)
                )
                and sample.pose.pose.position.x > initial_x + 0.03
                else None
            ),
            5.0,
            'positive forward odometry',
        )
        self.assertEqual(moved_odometry.header.frame_id, 'odom')
        self.assertEqual(moved_odometry.child_frame_id, 'base_footprint')
        forward_delta = moved_odometry.pose.pose.position.x - initial_x
        self.node.get_logger().info(
            f'forward_odometry_delta_m={forward_delta:.3f}'
        )
        forward_gazebo_pose = self.gazebo_pose()
        gazebo_forward_delta = (
            forward_gazebo_pose[0] - initial_gazebo_pose[0]
        )
        self.assertGreater(gazebo_forward_delta, 0.03)
        self.node.get_logger().info(
            'gazebo_ground_truth_forward_delta_m='
            f'{gazebo_forward_delta:.3f}'
        )
        self.node.get_logger().info(
            'gazebo_ground_truth_forward_x_m='
            f'{initial_gazebo_pose[0]:.3f}->'
            f'{forward_gazebo_pose[0]:.3f}'
        )

        def wheels_changed():
            joint_state = self.latest_sample(self.joint_states)
            if joint_state is None:
                return False
            positions = dict(
                zip(joint_state.name, joint_state.position)
            )
            return all(
                joint_name in positions
                and joint_name in initial_positions
                and abs(positions[joint_name] - initial_positions[joint_name])
                > 0.1
                for joint_name in ('left_wheel_joint', 'right_wheel_joint')
            )

        self.wait_until(wheels_changed, 5.0, 'both wheel positions to change')
        self.wait_until(
            lambda: any(
                transform.header.frame_id == 'odom'
                and transform.child_frame_id == 'base_footprint'
                for message in self.sample_snapshot(self.transforms)
                for transform in message.transforms
            ),
            5.0,
            'odom to base_footprint TF',
        )

        yaw_before = yaw_from_odometry(
            self.latest_sample(self.odometry)
        )
        self.publish_for(0.0, 0.6, 0.8)
        self.publish_for(0.0, 0.0, 0.15)
        rotated_odometry = self.wait_until(
            lambda: (
                sample
                if (
                    sample := self.latest_sample(self.odometry)
                )
                and positive_angle_delta(
                    yaw_before,
                    yaw_from_odometry(sample),
                )
                > 0.10
                else None
            ),
            5.0,
            'positive yaw odometry',
        )
        yaw_delta = positive_angle_delta(
            yaw_before,
            yaw_from_odometry(rotated_odometry),
        )
        self.node.get_logger().info(
            f'positive_yaw_delta_rad={yaw_delta:.3f}'
        )
        rotated_gazebo_pose = self.gazebo_pose()
        gazebo_yaw_delta = positive_angle_delta(
            forward_gazebo_pose[5],
            rotated_gazebo_pose[5],
        )
        self.assertGreater(gazebo_yaw_delta, 0.10)
        self.node.get_logger().info(
            'gazebo_ground_truth_positive_yaw_delta_rad='
            f'{gazebo_yaw_delta:.3f}'
        )
        self.node.get_logger().info(
            'gazebo_ground_truth_yaw_rad='
            f'{forward_gazebo_pose[5]:.3f}->'
            f'{rotated_gazebo_pose[5]:.3f}'
        )

        self.clear_samples(self.limited_commands)
        fault_publisher = self.node.create_publisher(
            TwistStamped,
            COMMAND_TOPIC,
            10,
        )
        self.wait_until(
            lambda: fault_publisher.get_subscription_count() == 1,
            5.0,
            'fault-injection publisher connection',
        )
        last_input_stamp = self.publish_for(
            0.12,
            0.0,
            0.5,
            publisher=fault_publisher,
        )
        self.node.destroy_publisher(fault_publisher)
        self.node.get_logger().info(
            'fault_injection=destroy_nonzero_command_publisher'
        )
        self.wait_until(
            lambda: any(
                abs(message.twist.linear.x) > 0.05
                for message in self.sample_snapshot(
                    self.limited_commands
                )
            ),
            3.0,
            'non-zero limited command',
        )

        def first_zero_after_timeout():
            for message in self.sample_snapshot(self.limited_commands):
                stamp = (
                    message.header.stamp.sec * 1_000_000_000
                    + message.header.stamp.nanosec
                )
                if (
                    stamp > last_input_stamp
                    and abs(message.twist.linear.x) < 1e-6
                    and abs(message.twist.angular.z) < 1e-6
                ):
                    return stamp
            return 0

        first_zero_stamp = self.wait_until(
            first_zero_after_timeout,
            5.0,
            'consumer-timeout zero command',
        )
        observed_timeout = (
            first_zero_stamp - last_input_stamp
        ) / 1_000_000_000.0
        self.node.get_logger().info(
            'consumer_deadman_simulation_stamps_seconds='
            f'{last_input_stamp / 1_000_000_000.0:.3f}->'
            f'{first_zero_stamp / 1_000_000_000.0:.3f}'
        )
        self.node.get_logger().info(
            f'consumer_deadman_zero_seconds={observed_timeout:.3f}'
        )
        self.assertGreaterEqual(
            observed_timeout,
            CONTROLLER_TIMEOUT_SECONDS - CONTROL_PERIOD_SECONDS,
        )
        self.assertLessEqual(
            observed_timeout,
            CONTROLLER_TIMEOUT_SECONDS
            + CONTROL_PERIOD_SECONDS
            + SIMULATION_STEP_EPSILON_SECONDS,
        )

        self.wait_until(
            lambda: (
                (sample := self.latest_sample(self.odometry))
                and abs(sample.twist.twist.linear.x) < 0.02
                and abs(sample.twist.twist.angular.z) < 0.02
            ),
            2.0,
            'physical velocity stationarity after command zero',
        )
        self.node.get_logger().info(
            'physical_stationarity='
            'linear_abs_lt_0.02,angular_abs_lt_0.02'
        )


@launch_testing.post_shutdown_test()
class SimulationControlShutdownTest(unittest.TestCase):

    def test_all_launch_managed_processes_exit_cleanly(self, proc_info):
        assertExitCodes(proc_info)
