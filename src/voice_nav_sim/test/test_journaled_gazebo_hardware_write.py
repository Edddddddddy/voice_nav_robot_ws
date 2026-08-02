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

"""Runtime proof that Gazebo writes through the journaled hardware adapter."""

import atexit
import importlib.util
import math
import os
from pathlib import Path
import struct
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
import xacro


def load_module_from_path(module_name, module_path):
    """Load one test module without relying on runner-specific sys.path."""
    specification = importlib.util.spec_from_file_location(
        module_name,
        module_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError(f'could not load test module {module_name}')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


ledger_support = load_module_from_path(
    'voice_nav_sim_hardware_write_ledger_test_support',
    Path(__file__).with_name('hardware_write_ledger_test_support.py'),
)
BANK_STATE_SEALED_OK = ledger_support.BANK_STATE_SEALED_OK
CONTROL_RESPONSE_OK = ledger_support.CONTROL_RESPONSE_OK
GLOBAL_ORACLE_FAULTS_WORD = ledger_support.GLOBAL_ORACLE_FAULTS_WORD
HardwareWriteLedgerRegionOwner = (
    ledger_support.HardwareWriteLedgerRegionOwner
)


COMMAND_TOPIC = '/diff_drive_controller/cmd_vel'
CONTROL_PERIOD_SECONDS = 0.01
LEDGER_GENERATION = 0x0011A001
LEDGER_INTERVAL_ID = 0x0011A002
LEDGER_SEGMENT_CAPACITY = 8192
LEDGER_PAGE_SEGMENT_LIMIT = 256
LEDGER_INVOCATION_BUDGET = 8192
WAIT_TIMEOUT_SECONDS = 60.0


def load_test_support(module_name):
    """Load one installed test-support module by its stable package path."""
    support_path = (
        Path(get_package_share_directory('voice_nav_sim'))
        / 'test_support'
        / f'{module_name}.py'
    )
    return load_module_from_path(
        f'voice_nav_sim_{module_name}',
        support_path,
    )


gazebo_shutdown = load_test_support('gazebo_shutdown')
crash_robot_description = load_test_support('crash_robot_description')
TEST_PARTITION = gazebo_shutdown.claim_unique_test_partition(
    'l0010_hw_write',
)
LEDGER_OWNER = HardwareWriteLedgerRegionOwner(
    generation=LEDGER_GENERATION,
    segment_capacity=LEDGER_SEGMENT_CAPACITY,
    page_segment_limit=LEDGER_PAGE_SEGMENT_LIMIT,
)
atexit.register(LEDGER_OWNER.cleanup)


def start_after_success(next_action, stage):
    """Start the next fixture stage only after an exact zero exit."""
    def handle_exit(event, _context):
        if event.returncode == 0:
            return [] if next_action is None else [next_action]
        return [
            Shutdown(
                reason=(
                    f'{stage} failed with exit code {event.returncode}; '
                    'aborting journaled simulation startup.'
                )
            )
        ]

    return handle_exit


def expanded_journaled_robot_description(package_share):
    """Expand the canonical product Xacro, then replace only its hardware."""
    xacro_file = package_share / 'urdf' / 'voice_nav_robot.urdf.xacro'
    controllers_file = package_share / 'config' / 'controllers.yaml'
    product_urdf = xacro.process_file(
        str(xacro_file),
        mappings={'controllers_file': str(controllers_file)},
    ).toxml()
    return crash_robot_description.transform_product_urdf(
        product_urdf,
        LEDGER_OWNER.name,
        LEDGER_OWNER.nonce,
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch one isolated canonical robot with the test-only adapter."""
    package_share = Path(get_package_share_directory('voice_nav_sim'))
    robot_description = expanded_journaled_robot_description(package_share)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
        on_exit=Shutdown(reason='Robot state publisher exited.'),
    )

    gazebo_environment = {
        'GZ_PARTITION': TEST_PARTITION,
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join(
            filter(
                None,
                (
                    os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH'),
                    os.environ.get('LD_LIBRARY_PATH'),
                ),
            )
        ),
        'GZ_SIM_RESOURCE_PATH': os.environ.get(
            'GZ_SIM_RESOURCE_PATH',
            '',
        ),
    }
    world_file = package_share / 'worlds' / 'voice_nav_test_world.sdf'
    gazebo = ExecuteProcess(
        cmd=[
            FindExecutable(name='ruby'),
            FindExecutable(name='gz'),
            'sim',
            '-r',
            '-v',
            '2',
            '-s',
            '--headless-rendering',
            str(world_file),
            '--force-version',
            '8',
        ],
        name='gazebo',
        output='screen',
        additional_env=gazebo_environment,
        sigterm_timeout='10',
        sigkill_timeout='5',
    )

    simulation_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='simulation_bridge',
        output='screen',
        parameters=[
            {
                'config_file': str(
                    package_share / 'config' / 'bridge.yaml'
                )
            }
        ],
        on_exit=Shutdown(reason='Simulation bridge exited.'),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_voice_nav_robot',
        output='screen',
        arguments=[
            '--world',
            'voice_nav_test_world',
            '--topic',
            'robot_description',
            '--name',
            'voice_nav_robot',
            '--z',
            '0.03',
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_state_broadcaster',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30',
            '--switch-timeout',
            '10',
            '--service-call-timeout',
            '10',
        ],
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_diff_drive_controller',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30',
            '--switch-timeout',
            '10',
            '--service-call-timeout',
            '10',
            '--controller-ros-args',
            '--ros-args --remap ~/odom:=/odom',
        ],
    )

    def cleanup_ledger(_event, _context):
        LEDGER_OWNER.cleanup()
        return []

    return (
        LaunchDescription(
            [
                RegisterEventHandler(
                    OnShutdown(on_shutdown=cleanup_ledger),
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=gazebo,
                        on_exit=[Shutdown(reason='Gazebo exited.')],
                    )
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=spawn_robot,
                        on_exit=start_after_success(
                            joint_state_broadcaster_spawner,
                            'Robot spawn',
                        ),
                    )
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=joint_state_broadcaster_spawner,
                        on_exit=start_after_success(
                            diff_drive_controller_spawner,
                            'Joint-state broadcaster startup',
                        ),
                    )
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=diff_drive_controller_spawner,
                        on_exit=start_after_success(
                            None,
                            'Differential-drive controller startup',
                        ),
                    )
                ),
                robot_state_publisher,
                gazebo,
                simulation_bridge,
                spawn_robot,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            'gazebo_action': gazebo,
            'ledger_owner': LEDGER_OWNER,
        },
    )


def double_from_bits(bits):
    """Decode one C++ double bit pattern without changing its payload."""
    return struct.unpack('<d', struct.pack('<Q', bits))[0]


class JournaledGazeboHardwareWriteTest(unittest.TestCase):
    """Prove the actual Gazebo process journals valid non-zero wheel writes."""

    def setUp(self, proc_info, ledger_owner):
        self.addCleanup(self.destroy_ros_fixture)
        self.addCleanup(ledger_owner.cleanup)
        self.addCleanup(
            gazebo_shutdown.structured_stop_gazebo,
            proc_info,
            expected_partition=TEST_PARTITION,
        )
        self.addCleanup(self.publish_zero_for_cleanup)

        rclpy.init()
        self.node = rclpy.create_node(
            'journaled_gazebo_hardware_write_test',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.command_publisher = self.node.create_publisher(
            TwistStamped,
            COMMAND_TOPIC,
            10,
        )
        self.list_controllers = self.node.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )
        self.spin_thread.start()

    def wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.02)
        self.fail(f'timed out waiting for {description}')

    def publish_for(self, linear_x, angular_z, wall_seconds):
        deadline = time.monotonic() + wall_seconds
        while time.monotonic() < deadline:
            message = TwistStamped()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.twist.linear.x = linear_x
            message.twist.angular.z = angular_z
            self.command_publisher.publish(message)
            time.sleep(0.04)

    def call_service(self, client, request, timeout=5.0):
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

    def controller_states(self):
        response = self.call_service(
            self.list_controllers,
            ListControllers.Request(),
        )
        return {
            controller.name: controller.state
            for controller in response.controller
        }

    def publish_zero_for_cleanup(self):
        if (
            rclpy.ok()
            and getattr(self, 'command_publisher', None) is not None
        ):
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
            'journaled Gazebo ROS fixture destruction failed',
            steps,
        )

    def test_real_gazebo_writer_records_nonzero_wheel_commands(
        self,
        proc_info,
        gazebo_action,
        ledger_owner,
    ):
        proc_info.assertWaitForStartup(
            process=gazebo_action,
            timeout=10.0,
        )
        launched_pid = gazebo_action.process_details['pid']
        self.assertGreater(launched_pid, 0)
        self.assertEqual(
            ledger_owner.wait_for_writer(
                launched_pid,
                timeout=WAIT_TIMEOUT_SECONDS,
            ),
            launched_pid,
        )
        self.wait_until(
            lambda: self.node.get_clock().now().nanoseconds > 0,
            15.0,
            'advancing simulation clock',
        )
        self.assertTrue(
            self.list_controllers.wait_for_service(timeout_sec=30.0),
            'controller manager did not become available',
        )
        controller_states = self.wait_until(
            lambda: (
                states
                if (
                    (states := self.controller_states()).get(
                        'joint_state_broadcaster'
                    ) == 'active'
                    and states.get('diff_drive_controller') == 'active'
                )
                else None
            ),
            30.0,
            'both controllers to become active',
        )
        self.assertEqual(
            controller_states['diff_drive_controller'],
            'active',
        )
        self.wait_until(
            lambda: self.command_publisher.get_subscription_count() == 1,
            30.0,
            'active differential-drive command subscriber',
        )

        arm_ticket = ledger_owner.post_arm(
            interval_id=LEDGER_INTERVAL_ID,
            segment_budget=LEDGER_SEGMENT_CAPACITY,
            invocation_budget=LEDGER_INVOCATION_BUDGET,
            require_zero_commands=False,
        )
        arm_response = ledger_owner.wait_response(
            arm_ticket,
            timeout=5.0,
        )
        self.assertEqual(arm_response[9], CONTROL_RESPONSE_OK)

        self.publish_for(0.15, 0.0, 0.12)
        seal_ticket = ledger_owner.post_seal(
            interval_id=LEDGER_INTERVAL_ID,
            bank_index=arm_response[10],
            bank_epoch=arm_response[11],
            not_before_sim_stamp_ns=0,
            require_exact_stamp=False,
        )
        seal_response = ledger_owner.wait_response(
            seal_ticket,
            timeout=5.0,
        )
        self.assertEqual(seal_response[9], CONTROL_RESPONSE_OK)
        self.assertEqual(seal_response[10], arm_response[10])
        self.assertEqual(seal_response[11], arm_response[11])
        self.assertGreater(seal_response[12], arm_response[12])

        snapshot = ledger_owner.read_sealed_interval(
            interval_id=LEDGER_INTERVAL_ID,
            bank_index=seal_response[10],
            bank_epoch=seal_response[11],
            seal_fence_write_seq=seal_response[12],
        )
        self.assertEqual(
            snapshot.terminal_state,
            BANK_STATE_SEALED_OK,
            'journaled Gazebo interval faulted: '
            f'bank_faults=0x{snapshot.oracle_faults:x}, '
            'global_faults='
            f'0x{ledger_owner.load_header_word(GLOBAL_ORACLE_FAULTS_WORD):x}',
        )
        self.assertEqual(snapshot.oracle_faults, 0)
        self.assertEqual(
            ledger_owner.load_header_word(GLOBAL_ORACLE_FAULTS_WORD),
            0,
        )
        self.assertEqual(
            snapshot.arm_fence_write_seq,
            arm_response[12],
        )

        segments = tuple(
            segment
            for page in snapshot.pages
            for segment in page.segments
        )
        self.assertTrue(segments)
        self.assertEqual(segments[0][1], arm_response[12] + 1)
        self.assertEqual(segments[-1][2], seal_response[12])
        self.assertTrue(
            all(
                following[1] == previous[2] + 1
                for previous, following in zip(segments, segments[1:])
            )
        )
        self.assertTrue(all(segment[5] == 0 for segment in segments))

        wheel_commands = tuple(
            (double_from_bits(segment[6]), double_from_bits(segment[7]))
            for segment in segments
        )
        self.assertTrue(
            all(
                math.isfinite(left) and math.isfinite(right)
                for left, right in wheel_commands
            )
        )
        self.assertTrue(
            any(
                abs(left) > 1e-3 and abs(right) > 1e-3
                for left, right in wheel_commands
            ),
            'sealed hardware-write evidence contains no non-zero wheel pair',
        )
        self.assertTrue(ledger_owner.acknowledge(snapshot))


@launch_testing.post_shutdown_test()
class JournaledGazeboHardwareWriteShutdownTest(unittest.TestCase):

    def test_all_launch_managed_processes_exit_cleanly(self, proc_info):
        assertExitCodes(proc_info)
