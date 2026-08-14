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

"""Headless product smoke for scripted Voice -> Agent -> Mission -> Motion."""

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import json
import math
import os
from pathlib import Path
import threading
import time
import unittest

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from voice_nav_interfaces.action import Speak
from voice_nav_interfaces.msg import MissionState, VoiceTurn
from voice_nav_mission.msg import InternalMotionGateState


MISSION_STATE_TOPIC = '/mission/state'
GATE_STATE_TOPIC = '/motion_gate/internal/state'
ODOMETRY_TOPIC = '/odom'
FINAL_COMMAND_TOPIC = '/diff_drive_controller/cmd_vel'
MISSION_STATUS_TOPIC = '/mission/execute/_action/status'
SPEAK_STATUS_TOPIC = '/voice/speak/_action/status'
VOICE_TURN_TOPIC = '/voice/turn'
ZERO_EPSILON = 1.0e-6


def _state_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _voice_turn_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class _LoopbackLlmServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self):
        super().__init__(('127.0.0.1', 0), _LoopbackLlmHandler)
        self.requests = []

    @property
    def endpoint(self):
        return f'http://127.0.0.1:{self.server_address[1]}'


class _LoopbackLlmHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        body = self.rfile.read(content_length)
        self.server.requests.append((self.path, body))
        result = {
            'kind': 'mission',
            'steps': [{
                'kind': 'rotate_angle',
                'angle_rad': 1.570796,
            }],
        }
        response = json.dumps({
            'choices': [{
                'message': {
                    'content': json.dumps(result, ensure_ascii=False),
                },
            }],
        }, ensure_ascii=False).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(response)))
        self.end_headers()
        self.wfile.write(response)

    def log_message(self, _format, *_args):
        pass


def _load_gazebo_shutdown_support():
    support_path = (
        Path(get_package_share_directory('voice_nav_sim'))
        / 'test_support'
        / 'gazebo_shutdown.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_scripted_voice_gazebo_shutdown',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load Gazebo shutdown support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


gazebo_shutdown = _load_gazebo_shutdown_support()
PRODUCT_TEST_PARTITION = gazebo_shutdown.claim_unique_test_partition(
    'i128_scripted_voice_gazebo'
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Compose the installed product with only test-only speech/LLM/Speak seams."""
    driver = os.environ.get('VOICE_NAV_SCRIPTED_SPEECH_DRIVER')
    assert driver and Path(driver).is_file()
    llm_server = _LoopbackLlmServer()
    llm_thread = threading.Thread(
        target=llm_server.serve_forever,
        daemon=True,
    )
    llm_thread.start()
    bringup_share = get_package_share_directory('voice_nav_bringup')
    product = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{bringup_share}/launch/product_sim.launch.py'
        ),
        launch_arguments={
            'headless': 'true',
            'shutdown_on_gazebo_exit': 'false',
        }.items(),
    )
    agent = Node(
        package='voice_nav_agent',
        executable='agent_node',
        name='agent_node',
        output='screen',
        parameters=[{'llm_endpoint': llm_server.endpoint}],
    )
    speech_driver = ExecuteProcess(cmd=[driver], output='screen')
    return LaunchDescription([
        product,
        agent,
        speech_driver,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'agent': agent,
        'llm_server': llm_server,
        'llm_thread': llm_thread,
        'speech_driver': speech_driver,
    }


def _yaw_from_odom(message):
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


def _wrapped_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _is_zero(message):
    twist = message.twist
    return all(
        abs(value) <= ZERO_EPSILON
        for value in (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
    )


class ScriptedVoiceGazeboSmokeTest(unittest.TestCase):
    def setUp(self, proc_info, llm_server, llm_thread):
        self.addCleanup(self._destroy_ros_fixture)
        self.addCleanup(
            gazebo_shutdown.structured_stop_gazebo,
            proc_info,
            expected_partition=PRODUCT_TEST_PARTITION,
        )
        self.llm_server = llm_server
        self.llm_thread = llm_thread
        rclpy.init()
        self.node = rclpy.create_node('voice_agent_gazebo_probe')
        self.voice_turns = deque(maxlen=8)
        self.speak_goals = deque(maxlen=8)
        self.mission_successes = set()
        self.mission_states = deque(maxlen=200)
        self.gate_states = deque(maxlen=400)
        self.odometry = deque(maxlen=800)
        self.final_commands = deque(maxlen=800)
        self.subscriptions = [
            self.node.create_subscription(
                VoiceTurn,
                VOICE_TURN_TOPIC,
                self._on_voice_turn,
                _voice_turn_qos(),
            ),
            self.node.create_subscription(
                MissionState,
                MISSION_STATE_TOPIC,
                lambda message: self.mission_states.append((
                    time.monotonic(), message,
                )),
                _state_qos(),
            ),
            self.node.create_subscription(
                InternalMotionGateState,
                GATE_STATE_TOPIC,
                self._on_gate_state,
                _state_qos(),
            ),
            self.node.create_subscription(
                Odometry,
                ODOMETRY_TOPIC,
                lambda message: self.odometry.append((time.monotonic(), message)),
                100,
            ),
            self.node.create_subscription(
                TwistStamped,
                FINAL_COMMAND_TOPIC,
                lambda message: self.final_commands.append((
                    time.monotonic(), message,
                )),
                100,
            ),
            self.node.create_subscription(
                GoalStatusArray,
                MISSION_STATUS_TOPIC,
                self._on_mission_status,
                10,
            ),
        ]
        self.initial_odom = self._wait_until(
            lambda: self.odometry[-1][1] if self._runtime_is_ready() and self.odometry else None,
            45.0,
            'available Runtime and initial odometry',
        )
        self.speak_server = ActionServer(
            self.node,
            Speak,
            '/voice/speak',
            execute_callback=self._speak,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
        )

    def tearDown(self):
        self.speak_server.destroy()
        self.llm_server.shutdown()
        self.llm_server.server_close()
        self.llm_thread.join(5.0)

    def _destroy_ros_fixture(self):
        node = getattr(self, 'node', None)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _on_voice_turn(self, turn):
        self.voice_turns.append(turn)

    def _on_mission_status(self, statuses):
        for status in statuses.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if status.status == GoalStatus.STATUS_SUCCEEDED:
                self.mission_successes.add(goal_id)

    def _on_gate_state(self, message):
        self.gate_states.append((time.monotonic(), message))

    def _speak(self, goal_handle):
        self.speak_goals.append(goal_handle.request)
        goal_handle.succeed()
        result = Speak.Result()
        result.code = Speak.Result.COMPLETED
        return result

    def _runtime_is_ready(self):
        if not self.mission_states:
            return False
        state = self.mission_states[-1][1]
        return (
            state.availability == MissionState.AVAILABLE
            and state.gate_state == MissionState.GATE_INHIBITED
        )

    def _wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            value = predicate()
            if value is not None and value is not False:
                return value
        self.fail(f'timed out waiting for {description}')

    def _wait_for_zero_and_stationarity(self):
        def zero_and_stationary():
            now = time.monotonic()
            recent_commands = [
                (received_at, message) for received_at, message in self.final_commands
                if now - received_at <= 0.25
            ]
            recent_odom = [
                (received_at, message) for received_at, message in self.odometry
                if now - received_at <= 0.25
            ]
            if not recent_commands or not recent_odom or not self.gate_states:
                return False
            gate = self.gate_states[-1][1]
            command_window = recent_commands[-1][0] - recent_commands[0][0]
            odom_window = recent_odom[-1][0] - recent_odom[0][0]
            stationary = all(
                abs(message.twist.twist.linear.x) <= 0.01
                and abs(message.twist.twist.angular.z) <= 0.02
                for _, message in recent_odom
            )
            return (
                stationary
                and command_window >= 0.20
                and odom_window >= 0.20
                and all(_is_zero(message) for _, message in recent_commands)
                and gate.motion_inhibited
                and gate.zero_selected
                and gate.zero_publish_seq >= gate.output_publish_seq
            )

        self._wait_until(
            zero_and_stationary,
            10.0,
            'final controller/Gate zero and stationary odometry',
        )

    def _assert_gate_armed_observed(self):
        self.assertTrue(any(
            state.state == InternalMotionGateState.ARMED
            and not state.motion_inhibited
            and bool(state.lease_id)
            for _, state in self.gate_states
        ))

    def test_two_scripted_voice_scenarios_reach_product_motion(self):
        self._wait_until(
            lambda: len(self.mission_successes) == 2,
            120.0,
            'two successful product ExecuteMission Goals',
        )
        self._wait_until(
            lambda: len(self.voice_turns) == 2 and len(self.speak_goals) == 2,
            30.0,
            'two VoiceTurn and Speak correlations',
        )
        self._wait_for_zero_and_stationarity()

        turns = tuple(self.voice_turns)
        self.assertEqual([turn.text for turn in turns], ['前进半米', '绕个弯'])
        self.assertEqual([turn.kind for turn in turns], [
            VoiceTurn.COMMAND,
            VoiceTurn.COMMAND,
        ])
        self.assertEqual([turn.voice_seq for turn in turns], [1, 2])
        self.assertEqual(turns[0].voice_instance_id, turns[1].voice_instance_id)
        self.assertTrue(all(turn.session_id and turn.turn_id for turn in turns))
        self.assertEqual(len(self.mission_successes), 2)

        speaks = tuple(self.speak_goals)
        self.assertEqual(
            [(goal.session_id, goal.turn_id) for goal in speaks],
            [(turn.session_id, turn.turn_id) for turn in turns],
        )
        self.assertEqual([goal.text for goal in speaks], ['任务已完成。', '任务已完成。'])

        self.assertEqual(len(self.llm_server.requests), 1)
        request_path, request_body = self.llm_server.requests[0]
        self.assertEqual(request_path, '/v1/chat/completions')
        self.assertEqual(
            json.loads(request_body.decode('utf-8'))['model'],
            'Qwen3-0.6B-Q8_0.gguf',
        )

        final_odom = self.odometry[-1][1]
        start_yaw = _yaw_from_odom(self.initial_odom)
        displacement = (
            (final_odom.pose.pose.position.x - self.initial_odom.pose.pose.position.x)
            * math.cos(start_yaw)
            + (final_odom.pose.pose.position.y - self.initial_odom.pose.pose.position.y)
            * math.sin(start_yaw)
        )
        yaw_delta = _wrapped_angle(_yaw_from_odom(final_odom) - start_yaw)
        self.assertAlmostEqual(displacement, 0.50, delta=0.10)
        self.assertAlmostEqual(yaw_delta, 1.570796, delta=0.12)

        self._assert_gate_armed_observed()
        self.assertTrue(self._runtime_is_ready())
        evidence = {
            'REAL_AUDIO_MODELS': 'NOT_RUN',
            'REAL_LLM_CORPUS': 'NOT_RUN',
            'displacement_m': displacement,
            'gate_armed_observed': True,
            'llm_requests': len(self.llm_server.requests),
            'mission_goals': len(self.mission_successes),
            'speak_correlations': [
                {'session_id': goal.session_id, 'turn_id': goal.turn_id}
                for goal in speaks
            ],
            'voice_turns': len(turns),
            'yaw_delta_rad': yaw_delta,
        }
        print(
            'EVIDENCE issue128_scripted_voice_gazebo '
            + json.dumps(evidence, sort_keys=True, separators=(',', ':')),
            flush=True,
        )


@launch_testing.post_shutdown_test()
class ScriptedVoiceGazeboShutdownTest(unittest.TestCase):
    def test_all_launch_managed_processes_exit_cleanly(self, proc_info, agent, speech_driver):
        assertExitCodes(
            proc_info,
            process=agent,
            allowable_exit_codes=[0, -2],
        )
        assertExitCodes(
            proc_info,
            process=speech_driver,
            allowable_exit_codes=[0, -2],
        )
