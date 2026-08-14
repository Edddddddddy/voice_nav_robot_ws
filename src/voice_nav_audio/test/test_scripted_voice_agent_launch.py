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

"""First tracer bullet from cleaned speech frames to one typed Mission."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import pathlib
import threading
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
import pytest
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from voice_nav_interfaces.action import ExecuteMission, Speak
from voice_nav_interfaces.msg import MissionState, MissionStep, VoiceTurn


class _LoopbackLlmServer(ThreadingHTTPServer):
    """Ephemeral test-only local LLM endpoint with request accounting."""

    daemon_threads = True

    def __init__(self):
        super().__init__(('127.0.0.1', 0), _LoopbackLlmHandler)
        self.requests = []
        self.request_event = threading.Event()

    @property
    def endpoint(self):
        return f'http://127.0.0.1:{self.server_address[1]}'


class _LoopbackLlmHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        request_body = self.rfile.read(content_length)
        self.server.requests.append((self.path, request_body))
        self.server.request_event.set()
        mission = {
            'kind': 'mission',
            'steps': [{
                'kind': 'rotate_angle',
                'angle_rad': 1.570796,
            }],
        }
        response = json.dumps({
            'choices': [{
                'message': {
                    'content': json.dumps(mission, ensure_ascii=False),
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


def _voice_turn_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _mission_state_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the installed Agent and an intentionally test-only speech driver."""
    scenario = os.environ.get('VOICE_NAV_SCRIPTED_SCENARIO', 'rule')
    assert scenario in {'rule', 'llm'}
    driver = os.environ.get('VOICE_NAV_SCRIPTED_SPEECH_DRIVER')
    assert driver, 'missing test-only scripted SpeechRecognizerAdapter driver'
    assert pathlib.Path(driver).is_file(), (
        'scripted SpeechRecognizerAdapter driver is not a test executable: '
        f'{driver}'
    )
    llm_server = None
    llm_thread = None
    parameters = []
    if scenario == 'llm':
        llm_server = _LoopbackLlmServer()
        llm_thread = threading.Thread(
            target=llm_server.serve_forever,
            daemon=True,
        )
        llm_thread.start()
        parameters = [{'llm_endpoint': llm_server.endpoint}]
    agent = Node(
        package='voice_nav_agent',
        executable='agent_node',
        name='agent_node',
        output='screen',
        parameters=parameters,
    )
    speech_driver = ExecuteProcess(cmd=[driver], output='screen')
    return LaunchDescription([
        agent,
        speech_driver,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'agent': agent,
        'speech_driver': speech_driver,
        'llm_server': llm_server,
        'llm_thread': llm_thread,
        'scenario': scenario,
    }


class ScriptedVoiceAgentLaunchTest(unittest.TestCase):
    """Test the public ROS behavior, never recognizer implementation details."""

    def setUp(self, agent, speech_driver, llm_server, llm_thread, scenario):
        self.llm_server = llm_server
        self.llm_thread = llm_thread
        self.scenario = scenario
        rclpy.init(args=[])
        self.node = rclpy.create_node('scripted_voice_agent_probe')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()
        self.voice_turns = []
        self.mission_goals = []
        self.speak_goals = []
        self.voice_turn_event = threading.Event()
        self.mission_event = threading.Event()
        self.speak_event = threading.Event()
        self.voice_subscription = self.node.create_subscription(
            VoiceTurn,
            '/voice/turn',
            self._on_voice_turn,
            _voice_turn_qos(),
        )
        self.state_publisher = self.node.create_publisher(
            MissionState,
            '/mission/state',
            _mission_state_qos(),
        )
        self.mission_server = ActionServer(
            self.node,
            ExecuteMission,
            '/mission/execute',
            execute_callback=self._execute_mission,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
        )
        self.speak_server = ActionServer(
            self.node,
            Speak,
            '/voice/speak',
            execute_callback=self._speak,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
        )
        self.state_publisher.publish(self._available_navigation_state())

    def tearDown(self):
        self.mission_server.destroy()
        self.speak_server.destroy()
        self.node.destroy_subscription(self.voice_subscription)
        self.node.destroy_publisher(self.state_publisher)
        self.executor.shutdown()
        self.spin_thread.join(5.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self.llm_server is not None:
            self.llm_server.shutdown()
            self.llm_server.server_close()
            self.llm_thread.join(5.0)

    @staticmethod
    def _available_navigation_state():
        state = MissionState()
        state.runtime_instance_id = 'runtime-smoke-a'
        state.admission_epoch = 17
        state.operating_mode = MissionState.NAVIGATION
        state.availability = MissionState.AVAILABLE
        state.gate_state = MissionState.GATE_INHIBITED
        state.active_step = 2**32 - 1
        state.supported_step_mask = 0b1111
        state.max_steps = 3
        return state

    def _on_voice_turn(self, turn):
        self.voice_turns.append(turn)
        self.voice_turn_event.set()

    def _execute_mission(self, goal_handle):
        self.mission_goals.append(goal_handle.request)
        self.mission_event.set()
        goal_handle.succeed()
        result = ExecuteMission.Result()
        result.code = ExecuteMission.Result.SUCCEEDED
        result.failed_step = -1
        return result

    def _speak(self, goal_handle):
        self.speak_goals.append(goal_handle.request)
        self.speak_event.set()
        goal_handle.succeed()
        result = Speak.Result()
        result.code = Speak.Result.COMPLETED
        return result

    def test_cleaned_frames_make_one_turn_and_one_move_mission(self):
        assert self.voice_turn_event.wait(15.0)
        if self.scenario == 'llm':
            assert self.llm_server.request_event.wait(15.0)
            assert len(self.llm_server.requests) == 1
        assert self.mission_event.wait(15.0)
        assert self.speak_event.wait(15.0)

        assert len(self.voice_turns) == 1
        turn = self.voice_turns[0]
        assert turn.kind == VoiceTurn.COMMAND
        expected_text = '前进半米' if self.scenario == 'rule' else '绕个弯'
        assert turn.text == expected_text
        assert turn.voice_seq == 1
        assert turn.voice_instance_id
        assert turn.session_id
        assert turn.turn_id

        assert len(self.mission_goals) == 1
        goal = self.mission_goals[0]
        assert goal.runtime_instance_id == 'runtime-smoke-a'
        assert goal.admission_epoch == 17
        assert len(goal.steps) == 1
        if self.scenario == 'rule':
            assert goal.steps[0].kind == MissionStep.MOVE_DISTANCE
            assert abs(goal.steps[0].distance_m - 0.5) < 1e-6
            assert self.llm_server is None
        else:
            assert goal.steps[0].kind == MissionStep.ROTATE_ANGLE
            assert abs(goal.steps[0].angle_rad - 1.570796) < 1e-5
            assert self.llm_server is not None
            assert len(self.llm_server.requests) == 1
            request_path, request_body = self.llm_server.requests[0]
            assert request_path == '/v1/chat/completions'
            request = json.loads(request_body.decode('utf-8'))
            assert request['model'] == 'Qwen3-0.6B-Q8_0.gguf'
        assert goal.source_instance_id
        assert goal.source_seq == 1

        assert len(self.speak_goals) == 1
        spoken = self.speak_goals[0]
        assert spoken.session_id == turn.session_id
        assert spoken.turn_id == turn.turn_id
        assert spoken.text == '任务已完成。'


@launch_testing.post_shutdown_test()
class ScriptedVoiceAgentShutdownTest(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info, agent, speech_driver):
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
