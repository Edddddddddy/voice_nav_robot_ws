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
import re
import subprocess
import threading
import time
import unittest
from unittest.mock import patch

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

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
CLARIFICATION_TEXT = '请说明需要前进多少米。'
FROZEN_SNAPSHOT = {
    'runtime_instance_id': 'runtime-a',
    'admission_epoch': 7,
    'operating_mode': 1,
    'availability': 1,
    'gate_state': 0,
    'active_step': 2**32 - 1,
    'supported_step_mask': 0b1011,
    'max_steps': 3,
    'named_place_ids': [],
}
_REQUIRES_FROZEN_SNAPSHOT = object()


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
        self.response_kinds = []
        self.agent_identity = None
        self.runtime_identity = None
        self.lock = threading.Lock()

    @property
    def endpoint(self):
        return f'http://127.0.0.1:{self.server_address[1]}'


class _LoopbackLlmHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        body = self.rfile.read(content_length)
        try:
            payload = json.loads(body.decode('utf-8'))
            content = json.loads(payload['messages'][-1]['content'])
        except (KeyError, TypeError, UnicodeDecodeError, json.JSONDecodeError):
            self.send_error(400, 'invalid loopback request')
            return
        with self.server.lock:
            self.server.requests.append((self.path, body))
            request_index = len(self.server.requests)
        result = self._result_for(request_index, content)
        if result is None:
            self.send_error(400, 'unexpected loopback request sequence')
            return
        with self.server.lock:
            self.server.response_kinds.append(result['kind'])
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

    def _result_for(self, request_index, content):
        if not isinstance(content, dict):
            return None
        expected = {
            1: ('绕到大厅', None, 1, None),
            2: ('半米', CLARIFICATION_TEXT, 1, None),
            3: ('半米', CLARIFICATION_TEXT, 2, _REQUIRES_FROZEN_SNAPSHOT),
        }.get(request_index)
        if expected is None:
            return None
        text, clarification, round_number, snapshot_output = expected
        turn = content.get('turn')
        if (
            set(content) != {
                'agent_system_version', 'tool_schema_version',
                'mission_schema_sha256', 'agent', 'runtime_snapshot', 'turn',
                'clarification', 'round', 'snapshot_output',
            }
            or not isinstance(turn, dict)
            or set(turn) != {
                'voice_instance_id', 'voice_seq', 'session_id', 'turn_id',
                'text',
            }
            or turn.get('text') != text
            or content.get('clarification') != clarification
            or content.get('round') != round_number
            or (
                content.get('snapshot_output') != snapshot_output
                if snapshot_output is not _REQUIRES_FROZEN_SNAPSHOT
                else not _is_frozen_snapshot(content.get('snapshot_output'))
            )
        ):
            return None
        if not self._accept_identity(request_index, content):
            return None
        if request_index == 1:
            return {'kind': 'clarify', 'text': CLARIFICATION_TEXT}
        if request_index == 2:
            return {
                'kind': 'tool',
                'tool_call': {
                    'name': 'read_runtime_snapshot',
                    'arguments': {},
                },
            }
        return {
            'kind': 'tool',
            'tool_call': {
                'name': 'propose_mission',
                'arguments': {
                    'kind': 'mission',
                    'steps': [{
                        'kind': 'move_distance',
                        'distance_m': 0.5,
                    }],
                },
            },
        }

    def _accept_identity(self, request_index, content):
        agent = content['agent']
        runtime = content['runtime_snapshot']
        expected_turn_generation = (1, 2, 2)[request_index - 1]
        if (
            not isinstance(agent, dict)
            or set(agent) != {
                'agent_generation', 'turn_generation',
            }
            or not isinstance(agent['agent_generation'], int)
            or agent['agent_generation'] < 1
            or agent['turn_generation'] != expected_turn_generation
            or not isinstance(runtime, dict)
            or set(runtime) != set(FROZEN_SNAPSHOT)
            or not isinstance(runtime['runtime_instance_id'], str)
            or not 1 <= len(runtime['runtime_instance_id']) <= 64
            or not isinstance(runtime['admission_epoch'], int)
            or runtime['admission_epoch'] < 1
        ):
            return False
        agent_identity = {'agent_generation': agent['agent_generation']}
        if request_index == 1:
            with self.server.lock:
                self.server.agent_identity = agent_identity
                self.server.runtime_identity = runtime
            return True
        with self.server.lock:
            return (
                self.server.agent_identity == agent_identity
                and self.server.runtime_identity == runtime
            )

    def log_message(self, _format, *_args):
        pass


def _is_frozen_snapshot(value):
    if not isinstance(value, dict) or set(value) != set(FROZEN_SNAPSHOT):
        return False
    return (
        isinstance(value['runtime_instance_id'], str)
        and bool(value['runtime_instance_id'])
        and isinstance(value['admission_epoch'], int)
        and value['admission_epoch'] > 0
        and value['operating_mode'] == MissionState.MAPPING
        and value['availability'] == MissionState.AVAILABLE
        and value['gate_state'] == MissionState.GATE_INHIBITED
        and value['active_step'] == 2**32 - 1
        and value['supported_step_mask'] == 0b1011
        and value['max_steps'] == 3
        and value['named_place_ids'] == []
    )


def _replay_issue136_evidence(evidence):
    """Reject a changed causal record instead of trusting the printed summary."""
    assert set(evidence) == {
        'schema_version', 'head', 'REAL_AUDIO_MODELS', 'REAL_LLM_CORPUS',
        'voice', 'provider', 'missions', 'motion',
    }
    assert evidence['schema_version'] == 1
    assert re.fullmatch(r'[0-9a-f]{40}', evidence['head'])
    assert evidence['REAL_AUDIO_MODELS'] == 'NOT_RUN'
    assert evidence['REAL_LLM_CORPUS'] == 'NOT_RUN'

    voice = evidence['voice']
    assert set(voice) == {'turns', 'speaks'}
    turns = voice['turns']
    speaks = voice['speaks']
    assert isinstance(turns, list) and len(turns) == 2
    assert [turn['text'] for turn in turns] == ['绕到大厅', '半米']
    assert [turn['kind'] for turn in turns] == [
        VoiceTurn.COMMAND, VoiceTurn.COMMAND,
    ]
    assert [turn['voice_seq'] for turn in turns] == [1, 2]
    assert all(set(turn) == {
        'voice_instance_id', 'voice_seq', 'session_id', 'turn_id', 'kind', 'text',
    } for turn in turns)
    assert turns[0]['voice_instance_id'] == turns[1]['voice_instance_id']
    assert turns[0]['session_id'] == turns[1]['session_id']
    assert turns[0]['turn_id'] != turns[1]['turn_id']
    assert speaks == {
        'tts_texts': [CLARIFICATION_TEXT, '任务已完成。'],
        'manual_nonzero_pcm': True,
        'played_feedback_scope_count': 2,
        'completed_scope_count': 2,
        'completed_goal_count': 2,
        'first_completed_before_followup': True,
    }

    provider = evidence['provider']
    assert set(provider) == {'model', 'response_kinds', 'requests'}
    assert provider['model'] == 'Qwen3-0.6B-Q8_0.gguf'
    assert provider['response_kinds'] == ['clarify', 'tool', 'tool']
    requests = provider['requests']
    assert isinstance(requests, list) and len(requests) == 3
    assert [request['turn']['text'] for request in requests] == [
        '绕到大厅', '半米', '半米',
    ]
    assert [request['clarification'] for request in requests] == [
        None, CLARIFICATION_TEXT, CLARIFICATION_TEXT,
    ]
    assert [request['round'] for request in requests] == [1, 1, 2]
    assert [request['snapshot_output'] for request in requests[:2]] == [None, None]
    assert all(set(request) == {
        'agent_system_version', 'tool_schema_version', 'mission_schema_sha256',
        'agent', 'runtime_snapshot', 'turn', 'clarification', 'round',
        'snapshot_output',
    } for request in requests)
    assert all(set(request['agent']) == {
        'agent_generation', 'turn_generation',
    } for request in requests)
    assert all(set(request['runtime_snapshot']) == {
        'runtime_instance_id', 'admission_epoch', 'operating_mode',
        'availability', 'gate_state', 'active_step', 'supported_step_mask',
        'max_steps', 'named_place_ids',
    } for request in requests)
    assert [request['agent']['turn_generation'] for request in requests] == [1, 2, 2]
    assert all(
        request['agent']['agent_generation'] ==
        requests[0]['agent']['agent_generation']
        and request['runtime_snapshot'] == requests[0]['runtime_snapshot']
        for request in requests
    )
    assert all(request['turn'] == {
        'voice_instance_id': turn['voice_instance_id'],
        'voice_seq': turn['voice_seq'],
        'session_id': turn['session_id'],
        'turn_id': turn['turn_id'],
        'text': turn['text'],
    } for request, turn in zip(requests[:2], turns))
    assert requests[2]['turn'] == requests[1]['turn']
    frozen_snapshot = requests[2]['snapshot_output']
    assert _is_frozen_snapshot(frozen_snapshot)
    assert frozen_snapshot['runtime_instance_id'] == (
        requests[2]['runtime_snapshot']['runtime_instance_id']
    )
    assert frozen_snapshot['admission_epoch'] == (
        requests[2]['runtime_snapshot']['admission_epoch']
    )

    missions = evidence['missions']
    assert missions == {
        'pre_followup_goal_count': 0,
        'unique_goal_count': 1,
        'successful_goal_count': 1,
    }
    motion = evidence['motion']
    assert set(motion) == {
        'displacement_m', 'yaw_delta_rad', 'pre_followup_displacement_m',
        'pre_followup_nonzero_command_count', 'pre_followup_armed_count',
        'gate_armed_observed', 'final_zero_stationary',
    }
    assert abs(motion['displacement_m'] - 0.50) <= 0.10
    assert abs(motion['yaw_delta_rad']) <= 0.12
    assert abs(motion['pre_followup_displacement_m']) <= 0.02
    assert motion['pre_followup_nonzero_command_count'] == 0
    assert motion['pre_followup_armed_count'] == 0
    assert motion['gate_armed_observed'] is True
    assert motion['final_zero_stationary'] is True
    return True


def _sample_issue136_evidence():
    runtime = {
        'runtime_instance_id': 'runtime-a',
        'admission_epoch': 7,
    }
    agent = {'agent_generation': 1}
    turns = [
        {
            'voice_instance_id': 'voice-a', 'voice_seq': 1,
            'session_id': 'session-a', 'turn_id': 'turn-a',
            'kind': VoiceTurn.COMMAND, 'text': '绕到大厅',
        },
        {
            'voice_instance_id': 'voice-a', 'voice_seq': 2,
            'session_id': 'session-a', 'turn_id': 'turn-b',
            'kind': VoiceTurn.COMMAND, 'text': '半米',
        },
    ]

    def request(turn, generation, clarification, round_number, snapshot):
        return {
            'agent_system_version': 'voice_nav.agent.system.v1',
            'tool_schema_version': 'voice_nav.agent.tools.v1',
            'mission_schema_sha256': '0' * 64,
            'agent': {**agent, 'turn_generation': generation},
            'runtime_snapshot': {**FROZEN_SNAPSHOT},
            'turn': {
                'voice_instance_id': turn['voice_instance_id'],
                'voice_seq': turn['voice_seq'],
                'session_id': turn['session_id'],
                'turn_id': turn['turn_id'],
                'text': turn['text'],
            },
            'clarification': clarification,
            'round': round_number,
            'snapshot_output': snapshot,
        }
    snapshot = {**FROZEN_SNAPSHOT, **runtime}
    return {
        'schema_version': 1,
        'head': 'a' * 40,
        'REAL_AUDIO_MODELS': 'NOT_RUN',
        'REAL_LLM_CORPUS': 'NOT_RUN',
        'voice': {
            'turns': turns,
            'speaks': {
                'tts_texts': [CLARIFICATION_TEXT, '任务已完成。'],
                'manual_nonzero_pcm': True,
                'played_feedback_scope_count': 2,
                'completed_scope_count': 2,
                'completed_goal_count': 2,
                'first_completed_before_followup': True,
            },
        },
        'provider': {
            'model': 'Qwen3-0.6B-Q8_0.gguf',
            'response_kinds': ['clarify', 'tool', 'tool'],
            'requests': [
                request(turns[0], 1, None, 1, None),
                request(turns[1], 2, CLARIFICATION_TEXT, 1, None),
                request(turns[1], 2, CLARIFICATION_TEXT, 2, snapshot),
            ],
        },
        'missions': {
            'pre_followup_goal_count': 0,
            'unique_goal_count': 1,
            'successful_goal_count': 1,
        },
        'motion': {
            'displacement_m': 0.50,
            'yaw_delta_rad': 0.0,
            'pre_followup_displacement_m': 0.0,
            'pre_followup_nonzero_command_count': 0,
            'pre_followup_armed_count': 0,
            'gate_armed_observed': True,
            'final_zero_stationary': True,
        },
    }


class ScriptedVoiceLoopbackProtocolTest(unittest.TestCase):
    """The fixture itself must honor the frozen three-request dialogue."""

    def test_evidence_replay_rejects_first_turn_mission_mutation(self):
        evidence = _sample_issue136_evidence()
        self.assertTrue(_replay_issue136_evidence(evidence))
        evidence['missions']['pre_followup_goal_count'] = 1
        with self.assertRaises(AssertionError):
            _replay_issue136_evidence(evidence)

    def test_exact_head_injection_must_match_the_checkout(self):
        exact_head = _git_head()
        with patch.dict(os.environ, {'VOICE_NAV_EXACT_HEAD': exact_head}):
            self.assertEqual(_git_head(), exact_head)
        for malformed in ('', 'not-a-commit', 'a' * 39, 'A' * 40):
            with patch.dict(os.environ, {'VOICE_NAV_EXACT_HEAD': malformed}):
                with self.assertRaises(AssertionError):
                    _git_head()
        different_head = (
            ('0' if exact_head[0] != '0' else '1') + exact_head[1:]
        )
        with patch.dict(os.environ, {'VOICE_NAV_EXACT_HEAD': different_head}):
            with self.assertRaises(AssertionError):
                _git_head()

    def test_checkout_head_resolution_failure_is_rejected(self):
        def unavailable_checkout_head():
            raise OSError('unavailable checkout metadata')

        with self.assertRaises(AssertionError):
            _git_head(unavailable_checkout_head)


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


def _load_scripted_voice_demo_launch():
    launch_path = (
        Path(get_package_share_directory('voice_nav_bringup'))
        / 'launch' / 'scripted_voice_demo.launch.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_scripted_voice_demo_launch',
        launch_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load scripted simulation demo launch')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


gazebo_shutdown = _load_gazebo_shutdown_support()
PRODUCT_TEST_PARTITION = gazebo_shutdown.claim_unique_test_partition(
    'i136_multiturn_voice_gazebo'
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Exercise the installed demo launch rather than a test-only graph."""
    launch_module = _load_scripted_voice_demo_launch()
    actions, fixtures = launch_module.create_scripted_voice_demo(
        headless='true',
        shutdown_on_gazebo_exit='false',
        shutdown_when_demo_exits=False,
    )
    return LaunchDescription([
        *actions,
        launch_testing.actions.ReadyToTest(),
    ]), fixtures


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


def _provider_contents(server):
    with server.lock:
        requests = tuple(server.requests)
    return [
        json.loads(json.loads(body.decode('utf-8'))['messages'][-1]['content'])
        for _, body in requests
    ]


def _source_checkout_git_context():
    """Resolve the Git metadata owned by this test source checkout."""
    source_checkout = Path(__file__).resolve().parents[3]
    git_pointer = source_checkout / '.git'
    if git_pointer.is_dir():
        return source_checkout, git_pointer

    pointer_contents = git_pointer.read_text(encoding='utf-8')
    assert '\r' not in pointer_contents
    pointer_lines = pointer_contents.splitlines()
    assert len(pointer_lines) == 1
    pointer_line = pointer_lines[0]
    assert pointer_line.startswith('gitdir: ')
    gitdir_text = pointer_line.removeprefix('gitdir: ')
    assert gitdir_text

    windows_gitdir = re.fullmatch(
        r'([A-Za-z]):[\\/]([^\\/:]+(?:[\\/][^\\/:]+)*)', gitdir_text
    )
    if windows_gitdir is not None and os.name != 'nt':
        git_dir = Path('/mnt', windows_gitdir.group(1).lower())
        for component in windows_gitdir.group(2).replace('\\', '/').split('/'):
            git_dir /= component
    else:
        candidate = Path(gitdir_text)
        git_dir = candidate if candidate.is_absolute() else git_pointer.parent / candidate
    assert git_dir.is_dir()
    return source_checkout, git_dir


def _actual_checkout_head():
    source_checkout, git_dir = _source_checkout_git_context()
    return subprocess.check_output(
        [
            'git', f'--git-dir={git_dir}', f'--work-tree={source_checkout}',
            'rev-parse', '--verify', 'HEAD^{commit}',
        ],
        text=True,
    ).strip()


def _git_head(actual_head_resolver=_actual_checkout_head):
    try:
        actual_head = actual_head_resolver()
    except (OSError, subprocess.CalledProcessError) as error:
        raise AssertionError('unable to resolve the source checkout HEAD') from error
    assert re.fullmatch(r'[0-9a-f]{40}', actual_head)

    injected_head = os.environ.get('VOICE_NAV_EXACT_HEAD')
    if injected_head is not None:
        assert re.fullmatch(r'[0-9a-f]{40}', injected_head)
        assert injected_head == actual_head
    return actual_head


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
        self.speak_successes = set()
        self.speak_success_times = deque(maxlen=8)
        self.mission_successes = set()
        self.mission_goal_ids = set()
        self.mission_statuses = deque(maxlen=200)
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
            self.node.create_subscription(
                GoalStatusArray,
                SPEAK_STATUS_TOPIC,
                self._on_speak_status,
                10,
            ),
        ]
        self.initial_odom = self._wait_until(
            lambda: self.odometry[-1][1] if self._runtime_is_ready() and self.odometry else None,
            45.0,
            'available Runtime and initial odometry',
        )

    def tearDown(self):
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
        self.voice_turns.append((time.monotonic(), turn))

    def _on_mission_status(self, statuses):
        for status in statuses.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)
            self.mission_goal_ids.add(goal_id)
            self.mission_statuses.append((
                time.monotonic(), status.status, goal_id,
            ))
            if status.status == GoalStatus.STATUS_SUCCEEDED:
                self.mission_successes.add(goal_id)

    def _on_gate_state(self, message):
        self.gate_states.append((time.monotonic(), message))

    def _on_speak_status(self, statuses):
        for status in statuses.status_list:
            if status.status != GoalStatus.STATUS_SUCCEEDED:
                continue
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if goal_id not in self.speak_successes:
                self.speak_successes.add(goal_id)
                self.speak_success_times.append((time.monotonic(), goal_id))

    def _runtime_is_ready(self):
        if not self.mission_states:
            return False
        state = self.mission_states[-1][1]
        return (
            state.operating_mode == MissionState.MAPPING
            and
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

        return self._wait_until(
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

    def _signed_displacement(self, odometry):
        start_yaw = _yaw_from_odom(self.initial_odom)
        return (
            (odometry.pose.pose.position.x - self.initial_odom.pose.pose.position.x)
            * math.cos(start_yaw)
            + (odometry.pose.pose.position.y - self.initial_odom.pose.pose.position.y)
            * math.sin(start_yaw)
        )

    def _assert_first_turn_was_side_effect_free(self, first_at, followup_at):
        window_commands = [
            message for received_at, message in self.final_commands
            if first_at <= received_at < followup_at
        ]
        window_odom = [
            message for received_at, message in self.odometry
            if first_at <= received_at < followup_at
        ]
        self.assertGreaterEqual(
            len(window_commands), 4,
            'driver barrier must leave actual first-turn command samples',
        )
        self.assertGreaterEqual(
            len(window_odom), 4,
            'driver barrier must leave actual first-turn odometry samples',
        )
        self.assertEqual(
            [entry for entry in self.mission_statuses
             if first_at <= entry[0] < followup_at],
            [],
        )
        self.assertFalse(any(
            state.state == InternalMotionGateState.ARMED
            and not state.motion_inhibited
            for received_at, state in self.gate_states
            if first_at <= received_at < followup_at
        ))
        self.assertTrue(all(_is_zero(message) for message in window_commands))
        self.assertTrue(all(
            abs(message.twist.twist.linear.x) <= 0.01
            and abs(message.twist.twist.angular.z) <= 0.02
            for message in window_odom
        ))
        self.assertAlmostEqual(
            self._signed_displacement(window_odom[-1]), 0.0, delta=0.02,
        )

    def test_multiturn_clarify_then_move_reaches_product_motion(
        self, proc_output, speech_driver,
    ):
        self._wait_until(
            lambda: len(self.mission_successes) == 1,
            120.0,
            'one successful product ExecuteMission Goal',
        )
        self._wait_until(
            lambda: len(self.voice_turns) == 2 and len(self.speak_successes) == 2,
            30.0,
            'two completed SpeechOutput Speak goals',
        )
        self._wait_for_zero_and_stationarity()

        timed_turns = tuple(self.voice_turns)
        turns = tuple(turn for _, turn in timed_turns)
        self.assertEqual([turn.text for turn in turns], ['绕到大厅', '半米'])
        self.assertEqual([turn.kind for turn in turns], [
            VoiceTurn.COMMAND,
            VoiceTurn.COMMAND,
        ])
        self.assertEqual([turn.voice_seq for turn in turns], [1, 2])
        self.assertEqual(turns[0].voice_instance_id, turns[1].voice_instance_id)
        self.assertTrue(all(turn.session_id and turn.turn_id for turn in turns))
        self.assertEqual(turns[0].session_id, turns[1].session_id)
        self.assertNotEqual(turns[0].turn_id, turns[1].turn_id)
        self.assertEqual(len(self.mission_goal_ids), 1)
        self.assertEqual(len(self.mission_successes), 1)
        self._assert_first_turn_was_side_effect_free(
            timed_turns[0][0], timed_turns[1][0],
        )

        successful_speaks = tuple(self.speak_success_times)
        self.assertEqual(len(successful_speaks), 2)
        self.assertLess(successful_speaks[0][0], timed_turns[1][0])
        proc_output.assertWaitFor(
            expected_output=(
                'EVIDENCE scripted_voice_demo '
                '{"schema_version":1,"simulation_only":true,'
                '"node_graph":["agent_node","mission_runtime_node",'
                '"motion_gate_node","voice_speech_input","voice_speech_output"],'
                '"voice":'
            ),
            process=speech_driver,
            timeout=30.0,
            stream='stdout',
        )

        with self.llm_server.lock:
            request_paths = [path for path, _ in self.llm_server.requests]
            response_kinds = list(self.llm_server.response_kinds)
            first_request_body = self.llm_server.requests[0][1]
        self.assertEqual(request_paths, ['/v1/chat/completions'] * 3)
        self.assertEqual(response_kinds, ['clarify', 'tool', 'tool'])
        contents = _provider_contents(self.llm_server)

        final_odom = self.odometry[-1][1]
        start_yaw = _yaw_from_odom(self.initial_odom)
        displacement = self._signed_displacement(final_odom)
        yaw_delta = _wrapped_angle(_yaw_from_odom(final_odom) - start_yaw)
        self.assertAlmostEqual(displacement, 0.50, delta=0.10)
        self.assertAlmostEqual(yaw_delta, 0.0, delta=0.12)

        self._assert_gate_armed_observed()
        self.assertTrue(self._runtime_is_ready())
        evidence = {
            'schema_version': 1,
            'head': _git_head(),
            'REAL_AUDIO_MODELS': 'NOT_RUN',
            'REAL_LLM_CORPUS': 'NOT_RUN',
            'voice': {
                'turns': [
                    {
                        'voice_instance_id': turn.voice_instance_id,
                        'voice_seq': turn.voice_seq,
                        'session_id': turn.session_id,
                        'turn_id': turn.turn_id,
                        'kind': turn.kind,
                        'text': turn.text,
                    }
                    for turn in turns
                ],
                'speaks': {
                    'tts_texts': [CLARIFICATION_TEXT, '任务已完成。'],
                    'manual_nonzero_pcm': True,
                    'played_feedback_scope_count': 2,
                    'completed_scope_count': 2,
                    'completed_goal_count': len(self.speak_successes),
                    'first_completed_before_followup': True,
                },
            },
            'provider': {
                'model': json.loads(
                    first_request_body.decode('utf-8')
                )['model'],
                'response_kinds': response_kinds,
                'requests': contents,
            },
            'missions': {
                'pre_followup_goal_count': 0,
                'unique_goal_count': len(self.mission_goal_ids),
                'successful_goal_count': len(self.mission_successes),
            },
            'motion': {
                'displacement_m': displacement,
                'yaw_delta_rad': yaw_delta,
                'pre_followup_displacement_m': self._signed_displacement(
                    [message for received_at, message in self.odometry
                     if timed_turns[0][0] <= received_at < timed_turns[1][0]][-1]
                ),
                'pre_followup_nonzero_command_count': sum(
                    not _is_zero(message)
                    for received_at, message in self.final_commands
                    if timed_turns[0][0] <= received_at < timed_turns[1][0]
                ),
                'pre_followup_armed_count': sum(
                    state.state == InternalMotionGateState.ARMED
                    and not state.motion_inhibited
                    for received_at, state in self.gate_states
                    if timed_turns[0][0] <= received_at < timed_turns[1][0]
                ),
                'gate_armed_observed': True,
                'final_zero_stationary': True,
            },
        }
        self.assertTrue(_replay_issue136_evidence(
            json.loads(json.dumps(evidence, ensure_ascii=False))
        ))
        print(
            'EVIDENCE issue136_multiturn_scripted_voice_gazebo '
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
