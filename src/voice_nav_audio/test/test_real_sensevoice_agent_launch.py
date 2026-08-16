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

"""Real locked SenseVoice -> existing SpeechInputNode -> Agent gate."""

from hashlib import sha256
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest

from launch import LaunchDescription
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
from voice_nav_interfaces.msg import VoiceTurn
from voice_nav_interfaces.srv import StopMission


_EXPECTED_TEXT = '开放时间早上9点至下午5点。'
_EXPECTED_SUBSCRIPTIONS = {
    '/voice/turn',
    '/mission/state',
    '/mission/execute/_action/feedback',
    '/mission/execute/_action/status',
    '/voice/speak/_action/feedback',
    '/voice/speak/_action/status',
}
_ALLOWED_AGENT_PUBLISHERS = {'/rosout', '/parameter_events'}
_EXPECTED_ASSETS = {
    'wav': (178988, 'b77f1794fe374a0ba1ee1dc458bfaf9349496cbbfc32780c50ba3c5a7ad8e373'),
    'sensevoice_model': (
        239233841,
        'c71f0ce00bec95b07744e116345e33d8cbbe08cef896382cf907bf4b51a2cd51',
    ),
    'tokens': (315894, 'f449eb28dc567533d7fa59be34e2abca8784f771850c78a47fb731a31429a1dc'),
    'silero_vad': (
        212860,
        'c36d490aff5ab924ca6c7aeec4d8f6bd3d22db6fa17611b9c5b17eae58ac3a20',
    ),
}


def _gate_env(name):
    return os.environ.get(
        f'VOICE_NAV_REAL_GATE_{name}_OVERRIDE',
        os.environ.get(f'VOICE_NAV_REAL_GATE_{name}'),
    )


def _voice_turn_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _sha256(path):
    digest = sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _wait_for_final_report(path, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.is_file():
            try:
                report = json.loads(path.read_text(encoding='utf-8'))
            except (OSError, json.JSONDecodeError):
                report = None
            if report is not None and report.get('status') == 'passed':
                return report
        time.sleep(0.01)
    raise AssertionError('atomic real-model gate report was not published')


def _asset_paths():
    paths = {
        'wav': Path(os.environ['VOICE_NAV_SENSEVOICE_WAV']),
        'sensevoice_model': Path(os.environ['VOICE_NAV_SENSEVOICE_MODEL']),
        'tokens': Path(os.environ['VOICE_NAV_SENSEVOICE_TOKENS']),
        'silero_vad': Path(os.environ['VOICE_NAV_SENSEVOICE_VAD_MODEL']),
    }
    expected_suffixes = {
        'wav': ('asr', 'sensevoice-small-int8-2024-07-17', 'test_wavs', 'zh.wav'),
        'sensevoice_model': ('asr', 'sensevoice-small-int8-2024-07-17', 'model.int8.onnx'),
        'tokens': ('asr', 'sensevoice-small-int8-2024-07-17', 'tokens.txt'),
        'silero_vad': ('vad', 'silero_vad.int8.onnx'),
    }
    for name, path in paths.items():
        assert path.is_file(), f'missing canonical real-gate asset: {path}'
        assert tuple(path.parts[-len(expected_suffixes[name]):]) == expected_suffixes[name]
        expected_size, expected_hash = _EXPECTED_ASSETS[name]
        assert path.stat().st_size == expected_size
        assert _sha256(path) == expected_hash
    return paths


def _agent_graph(node):
    subscriptions = set()
    publishers = set()
    for topic_name, _topic_types in node.get_topic_names_and_types():
        for endpoint in node.get_subscriptions_info_by_topic(topic_name):
            if endpoint.node_name == 'agent_node' and endpoint.node_namespace == '/':
                subscriptions.add(topic_name)
        for endpoint in node.get_publishers_info_by_topic(topic_name):
            if endpoint.node_name == 'agent_node' and endpoint.node_namespace == '/':
                publishers.add(topic_name)
    return subscriptions, publishers


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the installed real voice root and the installed Agent."""
    gate_head = _gate_env('HEAD')
    assert gate_head not in (None, '', 'unknown')
    assets = _asset_paths()
    report_path = Path(_gate_env('REPORT'))
    report_path.unlink(missing_ok=True)
    agent = Node(
        package='voice_nav_agent',
        executable='agent_node',
        name='agent_node',
        output='screen',
    )
    voice_node = Node(
        package='voice_nav_audio',
        executable='voice_node',
        name='voice_node',
        output='screen',
        parameters=[{
            'input_profile': 'sensevoice_wav',
            'input_wav': str(assets['wav']),
            'silero_vad_model': str(assets['silero_vad']),
            'sensevoice_model': str(assets['sensevoice_model']),
            'sensevoice_tokens': str(assets['tokens']),
            'result_path': str(report_path),
            'exact_head': gate_head,
        }],
    )
    return LaunchDescription([
        agent,
        voice_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'agent': agent,
        'voice_node': voice_node,
    }


class RealSenseVoiceAgentLaunchTest(unittest.TestCase):
    """Assert the actual model path produces one safe Agent response."""

    def setUp(self, agent, voice_node):
        del agent, voice_node
        rclpy.init(args=[])
        self.node = rclpy.create_node('real_sensevoice_agent_probe')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.voice_turns = []
        self.mission_goals = []
        self.stop_requests = []
        self.speak_goals = []
        self.voice_event = threading.Event()
        self.speak_event = threading.Event()
        self.voice_subscription = self.node.create_subscription(
            VoiceTurn,
            '/voice/turn',
            self._on_voice_turn,
            _voice_turn_qos(),
        )
        self.mission_server = ActionServer(
            self.node,
            ExecuteMission,
            '/mission/execute',
            execute_callback=self._execute_mission,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
        )
        self.stop_service = self.node.create_service(
            StopMission,
            '/mission/stop',
            self._stop_mission,
        )
        self.speak_server = ActionServer(
            self.node,
            Speak,
            '/voice/speak',
            execute_callback=self._speak,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
        )

    def tearDown(self):
        self.mission_server.destroy()
        self.speak_server.destroy()
        self.node.destroy_service(self.stop_service)
        self.node.destroy_subscription(self.voice_subscription)
        self.executor.shutdown()
        self.spin_thread.join(5.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _on_voice_turn(self, turn):
        self.voice_turns.append(turn)
        self.voice_event.set()

    def _execute_mission(self, goal_handle):
        self.mission_goals.append(goal_handle.request)
        goal_handle.succeed()
        result = ExecuteMission.Result()
        result.code = ExecuteMission.Result.SUCCEEDED
        result.failed_step = -1
        return result

    def _stop_mission(self, request, response):
        self.stop_requests.append(request)
        response.code = StopMission.Response.DUPLICATE
        response.motion_inhibited = True
        return response

    def _speak(self, goal_handle):
        self.speak_goals.append(goal_handle.request)
        self.speak_event.set()
        goal_handle.succeed()
        result = Speak.Result()
        result.code = Speak.Result.COMPLETED
        return result

    def test_locked_zh_wav_is_one_command_and_safe_reply(self):
        assert self.voice_event.wait(45.0)
        assert self.speak_event.wait(30.0)
        assert len(self.voice_turns) == 1
        turn = self.voice_turns[0]
        assert turn.kind == VoiceTurn.COMMAND
        assert turn.text == _EXPECTED_TEXT
        assert turn.voice_seq == 1
        assert len(self.mission_goals) == 0
        assert len(self.stop_requests) == 0
        assert len(self.speak_goals) == 1
        assert self.speak_goals[0].text == '当前没有可用的 Runtime 状态。'
        assert self.speak_goals[0].session_id == turn.session_id
        assert self.speak_goals[0].turn_id == turn.turn_id

        subscriptions, publishers = _agent_graph(self.node)
        assert subscriptions == _EXPECTED_SUBSCRIPTIONS
        assert publishers.issubset(_ALLOWED_AGENT_PUBLISHERS)
        graph_forbidden = ' '.join(sorted(subscriptions | publishers)).lower()
        for forbidden in (
            'velocity', 'nav2', 'gazebo', 'controller', '/voice/kws', '/voice/audio'
        ):
            assert forbidden not in graph_forbidden

        report_path = Path(_gate_env('REPORT'))
        report = _wait_for_final_report(report_path, timeout=10.0)
        assert report['schema_version'] == 'voice_nav.real_model_gate.v1'
        assert report['status'] == 'passed'
        assert report['exact_head'] == _gate_env('HEAD')
        assert report['provider']['voice_turn_count'] == 1
        assert report['provider']['command_count'] == 1
        assert report['turns'][0]['text'] == _EXPECTED_TEXT
        verified_paths = _asset_paths()
        for name, path in verified_paths.items():
            expected_size, expected_hash = _EXPECTED_ASSETS[name]
            evidence = report['assets'][name]
            assert evidence['expected_size'] == expected_size
            assert evidence['expected_sha256'] == expected_hash
            evidence['verified_size'] = path.stat().st_size
            evidence['verified_sha256'] = _sha256(path)
            assert evidence['verified_size'] == expected_size
            assert evidence['verified_sha256'] == expected_hash
        report['agent'] = {
            'voice_turn_count': len(self.voice_turns),
            'command_count': sum(turn.kind == VoiceTurn.COMMAND for turn in self.voice_turns),
            'speak_count': len(self.speak_goals),
            'mission_count': len(self.mission_goals),
            'stop_mission_count': len(self.stop_requests),
            'safe_reply': self.speak_goals[0].text,
        }
        report['graph_allowlist'] = {
            'agent_subscriptions': sorted(subscriptions),
            'agent_publishers': sorted(publishers),
            'forbidden_absent': True,
        }
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )


class RealModelReportTransactionTest(unittest.TestCase):
    """A stale/partial report is never accepted as the gate result."""

    def test_failed_and_partial_reports_wait_for_atomic_passed_final(self):
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / 'gate.json'
            report_path.write_text('{"status":"failed"}\n', encoding='utf-8')
            partial_path = Path(f'{report_path}.tmp.partial')
            partial_path.write_text('{"status":', encoding='utf-8')

            with self.assertRaises(AssertionError):
                _wait_for_final_report(report_path, timeout=0.1)
            self.assertTrue(partial_path.is_file())

            complete_path = Path(f'{report_path}.tmp.complete')
            complete_path.write_text('{"status":"passed"}\n', encoding='utf-8')
            os.replace(complete_path, report_path)
            self.assertEqual(_wait_for_final_report(report_path)['status'], 'passed')


@launch_testing.post_shutdown_test()
class RealSenseVoiceAgentShutdownTest(unittest.TestCase):
    """Require both the Agent and actual gate runner to exit cleanly."""

    def test_processes_exit_cleanly(self, proc_info, agent, voice_node):
        assertExitCodes(
            proc_info,
            process=agent,
            allowable_exit_codes=[0, -2],
        )
        assertExitCodes(
            proc_info,
            process=voice_node,
            allowable_exit_codes=[0, -2],
        )
