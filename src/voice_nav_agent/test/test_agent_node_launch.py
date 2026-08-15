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

import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.metadata import distribution
import json
import pathlib
import threading
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from voice_nav_interfaces.action import ExecuteMission, Speak
from voice_nav_interfaces.msg import MissionState, VoiceTurn
from voice_nav_interfaces.srv import StopMission


class _LoopbackResponseServer:
    """Test-only literal loopback provider with the frozen three-round script."""

    def __init__(self):
        self.requests = []
        self.request_received = threading.Event()
        self.blocked_response_entered = threading.Event()
        self.release_blocked_response = threading.Event()
        self.blocked_response_released = threading.Event()
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                if self.path != '/v1/chat/completions':
                    self.send_error(404)
                    return
                length = int(self.headers['Content-Length'])
                body = json.loads(self.rfile.read(length).decode('utf-8'))
                owner.requests.append(body)
                owner.request_received.set()
                content = json.loads(body['messages'][-1]['content'])
                turn_id = content['turn']['turn_id']
                blocked = turn_id == 'provider-blocked'
                if blocked:
                    owner.blocked_response_entered.set()
                    assert owner.release_blocked_response.wait(10.0)
                    inner = {'kind': 'reply', 'text': '旧响应不得播报。'}
                elif turn_id == 'provider-after-stop':
                    inner = {'kind': 'reply', 'text': '新响应。'}
                elif turn_id == 'llm-clarify':
                    inner = {'kind': 'clarify', 'text': '请说明目的地。'}
                elif content['round'] == 1:
                    inner = {
                        'kind': 'tool',
                        'tool_call': {
                            'name': 'read_runtime_snapshot',
                            'arguments': {},
                        },
                    }
                else:
                    inner = {
                        'kind': 'tool',
                        'tool_call': {
                            'name': 'propose_mission',
                            'arguments': {
                                'kind': 'mission',
                                'steps': [
                                    {
                                        'kind': 'navigate_to',
                                        'target_id': 'lobby',
                                    }
                                ],
                            },
                        },
                    }
                response = json.dumps(
                    {
                        'choices': [
                            {'message': {'content': json.dumps(inner)}}
                        ]
                    }
                ).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(response)))
                self.end_headers()
                try:
                    self.wfile.write(response)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    if blocked:
                        owner.blocked_response_released.set()

            def log_message(self, _format, *_args):
                pass

        self._server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        port = self._server.server_address[1]
        self.endpoint = f'http://127.0.0.1:{port}'
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
        )
        self._thread.start()

    def close(self):
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(5.0)


_LOOPBACK_RESPONSE_SERVER = None


def _test_state_qos():
    """Create the public MissionState publisher contract for this launch test."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _test_voice_turn_qos():
    """Create the public VoiceTurn publisher contract for this launch test."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the installed product entry point under test."""
    global _LOOPBACK_RESPONSE_SERVER
    _LOOPBACK_RESPONSE_SERVER = _LoopbackResponseServer()
    agent = Node(
        package='voice_nav_agent',
        executable='agent_node',
        name='agent_node',
        output='screen',
        parameters=[{'llm_endpoint': _LOOPBACK_RESPONSE_SERVER.endpoint}],
    )
    return LaunchDescription(
        [agent, launch_testing.actions.ReadyToTest()]
    ), {'agent': agent}


def _wait_for_agent_graph_snapshot(node, timer_factory=None):
    if timer_factory is None:
        timer_factory = node.create_timer
    expected_subscriptions = {
        '/voice/turn',
        '/mission/state',
        '/mission/execute/_action/feedback',
        '/mission/execute/_action/status',
        '/voice/speak/_action/feedback',
        '/voice/speak/_action/status',
    }
    allowed_publishers = {'/rosout', '/parameter_events'}
    graph_converged = threading.Event()
    state_lock = threading.Lock()
    closing = False
    last_snapshot = {
        'subscriptions': {},
        'publishers': {},
    }
    converged_snapshot = None

    def collect_graph_snapshot():
        nonlocal converged_snapshot, last_snapshot
        with state_lock:
            if closing:
                return
            subscriptions = {}
            publishers = {}
            for topic_name, _topic_types in node.get_topic_names_and_types():
                subscription_infos = [
                    info
                    for info in node.get_subscriptions_info_by_topic(
                        topic_name
                    )
                    if (
                        info.node_name == 'agent_node'
                        and info.node_namespace == '/'
                    )
                ]
                if subscription_infos:
                    subscriptions[topic_name] = sorted({
                        info.topic_type for info in subscription_infos
                    })

                publisher_infos = [
                    info
                    for info in node.get_publishers_info_by_topic(topic_name)
                    if (
                        info.node_name == 'agent_node'
                        and info.node_namespace == '/'
                    )
                ]
                if publisher_infos:
                    publishers[topic_name] = sorted({
                        info.topic_type for info in publisher_infos
                    })

            snapshot = {
                'subscriptions': subscriptions,
                'publishers': publishers,
            }
            last_snapshot = snapshot
            if (
                set(snapshot['subscriptions']) == expected_subscriptions
                and set(snapshot['publishers']).issubset(allowed_publishers)
            ):
                converged_snapshot = snapshot
                graph_converged.set()

    graph_timer = timer_factory(
        0.05,
        collect_graph_snapshot,
        clock=Clock(clock_type=ClockType.STEADY_TIME),
    )
    try:
        if not graph_converged.wait(10.0):
            with state_lock:
                timeout_snapshot = last_snapshot
            raise AssertionError(
                'agent_node graph did not converge; last snapshot: '
                f'{timeout_snapshot!r}'
            )
        with state_lock:
            result = converged_snapshot
        assert result is not None
        return result
    finally:
        with state_lock:
            closing = True
            graph_timer.cancel()


class AgentNodeLaunchTest(unittest.TestCase):
    """Verify the installed node's ports and one formal VoiceTurn path."""

    def setUp(self, proc_info, agent):
        rclpy.init(args=[])
        identity = hashlib.sha256(
            self._testMethodName.encode('utf-8')
        ).hexdigest()[:12]
        self.voice_instance_id = f'voice-{identity}'
        self.session_id = f'session-{identity}'
        self.node = rclpy.create_node('agent_launch_probe')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.state_b_node = None
        self.state_publisher_node = None
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()
        self.state_matched_events = {
            'A': threading.Event(),
            'B': threading.Event(),
        }
        self.state_current_match_events = {
            'A': threading.Event(),
            'B': threading.Event(),
        }
        self.state_disconnect_events = {
            'A': threading.Event(),
            'B': threading.Event(),
        }
        self.b_state_published = threading.Event()
        self.b_state_publish_count = 0
        self.b_publish_on_speak = False
        self.state_publisher = self._create_state_publisher('A')
        self.state_matched = self.state_matched_events['A']
        self.state_publisher.publish(
            self._state_message('runtime-a', 11)
        )
        proc_info.assertWaitForStartup(agent, timeout=10.0)

        self.turn_matched = threading.Event()
        self.turn_publisher = self.node.create_publisher(
            VoiceTurn,
            '/voice/turn',
            _test_voice_turn_qos(),
            event_callbacks=PublisherEventCallbacks(
                matched=lambda _event: self.turn_matched.set(),
            ),
        )
        self.stop_event = threading.Event()
        self.speak_event = threading.Event()
        self.new_provider_reply_event = threading.Event()
        self.clarification_speak_event = threading.Event()
        self.mission_event = threading.Event()
        self.stop_requests = []
        self.mission_goals = []
        self.speak_goals = []
        self.hold_mission_result = False
        self.release_mission_result = threading.Event()
        self.stop_service = self.node.create_service(
            StopMission,
            '/mission/stop',
            self._stop,
        )
        self.speak_server = ActionServer(
            self.node,
            Speak,
            '/voice/speak',
            execute_callback=self._speak,
            goal_callback=self._accept_speak_goal,
            cancel_callback=self._accept_cancel,
        )
        self.stop_probe = self.node.create_client(
            StopMission,
            '/mission/stop',
        )
        self.mission_server = ActionServer(
            self.node,
            ExecuteMission,
            '/mission/execute',
            execute_callback=self._mission,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
        )
        self.speak_probe = ActionClient(
            self.node,
            Speak,
            '/voice/speak',
        )
        self.mission_probe = ActionClient(
            self.node,
            ExecuteMission,
            '/mission/execute',
        )

    def _create_state_publisher(self, label):
        publisher_node = self.node
        if label == 'B':
            if self.state_b_node is None:
                self.state_b_node = rclpy.create_node(
                    'agent_launch_state_b'
                )
                self.executor.add_node(self.state_b_node)
            publisher_node = self.state_b_node
        self.state_publisher_node = publisher_node
        return publisher_node.create_publisher(
            MissionState,
            '/mission/state',
            _test_state_qos(),
            event_callbacks=PublisherEventCallbacks(
                matched=self._state_match_callback(label),
            ),
        )

    def _state_match_callback(self, label):
        def matched(event):
            if event.current_count_change < 0:
                self.state_current_match_events[label].clear()
                self.state_disconnect_events[label].set()
                return
            if (
                event.current_count_change > 0
                or event.total_count_change > 0
            ):
                self.state_current_match_events[label].set()
                self.state_matched_events[label].set()

        return matched

    def _publish_b_state_once(self):
        if self.b_state_published.is_set():
            return
        self.b_state_publish_count += 1
        self.state_publisher.publish(
            self._state_message('runtime-b', 22)
        )
        self.b_state_published.set()

    @staticmethod
    def _state_message(runtime_instance_id, admission_epoch):
        message = MissionState()
        message.runtime_instance_id = runtime_instance_id
        message.admission_epoch = admission_epoch
        message.operating_mode = MissionState.NAVIGATION
        message.availability = MissionState.AVAILABLE
        message.gate_state = MissionState.GATE_INHIBITED
        message.active_step = 2**32 - 1
        message.supported_step_mask = 0b1111
        message.max_steps = 3
        message.named_place_ids = ['lobby']
        return message

    def _publish_rule_turn(self, sequence, turn_id):
        self._publish_command_turn(sequence, turn_id, '前进 1 米')

    def _publish_command_turn(self, sequence, turn_id, text):
        turn = VoiceTurn()
        turn.voice_instance_id = self.voice_instance_id
        turn.voice_seq = sequence
        turn.session_id = self.session_id
        turn.turn_id = turn_id
        turn.kind = VoiceTurn.COMMAND
        turn.text = text
        turn.confidence = 1.0
        self.turn_publisher.publish(turn)

    def _publish_unknown_turn(
        self,
        sequence,
        turn_id,
        text,
        *,
        voice_instance_id=None,
        session_id=None,
    ):
        turn = VoiceTurn()
        turn.voice_instance_id = voice_instance_id or self.voice_instance_id
        turn.voice_seq = sequence
        turn.session_id = session_id or self.session_id
        turn.turn_id = turn_id
        turn.kind = VoiceTurn.COMMAND
        turn.text = text
        turn.confidence = 1.0
        self.turn_publisher.publish(turn)

    def _publish_stop_turn(
        self,
        sequence,
        turn_id,
        *,
        voice_instance_id=None,
        session_id=None,
    ):
        turn = VoiceTurn()
        turn.voice_instance_id = voice_instance_id or self.voice_instance_id
        turn.voice_seq = sequence
        turn.session_id = session_id or self.session_id
        turn.turn_id = turn_id
        turn.kind = VoiceTurn.STOP
        turn.text = '停止'
        turn.confidence = 1.0
        self.turn_publisher.publish(turn)

    def _refresh_runtime_snapshot(self, sequence, turn_id):
        """Force A-to-B recovery and prove that the new Runtime is usable."""
        assert self.stop_probe.wait_for_service(timeout_sec=10.0)
        assert self.state_current_match_events['A'].wait(10.0)
        old_subscription_gid = self._agent_state_subscription_gid()
        old_publisher = self.state_publisher
        old_node = self.state_publisher_node
        old_node.destroy_publisher(old_publisher)
        self.state_current_match_events['B'].clear()
        self.state_disconnect_events['B'].clear()
        self.state_publisher = self._create_state_publisher('B')
        assert self.state_current_match_events['B'].wait(10.0)
        rebuilt_before_stop = old_subscription_gid is None or (
            self._agent_state_subscription_gid() != old_subscription_gid
        )

        self.stop_event.clear()
        self.speak_event.clear()
        if not rebuilt_before_stop:
            self.state_current_match_events['B'].clear()
        self._publish_stop_turn(sequence, turn_id)
        assert self.stop_event.wait(10.0)
        assert self.speak_event.wait(10.0)
        if not rebuilt_before_stop:
            assert self.state_disconnect_events['B'].wait(10.0)
            assert self.state_current_match_events['B'].wait(10.0)
        if old_subscription_gid is not None:
            assert self._agent_state_subscription_gid() != old_subscription_gid
        self.state_publisher.publish(self._state_message('runtime-b', 22))
        assert self.state_publisher.wait_for_all_acked(
            Duration(seconds=10)
        )

        initial_missions = len(self.mission_goals)
        initial_requests = (
            len(_LOOPBACK_RESPONSE_SERVER.requests)
            if _LOOPBACK_RESPONSE_SERVER is not None
            else 0
        )
        self.speak_event.clear()
        visible_turn_id = f'snapshot-{sequence}'
        self._publish_command_turn(sequence + 1, visible_turn_id, '前进 3 米')
        assert self.speak_event.wait(10.0)
        visible_goals = [
            goal for goal in self.speak_goals
            if goal.turn_id == visible_turn_id
        ]
        assert [goal.text for goal in visible_goals] == [
            '移动距离超出安全范围。'
        ]
        assert len(self.mission_goals) == initial_missions
        if _LOOPBACK_RESPONSE_SERVER is not None:
            assert len(_LOOPBACK_RESPONSE_SERVER.requests) == initial_requests
        return sequence + 1

    def _state_endpoint_gid(self, label='A'):
        node_name = (
            'agent_launch_probe'
            if label == 'A'
            else 'agent_launch_state_b'
        )
        infos = [
            info
            for info in self.node.get_publishers_info_by_topic(
                '/mission/state'
            )
            if info.node_name == node_name
        ]
        assert len(infos) == 1
        endpoint_gid = bytes(infos[0].endpoint_gid)
        assert endpoint_gid
        return endpoint_gid

    def _agent_state_subscription_gid(self):
        infos = [
            info
            for info in self.node.get_subscriptions_info_by_topic(
                '/mission/state'
            )
            if info.node_name == 'agent_node'
        ]
        assert len(infos) <= 1
        if not infos:
            return None
        endpoint_gid = bytes(infos[0].endpoint_gid)
        assert endpoint_gid
        return endpoint_gid

    @staticmethod
    def _accept_goal(_request):
        return GoalResponse.ACCEPT

    def _accept_speak_goal(self, _request):
        if self.b_publish_on_speak:
            self._publish_b_state_once()
            self.b_publish_on_speak = False
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _mission(self, goal_handle):
        self.mission_goals.append(goal_handle.request)
        self.mission_event.set()
        if self.hold_mission_result:
            assert self.release_mission_result.wait(10.0)
        goal_handle.succeed()
        result = ExecuteMission.Result()
        result.code = ExecuteMission.Result.SUCCEEDED
        result.failed_step = -1
        return result

    def _speak(self, goal_handle):
        self.speak_goals.append(goal_handle.request)
        self.speak_event.set()
        if goal_handle.request.text == '新响应。':
            self.new_provider_reply_event.set()
        if goal_handle.request.text == '请说明目的地。':
            self.clarification_speak_event.set()
        goal_handle.succeed()
        result = Speak.Result()
        result.code = Speak.Result.COMPLETED
        return result

    def _stop(self, request, response):
        self.stop_requests.append(request)
        self.stop_event.set()
        response.code = StopMission.Response.APPLIED
        response.motion_inhibited = True
        return response

    def tearDown(self):
        self.release_mission_result.set()
        if _LOOPBACK_RESPONSE_SERVER is not None:
            _LOOPBACK_RESPONSE_SERVER.release_blocked_response.set()
        self.executor.shutdown()
        self.spin_thread.join(5.0)
        self.speak_probe.destroy()
        self.mission_probe.destroy()
        self.stop_probe.destroy()
        self.mission_server.destroy()
        self.speak_server.destroy()
        self.stop_service.destroy()
        self.node.destroy_publisher(self.turn_publisher)
        if self.state_publisher is not None:
            self.state_publisher_node.destroy_publisher(
                self.state_publisher
            )
        if self.state_b_node is not None:
            self.executor.remove_node(self.state_b_node)
            self.state_b_node.destroy_node()
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_entrypoint_ports_qos_and_runtime_graph(self):
        """Exercise STOP through the launched product node and inspect graph."""
        assert self.turn_matched.wait(10.0)
        assert self.state_matched.wait(10.0)
        assert self.speak_probe.wait_for_server(timeout_sec=10.0)
        assert self.mission_probe.wait_for_server(timeout_sec=10.0)

        turn = VoiceTurn()
        turn.voice_instance_id = 'voice-launch'
        turn.voice_seq = 1
        turn.session_id = 'session-launch'
        turn.turn_id = 'stop-launch'
        turn.kind = VoiceTurn.STOP
        turn.text = '停止'
        turn.confidence = 1.0
        self.turn_publisher.publish(turn)
        assert self.stop_event.wait(10.0)
        assert self.speak_event.wait(10.0)
        assert self.state_current_match_events['A'].wait(10.0)

        assert len(self.stop_requests) == 1
        assert self.stop_requests[0].request_id == 'stop-launch'
        assert self.stop_requests[0].source_instance_id == 'voice-launch'
        assert self.stop_requests[0].source_seq == 1
        assert self.stop_requests[0].reason == 'voice_stop'
        assert self.speak_goals[0].priority == Speak.Goal.URGENT
        assert self.speak_goals[0].allow_barge_in

        graph_snapshot = _wait_for_agent_graph_snapshot(self.node)
        subscriptions = graph_snapshot['subscriptions']
        publishers = graph_snapshot['publishers']
        assert set(subscriptions) == {
            '/voice/turn',
            '/mission/state',
            '/mission/execute/_action/feedback',
            '/mission/execute/_action/status',
            '/voice/speak/_action/feedback',
            '/voice/speak/_action/status',
        }
        assert set(publishers).issubset({'/rosout', '/parameter_events'})
        graph_text = ' '.join([*subscriptions, *publishers]).lower()
        for forbidden in ('velocity', 'nav2', 'gazebo', 'controller'):
            assert forbidden not in graph_text

        turn_qos = self._subscription_info('/voice/turn')
        state_qos = self._subscription_info('/mission/state')
        state_publishers = [
            info
            for info in self.node.get_publishers_info_by_topic(
                '/mission/state'
            )
            if info.node_name == 'agent_launch_probe'
        ]
        assert len(state_publishers) == 1
        assert bytes(state_publishers[0].endpoint_gid)
        assert turn_qos.reliability == ReliabilityPolicy.RELIABLE
        assert turn_qos.durability == DurabilityPolicy.VOLATILE
        assert turn_qos.history in (
            HistoryPolicy.KEEP_LAST,
            HistoryPolicy.UNKNOWN,
        )
        assert turn_qos.depth == 1 or (
            turn_qos.history == HistoryPolicy.UNKNOWN
            and turn_qos.depth == 0
        )
        assert state_qos.reliability == ReliabilityPolicy.RELIABLE
        assert state_qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert state_qos.history in (
            HistoryPolicy.KEEP_LAST,
            HistoryPolicy.UNKNOWN,
        )
        assert state_qos.depth == 1 or (
            state_qos.history == HistoryPolicy.UNKNOWN
            and state_qos.depth == 0
        )

    def test_installed_response_provider_clarifies_then_proposes_one_mission(self):
        """Exercise the installed Agent through the frozen loopback provider."""
        assert _LOOPBACK_RESPONSE_SERVER is not None
        assert self.turn_matched.wait(10.0)
        assert self.state_matched.wait(10.0)
        assert self.speak_probe.wait_for_server(timeout_sec=10.0)
        assert self.mission_probe.wait_for_server(timeout_sec=10.0)
        barrier_sequence = self._refresh_runtime_snapshot(
            9, 'response-state-refresh'
        )
        first_request = len(_LOOPBACK_RESPONSE_SERVER.requests)

        self.speak_event.clear()
        self.clarification_speak_event.clear()
        _LOOPBACK_RESPONSE_SERVER.request_received.clear()
        self._publish_unknown_turn(
            barrier_sequence + 1,
            'llm-clarify',
            '请沿着大厅右侧绕过去',
        )
        assert _LOOPBACK_RESPONSE_SERVER.request_received.wait(10.0)
        assert self.clarification_speak_event.wait(10.0)
        clarification_goals = [
            goal for goal in self.speak_goals
            if goal.turn_id == 'llm-clarify'
        ]
        assert [goal.text for goal in clarification_goals] == ['请说明目的地。']

        self.hold_mission_result = True
        self.mission_event.clear()
        self._publish_unknown_turn(
            barrier_sequence + 2,
            'llm-mission',
            '请继续带我去那里',
        )
        assert self.mission_event.wait(10.0)

        requests = _LOOPBACK_RESPONSE_SERVER.requests[first_request:]
        assert len(requests) == 3
        assert requests[0]['model'] == 'Qwen3-0.6B-Q8_0.gguf'
        assert requests[0]['stream'] is False
        assert requests[0]['messages'][0]['content'] == '/no_think'
        assert [tool['function']['name'] for tool in requests[0]['tools']] == [
            'read_runtime_snapshot',
            'propose_mission',
            'cancel_owned_mission',
        ]
        second = json.loads(requests[1]['messages'][-1]['content'])
        third = json.loads(requests[2]['messages'][-1]['content'])
        assert second['clarification'] == '请说明目的地。'
        assert second['round'] == 1
        assert third['round'] == 2
        assert third['snapshot_output'] == {
            'runtime_instance_id': 'runtime-b',
            'admission_epoch': 22,
            'operating_mode': MissionState.NAVIGATION,
            'availability': MissionState.AVAILABLE,
            'gate_state': MissionState.GATE_INHIBITED,
            'supported_step_mask': 0b1111,
            'max_steps': 3,
            'named_place_ids': ['lobby'],
        }
        response_missions = [
            goal for goal in self.mission_goals
            if goal.source_seq > 0 and goal.runtime_instance_id == 'runtime-b'
        ]
        assert len(response_missions) == 1
        goal = response_missions[0]
        assert goal.source_seq > 0
        assert goal.runtime_instance_id == 'runtime-b'
        assert goal.admission_epoch == 22
        assert goal.steps[0].target_id == 'lobby'

        self.release_mission_result.set()

    def test_z_blocked_provider_stop_is_direct_and_late_reply_is_fenced(self):
        """STOP reaches its port before a blocked provider can create effects."""
        assert _LOOPBACK_RESPONSE_SERVER is not None
        assert self.turn_matched.wait(10.0)
        assert self.state_matched.wait(10.0)
        assert self.stop_probe.wait_for_service(timeout_sec=10.0)
        barrier_sequence = self._refresh_runtime_snapshot(
            12, 'blocked-state-refresh'
        )

        self.stop_event.clear()
        self.speak_event.clear()
        self.new_provider_reply_event.clear()
        _LOOPBACK_RESPONSE_SERVER.blocked_response_entered.clear()
        _LOOPBACK_RESPONSE_SERVER.blocked_response_released.clear()
        _LOOPBACK_RESPONSE_SERVER.release_blocked_response.clear()
        initial_stops = len(self.stop_requests)
        initial_speaks = len(self.speak_goals)
        initial_missions = len(self.mission_goals)

        self._publish_unknown_turn(
            barrier_sequence + 1,
            'provider-blocked',
            '请沿着大厅右侧绕过去',
        )
        assert _LOOPBACK_RESPONSE_SERVER.blocked_response_entered.wait(10.0)

        self._publish_stop_turn(
            barrier_sequence + 2,
            'stop-during-provider',
        )
        assert self.stop_event.wait(10.0)
        assert self.speak_event.wait(10.0)
        assert len(self.stop_requests) == initial_stops + 1
        assert len(self.speak_goals) == initial_speaks + 1
        assert len(self.mission_goals) == initial_missions

        _LOOPBACK_RESPONSE_SERVER.release_blocked_response.set()
        assert _LOOPBACK_RESPONSE_SERVER.blocked_response_released.wait(10.0)
        self._publish_unknown_turn(
            barrier_sequence + 3,
            'provider-after-stop',
            '请继续说明情况',
        )
        assert self.new_provider_reply_event.wait(10.0)

        assert len(self.stop_requests) == initial_stops + 1
        assert len(self.speak_goals) == initial_speaks + 2
        assert self.speak_goals[-1].text == '新响应。'
        assert len(self.mission_goals) == initial_missions

    def test_installed_agent_restarts_state_epoch_through_public_ros_behavior(self):
        """Prove installed A-to-B GID rebuild, B live delivery, and B-token Missions."""
        assert self.turn_matched.wait(10.0)
        assert self.state_matched.wait(10.0)
        assert self.speak_probe.wait_for_server(timeout_sec=10.0)
        assert self.mission_probe.wait_for_server(timeout_sec=10.0)
        barrier_sequence = self._refresh_runtime_snapshot(
            1, 'state-a-refresh'
        )
        state_a_gid = self._state_endpoint_gid('B')

        self.mission_event.clear()
        self.speak_event.clear()
        self._publish_rule_turn(barrier_sequence + 1, 'mission-a')
        assert self.speak_event.wait(10.0)
        assert self.mission_event.wait(10.0)
        assert self.speak_event.wait(10.0)
        assert len(self.mission_goals) == 1
        assert self.mission_goals[0].runtime_instance_id == 'runtime-b'
        assert self.mission_goals[0].admission_epoch == 22
        state_a_subscription_gid = self._agent_state_subscription_gid()
        assert state_a_subscription_gid

        state_a_publisher = self.state_publisher
        # Destroy A immediately after its accepted sample; the old sample may
        # still be queued while the graph changes to B.
        self.state_matched_events['B'].clear()
        self.state_current_match_events['B'].clear()
        self.state_disconnect_events['B'].clear()
        self.state_publisher_node.destroy_publisher(state_a_publisher)
        self.state_publisher = self._create_state_publisher('B')
        assert self.state_matched_events['B'].wait(10.0)
        assert self.state_current_match_events['B'].wait(10.0)
        state_b_gid = self._state_endpoint_gid('B')
        assert state_b_gid != state_a_gid
        assert self._agent_state_subscription_gid() == (
            state_a_subscription_gid
        )
        self.state_current_match_events['B'].clear()

        self.mission_event.clear()
        self.speak_event.clear()
        self._publish_rule_turn(barrier_sequence + 2, 'mission-without-b-state')
        assert self.speak_event.wait(10.0)
        assert len(self.mission_goals) == 1
        assert not self.mission_event.is_set()

        self.stop_event.clear()
        self.speak_event.clear()
        self._publish_stop_turn(barrier_sequence + 3, 'state-b-barrier')
        assert self.stop_event.wait(10.0)
        assert self.speak_event.wait(10.0)
        assert self.state_current_match_events['B'].wait(10.0)
        state_b_subscription_gid = self._agent_state_subscription_gid()
        assert state_b_subscription_gid
        assert state_b_subscription_gid != state_a_subscription_gid

        self.b_publish_on_speak = True
        self.mission_event.clear()
        self.speak_event.clear()
        self._publish_rule_turn(barrier_sequence + 4, 'publish-b-state')
        assert self.speak_event.wait(10.0)
        assert len(self.mission_goals) == 1
        assert not self.mission_event.is_set()
        assert self.b_state_published.wait(10.0)
        assert self.b_state_publish_count == 1

        self.stop_event.clear()
        self.speak_event.clear()
        self._publish_stop_turn(
            barrier_sequence + 5, 'state-b-barrier-after-publish'
        )
        assert self.stop_event.wait(10.0)
        assert self.speak_event.wait(10.0)

        self.mission_event.clear()
        self.speak_event.clear()
        self._publish_rule_turn(barrier_sequence + 6, 'mission-b')
        assert self.mission_event.wait(10.0)
        assert self.speak_event.wait(10.0)
        assert len(self.mission_goals) == 2
        assert self.mission_goals[1].runtime_instance_id == 'runtime-b'
        assert self.mission_goals[1].admission_epoch == 22

        self.mission_event.clear()
        self.speak_event.clear()
        self._publish_rule_turn(barrier_sequence + 7, 'mission-b-again')
        assert self.mission_event.wait(10.0)
        assert self.speak_event.wait(10.0)
        assert len(self.mission_goals) == 3
        assert self.mission_goals[2].runtime_instance_id == 'runtime-b'
        assert self.mission_goals[2].admission_epoch == 22

    def _subscription_info(self, topic):
        infos = self.node.get_subscriptions_info_by_topic(topic)
        matching = [info for info in infos if info.node_name == 'agent_node']
        assert len(matching) == 1
        return matching[0].qos_profile

    def test_installed_metadata_has_only_product_dependencies(self):
        """Inspect installed package metadata without scanning source files."""
        package_xml = pathlib.Path(
            get_package_share_directory('voice_nav_agent')
        ) / 'package.xml'
        root = ElementTree.parse(package_xml).getroot()
        exec_dependencies = {
            element.text
            for element in root.findall('exec_depend')
        }
        assert exec_dependencies == {
            'action_msgs',
            'rclpy',
            'voice_nav_interfaces',
        }
        installed_files = [str(path).replace('\\', '/') for path in (
            distribution('voice_nav_agent').files or []
        )]
        assert not any('/test/' in path for path in installed_files)
        assert not any(
            path.startswith('voice_nav_agent/test')
            for path in installed_files
        )


def _run_graph_snapshot_timer_callback_before_factory_return():
    """Keep synchronous timer callbacks safe before factory return."""
    expected_subscriptions = {
        '/voice/turn',
        '/mission/state',
        '/mission/execute/_action/feedback',
        '/mission/execute/_action/status',
        '/voice/speak/_action/feedback',
        '/voice/speak/_action/status',
    }
    expected_publishers = {'/rosout', '/parameter_events'}
    all_topics = expected_subscriptions | expected_publishers
    endpoint = SimpleNamespace(
        node_name='agent_node',
        node_namespace='/',
        topic_type='test/type',
    )

    class FakeNode:
        def __init__(self):
            self.calls = {
                'topics': 0,
                'subscriptions': 0,
                'publishers': 0,
            }

        def get_topic_names_and_types(self):
            self.calls['topics'] += 1
            return [(topic, ['test/type']) for topic in all_topics]

        def get_subscriptions_info_by_topic(self, topic):
            self.calls['subscriptions'] += 1
            return [endpoint] if topic in expected_subscriptions else []

        def get_publishers_info_by_topic(self, topic):
            self.calls['publishers'] += 1
            return [endpoint] if topic in expected_publishers else []

    class FakeTimer:
        def __init__(self):
            self.cancel_calls = 0
            self.cancel_thread = None

        def cancel(self):
            self.cancel_calls += 1
            self.cancel_thread = threading.current_thread()

    fake_node = FakeNode()
    fake_timer = FakeTimer()

    def fake_timer_factory(_period, callback, *, clock):
        assert clock.clock_type == ClockType.STEADY_TIME
        callback()
        return fake_timer

    snapshot = _wait_for_agent_graph_snapshot(
        fake_node,
        fake_timer_factory,
    )
    expected_snapshot = {
        'subscriptions': {
            topic: ['test/type'] for topic in expected_subscriptions
        },
        'publishers': {
            topic: ['test/type'] for topic in expected_publishers
        },
    }

    assert snapshot == expected_snapshot
    assert fake_node.calls == {
        'topics': 1,
        'subscriptions': len(all_topics),
        'publishers': len(all_topics),
    }
    assert fake_timer.cancel_calls == 1
    assert fake_timer.cancel_thread is threading.current_thread()


class GraphSnapshotTimerRegressionTest(unittest.TestCase):
    """Exercise the timer seam without ROS launch fixtures."""

    def test_callback_before_factory_return(self):
        _run_graph_snapshot_timer_callback_before_factory_return()


@launch_testing.post_shutdown_test()
class AgentNodeLaunchShutdownTest(unittest.TestCase):
    """Require the launched agent process to exit cleanly."""

    def test_agent_exits_cleanly(self, proc_info, agent):
        """Check launch-managed process exit status after the test."""
        assertExitCodes(proc_info, process=agent, allowable_exit_codes=[0, -2])

    def test_loopback_server_exits_cleanly(self):
        """Release the test-only HTTP server after the installed-node scenario."""
        global _LOOPBACK_RESPONSE_SERVER
        if _LOOPBACK_RESPONSE_SERVER is not None:
            _LOOPBACK_RESPONSE_SERVER.close()
            _LOOPBACK_RESPONSE_SERVER = None
