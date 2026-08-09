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

from importlib.metadata import distribution
import pathlib
import threading
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
    agent = Node(
        package='voice_nav_agent',
        executable='agent_node',
        name='agent_node',
        output='screen',
    )
    return LaunchDescription(
        [agent, launch_testing.actions.ReadyToTest()]
    ), {'agent': agent}


class AgentNodeLaunchTest(unittest.TestCase):
    """Verify the installed node's ports and one formal VoiceTurn path."""

    def setUp(self, proc_info, agent):
        rclpy.init(args=[])
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
        self.mission_event = threading.Event()
        self.stop_requests = []
        self.mission_goals = []
        self.speak_goals = []
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
        turn = VoiceTurn()
        turn.voice_instance_id = 'voice-launch'
        turn.voice_seq = sequence
        turn.session_id = 'session-launch'
        turn.turn_id = turn_id
        turn.kind = VoiceTurn.COMMAND
        turn.text = '前进 1 米'
        turn.confidence = 1.0
        self.turn_publisher.publish(turn)

    def _publish_stop_turn(self, sequence, turn_id):
        turn = VoiceTurn()
        turn.voice_instance_id = 'voice-launch'
        turn.voice_seq = sequence
        turn.session_id = 'session-launch'
        turn.turn_id = turn_id
        turn.kind = VoiceTurn.STOP
        turn.text = '停止'
        turn.confidence = 1.0
        self.turn_publisher.publish(turn)

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

    def _wait_for_agent_graph_snapshot(self):
        expected_subscriptions = {
            '/voice/turn',
            '/mission/state',
            '/mission/execute/_action/feedback',
            '/mission/execute/_action/status',
            '/voice/speak/_action/feedback',
            '/voice/speak/_action/status',
        }
        allowed_publishers = {'/rosout', '/parameter_events'}
        expected_clients = {
            '/mission/execute/_action/send_goal',
            '/mission/execute/_action/get_result',
            '/mission/execute/_action/cancel_goal',
            '/mission/stop',
            '/voice/speak/_action/send_goal',
            '/voice/speak/_action/get_result',
            '/voice/speak/_action/cancel_goal',
        }
        graph_converged = threading.Event()
        last_snapshot = {
            'subscriptions': {},
            'publishers': {},
            'clients': {},
        }
        converged_snapshot = None

        def collect_graph_snapshot():
            nonlocal converged_snapshot, last_snapshot
            snapshot = {
                'subscriptions': dict(
                    self.node.get_subscriber_names_and_types_by_node(
                        'agent_node', '/'
                    )
                ),
                'publishers': dict(
                    self.node.get_publisher_names_and_types_by_node(
                        'agent_node', '/'
                    )
                ),
                'clients': dict(
                    self.node.get_client_names_and_types_by_node(
                        'agent_node', '/'
                    )
                ),
            }
            last_snapshot = snapshot
            if (
                set(snapshot['subscriptions']) == expected_subscriptions
                and set(snapshot['publishers']).issubset(allowed_publishers)
                and set(snapshot['clients']) == expected_clients
            ):
                converged_snapshot = snapshot
                graph_converged.set()
                graph_timer.cancel()

        graph_timer = self.node.create_timer(
            0.05,
            collect_graph_snapshot,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        try:
            if not graph_converged.wait(10.0):
                self.fail(
                    'agent_node graph did not converge; last snapshot: '
                    f'{last_snapshot!r}'
                )
            return converged_snapshot
        finally:
            self.node.destroy_timer(graph_timer)

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

    def _stop(self, request, response):
        self.stop_requests.append(request)
        self.stop_event.set()
        response.code = StopMission.Response.APPLIED
        response.motion_inhibited = True
        return response

    def tearDown(self):
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

        graph_snapshot = self._wait_for_agent_graph_snapshot()
        subscriptions = graph_snapshot['subscriptions']
        publishers = graph_snapshot['publishers']
        clients = graph_snapshot['clients']
        assert set(subscriptions) == {
            '/voice/turn',
            '/mission/state',
            '/mission/execute/_action/feedback',
            '/mission/execute/_action/status',
            '/voice/speak/_action/feedback',
            '/voice/speak/_action/status',
        }
        assert set(publishers).issubset({'/rosout', '/parameter_events'})
        assert set(clients) == {
            '/mission/execute/_action/send_goal',
            '/mission/execute/_action/get_result',
            '/mission/execute/_action/cancel_goal',
            '/mission/stop',
            '/voice/speak/_action/send_goal',
            '/voice/speak/_action/get_result',
            '/voice/speak/_action/cancel_goal',
        }
        graph_text = ' '.join(
            [*subscriptions, *publishers, *clients]
        ).lower()
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

    def test_installed_agent_restarts_state_epoch_through_public_ros_behavior(self):
        """Prove installed A-to-B GID rebuild, B live delivery, and B-token Missions."""
        assert self.turn_matched.wait(10.0)
        assert self.state_matched.wait(10.0)
        assert self.state_current_match_events['A'].wait(10.0)
        assert self.speak_probe.wait_for_server(timeout_sec=10.0)
        assert self.mission_probe.wait_for_server(timeout_sec=10.0)

        state_a_gid = self._state_endpoint_gid()

        assert self.stop_probe.wait_for_service(timeout_sec=10.0)
        self.stop_event.clear()
        self.speak_event.clear()
        self._publish_stop_turn(1, 'state-a-barrier')
        assert self.stop_event.wait(10.0)
        assert self.speak_event.wait(10.0)

        self.mission_event.clear()
        self.speak_event.clear()
        self._publish_rule_turn(2, 'mission-a')
        assert self.speak_event.wait(10.0)
        assert self.mission_event.wait(10.0)
        assert self.speak_event.wait(10.0)
        assert len(self.mission_goals) == 1
        assert self.mission_goals[0].runtime_instance_id == 'runtime-a'
        assert self.mission_goals[0].admission_epoch == 11
        state_a_subscription_gid = self._agent_state_subscription_gid()
        assert state_a_subscription_gid

        state_a_publisher = self.state_publisher
        # Destroy A immediately after its accepted sample; the old sample may
        # still be queued while the graph changes to B.
        self.node.destroy_publisher(state_a_publisher)
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
        self._publish_rule_turn(3, 'mission-without-b-state')
        assert self.speak_event.wait(10.0)
        assert len(self.mission_goals) == 1
        assert not self.mission_event.is_set()

        self.stop_event.clear()
        self.speak_event.clear()
        self._publish_stop_turn(4, 'state-b-barrier')
        assert self.stop_event.wait(10.0)
        assert self.speak_event.wait(10.0)
        assert self.state_current_match_events['B'].wait(10.0)
        state_b_subscription_gid = self._agent_state_subscription_gid()
        assert state_b_subscription_gid
        assert state_b_subscription_gid != state_a_subscription_gid

        self.b_publish_on_speak = True
        self.mission_event.clear()
        self.speak_event.clear()
        self._publish_rule_turn(5, 'publish-b-state')
        assert self.speak_event.wait(10.0)
        assert len(self.mission_goals) == 1
        assert not self.mission_event.is_set()
        assert self.b_state_published.wait(10.0)
        assert self.b_state_publish_count == 1

        self.stop_event.clear()
        self.speak_event.clear()
        self._publish_stop_turn(6, 'state-b-barrier-after-publish')
        assert self.stop_event.wait(10.0)
        assert self.speak_event.wait(10.0)

        self.mission_event.clear()
        self.speak_event.clear()
        self._publish_rule_turn(7, 'mission-b')
        assert self.mission_event.wait(10.0)
        assert self.speak_event.wait(10.0)
        assert len(self.mission_goals) == 2
        assert self.mission_goals[1].runtime_instance_id == 'runtime-b'
        assert self.mission_goals[1].admission_epoch == 22

        self.mission_event.clear()
        self.speak_event.clear()
        self._publish_rule_turn(8, 'mission-b-again')
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


@launch_testing.post_shutdown_test()
class AgentNodeLaunchShutdownTest(unittest.TestCase):
    """Require the launched agent process to exit cleanly."""

    def test_agent_exits_cleanly(self, proc_info, agent):
        """Check launch-managed process exit status after the test."""
        assertExitCodes(proc_info, process=agent, allowable_exit_codes=[0, -2])
