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

import threading

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.executors import SingleThreadedExecutor

from voice_nav_agent.agent_node import (
    _state_qos,
    _voice_turn_qos,
    AgentNode,
)
from voice_nav_interfaces.action import ExecuteMission, Speak
from voice_nav_interfaces.msg import MissionState, VoiceTurn
from voice_nav_interfaces.srv import StopMission


class FakeEndpoints:
    """Test-only ROS endpoints for the formal VoiceTurn path."""

    def __init__(self):
        self.node = rclpy.create_node('agent_fake_endpoints')
        self.mission_goals = []
        self.speak_goals = []
        self.stop_requests = []
        self.mission_event = threading.Event()
        self.speak_event = threading.Event()
        self.stop_event = threading.Event()
        self.state_matched = threading.Event()
        self.state_publisher = self.node.create_publisher(
            MissionState,
            '/mission/state',
            _state_qos(),
            event_callbacks=PublisherEventCallbacks(
                matched=lambda _event: self.state_matched.set(),
            ),
        )
        self.mission_server = ActionServer(
            self.node,
            ExecuteMission,
            '/mission/execute',
            execute_callback=self._execute_mission,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
        )
        self.speak_server = ActionServer(
            self.node,
            Speak,
            '/voice/speak',
            execute_callback=self._execute_speak,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
        )
        self.stop_service = self.node.create_service(
            StopMission,
            '/mission/stop',
            self._stop,
        )

    @staticmethod
    def _accept_goal(_request):
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute_mission(self, goal_handle):
        self.mission_goals.append(goal_handle.request)
        self.mission_event.set()
        goal_handle.succeed()
        result = ExecuteMission.Result()
        result.code = ExecuteMission.Result.SUCCEEDED
        result.failed_step = -1
        result.detail = 'ok'
        return result

    def _execute_speak(self, goal_handle):
        self.speak_goals.append(goal_handle.request)
        self.speak_event.set()
        goal_handle.succeed()
        result = Speak.Result()
        result.code = Speak.Result.COMPLETED
        result.detail = 'ok'
        return result

    def _stop(self, request, response):
        self.stop_requests.append(request)
        self.stop_event.set()
        response.code = StopMission.Response.APPLIED
        response.motion_inhibited = True
        response.runtime_instance_id = 'runtime-a'
        response.admission_epoch = 8
        response.detail = 'ok'
        return response

    def destroy(self):
        """Destroy only test endpoints and never install them."""
        self.mission_server.destroy()
        self.speak_server.destroy()
        self.stop_service.destroy()
        self.node.destroy_node()


@pytest.fixture(scope='module')
def ros_context():
    """Initialize a ROS context for the formal endpoint test."""
    rclpy.init(args=[])
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _state_message():
    message = MissionState()
    message.runtime_instance_id = 'runtime-a'
    message.admission_epoch = 7
    message.operating_mode = MissionState.NAVIGATION
    message.availability = MissionState.AVAILABLE
    message.gate_state = MissionState.GATE_INHIBITED
    message.active_step = 2**32 - 1
    message.supported_step_mask = 0b1111
    message.max_steps = 3
    message.named_place_ids = ['lobby']
    return message


def _voice_turn(sequence, text, *, kind=VoiceTurn.COMMAND, turn_id=None):
    message = VoiceTurn()
    message.voice_instance_id = 'voice-formal'
    message.voice_seq = sequence
    message.session_id = 'session-formal'
    message.turn_id = turn_id or f'turn-{sequence}'
    message.kind = kind
    message.text = text
    message.confidence = 1.0
    return message


def test_formal_voice_turn_reaches_fake_mission_speak_and_stop(
    ros_context,
):
    del ros_context
    agent = AgentNode(agent_instance_id='e' * 32)
    fake = FakeEndpoints()
    driver = rclpy.create_node('agent_voice_test_driver')
    turn_matched = threading.Event()
    turn_publisher = driver.create_publisher(
        VoiceTurn,
        '/voice/turn',
        _voice_turn_qos(),
        event_callbacks=PublisherEventCallbacks(
            matched=lambda _event: turn_matched.set(),
        ),
    )
    executor = SingleThreadedExecutor()
    executor.add_node(agent)
    executor.add_node(fake.node)
    executor.add_node(driver)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    mission_probe = ActionClient(
        driver,
        ExecuteMission,
        '/mission/execute',
    )
    try:
        assert fake.state_matched.wait(5.0)
        publishers = agent._compatible_state_publishers()
        assert len(publishers) == 1
        generation = agent._state_subscription_generation
        epoch_gid = publishers[0].endpoint_gid
        agent._state_subscription_epoch_gid = epoch_gid
        agent._on_state_message(
            _state_message(),
            {
                'source_timestamp': 1,
                'received_timestamp': 1,
                'publication_sequence_number': 1,
                'reception_sequence_number': None,
            },
            generation,
            epoch_gid,
        )
        assert turn_matched.wait(5.0)
        assert mission_probe.wait_for_server(timeout_sec=5.0)
        assert agent._planning_snapshot(
            require_execute_ready=False
        ).runtime_instance_id == 'runtime-a'

        turn_publisher.publish(_voice_turn(1, '前进 1 米'))
        assert fake.mission_event.wait(5.0)
        assert fake.speak_event.wait(5.0)

        goal = fake.mission_goals[0]
        assert goal.source_instance_id == 'e' * 32
        assert goal.source_seq == 1
        assert goal.runtime_instance_id == 'runtime-a'
        assert goal.admission_epoch == 7
        assert goal.steps[0].distance_m == 1.0
        assert fake.speak_goals[0].text == '任务已完成。'
        assert fake.speak_goals[0].allow_barge_in

        stop = _voice_turn(
            2,
            '停止',
            kind=VoiceTurn.STOP,
            turn_id='stop-formal',
        )
        fake.stop_event.clear()
        fake.speak_event.clear()
        turn_publisher.publish(stop)
        assert fake.stop_event.wait(5.0)
        assert fake.speak_event.wait(5.0)
        request = fake.stop_requests[0]
        assert request.request_id == 'stop-formal'
        assert request.source_instance_id == 'voice-formal'
        assert request.source_seq == 2
        assert request.reason == 'voice_stop'
        assert fake.speak_goals[1].priority == Speak.Goal.URGENT
        assert fake.speak_goals[1].text == '已停止。'

        fake.stop_event.clear()
        turn_publisher.publish(
            _voice_turn(
                3,
                '停止',
                kind=VoiceTurn.STOP,
                turn_id='stop-formal-2',
            )
        )
        assert fake.stop_event.wait(5.0)
    finally:
        executor.shutdown()
        spin_thread.join(5.0)
        executor.remove_node(driver)
        executor.remove_node(fake.node)
        executor.remove_node(agent)
        mission_probe.destroy()
        driver.destroy_publisher(turn_publisher)
        driver.destroy_node()
        fake.destroy()
        agent.destroy_node()


def test_jazzy_state_callback_rebuilds_transient_local_epoch(
    ros_context,
    monkeypatch,
):
    """A Jazzy callback without publisher_gid recovers through a fresh epoch."""
    del ros_context
    agent = AgentNode(agent_instance_id='f' * 32)
    fake = FakeEndpoints()
    executor = SingleThreadedExecutor()
    executor.add_node(agent)
    executor.add_node(fake.node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    observed = threading.Event()
    original_observe = agent._observe_state

    def observe_state(message, message_info, generation, epoch_gid):
        original_observe(message, message_info, generation, epoch_gid)
        if agent._latest_state is not None:
            observed.set()

    monkeypatch.setattr(agent, '_observe_state', observe_state)
    try:
        assert fake.state_matched.wait(5.0)
        fake.state_publisher.publish(_state_message())
        assert observed.wait(5.0)
        assert agent._latest_state.runtime_instance_id == 'runtime-a'
    finally:
        executor.shutdown()
        spin_thread.join(5.0)
        executor.remove_node(fake.node)
        executor.remove_node(agent)
        fake.destroy()
        agent.destroy_node()
