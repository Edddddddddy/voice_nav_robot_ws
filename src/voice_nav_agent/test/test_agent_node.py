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

from types import SimpleNamespace

from action_msgs.srv import CancelGoal
import pytest
import rclpy
from voice_nav_agent.agent_node import (
    _publisher_signature,
    AgentNode,
)
from voice_nav_agent.core import (
    Availability,
    GateState,
    MissionState,
    OperatingMode,
)
from voice_nav_interfaces.action import ExecuteMission, Speak
from voice_nav_interfaces.msg import VoiceTurn
from voice_nav_interfaces.srv import StopMission


class ScriptedFuture:
    """A deterministic future that never sleeps to deliver a callback."""

    def __init__(self, result=None, *, done=True):
        self._result = result
        self._done = done
        self._callbacks = []

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def result(self):
        return self._result

    def resolve(self, result):
        self._result = result
        self._done = True
        callbacks = tuple(self._callbacks)
        self._callbacks.clear()
        for callback in callbacks:
            callback(self)


class FakeGoalHandle:
    """Scripted Action Goal handle with an independently released result."""

    def __init__(self, result=None, *, result_done=True):
        self.accepted = True
        self.cancel_calls = 0
        response = CancelGoal.Response()
        response.return_code = CancelGoal.Response.ERROR_NONE
        self.cancel_future = ScriptedFuture(response)
        self.result_future = ScriptedFuture(
            _wrapped_result(result), done=result_done
        )

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future

    def get_result_async(self):
        return self.result_future


class FakeActionClient:
    """Fake Action client with scripted acceptance and terminal outcomes."""

    def __init__(self, result=None, *, result_done=True):
        self.goals = []
        self.handles = []
        self.result = result
        self.result_done = result_done

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        handle = FakeGoalHandle(
            self.result,
            result_done=self.result_done,
        )
        self.handles.append(handle)
        return ScriptedFuture(handle)


class FakeStopClient:
    """Fake Stop service that exposes each request future to the test."""

    def __init__(self):
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = ScriptedFuture(None, done=False)
        self.requests.append(request)
        self.futures.append(future)
        return future


def _wrapped_result(code):
    result = ExecuteMission.Result()
    result.code = (
        ExecuteMission.Result.SUCCEEDED
        if code is None
        else code
    )
    return SimpleNamespace(result=result)


def _make_state():
    return MissionState(
        runtime_instance_id='runtime-a',
        admission_epoch=7,
        operating_mode=OperatingMode.NAVIGATION,
        availability=Availability.AVAILABLE,
        gate_state=GateState.GATE_INHIBITED,
        active_step=2**32 - 1,
        supported_step_mask=0b1111,
        max_steps=3,
        named_place_ids=('lobby',),
    )


def _make_turn(text, sequence=1, *, turn_id=None, kind=VoiceTurn.COMMAND):
    message = VoiceTurn()
    message.voice_instance_id = 'voice-a'
    message.voice_seq = sequence
    message.session_id = 'session-a'
    message.turn_id = turn_id or f'turn-{sequence}'
    message.kind = kind
    message.text = text
    message.confidence = 1.0
    return message


def _make_node(monkeypatch, *, mission_result=None, result_done=True):
    node = AgentNode(agent_instance_id='a' * 32)
    node._mission_client = FakeActionClient(
        mission_result,
        result_done=result_done,
    )
    node._speak_client = FakeActionClient()
    node._stop_client = FakeStopClient()
    info = SimpleNamespace(
        endpoint_gid=b'state-publisher',
        node_name='fake_runtime',
        node_namespace='/',
        topic_type='voice_nav_interfaces/msg/MissionState',
    )
    node._latest_state = _make_state()
    node._state_sample_signature = _publisher_signature(info)
    monkeypatch.setattr(
        node,
        '_compatible_state_publishers',
        lambda: [info],
    )
    return node


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialize one ROS context for the fake-port adapter tests."""
    rclpy.init(args=[])
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_rule_mission_maps_typed_goal_and_terminal_speech(monkeypatch):
    node = _make_node(monkeypatch)
    try:
        node._on_turn_message(_make_turn('前进 1 米'))

        assert len(node._mission_client.goals) == 1
        goal = node._mission_client.goals[0]
        assert goal.source_instance_id == 'a' * 32
        assert goal.source_seq == 1
        assert goal.runtime_instance_id == 'runtime-a'
        assert goal.admission_epoch == 7
        assert len(goal.steps) == 1
        assert goal.steps[0].distance_m == 1.0
        assert node._mission_slot is None
        assert node._speak_client.goals[-1].text == '任务已完成。'
    finally:
        node.destroy_node()


def test_pending_cancel_cancels_exact_goal_once(monkeypatch):
    node = _make_node(
        monkeypatch,
        mission_result=ExecuteMission.Result.CANCELED,
        result_done=False,
    )
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        handle = node._mission_client.handles[0]
        node._on_turn_message(_make_turn('取消任务', sequence=2))

        assert handle.cancel_calls == 1
        assert node._mission_slot is not None
        handle.result_future.resolve(
            _wrapped_result(ExecuteMission.Result.CANCELED)
        )
        assert node._mission_slot is None
        assert node._speak_client.goals[-1].text == '任务已取消。'
    finally:
        node.destroy_node()


def test_stop_reuses_voice_identity_and_never_cancels_mission(monkeypatch):
    node = _make_node(
        monkeypatch,
        mission_result=ExecuteMission.Result.SUCCEEDED,
        result_done=False,
    )
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        handle = node._mission_client.handles[0]
        stop = _make_turn(
            '停止',
            sequence=8,
            turn_id='stop-8',
            kind=VoiceTurn.STOP,
        )
        node._on_turn_message(stop)

        assert len(node._stop_client.requests) == 1
        request = node._stop_client.requests[0]
        assert request.request_id == 'stop-8'
        assert request.source_instance_id == 'voice-a'
        assert request.source_seq == 8
        assert request.reason == 'voice_stop'
        assert handle.cancel_calls == 0

        response = StopMission.Response()
        response.code = StopMission.Response.APPLIED
        response.motion_inhibited = True
        node._stop_client.futures[0].resolve(response)
        assert node._speak_client.goals[-1].priority == Speak.Goal.URGENT
        assert node._speak_client.goals[-1].text == '已停止。'

        node._on_turn_message(stop)
        assert len(node._stop_client.requests) == 2
        assert handle.cancel_calls == 0
    finally:
        node.destroy_node()


def test_cancel_without_local_handle_only_speaks_reply(monkeypatch):
    node = _make_node(monkeypatch)
    try:
        node._on_turn_message(_make_turn('取消任务'))

        assert node._mission_client.goals == []
        assert node._mission_slot is None
        assert node._speak_client.goals[-1].text == '没有可取消的本地任务。'
    finally:
        node.destroy_node()


def test_new_turn_does_not_cancel_existing_mission(monkeypatch):
    node = _make_node(
        monkeypatch,
        mission_result=ExecuteMission.Result.SUCCEEDED,
        result_done=False,
    )
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        handle = node._mission_client.handles[0]
        node._on_turn_message(_make_turn('前进 0.5 米', sequence=2))

        assert len(node._mission_client.goals) == 1
        assert handle.cancel_calls == 0
        assert node._speak_client.goals[-1].text == '本地任务正在处理。'
    finally:
        node.destroy_node()
