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

from dataclasses import replace
from types import SimpleNamespace

from action_msgs.srv import CancelGoal
import pytest
import rclpy
from voice_nav_agent._response_session import _ProviderResponse, _ToolCall
from voice_nav_agent.agent_node import (
    _publisher_signature,
    _SerialSeam,
    AgentNode,
)
from voice_nav_agent.core import (
    Availability,
    GateState,
    MissionState,
    MissionStep,
    OperatingMode,
)
from voice_nav_interfaces.action import ExecuteMission, Speak
from voice_nav_interfaces.msg import VoiceTurn
from voice_nav_interfaces.srv import StopMission


class ScriptedFuture:
    """A pending future released only by an explicit scripted outcome."""

    def __init__(self):
        self._result = None
        self._error = None
        self._done = False
        self._callbacks = []

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def result(self):
        if not self._done:
            raise RuntimeError('scripted future is still pending')
        if self._error is not None:
            raise self._error
        return self._result

    def resolve(self, result):
        self._finish(result=result)

    def reject(self, error=None):
        self._finish(error=error or RuntimeError('scripted rejection'))

    def timeout(self):
        self.reject(TimeoutError('scripted timeout'))

    def _finish(self, *, result=None, error=None):
        if self._done:
            raise RuntimeError('scripted future already completed')
        self._result = result
        self._error = error
        self._done = True
        callbacks = tuple(self._callbacks)
        self._callbacks.clear()
        for callback in callbacks:
            callback(self)


class FakeGoalHandle:
    """Scripted Action Goal handle with independently released futures."""

    def __init__(self):
        self.accepted = True
        self.cancel_calls = 0
        self.cancel_future = ScriptedFuture()
        self.result_future = ScriptedFuture()

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future

    def get_result_async(self):
        return self.result_future


class FakeActionClient:
    """Fake Action client with explicit acceptance and terminal futures."""

    def __init__(self, *, ready=True):
        self.goals = []
        self.handles = []
        self.send_futures = []
        self._ready = ready

    def server_is_ready(self):
        return self._ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        handle = FakeGoalHandle()
        self.handles.append(handle)
        future = ScriptedFuture()
        self.send_futures.append(future)
        return future


class FakeStopClient:
    """Fake Stop service that exposes each request future to the test."""

    def __init__(self, *, ready=True):
        self.requests = []
        self.futures = []
        self._ready = ready

    def service_is_ready(self):
        return self._ready

    def call_async(self, request):
        future = ScriptedFuture()
        self.requests.append(request)
        self.futures.append(future)
        return future


class FakeResponseProvider:
    """Records ResponseSession work without opening an HTTP connection."""

    def __init__(self):
        self.submissions = []
        self.invalidations = []
        self.shutdown_called = False
        self.on_shutdown = None

    def submit(self, request):
        self.submissions.append(request)

    def invalidate(self, generation):
        self.invalidations.append(generation)

    def shutdown(self):
        self.shutdown_called = True
        if self.on_shutdown is not None:
            self.on_shutdown()


class ScriptedTimer:
    """A one-shot deadline that fires only when the test releases it."""

    def __init__(self, callback):
        self._callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True

    def timeout(self):
        if not self.cancelled:
            self.cancelled = True
            self._callback()


def _wrapped_result(code):
    result = ExecuteMission.Result()
    result.code = (
        ExecuteMission.Result.SUCCEEDED
        if code is None
        else code
    )
    return SimpleNamespace(result=result)


def _cancel_response(return_code=CancelGoal.Response.ERROR_NONE):
    response = CancelGoal.Response()
    response.return_code = return_code
    return response


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


def _make_state_message(runtime_instance_id, admission_epoch=7):
    """Build one state sample for a scripted publisher callback."""
    return SimpleNamespace(
        runtime_instance_id=runtime_instance_id,
        admission_epoch=admission_epoch,
        operating_mode=OperatingMode.NAVIGATION,
        availability=Availability.AVAILABLE,
        gate_state=GateState.GATE_INHIBITED,
        active_step=2**32 - 1,
        supported_step_mask=0b1111,
        max_steps=3,
        named_place_ids=['lobby'],
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


def _make_node(monkeypatch):
    node = AgentNode(agent_instance_id='a' * 32)
    node._mission_client = FakeActionClient()
    node._speak_client = FakeActionClient()
    node._stop_client = FakeStopClient()
    info = SimpleNamespace(
        endpoint_gid=b'state-publisher',
        node_name='fake_runtime',
        node_namespace='/',
        topic_type='voice_nav_interfaces/msg/MissionState',
    )
    node._latest_state = _make_state()
    node._state_subscription_epoch_gid = info.endpoint_gid
    node._state_sample_signature = _publisher_signature(info)
    monkeypatch.setattr(
        node,
        'get_publishers_info_by_topic',
        lambda _topic: [info],
    )
    return node


def _replace_response_provider(node):
    """Install an explicit fake without retaining the real transport worker."""
    original_provider = node._response_provider
    provider = FakeResponseProvider()
    node._response_provider = provider
    node._response_session._provider = provider
    original_provider.shutdown()
    return provider


def _response_mission_call():
    return _ProviderResponse(
        kind='tool',
        tool_calls=(
            _ToolCall(
                'propose_mission',
                {
                    'kind': 'mission',
                    'steps': [
                        {'kind': 'navigate_to', 'target_id': 'lobby'},
                    ],
                },
            ),
        ),
    )


def _script_deadlines(monkeypatch, node):
    """Replace only the ROS clock/timer seam with explicit deadlines."""
    timers = []

    def make_timer(_seconds, callback):
        timer = ScriptedTimer(callback)
        timers.append(timer)
        return timer

    monkeypatch.setattr(node, '_one_shot_timer', make_timer)
    return timers


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
        node._mission_client.send_futures[0].resolve(
            node._mission_client.handles[0]
        )
        node._mission_client.handles[0].result_future.resolve(
            _wrapped_result(ExecuteMission.Result.SUCCEEDED)
        )
        assert node._mission_slot is None
        assert node._speak_client.goals[-1].text == '任务已完成。'
    finally:
        node.destroy_node()


def test_serial_seam_returns_llm_admission_result():
    seam = _SerialSeam()
    observed = []

    accepted = seam.invoke(lambda request: observed.append(request) or True, 'request')

    assert accepted is True
    assert observed == ['request']


@pytest.mark.parametrize(
    ('response', 'expected_text'),
    [
        (_ProviderResponse(kind='clarify', text='请说明目标地点。'), '请说明目标地点。'),
        (_ProviderResponse(kind='reply', text='该请求不能执行。'), '该请求不能执行。'),
    ],
)
def test_response_clarify_or_reply_speaks_once(
    monkeypatch, response, expected_text
):
    node = _make_node(monkeypatch)
    provider = _replace_response_provider(node)
    try:
        node._on_turn_message(_make_turn('请沿着大厅右侧绕过去'))
        request = provider.submissions[-1]
        node._on_response_provider_response(request, response)
        node._on_response_provider_response(request, response)

        assert len(node._speak_client.goals) == 1
        goal = node._speak_client.goals[0]
        assert goal.priority == Speak.Goal.NORMAL
        assert goal.text == expected_text
        assert goal.session_id == 'session-a'
        assert goal.turn_id == 'turn-1'
    finally:
        node.destroy_node()


def test_response_failure_speaks_one_bounded_normal_reply(monkeypatch):
    node = _make_node(monkeypatch)
    provider = _replace_response_provider(node)
    try:
        node._on_turn_message(_make_turn('请沿着大厅右侧绕过去'))
        request = provider.submissions[-1]
        node._on_response_provider_failure(request)
        node._on_response_provider_failure(request)

        assert len(node._speak_client.goals) == 1
        goal = node._speak_client.goals[0]
        assert goal.priority == Speak.Goal.NORMAL
        assert goal.text == '当前无法处理该导航请求。'
        assert len(goal.text) <= 512
    finally:
        node.destroy_node()


@pytest.mark.parametrize(
    'state_changes',
    [
        {'runtime_instance_id': 'runtime-b'},
        {'admission_epoch': 8},
        {'operating_mode': OperatingMode.MAPPING},
        {'supported_step_mask': 0b0011},
        {'named_place_ids': ('charging',)},
        {'availability': Availability.BUSY},
        {'gate_state': GateState.GATE_ARMED},
    ],
)
def test_stale_response_mission_cannot_side_effect_after_runtime_changes(
    monkeypatch, state_changes
):
    node = _make_node(monkeypatch)
    provider = _replace_response_provider(node)
    try:
        node._on_turn_message(_make_turn('请沿着大厅右侧绕过去'))
        request = provider.submissions[-1]
        node._latest_state = replace(_make_state(), **state_changes)

        node._on_response_provider_response(request, _response_mission_call())

        assert node._mission_client.goals == []
        assert node._speak_client.goals == []
    finally:
        node.destroy_node()


def test_response_mission_revalidates_and_sends_a_typed_goal(monkeypatch):
    node = _make_node(monkeypatch)
    provider = _replace_response_provider(node)
    try:
        node._on_turn_message(_make_turn('请沿着大厅右侧绕过去'))
        request = provider.submissions[-1]
        node._on_response_provider_response(request, _response_mission_call())

        assert len(node._mission_client.goals) == 1
        goal = node._mission_client.goals[0]
        assert goal.source_instance_id == 'a' * 32
        assert goal.source_seq == request.token.source_seq
        assert goal.runtime_instance_id == 'runtime-a'
        assert goal.admission_epoch == 7
        assert len(goal.steps) == 1
        assert goal.steps[0].kind == MissionStep.NAVIGATE_TO
        assert goal.steps[0].target_id == 'lobby'
    finally:
        node.destroy_node()


def test_response_clarify_snapshot_then_mission_uses_one_speak_and_slot(
    monkeypatch,
):
    """The installed adapter carries one clarification into a two-round Mission."""
    node = _make_node(monkeypatch)
    provider = _replace_response_provider(node)
    try:
        node._on_turn_message(_make_turn('请沿着大厅右侧绕过去', sequence=1))
        first = provider.submissions[-1]
        node._on_response_provider_response(
            first, _ProviderResponse(kind='clarify', text='请说明目的地。')
        )
        assert len(node._speak_client.goals) == 1
        assert node._speak_client.goals[0].text == '请说明目的地。'

        node._on_turn_message(_make_turn('请继续帮我到那里', sequence=2))
        second = provider.submissions[-1]
        assert second.clarification == '请说明目的地。'
        node._on_response_provider_response(
            second,
            _ProviderResponse(
                kind='tool',
                tool_calls=(_ToolCall('read_runtime_snapshot', {}),),
            ),
        )
        continuation = provider.submissions[-1]
        assert continuation.round == 2
        assert continuation.snapshot_output.name == 'read_runtime_snapshot'

        node._on_response_provider_response(
            continuation, _response_mission_call()
        )

        assert len(node._mission_client.goals) == 1
        assert len(node._speak_client.goals) == 1
        goal = node._mission_client.goals[0]
        assert goal.source_seq == continuation.token.source_seq
        assert goal.runtime_instance_id == 'runtime-a'
        assert goal.admission_epoch == 7
    finally:
        node.destroy_node()


@pytest.mark.parametrize('late_delivery', ['result', 'failure'])
def test_destroy_node_revokes_response_before_provider_shutdown(
    monkeypatch, late_delivery
):
    """A late provider callback during teardown cannot start ROS side effects."""
    node = _make_node(monkeypatch)
    provider = _replace_response_provider(node)
    destroyed = False
    try:
        node._on_turn_message(_make_turn('请沿着大厅右侧绕过去'))
        request = provider.submissions[-1]

        if late_delivery == 'result':
            provider.on_shutdown = lambda: node._on_response_provider_response(
                request, _response_mission_call()
            )
        else:
            provider.on_shutdown = lambda: node._on_response_provider_failure(
                request
            )

        node.destroy_node()
        destroyed = True

        assert provider.shutdown_called is True
        assert node._mission_client.goals == []
        assert node._speak_client.goals == []
    finally:
        if not destroyed:
            node.destroy_node()


def test_rule_and_stop_never_submit_response_http(monkeypatch):
    node = _make_node(monkeypatch)
    provider = _replace_response_provider(node)
    try:
        node._on_turn_message(_make_turn('前进 1 米', sequence=1))
        node._on_turn_message(
            _make_turn('停止', sequence=2, kind=VoiceTurn.STOP)
        )
        node._on_turn_message(_make_turn('前进', sequence=3))

        assert provider.submissions == []
        assert len(provider.invalidations) == 3
    finally:
        node.destroy_node()


def test_higher_invalid_turn_fences_response_without_opening_provider(
    monkeypatch,
):
    """A malformed higher sequence revokes old Response work exactly once."""
    node = _make_node(monkeypatch)
    provider = _replace_response_provider(node)
    try:
        node._on_turn_message(_make_turn('请沿着大厅右侧绕过去'))
        old_request = provider.submissions[-1]

        malformed = SimpleNamespace(
            voice_instance_id='voice-a',
            voice_seq=2,
            session_id='session-a',
            turn_id='malformed-2',
            kind=VoiceTurn.COMMAND,
            text=object(),
            confidence=1.0,
            during_playback=False,
        )
        node._on_turn_message(malformed)
        node._on_turn_message(malformed)
        stale = SimpleNamespace(**{
            **malformed.__dict__,
            'voice_seq': 1,
            'turn_id': 'malformed-1',
        })
        node._on_turn_message(stale)
        node._on_response_provider_response(old_request, _response_mission_call())

        assert len(provider.submissions) == 1
        assert provider.invalidations == [1, 2]
        assert node._mission_client.goals == []
        assert node._speak_client.goals == []
        assert node._mission_slot is None
    finally:
        node.destroy_node()


def test_state_sample_requires_exact_publisher_gid_after_restart(monkeypatch):
    """An old publisher sample cannot refresh a replacement publisher token."""
    node = AgentNode(agent_instance_id='a' * 32)
    publisher_a = SimpleNamespace(
        endpoint_gid=b'publisher-a',
        node_name='runtime_a',
        node_namespace='/',
        topic_type='voice_nav_interfaces/msg/MissionState',
    )
    publisher_b = SimpleNamespace(
        endpoint_gid=b'publisher-b',
        node_name='runtime_b',
        node_namespace='/',
        topic_type='voice_nav_interfaces/msg/MissionState',
    )
    publishers = [publisher_a]
    monkeypatch.setattr(
        node,
        'get_publishers_info_by_topic',
        lambda _topic: list(publishers),
    )
    try:
        generation_a = node._state_subscription_generation
        node._state_subscription_epoch_gid = b'publisher-a'
        node._on_state_message(
            _make_state_message('runtime_a'),
            SimpleNamespace(publisher_gid=b'publisher-a'),
            generation_a,
            b'publisher-a',
        )
        assert node._planning_snapshot(require_execute_ready=False).runtime_instance_id == (
            'runtime_a'
        )

        publishers[:] = [publisher_b]
        node._on_state_message(
            _make_state_message('runtime_a'),
            SimpleNamespace(publisher_gid=b'publisher-a'),
            generation_a,
            b'publisher-a',
        )
        assert node._planning_snapshot(require_execute_ready=False) is None
        node._on_state_rebuild_event()
        generation_b = node._state_subscription_generation
        assert generation_b != generation_a
        assert node._planning_snapshot(require_execute_ready=False) is None

        node._on_state_message(
            _make_state_message('runtime_b'),
            {
                'publisher_gid': {
                    'implementation_identifier': 'rmw_fastrtps_cpp',
                    'data': b'publisher-b',
                }
            },
            generation_b,
            b'publisher-b',
        )
        assert node._planning_snapshot(require_execute_ready=False).runtime_instance_id == (
            'runtime_b'
        )
        node._on_state_message(
            _make_state_message('runtime_a'),
            SimpleNamespace(publisher_gid=b'publisher-a'),
            generation_a,
            b'publisher-a',
        )
        assert node._planning_snapshot(require_execute_ready=False).runtime_instance_id == (
            'runtime_b'
        )
    finally:
        node.destroy_node()


def test_state_ingress_replays_b_and_fences_late_a_before_mission(monkeypatch):
    """Inject B once after rebuild and fence a late A generation callback."""
    node = AgentNode(agent_instance_id='c' * 32)
    node._mission_client = FakeActionClient()
    node._speak_client = FakeActionClient()
    node._stop_client = FakeStopClient()
    publisher_a = SimpleNamespace(
        endpoint_gid=b'publisher-a',
        node_name='runtime_a',
        node_namespace='/',
        topic_type='voice_nav_interfaces/msg/MissionState',
    )
    publisher_b = SimpleNamespace(
        endpoint_gid=b'publisher-b',
        node_name='runtime_b',
        node_namespace='/',
        topic_type='voice_nav_interfaces/msg/MissionState',
    )
    publishers = [publisher_a]
    monkeypatch.setattr(
        node,
        'get_publishers_info_by_topic',
        lambda _topic: list(publishers),
    )
    jazzy_info = {
        'source_timestamp': 1,
        'received_timestamp': 1,
        'publication_sequence_number': 1,
        'reception_sequence_number': None,
    }
    try:
        node._on_state_rebuild_event()
        generation_a = node._state_subscription_generation
        node._on_state_message(
            _make_state_message('runtime_a', admission_epoch=11),
            jazzy_info,
            generation_a,
            b'publisher-a',
        )
        assert node._planning_snapshot(
            require_execute_ready=False
        ).runtime_instance_id == 'runtime_a'

        publishers[:] = [publisher_b]
        node._on_state_message(
            _make_state_message('runtime_a', admission_epoch=11),
            SimpleNamespace(publisher_gid=b'publisher-a'),
            generation_a,
            b'publisher-a',
        )
        assert node._planning_snapshot(require_execute_ready=False) is None

        node._on_state_rebuild_event()
        generation_b = node._state_subscription_generation
        assert generation_b != generation_a
        node._on_state_message(
            _make_state_message('runtime_b', admission_epoch=22),
            jazzy_info,
            generation_b,
            b'publisher-b',
        )
        snapshot = node._planning_snapshot(require_execute_ready=False)
        assert snapshot.runtime_instance_id == 'runtime_b'
        assert snapshot.admission_epoch == 22

        node._on_turn_message(_make_turn('前进 1 米', sequence=9))
        assert len(node._mission_client.goals) == 1
        assert node._mission_client.goals[0].runtime_instance_id == 'runtime_b'
        assert node._mission_client.goals[0].admission_epoch == 22

        node._on_state_message(
            _make_state_message('runtime_a', admission_epoch=11),
            SimpleNamespace(publisher_gid=b'publisher-a'),
            generation_a,
            b'publisher-a',
        )
        snapshot = node._planning_snapshot(require_execute_ready=False)
        assert snapshot.runtime_instance_id == 'runtime_b'
        assert snapshot.admission_epoch == 22
        assert len(node._mission_client.goals) == 1
    finally:
        node.destroy_node()


def test_state_epoch_proof_accepts_jazzy_metadata_and_rebuilds_after_restart(
    monkeypatch,
):
    """Jazzy metadata uses the subscription epoch to fence a restarted publisher."""
    node = AgentNode(agent_instance_id='b' * 32)
    publisher_a = SimpleNamespace(
        endpoint_gid=b'publisher-a',
        node_name='runtime_a',
        node_namespace='/',
        topic_type='voice_nav_interfaces/msg/MissionState',
    )
    publisher_b = SimpleNamespace(
        endpoint_gid=b'publisher-b',
        node_name='runtime_b',
        node_namespace='/',
        topic_type='voice_nav_interfaces/msg/MissionState',
    )
    publishers = [publisher_a]
    monkeypatch.setattr(
        node,
        'get_publishers_info_by_topic',
        lambda _topic: list(publishers),
    )
    jazzy_info = {
        'source_timestamp': 1,
        'received_timestamp': 1,
        'publication_sequence_number': 1,
        'reception_sequence_number': None,
    }
    try:
        generation_a = node._state_subscription_generation
        node._state_subscription_epoch_gid = b'publisher-a'
        node._on_state_message(
            _make_state_message('runtime_a'),
            jazzy_info,
            generation_a,
            b'publisher-a',
        )
        assert node._planning_snapshot(require_execute_ready=False).runtime_instance_id == (
            'runtime_a'
        )

        publishers[:] = [publisher_a, publisher_b]
        node._on_state_message(
            _make_state_message('runtime_a'),
            jazzy_info,
            generation_a,
            b'publisher-a',
        )
        assert node._planning_snapshot(require_execute_ready=False) is None
        publishers[:] = []
        node._on_state_message(
            _make_state_message('runtime_a'),
            jazzy_info,
            generation_a,
            b'publisher-a',
        )
        assert node._planning_snapshot(require_execute_ready=False) is None
        publishers[:] = [publisher_b]
        node._on_state_rebuild_event()
        generation_b = node._state_subscription_generation
        assert node._planning_snapshot(require_execute_ready=False) is None

        node._on_state_message(
            _make_state_message('runtime_a'),
            jazzy_info,
            generation_a,
            b'publisher-a',
        )
        assert node._planning_snapshot(require_execute_ready=False) is None
        node._on_state_message(
            _make_state_message('runtime_b'),
            jazzy_info,
            generation_b,
            b'publisher-b',
        )
        assert node._planning_snapshot(require_execute_ready=False).runtime_instance_id == (
            'runtime_b'
        )
    finally:
        node.destroy_node()


def test_turn_reconciles_empty_state_epoch_before_fail_closed(monkeypatch):
    """A legal Turn reconciles a unique publisher before returning no state."""
    monkeypatch.setattr(
        AgentNode,
        '_capture_state_epoch_gid',
        lambda _node: None,
    )
    node = AgentNode(agent_instance_id='d' * 32)
    node._mission_client = FakeActionClient()
    node._speak_client = FakeActionClient()
    node._stop_client = FakeStopClient()
    publisher = SimpleNamespace(
        endpoint_gid=b'publisher-b',
        node_name='runtime_b',
        node_namespace='/',
        topic_type='voice_nav_interfaces/msg/MissionState',
    )
    monkeypatch.setattr(
        node,
        'get_publishers_info_by_topic',
        lambda _topic: [publisher],
    )
    rebuilds = []
    monkeypatch.setattr(
        node,
        '_schedule_state_subscription_rebuild',
        lambda: rebuilds.append(True),
    )
    try:
        node._on_turn_message(_make_turn('\u524d\u8fdb 1 \u7c73'))

        assert node._mission_client.goals == []
        assert rebuilds == [True]
    finally:
        node.destroy_node()


def test_pending_cancel_cancels_exact_goal_once(monkeypatch):
    node = _make_node(monkeypatch)
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        handle = node._mission_client.handles[0]
        node._on_turn_message(_make_turn('取消任务', sequence=2))

        assert handle.cancel_calls == 0
        assert node._mission_slot is not None
        node._mission_client.send_futures[0].resolve(handle)
        assert handle.cancel_calls == 1
        response = CancelGoal.Response()
        response.return_code = CancelGoal.Response.ERROR_NONE
        handle.cancel_future.resolve(response)
        handle.result_future.resolve(
            _wrapped_result(ExecuteMission.Result.CANCELED)
        )
        assert node._mission_slot is None
        assert node._speak_client.goals[-1].text == '任务已取消。'
    finally:
        node.destroy_node()


def test_repeated_cancel_rebinds_terminal_correlation_without_extra_cancel(
    monkeypatch,
):
    """The latest Cancel turn owns one real terminal reply."""
    node = _make_node(monkeypatch)
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        handle = node._mission_client.handles[0]
        node._mission_client.send_futures[0].resolve(handle)

        node._on_turn_message(_make_turn('取消任务', sequence=2))
        node._on_turn_message(_make_turn('取消任务', sequence=3))

        assert handle.cancel_calls == 1
        assert node._speak_client.goals == []

        response = CancelGoal.Response()
        response.return_code = CancelGoal.Response.ERROR_NONE
        handle.cancel_future.resolve(response)
        handle.result_future.resolve(
            _wrapped_result(ExecuteMission.Result.CANCELED)
        )

        assert [goal.text for goal in node._speak_client.goals] == [
            '任务已取消。'
        ]
        assert node._mission_slot is None
    finally:
        node.destroy_node()


@pytest.mark.parametrize(
    ('terminal_code', 'expected_text'),
    [
        (ExecuteMission.Result.CANCELED, '任务已取消。'),
        (ExecuteMission.Result.SUCCEEDED, '任务已完成。'),
        (ExecuteMission.Result.EXECUTION_FAILED, '任务执行失败。'),
    ],
)
@pytest.mark.parametrize('accept_before_cancel', [False, True])
def test_repeated_cancel_keeps_first_real_terminal_code(
    monkeypatch,
    terminal_code,
    expected_text,
    accept_before_cancel,
):
    """SEND_PENDING and ACTIVE share one cancel and one code-based reply."""
    node = _make_node(monkeypatch)
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        client = node._mission_client
        handle = client.handles[0]
        if accept_before_cancel:
            client.send_futures[0].resolve(handle)

        node._on_turn_message(_make_turn('取消任务', sequence=2))
        node._on_turn_message(_make_turn('取消任务', sequence=3))
        if not accept_before_cancel:
            client.send_futures[0].resolve(handle)

        assert handle.cancel_calls == 1
        assert node._speak_client.goals == []
        cancel_response = CancelGoal.Response()
        cancel_response.return_code = CancelGoal.Response.ERROR_NONE
        handle.cancel_future.resolve(cancel_response)
        handle.result_future.resolve(_wrapped_result(terminal_code))

        assert [goal.text for goal in node._speak_client.goals] == [
            expected_text
        ]
        assert node._mission_slot is None
    finally:
        node.destroy_node()


def test_stop_reuses_voice_identity_and_never_cancels_mission(monkeypatch):
    node = _make_node(monkeypatch)
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        handle = node._mission_client.handles[0]
        node._mission_client.send_futures[0].resolve(handle)
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
    node = _make_node(monkeypatch)
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        handle = node._mission_client.handles[0]
        node._mission_client.send_futures[0].resolve(handle)
        node._on_turn_message(_make_turn('前进 0.5 米', sequence=2))

        assert len(node._mission_client.goals) == 1
        assert handle.cancel_calls == 0
        assert node._speak_client.goals[-1].text == '本地任务正在处理。'
    finally:
        node.destroy_node()


def test_mission_send_timeout_cancels_late_accepted_handle(monkeypatch):
    node = _make_node(monkeypatch)
    timers = _script_deadlines(monkeypatch, node)
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        client = node._mission_client
        handle = client.handles[0]
        timers[0].timeout()

        assert node._mission_slot is None
        assert [goal.text for goal in node._speak_client.goals] == [
            '任务提交未确认。'
        ]

        client.send_futures[0].resolve(handle)
        assert handle.cancel_calls == 1
        assert node._mission_slot is None
        assert [goal.text for goal in node._speak_client.goals] == [
            '任务提交未确认。'
        ]
    finally:
        node.destroy_node()


@pytest.mark.parametrize('outcome', ['reject', 'unaccepted'])
def test_mission_goal_send_failure_clears_slot(monkeypatch, outcome):
    node = _make_node(monkeypatch)
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        future = node._mission_client.send_futures[0]
        if outcome == 'reject':
            future.reject(RuntimeError('send failed'))
        else:
            future.resolve(SimpleNamespace(accepted=False))

        assert node._mission_slot is None
        assert [goal.text for goal in node._speak_client.goals] == [
            '任务提交失败。'
        ]
    finally:
        node.destroy_node()


@pytest.mark.parametrize(
    ('outcome', 'expected_texts'),
    [
        ('cancel_reject', ['取消请求未确认。', '任务已完成。']),
        ('cancel_timeout', ['取消请求未确认。', '任务已完成。']),
        ('result_first', ['任务已完成。']),
        ('cancel_success', ['任务已取消。']),
    ],
)
def test_cancel_response_and_result_races_use_real_terminal_code(
    monkeypatch,
    outcome,
    expected_texts,
):
    node = _make_node(monkeypatch)
    timers = _script_deadlines(monkeypatch, node)
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        client = node._mission_client
        handle = client.handles[0]
        client.send_futures[0].resolve(handle)
        node._on_turn_message(_make_turn('取消任务', sequence=2))

        if outcome == 'cancel_reject':
            handle.cancel_future.reject(RuntimeError('cancel rejected'))
            handle.result_future.resolve(
                _wrapped_result(ExecuteMission.Result.SUCCEEDED)
            )
        elif outcome == 'cancel_timeout':
            timers[-1].timeout()
            handle.result_future.resolve(
                _wrapped_result(ExecuteMission.Result.SUCCEEDED)
            )
        elif outcome == 'result_first':
            handle.result_future.resolve(
                _wrapped_result(ExecuteMission.Result.SUCCEEDED)
            )
            handle.cancel_future.resolve(_cancel_response())
        else:
            handle.cancel_future.resolve(_cancel_response())
            handle.result_future.resolve(
                _wrapped_result(ExecuteMission.Result.CANCELED)
            )

        assert [goal.text for goal in node._speak_client.goals] == (
            expected_texts
        )
        assert node._mission_slot is None
    finally:
        node.destroy_node()


@pytest.mark.parametrize(
    ('code', 'inhibited', 'expected_text'),
    [
        (StopMission.Response.APPLIED, True, '已停止。'),
        (StopMission.Response.DUPLICATE, True, '已停止。'),
        (StopMission.Response.APPLIED, False, '停止请求未确认。'),
        (StopMission.Response.SAFETY_FAULT, True, '停止请求未确认。'),
    ],
)
def test_stop_response_matrix_is_urgent_and_code_bounded(
    monkeypatch,
    code,
    inhibited,
    expected_text,
):
    node = _make_node(monkeypatch)
    try:
        node._on_turn_message(_make_turn('停止', kind=VoiceTurn.STOP))
        response = StopMission.Response()
        response.code = code
        response.motion_inhibited = inhibited
        node._stop_client.futures[0].resolve(response)

        assert node._speak_client.goals[-1].priority == Speak.Goal.URGENT
        assert node._speak_client.goals[-1].text == expected_text
    finally:
        node.destroy_node()


def test_stop_timeout_and_stale_response_are_fenced(monkeypatch):
    node = _make_node(monkeypatch)
    timers = _script_deadlines(monkeypatch, node)
    try:
        first = _make_turn(
            '停止', sequence=1, turn_id='stop-1', kind=VoiceTurn.STOP
        )
        second = _make_turn(
            '停止', sequence=2, turn_id='stop-2', kind=VoiceTurn.STOP
        )
        node._on_turn_message(first)
        first_future = node._stop_client.futures[0]
        node._on_turn_message(second)
        second_future = node._stop_client.futures[1]

        first_response = StopMission.Response()
        first_response.code = StopMission.Response.APPLIED
        first_response.motion_inhibited = True
        first_future.resolve(first_response)
        assert node._speak_client.goals == []

        timers[-1].timeout()
        second_response = StopMission.Response()
        second_response.code = StopMission.Response.DUPLICATE
        second_response.motion_inhibited = True
        second_future.resolve(second_response)

        assert [goal.text for goal in node._speak_client.goals] == [
            '停止请求未确认。'
        ]
    finally:
        node.destroy_node()


def test_speak_late_acceptance_is_canceled_after_timeout(monkeypatch):
    node = _make_node(monkeypatch)
    timers = _script_deadlines(monkeypatch, node)
    try:
        node._speak_text(
            '稍后回答',
            Speak.Goal.NORMAL,
            'session-a',
            'turn-a',
            1,
        )
        future = node._speak_client.send_futures[0]
        handle = node._speak_client.handles[0]
        timers[0].timeout()
        future.resolve(handle)

        assert node._speak_operation is None
        assert handle.cancel_calls == 1
    finally:
        node.destroy_node()


def test_speak_rechecks_action_server_once_before_dropping(monkeypatch):
    node = _make_node(monkeypatch)
    timers = _script_deadlines(monkeypatch, node)
    node._speak_client = FakeActionClient(ready=False)
    try:
        node._speak_text(
            '请说明目标地点。',
            Speak.Goal.NORMAL,
            'session-a',
            'turn-a',
            1,
        )
        assert node._speak_client.goals == []
        assert node._speak_operation is not None
        assert len(timers) == 1

        node._speak_client._ready = True
        timers[0].timeout()

        assert len(node._speak_client.goals) == 1
        assert node._speak_client.goals[0].text == '请说明目标地点。'
    finally:
        node.destroy_node()


def test_speak_failure_and_barge_in_never_cancel_mission(monkeypatch):
    node = _make_node(monkeypatch)
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        mission_client = node._mission_client
        mission_handle = mission_client.handles[0]
        mission_client.send_futures[0].resolve(mission_handle)

        node._on_turn_message(_make_turn('前进 0.5 米', sequence=2))
        first_speak = node._speak_client.handles[0]
        node._speak_client.send_futures[0].resolve(first_speak)
        first_speak.result_future.resolve(
            SimpleNamespace(result=SimpleNamespace(code=Speak.Result.FAILED))
        )
        assert node._mission_slot is not None
        assert mission_handle.cancel_calls == 0

        node._on_turn_message(_make_turn('前进 0.25 米', sequence=3))
        second_speak = node._speak_client.handles[1]
        node._speak_client.send_futures[1].resolve(second_speak)
        node._on_turn_message(_make_turn('前进 0.1 米', sequence=4))
        assert second_speak.cancel_calls == 1
        assert mission_handle.cancel_calls == 0
        assert node._mission_slot is not None
    finally:
        node.destroy_node()


def test_new_turn_fences_old_mission_terminal_speech(monkeypatch):
    node = _make_node(monkeypatch)
    try:
        node._on_turn_message(_make_turn('前进 1 米'))
        mission_client = node._mission_client
        mission_handle = mission_client.handles[0]
        mission_client.send_futures[0].resolve(mission_handle)
        node._on_turn_message(_make_turn('前进 0.5 米', sequence=2))
        mission_handle.result_future.resolve(
            _wrapped_result(ExecuteMission.Result.SUCCEEDED)
        )

        assert node._mission_slot is None
        assert '任务已完成。' not in [
            goal.text for goal in node._speak_client.goals
        ]
        assert node._speak_client.goals[-1].text == '本地任务正在处理。'
    finally:
        node.destroy_node()
