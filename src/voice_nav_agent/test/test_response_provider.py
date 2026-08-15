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

"""Closed literal-loopback Response Provider protocol tests."""

import json
import threading

import pytest

from voice_nav_agent import _response_provider
from voice_nav_agent._response_provider import (
    _decode_response,
    _LoopbackResponseProvider,
    _request_body,
)
from voice_nav_agent._response_session import (
    _ProviderResponse,
    _ResponseSession,
    _ToolCall,
)
from voice_nav_agent.core import (
    Availability,
    GateState,
    MissionState,
    OperatingMode,
    VoiceTurn,
)


class _Provider:
    def __init__(self):
        self.requests = []

    def submit(self, request):
        self.requests.append(request)


class _MissionPort:
    def prepare_mission(self, mission):
        return mission

    def commit_mission(self, _prepared):
        return object()

    def prepare_cancel(self, identity):
        return identity

    def is_active(self, _identity):
        return True

    def commit_cancel(self, _prepared):
        pass


def _state():
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


def test_literal_loopback_request_and_closed_clarify_response():
    """The provider speaks the frozen OpenAI envelope and closed inner JSON."""
    provider = _Provider()
    session = _ResponseSession('agent-a', provider, _state, _MissionPort())
    turn = VoiceTurn(
        voice_instance_id='voice-a',
        voice_seq=1,
        session_id='session-a',
        turn_id='turn-a',
        kind=VoiceTurn.COMMAND,
        text='请沿着大厅右侧绕过去',
        confidence=1.0,
    )
    session.accept_turn(turn)
    request = provider.requests[-1]

    body = json.loads(_request_body(request, session.tool_registry))
    assert body['model'] == 'Qwen3-0.6B-Q8_0.gguf'
    assert body['stream'] is False
    assert body['messages'][0]['content'] == '/no_think'
    assert [tool['function']['name'] for tool in body['tools']] == [
        'read_runtime_snapshot',
        'propose_mission',
        'cancel_owned_mission',
    ]
    assert json.loads(body['messages'][-1]['content']) == {
        'agent': {
            'source_instance_id': 'agent-a',
            'lifetime_generation': 1,
            'turn_generation': 0,
        },
        'runtime': {
            'runtime_instance_id': 'runtime-a',
            'admission_epoch': 7,
        },
        'clarification': None,
        'round': 1,
        'snapshot_output': None,
        'turn': {
            'session_id': 'session-a',
            'text': '请沿着大厅右侧绕过去',
            'turn_id': 'turn-a',
            'voice_instance_id': 'voice-a',
            'voice_seq': 1,
        },
    }

    response = _decode_response(
        json.dumps(
            {
                'choices': [
                    {
                        'message': {
                            'content': json.dumps(
                                {'kind': 'clarify', 'text': '请说明目的地。'}
                            )
                        }
                    }
                ]
            }
        ).encode('utf-8')
    )
    assert response is not None
    assert response.kind == 'clarify'
    assert response.text == '请说明目的地。'


def test_multiturn_request_keeps_agent_runtime_and_clarification_identity():
    """Provider recording carries the frozen identity chain across rounds."""
    provider = _Provider()
    session = _ResponseSession('agent-a', provider, _state, _MissionPort())
    first_turn = VoiceTurn(
        voice_instance_id='voice-a',
        voice_seq=1,
        session_id='session-a',
        turn_id='turn-a',
        kind=VoiceTurn.COMMAND,
        text='绕到大厅',
        confidence=1.0,
    )
    session.accept_turn(first_turn, adapter_generation=1)
    first = provider.requests[-1]
    session.complete(
        first, _ProviderResponse(kind='clarify', text='请说明需要前进多少米。')
    )

    second_turn = VoiceTurn(
        voice_instance_id='voice-a',
        voice_seq=2,
        session_id='session-a',
        turn_id='turn-b',
        kind=VoiceTurn.COMMAND,
        text='半米',
        confidence=1.0,
    )
    session.accept_turn(second_turn, adapter_generation=2)
    second = provider.requests[-1]
    session.complete(
        second,
        _ProviderResponse(
            kind='tool',
            tool_calls=(_ToolCall('read_runtime_snapshot', {}),),
        ),
    )
    continuation = provider.requests[-1]

    first_body = json.loads(_request_body(first, session.tool_registry))
    second_body = json.loads(_request_body(second, session.tool_registry))
    continuation_body = json.loads(
        _request_body(continuation, session.tool_registry)
    )
    for body, turn, generation in (
        (first_body, first_turn, 1),
        (second_body, second_turn, 2),
        (continuation_body, second_turn, 2),
    ):
        content = json.loads(body['messages'][-1]['content'])
        assert content['agent'] == {
            'source_instance_id': 'agent-a',
            'lifetime_generation': 1,
            'turn_generation': generation,
        }
        assert content['runtime'] == {
            'runtime_instance_id': 'runtime-a',
            'admission_epoch': 7,
        }
        assert content['turn']['voice_instance_id'] == turn.voice_instance_id
        assert content['turn']['voice_seq'] == turn.voice_seq
        assert content['turn']['session_id'] == 'session-a'
        assert content['turn']['turn_id'] == turn.turn_id
    assert json.loads(second_body['messages'][-1]['content'])['clarification'] == (
        '请说明需要前进多少米。'
    )
    assert json.loads(continuation_body['messages'][-1]['content'])['round'] == 2


@pytest.mark.parametrize('operation', ['invalidate', 'shutdown'])
def test_post_registration_transport_admission_is_before_teardown_return(
    monkeypatch, operation
):
    """Teardown cannot return between connection publication and request."""
    pre_request = threading.Event()
    release_request = threading.Event()
    teardown_returned = threading.Event()
    request_called = threading.Event()
    transport_called_after_teardown_return = []
    teardown_errors = []

    class _Connection:
        sock = None

        def __init__(self, *_args, **_kwargs):
            pass

        def __getattribute__(self, name):
            if name == 'request':
                pre_request.set()
                assert release_request.wait(1.0)
            return object.__getattribute__(self, name)

        def request(self, *_args, **_kwargs):
            transport_called_after_teardown_return.append(
                teardown_returned.is_set()
            )
            request_called.set()
            raise OSError('stop after admission probe')

        def close(self):
            pass

    monkeypatch.setattr(
        _response_provider.http.client, 'HTTPConnection', _Connection
    )
    original_join = threading.Thread.join

    def skip_provider_worker_join(worker, timeout=None):
        if worker.name == 'voice-nav-response-provider':
            return None
        return original_join(worker, timeout)

    monkeypatch.setattr(threading.Thread, 'join', skip_provider_worker_join)
    tool_registry = _ResponseSession(
        'agent-a', _Provider(), _state, _MissionPort()
    ).tool_registry
    provider = _LoopbackResponseProvider(
        'http://127.0.0.1:8080',
        tool_registry,
        lambda _request, _response: None,
        lambda _request: None,
    )
    provider.invalidate(1)
    session = _ResponseSession('agent-a', provider, _state, _MissionPort())
    session.accept_turn(
        VoiceTurn(
            voice_instance_id='voice-a',
            voice_seq=1,
            session_id='session-a',
            turn_id='turn-a',
            kind=VoiceTurn.COMMAND,
            text='需要推理',
            confidence=1.0,
        ),
        adapter_generation=1,
    )

    def teardown():
        try:
            if operation == 'invalidate':
                provider.invalidate(2)
            else:
                provider.shutdown()
        except RuntimeError as error:
            teardown_errors.append(error)
        finally:
            teardown_returned.set()

    teardown_thread = None
    try:
        assert pre_request.wait(1.0)
        teardown_thread = threading.Thread(target=teardown, daemon=True)
        teardown_thread.start()
        assert not teardown_returned.wait(0.1)
        release_request.set()
        assert request_called.wait(1.0)
        assert teardown_returned.wait(1.0)
        assert transport_called_after_teardown_return == [False]
        assert teardown_errors == []
    finally:
        release_request.set()
        if teardown_thread is not None:
            teardown_thread.join(1.0)
        provider.shutdown()


def test_shutdown_waits_for_post_delivery_callback_cleanup(monkeypatch):
    """A delivered callback stays registered through its cleanup."""
    callback_entered = threading.Event()
    release_callback = threading.Event()
    callback_finished = threading.Event()
    shutdown_returned = threading.Event()
    callbacks = []
    shutdown_errors = []

    def on_response(request, response):
        callback_entered.set()
        assert release_callback.wait(1.0)
        callbacks.append((request, response))
        callback_finished.set()

    tool_registry = _ResponseSession(
        'agent-a', _Provider(), _state, _MissionPort()
    ).tool_registry
    provider = _LoopbackResponseProvider(
        'http://127.0.0.1:8080',
        tool_registry,
        on_response,
        lambda _request: None,
    )
    provider.invalidate(1)
    monkeypatch.setattr(
        provider,
        '_complete',
        lambda _request: _response_provider._ProviderResponse(
            kind='reply', text='已收到。'
        ),
    )
    session = _ResponseSession('agent-a', provider, _state, _MissionPort())
    session.accept_turn(
        VoiceTurn(
            voice_instance_id='voice-a',
            voice_seq=1,
            session_id='session-a',
            turn_id='turn-a',
            kind=VoiceTurn.COMMAND,
            text='需要推理',
            confidence=1.0,
        ),
        adapter_generation=1,
    )

    def shutdown():
        try:
            provider.shutdown()
        except RuntimeError as error:
            shutdown_errors.append(error)
        finally:
            shutdown_returned.set()

    shutdown_thread = None
    try:
        assert callback_entered.wait(1.0)
        shutdown_thread = threading.Thread(target=shutdown, daemon=True)
        shutdown_thread.start()
        assert not shutdown_returned.wait(0.1)
        release_callback.set()
        assert callback_finished.wait(1.0)
        assert shutdown_returned.wait(1.0)
        assert len(callbacks) == 1
        assert shutdown_errors == []
    finally:
        release_callback.set()
        if shutdown_thread is not None:
            shutdown_thread.join(1.0)
        provider.shutdown()


def test_shutdown_waits_for_capacity_ready_callback_cleanup(monkeypatch):
    """Capacity notification remains part of the registered worker lifetime."""
    capacity_ready_entered = threading.Event()
    release_capacity_ready = threading.Event()
    shutdown_returned = threading.Event()
    responses = []
    shutdown_errors = []

    def on_capacity_ready():
        capacity_ready_entered.set()
        assert release_capacity_ready.wait(1.0)

    tool_registry = _ResponseSession(
        'agent-a', _Provider(), _state, _MissionPort()
    ).tool_registry
    provider = _LoopbackResponseProvider(
        'http://127.0.0.1:8080',
        tool_registry,
        lambda request, response: responses.append((request, response)),
        lambda _request: None,
        on_capacity_ready,
    )
    provider.invalidate(1)
    monkeypatch.setattr(
        provider,
        '_complete',
        lambda _request: _response_provider._ProviderResponse(
            kind='reply', text='已收到。'
        ),
    )
    session = _ResponseSession('agent-a', provider, _state, _MissionPort())
    session.accept_turn(
        VoiceTurn(
            voice_instance_id='voice-a',
            voice_seq=1,
            session_id='session-a',
            turn_id='turn-a',
            kind=VoiceTurn.COMMAND,
            text='需要推理',
            confidence=1.0,
        ),
        adapter_generation=1,
    )

    def shutdown():
        try:
            provider.shutdown()
        except RuntimeError as error:
            shutdown_errors.append(error)
        finally:
            shutdown_returned.set()

    shutdown_thread = None
    try:
        assert capacity_ready_entered.wait(1.0)
        shutdown_thread = threading.Thread(target=shutdown, daemon=True)
        shutdown_thread.start()
        assert not shutdown_returned.wait(0.1)
        release_capacity_ready.set()
        assert shutdown_returned.wait(1.0)
        assert len(responses) == 1
        assert shutdown_errors == []
    finally:
        release_capacity_ready.set()
        if shutdown_thread is not None:
            shutdown_thread.join(1.0)
        provider.shutdown()


def test_invalidated_worker_cannot_admit_http_after_active_registration(
    monkeypatch,
):
    """The final network admission rechecks generation without timing sleeps."""
    constructor_entered = threading.Event()
    release_constructor = threading.Event()
    http_requests = []
    delivered = []
    failed = []

    class _BlockedConnection:
        sock = None

        def __init__(self, *_args, **_kwargs):
            constructor_entered.set()
            assert release_constructor.wait(1.0)

        def request(self, *_args, **_kwargs):
            http_requests.append(True)

        def close(self):
            pass

    monkeypatch.setattr(
        _response_provider.http.client, 'HTTPConnection', _BlockedConnection
    )
    provider = _LoopbackResponseProvider(
        'http://127.0.0.1:8080',
        _ResponseSession('agent-a', _Provider(), _state, _MissionPort()).tool_registry,
        lambda request, response: delivered.append((request, response)),
        failed.append,
    )
    session = _ResponseSession('agent-a', provider, _state, _MissionPort())
    provider.invalidate(1)
    session.accept_turn(
        VoiceTurn(
            voice_instance_id='voice-a',
            voice_seq=1,
            session_id='session-a',
            turn_id='turn-a',
            kind=VoiceTurn.COMMAND,
            text='需要推理',
            confidence=1.0,
        ),
        adapter_generation=1,
    )

    assert constructor_entered.wait(1.0)
    provider.invalidate(2)
    release_constructor.set()
    provider.shutdown()

    assert http_requests == []
    assert delivered == []
    assert failed == []


def test_shutdown_before_worker_entry_skips_transport_and_cleans_up(
    monkeypatch,
):
    """A published worker starting after shutdown cannot enter transport."""
    worker_start_entered = threading.Event()
    release_worker_start = threading.Event()
    worker_start_returned = threading.Event()
    complete_entered = threading.Event()
    shutdown_completed = threading.Event()
    callbacks = []
    submissions = []
    shutdown_errors = []
    workers = []
    original_start = threading.Thread.start

    def block_provider_worker_start(worker):
        if worker.name != 'voice-nav-response-provider':
            return original_start(worker)
        workers.append(worker)
        worker_start_entered.set()
        assert release_worker_start.wait(1.0)
        result = original_start(worker)
        worker_start_returned.set()
        return result

    monkeypatch.setattr(threading.Thread, 'start', block_provider_worker_start)
    request_provider = _Provider()
    request_session = _ResponseSession(
        'agent-a', request_provider, _state, _MissionPort()
    )
    request_session.accept_turn(
        VoiceTurn(
            voice_instance_id='voice-a',
            voice_seq=1,
            session_id='session-a',
            turn_id='turn-a',
            kind=VoiceTurn.COMMAND,
            text='需要推理',
            confidence=1.0,
        )
    )
    request = request_provider.requests[-1]
    provider = _LoopbackResponseProvider(
        'http://127.0.0.1:8080',
        request_session.tool_registry,
        lambda _request, _response: callbacks.append('response'),
        lambda _request: callbacks.append('failure'),
    )

    def complete(_request):
        complete_entered.set()
        return None

    monkeypatch.setattr(provider, '_complete', complete)
    submitter = threading.Thread(
        target=lambda: submissions.append(provider.submit(request)), daemon=True
    )
    submitter.start()
    assert worker_start_entered.wait(1.0)

    def shutdown():
        try:
            provider.shutdown()
        except RuntimeError as error:
            shutdown_errors.append(error)
        finally:
            shutdown_completed.set()

    shutdown_thread = threading.Thread(target=shutdown, daemon=True)
    shutdown_thread.start()
    try:
        assert shutdown_completed.wait(1.0)
        release_worker_start.set()
        assert worker_start_returned.wait(1.0)
    finally:
        release_worker_start.set()
        submitter.join(1.0)
        shutdown_thread.join(1.0)

    assert not submitter.is_alive()
    assert not shutdown_thread.is_alive()
    assert shutdown_errors == []
    assert submissions == [True]
    assert len(workers) == 1
    assert not workers[0].is_alive()
    assert not complete_entered.is_set()
    assert callbacks == []


def test_invalidated_active_request_retries_latest_only_after_worker_exits(
    monkeypatch,
):
    """The one HTTP capacity slot moves to latest only after old work exits."""
    first_request_entered = threading.Event()
    first_connection_closed = threading.Event()
    second_request_entered = threading.Event()
    release_requests = threading.Event()
    latest_failure = threading.Event()
    invalidation_returned = threading.Event()
    request_indices = []
    invalidation_errors = []
    counter = [0]
    in_flight = [0]
    max_in_flight = [0]
    accounting_lock = threading.Lock()

    class _BlockedConnection:
        sock = None

        def __init__(self, *_args, **_kwargs):
            self.index = counter[0]
            counter[0] += 1

        def request(self, *_args, **_kwargs):
            with accounting_lock:
                request_indices.append(self.index)
                in_flight[0] += 1
                max_in_flight[0] = max(max_in_flight[0], in_flight[0])
            if self.index == 0:
                first_request_entered.set()
                assert release_requests.wait(1.0)
            with accounting_lock:
                in_flight[0] -= 1
            if self.index == 1:
                second_request_entered.set()

        def close(self):
            if self.index == 0:
                first_connection_closed.set()

        def getresponse(self):
            raise OSError('test transport is released')

    monkeypatch.setattr(
        _response_provider.http.client, 'HTTPConnection', _BlockedConnection
    )
    session_ref = {}
    failures = []

    def on_failure(request):
        failures.append(request)
        session_ref['session'].fail(request)
        if request.turn.turn_id == 'turn-b':
            latest_failure.set()

    provider = _LoopbackResponseProvider(
        'http://127.0.0.1:8080',
        _ResponseSession('agent-a', _Provider(), _state, _MissionPort()).tool_registry,
        lambda _request, _response: None,
        on_failure,
        lambda: session_ref['session'].provider_capacity_ready(),
    )
    session = _ResponseSession('agent-a', provider, _state, _MissionPort())
    session_ref['session'] = session
    provider.invalidate(1)
    session.accept_turn(
        VoiceTurn(
            voice_instance_id='voice-a',
            voice_seq=1,
            session_id='session-a',
            turn_id='turn-a',
            kind=VoiceTurn.COMMAND,
            text='first',
            confidence=1.0,
        ),
        adapter_generation=1,
    )
    assert first_request_entered.wait(1.0)

    def invalidate():
        try:
            provider.invalidate(2)
        except RuntimeError as error:
            invalidation_errors.append(error)
        finally:
            invalidation_returned.set()

    invalidation_thread = threading.Thread(target=invalidate, daemon=True)
    invalidation_thread.start()
    try:
        assert first_connection_closed.wait(1.0)
        assert not invalidation_returned.is_set()
        session.invalidate()
        session.accept_turn(
            VoiceTurn(
                voice_instance_id='voice-b',
                voice_seq=2,
                session_id='session-b',
                turn_id='turn-b',
                kind=VoiceTurn.COMMAND,
                text='latest',
                confidence=1.0,
            ),
            adapter_generation=2,
        )

        assert not second_request_entered.is_set()
        assert request_indices == [0]
        release_requests.set()
        assert invalidation_returned.wait(1.0)
        assert second_request_entered.wait(1.0)
        assert latest_failure.wait(1.0)
        assert max_in_flight == [1]
        assert request_indices == [0, 1]
        assert [request.turn.turn_id for request in failures] == ['turn-b']
        assert invalidation_errors == []
    finally:
        release_requests.set()
        invalidation_thread.join(1.0)
        provider.shutdown()


def test_lone_surrogate_content_fails_closed_and_releases_session(
    monkeypatch,
):
    """An outer-valid but non-UTF-8 inner content becomes one failure event."""
    raw = json.dumps(
        {
            'choices': [
                {'message': {'content': '\ud800'}},
            ]
        },
        ensure_ascii=True,
    ).encode('utf-8')
    failure_delivered = threading.Event()

    class _Response:
        status = 200

        def __init__(self):
            self._remaining = raw

        def getheader(self, name):
            if name == 'Content-Length':
                return str(len(raw))
            return None

        def read(self, _amount):
            result = self._remaining
            self._remaining = b''
            return result

    class _Connection:
        sock = None

        def __init__(self, *_args, **_kwargs):
            pass

        def request(self, *_args, **_kwargs):
            pass

        def getresponse(self):
            return _Response()

        def close(self):
            pass

    monkeypatch.setattr(
        _response_provider.http.client, 'HTTPConnection', _Connection
    )
    session_ref = {}

    def on_failure(request):
        session_ref['session'].fail(request)
        failure_delivered.set()

    provider = _LoopbackResponseProvider(
        'http://127.0.0.1:8080',
        _ResponseSession('agent-a', _Provider(), _state, _MissionPort()).tool_registry,
        lambda _request, _response: None,
        on_failure,
    )
    session = _ResponseSession('agent-a', provider, _state, _MissionPort())
    session_ref['session'] = session
    provider.invalidate(1)
    session.accept_turn(
        VoiceTurn(
            voice_instance_id='voice-a',
            voice_seq=1,
            session_id='session-a',
            turn_id='turn-a',
            kind=VoiceTurn.COMMAND,
            text='需要推理',
            confidence=1.0,
        ),
        adapter_generation=1,
    )

    assert failure_delivered.wait(1.0)
    assert not session.retained_state.active
    assert [event.kind for event in session.events] == ['failure']
    provider.shutdown()
