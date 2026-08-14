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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import queue
import socket
import threading
import time

import pytest

from voice_nav_agent.core import (
    AgentCore,
    Availability,
    DecisionKind,
    GateState,
    Mission,
    MissionState,
    OperatingMode,
    VoiceTurn,
)
import voice_nav_agent.llm_fallback as llm_fallback_module
from voice_nav_agent.llm_fallback import (
    LlmFallback,
    LlmFallbackRequest,
    LlmFallbackResult,
)


def _state():
    return MissionState(
        runtime_instance_id='runtime-a',
        admission_epoch=7,
        operating_mode=OperatingMode.NAVIGATION,
        availability=Availability.AVAILABLE,
        gate_state=GateState.GATE_INHIBITED,
        active_step=4294967295,
        supported_step_mask=0b1111,
        max_steps=3,
        named_place_ids=('lobby', 'charging'),
    )


def _unknown_turn():
    return VoiceTurn(
        voice_instance_id='voice-a',
        voice_seq=1,
        session_id='session-a',
        turn_id='turn-1',
        kind=VoiceTurn.COMMAND,
        text='请沿着大厅右侧绕过去',
        confidence=1.0,
        during_playback=False,
    )


def _locked_llama_completion(content):
    return {
        'id': 'chatcmpl-locked-smoke',
        'object': 'chat.completion',
        'created': 1722021900,
        'model': 'Qwen3-0.6B-Q8_0.gguf',
        'usage': {
            'prompt_tokens': 1,
            'completion_tokens': 2,
            'total_tokens': 3,
        },
        'choices': [
            {
                'index': 0,
                'finish_reason': 'stop',
                'message': {
                    'role': 'assistant',
                    'content': content,
                },
            }
        ],
    }


def _duplicate_semantic_path_bodies():
    mission_content = json.dumps(
        {
            'kind': 'mission',
            'steps': [{'kind': 'navigate_to', 'target_id': 'lobby'}],
        }
    )
    return (
        (
            '{"choices":[],"choices":[{"message":{"content":'
            + json.dumps(mission_content)
            + '}}]}'
        ).encode('utf-8'),
        json.dumps(
            {
                'choices': [
                    {
                        'message': {
                            'content': (
                                '{"kind":"reply","text":"first",'
                                '"text":"second"}'
                            )
                        }
                    }
                ]
            }
        ).encode('utf-8'),
    )


def _malformed_transport_envelopes():
    mission_content = json.dumps(
        {
            'kind': 'mission',
            'steps': [{'kind': 'navigate_to', 'target_id': 'lobby'}],
        }
    )
    valid_choice = {'message': {'content': mission_content}}
    return (
        [],
        {},
        {'choices': {}},
        {'choices': []},
        {'choices': [valid_choice, valid_choice]},
        {'choices': [None]},
        {'choices': [{}]},
        {'choices': [{'message': None}]},
        {'choices': [{'message': {}}]},
        {'choices': [{'message': {'content': {}}}]},
    )


def _json_response_frame(body):
    return (
        b'HTTP/1.1 200 OK\r\n'
        b'Content-Type: application/json\r\n'
        + b'Content-Length: '
        + str(len(body)).encode('ascii')
        + b'\r\n\r\n'
        + body
    )


class _CompletionServer:
    def __init__(self, response, *, status=200):
        self.requests = queue.Queue()
        self._response = response
        self._status = status
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                size = int(self.headers['Content-Length'])
                owner.requests.put((self.path, self.rfile.read(size)))
                body = json.dumps(owner._response).encode('utf-8')
                self.send_response(owner._status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, _format, *_args):
                pass

        self._server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.endpoint = (
            f'http://127.0.0.1:{self._server.server_address[1]}'
        )
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True
        )

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *_unused):
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(1.0)


class _BarrierCompletionServer:
    """Test-only server whose first request cannot finish before an event."""

    def __init__(self, first_response, next_response):
        self.requests = queue.Queue()
        self.first_entered = threading.Event()
        self.release_first = threading.Event()
        self._first_response = first_response
        self._next_response = next_response
        self._request_count = 0
        self._count_lock = threading.Lock()
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                size = int(self.headers['Content-Length'])
                with owner._count_lock:
                    owner._request_count += 1
                    request_number = owner._request_count
                owner.requests.put((request_number, self.path, self.rfile.read(size)))
                if request_number == 1:
                    owner.first_entered.set()
                    if not owner.release_first.wait(1.0):
                        self.close_connection = True
                        return
                    response = owner._first_response
                else:
                    response = owner._next_response
                body = json.dumps(response).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, _format, *_args):
                pass

        self._server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.endpoint = (
            f'http://127.0.0.1:{self._server.server_address[1]}'
        )
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True
        )

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *_unused):
        self.release_first.set()
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(1.0)


class _RawCompletionServer:
    """Test-only server that emits one exact HTTP response frame."""

    def __init__(self, raw_response):
        self.requests = queue.Queue()
        self._raw_response = raw_response
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                size = int(self.headers['Content-Length'])
                owner.requests.put((self.path, self.rfile.read(size)))
                self.connection.sendall(owner._raw_response)
                self.close_connection = True

            def log_message(self, _format, *_args):
                pass

        self._server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.endpoint = (
            f'http://127.0.0.1:{self._server.server_address[1]}'
        )
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True
        )

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *_unused):
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(1.0)


def test_unknown_turn_completion_yields_validated_typed_mission():
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    assert decision.kind is DecisionKind.LLM_NEEDED
    completions = {
        'choices': [
            {
                'message': {
                    'content': json.dumps(
                        {
                            'kind': 'mission',
                            'steps': [
                                {'kind': 'navigate_to', 'target_id': 'lobby'}
                            ]
                        }
                    )
                }
            }
        ]
    }
    delivered = queue.Queue()

    with _CompletionServer(completions) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put((request, result)),
            lambda request: delivered.put((request, None)),
        )
        request = LlmFallbackRequest(
            decision=decision,
            turn_generation=1,
        )
        fallback.submit(request)
        callback_request, mission = delivered.get(timeout=1.0)
        path, _body = server.requests.get(timeout=1.0)
        fallback.shutdown()

    assert path == '/v1/chat/completions'
    assert callback_request == request
    assert isinstance(mission, LlmFallbackResult)
    assert isinstance(mission.mission, Mission)
    assert mission.mission.token == decision.token
    assert mission.mission.steps[0].target_id == 'lobby'


def test_locked_llama_server_envelope_delivers_validated_typed_mission():
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()
    completion = _locked_llama_completion(
        json.dumps(
            {
                'kind': 'mission',
                'steps': [{'kind': 'navigate_to', 'target_id': 'lobby'}],
            }
        )
    )

    with _CompletionServer(completion) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put((request, result)),
            lambda request: delivered.put((request, None)),
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)
        fallback.submit(request)
        callback_request, mission = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert callback_request == request
    assert isinstance(mission, LlmFallbackResult)
    assert isinstance(mission.mission, Mission)
    assert mission.mission.token == decision.token
    assert mission.mission.steps[0].target_id == 'lobby'


@pytest.mark.parametrize(
    ('content', 'expected_kind', 'expected_text'),
    [
        (
            json.dumps({'kind': 'clarify', 'prompt': '请说明目标地点。'}),
            'clarify',
            '请说明目标地点。',
        ),
        (
            json.dumps({'kind': 'reply', 'text': '该请求不能执行。'}),
            'reply',
            '该请求不能执行。',
        ),
    ],
)
def test_locked_llama_server_envelope_delivers_clarify_or_reply(
    content, expected_kind, expected_text
):
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()

    with _CompletionServer(_locked_llama_completion(content)) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put((request, result)),
            lambda request: delivered.put((request, None)),
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)
        fallback.submit(request)
        callback_request, result = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert callback_request == request
    assert isinstance(result, LlmFallbackResult)
    assert result.kind == expected_kind
    assert result.text == expected_text


def test_request_body_matches_locked_llama_smoke_fields():
    decision = AgentCore('agent-a').handle_turn(_unknown_turn(), _state())
    schema = {'type': 'object', 'additionalProperties': False}

    payload = json.loads(
        llm_fallback_module._request_body(decision, schema).decode('utf-8')
    )

    assert payload['model'] == 'Qwen3-0.6B-Q8_0.gguf'
    assert payload['messages'][0] == {'role': 'system', 'content': '/no_think'}
    assert payload['stream'] is False
    assert payload['max_tokens'] == 256
    assert payload['response_format'] == {
        'type': 'json_schema',
        'schema': schema,
    }


@pytest.mark.parametrize('body', _duplicate_semantic_path_bodies())
def test_duplicate_semantic_paths_never_deliver_a_completion(body):
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()

    with _RawCompletionServer(_json_response_frame(body)) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)
        fallback.submit(request)
        delivered_kind, callback_request, result = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert delivered_kind == 'failure'
    assert callback_request == request
    assert result is None


@pytest.mark.parametrize('completion', _malformed_transport_envelopes())
def test_malformed_transport_paths_never_deliver_a_completion(completion):
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()

    with _CompletionServer(completion) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)
        fallback.submit(request)
        delivered_kind, callback_request, result = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert delivered_kind == 'failure'
    assert callback_request == request
    assert result is None


def test_transport_metadata_cannot_supply_mission_content():
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()
    mission_content = json.dumps(
        {
            'kind': 'mission',
            'steps': [{'kind': 'navigate_to', 'target_id': 'lobby'}],
        }
    )
    completion = _locked_llama_completion('{}')
    completion['mission'] = mission_content
    completion['choices'][0]['function_call'] = {
        'arguments': mission_content,
    }
    completion['choices'][0]['message']['tool_calls'] = [
        {'function': {'arguments': mission_content}}
    ]

    with _CompletionServer(completion) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)
        fallback.submit(request)
        delivered_kind, callback_request, result = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert delivered_kind == 'failure'
    assert callback_request == request
    assert result is None


@pytest.mark.parametrize(
    ('result_json', 'expected_kind', 'expected_text'),
    [
        ({'kind': 'clarify', 'prompt': '请说明目标地点。'}, 'clarify', '请说明目标地点。'),
        ({'kind': 'reply', 'text': '该请求不能执行。'}, 'reply', '该请求不能执行。'),
    ],
)
def test_closed_completion_delivers_legal_clarify_or_reply(
    result_json, expected_kind, expected_text
):
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()
    completions = {
        'choices': [
            {'message': {'content': json.dumps(result_json)}}
        ]
    }

    with _CompletionServer(completions) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put((request, result)),
            lambda request: delivered.put((request, None)),
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)
        fallback.submit(request)
        callback_request, result = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert callback_request == request
    assert result.kind == expected_kind
    assert result.text == expected_text


def test_burst_keeps_only_active_and_latest_pending_completion():
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    stale_response = {'choices': [{'message': {'content': '{'}}]}
    latest_response = {
        'choices': [
            {
                'message': {
                    'content': json.dumps(
                        {
                            'kind': 'mission',
                            'steps': [
                                {'kind': 'navigate_to', 'target_id': 'lobby'}
                            ],
                        }
                    )
                }
            }
        ]
    }
    delivered = queue.Queue()

    with _BarrierCompletionServer(stale_response, latest_response) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        fallback.submit(LlmFallbackRequest(decision=decision, turn_generation=1))
        assert server.first_entered.wait(1.0)
        for generation in range(2, 1001):
            fallback.submit(
                LlmFallbackRequest(
                    decision=decision,
                    turn_generation=generation,
                )
            )
        server.release_first.set()
        kind, request, result = delivered.get(timeout=2.0)
        fallback.shutdown()

    observed = []
    while not server.requests.empty():
        observed.append(server.requests.get_nowait())
    assert kind == 'result'
    assert request.turn_generation == 1000
    assert result.kind == 'mission'
    assert [number for number, _path, _body in observed] == [1, 2]


@pytest.mark.parametrize(
    'content',
    [
        '{',
        '```json\n{"kind":"reply","text":"不应接受"}\n```',
        '{"kind":"mission","steps":[{"kind":"move_distance","distance_m":NaN}]}',
        '{"kind":"mission","steps":[{"kind":"move_distance","distance_m":Infinity}]}',
        json.dumps(
            {
                'kind': 'mission',
                'steps': [
                    {'kind': 'navigate_to', 'target_id': 'lobby'},
                    {'kind': 'navigate_to', 'target_id': 'lobby'},
                    {'kind': 'navigate_to', 'target_id': 'lobby'},
                    {'kind': 'navigate_to', 'target_id': 'lobby'},
                ],
            }
        ),
        json.dumps(
            {
                'kind': 'mission',
                'steps': [
                    {
                        'kind': 'navigate_to',
                        'target_id': 'lobby',
                        'override': True,
                    }
                ],
            }
        ),
        json.dumps({'kind': 'reply', 'text': '不应接受', 'extra': True}),
        json.dumps(
            {
                'kind': 'mission',
                'steps': [{'kind': 'navigate_to', 'target_id': 'unknown'}],
            }
        ),
        json.dumps(
            {
                'kind': 'mission',
                'steps': [{'kind': 'save_map', 'target_id': 'map_a'}],
            }
        ),
    ],
)
def test_invalid_or_unauthorized_completion_never_delivers_a_mission(content):
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()
    completion = {'choices': [{'message': {'content': content}}]}

    with _CompletionServer(completion) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)
        fallback.submit(request)
        delivered_kind, callback_request, result = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert delivered_kind == 'failure'
    assert callback_request == request
    assert result is None


def test_non_200_completion_never_delivers_a_mission():
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()
    completion = {'choices': [{'message': {'content': '{}'}}]}

    with _CompletionServer(completion, status=302) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)
        fallback.submit(request)
        delivered_kind, callback_request, result = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert delivered_kind == 'failure'
    assert callback_request == request
    assert result is None


@pytest.mark.parametrize(
    'raw_response',
    [
        lambda body: (
            b'HTTP/1.1 200 OK\r\n'
            b'Content-Type: application/json\r\n'
            b'Transfer-Encoding: chunked\r\n\r\n'
            + f'{len(body):X}\r\n'.encode('ascii')
            + body
            + b'\r\n0\r\n\r\n'
        ),
        lambda body: (
            b'HTTP/1.1 200 OK\r\n'
            b'Content-Type: application/json\r\n\r\n' + body
        ),
    ],
)
def test_chunked_or_unbounded_completion_response_never_delivers_mission(
    raw_response,
):
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    body = json.dumps(
        {
            'choices': [
                {
                    'message': {
                        'content': json.dumps(
                            {
                                'kind': 'mission',
                                'steps': [
                                    {'kind': 'navigate_to', 'target_id': 'lobby'}
                                ],
                            }
                        )
                    }
                }
            ]
        }
    ).encode('utf-8')
    delivered = queue.Queue()

    with _RawCompletionServer(raw_response(body)) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)
        fallback.submit(request)
        delivered_kind, callback_request, result = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert delivered_kind == 'failure'
    assert callback_request == request
    assert result is None


@pytest.mark.parametrize(
    'endpoint',
    [
        'http://localhost:8080',
        'http://127.0.0.1:0',
        'http://127.0.0.1:65536',
        'http://127.0.0.1:08080',
        'https://127.0.0.1:8080',
        'http://[::1]:8080',
        'http://user@127.0.0.1:8080',
        'http://127.0.0.1:8080/path',
        'http://127.0.0.1:8080?query',
        'http://127.0.0.1:8080#fragment',
    ],
)
def test_endpoint_must_be_literal_localhost_authority(endpoint):
    core = AgentCore('agent-a')

    with pytest.raises(ValueError):
        LlmFallback(
            endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda _request, _result: None,
            lambda _request: None,
        )


@pytest.mark.parametrize(
    'raw_response',
    [
        lambda body: (
            b'HTTP/1.1 200 OK\r\nContent-Length: 1\r\n\r\n\xff'
        ),
        lambda body: (
            b'HTTP/1.1 200 OK\r\nContent-Length: '
            + str(len(body) + 1).encode('ascii')
            + b'\r\n\r\n'
            + body
        ),
        lambda body: (
            b'HTTP/1.1 200 OK\r\nX-Boundary: '
            + b'x' * (70 * 1024)
            + b'\r\nContent-Length: '
            + str(len(body)).encode('ascii')
            + b'\r\n\r\n'
            + body
        ),
        lambda body: (
            b'HTTP/1.1 200 OK\r\nContent-Length: '
            + str(64 * 1024 + 1).encode('ascii')
            + b'\r\n\r\n'
            + body
            + b' ' * (64 * 1024 + 1 - len(body))
        ),
        lambda body: b'',
    ],
)
def test_malformed_or_oversized_transport_never_delivers_mission(raw_response):
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    body = json.dumps(
        {
            'choices': [
                {
                    'message': {
                        'content': json.dumps(
                            {
                                'kind': 'mission',
                                'steps': [
                                    {'kind': 'navigate_to', 'target_id': 'lobby'}
                                ],
                            }
                        )
                    }
                }
            ]
        }
    ).encode('utf-8')
    delivered = queue.Queue()

    with _RawCompletionServer(raw_response(body)) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)
        fallback.submit(request)
        delivered_kind, callback_request, result = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert delivered_kind == 'failure'
    assert callback_request == request
    assert result is None


def test_refused_completion_never_delivers_a_mission(monkeypatch):
    monkeypatch.setattr(llm_fallback_module, '_DEADLINE_SECONDS', 0.05)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(('127.0.0.1', 0))
        port = listener.getsockname()[1]
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()
    fallback = LlmFallback(
        f'http://127.0.0.1:{port}',
        core.semantic_validator,
        lambda callback, *args: callback(*args),
        lambda request: True,
        lambda request, result: delivered.put(('result', request, result)),
        lambda request: delivered.put(('failure', request, None)),
    )
    request = LlmFallbackRequest(decision=decision, turn_generation=1)
    fallback.submit(request)
    delivered_kind, callback_request, result = delivered.get(timeout=1.0)
    fallback.shutdown()

    assert delivered_kind == 'failure'
    assert callback_request == request
    assert result is None


def test_slow_completion_hits_the_total_deadline_without_a_mission(monkeypatch):
    monkeypatch.setattr(llm_fallback_module, '_DEADLINE_SECONDS', 0.05)
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()
    completion = {'choices': [{'message': {'content': '{}'}}]}

    with _BarrierCompletionServer(completion, completion) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)
        fallback.submit(request)
        assert server.first_entered.wait(1.0)
        delivered_kind, callback_request, result = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert delivered_kind == 'failure'
    assert callback_request == request
    assert result is None


def test_shutdown_invalidates_active_request_and_joins_without_delivery():
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()
    completion = {'choices': [{'message': {'content': '{}'}}]}

    with _BarrierCompletionServer(completion, completion) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        fallback.submit(LlmFallbackRequest(decision=decision, turn_generation=1))
        assert server.first_entered.wait(1.0)
        started = time.monotonic()
        fallback.shutdown()
        elapsed = time.monotonic() - started

    assert elapsed <= 1.1
    assert delivered.empty()


@pytest.mark.parametrize(
    ('completion', 'expected_kind'),
    [
        (
            {
                'choices': [
                    {
                        'message': {
                            'content': json.dumps(
                                {
                                    'kind': 'mission',
                                    'steps': [
                                        {
                                            'kind': 'navigate_to',
                                            'target_id': 'lobby',
                                        }
                                    ],
                                }
                            )
                        }
                    }
                ]
            },
            'result',
        ),
        ({'choices': [{'message': {'content': '{'}}]}, 'failure'),
    ],
)
def test_shutdown_after_current_fence_suppresses_serial_callback(
    monkeypatch, completion, expected_kind
):
    """A shutdown after the current fence cannot leave a late serial callback."""
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    current_passed = threading.Event()
    release_current = threading.Event()
    shutdown_finished = threading.Event()
    delivered = queue.Queue()

    def admission(_request):
        return True

    def on_result(request, result):
        delivered.put(('result', request, result))

    def on_failure(request):
        delivered.put(('failure', request, None))

    with _CompletionServer(completion) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            admission,
            on_result,
            on_failure,
        )
        request = LlmFallbackRequest(decision=decision, turn_generation=1)

        original_current = fallback._current

        def pause_after_current(job):
            assert original_current(job)
            current_passed.set()
            assert release_current.wait(2.0)
            return True

        monkeypatch.setattr(fallback, '_current', pause_after_current)
        fallback.submit(request)
        assert current_passed.wait(1.0)

        shutdown_thread = threading.Thread(
            target=lambda: (fallback.shutdown(), shutdown_finished.set()),
            daemon=True,
        )
        shutdown_thread.start()
        assert shutdown_finished.wait(1.1)

        release_current.set()
        shutdown_thread.join(1.0)

    assert shutdown_finished.is_set()
    assert delivered.empty(), expected_kind


@pytest.mark.parametrize(
    ('result', 'expected_delivery'),
    [
        (LlmFallbackResult('reply', text='当前无法处理该导航请求。'), 'result'),
        (None, 'failure'),
    ],
    ids=('result', 'failure'),
)
def test_new_turn_invalidation_does_not_wait_for_delivery_serial_seam(
    monkeypatch, result, expected_delivery
):
    """A new Turn invalidates without circularly waiting for delivery."""
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivery_ready = threading.Event()
    delivery_waiting_for_seam = threading.Event()
    new_turn_in_seam = threading.Event()
    release_delivery = threading.Event()
    release_new_turn = threading.Event()
    invalidation_finished = threading.Event()
    delivered = queue.Queue()

    def serial_invoke(callback, *args):
        assert new_turn_in_seam.is_set()
        delivery_waiting_for_seam.set()
        assert release_new_turn.wait(2.0)
        return callback(*args)

    fallback = LlmFallback(
        'http://127.0.0.1:8080',
        core.semantic_validator,
        serial_invoke,
        lambda request: True,
        lambda request, completion: delivered.put(
            ('result', request, completion)
        ),
        lambda request: delivered.put(('failure', request, None)),
    )
    original_current = fallback._current

    def pause_before_delivery(job):
        assert original_current(job)
        delivery_ready.set()
        assert release_delivery.wait(1.0)
        return True

    monkeypatch.setattr(fallback, '_complete', lambda _job: (True, result))
    monkeypatch.setattr(fallback, '_current', pause_before_delivery)
    request = LlmFallbackRequest(decision=decision, turn_generation=1)
    fallback.submit(request)
    assert delivery_ready.wait(1.0)

    def invalidate_from_new_turn():
        new_turn_in_seam.set()
        release_delivery.set()
        assert delivery_waiting_for_seam.wait(1.0)
        fallback.invalidate(2)
        invalidation_finished.set()

    invalidation_thread = threading.Thread(
        target=invalidate_from_new_turn, daemon=True
    )
    invalidation_thread.start()
    try:
        assert invalidation_finished.wait(1.0), expected_delivery
    finally:
        release_new_turn.set()
        invalidation_thread.join(1.0)
        fallback.shutdown()

    assert not invalidation_thread.is_alive()
    assert delivered.empty()


def test_stale_failure_is_silent_after_invalidation():
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    delivered = queue.Queue()
    invalid = {'choices': [{'message': {'content': '{'}}]}

    with _BarrierCompletionServer(invalid, invalid) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        fallback.submit(LlmFallbackRequest(decision=decision, turn_generation=1))
        assert server.first_entered.wait(1.0)
        fallback.invalidate(2)
        server.release_first.set()
        fallback.shutdown()

    assert delivered.empty()


def test_oversized_request_is_rejected_before_any_http_submission():
    core = AgentCore('agent-a')
    decision = core.handle_turn(_unknown_turn(), _state())
    oversized_decision = replace(decision, normalized_text='x' * (32 * 1024))
    delivered = queue.Queue()
    completion = {'choices': [{'message': {'content': '{}'}}]}

    with _CompletionServer(completion) as server:
        fallback = LlmFallback(
            server.endpoint,
            core.semantic_validator,
            lambda callback, *args: callback(*args),
            lambda request: True,
            lambda request, result: delivered.put(('result', request, result)),
            lambda request: delivered.put(('failure', request, None)),
        )
        request = LlmFallbackRequest(
            decision=oversized_decision,
            turn_generation=1,
        )
        fallback.submit(request)
        delivered_kind, callback_request, result = delivered.get(timeout=1.0)
        fallback.shutdown()

    assert delivered_kind == 'failure'
    assert callback_request == request
    assert result is None
    assert server.requests.empty()
