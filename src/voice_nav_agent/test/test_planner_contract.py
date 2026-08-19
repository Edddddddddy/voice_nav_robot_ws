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

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading

from voice_nav_agent._planner import (
    LoopbackPlanner,
    PlannerRequest,
    PlannerResponse,
)
from voice_nav_agent.core import (
    Availability,
    GateState,
    MissionState,
    OperatingMode,
    PlanningToken,
    VoiceTurn,
)
from voice_nav_agent.planner_schema import (
    decode_completion,
    decode_completion_with_reason,
    decode_planner_value,
    decode_planner_value_with_reason,
    load_response_schema,
    load_system_prompt,
)


def _request():
    snapshot = MissionState(
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
    turn = VoiceTurn(
        'voice-a', 1, 'session-a', 'turn-1', VoiceTurn.COMMAND, '绕到大厅', 1.0
    )
    token = PlanningToken(
        'agent-a', 1, 'voice-a', 1, 'session-a', 'turn-1', 1,
        'runtime-a', 7, OperatingMode.NAVIGATION, 0b1111, 3, ('lobby',),
        Availability.AVAILABLE, GateState.GATE_INHIBITED,
    )
    return PlannerRequest(turn, token, snapshot, 1)


def test_loopback_request_contains_snapshot_prompt_versions_and_bounded_decode():
    captured = {}

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            captured.update(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
            response = json.dumps({
                'choices': [{
                    'message': {
                        'content': json.dumps({'kind': 'reply', 'text': 'ok'})
                    }
                }]
            }).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    planner = LoopbackPlanner(
        f'http://127.0.0.1:{server.server_address[1]}'
    )
    completed = threading.Event()
    responses = []
    try:
        assert planner.submit(
            _request(),
            lambda _request, response: (
                responses.append(response), completed.set()
            ),
        )
        assert completed.wait(2.0)
    finally:
        planner.shutdown()
        server.shutdown()
        server.server_close()
        worker.join(2.0)

    assert responses[0].kind == 'reply'
    assert captured['messages'][0] == {'role': 'system', 'content': '/no_think'}
    assert '/no_think' in captured['messages'][1]['content']
    assert captured['temperature'] == 0
    assert captured['stream'] is False
    assert captured['max_tokens'] == 256
    assert captured['parallel_tool_calls'] is False
    assert all(
        tool['function']['strict'] is True for tool in captured['tools']
    )
    assert captured['messages'][-1]['content'].find('runtime_snapshot') >= 0
    assert [tool['function']['name'] for tool in captured['tools']] == [
        'read_runtime_snapshot', 'propose_mission', 'cancel_owned_mission'
    ]


def test_loopback_propagates_bounded_inner_schema_reason():
    captured = {}

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            captured.update(json.loads(self.rfile.read(int(
                self.headers['Content-Length']
            ))))
            content = json.dumps({
                'name': 'propose_mission',
                'arguments': {
                    'kind': 'mission',
                    'steps': [{
                        'kind': 'navigate_to',
                        'target_id': 'lobby',
                    }],
                },
            })
            response = json.dumps({
                'choices': [{
                    'index': 0,
                    'finish_reason': 'stop',
                    'logprobs': None,
                    'message': {
                        'role': 'assistant',
                        'content': content,
                        'reasoning_content': None,
                    },
                }],
            }).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    planner = LoopbackPlanner(
        f'http://127.0.0.1:{server.server_address[1]}'
    )
    completed = threading.Event()
    responses = []
    try:
        assert planner.submit(
            _request(),
            lambda _request, response: (
                responses.append(response), completed.set()
            ),
        )
        assert completed.wait(2.0)
    finally:
        planner.shutdown()
        server.shutdown()
        server.server_close()
        worker.join(2.0)

    assert responses[0].kind == 'invalid'
    assert responses[0].reason == 'missing_kind_name_arguments'
    assert captured['model'] == 'Qwen3-0.6B-Q8_0.gguf'


def test_loopback_normalizes_embedded_openai_tool_call():
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            content = json.dumps({
                'kind': 'tool',
                'tool_call': {
                    'type': 'function',
                    'function': {
                        'name': 'read_runtime_snapshot',
                        'arguments': '{}',
                    },
                },
            })
            response = json.dumps({
                'choices': [{
                    'message': {
                        'content': content,
                    },
                }],
            }).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    planner = LoopbackPlanner(
        f'http://127.0.0.1:{server.server_address[1]}'
    )
    completed = threading.Event()
    responses = []
    try:
        assert planner.submit(
            _request(),
            lambda _request, response: (
                responses.append(response), completed.set()
            ),
        )
        assert completed.wait(2.0)
    finally:
        planner.shutdown()
        server.shutdown()
        server.server_close()
        worker.join(2.0)

    assert responses[0].kind == 'tool'
    assert responses[0].tool_name == 'read_runtime_snapshot'
    assert dict(responses[0].tool_arguments) == {}


def test_loopback_normalizes_embedded_tool_call_id_and_mission_arguments():
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            content = json.dumps({
                'kind': 'tool',
                'tool_call': {
                    'id': 'call-1',
                    'type': 'function',
                    'function': {
                        'name': 'propose_mission',
                        'arguments': json.dumps({
                            'kind': 'mission',
                            'steps': [{
                                'kind': 'navigate_to',
                                'target_id': 'lobby',
                            }],
                        }),
                    },
                },
            })
            response = json.dumps({
                'choices': [{
                    'message': {
                        'content': content,
                    },
                }],
            }).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    planner = LoopbackPlanner(
        f'http://127.0.0.1:{server.server_address[1]}'
    )
    completed = threading.Event()
    responses = []
    try:
        assert planner.submit(
            _request(),
            lambda _request, response: (
                responses.append(response), completed.set()
            ),
        )
        assert completed.wait(2.0)
    finally:
        planner.shutdown()
        server.shutdown()
        server.server_close()
        worker.join(2.0)

    assert responses[0].kind == 'tool'
    assert responses[0].tool_name == 'propose_mission'
    arguments = dict(responses[0].tool_arguments)
    assert arguments['kind'] == 'mission'
    assert arguments['steps'][0].target_id == 'lobby'


def test_loopback_normalizes_locked_qwen_mission_content_shape():
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            content = json.dumps({
                'kind': 'tool',
                'tool_call': {
                    'kind': 'mission',
                    'steps': [{
                        'kind': 'navigate_to',
                        'target_id': 'lobby',
                    }],
                },
            })
            response = json.dumps({
                'choices': [{
                    'message': {
                        'content': content,
                    },
                }],
            }).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    planner = LoopbackPlanner(
        f'http://127.0.0.1:{server.server_address[1]}'
    )
    completed = threading.Event()
    responses = []
    try:
        assert planner.submit(
            _request(),
            lambda _request, response: (
                responses.append(response), completed.set()
            ),
        )
        assert completed.wait(2.0)
    finally:
        planner.shutdown()
        server.shutdown()
        server.server_close()
        worker.join(2.0)

    assert responses[0].kind == 'tool'
    assert responses[0].tool_name == 'propose_mission'
    arguments = dict(responses[0].tool_arguments)
    assert arguments['kind'] == 'mission'
    assert arguments['steps'][0].target_id == 'lobby'


def test_decoder_rejects_think_markers_duplicate_keys_and_extra_inner_fields():
    def envelope(content):
        return json.dumps(
            {'choices': [{'message': {'role': 'assistant', 'content': content}}]},
            ensure_ascii=False,
        ).encode()

    assert decode_completion(
        envelope('{"kind":"reply","text":"<think>x"}')
    ) is None
    duplicate = (
        b'{"choices":[{"message":{"content":"{\\"kind\\":\\"reply\\",'
        b'\\"kind\\":\\"reply\\",\\"text\\":\\"x\\"}"}}]}'
    )
    assert decode_completion(duplicate) is None
    assert decode_completion(b'{"choices":[{"message":{}}]}') is None
    assert decode_planner_value({'kind': 'reply', 'text': 'x', 'extra': 1}) is None
    assert decode_planner_value({
        'kind': 'tool',
        'tool_call': {
            'type': 'function',
            'function': {
                'name': 'read_runtime_snapshot',
                'arguments': '{}',
            },
        },
    }) is None


def test_planner_value_decoder_reports_only_bounded_schema_reasons():
    cases = [
        (None, 'root'),
        ({'kind': 1}, 'kind_non_string'),
        ({'steps': []}, 'missing_kind_steps_only'),
        (
            {'name': 'propose_mission', 'arguments': {}},
            'missing_kind_name_arguments',
        ),
        ({'tool_call': {}}, 'missing_kind_tool_call_only'),
        ({'text': 'x'}, 'missing_kind_other'),
        ({'kind': 'unknown'}, 'kind_unknown'),
        ({'kind': 'reply'}, 'reply_fields'),
        ({'kind': 'reply', 'text': None}, 'text'),
        ({'kind': 'mission'}, 'mission_fields'),
        ({'kind': 'mission', 'steps': {}}, 'steps_type'),
        ({'kind': 'mission', 'steps': []}, 'steps_count'),
        ({'kind': 'mission', 'steps': [None]}, 'step_shape'),
        ({'kind': 'mission', 'steps': [{}]}, 'step_kind'),
        ({
            'kind': 'mission',
            'steps': [{'kind': 'navigate_to'}],
        }, 'step_fields'),
        ({
            'kind': 'mission',
            'steps': [{
                'kind': 'move_distance',
                'distance_m': 'bad',
            }],
        }, 'step_value'),
        ({'kind': 'tool'}, 'tool_fields'),
        ({'kind': 'tool', 'tool_call': None}, 'call_shape'),
        ({
            'kind': 'tool',
            'tool_call': {'name': 'unknown', 'arguments': {}},
        }, 'tool_name'),
        ({
            'kind': 'tool',
            'tool_call': {
                'name': 'read_runtime_snapshot',
                'arguments': {'extra': 1},
            },
        }, 'empty_args'),
        ({
            'kind': 'tool',
            'tool_call': {
                'name': 'propose_mission',
                'arguments': {},
            },
        }, 'mission_args_fields'),
        ({
            'kind': 'tool',
            'tool_call': {
                'name': 'propose_mission',
                'arguments': {'kind': 'other', 'steps': []},
            },
        }, 'mission_args_kind'),
        ({
            'kind': 'tool',
            'tool_call': {
                'name': 'propose_mission',
                'arguments': {'kind': 'mission', 'steps': []},
            },
        }, 'mission_args_steps'),
    ]

    for value, expected_reason in cases:
        decoded, reason = decode_planner_value_with_reason(value)
        assert decoded is None
        assert reason == expected_reason

    decoded, reason = decode_planner_value_with_reason({
        'kind': 'reply', 'text': 'ok'
    })
    assert decoded == {'kind': 'reply', 'text': 'ok'}
    assert reason == 'ok'


def test_decoder_accepts_null_content_single_function_tool_call():
    envelope = json.dumps({
        'choices': [{
            'message': {
                'role': 'assistant',
                'content': None,
                'tool_calls': [{
                    'type': 'function',
                    'function': {
                        'name': 'read_runtime_snapshot',
                        'arguments': '{}',
                    },
                }],
            },
        }],
    }).encode()

    value = decode_planner_value(decode_completion(envelope))
    assert value == {
        'kind': 'tool',
        'name': 'read_runtime_snapshot',
        'arguments': {},
    }
    response = PlannerResponse.tool(value['name'], value['arguments'])
    assert response.kind == 'tool'
    assert response.tool_name == 'read_runtime_snapshot'
    assert dict(response.tool_arguments) == {}


def test_decoder_accepts_standard_choice_metadata_and_null_reasoning():
    envelope = json.dumps({
        'choices': [{
            'index': 0,
            'finish_reason': 'tool_calls',
            'logprobs': None,
            'message': {
                'role': 'assistant',
                'content': None,
                'reasoning_content': None,
                'tool_calls': [{
                    'type': 'function',
                    'function': {
                        'name': 'read_runtime_snapshot',
                        'arguments': '{}',
                    },
                }],
            },
        }],
    }).encode()

    value = decode_planner_value(decode_completion(envelope))
    assert value == {
        'kind': 'tool',
        'name': 'read_runtime_snapshot',
        'arguments': {},
    }


def test_decoder_rejects_invalid_standard_completion_metadata():
    base_call = {
        'type': 'function',
        'function': {
            'name': 'read_runtime_snapshot',
            'arguments': '{}',
        },
    }

    def envelope(choice):
        return json.dumps({'choices': [choice]}).encode()

    base_choice = {
        'index': 0,
        'finish_reason': 'tool_calls',
        'logprobs': None,
        'message': {
            'role': 'assistant',
            'content': None,
            'reasoning_content': None,
            'tool_calls': [base_call],
        },
    }
    cases = [
        ('choice_index', {**base_choice, 'index': True}),
        ('choice_index', {**base_choice, 'index': 1}),
        ('finish_reason', {**base_choice, 'finish_reason': 'length'}),
        ('finish_reason', {**base_choice, 'finish_reason': 'content_filter'}),
        ('logprobs', {**base_choice, 'logprobs': {}}),
        ('choice_fields', {**base_choice, 'unexpected': None}),
    ]
    for reason, choice in cases:
        value, actual_reason = decode_completion_with_reason(envelope(choice))
        assert value is None
        assert actual_reason == reason

    reasoning_choice = {
        **base_choice,
        'message': {**base_choice['message'], 'reasoning_content': 'hidden'},
    }
    value, reason = decode_completion_with_reason(envelope(reasoning_choice))
    assert value is None
    assert reason == 'reasoning_content'


def test_decoder_rejects_ambiguous_or_untrusted_tool_call_envelopes():
    def envelope(message):
        return json.dumps({'choices': [{'message': message}]}).encode()

    call = {
        'type': 'function',
        'function': {
            'name': 'read_runtime_snapshot',
            'arguments': '{}',
        },
    }
    cases = [
        {
            'role': 'assistant',
            'content': '{}',
            'tool_calls': [call],
        },
        {
            'role': 'assistant',
            'content': None,
            'tool_calls': [call, call],
        },
        {
            'role': 'assistant',
            'content': None,
            'tool_calls': [{
                **call,
                'function': {**call['function'], 'name': 'unknown_tool'},
            }],
        },
        {
            'role': 'assistant',
            'content': None,
            'tool_calls': [{**call, 'type': 'custom'}],
        },
    ]
    for message in cases:
        assert decode_completion(envelope(message)) is None

    def arguments_envelope(arguments):
        return envelope({
            'role': 'assistant',
            'content': None,
            'tool_calls': [{
                'type': 'function',
                'function': {
                    'name': 'read_runtime_snapshot',
                    'arguments': arguments,
                },
            }],
        })

    assert decode_completion(arguments_envelope('[]')) is None
    assert decode_completion(arguments_envelope('{"value":NaN}')) is None
    assert decode_completion(arguments_envelope('{"value":1,"value":2}')) is None


def test_installed_system_prompt_is_versioned_and_closed():
    prompt = load_system_prompt()

    assert prompt.startswith('voice_nav.agent.system.v1')
    assert '/no_think' in prompt
    assert 'read_runtime_snapshot' in prompt
    assert 'propose_mission' in prompt
    assert 'cancel_owned_mission' in prompt
    assert 'ROS' in prompt
    assert 'shell' in prompt
    assert 'filesystem' in prompt
    assert '优先使用原生 tool_calls' in prompt
    assert '锁定 Qwen 的 content 表示' in prompt
    assert (
        '{"kind":"tool","tool_call":{"kind":"mission",'
        '"steps":[...]}}' in prompt
    )
    assert '禁止只输出 arguments' in prompt


def test_response_schema_has_closed_locked_qwen_tool_alternative():
    schema = load_response_schema()
    alternatives = [
        branch for branch in schema['oneOf']
        if branch.get('properties', {}).get('kind') == {'const': 'tool'}
    ]

    assert any(
        branch['properties']['tool_call']['required'] == ['kind', 'steps']
        and branch['additionalProperties'] is False
        and branch['properties']['tool_call']['additionalProperties'] is False
        and branch['properties']['tool_call']['properties']['kind'] == {
            'const': 'mission'
        }
        for branch in alternatives
    )
