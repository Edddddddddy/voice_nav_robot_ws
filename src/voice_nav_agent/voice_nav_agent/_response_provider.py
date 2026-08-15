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

"""Literal loopback transport for the package-private Response session."""

from __future__ import annotations

import http.client
import json
import threading
import time
from typing import Callable, Optional

from ._response_session import (
    _ProviderResponse,
    _ResponseRequest,
    _ToolCall,
    _ToolDefinition,
)
from .llm_fallback import (
    _close_connection,
    _content_length,
    _DEADLINE_SECONDS,
    _json_object,
    _LOCKED_MAX_OUTPUT,
    _LOCKED_MODEL,
    _LOCKED_NON_THINKING,
    _MAX_REQUEST_BYTES,
    _parse_endpoint,
    _read_response,
    _remaining,
    _RESPONSE_PATH,
    _set_socket_timeout,
)


_CONTENT_SCHEMA = {
    'oneOf': [
        {
            'type': 'object',
            'properties': {
                'kind': {'const': 'clarify'},
                'text': {'type': 'string', 'minLength': 1, 'maxLength': 512},
            },
            'required': ['kind', 'text'],
            'additionalProperties': False,
        },
        {
            'type': 'object',
            'properties': {
                'kind': {'const': 'reply'},
                'text': {'type': 'string', 'minLength': 1, 'maxLength': 512},
            },
            'required': ['kind', 'text'],
            'additionalProperties': False,
        },
        {
            'type': 'object',
            'properties': {
                'kind': {'const': 'tool'},
                'tool_call': {
                    'type': 'object',
                    'properties': {
                        'name': {
                            'enum': [
                                'read_runtime_snapshot',
                                'propose_mission',
                                'cancel_owned_mission',
                            ]
                        },
                        'arguments': {'type': 'object'},
                    },
                    'required': ['name', 'arguments'],
                    'additionalProperties': False,
                },
            },
            'required': ['kind', 'tool_call'],
            'additionalProperties': False,
        },
    ]
}


class _LoopbackResponseProvider:
    """Run at most one literal-loopback Response request without a queue."""

    def __init__(
        self,
        endpoint: str,
        tool_registry: tuple[_ToolDefinition, ...],
        on_response: Callable[[_ResponseRequest, _ProviderResponse], None],
        on_failure: Callable[[_ResponseRequest], None],
        on_capacity_ready: Callable[[], None] = lambda: None,
    ) -> None:
        self._port = _parse_endpoint(endpoint)
        self._tool_registry = tool_registry
        self._on_response = on_response
        self._on_failure = on_failure
        self._on_capacity_ready = on_capacity_ready
        self._lock = threading.Lock()
        self._transport_gate = threading.Lock()
        self._active: Optional[_ResponseRequest] = None
        self._connection: Optional[http.client.HTTPConnection] = None
        self._worker: Optional[threading.Thread] = None
        self._started_worker: Optional[threading.Thread] = None
        self._generation = 0
        self._closed = False

    def submit(self, request: _ResponseRequest) -> bool:
        """Start the one request that ResponseSession has already admitted."""
        if not isinstance(request, _ResponseRequest):
            raise TypeError('request must be a Response request')
        with self._lock:
            if (
                self._closed
                or request.adapter_generation != self._generation
                or self._active is not None
            ):
                return False
            self._active = request
            worker = threading.Thread(
                target=self._run,
                args=(request,),
                name='voice-nav-response-provider',
                daemon=True,
            )
            self._worker = worker
        worker.start()
        return True

    def invalidate(self, generation: int) -> None:
        """Advance the accepted-turn high-watermark and interrupt one socket."""
        if not isinstance(generation, int) or generation < 1:
            raise ValueError('generation must be positive')
        with self._lock:
            if generation <= self._generation:
                return
            self._generation = generation
            connection = self._connection
        _close_connection(connection)
        # Both teardown paths acquire this gate after publishing their fence.
        # A worker holds it from its final currentness check through the first
        # transport call, so request admission and teardown have one order.
        with self._transport_gate:
            pass

    def shutdown(self) -> None:
        """Prevent post-shutdown delivery and briefly join the one worker."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            connection = self._connection
        _close_connection(connection)
        with self._transport_gate:
            pass
        with self._lock:
            worker = self._started_worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(1.0)

    def _run(self, request: _ResponseRequest) -> None:
        """Register a started worker before it can enter transport work."""
        worker = threading.current_thread()
        with self._lock:
            run = (
                not self._closed
                and self._active is request
                and request.adapter_generation == self._generation
            )
            if run:
                self._started_worker = worker
            else:
                if self._active is request:
                    self._active = None
                if self._worker is worker:
                    self._worker = None
        if not run:
            self._notify_capacity_ready()
            return
        try:
            response = self._complete(request)
            with self._lock:
                deliver = (
                    not self._closed
                    and self._active is request
                    and request.adapter_generation == self._generation
                )
            if deliver:
                if response is None:
                    self._on_failure(request)
                else:
                    self._on_response(request, response)
        finally:
            try:
                self._release_worker_capacity(request, worker)
                self._notify_capacity_ready()
            finally:
                self._unregister_worker(worker)

    def _release_worker_capacity(
        self, request: _ResponseRequest, worker: threading.Thread
    ) -> None:
        """Release capacity after delivery while retaining worker registration."""
        with self._lock:
            if self._active is request:
                self._active = None
            if self._worker is worker:
                self._worker = None

    def _unregister_worker(self, worker: threading.Thread) -> None:
        """Clear only this worker's registration after all cleanup callbacks."""
        with self._lock:
            if self._started_worker is worker:
                self._started_worker = None

    def _notify_capacity_ready(self) -> None:
        """Wake the Session only after releasing the provider lock."""
        with self._lock:
            ready = not self._closed and self._active is None
        if ready:
            self._on_capacity_ready()

    def _complete(self, request: _ResponseRequest) -> Optional[_ProviderResponse]:
        deadline = time.monotonic() + _DEADLINE_SECONDS
        connection: Optional[http.client.HTTPConnection] = None
        try:
            body = _request_body(request, self._tool_registry)
            if len(body) > _MAX_REQUEST_BYTES:
                return None
            connection = http.client.HTTPConnection(
                '127.0.0.1', self._port, timeout=_remaining(deadline)
            )
            with self._transport_gate:
                with self._lock:
                    if (
                        self._closed
                        or self._active is not request
                        or request.adapter_generation != self._generation
                    ):
                        return None
                    self._connection = connection
                connection.request(
                    'POST',
                    _RESPONSE_PATH,
                    body=body,
                    headers={
                        'Content-Type': 'application/json',
                        'Content-Length': str(len(body)),
                        'Accept': 'application/json',
                    },
                )
            _set_socket_timeout(connection, deadline)
            response = connection.getresponse()
            if response.status != 200 or _content_length(response) is None:
                return None
            raw = _read_response(response, connection, deadline)
            return _decode_response(raw)
        except (
            OSError,
            UnicodeError,
            ValueError,
            http.client.HTTPException,
            json.JSONDecodeError,
        ):
            return None
        finally:
            with self._lock:
                if self._connection is connection:
                    self._connection = None
            _close_connection(connection)


def _request_body(
    request: _ResponseRequest,
    request_tools: tuple[_ToolDefinition, ...],
) -> bytes:
    """Encode one bounded turn with the frozen inner protocol schema."""
    snapshot_output = None
    if request.snapshot_output is not None:
        snapshot_output = dict(request.snapshot_output.value)
    content = {
        'turn': {
            'voice_instance_id': request.turn.voice_instance_id,
            'voice_seq': request.turn.voice_seq,
            'session_id': request.turn.session_id,
            'turn_id': request.turn.turn_id,
            'text': request.turn.text,
        },
        'clarification': request.clarification,
        'round': request.round,
        'snapshot_output': snapshot_output,
    }
    payload = {
        'model': _LOCKED_MODEL,
        'messages': [
            {'role': 'system', 'content': _LOCKED_NON_THINKING},
            {
                'role': 'system',
                'content': (
                    'Return only one JSON object matching the response schema. '
                    'Use at most one listed semantic tool call.'
                ),
            },
            {
                'role': 'user',
                'content': json.dumps(
                    content,
                    ensure_ascii=False,
                    separators=(',', ':'),
                    allow_nan=False,
                ),
            },
        ],
        'tools': [
            {
                'type': 'function',
                'function': {
                    'name': tool.name,
                    'parameters': tool.parameters,
                },
            }
            for tool in request_tools
        ],
        'temperature': 0,
        'stream': False,
        'max_tokens': _LOCKED_MAX_OUTPUT,
        'response_format': {'type': 'json_schema', 'schema': _CONTENT_SCHEMA},
    }
    return json.dumps(
        payload, ensure_ascii=False, separators=(',', ':'), allow_nan=False
    ).encode('utf-8')


def _decode_response(raw: bytes) -> Optional[_ProviderResponse]:
    """Decode exactly one OpenAI-compatible envelope and closed inner value."""
    outer = _json_object(raw)
    if outer is None:
        return None
    choices = outer.get('choices')
    if not isinstance(choices, list) or len(choices) != 1:
        return None
    choice = choices[0]
    if not isinstance(choice, dict):
        return None
    message = choice.get('message')
    if not isinstance(message, dict):
        return None
    content = message.get('content')
    if not isinstance(content, str):
        return None
    inner = _json_object(content.encode('utf-8'))
    if inner is None or not isinstance(inner.get('kind'), str):
        return None
    kind = inner['kind']
    if kind in ('clarify', 'reply'):
        if set(inner) != {'kind', 'text'} or not _bounded_text(inner['text']):
            return None
        return _ProviderResponse(kind=kind, text=inner['text'])
    if kind != 'tool' or set(inner) != {'kind', 'tool_call'}:
        return None
    tool_call = inner['tool_call']
    if not isinstance(tool_call, dict) or set(tool_call) != {'name', 'arguments'}:
        return None
    if tool_call['name'] not in {
        'read_runtime_snapshot',
        'propose_mission',
        'cancel_owned_mission',
    } or type(tool_call['arguments']) is not dict:
        return None
    return _ProviderResponse(
        kind='tool',
        tool_calls=(_ToolCall(tool_call['name'], tool_call['arguments']),),
    )


def _bounded_text(value: object) -> bool:
    return isinstance(value, str) and 0 < len(value) <= 512
