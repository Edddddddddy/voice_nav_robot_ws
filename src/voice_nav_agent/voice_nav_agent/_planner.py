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

"""The only PlannerPort implementations: deterministic fake and loopback HTTP."""

from __future__ import annotations

from dataclasses import dataclass
import http.client
import json
import re
import threading
import time
from typing import Callable, Optional, Protocol

from .core import MissionState, MissionStep, PlanningToken, VoiceTurn
from .planner_schema import (
    _json_object,
    AGENT_SYSTEM_VERSION,
    ALLOWED_TOOLS,
    decode_completion_with_reason,
    decode_planner_value_with_reason,
    decode_steps,
    load_mission_schema,
    load_response_schema,
    load_system_prompt,
    MAX_OUTPUT_TOKENS,
    MAX_REQUEST_BYTES,
    MAX_RESPONSE_BYTES,
    mission_schema_sha256,
    PLANNER_FAILURE_REASONS,
    SAFE_REPLY,
    TOOL_SCHEMA_VERSION,
)


_ENDPOINT_RE = re.compile(r'http://127\.0\.0\.1:([1-9][0-9]{0,4})')
_RESPONSE_PATH = '/v1/chat/completions'
_DEADLINE_SECONDS = 10.0
_LOCKED_MODEL = 'Qwen3-0.6B-Q8_0.gguf'
_MAX_CONTENT_TOOL_ID_CHARS = 128


@dataclass(frozen=True, slots=True)
class PlannerRequest:
    """Immutable planner input, including the complete Runtime snapshot."""

    turn: VoiceTurn
    token: PlanningToken
    runtime_snapshot: MissionState
    generation: int
    agent_generation: int = 1
    clarification: Optional[str] = None
    round: int = 1  # noqa: A003 - protocol field
    snapshot_output: Optional[dict[str, object]] = None


@dataclass(frozen=True, slots=True)
class PlannerResponse:
    """Closed response decoded by the Planner adapter."""

    kind: str
    text: str = ''
    steps: tuple[MissionStep, ...] = ()
    tool_name: str = ''
    tool_arguments: tuple[tuple[str, object], ...] = ()
    reason: str = ''

    @classmethod
    def invalid(cls, reason: str = 'transport') -> 'PlannerResponse':
        """Build the fail-closed transport/schema response."""
        if reason not in PLANNER_FAILURE_REASONS:
            reason = 'transport'
        return cls('invalid', text=SAFE_REPLY, reason=reason)

    @classmethod
    def mission(cls, steps: tuple[MissionStep, ...]) -> 'PlannerResponse':
        return cls('mission', steps=tuple(steps))

    @classmethod
    def tool(cls, name: str, arguments: Optional[dict[str, object]] = None) -> 'PlannerResponse':
        values = dict(arguments or {})
        if name == 'propose_mission' and isinstance(values.get('steps'), list):
            steps = decode_steps(values['steps'])
            if steps is not None:
                values['steps'] = steps
        return cls('tool', tool_name=name, tool_arguments=tuple(values.items()))


class PlannerPort(Protocol):
    """Single async seam consumed by AgentEngine."""

    def submit(
        self,
        request: PlannerRequest,
        done: Callable[[PlannerRequest, PlannerResponse], None],
    ) -> bool:
        ...

    def invalidate(self, generation: int) -> None:
        ...

    def shutdown(self) -> None:
        ...


class FakePlanner:
    """Deterministic capacity-one PlannerPort for behavior tests."""

    def __init__(self) -> None:
        self.requests: list[PlannerRequest] = []
        self.invalidations: list[int] = []
        self.completed: list[PlannerRequest] = []
        self.shutdown_called = False
        self._active: Optional[tuple[PlannerRequest, Callable[..., None]]] = None
        self._callbacks: dict[int, Callable[..., None]] = {}
        self._generation = 0
        self._lock = threading.Lock()

    def submit(self, request: PlannerRequest, done: Callable[..., None]) -> bool:
        if not isinstance(request, PlannerRequest):
            raise TypeError('request must be PlannerRequest')
        with self._lock:
            if self.shutdown_called or self._active is not None:
                return False
            if request.generation < self._generation:
                return False
            self._generation = request.generation
            self._active = (request, done)
            self._callbacks[id(request)] = done
            self.requests.append(request)
            return True

    def invalidate(self, generation: int) -> None:
        if not isinstance(generation, int) or generation < 1:
            raise ValueError('generation must be positive')
        with self._lock:
            if self.invalidations and generation <= self.invalidations[-1]:
                return
            self.invalidations.append(generation)
            self._active = None

    def complete(
        self,
        response: PlannerResponse | str,
        request: Optional[PlannerRequest] = None,
    ) -> bool:
        """Release the active request with a deterministic response."""
        with self._lock:
            active = self._active
            if active is None:
                return False
            current, done = active
            if request is not None and request is not current:
                return False
            self._active = None
            self.completed.append(current)
        if isinstance(response, str):
            response = PlannerResponse(response)
        done(current, response)
        return True

    def complete_late(
        self, request: PlannerRequest, response: PlannerResponse
    ) -> bool:
        """Invoke a retained callback after invalidation for stale-fence tests."""
        with self._lock:
            done = self._callbacks.get(id(request))
        if done is None:
            return False
        done(request, response)
        return True

    @property
    def active_request(self) -> Optional[PlannerRequest]:
        with self._lock:
            return self._active[0] if self._active is not None else None

    def shutdown(self) -> None:
        with self._lock:
            self.shutdown_called = True
            self._active = None


class LoopbackPlanner:
    """Production PlannerPort for one bounded localhost llama-server request."""

    def __init__(self, endpoint: str) -> None:
        self._port = _parse_endpoint(endpoint)
        self._lock = threading.Lock()
        self._active: Optional[PlannerRequest] = None
        self._connection: Optional[http.client.HTTPConnection] = None
        self._worker: Optional[threading.Thread] = None
        self._generation = 0
        self._closed = False
        self._prompt = load_system_prompt()
        self._mission_schema = load_mission_schema()
        self._response_schema = load_response_schema()
        self._prompt_digest = _sha256(self._prompt.encode('utf-8'))
        self._mission_schema_digest = mission_schema_sha256()

    @property
    def metadata(self) -> dict[str, str]:
        """Return version/digest evidence for the real-model gate."""
        return {
            'agent_system_version': AGENT_SYSTEM_VERSION,
            'tool_schema_version': TOOL_SCHEMA_VERSION,
            'prompt_sha256': self._prompt_digest,
            'mission_schema_sha256': self._mission_schema_digest,
        }

    def submit(self, request: PlannerRequest, done: Callable[..., None]) -> bool:
        if not isinstance(request, PlannerRequest):
            raise TypeError('request must be PlannerRequest')
        with self._lock:
            if self._closed or request.generation < self._generation:
                return False
            self._generation = request.generation
            if self._active is not None:
                return False
            self._active = request
            worker = threading.Thread(
                target=self._run,
                args=(request, done),
                name='voice-nav-loopback-planner',
                daemon=True,
            )
            self._worker = worker
        worker.start()
        return True

    def invalidate(self, generation: int) -> None:
        if not isinstance(generation, int) or generation < 1:
            raise ValueError('generation must be positive')
        with self._lock:
            if generation <= self._generation:
                return
            self._generation = generation
            connection = self._connection
            self._active = None
        _close_connection(connection)

    def shutdown(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._active = None
            connection = self._connection
            worker = self._worker
        _close_connection(connection)
        if worker is not None and worker is not threading.current_thread():
            worker.join(1.0)

    def _run(
        self, request: PlannerRequest, done: Callable[..., None]
    ) -> None:
        try:
            response = self._complete(request)
            with self._lock:
                deliver = (
                    not self._closed
                    and self._active is request
                    and request.generation == self._generation
                )
            if deliver:
                with self._lock:
                    if self._active is request:
                        self._active = None
                done(request, response)
        finally:
            with self._lock:
                if self._active is request:
                    self._active = None
                if self._worker is threading.current_thread():
                    self._worker = None

    def _complete(self, request: PlannerRequest) -> PlannerResponse:
        deadline = time.monotonic() + _DEADLINE_SECONDS
        connection: Optional[http.client.HTTPConnection] = None
        try:
            body = self._request_body(request)
            if len(body) > MAX_REQUEST_BYTES:
                return PlannerResponse.invalid('request_size')
            connection = http.client.HTTPConnection(
                '127.0.0.1', self._port, timeout=_remaining(deadline)
            )
            with self._lock:
                if (
                    self._closed
                    or self._active is not request
                    or request.generation != self._generation
                ):
                    return PlannerResponse.invalid('stale_request')
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
            if response.status != 200:
                return PlannerResponse.invalid('http_status')
            content_length = _content_length(response)
            if content_length is None:
                return PlannerResponse.invalid('response_length')
            raw = _read_response(response, connection, deadline)
            if len(raw) != content_length:
                return PlannerResponse.invalid('response_length')
            value, decode_reason = decode_completion_with_reason(raw)
            if value is None:
                return PlannerResponse.invalid(decode_reason)
            value = _normalize_content_tool_call(value)
            value, decode_reason = decode_planner_value_with_reason(value)
            if value is None:
                return PlannerResponse.invalid(decode_reason)
            if value['kind'] in ('reply', 'clarify'):
                return PlannerResponse(value['kind'], text=value['text'])
            if value['kind'] == 'mission':
                return PlannerResponse.mission(value['steps'])
            return PlannerResponse.tool(
                value['name'], value['arguments']
            )
        except (
            OSError,
            UnicodeError,
            ValueError,
            TimeoutError,
            http.client.HTTPException,
            json.JSONDecodeError,
        ):
            return PlannerResponse.invalid('transport')
        finally:
            with self._lock:
                if self._connection is connection:
                    self._connection = None
            _close_connection(connection)

    def _request_body(self, request: PlannerRequest) -> bytes:
        snapshot = request.runtime_snapshot
        snapshot_value = {
            'runtime_instance_id': snapshot.runtime_instance_id,
            'admission_epoch': snapshot.admission_epoch,
            'operating_mode': snapshot.operating_mode,
            'availability': snapshot.availability,
            'gate_state': snapshot.gate_state,
            'active_step': snapshot.active_step,
            'supported_step_mask': snapshot.supported_step_mask,
            'max_steps': snapshot.max_steps,
            'named_place_ids': list(snapshot.named_place_ids),
        }
        payload = {
            'model': _LOCKED_MODEL,
            'messages': [
                {'role': 'system', 'content': '/no_think'},
                {'role': 'system', 'content': self._prompt},
                {
                    'role': 'user',
                    'content': json.dumps(
                        {
                            'agent_system_version': AGENT_SYSTEM_VERSION,
                            'tool_schema_version': TOOL_SCHEMA_VERSION,
                            'mission_schema_sha256': self._mission_schema_digest,
                            'agent': {
                                'agent_generation': request.agent_generation,
                                'turn_generation': request.generation,
                            },
                            'runtime_snapshot': snapshot_value,
                            'turn': {
                                'voice_instance_id': request.turn.voice_instance_id,
                                'voice_seq': request.turn.voice_seq,
                                'session_id': request.turn.session_id,
                                'turn_id': request.turn.turn_id,
                                'text': request.turn.text,
                            },
                            'clarification': request.clarification,
                            'round': request.round,
                            'snapshot_output': request.snapshot_output,
                        },
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
                        'name': name,
                        'parameters': self._tool_parameters(name),
                        'strict': True,
                    },
                }
                for name in ALLOWED_TOOLS
            ],
            'temperature': 0,
            'stream': False,
            'parallel_tool_calls': False,
            'max_tokens': MAX_OUTPUT_TOKENS,
            'response_format': {
                'type': 'json_schema',
                'schema': self._response_schema,
            },
        }
        return json.dumps(
            payload,
            ensure_ascii=False,
            separators=(',', ':'),
            allow_nan=False,
        ).encode('utf-8')

    def _tool_parameters(self, name: str) -> dict[str, object]:
        if name == 'propose_mission':
            return self._mission_schema['oneOf'][0]
        return {'type': 'object', 'properties': {}, 'additionalProperties': False}


def _parse_endpoint(endpoint: str) -> int:
    if not isinstance(endpoint, str):
        raise ValueError('endpoint must be a literal localhost URL')
    match = _ENDPOINT_RE.fullmatch(endpoint)
    if match is None:
        raise ValueError('endpoint must be http://127.0.0.1:<port>')
    port = int(match.group(1))
    if not 1 <= port <= 65535:
        raise ValueError('endpoint port is out of range')
    return port


def _normalize_content_tool_call(value: object) -> object:
    """Normalize only the OpenAI tool envelope embedded in content JSON."""
    if not isinstance(value, dict) or value.get('kind') != 'tool':
        return value
    call = value.get('tool_call')
    if not isinstance(call, dict):
        return value
    fields = set(call)
    if fields == {'kind', 'steps'}:
        if call.get('kind') != 'mission':
            return value
        steps = call.get('steps')
        if decode_steps(steps) is None:
            return value
        return {
            **value,
            'tool_call': {
                'name': 'propose_mission',
                'arguments': {
                    'kind': 'mission',
                    'steps': steps,
                },
            },
        }
    if fields == {'name', 'arguments'}:
        name = call['name']
        arguments = call['arguments']
    elif fields in ({'type', 'function'}, {'id', 'type', 'function'}):
        if call.get('type') != 'function':
            return value
        if 'id' in call and not _bounded_content_tool_id(call['id']):
            return value
        function = call['function']
        if not isinstance(function, dict) or set(function) != {
            'name', 'arguments'
        }:
            return value
        name = function['name']
        arguments = function['arguments']
    else:
        return value
    normalized_arguments = _normalize_content_tool_arguments(arguments)
    if normalized_arguments is None:
        return value
    return {
        **value,
        'tool_call': {
            'name': name,
            'arguments': normalized_arguments,
        },
    }


def _normalize_content_tool_arguments(value: object) -> Optional[dict[str, object]]:
    if isinstance(value, dict):
        return value
    if not isinstance(value, str):
        return None
    try:
        return _json_object(value.encode('utf-8'))
    except UnicodeError:
        return None


def _bounded_content_tool_id(value: object) -> bool:
    return (
        isinstance(value, str)
        and 0 < len(value) <= _MAX_CONTENT_TOOL_ID_CHARS
    )


def _remaining(deadline: float) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0.0:
        raise TimeoutError('planner deadline expired')
    return remaining


def _set_socket_timeout(
    connection: http.client.HTTPConnection,
    deadline: float,
    response: Optional[http.client.HTTPResponse] = None,
) -> None:
    sock = connection.sock
    if sock is None and response is not None:
        raw = getattr(getattr(response, 'fp', None), 'raw', None)
        sock = getattr(raw, '_sock', None)
    if sock is None and response is not None:
        return
    if sock is None:
        raise OSError('planner socket is not connected')
    if sock.fileno() >= 0:
        sock.settimeout(_remaining(deadline))


def _read_response(
    response: http.client.HTTPResponse,
    connection: http.client.HTTPConnection,
    deadline: float,
) -> bytes:
    body = bytearray()
    while True:
        _set_socket_timeout(connection, deadline, response)
        chunk = response.read(min(8192, MAX_RESPONSE_BYTES + 1 - len(body)))
        if not chunk:
            return bytes(body)
        body.extend(chunk)
        if len(body) > MAX_RESPONSE_BYTES:
            raise ValueError('planner response is too large')


def _content_length(response: http.client.HTTPResponse) -> Optional[int]:
    if response.getheader('Transfer-Encoding') is not None:
        return None
    value = response.getheader('Content-Length')
    if value is None or not re.fullmatch(r'(?:0|[1-9][0-9]*)', value):
        return None
    length = int(value)
    return length if length <= MAX_RESPONSE_BYTES else None


def _close_connection(connection: Optional[http.client.HTTPConnection]) -> None:
    if connection is None:
        return
    try:
        connection.close()
    except OSError:
        pass


def _sha256(value: bytes) -> str:
    import hashlib

    return hashlib.sha256(value).hexdigest()


# Friendly aliases used by package-private contract tests.
_PlannerRequest = PlannerRequest
_PlannerResponse = PlannerResponse
_FakePlanner = FakePlanner
_LoopbackPlanner = LoopbackPlanner
