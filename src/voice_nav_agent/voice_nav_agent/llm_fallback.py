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

"""Bounded localhost completion fallback for unsupported voice expressions."""

from __future__ import annotations

from dataclasses import dataclass
import http.client
import json
from pathlib import Path
import re
import threading
import time
from typing import Any, Callable, Optional

from .core import LLMNeededDecision, Mission, MissionProposal, MissionStep, SemanticValidator


_ENDPOINT_RE = re.compile(r'http://127\.0\.0\.1:([1-9][0-9]{0,4})')
_MAX_REQUEST_BYTES = 32 * 1024
_MAX_RESPONSE_BYTES = 64 * 1024
_DEADLINE_SECONDS = 10.0
_RESPONSE_PATH = '/v1/chat/completions'
_MISSION_SCHEMA_PATH = Path(__file__).with_name('schemas') / 'mission.schema.json'
_LOCKED_MODEL = 'Qwen3-0.6B-Q8_0.gguf'
_LOCKED_NON_THINKING = '/no_think'
_LOCKED_MAX_OUTPUT = 256


@dataclass(frozen=True, slots=True)
class LlmFallbackRequest:
    """One immutable fallback request, fenced by its Core planning token."""

    decision: LLMNeededDecision
    turn_generation: int


@dataclass(frozen=True, slots=True)
class LlmFallbackResult:
    """One closed completion result admitted before any ROS side effect."""

    kind: str
    mission: Optional[Mission] = None
    text: str = ''


@dataclass(frozen=True, slots=True)
class _Job:
    request: LlmFallbackRequest


class LlmFallback:
    """Submit at most one active and one latest pending local completion."""

    def __init__(
        self,
        endpoint: str,
        validator: SemanticValidator,
        serial_invoke: Callable[..., None],
        admission_fence: Callable[[LlmFallbackRequest], bool],
        on_result: Callable[[LlmFallbackRequest, LlmFallbackResult], None],
        on_failure: Callable[[LlmFallbackRequest], None],
    ) -> None:
        self._port = _parse_endpoint(endpoint)
        self._validator = validator
        self._serial_invoke = serial_invoke
        self._admission_fence = admission_fence
        self._on_result = on_result
        self._on_failure = on_failure
        self._schema = _load_schema()
        self._lock = threading.Lock()
        self._active: Optional[_Job] = None
        self._pending: Optional[_Job] = None
        self._connection: Optional[http.client.HTTPConnection] = None
        self._turn_generation = 0
        self._closed = False
        self._worker: Optional[threading.Thread] = None

    def submit(self, request: LlmFallbackRequest) -> None:
        """Queue a fenced completion request, retaining only the newest pending one."""
        if not isinstance(request, LlmFallbackRequest):
            raise TypeError('request must be LlmFallbackRequest')
        if not isinstance(request.decision, LLMNeededDecision):
            raise ValueError('request must carry LLM_NEEDED decision')
        if not isinstance(request.turn_generation, int) or request.turn_generation < 1:
            raise ValueError('turn_generation must be positive')

        start_worker = False
        with self._lock:
            if self._closed:
                return
            if request.turn_generation > self._turn_generation:
                self._invalidate_locked(request.turn_generation)
            if request.turn_generation != self._turn_generation:
                return
            job = _Job(request)
            if self._active is None:
                self._active = job
                start_worker = True
            else:
                self._pending = job
            if start_worker:
                self._worker = threading.Thread(
                    target=self._run, name='voice-nav-llm-fallback', daemon=True
                )
                self._worker.start()

    def invalidate(self, new_turn_generation: int) -> None:
        """Drop pending work and interrupt a prior generation's active socket."""
        if not isinstance(new_turn_generation, int) or new_turn_generation < 1:
            raise ValueError('new_turn_generation must be positive')
        with self._lock:
            if self._closed or new_turn_generation <= self._turn_generation:
                return
            self._invalidate_locked(new_turn_generation)

    def shutdown(self) -> None:
        """Invalidate, close the exact active socket, and join the daemon briefly."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._active = None
            self._pending = None
            connection = self._connection
            self._connection = None
            worker = self._worker
        _close_connection(connection)
        if worker is not None and worker is not threading.current_thread():
            worker.join(1.0)

    def _invalidate_locked(self, generation: int) -> None:
        self._turn_generation = generation
        self._pending = None
        _close_connection(self._connection)

    def _run(self) -> None:
        while True:
            with self._lock:
                job = self._active
            if job is None:
                return

            admitted, result = self._complete(job)
            if admitted and self._current(job):
                self._deliver(job, result)

            with self._lock:
                if self._active is job:
                    self._active = self._pending
                    self._pending = None
                if self._active is None:
                    self._worker = None
                    return

    def _deliver(self, job: _Job, result: Optional[LlmFallbackResult]) -> None:
        """Enter the Agent seam before sharing the fallback lock with shutdown."""
        self._serial_invoke(self._deliver_on_serial_seam, job, result)

    def _deliver_on_serial_seam(
        self, job: _Job, result: Optional[LlmFallbackResult]
    ) -> None:
        """Deliver only if this completion remains current inside the serial seam."""
        with self._lock:
            if not self._current_locked(job):
                return
            if result is None:
                self._on_failure(job.request)
            else:
                self._on_result(job.request, result)

    def _complete(self, job: _Job) -> tuple[bool, Optional[LlmFallbackResult]]:
        deadline = time.monotonic() + _DEADLINE_SECONDS
        try:
            request_body = _request_body(job.request.decision, self._schema)
            if len(request_body) > _MAX_REQUEST_BYTES:
                return True, None
            connection = http.client.HTTPConnection(
                '127.0.0.1', self._port, timeout=_remaining(deadline)
            )
            with self._lock:
                if not self._current_locked(job):
                    return False, None
                self._connection = connection
            connection.request(
                'POST',
                _RESPONSE_PATH,
                body=request_body,
                headers={
                    'Content-Type': 'application/json',
                    'Content-Length': str(len(request_body)),
                    'Accept': 'application/json',
                },
            )
            _set_socket_timeout(connection, deadline)
            response = connection.getresponse()
            if response.status != 200:
                return True, None
            content_length = _content_length(response)
            if content_length is None:
                return True, None
            raw = _read_response(response, connection, deadline)
            if len(raw) != content_length:
                return True, None
            if not self._serial_invoke(self._admission_fence, job.request):
                return False, None
            result = _decode_completion(raw, job.request.decision)
            if result is None:
                return True, None
            if result.kind != 'mission':
                return True, result
            proposal = MissionProposal(
                result.mission.steps, job.request.decision.token
            )
            validation = self._validator.validate(proposal, job.request.decision.token)
            if not validation.accepted:
                return True, None
            return True, LlmFallbackResult('mission', mission=validation.mission)
        except (OSError, ValueError, http.client.HTTPException, json.JSONDecodeError):
            return True, None
        finally:
            with self._lock:
                connection = self._connection
                self._connection = None
            _close_connection(connection)

    def _current(self, job: _Job) -> bool:
        with self._lock:
            return self._current_locked(job)

    def _current_locked(self, job: _Job) -> bool:
        return (
            not self._closed
            and self._active is job
            and job.request.turn_generation == self._turn_generation
        )


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


def _load_schema() -> dict[str, Any]:
    with _MISSION_SCHEMA_PATH.open(encoding='utf-8') as stream:
        schema = json.load(stream)
    if not isinstance(schema, dict):
        raise RuntimeError('mission schema must be an object')
    return schema


def _request_body(decision: LLMNeededDecision, schema: dict[str, Any]) -> bytes:
    payload = {
        'model': _LOCKED_MODEL,
        'messages': [
            {'role': 'system', 'content': _LOCKED_NON_THINKING},
            {
                'role': 'system',
                'content': (
                    'Return only a JSON Mission object that conforms to the '
                    'provided schema. Do not call tools or stream output.'
                ),
            },
            {
                'role': 'user',
                'content': json.dumps(
                    {
                        'text': decision.normalized_text,
                        'operating_mode': int(decision.token.operating_mode),
                        'named_place_ids': decision.token.named_place_ids,
                    },
                    ensure_ascii=False,
                    separators=(',', ':'),
                ),
            },
        ],
        'temperature': 0,
        'stream': False,
        'max_tokens': _LOCKED_MAX_OUTPUT,
        'response_format': {
            'type': 'json_schema',
            'schema': schema,
        },
    }
    return json.dumps(
        payload, ensure_ascii=False, separators=(',', ':'), allow_nan=False
    ).encode('utf-8')


def _remaining(deadline: float) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0.0:
        raise TimeoutError('completion deadline expired')
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
        raise OSError('completion socket was not connected')
    if sock.fileno() < 0:
        return
    sock.settimeout(_remaining(deadline))


def _read_response(
    response: http.client.HTTPResponse,
    connection: http.client.HTTPConnection,
    deadline: float,
) -> bytes:
    body = bytearray()
    while True:
        _set_socket_timeout(connection, deadline, response)
        chunk = response.read(min(8192, _MAX_RESPONSE_BYTES + 1 - len(body)))
        if not chunk:
            return bytes(body)
        body.extend(chunk)
        if len(body) > _MAX_RESPONSE_BYTES:
            raise ValueError('completion response is too large')


def _content_length(response: http.client.HTTPResponse) -> Optional[int]:
    if response.getheader('Transfer-Encoding') is not None:
        return None
    value = response.getheader('Content-Length')
    if value is None or not re.fullmatch(r'(?:0|[1-9][0-9]*)', value):
        return None
    length = int(value)
    return length if length <= _MAX_RESPONSE_BYTES else None


def _decode_completion(
    raw: bytes, decision: LLMNeededDecision
) -> Optional[LlmFallbackResult]:
    response = _json_object(raw)
    if response is None:
        return None
    choices = response.get('choices')
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
    result_json = _json_object(content.encode('utf-8'))
    if result_json is None or not isinstance(result_json.get('kind'), str):
        return None
    if result_json['kind'] == 'mission':
        if set(result_json) != {'kind', 'steps'}:
            return None
        steps = _decode_steps({'steps': result_json['steps']})
        if steps is None:
            return None
        return LlmFallbackResult(
            'mission', mission=Mission(tuple(steps), decision.token)
        )
    if result_json['kind'] == 'clarify' and set(result_json) == {
        'kind', 'prompt'
    }:
        text = result_json['prompt']
        if _bounded_text(text):
            return LlmFallbackResult('clarify', text=text)
    if result_json['kind'] == 'reply' and set(result_json) == {'kind', 'text'}:
        text = result_json['text']
        if _bounded_text(text):
            return LlmFallbackResult('reply', text=text)
    return None


def _json_object(raw: bytes) -> Optional[dict[str, Any]]:
    try:
        value = json.loads(
            raw.decode('utf-8'),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_non_finite,
        )
    except (UnicodeDecodeError, ValueError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def _reject_non_finite(value: str) -> None:
    raise ValueError(f'non-finite JSON value: {value}')


def _decode_steps(mission_json: dict[str, Any]) -> Optional[list[MissionStep]]:
    if set(mission_json) != {'steps'}:
        return None
    values = mission_json['steps']
    if not isinstance(values, list) or not 1 <= len(values) <= 3:
        return None
    steps = []
    for value in values:
        step = _decode_step(value)
        if step is None:
            return None
        steps.append(step)
    return steps


def _decode_step(value: Any) -> Optional[MissionStep]:
    if not isinstance(value, dict) or not isinstance(value.get('kind'), str):
        return None
    kind = value['kind']
    if kind == 'move_distance' and set(value) == {'kind', 'distance_m'}:
        distance = value['distance_m']
        if _number(distance):
            return MissionStep(MissionStep.MOVE_DISTANCE, distance_m=float(distance))
    elif kind == 'rotate_angle' and set(value) == {'kind', 'angle_rad'}:
        angle = value['angle_rad']
        if _number(angle):
            return MissionStep(MissionStep.ROTATE_ANGLE, angle_rad=float(angle))
    elif kind == 'navigate_to' and set(value) == {'kind', 'target_id'}:
        target = value['target_id']
        if isinstance(target, str):
            return MissionStep(MissionStep.NAVIGATE_TO, target_id=target)
    elif kind == 'save_map' and set(value) == {'kind', 'target_id'}:
        target = value['target_id']
        if isinstance(target, str):
            return MissionStep(MissionStep.SAVE_MAP, target_id=target)
    return None


def _number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _bounded_text(value: Any) -> bool:
    return isinstance(value, str) and 0 < len(value) <= 512


def _close_connection(connection: Optional[http.client.HTTPConnection]) -> None:
    if connection is None:
        return
    try:
        connection.close()
    except OSError:
        pass
