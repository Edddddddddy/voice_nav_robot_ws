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

"""The single closed schema/decoder boundary for local Agent planning."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Optional

from .core import MissionStep


AGENT_SYSTEM_VERSION = 'voice_nav.agent.system.v1'
TOOL_SCHEMA_VERSION = 'voice_nav.agent.tools.v1'
MAX_REQUEST_BYTES = 32 * 1024
MAX_RESPONSE_BYTES = 64 * 1024
MAX_OUTPUT_TOKENS = 256
SAFE_REPLY = '当前无法处理该导航请求。'
ALLOWED_TOOLS = (
    'read_runtime_snapshot',
    'propose_mission',
    'cancel_owned_mission',
)
DECODER_REASON_CODES = frozenset({
    'outer_json',
    'choices',
    'choice_shape',
    'choice_fields',
    'choice_index',
    'finish_reason',
    'logprobs',
    'message_shape',
    'message_fields',
    'reasoning_content',
    'content',
    'tool_calls',
    'tool_call_shape',
    'tool_call_type',
    'tool_call_function',
    'tool_name',
    'tool_arguments_type',
    'tool_arguments_json',
    'inner_json',
})
PLANNER_VALUE_REASON_CODES = frozenset({
    'ok',
    'root',
    'kind',
    'missing_kind_steps_only',
    'missing_kind_name_arguments',
    'missing_kind_tool_call_only',
    'missing_kind_other',
    'kind_non_string',
    'kind_unknown',
    'reply_fields',
    'text',
    'mission_fields',
    'steps_type',
    'steps_count',
    'step_shape',
    'step_kind',
    'step_fields',
    'step_value',
    'tool_fields',
    'call_shape',
    'tool_name',
    'empty_args',
    'mission_args_fields',
    'mission_args_kind',
    'mission_args_steps',
})
PLANNER_FAILURE_REASONS = DECODER_REASON_CODES | PLANNER_VALUE_REASON_CODES | frozenset({
    'transport',
    'request_size',
    'stale_request',
    'http_status',
    'response_length',
    'inner_schema',
})
_ALLOWED_FINISH_REASONS = frozenset({'stop', 'tool_calls'})
_MAX_FINISH_REASON_CHARS = 32

_PACKAGE_ROOT = Path(__file__).resolve().parent
MISSION_SCHEMA_PATH = _PACKAGE_ROOT / 'schemas' / 'mission.schema.json'
RESPONSE_SCHEMA_PATH = _PACKAGE_ROOT / 'schemas' / 'agent_response.schema.json'
PROMPT_PATH = _PACKAGE_ROOT / 'prompts' / 'agent_system_v1.txt'

TOOL_SCHEMA = {
    'type': 'object',
    'properties': {},
    'additionalProperties': False,
}


def _load_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f'cannot load planner schema: {path}') from error
    if not isinstance(value, dict):
        raise RuntimeError(f'planner schema must be an object: {path}')
    return value


def load_mission_schema() -> dict[str, Any]:
    """Load the only Mission proposal schema used by all planner paths."""
    return _load_object(MISSION_SCHEMA_PATH)


def load_response_schema() -> dict[str, Any]:
    """Load the strict completion envelope schema."""
    return _load_object(RESPONSE_SCHEMA_PATH)


def load_system_prompt() -> str:
    """Load the versioned non-thinking system prompt as installed data."""
    try:
        prompt = PROMPT_PATH.read_text(encoding='utf-8')
    except (OSError, UnicodeDecodeError) as error:
        raise RuntimeError(f'cannot load Agent system prompt: {PROMPT_PATH}') from error
    if not prompt or not prompt.endswith('\n'):
        raise RuntimeError('Agent system prompt must be non-empty and newline terminated')
    return prompt


def _json_object(raw: bytes) -> Optional[dict[str, Any]]:
    """Decode one finite UTF-8 JSON object, rejecting duplicate keys."""
    if not isinstance(raw, bytes) or not 0 < len(raw) <= MAX_RESPONSE_BYTES:
        return None
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
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def _reject_non_finite(value: str) -> None:
    raise ValueError(f'non-finite JSON value: {value}')


def decode_steps(value: object) -> Optional[tuple[MissionStep, ...]]:
    """Decode a closed Mission steps array without performing semantic checks."""
    steps, _reason = _decode_steps_with_reason(value)
    return steps


def _decode_steps_with_reason(
    value: object,
) -> tuple[Optional[tuple[MissionStep, ...]], str]:
    if not isinstance(value, list):
        return None, 'steps_type'
    if not 1 <= len(value) <= 3:
        return None, 'steps_count'
    steps: list[MissionStep] = []
    for item in value:
        step, reason = _decode_step_with_reason(item)
        if step is None:
            return None, reason
        steps.append(step)
    return tuple(steps), 'ok'


def _decode_step(value: object) -> Optional[MissionStep]:
    step, _reason = _decode_step_with_reason(value)
    return step


def _decode_step_with_reason(
    value: object,
) -> tuple[Optional[MissionStep], str]:
    if not isinstance(value, dict):
        return None, 'step_shape'
    if not isinstance(value.get('kind'), str):
        return None, 'step_kind'
    kind = value['kind']
    if kind == 'move_distance':
        if set(value) != {'kind', 'distance_m'}:
            return None, 'step_fields'
        distance = value['distance_m']
        if _number(distance):
            return MissionStep(
                MissionStep.MOVE_DISTANCE, distance_m=float(distance)
            ), 'ok'
        return None, 'step_value'
    if kind == 'rotate_angle':
        if set(value) != {'kind', 'angle_rad'}:
            return None, 'step_fields'
        angle = value['angle_rad']
        if _number(angle):
            return MissionStep(
                MissionStep.ROTATE_ANGLE, angle_rad=float(angle)
            ), 'ok'
        return None, 'step_value'
    if kind == 'navigate_to':
        if set(value) != {'kind', 'target_id'}:
            return None, 'step_fields'
        target = value['target_id']
        if isinstance(target, str):
            return MissionStep(
                MissionStep.NAVIGATE_TO, target_id=target
            ), 'ok'
        return None, 'step_value'
    if kind == 'save_map':
        if set(value) != {'kind', 'target_id'}:
            return None, 'step_fields'
        target = value['target_id']
        if isinstance(target, str):
            return MissionStep(
                MissionStep.SAVE_MAP, target_id=target
            ), 'ok'
        return None, 'step_value'
    return None, 'step_kind'


def _number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def decode_completion(raw: bytes) -> Optional[dict[str, Any]]:
    """Decode the OpenAI-compatible outer envelope and one strict inner object."""
    value, _reason = decode_completion_with_reason(raw)
    return value


def decode_completion_with_reason(
    raw: bytes,
) -> tuple[Optional[dict[str, Any]], str]:
    """Decode one completion and return only a bounded failure reason."""
    outer = _json_object(raw)
    if outer is None:
        return None, 'outer_json'
    choices = outer.get('choices')
    if not isinstance(choices, list) or len(choices) != 1:
        return None, 'choices'
    choice = choices[0]
    if not isinstance(choice, dict):
        return None, 'choice_shape'
    if set(choice) - {
        'message', 'finish_reason', 'index', 'logprobs'
    }:
        return None, 'choice_fields'
    if 'index' in choice and (
        type(choice['index']) is not int or choice['index'] != 0
    ):
        return None, 'choice_index'
    if 'finish_reason' in choice:
        finish_reason = choice['finish_reason']
        if (
            not isinstance(finish_reason, str)
            or not 1 <= len(finish_reason) <= _MAX_FINISH_REASON_CHARS
            or finish_reason not in _ALLOWED_FINISH_REASONS
        ):
            return None, 'finish_reason'
    if 'logprobs' in choice and choice['logprobs'] is not None:
        return None, 'logprobs'
    message = choice.get('message')
    if not isinstance(message, dict):
        return None, 'message_shape'
    if set(message) - {
        'role', 'content', 'reasoning_content', 'tool_calls'
    }:
        return None, 'message_fields'
    if (
        'reasoning_content' in message
        and message['reasoning_content'] is not None
    ):
        return None, 'reasoning_content'
    if 'tool_calls' in message:
        if 'content' not in message or message['content'] is not None:
            return None, 'content'
        tool_calls = message['tool_calls']
        if not isinstance(tool_calls, list) or len(tool_calls) != 1:
            return None, 'tool_calls'
        call = tool_calls[0]
        if not isinstance(call, dict) or set(call) - {
            'id', 'type', 'function'
        }:
            return None, 'tool_call_shape'
        if call.get('type') != 'function':
            return None, 'tool_call_type'
        if 'id' in call and not isinstance(call['id'], str):
            return None, 'tool_call_shape'
        function = call.get('function')
        if not isinstance(function, dict) or set(function) != {
            'name', 'arguments'
        }:
            return None, 'tool_call_function'
        name = function['name']
        arguments_raw = function['arguments']
        if name not in ALLOWED_TOOLS:
            return None, 'tool_name'
        if not isinstance(arguments_raw, str):
            return None, 'tool_arguments_type'
        arguments = _json_object(arguments_raw.encode('utf-8'))
        if arguments is None:
            return None, 'tool_arguments_json'
        return {
            'kind': 'tool',
            'tool_call': {'name': name, 'arguments': arguments},
        }, 'ok'
    content = message.get('content')
    if not isinstance(content, str) or '<think>' in content or '</think>' in content:
        return None, 'content'
    inner = _json_object(content.encode('utf-8'))
    if inner is None:
        return None, 'inner_json'
    return inner, 'ok'


def decode_planner_value(value: object) -> Optional[dict[str, Any]]:
    """Validate one strict inner Agent completion and normalize its fields."""
    decoded, _reason = decode_planner_value_with_reason(value)
    return decoded


def decode_planner_value_with_reason(
    value: object,
) -> tuple[Optional[dict[str, Any]], str]:
    """Normalize one inner completion with a bounded schema reason."""
    if not isinstance(value, dict):
        return None, 'root'
    if 'kind' not in value:
        keys = set(value)
        if keys == {'steps'}:
            return None, 'missing_kind_steps_only'
        if keys == {'name', 'arguments'}:
            return None, 'missing_kind_name_arguments'
        if keys == {'tool_call'}:
            return None, 'missing_kind_tool_call_only'
        return None, 'missing_kind_other'
    if not isinstance(value['kind'], str):
        return None, 'kind_non_string'
    kind = value['kind']
    if kind in ('reply', 'clarify'):
        if set(value) != {'kind', 'text'}:
            return None, 'reply_fields'
        if not _bounded_text(value['text']):
            return None, 'text'
        return {'kind': kind, 'text': value['text']}, 'ok'
    if kind == 'mission':
        if set(value) != {'kind', 'steps'}:
            return None, 'mission_fields'
        steps, reason = _decode_steps_with_reason(value['steps'])
        if steps is None:
            return None, reason
        return {'kind': kind, 'steps': steps}, 'ok'
    if kind == 'tool':
        if set(value) != {'kind', 'tool_call'}:
            return None, 'tool_fields'
        call = value['tool_call']
        if not isinstance(call, dict) or set(call) != {'name', 'arguments'}:
            return None, 'call_shape'
        name = call['name']
        arguments = call['arguments']
        if not isinstance(name, str) or name not in ALLOWED_TOOLS:
            return None, 'tool_name'
        if name in ('read_runtime_snapshot', 'cancel_owned_mission'):
            if arguments != {}:
                return None, 'empty_args'
        else:
            if type(arguments) is not dict:
                return None, 'mission_args_fields'
            if set(arguments) != {'kind', 'steps'}:
                return None, 'mission_args_fields'
            if arguments.get('kind') != 'mission':
                return None, 'mission_args_kind'
            steps, _reason = _decode_steps_with_reason(arguments.get('steps'))
            if steps is None:
                return None, 'mission_args_steps'
        return {'kind': kind, 'name': name, 'arguments': arguments}, 'ok'
    return None, 'kind_unknown'


def _bounded_text(value: object) -> bool:
    return isinstance(value, str) and 0 < len(value) <= 512


def mission_schema_sha256() -> str:
    """Return the immutable Mission schema digest recorded in the prompt."""
    import hashlib

    return hashlib.sha256(MISSION_SCHEMA_PATH.read_bytes()).hexdigest()
