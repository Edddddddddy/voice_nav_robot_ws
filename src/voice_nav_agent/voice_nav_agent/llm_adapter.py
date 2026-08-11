"""Loopback-only llama-server adapter for bounded Agent fallback planning."""

import json
import math
import re
from urllib.parse import urlparse
from urllib.request import ProxyHandler, Request, build_opener


_SYSTEM_PROMPT = (
    '你是机器人指令解析器。只分析用户 JSON 中的 text，并只返回一个 JSON '
    '对象。text 没有明确的移动、旋转、地点导航或保存地图动作时必须返回 reply；'
    '有动作但缺少距离、角度、地点或地图名时返回 clarify；参数完整时才返回 '
    'mission。每个明确说出的动作只生成一个 step，保持口述顺序，绝不增加准备、'
    '对齐或接近动作，也不要猜测“一点、一下”等模糊数值。距离单位为米；角度必须'
    '转换成弧度，左转/逆时针为正，右转/顺时针为负：45 度=0.7853981634，'
    '90 度=1.5707963268，180 度=3.1415926536。NAVIGATE_TO 的 target_id '
    '只能逐字选自 named_place_ids；常用含义为 home=家/起点、study=书房、'
    'kitchen=厨房/做饭的地方。SAVE_MAP 使用用户明确说出的合法地图 ID。'
    '不要编造 ID、动作或控制参数。'
)


class LoopbackLlm:
    """Call a local llama-server without proxy inheritance."""

    def __init__(self, endpoint, timeout_s=10.0):
        """Validate and retain the loopback endpoint and steady deadline."""
        parsed = urlparse(endpoint)
        if parsed.scheme != 'http' or parsed.hostname != '127.0.0.1':
            raise ValueError('LLM endpoint must be loopback HTTP')
        self.endpoint = endpoint
        self.timeout_s = timeout_s
        self.opener = build_opener(ProxyHandler({}))

    def plan(self, text, mode, supported_step_mask, max_steps, places):
        """Return a closed-schema mission or reply from llama-server."""
        place_ids = tuple(dict.fromkeys(places))
        intent_kinds = _intent_kinds(text)
        step_schemas = _step_schemas(
            mode, supported_step_mask, place_ids, intent_kinds
        )
        variants = [_text_schema('clarify'), _text_schema('reply')]
        if step_schemas:
            variants.insert(0, _mission_schema(step_schemas, max_steps))
        response_schema = {'type': 'object', 'oneOf': variants}
        user_request = {
            'text': text,
            'mode': 'mapping' if mode == 1 else 'navigation',
            'supported_kinds': [schema['properties']['kind']['const']
                                for schema in step_schemas],
            'max_steps': max_steps,
            'named_place_ids': place_ids,
        }
        payload = json.dumps(
            {
                'model': 'Qwen3-0.6B-Q8_0.gguf',
                'messages': [
                    {'role': 'system', 'content': _SYSTEM_PROMPT},
                    {
                        'role': 'user',
                        'content': '/no_think\n' + json.dumps(
                            user_request, ensure_ascii=False, separators=(',', ':')
                        ),
                    },
                ],
                'temperature': 0,
                'max_tokens': 256,
                'stream': False,
                'response_format': {
                    'type': 'json_schema',
                    'json_schema': {
                        'name': 'voice_nav_plan',
                        'strict': True,
                        'schema': response_schema,
                    },
                },
            }
        ).encode()
        request = Request(
            self.endpoint,
            data=payload,
            headers={'Content-Type': 'application/json'},
        )
        with self.opener.open(request, timeout=self.timeout_s) as response:
            content = json.load(response)['choices'][0]['message']['content']
        result = json.loads(content)
        if not _valid_result(result, step_schemas, max_steps):
            raise ValueError('LLM response outside closed schema')
        return result


def _text_schema(kind):
    return {
        'properties': {
            kind: {
                'type': 'object',
                'properties': {'text': {'type': 'string', 'minLength': 1}},
                'required': ['text'],
                'additionalProperties': False,
            }
        },
        'required': [kind],
        'additionalProperties': False,
    }


def _step_schemas(mode, mask, places, intent_kinds):
    schemas = []
    if mask & 0x01 and 'MOVE_DISTANCE' in intent_kinds:
        schemas.append(
            _numeric_step_schema('MOVE_DISTANCE', 'distance_m', -2.0, 2.0)
        )
    if mask & 0x02 and 'ROTATE_ANGLE' in intent_kinds:
        schemas.append(
            _numeric_step_schema(
                'ROTATE_ANGLE', 'angle_rad', -math.pi, math.pi
            )
        )
    if (
        mode == 2
        and mask & 0x04
        and places
        and 'NAVIGATE_TO' in intent_kinds
    ):
        schemas.append(_target_step_schema('NAVIGATE_TO', {'enum': places}))
    if mode == 1 and mask & 0x08 and 'SAVE_MAP' in intent_kinds:
        schemas.append(
            _target_step_schema(
                'SAVE_MAP',
                {'type': 'string', 'pattern': '^[a-z][a-z0-9_-]{0,31}$'},
            )
        )
    return schemas


def _numeric_step_schema(kind, field, minimum, maximum):
    return {
        'properties': {
            'kind': {'const': kind},
            field: {
                'type': 'number',
                'minimum': minimum,
                'maximum': maximum,
            },
        },
        'required': ['kind', field],
        'additionalProperties': False,
    }


def _target_step_schema(kind, target_schema):
    return {
        'properties': {
            'kind': {'const': kind},
            'target_id': target_schema,
        },
        'required': ['kind', 'target_id'],
        'additionalProperties': False,
    }


def _mission_schema(step_schemas, max_steps):
    return {
        'properties': {
            'mission': {
                'type': 'object',
                'properties': {
                    'steps': {
                        'type': 'array',
                        'items': {'oneOf': step_schemas},
                        'minItems': 1,
                        'maxItems': min(max_steps, 3),
                    }
                },
                'required': ['steps'],
                'additionalProperties': False,
            }
        },
        'required': ['mission'],
        'additionalProperties': False,
    }


def _valid_result(result, step_schemas, max_steps):
    if not isinstance(result, dict) or len(result) != 1:
        return False
    kind = next(iter(result), '')
    if kind in {'clarify', 'reply'}:
        value = result[kind]
        return (
            isinstance(value, dict)
            and set(value) == {'text'}
            and isinstance(value['text'], str)
            and bool(value['text'].strip())
        )
    if kind != 'mission' or not step_schemas:
        return False
    mission = result['mission']
    if not isinstance(mission, dict) or set(mission) != {'steps'}:
        return False
    steps = mission['steps']
    if not isinstance(steps, list) or not 1 <= len(steps) <= min(max_steps, 3):
        return False
    allowed = {
        schema['properties']['kind']['const']: schema
        for schema in step_schemas
    }
    for step in steps:
        if not isinstance(step, dict) or step.get('kind') not in allowed:
            return False
        step_schema = allowed[step['kind']]
        if set(step) != set(step_schema['required']):
            return False
        field = next(key for key in step if key != 'kind')
        value = step[field]
        value_schema = step_schema['properties'][field]
        if step['kind'] in {'MOVE_DISTANCE', 'ROTATE_ANGLE'}:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return False
            if not math.isfinite(value):
                return False
            if not value_schema['minimum'] <= value <= value_schema['maximum']:
                return False
        elif not isinstance(value, str):
            return False
        elif 'enum' in value_schema and value not in value_schema['enum']:
            return False
        elif 'pattern' in value_schema and not re.fullmatch(
            value_schema['pattern'], value
        ):
            return False
    return True


def _intent_kinds(text):
    """Narrow grammar choices without extracting authoritative parameters."""
    kinds = set()
    if re.search(r'前进|向前|往前|后退|往后|倒着|移动|挪|走', text):
        kinds.add('MOVE_DISTANCE')
    if re.search(r'左转|右转|旋转|顺时针|逆时针|调头|掉头|转过去', text):
        kinds.add('ROTATE_ANGLE')
    if re.search(r'带我去|前往|导航到|回到|回家|去[^掉转]', text):
        kinds.add('NAVIGATE_TO')
    if re.search(r'保存.*地图|地图.*(?:保存|存成|名字|命名)|记录地图', text):
        kinds.add('SAVE_MAP')
    without_vague = re.sub(r'一点|一下|稍微|一些', '', text)
    has_quantity = bool(
        re.search(r'[0-9零一二两三四五六七八九十百半]', without_vague)
    )
    if not has_quantity and without_vague != text:
        kinds.difference_update({'MOVE_DISTANCE', 'ROTATE_ANGLE'})
    return kinds
