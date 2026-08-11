"""Focused checks for the rapid loopback LLM transport boundary."""

import io
import json

import pytest

from voice_nav_agent.llm_adapter import LoopbackLlm


class _Response(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


class _Opener:
    def __init__(self, content):
        envelope = {'choices': [{'message': {'content': content}}]}
        self.payload = json.dumps(envelope).encode()
        self.request = None

    def open(self, request, timeout):  # noqa: A003
        assert timeout == 10.0
        self.request = request
        return _Response(self.payload)


def test_loopback_adapter_returns_closed_mission_json():
    """A valid local response crosses the closed transport boundary."""
    client = LoopbackLlm('http://127.0.0.1:8080/v1/chat/completions')
    client.opener = _Opener(
        '{"mission":{"steps":['
        '{"kind":"NAVIGATE_TO","target_id":"kitchen"}]}}'
    )
    assert client.plan('去做饭的地方', 2, 0x0F, 3, ('kitchen',)) == {
        'mission': {
            'steps': [{'kind': 'NAVIGATE_TO', 'target_id': 'kitchen'}]
        }
    }
    payload = json.loads(client.opener.request.data)
    response_format = payload['response_format']
    assert response_format['type'] == 'json_schema'
    assert response_format['json_schema']['strict'] is True
    system_prompt = payload['messages'][0]['content']
    assert '决策顺序：' in system_prompt
    assert '输出契约：' in system_prompt
    assert '权限边界：' in system_prompt
    assert '去做饭的地方' not in system_prompt
    user_content = payload['messages'][1]['content']
    assert user_content.startswith('/no_think\n{')
    request = json.loads(user_content.split('\n', 1)[1])
    assert request == {
        'text': '去做饭的地方',
        'mode': 'navigation',
        'supported_kinds': ['NAVIGATE_TO'],
        'max_steps': 3,
        'named_place_ids': ['kitchen'],
    }


def test_loopback_adapter_rejects_remote_and_extra_fields():
    """Remote endpoints and authority-bearing fields are rejected."""
    with pytest.raises(ValueError):
        LoopbackLlm('https://example.com/v1/chat/completions')
    client = LoopbackLlm('http://127.0.0.1:8080/v1/chat/completions')
    client.opener = _Opener(
        '{"mission":{"steps":[],"speed":1}}'
    )
    with pytest.raises(ValueError):
        client.plan('快一点', 2, 0x0F, 3, ('kitchen',))


def test_loopback_adapter_rejects_unknown_or_duplicate_targets():
    """Reject model output outside the runtime snapshot."""
    client = LoopbackLlm('http://127.0.0.1:8080/v1/chat/completions')
    client.opener = _Opener(
        '{"mission":{"steps":['
        '{"kind":"FLY","target_id":"garage"}]}}'
    )
    with pytest.raises(ValueError):
        client.plan('去车库', 2, 0x0F, 3, ('kitchen',))

    client.opener = _Opener(
        '{"mission":{"steps":['
        '{"kind":"NAVIGATE_TO","target_id":"garage"}]}}'
    )
    with pytest.raises(ValueError):
        client.plan('去车库', 2, 0x0F, 3, ('kitchen',))

    client.opener = _Opener(
        '{"mission":{"steps":['
        '{"kind":"NAVIGATE_TO","target_id":"kitchen","speed":1}]}}'
    )
    with pytest.raises(ValueError):
        client.plan('快去厨房', 2, 0x0F, 3, ('kitchen',))


def test_loopback_adapter_accepts_typed_multi_step_and_clarify():
    """The private schema covers motion, map, and clarification results."""
    client = LoopbackLlm('http://127.0.0.1:8080/v1/chat/completions')
    client.opener = _Opener(
        '{"mission":{"steps":['
        '{"kind":"MOVE_DISTANCE","distance_m":1.0},'
        '{"kind":"ROTATE_ANGLE","angle_rad":-1.5707963}]}}'
    )
    result = client.plan('前进后右转', 2, 0x0F, 3, ('home',))
    assert len(result['mission']['steps']) == 2

    client.opener = _Opener('{"clarify":{"text":"请说明距离。"}}')
    assert client.plan('往前一点', 1, 0x0B, 3, ()) == {
        'clarify': {'text': '请说明距离。'}
    }
