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

"""Offline checks for the rapid same-origin HTTP boundary."""

import json

import pytest

from voice_nav_agent.web_console import ConsoleApi


class _Port:
    def __init__(self):
        self.commands = []
        self.stops = 0

    def state_snapshot(self):
        return {'connected': True, 'mode': 'navigation'}

    def map_snapshot(self):
        return {'available': False, 'revision': 0}

    def submit_command(self, text):
        self.commands.append(text)
        return {'accepted': True, 'turn_id': 'turn-1'}

    def request_stop(self):
        self.stops += 1
        return {'accepted': True, 'direct_stop': True, 'turn_id': 'stop-1'}


def _decode(response):
    status, content_type, body = response
    assert content_type == 'application/json; charset=utf-8'
    return status, json.loads(body)


def test_api_reads_state_map_and_health(tmp_path):
    """Read-only routes return bounded JSON from the narrow port."""
    api = ConsoleApi(_Port(), tmp_path)
    assert _decode(api.dispatch('GET', '/api/state')) == (
        200, {'connected': True, 'mode': 'navigation'}
    )
    assert _decode(api.dispatch('GET', '/api/map?revision=0')) == (
        200, {'available': False, 'revision': 0}
    )
    assert _decode(api.dispatch('GET', '/health')) == (
        200, {'status': 'ok'}
    )


def test_api_submits_command_and_direct_stop(tmp_path):
    """Mutation routes delegate only validated command and STOP effects."""
    port = _Port()
    api = ConsoleApi(port, tmp_path)
    status, command = _decode(api.dispatch(
        'POST', '/api/command', '{"text":" 去 kitchen "}'.encode()
    ))
    assert status == 202
    assert command['turn_id'] == 'turn-1'
    assert port.commands == ['去 kitchen']
    status, stop = _decode(api.dispatch('POST', '/api/stop', b'{}'))
    assert status == 202
    assert stop['direct_stop'] is True
    assert port.stops == 1


@pytest.mark.parametrize('body', (
    b'[]',
    b'{"text":""}',
    b'{"text":"go","speed":1}',
    b'{"text":"line\\nbreak"}',
    b'not-json',
))
def test_api_rejects_malformed_or_authority_bearing_commands(
    tmp_path, body
):
    """Invalid or expanded control bodies never reach the effect port."""
    port = _Port()
    with pytest.raises(ValueError):
        ConsoleApi(port, tmp_path).dispatch('POST', '/api/command', body)
    assert port.commands == []


def test_api_serves_only_allowlisted_assets(tmp_path):
    """Static delivery cannot escape or enumerate the installed web root."""
    (tmp_path / 'index.html').write_text('<h1>VoiceNav</h1>')
    api = ConsoleApi(_Port(), tmp_path)
    status, content_type, body = api.dispatch('GET', '/')
    assert status == 200
    assert content_type == 'text/html; charset=utf-8'
    assert body == b'<h1>VoiceNav</h1>'
    assert _decode(api.dispatch('GET', '/../package.xml')) == (
        404, {'error': 'not_found'}
    )
