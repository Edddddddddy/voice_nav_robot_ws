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

import json

from voice_nav_agent._console_trace import format_event


def test_console_event_preserves_utf8_transcript_and_execution_fields():
    line = format_event(
        'voice_turn',
        voice_seq=7,
        kind=0,
        text='小智开始建图。',
    )

    assert line.startswith('VOICE_NAV ')
    assert json.loads(line.removeprefix('VOICE_NAV ')) == {
        'event': 'voice_turn',
        'kind': 0,
        'text': '小智开始建图。',
        'voice_seq': 7,
    }


def test_console_event_rejects_unbounded_or_invalid_event_names():
    for event in ('', 'mission result', 'x' * 33):
        try:
            format_event(event)
        except ValueError:
            continue
        raise AssertionError(f'invalid event was accepted: {event!r}')
