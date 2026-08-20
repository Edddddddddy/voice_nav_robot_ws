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

"""Compact UTF-8 console events for the human VoiceNav acceptance loop."""

import json
import re
from typing import Any


_EVENT_NAME = re.compile(r'[a-z][a-z0-9_]{0,31}')


def format_event(event: str, **fields: Any) -> str:
    """Return one grep-friendly JSON line without duplicating log formatting."""
    if _EVENT_NAME.fullmatch(event) is None:
        raise ValueError(f'invalid console event: {event!r}')
    return 'VOICE_NAV ' + json.dumps(
        {'event': event, **fields},
        ensure_ascii=False,
        separators=(',', ':'),
        sort_keys=True,
    )
