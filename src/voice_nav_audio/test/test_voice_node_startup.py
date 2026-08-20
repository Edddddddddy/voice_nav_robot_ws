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

"""Behavior contract for fail-fast continuous voice startup diagnostics."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


_ASSET_ENVIRONMENT = (
    'VOICE_NAV_KWS_ROOT',
    'VOICE_NAV_SENSEVOICE_VAD_MODEL',
    'VOICE_NAV_SENSEVOICE_MODEL',
    'VOICE_NAV_SENSEVOICE_TOKENS',
    'VOICE_NAV_CHAOWEN_TTS_ROOT',
)


def main() -> int:
    executable = Path(sys.argv[1])
    environment = os.environ.copy()
    for name in _ASSET_ENVIRONMENT:
        environment.pop(name, None)

    result = subprocess.run(
        (str(executable), '--ros-args', '-p', 'input_profile:=vad_auto'),
        env=environment,
        capture_output=True,
        check=False,
        text=True,
        timeout=10,
    )

    assert result.returncode == 1
    assert result.stderr == (
        'voice_node: continuous voice assets are incomplete\n'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
