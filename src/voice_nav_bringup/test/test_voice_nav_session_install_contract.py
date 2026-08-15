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

"""Install-space contract for the persistent simulation session entrypoint."""

from pathlib import Path


def test_voice_nav_session_launch_source_is_installed():
    """The session launch must be present in the package share launch tree."""
    launch = Path(__file__).resolve().parents[1] / 'launch' / 'voice_nav_session.launch.py'
    assert launch.is_file()
