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

"""Pure VN-0011A crash-evidence primitives shared by launch tests."""


class CrashEvidenceError(AssertionError):
    """The observed process-exit evidence violates its closed contract."""


class CrashLedger:
    """Track exact launch actions and their one allowed terminal exit."""

    def __init__(self) -> None:
        raise NotImplementedError("VN-0011A tests-first RED")
