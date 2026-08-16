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

set(expected_executable "${PACKAGE_PREFIX}/lib/voice_nav_audio/voice_node")
set(expected_launch "${PACKAGE_PREFIX}/share/voice_nav_audio/launch/voice_node.launch.py")

if(NOT EXISTS "${expected_executable}")
  message(FATAL_ERROR "installed voice_node is missing: ${expected_executable}")
endif()
if(NOT EXISTS "${expected_launch}")
  message(FATAL_ERROR "installed voice_node launch entry is missing: ${expected_launch}")
endif()
