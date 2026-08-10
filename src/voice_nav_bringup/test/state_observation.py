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

"""Value semantics used by crash-stop state observations."""

from __future__ import annotations

from typing import Any


def values_equal(left: Any, right: Any) -> bool:
    """Return whether two observed ROS field values are equal."""
    left_to_list = getattr(left, 'tolist', None)
    right_to_list = getattr(right, 'tolist', None)
    normalized_left = left_to_list() if callable(left_to_list) else left
    normalized_right = right_to_list() if callable(right_to_list) else right
    return bool(normalized_left == normalized_right)
