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

"""Pure policy for accepting VN-0011A final-marker crash evidence."""


PRODUCER_REQUIRE_UNIQUE_FINAL_MARKER = False
GATE_REQUIRE_UNIQUE_FINAL_MARKER = True
GATE_FINAL_MARKER_MAX_COMMITS = 1
GATE_ACK_DEADLINE_OUTPUT_PERIODS = 1

JOURNAL_INSTRUMENTATION_ALLOCATION_FREE = True
UPSTREAM_GAZEBO_WRITE_ALLOCATION_FREE = False

UINT64_MAX = (1 << 64) - 1


class CrashStopPolicyError(ValueError):
    """Final-marker evidence violates the closed MotionGate crash policy."""


def _require_positive_uint64(value, field_name):
    if (
        isinstance(value, bool) or
        not isinstance(value, int) or
        value <= 0 or
        value > UINT64_MAX
    ):
        raise CrashStopPolicyError(
            f'{field_name} must be a positive uint64 sequence',
        )


def validate_gate_final_marker(
    marker_commit_count,
    ack_output_seq,
    next_output_seq,
):
    """Require one marker commit and its ACK before the first repeat."""
    if (
        isinstance(marker_commit_count, bool) or
        not isinstance(marker_commit_count, int) or
        marker_commit_count != GATE_FINAL_MARKER_MAX_COMMITS
    ):
        raise CrashStopPolicyError(
            'final marker was not committed exactly once',
        )
    _require_positive_uint64(ack_output_seq, 'ack_output_seq')
    _require_positive_uint64(next_output_seq, 'next_output_seq')
    if ack_output_seq >= next_output_seq:
        raise CrashStopPolicyError(
            'controller ACK must arrive before the next Gate output',
        )
