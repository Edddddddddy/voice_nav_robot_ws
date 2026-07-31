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

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math
import time
from typing import Any, TypeVar


ResponseT = TypeVar('ResponseT')

DEFAULT_BACKOFF_SECONDS = (0.01, 0.02, 0.04, 0.08, 0.10)
PENDING_DETAIL = 'candidate topic has no writer'


@dataclass(frozen=True)
class PreparedIdentity:
    gate_instance_id: str
    control_seq: int
    lease_id: str
    candidate_topic: str


@dataclass(frozen=True)
class ProtocolValues:
    applied: int
    rejected: int
    writer_unavailable: int
    prepared: int


class OpenProtocolInvariantError(RuntimeError):
    """A retryable-looking response violated the fail-closed contract."""


class OpenConvergenceTimeout(TimeoutError):
    """The one prepared generation did not converge before its deadline."""

    def __init__(self, last_response: Any, attempts: int):
        self.last_response = last_response
        self.attempts = attempts
        detail = getattr(last_response, 'detail', 'no response')
        super().__init__(
            'MotionGate OPEN convergence deadline reached after '
            f'{attempts} attempt(s); last detail: {detail}'
        )


def _is_writer_discovery_pending(response, protocol: ProtocolValues) -> bool:
    return (
        response.code == protocol.rejected
        and response.reason == protocol.writer_unavailable
        and response.detail == PENDING_DETAIL
    )


def _validate_pending_snapshot(
    response,
    expected: PreparedIdentity,
    protocol: ProtocolValues,
) -> None:
    mismatches = []
    expected_fields = {
        'gate_instance_id': expected.gate_instance_id,
        'control_seq': expected.control_seq,
        'lease_id': expected.lease_id,
        'candidate_topic': expected.candidate_topic,
        'state': protocol.prepared,
        'motion_inhibited': True,
        'authority_live': False,
        'candidate_fresh': False,
        'writer_bound': False,
        'zero_selected': True,
        'zero_published': True,
    }
    for field_name, expected_value in expected_fields.items():
        actual_value = getattr(response, field_name)
        if actual_value != expected_value:
            mismatches.append(
                f'{field_name} expected {expected_value!r}, '
                f'got {actual_value!r}'
            )
    if any(response.bound_writer_gid):
        mismatches.append('bound_writer_gid expected all zeros')

    if mismatches:
        raise OpenProtocolInvariantError(
            'transient OPEN rejection violated fail-closed invariants: '
            + '; '.join(mismatches)
        )


def converge_open(
    *,
    expected: PreparedIdentity,
    protocol: ProtocolValues,
    attempt: Callable[[str, float], ResponseT],
    new_request_id: Callable[[], str],
    deadline: float,
    now: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    backoff_seconds: Sequence[float] = DEFAULT_BACKOFF_SECONDS,
) -> ResponseT:
    """
    Converge only the typed, fail-closed writer-discovery rejection.

    The absolute deadline covers every RPC and backoff.  Every attempt gets a
    fresh request ID so a cached rejection is never mistaken for a new graph
    observation.  All other service outcomes are terminal for this helper.
    """
    if not math.isfinite(deadline):
        raise ValueError('deadline must be finite')
    if not backoff_seconds or any(
        not math.isfinite(delay) or delay <= 0.0
        for delay in backoff_seconds
    ):
        raise ValueError('backoff_seconds must contain finite positive delays')

    seen_request_ids = set()
    last_response = None
    attempts = 0

    while True:
        remaining = deadline - now()
        if remaining <= 0.0:
            raise OpenConvergenceTimeout(last_response, attempts)

        request_id = new_request_id()
        if not request_id or request_id in seen_request_ids:
            raise OpenProtocolInvariantError(
                'each OPEN attempt requires a fresh request ID'
            )
        seen_request_ids.add(request_id)

        response = attempt(request_id, remaining)
        last_response = response
        attempts += 1
        if not _is_writer_discovery_pending(response, protocol):
            return response

        _validate_pending_snapshot(response, expected, protocol)
        remaining = deadline - now()
        if remaining <= 0.0:
            raise OpenConvergenceTimeout(last_response, attempts)
        backoff = backoff_seconds[
            min(attempts - 1, len(backoff_seconds) - 1)
        ]
        sleep(min(backoff, remaining))
