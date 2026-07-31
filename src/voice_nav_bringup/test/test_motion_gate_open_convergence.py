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

from dataclasses import dataclass, replace
import importlib.util
from pathlib import Path
import unittest


SUPPORT_MODULE = Path(__file__).with_name(
    'motion_gate_open_convergence.py'
)


def load_support_module():
    specification = importlib.util.spec_from_file_location(
        'motion_gate_open_convergence',
        SUPPORT_MODULE,
    )
    if specification is None or specification.loader is None:
        raise AssertionError('could not load MotionGate convergence support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class FakeResponse:
    code: int
    reason: int
    state: int
    gate_instance_id: str = 'gate-1'
    control_seq: int = 7
    lease_id: str = 'lease-1'
    candidate_topic: str = '/candidate/lease-1'
    motion_inhibited: bool = True
    authority_live: bool = False
    candidate_fresh: bool = False
    writer_bound: bool = False
    zero_selected: bool = True
    zero_published: bool = True
    bound_writer_gid: bytes = bytes(16)
    detail: str = ''


class FakeClock:
    def __init__(self):
        self.value = 0.0
        self.sleeps = []

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.value += seconds


class MotionGateOpenConvergenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.support = load_support_module()
        cls.protocol = cls.support.ProtocolValues(
            applied=0,
            rejected=2,
            writer_unavailable=10,
            prepared=1,
        )
        cls.expected = cls.support.PreparedIdentity(
            gate_instance_id='gate-1',
            control_seq=7,
            lease_id='lease-1',
            candidate_topic='/candidate/lease-1',
        )

    def pending(self, **changes):
        response = FakeResponse(
            code=self.protocol.rejected,
            reason=self.protocol.writer_unavailable,
            state=self.protocol.prepared,
            detail='candidate topic has no writer',
        )
        return replace(response, **changes)

    def terminal(self, *, code=0, reason=0, state=2, detail=''):
        return FakeResponse(
            code=code,
            reason=reason,
            state=state,
            detail=detail,
        )

    def converge(self, responses, request_ids, *, deadline=1.0):
        calls = []
        clock = FakeClock()
        response_iterator = iter(responses)
        request_iterator = iter(request_ids)

        def attempt(request_id, remaining_seconds):
            calls.append((request_id, remaining_seconds))
            return next(response_iterator)

        result = self.support.converge_open(
            expected=self.expected,
            protocol=self.protocol,
            attempt=attempt,
            new_request_id=lambda: next(request_iterator),
            deadline=deadline,
            now=clock.now,
            sleep=clock.sleep,
        )
        return result, calls, clock

    def test_pending_then_applied_uses_fresh_request_ids(self):
        applied = self.terminal()
        result, calls, clock = self.converge(
            [self.pending(), applied],
            ['request-1', 'request-2'],
        )

        self.assertIs(result, applied)
        self.assertEqual([call[0] for call in calls], [
            'request-1',
            'request-2',
        ])
        self.assertEqual(clock.sleeps, [0.01])

    def test_pending_then_prepare_expired_returns_without_third_attempt(self):
        expired = self.terminal(code=2, reason=7, state=0)
        result, calls, clock = self.converge(
            [self.pending(), expired],
            ['request-1', 'request-2', 'request-3'],
        )

        self.assertIs(result, expired)
        self.assertEqual(len(calls), 2)
        self.assertEqual(clock.sleeps, [0.01])

    def test_late_convergence_before_deadline_is_not_cut_off_by_attempt_cap(
        self,
    ):
        applied = self.terminal()
        result, calls, clock = self.converge(
            [self.pending() for _ in range(12)] + [applied],
            [f'request-{index}' for index in range(1, 14)],
        )

        self.assertIs(result, applied)
        self.assertEqual(len(calls), 13)
        self.assertAlmostEqual(clock.value, 0.95)

    def test_writer_mismatch_is_not_retried(self):
        mismatch = self.terminal(code=2, reason=12, state=1)
        result, calls, clock = self.converge(
            [mismatch],
            ['request-1', 'request-2'],
        )

        self.assertIs(result, mismatch)
        self.assertEqual(len(calls), 1)
        self.assertEqual(clock.sleeps, [])

    def test_duplicate_request_id_fails_before_second_rpc(self):
        calls = []
        clock = FakeClock()

        with self.assertRaisesRegex(
            self.support.OpenProtocolInvariantError,
            'fresh request ID',
        ):
            self.support.converge_open(
                expected=self.expected,
                protocol=self.protocol,
                attempt=lambda request_id, _remaining: (
                    calls.append(request_id) or self.pending()
                ),
                new_request_id=lambda: 'duplicate-id',
                deadline=1.0,
                now=clock.now,
                sleep=clock.sleep,
            )

        self.assertEqual(calls, ['duplicate-id'])
        self.assertEqual(clock.sleeps, [0.01])

    def test_corrupted_pending_snapshot_fails_closed_without_retry(self):
        calls = []
        clock = FakeClock()

        with self.assertRaisesRegex(
            self.support.OpenProtocolInvariantError,
            'lease_id',
        ):
            self.support.converge_open(
                expected=self.expected,
                protocol=self.protocol,
                attempt=lambda request_id, _remaining: (
                    calls.append(request_id)
                    or self.pending(lease_id='changed-lease')
                ),
                new_request_id=lambda: 'request-1',
                deadline=1.0,
                now=clock.now,
                sleep=clock.sleep,
            )

        self.assertEqual(calls, ['request-1'])
        self.assertEqual(clock.sleeps, [])

    def test_absolute_deadline_stops_without_an_out_of_budget_rpc(self):
        calls = []
        clock = FakeClock()
        request_counter = 0

        def new_request_id():
            nonlocal request_counter
            request_counter += 1
            return f'request-{request_counter}'

        with self.assertRaises(self.support.OpenConvergenceTimeout) as caught:
            self.support.converge_open(
                expected=self.expected,
                protocol=self.protocol,
                attempt=lambda request_id, remaining: (
                    calls.append((request_id, remaining)) or self.pending()
                ),
                new_request_id=new_request_id,
                deadline=0.025,
                now=clock.now,
                sleep=clock.sleep,
            )

        self.assertEqual(len(calls), 2)
        self.assertEqual(caught.exception.attempts, 2)
        self.assertEqual(len(clock.sleeps), 2)
        self.assertAlmostEqual(clock.sleeps[0], 0.01)
        self.assertAlmostEqual(clock.sleeps[1], 0.015)
        self.assertAlmostEqual(clock.value, 0.025)


if __name__ == '__main__':
    unittest.main()
