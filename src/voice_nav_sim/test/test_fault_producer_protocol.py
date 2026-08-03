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

"""Lock the fault-producer authority's typed OPEN convergence."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import rclpy


SUPPORT_MODULE = Path(__file__).with_name('fault_producer.py')


def load_support_module():
    """Load the executable source so its pure protocol seam is testable."""
    specification = importlib.util.spec_from_file_location(
        'voice_nav_fault_producer_protocol',
        SUPPORT_MODULE,
    )
    if specification is None or specification.loader is None:
        raise AssertionError('could not load fault producer support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


support = load_support_module()


def prepared_response(**changes):
    """Build the exact unchanged PREPARED service view."""
    response = SimpleNamespace(
        code=support.InternalMotionGateControl.Response.APPLIED,
        reason=support.InternalMotionGateControl.Response.NONE,
        gate_instance_id='a' * 32,
        control_seq=1,
        state=support.InternalMotionGateState.PREPARED,
        lease_id='b' * 32,
        candidate_topic='/candidate/lease_b',
        bound_writer_gid=bytes(16),
        motion_inhibited=True,
        authority_live=False,
        candidate_fresh=False,
        writer_bound=False,
        zero_selected=True,
        zero_published=True,
        detail='',
    )
    for name, value in changes.items():
        setattr(response, name, value)
    return response


class ImmediateFuture:
    """Return one fake service response without scheduler timing."""

    def __init__(self, response):
        self.response = response

    def done(self):
        return True

    def exception(self):
        return None

    def result(self):
        return self.response


class RecordingClient:
    """Capture every generated request ID and return queued responses."""

    def __init__(self, responses):
        self.responses = iter(responses)
        self.requests = []

    def call_async(self, request):
        self.requests.append(request)
        return ImmediateFuture(next(self.responses))


class FakeExecutor:
    """Record bounded convergence backoffs without sleeping."""

    def __init__(self):
        self.timeouts = []

    def spin_once(self, *, timeout_sec):
        self.timeouts.append(timeout_sec)


class AuthorityHarness:
    """Reuse the real request and convergence methods without a ROS Node."""

    request_control = support.FaultProducerNode.request_control
    open_with_convergence = support.FaultProducerNode.open_with_convergence

    def __init__(self, responses):
        self.control_client = RecordingClient(responses)


class FaultProducerProtocolTest(unittest.TestCase):
    """Reject broadened retries and one-shot OPEN behavior."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_pending_then_applied_retries_with_fresh_request_ids(self):
        prepared = prepared_response()
        pending = prepared_response(
            code=support.InternalMotionGateControl.Response.REJECTED,
            reason=(
                support.InternalMotionGateControl.Response.
                WRITER_METADATA_PENDING
            ),
            detail='candidate writer identity unresolved',
        )
        applied = prepared_response(
            state=support.InternalMotionGateState.ARMED,
            control_seq=2,
        )
        harness = AuthorityHarness([pending, applied])
        executor = FakeExecutor()

        actual = harness.open_with_convergence(
            executor,
            prepared,
            support.time.monotonic() + 1.0,
        )

        self.assertIs(actual, applied)
        request_ids = [request.request_id for request in harness.control_client.requests]
        self.assertEqual(len(request_ids), 2)
        self.assertEqual(len(set(request_ids)), 2)
        self.assertTrue(all(len(request_id) == 32 for request_id in request_ids))
        self.assertEqual(len(executor.timeouts), 1)

    def test_only_exact_no_writer_and_metadata_pending_are_retryable(self):
        rejected = support.InternalMotionGateControl.Response.REJECTED
        unavailable = (
            support.InternalMotionGateControl.Response.WRITER_UNAVAILABLE
        )
        metadata = (
            support.InternalMotionGateControl.Response.WRITER_METADATA_PENDING
        )
        cases = (
            (prepared_response(code=rejected, reason=metadata), True),
            (
                prepared_response(
                    code=rejected,
                    reason=unavailable,
                    detail=support.NO_WRITER_PENDING_DETAIL,
                ),
                True,
            ),
            (
                prepared_response(
                    code=rejected,
                    reason=unavailable,
                    detail='final controller command endpoint is unavailable',
                ),
                False,
            ),
            (
                prepared_response(
                    code=rejected,
                    reason=(
                        support.InternalMotionGateControl.Response.
                        WRITER_AMBIGUOUS
                    ),
                ),
                False,
            ),
        )

        for response, expected in cases:
            with self.subTest(reason=response.reason, detail=response.detail):
                self.assertEqual(
                    support.is_retryable_open_response(response),
                    expected,
                )

    def test_corrupted_pending_snapshot_fails_before_second_request(self):
        prepared = prepared_response()
        corrupted = prepared_response(
            code=support.InternalMotionGateControl.Response.REJECTED,
            reason=(
                support.InternalMotionGateControl.Response.
                WRITER_METADATA_PENDING
            ),
            lease_id='changed',
        )
        harness = AuthorityHarness([corrupted])

        with self.assertRaisesRegex(RuntimeError, 'lease_id'):
            harness.open_with_convergence(
                FakeExecutor(),
                prepared,
                support.time.monotonic() + 1.0,
            )

        self.assertEqual(len(harness.control_client.requests), 1)


if __name__ == '__main__':
    unittest.main()
