import importlib.util
import unittest
from dataclasses import dataclass
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SUPPORT_MODULE = (
    REPOSITORY_ROOT
    / "src"
    / "voice_nav_bringup"
    / "test"
    / "motion_gate_open_convergence.py"
)


def load_support_module():
    specification = importlib.util.spec_from_file_location(
        "motion_gate_open_convergence",
        SUPPORT_MODULE,
    )
    if specification is None or specification.loader is None:
        raise AssertionError("could not load MotionGate convergence support")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class FakeRequest:
    request_id: str


@dataclass(frozen=True)
class FakeResponse:
    outcome: str


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += seconds


class MotionGateOpenConvergenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.support = load_support_module()

    def run_convergence(self, outcomes, *, deadline=1.0):
        requests = []
        validated = []
        responses = iter(FakeResponse(outcome) for outcome in outcomes)
        clock = FakeClock()

        def make_request():
            request = FakeRequest(f"request-{len(requests) + 1}")
            requests.append(request)
            return request

        result = self.support.converge_open(
            make_request=make_request,
            call=lambda _request: next(responses),
            is_retryable=lambda response: response.outcome == "pending",
            validate_retryable=lambda response: validated.append(response),
            deadline=deadline,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        return result, requests, validated, clock

    def test_pending_responses_converge_to_applied_with_fresh_requests(self):
        result, requests, validated, clock = self.run_convergence(
            ["pending", "pending", "applied"]
        )

        self.assertEqual(result.outcome, "applied")
        self.assertEqual(
            [request.request_id for request in requests],
            ["request-1", "request-2", "request-3"],
        )
        self.assertEqual([response.outcome for response in validated], [
            "pending",
            "pending",
        ])
        self.assertEqual(clock.sleeps, [0.01, 0.02])

    def test_retry_budget_uses_one_absolute_deadline(self):
        responses = []
        requests = []
        clock = FakeClock()

        def call(_request):
            response = FakeResponse("pending")
            responses.append(response)
            return response

        def make_request():
            request = FakeRequest(f"request-{len(requests) + 1}")
            requests.append(request)
            return request

        result = self.support.converge_open(
            make_request=make_request,
            call=call,
            is_retryable=lambda response: response.outcome == "pending",
            validate_retryable=lambda _response: None,
            deadline=0.025,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )

        self.assertIs(result, responses[-1])
        self.assertEqual(len(requests), 2)
        self.assertEqual(clock.sleeps, [0.01, 0.015])
        self.assertEqual(clock.now, 0.025)

    def test_non_retryable_failure_returns_immediately(self):
        result, requests, validated, clock = self.run_convergence(
            ["writer-mismatch", "applied"]
        )

        self.assertEqual(result.outcome, "writer-mismatch")
        self.assertEqual(len(requests), 1)
        self.assertEqual(validated, [])
        self.assertEqual(clock.sleeps, [])

    def test_fail_closed_validation_error_stops_retrying(self):
        requests = []
        clock = FakeClock()

        with self.assertRaisesRegex(AssertionError, "lease changed"):
            self.support.converge_open(
                make_request=lambda: requests.append(object()) or requests[-1],
                call=lambda _request: FakeResponse("pending"),
                is_retryable=lambda _response: True,
                validate_retryable=lambda _response: (_ for _ in ()).throw(
                    AssertionError("lease changed")
                ),
                deadline=1.0,
                monotonic=clock.monotonic,
                sleep=clock.sleep,
            )

        self.assertEqual(len(requests), 1)
        self.assertEqual(clock.sleeps, [])


if __name__ == "__main__":
    unittest.main()
