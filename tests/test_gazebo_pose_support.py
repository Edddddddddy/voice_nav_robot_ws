import importlib.util
import json
import math
from pathlib import Path
import subprocess
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SUPPORT_PATH = (
    REPOSITORY_ROOT
    / "src"
    / "voice_nav_sim"
    / "test_support"
    / "gazebo_pose.py"
)
POSE_TOPIC = "/world/voice_nav_test_world/pose/info"
MODEL_NAME = "voice_nav_robot"


def load_support():
    if not SUPPORT_PATH.is_file():
        raise AssertionError(f"missing Gazebo pose support: {SUPPORT_PATH}")
    specification = importlib.util.spec_from_file_location(
        "voice_nav_gazebo_pose_support",
        SUPPORT_PATH,
    )
    if specification is None or specification.loader is None:
        raise AssertionError("could not load Gazebo pose support")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def pose_payload(*, model_name=MODEL_NAME, x=0.25, yaw=0.5):
    return json.dumps(
        {
            "pose": [
                {
                    "name": "ground_plane",
                    "position": {},
                    "orientation": {"w": 1.0},
                },
                {
                    "name": model_name,
                    "position": {"x": x, "y": -0.1, "z": 0.05},
                    "orientation": {
                        "z": math.sin(yaw / 2.0),
                        "w": math.cos(yaw / 2.0),
                    },
                },
            ]
        }
    )


class SequenceRunner:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.calls = []

    def __call__(self, arguments, **kwargs):
        self.calls.append((arguments, kwargs))
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome


def completed(*, stdout=None, stderr="", returncode=0):
    return subprocess.CompletedProcess(
        args=[],
        returncode=returncode,
        stdout=pose_payload() if stdout is None else stdout,
        stderr=stderr,
    )


class GazeboPoseSupportTest(unittest.TestCase):
    def setUp(self):
        self.support = load_support()

    def read_pose(self, runner):
        return self.support.read_model_pose(
            POSE_TOPIC,
            MODEL_NAME,
            expected_partition="voice_nav_pose_test",
            environment={"GZ_PARTITION": "voice_nav_pose_test"},
            executable_lookup=lambda name: "/usr/bin/gz"
            if name == "gz"
            else None,
            runner=runner,
        )

    def test_wrong_partition_fails_before_query(self):
        runner = SequenceRunner([completed()])

        with self.assertRaisesRegex(AssertionError, "isolated test partition"):
            self.support.read_model_pose(
                POSE_TOPIC,
                MODEL_NAME,
                expected_partition="voice_nav_pose_test",
                environment={"GZ_PARTITION": "user_partition"},
                executable_lookup=lambda name: "/usr/bin/gz",
                runner=runner,
            )

        self.assertEqual(runner.calls, [])

    def test_single_snapshot_returns_finite_xyz_and_rpy(self):
        runner = SequenceRunner([completed()])

        pose = self.read_pose(runner)

        self.assertAlmostEqual(pose[0], 0.25)
        self.assertAlmostEqual(pose[1], -0.1)
        self.assertAlmostEqual(pose[2], 0.05)
        self.assertAlmostEqual(pose[5], 0.5)
        self.assertTrue(all(math.isfinite(value) for value in pose))
        arguments, keywords = runner.calls[0]
        self.assertEqual(
            arguments,
            [
                "/usr/bin/gz",
                "topic",
                "--echo",
                "--topic",
                POSE_TOPIC,
                "--num",
                "1",
                "--json-output",
            ],
        )
        self.assertEqual(keywords["timeout"], 10.0)
        self.assertFalse(keywords["shell"])

    def test_transient_timeout_is_retried_once(self):
        runner = SequenceRunner(
            [
                subprocess.TimeoutExpired("gz topic", 10.0),
                completed(),
            ]
        )

        pose = self.read_pose(runner)

        self.assertEqual(len(runner.calls), 2)
        self.assertAlmostEqual(pose[0], 0.25)

    def test_adjacent_snapshots_use_the_latest_complete_document(self):
        output = f"{pose_payload(x=0.1)}\n{pose_payload(x=0.3)}\n"

        pose = self.read_pose(
            SequenceRunner([completed(stdout=output)])
        )

        self.assertAlmostEqual(pose[0], 0.3)

    def test_excess_snapshot_documents_are_rejected(self):
        output = "\n".join(pose_payload(x=index / 10) for index in range(5))

        with self.assertRaisesRegex(AssertionError, "too many"):
            self.read_pose(SequenceRunner([completed(stdout=output)]))

    def test_persistent_timeout_fails_after_two_attempts(self):
        runner = SequenceRunner(
            [
                subprocess.TimeoutExpired("gz topic", 10.0),
                subprocess.TimeoutExpired("gz topic", 10.0),
            ]
        )

        with self.assertRaisesRegex(AssertionError, "2 attempts"):
            self.read_pose(runner)

        self.assertEqual(len(runner.calls), 2)

    def test_nonzero_command_exit_is_not_retried(self):
        runner = SequenceRunner(
            [completed(returncode=4, stderr="transport unavailable")]
        )

        with self.assertRaisesRegex(AssertionError, "query failed"):
            self.read_pose(runner)

        self.assertEqual(len(runner.calls), 1)

    def test_malformed_or_duplicate_model_payload_is_rejected(self):
        invalid_payloads = (
            "not-json",
            json.dumps({"pose": []}),
            json.dumps(
                {
                    "pose": [
                        json.loads(pose_payload())["pose"][1],
                        json.loads(pose_payload())["pose"][1],
                    ]
                }
            ),
        )
        for payload in invalid_payloads:
            with self.subTest(payload=payload):
                with self.assertRaises(AssertionError):
                    self.read_pose(SequenceRunner([completed(stdout=payload)]))

    def test_nonfinite_pose_is_rejected(self):
        payload = pose_payload(x="NaN")

        with self.assertRaisesRegex(AssertionError, "finite"):
            self.read_pose(SequenceRunner([completed(stdout=payload)]))

    def test_zero_norm_quaternion_is_rejected(self):
        payload = json.dumps(
            {
                "pose": [
                    {
                        "name": MODEL_NAME,
                        "position": {"x": 0.25},
                        "orientation": {},
                    }
                ]
            }
        )

        with self.assertRaisesRegex(AssertionError, "quaternion"):
            self.read_pose(SequenceRunner([completed(stdout=payload)]))

    def test_scaled_quaternion_is_normalized_before_rpy(self):
        yaw = 0.05
        payload = json.dumps(
            {
                "pose": [
                    {
                        "name": MODEL_NAME,
                        "position": {},
                        "orientation": {
                            "z": 2.0 * math.sin(yaw / 2.0),
                            "w": 2.0 * math.cos(yaw / 2.0),
                        },
                    }
                ]
            }
        )

        pose = self.read_pose(SequenceRunner([completed(stdout=payload)]))

        self.assertAlmostEqual(pose[5], yaw)
        self.assertLess(pose[5], 0.10)


if __name__ == "__main__":
    unittest.main()
