import importlib.util
import os
import re
import subprocess
import unittest
from unittest import mock
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SUPPORT_PATH = (
    REPOSITORY_ROOT
    / "src"
    / "voice_nav_sim"
    / "test_support"
    / "gazebo_shutdown.py"
)
SIM_PARTITION = "voice_nav_l0008_sim_test"


def load_support():
    if not SUPPORT_PATH.is_file():
        raise AssertionError(
            f"missing Gazebo shutdown support module: {SUPPORT_PATH}"
        )
    specification = importlib.util.spec_from_file_location(
        "voice_nav_gazebo_shutdown_support",
        SUPPORT_PATH,
    )
    if specification is None or specification.loader is None:
        raise AssertionError("could not load Gazebo shutdown support")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


class FakeProcInfo:
    def __init__(self, current=None, shutdown_error=None):
        self.current = current if current is not None else object()
        self.shutdown_error = shutdown_error
        self.startup_calls = []
        self.shutdown_calls = []

    def assertWaitForStartup(self, **kwargs):
        self.startup_calls.append(kwargs)

    def assertWaitForShutdown(self, **kwargs):
        self.shutdown_calls.append(kwargs)
        if self.shutdown_error is not None:
            raise self.shutdown_error

    def __getitem__(self, process):
        if process != "gazebo":
            raise KeyError(process)
        return self.current


class RecordingRunner:
    def __init__(
        self,
        *,
        returncode=0,
        stdout="data: true\n",
        stderr="",
        error=None,
    ):
        self.completed = subprocess.CompletedProcess(
            args=[],
            returncode=returncode,
            stdout=stdout,
            stderr=stderr,
        )
        self.error = error
        self.calls = []

    def __call__(self, arguments, **kwargs):
        self.calls.append((arguments, kwargs))
        if self.error is not None:
            raise self.error
        return self.completed


def executable_lookup(name):
    return {
        "ruby": "/usr/bin/ruby",
        "gz": "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz",
    }.get(name)


class GazeboShutdownSupportTest(unittest.TestCase):
    def setUp(self):
        self.support = load_support()

    def stop(self, proc_info, runner, *, partition=SIM_PARTITION):
        return self.support.structured_stop_gazebo(
            proc_info,
            expected_partition=SIM_PARTITION,
            environment={"GZ_PARTITION": partition},
            executable_lookup=executable_lookup,
            runner=runner,
        )

    def test_claimed_partition_is_process_unique_and_overrides_inherited(self):
        with mock.patch.dict(
            os.environ,
            {"GZ_PARTITION": "inherited_user_partition"},
        ):
            first = self.support.claim_unique_test_partition(
                "l0008_sim_control"
            )
            second = self.support.claim_unique_test_partition(
                "l0008_sim_control"
            )

            pattern = re.compile(
                rf"^voice_nav_l0008_sim_control_{os.getpid()}_"
                r"[0-9a-f]{32}$"
            )
            self.assertRegex(first, pattern)
            self.assertRegex(second, pattern)
            self.assertNotEqual(first, second)
            self.assertEqual(os.environ["GZ_PARTITION"], second)

    def test_positive_ack_is_followed_by_real_process_exit_barrier(self):
        proc_info = FakeProcInfo()
        runner = RecordingRunner()

        self.stop(proc_info, runner)

        self.assertEqual(
            proc_info.startup_calls,
            [{"process": "gazebo", "timeout": 10.0}],
        )
        self.assertEqual(
            proc_info.shutdown_calls,
            [{"process": "gazebo", "timeout": 10.0}],
        )
        self.assertEqual(len(runner.calls), 1)
        arguments, keywords = runner.calls[0]
        self.assertEqual(
            arguments,
            [
                "/usr/bin/ruby",
                "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz",
                "service",
                "-s",
                "/server_control",
                "--reqtype",
                "gz.msgs.ServerControl",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                "stop: true",
            ],
        )
        self.assertEqual(
            keywords,
            {
                "capture_output": True,
                "text": True,
                "timeout": 7.0,
                "check": False,
                "shell": False,
                "env": {"GZ_PARTITION": SIM_PARTITION},
            },
        )

    def test_missing_or_wrong_partition_fails_before_cli(self):
        for partition in ("", "default", "voice_nav_other_test"):
            with self.subTest(partition=partition):
                proc_info = FakeProcInfo()
                runner = RecordingRunner()

                with self.assertRaisesRegex(
                    AssertionError,
                    "isolated test partition",
                ):
                    self.stop(proc_info, runner, partition=partition)

                self.assertEqual(runner.calls, [])
                self.assertEqual(proc_info.shutdown_calls, [])

    def test_nonzero_cli_exit_fails_without_process_wait(self):
        proc_info = FakeProcInfo()
        runner = RecordingRunner(
            returncode=4,
            stderr="service unavailable",
        )

        with self.assertRaisesRegex(AssertionError, "RPC failed"):
            self.stop(proc_info, runner)

        self.assertEqual(proc_info.shutdown_calls, [])

    def test_timeout_fails_without_process_wait(self):
        proc_info = FakeProcInfo()
        runner = RecordingRunner(
            error=subprocess.TimeoutExpired("gz service", 7.0),
        )

        with self.assertRaisesRegex(AssertionError, "timed out"):
            self.stop(proc_info, runner)

        self.assertEqual(proc_info.shutdown_calls, [])

    def test_false_or_malformed_ack_fails_without_process_wait(self):
        for response in (
            "data: false\n",
            "true\n",
            "data: true trailing\n",
            "",
        ):
            with self.subTest(response=response):
                proc_info = FakeProcInfo()
                runner = RecordingRunner(stdout=response)

                with self.assertRaisesRegex(
                    AssertionError,
                    "positive ACK",
                ):
                    self.stop(proc_info, runner)

                self.assertEqual(proc_info.shutdown_calls, [])

    def test_server_already_exited_is_not_reported_as_clean_stop(self):
        class FakeProcessExited:
            pass

        previous_type = self.support.ProcessExited
        self.support.ProcessExited = FakeProcessExited
        self.addCleanup(
            setattr,
            self.support,
            "ProcessExited",
            previous_type,
        )
        proc_info = FakeProcInfo(current=FakeProcessExited())
        runner = RecordingRunner()

        with self.assertRaisesRegex(AssertionError, "already exited"):
            self.stop(proc_info, runner)

        self.assertEqual(runner.calls, [])
        self.assertEqual(proc_info.shutdown_calls, [])

    def test_ack_does_not_hide_process_exit_barrier_failure(self):
        proc_info = FakeProcInfo(
            shutdown_error=AssertionError("process did not exit"),
        )

        with self.assertRaisesRegex(AssertionError, "did not exit"):
            self.stop(proc_info, RecordingRunner())

        self.assertEqual(
            proc_info.shutdown_calls,
            [{"process": "gazebo", "timeout": 10.0}],
        )


if __name__ == "__main__":
    unittest.main()
