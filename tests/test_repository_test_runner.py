import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = REPOSITORY_ROOT / "scripts" / "run_repository_tests.py"


def load_runner():
    specification = importlib.util.spec_from_file_location(
        "voice_nav_repository_test_runner",
        RUNNER_PATH,
    )
    if specification is None or specification.loader is None:
        raise AssertionError("could not load repository test runner")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


class RepositoryTestRunnerTest(unittest.TestCase):
    def test_canonical_verify_uses_no_skip_runner(self):
        source = (REPOSITORY_ROOT / "scripts" / "verify.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn("python3 scripts/run_repository_tests.py", source)
        self.assertNotIn("python3 -m unittest discover", source)

    def test_skipped_contract_test_fails_the_run(self):
        runner = load_runner()

        class SkippedContract(unittest.TestCase):
            @unittest.skip("critical contract disabled")
            def test_contract(self):
                self.fail("unreachable")

        stream = io.StringIO()
        return_code = runner.run_suite(
            unittest.defaultTestLoader.loadTestsFromTestCase(
                SkippedContract
            ),
            stream=stream,
        )

        self.assertEqual(return_code, 1)
        self.assertIn("forbids skipped tests", stream.getvalue())

    def test_green_unskipped_contract_suite_succeeds(self):
        runner = load_runner()

        class PassingContract(unittest.TestCase):
            def test_contract(self):
                self.assertTrue(True)

        return_code = runner.run_suite(
            unittest.defaultTestLoader.loadTestsFromTestCase(
                PassingContract
            ),
            stream=io.StringIO(),
        )

        self.assertEqual(return_code, 0)

    def test_empty_contract_suite_fails_the_run(self):
        runner = load_runner()

        return_code = runner.run_suite(
            unittest.TestSuite(),
            stream=io.StringIO(),
        )

        self.assertEqual(return_code, 1)

    def test_expected_failure_contract_fails_the_run(self):
        runner = load_runner()

        class ExpectedFailureContract(unittest.TestCase):
            @unittest.expectedFailure
            def test_contract(self):
                self.fail("critical contract disabled")

        stream = io.StringIO()
        return_code = runner.run_suite(
            unittest.defaultTestLoader.loadTestsFromTestCase(
                ExpectedFailureContract
            ),
            stream=stream,
        )

        self.assertEqual(return_code, 1)
        self.assertIn("expected failures", stream.getvalue())

    def test_missing_required_test_id_fails_the_run(self):
        runner = load_runner()

        class PassingContract(unittest.TestCase):
            def test_contract(self):
                self.assertTrue(True)

        stream = io.StringIO()
        return_code = runner.run_suite(
            unittest.defaultTestLoader.loadTestsFromTestCase(
                PassingContract
            ),
            stream=stream,
            required_test_ids={"required.module.Contract.test_guard"},
        )

        self.assertEqual(return_code, 1)
        self.assertIn("required repository contracts", stream.getvalue())

    def test_executed_count_must_equal_discovery_snapshot(self):
        runner = load_runner()

        class NonExecutingContract(unittest.TestCase):
            def test_contract(self):
                self.fail("run() should be replaced by the fixture")

            def run(self, result=None):
                return result

        stream = io.StringIO()
        return_code = runner.run_suite(
            unittest.defaultTestLoader.loadTestsFromTestCase(
                NonExecutingContract
            ),
            stream=stream,
            required_test_ids=set(),
        )

        self.assertEqual(return_code, 1)
        self.assertIn("executed test count", stream.getvalue())

    def test_required_manifest_contains_critical_contract_ids(self):
        runner = load_runner()
        critical_ids = {
            (
                "test_ci_readiness_contract.CiReadinessContractTest."
                "test_generated_metadata_rejects_wrong_execution_properties"
            ),
            (
                "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
                "test_positive_ack_is_followed_by_real_process_exit_barrier"
            ),
            (
                "test_gazebo_pose_support.GazeboPoseSupportTest."
                "test_adjacent_snapshots_use_the_latest_complete_document"
            ),
            (
                "test_gazebo_pose_support.GazeboPoseSupportTest."
                "test_scaled_quaternion_is_normalized_before_rpy"
            ),
            (
                "test_gazebo_pose_support.GazeboPoseSupportTest."
                "test_zero_norm_quaternion_is_rejected"
            ),
            (
                "test_gazebo_teardown_contract."
                "GazeboTeardownMutationTest.test_repository_contract_passes"
            ),
            (
                "test_fault_producer_contract.FaultProducerContractTest."
                "test_repository_contract_passes"
            ),
            (
                "test_fault_producer_contract.FaultProducerContractTest."
                "test_candidate_cannot_own_control_client"
            ),
            (
                "test_repository_test_runner.RepositoryTestRunnerTest."
                "test_empty_contract_suite_fails_the_run"
            ),
        }

        self.assertLessEqual(critical_ids, runner.REQUIRED_TEST_IDS)

    def test_required_manifest_covers_every_repository_test_module(self):
        runner = load_runner()
        repository_modules = {
            path.stem
            for path in (REPOSITORY_ROOT / "tests").glob("test_*.py")
        }
        manifest_modules = {
            test_id.split(".", 1)[0]
            for test_id in runner.REQUIRED_TEST_IDS
        }

        self.assertLessEqual(repository_modules, manifest_modules)

    def test_load_tests_cannot_hide_non_manifest_contract(self):
        runner = load_runner()

        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            tests_directory = repository_root / "tests"
            tests_directory.mkdir()
            test_module = "test_repository_runner_load_tests_fixture"
            (tests_directory / f"{test_module}.py").write_text(
                "import unittest\n\n"
                "class ManifestAnchor(unittest.TestCase):\n"
                "    def test_anchor(self):\n"
                "        pass\n\n"
                "class HiddenContract(unittest.TestCase):\n"
                "    def test_must_execute(self):\n"
                "        self.fail('load_tests hid this contract')\n\n"
                "def load_tests(loader, tests, pattern):\n"
                "    return loader.loadTestsFromTestCase(ManifestAnchor)\n",
                encoding="utf-8",
            )

            repository_path = str(repository_root)
            stream = io.StringIO()
            try:
                suite = runner.discover_suite(repository_root)
                return_code = runner.run_suite(
                    suite,
                    stream=stream,
                    required_test_ids={
                        f"{test_module}.ManifestAnchor.test_anchor"
                    },
                )
            finally:
                while repository_path in sys.path:
                    sys.path.remove(repository_path)
                sys.modules.pop(test_module, None)

        self.assertEqual(return_code, 1, stream.getvalue())

    def test_assigned_load_tests_hook_cannot_hide_contract(self):
        runner = load_runner()

        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            tests_directory = repository_root / "tests"
            tests_directory.mkdir()
            test_module = "test_repository_runner_assigned_load_tests"
            (tests_directory / f"{test_module}.py").write_text(
                "import unittest\n\n"
                "class ManifestAnchor(unittest.TestCase):\n"
                "    def test_anchor(self):\n"
                "        pass\n\n"
                "class HiddenContract(unittest.TestCase):\n"
                "    def test_must_execute(self):\n"
                "        self.fail('assigned hook hid this contract')\n\n"
                "load_tests = lambda loader, tests, pattern: "
                "loader.loadTestsFromTestCase(ManifestAnchor)\n",
                encoding="utf-8",
            )

            repository_path = str(repository_root)
            stream = io.StringIO()
            try:
                suite = runner.discover_suite(repository_root)
                return_code = runner.run_suite(
                    suite,
                    stream=stream,
                    required_test_ids={
                        f"{test_module}.ManifestAnchor.test_anchor"
                    },
                )
            finally:
                while repository_path in sys.path:
                    sys.path.remove(repository_path)
                sys.modules.pop(test_module, None)

        self.assertEqual(return_code, 1, stream.getvalue())

    def test_post_definition_test_rebinding_is_rejected(self):
        runner = load_runner()

        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            tests_directory = repository_root / "tests"
            tests_directory.mkdir()
            test_module = "test_repository_runner_rebound_fixture"
            (tests_directory / f"{test_module}.py").write_text(
                "import unittest\n\n"
                "class Guard(unittest.TestCase):\n"
                "    def test_contract(self):\n"
                "        self.fail('original contract must execute')\n\n"
                "Guard.test_contract = lambda self: None\n",
                encoding="utf-8",
            )

            repository_path = str(repository_root)
            stream = io.StringIO()
            try:
                suite = runner.discover_suite(repository_root)
                return_code = runner.run_suite(
                    suite,
                    stream=stream,
                    required_test_ids={
                        f"{test_module}.Guard.test_contract"
                    },
                )
            finally:
                while repository_path in sys.path:
                    sys.path.remove(repository_path)
                sys.modules.pop(test_module, None)

        self.assertEqual(return_code, 1, stream.getvalue())

    def test_test_method_decorator_is_rejected(self):
        runner = load_runner()

        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            tests_directory = repository_root / "tests"
            tests_directory.mkdir()
            test_module = "test_repository_runner_decorated_fixture"
            (tests_directory / f"{test_module}.py").write_text(
                "import unittest\n\n"
                "def disable(function):\n"
                "    return lambda self: None\n\n"
                "class Guard(unittest.TestCase):\n"
                "    @disable\n"
                "    def test_contract(self):\n"
                "        self.fail('decorated contract must execute')\n",
                encoding="utf-8",
            )

            repository_path = str(repository_root)
            stream = io.StringIO()
            try:
                suite = runner.discover_suite(repository_root)
                return_code = runner.run_suite(
                    suite,
                    stream=stream,
                    required_test_ids={
                        f"{test_module}.Guard.test_contract"
                    },
                )
            finally:
                while repository_path in sys.path:
                    sys.path.remove(repository_path)
                sys.modules.pop(test_module, None)

        self.assertEqual(return_code, 1, stream.getvalue())

    def test_module_level_test_function_is_rejected(self):
        runner = load_runner()

        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            tests_directory = repository_root / "tests"
            tests_directory.mkdir()
            test_module = "test_repository_runner_function_fixture"
            (tests_directory / f"{test_module}.py").write_text(
                "import unittest\n\n"
                "class ManifestAnchor(unittest.TestCase):\n"
                "    def test_anchor(self):\n"
                "        pass\n\n"
                "def test_unittest_would_ignore_this():\n"
                "    raise AssertionError('must not be silently ignored')\n",
                encoding="utf-8",
            )

            repository_path = str(repository_root)
            stream = io.StringIO()
            try:
                suite = runner.discover_suite(repository_root)
                return_code = runner.run_suite(
                    suite,
                    stream=stream,
                    required_test_ids={
                        f"{test_module}.ManifestAnchor.test_anchor"
                    },
                )
            finally:
                while repository_path in sys.path:
                    sys.path.remove(repository_path)
                sys.modules.pop(test_module, None)

        self.assertEqual(return_code, 1, stream.getvalue())
        self.assertIn("module-level test function", stream.getvalue())

    def test_discovers_tests_directory_without_package_marker(self):
        runner = load_runner()

        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            tests_directory = repository_root / "tests"
            tests_directory.mkdir()
            support_module = "repository_runner_fixture_support"
            test_module = "test_repository_runner_fixture_layout"
            (repository_root / f"{support_module}.py").write_text(
                "VALUE = 7\n",
                encoding="utf-8",
            )
            (tests_directory / f"{test_module}.py").write_text(
                "import unittest\n"
                f"from {support_module} import VALUE\n\n"
                "class ExampleTest(unittest.TestCase):\n"
                "    def test_example(self):\n"
                "        self.assertEqual(VALUE, 7)\n",
                encoding="utf-8",
            )

            repository_path = str(repository_root)
            try:
                suite = runner.discover_suite(repository_root)
                return_code = runner.run_suite(
                    suite,
                    stream=io.StringIO(),
                )
            finally:
                while repository_path in sys.path:
                    sys.path.remove(repository_path)
                sys.modules.pop(support_module, None)
                sys.modules.pop(test_module, None)

        self.assertEqual(suite.countTestCases(), 1)
        self.assertEqual(return_code, 0)


if __name__ == "__main__":
    unittest.main()
