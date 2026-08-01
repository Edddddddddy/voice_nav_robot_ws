import ast
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "scripts" / "check_gazebo_teardown_contract.py"
SIMULATION_LAUNCH = (
    REPOSITORY_ROOT
    / "src"
    / "voice_nav_sim"
    / "launch"
    / "simulation.launch.py"
)
PRODUCT_LAUNCH = (
    REPOSITORY_ROOT
    / "src"
    / "voice_nav_bringup"
    / "launch"
    / "product_sim.launch.py"
)
SUPPORT = (
    REPOSITORY_ROOT
    / "src"
    / "voice_nav_sim"
    / "test_support"
    / "gazebo_shutdown.py"
)
GAZEBO_TESTS = (
    (
        REPOSITORY_ROOT
        / "src"
        / "voice_nav_sim"
        / "test"
        / "test_simulation_control.py",
        "l0008_sim_control",
    ),
    (
        REPOSITORY_ROOT
        / "src"
        / "voice_nav_sim"
        / "test"
        / "test_simulation_interfaces.py",
        "l0008_sim_interfaces",
    ),
    (
        REPOSITORY_ROOT
        / "src"
        / "voice_nav_bringup"
        / "test"
        / "test_motion_gate_product.py",
        "l0009_motion_gate_product",
    ),
)
CONTRACT_FILES = (
    "src/voice_nav_sim/test_support/gazebo_shutdown.py",
    "src/voice_nav_sim/launch/simulation.launch.py",
    "src/voice_nav_bringup/launch/product_sim.launch.py",
    "src/voice_nav_sim/test/test_simulation_control.py",
    "src/voice_nav_sim/test/test_simulation_interfaces.py",
    "src/voice_nav_bringup/test/test_motion_gate_product.py",
    "src/voice_nav_sim/CMakeLists.txt",
    "src/voice_nav_bringup/CMakeLists.txt",
    "scripts/verify.sh",
)


class GazeboTeardownContractTest(unittest.TestCase):
    def test_standard_library_support_exists_and_parses(self):
        self.assertTrue(SUPPORT.is_file(), f"missing {SUPPORT}")
        source = SUPPORT.read_text(encoding="utf-8")
        ast.parse(source, filename=str(SUPPORT))
        self.assertIn("def structured_stop_gazebo(", source)
        self.assertIn("/server_control", source)
        self.assertIn("stop: true", source)
        self.assertIn("data:", source)
        self.assertIn("assertWaitForShutdown", source)
        self.assertIn("shell=False", source)
        for forbidden in (
            "shell=True",
            "time.sleep(",
            "pkill",
            "killall",
            "os.kill(",
            "SIGKILL",
        ):
            self.assertNotIn(forbidden, source)

    def test_launch_has_default_on_unexpected_exit_and_test_seam(self):
        simulation = SIMULATION_LAUNCH.read_text(encoding="utf-8")
        product = PRODUCT_LAUNCH.read_text(encoding="utf-8")
        self.assertIn("shutdown_on_gazebo_exit", simulation)
        self.assertIn("default_value='true'", simulation)
        self.assertGreaterEqual(
            simulation.count("IfCondition(shutdown_on_gazebo_exit)"),
            2,
        )
        self.assertIn("shutdown_on_gazebo_exit", product)
        self.assertIn(
            "'shutdown_on_gazebo_exit': shutdown_on_gazebo_exit",
            product,
        )

    def test_every_gazebo_test_uses_failure_safe_cleanup_and_strict_exit(self):
        for path, scope in GAZEBO_TESTS:
            with self.subTest(path=path):
                source = path.read_text(encoding="utf-8")
                self.assertIn(
                    "'shutdown_on_gazebo_exit': 'false'",
                    source,
                )
                self.assertIn("def setUp(self, proc_info):", source)
                self.assertIn("self.addCleanup(", source)
                self.assertIn("structured_stop_gazebo", source)
                self.assertIn(scope, source)
                self.assertIn(
                    "@launch_testing.post_shutdown_test()",
                    source,
                )
                self.assertIn("assertExitCodes(proc_info)", source)
                self.assertNotIn("allowable_exit_codes=[-9", source)
                self.assertNotIn("allowable_exit_codes=[137", source)

    def test_each_launch_test_claims_a_runtime_unique_partition(self):
        for path, scope in GAZEBO_TESTS:
            with self.subTest(path=path):
                source = path.read_text(encoding="utf-8")
                self.assertIn(
                    "gazebo_shutdown.claim_unique_test_partition(",
                    source,
                )
                self.assertIn(f"'{scope}'", source)

        for relative_path in (
            "src/voice_nav_sim/CMakeLists.txt",
            "src/voice_nav_bringup/CMakeLists.txt",
        ):
            source = (REPOSITORY_ROOT / relative_path).read_text(
                encoding="utf-8"
            )
            self.assertNotIn("GZ_PARTITION=", source)


class GazeboTeardownMutationTest(unittest.TestCase):
    def run_checker(self, mutation=None):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for relative_path in CONTRACT_FILES:
                source = REPOSITORY_ROOT / relative_path
                destination = root / relative_path
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, destination)
            if mutation is not None:
                mutation(root)
            return subprocess.run(
                [sys.executable, str(CHECKER), "--root", str(root)],
                check=False,
                capture_output=True,
                text=True,
            )

    def replace(self, root, relative_path, old, new):
        path = root / relative_path
        source = path.read_text(encoding="utf-8")
        self.assertIn(old, source)
        path.write_text(source.replace(old, new, 1), encoding="utf-8")

    def assert_mutation_rejected(
        self,
        relative_path,
        old,
        new,
        diagnostic,
    ):
        def mutation(root):
            self.replace(root, relative_path, old, new)

        completed = self.run_checker(mutation)
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(diagnostic, completed.stderr)

    def test_repository_contract_passes(self):
        completed = self.run_checker()

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_ack_only_cleanup_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_sim/test_support/gazebo_shutdown.py",
            "    proc_info.assertWaitForShutdown(\n"
            "        process='gazebo',\n"
            "        timeout=PROCESS_TIMEOUT_SECONDS,\n"
            "    )",
            "    # ACK is incorrectly treated as process completion",
            "process-exit barrier",
        )

    def test_positive_ack_validation_is_required(self):
        self.assert_mutation_rejected(
            "src/voice_nav_sim/test_support/gazebo_shutdown.py",
            "if POSITIVE_ACK.fullmatch(completed.stdout) is None:",
            "if False:",
            "positive ACK",
        )

    def test_unreachable_positive_ack_validation_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_sim/test_support/gazebo_shutdown.py",
            "if POSITIVE_ACK.fullmatch(completed.stdout) is None:",
            (
                "if False and "
                "POSITIVE_ACK.fullmatch(completed.stdout) is None:"
            ),
            "positive ACK condition must be unconditional",
        )

    def test_shell_execution_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_sim/test_support/gazebo_shutdown.py",
            "            shell=False,",
            "            shell=True,",
            "shell=True",
        )

    def test_fixed_sleep_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_sim/test_support/gazebo_shutdown.py",
            "    arguments = [\n",
            "    time.sleep(1.0)\n    arguments = [\n",
            "fixed sleep",
        )

    def test_global_process_kill_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_sim/test_support/gazebo_shutdown.py",
            "    arguments = [\n",
            "    subprocess.run(['pkill', 'gz'])\n    arguments = [\n",
            "global process kill",
        )

    def test_wrong_test_partition_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_bringup/test/test_motion_gate_product.py",
            "        'l0009_motion_gate_product'",
            "        'default'",
            "must claim exact runtime-unique partition scope",
        )

    def test_fixed_cmake_partition_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_sim/CMakeLists.txt",
            "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST\"",
            (
                "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;"
                "GZ_PARTITION=fixed\""
            ),
            "must not reuse a fixed Gazebo test partition",
        )

    def test_rpc_uses_the_checked_environment_snapshot(self):
        self.assert_mutation_rejected(
            "src/voice_nav_sim/test_support/gazebo_shutdown.py",
            "            env=active_environment,\n",
            "",
            "must use the checked environment snapshot",
        )

    def test_failure_path_cleanup_registration_is_required(self):
        self.assert_mutation_rejected(
            "src/voice_nav_sim/test/test_simulation_control.py",
            "        self.addCleanup(self.cleanup_fixture, proc_info)",
            "        # cleanup omitted",
            "register failure-path cleanup first",
        )

    def test_test_launch_must_disable_early_shutdown(self):
        self.assert_mutation_rejected(
            "src/voice_nav_sim/test/test_simulation_interfaces.py",
            "'shutdown_on_gazebo_exit': 'false'",
            "'shutdown_on_gazebo_exit': 'true'",
            "structured test teardown seam",
        )

    def test_forced_exit_allowlist_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_bringup/test/test_motion_gate_product.py",
            "        assertExitCodes(proc_info)",
            (
                "        assertExitCodes(\n"
                "            proc_info, allowable_exit_codes=[-9]\n"
                "        )"
            ),
            "strictly check every process exit",
        )

    def test_unreachable_shutdown_assertion_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_bringup/test/test_motion_gate_product.py",
            "        assertExitCodes(proc_info)",
            (
                "        if False:\n"
                "            assertExitCodes(proc_info)"
            ),
            "single unconditional top-level assertion",
        )

    def test_module_level_skip_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_bringup/test/test_motion_gate_product.py",
            "import pytest\n",
            (
                "import pytest\n"
                "pytestmark = pytest.mark.skip(reason='disabled')\n"
            ),
            "must not disable or rebind critical teardown",
        )

    def test_assert_exit_codes_rebinding_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_bringup/test/test_motion_gate_product.py",
            "from launch_testing.asserts import assertExitCodes\n",
            (
                "from launch_testing.asserts import assertExitCodes\n"
                "original_assert_exit_codes = assertExitCodes\n"
                "def assertExitCodes(proc_info):\n"
                "    original_assert_exit_codes(\n"
                "        proc_info, allowable_exit_codes=[0, -3 * 3]\n"
                "    )\n"
            ),
            "must not disable or rebind critical teardown",
        )

    def test_cleanup_runtime_rebinding_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_bringup/test/test_motion_gate_product.py",
            "        assertExitCodes(proc_info)\n",
            (
                "        assertExitCodes(proc_info)\n\n"
                "MotionGateProductTest.cleanup_fixture = (\n"
                "    lambda self, proc_info: None\n"
                ")\n"
            ),
            "must not disable or rebind critical teardown",
        )

    def test_early_return_from_cleanup_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_bringup/test/test_motion_gate_product.py",
            (
                "    def cleanup_fixture(self, proc_info):\n"
                "        try:"
            ),
            (
                "    def cleanup_fixture(self, proc_info):\n"
                "        return\n"
                "        try:"
            ),
            "unconditional cleanup control flow",
        )

    def test_early_return_from_fixture_destroy_is_rejected(self):
        self.assert_mutation_rejected(
            "src/voice_nav_bringup/test/test_motion_gate_product.py",
            (
                "    def destroy_ros_fixture(self):\n"
                "        node = getattr(self, 'node', None)"
            ),
            (
                "    def destroy_ros_fixture(self):\n"
                "        return\n"
                "        node = getattr(self, 'node', None)"
            ),
            "destroy_ros_fixture must not terminate early",
        )

    def test_product_exit_policy_must_default_on(self):
        self.assert_mutation_rejected(
            "src/voice_nav_bringup/launch/product_sim.launch.py",
            (
                "'shutdown_on_gazebo_exit',\n"
                "                default_value='true'"
            ),
            (
                "'shutdown_on_gazebo_exit',\n"
                "                default_value='false'"
            ),
            "product shutdown_on_gazebo_exit must default to true",
        )

    def test_test_support_install_is_required(self):
        self.assert_mutation_rejected(
            "src/voice_nav_sim/CMakeLists.txt",
            "    test_support\n",
            "",
            "must install test_support",
        )

    def test_canonical_verify_must_run_checker(self):
        self.assert_mutation_rejected(
            "scripts/verify.sh",
            "python3 scripts/check_gazebo_teardown_contract.py --root .\n",
            "",
            "canonical verification",
        )

    def test_canonical_verify_must_fail_skipped_contracts(self):
        self.assert_mutation_rejected(
            "scripts/verify.sh",
            "python3 scripts/run_repository_tests.py\n",
            "",
            "canonical verification must fail skipped repository tests",
        )


if __name__ == "__main__":
    unittest.main()
