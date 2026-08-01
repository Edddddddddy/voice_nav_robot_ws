import ast
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
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
        "voice_nav_l0008_sim_test",
    ),
    (
        REPOSITORY_ROOT
        / "src"
        / "voice_nav_sim"
        / "test"
        / "test_simulation_interfaces.py",
        "voice_nav_l0008_sim_test",
    ),
    (
        REPOSITORY_ROOT
        / "src"
        / "voice_nav_bringup"
        / "test"
        / "test_motion_gate_product.py",
        "voice_nav_l0009_product_test",
    ),
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
        for path, partition in GAZEBO_TESTS:
            with self.subTest(path=path):
                source = path.read_text(encoding="utf-8")
                self.assertIn(
                    "'shutdown_on_gazebo_exit': 'false'",
                    source,
                )
                self.assertIn("def setUp(self, proc_info):", source)
                self.assertIn("self.addCleanup(", source)
                self.assertIn("structured_stop_gazebo", source)
                self.assertIn(partition, source)
                self.assertIn(
                    "@launch_testing.post_shutdown_test()",
                    source,
                )
                self.assertIn("assertExitCodes(proc_info)", source)
                self.assertNotIn("allowable_exit_codes=[-9", source)
                self.assertNotIn("allowable_exit_codes=[137", source)


if __name__ == "__main__":
    unittest.main()
