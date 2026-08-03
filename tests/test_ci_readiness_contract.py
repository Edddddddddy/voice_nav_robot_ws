import ast
import importlib.util
import re
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SIMULATION_CONTROL_TEST = (
    REPOSITORY_ROOT
    / "src"
    / "voice_nav_sim"
    / "test"
    / "test_simulation_control.py"
)
SIMULATION_CMAKE = (
    REPOSITORY_ROOT / "src" / "voice_nav_sim" / "CMakeLists.txt"
)
GAZEBO_POSE_SUPPORT = (
    REPOSITORY_ROOT
    / "src"
    / "voice_nav_sim"
    / "test_support"
    / "gazebo_pose.py"
)
BRINGUP_CMAKE = (
    REPOSITORY_ROOT / "src" / "voice_nav_bringup" / "CMakeLists.txt"
)
MISSION_CMAKE = (
    REPOSITORY_ROOT / "src" / "voice_nav_mission" / "CMakeLists.txt"
)
PACKAGE_MANIFESTS = (
    REPOSITORY_ROOT / "src" / "voice_nav_sim" / "package.xml",
    REPOSITORY_ROOT / "src" / "voice_nav_mission" / "package.xml",
    REPOSITORY_ROOT / "src" / "voice_nav_bringup" / "package.xml",
)
STARTUP_TIMEOUT_NAME = (
    "CONTROLLER_STARTUP_SERVICE_RESPONSE_TIMEOUT_SECONDS"
)
GENERATED_CHECKER = (
    REPOSITORY_ROOT / "scripts" / "check_generated_launch_tests.py"
)
GENERATED_MISSION_WORKING_DIRECTORY = (
    "/workspace/build/voice_nav_mission"
)
GENERATED_SIM_WORKING_DIRECTORY = "/workspace/build/voice_nav_sim"


def load_generated_checker():
    specification = importlib.util.spec_from_file_location(
        "voice_nav_generated_launch_test_checker",
        GENERATED_CHECKER,
    )
    if specification is None or specification.loader is None:
        raise AssertionError("could not load generated launch-test checker")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def generated_launch_test(
    package: str,
    name: str,
    source: str,
    timeout: float,
) -> dict:
    return {
        "name": name,
        "command": [
            "/usr/bin/python3",
            "-u",
            (
                "/opt/ros/jazzy/share/ament_cmake_ros/cmake/"
                "run_test_isolated.py"
            ),
            "/tmp/result.xunit.xml",
            "--command",
            f"/workspace/src/{package}/test/{source}",
        ],
        "properties": [
            {
                "name": "ENVIRONMENT",
                "value": [
                    "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
                    "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST",
                ],
            },
            {
                "name": "ENVIRONMENT_MODIFICATION",
                "value": [
                    "ROS_DOMAIN_ID=unset:",
                    "DISABLE_ROS_ISOLATION=unset:",
                ],
            },
            {"name": "LABELS", "value": ["launch_test"]},
            {"name": "RUN_SERIAL", "value": True},
            {"name": "TIMEOUT", "value": timeout},
            {
                "name": "WORKING_DIRECTORY",
                "value": f"/workspace/build/{package}",
            },
        ],
    }


def generated_mission_payload():
    return {
        "kind": "ctestInfo",
        "tests": [
            generated_launch_test(
                "voice_nav_mission",
                "test_test_motion_gate_node.py",
                "test_motion_gate_node.py",
                60.0,
            ),
            generated_launch_test(
                "voice_nav_mission",
                "test_test_motion_gate_node_journal.py",
                "test_motion_gate_node_journal.py",
                30.0,
            ),
        ],
    }


def generated_sim_payload():
    return {
        "kind": "ctestInfo",
        "tests": [
            generated_launch_test(
                "voice_nav_sim",
                "test_test_authority_process_death.py",
                "test_authority_process_death.py",
                30.0,
            ),
            generated_launch_test(
                "voice_nav_sim",
                "test_test_fault_producer_pair.py",
                "test_fault_producer_pair.py",
                30.0,
            ),
            generated_launch_test(
                "voice_nav_sim",
                "test_test_journaled_gazebo_hardware_write.py",
                "test_journaled_gazebo_hardware_write.py",
                180.0,
            ),
            generated_launch_test(
                "voice_nav_sim",
                "test_test_simulation_control.py",
                "test_simulation_control.py",
                120.0,
            ),
            generated_launch_test(
                "voice_nav_sim",
                "test_test_simulation_interfaces.py",
                "test_simulation_interfaces.py",
                120.0,
            ),
            generated_launch_test(
                "voice_nav_sim",
                "test_test_tf_ownership_conflict.py",
                "test_tf_ownership_conflict.py",
                30.0,
            ),
        ],
    }


def function_named(tree: ast.AST, name: str) -> ast.FunctionDef:
    matches = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    ]
    if len(matches) != 1:
        raise AssertionError(f"expected exactly one function named {name}")
    return matches[0]


def calls_to_method(tree: ast.AST, method_name: str) -> list[ast.Call]:
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == method_name
    ]


def launch_test_runtime(cmake_path: Path) -> tuple[set[str], dict[str, str]]:
    cmake = cmake_path.read_text(encoding="utf-8")
    matches = re.findall(
        r"set_tests_properties\(\s*([^)]*?)\s+PROPERTIES\s+"
        r"ENVIRONMENT\s+\"([^\"]+)\"\s+"
        r"RUN_SERIAL\s+TRUE\s*\)",
        cmake,
        flags=re.DOTALL,
    )
    if len(matches) != 1:
        raise AssertionError(
            f"expected one isolated launch-test block in {cmake_path}"
        )
    target_text, environment_text = matches[0]
    targets = set(re.findall(r"\btest_[A-Za-z0-9_]+\.py\b", target_text))
    environment = dict(
        entry.split("=", maxsplit=1)
        for entry in environment_text.split(";")
    )
    return targets, environment


class CiReadinessContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ast.parse(
            SIMULATION_CONTROL_TEST.read_text(encoding="utf-8")
        )

    def test_startup_service_response_budget_is_named_and_15_seconds(self):
        assignments = [
            node
            for node in self.tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == STARTUP_TIMEOUT_NAME
                for target in node.targets
            )
        ]
        self.assertEqual(len(assignments), 1)
        self.assertEqual(ast.literal_eval(assignments[0].value), 15.0)

    def test_generic_service_call_budget_remains_five_seconds(self):
        function = function_named(self.tree, "call_service")
        self.assertGreaterEqual(len(function.args.defaults), 1)
        self.assertEqual(ast.literal_eval(function.args.defaults[-1]), 5.0)

    def test_controller_states_requires_and_forwards_explicit_budget(self):
        function = function_named(self.tree, "controller_states")
        keyword_only = {
            argument.arg: default
            for argument, default in zip(
                function.args.kwonlyargs,
                function.args.kw_defaults,
            )
        }
        self.assertIn("timeout", keyword_only)
        self.assertIsNone(keyword_only["timeout"])

        calls = calls_to_method(function, "call_service")
        self.assertEqual(len(calls), 1)
        timeout_keywords = [
            keyword.value
            for keyword in calls[0].keywords
            if keyword.arg == "timeout"
        ]
        self.assertEqual(len(timeout_keywords), 1)
        self.assertIsInstance(timeout_keywords[0], ast.Name)
        self.assertEqual(timeout_keywords[0].id, "timeout")

    def test_15_second_budget_is_used_only_for_controller_startup(self):
        loaded_names = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Name)
            and isinstance(node.ctx, ast.Load)
            and node.id == STARTUP_TIMEOUT_NAME
        ]
        self.assertEqual(len(loaded_names), 1)

        test_function = function_named(
            self.tree,
            "test_stamped_drive_odometry_tf_and_consumer_timeout",
        )
        calls = calls_to_method(test_function, "controller_states")
        self.assertEqual(len(calls), 1)
        timeout_keywords = [
            keyword.value
            for keyword in calls[0].keywords
            if keyword.arg == "timeout"
        ]
        self.assertEqual(len(timeout_keywords), 1)
        self.assertIsInstance(timeout_keywords[0], ast.Name)
        self.assertEqual(timeout_keywords[0].id, STARTUP_TIMEOUT_NAME)

    def test_launch_tests_use_cross_package_isolated_runtimes(self):
        sim_targets, sim_environment = launch_test_runtime(SIMULATION_CMAKE)
        mission_targets, mission_environment = launch_test_runtime(
            MISSION_CMAKE
        )
        bringup_targets, bringup_environment = launch_test_runtime(
            BRINGUP_CMAKE
        )

        self.assertEqual(sim_targets, {
            "test_test_authority_process_death.py",
            "test_test_fault_producer_pair.py",
            "test_test_journaled_gazebo_hardware_write.py",
            "test_test_simulation_control.py",
            "test_test_simulation_interfaces.py",
            "test_test_tf_ownership_conflict.py",
        })
        self.assertEqual(
            mission_targets,
            {
                "test_test_motion_gate_node.py",
                "test_test_motion_gate_node_journal.py",
            },
        )
        self.assertEqual(
            bringup_targets,
            {"test_test_motion_gate_product.py"},
        )

        environments = {
            "sim": sim_environment,
            "mission": mission_environment,
            "bringup": bringup_environment,
        }
        for environment in environments.values():
            self.assertEqual(
                environment["RMW_IMPLEMENTATION"],
                "rmw_fastrtps_cpp",
            )
            self.assertEqual(
                environment["ROS_AUTOMATIC_DISCOVERY_RANGE"],
                "LOCALHOST",
            )

        for environment in environments.values():
            self.assertNotIn("ROS_DOMAIN_ID", environment)
        self.assertNotIn("GZ_PARTITION", sim_environment)
        self.assertNotIn("GZ_PARTITION", bringup_environment)

    def test_launch_tests_use_official_process_scoped_domain_leases(self):
        expected_launch_test_counts = {
            SIMULATION_CMAKE: 6,
            MISSION_CMAKE: 2,
            BRINGUP_CMAKE: 1,
        }
        isolated_runner = (
            'RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"'
        )
        environment_reset = (
            '"ROS_DOMAIN_ID=unset:;DISABLE_ROS_ISOLATION=unset:"'
        )

        for cmake_path, expected_count in expected_launch_test_counts.items():
            with self.subTest(cmake=cmake_path):
                cmake = cmake_path.read_text(encoding="utf-8")
                minimum = re.search(
                    r"cmake_minimum_required\(VERSION\s+([0-9.]+)\)",
                    cmake,
                )
                self.assertIsNotNone(minimum)
                self.assertGreaterEqual(
                    tuple(int(part) for part in minimum.group(1).split(".")),
                    (3, 22),
                )
                self.assertIn(
                    "find_package(ament_cmake_ros REQUIRED)",
                    cmake,
                )
                self.assertEqual(cmake.count(isolated_runner), expected_count)
                self.assertEqual(cmake.count("ENVIRONMENT_MODIFICATION"), 1)
                self.assertEqual(cmake.count(environment_reset), 1)
                self.assertNotIn("DISABLED", cmake)
                self.assertNotIn("SKIP_RETURN_CODE", cmake)
                self.assertIsNone(
                    re.search(
                        r"ROS_DOMAIN_ID=(?!unset:)",
                        cmake,
                    )
                )
                self.assertIsNone(
                    re.search(
                        r"DISABLE_ROS_ISOLATION=(?!unset:)",
                        cmake,
                    )
                )
                self.assertNotIn("ROS_DOMAIN_ID=91", cmake)
                self.assertNotIn("ROS_DOMAIN_ID=92", cmake)
                self.assertNotIn("ROS_DOMAIN_ID=93", cmake)

        for package_manifest in PACKAGE_MANIFESTS:
            with self.subTest(package=package_manifest):
                package = package_manifest.read_text(encoding="utf-8")
                self.assertIn(
                    "<test_depend>ament_cmake_ros</test_depend>",
                    package,
                )

    def test_generated_launch_test_metadata_contract_accepts_baseline(self):
        checker = load_generated_checker()

        checker.validate_package_payload(
            "voice_nav_mission",
            generated_mission_payload(),
            expected_working_directory=(
                GENERATED_MISSION_WORKING_DIRECTORY
            ),
        )
        checker.validate_package_payload(
            "voice_nav_sim",
            generated_sim_payload(),
            expected_working_directory=GENERATED_SIM_WORKING_DIRECTORY,
        )

    def test_generated_metadata_requires_node_journal_inventory(self):
        checker = load_generated_checker()
        payload = generated_mission_payload()
        payload["tests"].pop()

        with self.assertRaises(
            checker.GeneratedLaunchTestContractError,
        ):
            checker.validate_package_payload(
                "voice_nav_mission",
                payload,
                expected_working_directory=(
                    GENERATED_MISSION_WORKING_DIRECTORY
                ),
            )

    def test_generated_metadata_requires_fault_producer_pair_inventory(self):
        checker = load_generated_checker()
        payload = generated_sim_payload()
        payload["tests"].pop(0)

        with self.assertRaises(
            checker.GeneratedLaunchTestContractError,
        ):
            checker.validate_package_payload(
                "voice_nav_sim",
                payload,
                expected_working_directory=GENERATED_SIM_WORKING_DIRECTORY,
            )

    def test_generated_metadata_requires_authority_death_inventory(self):
        checker = load_generated_checker()
        payload = generated_sim_payload()
        payload["tests"].pop(0)

        with self.assertRaises(
            checker.GeneratedLaunchTestContractError,
        ):
            checker.validate_package_payload(
                "voice_nav_sim",
                payload,
                expected_working_directory=GENERATED_SIM_WORKING_DIRECTORY,
            )

    def test_generated_metadata_rejects_isolation_override(self):
        checker = load_generated_checker()
        payload = generated_mission_payload()
        environment_modification = next(
            prop
            for prop in payload["tests"][0]["properties"]
            if prop["name"] == "ENVIRONMENT_MODIFICATION"
        )
        environment_modification["value"] = ["ROS_DOMAIN_ID=set:77"]

        with self.assertRaises(
            checker.GeneratedLaunchTestContractError,
        ):
            checker.validate_package_payload(
                "voice_nav_mission",
                payload,
                expected_working_directory=(
                    GENERATED_MISSION_WORKING_DIRECTORY
                ),
            )

    def test_generated_metadata_rejects_disabled_launch_test(self):
        checker = load_generated_checker()
        payload = generated_mission_payload()
        payload["tests"][0]["properties"].append(
            {"name": "DISABLED", "value": True}
        )

        with self.assertRaises(
            checker.GeneratedLaunchTestContractError,
        ):
            checker.validate_package_payload(
                "voice_nav_mission",
                payload,
                expected_working_directory=(
                    GENERATED_MISSION_WORKING_DIRECTORY
                ),
            )

    def test_generated_metadata_rejects_result_semantics_overrides(self):
        checker = load_generated_checker()
        overrides = (
            ("WILL_FAIL", True),
            ("PASS_REGULAR_EXPRESSION", "looks good"),
            ("SKIP_REGULAR_EXPRESSION", "skip me"),
        )

        for property_name, value in overrides:
            with self.subTest(property_name=property_name):
                payload = generated_mission_payload()
                payload["tests"][0]["properties"].append(
                    {"name": property_name, "value": value}
                )
                with self.assertRaises(
                    checker.GeneratedLaunchTestContractError,
                ):
                    checker.validate_package_payload(
                        "voice_nav_mission",
                        payload,
                        expected_working_directory=(
                            GENERATED_MISSION_WORKING_DIRECTORY
                        ),
                    )

    def test_generated_metadata_rejects_wrong_execution_properties(self):
        checker = load_generated_checker()
        overrides = (
            ("LABELS", ["not_launch_test"]),
            ("TIMEOUT", 61.0),
            ("WORKING_DIRECTORY", "/wrong/build/voice_nav_mission"),
        )

        for property_name, value in overrides:
            with self.subTest(property_name=property_name):
                payload = generated_mission_payload()
                generated_property = next(
                    prop
                    for prop in payload["tests"][0]["properties"]
                    if prop["name"] == property_name
                )
                generated_property["value"] = value
                with self.assertRaises(
                    checker.GeneratedLaunchTestContractError,
                ):
                    checker.validate_package_payload(
                        "voice_nav_mission",
                        payload,
                        expected_working_directory=(
                            GENERATED_MISSION_WORKING_DIRECTORY
                        ),
                    )

    def test_ground_truth_pose_uses_bounded_pose_topic_snapshot(self):
        source = SIMULATION_CONTROL_TEST.read_text(encoding="utf-8")
        support = GAZEBO_POSE_SUPPORT.read_text(encoding="utf-8")

        self.assertIn("GAZEBO_POSE_TOPIC", source)
        self.assertIn("'/world/voice_nav_test_world/pose/info'", source)
        self.assertIn("gazebo_pose_support.read_model_pose(", source)
        self.assertIn(
            "expected_partition=SIMULATION_TEST_PARTITION",
            source,
        )
        self.assertIn("QUERY_TIMEOUT_SECONDS = 10.0", support)
        self.assertIn("QUERY_ATTEMPTS = 2", support)
        self.assertIn("MAX_SNAPSHOT_DOCUMENTS = 4", support)
        self.assertIn("'--json-output'", support)
        self.assertIn("decoder.raw_decode(output, offset)", support)
        self.assertIn("subprocess.TimeoutExpired", support)
        self.assertNotIn("'model',\n", source)

    def test_canonical_verify_checks_generated_metadata_after_build(self):
        verify = (REPOSITORY_ROOT / "scripts" / "verify.sh").read_text(
            encoding="utf-8"
        )
        generated_check = (
            "python3 scripts/check_generated_launch_tests.py "
            '"${test_result_args[@]}"'
        )

        self.assertIn(generated_check, verify)
        self.assertLess(verify.index("colcon build"), verify.index(generated_check))
        self.assertLess(verify.index(generated_check), verify.index("colcon test"))

    def test_convergence_unit_test_allows_runner_teardown_headroom(self):
        cmake = BRINGUP_CMAKE.read_text(encoding="utf-8")
        registration = re.search(
            r"ament_add_pytest_test\(\s*"
            r"motion_gate_open_convergence_test\s+"
            r"test/test_motion_gate_open_convergence\.py\s+"
            r"TIMEOUT\s+(\d+)\s*\)",
            cmake,
        )
        self.assertIsNotNone(registration)
        self.assertGreaterEqual(int(registration.group(1)), 30)


if __name__ == "__main__":
    unittest.main()
