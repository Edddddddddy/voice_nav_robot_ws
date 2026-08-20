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
    / "test_scenario_spec.py"
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
MISSION_RUNTIME_YAML = (
    REPOSITORY_ROOT
    / "src"
    / "voice_nav_bringup"
    / "config"
    / "mission_runtime.yaml"
)
MISSION_RUNTIME_INTERFACE_DOC = (
    REPOSITORY_ROOT
    / "docs"
    / "architecture"
    / "mission-runtime-interface.md"
)
MISSION_CMAKE = (
    REPOSITORY_ROOT / "src" / "voice_nav_mission" / "CMakeLists.txt"
)
PACKAGE_MANIFESTS = (
    REPOSITORY_ROOT / "src" / "voice_nav_sim" / "package.xml",
    REPOSITORY_ROOT / "src" / "voice_nav_mission" / "package.xml",
    REPOSITORY_ROOT / "src" / "voice_nav_bringup" / "package.xml",
)
GENERATED_CHECKER = (
    REPOSITORY_ROOT / "scripts" / "check_generated_launch_tests.py"
)
GENERATED_MISSION_WORKING_DIRECTORY = (
    "/workspace/build/voice_nav_mission"
)


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


def generated_mission_payload():
    return {
        "kind": "ctestInfo",
        "tests": [
            {
                "name": "test_test_motion_gate_node.py",
                "command": [
                    "/usr/bin/python3",
                    "-u",
                    (
                        "/opt/ros/jazzy/share/ament_cmake_ros/cmake/"
                        "run_test_isolated.py"
                    ),
                    "/tmp/result.xunit.xml",
                    "--command",
                    "/usr/bin/python3",
                    "-m",
                    "launch_testing.launch_test",
                    (
                        "/workspace/src/voice_nav_mission/test/"
                        "test_motion_gate_node.py"
                    ),
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
                    {"name": "TIMEOUT", "value": 60.0},
                    {
                        "name": "WORKING_DIRECTORY",
                        "value": GENERATED_MISSION_WORKING_DIRECTORY,
                    },
                ],
            },
            {
                "name": "test_test_mission_runtime_node.py",
                "command": [
                    "/usr/bin/python3",
                    "-u",
                    (
                        "/opt/ros/jazzy/share/ament_cmake_ros/cmake/"
                        "run_test_isolated.py"
                    ),
                    "/tmp/result.xunit.xml",
                    "--command",
                    "/usr/bin/python3",
                    "-m",
                    "launch_testing.launch_test",
                    (
                        "/workspace/src/voice_nav_mission/test/"
                        "test_mission_runtime_node.py"
                    ),
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
                    {"name": "TIMEOUT", "value": 60.0},
                    {
                        "name": "WORKING_DIRECTORY",
                        "value": GENERATED_MISSION_WORKING_DIRECTORY,
                    },
                ],
            },
            {
                "name": "test_test_mission_runtime_node_active_shutdown.py",
                "command": [
                    "/usr/bin/python3",
                    "-u",
                    (
                        "/opt/ros/jazzy/share/ament_cmake_ros/cmake/"
                        "run_test_isolated.py"
                    ),
                    "/tmp/result.xunit.xml",
                    "--command",
                    "/usr/bin/python3",
                    "-m",
                    "launch_testing.launch_test",
                    (
                        "/workspace/src/voice_nav_mission/test/"
                        "test_mission_runtime_node_active_shutdown.py"
                    ),
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
                    {"name": "TIMEOUT", "value": 60.0},
                    {
                        "name": "WORKING_DIRECTORY",
                        "value": GENERATED_MISSION_WORKING_DIRECTORY,
                    },
                ],
            },
            {
                "name": "test_test_mission_runtime_node_restart.py",
                "command": [
                    "/usr/bin/python3",
                    "-u",
                    (
                        "/opt/ros/jazzy/share/ament_cmake_ros/cmake/"
                        "run_test_isolated.py"
                    ),
                    "/tmp/result.xunit.xml",
                    "--command",
                    "/usr/bin/python3",
                    "-m",
                    "launch_testing.launch_test",
                    (
                        "/workspace/src/voice_nav_mission/test/"
                        "test_mission_runtime_node_restart.py"
                    ),
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
                    {"name": "TIMEOUT", "value": 60.0},
                    {
                        "name": "WORKING_DIRECTORY",
                        "value": GENERATED_MISSION_WORKING_DIRECTORY,
                    },
                ],
            },
        ],
    }


def launch_test_runtime(cmake_path: Path) -> tuple[set[str], dict[str, str]]:
    cmake = cmake_path.read_text(encoding="utf-8")
    matches = []
    for block in re.findall(
        r"set_tests_properties\(\s*(.*?)\s*\)",
        cmake,
        flags=re.DOTALL,
    ):
        environment = re.search(
            r"\bPROPERTIES\s+ENVIRONMENT\s+\"([^\"]+)\"",
            block,
            flags=re.DOTALL,
        )
        if environment is None:
            continue
        matches.append((block[: environment.start()], environment.group(1)))
    if not matches:
        raise AssertionError(
            f"expected isolated launch-test block in {cmake_path}"
        )
    target_text = "\n".join(target for target, _ in matches)
    environment_texts = {environment for _, environment in matches}
    environment_text = next(
        (
            value for value in environment_texts
            if "AMENT_PREFIX_PATH=" not in value
        ),
        next(iter(environment_texts)),
    )
    targets = set(re.findall(r"\btest_[A-Za-z0-9_]+\.py\b", target_text))
    for block in re.findall(
        r"add_launch_test\(\s*(.*?)\s*\)",
        cmake,
        flags=re.DOTALL,
    ):
        target = re.search(
            r"\bTARGET\s+([A-Za-z][A-Za-z0-9_]*)\b",
            block,
        )
        if target is not None:
            targets.add(target.group(1))
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

    def test_launch_tests_use_cross_package_isolated_runtimes(self):
        sim_targets, sim_environment = launch_test_runtime(SIMULATION_CMAKE)
        mission_targets, mission_environment = launch_test_runtime(
            MISSION_CMAKE
        )
        bringup_targets, bringup_environment = launch_test_runtime(
            BRINGUP_CMAKE
        )

        self.assertEqual(sim_targets, {
            "test_test_tf_ownership_conflict.py",
        })
        self.assertEqual(
            mission_targets,
            {
                "test_test_motion_gate_node.py",
                "test_test_mission_runtime_node.py",
                "test_test_mission_runtime_node_active_shutdown.py",
                "test_test_mission_runtime_node_restart.py",
            },
        )
        self.assertEqual(
            bringup_targets,
            {
                "test_test_motion_gate_product.py",
                "mission_runtime_crash_stop",
                "motion_gate_consumer_deadman",
                "test_test_relative_motion_product.py",
                "scripted_voice_demo_launch_test",
                "voice_nav_demo_stop_launch_test",
                "mapping_mvp_launch_test",
                "navigation_mvp_launch_test",
            },
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
            SIMULATION_CMAKE: 1,
            MISSION_CMAKE: 4,
            BRINGUP_CMAKE: 8,
        }
        expected_isolation_reset_counts = {
            SIMULATION_CMAKE: 1,
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
                environment_reset_groups = [
                    body
                    for body in re.findall(
                        r"set_tests_properties\((.*?)\)",
                        cmake,
                        flags=re.DOTALL,
                    )
                    if (
                        "ENVIRONMENT_MODIFICATION" in body
                        and environment_reset in body
                    )
                ]
                reset_group_targets = [
                    set(
                        re.findall(
                            r"\btest_[A-Za-z0-9_]+\.py\b",
                            group,
                        )
                    )
                    for group in environment_reset_groups
                ]
                if cmake_path == MISSION_CMAKE:
                    singleton_reset_groups = [
                        targets
                        for targets in reset_group_targets
                        if len(targets) == 1
                    ]
                    self.assertEqual(
                        len(singleton_reset_groups),
                        expected_isolation_reset_counts[cmake_path],
                    )
                    self.assertEqual(
                        set().union(*reset_group_targets),
                        {
                            "test_test_motion_gate_node.py",
                            "test_test_mission_runtime_node.py",
                            "test_test_mission_runtime_node_active_shutdown.py",
                            "test_test_mission_runtime_node_restart.py",
                        },
                    )
                else:
                    self.assertEqual(
                        len(environment_reset_groups),
                        expected_isolation_reset_counts[cmake_path],
                    )
                self.assertEqual(
                    cmake.count(environment_reset),
                    len(environment_reset_groups),
                )
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

    def test_generated_metadata_rejects_inventory_shape_drift(self):
        checker = load_generated_checker()

        def remove_test(payload):
            payload["tests"].pop()

        def add_extra_test(payload):
            extra = payload["tests"][0].copy()
            extra["name"] = "test_test_unapproved.py"
            payload["tests"].append(extra)

        def add_duplicate_test(payload):
            payload["tests"].append(payload["tests"][0])

        mutations = (
            ("missing", remove_test),
            ("extra", add_extra_test),
            ("duplicate", add_duplicate_test),
        )
        for name, mutate in mutations:
            with self.subTest(inventory=name):
                payload = generated_mission_payload()
                mutate(payload)
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

    def test_generated_metadata_rejects_source_and_runner_drift(self):
        checker = load_generated_checker()

        def replace_source(payload):
            payload["tests"][0]["command"][8] = (
                "/workspace/src/voice_nav_mission/test/unapproved.py"
            )

        def replace_runner(payload):
            payload["tests"][0]["command"][2] = (
                "/opt/ros/jazzy/share/ament_cmake_ros/cmake/run_test.py"
            )

        mutations = (
            ("source", replace_source),
            ("runner", replace_runner),
        )
        for name, mutate in mutations:
            with self.subTest(field=name):
                payload = generated_mission_payload()
                mutate(payload)
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

    def test_generated_metadata_rejects_nonstandard_labelled_extra(self):
        checker = load_generated_checker()
        payload = generated_mission_payload()
        extra = payload["tests"][0].copy()
        extra["name"] = "test_unapproved_launch.py"
        payload["tests"].append(extra)

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

    def test_generated_metadata_rejects_nonstandard_runner_extra(self):
        checker = load_generated_checker()
        payload = generated_mission_payload()
        extra = payload["tests"][0].copy()
        extra["name"] = "test_unapproved_runner.py"
        extra["properties"] = [
            prop.copy() for prop in extra["properties"]
        ]
        label = next(
            prop
            for prop in extra["properties"]
            if prop["name"] == "LABELS"
        )
        label["value"] = ["unit"]
        payload["tests"].append(extra)

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

    def test_generated_metadata_rejects_ambiguous_or_missing_command(self):
        checker = load_generated_checker()

        def wrong_source_with_decoy(payload):
            command = payload["tests"][0]["command"]
            command[8] = (
                "/workspace/src/voice_nav_mission/test/unapproved.py"
            )
            command.append(
                "/workspace/src/voice_nav_mission/test/"
                "test_motion_gate_node.py"
            )

        def duplicate_command(payload):
            payload["tests"][0]["command"].append("--command")

        def missing_source(payload):
            payload["tests"][0]["command"].pop()

        def wrong_module(payload):
            payload["tests"][0]["command"][7] = "launch_testing.wrong"

        mutations = (
            ("wrong source with decoy", wrong_source_with_decoy),
            ("duplicate command", duplicate_command),
            ("missing source", missing_source),
            ("wrong module", wrong_module),
        )
        for name, mutate in mutations:
            with self.subTest(command=name):
                payload = generated_mission_payload()
                mutate(payload)
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

        self.assertIn("resolve_scenario", source)
        self.assertIn("immutable", source)
        self.assertIn("QUERY_TIMEOUT_SECONDS = 10.0", support)
        self.assertIn("QUERY_ATTEMPTS = 2", support)

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

    def test_runtime_trusted_policy_is_frozen_and_documented(self):
        yaml = MISSION_RUNTIME_YAML.read_text(encoding="utf-8")
        policy = {
            "mission_deadline_ms": "30000",
            "gate_discovery_deadline_ms": "2000",
            "control_response_deadline_ms": "100",
            "stop_barrier_ms": "250",
            "cancel_grace_ms": "250",
            "source_cache_size": "64",
            "stop_cache_size": "64",
            "max_steps": "3",
            "move_distance_min_m": "0.05",
            "move_distance_max_m": "2.0",
            "rotate_angle_min_rad": "0.05",
            "rotate_angle_max_rad": "6.283185",
        }
        for key, value in policy.items():
            with self.subTest(parameter=key):
                self.assertRegex(yaml, rf"(?m)^    {key}: {value}$")

        documentation = MISSION_RUNTIME_INTERFACE_DOC.read_text(
            encoding="utf-8"
        )
        normalized_documentation = " ".join(documentation.split())
        self.assertIn("单一已审计策略记录", normalized_documentation)
        for key in policy:
            with self.subTest(documented_parameter=key):
                self.assertIn(f"`{key}`", documentation)

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
