#!/usr/bin/env python3
"""Validate generated CTest launch-test isolation after CMake configure."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path


EXPECTED_TESTS = {
    "voice_nav_mission": {
        "test_test_motion_gate_node.py": "test_motion_gate_node.py",
    },
    "voice_nav_sim": {
        "test_test_simulation_control.py": "test_simulation_control.py",
        "test_test_simulation_interfaces.py": "test_simulation_interfaces.py",
        "test_test_tf_ownership_conflict.py": "test_tf_ownership_conflict.py",
    },
    "voice_nav_bringup": {
        "test_test_motion_gate_product.py": "test_motion_gate_product.py",
    },
}
EXPECTED_ENVIRONMENT = [
    "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
    "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST",
]
EXPECTED_ENVIRONMENT_MODIFICATION = [
    "ROS_DOMAIN_ID=unset:",
    "DISABLE_ROS_ISOLATION=unset:",
]
PACKAGE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


class GeneratedLaunchTestContractError(ValueError):
    """Generated CTest metadata weakens launch-test isolation."""


def _properties(test: dict, package_name: str) -> dict[str, object]:
    raw_properties = test.get("properties")
    if not isinstance(raw_properties, list):
        raise GeneratedLaunchTestContractError(
            f"{package_name}:{test.get('name')} has invalid properties"
        )
    properties = {}
    for raw_property in raw_properties:
        if not isinstance(raw_property, dict):
            raise GeneratedLaunchTestContractError(
                f"{package_name}:{test.get('name')} has invalid properties"
            )
        name = raw_property.get("name")
        if not isinstance(name, str) or name in properties:
            raise GeneratedLaunchTestContractError(
                f"{package_name}:{test.get('name')} has duplicate properties"
            )
        properties[name] = raw_property.get("value")
    return properties


def validate_package_payload(package_name: str, payload: object) -> None:
    """Validate one package's generated `ctest --show-only` payload."""
    expected = EXPECTED_TESTS[package_name]
    if not isinstance(payload, dict) or payload.get("kind") != "ctestInfo":
        raise GeneratedLaunchTestContractError(
            f"{package_name} did not produce CTest JSON v1 metadata"
        )
    tests = payload.get("tests")
    if not isinstance(tests, list):
        raise GeneratedLaunchTestContractError(
            f"{package_name} has no generated CTest inventory"
        )
    launch_tests = [
        test
        for test in tests
        if isinstance(test, dict)
        and isinstance(test.get("name"), str)
        and test["name"].startswith("test_test_")
    ]
    names = [test["name"] for test in launch_tests]
    if len(names) != len(set(names)) or set(names) != set(expected):
        raise GeneratedLaunchTestContractError(
            f"{package_name} generated launch-test inventory differs: "
            f"expected={sorted(expected)}, actual={sorted(names)}"
        )

    for test in launch_tests:
        test_name = test["name"]
        properties = _properties(test, package_name)
        if properties.get("RUN_SERIAL") is not True:
            raise GeneratedLaunchTestContractError(
                f"{package_name}:{test_name} must be RUN_SERIAL"
            )
        if properties.get("DISABLED") is True:
            raise GeneratedLaunchTestContractError(
                f"{package_name}:{test_name} must remain enabled"
            )
        if "SKIP_RETURN_CODE" in properties:
            raise GeneratedLaunchTestContractError(
                f"{package_name}:{test_name} must not accept skipped exits"
            )
        if properties.get("ENVIRONMENT") != EXPECTED_ENVIRONMENT:
            raise GeneratedLaunchTestContractError(
                f"{package_name}:{test_name} has unexpected environment"
            )
        if (
            properties.get("ENVIRONMENT_MODIFICATION")
            != EXPECTED_ENVIRONMENT_MODIFICATION
        ):
            raise GeneratedLaunchTestContractError(
                f"{package_name}:{test_name} must clear inherited isolation"
            )

        command = test.get("command")
        if not isinstance(command, list) or not all(
            isinstance(token, str) for token in command
        ):
            raise GeneratedLaunchTestContractError(
                f"{package_name}:{test_name} has an invalid command"
            )
        if len(command) < 4 or not command[2].endswith(
            "/ament_cmake_ros/cmake/run_test_isolated.py"
        ):
            raise GeneratedLaunchTestContractError(
                f"{package_name}:{test_name} bypasses the official runner"
            )
        expected_source_suffix = (
            f"/src/{package_name}/test/{expected[test_name]}"
        )
        if not any(
            token.endswith(expected_source_suffix) for token in command
        ):
            raise GeneratedLaunchTestContractError(
                f"{package_name}:{test_name} runs the wrong source test"
            )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-base", type=Path, default=Path("build"))
    parser.add_argument(
        "--package",
        action="append",
        dest="packages",
        required=True,
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        for package_name in arguments.packages:
            if not PACKAGE_NAME.fullmatch(package_name):
                raise GeneratedLaunchTestContractError(
                    f"invalid package name: {package_name}"
                )
            if package_name not in EXPECTED_TESTS:
                continue
            package_directory = arguments.build_base / package_name
            if package_directory.is_symlink() or not package_directory.is_dir():
                raise GeneratedLaunchTestContractError(
                    f"missing generated package build: {package_name}"
                )
            completed = subprocess.run(
                [
                    "ctest",
                    "--test-dir",
                    str(package_directory),
                    "--show-only=json-v1",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=30.0,
            )
            if completed.returncode != 0:
                raise GeneratedLaunchTestContractError(
                    f"ctest metadata failed for {package_name}: "
                    f"{completed.stderr.strip()}"
                )
            try:
                payload = json.loads(completed.stdout)
            except json.JSONDecodeError as error:
                raise GeneratedLaunchTestContractError(
                    f"ctest metadata is not JSON for {package_name}"
                ) from error
            validate_package_payload(package_name, payload)
    except (
        GeneratedLaunchTestContractError,
        OSError,
        subprocess.SubprocessError,
    ) as error:
        print(f"Generated launch-test contract failed: {error}", file=sys.stderr)
        return 1
    print("Generated launch-test contract passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
