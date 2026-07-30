#!/usr/bin/env python3
"""Validate the ros2_control drive contract without starting ROS or Gazebo."""

from __future__ import annotations

import argparse
import math
import re
import sys
import xml.etree.ElementTree as element_tree
from pathlib import Path


class ControlContractError(ValueError):
    """The checked control-stack artifact violates the stable contract."""


EXPECTED_DEPENDENCIES = {
    "controller_manager",
    "diff_drive_controller",
    "gz_ros2_control",
    "joint_state_broadcaster",
    "launch",
    "launch_ros",
    "robot_state_publisher",
    "ros_gz_bridge",
    "ros_gz_sim",
    "xacro",
}

EXPECTED_PARAMETERS: dict[tuple[str, ...], object] = {
    (
        "controller_manager",
        "ros__parameters",
        "update_rate",
    ): 100,
    (
        "controller_manager",
        "ros__parameters",
        "use_sim_time",
    ): True,
    (
        "controller_manager",
        "ros__parameters",
        "joint_state_broadcaster",
        "type",
    ): "joint_state_broadcaster/JointStateBroadcaster",
    (
        "controller_manager",
        "ros__parameters",
        "diff_drive_controller",
        "type",
    ): "diff_drive_controller/DiffDriveController",
    (
        "diff_drive_controller",
        "ros__parameters",
        "use_sim_time",
    ): True,
    (
        "diff_drive_controller",
        "ros__parameters",
        "left_wheel_names",
    ): ["left_wheel_joint"],
    (
        "diff_drive_controller",
        "ros__parameters",
        "right_wheel_names",
    ): ["right_wheel_joint"],
    (
        "diff_drive_controller",
        "ros__parameters",
        "wheel_separation",
    ): 0.4,
    (
        "diff_drive_controller",
        "ros__parameters",
        "wheel_radius",
    ): 0.035,
    (
        "diff_drive_controller",
        "ros__parameters",
        "wheel_separation_multiplier",
    ): 1.0,
    (
        "diff_drive_controller",
        "ros__parameters",
        "left_wheel_radius_multiplier",
    ): 1.0,
    (
        "diff_drive_controller",
        "ros__parameters",
        "right_wheel_radius_multiplier",
    ): 1.0,
    (
        "diff_drive_controller",
        "ros__parameters",
        "tf_frame_prefix_enable",
    ): False,
    (
        "diff_drive_controller",
        "ros__parameters",
        "odom_frame_id",
    ): "odom",
    (
        "diff_drive_controller",
        "ros__parameters",
        "base_frame_id",
    ): "base_footprint",
    (
        "diff_drive_controller",
        "ros__parameters",
        "enable_odom_tf",
    ): True,
    (
        "diff_drive_controller",
        "ros__parameters",
        "open_loop",
    ): False,
    (
        "diff_drive_controller",
        "ros__parameters",
        "position_feedback",
    ): True,
    (
        "diff_drive_controller",
        "ros__parameters",
        "publish_rate",
    ): 50.0,
    (
        "diff_drive_controller",
        "ros__parameters",
        "cmd_vel_timeout",
    ): 0.35,
    (
        "diff_drive_controller",
        "ros__parameters",
        "publish_limited_velocity",
    ): True,
    (
        "diff_drive_controller",
        "ros__parameters",
        "linear.x.max_velocity",
    ): 0.4,
    (
        "diff_drive_controller",
        "ros__parameters",
        "linear.x.min_velocity",
    ): -0.2,
    (
        "diff_drive_controller",
        "ros__parameters",
        "linear.x.max_acceleration",
    ): 0.5,
    (
        "diff_drive_controller",
        "ros__parameters",
        "linear.x.max_deceleration",
    ): -0.5,
    (
        "diff_drive_controller",
        "ros__parameters",
        "linear.x.max_acceleration_reverse",
    ): -0.5,
    (
        "diff_drive_controller",
        "ros__parameters",
        "linear.x.max_deceleration_reverse",
    ): 0.5,
    (
        "diff_drive_controller",
        "ros__parameters",
        "angular.z.max_velocity",
    ): 1.2,
    (
        "diff_drive_controller",
        "ros__parameters",
        "angular.z.min_velocity",
    ): -1.2,
    (
        "diff_drive_controller",
        "ros__parameters",
        "angular.z.max_acceleration",
    ): 1.5,
    (
        "diff_drive_controller",
        "ros__parameters",
        "angular.z.max_deceleration",
    ): -1.5,
    (
        "diff_drive_controller",
        "ros__parameters",
        "angular.z.max_acceleration_reverse",
    ): -1.5,
    (
        "diff_drive_controller",
        "ros__parameters",
        "angular.z.max_deceleration_reverse",
    ): 1.5,
}

FORBIDDEN_PARAMETERS = {
    "enable_stamped_cmd_vel",
    "use_stamped_vel",
}


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise ControlContractError(f"cannot read {path}: {error}") from error


def parse_xml(path: Path) -> element_tree.Element:
    try:
        return element_tree.fromstring(read_text(path))
    except element_tree.ParseError as error:
        raise ControlContractError(f"cannot parse {path}: {error}") from error


def local_name(tag: str) -> str:
    return tag.rsplit("}", maxsplit=1)[-1]


def descendants(
    parent: element_tree.Element,
    name: str,
) -> list[element_tree.Element]:
    return [
        element
        for element in parent.iter()
        if local_name(element.tag) == name
    ]


def required_child_text(parent: element_tree.Element, name: str) -> str:
    matches = [
        child for child in parent if local_name(child.tag) == name
    ]
    if len(matches) != 1 or not (matches[0].text or "").strip():
        raise ControlContractError(
            f"expected exactly one non-empty {name} under "
            f"{local_name(parent.tag)}"
        )
    return (matches[0].text or "").strip()


def validate_robot_description(path: Path) -> None:
    root = parse_xml(path)
    native_plugins = [
        plugin
        for plugin in descendants(root, "plugin")
        if plugin.get("filename") == "gz-sim-diff-drive-system"
        or plugin.get("name") == "gz::sim::systems::DiffDrive"
    ]
    if native_plugins:
        raise ControlContractError(
            "native Gazebo DiffDrive plugin must not remain in the product model"
        )

    control_blocks = descendants(root, "ros2_control")
    if len(control_blocks) != 1:
        raise ControlContractError(
            "expected exactly one ros2_control system block"
        )
    control = control_blocks[0]
    if control.get("name") != "GazeboSimSystem":
        raise ControlContractError(
            "ros2_control name must be GazeboSimSystem"
        )
    if control.get("type") != "system":
        raise ControlContractError("ros2_control type must be system")

    hardware_blocks = [
        child for child in control if local_name(child.tag) == "hardware"
    ]
    if len(hardware_blocks) != 1:
        raise ControlContractError(
            "ros2_control must contain exactly one hardware block"
        )
    hardware_plugin = required_child_text(hardware_blocks[0], "plugin")
    if hardware_plugin != "gz_ros2_control/GazeboSimSystem":
        raise ControlContractError(
            "hardware plugin must be gz_ros2_control/GazeboSimSystem"
        )

    joints = {
        joint.get("name"): joint
        for joint in control
        if local_name(joint.tag) == "joint"
    }
    expected_joint_names = {"left_wheel_joint", "right_wheel_joint"}
    if set(joints) != expected_joint_names:
        raise ControlContractError(
            "ros2_control joints must be exactly left_wheel_joint and "
            "right_wheel_joint"
        )

    for joint_name in sorted(expected_joint_names):
        joint = joints[joint_name]
        command_interfaces = [
            interface
            for interface in joint
            if local_name(interface.tag) == "command_interface"
        ]
        if len(command_interfaces) != 1:
            raise ControlContractError(
                f"{joint_name} must have exactly one command interface"
            )
        command_interface = command_interfaces[0]
        if command_interface.get("name") != "velocity":
            raise ControlContractError(
                f"{joint_name} command interface must be velocity"
            )
        limits = {
            parameter.get("name"): (parameter.text or "").strip()
            for parameter in command_interface
            if local_name(parameter.tag) == "param"
        }
        if limits != {"min": "-20.0", "max": "20.0"}:
            raise ControlContractError(
                f"{joint_name} velocity command limits must be [-20.0, 20.0]"
            )
        state_interfaces = {
            interface.get("name")
            for interface in joint
            if local_name(interface.tag) == "state_interface"
        }
        if state_interfaces != {"position", "velocity"}:
            raise ControlContractError(
                f"{joint_name} state interfaces must be position and velocity"
            )

    gazebo_plugins = [
        plugin
        for plugin in descendants(root, "plugin")
        if plugin.get("name")
        == "gz_ros2_control::GazeboSimROS2ControlPlugin"
    ]
    if len(gazebo_plugins) != 1:
        raise ControlContractError(
            "expected exactly one GazeboSimROS2ControlPlugin"
        )
    gazebo_plugin = gazebo_plugins[0]
    if gazebo_plugin.get("filename") != "libgz_ros2_control-system.so":
        raise ControlContractError(
            "Gazebo ros2_control plugin filename must be "
            "libgz_ros2_control-system.so"
        )
    parameters = required_child_text(gazebo_plugin, "parameters")
    if "controllers_file" not in parameters:
        raise ControlContractError(
            "Gazebo ros2_control plugin must receive controllers_file"
        )
    hold_joints = required_child_text(gazebo_plugin, "hold_joints")
    if hold_joints.lower() != "true":
        raise ControlContractError(
            "Gazebo ros2_control plugin must hold unclaimed joints"
        )


def scalar_value(raw_value: str) -> object:
    value = raw_value.strip()
    if value in {"true", "false"}:
        return value == "true"
    if value.startswith("[") and value.endswith("]"):
        entries = [
            entry.strip().strip("\"'")
            for entry in value[1:-1].split(",")
            if entry.strip()
        ]
        return entries
    unquoted = value.strip("\"'")
    try:
        return int(unquoted)
    except ValueError:
        try:
            return float(unquoted)
        except ValueError:
            return unquoted


def parse_parameter_paths(path: Path) -> dict[tuple[str, ...], object]:
    values: dict[tuple[str, ...], object] = {}
    stack: list[tuple[int, str]] = []
    for line_number, raw_line in enumerate(
        read_text(path).splitlines(),
        start=1,
    ):
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        match = re.fullmatch(r"( *)([^:#][^:]*):(.*)", raw_line)
        if match is None:
            raise ControlContractError(
                f"unsupported controller YAML syntax at line {line_number}"
            )
        indent = len(match.group(1))
        if indent % 2:
            raise ControlContractError(
                f"controller YAML indentation must use two spaces at line "
                f"{line_number}"
            )
        key = match.group(2).strip()
        raw_value = match.group(3).strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        parameter_path = tuple(item[1] for item in stack) + (key,)
        if raw_value:
            if parameter_path in values:
                raise ControlContractError(
                    "duplicate controller parameter: "
                    + ".".join(parameter_path)
                )
            values[parameter_path] = scalar_value(raw_value)
        else:
            stack.append((indent, key))
    return values


def values_equal(actual: object, expected: object) -> bool:
    if isinstance(expected, float):
        return isinstance(actual, (float, int)) and math.isclose(
            float(actual),
            expected,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
    return actual == expected


def validate_controller_parameters(path: Path) -> None:
    values = parse_parameter_paths(path)
    leaf_names = {parameter_path[-1] for parameter_path in values}
    forbidden = sorted(FORBIDDEN_PARAMETERS & leaf_names)
    if forbidden:
        raise ControlContractError(
            "Jazzy uses TwistStamped intrinsically; remove unsupported "
            f"parameter(s): {', '.join(forbidden)}"
        )
    for parameter_path, expected in EXPECTED_PARAMETERS.items():
        if parameter_path not in values:
            raise ControlContractError(
                "missing controller parameter: "
                + ".".join(parameter_path)
            )
        actual = values[parameter_path]
        if not values_equal(actual, expected):
            raise ControlContractError(
                "controller parameter "
                + ".".join(parameter_path)
                + f" must be {expected!r}; found {actual!r}"
            )


def validate_package(path: Path) -> None:
    root = parse_xml(path)
    dependencies = {
        (element.text or "").strip()
        for element in root
        if local_name(element.tag) in {
            "depend",
            "exec_depend",
            "build_depend",
            "buildtool_depend",
        }
    }
    missing = sorted(EXPECTED_DEPENDENCIES - dependencies)
    if missing:
        raise ControlContractError(
            "voice_nav_sim is missing declared dependencies: "
            + ", ".join(missing)
        )


def validate_cmake(path: Path) -> None:
    cmake = read_text(path)
    install_match = re.search(
        r"install\s*\(\s*DIRECTORY(?P<body>.*?)"
        r"DESTINATION\s+share/\$\{PROJECT_NAME\}\s*\)",
        cmake,
        flags=re.DOTALL,
    )
    if install_match is None:
        raise ControlContractError(
            "CMake must install package share directories"
        )
    installed_directories = set(
        re.findall(r"\b(?:config|launch|urdf|worlds)\b", install_match.group("body"))
    )
    required_directories = {"config", "launch", "urdf"}
    missing = sorted(required_directories - installed_directories)
    if missing:
        raise ControlContractError(
            "CMake install is missing directories: " + ", ".join(missing)
        )


def validate_contract(
    robot_description: Path,
    controllers: Path,
    package: Path,
    cmake: Path,
) -> None:
    validate_robot_description(robot_description)
    validate_controller_parameters(controllers)
    validate_package(package)
    validate_cmake(cmake)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-description", required=True, type=Path)
    parser.add_argument("--controllers", required=True, type=Path)
    parser.add_argument("--package", required=True, type=Path)
    parser.add_argument("--cmake", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        validate_contract(
            arguments.robot_description,
            arguments.controllers,
            arguments.package,
            arguments.cmake,
        )
    except ControlContractError as error:
        print(f"Control contract failed: {error}", file=sys.stderr)
        return 1
    print("Control contract passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
