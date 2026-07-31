#!/usr/bin/env python3
"""Validate the Lesson 0009 MotionGate product contract without starting ROS."""

from __future__ import annotations

import argparse
import ast
import math
import re
import sys
import xml.etree.ElementTree as element_tree
from pathlib import Path


class MotionGateContractError(ValueError):
    """A checked artifact violates the Lesson 0009 MotionGate contract."""


ARTIFACTS = {
    "control_interface": (
        "src/voice_nav_mission/srv/InternalMotionGateControl.srv"
    ),
    "state_interface": (
        "src/voice_nav_mission/msg/InternalMotionGateState.msg"
    ),
    "core_header": (
        "src/voice_nav_mission/include/voice_nav_mission/"
        "motion_gate_core.hpp"
    ),
    "core_source": "src/voice_nav_mission/src/motion_gate_core.cpp",
    "node_source": "src/voice_nav_mission/src/motion_gate_node.cpp",
    "mission_package": "src/voice_nav_mission/package.xml",
    "mission_cmake": "src/voice_nav_mission/CMakeLists.txt",
    "gate_config": "src/voice_nav_bringup/config/motion_gate.yaml",
    "product_launch": (
        "src/voice_nav_bringup/launch/product_sim.launch.py"
    ),
    "bringup_package": "src/voice_nav_bringup/package.xml",
    "bringup_cmake": "src/voice_nav_bringup/CMakeLists.txt",
    "controller_config": "src/voice_nav_sim/config/controllers.yaml",
}

CONTROL_REQUIRED_LINES = {
    "uint8 PREPARE=1",
    "uint8 OPEN=2",
    "uint8 RENEW=3",
    "uint8 INHIBIT=4",
    "uint8 operation",
    "string<=36 request_id",
    "string<=36 gate_instance_id",
    "string<=36 lease_id",
    "uint64 expected_control_seq",
    "uint16 APPLIED=0",
    "uint16 DUPLICATE=1",
    "uint16 REJECTED=2",
    "uint16 FAULTED=3",
    "uint16 code",
    "uint16 reason",
    "uint64 control_seq",
    "uint8 state",
    "bool writer_bound",
    "bool zero_selected",
    "bool motion_inhibited",
    "bool zero_published",
    "uint64 output_publish_seq",
    "uint64 zero_publish_seq",
    "uint8[16] bound_writer_gid",
    "string<=128 candidate_topic",
    "string<=160 detail",
}

STATE_REQUIRED_LINES = {
    "uint8 INHIBITED=0",
    "uint8 PREPARED=1",
    "uint8 ARMED=2",
    "uint8 FAULTED=3",
    "string<=36 gate_instance_id",
    "uint64 control_seq",
    "uint64 state_seq",
    "uint8 state",
    "string<=36 lease_id",
    "bool authority_live",
    "bool candidate_fresh",
    "bool writer_bound",
    "bool zero_selected",
    "bool motion_inhibited",
    "uint64 output_publish_seq",
    "uint64 zero_publish_seq",
    "uint8[16] bound_writer_gid",
    "string<=128 candidate_topic",
    "uint16 reason",
    "string<=160 detail",
}

EXPECTED_GATE_PARAMETERS: dict[tuple[str, ...], object] = {
    ("motion_gate_node", "ros__parameters", "use_sim_time"): True,
    ("motion_gate_node", "ros__parameters", "output_frequency_hz"): 50.0,
    ("motion_gate_node", "ros__parameters", "authority_lease_ms"): 250,
    ("motion_gate_node", "ros__parameters", "candidate_freshness_ms"): 150,
    ("motion_gate_node", "ros__parameters", "prepare_timeout_ms"): 1000,
    ("motion_gate_node", "ros__parameters", "writer_graph_timeout_ms"): 1000,
    ("motion_gate_node", "ros__parameters", "candidate_qos_depth"): 1,
    (
        "motion_gate_node",
        "ros__parameters",
        "expected_candidate_writer_fqn",
    ): "/collision_monitor",
    ("motion_gate_node", "ros__parameters", "request_cache_size"): 64,
    ("motion_gate_node", "ros__parameters", "linear_x_min"): -0.2,
    ("motion_gate_node", "ros__parameters", "linear_x_max"): 0.4,
    ("motion_gate_node", "ros__parameters", "angular_z_min"): -1.2,
    ("motion_gate_node", "ros__parameters", "angular_z_max"): 1.2,
}

FORBIDDEN_CONFIGURABLE_ENDPOINTS = {
    "candidate_topic_prefix",
    "control_service",
    "final_command_topic",
    "state_topic",
}

MISSION_DEPENDENCIES = {
    "geometry_msgs",
    "rclcpp",
    "rmw",
    "rosidl_default_generators",
    "rosidl_default_runtime",
}

MISSION_TEST_DEPENDENCIES = {
    "ament_cmake_gtest",
    "launch_testing",
    "launch_testing_ament_cmake",
}

BRINGUP_DEPENDENCIES = {
    "launch",
    "launch_ros",
    "voice_nav_mission",
    "voice_nav_sim",
}


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise MotionGateContractError(
            f"cannot read {path}: {error}"
        ) from error


def required_artifacts(root: Path) -> dict[str, Path]:
    resolved = {
        name: root / relative_path
        for name, relative_path in ARTIFACTS.items()
    }
    for name in ARTIFACTS:
        path = resolved[name]
        if not path.is_file():
            relative = path.relative_to(root).as_posix()
            raise MotionGateContractError(
                f"missing Lesson 0009 MotionGate artifact: {relative}"
            )
    return resolved


def normalized_idl_lines(path: Path) -> list[str]:
    lines: list[str] = []
    for raw_line in read_text(path).splitlines():
        line = raw_line.split("#", maxsplit=1)[0].strip()
        if line:
            lines.append(re.sub(r"\s+", " ", line))
    return lines


def validate_idl(
    path: Path,
    required_lines: set[str],
    interface_name: str,
) -> None:
    lines = normalized_idl_lines(path)
    missing = sorted(required_lines - set(lines))
    if missing:
        raise MotionGateContractError(
            f"{interface_name} is missing bounded contract line(s): "
            + ", ".join(missing)
        )
    unbounded_strings = [
        line
        for line in lines
        if re.match(r"^string(?:\s|\[)", line)
    ]
    if unbounded_strings:
        raise MotionGateContractError(
            f"{interface_name} must not contain unbounded strings: "
            + ", ".join(unbounded_strings)
        )


def validate_private_idl_location(root: Path) -> None:
    public_root = root / "src" / "voice_nav_interfaces"
    if not public_root.exists():
        return
    leaked = sorted(
        path.relative_to(root).as_posix()
        for path in public_root.rglob("InternalMotionGate*")
        if path.is_file()
    )
    if leaked:
        raise MotionGateContractError(
            "MotionGate control IDL is package-internal and must not be "
            "duplicated in voice_nav_interfaces: "
            + ", ".join(leaked)
        )


def parse_xml(path: Path) -> element_tree.Element:
    try:
        return element_tree.fromstring(read_text(path))
    except element_tree.ParseError as error:
        raise MotionGateContractError(
            f"cannot parse {path}: {error}"
        ) from error


def local_name(tag: str) -> str:
    return tag.rsplit("}", maxsplit=1)[-1]


def package_dependencies(
    root: element_tree.Element,
    tags: set[str],
) -> set[str]:
    return {
        (element.text or "").strip()
        for element in root
        if local_name(element.tag) in tags
    }


def validate_mission_package(path: Path) -> None:
    root = parse_xml(path)
    dependencies = package_dependencies(
        root,
        {
            "depend",
            "build_depend",
            "buildtool_depend",
            "exec_depend",
        },
    )
    missing = sorted(MISSION_DEPENDENCIES - dependencies)
    if missing:
        raise MotionGateContractError(
            "voice_nav_mission is missing MotionGate dependency declarations: "
            + ", ".join(missing)
        )
    all_test_dependencies = dependencies | package_dependencies(
        root,
        {"test_depend"},
    )
    missing_test = sorted(
        MISSION_TEST_DEPENDENCIES - all_test_dependencies
    )
    if missing_test:
        raise MotionGateContractError(
            "voice_nav_mission is missing MotionGate test dependencies: "
            + ", ".join(missing_test)
        )
    groups = package_dependencies(root, {"member_of_group"})
    if "rosidl_interface_packages" not in groups:
        raise MotionGateContractError(
            "voice_nav_mission must declare membership in "
            "rosidl_interface_packages"
        )


def validate_bringup_package(path: Path) -> None:
    root = parse_xml(path)
    dependencies = package_dependencies(
        root,
        {
            "depend",
            "build_depend",
            "buildtool_depend",
            "exec_depend",
        },
    )
    missing = sorted(BRINGUP_DEPENDENCIES - dependencies)
    if missing:
        raise MotionGateContractError(
            "voice_nav_bringup is missing product-composition dependencies: "
            + ", ".join(missing)
        )


def scalar_value(raw_value: str) -> object:
    value = raw_value.strip()
    if value in {"true", "false"}:
        return value == "true"
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
            raise MotionGateContractError(
                f"unsupported MotionGate YAML syntax at line {line_number}"
            )
        indent = len(match.group(1))
        if indent % 2:
            raise MotionGateContractError(
                "MotionGate YAML indentation must use two spaces at line "
                f"{line_number}"
            )
        key = match.group(2).strip()
        raw_value = match.group(3).strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        parameter_path = tuple(item[1] for item in stack) + (key,)
        if raw_value:
            if parameter_path in values:
                raise MotionGateContractError(
                    "duplicate MotionGate parameter: "
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


def validate_gate_parameters(path: Path) -> dict[tuple[str, ...], object]:
    values = parse_parameter_paths(path)
    configured_endpoints = sorted(
        parameter_path[-1]
        for parameter_path in values
        if parameter_path[-1] in FORBIDDEN_CONFIGURABLE_ENDPOINTS
    )
    if configured_endpoints:
        raise MotionGateContractError(
            "MotionGate safety endpoints are code constants, not runtime "
            "parameters: "
            + ", ".join(configured_endpoints)
        )
    for parameter_path, expected in EXPECTED_GATE_PARAMETERS.items():
        if parameter_path not in values:
            raise MotionGateContractError(
                "missing MotionGate parameter: "
                + ".".join(parameter_path)
            )
        actual = values[parameter_path]
        if not values_equal(actual, expected):
            raise MotionGateContractError(
                "MotionGate parameter "
                + ".".join(parameter_path)
                + f" must be {expected!r}; found {actual!r}"
            )

    output_period_ms = 1000.0 / float(
        values[
            ("motion_gate_node", "ros__parameters", "output_frequency_hz")
        ]
    )
    candidate_ms = float(
        values[
            (
                "motion_gate_node",
                "ros__parameters",
                "candidate_freshness_ms",
            )
        ]
    )
    authority_ms = float(
        values[
            ("motion_gate_node", "ros__parameters", "authority_lease_ms")
        ]
    )
    if not output_period_ms < candidate_ms < authority_ms:
        raise MotionGateContractError(
            "MotionGate deadlines must satisfy output period < candidate "
            "freshness < authority lease"
        )
    return values


def controller_value(
    values: dict[tuple[str, ...], object],
    name: str,
) -> float:
    path = ("diff_drive_controller", "ros__parameters", name)
    if path not in values or not isinstance(values[path], (float, int)):
        raise MotionGateContractError(
            "controller configuration is missing numeric parameter: "
            + ".".join(path)
        )
    return float(values[path])


def validate_controller_compatibility(
    gate_values: dict[tuple[str, ...], object],
    controller_path: Path,
) -> None:
    controller_values = parse_parameter_paths(controller_path)
    controller_timeout_ms = 1000.0 * controller_value(
        controller_values,
        "cmd_vel_timeout",
    )
    authority_ms = float(
        gate_values[
            ("motion_gate_node", "ros__parameters", "authority_lease_ms")
        ]
    )
    if not authority_ms < controller_timeout_ms:
        raise MotionGateContractError(
            "MotionGate authority lease must be shorter than the controller "
            "consumer timeout"
        )

    comparisons = (
        ("linear_x_min", "linear.x.min_velocity", lambda gate, ctl: gate >= ctl),
        ("linear_x_max", "linear.x.max_velocity", lambda gate, ctl: gate <= ctl),
        ("angular_z_min", "angular.z.min_velocity", lambda gate, ctl: gate >= ctl),
        ("angular_z_max", "angular.z.max_velocity", lambda gate, ctl: gate <= ctl),
    )
    for gate_name, controller_name, predicate in comparisons:
        gate_value = float(
            gate_values[
                ("motion_gate_node", "ros__parameters", gate_name)
            ]
        )
        controller_limit = controller_value(
            controller_values,
            controller_name,
        )
        if not predicate(gate_value, controller_limit):
            raise MotionGateContractError(
                f"MotionGate {gate_name}={gate_value} is wider than "
                f"controller {controller_name}={controller_limit}"
            )


def require_source_tokens(
    source: str,
    tokens: tuple[str, ...],
    context: str,
) -> None:
    missing = [token for token in tokens if token not in source]
    if missing:
        raise MotionGateContractError(
            f"{context} is missing contract marker(s): "
            + ", ".join(missing)
        )


def function_body(source: str, signature: str, context: str) -> str:
    signature_index = source.find(signature)
    if signature_index < 0:
        raise MotionGateContractError(
            f"{context} must define {signature}"
        )
    opening = source.find("{", signature_index)
    if opening < 0:
        raise MotionGateContractError(
            f"{context} has no body for {signature}"
        )
    depth = 0
    for index in range(opening, len(source)):
        character = source[index]
        if character == "{":
            depth += 1
        elif character == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1 : index]
    raise MotionGateContractError(
        f"{context} has an unterminated body for {signature}"
    )


def method_body(
    source: str,
    class_name: str,
    method_name: str,
    context: str,
) -> str:
    definition = re.search(
        rf"(?:\b{re.escape(class_name)}::)?"
        rf"{re.escape(method_name)}\s*\([^;{{}}]*\)\s*\{{",
        source,
        flags=re.DOTALL,
    )
    if definition is None:
        raise MotionGateContractError(
            f"{context} must define {class_name}::{method_name}("
        )
    opening = source.find("{", definition.start())
    depth = 0
    for index in range(opening, len(source)):
        character = source[index]
        if character == "{":
            depth += 1
        elif character == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1 : index]
    raise MotionGateContractError(
        f"{context} has an unterminated body for "
        f"{class_name}::{method_name}("
    )


def validate_core(header_path: Path, source_path: Path) -> None:
    header = read_text(header_path)
    source = read_text(source_path)
    require_source_tokens(
        header,
        (
            "class MotionGateCore",
            "std::chrono::steady_clock::time_point",
            "State::Inhibited",
            "prepare(",
            "open(",
            "renew(",
            "inhibit(",
            "accept_candidate(",
            "tick(",
            "request_id_cache_",
            "control_seq_",
        ),
        "MotionGateCore header",
    )
    forbidden_clock_tokens = (
        "RCL_ROS_TIME",
        "rclcpp::Clock",
        "get_clock()",
        "this->now()",
    )
    present_forbidden = [
        token for token in forbidden_clock_tokens if token in source
    ]
    if present_forbidden:
        raise MotionGateContractError(
            "MotionGateCore deadlines must use injected steady time, not ROS "
            "time: "
            + ", ".join(present_forbidden)
        )

    prepare = function_body(
        source,
        "MotionGateCore::prepare(",
        "MotionGateCore",
    )
    require_source_tokens(
        prepare,
        (
            "State::Inhibited",
            "request_id_cache_",
            "cached->second.request_fingerprint",
            "request.expected_control_seq",
            "control_seq_",
            "make_lease_id",
        ),
        "MotionGateCore::prepare",
    )
    open_body = function_body(
        source,
        "MotionGateCore::open(",
        "MotionGateCore",
    )
    require_source_tokens(
        open_body,
        (
            "State::Prepared",
            "writer_bound_",
            "request.expected_control_seq",
            "control_seq_",
            "authority_deadline_",
            "candidate_deadline_",
        ),
        "MotionGateCore::open",
    )
    renew = function_body(
        source,
        "MotionGateCore::renew(",
        "MotionGateCore",
    )
    require_source_tokens(
        renew,
        (
            "State::Armed",
            "request.expected_control_seq != control_seq_",
            "now >= authority_deadline_",
            "authority_deadline_ = now + authority_lease_",
        ),
        "MotionGateCore::renew",
    )
    candidate = function_body(
        source,
        "MotionGateCore::accept_candidate(",
        "MotionGateCore",
    )
    require_source_tokens(
        candidate,
        (
            "std::isfinite",
            "linear_x",
            "linear_y",
            "linear_z",
            "angular_x",
            "angular_y",
            "angular_z",
            "std::clamp",
            "candidate_deadline_",
            "retire_lease",
        ),
        "MotionGateCore::accept_candidate",
    )
    if "authority_deadline_" in candidate or ".renew(" in candidate:
        raise MotionGateContractError(
            "candidate samples must never renew Runtime authority"
        )
    inhibit = function_body(
        source,
        "MotionGateCore::inhibit(",
        "MotionGateCore",
    )
    require_source_tokens(
        inhibit,
        ("retire_lease", "request_id_cache_"),
        "MotionGateCore::inhibit",
    )
    tick = function_body(
        source,
        "MotionGateCore::tick(",
        "MotionGateCore",
    )
    require_source_tokens(
        tick,
        (
            "now >= authority_deadline_",
            "now >= candidate_deadline_",
            "retire_lease",
            "zero_command",
        ),
        "MotionGateCore::tick",
    )


def validate_order(
    body: str,
    ordered_tokens: tuple[str, ...],
    context: str,
) -> None:
    cursor = -1
    for token in ordered_tokens:
        index = body.find(token, cursor + 1)
        if index < 0:
            raise MotionGateContractError(
                f"{context} must contain {token!r}"
            )
        if index <= cursor:
            raise MotionGateContractError(
                f"{context} has unsafe operation ordering near {token!r}"
            )
        cursor = index


def validate_node(path: Path) -> None:
    source = read_text(path)
    require_source_tokens(
        source,
        (
            "static_assert(RMW_GID_STORAGE_SIZE == 16",
            "std::chrono::steady_clock::now()",
            "create_wall_timer",
            "std::chrono::milliseconds(20)",
            "rclcpp::KeepLast(1)",
            ".best_effort()",
            ".durability_volatile()",
            "rclcpp::SystemDefaultsQoS()",
            '"/motion_gate/internal/control"',
            '"/motion_gate/internal/state"',
            '"/voice_nav_internal/motion_gate/candidate/lease_"',
            '"/diff_drive_controller/cmd_vel"',
            ".reliable()",
            ".transient_local()",
            "read_only = true",
            "rclcpp::MessageInfo",
            "publisher_gid",
            "discover_unique_writer_gid_on_topic",
            "SingleThreadedExecutor",
            "command.header.stamp",
            "get_clock()->now()",
        ),
        "motion_gate_node",
    )
    if re.search(r"\bget_gid\s*\(", source):
        raise MotionGateContractError(
            "OPEN must bind from Gate-local graph endpoint data, not a "
            "cross-process Publisher::get_gid value"
        )

    stamped_publishers = re.findall(
        r"create_publisher\s*<\s*geometry_msgs::msg::TwistStamped\s*>",
        source,
    )
    if len(stamped_publishers) != 1:
        raise MotionGateContractError(
            "motion_gate_node must create exactly one final TwistStamped "
            "publisher"
        )

    discovery = method_body(
        source,
        "MotionGateNode",
        "discover_unique_writer_gid_on_topic",
        "motion_gate_node",
    )
    require_source_tokens(
        discovery,
        (
            "get_publishers_info_by_topic",
            "endpoints.size() != 1",
            "endpoint_gid()",
        ),
        "MotionGate Gate-local writer discovery",
    )

    open_reader = method_body(
        source,
        "MotionGateNode",
        "open_candidate_reader",
        "motion_gate_node",
    )
    validate_order(
        open_reader,
        (
            "discover_unique_writer_gid_on_topic",
            "candidate_subscription_.reset()",
            "create_candidate_subscription",
            "bind_writer_gid",
            "core_.open(",
        ),
        "MotionGate OPEN queue barrier",
    )

    candidate = method_body(
        source,
        "MotionGateNode",
        "on_candidate",
        "motion_gate_node",
    )
    require_source_tokens(
        candidate,
        ("publisher_gid", "core_.accept_candidate("),
        "MotionGate candidate callback",
    )
    if "core_.renew(" in candidate:
        raise MotionGateContractError(
            "motion_gate_node candidate callback must not renew authority"
        )

    publication = method_body(
        source,
        "MotionGateNode",
        "publish_serialized",
        "motion_gate_node",
    )
    validate_order(
        publication,
        (
            "std::scoped_lock",
            "publication_mutex_",
            "final_command_publisher_->publish(",
        ),
        "MotionGate serialized publication point",
    )

    inhibit = method_body(
        source,
        "MotionGateNode",
        "handle_inhibit",
        "motion_gate_node",
    )
    validate_order(
        inhibit,
        (
            "core_.inhibit(",
            "publish_serialized(make_zero_command())",
            "response->motion_inhibited = true",
            "response->zero_published = true",
        ),
        "MotionGate INHIBIT acknowledgement",
    )


def validate_mission_cmake(path: Path) -> None:
    source = read_text(path)
    require_source_tokens(
        source,
        (
            "find_package(rosidl_default_generators REQUIRED)",
            "rosidl_generate_interfaces(${PROJECT_NAME}",
            '"msg/InternalMotionGateState.msg"',
            '"srv/InternalMotionGateControl.srv"',
            "add_library(motion_gate_core",
            "add_executable(motion_gate_node",
            "rosidl_get_typesupport_target(",
            "rosidl_typesupport_cpp",
            "target_link_libraries(motion_gate_node",
            "ament_add_gtest(",
            "install(",
            "motion_gate_node",
        ),
        "voice_nav_mission CMake",
    )
    if "rosidl_target_interfaces(" in source:
        raise MotionGateContractError(
            "voice_nav_mission must use rosidl_get_typesupport_target instead "
            "of deprecated rosidl_target_interfaces"
        )


def validate_bringup_cmake(path: Path) -> None:
    source = read_text(path)
    install_match = re.search(
        r"install\s*\(\s*DIRECTORY(?P<body>.*?)"
        r"DESTINATION\s+share/\$\{PROJECT_NAME\}",
        source,
        flags=re.DOTALL,
    )
    if install_match is None:
        raise MotionGateContractError(
            "voice_nav_bringup CMake must install launch and config"
        )
    installed = set(
        re.findall(r"\b(?:config|launch)\b", install_match.group("body"))
    )
    if installed != {"config", "launch"}:
        raise MotionGateContractError(
            "voice_nav_bringup CMake must install both config and launch"
        )


def call_name(call: ast.Call) -> str:
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return ""


def keyword_value(call: ast.Call, name: str) -> ast.expr | None:
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    return None


def literal_string(node: ast.expr | None) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def validate_product_launch(path: Path) -> None:
    source = read_text(path)
    try:
        tree = ast.parse(source, filename=str(path))
    except SyntaxError as error:
        raise MotionGateContractError(
            f"cannot parse product launch {path}: {error}"
        ) from error

    require_source_tokens(
        source,
        (
            "IncludeLaunchDescription",
            "PythonLaunchDescriptionSource",
            "FindPackageShare('voice_nav_sim')",
            "FindPackageShare('voice_nav_bringup')",
            "'simulation.launch.py'",
            "'motion_gate.yaml'",
            "PathJoinSubstitution",
        ),
        "MotionGate product launch",
    )
    forbidden = (
        "twist_mux",
        "teleop_twist_keyboard",
        "ros2 topic pub",
        "enable_stamped_cmd_vel",
    )
    present_forbidden = [token for token in forbidden if token in source]
    if present_forbidden:
        raise MotionGateContractError(
            "MotionGate product launch contains a bypass or unsupported "
            "control path: "
            + ", ".join(present_forbidden)
        )

    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
    ]
    gate_nodes = []
    for call in calls:
        if call_name(call) != "Node":
            continue
        package = literal_string(keyword_value(call, "package"))
        executable = literal_string(keyword_value(call, "executable"))
        if package == "voice_nav_mission" and executable == "motion_gate_node":
            gate_nodes.append(call)
    if len(gate_nodes) != 1:
        raise MotionGateContractError(
            "product launch must start exactly one motion_gate_node"
        )
    gate = gate_nodes[0]
    if literal_string(keyword_value(gate, "name")) != "motion_gate_node":
        raise MotionGateContractError(
            "product MotionGate node name must be motion_gate_node"
        )
    if keyword_value(gate, "parameters") is None:
        raise MotionGateContractError(
            "product motion_gate_node must load the trusted YAML"
        )
    if keyword_value(gate, "on_exit") is not None:
        raise MotionGateContractError(
            "motion_gate_node exit must leave simulation and the controller "
            "running so Lesson 0010 can verify the consumer deadman"
        )
    if keyword_value(gate, "respawn") is not None:
        raise MotionGateContractError(
            "motion_gate_node must not be automatically respawned"
        )


def validate_unique_final_publisher(root: Path, node_path: Path) -> None:
    final_topic = "/diff_drive_controller/cmd_vel"
    allowed = {
        node_path.resolve(),
        (root / ARTIFACTS["gate_config"]).resolve(),
    }
    offenders: list[str] = []
    source_root = root / "src"
    for path in source_root.rglob("*"):
        if not path.is_file() or path.suffix not in {
            ".cpp",
            ".hpp",
            ".h",
            ".py",
            ".yaml",
            ".yml",
        }:
            continue
        if "test" in path.parts or path.resolve() in allowed:
            continue
        if final_topic in read_text(path):
            offenders.append(path.relative_to(root).as_posix())
    if offenders:
        raise MotionGateContractError(
            "only motion_gate_node may name the final controller command "
            "endpoint in production source: "
            + ", ".join(sorted(offenders))
        )


def validate_contract(root: Path) -> None:
    paths = required_artifacts(root)
    validate_idl(
        paths["control_interface"],
        CONTROL_REQUIRED_LINES,
        "InternalMotionGateControl.srv",
    )
    control_lines = normalized_idl_lines(paths["control_interface"])
    separator = control_lines.index("---")
    if any("gid" in line.lower() for line in control_lines[:separator]):
        raise MotionGateContractError(
            "InternalMotionGateControl.srv must not carry a cross-process "
            "writer GID; OPEN binds in the Gate-local graph context"
        )
    validate_idl(
        paths["state_interface"],
        STATE_REQUIRED_LINES,
        "InternalMotionGateState.msg",
    )
    validate_private_idl_location(root)
    validate_mission_package(paths["mission_package"])
    validate_bringup_package(paths["bringup_package"])
    gate_values = validate_gate_parameters(paths["gate_config"])
    validate_controller_compatibility(
        gate_values,
        paths["controller_config"],
    )
    validate_core(paths["core_header"], paths["core_source"])
    validate_node(paths["node_source"])
    validate_mission_cmake(paths["mission_cmake"])
    validate_bringup_cmake(paths["bringup_cmake"])
    validate_product_launch(paths["product_launch"])
    validate_unique_final_publisher(root, paths["node_source"])


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    root = arguments.root.resolve()
    try:
        validate_contract(root)
    except MotionGateContractError as error:
        print(f"MotionGate contract failed: {error}", file=sys.stderr)
        return 1
    print("MotionGate contract passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
