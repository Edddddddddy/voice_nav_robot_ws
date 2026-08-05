#!/usr/bin/env python3
"""Validate the MotionGate product contract without starting ROS."""

from __future__ import annotations

import argparse
import ast
import math
import re
import sys
import xml.etree.ElementTree as element_tree
from pathlib import Path


class MotionGateContractError(ValueError):
    """A checked artifact violates the MotionGate contract."""


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
    "writer_observation_header": (
        "src/voice_nav_mission/src/writer_observation.hpp"
    ),
    "writer_observation_source": (
        "src/voice_nav_mission/src/writer_observation.cpp"
    ),
    "writer_observation_test": (
        "src/voice_nav_mission/test/writer_observation_test.cpp"
    ),
    "mission_package": "src/voice_nav_mission/package.xml",
    "mission_cmake": "src/voice_nav_mission/CMakeLists.txt",
    "gate_config": "src/voice_nav_bringup/config/motion_gate.yaml",
    "product_launch": (
        "src/voice_nav_bringup/launch/product_sim.launch.py"
    ),
    "bringup_package": "src/voice_nav_bringup/package.xml",
    "bringup_cmake": "src/voice_nav_bringup/CMakeLists.txt",
    "open_convergence": (
        "src/voice_nav_bringup/test/motion_gate_open_convergence.py"
    ),
    "controller_config": "src/voice_nav_sim/config/controllers.yaml",
}

CONTROL_REQUEST_CONSTANTS = {
    "uint8 PREPARE=1",
    "uint8 OPEN=2",
    "uint8 RENEW=3",
    "uint8 INHIBIT=4",
}

CONTROL_REQUEST_FIELDS = {
    "uint8 operation",
    "string<=36 request_id",
    "string<=36 gate_instance_id",
    "string<=36 lease_id",
    "uint64 expected_control_seq",
}

CONTROL_RESPONSE_CONSTANTS = {
    "uint16 APPLIED=0",
    "uint16 DUPLICATE=1",
    "uint16 REJECTED=2",
    "uint16 FAULTED=3",
}

REASON_CONSTANTS = {
    "uint16 NONE=0",
    "uint16 INVALID_REQUEST=1",
    "uint16 STALE_GATE=2",
    "uint16 STALE_SEQUENCE=3",
    "uint16 INVALID_STATE=4",
    "uint16 STALE_LEASE=5",
    "uint16 REQUEST_ID_COLLISION=6",
    "uint16 PREPARE_EXPIRED=7",
    "uint16 AUTHORITY_EXPIRED=8",
    "uint16 CANDIDATE_EXPIRED=9",
    "uint16 WRITER_UNAVAILABLE=10",
    "uint16 WRITER_AMBIGUOUS=11",
    "uint16 WRITER_MISMATCH=12",
    "uint16 WRITER_STILL_PRESENT=13",
    "uint16 INVALID_CANDIDATE=14",
    "uint16 SEQUENCE_EXHAUSTED=15",
    "uint16 CONFIGURATION_INVALID=16",
    "uint16 PUBLISH_FAILED=17",
    "uint16 INTERNAL_FAILURE=18",
    "uint16 WRITER_METADATA_PENDING=19",
}

CONTROL_RESPONSE_FIELDS = {
    "uint16 code",
    "uint16 reason",
    "string<=36 gate_instance_id",
    "uint64 control_seq",
    "uint8 state",
    "string<=36 lease_id",
    "string<=128 candidate_topic",
    "uint8[16] bound_writer_gid",
    "bool motion_inhibited",
    "bool authority_live",
    "bool candidate_fresh",
    "bool writer_bound",
    "bool zero_selected",
    "bool zero_published",
    "uint64 output_publish_seq",
    "uint64 zero_publish_seq",
    "string<=160 detail",
}

STATE_CONSTANTS = {
    "uint8 INHIBITED=0",
    "uint8 PREPARED=1",
    "uint8 ARMED=2",
    "uint8 FAULTED=3",
}

STATE_FIELDS = {
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
    "rcl_interfaces",
    "rclcpp",
    "rmw",
    "rmw_fastrtps_cpp",
    "rosidl_default_generators",
    "rosidl_default_runtime",
}

MISSION_TEST_DEPENDENCIES = {
    "ament_cmake_gtest",
    "launch_ros",
    "launch_testing",
    "launch_testing_ament_cmake",
}

BRINGUP_DEPENDENCIES = {
    "launch",
    "launch_ros",
    "rmw_fastrtps_cpp",
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
                f"missing MotionGate artifact: {relative}"
            )
    return resolved


def normalized_idl_lines(path: Path) -> list[str]:
    lines: list[str] = []
    for raw_line in read_text(path).splitlines():
        line = raw_line.split("#", maxsplit=1)[0].strip()
        if line:
            lines.append(re.sub(r"\s+", " ", line))
    return lines


def split_idl_sections(
    lines: list[str],
    expected_count: int,
    interface_name: str,
) -> list[list[str]]:
    separators = [
        index for index, line in enumerate(lines) if line == "---"
    ]
    if len(separators) != expected_count - 1:
        raise MotionGateContractError(
            f"{interface_name} must contain exactly "
            f"{expected_count} closed section(s)"
        )
    sections: list[list[str]] = []
    start = 0
    for separator in separators:
        sections.append(lines[start:separator])
        start = separator + 1
    sections.append(lines[start:])
    return sections


def validate_closed_idl_section(
    lines: list[str],
    expected_constants: set[str],
    expected_fields: set[str],
    context: str,
) -> None:
    def is_constant(line: str) -> bool:
        return re.match(
            r"^[^\s]+\s+[A-Za-z_][A-Za-z0-9_]*\s*=",
            line,
        ) is not None

    constants = [line for line in lines if is_constant(line)]
    fields = [line for line in lines if not is_constant(line)]
    duplicate_lines = sorted(
        line
        for line in set(lines)
        if lines.count(line) > 1
    )
    if duplicate_lines:
        raise MotionGateContractError(
            f"{context} contains duplicate declaration(s): "
            + ", ".join(duplicate_lines)
        )
    missing_constants = sorted(expected_constants - set(constants))
    extra_constants = sorted(set(constants) - expected_constants)
    missing_fields = sorted(expected_fields - set(fields))
    extra_fields = sorted(set(fields) - expected_fields)
    if (
        missing_constants
        or extra_constants
        or missing_fields
        or extra_fields
    ):
        details = []
        for label, values in (
            ("missing constants", missing_constants),
            ("unexpected constants", extra_constants),
            ("missing fields", missing_fields),
            ("unexpected fields", extra_fields),
        ):
            if values:
                details.append(f"{label}: {', '.join(values)}")
        raise MotionGateContractError(
            f"{context} is a closed private protocol; "
            + "; ".join(details)
        )


def validate_idl_common(
    path: Path,
    interface_name: str,
) -> list[str]:
    lines = normalized_idl_lines(path)
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
    unbounded_sequences = [
        line
        for line in lines
        if re.search(r"\[\s*\]", line)
        or re.search(r"\bsequence\s*<\s*[^,>]+\s*>", line)
    ]
    if unbounded_sequences:
        raise MotionGateContractError(
            f"{interface_name} must not contain unbounded sequences: "
            + ", ".join(unbounded_sequences)
        )
    return lines


def validate_control_idl(path: Path) -> None:
    interface_name = "InternalMotionGateControl.srv"
    sections = split_idl_sections(
        validate_idl_common(path, interface_name),
        2,
        interface_name,
    )
    if any("gid" in line.lower() for line in sections[0]):
        raise MotionGateContractError(
            f"{interface_name} must not carry a cross-process writer GID; "
            "OPEN binds in the Gate-local graph context"
        )
    validate_closed_idl_section(
        sections[0],
        CONTROL_REQUEST_CONSTANTS,
        CONTROL_REQUEST_FIELDS,
        f"{interface_name} request",
    )
    validate_closed_idl_section(
        sections[1],
        CONTROL_RESPONSE_CONSTANTS | REASON_CONSTANTS,
        CONTROL_RESPONSE_FIELDS,
        f"{interface_name} response",
    )


def validate_state_idl(path: Path) -> None:
    interface_name = "InternalMotionGateState.msg"
    sections = split_idl_sections(
        validate_idl_common(path, interface_name),
        1,
        interface_name,
    )
    validate_closed_idl_section(
        sections[0],
        STATE_CONSTANTS | REASON_CONSTANTS,
        STATE_FIELDS,
        interface_name,
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
    all_test_dependencies = package_dependencies(root, {"test_depend"})
    missing_test = sorted(
        MISSION_TEST_DEPENDENCIES - all_test_dependencies
    )
    if missing_test:
        raise MotionGateContractError(
            "voice_nav_mission is missing MotionGate test dependencies: "
            + ", ".join(missing_test)
        )
    runtime_dependencies = package_dependencies(root, {"exec_depend"})
    if "rmw_fastrtps_cpp" not in runtime_dependencies:
        raise MotionGateContractError(
            "voice_nav_mission must declare rmw_fastrtps_cpp as an "
            "exec_depend because the node runtime-checks that implementation"
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
    runtime_dependencies = package_dependencies(root, {"exec_depend"})
    if "rmw_fastrtps_cpp" not in runtime_dependencies:
        raise MotionGateContractError(
            "voice_nav_bringup must declare rmw_fastrtps_cpp as an "
            "exec_depend because product_sim.launch.py selects it"
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


def require_source_token_alternatives(
    source: str,
    alternatives: tuple[tuple[str, ...], ...],
    context: str,
) -> None:
    """Accept one of the approved equivalent source-shape marker sets."""
    for tokens in alternatives:
        if all(token in source for token in tokens):
            return
    missing = [token for token in alternatives[0] if token not in source]
    raise MotionGateContractError(
        f"{context} is missing contract marker(s): "
        + ", ".join(missing)
    )


def _cpp_translation_phase2(source: str) -> str:
    """Remove physical backslash-newline pairs before C++ tokenization."""
    return re.sub(r"\\(?:\r\n|\n|\r)", "", source)


def _cpp_lexical_views(source: str) -> tuple[str, str]:
    """Return same-length views of the phase-2 logical C++ source."""
    source = _cpp_translation_phase2(source)
    comment_free: list[str] = []
    code_only: list[str] = []

    def append_masked(fragment: str, preserve_literal: bool) -> None:
        if preserve_literal:
            comment_free.extend(fragment)
        else:
            comment_free.extend(
                "\n" if character == "\n" else " "
                for character in fragment
            )
        code_only.extend(
            "\n" if character == "\n" else " "
            for character in fragment
        )

    index = 0
    while index < len(source):
        following = source[index + 1] if index + 1 < len(source) else ""
        if source[index] == "/" and following == "/":
            end = source.find("\n", index + 2)
            end = len(source) if end < 0 else end
            append_masked(source[index:end], preserve_literal=False)
            index = end
            continue
        if source[index] == "/" and following == "*":
            closing = source.find("*/", index + 2)
            end = len(source) if closing < 0 else closing + 2
            append_masked(source[index:end], preserve_literal=False)
            index = end
            continue

        raw_match = re.match(r'(?:u8|u|U|L)?R"', source[index:])
        previous_is_identifier = index > 0 and (
            source[index - 1].isalnum() or source[index - 1] == "_"
        )
        if raw_match is not None and not previous_is_identifier:
            delimiter_start = index + len(raw_match.group(0))
            opening = source.find("(", delimiter_start, delimiter_start + 17)
            if opening >= 0:
                delimiter = source[delimiter_start:opening]
                delimiter_is_valid = not any(
                    character.isspace() or character in "()\\"
                    for character in delimiter
                )
                if delimiter_is_valid:
                    terminator = ")" + delimiter + '"'
                    closing = source.find(terminator, opening + 1)
                    end = (
                        len(source) if closing < 0
                        else closing + len(terminator)
                    )
                    append_masked(
                        source[index:end], preserve_literal=True
                    )
                    index = end
                    continue

        if source[index] in {'"', "'"}:
            quote = source[index]
            end = index + 1
            while end < len(source):
                if source[end] == "\\" and end + 1 < len(source):
                    end += 2
                    continue
                end += 1
                if source[end - 1] == quote:
                    break
            append_masked(source[index:end], preserve_literal=True)
            index = end
            continue

        comment_free.append(source[index])
        code_only.append(source[index])
        index += 1

    return "".join(comment_free), "".join(code_only)


def strip_cpp_comments(source: str) -> str:
    """Remove C++ comments while preserving literals and source offsets."""
    return _cpp_lexical_views(source)[0]


def cpp_code_mask(source: str) -> str:
    """Mask C++ comments and literals while preserving code source offsets."""
    return _cpp_lexical_views(source)[1]


def function_body(source: str, signature: str, context: str) -> str:
    comment_free, code_only = _cpp_lexical_views(source)
    signature_index = code_only.find(signature)
    if signature_index < 0:
        raise MotionGateContractError(
            f"{context} must define {signature}"
        )
    opening = code_only.find("{", signature_index)
    if opening < 0:
        raise MotionGateContractError(
            f"{context} has no body for {signature}"
        )
    depth = 0
    for index in range(opening, len(code_only)):
        character = code_only[index]
        if character == "{":
            depth += 1
        elif character == "}":
            depth -= 1
            if depth == 0:
                return comment_free[opening + 1 : index]
    raise MotionGateContractError(
        f"{context} has an unterminated body for {signature}"
    )


def reject_test_short_circuit(
    body: str,
    context: str,
    *,
    require_linear: bool = False,
) -> None:
    body_code = cpp_code_mask(body)
    if (
        re.search(r"\bGTEST_SKIP\s*\(", body_code)
        or re.search(r"\breturn\b", body_code)
    ):
        raise MotionGateContractError(
            f"{context} must execute without skip or early return"
        )
    if require_linear and re.search(
        r"\b(?:if|else|for|while|do|switch|goto|try|catch)\b",
        body_code,
    ):
        raise MotionGateContractError(
            f"{context} must remain a linear, unconditionally executed test"
        )


def require_top_level_source_snippets(
    source: str,
    snippets: tuple[str, ...],
    context: str,
) -> None:
    comment_free, code_only = _cpp_lexical_views(source)
    missing: list[str] = []
    for snippet in snippets:
        words = re.split(r"\s+", snippet.strip())
        pattern = re.compile(r"\s+".join(re.escape(word) for word in words))
        matched_at_top_level = False
        for match in pattern.finditer(comment_free):
            first_word = words[0]
            first_word_in_code = code_only[
                match.start() : match.start() + len(first_word)
            ]
            if first_word_in_code != first_word:
                continue
            prefix = code_only[: match.start()]
            if prefix.count("{") == prefix.count("}"):
                matched_at_top_level = True
                break
        if not matched_at_top_level:
            missing.append(snippet)
    if missing:
        raise MotionGateContractError(
            f"{context} must execute top-level statement(s): "
            + ", ".join(missing)
        )


def require_top_level_source_sequence(
    source: str,
    snippets: tuple[str, ...],
    context: str,
) -> None:
    comment_free, code_only = _cpp_lexical_views(source)
    search_start = 0
    for snippet in snippets:
        words = re.split(r"\s+", snippet.strip())
        pattern = re.compile(r"\s+".join(re.escape(word) for word in words))
        matching_position = None
        for match in pattern.finditer(comment_free, search_start):
            first_word = words[0]
            first_word_in_code = code_only[
                match.start() : match.start() + len(first_word)
            ]
            if first_word_in_code != first_word:
                continue
            prefix = code_only[: match.start()]
            if prefix.count("{") == prefix.count("}"):
                matching_position = match.end()
                break
        if matching_position is None:
            raise MotionGateContractError(
                f"{context} must execute ordered top-level statement: "
                + snippet
            )
        search_start = matching_position


def active_gtest_body(
    source: str,
    suite: str,
    name: str,
    context: str,
) -> str:
    cleaned, code_only = _cpp_lexical_views(source)
    if re.search(r"^\s*#\s*define\s+TEST\b", code_only, re.MULTILINE):
        raise MotionGateContractError(
            f"{context} must not redefine the TEST macro"
        )
    # This dedicated safety-regression source is intentionally free of all
    # conditional compilation, so a required TEST cannot be hidden by a
    # platform branch. The lexical view has already applied translation
    # phase 2, including removal of spliced physical newlines.
    if re.search(
        r"^\s*#\s*(?:if|ifdef|ifndef|elif)\b",
        code_only,
        re.MULTILINE,
    ):
        raise MotionGateContractError(
            f"{context}: dedicated test source forbids all conditional "
            "compilation"
        )
    signature_pattern = re.compile(
        rf"\bTEST\s*\(\s*{re.escape(suite)}\s*,\s*"
        rf"{re.escape(name)}\s*\)"
    )
    matches = list(signature_pattern.finditer(code_only))
    if len(matches) != 1:
        raise MotionGateContractError(
            f"{context} must define exactly one active TEST({suite}, {name})"
        )
    body = function_body(cleaned, matches[0].group(0), context)
    reject_test_short_circuit(body, context, require_linear=True)
    return body


def method_body(
    source: str,
    class_name: str,
    method_name: str,
    context: str,
) -> str:
    candidates = re.finditer(
        rf"\b{re.escape(method_name)}\s*\(",
        source,
    )
    for candidate in candidates:
        opening_parenthesis = source.find("(", candidate.start())
        depth = 0
        closing_parenthesis = -1
        for index in range(opening_parenthesis, len(source)):
            character = source[index]
            if character == "(":
                depth += 1
            elif character == ")":
                depth -= 1
                if depth == 0:
                    closing_parenthesis = index
                    break
        if closing_parenthesis < 0:
            continue
        suffix = re.match(
            r"\s*(?:const\s*)?(?:noexcept\s*)?\{",
            source[closing_parenthesis + 1 :],
        )
        if suffix is None:
            continue
        opening = closing_parenthesis + 1 + suffix.end() - 1
        brace_depth = 0
        for index in range(opening, len(source)):
            character = source[index]
            if character == "{":
                brace_depth += 1
            elif character == "}":
                brace_depth -= 1
                if brace_depth == 0:
                    return source[opening + 1 : index]
    raise MotionGateContractError(
        f"{context} must define {class_name}::{method_name}("
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
    require_source_token_alternatives(
        prepare,
        (
            (
                "State::Inhibited",
                "request_id_cache_",
                "cached->second.request_fingerprint",
                "request.expected_control_seq",
                "control_seq_",
                "make_lease_id",
            ),
            (
                "State::Inhibited",
                "replay_or_collision(request)",
                "request.expected_control_seq",
                "control_seq_",
                "make_lease_id",
            ),
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
    if open_body.count("binding_provider()") != 1:
        raise MotionGateContractError(
            "MotionGateCore::open must invoke the graph binding provider "
            "exactly once after pure validation"
        )
    validate_order(
        open_body,
        (
            "replay_or_collision(request)",
            "validate_common(request, Operation::Open",
            "state_ != State::Prepared",
            "request.expected_control_seq != control_seq_",
            "request.lease_id != lease_id_",
            "now >= prepare_deadline_",
            "!binding_provider",
            "binding_provider()",
            "!binding.ready",
            "binding.reason != Reason::None",
            "gid_is_nonzero(binding.writer_gid)",
        ),
        "MotionGateCore::open pure validation before graph provider",
    )
    contradictory_ready = function_body(
        open_body,
        "if (binding.reason != Reason::None)",
        "MotionGateCore::open contradictory ready binding",
    )
    validate_order(
        contradictory_ready,
        (
            "force_fault(",
            "Reason::InternalFailure",
            "result_from_snapshot(",
            "ResultCode::Faulted",
            "remember(request, fault)",
            "return fault",
        ),
        "MotionGateCore::open contradictory ready binding",
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
            "authority_deadline_ = now + authority_lease",
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
    require_source_token_alternatives(
        inhibit,
        (
            ("retire_lease", "request_id_cache_"),
            ("retire_lease", "replay_or_collision(request)"),
        ),
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


def parenthesized_call_bodies(
    source: str,
    call_token: str,
) -> list[str]:
    calls: list[str] = []
    cursor = 0
    while True:
        token_index = source.find(call_token, cursor)
        if token_index < 0:
            return calls
        opening = source.find("(", token_index + len(call_token))
        if opening < 0:
            raise MotionGateContractError(
                f"unterminated call marker {call_token}"
            )
        depth = 0
        quote = ""
        escaped = False
        for index in range(opening, len(source)):
            character = source[index]
            if quote:
                if escaped:
                    escaped = False
                elif character == "\\":
                    escaped = True
                elif character == quote:
                    quote = ""
                continue
            if character in {'"', "'"}:
                quote = character
            elif character == "(":
                depth += 1
            elif character == ")":
                depth -= 1
                if depth == 0:
                    calls.append(source[opening + 1 : index])
                    cursor = index + 1
                    break
        else:
            raise MotionGateContractError(
                f"unterminated call marker {call_token}"
            )


def validate_writer_observation(
    header_path: Path,
    source_path: Path,
    test_path: Path,
) -> None:
    header = read_text(header_path)
    source = read_text(source_path)
    test = read_text(test_path)

    require_source_tokens(
        header,
        (
            "struct WriterEndpointObservation",
            "std::string topic_type",
            "std::string node_name",
            "std::string node_namespace",
            "rmw_endpoint_type_t endpoint_type",
            "rmw_qos_profile_t qos",
            "WriterGid writer_gid",
            "struct WriterObservationPolicy",
            "class WriterObservationSession",
            "OpenBinding observe(",
            "void reset() noexcept",
            "std::optional<WriterGid> pinned_writer_gid_",
            "bool identity_confirmed_",
            "bool terminal_mismatch_",
            "std::string terminal_detail_",
        ),
        "private WriterObservationSession header",
    )
    require_source_tokens(
        source,
        (
            'include "writer_observation.hpp"',
            "constexpr std::size_t kMaximumDetailLength = 160U",
            "constexpr std::size_t kDigestSuffixLength = 9U",
            "constexpr std::size_t kSummaryFixedLength = 55U",
            "kMinimumSummaryLength",
            'constexpr char kUnknownNodeName[] = "_NODE_NAME_UNKNOWN_"',
            (
                'constexpr char kUnknownNodeNamespace[] = '
                '"_NODE_NAMESPACE_UNKNOWN_"'
            ),
            "detail.resize(kMaximumDetailLength)",
            "digest_text",
            "2166136261U",
            "16777619U",
            "compact_field",
            "observation_detail",
            "std::all_of(",
            "value == 0U",
            "candidate_qos_is_compatible",
            "normalized_namespace",
            "node_name_is_unresolved",
            "node_namespace_is_unresolved",
            "fqn_namespace",
            "fqn_name",
            "observation_summary",
            '"n=1 k="',
            '" id="',
            '" q="',
            '" g="',
            '" ms="',
            '" t="',
            "Reason::WriterMetadataPending",
            "Reason::WriterMismatch",
            "terminal_mismatch_ = true",
            "identity_confirmed_ = true",
            "pinned_writer_gid_ = endpoint.writer_gid",
            "return mismatch(terminal_detail_)",
            "void WriterObservationSession::reset() noexcept",
        ),
        "private WriterObservationSession implementation",
    )
    if source.count("Reason::WriterMetadataPending") != 1:
        raise MotionGateContractError(
            "WriterObservationSession may classify only unresolved node "
            "identity as WRITER_METADATA_PENDING"
        )

    unresolved_name = function_body(
        source,
        "bool node_name_is_unresolved",
        "WriterObservationSession unresolved node-name classifier",
    )
    require_source_tokens(
        unresolved_name,
        (
            "node_name.empty()",
            "node_name == kUnknownNodeName",
        ),
        "WriterObservationSession unresolved node-name classifier",
    )
    if re.fullmatch(
        r"\s*return\s+node_name\.empty\(\)\s*\|\|\s*"
        r"node_name\s*==\s*kUnknownNodeName\s*;\s*",
        unresolved_name,
    ) is None:
        raise MotionGateContractError(
            "WriterObservationSession may treat only an empty node name or "
            "the exact Jazzy unknown-name marker as unresolved"
        )
    unresolved_namespace = function_body(
        source,
        "bool node_namespace_is_unresolved",
        "WriterObservationSession unresolved namespace classifier",
    )
    require_source_tokens(
        unresolved_namespace,
        ("node_namespace == kUnknownNodeNamespace",),
        "WriterObservationSession unresolved namespace classifier",
    )
    if re.fullmatch(
        r"\s*return\s+node_namespace\s*==\s*"
        r"kUnknownNodeNamespace\s*;\s*",
        unresolved_namespace,
    ) is None:
        raise MotionGateContractError(
            "WriterObservationSession may treat only the exact Jazzy "
            "unknown-namespace marker as unresolved"
        )

    observe = method_body(
        source,
        "WriterObservationSession",
        "observe",
        "private WriterObservationSession implementation",
    )
    validate_order(
        observe,
        (
            "terminal_mismatch_",
            "endpoints.empty()",
            "endpoints.size() != 1U",
            "endpoint.endpoint_type",
            "endpoint.topic_type",
            "candidate_qos_is_compatible(endpoint.qos)",
            "gid_is_zero(endpoint.writer_gid)",
            "*pinned_writer_gid_ != endpoint.writer_gid",
            "node_name_is_unresolved(endpoint.node_name)",
            "node_namespace_is_unresolved(endpoint.node_namespace)",
            "fqn_namespace(policy_.expected_writer_fqn)",
            "fqn_name(policy_.expected_writer_fqn)",
            "!name_unresolved && endpoint.node_name != expected_name",
            "!namespace_unresolved",
            "observed_namespace != expected_namespace",
            "name_unresolved || namespace_unresolved",
            "endpoint_fqn(endpoint)",
            "identity_confirmed_ = true",
        ),
        "WriterObservationSession fail-closed classification",
    )
    pending_branch = function_body(
        observe,
        "if (name_unresolved || namespace_unresolved)",
        "WriterObservationSession unresolved node-identity branch",
    )
    require_source_tokens(
        pending_branch,
        (
            "identity_confirmed_",
            "true",
            "Reason::None",
            "!pinned_writer_gid_",
            "pinned_writer_gid_ = endpoint.writer_gid",
            "Reason::WriterMetadataPending",
            "endpoint.writer_gid",
            "summary",
        ),
        "WriterObservationSession unresolved node-identity branch",
    )
    validate_order(
        pending_branch,
        (
            "identity_confirmed_",
            "!pinned_writer_gid_",
            "pinned_writer_gid_ = endpoint.writer_gid",
            "Reason::WriterMetadataPending",
            "summary",
        ),
        "WriterObservationSession node-identity convergence",
    )

    reject_mismatch = function_body(
        observe,
        "[this](std::string detail)",
        "WriterObservationSession terminal mismatch closure",
    )
    require_source_tokens(
        reject_mismatch,
        (
            "if (pinned_writer_gid_)",
            "terminal_mismatch_ = true",
            "terminal_detail_ = detail",
            "return mismatch(",
        ),
        "WriterObservationSession terminal mismatch closure",
    )

    reset = method_body(
        source,
        "WriterObservationSession",
        "reset",
        "private WriterObservationSession implementation",
    )
    require_source_tokens(
        reset,
        (
            "pinned_writer_gid_.reset()",
            "identity_confirmed_ = false",
            "terminal_mismatch_ = false",
            "terminal_detail_.clear()",
        ),
        "WriterObservationSession generation reset",
    )
    require_source_tokens(
        test,
        (
            '#include "writer_observation.hpp"',
            "PinsUnresolvedIdentityUntilTheSameWriterResolves",
            "ReplacementPoisonsPinnedGenerationUntilReset",
            "ConfirmedSameGidSurvivesIdentityOnlyGraphRegression",
            "KnownWrongNamespaceCannotEnterPending",
            "ExactUnknownIdentityMarkersConvergeForPinnedGid",
            "KnownPartialIdentityMustAgreeBeforePending",
            '"_NODE_NAME_UNKNOWN_"',
            '"_NODE_NAMESPACE_UNKNOWN_"',
            "Reason::WriterMetadataPending",
            "Reason::WriterMismatch",
            "session.reset()",
            "EXPECT_LE(pending.detail.size(), 160U)",
            '"n=1"',
            '"t="',
            '"id="',
            '"q="',
            '"g="',
            '"ms=7"',
        ),
        "writer_observation_test",
    )

    diagnostic_helper = function_body(
        strip_cpp_comments(test),
        "void expect_complete_bounded_diagnostic(",
        "bounded writer diagnostic helper",
    )
    reject_test_short_circuit(
        diagnostic_helper,
        "bounded writer diagnostic helper",
    )
    require_source_tokens(
        diagnostic_helper,
        (
            "EXPECT_LE(observation.detail.size(), 160U)",
            '"n=1"',
            '" k="',
            '" id="',
            '" q="',
            '" g="',
            '" ms="',
            '" t="',
            "ASSERT_NE(positions[index], std::string::npos)",
            "ASSERT_LT(positions[index - 1U], positions[index])",
            "EXPECT_LT(value_start, value_end)",
            "const auto gid_start = positions[4] + 3U",
            "observation.detail.substr(",
            "gid_start, positions[5] - gid_start",
            "EXPECT_EQ(gid.size(), 32U)",
            'gid.find_first_not_of("0123456789abcdefABCDEF")',
            "std::string::npos",
        ),
        "bounded writer diagnostic helper",
    )
    require_top_level_source_sequence(
        diagnostic_helper,
        (
            "const auto gid_start = positions[4] + 3U;",
            (
                "const auto gid = observation.detail.substr( "
                "gid_start, positions[5] - gid_start);"
            ),
            "EXPECT_EQ(gid.size(), 32U);",
            (
                "EXPECT_EQ( gid.find_first_not_of("
                '"0123456789abcdefABCDEF"), std::string::npos);'
            ),
        ),
        "bounded writer diagnostic helper",
    )

    long_fields = active_gtest_body(
        test,
        "WriterObservationSession",
        "LongVariableFieldsPreserveEveryDiagnosticMarker",
        "bounded writer diagnostic regression",
    )
    require_source_tokens(
        long_fields,
        (
            "std::string(240U, 'n')",
            "std::string(240U, 's')",
            "std::string(240U, 't')",
            "expect_complete_bounded_diagnostic(long_name_rejected)",
            "expect_complete_bounded_diagnostic(long_namespace_rejected)",
            "expect_complete_bounded_diagnostic(long_type_rejected)",
            "first_name.back() = 'a'",
            "second_name.back() = 'b'",
            "EXPECT_NE(first.detail, second.detail)",
        ),
        "bounded writer diagnostic regression",
    )
    require_top_level_source_sequence(
        long_fields,
        (
            (
                "const auto long_name_rejected = long_name_session.observe( "
                "{endpoint(writer_gid(0x66U), std::string(240U, 'n'))}, "
                "123456ms);"
            ),
            "EXPECT_FALSE(long_name_rejected.ready);",
            (
                "EXPECT_EQ(long_name_rejected.reason, "
                "Reason::WriterMismatch);"
            ),
            "expect_complete_bounded_diagnostic(long_name_rejected);",
            (
                "const auto long_namespace_rejected = "
                "long_namespace_session.observe( { endpoint( "
                "writer_gid(0x67U), \"collision_monitor\", \"/\" + "
                "std::string(240U, 's'))}, 123456ms);"
            ),
            "EXPECT_FALSE(long_namespace_rejected.ready);",
            (
                "EXPECT_EQ(long_namespace_rejected.reason, "
                "Reason::WriterMismatch);"
            ),
            (
                "expect_complete_bounded_diagnostic("
                "long_namespace_rejected);"
            ),
            (
                "const auto long_type_rejected = long_type_session.observe( "
                "{WriterEndpointObservation{ std::string(240U, 't'), "
                '"collision_monitor", "/", RMW_ENDPOINT_PUBLISHER, '
                "candidate_qos(), writer_gid(0x68U)}}, 123456ms);"
            ),
            "EXPECT_FALSE(long_type_rejected.ready);",
            (
                "EXPECT_EQ(long_type_rejected.reason, "
                "Reason::WriterMismatch);"
            ),
            "expect_complete_bounded_diagnostic(long_type_rejected);",
        ),
        "bounded writer diagnostic observation flow",
    )

    terminal_replay = active_gtest_body(
        test,
        "WriterObservationSession",
        "PinnedReplacementAndTerminalReplayPreserveEveryDiagnosticMarker",
        "writer terminal diagnostic replay regression",
    )
    require_source_tokens(
        terminal_replay,
        (
            "Reason::WriterMetadataPending",
            "Reason::WriterMismatch",
            "expect_complete_bounded_diagnostic(replacement)",
            "expect_complete_bounded_diagnostic(replayed)",
            "EXPECT_EQ(replayed.detail, replacement.detail)",
        ),
        "writer terminal diagnostic replay regression",
    )


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
            "read_only = true",
            "rclcpp::MessageInfo",
            "publisher_gid",
            "discover_unique_writer_gid_on_topic",
            "WriterObservationSession writer_observation_session_",
            "SingleThreadedExecutor",
            "command.header.stamp",
            "get_clock()->now()",
            "add_on_set_parameters_callback",
            "MotionGate use_sim_time is immutable after startup",
            "use_sim_time runtime invariant was violated",
            "ros_time_is_active()",
            "command.header.stamp.sec = 0",
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

    state_qos_start = source.find("auto state_qos")
    state_publisher_start = source.find(
        "state_publisher_",
        state_qos_start + 1,
    )
    state_publisher_end = source.find(";", state_publisher_start)
    if (
        state_qos_start < 0
        or state_publisher_start < 0
        or state_publisher_end < 0
    ):
        raise MotionGateContractError(
            "motion_gate_node must construct the state publisher from a "
            "dedicated state_qos"
        )
    state_publisher_construction = source[
        state_qos_start : state_publisher_end + 1
    ]
    require_source_tokens(
        state_publisher_construction,
        (
            "rclcpp::QoS(rclcpp::KeepLast(1))",
            ".reliable()",
            ".transient_local()",
            "state_publisher_",
            "create_publisher<StateMessage>",
            "state_qos",
        ),
        "MotionGate state publisher construction",
    )
    validate_order(
        state_publisher_construction,
        (
            "rclcpp::QoS(rclcpp::KeepLast(1))",
            ".reliable()",
            ".transient_local()",
            "state_publisher_",
            "create_publisher<StateMessage>",
        ),
        "MotionGate state publisher QoS",
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
            "std::vector<WriterEndpointObservation> observations",
            "observations.reserve(endpoints.size())",
            "for (const auto & endpoint : endpoints)",
            "endpoint.topic_type()",
            "endpoint.node_name()",
            "endpoint.node_namespace()",
            "endpoint.endpoint_type()",
            "endpoint.qos_profile().get_rmw_qos_profile()",
            "endpoint_gid()",
            "std::copy(",
            "writer_gid.begin()",
            "writer_observation_started_at_",
            "writer_observation_session_.observe(observations, elapsed)",
        ),
        "MotionGate Gate-local TopicEndpointInfo adapter",
    )

    open_reader = method_body(
        source,
        "MotionGateNode",
        "open_candidate_reader",
        "motion_gate_node",
    )
    core_open_index = open_reader.find("core_.open(")
    if core_open_index < 0:
        raise MotionGateContractError(
            "MotionGate OPEN adapter must delegate admission to core_.open"
        )
    provider = function_body(
        open_reader,
        "[this, &request, &expected_binding]()",
        "MotionGate OPEN graph provider",
    )
    validate_order(
        provider,
        (
            "final_controller_health_error()",
            "discover_unique_writer_gid_on_topic",
        ),
        "MotionGate OPEN graph provider health-before-observation",
    )
    graph_before_core = open_reader[:core_open_index]
    forbidden_graph_before_core = [
        token
        for token in (
            "discover_unique_writer_gid_on_topic",
            "create_candidate_subscription",
            "final_controller_health_error",
        )
        if token in graph_before_core
    ]
    if forbidden_graph_before_core:
        raise MotionGateContractError(
            "MotionGate OPEN must enter Core validation before touching the "
            "DDS graph: "
            + ", ".join(forbidden_graph_before_core)
        )
    if open_reader.count("discover_unique_writer_gid_on_topic") != 3:
        raise MotionGateContractError(
            "MotionGate OPEN must take exactly three "
            "discover_unique_writer_gid_on_topic snapshots"
        )
    validate_order(
        open_reader,
        (
            "core_.open(",
            "final_controller_health_error()",
            "discover_unique_writer_gid_on_topic",
            "candidate_subscription_.reset()",
            "create_candidate_subscription",
            "discover_unique_writer_gid_on_topic",
            "result.code != ResultCode::Applied",
            "candidate_subscription_.reset()",
            "create_candidate_subscription",
            "discover_unique_writer_gid_on_topic",
        ),
        "MotionGate OPEN queue barrier with three snapshots",
    )
    reader_calls = parenthesized_call_bodies(
        open_reader,
        "create_candidate_subscription",
    )
    if (
        len(reader_calls) != 2
        or re.search(r",\s*true\s*$", reader_calls[0]) is None
        or re.search(r",\s*false\s*$", reader_calls[1]) is None
    ):
        raise MotionGateContractError(
            "MotionGate OPEN must build a discard reader inside the "
            "validated provider and an accepting reader only after APPLIED"
        )

    prepare = method_body(
        source,
        "MotionGateNode",
        "handle_prepare",
        "motion_gate_node",
    )
    applied_prepare = function_body(
        prepare,
        "if (result.code == ResultCode::Applied)",
        "MotionGate successful PREPARE adapter transition",
    )
    validate_order(
        applied_prepare,
        (
            "writer_observation_session_.reset()",
            "writer_observation_started_at_ = now",
            "create_candidate_subscription",
        ),
        "MotionGate successful PREPARE writer-observation reset",
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
    require_source_tokens(
        inhibit,
        (
            "core_.inhibit(",
            "reconcile_adapter_transition",
        ),
        "MotionGate INHIBIT adapter transition",
    )
    control = method_body(
        source,
        "MotionGateNode",
        "on_control",
        "motion_gate_node",
    )
    validate_order(
        control,
        (
            "case Operation::Inhibit",
            "result = handle_inhibit(",
            "core_.tick(",
            "publish_serialized(command)",
            "publish_state()",
            "fill_response(",
        ),
        "MotionGate INHIBIT zero-before-response linearization",
    )
    fill_response = method_body(
        source,
        "MotionGateNode",
        "fill_response",
        "motion_gate_node",
    )
    require_source_tokens(
        fill_response,
        (
            "motion_inhibited",
            "zero_published",
            "output_publish_seq",
            "zero_publish_seq",
        ),
        "MotionGate control response acknowledgement",
    )


def cmake_call_bodies(source: str, command: str) -> list[str]:
    uncommented = "\n".join(
        line.split("#", maxsplit=1)[0] for line in source.splitlines()
    )
    calls: list[str] = []
    for match in re.finditer(
        rf"\b{re.escape(command)}\s*\(",
        uncommented,
        flags=re.IGNORECASE,
    ):
        opening = uncommented.find("(", match.start())
        depth = 0
        quote = ""
        escaped = False
        for index in range(opening, len(uncommented)):
            character = uncommented[index]
            if quote:
                if escaped:
                    escaped = False
                elif character == "\\":
                    escaped = True
                elif character == quote:
                    quote = ""
                continue
            if character in {'"', "'"}:
                quote = character
            elif character == "(":
                depth += 1
            elif character == ")":
                depth -= 1
                if depth == 0:
                    calls.append(uncommented[opening + 1 : index])
                    break
        else:
            raise MotionGateContractError(
                f"CMake has an unterminated {command}( call"
            )
    return calls


def cmake_arguments(body: str) -> list[str]:
    return [
        token[1:-1]
        if len(token) >= 2 and token[0] == token[-1] in {'"', "'"}
        else token
        for token in re.findall(
            r'"(?:\\.|[^"])*"|\'(?:\\.|[^\'])*\'|[^\s]+',
            body,
        )
    ]


def validate_launch_test_registration(
    source: str,
    package_name: str,
    expected_path: str | tuple[str, ...],
    timeout_seconds: int | tuple[int, ...],
) -> None:
    launch_test_position = source.find("add_launch_test")
    for dependency in (
        "ament_cmake_ros",
        "launch_testing_ament_cmake",
    ):
        dependency_position = source.find(
            f"find_package({dependency} REQUIRED)"
        )
        if dependency_position < 0 or not (
            dependency_position < launch_test_position
        ):
            raise MotionGateContractError(
                f"{package_name} CMake must require {dependency} "
                "before add_launch_test"
            )
    launch_tests = cmake_call_bodies(source, "add_launch_test")
    expected_paths = (expected_path,) if isinstance(expected_path, str) else expected_path
    expected_timeouts = (
        (timeout_seconds,) if isinstance(timeout_seconds, int) else timeout_seconds
    )
    if len(expected_paths) != len(expected_timeouts):
        raise MotionGateContractError(
            f"{package_name} launch-test contract has mismatched path and timeout counts"
        )
    if len(launch_tests) != len(expected_paths):
        if len(expected_paths) == 1:
            raise MotionGateContractError(
                f"{package_name} CMake must register exactly one "
                "add_launch_test"
            )
        raise MotionGateContractError(
            f"{package_name} CMake must register exactly "
            f"{len(expected_paths)} approved add_launch_test registrations"
        )
    for index, (registered, path, timeout) in enumerate(
        zip(launch_tests, expected_paths, expected_timeouts)
    ):
        actual_arguments = cmake_arguments(registered)
        expected_arguments = [
            path,
            "TIMEOUT",
            str(timeout),
            "RUNNER",
            "${ament_cmake_ros_DIR}/run_test_isolated.py",
        ]
        if actual_arguments != expected_arguments:
            raise MotionGateContractError(
                f"{package_name} add_launch_test #{index + 1} must be exactly "
                f"{path} TIMEOUT {timeout} with the official isolated RUNNER; "
                "found " + " ".join(actual_arguments)
            )

    properties_calls = cmake_call_bodies(source, "set_tests_properties")
    for path in expected_paths:
        generated_test_name = f"test_{Path(path).name}"
        matching_properties = [
            cmake_arguments(body)
            for body in properties_calls
            if cmake_arguments(body)[:1] == [generated_test_name]
        ]
        if any(
            forbidden in properties
            for properties in matching_properties
            for forbidden in (
                "DISABLED",
                "PASS_REGULAR_EXPRESSION",
                "SKIP_REGULAR_EXPRESSION",
                "SKIP_RETURN_CODE",
                "WILL_FAIL",
            )
        ):
            raise MotionGateContractError(
                f"{package_name} launch test {generated_test_name} "
                "must remain enabled"
            )
        run_serial_properties = [
            properties
            for properties in matching_properties
            if "RUN_SERIAL" in properties
        ]
        if len(run_serial_properties) != 1:
            raise MotionGateContractError(
                f"{package_name} launch test {generated_test_name} must have "
                "one set_tests_properties call with RUN_SERIAL TRUE"
            )
        properties = run_serial_properties[0]
        run_serial_indices = [
            index
            for index, token in enumerate(properties)
            if token == "RUN_SERIAL"
        ]
        if (
            run_serial_indices != [len(properties) - 2]
            or properties[-1] != "TRUE"
            or "PROPERTIES" not in properties
        ):
            raise MotionGateContractError(
                f"{package_name} launch test {generated_test_name} must set "
                "RUN_SERIAL TRUE"
            )

        isolation_properties = [
            properties
            for properties in matching_properties
            if "ENVIRONMENT_MODIFICATION" in properties
        ]
        expected_isolation_properties = [
            generated_test_name,
            "PROPERTIES",
            "ENVIRONMENT_MODIFICATION",
            "ROS_DOMAIN_ID=unset:;DISABLE_ROS_ISOLATION=unset:",
        ]
        if isolation_properties != [expected_isolation_properties]:
            raise MotionGateContractError(
                f"{package_name} launch test {generated_test_name} must keep "
                "one exact process-scoped Domain isolation reset"
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
            "rosidl_get_typesupport_target(",
            "rosidl_typesupport_cpp",
            "ament_add_gtest(",
            "install(",
            "motion_gate_node",
        ),
        "voice_nav_mission CMake",
    )
    core_library_calls = [
        cmake_arguments(body)
        for body in cmake_call_bodies(source, "add_library")
        if cmake_arguments(body)[:1] == ["motion_gate_core"]
    ]
    expected_core_library = [
        "motion_gate_core",
        "STATIC",
        "src/motion_gate_core.cpp",
    ]
    if core_library_calls != [expected_core_library]:
        raise MotionGateContractError(
            "motion_gate_core must be one internal STATIC target built from "
            "src/motion_gate_core.cpp"
        )

    for body in cmake_call_bodies(source, "install"):
        arguments = cmake_arguments(body)
        if any("writer_observation" in argument for argument in arguments):
            raise MotionGateContractError(
                "writer observation adapter and session must remain "
                "package-private"
            )
        if any("motion_gate_core" in argument for argument in arguments):
            if any(
                argument.endswith("motion_gate_core.hpp")
                for argument in arguments
            ):
                raise MotionGateContractError(
                    "motion_gate_core.hpp must not be installed"
                )
            raise MotionGateContractError(
                "motion_gate_core must not be installed"
            )
        if (
            arguments[:1] == ["DIRECTORY"]
            and any(argument.rstrip("/") == "include" for argument in arguments)
        ):
            raise MotionGateContractError(
                "motion_gate_core.hpp must not be installed through the "
                "package include directory"
            )

    for command in (
        "ament_export_targets",
        "ament_export_libraries",
        "ament_export_include_directories",
        "export",
    ):
        for body in cmake_call_bodies(source, command):
            arguments = cmake_arguments(body)
            if any("writer_observation" in argument for argument in arguments):
                raise MotionGateContractError(
                    "writer observation adapter and session must remain "
                    "package-private"
                )
            if any("motion_gate_core" in argument for argument in arguments):
                raise MotionGateContractError(
                    "motion_gate_core must not be exported"
                )
            if (
                command == "ament_export_include_directories"
                and any(
                    argument.rstrip("/") == "include"
                    for argument in arguments
                )
            ):
                raise MotionGateContractError(
                    "motion_gate_core.hpp must not be exported through the "
                    "package include directory"
                )

    node_executable_calls = [
        cmake_arguments(body)
        for body in cmake_call_bodies(source, "add_executable")
        if cmake_arguments(body)[:1] == ["motion_gate_node"]
    ]
    expected_node_executable = [
        "motion_gate_node",
        "src/motion_gate_node.cpp",
        "src/writer_observation.cpp",
    ]
    if node_executable_calls != [expected_node_executable]:
        raise MotionGateContractError(
            "motion_gate_node must compile the private "
            "src/writer_observation.cpp adapter"
        )

    writer_test_calls = [
        cmake_arguments(body)
        for body in cmake_call_bodies(source, "ament_add_gtest")
        if cmake_arguments(body)[:1] == ["writer_observation_test"]
    ]
    expected_writer_test = [
        "writer_observation_test",
        "test/writer_observation_test.cpp",
        "src/writer_observation.cpp",
    ]
    if writer_test_calls != [expected_writer_test]:
        raise MotionGateContractError(
            "writer_observation_test must compile the same private "
            "writer_observation.cpp implementation"
        )

    target_patterns = {
        r"add_executable\s*\(\s*motion_gate_node\b": (
            "CMake must build motion_gate_node"
        ),
        r"target_link_libraries\s*\(\s*motion_gate_node\b": (
            "motion_gate_node must link its Core and generated typesupport"
        ),
    }
    for pattern, error_message in target_patterns.items():
        if re.search(pattern, source, flags=re.DOTALL) is None:
            raise MotionGateContractError(error_message)
    if "rosidl_target_interfaces(" in source:
        raise MotionGateContractError(
            "voice_nav_mission must use rosidl_get_typesupport_target instead "
            "of deprecated rosidl_target_interfaces"
        )
    if "test/test_mission_runtime_node.py" in source:
        launch_test_paths = (
            "test/test_motion_gate_node.py",
            "test/test_mission_runtime_node.py",
        )
        launch_test_timeouts = (60, 60)
    else:
        launch_test_paths = "test/test_motion_gate_node.py"
        launch_test_timeouts = 60
    validate_launch_test_registration(
        source,
        "voice_nav_mission",
        launch_test_paths,
        launch_test_timeouts,
    )


def validate_bringup_cmake(path: Path) -> None:
    source = read_text(path)
    validate_launch_test_registration(
        source,
        "voice_nav_bringup",
        "test/test_motion_gate_product.py",
        180,
    )
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


def validate_open_convergence(path: Path) -> None:
    context = "MotionGate OPEN immediate post-attempt deadline"
    try:
        tree = ast.parse(read_text(path), filename=str(path))
    except SyntaxError as error:
        raise MotionGateContractError(
            f"{path.name} is not valid Python: {error}"
        ) from error

    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "converge_open"
    ]
    if len(functions) != 1:
        raise MotionGateContractError(
            f"{context} requires exactly one converge_open function"
        )
    loops = [
        statement
        for statement in functions[0].body
        if isinstance(statement, ast.While)
        and isinstance(statement.test, ast.Constant)
        and statement.test.value is True
    ]
    if len(loops) != 1:
        raise MotionGateContractError(
            f"{context} requires exactly one top-level while True loop"
        )
    statements = loops[0].body

    def simple_assignment(
        statement: ast.stmt,
        target_name: str,
        value_text: str,
    ) -> bool:
        return (
            isinstance(statement, ast.Assign)
            and len(statement.targets) == 1
            and isinstance(statement.targets[0], ast.Name)
            and statement.targets[0].id == target_name
            and ast.unparse(statement.value) == value_text
        )

    response_indices = [
        index
        for index, statement in enumerate(statements)
        if simple_assignment(
            statement,
            "response",
            "attempt(request_id, remaining)",
        )
    ]
    if len(response_indices) != 1:
        raise MotionGateContractError(
            f"{context} requires one response = attempt(...) statement"
        )
    response_index = response_indices[0]
    if response_index + 4 >= len(statements):
        raise MotionGateContractError(
            f"{context} check is missing after the attempt"
        )

    last_response = statements[response_index + 1]
    attempts = statements[response_index + 2]
    remaining = statements[response_index + 3]
    deadline_check = statements[response_index + 4]
    if not simple_assignment(last_response, "last_response", "response"):
        raise MotionGateContractError(
            f"{context} must record last_response first"
        )
    if not (
        isinstance(attempts, ast.AugAssign)
        and isinstance(attempts.target, ast.Name)
        and attempts.target.id == "attempts"
        and isinstance(attempts.op, ast.Add)
        and isinstance(attempts.value, ast.Constant)
        and attempts.value.value == 1
    ):
        raise MotionGateContractError(
            f"{context} must increment attempts before checking time"
        )
    if not simple_assignment(remaining, "remaining", "deadline - now()"):
        raise MotionGateContractError(
            f"{context} must immediately recompute remaining time"
        )
    if not (
        isinstance(deadline_check, ast.If)
        and ast.unparse(deadline_check.test) == "remaining <= 0.0"
        and len(deadline_check.body) == 1
        and isinstance(deadline_check.body[0], ast.Raise)
        and isinstance(deadline_check.body[0].exc, ast.Call)
        and ast.unparse(deadline_check.body[0].exc)
        == "OpenConvergenceTimeout(last_response, attempts)"
    ):
        raise MotionGateContractError(
            f"{context} must immediately raise the typed timeout"
        )

    terminal_indices = [
        index
        for index, statement in enumerate(statements)
        if isinstance(statement, ast.If)
        and ast.unparse(statement.test)
        == "not _is_writer_discovery_pending(response, protocol)"
        and len(statement.body) == 1
        and isinstance(statement.body[0], ast.Return)
        and ast.unparse(statement.body[0].value) == "response"
    ]
    pending_validation_indices = [
        index
        for index, statement in enumerate(statements)
        if isinstance(statement, ast.Expr)
        and isinstance(statement.value, ast.Call)
        and ast.unparse(statement.value)
        == "_validate_pending_snapshot(response, expected, protocol)"
    ]
    if (
        len(terminal_indices) != 1
        or len(pending_validation_indices) != 1
        or not (
            response_index + 4
            < terminal_indices[0]
            < pending_validation_indices[0]
        )
    ):
        raise MotionGateContractError(
            f"{context} must precede terminal return and pending validation"
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


def assigned_call_names(tree: ast.AST) -> dict[int, str]:
    names: dict[int, str] = {}
    for assignment in ast.walk(tree):
        if not isinstance(assignment, ast.Assign):
            continue
        if not isinstance(assignment.value, ast.Call):
            continue
        if len(assignment.targets) != 1:
            continue
        target = assignment.targets[0]
        if isinstance(target, ast.Name):
            names[id(assignment.value)] = target.id
    return names


def assigned_values(tree: ast.AST) -> dict[str, ast.expr]:
    values: dict[str, ast.expr] = {}
    for assignment in ast.walk(tree):
        if not isinstance(assignment, ast.Assign):
            continue
        if len(assignment.targets) != 1:
            continue
        target = assignment.targets[0]
        if isinstance(target, ast.Name):
            values[target.id] = assignment.value
    return values


def path_join_matches(
    node: ast.expr,
    package_name: str,
    path_parts: tuple[str, ...],
) -> bool:
    if not isinstance(node, ast.Call):
        return False
    if call_name(node) != "PathJoinSubstitution":
        return False
    if len(node.args) != 1 or node.keywords:
        return False
    parts = node.args[0]
    if not isinstance(parts, (ast.List, ast.Tuple)):
        return False
    if len(parts.elts) != len(path_parts) + 1:
        return False
    package = parts.elts[0]
    if not isinstance(package, ast.Call):
        return False
    if call_name(package) != "FindPackageShare":
        return False
    if (
        len(package.args) != 1
        or package.keywords
        or literal_string(package.args[0]) != package_name
    ):
        return False
    return tuple(
        literal_string(part) for part in parts.elts[1:]
    ) == path_parts


def launch_description_returned_actions(tree: ast.AST) -> list[ast.expr]:
    functions = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
        and node.name == "generate_launch_description"
    ]
    if len(functions) != 1:
        raise MotionGateContractError(
            "product launch must define exactly one "
            "generate_launch_description"
        )
    returns = [
        node
        for node in ast.walk(functions[0])
        if isinstance(node, ast.Return)
    ]
    if len(returns) != 1 or not isinstance(returns[0].value, ast.Call):
        raise MotionGateContractError(
            "generate_launch_description must directly return one "
            "LaunchDescription"
        )
    description = returns[0].value
    if call_name(description) != "LaunchDescription":
        raise MotionGateContractError(
            "generate_launch_description must return LaunchDescription"
        )
    if len(description.args) != 1 or description.keywords:
        raise MotionGateContractError(
            "product LaunchDescription must receive one explicit action list"
        )
    actions = description.args[0]
    if not isinstance(actions, (ast.List, ast.Tuple)):
        raise MotionGateContractError(
            "product LaunchDescription actions must be an explicit list"
        )
    return list(actions.elts)


def returned_call_position(
    call: ast.Call,
    assigned_names: dict[int, str],
    returned_actions: list[ast.expr],
) -> int | None:
    assigned_name = assigned_names.get(id(call))
    for index, action in enumerate(returned_actions):
        if action is call:
            return index
        if (
            assigned_name is not None
            and isinstance(action, ast.Name)
            and action.id == assigned_name
        ):
            return index
    return None


def environment_action_values(call: ast.Call) -> tuple[str | None, str | None]:
    if len(call.args) > 2:
        return None, None
    name_node = (
        call.args[0]
        if call.args
        else keyword_value(call, "name")
    )
    value_node = (
        call.args[1]
        if len(call.args) > 1
        else keyword_value(call, "value")
    )
    allowed_keywords = {"name", "value"}
    if any(
        keyword.arg not in allowed_keywords for keyword in call.keywords
    ):
        return None, None
    return literal_string(name_node), literal_string(value_node)


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
    assigned_names = assigned_call_names(tree)
    assignments = assigned_values(tree)
    returned_actions = launch_description_returned_actions(tree)

    simulation_calls = [
        call for call in calls if call_name(call) == "IncludeLaunchDescription"
    ]
    if len(simulation_calls) != 1:
        raise MotionGateContractError(
            "product launch must construct exactly one simulation include"
        )
    simulation_sources = [
        node
        for node in ast.walk(simulation_calls[0])
        if isinstance(node, ast.Call)
        and call_name(node) == "PythonLaunchDescriptionSource"
    ]
    if (
        len(simulation_sources) != 1
        or len(simulation_sources[0].args) != 1
        or not path_join_matches(
            simulation_sources[0].args[0],
            "voice_nav_sim",
            ("launch", "simulation.launch.py"),
        )
    ):
        raise MotionGateContractError(
            "returned simulation action must include the installed "
            "voice_nav_sim/launch/simulation.launch.py"
        )
    simulation_position = returned_call_position(
        simulation_calls[0],
        assigned_names,
        returned_actions,
    )
    if simulation_position is None:
        raise MotionGateContractError(
            "product launch must actually return the simulation action"
        )

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
    gate_position = returned_call_position(
        gate,
        assigned_names,
        returned_actions,
    )
    if gate_position is None:
        raise MotionGateContractError(
            "product launch must actually return the motion_gate action"
        )
    if literal_string(keyword_value(gate, "name")) != "motion_gate_node":
        raise MotionGateContractError(
            "product MotionGate node name must be motion_gate_node"
        )
    parameters = keyword_value(gate, "parameters")
    if not (
        isinstance(parameters, ast.List)
        and len(parameters.elts) == 1
        and isinstance(parameters.elts[0], ast.Name)
        and parameters.elts[0].id == "gate_config"
    ):
        raise MotionGateContractError(
            "product motion_gate_node parameters must be exactly [gate_config]"
        )
    gate_config = assignments.get("gate_config")
    if gate_config is None or not path_join_matches(
        gate_config,
        "voice_nav_bringup",
        ("config", "motion_gate.yaml"),
    ):
        raise MotionGateContractError(
            "gate_config must resolve the installed trusted "
            "voice_nav_bringup/config/motion_gate.yaml"
        )
    if keyword_value(gate, "remappings") is not None:
        raise MotionGateContractError(
            "product motion_gate_node must not accept endpoint remappings"
        )
    if keyword_value(gate, "on_exit") is not None:
        raise MotionGateContractError(
            "motion_gate_node exit must leave simulation and the controller "
            "running so process-death tests can verify the consumer deadman"
        )
    if keyword_value(gate, "respawn") is not None:
        raise MotionGateContractError(
            "motion_gate_node must not be automatically respawned"
        )

    rmw_actions = [
        call
        for call in calls
        if call_name(call) == "SetEnvironmentVariable"
    ]
    if len(rmw_actions) != 1:
        raise MotionGateContractError(
            "product launch must construct exactly one locked RMW "
            "SetEnvironmentVariable action"
        )
    rmw_action = rmw_actions[0]
    if environment_action_values(rmw_action) != (
        "RMW_IMPLEMENTATION",
        "rmw_fastrtps_cpp",
    ):
        raise MotionGateContractError(
            "product RMW action must unconditionally set "
            "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
        )
    rmw_position = returned_call_position(
        rmw_action,
        assigned_names,
        returned_actions,
    )
    if rmw_position is None:
        raise MotionGateContractError(
            "product launch must actually return the locked RMW action"
        )
    if not (
        rmw_position < simulation_position
        and rmw_position < gate_position
    ):
        raise MotionGateContractError(
            "locked RMW action must execute before simulation and MotionGate"
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
    validate_control_idl(paths["control_interface"])
    validate_state_idl(paths["state_interface"])
    validate_private_idl_location(root)
    validate_mission_package(paths["mission_package"])
    validate_bringup_package(paths["bringup_package"])
    gate_values = validate_gate_parameters(paths["gate_config"])
    validate_controller_compatibility(
        gate_values,
        paths["controller_config"],
    )
    validate_core(paths["core_header"], paths["core_source"])
    validate_writer_observation(
        paths["writer_observation_header"],
        paths["writer_observation_source"],
        paths["writer_observation_test"],
    )
    validate_node(paths["node_source"])
    validate_mission_cmake(paths["mission_cmake"])
    validate_bringup_cmake(paths["bringup_cmake"])
    validate_open_convergence(paths["open_convergence"])
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
