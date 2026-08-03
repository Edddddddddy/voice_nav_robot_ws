#!/usr/bin/env python3
"""Validate the closed authority/candidate crash-fixture topology."""

from __future__ import annotations

import argparse
import ast
import re
import sys
import xml.etree.ElementTree as element_tree
from pathlib import Path


class FaultProducerContractError(ValueError):
    """A test-only producer artifact weakens process-death evidence."""


ARTIFACTS = {
    "actions": (
        "src/voice_nav_sim/test_support/fault_producer_actions.py"
    ),
    "helper": "src/voice_nav_sim/test/fault_producer.py",
    "launch_test": "src/voice_nav_sim/test/test_fault_producer_pair.py",
    "protocol_test": (
        "src/voice_nav_sim/test/test_fault_producer_protocol.py"
    ),
    "cmake": "src/voice_nav_sim/CMakeLists.txt",
    "package": "src/voice_nav_sim/package.xml",
}


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise FaultProducerContractError(
            f"cannot read {path}: {error}"
        ) from error


def parse_python(path: Path) -> tuple[str, ast.Module]:
    source = read_text(path)
    try:
        return source, ast.parse(source, filename=str(path))
    except SyntaxError as error:
        raise FaultProducerContractError(
            f"cannot parse {path}: {error}"
        ) from error


def required_artifacts(root: Path) -> dict[str, Path]:
    paths = {name: root / relative for name, relative in ARTIFACTS.items()}
    missing = [
        path.relative_to(root).as_posix()
        for path in paths.values()
        if not path.is_file()
    ]
    if missing:
        raise FaultProducerContractError(
            "missing fault-producer artifact(s): " + ", ".join(missing)
        )
    return paths


def final_call_name(call: ast.Call) -> str:
    function = call.func
    if isinstance(function, ast.Name):
        return function.id
    if isinstance(function, ast.Attribute):
        return function.attr
    return ""


def function_named(tree: ast.AST, name: str) -> ast.FunctionDef:
    matches = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    ]
    if len(matches) != 1:
        raise FaultProducerContractError(
            f"expected exactly one function named {name}"
        )
    return matches[0]


def keyword(call: ast.Call, name: str) -> ast.expr | None:
    return next(
        (item.value for item in call.keywords if item.arg == name),
        None,
    )


def literal_string(node: ast.AST | None) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def parameter_role(call: ast.Call) -> str | None:
    parameters = keyword(call, "parameters")
    if parameters is None:
        return None
    for mapping in ast.walk(parameters):
        if not isinstance(mapping, ast.Dict):
            continue
        for key, value in zip(mapping.keys, mapping.values):
            if literal_string(key) == "role":
                return literal_string(value)
    return None


def validate_actions(path: Path) -> None:
    source, tree = parse_python(path)
    factory = function_named(tree, "make_fault_producers")
    assignments = {
        target.id: statement.value
        for statement in factory.body
        if isinstance(statement, ast.Assign)
        and isinstance(statement.value, ast.Call)
        and final_call_name(statement.value) == "Node"
        for target in statement.targets
        if isinstance(target, ast.Name)
    }
    if set(assignments) != {"authority", "candidate"}:
        raise FaultProducerContractError(
            "factory must create exactly authority and candidate Node actions"
        )

    for role, action in assignments.items():
        if (
            literal_string(keyword(action, "package")) != "voice_nav_sim"
            or literal_string(keyword(action, "executable"))
            != "fault_producer_helper"
            or parameter_role(action) != role
        ):
            raise FaultProducerContractError(
                f"{role} action must use the closed helper role"
            )
        if keyword(action, "respawn") is not None:
            raise FaultProducerContractError(
                "fault-producer actions must never respawn"
            )
        if any(
            keyword(action, option) is not None
            for option in ("namespace", "remappings")
        ):
            raise FaultProducerContractError(
                "fault-producer identity must not use namespace or remapping"
            )

    if literal_string(keyword(assignments["candidate"], "name")) != (
        "collision_monitor"
    ):
        raise FaultProducerContractError(
            "candidate action FQN must remain /collision_monitor"
        )
    authority_name = ast.unparse(keyword(assignments["authority"], "name"))
    if "case_id" not in authority_name or "_authority" not in authority_name:
        raise FaultProducerContractError(
            "authority action name must be derived only from case_id"
        )

    dictionary_keys = {
        literal_string(key)
        for mapping in ast.walk(factory)
        if isinstance(mapping, ast.Dict)
        for key in mapping.keys
        if literal_string(key) is not None
    }
    if dictionary_keys != {"case_id", "use_sim_time", "role"}:
        raise FaultProducerContractError(
            "factory parameters must remain identity-only, without raw motion"
        )
    if "respawn" in source:
        raise FaultProducerContractError(
            "fault-producer action module must not expose respawn"
        )


def method_named(class_node: ast.ClassDef, name: str) -> ast.FunctionDef:
    matches = [
        node
        for node in class_node.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    ]
    if len(matches) != 1:
        raise FaultProducerContractError(
            f"FaultProducerNode must define exactly one {name} method"
        )
    return matches[0]


def calls_named(node: ast.AST, name: str) -> list[ast.Call]:
    return [
        call
        for call in ast.walk(node)
        if isinstance(call, ast.Call) and final_call_name(call) == name
    ]


def contains_attribute(node: ast.AST, name: str) -> bool:
    return any(
        isinstance(item, ast.Attribute) and item.attr == name
        for item in ast.walk(node)
    )


def validate_retry_classifier(tree: ast.Module) -> None:
    classifier = function_named(tree, "is_retryable_open_response")
    classifier_source = ast.unparse(classifier)
    required = {
        "REJECTED",
        "WRITER_METADATA_PENDING",
        "WRITER_UNAVAILABLE",
        "NO_WRITER_PENDING_DETAIL",
        "response.detail",
    }
    missing = sorted(
        marker for marker in required if marker not in classifier_source
    )
    detail_matches = [
        compare
        for compare in ast.walk(classifier)
        if isinstance(compare, ast.Compare)
        and ast.unparse(compare.left) == "response.detail"
        and len(compare.ops) == 1
        and isinstance(compare.ops[0], ast.Eq)
        and len(compare.comparators) == 1
        and ast.unparse(compare.comparators[0]) == "NO_WRITER_PENDING_DETAIL"
    ]
    if missing or len(detail_matches) != 1 or any(
        isinstance(node, ast.Constant) and node.value is True
        for node in ast.walk(classifier)
    ):
        raise FaultProducerContractError(
            "OPEN retry classification must remain typed and exact-detail only"
        )


def validate_helper(path: Path) -> None:
    source, tree = parse_python(path)
    classes = [
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "FaultProducerNode"
    ]
    if len(classes) != 1:
        raise FaultProducerContractError(
            "helper must define one closed FaultProducerNode"
        )
    producer = classes[0]
    initializer = method_named(producer, "__init__")
    authority = method_named(producer, "run_authority")
    candidate = method_named(producer, "run_candidate")
    dispatch = method_named(producer, "run")

    role_guard = [
        node
        for node in ast.walk(initializer)
        if isinstance(node, ast.If)
        and ast.unparse(node.test) == "self.role == 'authority'"
    ]
    all_clients = calls_named(producer, "create_client")
    guarded_clients = []
    candidate_else_clients = []
    if len(role_guard) == 1:
        guarded_clients = [
            call
            for statement in role_guard[0].body
            for call in calls_named(statement, "create_client")
        ]
        candidate_else_clients = [
            call
            for statement in role_guard[0].orelse
            for call in calls_named(statement, "create_client")
        ]
    if (
        len(all_clients) != 1
        or len(guarded_clients) != 1
        or id(all_clients[0]) != id(guarded_clients[0])
        or candidate_else_clients
    ):
        raise FaultProducerContractError(
            "only the authority role may own the Gate control client"
        )
    if "InternalMotionGateControl" not in ast.unparse(all_clients[0]):
        raise FaultProducerContractError(
            "authority client must use InternalMotionGateControl"
        )

    initializer_source = ast.unparse(initializer)
    if "('authority', 'candidate')" not in initializer_source:
        raise FaultProducerContractError(
            "helper roles must remain the closed authority/candidate pair"
        )
    if not all(
        marker in initializer_source
        for marker in (
            "self.get_fully_qualified_name() != '/collision_monitor'",
            "candidate FQN must be /collision_monitor",
        )
    ):
        raise FaultProducerContractError(
            "helper must enforce the exact /collision_monitor candidate FQN"
        )
    dispatch_source = ast.unparse(dispatch)
    if not all(
        marker in dispatch_source
        for marker in (
            "self.role == 'authority'",
            "self.run_authority(executor)",
            "self.run_candidate(executor)",
        )
    ):
        raise FaultProducerContractError(
            "role dispatch must remain closed inside the helper"
        )

    authority_source = ast.unparse(authority)
    if not all(
        marker in authority_source
        for marker in (
            "Request.PREPARE",
            "self.open_with_convergence",
            "Request.RENEW",
        )
    ) or "Request.INHIBIT" in source:
        raise FaultProducerContractError(
            "authority must own PREPARE/OPEN/RENEW and never INHIBIT"
        )
    candidate_source = ast.unparse(candidate)
    if not all(
        marker in candidate_source
        for marker in (
            "self.ready_publisher.publish",
            "self.bind_candidate_topic",
            "self.candidate_publisher.publish",
        )
    ) or any(
        marker in candidate_source
        for marker in (
            "call_control",
            "request_control",
            "open_with_convergence",
            "create_client",
        )
    ):
        raise FaultProducerContractError(
            "candidate may publish readiness and velocity but own no control"
        )

    ready_calls = calls_named(authority, "wait_for_candidate_ready")
    controller_calls = calls_named(
        authority,
        "wait_for_final_controller_reader",
    )
    prepare_calls = [
        call
        for call in calls_named(authority, "call_control")
        if contains_attribute(call, "PREPARE")
    ]
    if (
        len(ready_calls) != 1
        or len(controller_calls) != 1
        or len(prepare_calls) != 1
        or ready_calls[0].lineno >= prepare_calls[0].lineno
        or controller_calls[0].lineno >= prepare_calls[0].lineno
    ):
        raise FaultProducerContractError(
            "candidate and final controller readiness must precede PREPARE"
        )
    validate_retry_classifier(tree)


def validate_launch_test(path: Path) -> None:
    source, tree = parse_python(path)
    forbidden = (
        "InternalMotionGateControl",
        ".create_client(",
        ".call_async(",
        "Request.RENEW",
    )
    if any(marker in source for marker in forbidden):
        raise FaultProducerContractError(
            "parent launch test must observe only and never control or renew"
        )
    constants = {
        target.id: statement.value.value
        for statement in tree.body
        if isinstance(statement, ast.Assign)
        and isinstance(statement.value, ast.Constant)
        and isinstance(statement.value.value, (int, float))
        for target in statement.targets
        if isinstance(target, ast.Name)
    }
    if constants.get("SUSTAINED_LIVE_SECONDS", 0.0) < 0.5:
        raise FaultProducerContractError(
            "launch evidence must outlive authority and candidate leases"
        )
    test_method = function_named(
        tree,
        "test_independent_helpers_arm_gate_without_parent_control",
    )
    sustained_calls = calls_named(
        test_method,
        "require_sustained_renewal_and_candidate_traffic",
    )
    final_statement = test_method.body[-1] if test_method.body else None
    final_call = (
        final_statement.value
        if isinstance(final_statement, ast.Expr)
        and isinstance(final_statement.value, ast.Call)
        else None
    )
    if (
        len(sustained_calls) != 1
        or final_call is not sustained_calls[0]
        or any(isinstance(node, ast.Return) for node in ast.walk(test_method))
    ):
        raise FaultProducerContractError(
            "launch test must unconditionally finish with sustained evidence"
        )
    if "make_fault_producers('authority_arm')" not in source:
        raise FaultProducerContractError(
            "launch test must use the one exact fault-producer action factory"
        )
    controller_nodes = [
        call
        for call in calls_named(tree, "create_node")
        if call.args and literal_string(call.args[0]) == "diff_drive_controller"
    ]
    if len(controller_nodes) != 1:
        raise FaultProducerContractError(
            "test must provide the exact final controller health endpoint"
        )
    if "'expected_candidate_writer_fqn': '/collision_monitor'" not in source:
        raise FaultProducerContractError(
            "Gate fixture must expect the exact /collision_monitor FQN"
        )


def validate_protocol_test(path: Path) -> None:
    _source, tree = parse_python(path)
    classes = [
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "FaultProducerProtocolTest"
    ]
    if len(classes) != 1:
        raise FaultProducerContractError(
            "protocol test must expose one collected TestCase"
        )
    required_markers = {
        "test_pending_then_applied_retries_with_fresh_request_ids": (
            "open_with_convergence",
            "request_ids",
            "len(set(request_ids))",
        ),
        "test_only_exact_no_writer_and_metadata_pending_are_retryable": (
            "is_retryable_open_response",
            "WRITER_AMBIGUOUS",
            "final controller command endpoint is unavailable",
        ),
        "test_corrupted_pending_snapshot_fails_before_second_request": (
            "assertRaisesRegex",
            "open_with_convergence",
            "lease_id",
        ),
    }
    methods = {
        node.name: node
        for node in classes[0].body
        if isinstance(node, ast.FunctionDef) and node.name.startswith("test_")
    }
    if set(methods) != set(required_markers):
        raise FaultProducerContractError(
            "protocol test inventory must remain exact and collected"
        )
    for name, markers in required_markers.items():
        method = methods[name]
        method_source = ast.unparse(method)
        skipped = any(
            final_call_name(call) in {"skip", "skipTest"}
            for call in ast.walk(method)
            if isinstance(call, ast.Call)
        )
        if (
            method.decorator_list
            or any(isinstance(node, ast.Return) for node in ast.walk(method))
            or skipped
            or not all(marker in method_source for marker in markers)
        ):
            raise FaultProducerContractError(
                f"protocol evidence {name} must execute without skip/return"
            )


def validate_cmake(path: Path) -> None:
    source = read_text(path)
    launch_registration = re.search(
        r"add_launch_test\s*\(\s*test/test_fault_producer_pair\.py\s+"
        r"TIMEOUT\s+30\s+RUNNER\s+"
        r'"\$\{ament_cmake_ros_DIR\}/run_test_isolated\.py"\s*\)',
        source,
        flags=re.MULTILINE,
    )
    helper_install = re.search(
        r"install\s*\(\s*PROGRAMS\s+test/fault_producer\.py\s+"
        r"DESTINATION\s+lib/\$\{PROJECT_NAME\}\s+"
        r"RENAME\s+fault_producer_helper\s*\)",
        source,
        flags=re.MULTILINE,
    )
    if not launch_registration or not helper_install:
        raise FaultProducerContractError(
            "CMake must install the helper and register its isolated launch test"
        )
    if (
        "fault_producer_protocol_test" not in source
        or source.count("test_test_fault_producer_pair.py") != 2
        or "RUN_SERIAL TRUE" not in source
        or "ROS_DOMAIN_ID=unset:" not in source
    ):
        raise FaultProducerContractError(
            "CMake must lock protocol, serialization, and isolation metadata"
        )


def validate_package(path: Path) -> None:
    try:
        root = element_tree.fromstring(read_text(path))
    except element_tree.ParseError as error:
        raise FaultProducerContractError(
            f"cannot parse {path}: {error}"
        ) from error
    dependencies = {
        (dependency.text or "").strip()
        for tag in ("depend", "test_depend")
        for dependency in root.findall(tag)
    }
    required = {"rcl_interfaces", "std_msgs", "voice_nav_mission"}
    missing = sorted(required - dependencies)
    if missing:
        raise FaultProducerContractError(
            "voice_nav_sim is missing helper test dependencies: "
            + ", ".join(missing)
        )


def validate_contract(root: Path) -> None:
    paths = required_artifacts(root)
    validate_actions(paths["actions"])
    validate_helper(paths["helper"])
    validate_launch_test(paths["launch_test"])
    validate_protocol_test(paths["protocol_test"])
    validate_cmake(paths["cmake"])
    validate_package(paths["package"])


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        validate_contract(arguments.root.resolve())
    except FaultProducerContractError as error:
        print(f"Fault-producer contract failed: {error}", file=sys.stderr)
        return 1
    print("Fault-producer contract passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
