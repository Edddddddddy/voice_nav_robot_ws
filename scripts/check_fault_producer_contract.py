#!/usr/bin/env python3
"""Validate the closed authority/candidate crash-fixture topology."""

from __future__ import annotations

import argparse
import ast
import hashlib
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
    "authority_death_test": (
        "src/voice_nav_sim/test/test_authority_process_death.py"
    ),
    "cmake": "src/voice_nav_sim/CMakeLists.txt",
    "package": "src/voice_nav_sim/package.xml",
}

AUTHORITY_TRACER_AST_SHA256 = (
    "797084a05b25800e576b478c94b28190aeda6ddc73164c1dd17ad2743baa595a"
)


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
    actions_property = function_named(tree, "actions")
    action_statements = actions_property.body
    if (
        action_statements
        and isinstance(action_statements[0], ast.Expr)
        and isinstance(action_statements[0].value, ast.Constant)
        and isinstance(action_statements[0].value.value, str)
    ):
        action_statements = action_statements[1:]
    if (
        [ast.unparse(item) for item in actions_property.decorator_list]
        != ["property"]
        or [argument.arg for argument in actions_property.args.args]
        != ["self"]
        or len(action_statements) != 1
        or not isinstance(action_statements[0], ast.Return)
        or ast.unparse(action_statements[0].value)
        != "(self.authority, self.candidate)"
    ):
        raise FaultProducerContractError(
            "fault-producer actions property must remain the exact pair"
        )
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


def validate_authority_death_test(path: Path) -> None:
    """Require SIGKILL to target the one exact authority launch action."""
    source, tree = parse_python(path)
    forbidden = (
        "InternalMotionGateControl",
        "create_client(",
        "create_publisher(",
        "os.kill(",
        "subprocess",
        "SignalProcess",
        "matches_action",
        ".emit_event(",
    )
    if any(marker in source for marker in forbidden):
        raise FaultProducerContractError(
            "observer-only authority tracer must not control or inject output"
        )
    publisher_factories = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute)
        and node.attr in {"create_publisher", "create_generic_publisher"}
    ]
    publish_calls = calls_named(tree, "publish")
    if publisher_factories or publish_calls:
        raise FaultProducerContractError(
            "observer-only authority tracer must not control or inject output"
        )
    reflection_names = {
        "getattr",
        "getattr_static",
        "setattr",
        "delattr",
        "vars",
        "globals",
        "locals",
        "eval",
        "exec",
        "compile",
        "__import__",
        "import_module",
        "attrgetter",
        "methodcaller",
        "__getattribute__",
        "__setattr__",
    }
    reflection_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and final_call_name(node) in reflection_names
    ]
    reflection_attributes = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute)
        and node.attr
        in {"__dict__", "__getattribute__", "__setattr__"}
    ]
    if reflection_calls or reflection_attributes:
        raise FaultProducerContractError(
            "authority observer must not use dynamic reflection"
        )
    called_names = {
        final_call_name(node)
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and final_call_name(node)
    } - {"exit_observation", "request_sigkill", "wait_for_exact_exit"}
    callable_identity_writes = [
        node
        for node in ast.walk(tree)
        if (
            isinstance(node, ast.Name)
            and isinstance(node.ctx, (ast.Store, ast.Del))
            and node.id in called_names
        )
        or (
            isinstance(node, ast.Attribute)
            and isinstance(node.ctx, (ast.Store, ast.Del))
            and node.attr in called_names
        )
    ]
    if callable_identity_writes:
        raise FaultProducerContractError(
            "authority tracer must preserve assertion and predicate callable "
            "identity"
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
    required_constants = {
        "ARMING_MAX_AGE_NS": 20_000_000,
        "ARMING_WINDOW_NS": 40_000_000,
        "AUTHORITY_STOP_DEADLINE_NS": 300_000_000,
        "ZERO_HOLD_SECONDS": 0.12,
        "ZERO_HOLD_MIN_SPAN_NS": 100_000_000,
        "MINIMUM_ZERO_HOLD_SAMPLES": 5,
    }
    if any(
        constants.get(name) != value
        for name, value in required_constants.items()
    ):
        raise FaultProducerContractError(
            "authority timing constants must remain pinned"
        )
    for callback_name, collection_name in (
        ("on_state", "states"),
        ("on_output", "outputs"),
        ("on_candidate", "candidate_messages"),
    ):
        callback = function_named(tree, callback_name)
        receipt_assignments = [
            statement
            for statement in callback.body
            if isinstance(statement, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "receipt_ns"
                for target in statement.targets
            )
            and ast.unparse(statement.value) == "time.monotonic_ns()"
        ]
        locked_sections = [
            statement
            for statement in callback.body
            if isinstance(statement, ast.With)
            and len(statement.items) == 1
            and ast.unparse(statement.items[0].context_expr) == "self.lock"
        ]
        append_calls = (
            calls_named(locked_sections[0], "append")
            if len(locked_sections) == 1
            else []
        )
        if (
            len(receipt_assignments) != 1
            or len(locked_sections) != 1
            or receipt_assignments[0].lineno >= locked_sections[0].lineno
            or len(calls_named(locked_sections[0], "monotonic_ns")) != 0
            or len(append_calls) != 1
            or ast.unparse(append_calls[0].func)
            != f"self.{collection_name}.append"
            or [ast.unparse(argument) for argument in append_calls[0].args]
            != ["(receipt_ns, message)"]
        ):
            raise FaultProducerContractError(
                "authority observer must preserve callback-entry receipt "
                "fences before its lock"
            )
    generate = function_named(tree, "generate_test_description")
    class_inventory = {
        node.name: [ast.unparse(base) for base in node.bases]
        for node in tree.body
        if isinstance(node, ast.ClassDef)
    }
    adapter_module_bindings = [
        statement
        for statement in tree.body
        if isinstance(statement, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == "launch_crash_adapter"
            for target in statement.targets
        )
    ]
    adapter_constructors = [
        statement
        for statement in generate.body
        if isinstance(statement, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "crash_adapter"
            for target in statement.targets
        )
    ]
    ledger_constructors = [
        statement
        for statement in generate.body
        if isinstance(statement, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "ledger"
            for target in statement.targets
        )
    ]
    adapter_type_writes = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute)
        and isinstance(node.ctx, ast.Store)
        and node.attr == "LaunchCrashAdapter"
    ]
    if (
        class_inventory
        != {
            "AuthorityProcessDeathTest": ["unittest.TestCase"],
            "AuthorityProcessDeathShutdownTest": ["unittest.TestCase"],
        }
        or len(adapter_module_bindings) != 1
        or ast.unparse(adapter_module_bindings[0].value)
        != (
            "load_test_support('launch_crash_adapter.py', "
            "'voice_nav_authority_launch_crash_adapter')"
        )
        or len(adapter_constructors) != 1
        or ast.unparse(adapter_constructors[0].value)
        != "launch_crash_adapter.LaunchCrashAdapter(ledger)"
        or len(ledger_constructors) != 1
        or ast.unparse(ledger_constructors[0].value)
        != "crash_evidence.CrashLedger()"
        or adapter_type_writes
    ):
        raise FaultProducerContractError(
            "authority tracer must preserve exact LaunchCrashAdapter "
            "construction"
        )
    expected_clean = {
        tuple(ast.unparse(argument) for argument in call.args)
        for call in calls_named(generate, "expect_clean")
    }
    expected_kill = [
        tuple(ast.unparse(argument) for argument in call.args)
        for call in calls_named(generate, "expect_sigkill")
    ]
    launch_descriptions = calls_named(generate, "LaunchDescription")
    launch_elements = []
    if (
        len(launch_descriptions) == 1
        and launch_descriptions[0].args
        and isinstance(launch_descriptions[0].args[0], ast.List)
    ):
        launch_elements = [
            ast.unparse(element)
            for element in launch_descriptions[0].args[0].elts
        ]
    generate_returns = [
        statement
        for statement in generate.body
        if isinstance(statement, ast.Return)
    ]
    context_mapping = {}
    if (
        len(generate_returns) == 1
        and isinstance(generate_returns[0].value, ast.Tuple)
        and len(generate_returns[0].value.elts) == 2
        and isinstance(generate_returns[0].value.elts[1], ast.Dict)
    ):
        context = generate_returns[0].value.elts[1]
        context_mapping = {
            literal_string(key): ast.unparse(value)
            for key, value in zip(context.keys, context.values)
        }
    if context_mapping != {
        "crash_adapter": "crash_adapter",
        "motion_gate": "motion_gate",
        "authority": "producers.authority",
        "candidate": "producers.candidate",
    }:
        raise FaultProducerContractError(
            "authority tracer must preserve its exact launch context"
        )
    registration_assignments = [
        statement
        for statement in generate.body
        if isinstance(statement, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == "exit_registrations"
            for target in statement.targets
        )
    ]
    registration_elements = []
    if (
        len(registration_assignments) == 1
        and isinstance(registration_assignments[0].value, ast.Tuple)
    ):
        registration_elements = [
            ast.unparse(element)
            for element in registration_assignments[0].value.elts
        ]
    if (
        expected_clean != {
            ("motion_gate", "'motion_gate'"),
            ("producers.candidate", "'candidate'"),
        }
        or expected_kill
        != [("producers.authority", "'authority'")]
        or registration_elements
        != [
            "crash_adapter.expect_clean(motion_gate, 'motion_gate')",
            (
                "crash_adapter.expect_sigkill(producers.authority, "
                "'authority')"
            ),
            (
                "crash_adapter.expect_clean(producers.candidate, "
                "'candidate')"
            ),
        ]
        or launch_elements
        != [
            "*exit_registrations",
            "motion_gate",
            "*producers.actions",
            "launch_testing.actions.ReadyToTest()",
        ]
        or "make_fault_producers('authority_death')"
        not in ast.unparse(generate)
    ):
        raise FaultProducerContractError(
            "authority tracer must preserve exact launch exit registrations"
        )
    test_method = function_named(
        tree,
        "test_exact_authority_sigkill_expires_gate_to_zero",
    )
    skipped = any(
        final_call_name(call) in {"skip", "skipTest"}
        for call in ast.walk(test_method)
        if isinstance(call, ast.Call)
    )
    if (
        test_method.decorator_list
        or any(isinstance(node, ast.Return) for node in ast.walk(test_method))
        or skipped
    ):
        raise FaultProducerContractError(
            "authority tracer must execute without skip or early return"
        )
    exact_exit = function_named(tree, "wait_for_exact_exit")
    exit_observations = [
        call
        for call in calls_named(exact_exit, "exit_observation")
        if ast.unparse(call.func) == "crash_adapter.exit_observation"
    ]
    exit_returns = [
        node for node in ast.walk(exact_exit) if isinstance(node, ast.Return)
    ]
    exit_handlers = [
        node
        for node in ast.walk(exact_exit)
        if isinstance(node, ast.ExceptHandler)
    ]
    if (
        len(exit_observations) != 1
        or [ast.unparse(argument) for argument in exit_observations[0].args]
        != ["authority"]
        or len(exit_returns) != 1
        or ast.unparse(exit_returns[0].value)
        != "crash_adapter.exit_observation(authority)"
        or len(exit_handlers) != 1
        or ast.unparse(exit_handlers[0].type)
        != "crash_evidence.CrashEvidenceError"
    ):
        raise FaultProducerContractError(
            "authority tracer must use the exact ProcessExited observation "
            "source"
        )
    protected_adapter_names = {
        "crash_adapter",
        "exit_observation",
        "request_sigkill",
        "wait_for_exact_exit",
    }
    allowed_adapter_calls = {
        "crash_adapter.request_sigkill",
        "self.wait_for_exact_exit",
    }
    adapter_rebound = False
    for method in (test_method, exact_exit):
        for node in ast.walk(method):
            targets = []
            value = None
            if isinstance(node, ast.Assign):
                targets = node.targets
                value = node.value
            elif isinstance(node, (ast.AnnAssign, ast.AugAssign)):
                targets = [node.target]
                value = node.value
            elif isinstance(node, ast.NamedExpr):
                targets = [node.target]
                value = node.value
            if targets and any(
                (
                    isinstance(item, ast.Name)
                    and item.id in protected_adapter_names
                )
                or (
                    isinstance(item, ast.Attribute)
                    and item.attr in protected_adapter_names
                )
                or isinstance(item, ast.Name)
                and item.id == "crash_adapter"
                for target in targets
                for item in ast.walk(target)
            ):
                adapter_rebound = True
            if value is not None and any(
                isinstance(item, ast.Name) and item.id == "crash_adapter"
                for item in ast.walk(value)
            ):
                direct_call = (
                    isinstance(value, ast.Call)
                    and ast.unparse(value.func) in allowed_adapter_calls
                )
                if not direct_call:
                    adapter_rebound = True
            if isinstance(node, ast.Call) and final_call_name(node) in {
                "setattr",
                "__setattr__",
            }:
                adapter_rebound = True
    if adapter_rebound:
        raise FaultProducerContractError(
            "authority tracer must reject adapter rebinding and aliases"
        )
    kill_calls = calls_named(test_method, "request_sigkill")
    if (
        len(kill_calls) != 1
        or [ast.unparse(argument) for argument in kill_calls[0].args]
        != ["launch_service", "authority"]
    ):
        raise FaultProducerContractError(
            "authority tracer must request exact authority SIGKILL"
        )
    deadline_assertions = [
        call
        for call in calls_named(test_method, "assertLessEqual")
        if len(call.args) == 2
        and ast.unparse(call.args[1]) == "AUTHORITY_STOP_DEADLINE_NS"
    ]
    measured_latencies = {
        ast.unparse(call.args[0])
        for call in deadline_assertions
    }
    if measured_latencies != {
        "terminal_receipt - exit_ns",
        "zero_receipt - exit_ns",
    }:
        raise FaultProducerContractError(
            "authority tracer must measure exact ProcessExited latency"
        )
    arming_assertions = [
        call
        for call in calls_named(test_method, "assertLessEqual")
        if len(call.args) == 2
        and ast.unparse(call.args[1])
        in {"ARMING_WINDOW_NS", "ARMING_MAX_AGE_NS"}
    ]
    arming_pairs = [
        tuple(ast.unparse(argument) for argument in call.args)
        for call in arming_assertions
    ]
    if len(arming_pairs) != 3 or set(arming_pairs) != {
        ("signal_intent_ns - barrier_started", "ARMING_WINDOW_NS"),
        ("signal_intent_ns - state_receipt", "ARMING_MAX_AGE_NS"),
        ("signal_intent_ns - output_receipt", "ARMING_MAX_AGE_NS"),
    }:
        raise FaultProducerContractError(
            "authority tracer must preserve exact fresh arming barrier "
            "assertions"
        )
    ordered_names = (
        "wait_for_fresh_arming_barrier",
        "request_sigkill",
        "wait_for_exact_exit",
        "assert_no_preexit_retirement",
        "wait_for_terminal_evidence",
        "assert_terminal_state",
        "assert_zero_hold_and_candidate_counter_evidence",
    )
    ordered_calls = [
        calls_named(test_method, name)
        for name in ordered_names
    ]
    if (
        any(len(matches) != 1 for matches in ordered_calls)
        or [matches[0].lineno for matches in ordered_calls]
        != sorted(matches[0].lineno for matches in ordered_calls)
    ):
        raise FaultProducerContractError(
            "authority tracer must preserve its fresh arming barrier order"
        )
    fresh_barrier = function_named(tree, "wait_for_fresh_arming_barrier")
    fresh_source = ast.unparse(fresh_barrier)
    if not all(
        marker in fresh_source
        for marker in (
            "sample[1].control_seq > baseline_state.control_seq",
            "sample[1].gate_instance_id == baseline_state.gate_instance_id",
            "sample[1].lease_id == baseline_state.lease_id",
            "now_ns - renewed[0] <= ARMING_MAX_AGE_NS",
            "now_ns - nonzero[0] <= ARMING_MAX_AGE_NS",
            "self.is_finite_nonzero(sample[1])",
        )
    ):
        raise FaultProducerContractError(
            "authority tracer must require fresh RENEW arming evidence"
        )
    preexit = function_named(tree, "assert_no_preexit_retirement")
    preexit_source = ast.unparse(preexit)
    if (
        preexit_source.count(
            "barrier_started_ns <= receipt < exit_ns"
        ) != 2
        or not all(
            marker in preexit_source
            for marker in (
                "self.is_live_armed(state)",
                "state.gate_instance_id == armed_state.gate_instance_id",
                "state.lease_id == armed_state.lease_id",
                "all((self.is_finite_nonzero(output)",
            )
        )
    ):
        raise FaultProducerContractError(
            "authority tracer must preserve pre-exit live counter-evidence"
        )

    terminal_wait = function_named(tree, "wait_for_terminal_evidence")
    terminal_assertion = function_named(tree, "assert_terminal_state")
    terminal_source = ast.unparse(terminal_wait)
    assertion_source = ast.unparse(terminal_assertion)
    new_zero_assertions = [
        call
        for call in calls_named(terminal_assertion, "assertGreater")
        if [ast.unparse(argument) for argument in call.args]
        == ["terminal.zero_publish_seq", "armed.zero_publish_seq"]
    ]
    if len(new_zero_assertions) != 1:
        raise FaultProducerContractError(
            "authority terminal evidence must require a new zero publish "
            "sequence"
        )
    terminal_loops = [
        node for node in ast.walk(terminal_wait) if isinstance(node, ast.While)
    ]
    terminal_snapshots = calls_named(terminal_wait, "snapshot")
    deadline_clocks = [
        call
        for call in calls_named(terminal_wait, "monotonic_ns")
        if ast.unparse(call.func) == "time.monotonic_ns"
    ]
    if (
        terminal_source.count("sample[0] >= exit_ns") != 2
        or terminal_source.count("sample[0] <= deadline_ns") != 2
        or len(terminal_loops) != 1
        or not isinstance(terminal_loops[0].test, ast.Constant)
        or terminal_loops[0].test.value is not True
        or len(terminal_snapshots) != 1
        or len(deadline_clocks) != 1
        or terminal_snapshots[0].lineno >= deadline_clocks[0].lineno
        or "if time.monotonic_ns() > deadline_ns:" not in terminal_source
    ):
        raise FaultProducerContractError(
            "authority terminal evidence must use a scan-first receipt "
            "deadline"
        )
    if (
        "InternalMotionGateState.CANDIDATE_EXPIRED" in source
        or not all(
            marker in terminal_source
            for marker in (
                "deadline_ns = exit_ns + AUTHORITY_STOP_DEADLINE_NS",
                "sample[0] >= exit_ns",
                "InternalMotionGateState.AUTHORITY_EXPIRED",
                "self.is_zero(sample[1])",
            )
        )
        or not all(
            marker in assertion_source
            for marker in (
                "InternalMotionGateState.AUTHORITY_EXPIRED",
                "'authority lease expired'",
                "terminal.lease_id, ''",
                "terminal.candidate_topic, ''",
                "terminal.motion_inhibited",
                "terminal.zero_selected",
            )
        )
    ):
        raise FaultProducerContractError(
            "authority tracer must require AUTHORITY_EXPIRED terminal evidence"
        )
    counter_evidence = function_named(
        tree,
        "assert_zero_hold_and_candidate_counter_evidence",
    )
    candidate_binding = function_named(tree, "bind_candidate_observer")
    binding_source = ast.unparse(candidate_binding)
    endpoint_filter = (
        "endpoint.node_name == 'collision_monitor' and "
        "endpoint.node_namespace == '/' and "
        "(endpoint.topic_type == 'geometry_msgs/msg/TwistStamped')"
    )
    compatible_assignments = {
        target.id: statement.value
        for function in (candidate_binding, counter_evidence)
        for statement in ast.walk(function)
        if isinstance(statement, ast.Assign)
        and isinstance(statement.value, ast.ListComp)
        for target in statement.targets
        if isinstance(target, ast.Name)
        and target.id in {"compatible", "compatible_writers"}
    }
    endpoint_collection_names = {"compatible", "compatible_writers"}
    endpoint_collection_nodes = tuple(
        node
        for function in (candidate_binding, counter_evidence)
        for node in ast.walk(function)
    )
    endpoint_store_counts = {
        name: sum(
            1
            for node in endpoint_collection_nodes
            if isinstance(node, ast.Name)
            and isinstance(node.ctx, ast.Store)
            and node.id == name
        )
        for name in endpoint_collection_names
    }
    endpoint_aliases = []
    endpoint_indirect_writes = []
    for node in endpoint_collection_nodes:
        targets = []
        value = None
        if isinstance(node, ast.Assign):
            targets = node.targets
            value = node.value
        elif isinstance(node, (ast.AnnAssign, ast.AugAssign)):
            targets = [node.target]
            value = node.value
        elif isinstance(node, ast.NamedExpr):
            targets = [node.target]
            value = node.value
        elif isinstance(node, ast.Delete):
            targets = node.targets
        if value is not None and any(
            isinstance(item, ast.Name)
            and item.id in endpoint_collection_names
            for item in ast.walk(value)
        ):
            endpoint_aliases.append(node)
        for target in targets:
            referenced = {
                item.id
                for item in ast.walk(target)
                if isinstance(item, ast.Name)
                and item.id in endpoint_collection_names
            }
            if referenced and not (
                isinstance(target, ast.Name)
                and target.id in endpoint_collection_names
            ):
                endpoint_indirect_writes.append(node)
    endpoint_mutation_calls = [
        node
        for node in endpoint_collection_nodes
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and any(
            isinstance(item, ast.Name)
            and item.id in endpoint_collection_names
            for item in ast.walk(node.func.value)
        )
    ]
    endpoint_deletes = [
        node
        for node in endpoint_collection_nodes
        if isinstance(node, ast.Name)
        and isinstance(node.ctx, ast.Del)
        and node.id in endpoint_collection_names
    ]
    if (
        endpoint_store_counts
        != {"compatible": 1, "compatible_writers": 1}
        or endpoint_aliases
        or endpoint_indirect_writes
        or endpoint_mutation_calls
        or endpoint_deletes
    ):
        raise FaultProducerContractError(
            "authority tracer must preserve immutable endpoint evidence "
            "collections"
        )
    compatible_shapes = {}
    for name, comprehension in compatible_assignments.items():
        if (
            len(comprehension.generators) == 1
            and len(comprehension.generators[0].ifs) == 1
        ):
            compatible_shapes[name] = ast.unparse(
                comprehension.generators[0].ifs[0]
            )
    if not all(
        marker in binding_source
        for marker in (
            "self.node.create_subscription(TwistStamped, topic, "
            "self.on_candidate, candidate_qos())",
            "expected_gid = bytes(writer_gid)",
            "endpoint.node_name == 'collision_monitor'",
            "endpoint.node_namespace == '/'",
            "endpoint.topic_type == 'geometry_msgs/msg/TwistStamped'",
            "len(compatible) == 1",
            "bytes(compatible[0].endpoint_gid) == expected_gid",
            "self.candidate_snapshot()",
        )
    ) or compatible_shapes != {
        "compatible": endpoint_filter,
        "compatible_writers": endpoint_filter,
    }:
        raise FaultProducerContractError(
            "authority tracer must preserve surviving candidate "
            "counter-evidence and exact writer binding"
        )
    counter_source = ast.unparse(counter_evidence)
    receipt_span_assertions = [
        call
        for call in calls_named(counter_evidence, "assertGreaterEqual")
        if len(call.args) == 2
        and ast.unparse(call.args[1]) == "ZERO_HOLD_MIN_SPAN_NS"
    ]
    receipt_span_pairs = [
        tuple(ast.unparse(argument) for argument in call.args)
        for call in receipt_span_assertions
    ]
    if len(receipt_span_pairs) != 2 or set(receipt_span_pairs) != {
        ("held[-1][0] - held[0][0]", "ZERO_HOLD_MIN_SPAN_NS"),
        (
            "candidate_after_exit[-1][0] - candidate_after_exit[0][0]",
            "ZERO_HOLD_MIN_SPAN_NS",
        ),
    }:
        raise FaultProducerContractError(
            "authority tracer must preserve exact receipt-span assertions"
        )
    counter_equal_pairs = [
        tuple(ast.unparse(argument) for argument in call.args[:2])
        for call in calls_named(counter_evidence, "assertEqual")
        if len(call.args) >= 2
    ]
    if (
        counter_equal_pairs.count(("len(compatible_writers)", "1")) != 1
        or counter_equal_pairs.count(
            (
                "bytes(compatible_writers[0].endpoint_gid)",
                "expected_gid",
            )
        )
        != 1
    ):
        raise FaultProducerContractError(
            "authority tracer must require one compatible writer with the "
            "armed GID"
        )
    if not all(
        marker in counter_source
        for marker in (
            "all((self.is_zero(output) for _receipt, output in held))",
            "held[-1][0] - held[0][0]",
            "ZERO_HOLD_MIN_SPAN_NS",
            "get_publishers_info_by_topic(candidate_topic)",
            "endpoint.node_name == 'collision_monitor'",
            "endpoint.node_namespace == '/'",
            "geometry_msgs/msg/TwistStamped",
            "self.assertEqual(len(compatible_writers), 1",
            "bytes(compatible_writers[0].endpoint_gid)",
            "self.candidate_snapshot()",
            "receipt >= exit_ns",
            "candidate_after_exit[-1][0] - candidate_after_exit[0][0]",
            "self.is_finite_nonzero(message)",
        )
    ):
        raise FaultProducerContractError(
            "authority tracer must preserve surviving candidate counter-evidence"
        )

    shutdown_method = function_named(
        tree,
        "test_exact_exit_ledger_is_complete",
    )
    exit_calls = calls_named(shutdown_method, "assertExitCodes")
    exact_allowlists = {}
    for call in exit_calls:
        process_value = keyword(call, "process")
        allowlist_value = keyword(call, "allowable_exit_codes")
        if isinstance(process_value, ast.Name):
            exact_allowlists[process_value.id] = ast.unparse(allowlist_value)
    shutdown_source = ast.unparse(shutdown_method)
    if (
        len(exit_calls) != 3
        or exact_allowlists != {
            "motion_gate": "[0]",
            "authority": "[-signal.SIGKILL]",
            "candidate": "[0]",
        }
        or shutdown_source.count("crash_adapter.assert_complete()") != 1
        or not all(
            marker in shutdown_source
            for marker in (
                "('motion_gate', 0)",
                "('authority', -signal.SIGKILL)",
                "('candidate', 0)",
            )
        )
    ):
        raise FaultProducerContractError(
            "authority tracer must preserve the exact exhaustive exit ledger"
        )
    authority_ast_digest = hashlib.sha256(
        ast.dump(tree, include_attributes=False).encode("utf-8")
    ).hexdigest()
    if authority_ast_digest != AUTHORITY_TRACER_AST_SHA256:
        raise FaultProducerContractError(
            "review-locked authority tracer AST fingerprint changed"
        )


def validate_cmake(path: Path) -> None:
    source = read_text(path)
    authority_registration = re.search(
        r"add_launch_test\s*\(\s*test/test_authority_process_death\.py\s+"
        r"TIMEOUT\s+30\s+RUNNER\s+"
        r'"\$\{ament_cmake_ros_DIR\}/run_test_isolated\.py"\s*\)',
        source,
        flags=re.MULTILINE,
    )
    if (
        not authority_registration
        or source.count("test_test_authority_process_death.py") != 2
    ):
        raise FaultProducerContractError(
            "CMake must register the isolated authority process-death launch test"
        )
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
    validate_authority_death_test(paths["authority_death_test"])
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
