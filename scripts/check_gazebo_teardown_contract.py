#!/usr/bin/env python3
"""Validate deterministic, isolated Gazebo launch-test teardown."""

from __future__ import annotations

import argparse
import ast
import re
import sys
from pathlib import Path


class GazeboTeardownContractError(ValueError):
    """A source artifact weakens the Gazebo teardown contract."""


PATHS = {
    "support": "src/voice_nav_sim/test_support/gazebo_shutdown.py",
    "simulation_launch": "src/voice_nav_sim/launch/simulation.launch.py",
    "product_launch": "src/voice_nav_bringup/launch/product_sim.launch.py",
    "simulation_control": (
        "src/voice_nav_sim/test/test_simulation_control.py"
    ),
    "simulation_interfaces": (
        "src/voice_nav_sim/test/test_simulation_interfaces.py"
    ),
    "product_test": (
        "src/voice_nav_bringup/test/test_motion_gate_product.py"
    ),
    "simulation_cmake": "src/voice_nav_sim/CMakeLists.txt",
    "bringup_cmake": "src/voice_nav_bringup/CMakeLists.txt",
    "verify": "scripts/verify.sh",
}

TEST_POLICIES = {
    "simulation_control": {
        "class": "SimulationControlTest",
        "shutdown_class": "SimulationControlShutdownTest",
        "partition_name": "SIMULATION_TEST_PARTITION",
        "partition_scope": "l0008_sim_control",
        "active_test": (
            "test_stamped_drive_odometry_tf_and_consumer_timeout"
        ),
        "pre_cleanup": "publish_zero_for_cleanup",
        "pre_action": "publish_for",
        "pre_action_args": (0.0, 0.0, 0.15),
        "destroy_attributes": {
            ("executor", "shutdown"),
            ("gazebo_shutdown", "join_started_thread"),
            ("node", "destroy_node"),
            ("rclpy", "shutdown"),
        },
    },
    "simulation_interfaces": {
        "class": "SimulationInterfacesTest",
        "shutdown_class": "SimulationInterfacesShutdownTest",
        "partition_name": "SIMULATION_TEST_PARTITION",
        "partition_scope": "l0008_sim_interfaces",
        "active_test": "test_perception_odom_tf_and_ownership_contract",
        "pre_cleanup": "publish_zero_for_cleanup",
        "pre_action": "publish_command_for",
        "pre_action_args": (0.0, 0.0, 0.25),
        "destroy_attributes": {
            ("executor", "shutdown"),
            ("gazebo_shutdown", "join_started_thread"),
            ("node", "destroy_node"),
            ("rclpy", "shutdown"),
            ("tf_listener", "unregister"),
        },
    },
    "product_test": {
        "class": "MotionGateProductTest",
        "shutdown_class": "MotionGateProductShutdownTest",
        "partition_name": "PRODUCT_TEST_PARTITION",
        "partition_scope": "l0009_motion_gate_product",
        "active_test": "test_motion_gate_product_contract",
        "pre_cleanup": "inhibit_for_cleanup",
        "pre_action": "inhibit",
        "pre_action_args": (),
        "destroy_attributes": {
            ("executor", "shutdown"),
            ("gazebo_shutdown", "join_started_thread"),
            ("node", "destroy_node"),
            ("node", "destroy_publisher"),
            ("rclpy", "shutdown"),
        },
    },
}


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise GazeboTeardownContractError(
            f"cannot read {path}: {error}"
        ) from error


def parse_python(path: Path) -> tuple[str, ast.Module]:
    source = read_text(path)
    try:
        return source, ast.parse(source, filename=str(path))
    except SyntaxError as error:
        raise GazeboTeardownContractError(
            f"cannot parse {path}: {error}"
        ) from error


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


def function_named(parent: ast.AST, name: str) -> ast.FunctionDef:
    matches = [
        node
        for node in ast.iter_child_nodes(parent)
        if isinstance(node, ast.FunctionDef) and node.name == name
    ]
    if len(matches) != 1:
        raise GazeboTeardownContractError(
            f"expected exactly one function named {name}"
        )
    return matches[0]


def class_named(tree: ast.Module, name: str) -> ast.ClassDef:
    matches = [
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == name
    ]
    if len(matches) != 1:
        raise GazeboTeardownContractError(
            f"expected exactly one class named {name}"
        )
    return matches[0]


def bound_names(statement: ast.stmt) -> set[str]:
    """Return names a simple top-level statement binds at runtime."""
    if isinstance(
        statement,
        (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef),
    ):
        return {statement.name}
    if isinstance(statement, ast.Import):
        return {
            alias.asname or alias.name.split(".", 1)[0]
            for alias in statement.names
        }
    if isinstance(statement, ast.ImportFrom):
        return {alias.asname or alias.name for alias in statement.names}

    targets: list[ast.expr] = []
    if isinstance(statement, ast.Assign):
        targets.extend(statement.targets)
    elif isinstance(statement, ast.AnnAssign):
        targets.append(statement.target)
    elif isinstance(statement, ast.AugAssign):
        targets.append(statement.target)
    return {
        target.id for target in targets if isinstance(target, ast.Name)
    }


def method_calls(function: ast.FunctionDef, name: str) -> list[ast.Call]:
    return [
        node
        for node in ast.walk(function)
        if isinstance(node, ast.Call) and call_name(node) == name
    ]


def contains_generator_statement(function: ast.FunctionDef) -> bool:
    return any(
        isinstance(node, (ast.Yield, ast.YieldFrom))
        for node in ast.walk(function)
    )


def walk_direct_function_body(
    function: ast.FunctionDef,
):
    """Walk one function body without entering nested callables."""
    pending = list(reversed(function.body))
    while pending:
        node = pending.pop()
        yield node
        if isinstance(
            node,
            (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda),
        ):
            continue
        pending.extend(reversed(list(ast.iter_child_nodes(node))))


def exact_attribute_call(
    statement: ast.stmt,
    owner: str,
    name: str,
) -> ast.Call | None:
    if not (
        isinstance(statement, ast.Expr)
        and isinstance(statement.value, ast.Call)
        and isinstance(statement.value.func, ast.Attribute)
        and isinstance(statement.value.func.value, ast.Name)
        and statement.value.func.value.id == owner
        and statement.value.func.attr == name
    ):
        return None
    return statement.value


def has_plain_arguments(
    function: ast.FunctionDef,
    expected: list[str],
) -> bool:
    arguments = function.args
    return (
        not function.decorator_list
        and not arguments.posonlyargs
        and [argument.arg for argument in arguments.args] == expected
        and arguments.vararg is None
        and not arguments.kwonlyargs
        and arguments.kwarg is None
        and not arguments.defaults
        and not arguments.kw_defaults
    )


def has_exact_literal_arguments(
    call: ast.Call,
    expected: tuple[float, ...],
) -> bool:
    if len(call.args) != len(expected) or call.keywords:
        return False
    return all(
        isinstance(argument, ast.Constant)
        and type(argument.value) is type(value)
        and argument.value == value
        for argument, value in zip(call.args, expected)
    )


def is_exact_positive_ack_guard(statement: ast.stmt) -> bool:
    if not isinstance(statement, ast.If):
        return False
    condition = statement.test
    if not (
        isinstance(condition, ast.Compare)
        and len(condition.ops) == 1
        and isinstance(condition.ops[0], ast.Is)
        and len(condition.comparators) == 1
        and isinstance(condition.comparators[0], ast.Constant)
        and condition.comparators[0].value is None
        and isinstance(condition.left, ast.Call)
        and isinstance(condition.left.func, ast.Attribute)
        and isinstance(condition.left.func.value, ast.Name)
        and condition.left.func.value.id == "POSITIVE_ACK"
        and condition.left.func.attr == "fullmatch"
        and len(condition.left.args) == 1
        and isinstance(condition.left.args[0], ast.Attribute)
        and isinstance(condition.left.args[0].value, ast.Name)
        and condition.left.args[0].value.id == "completed"
        and condition.left.args[0].attr == "stdout"
        and not condition.left.keywords
        and len(statement.body) == 1
        and isinstance(statement.body[0], ast.Raise)
        and isinstance(statement.body[0].exc, ast.Call)
        and isinstance(statement.body[0].exc.func, ast.Name)
        and statement.body[0].exc.func.id == "AssertionError"
        and not statement.orelse
    ):
        return False
    return True


def is_exact_process_exit_barrier(statement: ast.stmt) -> bool:
    call = exact_attribute_call(
        statement,
        "proc_info",
        "assertWaitForShutdown",
    )
    if not isinstance(call, ast.Call) or call.args:
        return False
    if [keyword.arg for keyword in call.keywords] != [
        "process",
        "timeout",
    ]:
        return False
    return (
        literal_string(call.keywords[0].value) == "gazebo"
        and isinstance(call.keywords[1].value, ast.Name)
        and call.keywords[1].value.id == "PROCESS_TIMEOUT_SECONDS"
    )


def validate_support(path: Path) -> None:
    source, tree = parse_python(path)
    forbidden = {
        "shell=True": "shell=True",
        "time.sleep(": "fixed sleep",
        "pkill": "global process kill",
        "killall": "global process kill",
        "os.kill(": "direct process kill",
        "SIGKILL": "SIGKILL",
        "allowable_exit_codes": "exit-code allowlist",
    }
    for token, diagnostic in forbidden.items():
        if token in source:
            raise GazeboTeardownContractError(
                f"Gazebo teardown helper must not use {diagnostic}"
            )

    required = (
        "/server_control",
        "gz.msgs.ServerControl",
        "gz.msgs.Boolean",
        "stop: true",
        "SERVICE_TIMEOUT_MILLISECONDS = 5000",
        "SUBPROCESS_TIMEOUT_SECONDS = 7.0",
        "PROCESS_TIMEOUT_SECONDS = 10.0",
        "secrets.token_hex(16)",
        "os.getpid()",
    )
    for token in required:
        if token not in source:
            raise GazeboTeardownContractError(
                f"Gazebo structured stop is missing {token}"
            )

    function = function_named(tree, "structured_stop_gazebo")
    if any(
        isinstance(node, (ast.Return, ast.Yield, ast.YieldFrom))
        for node in ast.walk(function)
    ):
        raise GazeboTeardownContractError(
            "process-exit barrier must be unconditional and reachable"
        )
    runner_calls = method_calls(function, "runner")
    ack_calls = method_calls(function, "fullmatch")
    ack_guards = [
        statement
        for statement in function.body
        if is_exact_positive_ack_guard(statement)
    ]
    startup_calls = method_calls(function, "assertWaitForStartup")
    shutdown_calls = method_calls(function, "assertWaitForShutdown")
    if len(runner_calls) != 1:
        raise GazeboTeardownContractError(
            "structured stop must invoke exactly one command runner"
        )
    runner = runner_calls[0]
    shell = keyword_value(runner, "shell")
    runner_environment = keyword_value(runner, "env")
    if not (
        isinstance(shell, ast.Constant) and shell.value is False
    ):
        raise GazeboTeardownContractError(
            "Gazebo structured stop runner must set shell=False"
        )
    if not (
        isinstance(runner_environment, ast.Name)
        and runner_environment.id == "active_environment"
    ):
        raise GazeboTeardownContractError(
            "Gazebo RPC must use the checked environment snapshot"
        )
    if len(startup_calls) != 1:
        raise GazeboTeardownContractError(
            "structured stop must prove the launch-managed process started"
        )
    if len(ack_calls) != 1:
        raise GazeboTeardownContractError(
            "structured stop must require one exact positive ACK"
        )
    if len(ack_guards) != 1:
        raise GazeboTeardownContractError(
            "positive ACK condition must be unconditional"
        )
    if len(shutdown_calls) != 1:
        raise GazeboTeardownContractError(
            "positive ACK must be followed by a real process-exit barrier"
        )
    if not (
        function.body
        and is_exact_process_exit_barrier(function.body[-1])
    ):
        raise GazeboTeardownContractError(
            "process-exit barrier must be unconditional and final"
        )
    if not (
        runner.lineno < ack_calls[0].lineno < shutdown_calls[0].lineno
    ):
        raise GazeboTeardownContractError(
            "runner, positive ACK, and process-exit barrier are out of order"
        )
    partition_guard = source.find(
        "actual_partition != expected_partition"
    )
    runner_position = source.find("completed = runner(")
    if partition_guard < 0 or not partition_guard < runner_position:
        raise GazeboTeardownContractError(
            "exact isolated partition guard must precede the command runner"
        )
    if "if not expected_partition" not in source:
        raise GazeboTeardownContractError(
            "expected isolated partition must be non-empty"
        )
    if "isinstance(proc_info['gazebo'], ProcessExited)" not in source:
        raise GazeboTeardownContractError(
            "an already-exited Gazebo must not count as a clean stop"
        )


def declaration_default(
    calls: list[ast.Call],
    argument_name: str,
) -> str | None:
    declarations = [
        call
        for call in calls
        if call_name(call) == "DeclareLaunchArgument"
        and call.args
        and literal_string(call.args[0]) == argument_name
    ]
    if len(declarations) != 1:
        return None
    return literal_string(keyword_value(declarations[0], "default_value"))


def validate_launches(simulation_path: Path, product_path: Path) -> None:
    simulation, simulation_tree = parse_python(simulation_path)
    product, product_tree = parse_python(product_path)
    simulation_calls = [
        node for node in ast.walk(simulation_tree)
        if isinstance(node, ast.Call)
    ]
    product_calls = [
        node for node in ast.walk(product_tree)
        if isinstance(node, ast.Call)
    ]
    for calls, label in (
        (simulation_calls, "simulation"),
        (product_calls, "product"),
    ):
        if declaration_default(calls, "shutdown_on_gazebo_exit") != "true":
            raise GazeboTeardownContractError(
                f"{label} shutdown_on_gazebo_exit must default to true"
            )
    if simulation.count(
        "condition=IfCondition(shutdown_on_gazebo_exit)"
    ) != 2:
        raise GazeboTeardownContractError(
            "both Gazebo exit handlers must use the test-disable condition"
        )
    if re.search(
        r"on_exit\s*=\s*Shutdown\(reason=['\"]Gazebo",
        simulation,
    ):
        raise GazeboTeardownContractError(
            "Gazebo ExecuteProcess exit must use the conditional handler"
        )
    if (
        "'shutdown_on_gazebo_exit': shutdown_on_gazebo_exit"
        not in product
    ):
        raise GazeboTeardownContractError(
            "product launch must pass shutdown_on_gazebo_exit to simulation"
        )


def validate_cleanup_registration(
    test_class: ast.ClassDef,
    policy: dict[str, object],
) -> None:
    setup = function_named(test_class, "setUp")
    if not has_plain_arguments(setup, ["self", "proc_info"]):
        raise GazeboTeardownContractError(
            f"{test_class.name}.setUp must receive proc_info"
        )
    registrations = setup.body[:3]
    if any(
        isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "self"
        and node.attr == "_cleanups"
        for node in ast.walk(setup)
    ):
        raise GazeboTeardownContractError(
            f"{test_class.name} must not disable or rebind "
            "critical teardown"
        )
    if (
        len(registrations) != 3
        or len(method_calls(setup, "addCleanup")) != 3
    ):
        raise GazeboTeardownContractError(
            f"{test_class.name}.setUp must register independent "
            "failure-path cleanups first"
        )

    calls: list[ast.Call] = []
    for statement in registrations:
        call = exact_attribute_call(statement, "self", "addCleanup")
        if not isinstance(call, ast.Call):
            raise GazeboTeardownContractError(
                f"{test_class.name}.setUp must register independent "
                "failure-path cleanups first"
            )
        calls.append(call)

    destroy, structured_stop, pre_cleanup = calls
    expected_partition = keyword_value(
        structured_stop,
        "expected_partition",
    )
    valid = (
        len(destroy.args) == 1
        and not destroy.keywords
        and isinstance(destroy.args[0], ast.Attribute)
        and isinstance(destroy.args[0].value, ast.Name)
        and destroy.args[0].value.id == "self"
        and destroy.args[0].attr == "destroy_ros_fixture"
        and len(structured_stop.args) == 2
        and isinstance(structured_stop.args[0], ast.Attribute)
        and isinstance(structured_stop.args[0].value, ast.Name)
        and structured_stop.args[0].value.id == "gazebo_shutdown"
        and structured_stop.args[0].attr == "structured_stop_gazebo"
        and isinstance(structured_stop.args[1], ast.Name)
        and structured_stop.args[1].id == "proc_info"
        and len(structured_stop.keywords) == 1
        and isinstance(expected_partition, ast.Name)
        and expected_partition.id == policy["partition_name"]
        and len(pre_cleanup.args) == 1
        and not pre_cleanup.keywords
        and isinstance(pre_cleanup.args[0], ast.Attribute)
        and isinstance(pre_cleanup.args[0].value, ast.Name)
        and pre_cleanup.args[0].value.id == "self"
        and pre_cleanup.args[0].attr == policy["pre_cleanup"]
    )
    if not valid:
        raise GazeboTeardownContractError(
            f"{test_class.name}.setUp must register independent "
            "failure-path cleanups first"
        )
    if any(
        bound_names(statement)
        & {"tearDown", "cleanup_fixture", "doCleanups"}
        for statement in test_class.body
    ):
        raise GazeboTeardownContractError(
            f"{test_class.name} must not bypass independent registered cleanup"
        )
    if any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "doCleanups"
        for node in ast.walk(test_class)
    ):
        raise GazeboTeardownContractError(
            f"{test_class.name} must not bypass independent registered cleanup"
        )


def validate_pre_cleanup(
    test_class: ast.ClassDef,
    policy: dict[str, object],
    path: Path,
) -> None:
    cleanup = function_named(test_class, str(policy["pre_cleanup"]))
    action = (
        exact_attribute_call(
            cleanup.body[-1],
            "self",
            str(policy["pre_action"]),
        )
        if cleanup.body
        else None
    )
    guard = cleanup.body[-2] if len(cleanup.body) >= 2 else None
    prefix = cleanup.body[:-2]
    guard_return = (
        guard.body[0]
        if isinstance(guard, ast.If)
        and len(guard.body) == 1
        and isinstance(guard.body[0], ast.Return)
        and guard.body[0].value is None
        and not guard.orelse
        else None
    )
    action_calls = [
        call
        for call in ast.walk(cleanup)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "self"
        and call.func.attr == policy["pre_action"]
    ]
    if not (
        has_plain_arguments(cleanup, ["self"])
        and not contains_generator_statement(cleanup)
        and not any(isinstance(node, ast.Try) for node in ast.walk(cleanup))
        and all(
            isinstance(statement, (ast.Assign, ast.AnnAssign))
            for statement in prefix
        )
        and isinstance(guard, ast.If)
        and not (
            isinstance(guard.test, ast.Constant)
            and bool(guard.test.value)
        )
        and isinstance(guard_return, ast.Return)
        and sum(
            isinstance(node, ast.Return)
            for node in ast.walk(cleanup)
        )
        == 1
        and isinstance(action, ast.Call)
        and len(action_calls) == 1
        and has_exact_literal_arguments(
            action,
            tuple(policy["pre_action_args"]),
        )
    ):
        raise GazeboTeardownContractError(
            f"{path.name} cleanup zero/inhibit must not swallow failures"
        )


def validate_active_test_class(
    test_class: ast.ClassDef,
    policy: dict[str, object],
    path: Path,
) -> None:
    expected_method_name = str(policy["active_test"])
    test_methods = [
        statement
        for statement in test_class.body
        if isinstance(statement, ast.FunctionDef)
        and statement.name.startswith("test_")
    ]
    method = (
        test_methods[0]
        if len(test_methods) == 1
        and test_methods[0].name == expected_method_name
        else None
    )
    class_valid = (
        not test_class.decorator_list
        and len(test_class.bases) == 1
        and isinstance(test_class.bases[0], ast.Attribute)
        and isinstance(test_class.bases[0].value, ast.Name)
        and test_class.bases[0].value.id == "unittest"
        and test_class.bases[0].attr == "TestCase"
        and not test_class.keywords
        and not any(
            isinstance(statement, (ast.Assign, ast.AnnAssign))
            and any(
                isinstance(target, ast.Name)
                and target.id in {
                    "__test__",
                    "__unittest_skip__",
                    "__unittest_expecting_failure__",
                }
                for target in (
                    statement.targets
                    if isinstance(statement, ast.Assign)
                    else [statement.target]
                )
            )
            for statement in test_class.body
        )
    )
    if not class_valid:
        raise GazeboTeardownContractError(
            f"{path.name} active launch test class is not "
            "collectable exactly once"
        )
    if not (
        isinstance(method, ast.FunctionDef)
        and not method.decorator_list
        and not contains_generator_statement(method)
        and not any(
            isinstance(node, ast.Return)
            for node in walk_direct_function_body(method)
        )
        and [argument.arg for argument in method.args.args][0:1] == ["self"]
    ):
        raise GazeboTeardownContractError(
            f"{path.name} active launch test method is not "
            "collectable exactly once"
        )


def validate_destroy_control_flow(
    destroy: ast.FunctionDef,
    policy: dict[str, object],
    path: Path,
) -> None:
    aggregate_calls = [
        call
        for call in ast.walk(destroy)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "gazebo_shutdown"
        and call.func.attr == "run_cleanup_steps"
    ]
    join_calls = [
        call
        for call in ast.walk(destroy)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "gazebo_shutdown"
        and call.func.attr == "join_started_thread"
    ]
    direct_join_calls = [
        call
        for call in ast.walk(destroy)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "join"
    ]
    final_call = (
        exact_attribute_call(
            destroy.body[-1],
            "gazebo_shutdown",
            "run_cleanup_steps",
        )
        if destroy.body
        else None
    )
    steps_bindings = [
        node
        for node in ast.walk(destroy)
        if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign))
        and "steps" in bound_names(node)
    ]
    illegal_steps_targets = []
    for node in ast.walk(destroy):
        targets: list[ast.expr] = []
        if isinstance(node, ast.Assign):
            targets.extend(node.targets)
        elif isinstance(node, (ast.AnnAssign, ast.AugAssign)):
            targets.append(node.target)
        elif isinstance(node, ast.Delete):
            targets.extend(node.targets)
        for target in targets:
            if (
                isinstance(target, ast.Subscript)
                and isinstance(target.value, ast.Name)
                and target.value.id == "steps"
            ):
                illegal_steps_targets.append(target)
    steps_initialization = (
        steps_bindings[0]
        if len(steps_bindings) == 1
        and isinstance(steps_bindings[0], ast.Assign)
        and isinstance(steps_bindings[0].value, ast.List)
        and not steps_bindings[0].value.elts
        else None
    )
    steps_method_calls = [
        call
        for call in ast.walk(destroy)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "steps"
    ]
    append_calls = [
        call for call in steps_method_calls if call.func.attr == "append"
    ]
    local_functions = {
        statement.name: statement
        for statement in destroy.body
        if isinstance(statement, ast.FunctionDef)
    }
    append_evidence_nodes: list[ast.AST] = [
        argument
        for call in append_calls
        for argument in call.args
    ]
    referenced_callbacks = {
        node.id
        for evidence in append_evidence_nodes
        for node in ast.walk(evidence)
        if isinstance(node, ast.Name)
        and isinstance(node.ctx, ast.Load)
        and node.id in local_functions
    }
    append_evidence_nodes.extend(
        local_functions[name] for name in referenced_callbacks
    )
    appended_attributes = {
        (node.value.id, node.attr)
        for evidence in append_evidence_nodes
        for node in ast.walk(evidence)
        if isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
    }
    required_attributes = set(policy["destroy_attributes"])
    if not (
        has_plain_arguments(destroy, ["self"])
        and destroy.body
        and not contains_generator_statement(destroy)
        and not any(
            isinstance(node, (ast.Return, ast.Raise))
            for node in ast.walk(destroy)
        )
        and len(aggregate_calls) == 1
        and isinstance(final_call, ast.Call)
        and len(final_call.args) == 2
        and isinstance(final_call.args[0], ast.Constant)
        and isinstance(final_call.args[0].value, str)
        and isinstance(final_call.args[1], ast.Name)
        and final_call.args[1].id == "steps"
        and not final_call.keywords
        and len(join_calls) == 1
        and not direct_join_calls
        and isinstance(steps_initialization, ast.Assign)
        and not illegal_steps_targets
        and append_calls
        and steps_method_calls == append_calls
        and all(
            len(call.args) == 1 and not call.keywords
            for call in append_calls
        )
        and not any(
            isinstance(node, ast.If)
            and isinstance(node.test, ast.Constant)
            and not bool(node.test.value)
            for node in ast.walk(destroy)
        )
        and required_attributes <= appended_attributes
    ):
        raise GazeboTeardownContractError(
            f"{path.name} destroy_ros_fixture must not terminate early "
            "and must exhaust all cleanup steps"
        )


def validate_shutdown_assertion(
    tree: ast.Module,
    shutdown_class_name: str,
) -> None:
    shutdown_class = class_named(tree, shutdown_class_name)
    if not tree.body or tree.body[-1] is not shutdown_class:
        raise GazeboTeardownContractError(
            f"{shutdown_class_name} module must not disable or rebind "
            "critical teardown"
        )
    if not (
        len(shutdown_class.decorator_list) == 1
        and isinstance(shutdown_class.decorator_list[0], ast.Call)
        and isinstance(
            shutdown_class.decorator_list[0].func,
            ast.Attribute,
        )
        and isinstance(
            shutdown_class.decorator_list[0].func.value,
            ast.Name,
        )
        and shutdown_class.decorator_list[0].func.value.id
        == "launch_testing"
        and shutdown_class.decorator_list[0].func.attr
        == "post_shutdown_test"
        and not shutdown_class.decorator_list[0].args
        and not shutdown_class.decorator_list[0].keywords
        and len(shutdown_class.bases) == 1
        and isinstance(shutdown_class.bases[0], ast.Attribute)
        and isinstance(shutdown_class.bases[0].value, ast.Name)
        and shutdown_class.bases[0].value.id == "unittest"
        and shutdown_class.bases[0].attr == "TestCase"
        and not shutdown_class.keywords
    ):
        raise GazeboTeardownContractError(
            f"{shutdown_class_name} must be only a post-shutdown test"
        )

    method = function_named(
        shutdown_class,
        "test_all_launch_managed_processes_exit_cleanly",
    )
    arguments = method.args
    statement = method.body[0] if len(method.body) == 1 else None
    call = (
        statement.value
        if isinstance(statement, ast.Expr)
        and isinstance(statement.value, ast.Call)
        else None
    )
    if not (
        len(shutdown_class.body) == 1
        and shutdown_class.body[0] is method
        and not method.decorator_list
        and not arguments.posonlyargs
        and [argument.arg for argument in arguments.args]
        == ["self", "proc_info"]
        and arguments.vararg is None
        and not arguments.kwonlyargs
        and arguments.kwarg is None
        and not arguments.defaults
        and not arguments.kw_defaults
        and isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "assertExitCodes"
        and len(call.args) == 1
        and isinstance(call.args[0], ast.Name)
        and call.args[0].id == "proc_info"
        and not call.keywords
    ):
        raise GazeboTeardownContractError(
            f"{shutdown_class_name} must use a single unconditional "
            "top-level assertion to strictly check every process exit"
        )


def validate_no_module_disable_or_rebind(
    tree: ast.Module,
    path: Path,
    policy: dict[str, object],
) -> None:
    if any(
        isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef))
        and statement.name == "load_tests"
        for statement in tree.body
    ):
        raise GazeboTeardownContractError(
            f"{path.name} must not control critical test collection"
        )
    exact_imports = [
        statement
        for statement in tree.body
        if isinstance(statement, ast.ImportFrom)
        and statement.module == "launch_testing.asserts"
        and [
            (alias.name, alias.asname)
            for alias in statement.names
        ] == [("assertExitCodes", None)]
    ]
    if len(exact_imports) != 1:
        raise GazeboTeardownContractError(
            f"{path.name} must not disable or rebind critical teardown"
        )

    assert_exit_bindings = []
    for statement in tree.body:
        if isinstance(statement, (ast.Import, ast.ImportFrom)):
            for alias in statement.names:
                bound_name = alias.asname or alias.name.split(".")[0]
                if bound_name == "assertExitCodes":
                    assert_exit_bindings.append(statement)
    if assert_exit_bindings != exact_imports:
        raise GazeboTeardownContractError(
            f"{path.name} must not disable or rebind critical teardown"
        )

    gazebo_bindings = [
        statement
        for statement in tree.body
        if isinstance(statement, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == "gazebo_shutdown"
            for target in statement.targets
        )
    ]
    if not (
        len(gazebo_bindings) == 1
        and isinstance(gazebo_bindings[0].value, ast.Call)
        and isinstance(gazebo_bindings[0].value.func, ast.Name)
        and gazebo_bindings[0].value.func.id
        == "load_gazebo_shutdown_support"
        and not gazebo_bindings[0].value.args
        and not gazebo_bindings[0].value.keywords
    ):
        raise GazeboTeardownContractError(
            f"{path.name} must not disable or rebind critical teardown"
        )
    if [
        statement
        for statement in tree.body
        if "gazebo_shutdown" in bound_names(statement)
    ] != gazebo_bindings:
        raise GazeboTeardownContractError(
            f"{path.name} must not disable or rebind critical teardown"
        )

    launch_testing_bindings = [
        statement
        for statement in tree.body
        if "launch_testing" in bound_names(statement)
    ]
    allowed_launch_imports = {
        "launch_testing",
        "launch_testing.actions",
        "launch_testing.markers",
    }
    if not (
        launch_testing_bindings
        and all(
            isinstance(statement, ast.Import)
            and all(
                alias.asname is None
                and alias.name in allowed_launch_imports
                for alias in statement.names
                if (alias.asname or alias.name.split(".", 1)[0])
                == "launch_testing"
            )
            for statement in launch_testing_bindings
        )
    ):
        raise GazeboTeardownContractError(
            f"{path.name} must not disable or rebind critical teardown"
        )

    for protected_name in (
        str(policy["class"]),
        str(policy["shutdown_class"]),
        str(policy["partition_name"]),
    ):
        bindings = [
            statement
            for statement in tree.body
            if protected_name in bound_names(statement)
        ]
        if len(bindings) != 1:
            raise GazeboTeardownContractError(
                f"{path.name} must not disable or rebind critical teardown"
            )

    forbidden_names = {
        "pytestmark",
        "load_tests",
        "__test__",
        "__unittest_skip__",
        "__unittest_expecting_failure__",
        str(policy["active_test"]),
        "assertExitCodes",
    }
    forbidden_attributes = {
        "__unittest_skip__",
        "assertExitCodes",
        "cleanup_fixture",
        "doCleanups",
        "destroy_ros_fixture",
        "structured_stop_gazebo",
        "post_shutdown_test",
        "__test__",
        "__unittest_expecting_failure__",
    }
    for statement in tree.body:
        if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef)) and (
            statement.name in forbidden_names
        ):
            raise GazeboTeardownContractError(
                f"{path.name} must not disable or rebind critical teardown"
            )
        targets: list[ast.expr] = []
        if isinstance(statement, ast.Assign):
            targets.extend(statement.targets)
        elif isinstance(statement, ast.AnnAssign):
            targets.append(statement.target)
        elif isinstance(statement, ast.AugAssign):
            targets.append(statement.target)
        if any(
            (
                isinstance(target, ast.Name)
                and target.id in forbidden_names
            )
            or (
                isinstance(target, ast.Attribute)
                and target.attr in forbidden_attributes
            )
            for target in targets
        ):
            raise GazeboTeardownContractError(
                f"{path.name} must not disable or rebind critical teardown"
            )

    for node in ast.walk(tree):
        targets: list[ast.expr] = []
        if isinstance(node, ast.Assign):
            targets.extend(node.targets)
        elif isinstance(node, ast.AnnAssign):
            targets.append(node.target)
        elif isinstance(node, ast.AugAssign):
            targets.append(node.target)
        if any(
            isinstance(target, ast.Attribute)
            and isinstance(target.value, ast.Name)
            and target.value.id == policy["class"]
            and target.attr == policy["active_test"]
            for target in targets
        ):
            raise GazeboTeardownContractError(
                f"{path.name} must not disable or rebind critical teardown"
            )


def validate_test(path: Path, policy: dict[str, object]) -> None:
    source, tree = parse_python(path)
    validate_no_module_disable_or_rebind(tree, path, policy)
    if "'shutdown_on_gazebo_exit': 'false'" not in source:
        raise GazeboTeardownContractError(
            f"{path.name} must enable the structured test teardown seam"
        )
    if (
        "'test_support'" not in source
        or "'gazebo_shutdown.py'" not in source
    ):
        raise GazeboTeardownContractError(
            f"{path.name} must load installed Gazebo shutdown support"
        )
    assignments = {
        target.id: node.value
        for node in tree.body
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name)
    }
    partition_assignment = assignments.get(policy["partition_name"])
    if not (
        isinstance(partition_assignment, ast.Call)
        and isinstance(partition_assignment.func, ast.Attribute)
        and isinstance(partition_assignment.func.value, ast.Name)
        and partition_assignment.func.value.id == "gazebo_shutdown"
        and partition_assignment.func.attr
        == "claim_unique_test_partition"
        and len(partition_assignment.args) == 1
        and literal_string(partition_assignment.args[0])
        == policy["partition_scope"]
        and not partition_assignment.keywords
    ):
        raise GazeboTeardownContractError(
            f"{path.name} must claim exact runtime-unique partition scope "
            f"{policy['partition_scope']}"
        )
    test_class = class_named(tree, policy["class"])
    validate_active_test_class(test_class, policy, path)
    validate_cleanup_registration(test_class, policy)
    validate_pre_cleanup(test_class, policy, path)
    destroy = function_named(test_class, "destroy_ros_fixture")
    validate_destroy_control_flow(destroy, policy, path)
    validate_shutdown_assertion(tree, policy["shutdown_class"])
    if re.search(r"allowable_exit_codes\s*=\s*\[[^]]*(?:-9|137)", source):
        raise GazeboTeardownContractError(
            f"{path.name} must not allow forced Gazebo exit codes"
        )


def validate_cmake(simulation_path: Path, bringup_path: Path) -> None:
    simulation = read_text(simulation_path)
    bringup = read_text(bringup_path)
    if not re.search(
        r"install\s*\(\s*DIRECTORY(?:(?!DESTINATION).)*\btest_support\b",
        simulation,
        flags=re.DOTALL,
    ):
        raise GazeboTeardownContractError(
            "voice_nav_sim must install test_support"
        )
    for cmake in (simulation, bringup):
        if "GZ_PARTITION=" in cmake:
            raise GazeboTeardownContractError(
                "CMake must not reuse a fixed Gazebo test partition"
            )
        if "RUN_SERIAL TRUE" not in cmake:
            raise GazeboTeardownContractError(
                "Gazebo launch tests must remain RUN_SERIAL TRUE"
            )


def validate_contract(root: Path) -> None:
    root = root.resolve()
    paths = {name: root / relative for name, relative in PATHS.items()}
    validate_support(paths["support"])
    validate_launches(paths["simulation_launch"], paths["product_launch"])
    for name, policy in TEST_POLICIES.items():
        validate_test(paths[name], policy)
    validate_cmake(paths["simulation_cmake"], paths["bringup_cmake"])
    verify = read_text(paths["verify"])
    if "python3 scripts/check_gazebo_teardown_contract.py --root ." not in verify:
        raise GazeboTeardownContractError(
            "canonical verification must run the Gazebo teardown checker"
        )
    if (
        "python3 scripts/run_repository_tests.py" not in verify
        or "python3 -m unittest discover" in verify
    ):
        raise GazeboTeardownContractError(
            "canonical verification must fail skipped repository tests"
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        validate_contract(arguments.root)
    except GazeboTeardownContractError as error:
        print(f"Gazebo teardown contract failed: {error}", file=sys.stderr)
        return 1
    print("Gazebo teardown contract passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
