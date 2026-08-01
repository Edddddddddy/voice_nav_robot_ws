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
        "pre_stop": "publish_for",
        "pre_stop_args": (0.0, 0.0, 0.15),
    },
    "simulation_interfaces": {
        "class": "SimulationInterfacesTest",
        "shutdown_class": "SimulationInterfacesShutdownTest",
        "partition_name": "SIMULATION_TEST_PARTITION",
        "partition_scope": "l0008_sim_interfaces",
        "pre_stop": "publish_command_for",
        "pre_stop_args": (0.0, 0.0, 0.25),
    },
    "product_test": {
        "class": "MotionGateProductTest",
        "shutdown_class": "MotionGateProductShutdownTest",
        "partition_name": "PRODUCT_TEST_PARTITION",
        "partition_scope": "l0009_motion_gate_product",
        "pre_stop": "best_effort_inhibit",
        "pre_stop_args": (),
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


def method_calls(function: ast.FunctionDef, name: str) -> list[ast.Call]:
    return [
        node
        for node in ast.walk(function)
        if isinstance(node, ast.Call) and call_name(node) == name
    ]


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
) -> None:
    setup = function_named(test_class, "setUp")
    argument_names = [argument.arg for argument in setup.args.args]
    if argument_names != ["self", "proc_info"]:
        raise GazeboTeardownContractError(
            f"{test_class.name}.setUp must receive proc_info"
        )
    if not setup.body:
        raise GazeboTeardownContractError(
            f"{test_class.name}.setUp must register cleanup first"
        )
    first = setup.body[0]
    if not (
        isinstance(first, ast.Expr)
        and isinstance(first.value, ast.Call)
        and call_name(first.value) == "addCleanup"
        and len(first.value.args) == 2
        and isinstance(first.value.args[0], ast.Attribute)
        and first.value.args[0].attr == "cleanup_fixture"
        and isinstance(first.value.args[1], ast.Name)
        and first.value.args[1].id == "proc_info"
    ):
        raise GazeboTeardownContractError(
            f"{test_class.name}.setUp must register failure-path cleanup first"
        )
    if any(
        isinstance(node, ast.FunctionDef) and node.name == "tearDown"
        for node in test_class.body
    ):
        raise GazeboTeardownContractError(
            f"{test_class.name} must not bypass registered cleanup in tearDown"
        )


def validate_cleanup_control_flow(
    cleanup: ast.FunctionDef,
    policy: dict[str, object],
    path: Path,
) -> None:
    outer = cleanup.body[0] if len(cleanup.body) == 1 else None
    pre_stop_try = (
        outer.body[0]
        if isinstance(outer, ast.Try) and len(outer.body) == 1
        else None
    )
    stop_try = (
        outer.finalbody[0]
        if isinstance(outer, ast.Try) and len(outer.finalbody) == 1
        else None
    )
    pre_stop_call = (
        exact_attribute_call(
            pre_stop_try.body[0],
            "self",
            str(policy["pre_stop"]),
        )
        if isinstance(pre_stop_try, ast.Try)
        and len(pre_stop_try.body) == 1
        else None
    )
    handler = (
        pre_stop_try.handlers[0]
        if isinstance(pre_stop_try, ast.Try)
        and len(pre_stop_try.handlers) == 1
        else None
    )
    stop_call = (
        exact_attribute_call(
            stop_try.body[0],
            "gazebo_shutdown",
            "structured_stop_gazebo",
        )
        if isinstance(stop_try, ast.Try) and len(stop_try.body) == 1
        else None
    )
    destroy_call = (
        exact_attribute_call(
            stop_try.finalbody[0],
            "self",
            "destroy_ros_fixture",
        )
        if isinstance(stop_try, ast.Try)
        and len(stop_try.finalbody) == 1
        else None
    )
    expected_pre_stop_args = tuple(policy["pre_stop_args"])
    expected_partition = (
        keyword_value(stop_call, "expected_partition")
        if isinstance(stop_call, ast.Call)
        else None
    )
    if not (
        has_plain_arguments(cleanup, ["self", "proc_info"])
        and isinstance(outer, ast.Try)
        and not outer.handlers
        and not outer.orelse
        and isinstance(pre_stop_try, ast.Try)
        and not pre_stop_try.orelse
        and not pre_stop_try.finalbody
        and isinstance(pre_stop_call, ast.Call)
        and has_exact_literal_arguments(
            pre_stop_call,
            expected_pre_stop_args,
        )
        and isinstance(handler, ast.ExceptHandler)
        and isinstance(handler.type, ast.Name)
        and handler.type.id == "Exception"
        and handler.name is None
        and len(handler.body) == 1
        and isinstance(handler.body[0], ast.Pass)
        and isinstance(stop_try, ast.Try)
        and not stop_try.handlers
        and not stop_try.orelse
        and isinstance(stop_call, ast.Call)
        and len(stop_call.args) == 1
        and isinstance(stop_call.args[0], ast.Name)
        and stop_call.args[0].id == "proc_info"
        and len(stop_call.keywords) == 1
        and isinstance(expected_partition, ast.Name)
        and expected_partition.id == policy["partition_name"]
        and isinstance(destroy_call, ast.Call)
        and not destroy_call.args
        and not destroy_call.keywords
    ):
        raise GazeboTeardownContractError(
            f"{path.name} must use unconditional cleanup control flow: "
            "best-effort zero/inhibit, structured stop, then destroy"
        )


def validate_destroy_control_flow(
    destroy: ast.FunctionDef,
    path: Path,
) -> None:
    if not (
        has_plain_arguments(destroy, ["self"])
        and destroy.body
        and not any(
            isinstance(node, (ast.Return, ast.Raise))
            for node in ast.walk(destroy)
        )
    ):
        raise GazeboTeardownContractError(
            f"{path.name} destroy_ros_fixture must not terminate early"
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
        and call_name(shutdown_class.decorator_list[0])
        == "post_shutdown_test"
        and not shutdown_class.decorator_list[0].args
        and not shutdown_class.decorator_list[0].keywords
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
) -> None:
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

    forbidden_names = {
        "pytestmark",
        "__unittest_skip__",
        "assertExitCodes",
    }
    forbidden_attributes = {
        "__unittest_skip__",
        "assertExitCodes",
        "cleanup_fixture",
        "destroy_ros_fixture",
        "structured_stop_gazebo",
    }
    for statement in tree.body:
        if (
            isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef))
            and statement.name == "assertExitCodes"
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


def validate_test(path: Path, policy: dict[str, object]) -> None:
    source, tree = parse_python(path)
    validate_no_module_disable_or_rebind(tree, path)
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
    validate_cleanup_registration(test_class)
    cleanup = function_named(test_class, "cleanup_fixture")
    validate_cleanup_control_flow(cleanup, policy, path)
    destroy = function_named(test_class, "destroy_ros_fixture")
    validate_destroy_control_flow(destroy, path)
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
