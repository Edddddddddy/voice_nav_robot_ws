import ast
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
STARTUP_TIMEOUT_NAME = (
    "CONTROLLER_STARTUP_SERVICE_RESPONSE_TIMEOUT_SECONDS"
)


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


if __name__ == "__main__":
    unittest.main()
