#!/usr/bin/env python3
"""Run repository contract tests and fail closed on every skip."""

from __future__ import annotations

import ast
from collections import Counter
from collections.abc import Collection, Iterator
from pathlib import Path
import sys
import unittest


REQUIRED_TEST_IDS: frozenset[str] = frozenset(
    {
        (
            "test_ci_contract.CiWorkflowContractTest."
            "test_rosdep_install_uses_supported_noninteractive_flag"
        ),
        (
            "test_ci_readiness_contract.CiReadinessContractTest."
            "test_launch_tests_use_official_process_scoped_domain_leases"
        ),
        (
            "test_control_contract.ControlContractTest."
            "test_repository_control_contract_passes"
        ),
        (
            "test_motion_gate_contract.MotionGateContractTest."
            "test_repository_motion_gate_contract_passes"
        ),
        (
            "test_repository_contract.RepositoryContractTest."
            "test_repository_course_catalog_passes"
        ),
        (
            "test_scoped_test_results.ScopedTestResultsTest."
            "test_report_requires_complete_critical_launch_inventory"
        ),
        (
            "test_sdf_contract.SdfContractTest.test_valid_model_passes"
        ),
        (
            "test_simulation_contract.SimulationContractTest."
            "test_repository_simulation_contract_passes"
        ),
        (
            "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
            "test_ack_does_not_hide_process_exit_barrier_failure"
        ),
        (
            "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
            "test_claimed_partition_is_process_unique_and_overrides_inherited"
        ),
        (
            "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
            "test_cleanup_steps_attempt_every_callback_before_raising"
        ),
        (
            "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
            "test_false_or_malformed_ack_fails_without_process_wait"
        ),
        (
            "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
            "test_missing_or_wrong_partition_fails_before_cli"
        ),
        (
            "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
            "test_never_started_thread_is_safe_to_cleanup"
        ),
        (
            "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
            "test_nonzero_cli_exit_fails_without_process_wait"
        ),
        (
            "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
            "test_positive_ack_is_followed_by_real_process_exit_barrier"
        ),
        (
            "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
            "test_server_already_exited_is_not_reported_as_clean_stop"
        ),
        (
            "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
            "test_started_thread_timeout_is_reported"
        ),
        (
            "test_gazebo_shutdown_support.GazeboShutdownSupportTest."
            "test_timeout_fails_without_process_wait"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownContractTest."
            "test_cleanup_phases_are_independent_and_destroy_is_exception_safe"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownContractTest."
            "test_each_launch_test_claims_a_runtime_unique_partition"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownContractTest."
            "test_every_gazebo_test_uses_failure_safe_cleanup_and_strict_exit"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownContractTest."
            "test_launch_has_default_on_unexpected_exit_and_test_seam"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownContractTest."
            "test_standard_library_support_exists_and_parses"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_ack_only_cleanup_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_active_class_collection_disable_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_active_class_skip_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_active_test_expected_failure_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_assert_exit_codes_import_rebinding_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_assert_exit_codes_rebinding_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_canonical_verify_must_fail_skipped_contracts"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_canonical_verify_must_run_checker"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_cleanup_phase_registration_order_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_cleanup_runtime_rebinding_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_early_return_from_fixture_destroy_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_failure_path_cleanup_registration_is_required"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_fixed_cmake_partition_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_fixed_sleep_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_forced_exit_allowlist_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_gazebo_shutdown_owner_rebinding_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_global_process_kill_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_module_level_skip_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_module_load_tests_hook_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_positive_ack_validation_is_required"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_post_shutdown_decorator_rebinding_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_product_exit_policy_must_default_on"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_repository_contract_passes"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_rpc_uses_the_checked_environment_snapshot"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_shell_execution_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_test_launch_must_disable_early_shutdown"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_test_support_install_is_required"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_unreachable_positive_ack_validation_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_unreachable_process_exit_barrier_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_unreachable_shutdown_assertion_is_rejected"
        ),
        (
            "test_gazebo_teardown_contract.GazeboTeardownMutationTest."
            "test_wrong_test_partition_is_rejected"
        ),
        (
            "test_repository_test_runner.RepositoryTestRunnerTest."
            "test_canonical_verify_uses_no_skip_runner"
        ),
        (
            "test_repository_test_runner.RepositoryTestRunnerTest."
            "test_discovers_tests_directory_without_package_marker"
        ),
        (
            "test_repository_test_runner.RepositoryTestRunnerTest."
            "test_empty_contract_suite_fails_the_run"
        ),
        (
            "test_repository_test_runner.RepositoryTestRunnerTest."
            "test_executed_count_must_equal_discovery_snapshot"
        ),
        (
            "test_repository_test_runner.RepositoryTestRunnerTest."
            "test_expected_failure_contract_fails_the_run"
        ),
        (
            "test_repository_test_runner.RepositoryTestRunnerTest."
            "test_green_unskipped_contract_suite_succeeds"
        ),
        (
            "test_repository_test_runner.RepositoryTestRunnerTest."
            "test_missing_required_test_id_fails_the_run"
        ),
        (
            "test_repository_test_runner.RepositoryTestRunnerTest."
            "test_required_manifest_contains_critical_contract_ids"
        ),
        (
            "test_repository_test_runner.RepositoryTestRunnerTest."
            "test_skipped_contract_test_fails_the_run"
        ),
    }
)


def _iter_tests(suite: unittest.TestSuite) -> Iterator[unittest.TestCase]:
    for test in suite:
        if isinstance(test, unittest.TestSuite):
            yield from _iter_tests(test)
        else:
            yield test


def _source_test_inventory(
    repository_root: Path,
) -> tuple[frozenset[str], tuple[str, ...]]:
    test_ids = []
    errors = []
    tests_directory = repository_root / "tests"
    for path in sorted(tests_directory.glob("test_*.py")):
        try:
            tree = ast.parse(
                path.read_text(encoding="utf-8"),
                filename=str(path),
            )
        except (OSError, UnicodeError, SyntaxError) as error:
            errors.append(f"cannot inventory {path.name}: {error}")
            continue
        module_bindings = []
        for statement in tree.body:
            if isinstance(
                statement,
                (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef),
            ):
                module_bindings.append(statement.name)
            elif isinstance(statement, (ast.Assign, ast.AnnAssign)):
                targets = (
                    statement.targets
                    if isinstance(statement, ast.Assign)
                    else (statement.target,)
                )
                module_bindings.extend(
                    target.id
                    for target in targets
                    if isinstance(target, ast.Name)
                )
            elif isinstance(statement, (ast.Import, ast.ImportFrom)):
                module_bindings.extend(
                    alias.asname or alias.name.split(".", 1)[0]
                    for alias in statement.names
                )
        if "load_tests" in module_bindings:
            errors.append(
                f"{path.name} must not bind a load_tests collection hook"
            )
        test_case_names = set()
        for statement in tree.body:
            if not isinstance(statement, ast.ClassDef):
                continue
            is_test_case = any(
                isinstance(base, ast.Attribute)
                and isinstance(base.value, ast.Name)
                and base.value.id == "unittest"
                and base.attr == "TestCase"
                for base in statement.bases
            )
            if not is_test_case:
                continue
            test_case_names.add(statement.name)
            for member in statement.body:
                if (
                    isinstance(member, (ast.FunctionDef, ast.AsyncFunctionDef))
                    and member.name.startswith("test_")
                ):
                    if member.decorator_list:
                        errors.append(
                            f"{path.name}:{statement.name}.{member.name} "
                            "must not use decorators"
                        )
                    test_ids.append(
                        f"{path.stem}.{statement.name}.{member.name}"
                    )

        for statement in tree.body:
            targets = ()
            if isinstance(statement, ast.Assign):
                targets = statement.targets
            elif isinstance(statement, (ast.AnnAssign, ast.AugAssign)):
                targets = (statement.target,)
            elif isinstance(statement, ast.Delete):
                targets = statement.targets
            for target in targets:
                if (
                    isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id in test_case_names
                    and target.attr.startswith("test_")
                ):
                    errors.append(
                        f"{path.name}:{target.value.id}.{target.attr} "
                        "must not be rebound after its definition"
                    )

    duplicates = sorted(
        test_id
        for test_id, count in Counter(test_ids).items()
        if count > 1
    )
    if duplicates:
        errors.append(
            "duplicate source test IDs: " + ", ".join(duplicates)
        )
    return frozenset(test_ids), tuple(errors)


def run_suite(
    suite: unittest.TestSuite,
    *,
    stream,
    required_test_ids: Collection[str] = (),
) -> int:
    """Run one complete, unique contract inventory without exemptions."""
    tests = tuple(_iter_tests(suite))
    discovered_ids = tuple(test.id() for test in tests)
    id_counts = Counter(discovered_ids)
    duplicate_ids = sorted(
        test_id for test_id, count in id_counts.items() if count > 1
    )
    missing_ids = sorted(set(required_test_ids) - set(discovered_ids))
    source_ids = getattr(suite, "_voice_nav_source_test_ids", None)
    inventory_errors = tuple(
        getattr(suite, "_voice_nav_inventory_errors", ())
    )
    missing_source_ids = (
        sorted(set(source_ids) - set(discovered_ids))
        if source_ids is not None
        else []
    )
    unexpected_source_ids = (
        sorted(set(discovered_ids) - set(source_ids))
        if source_ids is not None
        else []
    )

    preflight_failed = False
    if not tests:
        stream.write("Repository contract discovery found zero tests.\n")
        preflight_failed = True
    if duplicate_ids:
        stream.write("Duplicate repository contract IDs were discovered:\n")
        for test_id in duplicate_ids:
            stream.write(f"- {test_id}\n")
        preflight_failed = True
    if missing_ids:
        stream.write("Missing required repository contracts:\n")
        for test_id in missing_ids:
            stream.write(f"- {test_id}\n")
        preflight_failed = True
    if inventory_errors:
        stream.write("Repository contract source inventory failed:\n")
        for error in inventory_errors:
            stream.write(f"- {error}\n")
        preflight_failed = True
    if missing_source_ids or unexpected_source_ids:
        stream.write(
            "Repository contract discovery does not match source inventory:\n"
        )
        for test_id in missing_source_ids:
            stream.write(f"- missing: {test_id}\n")
        for test_id in unexpected_source_ids:
            stream.write(f"- unexpected: {test_id}\n")
        preflight_failed = True
    if preflight_failed:
        return 1

    result = unittest.TextTestRunner(
        stream=stream,
        verbosity=2,
    ).run(unittest.TestSuite(tests))
    contract_failed = False
    if result.testsRun != len(tests):
        stream.write(
            "\nRepository contract executed test count does not match "
            f"discovery snapshot: discovered={len(tests)}, "
            f"executed={result.testsRun}.\n"
        )
        contract_failed = True
    if result.skipped:
        stream.write(
            "\nRepository contract forbids skipped tests:\n"
        )
        for test, reason in result.skipped:
            stream.write(f"- {test.id()}: {reason}\n")
        contract_failed = True
    if result.expectedFailures:
        stream.write("\nRepository contract forbids expected failures:\n")
        for test, _traceback in result.expectedFailures:
            stream.write(f"- {test.id()}\n")
        contract_failed = True
    if result.unexpectedSuccesses:
        stream.write("\nRepository contract forbids unexpected successes:\n")
        for test in result.unexpectedSuccesses:
            stream.write(f"- {test.id()}\n")
        contract_failed = True
    if not result.wasSuccessful():
        contract_failed = True
    return 1 if contract_failed else 0


def discover_suite(repository_root: Path) -> unittest.TestSuite:
    """Discover contract tests from the repository's non-package tests tree."""
    repository_path = str(repository_root.resolve())
    tests_path = str((repository_root / "tests").resolve())
    if repository_path not in sys.path:
        sys.path.insert(0, repository_path)

    tests_path_was_present = tests_path in sys.path
    try:
        suite = unittest.TestLoader().discover(
            tests_path,
            pattern="test_*.py",
        )
        source_ids, inventory_errors = _source_test_inventory(
            repository_root
        )
        suite._voice_nav_source_test_ids = source_ids
        suite._voice_nav_inventory_errors = inventory_errors
        return suite
    finally:
        if not tests_path_was_present:
            while tests_path in sys.path:
                sys.path.remove(tests_path)


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    suite = discover_suite(repository_root)
    return run_suite(
        suite,
        stream=sys.stderr,
        required_test_ids=REQUIRED_TEST_IDS,
    )


if __name__ == "__main__":
    raise SystemExit(main())
