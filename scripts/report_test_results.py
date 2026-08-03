#!/usr/bin/env python3

"""Report or clear test results for an explicit set of colcon packages."""

import argparse
import inspect
import sys
import tempfile
from collections import Counter
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree

from colcon_test_result.test_result import get_test_result_extensions
from colcon_test_result.test_result import Result

from colcon_evidence import ctest_result_path
from colcon_evidence import open_result_deletion_plan
from colcon_evidence import open_result_snapshot
from colcon_evidence import PackageSnapshotIdentity
from colcon_evidence import ResultFileIdentity
from colcon_evidence import selected_package_directories


CRITICAL_LAUNCH_CASES: dict[
    str,
    dict[Path, frozenset[tuple[str, str]]],
] = {
    "voice_nav_mission": {
        Path(
            "test_results/voice_nav_mission/"
            "test_test_motion_gate_node.py.xunit.xml"
        ): frozenset(
            {
                (
                    "voice_nav_mission.MotionGateNodeTest",
                    (
                        "test_journal_parameters_are_declared_read_only_"
                        "and_default_off"
                    ),
                ),
                (
                    "voice_nav_mission.MotionGateNodeTest",
                    "test_steady_fail_closed_protocol_without_clock",
                ),
                (
                    "voice_nav_mission.MotionGateNodeShutdownTest",
                    "test_motion_gate_exits_cleanly",
                ),
            }
        ),
        Path(
            "test_results/voice_nav_mission/"
            "test_test_motion_gate_node_journal.py.xunit.xml"
        ): frozenset(
            {
                (
                    "voice_nav_mission.MotionGateNodeJournalTest",
                    (
                        "test_partial_configuration_exits_without_"
                        "writer_claim"
                    ),
                ),
                (
                    "voice_nav_mission.MotionGateNodeJournalTest",
                    (
                        "test_full_configuration_journals_zero_output_"
                        "and_survives_exit"
                    ),
                ),
                (
                    (
                        "voice_nav_mission."
                        "MotionGateNodeJournalShutdownTest"
                    ),
                    "test_exit_codes_match_configuration_contract",
                ),
            }
        ),
    },
    "voice_nav_bringup": {
        Path(
            "test_results/voice_nav_bringup/"
            "test_test_motion_gate_product.py.xunit.xml"
        ): frozenset(
            {
                (
                    "voice_nav_bringup.MotionGateProductTest",
                    "test_motion_gate_product_contract",
                ),
                (
                    "voice_nav_bringup.MotionGateProductShutdownTest",
                    "test_all_launch_managed_processes_exit_cleanly",
                ),
            }
        ),
    },
    "voice_nav_sim": {
        Path(
            "test_results/voice_nav_sim/"
            "test_test_fault_producer_pair.py.xunit.xml"
        ): frozenset(
            {
                (
                    "voice_nav_sim.FaultProducerPairTest",
                    (
                        "test_independent_helpers_arm_gate_without_"
                        "parent_control"
                    ),
                ),
                (
                    "voice_nav_sim.FaultProducerPairShutdownTest",
                    "test_all_fixture_processes_exit_cleanly",
                ),
            }
        ),
        Path(
            "test_results/voice_nav_sim/"
            "test_test_simulation_control.py.xunit.xml"
        ): frozenset(
            {
                (
                    "voice_nav_sim.LaunchStartupPolicyTest",
                    "test_startup_handler_stops_after_failed_stage",
                ),
                (
                    "voice_nav_sim.SimulationControlTest",
                    "test_stamped_drive_odometry_tf_and_consumer_timeout",
                ),
                (
                    "voice_nav_sim.SimulationControlShutdownTest",
                    "test_all_launch_managed_processes_exit_cleanly",
                ),
            }
        ),
        Path(
            "test_results/voice_nav_sim/"
            "test_test_simulation_interfaces.py.xunit.xml"
        ): frozenset(
            {
                (
                    "voice_nav_sim.SimulationInterfacesTest",
                    "test_perception_odom_tf_and_ownership_contract",
                ),
                (
                    "voice_nav_sim.SimulationInterfacesShutdownTest",
                    "test_all_launch_managed_processes_exit_cleanly",
                ),
            }
        ),
        Path(
            "test_results/voice_nav_sim/"
            "test_test_tf_ownership_conflict.py.xunit.xml"
        ): frozenset(
            {
                (
                    "voice_nav_sim.TfOwnershipConflictTest",
                    (
                        "test_normal_audit_rejects_and_sentinel_"
                        "proves_the_conflict"
                    ),
                ),
            }
        ),
    },
}


@dataclass(frozen=True)
class PackageEvidence:
    """Parsed results and their original files for one selected package."""

    package_directory: Path
    package_identity: PackageSnapshotIdentity
    results: tuple[Result, ...]
    files: tuple[Path, ...]
    file_identities: tuple[ResultFileIdentity, ...]


@dataclass(frozen=True)
class ParsedSandbox:
    """Official parser output with CTest provenance kept explicit."""

    results: tuple[Result, ...]
    files: tuple[str, ...]
    ctest_results: tuple[Result, ...]
    ctest_files: tuple[str, ...]
    xunit_results: tuple[Result, ...]
    xunit_files: tuple[str, ...]


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Collect test results only from explicitly selected package "
            "build directories."
        )
    )
    parser.add_argument(
        "--build-base",
        type=Path,
        default=Path("build"),
        help="colcon build base containing one directory per package",
    )
    parser.add_argument(
        "--package",
        action="append",
        dest="packages",
        required=True,
        help="exact package name to include; repeat for multiple packages",
    )
    parser.add_argument(
        "--clear",
        action="store_true",
        help="delete selected packages' generated result files before testing",
    )
    return parser.parse_args()


def _map_sandbox_path(
    sandbox_path: str,
    sandbox_package: Path,
    source_by_relative_path: dict[Path, Path],
) -> Path:
    path = Path(sandbox_path)
    if not path.is_absolute():
        path = sandbox_package / path
    try:
        relative_path = path.relative_to(sandbox_package)
    except ValueError as error:
        raise ValueError(
            f"test-result parser escaped its sandbox: {sandbox_path}"
        ) from error
    if any(part in (".", "..") for part in relative_path.parts):
        raise ValueError(
            f"test-result parser escaped its sandbox: {sandbox_path}"
        )
    try:
        return source_by_relative_path[relative_path]
    except KeyError as error:
        raise ValueError(
            f"test-result parser reported an unvalidated input: {sandbox_path}"
        ) from error


def _parse_sandbox(
    sandbox_package: Path,
    *,
    collect_details: bool,
) -> ParsedSandbox:
    all_results: set[Result] = set()
    all_files: set[str] = set()
    ctest_results: set[Result] = set()
    ctest_files: set[str] = set()
    xunit_results: set[Result] = set()
    xunit_files: set[str] = set()

    for extension_name, extension in get_test_result_extensions().items():
        extension_files: set[str] = set()
        function = extension.get_test_results
        keyword_arguments = {"collect_details": collect_details}
        if "files" in inspect.signature(function).parameters:
            keyword_arguments["files"] = extension_files
        try:
            extension_results = function(
                sandbox_package,
                **keyword_arguments,
            )
        except Exception as error:
            raise ValueError(
                f"{extension_name} test-result parser failed: {error}"
            ) from error
        if not isinstance(extension_results, set):
            raise ValueError(
                f"{extension_name} test-result parser returned invalid data"
            )
        all_results |= extension_results
        all_files |= extension_files
        if extension_name == "ctest":
            ctest_results = extension_results
            ctest_files = extension_files
        elif extension_name == "xunit":
            xunit_results = extension_results
            xunit_files = extension_files

    return ParsedSandbox(
        results=tuple(all_results),
        files=tuple(all_files),
        ctest_results=tuple(ctest_results),
        ctest_files=tuple(ctest_files),
        xunit_results=tuple(xunit_results),
        xunit_files=tuple(xunit_files),
    )


def _require_complete_ctest_evidence(
    sandbox_package: Path,
    parsed: ParsedSandbox,
) -> None:
    ctest_files = {Path(path) for path in parsed.ctest_files}
    ctest_result_paths = {Path(result.path) for result in parsed.ctest_results}
    for tag_file in sorted(sandbox_package.glob("**/Testing/TAG")):
        latest_xml = ctest_result_path(tag_file, sandbox_package)
        if (
            tag_file not in ctest_files
            or latest_xml not in ctest_files
            or latest_xml not in ctest_result_paths
        ):
            raise ValueError(
                f"CTest TAG did not produce a result: {tag_file}"
            )


def _require_complete_xunit_evidence(
    sandbox_package: Path,
    parsed: ParsedSandbox,
) -> None:
    xunit_files = {Path(path) for path in parsed.xunit_files}
    xunit_result_paths = {Path(result.path) for result in parsed.xunit_results}
    for xml_file in sorted(sandbox_package.rglob("*.xml")):
        relative_path = xml_file.relative_to(sandbox_package)
        mandatory = (
            "test_results" in relative_path.parts
            or relative_path.parts == ("pytest.xml",)
            or xml_file.name.endswith(".xunit.xml")
            or xml_file.name.endswith(".gtest.xml")
        )
        if not mandatory:
            continue
        if xml_file not in xunit_files or xml_file not in xunit_result_paths:
            raise ValueError(
                "mandatory xUnit result was not consumed: "
                f"{xml_file}"
            )


def _require_skip_policy(
    sandbox_package: Path,
    parsed: ParsedSandbox,
) -> None:
    allowed_path = Path(
        "test_results",
        sandbox_package.name,
        "cppcheck.xunit.xml",
    )
    allowed_classname = f"{sandbox_package.name}.cppcheck"
    for raw_path in sorted(parsed.xunit_files):
        result_path = Path(raw_path)
        try:
            relative_path = result_path.relative_to(sandbox_package)
        except ValueError as error:
            raise ValueError(
                f"xUnit skip policy escaped its sandbox: {result_path}"
            ) from error
        try:
            root = ElementTree.parse(result_path).getroot()
        except (ElementTree.ParseError, OSError) as error:
            raise ValueError(
                f"xUnit skip policy could not parse: {relative_path}"
            ) from error

        declared_skip = False
        for suite in root.iter("testsuite"):
            raw_count = suite.get("skipped")
            if raw_count is None:
                continue
            try:
                skip_count = int(raw_count)
            except ValueError as error:
                raise ValueError(
                    "xUnit skip policy found an invalid skipped count: "
                    f"{relative_path}"
                ) from error
            if skip_count < 0:
                raise ValueError(
                    "xUnit skip policy found an invalid skipped count: "
                    f"{relative_path}"
                )
            declared_skip = declared_skip or skip_count > 0

        skipped_cases = tuple(
            case
            for case in root.iter("testcase")
            if case.find("skipped") is not None
        )
        if not declared_skip and not skipped_cases:
            continue
        if relative_path != allowed_path:
            raise ValueError(
                "skipped tests are not allowlisted: "
                f"{relative_path}"
            )
        if not skipped_cases or any(
            case.get("classname") != allowed_classname
            for case in skipped_cases
        ):
            raise ValueError(
                "cppcheck skip evidence is malformed: "
                f"{relative_path}"
            )


def _xunit_count(element, attribute: str, result_path: Path) -> int:
    value = element.get(attribute)
    try:
        count = int(value) if value is not None else -1
    except ValueError as error:
        raise ValueError(
            "critical launch evidence has an invalid "
            f"{attribute} count: {result_path}"
        ) from error
    if count < 0:
        raise ValueError(
            "critical launch evidence is missing a valid "
            f"{attribute} count: {result_path}"
        )
    return count


def _require_critical_launch_evidence(sandbox_package: Path) -> None:
    required_results = CRITICAL_LAUNCH_CASES.get(sandbox_package.name)
    if required_results is None:
        return

    for relative_path, required_cases in required_results.items():
        result_path = sandbox_package / relative_path
        if not result_path.is_file():
            raise ValueError(
                "critical launch evidence is missing: "
                f"{relative_path}"
            )
        try:
            root = ElementTree.parse(result_path).getroot()
        except (ElementTree.ParseError, OSError) as error:
            raise ValueError(
                "critical launch evidence is not valid XML: "
                f"{relative_path}"
            ) from error

        if root.tag == "testsuite":
            suites = [root]
            aggregate_root = None
        elif root.tag == "testsuites":
            suites = [
                child for child in root if child.tag == "testsuite"
            ]
            aggregate_root = root
        else:
            raise ValueError(
                "critical launch evidence has an invalid root: "
                f"{relative_path}"
            )
        if not suites:
            raise ValueError(
                "critical launch evidence has no test suite: "
                f"{relative_path}"
            )
        if aggregate_root is not None and any(
            child.tag == "testcase" for child in aggregate_root
        ):
            raise ValueError(
                "critical launch evidence has test cases outside a suite: "
                f"{relative_path}"
            )

        case_elements = []
        suite_totals = {
            "tests": 0,
            "errors": 0,
            "failures": 0,
            "skipped": 0,
        }
        for suite in suites:
            if any(child.tag == "testsuite" for child in suite):
                raise ValueError(
                    "critical launch evidence has nested test suites: "
                    f"{relative_path}"
                )
            suite_cases = [
                child for child in suite if child.tag == "testcase"
            ]
            actual = {
                "tests": len(suite_cases),
                "errors": sum(
                    case.find("error") is not None for case in suite_cases
                ),
                "failures": sum(
                    case.find("failure") is not None for case in suite_cases
                ),
                "skipped": sum(
                    case.find("skipped") is not None for case in suite_cases
                ),
            }
            for attribute, actual_count in actual.items():
                if _xunit_count(suite, attribute, relative_path) != actual_count:
                    raise ValueError(
                        "critical launch evidence has an inconsistent "
                        f"{attribute} count: {relative_path}"
                    )
                suite_totals[attribute] += actual_count
            if any(actual[outcome] for outcome in ("errors", "failures", "skipped")):
                raise ValueError(
                    "critical launch evidence contains a non-passing case: "
                    f"{relative_path}"
                )
            case_elements.extend(suite_cases)

        if aggregate_root is not None:
            for attribute in ("tests", "errors", "failures"):
                if (
                    _xunit_count(
                        aggregate_root,
                        attribute,
                        relative_path,
                    )
                    != suite_totals[attribute]
                ):
                    raise ValueError(
                        "critical launch evidence has an inconsistent "
                        f"aggregate {attribute} count: {relative_path}"
                    )
            if aggregate_root.get("skipped") is not None and (
                _xunit_count(
                    aggregate_root,
                    "skipped",
                    relative_path,
                )
                != suite_totals["skipped"]
            ):
                raise ValueError(
                    "critical launch evidence has an inconsistent "
                    f"aggregate skipped count: {relative_path}"
                )

        if not case_elements:
            raise ValueError(
                "critical launch evidence has no test cases: "
                f"{relative_path}"
            )
        case_ids = tuple(
            (case.get("classname", ""), case.get("name", ""))
            for case in case_elements
        )
        if any(not classname or not name for classname, name in case_ids):
            raise ValueError(
                "critical launch evidence contains an unnamed test case: "
                f"{relative_path}"
            )
        duplicate_cases = sorted(
            case_id
            for case_id, count in Counter(case_ids).items()
            if count > 1
        )
        if duplicate_cases:
            raise ValueError(
                "critical launch evidence contains duplicate test cases: "
                f"{relative_path}: {duplicate_cases}"
            )
        missing_cases = sorted(required_cases - set(case_ids))
        if missing_cases:
            raise ValueError(
                "critical launch evidence inventory is incomplete: "
                f"{relative_path}: missing={missing_cases}"
            )


def collect_package_evidence(
    package_directory: Path,
    *,
    collect_details: bool,
    require_results: bool,
) -> PackageEvidence:
    """Parse one package from a symlink-free copy of validated inputs."""

    with (
        open_result_snapshot(package_directory) as snapshot,
        tempfile.TemporaryDirectory(
            prefix=f"voice-nav-results-{package_directory.name}-"
        ) as temporary_directory,
    ):
        sandbox_package = Path(temporary_directory) / package_directory.name
        source_by_relative_path = snapshot.stage(sandbox_package)

        parsed = _parse_sandbox(
            sandbox_package,
            collect_details=collect_details,
        )
        _require_complete_ctest_evidence(sandbox_package, parsed)
        _require_complete_xunit_evidence(sandbox_package, parsed)
        package_results = set(parsed.results)
        if require_results and not package_results:
            raise ValueError(
                "no test results found for selected package: "
                f"{package_directory.name}"
            )
        if require_results:
            _require_critical_launch_evidence(sandbox_package)
            _require_skip_policy(sandbox_package, parsed)

        mapped_files = tuple(
            sorted(
                _map_sandbox_path(
                    sandbox_file,
                    sandbox_package,
                    source_by_relative_path,
                )
                for sandbox_file in parsed.files
            )
        )
        mapped_identities = tuple(
            snapshot.identity_for(
                result_file.relative_to(package_directory)
            )
            for result_file in mapped_files
        )
        package_identity = snapshot.package_identity()
        for result in package_results:
            result.path = str(
                _map_sandbox_path(
                    result.path,
                    sandbox_package,
                    source_by_relative_path,
                )
            )

    return PackageEvidence(
        package_directory=package_directory,
        package_identity=package_identity,
        results=tuple(sorted(package_results, key=lambda result: result.path)),
        files=mapped_files,
        file_identities=mapped_identities,
    )


def collect_evidence(
    package_directories: list[Path],
    *,
    collect_details: bool,
    require_results: bool,
) -> tuple[PackageEvidence, ...]:
    return tuple(
        collect_package_evidence(
            package_directory,
            collect_details=collect_details,
            require_results=require_results,
        )
        for package_directory in package_directories
    )


def clear_results(package_directories: list[Path]) -> int:
    evidence = collect_evidence(
        package_directories,
        collect_details=False,
        require_results=False,
    )

    files_to_clear = sum(
        (len(package_evidence.files) for package_evidence in evidence),
        start=0,
    )
    with ExitStack() as stack:
        deletion_plans = tuple(
            stack.enter_context(
                open_result_deletion_plan(
                    package_evidence.package_directory,
                    package_evidence.files,
                    expected_identities=package_evidence.file_identities,
                    expected_package_identity=package_evidence.package_identity,
                )
            )
            for package_evidence in evidence
        )
        # Every selected package completes its anchored preflight before the
        # first mutation.  unlink_all() then rechecks each identity and uses
        # unlinkat-style dir_fd deletion without following intermediate links.
        for deletion_plan in deletion_plans:
            deletion_plan.validate_all()
        for deletion_plan in deletion_plans:
            deletion_plan.unlink_all()

    suffix = "file" if files_to_clear == 1 else "files"
    print(
        f"Cleared {files_to_clear} selected package test-result "
        f"{suffix}."
    )
    return 0


def report_results(package_directories: list[Path]) -> int:
    evidence = collect_evidence(
        package_directories,
        collect_details=True,
        require_results=True,
    )
    all_results = [
        result
        for package_evidence in evidence
        for result in package_evidence.results
    ]
    failed_results = [
        result
        for result in all_results
        if result.error_count or result.failure_count
    ]
    for result in failed_results:
        print(result)
        for detail in result.details:
            for index, line in enumerate(detail.splitlines()):
                print("-" if index == 0 else " ", line)

    summary = Result("Summary")
    for result in all_results:
        summary.add_result(result)
    if failed_results:
        print()
    print(summary)
    return 1 if summary.error_count or summary.failure_count else 0


def main() -> int:
    arguments = parse_arguments()
    try:
        package_directories = selected_package_directories(
            arguments.build_base,
            arguments.packages,
        )
        if arguments.clear:
            return clear_results(package_directories)
        return report_results(package_directories)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
