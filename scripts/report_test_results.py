#!/usr/bin/env python3

"""Report or clear test results for an explicit set of colcon packages."""

import argparse
import shutil
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

from colcon_test_result.test_result import get_test_results
from colcon_test_result.test_result import Result

from colcon_evidence import discover_result_inputs
from colcon_evidence import selected_package_directories
from colcon_evidence import validate_result_input


@dataclass(frozen=True)
class PackageEvidence:
    """Parsed results and their original files for one selected package."""

    package_directory: Path
    results: tuple[Result, ...]
    files: tuple[Path, ...]


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


def collect_package_evidence(
    package_directory: Path,
    *,
    collect_details: bool,
    require_results: bool,
) -> PackageEvidence:
    """Parse one package from a symlink-free copy of validated inputs."""

    source_inputs = discover_result_inputs(package_directory)
    with tempfile.TemporaryDirectory(
        prefix=f"voice-nav-results-{package_directory.name}-"
    ) as temporary_directory:
        sandbox_package = Path(temporary_directory) / package_directory.name
        source_by_relative_path: dict[Path, Path] = {}
        for source_path in source_inputs:
            relative_path = source_path.relative_to(package_directory)
            destination_path = sandbox_package / relative_path
            destination_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(
                source_path,
                destination_path,
                follow_symlinks=False,
            )
            source_by_relative_path[relative_path] = source_path

        sandbox_files: set[str] = set()
        package_results = get_test_results(
            sandbox_package,
            collect_details=collect_details,
            files=sandbox_files,
        )
        if require_results and not package_results:
            raise ValueError(
                "no test results found for selected package: "
                f"{package_directory.name}"
            )

        mapped_files = tuple(
            sorted(
                _map_sandbox_path(
                    sandbox_file,
                    sandbox_package,
                    source_by_relative_path,
                )
                for sandbox_file in sandbox_files
            )
        )
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
        results=tuple(sorted(package_results, key=lambda result: result.path)),
        files=mapped_files,
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

    # Revalidate every original file before making the first mutation.  Keep
    # the source package paired with each path so selected roots never form a
    # cross-package ownership union.
    files_to_clear = tuple(
        (package_evidence.package_directory, result_file)
        for package_evidence in evidence
        for result_file in package_evidence.files
    )
    for package_directory, result_file in files_to_clear:
        validate_result_input(result_file, package_directory)
    for _, result_file in files_to_clear:
        result_file.unlink()

    suffix = "file" if len(files_to_clear) == 1 else "files"
    print(
        f"Cleared {len(files_to_clear)} selected package test-result "
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
