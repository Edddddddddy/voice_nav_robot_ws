#!/usr/bin/env python3

"""Report or clear test results for an explicit set of colcon packages."""

import argparse
import re
import sys
from pathlib import Path

from colcon_test_result.test_result import get_test_results
from colcon_test_result.test_result import Result


PACKAGE_NAME_PATTERN = re.compile(r"[A-Za-z0-9_]+")


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


def selected_package_directories(
    build_base: Path,
    package_names: list[str],
) -> list[Path]:
    try:
        resolved_base = build_base.resolve(strict=True)
    except FileNotFoundError as error:
        raise ValueError(f"build base does not exist: {build_base}") from error
    if not resolved_base.is_dir():
        raise ValueError(f"build base is not a directory: {resolved_base}")

    if len(set(package_names)) != len(package_names):
        raise ValueError("duplicate package name")

    directories = []
    for package_name in package_names:
        if PACKAGE_NAME_PATTERN.fullmatch(package_name) is None:
            raise ValueError(f"invalid package name: {package_name}")
        package_directory = resolved_base / package_name
        try:
            resolved_package = package_directory.resolve(strict=True)
        except FileNotFoundError as error:
            raise ValueError(
                f"package build directory does not exist: {package_directory}"
            ) from error
        if not resolved_package.is_dir() or resolved_package.parent != resolved_base:
            raise ValueError(
                "package build directory must be a direct, non-symlinked child "
                f"of the build base: {package_directory}"
            )
        directories.append(resolved_package)
    return directories


def collect_results(
    package_directories: list[Path],
    *,
    collect_details: bool,
    files: set[str] | None = None,
) -> list[Result]:
    results = set()
    for package_directory in package_directories:
        results |= get_test_results(
            package_directory,
            collect_details=collect_details,
            files=files,
        )
    return sorted(results, key=lambda result: result.path)


def clear_results(package_directories: list[Path]) -> int:
    files: set[str] = set()
    collect_results(
        package_directories,
        collect_details=False,
        files=files,
    )

    allowed_roots = [path.resolve(strict=True) for path in package_directories]
    for result_file in sorted(files):
        resolved_file = Path(result_file).resolve(strict=True)
        if not any(
            resolved_file.is_relative_to(package_root)
            for package_root in allowed_roots
        ):
            raise ValueError(
                f"refusing to clear result outside selected packages: {result_file}"
            )
        resolved_file.unlink()

    suffix = "file" if len(files) == 1 else "files"
    print(f"Cleared {len(files)} selected package test-result {suffix}.")
    return 0


def report_results(package_directories: list[Path]) -> int:
    all_results = collect_results(
        package_directories,
        collect_details=True,
    )
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
