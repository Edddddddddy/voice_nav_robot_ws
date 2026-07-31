#!/usr/bin/env python3

"""Fail when colcon's build root contains stale package directories."""

import argparse
import sys
from pathlib import Path

from colcon_evidence import unexpected_build_entries


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Require every direct build directory to belong to the current "
            "workspace package set."
        )
    )
    parser.add_argument(
        "--build-base",
        type=Path,
        default=Path("build"),
        help="colcon build base to inspect",
    )
    parser.add_argument(
        "--package",
        action="append",
        dest="packages",
        default=[],
        help="current workspace package name; repeat for every package",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        offenders = unexpected_build_entries(
            arguments.build_base,
            arguments.packages,
        )
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if offenders:
        print(
            "Unexpected colcon build entries; move or remove generated "
            "artifacts before verification:",
            file=sys.stderr,
        )
        for relative_path, reason in offenders:
            print(f"- {relative_path}: {reason}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
