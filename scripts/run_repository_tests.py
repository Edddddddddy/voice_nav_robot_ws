#!/usr/bin/env python3
"""Run repository contract tests and fail closed on every skip."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


def run_suite(
    suite: unittest.TestSuite,
    *,
    stream,
) -> int:
    """Run one contract suite; skipped tests are contract failures."""
    result = unittest.TextTestRunner(
        stream=stream,
        verbosity=2,
    ).run(suite)
    if result.skipped:
        stream.write(
            "\nRepository contract forbids skipped tests:\n"
        )
        for test, reason in result.skipped:
            stream.write(f"- {test.id()}: {reason}\n")
        return 1
    return 0 if result.wasSuccessful() else 1


def discover_suite(repository_root: Path) -> unittest.TestSuite:
    """Discover contract tests from the repository's non-package tests tree."""
    repository_path = str(repository_root.resolve())
    tests_path = str((repository_root / "tests").resolve())
    if repository_path not in sys.path:
        sys.path.insert(0, repository_path)

    tests_path_was_present = tests_path in sys.path
    try:
        return unittest.TestLoader().discover(
            tests_path,
            pattern="test_*.py",
        )
    finally:
        if not tests_path_was_present:
            while tests_path in sys.path:
                sys.path.remove(tests_path)


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    suite = discover_suite(repository_root)
    return run_suite(suite, stream=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
