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


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    suite = unittest.defaultTestLoader.discover(
        str(repository_root / "tests"),
        pattern="test_*.py",
        top_level_dir=str(repository_root),
    )
    return run_suite(suite, stream=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
