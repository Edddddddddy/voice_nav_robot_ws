#!/usr/bin/env python3
"""Check Conventional Commit subjects in a supplied range or direct input."""

import argparse
import re
import subprocess
import sys


SUBJECT_PATTERN = re.compile(
    r"^(feat|fix|test|docs|refactor|perf|build|ci|chore|revert)"
    r"(?:\([a-z0-9][a-z0-9._/-]*\))?!?: .+"
)
HAN_CHARACTER_PATTERN = re.compile(r"[\u4e00-\u9fff]")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--subject", action="append")
    source.add_argument("--range")
    return parser.parse_args()


def subjects_in_range(commit_range: str) -> list[str]:
    result = subprocess.run(
        ["git", "log", "--format=%s", commit_range],
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise ValueError(result.stderr.strip() or "无法读取 Git 提交范围。")
    subjects = [subject for subject in result.stdout.splitlines() if subject]
    if not subjects:
        raise ValueError("提交范围为空，无法验证 Conventional Commit。")
    return subjects


def main() -> int:
    arguments = parse_arguments()
    try:
        subjects = arguments.subject or subjects_in_range(arguments.range)
    except ValueError as error:
        print(f"Conventional Commit 检查失败：{error}", file=sys.stderr)
        return 1

    invalid_format = [
        subject for subject in subjects if not SUBJECT_PATTERN.fullmatch(subject)
    ]
    if invalid_format:
        print("Conventional Commit 检查失败：以下提交格式无效：", file=sys.stderr)
        for subject in invalid_format:
            print(f"- {subject}", file=sys.stderr)
        return 1
    english_only = [
        subject for subject in subjects if not HAN_CHARACTER_PATTERN.search(subject)
    ]
    if english_only:
        print("Conventional Commit 检查失败：摘要必须包含简体中文。", file=sys.stderr)
        for subject in english_only:
            print(f"- {subject}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
