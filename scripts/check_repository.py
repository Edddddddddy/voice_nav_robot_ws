#!/usr/bin/env python3
"""Validate versioned repository and course contracts."""

from __future__ import annotations

import argparse
import ast
import os
import re
import sys
import tomllib
import xml.etree.ElementTree as element_tree
from pathlib import Path
from urllib.parse import unquote, urlparse


LESSON_ID_PATTERN = re.compile(r"\d{4}\Z")
SLUG_PATTERN = re.compile(r"[a-z0-9]+(?:-[a-z0-9]+)*\Z")
LESSON_STATUSES = {"planned", "ready", "in_progress", "completed"}
LEGACY_DOCUMENT_PATHS = (
    "lessons",
    "learning-records",
    "reference",
    "assets/lesson.css",
    "MISSION.md",
    "CONTEXT.md",
    "NOTES.md",
    "RESOURCES.md",
)
MARKDOWN_LINK_PATTERN = re.compile(r"!?\[[^\]]*]\(([^)]+)\)")
SEMANTIC_VERSION_PATTERN = re.compile(
    r"(?:0|[1-9]\d*)\.(?:0|[1-9]\d*)\.(?:0|[1-9]\d*)\Z"
)
EXTERNAL_MARKDOWN_SCHEMES = {"http", "https", "mailto"}
EXCLUDED_SCAN_DIRECTORIES = {
    ".deps",
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".venv",
    "__pycache__",
    "artifacts",
    "bags",
    "build",
    "install",
    "log",
    "models",
    "recordings",
    "venv",
}
TEXT_FILE_NAMES = {
    ".editorconfig",
    ".gitattributes",
    ".gitignore",
    "CMakeLists.txt",
    "LICENSE",
    "VERSION",
}
TEXT_FILE_SUFFIXES = {
    ".action",
    ".cfg",
    ".cmake",
    ".cpp",
    ".css",
    ".h",
    ".hpp",
    ".html",
    ".idl",
    ".json",
    ".md",
    ".msg",
    ".py",
    ".sdf",
    ".sh",
    ".srv",
    ".toml",
    ".txt",
    ".urdf",
    ".xacro",
    ".xml",
    ".xsd",
    ".yaml",
    ".yml",
}


class ContractError(ValueError):
    """A repository contract was not satisfied."""


def validate_course_path(
    root: Path,
    lesson_id: str,
    path_field: str,
    configured_path: str,
) -> None:
    relative_path = Path(configured_path)
    expected_directory = (root / "course" / f"{path_field}s").resolve()
    if relative_path.is_absolute():
        raise ContractError(
            f"course lesson {lesson_id} {path_field} path must be relative"
        )

    referenced_path = (root / relative_path).resolve()
    if not referenced_path.is_relative_to(expected_directory):
        raise ContractError(
            f"course lesson {lesson_id} {path_field} must stay under "
            f"course/{path_field}s"
        )
    if referenced_path.suffix != ".md":
        raise ContractError(
            f"course lesson {lesson_id} {path_field} must be a Markdown file"
        )
    if not referenced_path.name.startswith(f"{lesson_id}-"):
        raise ContractError(
            f"course lesson {lesson_id} {path_field} filename must start "
            f"with {lesson_id}-"
        )
    if not referenced_path.is_file():
        raise ContractError(
            f"course lesson {lesson_id} references missing "
            f"{path_field}: {configured_path}"
        )


def load_course_catalog(root: Path) -> dict[str, object]:
    catalog_path = root / "course" / "catalog.toml"
    try:
        with catalog_path.open("rb") as catalog_file:
            catalog = tomllib.load(catalog_file)
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise ContractError(f"cannot read {catalog_path}: {error}") from error

    if catalog.get("schema_version") != 1:
        raise ContractError("course catalog schema_version must be 1")

    lessons = catalog.get("lessons")
    if not isinstance(lessons, list) or not lessons:
        raise ContractError("course catalog must contain at least one lesson")

    lesson_ids: set[str] = set()
    for lesson in lessons:
        if not isinstance(lesson, dict):
            raise ContractError("each course lesson must be a table")
        for field in ("id", "slug", "status", "lesson", "record"):
            if not isinstance(lesson.get(field), str) or not lesson[field]:
                raise ContractError(f"course lesson field {field!r} is required")
        lesson_id = lesson["id"]
        if LESSON_ID_PATTERN.fullmatch(lesson_id) is None:
            raise ContractError(
                f"course lesson id must contain exactly four digits: {lesson_id}"
            )
        if lesson_id in lesson_ids:
            raise ContractError(f"duplicate course lesson id: {lesson_id}")
        lesson_ids.add(lesson_id)
        if SLUG_PATTERN.fullmatch(lesson["slug"]) is None:
            raise ContractError(
                f"course lesson {lesson_id} has invalid slug: {lesson['slug']}"
            )
        if lesson["status"] not in LESSON_STATUSES:
            raise ContractError(
                f"course lesson {lesson_id} has invalid status: {lesson['status']}"
            )
        for path_field in ("lesson", "record"):
            validate_course_path(
                root,
                lesson_id,
                path_field,
                lesson[path_field],
            )

    ordered_ids = [lesson["id"] for lesson in lessons]
    expected_ids = [f"{index:04d}" for index in range(1, len(lessons) + 1)]
    if ordered_ids != expected_ids:
        raise ContractError(
            "course lesson ids must be sorted and contiguous from 0001; "
            f"found {', '.join(ordered_ids)}"
        )

    return catalog


def validate_documentation_layout(root: Path) -> None:
    for legacy_path in LEGACY_DOCUMENT_PATHS:
        if (root / legacy_path).exists():
            raise ContractError(
                f"legacy documentation path remains: {legacy_path}"
            )


def repository_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for directory, child_directories, filenames in os.walk(root):
        child_directories[:] = [
            child
            for child in child_directories
            if child not in EXCLUDED_SCAN_DIRECTORIES
        ]
        current_directory = Path(directory)
        files.extend(current_directory / filename for filename in filenames)
    return sorted(files)


def markdown_files(root: Path) -> list[Path]:
    return [path for path in repository_files(root) if path.suffix == ".md"]


def validate_text_hygiene(root: Path) -> None:
    for path in repository_files(root):
        if path.name not in TEXT_FILE_NAMES and path.suffix not in TEXT_FILE_SUFFIXES:
            continue
        try:
            data = path.read_bytes()
        except OSError as error:
            raise ContractError(f"cannot read {path}: {error}") from error
        try:
            text = data.decode("utf-8")
        except UnicodeError as error:
            raise ContractError(
                f"text file must be UTF-8: {path.relative_to(root)}"
            ) from error
        if text and not text.endswith("\n"):
            raise ContractError(
                f"text file must end with a newline: {path.relative_to(root)}"
            )
        for line_number, line in enumerate(text.splitlines(), start=1):
            if line.endswith((" ", "\t")):
                raise ContractError(
                    "trailing whitespace: "
                    f"{path.relative_to(root)}:{line_number}"
                )


def link_target(raw_target: str) -> str:
    target = raw_target.strip()
    if target.startswith("<") and ">" in target:
        return target[1 : target.index(">")]
    return target.split(maxsplit=1)[0]


def validate_markdown_links(root: Path) -> None:
    resolved_root = root.resolve()
    for markdown_path in markdown_files(root):
        try:
            lines = markdown_path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError) as error:
            raise ContractError(
                f"cannot read Markdown file {markdown_path}: {error}"
            ) from error

        fence_character: str | None = None
        fence_length = 0
        fence_start = 0
        for line_number, line in enumerate(lines, start=1):
            stripped = line.lstrip()
            if stripped.startswith(("```", "~~~")):
                marker_character = stripped[0]
                marker_length = len(stripped) - len(
                    stripped.lstrip(marker_character)
                )
                if fence_character is None:
                    fence_character = marker_character
                    fence_length = marker_length
                    fence_start = line_number
                elif (
                    marker_character == fence_character
                    and marker_length >= fence_length
                ):
                    fence_character = None
                    fence_length = 0
                    fence_start = 0
                continue
            if fence_character is not None:
                continue
            for match in MARKDOWN_LINK_PATTERN.finditer(line):
                target = unquote(link_target(match.group(1)))
                if not target or target.startswith("#"):
                    continue
                parsed = urlparse(target)
                if parsed.scheme:
                    if parsed.scheme.lower() not in EXTERNAL_MARKDOWN_SCHEMES:
                        raise ContractError(
                            "unsupported Markdown link scheme: "
                            f"{parsed.scheme.lower()} in "
                            f"{markdown_path.relative_to(root)}:{line_number}"
                        )
                    continue
                if target.startswith("//"):
                    raise ContractError(
                        "protocol-relative Markdown links are not allowed: "
                        f"{markdown_path.relative_to(root)}:{line_number}"
                    )
                path_without_fragment = target.split("#", 1)[0].split("?", 1)[0]
                if not path_without_fragment:
                    continue
                if path_without_fragment.startswith("/"):
                    linked_path = resolved_root / path_without_fragment.lstrip("/")
                else:
                    linked_path = markdown_path.parent / path_without_fragment
                resolved_link = linked_path.resolve()
                if not resolved_link.is_relative_to(resolved_root):
                    raise ContractError(
                        "local Markdown link escapes repository: "
                        f"{markdown_path.relative_to(root)}:{line_number}: {target}"
                    )
                if not resolved_link.exists():
                    raise ContractError(
                        "broken local Markdown link: "
                        f"{markdown_path.relative_to(root)}:{line_number}: {target}"
                    )
        if fence_character is not None:
            raise ContractError(
                "unclosed Markdown fence: "
                f"{markdown_path.relative_to(root)}:{fence_start}"
            )


def validate_ros_package_versions(root: Path) -> None:
    package_paths = sorted((root / "src").glob("*/package.xml"))
    if not package_paths:
        return

    version_path = root / "VERSION"
    try:
        project_version = version_path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError) as error:
        raise ContractError(f"cannot read {version_path}: {error}") from error
    if SEMANTIC_VERSION_PATTERN.fullmatch(project_version) is None:
        raise ContractError(
            f"VERSION must contain a stable semantic version: {project_version}"
        )

    for package_path in package_paths:
        try:
            package = element_tree.parse(package_path).getroot()
        except (OSError, element_tree.ParseError) as error:
            raise ContractError(f"cannot parse {package_path}: {error}") from error
        package_name = package.findtext("name")
        package_version = package.findtext("version")
        if not package_name or not package_version:
            raise ContractError(
                f"{package_path.relative_to(root)} must define name and version"
            )
        if package_version != project_version:
            raise ContractError(
                f"{package_name} version {package_version} does not match "
                f"VERSION {project_version}"
            )
        setup_path = package_path.parent / "setup.py"
        if setup_path.is_file():
            try:
                setup_text = setup_path.read_text(encoding="utf-8")
            except (OSError, UnicodeError) as error:
                raise ContractError(f"cannot read {setup_path}: {error}") from error
            try:
                setup_tree = ast.parse(setup_text, filename=str(setup_path))
            except SyntaxError as error:
                raise ContractError(f"cannot parse {setup_path}: {error}") from error
            setup_versions: list[str] = []
            for node in ast.walk(setup_tree):
                if not isinstance(node, ast.Call):
                    continue
                function = node.func
                is_setup_call = (
                    isinstance(function, ast.Name) and function.id == "setup"
                ) or (
                    isinstance(function, ast.Attribute)
                    and function.attr == "setup"
                )
                if not is_setup_call:
                    continue
                for keyword in node.keywords:
                    if (
                        keyword.arg == "version"
                        and isinstance(keyword.value, ast.Constant)
                        and isinstance(keyword.value.value, str)
                    ):
                        setup_versions.append(keyword.value.value)
            if len(setup_versions) != 1:
                raise ContractError(
                    f"{package_name} setup.py must define exactly one literal "
                    "setup(version=...)"
                )
            setup_version = setup_versions[0]
            if setup_version != project_version:
                raise ContractError(
                    f"{package_name} setup.py version {setup_version} does not "
                    f"match VERSION {project_version}"
                )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="repository root to validate",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        root = arguments.root.resolve()
        load_course_catalog(root)
        validate_documentation_layout(root)
        validate_text_hygiene(root)
        validate_markdown_links(root)
        validate_ros_package_versions(root)
    except ContractError as error:
        print(f"Repository contract failed: {error}", file=sys.stderr)
        return 1

    print("Repository contract passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
