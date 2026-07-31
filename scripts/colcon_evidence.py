"""Path-safety primitives for deterministic colcon verification evidence."""

import re
from pathlib import Path


PACKAGE_NAME_PATTERN = re.compile(r"[A-Za-z0-9_]+")


def validate_package_names(package_names: list[str]) -> tuple[str, ...]:
    if not package_names:
        raise ValueError("at least one --package is required")
    if len(set(package_names)) != len(package_names):
        raise ValueError("duplicate package name")
    for package_name in package_names:
        if PACKAGE_NAME_PATTERN.fullmatch(package_name) is None:
            raise ValueError(f"invalid package name: {package_name}")
    return tuple(package_names)


def resolve_build_base(
    build_base: Path,
    *,
    allow_missing: bool,
) -> Path | None:
    if build_base.is_symlink():
        raise ValueError(f"build base must not be a symbolic link: {build_base}")
    if not build_base.exists():
        if allow_missing:
            return None
        raise ValueError(f"build base does not exist: {build_base}")
    resolved_base = build_base.resolve(strict=True)
    if not resolved_base.is_dir():
        raise ValueError(f"build base is not a directory: {resolved_base}")
    return resolved_base


def selected_package_directories(
    build_base: Path,
    package_names: list[str],
) -> list[Path]:
    validated_names = validate_package_names(package_names)
    resolved_base = resolve_build_base(build_base, allow_missing=False)
    assert resolved_base is not None

    directories = []
    for package_name in validated_names:
        package_directory = resolved_base / package_name
        if package_directory.is_symlink():
            raise ValueError(
                "package build directory must be a direct, non-symlinked child "
                f"of the build base: {package_directory}"
            )
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


def unexpected_build_entries(
    build_base: Path,
    package_names: list[str],
) -> list[tuple[str, str]]:
    allowed_names = set(validate_package_names(package_names))
    resolved_base = resolve_build_base(build_base, allow_missing=True)
    if resolved_base is None:
        return []

    offenders = []
    for entry in sorted(resolved_base.iterdir(), key=lambda path: path.name):
        if entry.is_symlink():
            offenders.append((entry.name, "symbolic link"))
        elif entry.is_dir() and entry.name not in allowed_names:
            offenders.append((entry.name, "unexpected directory"))
    return offenders
