"""Path-safety primitives for deterministic colcon verification evidence."""

import os
import re
import stat
from pathlib import Path


PACKAGE_NAME_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*")
CTEST_TAG_ENTRY_PATTERN = re.compile(r"[A-Za-z0-9_.-]+")


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


def validate_result_input(path: Path, package_directory: Path) -> Path:
    """Return a regular result input owned lexically by one package.

    The caller passes an already resolved package directory.  Result inputs
    must not use symbolic links at any component of their package-relative
    path.  Keeping this check package-local prevents one selected package
    from borrowing another selected package's evidence.
    """

    try:
        relative_path = path.relative_to(package_directory)
    except ValueError as error:
        raise ValueError(
            f"result path is outside its selected package: {path}"
        ) from error
    if not relative_path.parts or any(
        part in (".", "..") for part in relative_path.parts
    ):
        raise ValueError(f"unsafe result path: {path}")

    current_path = package_directory
    for part in relative_path.parts:
        current_path = current_path / part
        if current_path.is_symlink():
            raise ValueError(
                f"result path contains symbolic link: {current_path}"
            )

    try:
        mode = path.lstat().st_mode
    except FileNotFoundError as error:
        raise ValueError(f"result path does not exist: {path}") from error
    if not stat.S_ISREG(mode):
        raise ValueError(f"result path is not a regular file: {path}")

    resolved_path = path.resolve(strict=True)
    if (
        not resolved_path.is_relative_to(package_directory)
        or resolved_path != path
    ):
        raise ValueError(
            f"result path is outside its selected package: {path}"
        )
    return path


def discover_result_inputs(package_directory: Path) -> tuple[Path, ...]:
    """Discover safe XML and CTest inputs without following symlinks."""

    result_inputs: set[Path] = set()
    tag_files: list[Path] = []

    for directory, directory_names, file_names in os.walk(
        package_directory,
        topdown=True,
        followlinks=False,
    ):
        directory_path = Path(directory)

        traversable_directories = []
        for directory_name in sorted(directory_names):
            child_directory = directory_path / directory_name
            if child_directory.is_symlink():
                if directory_name == "Testing":
                    raise ValueError(
                        "result path contains symbolic link: "
                        f"{child_directory}"
                    )
                continue
            traversable_directories.append(directory_name)
        directory_names[:] = traversable_directories

        for file_name in sorted(file_names):
            path = directory_path / file_name
            is_ctest_tag = (
                file_name == "TAG" and directory_path.name == "Testing"
            )
            is_xml = file_name.endswith(".xml")
            if not is_ctest_tag and not is_xml:
                continue

            # Python packages built with --symlink-install contain this source
            # metadata link.  It can never contribute test evidence, so ignore
            # it without following it while rejecting all result-like links.
            if is_xml and file_name == "package.xml" and path.is_symlink():
                continue

            validate_result_input(path, package_directory)
            result_inputs.add(path)
            if is_ctest_tag:
                tag_files.append(path)

    for tag_file in tag_files:
        lines = tag_file.read_text(encoding="utf-8").splitlines()
        if not lines:
            raise ValueError(f"malformed CTest TAG file: {tag_file}")
        tag_entry = lines[0]
        if (
            tag_entry in (".", "..")
            or CTEST_TAG_ENTRY_PATTERN.fullmatch(tag_entry) is None
        ):
            raise ValueError(
                f"unsafe CTest TAG entry in {tag_file}: {tag_entry!r}"
            )

        latest_xml = tag_file.parent / tag_entry / "Test.xml"
        if latest_xml.exists() or latest_xml.is_symlink():
            validate_result_input(latest_xml, package_directory)
            result_inputs.add(latest_xml)

    return tuple(sorted(result_inputs))


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
