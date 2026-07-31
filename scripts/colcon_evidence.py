"""Path-safety primitives for deterministic colcon verification evidence."""

import os
import re
import stat
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator


PACKAGE_NAME_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*")
CTEST_TAG_ENTRY_PATTERN = re.compile(r"[A-Za-z0-9_.-]+")
DIRECTORY_OPEN_FLAGS = (
    os.O_RDONLY
    | os.O_DIRECTORY
    | os.O_NOFOLLOW
    | getattr(os, "O_CLOEXEC", 0)
)


@dataclass(frozen=True)
class ResultFileIdentity:
    """Package-relative identity captured before any clear mutation."""

    relative_path: Path
    device: int
    inode: int


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
        result_inputs.add(
            ctest_result_path(tag_file, package_directory)
        )

    return tuple(sorted(result_inputs))


def ctest_result_path(tag_file: Path, package_directory: Path) -> Path:
    """Resolve one validated CTest TAG to its mandatory local result."""

    validate_result_input(tag_file, package_directory)
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
    if not latest_xml.exists() and not latest_xml.is_symlink():
        raise ValueError(f"CTest TAG result does not exist: {latest_xml}")
    return validate_result_input(latest_xml, package_directory)


class ResultDeletionPlan:
    """Delete recognized results through an anchored package directory."""

    def __init__(
        self,
        package_directory: Path,
        result_files: tuple[Path, ...],
    ) -> None:
        self.package_directory = package_directory
        self._workspace_fd = -1
        self._build_fd = -1
        self._package_fd = -1
        self._build_identity: tuple[int, int] | None = None
        self._package_identity: tuple[int, int] | None = None
        self._identities: tuple[ResultFileIdentity, ...] = ()

        try:
            self._open_anchor()
            identities = []
            for result_file in result_files:
                relative_path = self._relative_result_path(result_file)
                file_status = self._stat_result(relative_path)
                identities.append(
                    ResultFileIdentity(
                        relative_path=relative_path,
                        device=file_status.st_dev,
                        inode=file_status.st_ino,
                    )
                )
            self._identities = tuple(identities)
        except Exception:
            self.close()
            raise

    def _open_anchor(self) -> None:
        build_directory = self.package_directory.parent
        workspace_directory = build_directory.parent
        try:
            self._workspace_fd = os.open(
                workspace_directory,
                DIRECTORY_OPEN_FLAGS,
            )
            self._build_fd = os.open(
                build_directory.name,
                DIRECTORY_OPEN_FLAGS,
                dir_fd=self._workspace_fd,
            )
            self._package_fd = os.open(
                self.package_directory.name,
                DIRECTORY_OPEN_FLAGS,
                dir_fd=self._build_fd,
            )
        except OSError as error:
            raise ValueError(
                "selected package changed after evidence collection: "
                f"{self.package_directory}"
            ) from error

        build_status = os.fstat(self._build_fd)
        package_status = os.fstat(self._package_fd)
        self._build_identity = (build_status.st_dev, build_status.st_ino)
        self._package_identity = (
            package_status.st_dev,
            package_status.st_ino,
        )

    def _assert_anchor_attached(self) -> None:
        if self._build_identity is None or self._package_identity is None:
            raise ValueError("result deletion plan is closed")
        try:
            build_status = os.stat(
                self.package_directory.parent.name,
                dir_fd=self._workspace_fd,
                follow_symlinks=False,
            )
            package_status = os.stat(
                self.package_directory.name,
                dir_fd=self._build_fd,
                follow_symlinks=False,
            )
        except OSError as error:
            raise ValueError(
                "selected package changed after evidence collection: "
                f"{self.package_directory}"
            ) from error
        if (
            not stat.S_ISDIR(build_status.st_mode)
            or not stat.S_ISDIR(package_status.st_mode)
            or (build_status.st_dev, build_status.st_ino)
            != self._build_identity
            or (package_status.st_dev, package_status.st_ino)
            != self._package_identity
        ):
            raise ValueError(
                "selected package changed after evidence collection: "
                f"{self.package_directory}"
            )

    def _relative_result_path(self, result_file: Path) -> Path:
        try:
            relative_path = result_file.relative_to(self.package_directory)
        except ValueError as error:
            raise ValueError(
                f"result path is outside its selected package: {result_file}"
            ) from error
        if not relative_path.parts or any(
            part in (".", "..") for part in relative_path.parts
        ):
            raise ValueError(f"unsafe result path: {result_file}")
        return relative_path

    def _open_parent(self, relative_path: Path) -> int:
        current_fd = os.dup(self._package_fd)
        try:
            for part in relative_path.parent.parts:
                next_fd = os.open(
                    part,
                    DIRECTORY_OPEN_FLAGS,
                    dir_fd=current_fd,
                )
                os.close(current_fd)
                current_fd = next_fd
            return current_fd
        except OSError as error:
            os.close(current_fd)
            raise ValueError(
                "result path changed after evidence collection: "
                f"{self.package_directory / relative_path}"
            ) from error

    def _stat_result(self, relative_path: Path) -> os.stat_result:
        self._assert_anchor_attached()
        parent_fd = self._open_parent(relative_path)
        try:
            file_status = os.stat(
                relative_path.name,
                dir_fd=parent_fd,
                follow_symlinks=False,
            )
        except OSError as error:
            raise ValueError(
                "result path changed after evidence collection: "
                f"{self.package_directory / relative_path}"
            ) from error
        finally:
            os.close(parent_fd)
        if not stat.S_ISREG(file_status.st_mode):
            raise ValueError(
                "result path changed after evidence collection: "
                f"{self.package_directory / relative_path}"
            )
        return file_status

    def validate_all(self) -> None:
        """Recheck every identity without mutating the filesystem."""

        for identity in self._identities:
            file_status = self._stat_result(identity.relative_path)
            if (file_status.st_dev, file_status.st_ino) != (
                identity.device,
                identity.inode,
            ):
                raise ValueError(
                    "result path changed after evidence collection: "
                    f"{self.package_directory / identity.relative_path}"
                )

    def unlink_all(self) -> None:
        """Revalidate and unlink names relative to no-follow directory FDs."""

        self.validate_all()
        for identity in self._identities:
            self._assert_anchor_attached()
            parent_fd = self._open_parent(identity.relative_path)
            try:
                file_status = os.stat(
                    identity.relative_path.name,
                    dir_fd=parent_fd,
                    follow_symlinks=False,
                )
                if (
                    not stat.S_ISREG(file_status.st_mode)
                    or (file_status.st_dev, file_status.st_ino)
                    != (identity.device, identity.inode)
                ):
                    raise ValueError(
                        "result path changed after evidence collection: "
                        f"{self.package_directory / identity.relative_path}"
                    )
                os.unlink(identity.relative_path.name, dir_fd=parent_fd)
            except OSError as error:
                raise ValueError(
                    "result path changed after evidence collection: "
                    f"{self.package_directory / identity.relative_path}"
                ) from error
            finally:
                os.close(parent_fd)

    def close(self) -> None:
        for descriptor_name in (
            "_package_fd",
            "_build_fd",
            "_workspace_fd",
        ):
            descriptor = getattr(self, descriptor_name)
            if descriptor >= 0:
                os.close(descriptor)
                setattr(self, descriptor_name, -1)
        self._build_identity = None
        self._package_identity = None


@contextmanager
def open_result_deletion_plan(
    package_directory: Path,
    result_files: tuple[Path, ...],
) -> Iterator[ResultDeletionPlan]:
    """Hold an anchored result-deletion plan for one selected package."""

    plan = ResultDeletionPlan(package_directory, result_files)
    try:
        yield plan
    finally:
        plan.close()


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
