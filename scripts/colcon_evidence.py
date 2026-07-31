"""Path-safety primitives for deterministic colcon verification evidence."""

import os
import re
import shutil
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
FILE_OPEN_FLAGS = (
    os.O_RDONLY | os.O_NOFOLLOW | getattr(os, "O_CLOEXEC", 0)
)


@dataclass(frozen=True)
class ResultFileIdentity:
    """Package-relative identity captured before any clear mutation."""

    relative_path: Path
    device: int
    inode: int
    size: int
    modified_ns: int
    changed_ns: int


@dataclass(frozen=True)
class ResultDirectoryIdentity:
    """Directory membership identity for a closed result snapshot."""

    relative_path: Path
    device: int
    inode: int
    modified_ns: int
    changed_ns: int


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


def _validate_staged_result_input(
    path: Path,
    package_directory: Path,
) -> Path:
    """Return a regular input owned by the private staging directory.

    Original build inputs use anchored file descriptors instead.  This helper
    is intentionally limited to the private, immutable sandbox snapshot.
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


def ctest_result_path(tag_file: Path, package_directory: Path) -> Path:
    """Resolve one validated CTest TAG to its mandatory local result."""

    _validate_staged_result_input(tag_file, package_directory)
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
    return _validate_staged_result_input(latest_xml, package_directory)


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
                        size=file_status.st_size,
                        modified_ns=file_status.st_mtime_ns,
                        changed_ns=file_status.st_ctime_ns,
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
        return self._open_relative_directory(relative_path.parent)

    def _open_relative_directory(self, relative_directory: Path) -> int:
        current_fd = os.dup(self._package_fd)
        try:
            for part in relative_directory.parts:
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
                f"{self.package_directory / relative_directory}"
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


class ResultSnapshotPlan(ResultDeletionPlan):
    """Stage a stable, no-follow snapshot of one package's result inputs."""

    def __init__(self, package_directory: Path) -> None:
        super().__init__(package_directory, ())
        try:
            (
                self._snapshot_inputs,
                self._snapshot_directories,
            ) = self._discover_snapshot_inputs()
        except Exception:
            self.close()
            raise

    @staticmethod
    def _snapshot_identity(
        relative_path: Path,
        file_status: os.stat_result,
    ) -> ResultFileIdentity:
        return ResultFileIdentity(
            relative_path=relative_path,
            device=file_status.st_dev,
            inode=file_status.st_ino,
            size=file_status.st_size,
            modified_ns=file_status.st_mtime_ns,
            changed_ns=file_status.st_ctime_ns,
        )

    @staticmethod
    def _same_snapshot(
        identity: ResultFileIdentity,
        file_status: os.stat_result,
    ) -> bool:
        return (
            stat.S_ISREG(file_status.st_mode)
            and (
                file_status.st_dev,
                file_status.st_ino,
                file_status.st_size,
                file_status.st_mtime_ns,
                file_status.st_ctime_ns,
            )
            == (
                identity.device,
                identity.inode,
                identity.size,
                identity.modified_ns,
                identity.changed_ns,
            )
        )

    def _discover_snapshot_inputs(
        self,
    ) -> tuple[
        tuple[ResultFileIdentity, ...],
        tuple[ResultDirectoryIdentity, ...],
    ]:
        self._assert_anchor_attached()
        identities = []
        directory_identities = []
        try:
            walker = os.fwalk(
                ".",
                topdown=True,
                follow_symlinks=False,
                dir_fd=self._package_fd,
            )
            for directory, directory_names, file_names, directory_fd in walker:
                relative_directory = (
                    Path() if directory == "." else Path(directory)
                )
                directory_status = os.fstat(directory_fd)
                directory_identities.append(
                    ResultDirectoryIdentity(
                        relative_path=relative_directory,
                        device=directory_status.st_dev,
                        inode=directory_status.st_ino,
                        modified_ns=directory_status.st_mtime_ns,
                        changed_ns=directory_status.st_ctime_ns,
                    )
                )

                traversable_directories = []
                for directory_name in sorted(directory_names):
                    child_path = (
                        self.package_directory
                        / relative_directory
                        / directory_name
                    )
                    child_status = os.stat(
                        directory_name,
                        dir_fd=directory_fd,
                        follow_symlinks=False,
                    )
                    if stat.S_ISLNK(child_status.st_mode):
                        if (
                            directory_name == "Testing"
                            or relative_directory.name == "Testing"
                        ):
                            raise ValueError(
                                "result path contains symbolic link: "
                                f"{child_path}"
                            )
                        continue
                    if not stat.S_ISDIR(child_status.st_mode):
                        raise ValueError(
                            "result directory is not a directory: "
                            f"{child_path}"
                        )
                    traversable_directories.append(directory_name)
                directory_names[:] = traversable_directories

                for file_name in sorted(file_names):
                    is_ctest_tag = (
                        file_name == "TAG"
                        and relative_directory.name == "Testing"
                    )
                    is_xml = file_name.endswith(".xml")
                    if not is_ctest_tag and not is_xml:
                        continue

                    file_status = os.stat(
                        file_name,
                        dir_fd=directory_fd,
                        follow_symlinks=False,
                    )
                    if stat.S_ISLNK(file_status.st_mode):
                        if is_xml and file_name == "package.xml":
                            continue
                        raise ValueError(
                            "result path contains symbolic link: "
                            f"{self.package_directory / relative_directory / file_name}"
                        )
                    if not stat.S_ISREG(file_status.st_mode):
                        raise ValueError(
                            "result path is not a regular file: "
                            f"{self.package_directory / relative_directory / file_name}"
                        )
                    identities.append(
                        self._snapshot_identity(
                            relative_directory / file_name,
                            file_status,
                        )
                    )
        except OSError as error:
            raise ValueError(
                "selected package changed after evidence discovery: "
                f"{self.package_directory}"
            ) from error

        self._assert_anchor_attached()
        return tuple(identities), tuple(directory_identities)

    def _validate_snapshot_directories(self) -> None:
        for identity in self._snapshot_directories:
            directory_fd = self._open_relative_directory(
                identity.relative_path
            )
            try:
                directory_status = os.fstat(directory_fd)
            finally:
                os.close(directory_fd)
            if (
                not stat.S_ISDIR(directory_status.st_mode)
                or (
                    directory_status.st_dev,
                    directory_status.st_ino,
                    directory_status.st_mtime_ns,
                    directory_status.st_ctime_ns,
                )
                != (
                    identity.device,
                    identity.inode,
                    identity.modified_ns,
                    identity.changed_ns,
                )
            ):
                raise ValueError(
                    "result directory changed after evidence discovery: "
                    f"{self.package_directory / identity.relative_path}"
                )

    def stage(self, sandbox_package: Path) -> dict[Path, Path]:
        """Copy the discovered manifest through anchored no-follow FDs."""

        self._validate_snapshot_directories()
        source_by_relative_path: dict[Path, Path] = {}
        for identity in self._snapshot_inputs:
            self._assert_anchor_attached()
            parent_fd = self._open_parent(identity.relative_path)
            source_fd = -1
            try:
                before_status = os.stat(
                    identity.relative_path.name,
                    dir_fd=parent_fd,
                    follow_symlinks=False,
                )
                if not self._same_snapshot(identity, before_status):
                    raise ValueError(
                        "result path changed after evidence discovery: "
                        f"{self.package_directory / identity.relative_path}"
                    )
                source_fd = os.open(
                    identity.relative_path.name,
                    FILE_OPEN_FLAGS,
                    dir_fd=parent_fd,
                )
                opened_status = os.fstat(source_fd)
                if not self._same_snapshot(identity, opened_status):
                    raise ValueError(
                        "result path changed after evidence discovery: "
                        f"{self.package_directory / identity.relative_path}"
                    )

                destination_path = sandbox_package / identity.relative_path
                destination_path.parent.mkdir(parents=True, exist_ok=True)
                source = os.fdopen(source_fd, "rb", closefd=True)
                source_fd = -1
                with source:
                    with destination_path.open("xb") as destination:
                        shutil.copyfileobj(source, destination)
                        after_status = os.fstat(source.fileno())
                if not self._same_snapshot(identity, after_status):
                    raise ValueError(
                        "result path changed after evidence discovery: "
                        f"{self.package_directory / identity.relative_path}"
                    )
                destination_status = destination_path.lstat()
                if not stat.S_ISREG(destination_status.st_mode):
                    raise ValueError(
                        f"sandbox result is not a regular file: {destination_path}"
                    )
                source_by_relative_path[identity.relative_path] = (
                    self.package_directory / identity.relative_path
                )
            except OSError as error:
                raise ValueError(
                    "result path changed after evidence discovery: "
                    f"{self.package_directory / identity.relative_path}"
                ) from error
            finally:
                if source_fd >= 0:
                    os.close(source_fd)
                os.close(parent_fd)

        self._validate_snapshot_directories()
        self._assert_anchor_attached()
        for tag_file in sorted(sandbox_package.glob("**/Testing/TAG")):
            ctest_result_path(tag_file, sandbox_package)
        return source_by_relative_path


@contextmanager
def open_result_snapshot(
    package_directory: Path,
) -> Iterator[ResultSnapshotPlan]:
    """Open one package evidence snapshot anchored below its build base."""

    plan = ResultSnapshotPlan(package_directory)
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
