#!/usr/bin/env python3
"""Provision and verify the locked local Qwen/llama.cpp artifact bundle.

This module is the only implementation of the LLM artifact workflow.  The
shell entry points are deliberately thin wrappers around its subcommands.
The production path uses only the Python standard library and never performs
runtime model discovery or download.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping, Sequence
import ctypes
from dataclasses import dataclass
import datetime as datetime_module
import errno
import hashlib
from http.client import IncompleteRead
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import secrets
import shutil
import signal
import stat
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
from types import TracebackType
from typing import Any, Protocol
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin, urlparse
from urllib.request import (
    HTTPRedirectHandler,
    ProxyHandler,
    Request,
    build_opener,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
LOCK_PATH = REPOSITORY_ROOT / "models" / "locks" / "voice_nav_llm_v1.lock.json"
NOTICE_PATH = REPOSITORY_ROOT / "docs" / "process" / "third-party-llm-notices.md"
DEFAULT_ARTIFACT_ROOT = REPOSITORY_ROOT / ".deps" / "llm"
LOCK_DIGEST_PATTERN = re.compile(r"\A[0-9a-f]{64}\Z")
SHA256_PATTERN = LOCK_DIGEST_PATTERN
REVISION_PATTERN = re.compile(r"\A[0-9a-f]{40}\Z")
CHUNK_SIZE = 1024 * 1024
DOWNLOAD_RETRIES = 3
REAL_READINESS_SECONDS = 30.0
REAL_REQUEST_SECONDS = 10.0
PROCESS_TERM_SECONDS = 3.0
REAL_LOG_MAX_BYTES = 1024 * 1024
RENAME_NOREPLACE = 1
AT_FDCWD = -100

LOCKED_BUILD_FLAGS: tuple[tuple[str, str], ...] = (
    ("BUILD_SHARED_LIBS", "OFF"),
    ("LLAMA_BUILD_SERVER", "ON"),
    ("LLAMA_BUILD_TESTS", "OFF"),
    ("LLAMA_BUILD_EXAMPLES", "OFF"),
    ("GGML_NATIVE", "OFF"),
    ("GGML_OPENMP", "ON"),
)


APPROVED_DOCUMENT: dict[str, Any] = {
    "schema_version": 1,
    "model": {
        "repo": "Qwen/Qwen3-0.6B-GGUF",
        "revision": "23749fefcc72300e3a2ad315e1317431b06b590a",
        "file": "Qwen3-0.6B-Q8_0.gguf",
        "size": 639446688,
        "sha256": "9465e63a22add5354d9bb4b99e90117043c7124007664907259bd16d043bb031",
        "license": "Apache-2.0",
        "download_url": "https://huggingface.co/Qwen/Qwen3-0.6B-GGUF/resolve/23749fefcc72300e3a2ad315e1317431b06b590a/Qwen3-0.6B-Q8_0.gguf?download=true",
        "source_url": "https://huggingface.co/Qwen/Qwen3-0.6B-GGUF/tree/23749fefcc72300e3a2ad315e1317431b06b590a",
        "license_url": "https://huggingface.co/Qwen/Qwen3-0.6B-GGUF/tree/23749fefcc72300e3a2ad315e1317431b06b590a",
    },
    "llama_cpp": {
        "tag": "b10276",
        "commit": "6ea215d171fd31df943bf1ac8227129f2b963160",
        "source_url": "https://codeload.github.com/ggml-org/llama.cpp/tar.gz/6ea215d171fd31df943bf1ac8227129f2b963160",
        "source_size": 36570950,
        "source_sha256": "aa90f46e3744796af244af17c2b448589669bb02ec0755ffa8516b07bbc73098",
        "license": "MIT",
        "license_url": "https://github.com/ggml-org/llama.cpp/blob/6ea215d171fd31df943bf1ac8227129f2b963160/LICENSE",
        "build": {
            "type": "Release",
            "flags": dict(LOCKED_BUILD_FLAGS),
        },
    },
    "runtime": {
        "host": "127.0.0.1",
        "port": 8080,
        "context": 2048,
        "max_output": 256,
        "parallel": 1,
        "stream": False,
        "non_thinking": "/no_think",
    },
}

TOP_LEVEL_KEYS = frozenset(APPROVED_DOCUMENT)
MODEL_KEYS = frozenset(APPROVED_DOCUMENT["model"])
LLAMA_KEYS = frozenset(APPROVED_DOCUMENT["llama_cpp"])
BUILD_KEYS = frozenset(APPROVED_DOCUMENT["llama_cpp"]["build"])
RUNTIME_KEYS = frozenset(APPROVED_DOCUMENT["runtime"])


class ArtifactError(RuntimeError):
    """The artifact contract or an artifact operation failed."""


class ManifestError(ArtifactError):
    """The lock manifest is malformed or has drifted from the approved lock."""


class OfflineGateUnavailable(ArtifactError):
    """The requested offline namespace could not be established."""


class RealGateError(ArtifactError):
    """A real gate failure with a retained bounded server log."""

    def __init__(self, message: str, log_path: Path) -> None:
        super().__init__(message)
        self.log_path = log_path


class ListenerOwnershipError(ArtifactError):
    """The locked listener is present but cannot be attributed to this gate."""


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ManifestError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _strict_json_bytes(data: bytes, label: str) -> Any:
    try:
        return json.loads(data.decode("utf-8"), object_pairs_hook=_reject_duplicate_keys)
    except (UnicodeDecodeError, json.JSONDecodeError, ManifestError) as error:
        raise ManifestError(f"invalid JSON in {label}") from error


def _require_exact_keys(document: Mapping[str, Any], expected: frozenset[str], label: str) -> None:
    actual = frozenset(document)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        details = []
        if missing:
            details.append("missing=" + ",".join(missing))
        if extra:
            details.append("unknown=" + ",".join(extra))
        raise ManifestError(f"{label} keys are not closed ({'; '.join(details)})")


def _require_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ManifestError(f"{label} must be a non-empty string")
    return value


def _require_positive_integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ManifestError(f"{label} must be a positive integer")
    return value


def _require_sha(value: Any, label: str) -> str:
    value = _require_string(value, label)
    if SHA256_PATTERN.fullmatch(value) is None:
        raise ManifestError(f"{label} must be lowercase 64-hex SHA-256")
    return value


def _require_revision(value: Any, label: str) -> str:
    value = _require_string(value, label)
    if REVISION_PATTERN.fullmatch(value) is None:
        raise ManifestError(f"{label} must be a lowercase 40-hex revision")
    return value


def _require_https_url(value: Any, label: str) -> str:
    value = _require_string(value, label)
    parsed = urlparse(value)
    if parsed.scheme != "https" or not parsed.hostname:
        raise ManifestError(f"{label} must be an HTTPS URL")
    if parsed.username is not None or parsed.password is not None:
        raise ManifestError(f"{label} must not contain credentials")
    path_parts = {part.lower() for part in parsed.path.split("/") if part}
    if path_parts.intersection({"latest", "master", "main", "head"}):
        raise ManifestError(f"{label} must not use a floating revision")
    return value


@dataclass(frozen=True)
class LockManifest:
    """Validated immutable view of one lock document."""

    document: Mapping[str, Any]

    @property
    def model(self) -> Mapping[str, Any]:
        return self.document["model"]

    @property
    def llama_cpp(self) -> Mapping[str, Any]:
        return self.document["llama_cpp"]

    @property
    def runtime(self) -> Mapping[str, Any]:
        return self.document["runtime"]

    @property
    def model_file(self) -> str:
        return self.model["file"]

    @property
    def model_size(self) -> int:
        return self.model["size"]

    @property
    def model_sha256(self) -> str:
        return self.model["sha256"]

    @property
    def source_size(self) -> int:
        return self.llama_cpp["source_size"]

    @property
    def source_sha256(self) -> str:
        return self.llama_cpp["source_sha256"]

    @property
    def build_flags(self) -> Mapping[str, str]:
        return self.llama_cpp["build"]["flags"]


def validate_manifest_document(document: Any, *, approved: bool = False) -> LockManifest:
    """Validate a lock-shaped mapping and optionally require the approved values."""

    if not isinstance(document, dict):
        raise ManifestError("lock manifest must be a JSON object")
    _require_exact_keys(document, TOP_LEVEL_KEYS, "manifest")
    if document["schema_version"] != 1 or isinstance(document["schema_version"], bool):
        raise ManifestError("schema_version must be exactly 1")

    model = document["model"]
    if not isinstance(model, dict):
        raise ManifestError("model must be an object")
    _require_exact_keys(model, MODEL_KEYS, "model")
    _require_string(model["repo"], "model.repo")
    _require_revision(model["revision"], "model.revision")
    model_file = _require_string(model["file"], "model.file")
    if (
        model_file in {".", ".."}
        or model_file != Path(model_file).name
        or "/" in model_file
        or "\\" in model_file
    ):
        raise ManifestError("model.file must be a basename")
    _require_positive_integer(model["size"], "model.size")
    _require_sha(model["sha256"], "model.sha256")
    _require_string(model["license"], "model.license")
    for key in ("download_url", "source_url", "license_url"):
        _require_https_url(model[key], f"model.{key}")

    llama_cpp = document["llama_cpp"]
    if not isinstance(llama_cpp, dict):
        raise ManifestError("llama_cpp must be an object")
    _require_exact_keys(llama_cpp, LLAMA_KEYS, "llama_cpp")
    _require_string(llama_cpp["tag"], "llama_cpp.tag")
    _require_revision(llama_cpp["commit"], "llama_cpp.commit")
    _require_https_url(llama_cpp["source_url"], "llama_cpp.source_url")
    _require_positive_integer(llama_cpp["source_size"], "llama_cpp.source_size")
    _require_sha(llama_cpp["source_sha256"], "llama_cpp.source_sha256")
    _require_string(llama_cpp["license"], "llama_cpp.license")
    _require_https_url(llama_cpp["license_url"], "llama_cpp.license_url")
    build = llama_cpp["build"]
    if not isinstance(build, dict):
        raise ManifestError("llama_cpp.build must be an object")
    _require_exact_keys(build, BUILD_KEYS, "llama_cpp.build")
    if build["type"] != "Release":
        raise ManifestError("llama_cpp.build.type must be Release")
    flags = build["flags"]
    if not isinstance(flags, dict):
        raise ManifestError("llama_cpp.build.flags must be an object")
    expected_flags = frozenset(name for name, _ in LOCKED_BUILD_FLAGS)
    _require_exact_keys(flags, expected_flags, "llama_cpp.build.flags")
    for name, value in LOCKED_BUILD_FLAGS:
        if flags[name] not in {"ON", "OFF"}:
            raise ManifestError(f"llama_cpp.build.flags.{name} must be ON or OFF")

    runtime = document["runtime"]
    if not isinstance(runtime, dict):
        raise ManifestError("runtime must be an object")
    _require_exact_keys(runtime, RUNTIME_KEYS, "runtime")
    if runtime["host"] != "127.0.0.1":
        raise ManifestError("runtime.host must be the loopback literal 127.0.0.1")
    port = _require_positive_integer(runtime["port"], "runtime.port")
    if port > 65535:
        raise ManifestError("runtime.port is outside the TCP port range")
    if _require_positive_integer(runtime["context"], "runtime.context") != 2048:
        raise ManifestError("runtime.context must be 2048")
    if _require_positive_integer(runtime["max_output"], "runtime.max_output") != 256:
        raise ManifestError("runtime.max_output must be 256")
    if _require_positive_integer(runtime["parallel"], "runtime.parallel") != 1:
        raise ManifestError("runtime.parallel must be 1")
    if not isinstance(runtime["stream"], bool) or runtime["stream"] is not False:
        raise ManifestError("runtime.stream must be false")
    if runtime["non_thinking"] != "/no_think":
        raise ManifestError("runtime.non_thinking must be /no_think")

    if approved and document != APPROVED_DOCUMENT:
        raise ManifestError("lock manifest does not match the approved Qwen/llama.cpp lock")
    return LockManifest(document=document)


def load_lock_manifest(path: Path = LOCK_PATH, *, approved: bool = True) -> LockManifest:
    """Read and validate a lock manifest without any network access."""

    try:
        data = path.read_bytes()
    except OSError as error:
        raise ManifestError(f"cannot read lock manifest: {path.name}") from error
    document = _strict_json_bytes(data, path.name)
    return validate_manifest_document(document, approved=approved)


def lock_sha256(path: Path = LOCK_PATH) -> str:
    """Return the digest of the exact lock bytes used for a bundle directory."""

    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError as error:
        raise ManifestError("cannot hash lock manifest") from error


def validate_notice_consistency(
    manifest: LockManifest,
    notice_path: Path = NOTICE_PATH,
) -> None:
    """Require the license notice to repeat every approved lock identity."""

    try:
        notice = notice_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise ArtifactError("cannot read third-party LLM notice") from error
    markers = [
        manifest.model["repo"],
        manifest.model["revision"],
        manifest.model["file"],
        str(manifest.model["size"]),
        manifest.model["sha256"],
        manifest.model["license"],
        manifest.model["download_url"],
        manifest.model["source_url"],
        manifest.model["license_url"],
        manifest.llama_cpp["tag"],
        manifest.llama_cpp["commit"],
        str(manifest.llama_cpp["source_size"]),
        manifest.llama_cpp["source_sha256"],
        manifest.llama_cpp["license"],
        manifest.llama_cpp["source_url"],
        manifest.llama_cpp["license_url"],
        manifest.llama_cpp["build"]["type"],
        *(
            f"{name}={value}"
            for name, value in LOCKED_BUILD_FLAGS
        ),
        manifest.runtime["host"],
        str(manifest.runtime["port"]),
        str(manifest.runtime["context"]),
        str(manifest.runtime["max_output"]),
        str(manifest.runtime["parallel"]),
        f"stream={str(manifest.runtime['stream']).lower()}",
        manifest.runtime["non_thinking"],
    ]
    missing = [marker for marker in markers if marker not in notice]
    if missing:
        raise ArtifactError("third-party LLM notice is inconsistent with the lock")


def _tracked_repository_paths(repo_root: Path) -> list[str]:
    try:
        completed = subprocess.run(
            [
                "git",
                "-c",
                "safe.directory=*",
                "-C",
                str(repo_root),
                "ls-files",
                "-z",
            ],
            check=False,
            capture_output=True,
        )
    except OSError as error:
        raise ArtifactError("cannot inspect tracked repository files") from error
    if completed.returncode != 0:
        raise ArtifactError("cannot inspect tracked repository files")
    return [item.decode("utf-8") for item in completed.stdout.split(b"\0") if item]


def forbidden_tracked_artifacts(paths: Sequence[str]) -> list[str]:
    """Return tracked model/server/source/build/log artifacts that must be absent."""

    forbidden: list[str] = []
    for raw_path in paths:
        path = raw_path.replace("\\", "/")
        lower = path.lower()
        name = lower.rsplit("/", 1)[-1]
        parts = set(part for part in lower.split("/") if part)
        if (
            name.endswith(".gguf")
            or name in {"llama-server", "llama-server.exe"}
            or any(part.startswith("llama.cpp") for part in parts)
            or name.endswith(".tar.gz") and "llama" in name
            or name.endswith(".log")
            or any(part in {"build", "install", "log", "logs", "source"} for part in parts)
            or ".deps" in parts
            or "weights" in parts and "models" in parts
        ):
            forbidden.append(raw_path)
    return sorted(forbidden)


def verify_repository_artifact_boundary(
    repo_root: Path,
    *,
    tracked_paths: Sequence[str] | None = None,
) -> None:
    """Fail if Git tracks a model, server, source tree, build, or runtime log."""

    paths = list(tracked_paths) if tracked_paths is not None else _tracked_repository_paths(repo_root)
    forbidden = forbidden_tracked_artifacts(paths)
    if forbidden:
        raise ArtifactError("forbidden tracked LLM artifacts: " + ", ".join(forbidden))


def _resolved_without_symlinks(path: Path) -> Path:
    return path.resolve(strict=False)


def _is_mnt_path(path: Path) -> bool:
    resolved = _resolved_without_symlinks(path)
    return len(resolved.parts) >= 2 and resolved.parts[0] == "/" and resolved.parts[1] == "mnt"


def _check_owned_private_directory(path: Path, *, create: bool) -> Path:
    if not path.is_absolute():
        raise ArtifactError("artifact root must be an absolute Linux path")
    resolved = _resolved_without_symlinks(path)
    if resolved == Path("/"):
        raise ArtifactError("artifact root may not be the filesystem root")
    if _is_mnt_path(path):
        raise ArtifactError("artifact root under /mnt is forbidden; choose Linux filesystem storage")
    if path.exists():
        try:
            info = path.lstat()
        except OSError as error:
            raise ArtifactError("cannot inspect artifact root") from error
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise ArtifactError("artifact root must be a real directory")
    elif create:
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.mkdir(mode=0o700)
        except FileExistsError:
            pass
        except OSError as error:
            raise ArtifactError("cannot create artifact root") from error
        try:
            path.chmod(0o700)
        except OSError as error:
            raise ArtifactError("cannot set artifact root permissions") from error
    else:
        raise ArtifactError("artifact root does not exist")

    try:
        info = path.lstat()
    except OSError as error:
        raise ArtifactError("cannot inspect artifact root") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise ArtifactError("artifact root must be a real directory")
    if hasattr(os, "geteuid") and info.st_uid != os.geteuid():
        raise ArtifactError("artifact root must be owned by the current user")
    if os.name == "posix" and info.st_mode & 0o022:
        raise ArtifactError("artifact root must not be group/world writable")
    return path


def validate_artifact_root(path: Path, *, create: bool = False) -> Path:
    """Validate the root safety boundary used by provisioning and real verify."""

    return _check_owned_private_directory(Path(path), create=create)


def _ensure_private_directory(path: Path) -> None:
    try:
        if path.exists():
            info = path.lstat()
            if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
                raise ArtifactError(f"artifact directory is not a real directory: {path.name}")
        else:
            path.mkdir(mode=0o700)
        path.chmod(0o700)
    except ArtifactError:
        raise
    except OSError as error:
        raise ArtifactError(f"cannot create private artifact directory: {path.name}") from error


def _require_private_directory(path: Path, label: str) -> None:
    """Check a private directory without creating or changing it."""

    try:
        info = path.lstat()
    except FileNotFoundError as error:
        raise ArtifactError(f"{label} is missing") from error
    except OSError as error:
        raise ArtifactError(f"cannot inspect {label}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise ArtifactError(f"{label} is not a real directory")
    if os.name == "posix" and info.st_mode & 0o077:
        raise ArtifactError(f"{label} is not private")


def _ensure_regular_non_symlink(path: Path, label: str) -> os.stat_result:
    try:
        info = path.lstat()
    except OSError as error:
        raise ArtifactError(f"{label} is missing") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise ArtifactError(f"{label} must be a regular non-symlink file")
    return info


def file_sha256(path: Path) -> tuple[int, str]:
    """Return exact file size and SHA-256 for a regular file."""

    _ensure_regular_non_symlink(path, path.name)
    size = 0
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            while True:
                chunk = stream.read(CHUNK_SIZE)
                if not chunk:
                    break
                size += len(chunk)
                digest.update(chunk)
    except OSError as error:
        raise ArtifactError(f"cannot read artifact file: {path.name}") from error
    return size, digest.hexdigest()


def verify_file_identity(path: Path, expected_size: int, expected_sha256: str, label: str) -> None:
    actual_size, actual_sha256 = file_sha256(path)
    if actual_size != expected_size:
        raise ArtifactError(f"{label} size mismatch")
    if actual_sha256 != expected_sha256:
        raise ArtifactError(f"{label} SHA-256 mismatch")


class _HttpsRedirectHandler(HTTPRedirectHandler):
    def redirect_request(self, req: Request, fp: Any, code: int, msg: str, headers: Any, newurl: str) -> Request | None:
        target = urljoin(req.full_url, newurl)
        _require_https_url(target, "redirect URL")
        redirected = super().redirect_request(req, fp, code, msg, headers, target)
        if redirected is not None and req.has_header("Range"):
            redirected.add_unredirected_header("Range", req.get_header("Range"))
        return redirected


def _default_https_opener() -> Any:
    return build_opener(
        ProxyHandler({}),
        _HttpsRedirectHandler(),
    )


def download_verified(
    url: str,
    destination: Path,
    expected_size: int,
    expected_sha256: str,
    *,
    opener: Any | None = None,
    retries: int = DOWNLOAD_RETRIES,
    timeout: float = 60.0,
) -> None:
    """Download an HTTPS artifact to a same-directory part and verify it."""

    _require_https_url(url, "download URL")
    if retries <= 0:
        raise ArtifactError("download retry budget must be positive")
    _ensure_private_directory(destination.parent)
    part = destination.with_name(destination.name + ".part")
    if part.exists() or os.path.lexists(str(part)):
        if part.is_dir() or part.is_symlink():
            raise ArtifactError("download part path is not disposable")
        part.unlink()
    opener = opener or _default_https_opener()
    last_error: Exception | None = None
    for attempt in range(retries):
        try:
            observed = part.stat().st_size if part.exists() else 0
            source: str | Request = url
            if observed:
                source = Request(url, headers={"Range": f"bytes={observed}-"})
            with opener.open(source, timeout=timeout) as response:
                status = getattr(response, "status", None)
                append = observed > 0 and status == 206
                if observed and not append:
                    _remove_path(part)
                    observed = 0
                mode = "ab" if append else "xb"
                with part.open(mode) as stream:
                    while True:
                        chunk = response.read(CHUNK_SIZE)
                        if not chunk:
                            break
                        observed += len(chunk)
                        if observed > expected_size:
                            raise ArtifactError("download exceeded the locked size")
                        stream.write(chunk)
                    stream.flush()
                    os.fsync(stream.fileno())
            verify_file_identity(part, expected_size, expected_sha256, "download")
            os.replace(part, destination)
            verify_file_identity(destination, expected_size, expected_sha256, "published download")
            return
        except ArtifactError as error:
            last_error = error
            retryable_identity_error = str(error) in {
                "download size mismatch",
                "download SHA-256 mismatch",
            }
            if retryable_identity_error and attempt + 1 < retries:
                if "SHA-256" in str(error) and part.exists():
                    _remove_path(part)
                continue
            if part.exists() or os.path.lexists(str(part)):
                _remove_path(part)
            raise
        except (OSError, HTTPError, URLError, IncompleteRead) as error:
            last_error = error
            if attempt + 1 < retries:
                continue
            if part.exists() or os.path.lexists(str(part)):
                _remove_path(part)
    raise ArtifactError("HTTPS download failed after bounded retries") from last_error


def safe_extract_tar(archive: Path, destination: Path) -> Path:
    """Extract one safe, single-top-level source tree without links or devices."""

    _ensure_regular_non_symlink(archive, "source archive")
    _ensure_private_directory(destination)
    try:
        with tarfile.open(archive, mode="r:*") as tar:
            members = tar.getmembers()
            if not members:
                raise ArtifactError("source archive is empty")
            normalized: list[tuple[tarfile.TarInfo, str, tuple[str, ...]]] = []
            names: set[str] = set()
            top_levels: set[str] = set()
            for member in members:
                name = member.name
                if not name or "\\" in name:
                    raise ArtifactError("source archive contains an unsafe path")
                pure = PurePosixPath(name)
                if pure.is_absolute() or any(part in {"", ".", ".."} for part in pure.parts):
                    raise ArtifactError("source archive contains an unsafe path")
                parts = pure.parts
                normalized_name = "/".join(parts)
                if normalized_name in names:
                    raise ArtifactError("source archive contains duplicate paths")
                names.add(normalized_name)
                if member.issym() or member.islnk() or member.isdev() or member.isfifo():
                    raise ArtifactError("source archive contains a link or special file")
                if not member.isdir() and not member.isreg():
                    raise ArtifactError("source archive contains an unsupported member")
                top_levels.add(parts[0])
                normalized.append((member, normalized_name, parts))
            if len(top_levels) != 1:
                raise ArtifactError("source archive must contain one top-level directory")

            for member, normalized_name, parts in normalized:
                target = destination.joinpath(*parts)
                if member.isdir():
                    if target.exists() and not target.is_dir():
                        raise ArtifactError("source archive path collision")
                    target.mkdir(parents=True, exist_ok=True, mode=0o700)
                    target.chmod(0o700)
                    continue
                target.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
                if target.exists() or os.path.lexists(str(target)):
                    raise ArtifactError("source archive path collision")
                extracted = tar.extractfile(member)
                if extracted is None:
                    raise ArtifactError("source archive regular file has no payload")
                with extracted, target.open("xb") as stream:
                    shutil.copyfileobj(extracted, stream)
                    stream.flush()
                    os.fsync(stream.fileno())
                mode = member.mode & 0o777
                target.chmod(mode or 0o600)
    except ArtifactError:
        raise
    except (OSError, tarfile.TarError) as error:
        raise ArtifactError("cannot safely extract source archive") from error
    return destination / next(iter(top_levels))


def _run_command(command: Sequence[str], *, timeout: float, label: str) -> subprocess.CompletedProcess[str]:
    try:
        completed = subprocess.run(
            list(command),
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ArtifactError(f"{label} did not complete") from error
    if completed.returncode != 0:
        raise ArtifactError(f"{label} failed")
    return completed


def _bounded_output(completed: subprocess.CompletedProcess[str], label: str) -> str:
    output = (completed.stdout or "") + (completed.stderr or "")
    output = output.strip()
    if not output:
        raise ArtifactError(f"{label} returned no version")
    return output[:4096]


@dataclass(frozen=True)
class BuildResult:
    """Build evidence copied into provenance without source-tree logs."""

    server_version: str
    compiler: str
    cmake: str


class Builder(Protocol):
    def build(
        self,
        source_root: Path,
        build_root: Path,
        output_path: Path,
    ) -> BuildResult: ...


class CMakeBuilder:
    """Build the locked upstream source with the exact six flags."""

    def __init__(self, manifest: LockManifest) -> None:
        self.manifest = manifest

    def build(self, source_root: Path, build_root: Path, output_path: Path) -> BuildResult:
        _ensure_private_directory(build_root)
        configure = [
            "cmake",
            "-S",
            str(source_root),
            "-B",
            str(build_root),
            "-DCMAKE_BUILD_TYPE=Release",
        ]
        configure.extend(
            f"-D{name}={self.manifest.build_flags[name]}"
            for name, _ in LOCKED_BUILD_FLAGS
        )
        _run_command(configure, timeout=300.0, label="cmake configure")
        _run_command(
            ["cmake", "--build", str(build_root), "--config", "Release", "--target", "llama-server"],
            timeout=1800.0,
            label="cmake build",
        )
        built_server = build_root / "bin" / "llama-server"
        _ensure_regular_non_symlink(built_server, "built llama-server")
        output_path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        shutil.copyfile(built_server, output_path)
        output_path.chmod(0o755)
        version = _run_command([str(output_path), "--version"], timeout=30.0, label="llama-server --version")
        compiler_command = os.environ.get("CXX", "c++")
        compiler = _bounded_output(
            _run_command([compiler_command, "--version"], timeout=30.0, label="compiler --version"),
            "compiler --version",
        )
        cmake = _bounded_output(
            _run_command(["cmake", "--version"], timeout=30.0, label="cmake --version"),
            "cmake --version",
        )
        return BuildResult(
            server_version=_bounded_output(version, "llama-server --version"),
            compiler=compiler,
            cmake=cmake,
        )


def _fsync_file(path: Path) -> None:
    try:
        with path.open("rb") as stream:
            os.fsync(stream.fileno())
    except OSError as error:
        raise ArtifactError("file fsync is not supported") from error


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY
    if hasattr(os, "O_DIRECTORY"):
        flags |= os.O_DIRECTORY
    try:
        descriptor = os.open(path, flags)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    except OSError as error:
        raise ArtifactError("directory fsync is not supported") from error


def fsync_tree(path: Path) -> None:
    """Fsync every regular file and directory in a staged bundle."""

    _ensure_private_directory(path)
    for current, directories, files in os.walk(path, followlinks=False):
        current_path = Path(current)
        for filename in files:
            candidate = current_path / filename
            _ensure_regular_non_symlink(candidate, candidate.name)
            _fsync_file(candidate)
        for directory in directories:
            candidate = current_path / directory
            if candidate.is_symlink():
                raise ArtifactError("staged bundle contains a symlink")
        _fsync_directory(current_path)


def _rename_noreplace(source: Path, destination: Path) -> None:
    if os.name != "posix":
        raise ArtifactError("renameat2 no-replace is required on Linux")
    source_info = source.lstat()
    destination_parent = destination.parent
    destination_parent_info = destination_parent.stat()
    if source_info.st_dev != destination_parent_info.st_dev:
        raise ArtifactError("staging and bundle directories must share a filesystem")
    if os.path.lexists(str(destination)):
        raise ArtifactError("final bundle already exists")
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        function = libc.renameat2
    except (AttributeError, OSError) as error:
        raise ArtifactError("renameat2 no-replace is unavailable") from error
    function.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    function.restype = ctypes.c_int
    result = function(
        AT_FDCWD,
        os.fsencode(str(source)),
        AT_FDCWD,
        os.fsencode(str(destination)),
        RENAME_NOREPLACE,
    )
    if result != 0:
        error_number = ctypes.get_errno()
        if error_number == errno.EEXIST:
            raise ArtifactError("final bundle already exists")
        if error_number in {errno.EINVAL, errno.ENOSYS, errno.EOPNOTSUPP}:
            raise ArtifactError("renameat2 no-replace is unavailable")
        raise ArtifactError("atomic bundle publication failed")


def atomic_publish(staging_bundle: Path, final_bundle: Path) -> None:
    """Publish one complete bundle with Linux renameat2(RENAME_NOREPLACE)."""

    _ensure_private_directory(staging_bundle)
    _ensure_private_directory(final_bundle.parent)
    _rename_noreplace(staging_bundle, final_bundle)


def _remove_path(path: Path) -> None:
    try:
        if os.path.lexists(str(path)):
            if path.is_symlink() or path.is_file():
                path.unlink()
            elif path.is_dir():
                shutil.rmtree(path)
            else:
                raise ArtifactError("cannot clean an unexpected staging path")
    except ArtifactError:
        raise
    except OSError as error:
        raise ArtifactError("cannot clean a staging path") from error


def _write_json(path: Path, document: Mapping[str, Any]) -> None:
    data = (json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8")
    try:
        with path.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
    except FileExistsError as error:
        raise ArtifactError("provenance already exists in staging") from error
    except OSError as error:
        raise ArtifactError("cannot write provenance") from error


def _copy_regular(source: Path, destination: Path, *, executable: bool = False) -> None:
    info = _ensure_regular_non_symlink(source, source.name)
    destination.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if destination.exists() or os.path.lexists(str(destination)):
        raise ArtifactError("staged output path already exists")
    try:
        shutil.copyfile(source, destination)
        mode = info.st_mode & 0o777
        if executable:
            mode |= stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH
        destination.chmod(mode or (0o755 if executable else 0o600))
    except OSError as error:
        raise ArtifactError("cannot stage artifact output") from error


def _server_version(server: Path) -> str:
    _ensure_regular_non_symlink(server, "llama-server")
    if not os.access(server, os.X_OK):
        raise ArtifactError("llama-server must be executable")
    return _bounded_output(
        _run_command([str(server), "--version"], timeout=30.0, label="llama-server --version"),
        "llama-server --version",
    )


def _provenance_document(
    manifest: LockManifest,
    digest: str,
    build_result: BuildResult,
    server_path: Path,
) -> dict[str, Any]:
    server_size, server_sha256 = file_sha256(server_path)
    return {
        "schema_version": 1,
        "lock_sha256": digest,
        "model": {
            "repo": manifest.model["repo"],
            "revision": manifest.model["revision"],
            "file": manifest.model["file"],
            "size": manifest.model["size"],
            "sha256": manifest.model["sha256"],
        },
        "llama_cpp": {
            "tag": manifest.llama_cpp["tag"],
            "commit": manifest.llama_cpp["commit"],
            "source_size": manifest.llama_cpp["source_size"],
            "source_sha256": manifest.llama_cpp["source_sha256"],
        },
        "build": {
            "type": manifest.llama_cpp["build"]["type"],
            "flags": dict(manifest.build_flags),
            "compiler": build_result.compiler,
            "cmake": build_result.cmake,
        },
        "server": {
            "version": build_result.server_version,
            "size": server_size,
            "sha256": server_sha256,
        },
        "created_at": datetime_module.datetime.now(datetime_module.timezone.utc).isoformat(),
    }


def _validate_provenance(
    provenance: Mapping[str, Any],
    manifest: LockManifest,
    digest: str,
) -> None:
    if provenance.get("schema_version") != 1 or provenance.get("lock_sha256") != digest:
        raise ArtifactError("provenance does not match the lock digest")
    model = provenance.get("model")
    if not isinstance(model, dict):
        raise ArtifactError("provenance model identity is missing")
    for key in ("repo", "revision", "file", "size", "sha256"):
        if model.get(key) != manifest.model[key]:
            raise ArtifactError("provenance model identity does not match the lock")
    llama_cpp = provenance.get("llama_cpp")
    if not isinstance(llama_cpp, dict):
        raise ArtifactError("provenance llama.cpp identity is missing")
    for key in ("tag", "commit", "source_size", "source_sha256"):
        if llama_cpp.get(key) != manifest.llama_cpp[key]:
            raise ArtifactError("provenance source identity does not match the lock")
    build = provenance.get("build")
    if not isinstance(build, dict):
        raise ArtifactError("provenance build evidence is missing")
    if build.get("type") != "Release" or build.get("flags") != dict(manifest.build_flags):
        raise ArtifactError("provenance build flags do not match the lock")
    for key in ("compiler", "cmake"):
        _require_string(build.get(key), f"provenance.build.{key}")
    server = provenance.get("server")
    if not isinstance(server, dict):
        raise ArtifactError("provenance server evidence is missing")
    _require_string(server.get("version"), "provenance.server.version")
    _require_positive_integer(server.get("size"), "provenance.server.size")
    _require_sha(server.get("sha256"), "provenance.server.sha256")
    _require_string(provenance.get("created_at"), "provenance.created_at")


def validate_bundle_directory(
    bundle: Path,
    manifest: LockManifest,
    digest: str,
    *,
    check_server: bool = False,
) -> None:
    """Validate a complete bundle, optionally executing server --version."""

    _require_private_directory(bundle, "bundle")
    expected_top = {"bin", "models", "provenance.json"}
    actual_top = {entry.name for entry in bundle.iterdir()}
    if actual_top != expected_top:
        raise ArtifactError("bundle is incomplete or contains unexpected files")
    binary_dir = bundle / "bin"
    model_dir = bundle / "models"
    _require_private_directory(binary_dir, "bundle binary directory")
    _require_private_directory(model_dir, "bundle model directory")
    binary = binary_dir / "llama-server"
    model = model_dir / manifest.model_file
    if {entry.name for entry in binary_dir.iterdir()} != {"llama-server"}:
        raise ArtifactError("bundle binary directory is not closed")
    if {entry.name for entry in model_dir.iterdir()} != {manifest.model_file}:
        raise ArtifactError("bundle model directory is not closed")
    _ensure_regular_non_symlink(binary, "bundle llama-server")
    if not os.access(binary, os.X_OK):
        raise ArtifactError("bundle llama-server is not executable")
    binary_size, binary_sha256 = file_sha256(binary)
    verify_file_identity(model, manifest.model_size, manifest.model_sha256, "bundle model")
    provenance_path = bundle / "provenance.json"
    _ensure_regular_non_symlink(provenance_path, "bundle provenance")
    provenance = _strict_json_bytes(provenance_path.read_bytes(), "provenance.json")
    if not isinstance(provenance, dict):
        raise ArtifactError("provenance must be an object")
    _validate_provenance(provenance, manifest, digest)
    if (
        provenance["server"]["size"] != binary_size
        or provenance["server"]["sha256"] != binary_sha256
    ):
        raise ArtifactError("provenance server identity does not match the bundle")
    if check_server and _server_version(binary) != provenance["server"]["version"]:
        raise ArtifactError("server version differs from provenance")


def verify_bundle(
    root: Path,
    manifest: LockManifest,
    digest: str,
    *,
    check_server: bool = False,
) -> Path:
    """Validate the lock-digest bundle under an already validated root."""

    bundle = root / "bundles" / digest
    validate_bundle_directory(bundle, manifest, digest, check_server=check_server)
    return bundle


class _NoopLock:
    def __enter__(self) -> _NoopLock:
        return self

    def __exit__(self, exc_type: type[BaseException] | None, exc: BaseException | None, traceback: TracebackType | None) -> None:
        return None


class BundleLock:
    """Advisory root-local flock for all provisioning in one artifact root."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.stream: Any | None = None

    def __enter__(self) -> BundleLock:
        if os.name != "posix":
            raise ArtifactError("flock is required on Linux")
        try:
            import fcntl

            lock_path = self.root / ".provision.lock"
            self.stream = lock_path.open("a+b")
            lock_path.chmod(0o600)
            fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX)
        except (ImportError, OSError) as error:
            raise ArtifactError("cannot acquire artifact provisioning lock") from error
        return self

    def __exit__(self, exc_type: type[BaseException] | None, exc: BaseException | None, traceback: TracebackType | None) -> None:
        if self.stream is None:
            return
        try:
            import fcntl

            fcntl.flock(self.stream.fileno(), fcntl.LOCK_UN)
        finally:
            self.stream.close()
            self.stream = None


class Downloader(Protocol):
    def __call__(self, url: str, destination: Path, expected_size: int, expected_sha256: str) -> None: ...


@dataclass(frozen=True)
class ProvisionResult:
    lock_sha256: str
    bundle: Path
    idempotent: bool


class Provisioner:
    """Build and atomically publish one complete lock-digest bundle."""

    def __init__(
        self,
        root: Path,
        manifest: LockManifest,
        digest: str,
        *,
        downloader: Downloader = download_verified,
        builder: Builder | None = None,
        publisher: Callable[[Path, Path], None] = atomic_publish,
        fsync: Callable[[Path], None] = fsync_tree,
        fsync_directory: Callable[[Path], None] = _fsync_directory,
        lock_factory: Callable[[Path], Any] = BundleLock,
        token_factory: Callable[[], str] = lambda: secrets.token_hex(16),
        check_existing_server: bool = True,
    ) -> None:
        if LOCK_DIGEST_PATTERN.fullmatch(digest) is None:
            raise ArtifactError("lock digest must be lowercase 64-hex")
        self.root = Path(root)
        self.manifest = manifest
        self.digest = digest
        self.downloader = downloader
        self.builder = builder or CMakeBuilder(manifest)
        self.publisher = publisher
        self.fsync = fsync
        self.fsync_directory = fsync_directory
        self.lock_factory = lock_factory
        self.token_factory = token_factory
        self.check_existing_server = check_existing_server

    def provision(self) -> ProvisionResult:
        root = validate_artifact_root(self.root, create=True)
        bundles = root / "bundles"
        _ensure_private_directory(bundles)
        final_bundle = bundles / self.digest
        with self.lock_factory(root):
            if os.path.lexists(str(final_bundle)):
                try:
                    validate_bundle_directory(
                        final_bundle,
                        self.manifest,
                        self.digest,
                        check_server=self.check_existing_server,
                    )
                except ArtifactError as error:
                    raise ArtifactError("existing bundle is invalid; refusing to replace") from error
                return ProvisionResult(self.digest, final_bundle, True)

            staging = root / f".staging-{self.digest}-{self.token_factory()}"
            if os.path.lexists(str(staging)):
                raise ArtifactError("staging name collision")
            work = staging / "work"
            bundle = staging / "bundle"
            published = False
            try:
                staging.mkdir(mode=0o700)
                work.mkdir(mode=0o700)
                bundle.mkdir(mode=0o700)
                source_archive = work / "llama.cpp.tar.gz"
                model_download = work / self.manifest.model_file
                self.downloader(
                    self.manifest.llama_cpp["source_url"],
                    source_archive,
                    self.manifest.source_size,
                    self.manifest.source_sha256,
                )
                verify_file_identity(
                    source_archive,
                    self.manifest.source_size,
                    self.manifest.source_sha256,
                    "source archive",
                )
                self.downloader(
                    self.manifest.model["download_url"],
                    model_download,
                    self.manifest.model_size,
                    self.manifest.model_sha256,
                )
                verify_file_identity(
                    model_download,
                    self.manifest.model_size,
                    self.manifest.model_sha256,
                    "model download",
                )
                source_root = safe_extract_tar(source_archive, work / "source")
                build_result = self.builder.build(source_root, work / "build", work / "server")
                if not isinstance(build_result, BuildResult):
                    raise ArtifactError("builder returned invalid provenance evidence")
                _require_string(build_result.server_version, "builder.server_version")
                _require_string(build_result.compiler, "builder.compiler")
                _require_string(build_result.cmake, "builder.cmake")
                _copy_regular(work / "server", bundle / "bin" / "llama-server", executable=True)
                _copy_regular(model_download, bundle / "models" / self.manifest.model_file)
                _write_json(
                    bundle / "provenance.json",
                    _provenance_document(
                        self.manifest,
                        self.digest,
                        build_result,
                        work / "server",
                    ),
                )
                validate_bundle_directory(bundle, self.manifest, self.digest, check_server=False)
                self.fsync(bundle)
                self.fsync_directory(staging)
                _remove_path(work)
                self.fsync(bundle)
                self.fsync_directory(staging)
                self.fsync_directory(bundles)
                self.publisher(bundle, final_bundle)
                published = True
                try:
                    self.fsync_directory(bundles)
                except ArtifactError as publish_error:
                    # The just-published path did not become durable.  It is
                    # ours (RENAME_NOREPLACE proved no prior bundle existed),
                    # so remove that incomplete publication and persist the
                    # cleanup before failing.
                    cleanup_errors: list[ArtifactError] = []
                    try:
                        _remove_path(final_bundle)
                    except ArtifactError as error:
                        cleanup_errors.append(error)
                    try:
                        self.fsync_directory(bundles)
                    except ArtifactError as error:
                        cleanup_errors.append(error)
                    if cleanup_errors:
                        details = "; ".join(str(error) for error in cleanup_errors)
                        raise ArtifactError(f"stranded publication: {details}") from cleanup_errors[0]
                    raise publish_error
                return ProvisionResult(self.digest, final_bundle, False)
            finally:
                if not published and (os.path.lexists(str(staging))):
                    try:
                        _remove_path(staging)
                    except ArtifactError:
                        pass
                elif os.path.lexists(str(staging)):
                    try:
                        _remove_path(staging)
                    except ArtifactError:
                        pass


def _safe_error(error: BaseException) -> str:
    message = str(error).strip().lower()
    message = re.sub(r"[^a-z0-9_-]+", "-", message)
    return message.strip("-")[:160] or "unknown"


def _command_output(command: Sequence[str], *, timeout: float) -> str:
    return _bounded_output(_run_command(command, timeout=timeout, label="runtime probe"), "runtime probe")


def _listener_records(port: int) -> list[tuple[str, tuple[int, ...]]]:
    if shutil.which("ss") is None:
        raise ArtifactError("ss is required for loopback proof")
    completed = _run_command(["ss", "-ltnp"], timeout=10.0, label="ss loopback proof")
    records: list[tuple[str, tuple[int, ...]]] = []
    for line in completed.stdout.splitlines()[1:]:
        fields = line.split()
        if len(fields) >= 4:
            local_address = fields[3]
            if local_address.endswith(f":{port}"):
                pids = tuple(int(value) for value in re.findall(r"\bpid=(\d+)\b", line))
                records.append((local_address, pids))
    return records


def _check_port_available(host: str, port: int) -> None:
    del host
    if _listener_records(port):
        raise ArtifactError("locked loopback port is already in use")


def _check_loopback_listener(host: str, port: int, process: subprocess.Popen[Any]) -> None:
    records = _listener_records(port)
    local_addresses = [address for address, _ in records]
    expected = f"{host}:{port}"
    if expected not in local_addresses:
        raise ArtifactError("llama-server is not listening on the locked loopback address")
    forbidden = {
        f"0.0.0.0:{port}",
        f"*:{port}",
        f"[::]:{port}",
        f":::{port}",
    }
    if any(address in forbidden for address in local_addresses):
        raise ListenerOwnershipError("llama-server is listening outside loopback")
    if any(address.endswith(f":{port}") and address != expected for address in local_addresses):
        raise ListenerOwnershipError("unexpected listener exists on the locked port")
    try:
        expected_group = os.getpgid(process.pid)
    except (AttributeError, OSError) as error:
        raise ListenerOwnershipError("cannot inspect launched llama-server process group") from error
    exact_records = [pids for address, pids in records if address == expected]
    if not exact_records or not all(exact_records):
        raise ListenerOwnershipError("listener process evidence is missing")
    listener_pids = {pid for pids in exact_records for pid in pids}
    try:
        listener_groups = {pid: os.getpgid(pid) for pid in sorted(listener_pids)}
    except (AttributeError, OSError) as error:
        raise ListenerOwnershipError("cannot inspect listener process group") from error
    if set(listener_groups.values()) != {expected_group}:
        raise ListenerOwnershipError("listener is not owned by launched llama-server process group")


def _wait_for_owned_listener(
    host: str,
    port: int,
    process: subprocess.Popen[Any],
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    last_error: ArtifactError | None = None
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise ArtifactError("llama-server exited before listener ownership proof")
        try:
            _check_loopback_listener(host, port, process)
            return
        except ListenerOwnershipError:
            raise
        except ArtifactError as error:
            last_error = error
            if "ss is required" in str(error):
                raise
            time.sleep(0.1)
    raise ArtifactError("llama-server listener ownership proof timed out") from last_error


def _loopback_opener() -> Any:
    return build_opener(ProxyHandler({}))


def _wait_for_server(host: str, port: int, process: subprocess.Popen[Any], timeout: float) -> None:
    opener = _loopback_opener()
    deadline = time.monotonic() + timeout
    url = f"http://{host}:{port}/health"
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise ArtifactError("llama-server exited before readiness")
        request = Request(url, method="GET")
        try:
            with opener.open(request, timeout=1.0) as response:
                if 200 <= response.status < 300:
                    return
        except HTTPError as error:
            if error.code not in {404, 503}:
                raise ArtifactError("llama-server readiness request failed") from error
        except (OSError, URLError):
            pass
        time.sleep(0.1)
    raise ArtifactError("llama-server readiness timed out")


def _post_schema_smoke(manifest: LockManifest) -> None:
    runtime = manifest.runtime
    url = f"http://{runtime['host']}:{runtime['port']}/v1/chat/completions"
    schema = {
        "type": "object",
        "properties": {"status": {"type": "string", "const": "ok"}},
        "required": ["status"],
        "additionalProperties": False,
    }
    body = {
        "model": manifest.model_file,
        "messages": [
            {"role": "system", "content": manifest.runtime["non_thinking"]},
            {"role": "user", "content": "Return the locked smoke object."},
        ],
        "stream": manifest.runtime["stream"],
        "max_tokens": manifest.runtime["max_output"],
        "response_format": {
            "type": "json_schema",
            "json_schema": {
                "name": "voice_nav_smoke",
                "strict": True,
                "schema": schema,
            },
        },
    }
    request = Request(
        url,
        data=json.dumps(body, separators=(",", ":")).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with _loopback_opener().open(request, timeout=REAL_REQUEST_SECONDS) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except (OSError, URLError, json.JSONDecodeError) as error:
        raise ArtifactError("loopback JSON-schema smoke failed") from error
    try:
        content = payload["choices"][0]["message"]["content"]
        result = json.loads(content)
    except (KeyError, IndexError, TypeError, json.JSONDecodeError) as error:
        raise ArtifactError("loopback JSON-schema smoke returned invalid content") from error
    if result != {"status": "ok"}:
        raise ArtifactError("loopback JSON-schema smoke returned an unexpected object")


def _terminate_process_group(process: subprocess.Popen[Any]) -> bool:
    """Terminate the exact process group and report whether SIGKILL was needed."""

    if process.poll() is not None:
        return False
    if os.name != "posix":
        try:
            process.terminate()
        except OSError as error:
            raise ArtifactError("llama-server termination failed") from error
        try:
            process.wait(timeout=PROCESS_TERM_SECONDS)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
                process.wait(timeout=PROCESS_TERM_SECONDS)
            except (OSError, subprocess.TimeoutExpired) as error:
                raise ArtifactError("llama-server kill/wait failed") from error
            return True
        except OSError as error:
            raise ArtifactError("llama-server termination wait failed") from error
        return False
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return False
    except OSError as error:
        raise ArtifactError("llama-server SIGTERM failed") from error
    try:
        process.wait(timeout=PROCESS_TERM_SECONDS)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except OSError as error:
            raise ArtifactError("llama-server SIGKILL failed") from error
        try:
            process.wait(timeout=PROCESS_TERM_SECONDS)
        except (OSError, subprocess.TimeoutExpired) as error:
            raise ArtifactError("llama-server kill/wait failed") from error
        return True
    except OSError as error:
        raise ArtifactError("llama-server termination wait failed") from error
    return False


def _cpu_identity() -> str:
    processor = platform.processor().strip()
    if processor:
        return processor
    try:
        for line in Path("/proc/cpuinfo").read_text(encoding="utf-8").splitlines():
            if line.lower().startswith("model name") and ":" in line:
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return "unknown"


def _repository_head(repo_root: Path) -> str:
    completed = _run_command(
        [
            "git",
            "-c",
            "safe.directory=*",
            "-C",
            str(repo_root),
            "rev-parse",
            "HEAD",
        ],
        timeout=10.0,
        label="repository HEAD probe",
    )
    head = completed.stdout.strip()
    if not re.fullmatch(r"[0-9a-f]{40}", head):
        raise ArtifactError("repository HEAD probe returned an invalid identity")
    return head


def _bring_up_loopback() -> None:
    if shutil.which("ip") is None:
        raise OfflineGateUnavailable("ip command unavailable")
    try:
        completed = subprocess.run(
            ["ip", "link", "set", "lo", "up"],
            check=False,
            capture_output=True,
            timeout=10.0,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise OfflineGateUnavailable("cannot configure loopback namespace") from error
    if completed.returncode != 0:
        raise OfflineGateUnavailable("cannot configure loopback namespace")


def _capture_process_log(
    stream: Any,
    log_path: Path,
    state: dict[str, Any],
    errors: list[BaseException],
) -> None:
    """Drain the server pipe while retaining only a bounded raw prefix."""

    captured = 0
    truncated = False
    try:
        with log_path.open("wb") as log_stream:
            while True:
                chunk = stream.read(CHUNK_SIZE)
                if not chunk:
                    break
                if captured < REAL_LOG_MAX_BYTES:
                    retained = chunk[: REAL_LOG_MAX_BYTES - captured]
                    log_stream.write(retained)
                    captured += len(retained)
                    if len(retained) != len(chunk):
                        truncated = True
                else:
                    truncated = True
            log_stream.flush()
            os.fsync(log_stream.fileno())
    except BaseException as error:
        errors.append(error)
    finally:
        state["bytes"] = captured
        state["truncated"] = truncated


def real_smoke(
    root: Path,
    manifest: LockManifest,
    digest: str,
    repo_root: Path,
    *,
    in_offline_namespace: bool = False,
) -> None:
    """Run one exact server process group and one closed schema request."""

    if in_offline_namespace:
        _bring_up_loopback()
    root = validate_artifact_root(root, create=False)
    bundle = verify_bundle(root, manifest, digest, check_server=True)
    server = bundle / "bin" / "llama-server"
    model = bundle / "models" / manifest.model_file
    runtime = manifest.runtime
    _check_port_available(runtime["host"], runtime["port"])
    evidence_parent = Path("/tmp") if Path("/tmp").is_dir() else None
    evidence = tempfile.mkdtemp(
        prefix="voice-nav-llm-real-",
        dir=str(evidence_parent) if evidence_parent else None,
    )
    log_path = Path(evidence) / "server.log"
    command = [
        str(server),
        "--model",
        str(model),
        "--host",
        runtime["host"],
        "--port",
        str(runtime["port"]),
        "--ctx-size",
        str(runtime["context"]),
        "--n-predict",
        str(runtime["max_output"]),
        "--parallel",
        str(runtime["parallel"]),
    ]
    try:
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except OSError as error:
        raise RealGateError("cannot start locked llama-server", log_path) from error
    if process.stdout is None:
        raise RealGateError("llama-server log pipe was not created", log_path)
    log_state: dict[str, Any] = {}
    log_errors: list[BaseException] = []
    log_reader = threading.Thread(
        target=_capture_process_log,
        args=(process.stdout, log_path, log_state, log_errors),
        name="voice-nav-llm-log-reader",
        daemon=True,
    )
    log_reader.start()
    gate_error: ArtifactError | None = None
    termination_error: ArtifactError | None = None
    termination_escalated = False
    try:
        _wait_for_owned_listener(runtime["host"], runtime["port"], process, REAL_READINESS_SECONDS)
        _wait_for_server(runtime["host"], runtime["port"], process, REAL_READINESS_SECONDS)
        _check_loopback_listener(runtime["host"], runtime["port"], process)
        _post_schema_smoke(manifest)
    except ArtifactError as error:
        gate_error = error
    finally:
        try:
            termination_escalated = _terminate_process_group(process)
        except ArtifactError as error:
            termination_error = error
        finally:
            log_reader.join(timeout=PROCESS_TERM_SECONDS + 2.0)
            if log_reader.is_alive():
                process.stdout.close()
                log_reader.join(timeout=PROCESS_TERM_SECONDS)
    if log_errors:
        raise RealGateError("server log capture failed", log_path) from log_errors[0]
    if termination_error is not None:
        raise RealGateError(str(termination_error), log_path) from termination_error
    if termination_escalated:
        raise RealGateError("llama-server required SIGKILL during shutdown", log_path)
    if gate_error is not None:
        raise RealGateError(str(gate_error), log_path) from gate_error
    head = _repository_head(repo_root)
    server_identity = _server_version(server).splitlines()[0].strip()
    print(
        "REAL_MODEL_GATE=PASS "
        f"HEAD={head} lock_sha256={digest} "
        f"cpu={_cpu_identity()} "
        f"server={server_identity} model={manifest.model_file} "
        f"listen={runtime['host']}:{runtime['port']} smoke=status:ok "
        f"log={log_path} log_bytes={log_state.get('bytes', 0)} "
        f"log_truncated={str(log_state.get('truncated', False)).lower()}"
    )


def run_real_gate(
    root: Path,
    manifest: LockManifest,
    digest: str,
    repo_root: Path,
    *,
    offline: bool,
) -> int:
    """Run the real gate directly or inside the required network namespace."""

    if not offline:
        real_smoke(root, manifest, digest, repo_root)
        return 0
    if shutil.which("unshare") is None:
        print("REAL_MODEL_GATE=NOT_RUN reason=offline-namespace-unavailable")
        return 2
    command = [
        "unshare",
        "--user",
        "--map-root-user",
        "--net",
        sys.executable,
        str(Path(__file__).resolve()),
        "_real_smoke",
        "--root",
        str(root),
        "--repo-root",
        str(repo_root),
    ]
    try:
        completed = subprocess.run(command, check=False, capture_output=True, text=True)
    except OSError:
        print("REAL_MODEL_GATE=NOT_RUN reason=offline-namespace-unavailable")
        return 2
    if completed.stdout:
        sys.stdout.write(completed.stdout)
    if completed.stderr:
        sys.stderr.write(completed.stderr)
    child_output = (completed.stdout or "") + (completed.stderr or "")
    if completed.returncode != 0 and "REAL_MODEL_GATE=" not in child_output:
        # `unshare` returns before the child can print evidence when user
        # namespaces are disabled or iproute2 cannot create the namespace.
        print("REAL_MODEL_GATE=NOT_RUN reason=offline-namespace-unavailable")
        return 2
    return completed.returncode


def _verify_manifest_only(repo_root: Path) -> tuple[LockManifest, str]:
    manifest = load_lock_manifest()
    validate_notice_consistency(manifest)
    verify_repository_artifact_boundary(repo_root)
    digest = lock_sha256()
    return manifest, digest


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    provision = subparsers.add_parser("provision", help="download, build, and publish the locked bundle")
    provision.add_argument("--root", type=Path, default=None)
    provision.add_argument("--repo-root", type=Path, default=REPOSITORY_ROOT)

    verify = subparsers.add_parser("verify", help="verify the manifest or run the explicit real gate")
    verify.add_argument("--root", type=Path, default=None)
    verify.add_argument("--repo-root", type=Path, default=REPOSITORY_ROOT)
    verify.add_argument("--real", action="store_true")
    verify.add_argument("--offline", action="store_true")

    real = subparsers.add_parser("_real_smoke", help=argparse.SUPPRESS)
    real.add_argument("--root", type=Path, required=True)
    real.add_argument("--repo-root", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _build_parser().parse_args(argv)
    try:
        if arguments.command == "provision":
            if os.name != "posix":
                raise ArtifactError("provisioning requires a Linux/WSL filesystem")
            manifest = load_lock_manifest()
            validate_notice_consistency(manifest)
            verify_repository_artifact_boundary(arguments.repo_root)
            root = arguments.root or DEFAULT_ARTIFACT_ROOT
            digest = lock_sha256()
            result = Provisioner(root, manifest, digest).provision()
            state = "IDEMPOTENT" if result.idempotent else "PASS"
            print(f"PROVISION={state} lock_sha256={result.lock_sha256} bundle={result.bundle}")
            return 0

        if arguments.command == "_real_smoke":
            manifest = load_lock_manifest()
            digest = lock_sha256()
            try:
                real_smoke(arguments.root, manifest, digest, arguments.repo_root, in_offline_namespace=True)
            except OfflineGateUnavailable:
                print("REAL_MODEL_GATE=NOT_RUN reason=offline-namespace-unavailable")
                return 2
            return 0

        if arguments.offline and not arguments.real:
            raise ArtifactError("--offline is valid only with --real")
        manifest, digest = _verify_manifest_only(arguments.repo_root)
        if not arguments.real:
            print("MANIFEST_GATE=PASS")
            print("REAL_MODEL_GATE=NOT_RUN reason=--real-not-requested")
            return 0
        root = arguments.root
        if root is None:
            raise ArtifactError("--real requires an explicit --root")
        return run_real_gate(root, manifest, digest, arguments.repo_root, offline=arguments.offline)
    except ArtifactError as error:
        if arguments.command == "provision":
            print(f"PROVISION=FAIL reason={_safe_error(error)}", file=sys.stderr)
        elif arguments.command in {"verify", "_real_smoke"} and getattr(arguments, "real", arguments.command == "_real_smoke"):
            if isinstance(error, RealGateError):
                print(
                    f"REAL_MODEL_GATE=FAIL reason={_safe_error(error)} log={error.log_path}",
                    file=sys.stderr,
                )
            else:
                print(f"REAL_MODEL_GATE=FAIL reason={_safe_error(error)}", file=sys.stderr)
        else:
            print(f"MANIFEST_GATE=FAIL reason={_safe_error(error)}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
