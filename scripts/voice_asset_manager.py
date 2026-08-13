#!/usr/bin/env python3
"""Validate and explicitly provision immutable, offline VoiceNav audio assets."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import secrets
import stat
import subprocess
from typing import Any, Callable, Mapping, Sequence
from urllib.parse import urlparse
from urllib.request import Request, urlopen


DEPENDENCY_MANIFEST_PATH = Path("third_party/locks/audio-dependencies.yaml")
MODEL_MANIFEST_PATH = Path("models/manifests/voice-models.yaml")
DEPENDENCY_ROOT_RELATIVE = Path(".deps/voice-assets")
MODEL_ROOT_RELATIVE = Path("models/weights/voice-assets")
SHA256_PATTERN = re.compile(r"\A[0-9a-f]{64}\Z")
IDENTIFIER_PATTERN = re.compile(r"\A[a-z0-9][a-z0-9-]*\Z")
FLOATING_REFERENCE_PARTS = frozenset({"head", "latest", "main", "master"})
PROVENANCE_STATUSES = frozenset({"resolved", "restricted", "unresolved"})


class ManifestError(ValueError):
    """A versioned VoiceNav asset manifest is unsafe or incomplete."""


class ProvisionError(RuntimeError):
    """A VoiceNav asset cannot be proven complete and safe to use."""


@dataclass(frozen=True)
class ModelProvenance:
    """Separate model-weight and training-data licensing from runtime licensing."""

    status: str
    weights_license: str | None
    weights_license_url: str | None
    training_data_provenance: str
    training_data_url: str | None
    model_card_url: str


@dataclass(frozen=True)
class Asset:
    """One immutable source archive, patch, or model archive."""

    identifier: str
    version: str
    revision: str
    url: str
    size: int
    sha256: str
    license: str
    license_url: str
    destination: PurePosixPath
    build_options: Mapping[str, str]
    sample_rate: int | None = None
    model_card_url: str | None = None
    runtime_license: str | None = None
    runtime_license_url: str | None = None
    model_provenance: ModelProvenance | None = None
    release_asset_id: int | None = None


@dataclass(frozen=True)
class AssetManifest:
    """A validated dependency or model manifest."""

    schema_version: int
    assets: tuple[Asset, ...]
    category: str


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    document: dict[str, Any] = {}
    for key, value in pairs:
        if key in document:
            raise ManifestError(f"duplicate JSON key: {key}")
        document[key] = value
    return document


def _load_json_yaml(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_reject_duplicate_keys)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ManifestError) as error:
        raise ManifestError(f"cannot load manifest: {path}") from error


def _require_exact_keys(document: Mapping[str, Any], expected: frozenset[str], label: str) -> None:
    actual = frozenset(document)
    if actual != expected:
        missing = ",".join(sorted(expected - actual))
        unknown = ",".join(sorted(actual - expected))
        raise ManifestError(f"{label} keys are closed (missing={missing}; unknown={unknown})")


def _require_text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ManifestError(f"{label} must be a non-empty string")
    return value


def _require_immutable_reference(value: Any, label: str) -> str:
    reference = _require_text(value, label)
    parts = [part for part in re.split(r"[^a-z0-9]+", reference.lower()) if part]
    if any(part in FLOATING_REFERENCE_PARTS for part in parts):
        raise ManifestError(f"{label} must not be floating")
    return reference


def _require_https_url(value: Any, label: str) -> str:
    url = _require_text(value, label)
    parsed = urlparse(url)
    if parsed.scheme != "https" or not parsed.netloc or parsed.username or parsed.password:
        raise ManifestError(f"{label} must be a credential-free HTTPS URL")
    path_parts = [part.lower() for part in parsed.path.split("/") if part]
    if parsed.fragment or any(
        part in FLOATING_REFERENCE_PARTS
        or part.split(".", 1)[0] in FLOATING_REFERENCE_PARTS
        for part in path_parts
    ):
        raise ManifestError(f"{label} must not use a floating reference")
    return url


def _require_optional_https_url(value: Any, label: str) -> str | None:
    if value is None:
        return None
    return _require_https_url(value, label)


def _require_destination(value: Any, label: str) -> PurePosixPath:
    raw = _require_text(value, label)
    path = PurePosixPath(raw)
    if (
        path.is_absolute()
        or "\\" in raw
        or any(part in {"", ".", ".."} for part in path.parts)
        or len(path.parts) < 2
    ):
        raise ManifestError(f"{label} must be a safe relative artifact path")
    return path


def _require_positive_integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ManifestError(f"{label} must be a positive integer")
    return value


def _require_build_options(value: Any, label: str) -> Mapping[str, str]:
    if not isinstance(value, dict) or not value:
        raise ManifestError(f"{label} must be a non-empty object")
    result: dict[str, str] = {}
    for name, option in value.items():
        result[_require_text(name, f"{label} name")] = _require_text(option, f"{label}.{name}")
    return result


def _validate_model_provenance(value: Any) -> ModelProvenance:
    expected = frozenset(
        {
            "status",
            "weights_license",
            "weights_license_url",
            "training_data_provenance",
            "training_data_url",
            "model_card_url",
        }
    )
    if not isinstance(value, dict):
        raise ManifestError("asset.model_provenance must be an object")
    _require_exact_keys(value, expected, "asset.model_provenance")
    status = _require_text(value["status"], "asset.model_provenance.status")
    if status not in PROVENANCE_STATUSES:
        raise ManifestError("asset.model_provenance.status is unsupported")
    weights_license = value["weights_license"]
    weights_license_url = _require_optional_https_url(
        value["weights_license_url"], "asset.model_provenance.weights_license_url"
    )
    training_data_url = _require_optional_https_url(
        value["training_data_url"], "asset.model_provenance.training_data_url"
    )
    training_data_provenance = _require_text(
        value["training_data_provenance"], "asset.model_provenance.training_data_provenance"
    )
    if status == "unresolved":
        if weights_license is not None or weights_license_url is not None or training_data_url is not None:
            raise ManifestError("unresolved model provenance must not claim an authoritative license")
    else:
        weights_license = _require_text(weights_license, "asset.model_provenance.weights_license")
        if weights_license_url is None or training_data_url is None:
            raise ManifestError("resolved or restricted model provenance requires source URLs")
    return ModelProvenance(
        status=status,
        weights_license=weights_license,
        weights_license_url=weights_license_url,
        training_data_provenance=training_data_provenance,
        training_data_url=training_data_url,
        model_card_url=_require_https_url(value["model_card_url"], "asset.model_provenance.model_card_url"),
    )


def _validate_asset(document: Any, category: str) -> Asset:
    dependency_keys = frozenset(
        {
            "id",
            "version",
            "revision",
            "url",
            "size",
            "sha256",
            "license",
            "license_url",
            "destination",
            "build_options",
        }
    )
    model_keys = frozenset(
        {
            "id",
            "version",
            "revision",
            "url",
            "size",
            "sha256",
            "destination",
            "build_options",
            "sample_rate",
            "runtime_license",
            "runtime_license_url",
            "model_provenance",
            "release_asset_id",
        }
    )
    if not isinstance(document, dict):
        raise ManifestError("asset must be an object")
    _require_exact_keys(document, dependency_keys if category == "dependencies" else model_keys, "asset")
    identifier = _require_text(document["id"], "asset.id")
    if IDENTIFIER_PATTERN.fullmatch(identifier) is None:
        raise ManifestError("asset.id must be a lowercase stable identifier")
    sha256 = _require_text(document["sha256"], "asset.sha256")
    if SHA256_PATTERN.fullmatch(sha256) is None:
        raise ManifestError("asset.sha256 must be lowercase 64-hex")
    version = _require_immutable_reference(document["version"], "asset.version")
    revision = _require_immutable_reference(document["revision"], "asset.revision")
    url = _require_https_url(document["url"], "asset.url")
    destination = _require_destination(document["destination"], "asset.destination")
    build_options = _require_build_options(document["build_options"], "asset.build_options")
    if category == "dependencies":
        return Asset(
            identifier=identifier,
            version=version,
            revision=revision,
            url=url,
            size=_require_positive_integer(document["size"], "asset.size"),
            sha256=sha256,
            license=_require_text(document["license"], "asset.license"),
            license_url=_require_https_url(document["license_url"], "asset.license_url"),
            destination=destination,
            build_options=build_options,
        )

    release_asset_id = document["release_asset_id"]
    if release_asset_id is not None:
        release_asset_id = _require_positive_integer(release_asset_id, "asset.release_asset_id")
        expected_url = f"https://api.github.com/repos/k2-fsa/sherpa-onnx/releases/assets/{release_asset_id}"
        if url != expected_url or revision != str(release_asset_id):
            raise ManifestError("GitHub release asset locks must use its immutable API identity")
    provenance = _validate_model_provenance(document["model_provenance"])
    runtime_license = _require_text(document["runtime_license"], "asset.runtime_license")
    runtime_license_url = _require_https_url(document["runtime_license_url"], "asset.runtime_license_url")
    return Asset(
        identifier=identifier,
        version=version,
        revision=revision,
        url=url,
        size=_require_positive_integer(document["size"], "asset.size"),
        sha256=sha256,
        license=runtime_license,
        license_url=runtime_license_url,
        destination=destination,
        build_options=build_options,
        sample_rate=_require_positive_integer(document["sample_rate"], "asset.sample_rate"),
        model_card_url=provenance.model_card_url,
        runtime_license=runtime_license,
        runtime_license_url=runtime_license_url,
        model_provenance=provenance,
        release_asset_id=release_asset_id,
    )


def _require_unique_assets(assets: Sequence[Asset], label: str) -> None:
    identifiers = [asset.identifier for asset in assets]
    destinations = [asset.destination.as_posix() for asset in assets]
    if len(set(identifiers)) != len(identifiers):
        raise ManifestError(f"{label} contains duplicate asset id")
    if len(set(destinations)) != len(destinations):
        raise ManifestError(f"{label} contains duplicate artifact destination")


def load_manifest(path: Path, category: str) -> AssetManifest:
    """Load one closed, JSON-compatible YAML manifest without network access."""

    if category not in {"dependencies", "models"}:
        raise ValueError(f"unknown asset category: {category}")
    document = _load_json_yaml(path)
    if not isinstance(document, dict):
        raise ManifestError("manifest must be an object")
    _require_exact_keys(document, frozenset({"schema_version", "assets"}), "manifest")
    if document["schema_version"] != 1 or isinstance(document["schema_version"], bool):
        raise ManifestError("manifest.schema_version must be exactly 1")
    if not isinstance(document["assets"], list) or not document["assets"]:
        raise ManifestError("manifest.assets must be a non-empty list")
    assets = tuple(_validate_asset(asset, category) for asset in document["assets"])
    _require_unique_assets(assets, "manifest")
    return AssetManifest(schema_version=1, assets=assets, category=category)


def load_manifests(repo_root: Path) -> tuple[AssetManifest, AssetManifest]:
    """Load the versioned dependency and model locks from one repository."""

    dependencies = load_manifest(repo_root / DEPENDENCY_MANIFEST_PATH, "dependencies")
    models = load_manifest(repo_root / MODEL_MANIFEST_PATH, "models")
    _require_unique_assets((*dependencies.assets, *models.assets), "manifests")
    return dependencies, models


def _require_regular_file(path: Path, label: str) -> None:
    try:
        info = path.lstat()
    except OSError as error:
        raise ProvisionError(f"cannot inspect {label}") from error
    if not path.is_file() or path.is_symlink() or not stat.S_ISREG(info.st_mode):
        raise ProvisionError(f"{label} must be a regular non-symlink file")


def _file_identity(path: Path, asset: Asset, label: str) -> None:
    _require_regular_file(path, label)
    digest = hashlib.sha256()
    size = 0
    try:
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                size += len(chunk)
                digest.update(chunk)
    except OSError as error:
        raise ProvisionError(f"cannot read {label}") from error
    if size != asset.size or digest.hexdigest() != asset.sha256:
        raise ProvisionError(f"{label} does not match its locked size and SHA-256")


def _ensure_real_directory(path: Path, label: str) -> Path:
    try:
        path.mkdir(parents=True, exist_ok=True)
        info = path.lstat()
    except OSError as error:
        raise ProvisionError(f"cannot prepare {label}") from error
    if path.is_symlink() or not stat.S_ISDIR(info.st_mode):
        raise ProvisionError(f"{label} must be a real directory")
    return path.resolve()


def _safe_target(root: Path, destination: PurePosixPath) -> Path:
    root = _ensure_real_directory(root, "artifact root")
    current = root
    for part in destination.parts[:-1]:
        current = _ensure_real_directory(current / part, "artifact directory")
        if not current.is_relative_to(root):
            raise ProvisionError("artifact destination escapes its root")
    target = current / destination.name
    if not target.is_relative_to(root):
        raise ProvisionError("artifact destination escapes its root")
    if target.exists() and target.is_symlink():
        raise ProvisionError("artifact destination must not be a symlink")
    return target


def _download(asset: Asset, destination: Path) -> None:
    headers = {"User-Agent": "VoiceNav-asset-provisioner"}
    if asset.release_asset_id is not None:
        headers["Accept"] = "application/octet-stream"
    request = Request(asset.url, headers=headers)
    try:
        with urlopen(request, timeout=60) as response, destination.open("xb") as stream:
            while chunk := response.read(1024 * 1024):
                stream.write(chunk)
    except OSError as error:
        raise ProvisionError("asset download failed") from error


@dataclass(frozen=True)
class ProvisionResult:
    """Names published by this call and names reused after verification."""

    installed: tuple[str, ...]
    reused: tuple[str, ...]


class Provisioner:
    """Explicitly download, verify, and atomically publish locked assets."""

    def __init__(
        self,
        dependencies: AssetManifest,
        models: AssetManifest,
        *,
        dependency_root: Path,
        model_root: Path,
        downloader: Callable[[Asset, Path], None] = _download,
    ) -> None:
        if dependencies.category != "dependencies" or models.category != "models":
            raise ValueError("asset manifests must retain their category")
        self._groups = ((dependencies, dependency_root), (models, model_root))
        self._downloader = downloader

    def _preflight_licenses(self) -> None:
        for manifest, _root in self._groups:
            for asset in manifest.assets:
                provenance = asset.model_provenance
                if provenance is not None and provenance.status == "unresolved":
                    raise ProvisionError(f"model license provenance unresolved: {asset.identifier}")

    def _verify_group(self, manifest: AssetManifest, root: Path) -> tuple[str, ...]:
        verified: list[str] = []
        for asset in manifest.assets:
            target = _safe_target(root, asset.destination)
            if not target.exists():
                raise ProvisionError(f"missing verified asset: {asset.identifier}")
            _file_identity(target, asset, asset.identifier)
            verified.append(asset.identifier)
        return tuple(verified)

    def verify(self) -> tuple[str, ...]:
        """Fail closed unless every runtime asset already verifies offline."""

        self._preflight_licenses()
        verified: list[str] = []
        for manifest, root in self._groups:
            verified.extend(self._verify_group(manifest, root))
        return tuple(verified)

    def verified_path(self, identifier: str) -> Path:
        """Return one runtime path only after rechecking its locked identity."""

        self._preflight_licenses()
        for manifest, root in self._groups:
            for asset in manifest.assets:
                if asset.identifier != identifier:
                    continue
                target = _safe_target(root, asset.destination)
                if not target.exists():
                    raise ProvisionError(f"missing verified asset: {identifier}")
                _file_identity(target, asset, identifier)
                return target
        raise ProvisionError(f"unknown locked asset: {identifier}")

    def provision(self, *, offline: bool = False) -> ProvisionResult:
        """Install missing assets after validation; never publish a partial file."""

        self._preflight_licenses()
        installed: list[str] = []
        reused: list[str] = []
        for manifest, root in self._groups:
            for asset in manifest.assets:
                target = _safe_target(root, asset.destination)
                if target.exists():
                    _file_identity(target, asset, asset.identifier)
                    reused.append(asset.identifier)
                    continue
                if offline:
                    raise ProvisionError(f"missing verified asset while offline: {asset.identifier}")
                temporary = target.with_name(f".{target.name}.{secrets.token_hex(8)}.partial")
                try:
                    self._downloader(asset, temporary)
                    _file_identity(temporary, asset, f"temporary {asset.identifier}")
                    try:
                        os.link(temporary, target)
                    except FileExistsError as error:
                        raise ProvisionError(f"refusing to overwrite existing asset: {asset.identifier}") from error
                finally:
                    try:
                        temporary.unlink(missing_ok=True)
                    except OSError as error:
                        raise ProvisionError(f"cannot remove temporary asset: {asset.identifier}") from error
                _file_identity(target, asset, asset.identifier)
                installed.append(asset.identifier)
        return ProvisionResult(installed=tuple(installed), reused=tuple(reused))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("provision", "verify"))
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--dependency-root", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--model-root", type=Path, help=argparse.SUPPRESS)
    parser.add_argument(
        "--offline",
        action="store_true",
        help="refuse downloads and require all assets to be present already",
    )
    return parser


def _reject_linked_approved_components(repo_root: Path, relative: Path, label: str) -> None:
    """Reject a symlink or Windows reparse point before it can escape ``repo_root``."""

    current = repo_root
    reparse_point = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x0400)
    for component in relative.parts:
        current = current / component
        try:
            info = current.lstat()
        except FileNotFoundError:
            return
        except OSError as error:
            raise ProvisionError(f"cannot inspect {label}: {current}") from error
        if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & reparse_point:
            raise ProvisionError(f"{label} must not contain a symlink or reparse point")


def _resolve_approved_cli_root(repo_root: Path, relative: Path, label: str) -> Path:
    _reject_linked_approved_components(repo_root, relative, label)
    try:
        resolved = (repo_root / relative).resolve()
    except OSError as error:
        raise ProvisionError(f"cannot resolve {label}") from error
    if resolved == repo_root or not resolved.is_relative_to(repo_root):
        raise ProvisionError(f"{label} must resolve strictly within the repository")
    return resolved


def _approved_cli_root(repo_root: Path, requested: Path | None, relative: Path, label: str) -> Path:
    expected = _resolve_approved_cli_root(repo_root, relative, label)
    candidate = expected if requested is None else requested.resolve()
    if candidate != expected:
        raise ProvisionError(f"{label} must be the approved ignored repository root: {relative.as_posix()}")
    completed = subprocess.run(
        ["git", "-c", f"safe.directory={repo_root}", "-C", str(repo_root), "check-ignore", "-q", "--", relative.as_posix()],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise ProvisionError(f"{label} must be ignored by repository policy: {relative.as_posix()}")
    return expected


def _validated_cli_roots(arguments: argparse.Namespace) -> tuple[Path, Path, Path]:
    repo_root = arguments.repo_root.resolve()
    dependency_root = _approved_cli_root(repo_root, arguments.dependency_root, DEPENDENCY_ROOT_RELATIVE, "dependency root")
    model_root = _approved_cli_root(repo_root, arguments.model_root, MODEL_ROOT_RELATIVE, "model root")
    return repo_root, dependency_root, model_root


def main(argv: Sequence[str] | None = None) -> int:
    """Run an explicit provision or offline verification operation."""

    arguments = _parser().parse_args(argv)
    try:
        repo_root, dependency_root, model_root = _validated_cli_roots(arguments)
        dependencies, models = load_manifests(repo_root)
        provisioner = Provisioner(
            dependencies,
            models,
            dependency_root=dependency_root,
            model_root=model_root,
        )
        if arguments.command == "verify":
            verified = provisioner.verify()
            print("verified voice assets: " + ", ".join(verified))
        else:
            result = provisioner.provision(offline=arguments.offline)
            print("installed voice assets: " + ", ".join(result.installed))
            print("reused voice assets: " + ", ".join(result.reused))
    except (ManifestError, ProvisionError) as error:
        print(f"voice asset operation failed closed: {error}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
