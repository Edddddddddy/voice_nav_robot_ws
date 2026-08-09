"""Focused offline tests for the locked LLM artifact manager."""

from __future__ import annotations

import copy
import hashlib
from http.client import IncompleteRead
import io
import json
import multiprocessing
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest import mock

from scripts.llm import artifact_manager as llm


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class _Response:
    def __init__(self, payload: bytes) -> None:
        self.payload = payload

    def __enter__(self) -> _Response:
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        return None

    def read(self, size: int = -1) -> bytes:
        if size < 0:
            payload, self.payload = self.payload, b""
            return payload
        payload, self.payload = self.payload[:size], self.payload[size:]
        return payload


class _Opener:
    def __init__(self, payload: bytes) -> None:
        self.payload = payload

    def open(self, url: str, timeout: float) -> _Response:
        return _Response(self.payload)


class _FakeProcess:
    pid = 4242

    def __init__(self, wait_results: list[object]) -> None:
        self.wait_results = list(wait_results)
        self.wait_calls: list[float] = []

    def poll(self) -> None:
        return None

    def wait(self, timeout: float) -> int:
        self.wait_calls.append(timeout)
        result = self.wait_results.pop(0)
        if isinstance(result, BaseException):
            raise result
        return int(result)


def _hold_bundle_lock(root: str, acquired, release) -> None:
    with llm.BundleLock(Path(root)):
        acquired.set()
        release.wait(5)


def _wait_for_bundle_lock(root: str, acquired) -> None:
    with llm.BundleLock(Path(root)):
        acquired.set()


class LlmManifestTest(unittest.TestCase):
    def test_approved_manifest_and_notice_are_consistent(self) -> None:
        manifest = llm.load_lock_manifest()

        self.assertEqual(manifest.model["repo"], "Qwen/Qwen3-0.6B-GGUF")
        self.assertEqual(manifest.model["revision"], "23749fefcc72300e3a2ad315e1317431b06b590a")
        self.assertEqual(manifest.model["file"], "Qwen3-0.6B-Q8_0.gguf")
        self.assertEqual(manifest.model["size"], 639446688)
        self.assertEqual(
            manifest.model["sha256"],
            "9465e63a22add5354d9bb4b99e90117043c7124007664907259bd16d043bb031",
        )
        self.assertEqual(manifest.llama_cpp["tag"], "b10276")
        self.assertEqual(
            manifest.llama_cpp["commit"],
            "6ea215d171fd31df943bf1ac8227129f2b963160",
        )
        self.assertEqual(manifest.runtime["host"], "127.0.0.1")
        self.assertEqual(manifest.runtime["port"], 8080)
        self.assertEqual(manifest.runtime["context"], 2048)
        self.assertEqual(manifest.runtime["max_output"], 256)
        self.assertEqual(manifest.runtime["parallel"], 1)
        self.assertFalse(manifest.runtime["stream"])
        self.assertEqual(manifest.runtime["non_thinking"], "/no_think")
        llm.validate_notice_consistency(manifest)


class RealGateLogTest(unittest.TestCase):
    def test_server_log_is_retained_with_a_hard_size_bound(self) -> None:
        payload = b"raw-log\n" * ((llm.REAL_LOG_MAX_BYTES // 8) + 32)
        with tempfile.TemporaryDirectory() as temporary_directory:
            log_path = Path(temporary_directory) / "server.log"
            state: dict[str, object] = {}
            errors: list[BaseException] = []
            llm._capture_process_log(io.BytesIO(payload), log_path, state, errors)

            self.assertEqual(errors, [])
            self.assertEqual(log_path.read_bytes(), payload[: llm.REAL_LOG_MAX_BYTES])
            self.assertEqual(state["bytes"], llm.REAL_LOG_MAX_BYTES)
            self.assertTrue(state["truncated"])


class ManifestBoundaryTest(unittest.TestCase):
    def test_duplicate_and_unknown_manifest_keys_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            duplicate = Path(temporary_directory) / "duplicate.json"
            duplicate.write_text(
                '{"schema_version":1,"schema_version":1}\n',
                encoding="utf-8",
            )
            with self.assertRaises(llm.ManifestError):
                llm.load_lock_manifest(duplicate, approved=False)

            unknown = copy.deepcopy(llm.APPROVED_DOCUMENT)
            unknown["unexpected"] = True
            with self.assertRaises(llm.ManifestError):
                llm.validate_manifest_document(unknown)

    def test_mutated_approved_value_fails_even_when_shape_is_valid(self) -> None:
        document = copy.deepcopy(llm.APPROVED_DOCUMENT)
        document["runtime"]["host"] = "0.0.0.0"
        with self.assertRaises(llm.ManifestError):
            llm.validate_manifest_document(document, approved=True)

        document = copy.deepcopy(llm.APPROVED_DOCUMENT)
        document["llama_cpp"]["build"]["flags"]["GGML_NATIVE"] = "ON"
        with self.assertRaises(llm.ManifestError):
            llm.validate_manifest_document(document, approved=True)

    def test_repo_boundary_rejects_tracked_model_and_build_artifacts(self) -> None:
        with self.assertRaises(llm.ArtifactError):
            llm.verify_repository_artifact_boundary(
                REPOSITORY_ROOT,
                tracked_paths=[
                    "models/locks/voice_nav_llm_v1.lock.json",
                    "models/weights/Qwen3-0.6B-Q8_0.gguf",
                    "build/llama-server",
                    "llama.cpp-locked/source/CMakeLists.txt",
                ],
            )
        llm.verify_repository_artifact_boundary(
            REPOSITORY_ROOT,
            tracked_paths=[
                "models/locks/voice_nav_llm_v1.lock.json",
                "docs/process/third-party-llm-notices.md",
            ],
        )

    def test_repo_boundary_rejects_tracked_runtime_log_files(self) -> None:
        with self.assertRaises(llm.ArtifactError):
            llm.verify_repository_artifact_boundary(
                REPOSITORY_ROOT,
                tracked_paths=[
                    "server.log",
                    "evidence/gate.log",
                    "logs/server.txt",
                ],
            )

    @unittest.skipIf(
        os.name == "posix" and REPOSITORY_ROOT.as_posix().startswith("/mnt/"),
        "managed WSL Git context is initialized by the shell wrapper",
    )
    def test_manifest_cli_reports_real_gate_not_run(self) -> None:
        completed = subprocess.run(
            [
                sys.executable,
                str(REPOSITORY_ROOT / "scripts" / "llm" / "artifact_manager.py"),
                "verify",
                "--repo-root",
                str(REPOSITORY_ROOT),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0)
        self.assertIn("MANIFEST_GATE=PASS", completed.stdout)
        self.assertIn(
            "REAL_MODEL_GATE=NOT_RUN reason=--real-not-requested",
            completed.stdout,
        )
        self.assertNotIn("REAL_MODEL_GATE=PASS", completed.stdout)


@unittest.skipUnless(os.name == "posix", "process-group termination is a POSIX gate seam")
class ProcessTerminationTest(unittest.TestCase):
    def test_term_normal_exit_reports_no_escalation(self) -> None:
        process = _FakeProcess([0])
        with mock.patch.object(llm.os, "killpg") as killpg:
            result = llm._terminate_process_group(process)

        self.assertEqual(result, False)
        killpg.assert_called_once_with(process.pid, llm.signal.SIGTERM)
        self.assertEqual(process.wait_calls, [llm.PROCESS_TERM_SECONDS])

    def test_term_timeout_kill_reports_escalation_after_group_cleanup(self) -> None:
        process = _FakeProcess(
            [
                subprocess.TimeoutExpired("llama-server", llm.PROCESS_TERM_SECONDS),
                0,
            ]
        )
        with mock.patch.object(llm.os, "killpg") as killpg:
            result = llm._terminate_process_group(process)

        self.assertEqual(result, True)
        self.assertEqual(
            killpg.call_args_list,
            [
                mock.call(process.pid, llm.signal.SIGTERM),
                mock.call(process.pid, llm.signal.SIGKILL),
            ],
        )
        self.assertEqual(process.wait_calls, [llm.PROCESS_TERM_SECONDS] * 2)


@unittest.skipUnless(os.name == "posix", "flock concurrency is a Linux provisioning seam")
class BundleLockTest(unittest.TestCase):
    def test_concurrent_provisioners_serialize_on_root_local_flock(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "llm"
            root.mkdir(mode=0o700)
            holder_acquired = multiprocessing.Event()
            release_holder = multiprocessing.Event()
            waiter_acquired = multiprocessing.Event()
            holder = multiprocessing.Process(
                target=_hold_bundle_lock,
                args=(str(root), holder_acquired, release_holder),
            )
            waiter = multiprocessing.Process(
                target=_wait_for_bundle_lock,
                args=(str(root), waiter_acquired),
            )
            try:
                holder.start()
                self.assertTrue(holder_acquired.wait(5))
                waiter.start()
                self.assertFalse(waiter_acquired.wait(0.3))
                release_holder.set()
                self.assertTrue(waiter_acquired.wait(5))
                holder.join(5)
                waiter.join(5)
            finally:
                release_holder.set()
                if holder.is_alive():
                    holder.terminate()
                if waiter.is_alive():
                    waiter.terminate()
                holder.join(5)
                waiter.join(5)

            self.assertEqual(holder.exitcode, 0)
            self.assertEqual(waiter.exitcode, 0)

    def test_real_gate_fails_when_server_cleanup_escalates_to_kill(self) -> None:
        manifest = llm.load_lock_manifest()
        digest = llm.lock_sha256()
        fake_process = mock.Mock()
        fake_process.stdout = io.BytesIO()
        fake_process.pid = 4242

        with tempfile.TemporaryDirectory() as temporary_directory:
            evidence = Path(temporary_directory)
            with (
                mock.patch.object(llm, "validate_artifact_root", return_value=Path(temporary_directory)),
                mock.patch.object(llm, "verify_bundle", return_value=evidence),
                mock.patch.object(llm, "_check_port_available"),
                mock.patch.object(llm.tempfile, "mkdtemp", return_value=str(evidence)),
                mock.patch.object(llm.subprocess, "Popen", return_value=fake_process),
                mock.patch.object(llm, "_capture_process_log"),
                mock.patch.object(llm, "_wait_for_owned_listener"),
                mock.patch.object(llm, "_wait_for_server"),
                mock.patch.object(llm, "_check_loopback_listener"),
                mock.patch.object(llm, "_post_schema_smoke"),
                mock.patch.object(llm, "_terminate_process_group", return_value=True),
                mock.patch.object(llm, "_repository_head", return_value="a" * 40),
                mock.patch.object(llm, "_server_version", return_value="fixture-server 1\n"),
                mock.patch.object(llm, "_cpu_identity", return_value="fixture-cpu"),
            ):
                with self.assertRaises(llm.RealGateError) as context:
                    llm.real_smoke(
                        Path(temporary_directory),
                        manifest,
                        digest,
                        REPOSITORY_ROOT,
                    )

        self.assertIn("SIGKILL", str(context.exception))

    def test_kill_wait_failure_is_a_controlled_gate_error(self) -> None:
        process = _FakeProcess(
            [
                subprocess.TimeoutExpired("llama-server", llm.PROCESS_TERM_SECONDS),
                subprocess.TimeoutExpired("llama-server", llm.PROCESS_TERM_SECONDS),
            ]
        )
        with mock.patch.object(llm.os, "killpg") as killpg:
            with self.assertRaises(llm.ArtifactError) as context:
                llm._terminate_process_group(process)

        self.assertIn("kill/wait", str(context.exception))
        self.assertEqual(
            killpg.call_args_list,
            [
                mock.call(process.pid, llm.signal.SIGTERM),
                mock.call(process.pid, llm.signal.SIGKILL),
            ],
        )


@unittest.skipUnless(os.name == "posix", "listener ownership is a Linux gate seam")
class ListenerOwnershipTest(unittest.TestCase):
    def test_unrelated_listener_blocks_gate_without_killing_it(self) -> None:
        if shutil.which("ss") is None:
            self.skipTest("ss is required for listener ownership evidence")
        listener = subprocess.Popen(
            [
                sys.executable,
                "-c",
                (
                    "import socket,time; "
                    "sock=socket.socket(); sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); "
                    "sock.bind(('127.0.0.1', 8080)); sock.listen(); "
                    "print('READY', flush=True); time.sleep(30)"
                ),
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        try:
            ready = listener.stdout.readline() if listener.stdout is not None else ""
            if ready.strip() != "READY":
                self.skipTest("127.0.0.1:8080 was already occupied before fixture startup")

            manifest = llm.load_lock_manifest()
            digest = llm.lock_sha256()
            with tempfile.TemporaryDirectory() as temporary_directory:
                bundle = Path(temporary_directory)
                original_popen = subprocess.Popen

                def reject_server_start(command, *args, **kwargs):
                    if list(command)[:2] == ["ss", "-ltnp"]:
                        return original_popen(command, *args, **kwargs)
                    raise AssertionError("gate attempted to start beside an existing listener")

                with (
                    mock.patch.object(llm, "validate_artifact_root", return_value=bundle),
                    mock.patch.object(llm, "verify_bundle", return_value=bundle),
                    mock.patch.object(
                        llm.subprocess,
                        "Popen",
                        side_effect=reject_server_start,
                    ),
                ):
                    with self.assertRaises(llm.ArtifactError) as context:
                        llm.real_smoke(bundle, manifest, digest, REPOSITORY_ROOT)

            self.assertIn("already in use", str(context.exception))
            self.assertIsNone(listener.poll())
        finally:
            if listener.poll() is None:
                listener.terminate()
                try:
                    listener.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    listener.kill()
                    listener.wait(timeout=5)
            if listener.stdout is not None:
                listener.stdout.close()
            if listener.stderr is not None:
                listener.stderr.close()

    def test_listener_pid_must_belong_to_launched_process_group(self) -> None:
        process = _FakeProcess([0])
        ss_result = subprocess.CompletedProcess(
            ["ss", "-ltnp"],
            0,
            stdout=(
                "State Recv-Q Send-Q Local Address:Port Peer Address:Port Process\n"
                'LISTEN 0 128 127.0.0.1:8080 0.0.0.0:* users:(("unrelated",pid=5151,fd=3))\n'
            ),
            stderr="",
        )
        with (
            mock.patch.object(llm.shutil, "which", return_value="/usr/bin/ss"),
            mock.patch.object(llm, "_run_command", return_value=ss_result),
            mock.patch.object(
                llm.os,
                "getpgid",
                side_effect=lambda pid: {process.pid: process.pid, 5151: 5151}[pid],
            ),
        ):
            with self.assertRaises(llm.ListenerOwnershipError):
                llm._check_loopback_listener("127.0.0.1", 8080, process)


class LlmDownloadAndExtractionTest(unittest.TestCase):
    def test_download_verifies_size_and_hash_before_publish(self) -> None:
        payload = b"tiny fixture"
        expected_hash = hashlib.sha256(payload).hexdigest()
        with tempfile.TemporaryDirectory() as temporary_directory:
            destination = Path(temporary_directory) / "model.bin"
            llm.download_verified(
                "https://example.test/model.bin",
                destination,
                len(payload),
                expected_hash,
                opener=_Opener(payload),
            )
            self.assertEqual(destination.read_bytes(), payload)
            self.assertFalse(destination.with_name("model.bin.part").exists())

            wrong = Path(temporary_directory) / "wrong.bin"
            with self.assertRaises(llm.ArtifactError):
                llm.download_verified(
                    "https://example.test/model.bin",
                    wrong,
                    len(payload) + 1,
                    expected_hash,
                    opener=_Opener(payload),
                )
            self.assertFalse(wrong.exists())
            self.assertFalse(wrong.with_name("wrong.bin.part").exists())

            same_size_wrong_hash = Path(temporary_directory) / "same-size-wrong-hash.bin"
            with self.assertRaises(llm.ArtifactError):
                llm.download_verified(
                    "https://example.test/model.bin",
                    same_size_wrong_hash,
                    len(payload),
                    hashlib.sha256(b"different payload").hexdigest(),
                    opener=_Opener(payload),
                )
            self.assertFalse(same_size_wrong_hash.exists())
            self.assertFalse(same_size_wrong_hash.with_name("same-size-wrong-hash.bin.part").exists())

    def test_incomplete_http_read_uses_bounded_retry_and_cleans_part(self) -> None:
        payload = b"retry fixture"
        expected_hash = hashlib.sha256(payload).hexdigest()

        class FlakyOpener:
            calls = 0

            def open(self, url: str, timeout: float) -> _Response:
                self.calls += 1
                if self.calls < 3:
                    raise IncompleteRead(b"partial", 17)
                return _Response(payload)

        with tempfile.TemporaryDirectory() as temporary_directory:
            destination = Path(temporary_directory) / "model.bin"
            opener = FlakyOpener()
            llm.download_verified(
                "https://example.test/model.bin",
                destination,
                len(payload),
                expected_hash,
                opener=opener,
                retries=3,
            )
            self.assertEqual(opener.calls, 3)
            self.assertEqual(destination.read_bytes(), payload)
            self.assertFalse(destination.with_name("model.bin.part").exists())

    def test_missing_bundle_verification_does_not_create_a_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "llm"
            bundles = root / "bundles"
            bundles.mkdir(parents=True)
            manifest = llm.load_lock_manifest()
            digest = llm.lock_sha256()
            missing_bundle = bundles / digest

            with self.assertRaises(llm.ArtifactError):
                llm.verify_bundle(root, manifest, digest)
            self.assertFalse(missing_bundle.exists())

    def test_redirect_handler_rejects_non_https(self) -> None:
        handler = llm._HttpsRedirectHandler()
        request = llm.Request("https://example.test/source")
        with self.assertRaises(llm.ManifestError):
            handler.redirect_request(request, None, 302, "redirect", {}, "http://example.test/source")

    def test_safe_extraction_rejects_links_and_path_traversal(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            archive = root / "bad.tar.gz"
            with tarfile.open(archive, "w:gz") as tar:
                link = tarfile.TarInfo("llama.cpp/link")
                link.type = tarfile.SYMTYPE
                link.linkname = "/etc/passwd"
                tar.addfile(link)
            with self.assertRaises(llm.ArtifactError):
                llm.safe_extract_tar(archive, root / "extract-link")

            traversal = root / "traversal.tar.gz"
            with tarfile.open(traversal, "w:gz") as tar:
                entry = tarfile.TarInfo("llama.cpp/../escape")
                entry.size = 1
                tar.addfile(entry, io.BytesIO(b"x"))
            with self.assertRaises(llm.ArtifactError):
                llm.safe_extract_tar(traversal, root / "extract-traversal")

    def test_safe_extraction_requires_one_top_level_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            archive = root / "multi.tar.gz"
            with tarfile.open(archive, "w:gz") as tar:
                for name in ("one/file", "two/file"):
                    entry = tarfile.TarInfo(name)
                    entry.size = 1
                    tar.addfile(entry, io.BytesIO(b"x"))
            with self.assertRaises(llm.ArtifactError):
                llm.safe_extract_tar(archive, root / "extract")

    def test_safe_extraction_rejects_hardlink_device_and_fifo(self) -> None:
        fixtures = (
            ("hardlink", tarfile.LNKTYPE),
            ("device", tarfile.CHRTYPE),
            ("fifo", tarfile.FIFOTYPE),
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for name, member_type in fixtures:
                archive = root / f"{name}.tar"
                with tarfile.open(archive, "w") as tar:
                    member = tarfile.TarInfo(f"llama.cpp/{name}")
                    member.type = member_type
                    member.linkname = "llama.cpp/target"
                    tar.addfile(member)
                with self.assertRaises(llm.ArtifactError):
                    llm.safe_extract_tar(archive, root / f"extract-{name}")


def _fixture_manifest(source_payload: bytes, model_payload: bytes) -> llm.LockManifest:
    document = copy.deepcopy(llm.APPROVED_DOCUMENT)
    document["model"]["size"] = len(model_payload)
    document["model"]["sha256"] = hashlib.sha256(model_payload).hexdigest()
    document["model"]["download_url"] = "https://fixture.test/model.gguf"
    document["model"]["source_url"] = "https://fixture.test/model-source"
    document["model"]["license_url"] = "https://fixture.test/model-license"
    document["llama_cpp"]["source_size"] = len(source_payload)
    document["llama_cpp"]["source_sha256"] = hashlib.sha256(source_payload).hexdigest()
    document["llama_cpp"]["source_url"] = "https://fixture.test/llama.tar.gz"
    document["llama_cpp"]["license_url"] = "https://fixture.test/llama-license"
    return llm.validate_manifest_document(document)


class ProvisioningFixtureTest(unittest.TestCase):
    def _source_archive(self) -> bytes:
        output = io.BytesIO()
        with tarfile.open(fileobj=output, mode="w:gz") as tar:
            directory = tarfile.TarInfo("llama.cpp-fixture")
            directory.type = tarfile.DIRTYPE
            directory.mode = 0o755
            tar.addfile(directory)
            cmake = tarfile.TarInfo("llama.cpp-fixture/CMakeLists.txt")
            cmake.size = 12
            tar.addfile(cmake, io.BytesIO(b"project(test)\n"))
        return output.getvalue()

    def _provisioner(self, root: Path, source: bytes, model: bytes) -> tuple[llm.Provisioner, str]:
        manifest = _fixture_manifest(source, model)
        digest = "a" * 64

        def downloader(url: str, destination: Path, expected_size: int, expected_sha256: str) -> None:
            payload = source if url == manifest.llama_cpp["source_url"] else model
            self.assertEqual(len(payload), expected_size)
            self.assertEqual(hashlib.sha256(payload).hexdigest(), expected_sha256)
            destination.write_bytes(payload)

        test_case = self

        class FixtureBuilder:
            def build(self, source_root: Path, build_root: Path, output_path: Path) -> llm.BuildResult:
                test_case.assertTrue((source_root / "CMakeLists.txt").is_file())
                build_root.mkdir(parents=True)
                output_path.write_bytes(b"fixture-server")
                output_path.chmod(0o755)
                return llm.BuildResult("fixture-server 1", "fixture-compiler 1", "cmake 1")

        def publisher(staging: Path, destination: Path) -> None:
            if os.path.lexists(str(destination)):
                raise llm.ArtifactError("destination already exists")
            staging.rename(destination)

        provisioner = llm.Provisioner(
            root,
            manifest,
            digest,
            downloader=downloader,
            builder=FixtureBuilder(),
            publisher=publisher,
            fsync=lambda path: None,
            fsync_directory=lambda path: None,
            lock_factory=lambda path: llm._NoopLock(),
            check_existing_server=False,
        )
        return provisioner, digest

    def test_provisioning_publishes_complete_bundle_and_is_idempotent(self) -> None:
        source = self._source_archive()
        model = b"fixture-model"
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "llm"
            provisioner, digest = self._provisioner(root, source, model)
            first = provisioner.provision()
            self.assertFalse(first.idempotent)
            self.assertTrue((first.bundle / "bin" / "llama-server").is_file())
            self.assertEqual((first.bundle / "models" / provisioner.manifest.model_file).read_bytes(), model)
            self.assertTrue((first.bundle / "provenance.json").is_file())
            self.assertEqual(
                {entry.name for entry in (root / "bundles").iterdir()},
                {digest},
            )

            def should_not_download(*args) -> None:
                raise AssertionError("idempotent provisioning downloaded again")

            second = llm.Provisioner(
                root,
                provisioner.manifest,
                digest,
                downloader=should_not_download,
                builder=provisioner.builder,
                publisher=provisioner.publisher,
                fsync=lambda path: None,
                fsync_directory=lambda path: None,
                lock_factory=lambda path: llm._NoopLock(),
                check_existing_server=False,
            ).provision()
            self.assertTrue(second.idempotent)

    def test_invalid_existing_bundle_is_never_replaced(self) -> None:
        source = self._source_archive()
        model = b"fixture-model"
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "llm"
            provisioner, digest = self._provisioner(root, source, model)
            (root / "bundles" / digest).mkdir(parents=True)
            marker = root / "bundles" / digest / "marker"
            marker.write_text("old", encoding="utf-8")
            with self.assertRaises(llm.ArtifactError):
                provisioner.provision()
            self.assertEqual(marker.read_text(encoding="utf-8"), "old")
            self.assertFalse(any(path.name.startswith(".staging-") for path in root.iterdir()))

    def test_partial_download_failure_leaves_no_final_bundle(self) -> None:
        source = self._source_archive()
        model = b"fixture-model"
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "llm"
            provisioner, digest = self._provisioner(root, source, model)
            calls = 0

            def fail_on_model(url: str, destination: Path, expected_size: int, expected_sha256: str) -> None:
                nonlocal calls
                calls += 1
                if calls == 2:
                    destination.write_bytes(b"truncated")
                    raise llm.ArtifactError("fixture download failed")
                destination.write_bytes(source)

            failing = llm.Provisioner(
                root,
                provisioner.manifest,
                digest,
                downloader=fail_on_model,
                builder=provisioner.builder,
                publisher=provisioner.publisher,
                fsync=lambda path: None,
                fsync_directory=lambda path: None,
                lock_factory=lambda path: llm._NoopLock(),
                check_existing_server=False,
            )
            with self.assertRaises(llm.ArtifactError):
                failing.provision()
            self.assertFalse((root / "bundles" / digest).exists())
            self.assertFalse(any(path.name.startswith(".staging-") for path in root.iterdir()))

    def test_builder_failure_leaves_no_final_bundle_or_staging(self) -> None:
        source = self._source_archive()
        model = b"fixture-model"
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "llm"
            provisioner, digest = self._provisioner(root, source, model)

            class FailingBuilder:
                def build(self, source_root: Path, build_root: Path, output_path: Path) -> llm.BuildResult:
                    raise llm.ArtifactError("fixture build failed")

            failing = llm.Provisioner(
                root,
                provisioner.manifest,
                digest,
                downloader=provisioner.downloader,
                builder=FailingBuilder(),
                publisher=provisioner.publisher,
                fsync=lambda path: None,
                fsync_directory=lambda path: None,
                lock_factory=lambda path: llm._NoopLock(),
                check_existing_server=False,
            )
            with self.assertRaises(llm.ArtifactError) as context:
                failing.provision()

            self.assertIn("fixture build failed", str(context.exception))
            self.assertFalse((root / "bundles" / digest).exists())
            self.assertFalse(any(path.name.startswith(".staging-") for path in root.iterdir()))

    def test_fixture_adapter_output_is_rechecked_before_build(self) -> None:
        source = self._source_archive()
        model = b"fixture-model"
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "llm"
            provisioner, digest = self._provisioner(root, source, model)

            def wrong_model(url: str, destination: Path, expected_size: int, expected_sha256: str) -> None:
                destination.write_bytes(source if url == provisioner.manifest.llama_cpp["source_url"] else b"wrong")

            failing = llm.Provisioner(
                root,
                provisioner.manifest,
                digest,
                downloader=wrong_model,
                builder=provisioner.builder,
                publisher=provisioner.publisher,
                fsync=lambda path: None,
                fsync_directory=lambda path: None,
                lock_factory=lambda path: llm._NoopLock(),
                check_existing_server=False,
            )
            with self.assertRaises(llm.ArtifactError):
                failing.provision()
            self.assertFalse((root / "bundles" / digest).exists())

    def test_post_publish_fsync_failure_removes_new_bundle(self) -> None:
        source = self._source_archive()
        model = b"fixture-model"
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "llm"
            provisioner, digest = self._provisioner(root, source, model)
            fsync_calls = 0

            def fail_final_directory(path: Path) -> None:
                nonlocal fsync_calls
                fsync_calls += 1
                if fsync_calls == 4:
                    raise llm.ArtifactError("fixture directory fsync failed")

            failing = llm.Provisioner(
                root,
                provisioner.manifest,
                digest,
                downloader=provisioner.downloader,
                builder=provisioner.builder,
                publisher=provisioner.publisher,
                fsync=lambda path: None,
                fsync_directory=fail_final_directory,
                lock_factory=lambda path: llm._NoopLock(),
                check_existing_server=False,
            )
            with self.assertRaises(llm.ArtifactError):
                failing.provision()
            self.assertFalse((root / "bundles" / digest).exists())

    def test_post_publish_cleanup_failure_reports_stranded_publication(self) -> None:
        source = self._source_archive()
        model = b"fixture-model"
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "llm"
            provisioner, digest = self._provisioner(root, source, model)
            final_bundle = root / "bundles" / digest
            fsync_calls = 0

            def fail_persistence(path: Path) -> None:
                nonlocal fsync_calls
                fsync_calls += 1
                if fsync_calls in {4, 5}:
                    raise llm.ArtifactError("fixture directory fsync failed")

            original_remove = llm._remove_path

            def fail_final_cleanup(path: Path) -> None:
                if path == final_bundle:
                    raise llm.ArtifactError("fixture final cleanup failed")
                original_remove(path)

            failing = llm.Provisioner(
                root,
                provisioner.manifest,
                digest,
                downloader=provisioner.downloader,
                builder=provisioner.builder,
                publisher=provisioner.publisher,
                fsync=lambda path: None,
                fsync_directory=fail_persistence,
                lock_factory=lambda path: llm._NoopLock(),
                check_existing_server=False,
            )
            with mock.patch.object(llm, "_remove_path", side_effect=fail_final_cleanup):
                with self.assertRaises(llm.ArtifactError) as context:
                    failing.provision()

            self.assertIn("stranded publication", str(context.exception))
            self.assertTrue(final_bundle.exists())


@unittest.skipUnless(os.name == "posix", "renameat2 is a Linux provisioning seam")
class AtomicPublicationTest(unittest.TestCase):
    def test_no_replace_does_not_overwrite_existing_bundle(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "staging"
            destination = root / "bundle"
            source.mkdir()
            destination.mkdir()
            (destination / "marker").write_text("old", encoding="utf-8")
            with self.assertRaises(llm.ArtifactError):
                llm.atomic_publish(source, destination)
            self.assertTrue(source.exists())
            self.assertEqual((destination / "marker").read_text(encoding="utf-8"), "old")


@unittest.skipUnless(os.name == "posix", "Linux artifact-root safety seam")
class ArtifactRootSafetyTest(unittest.TestCase):
    def test_mnt_root_is_rejected_before_creation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            candidate = Path("/mnt") / "voice-nav-test-root" / Path(temporary_directory).name
            with self.assertRaises(llm.ArtifactError):
                llm.validate_artifact_root(candidate, create=True)
            self.assertFalse(candidate.exists())

    def test_root_symlink_ownership_and_mode_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            parent = Path(temporary_directory)
            target = parent / "target"
            target.mkdir(mode=0o700)
            target.chmod(0o700)
            link = parent / "link"
            link.symlink_to(target, target_is_directory=True)

            with self.assertRaises(llm.ArtifactError):
                llm.validate_artifact_root(link, create=False)

            target.chmod(0o777)
            with self.assertRaises(llm.ArtifactError):
                llm.validate_artifact_root(target, create=False)

            target.chmod(0o700)
            with mock.patch.object(llm.os, "geteuid", return_value=os.geteuid() + 1):
                with self.assertRaises(llm.ArtifactError):
                    llm.validate_artifact_root(target, create=False)


@unittest.skipUnless(os.name == "posix", "shell wrapper behavior is a Linux/WSL seam")
class WrapperCliTest(unittest.TestCase):
    def _run_wrapper(self, script: str, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["bash", str(REPOSITORY_ROOT / "scripts" / "llm" / script), *arguments],
            cwd=REPOSITORY_ROOT,
            check=False,
            capture_output=True,
            text=True,
        )

    def test_verify_wrapper_reports_manifest_and_not_run_real_gate(self) -> None:
        completed = self._run_wrapper("verify.sh")

        self.assertEqual(completed.returncode, 0)
        self.assertIn("MANIFEST_GATE=PASS", completed.stdout)
        self.assertIn(
            "REAL_MODEL_GATE=NOT_RUN reason=--real-not-requested",
            completed.stdout,
        )

    def test_provision_wrapper_rejects_mnt_root_before_network(self) -> None:
        completed = self._run_wrapper(
            "provision.sh",
            "--root",
            "/mnt/c/voice-nav-issue-48-wrapper-fixture",
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("PROVISION=FAIL", completed.stderr)
        self.assertIn("mnt", completed.stderr.lower())


if __name__ == "__main__":
    unittest.main()
