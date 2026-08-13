import importlib.util
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path, PurePosixPath


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
MANAGER_PATH = REPOSITORY_ROOT / "scripts" / "voice_asset_manager.py"


def load_manager():
    spec = importlib.util.spec_from_file_location("voice_asset_manager", MANAGER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load voice asset manager")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


voice_assets = load_manager()


class VoiceAssetManifestTest(unittest.TestCase):
    def test_frozen_manifests_cover_the_approved_voice_assets(self) -> None:
        dependencies, models = voice_assets.load_manifests(REPOSITORY_ROOT)

        self.assertEqual(dependencies.schema_version, 1)
        self.assertEqual(models.schema_version, 1)
        self.assertEqual(
            {asset.identifier for asset in dependencies.assets},
            {
                "abseil-cpp",
                "abseil-cpp-meson-patch",
                "portaudio",
                "sherpa-onnx",
                "webrtc-audio-processing",
            },
        )
        self.assertEqual(
            {asset.identifier for asset in models.assets},
            {
                "asr-zh-int8-2025-06-30",
                "kws-zh-en-3m-2025-12-20",
                "tts-chaowen-medium-int8",
                "vad-silero-int8",
            },
        )
        assets = {asset.identifier: asset for asset in (*dependencies.assets, *models.assets)}
        self.assertEqual(assets["portaudio"].revision, "147dd722548358763a8b649b3e4b41dfffbcfbb6")
        self.assertEqual(assets["webrtc-audio-processing"].version, "2.1")
        self.assertEqual(assets["abseil-cpp"].version, "20240722.0")
        self.assertEqual(assets["sherpa-onnx"].revision, "142807252687d81b40d6315f23470a1512a00de3")
        self.assertEqual(assets["kws-zh-en-3m-2025-12-20"].sha256, "68447f4fbc67e70eee3a93961f36e81e98f47aef73ce7e7ca00885c6cd3616a6")
        self.assertEqual(assets["vad-silero-int8"].sha256, "c36d490aff5ab924ca6c7aeec4d8f6bd3d22db6fa17611b9c5b17eae58ac3a20")
        self.assertEqual(assets["asr-zh-int8-2025-06-30"].sha256, "5a2832047ea1f97dd0dc595b816c230c4bafad65cfc0341fa57517cadc50afd0")
        self.assertEqual(assets["tts-chaowen-medium-int8"].sha256, "f5f7c8628427fbb259ea4b7ec1a9a822a0c04e3f267071f0abfa0610371d9e0c")
        self.assertEqual(assets["tts-chaowen-medium-int8"].revision, "406468505")
        self.assertEqual(
            assets["tts-chaowen-medium-int8"].url,
            "https://api.github.com/repos/k2-fsa/sherpa-onnx/releases/assets/406468505",
        )
        self.assertEqual(assets["tts-chaowen-medium-int8"].size, 14011298)

    def test_manifest_rejects_floating_or_incomplete_asset_identity(self) -> None:
        valid_asset = {
            "id": "fixture",
            "version": "fixture-v1",
            "revision": "fixture-r1",
            "url": "https://fixtures.invalid/releases/fixture-r1.tar.gz",
            "size": 1,
            "sha256": "a" * 64,
            "license": "Apache-2.0",
            "license_url": "https://fixtures.invalid/licenses/fixture-r1",
            "destination": "fixture/fixture.tar.gz",
            "build_options": {"system": "fixture"},
        }
        mutations = {
            "floating": {**valid_asset, "url": "https://fixtures.invalid/releases/latest.tar.gz"},
            "empty-checksum": {**valid_asset, "sha256": ""},
            "missing-license": {key: value for key, value in valid_asset.items() if key != "license"},
            "empty-build-options": {**valid_asset, "build_options": {}},
            "path-traversal": {**valid_asset, "destination": "fixture/../escape.tar.gz"},
        }
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "fixture.yaml"
            for label, asset in mutations.items():
                with self.subTest(label=label):
                    path.write_text(json.dumps({"schema_version": 1, "assets": [asset]}), encoding="utf-8")
                    with self.assertRaises(voice_assets.ManifestError):
                        voice_assets.load_manifest(path, "dependencies")

            path.write_text(
                json.dumps({"schema_version": 1, "assets": [valid_asset, valid_asset]}),
                encoding="utf-8",
            )
            with self.assertRaises(voice_assets.ManifestError):
                voice_assets.load_manifest(path, "dependencies")

    def test_notices_and_voice_architecture_record_the_locked_offline_boundary(self) -> None:
        dependencies, models = voice_assets.load_manifests(REPOSITORY_ROOT)
        notices = (REPOSITORY_ROOT / "THIRD_PARTY_NOTICES.md").read_text(encoding="utf-8")
        architecture = (REPOSITORY_ROOT / "docs" / "architecture" / "voice-and-agent.md").read_text(encoding="utf-8")

        for asset in (*dependencies.assets, *models.assets):
            with self.subTest(asset=asset.identifier):
                self.assertIn(asset.identifier, notices)
                self.assertIn(asset.version, notices)
                self.assertIn(asset.sha256, notices)
                self.assertIn(asset.license, notices)
        self.assertIn("Xiao Ya", notices)
        self.assertIn("非商业", notices)
        self.assertIn("third_party/locks/audio-dependencies.yaml", architecture)
        self.assertIn("models/manifests/voice-models.yaml", architecture)
        self.assertIn("scripts/provision_voice_assets.sh --verify", architecture)
        self.assertIn("运行时不得下载", architecture)
        self.assertNotIn("14M Chinese Streaming Zipformer", architecture)
        self.assertNotIn("自动选", architecture)
        self.assertIn("ASR：唯一冻结 `2025-06-30 int8`", architecture)
        self.assertIn("TTS：`vits-piper-zh_CN-chaowen-medium-int8.tar.bz2`", architecture)


class VoiceAssetProvisioningTest(unittest.TestCase):
    def test_first_install_publishes_only_verified_assets_and_is_idempotent(self) -> None:
        payloads = {
            "dependency": b"locked dependency fixture\n",
            "model": b"locked model fixture\n",
        }

        def asset(identifier: str, destination: str, sample_rate: int | None = None):
            payload = payloads[identifier]
            return voice_assets.Asset(
                identifier=identifier,
                version="fixture-v1",
                revision="fixture-revision",
                url=f"https://fixtures.invalid/{identifier}",
                size=len(payload),
                sha256=hashlib.sha256(payload).hexdigest(),
                license="Apache-2.0",
                license_url="https://fixtures.invalid/license",
                destination=PurePosixPath(destination),
                build_options={"fixture": "ON"},
                sample_rate=sample_rate,
                model_card_url=("https://fixtures.invalid/model-card" if sample_rate else None),
            )

        dependencies = voice_assets.AssetManifest(
            schema_version=1,
            category="dependencies",
            assets=(asset("dependency", "dependency/source.tar.gz"),),
        )
        models = voice_assets.AssetManifest(
            schema_version=1,
            category="models",
            assets=(asset("model", "model/weights.tar.bz2", sample_rate=16000),),
        )
        download_calls: list[str] = []

        def downloader(source: voice_assets.Asset, destination: Path) -> None:
            download_calls.append(source.url)
            destination.write_bytes(payloads[source.identifier])

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            provisioner = voice_assets.Provisioner(
                dependencies,
                models,
                dependency_root=root / ".deps" / "voice-assets",
                model_root=root / "models" / "weights" / "voice-assets",
                downloader=downloader,
            )

            first = provisioner.provision()
            second = provisioner.provision(offline=True)

            self.assertEqual(first.installed, ("dependency", "model"))
            self.assertEqual(first.reused, ())
            self.assertEqual(second.installed, ())
            self.assertEqual(second.reused, ("dependency", "model"))
            self.assertEqual(download_calls, [
                "https://fixtures.invalid/dependency",
                "https://fixtures.invalid/model",
            ])
            self.assertEqual(
                (root / ".deps" / "voice-assets" / "dependency" / "source.tar.gz").read_bytes(),
                payloads["dependency"],
            )
            self.assertEqual(
                (root / "models" / "weights" / "voice-assets" / "model" / "weights.tar.bz2").read_bytes(),
                payloads["model"],
            )
            dependency_path = root / ".deps" / "voice-assets" / "dependency" / "source.tar.gz"
            self.assertEqual(provisioner.verified_path("dependency"), dependency_path)
            dependency_path.write_bytes(b"tampered")
            with self.assertRaises(voice_assets.ProvisionError):
                provisioner.verified_path("dependency")

    def test_bad_or_missing_assets_fail_closed_without_publishing_a_partial_file(self) -> None:
        expected = b"complete fixture\n"
        asset = voice_assets.Asset(
            identifier="fixture",
            version="fixture-v1",
            revision="fixture-r1",
            url="https://fixtures.invalid/fixture",
            size=len(expected),
            sha256=hashlib.sha256(expected).hexdigest(),
            license="Apache-2.0",
            license_url="https://fixtures.invalid/license",
            destination=PurePosixPath("fixture/fixture.tar.gz"),
            build_options={"system": "fixture"},
        )
        dependencies = voice_assets.AssetManifest(1, (asset,), "dependencies")
        models = voice_assets.AssetManifest(1, (), "models")

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            target = root / ".deps" / "voice-assets" / "fixture" / "fixture.tar.gz"
            provisioner = voice_assets.Provisioner(
                dependencies,
                models,
                dependency_root=root / ".deps" / "voice-assets",
                model_root=root / "models" / "weights" / "voice-assets",
                downloader=lambda _url, destination: destination.write_bytes(b"truncated"),
            )

            with self.assertRaises(voice_assets.ProvisionError):
                provisioner.provision()
            self.assertFalse(target.exists())
            self.assertEqual(list(target.parent.glob("*.partial")), [])

            wrong_hash = voice_assets.Provisioner(
                dependencies,
                models,
                dependency_root=root / ".deps" / "voice-assets",
                model_root=root / "models" / "weights" / "voice-assets",
                downloader=lambda _url, destination: destination.write_bytes(b"x" * len(expected)),
            )
            with self.assertRaises(voice_assets.ProvisionError):
                wrong_hash.provision()
            self.assertFalse(target.exists())
            self.assertEqual(list(target.parent.glob("*.partial")), [])

            offline = voice_assets.Provisioner(
                dependencies,
                models,
                dependency_root=root / ".deps" / "voice-assets",
                model_root=root / "models" / "weights" / "voice-assets",
                downloader=lambda _url, _destination: self.fail("offline must not download"),
            )
            with self.assertRaises(voice_assets.ProvisionError):
                offline.provision(offline=True)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.with_name(".fixture.tar.gz.interrupted.partial").write_bytes(expected)
            with self.assertRaises(voice_assets.ProvisionError):
                offline.verify()


class VoiceAssetWrapperTest(unittest.TestCase):
    def test_explicit_provision_wrapper_exposes_an_offline_verify_mode(self) -> None:
        completed = subprocess.run(
            ["bash", "scripts/provision_voice_assets.sh", "--help"],
            cwd=REPOSITORY_ROOT,
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--verify", completed.stdout)
        self.assertIn("--offline", completed.stdout)


class VoiceAssetReviewContractTest(unittest.TestCase):
    def test_every_locked_asset_has_exact_identity_options_and_license_status(self) -> None:
        dependencies, models = voice_assets.load_manifests(REPOSITORY_ROOT)

        def contract(asset: voice_assets.Asset) -> tuple[object, ...]:
            provenance = asset.model_provenance
            return (
                asset.version, asset.revision, asset.url, asset.size, asset.sha256,
                asset.destination.as_posix(), dict(asset.build_options), asset.license,
                None if provenance is None else (
                    provenance.status, provenance.weights_license, provenance.weights_license_url,
                    provenance.training_data_provenance, provenance.training_data_url, provenance.model_card_url,
                ),
                asset.release_asset_id,
            )

        self.assertEqual({asset.identifier: contract(asset) for asset in (*dependencies.assets, *models.assets)}, {
            "portaudio": ("v19.7.0", "147dd722548358763a8b649b3e4b41dfffbcfbb6", "https://codeload.github.com/PortAudio/portaudio/tar.gz/147dd722548358763a8b649b3e4b41dfffbcfbb6", 1463329, "95457b809ce60d4d4790f84bb692e271f644e59d8adf96feb988c89ab52a506a", "portaudio/portaudio-147dd722548358763a8b649b3e4b41dfffbcfbb6.tar.gz", {"system": "CMake", "PA_BUILD_SHARED": "OFF", "PA_BUILD_TESTS": "OFF"}, "MIT", None, None),
            "webrtc-audio-processing": ("2.1", "2.1", "https://gstreamer.freedesktop.org/src/mirror/webrtc-audio-processing/webrtc-audio-processing-2.1.tar.xz", 604192, "ae9302824b2038d394f10213cab05312c564a038434269f11dbf68f511f9f9fe", "webrtc-audio-processing/webrtc-audio-processing-2.1.tar.xz", {"system": "Meson", "default_library": "static", "tests": "disabled"}, "BSD-3-Clause", None, None),
            "abseil-cpp": ("20240722.0", "20240722.0", "https://github.com/abseil/abseil-cpp/releases/download/20240722.0/abseil-cpp-20240722.0.tar.gz", 2242861, "f50e5ac311a81382da7fa75b97310e4b9006474f9560ac46f54a9967f07d4ae3", "abseil-cpp/abseil-cpp-20240722.0.tar.gz", {"system": "CMake", "BUILD_SHARED_LIBS": "OFF", "ABSL_BUILD_TESTING": "OFF"}, "Apache-2.0", None, None),
            "abseil-cpp-meson-patch": ("20240722.0-3", "20240722.0-3", "https://wrapdb.mesonbuild.com/v2/abseil-cpp_20240722.0-3/get_patch", 5929, "12dd8df1488a314c53e3751abd2750cf233b830651d168b6a9f15e7d0cf71f7b", "abseil-cpp/abseil-cpp_20240722.0-3_patch.zip", {"system": "Meson", "wrap": "20240722.0-3"}, "Apache-2.0", None, None),
            "sherpa-onnx": ("v1.13.4", "142807252687d81b40d6315f23470a1512a00de3", "https://codeload.github.com/k2-fsa/sherpa-onnx/tar.gz/142807252687d81b40d6315f23470a1512a00de3", 9840362, "f0dc7c9b41b8691313daee671e826eb23946fa1320559a8d37e84f8774af76b2", "sherpa-onnx/sherpa-onnx-142807252687d81b40d6315f23470a1512a00de3.tar.gz", {"system": "CMake", "BUILD_SHARED_LIBS": "OFF", "SHERPA_ONNX_ENABLE_TESTS": "OFF"}, "Apache-2.0", None, None),
            "kws-zh-en-3m-2025-12-20": ("sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20", "2025-12-20", "https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2", 32885699, "68447f4fbc67e70eee3a93961f36e81e98f47aef73ce7e7ca00885c6cd3616a6", "kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2", {"runtime": "sherpa-onnx", "provider": "cpu"}, "Apache-2.0", ("unresolved", None, None, "未找到可核验的 KWS 权重或训练数据权威许可；sherpa-onnx Apache-2.0 仅覆盖运行时框架。", None, "https://k2-fsa.github.io/sherpa/onnx/kws/pretrained_models/index.html"), None),
            "vad-silero-int8": ("silero_vad.int8.onnx", "silero-vad-int8", "https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/silero_vad.int8.onnx", 212860, "c36d490aff5ab924ca6c7aeec4d8f6bd3d22db6fa17611b9c5b17eae58ac3a20", "vad/silero_vad.int8.onnx", {"runtime": "sherpa-onnx", "provider": "cpu"}, "Apache-2.0", ("resolved", "MIT", "https://github.com/snakers4/silero-vad/blob/v5.1.2/LICENSE", "Silero VAD 上游 MIT 模型仓库与模型卡是权重及训练来源的可追溯出处；sherpa-onnx Apache-2.0 不替代该模型许可证。", "https://github.com/snakers4/silero-vad/tree/v5.1.2", "https://k2-fsa.github.io/sherpa/onnx/vad/silero-vad.html"), None),
            "asr-zh-int8-2025-06-30": ("sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30", "2025-06-30", "https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30.tar.bz2", 132634597, "5a2832047ea1f97dd0dc595b816c230c4bafad65cfc0341fa57517cadc50afd0", "asr/sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30.tar.bz2", {"runtime": "sherpa-onnx", "provider": "cpu", "streaming": "ON"}, "Apache-2.0", ("unresolved", None, None, "未找到可核验的 ASR 权重及训练数据权威许可；sherpa-onnx Apache-2.0 仅覆盖运行时框架。", None, "https://k2-fsa.github.io/sherpa/onnx/pretrained_models/online-transducer/zipformer-transducer-models.html"), None),
            "tts-chaowen-medium-int8": ("vits-piper-zh_CN-chaowen-medium-int8.tar.bz2", "406468505", "https://api.github.com/repos/k2-fsa/sherpa-onnx/releases/assets/406468505", 14011298, "f5f7c8628427fbb259ea4b7ec1a9a822a0c04e3f267071f0abfa0610371d9e0c", "tts/vits-piper-zh_CN-chaowen-medium-int8.tar.bz2", {"runtime": "sherpa-onnx", "provider": "cpu", "quantization": "int8"}, "Apache-2.0", ("restricted", "Xiao Ya/BZNSYP upstream data restricted to non-commercial use", "https://k2-fsa.github.io/sherpa/onnx/tts/all/Chinese/vits-piper-zh_CN-chaowen-medium.html", "Chaowen 模型卡记录 Xiao Ya 基座与 BZNSYP 训练数据链；上游限制为非商业使用。", "https://k2-fsa.github.io/sherpa/onnx/tts/all/Chinese/vits-piper-zh_CN-chaowen-medium.html", "https://k2-fsa.github.io/sherpa/onnx/tts/all/Chinese/vits-piper-zh_CN-chaowen-medium.html"), 406468505),
        })
        self.assertEqual(
            {
                asset.identifier: (asset.sample_rate, asset.license_url, asset.runtime_license_url)
                for asset in (*dependencies.assets, *models.assets)
            },
            {
                "portaudio": (None, "https://github.com/PortAudio/portaudio/blob/147dd722548358763a8b649b3e4b41dfffbcfbb6/LICENSE.txt", None),
                "webrtc-audio-processing": (None, "https://cgit.freedesktop.org/pulseaudio/webrtc-audio-processing/tree/COPYING?h=2.1", None),
                "abseil-cpp": (None, "https://github.com/abseil/abseil-cpp/blob/20240722.0/LICENSE", None),
                "abseil-cpp-meson-patch": (None, "https://wrapdb.mesonbuild.com/v2/abseil-cpp_20240722.0-3/get_patch", None),
                "sherpa-onnx": (None, "https://github.com/k2-fsa/sherpa-onnx/blob/142807252687d81b40d6315f23470a1512a00de3/LICENSE", None),
                "kws-zh-en-3m-2025-12-20": (16000, "https://github.com/k2-fsa/sherpa-onnx/blob/142807252687d81b40d6315f23470a1512a00de3/LICENSE", "https://github.com/k2-fsa/sherpa-onnx/blob/142807252687d81b40d6315f23470a1512a00de3/LICENSE"),
                "vad-silero-int8": (16000, "https://github.com/k2-fsa/sherpa-onnx/blob/142807252687d81b40d6315f23470a1512a00de3/LICENSE", "https://github.com/k2-fsa/sherpa-onnx/blob/142807252687d81b40d6315f23470a1512a00de3/LICENSE"),
                "asr-zh-int8-2025-06-30": (16000, "https://github.com/k2-fsa/sherpa-onnx/blob/142807252687d81b40d6315f23470a1512a00de3/LICENSE", "https://github.com/k2-fsa/sherpa-onnx/blob/142807252687d81b40d6315f23470a1512a00de3/LICENSE"),
                "tts-chaowen-medium-int8": (22050, "https://github.com/k2-fsa/sherpa-onnx/blob/142807252687d81b40d6315f23470a1512a00de3/LICENSE", "https://github.com/k2-fsa/sherpa-onnx/blob/142807252687d81b40d6315f23470a1512a00de3/LICENSE"),
            },
        )

    def test_schema_rejects_floating_references_and_cross_manifest_collisions(self) -> None:
        dependency_document = json.loads((REPOSITORY_ROOT / "third_party" / "locks" / "audio-dependencies.yaml").read_text(encoding="utf-8"))
        model_document = json.loads((REPOSITORY_ROOT / "models" / "manifests" / "voice-models.yaml").read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            path = root / "fixture.yaml"
            for field, floating in (("version", "main"), ("revision", "refs/heads/master")):
                document = json.loads(json.dumps(dependency_document))
                document["assets"][0][field] = floating
                path.write_text(json.dumps(document), encoding="utf-8")
                with self.subTest(field=field), self.assertRaises(voice_assets.ManifestError):
                    voice_assets.load_manifest(path, "dependencies")
            for collision in ("id", "destination"):
                dependencies = json.loads(json.dumps(dependency_document))
                models = json.loads(json.dumps(model_document))
                models["assets"][0][collision] = dependencies["assets"][0][collision]
                dependency_path = root / "third_party" / "locks" / "audio-dependencies.yaml"
                model_path = root / "models" / "manifests" / "voice-models.yaml"
                dependency_path.parent.mkdir(parents=True, exist_ok=True)
                model_path.parent.mkdir(parents=True, exist_ok=True)
                dependency_path.write_text(json.dumps(dependencies), encoding="utf-8")
                model_path.write_text(json.dumps(models), encoding="utf-8")
                with self.subTest(collision=collision), self.assertRaises(voice_assets.ManifestError):
                    voice_assets.load_manifests(root)

    def test_release_asset_download_requests_octet_stream(self) -> None:
        _dependencies, models = voice_assets.load_manifests(REPOSITORY_ROOT)
        asset = next(asset for asset in models.assets if asset.identifier == "tts-chaowen-medium-int8")

        class Response:
            def __enter__(self): return self
            def __exit__(self, *_args): return False
            def read(self, _size: int) -> bytes:
                if getattr(self, "sent", False): return b""
                self.sent = True
                return b"fixture"

        with tempfile.TemporaryDirectory() as temporary_directory, mock.patch.object(voice_assets, "urlopen", return_value=Response()) as opened:
            destination = Path(temporary_directory) / "asset"
            voice_assets._download(asset, destination)
            request = opened.call_args.args[0]
            self.assertEqual(request.full_url, asset.url)
            self.assertEqual(request.get_header("Accept"), "application/octet-stream")
            self.assertEqual(destination.read_bytes(), b"fixture")

    def test_unresolved_model_license_preflights_provision_and_verify_before_side_effects(self) -> None:
        provenance = voice_assets.ModelProvenance("unresolved", None, None, "fixture unresolved", None, "https://fixtures.invalid/model-card")
        model = voice_assets.Asset("fixture-model", "fixture-v1", "fixture-r1", "https://fixtures.invalid/model", 1, hashlib.sha256(b"x").hexdigest(), "Apache-2.0", "https://fixtures.invalid/runtime-license", PurePosixPath("model/fixture.bin"), {"runtime": "fixture"}, 16000, provenance.model_card_url, "Apache-2.0", "https://fixtures.invalid/runtime-license", provenance)
        for operation in ("provision", "verify"):
            with self.subTest(operation=operation), tempfile.TemporaryDirectory() as temporary_directory:
                root = Path(temporary_directory)
                download_calls: list[str] = []

                def downloader(source: voice_assets.Asset, _destination: Path) -> None:
                    download_calls.append(source.identifier)
                    self.fail("unresolved provenance must not download")

                provisioner = voice_assets.Provisioner(
                    voice_assets.AssetManifest(1, (), "dependencies"),
                    voice_assets.AssetManifest(1, (model,), "models"),
                    dependency_root=root / ".deps" / "voice-assets",
                    model_root=root / "models" / "weights" / "voice-assets",
                    downloader=downloader,
                )
                with self.assertRaisesRegex(voice_assets.ProvisionError, "license provenance unresolved"):
                    getattr(provisioner, operation)()
                self.assertEqual(download_calls, [])
                self.assertFalse((root / ".deps").exists())
                self.assertFalse((root / "models").exists())
                self.assertEqual(list(root.rglob("*.partial")), [])


class VoiceAssetCliContractTest(unittest.TestCase):
    def _write_fixture_repository(self, root: Path, status: str) -> Path:
        subprocess.run(["git", "init", "-q", str(root)], check=True)
        (root / ".gitignore").write_text("/.deps/\n/models/weights/\n", encoding="utf-8")
        payload = b"fixture"
        digest = hashlib.sha256(payload).hexdigest()
        dependency = {"id": "fixture-dependency", "version": "fixture-v1", "revision": "fixture-r1", "url": "https://fixtures.invalid/dependency", "size": len(payload), "sha256": digest, "license": "MIT", "license_url": "https://fixtures.invalid/dependency-license", "destination": "dependency/fixture.bin", "build_options": {"system": "fixture"}}
        provenance = {"status": status, "weights_license": None if status == "unresolved" else "MIT", "weights_license_url": None if status == "unresolved" else "https://fixtures.invalid/model-license", "training_data_provenance": "fixture training-data provenance", "training_data_url": None if status == "unresolved" else "https://fixtures.invalid/training-data", "model_card_url": "https://fixtures.invalid/model-card"}
        model = {"id": "fixture-model", "version": "fixture-v1", "revision": "fixture-r1", "url": "https://fixtures.invalid/model", "size": len(payload), "sha256": digest, "destination": "model/fixture.bin", "build_options": {"runtime": "fixture"}, "sample_rate": 16000, "runtime_license": "Apache-2.0", "runtime_license_url": "https://fixtures.invalid/runtime-license", "model_provenance": provenance, "release_asset_id": None}
        dependency_path = root / "third_party" / "locks" / "audio-dependencies.yaml"
        model_path = root / "models" / "manifests" / "voice-models.yaml"
        dependency_path.parent.mkdir(parents=True)
        model_path.parent.mkdir(parents=True)
        dependency_path.write_text(json.dumps({"schema_version": 1, "assets": [dependency]}), encoding="utf-8")
        model_path.write_text(json.dumps({"schema_version": 1, "assets": [model]}), encoding="utf-8")
        return root / ".deps" / "voice-assets" / "dependency" / "fixture.bin"

    def _run(self, root: Path, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run([sys.executable, str(MANAGER_PATH), *arguments, "--repo-root", str(root)], cwd=REPOSITORY_ROOT, check=False, capture_output=True, text=True)

    def test_cli_fails_closed_for_unresolved_offline_missing_and_corrupt_assets(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "unresolved"; root.mkdir(); self._write_fixture_repository(root, "unresolved")
            result = self._run(root, "provision")
            self.assertEqual(result.returncode, 2); self.assertIn("license provenance unresolved", result.stdout); self.assertFalse((root / ".deps").exists())
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "offline"; root.mkdir(); self._write_fixture_repository(root, "resolved")
            result = self._run(root, "provision", "--offline")
            self.assertEqual(result.returncode, 2); self.assertIn("missing verified asset while offline", result.stdout); self.assertEqual(list(root.rglob("*.partial")), [])
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "corrupt"; root.mkdir(); target = self._write_fixture_repository(root, "resolved")
            target.parent.mkdir(parents=True); target.write_bytes(b"wrong!!")
            result = self._run(root, "verify")
            self.assertEqual(result.returncode, 2); self.assertIn("locked size and SHA-256", result.stdout)

    def test_cli_rejects_each_illegal_root_before_directory_or_download_side_effects(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "illegal"
            root.mkdir()
            self._write_fixture_repository(root, "resolved")
            for option in ("--dependency-root", "--model-root"):
                with self.subTest(option=option):
                    forbidden = root / f"unignored-{option[2:]}"
                    with mock.patch.object(
                        voice_assets,
                        "urlopen",
                        side_effect=AssertionError("illegal root must not download"),
                    ) as opened, mock.patch("sys.stdout"):
                        exit_code = voice_assets.main(
                            ("provision", "--repo-root", str(root), option, str(forbidden))
                        )
                    self.assertEqual(exit_code, 2)
                    opened.assert_not_called()
                    self.assertFalse(forbidden.exists())
                    self.assertFalse((root / ".deps" / "voice-assets").exists())
                    self.assertFalse((root / "models" / "weights" / "voice-assets").exists())
                    self.assertEqual(list(root.rglob("*.partial")), [])

    def test_cli_rejects_linked_approved_roots_before_directory_or_download_side_effects(self) -> None:
        roots = (
            ("dependency", Path(".deps") / "voice-assets"),
            ("model", Path("models") / "weights" / "voice-assets"),
        )
        for label, relative_root in roots:
            for operation in ("provision", "verify"):
                with self.subTest(root=label, operation=operation), tempfile.TemporaryDirectory() as temporary_directory:
                    root = Path(temporary_directory) / "linked"
                    external = Path(temporary_directory) / f"outside-{label}-{operation}"
                    root.mkdir()
                    dependency_target = self._write_fixture_repository(root, "resolved")
                    if label == "model":
                        dependency_target.parent.mkdir(parents=True)
                        dependency_target.write_bytes(b"fixture")
                    approved_root = root / relative_root
                    approved_root.parent.mkdir(parents=True)
                    try:
                        approved_root.symlink_to(external, target_is_directory=True)
                    except OSError as error:
                        self.skipTest(f"cannot create fixture symlink: {error}")

                    with mock.patch.object(
                        voice_assets,
                        "urlopen",
                        side_effect=AssertionError("linked approved root must not download"),
                    ) as opened, mock.patch("sys.stdout"):
                        exit_code = voice_assets.main(
                            (operation, "--repo-root", str(root))
                        )

                    self.assertEqual(exit_code, 2)
                    opened.assert_not_called()
                    self.assertFalse(external.exists())
                    self.assertEqual(list(root.rglob("*.partial")), [])


if __name__ == "__main__":
    unittest.main()
