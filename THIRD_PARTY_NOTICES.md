# Third-Party Notices

This file records third-party material redistributed in the source
repository. Runtime dependencies installed through the operating system or
rosdep are not copied into this repository and are governed by their own
packages.

## ROS package manifest schemas

Files:

- `tools/schema/package_format3.xsd`
- `tools/schema/package_common.xsd`

Provenance:

- upstream repository: `https://github.com/ros-infrastructure/rep`
- upstream commit: `11ca24a41f31480dfb9562ba99f2a5b93d3ebda5`
- upstream paths: `xsd/package_format3.xsd` and `xsd/package_common.xsd`
- associated specification: REP-149, Package Manifest Format Three

Licensing basis:

REP-149 explicitly places the specification in the public domain and links
`package_format3.xsd` as its schema. The pinned upstream repository snapshot
does not contain a separate repository-level license file, and the XSD files
do not carry their own license header. This provenance and limitation are
recorded rather than assigning a new license to the upstream files.

The copies are used unchanged so package metadata validation can run without a
network request. When updating them, update both files together, pin the new
upstream commit, compare the exact diff, and re-check the licensing basis.

## 语音离线依赖与模型

本仓库不重新分发 source archive、patch 或模型权重，只提交可审计、可离线核验的 lock 元数据。依赖锁位于
[`third_party/locks/audio-dependencies.yaml`](third_party/locks/audio-dependencies.yaml)，模型锁位于
[`models/manifests/voice-models.yaml`](models/manifests/voice-models.yaml)。模型锁把运行时/框架许可证与权重、训练数据
provenance 分开：`runtime_license` 不能被当作模型许可证。

| 资产 ID | 冻结 identity | SHA-256 | 运行时/框架许可证 | 模型许可证状态 |
| --- | --- | --- | --- | --- |
| `portaudio` | `v19.7.0`, `147dd722548358763a8b649b3e4b41dfffbcfbb6` | `95457b809ce60d4d4790f84bb692e271f644e59d8adf96feb988c89ab52a506a` | `MIT` | 不适用 |
| `webrtc-audio-processing` | `2.1` | `ae9302824b2038d394f10213cab05312c564a038434269f11dbf68f511f9f9fe` | `BSD-3-Clause` | 不适用 |
| `abseil-cpp` | `20240722.0` | `f50e5ac311a81382da7fa75b97310e4b9006474f9560ac46f54a9967f07d4ae3` | `Apache-2.0` | 不适用 |
| `abseil-cpp-meson-patch` | `20240722.0-3` | `12dd8df1488a314c53e3751abd2750cf233b830651d168b6a9f15e7d0cf71f7b` | `Apache-2.0` | 不适用 |
| `sherpa-onnx` | `v1.13.4`, `142807252687d81b40d6315f23470a1512a00de3` | `f0dc7c9b41b8691313daee671e826eb23946fa1320559a8d37e84f8774af76b2` | `Apache-2.0` | 不适用 |
| `onnxruntime` | shared `1.27.0`, `GIT_COMMIT_ID=8f0278c77bf44b0cc83c098c6c722b92a36ac4b5` | ZIP `9f0c0a6998f1b94c399eeddcb443beb4a922c9a4fd431fdc9cd6de67a1935d00`; `libonnxruntime.so` `026c7d5c609323fb16506dbc3cce801bcdffdd7566fdba49a50727e2e1e881ca` | `MIT` | exact shared runtime；禁止旧 `libonnxruntime.a` |
| `kws-zh-en-3m-2025-12-20` | `sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20`, `2025-12-20` | `68447f4fbc67e70eee3a93961f36e81e98f47aef73ce7e7ca00885c6cd3616a6` | `Apache-2.0`（sherpa-onnx runtime） | **unresolved**：未找到权威权重/训练数据许可 |
| `vad-silero-int8` | `silero_vad.int8.onnx`, `silero-vad-int8` | `c36d490aff5ab924ca6c7aeec4d8f6bd3d22db6fa17611b9c5b17eae58ac3a20` | `Apache-2.0`（sherpa-onnx runtime） | **resolved**：Silero 上游模型仓库 `MIT` provenance |
| `asr-sensevoice-small-int8-2024-07-17` | GitHub release asset `288366523`; `sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17`; `163002883` bytes | `7d1efa2138a65b0b488df37f8b89e3d91a60676e416f515b952358d83dfd347e` | `Apache-2.0`（sherpa-onnx runtime） | **resolved**：`FunASR Model Open Source License Agreement 1.1`；来源 `FunAudioLLM/SenseVoiceSmall`，作者 FunAudioLLM，source revision `3847d57b6bdf2dd8875cb1508d2af43d80a16bf7` |
| `asr-zh-int8-2025-06-30` | `sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30`, `2025-06-30` | `5a2832047ea1f97dd0dc595b816c230c4bafad65cfc0341fa57517cadc50afd0` | `Apache-2.0`（sherpa-onnx runtime） | **unresolved**：未找到权威权重/训练数据许可 |
| `tts-chaowen-medium-int8` | GitHub release asset `406468505`; `vits-piper-zh_CN-chaowen-medium-int8.tar.bz2`; `14011298` bytes | `f5f7c8628427fbb259ea4b7ec1a9a822a0c04e3f267071f0abfa0610371d9e0c` | `Apache-2.0`（sherpa-onnx runtime） | **restricted**：Xiao Ya/BZNSYP 上游数据链限制非商业使用 |

TTS 使用 `https://api.github.com/repos/k2-fsa/sherpa-onnx/releases/assets/406468505`，provision 请求该 API 时必须携带
`Accept: application/octet-stream`，随后仍核验冻结的 size 与 SHA-256。tag URL、非 int8 文件和临时下载文件都不能冒充完成资产。

ONNX Runtime 1.27.0 仅使用 sherpa-onnx v1.13.4 source 记录的 exact shared artifact：
`https://github.com/csukuangfj/onnxruntime-libs/releases/download/v1.27.0/onnxruntime-linux-x64-glibc2_17-Release-1.27.0.zip`，
ZIP size `8509524`、SHA-256 `9f0c0a6998f1b94c399eeddcb443beb4a922c9a4fd431fdc9cd6de67a1935d00`；
解包后的 `lib/libonnxruntime.so` size `26403889`、SHA-256
`026c7d5c609323fb16506dbc3cce801bcdffdd7566fdba49a50727e2e1e881ca`，SONAME 为
`libonnxruntime.so`，`GIT_COMMIT_ID=8f0278c77bf44b0cc83c098c6c722b92a36ac4b5`，许可证为 `MIT`。
产品只接受 canonical prefix receipt 的这一 shared identity，最终 ELF 使用 approved prefix 的单一 `DT_RPATH`；
系统库、`LD_LIBRARY_PATH` 覆盖、旧 static archive 与旧 receipt 均 fail-closed。

KWS 与原 Zipformer ASR 的模型许可证状态明确为 **unresolved**。默认 provision、verify 与 release 检查会在创建目录或下载前
fail-closed；在取得权威的权重及训练数据许可前，不能进入这些 Runtime。SenseVoice 的 `FunASR Model Open Source License Agreement 1.1`
仅适用于其自定义模型来源，来源、作者、名称和 exact revision 已单独锁定。VAD 的 `MIT` 是 Silero 上游模型 provenance，和
sherpa-onnx 的 Apache-2.0 runtime 许可证分开记录。Chaowen 的模型卡记录 Xiao Ya 基座和 BZNSYP 训练数据链，
因此仅限研究、学习与仿真；商业用途必须更换模型并重新完成语音验收。

正式 CLI 只能使用仓库内、且由 `.gitignore` 覆盖的 `.deps/voice-assets` 和 `models/weights/voice-assets`；自定义 root
只允许测试通过 Python `Provisioner` seam 使用。运行时不得下载任何资产，真实完整资产的下载与 verify 是显式维护/Release 步骤，
不属于普通 PR 测试。
