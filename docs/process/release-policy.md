# 发布策略与路线图

VoiceNav Robot 使用 [Semantic Versioning](https://semver.org/spec/v2.0.0.html)。1.0 前兼容性可以变化，
但每项变化都必须明确且可审查。路线图 milestone 使用 `v0.1` 之类短名；不可变 release tag 使用完整
SemVer，例如 `v0.1.0`。

## 已批准的 walking-skeleton 路线图

`v0.1` 仓库基础已存在。已批准能力序列固定为：
`v0.2` → `v0.3` → `v0.4` → `v0.5` → `v0.6` → `v0.7` → `v1.0`。
它们是在一个纵向可运行 walking skeleton 中的 capability milestone；每个 milestone 都扩展同一可运行产品路径。

| 里程碑 | 能力边界 |
| --- | --- |
| `v0.2` | 运动基线：`gz_ros2_control`、TF ownership、LiDAR/world、独立 MotionGate 与 consumer deadman |
| `v0.3` | 建图与导航：slam_toolbox mapping、atomic map package、AMCL、Named Place 与 Nav2 safety navigation |
| `v0.4` | 文本 Mission：Mission v1 Interface、validator/FSM、in-memory adapter、relative motion、navigation 与 map adapter |
| `v0.5` | 本地 Agent：deterministic Mandarin rule、本地 Qwen/llama.cpp fallback、clarification 和 stale-result isolation |
| `v0.6` | 实时语音：PortAudio、WebRTC APM、KWS/VAD/ASR、TTS 与 offline audio fixture |
| `v0.7` | 全双工语音：AEC、barge-in、voice STOP 与端到端 Mapping/Navigation flow |
| `v1.0` | 端到端 Hardening：fault recovery、performance/soak、license/model inventory 与完整端到端 release evidence |

每个 milestone 扩展前一个可运行切片。LLM availability 永远不是仿真、建图、导航、deterministic rule、
Mission safety 或固定 STOP path 的前置条件。

## 版本含义

- `MAJOR`：1.0 后不兼容的 Stable Interface change。
- `MINOR`：向后兼容 capability 或明确的 pre-1.0 milestone。
- `PATCH`：不改变预期 Interface 的向后兼容 correction。

所有项目 ROS package 在 release boundary 使用同一版本；不会每个 commit 都 bump package version。

1.0 前，不兼容 IDL 或 behavior change 必须：

- 在一个 Issue 内更新全部 producer 与 consumer；
- 更新 Interface 和 acceptance 文档；
- 添加或修改 contract test；
- 出现在 `CHANGELOG.md` 的 `Unreleased` 下；并且
- 在 Issue 与 pull request 中说明 migration impact。

v1.0 后，破坏性 Mission IDL 变化创建 V2 type 与 endpoint，并提供有界 V1 migration Adapter。
`api_version` field 不会使不兼容 DDS type 兼容。

## 发布门禁

仅在以下全部完成后创建 release：

- milestone Issue 集合满足每条验收标准；
- PR CI 从 clean checkout 通过；
- architecture、Interface、process 与 operational 文档匹配发布 behavior；
- 相关 `Unreleased` changelog entry 移至带日期版本；
- package metadata、dependency declaration 和 version 一致；
- milestone 所需 dependency、model 与 license record 存在；
- 不存在未解决的 critical motion、data-loss、privacy 或 license issue；以及
- milestone-specific automated 与有界 manual acceptance evidence 已记录。

release 从审查过的 `main` 创建为不可变 annotated Git tag 和 GitHub Release，包含 notes 与 acceptance evidence
链接。release tag 与 artifact 不可变；发现问题时发布较新版本修复，而不是重写已发布 release。

## v1.0 发布证据

v1.0 还需要：

- [产品规格](../product/v1.0-product-spec.md)中的完整 flow；
- [测试策略](testing-strategy.md)中所有量化 completion criteria；
- Mission invalid/busy/timeout/cancel/STOP race 与 late-result evidence；
- Mission Runtime death、MotionGate death、consumer-timeout 和 zero-velocity evidence；
- Mapping 与 Navigation TF ownership evidence 及 atomic saved-map handoff；
- 在支持的 WSL analog-audio setup 上真实本地 Mandarin KWS/ASR/TTS、playback-reference AEC、barge-in 和固定
  STOP evidence；
- performance 与 soak evidence；
- license 与 locked-model inventory；
- clean-checkout reproduction、release note 和可复现实验记录；以及
- 确认验收 flow 不需要 cloud request。

## 分发与恢复

1.0 前以 source 加 tagged GitHub repository release 分发。Bloom 与 ROS apt packaging 不在范围内。本地 model
weight、大型生成 map 与私有 runtime evidence 不附加到 release。commit `075c0f4` 上的 archive tag
`archive/vn-0011a-pre-workflow-reset-20260804` 与外部已验证 all-refs bundle 是已淘汰仓库材料的 recovery
evidence，而非 release input。
