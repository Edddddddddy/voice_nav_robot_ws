# Release policy and roadmap

VoiceNav Robot uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html). Before 1.0, compatibility changes remain possible, but every change remains explicit and reviewable.

The approved roadmap uses short milestone names such as `v0.1`; the immutable release tag uses the complete SemVer form, such as `v0.1.0`.

## Approved delivery route

This table is the approved release-to-course mapping and is the source of truth for milestone scope.

| Release | 课程/成果 |
| --- | --- |
| `v0.1` | 历史净化、文档重构、Issue/PR 模板、CI、main 保护、课程双轨 |
| `v0.2` | 0007–0010：`gz_ros2_control` 迁移、唯一 TF、LiDAR/world、独立 MotionGate 和 crash-stop |
| `v0.3` | 0011–0013：slam_toolbox 建图、原子地图包、AMCL、Named Places、Nav2 安全导航 |
| `v0.4` | 0014–0016：Mission V1 Interface、纯 C++ Validator/FSM、Fake Ports、相对运动/Nav/Map Adapter |
| `v0.5` | 0017–0018：确定性中文规则、Qwen/llama.cpp fallback、澄清和晚到结果隔离 |
| `v0.6` | 0019–0021：PortAudio、WebRTC APM、KWS/VAD/ASR、TTS 和离线音频 fixtures |
| `v0.7` | 0022–0023：AEC、barge-in、语音 STOP、Voice Mapping 与 Voice Navigation 两条端到端链 |
| `v1.0` | 故障恢复、性能/soak、许可证与模型清单、完整课程、发布说明和可复现实验记录 |

The order keeps every milestone vertically runnable. LLM availability is never a prerequisite for simulation, mapping, navigation, deterministic rules, Mission safety, or the fixed STOP path.

Lessons, Work Items, merged branches, local backups, and course tags do not automatically create releases.

## Version meaning

- `MAJOR`: an incompatible stable Interface change after 1.0.
- `MINOR`: a backward-compatible capability or an explicit pre-1.0 milestone.
- `PATCH`: a backward-compatible correction that changes no intended Interface.

All project ROS packages use the same version at a release boundary. Package versions are not bumped for every commit.

Before 1.0, an incompatible IDL or behavioral change must still:

- update every producer and consumer in one Work Item;
- update Interface and acceptance documentation;
- add or change contract tests;
- appear under `Unreleased` in `CHANGELOG.md`;
- state migration impact in the Work Item.

After v1.0, a breaking Mission IDL change creates a V2 type and endpoint plus a bounded V1 migration Adapter. An `api_version` field does not make incompatible DDS types compatible.

## Release gate

Every release is created only after all of the following are complete:

- the milestone Work Items and linked GitHub Issues satisfy every acceptance criterion;
- PR CI passes from a clean checkout;
- the matching lessons, course records, and review evidence are complete;
- architecture, Interface, process, and operational documentation match released behavior;
- relevant `Unreleased` changelog entries move to the dated version;
- package metadata, dependency declarations, and versions are consistent;
- dependency, model, and license records needed by the milestone are present;
- no unresolved critical motion, data-loss, privacy, or license issue remains;
- the milestone-specific automated and bounded manual acceptance evidence is recorded.

The release is then created from reviewed `main` as:

1. an immutable annotated Git tag using the full SemVer version;
2. a GitHub Release containing the release notes and links to its acceptance evidence.

Release tags and release artifacts are immutable. A discovered problem is fixed in a newer version rather than by rewriting a published release.

## v1.0 release evidence

v1.0 additionally requires:

- the complete flow in the [product specification](../product/v1.0-product-spec.md);
- all quantitative completion criteria in [Testing strategy](testing-strategy.md);
- Mission invalid/busy/timeout/cancel/STOP race and late-result evidence;
- MissionRuntime-death, MotionGate-death, consumer-timeout, and zero-velocity evidence;
- Mapping and Navigation TF ownership evidence plus an atomic saved-map handoff;
- real local Mandarin KWS/ASR/TTS, playback-reference AEC, barge-in, and fixed STOP evidence on the supported WSL analog-audio setup;
- performance and soak evidence;
- the license and locked-model inventory;
- clean-checkout reproduction, the complete course, release notes, and reproducible experiment records;
- confirmation that no cloud request is required for the acceptance flow.

## Distribution

Before 1.0, distribution is source plus a tagged GitHub repository release. Bloom and ROS apt packaging are out of scope. Local model weights, large generated maps, and private runtime evidence are not attached to releases; releases contain their lock records, checksums, licenses, and only the small deterministic test assets approved for Git.
