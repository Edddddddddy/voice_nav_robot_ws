# Release policy and roadmap

VoiceNav Robot uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
Before 1.0, compatibility may change, but every change remains explicit and
reviewable. Roadmap milestones use short names such as `v0.1`; immutable
release tags use complete SemVer, such as `v0.1.0`.

## Approved walking-skeleton roadmap

The `v0.1` repository foundation already exists. The approved capability
sequence is fixed as:
`v0.2` → `v0.3` → `v0.4` → `v0.5` → `v0.6` → `v0.7` → `v1.0`.
These are capability milestones in one vertically runnable walking skeleton,
and each milestone extends the same runnable product path.

| Milestone | Capability boundary |
| --- | --- |
| `v0.2` | 运动基线：`gz_ros2_control`, TF ownership, LiDAR/world, independent MotionGate, and consumer deadman |
| `v0.3` | 建图与导航：slam_toolbox mapping, atomic map package, AMCL, Named Places, and Nav2 safety navigation |
| `v0.4` | 文本 Mission：Mission v1 Interface, validator/FSM, in-memory adapters, relative motion, navigation, and map adapters |
| `v0.5` | 本地 Agent：deterministic Mandarin rules, local Qwen/llama.cpp fallback, clarification, and stale-result isolation |
| `v0.6` | 实时语音：PortAudio, WebRTC APM, KWS/VAD/ASR, TTS, and offline audio fixtures |
| `v0.7` | 全双工语音：AEC, barge-in, voice STOP, and end-to-end Mapping and Navigation flows |
| `v1.0` | 端到端 Hardening：fault recovery, performance/soak, license and model inventory, and complete end-to-end release evidence |

Each milestone extends the preceding runnable slice. LLM availability is never
a prerequisite for simulation, mapping, navigation, deterministic rules,
Mission safety, or the fixed STOP path.

## Version meaning

- `MAJOR`: an incompatible Stable Interface change after 1.0.
- `MINOR`: a backward-compatible capability or an explicit pre-1.0 milestone.
- `PATCH`: a backward-compatible correction that changes no intended Interface.

All project ROS packages use the same version at a release boundary. Package
versions are not bumped for every commit.

Before 1.0, an incompatible IDL or behavioral change must:

- update every producer and consumer in one Issue;
- update Interface and acceptance documentation;
- add or change contract tests;
- appear under `Unreleased` in `CHANGELOG.md`; and
- state migration impact in the Issue and pull request.

After v1.0, a breaking Mission IDL change creates a V2 type and endpoint plus a
bounded V1 migration Adapter. An `api_version` field does not make incompatible
DDS types compatible.

## Release gate

Every release is created only after all of the following are complete:

- the milestone Issue set satisfies every acceptance criterion;
- PR CI passes from a clean checkout;
- architecture, Interface, process, and operational documentation match
  released behavior;
- relevant `Unreleased` changelog entries move to the dated version;
- package metadata, dependency declarations, and versions are consistent;
- dependency, model, and license records needed by the milestone are present;
- no unresolved critical motion, data-loss, privacy, or license issue remains;
  and
- milestone-specific automated and bounded manual acceptance evidence is
  recorded.

The release is created from reviewed `main` as an immutable annotated Git tag
and a GitHub Release containing notes and links to its acceptance evidence.
Release tags and artifacts are immutable; a discovered problem is fixed in a
newer version rather than by rewriting a published release.

## v1.0 release evidence

v1.0 additionally requires:

- the complete flow in the [product specification](../product/v1.0-product-spec.md);
- all quantitative completion criteria in [Testing strategy](testing-strategy.md);
- Mission invalid/busy/timeout/cancel/STOP race and late-result evidence;
- Mission Runtime death, MotionGate death, consumer-timeout, and zero-velocity
  evidence;
- Mapping and Navigation TF ownership evidence plus an atomic saved-map handoff;
- real local Mandarin KWS/ASR/TTS, playback-reference AEC, barge-in, and fixed
  STOP evidence on the supported WSL analog-audio setup;
- performance and soak evidence;
- the license and locked-model inventory;
- clean-checkout reproduction, release notes, and reproducible experiment
  records; and
- confirmation that no cloud request is required for the acceptance flow.

## Distribution and recovery

Before 1.0, distribution is source plus a tagged GitHub repository release.
Bloom and ROS apt packaging are out of scope. Local model weights, large
generated maps, and private runtime evidence are not attached to releases.
The archive tag `archive/vn-0011a-pre-workflow-reset-20260804` at commit
`075c0f4` and the external verified all-refs bundle are recovery evidence for
retired repository material, not release inputs.
