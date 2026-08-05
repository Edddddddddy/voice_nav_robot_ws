# VoiceNav Robot

VoiceNav Robot is a simulation-only ROS 2 system that turns local Mandarin
voice input into a bounded, strongly typed Mission and executes it through a
guarded Gazebo differential-drive chain. The approved target includes local
KWS, AEC, ASR, deterministic rules, a local LLM, TTS, mapping, and navigation.

The supported baseline is Windows 11 with WSL2 Ubuntu 24.04, ROS 2 Jazzy, and
Gazebo Harmonic. Real robots, manipulators, cameras, cloud services, and
functional-safety emergency stops are outside the supported scope.

## Current status

The repository currently verifies a v0.2 simulation, MotionGate, and Mission
Runtime control-plane slice:

- six ROS packages with explicit responsibility and dependency boundaries;
- a hand-written differential-drive Xacro with collision, inertia, and stable
  spawning;
- `gz_ros2_control`, Jazzy `diff_drive_controller`, and a 0.35 s consumer
  timeout;
- a self-contained Gazebo world with one 360-degree 2D LiDAR on `laser_link`;
- `/clock` and `/scan` as the only ROS/Gazebo bridge traffic;
- direct product `/odom` from the controller and per-edge TF ownership checks;
- an independent fail-closed MotionGate with a 250 ms Runtime authority lease,
  150 ms candidate freshness deadline, writer binding, and sole final velocity
  ownership.
- a package-private Mission Runtime Core and ROS Adapter with bounded admission,
  STOP fencing, typed state/feedback/results, and scripted behavior fakes;
  the production RelativeMotion Adapter is intentionally unavailable until
  #35, and physical motion is not implemented by this slice.

The configured controller timeout is a consumer-side deadman, not by itself
evidence of process-death recovery or physical stationarity. This slice does
not claim the production RelativeMotion Adapter, smoother, Collision Monitor,
process-kill, or managed-pause integration. See the
[architecture overview](docs/architecture/overview.md) for the current/target
boundary and the [v1.0 product specification](docs/product/v1.0-product-spec.md)
for the approved acceptance flow.

## Target control path

```text
microphone + speakers
  -> local AEC / wake word / VAD / ASR
  -> STOP-first Agent + deterministic rules + local LLM fallback
  -> validated Mission with admission fencing
  -> Nav2 or relative-motion candidate velocity
  -> velocity smoother -> collision monitor -> MotionGate
  -> diff_drive_controller -> gz_ros2_control -> Gazebo
  -> slam_toolbox in Mapping Mode, or AMCL + Nav2 in Navigation Mode
  -> local TTS
```

Mapping and Navigation are separately launched modes. `slam_toolbox` owns
`map -> odom` in Mapping Mode and AMCL owns it in Navigation Mode; they never
run as simultaneous owners. MotionGate is the only final velocity publisher,
and the Agent, Nav2, and Gazebo bridge cannot bypass it.

## Repository structure

| Path | Responsibility |
| --- | --- |
| `src/voice_nav_interfaces` | Stable ROS messages, services, and actions |
| `src/voice_nav_sim` | Xacro, Gazebo, and simulation adapters |
| `src/voice_nav_mission` | Mission Runtime, MotionGate, and adapters |
| `src/voice_nav_agent` | Mandarin rules, local LLM adapter, and dialogue policy |
| `src/voice_nav_audio` | Audio I/O, AEC, KWS, VAD, ASR, and TTS |
| `src/voice_nav_bringup` | Mode launches, configuration, and composition |
| `docs/product` | Product scope, acceptance, and terminology |
| `docs/architecture` | System, Interface, safety, TF, and mode contracts |
| `docs/process` | Quality, testing, change, release, and recurrence controls |
| `docs/adr` | Consequential architecture decisions and supersession |
| `docs/agents` | Issue/PR delivery protocol |

## Build and verification

Run the focused repository checks during development:

```bash
python3 -m unittest tests.test_repository_contract
python3 scripts/check_repository.py --root .
```

After the final change, run the complete quality gate exactly once on the
final PR HEAD in the same managed worktree:

```bash
bash scripts/verify.sh
```

The entry point resolves ordinary `.git` directories, relative worktree
pointers, and Windows absolute `gitdir:` pointers automatically. Do not
pre-set `GIT_DIR` or `GIT_WORK_TREE`, and do not point the command at another
checkout.

Record the gate's true exit status before running any separate diagnostics. A
later successful command must not overwrite a failed result. The complete
cadence and evidence ownership are defined in the
[change lifecycle](docs/process/change-lifecycle.md).

Generated `build/`, `install/`, and `log/` trees, model weights, maps,
recordings, and runtime evidence must not be committed. The supported
MotionGate implementation is locked to `rmw_fastrtps_cpp`; the canonical
launch selects it and the node rejects another RMW. This is an implementation
support boundary, not a portability claim for every DDS implementation.

## Development workflow

GitHub Issues are the canonical requirements, decision, acceptance, dependency,
and status record. The delivery path is:

```text
GitHub Issue -> isolated branch -> tests-first implementation
  -> documentation / ADR / changelog as needed -> local verification
  -> Draft PR -> review -> CI -> rebase merge
```

Read [Contributing](CONTRIBUTING.md), the [change lifecycle](docs/process/change-lifecycle.md),
and the relevant product and architecture contracts before editing.

## Safety, license, and reporting

“Stop” means a high-priority operational stop in the simulation. It requests
zero command and MotionGate inhibition; it does not prove physical stationarity
and is not a certified emergency stop. See [SECURITY.md](SECURITY.md) for
reporting and deployment boundaries.

The project is licensed under Apache License 2.0. Third-party sources and
restrictions are recorded in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
