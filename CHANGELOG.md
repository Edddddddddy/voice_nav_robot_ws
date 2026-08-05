# Changelog

All notable user-visible and Interface changes to VoiceNav Robot are recorded
here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and releases follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Added the reviewed VN-0010 implementation for an independent,
  fail-closed MotionGate: package-private bounded ROS types, an internal
  non-installed static `MotionGateCore`, the installed `motion_gate_node`,
  trusted configuration, canonical product bringup, and Core/node/headless
  product test layers, plus a fresh-prefix install audit proving that the
  internal Core header/library are not exported. Exact-head local verification,
  independent code/safety/documentation review, required hosted CI, rebase
  merge are complete. The configured controller timeout is not by itself
  evidence of process-death recovery or physical stationarity.
- Added a packaged, self-contained Gazebo world with an analytic obstacle and
  a 360-degree single-layer GPU LiDAR on `laser_link`.
- Added a headless perception integration gate that verifies `/scan` type,
  QoS, simulation timestamps, scan-time TF, analytic range, direct `/odom`,
  matched odometry/TF pose, bounded motion, and clean shutdown.
- Added a per-edge TF ownership auditor that correlates message publisher GIDs
  with fully qualified graph endpoint owners over a complete observation
  window, plus duplicate-writer and disjoint-writer fault fixtures.
- Added the repository contract for the packaged LiDAR world and product graph.
- Added the tests-first simulation-drive contract for `gz_ros2_control` and
  Jazzy's native `TwistStamped` `diff_drive_controller` input.
- Added a headless Gazebo integration gate that proves controller activation,
  wheel Interface claims, forward/positive-yaw odometry, TF output, publisher
  loss, and the 0.35 s consumer deadman.

### Changed

- Finalized the pre-1.0 Mission V1 public Interface migration: bounded
  `MissionStep`, fenced `ExecuteMission`, `MissionState`, and `StopMission`
  types now generate successfully for C++ and Python, with public contract
  tests covering field order, constants, bounds, and construction. This is a
  breaking Interface change; downstream consumers must migrate before
  rebuilding.
- Updated the canonical verification entry point to resolve ordinary and
  managed-worktree `.git` contexts before Git subprocesses, including Windows
  absolute `gitdir:` pointers under WSL, with fail-closed diagnostics and
  temporary-fixture regression coverage.
- Consolidated quality, testing, evidence, contributor, pull-request, and
  release governance around focused development checks, one complete final
  gate, Issue/PR evidence ownership, and the approved walking-skeleton order.
- Locked the private Gate seam to
  `InternalMotionGateControl`/`InternalMotionGateState`, the operations
  `PREPARE`/`OPEN`/`RENEW`/`INHIBIT`, Gate-generated lease topics, global
  compare-and-swap sequencing, exact 32-character lowercase hexadecimal
  request/Gate/lease identities, Gate-local 16-byte writer identity, reader
  and publication barriers, finite-value clamping, and fail-closed
  invalid-input retirement. OPEN now validates in the pure Core before graph
  access, crosses three writer snapshots with discard readers A/B and the
  first accepting reader C, and faults if the unique writer changes. The node
  FQN is `/motion_gate_node`, private endpoints are
  `/motion_gate/internal/control` and `/motion_gate/internal/state`, and
  candidate topics use `/voice_nav_internal/motion_gate/candidate/lease_`.
  The product launch selects `rmw_fastrtps_cpp`, the Gate rejects any other
  RMW at startup, and both runtime packages declare the Fast DDS dependency.
  The final controller publisher follows
  `rclcpp::SystemDefaultsQoS()` and must prove actual endpoint compatibility;
  process-death crash-stop and pause/resume acceptance remain separate target
  evidence.
- Replaced the product model's native Gazebo DiffDrive plugin with
  `gz_ros2_control`, a Jazzy `diff_drive_controller`, and an event-ordered
  launch owned directly by ROS 2 launch.
- Changed controller odometry from its private topic to a direct product
  `/odom` remap without a relay.
- Expanded the Gazebo bridge allowlist from `/clock` to exactly `/clock` and
  `/scan`; command, joint state, odometry, and TF remain unbridged.
- Documented normal acceleration shaping as an upstream velocity-smoother
  responsibility.

### Fixed

- Prevented otherwise-green Gazebo launch tests from relying only on signal
  escalation during teardown. Isolated fixtures now request structured server
  stop, require a positive ACK, wait for the launch-managed process, and retain
  strict global exit-code checks. Each test process claims a unique random
  Gazebo partition, binds the stop RPC to the validated environment snapshot,
  and runs cleanup as independent must-run phases. A typed transient CLI
  timeout retries the idempotent stop once; all other failures remain
  fail-closed. Product launch behavior still defaults to shutdown on an
  unexpected Gazebo exit.
- Prevented green test summaries from omitting or semantically overriding
  critical evidence. Repository source inventory, official isolated launch
  runners, generated CTest metadata, exact xUnit structure, and a narrow skip
  allowlist now form one fail-closed chain. Gazebo movement evidence uses a
  bounded, partition-checked pose snapshot parser that handles a small
  complete JSON burst while rejecting malformed, duplicate, non-finite, or
  invalid-quaternion data.
- Prevented a DDS-matched candidate writer from being terminally rejected
  solely because its Gate-local ROS node identity snapshot had not converged.
  MotionGate now types only that narrow state as
  `WRITER_METADATA_PENDING`, provisionally pins the non-zero GID, keeps the
  prepared generation at published zero, and retries within the original
  absolute deadline. Wrong type, FQN, partial namespace, QoS, endpoint kind,
  duplicate/zero/replaced GID, and barrier changes remain terminal.
- Prevented a runtime `use_sim_time=false` change or a disabled ROS-time
  override from emitting future system-time-stamped motion that could defeat
  the controller consumer timeout. The Gate rejects parameter mutation and
  independently faults closed to zero/stamp-zero at the publication barrier
  unless both clock invariants hold.
- Prevented WSL launch tests from orphaning a shell-owned Gazebo process and
  accidentally reusing an old controller graph.
- Prevented a transient RMW `_NODE_*_UNKNOWN_` graph identity from being
  misclassified as a confirmed TF owner mismatch; unresolved identities now
  remain pending until resolution or bounded timeout.
- Prevented local or relative world resource URIs and Xacro-macro expansion
  from bypassing the self-contained world and single-LiDAR source contracts.
- Bound the generated LiDAR to the outer robot model's unique
  `base_footprint` element and canonical finite pose, rejecting duplicate
  sensors/poses, wheel-local mounts, nested same-name frame escapes, and
  unreviewed sibling SDF root objects.

## [0.1.0] - 2026-07-30

### Added

- Strongly typed Mission step and execution Action interfaces.
- Hand-written differential-drive Xacro model and internal TF tree.
- Gazebo collision, inertia, stable spawning, native DiffDrive, velocity
  limits, and odometry.
- Repository contract tests for path confinement, local links, documentation
  layout, and synchronized ROS package versions.
- GitHub Issue and PR templates, pinned Ubuntu 24.04 / ROS 2 Jazzy CI, and
  monthly GitHub Actions dependency updates.
- Product, architecture, Mission, motion, TF/mode, quality, testing, release,
  security, and ADR documentation for the accepted v1.0 plan.

### Changed

- Reorganized documentation into `docs/product`, `docs/architecture`,
  `docs/process`, `docs/adr`, and `docs/agents`.
- Synchronized all ROS package metadata with the repository `0.1.0` version.
- Defined native Gazebo DiffDrive as a historical simulation baseline; the
  product path now migrates to `gz_ros2_control`, `diff_drive_controller`, an
  independent MotionGate, and consumer-side command timeout.
- Extended the canonical verification command to run repository contracts and
  reject tracked colcon outputs before building.

### Fixed

- Recorded the accepted cylinder implementation, wheel links, dimensions, and
  measured LiDAR transform in the current model and architecture contracts.
- Preserved the original repository baseline and its verification boundaries in
  the immutable recovery archive.

### Security

- Ignore rules exclude generated data, local dependencies, credentials,
  recordings, bags, model weights, and runtime evidence.
- The quality gate rejects tracked `build/`, `install/`, and `log/` paths.
- Operational stop terminology no longer implies a certified emergency-stop
  or physical-stop guarantee.

### Archive note

- Retired Course and repository Work Item material is recoverable from
  `archive/vn-0011a-pre-workflow-reset-20260804` at commit `075c0f4` and the
  external verified all-refs bundle.
