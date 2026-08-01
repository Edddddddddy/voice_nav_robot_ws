# Changelog

All notable user-visible and Interface changes to VoiceNav Robot are recorded
here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and releases follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Added the VN-0010 local-GREEN implementation for an independent,
  fail-closed MotionGate: package-private bounded ROS types, an internal
  non-installed static `MotionGateCore`, the installed `motion_gate_node`,
  trusted configuration, canonical product bringup, and Core/node/headless
  product test layers, plus a fresh-prefix install audit proving that the
  internal Core header/library are not exported. The full local gate and
  independent local evidence review pass; this does not claim hosted CI,
  merge, release, or solution-tag closure.
- Added a packaged, self-contained Gazebo world with an analytic obstacle and
  a 360-degree single-layer GPU LiDAR on `laser_link`.
- Added a headless perception integration gate that verifies `/scan` type,
  QoS, simulation timestamps, scan-time TF, analytic range, direct `/odom`,
  matched odometry/TF pose, bounded motion, and clean shutdown.
- Added a per-edge TF ownership auditor that correlates message publisher GIDs
  with fully qualified graph endpoint owners over a complete observation
  window, plus duplicate-writer and disjoint-writer fault fixtures.
- Added the Lesson 0008 Work Item, course contract, and evidence record for
  the packaged LiDAR world and product graph.
- Added the tests-first Lesson 0007 contract, Work Item, and pending evidence
  record for migrating the simulation drive path to `gz_ros2_control` and
  Jazzy's native `TwistStamped` `diff_drive_controller` input.
- Added a headless Gazebo integration gate that proves controller activation,
  wheel Interface claims, forward/positive-yaw odometry, TF output, publisher
  loss, and the 0.35 s consumer deadman.

### Changed

- Locked the Lesson 0009 private Gate seam to
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
  crash-stop and pause/resume acceptance remain Lesson 0010 scope.
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
- Versioned Markdown course catalog, Lessons 0001–0006, engineering references,
  and evidence-backed learning records.
- Repository contract tests for course ordering, path confinement, local links,
  documentation layout, and synchronized ROS package versions.
- GitHub Issue and PR templates, pinned Ubuntu 24.04 / ROS 2 Jazzy CI, and
  monthly GitHub Actions dependency updates.
- Product, architecture, Mission, motion, TF/mode, quality, testing, release,
  security, ADR, and Work Item documentation for the accepted v1.0 plan.

### Changed

- Reorganized documentation into `docs/product`, `docs/architecture`,
  `docs/process`, `docs/adr`, `docs/work-items`, and `course`.
- Synchronized all ROS package metadata with the repository `0.1.0` version.
- Defined native Gazebo DiffDrive as a historical teaching checkpoint; the
  product path now migrates to `gz_ros2_control`, `diff_drive_controller`, an
  independent MotionGate, and consumer-side command timeout.
- Extended the canonical verification command to run repository contracts and
  reject tracked colcon outputs before building.

### Fixed

- Preserved the original Lesson 0003 box-robot assignment and added a separate
  erratum for the accepted cylinder implementation, wheel links, dimensions,
  and measured LiDAR transform.
- Closed the Lesson 0006 evidence gap using the learner's recorded commits and
  verification output.

### Security

- Ignore rules exclude generated data, local dependencies, credentials,
  recordings, bags, model weights, and runtime evidence.
- The quality gate rejects tracked `build/`, `install/`, and `log/` paths.
- Operational stop terminology no longer implies a certified emergency-stop
  or physical-stop guarantee.
