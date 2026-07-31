# VN-0009: Add LiDAR world and prove TF ownership

**Status:** In Progress

**GitHub Issue:**
[#8](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/8)

**GitHub PR:** TBD

**Branch:** `feat/vn-0009-l0008-lidar-tf-ownership`

## Goal

Deliver the smallest complete simulation-perception and coordinate-contract
slice:

```text
packaged non-empty world
  -> 360-degree 2D LiDAR on laser_link
  -> Gazebo /scan
  -> one-way ROS /scan bridge
  -> scan-time odom -> laser_link transform

diff_drive_controller private odometry
  -> direct remap
  -> product /odom

/tf + /tf_static samples
  -> per-edge publisher GID audit
  -> one expected owner for every observed edge
```

The slice must be reproducible from `course/0008-start`, run headlessly without
network assets, and provide evidence strong enough for later SLAM and Nav2
lessons to consume `/scan`, `/odom`, and TF without redefining their ownership.

## Stable contracts

### World

- `voice_nav_sim` packages
  `worlds/voice_nav_test_world.sdf` and loads it from package share.
- The world is self-contained and network-independent. It does not include a
  Fuel URI, HTTP(S) URI, or a model that exists only in a developer cache.
- The world contains the required Gazebo physics, user-command,
  scene-broadcaster, and rendering-sensor systems.
- A fixed collision box named by the repository contract has pose center
  `(2.0, 0.0, 0.5)` m and size `(0.5, 1.0, 1.0)` m. Its front face is therefore
  at world x `1.75` m.
- The robot still starts at the existing origin. With `laser_link` at x
  `0.10` m, a beam nearest zero radians has an analytic front-face distance
  close to `1.65` m. Tests derive the expected value from the reported beam
  angle instead of assuming a center array index.

### LiDAR and bridge

- The model contains exactly one single-layer `gpu_lidar` on `laser_link`.
- The Gazebo sensor publishes `/scan`, reports frame `laser_link`, and uses:
  - update rate `10` Hz;
  - horizontal samples `360`, resolution `1`, angle range `[-pi, +pi]`;
  - vertical samples `1` at angle `0`;
  - range `0.05..8.0` m with `0.01` m resolution;
  - no configured measurement noise.
- ROS `/scan` is `sensor_msgs/msg/LaserScan`. Its frame is exactly
  `laser_link`, its stamps come from simulation time and advance, its geometry
  matches the model contract, and it contains multiple scans rather than one
  latched fixture.
- `config/bridge.yaml` is an allowlist containing exactly:
  - `/clock`: `gz.msgs.Clock` to `rosgraph_msgs/msg/Clock`;
  - `/scan`: `gz.msgs.LaserScan` to
    `sensor_msgs/msg/LaserScan`.
- Both bridges are `GZ_TO_ROS`. `/scan` uses sensor-data QoS; `/clock` uses
  clock QoS. Wall-time timestamp override is forbidden.
- Velocity, joint state, odometry, `/tf`, and `/tf_static` never cross
  `ros_gz_bridge`.

### Odometry and TF ownership

- The controller spawner directly remaps its private `~/odom` output to
  `/odom` using controller ROS arguments. A relay or republisher is forbidden.
- `/odom` has exactly one publisher endpoint and that endpoint belongs to
  `diff_drive_controller`.
- `/diff_drive_controller/odom` has zero publisher endpoints after the direct
  remap. Topic-list absence alone is not sufficient evidence.
- `diff_drive_controller` remains the only owner of
  `odom -> base_footprint`; `robot_state_publisher` remains the only owner of
  internal robot edges.
- The graph has no `map -> odom` writer in this lesson.
- TF uniqueness is evaluated by semantic
  `(expected_topic, parent_frame, child_frame)` contract, not by total
  publisher count or node name:
  1. callbacks capture `MessageInfo.publisher_gid` for every received
     `TFMessage` on `/tf` and `/tf_static`;
  2. graph endpoint records from both topics map each endpoint GID to its node
     identity;
  3. every observed edge must appear on its expected topic with exactly one
     GID, and that endpoint must map to the expected fully qualified owner.
- A GID is scoped to one publisher endpoint. The same node may legitimately
  have different GIDs for `/tf` and `/tf_static`; the audit does not equate
  those endpoint identities.
- Expected owners are absolute fully qualified node names. Short names cannot
  satisfy the contract across namespaces.
- A violation fails immediately. Success is returned only after the complete
  bounded observation window ends with a continuously stable valid tail;
  reaching the minimum stable duration never ends the audit early.
- Exact frame names are required and leading-slash aliases are rejected.
- At a matched odometry timestamp, `/odom` pose and
  `odom -> base_footprint` TF agree within numeric tolerance.

## Non-goals

- SLAM, AMCL, Nav2, map saving, map reload, Named Places, or any
  `map -> odom` owner.
- MotionGate, authority or candidate leases, crash-stop, managed pause,
  Collision Monitor, or velocity smoothing.
- Mission Runtime, Mission interfaces, Agent, local LLM, audio, ASR, or TTS.
- Multi-robot namespaces, robot spawning as a public API, public arbitrary
  world-path selection, Fuel assets, cameras, IMU, or real hardware.
- Reworking the controller consumer deadman completed by Lesson 0007.
- Treating the sensor or bridge as a TF owner. Sensor placement remains in
  Xacro and `robot_state_publisher` owns its frame edge.

## Acceptance criteria

- [x] Annotated tag `course/0008-start` exists locally and remotely and peels
  to reviewed Lesson 0007 closure
  `f99210d8830cd2cd16eb801ffe0de10422cf4584`.
- [x] Tests-first RED proves the repository lacks the packaged
  world/LiDAR/product graph while the checker and all synthetic valid and
  negative fixtures execute successfully.
- [ ] Static contracts reject an empty or network-dependent world, a missing
  required Gazebo system, incorrect obstacle collision geometry, an absent or
  duplicate LiDAR, incorrect sensor frame/topic/geometry/noise, and an
  uninstalled runtime asset.
- [ ] Static contracts reject a bridge with an extra entry, wrong message
  type, wrong direction, wrong QoS, timestamp override, or any bridge for
  command, joint state, odometry, `/tf`, or `/tf_static`.
- [ ] Static and launch contracts prove that `/odom` is a direct controller
  remap and reject a relay, an absent remap, or a remaining publisher on the
  controller-native odometry name.
- [ ] Headless Gazebo uses the packaged world with
  `--headless-rendering`, starts without network access, and publishes
  advancing `/clock`.
- [ ] Gazebo `/scan` and ROS `/scan` publish repeated valid scans with the
  exact frame, geometry, range limits, and simulation-time semantics.
- [ ] The beam whose reported angle has minimum absolute value sees the fixed
  box front face within an analytic tolerance derived from its angle and the
  configured range resolution.
- [ ] At least three increasing scan stamps can each resolve
  `odom -> laser_link` at that scan's own timestamp.
- [ ] `/odom` has one controller publisher endpoint;
  `/diff_drive_controller/odom` has no publisher endpoint.
- [ ] `/odom` pose and `odom -> base_footprint` TF agree at a matched
  timestamp.
- [ ] The runtime audit observes every expected dynamic and static TF edge,
  maps its publisher GID to the graph endpoint, and reports exactly the
  expected owner for each edge.
- [ ] The audit reports that no `map -> odom` edge is present.
- [ ] A negative fixture with a second writer for the same edge fails even
  when both endpoint records reuse the same node name.
- [ ] A separate valid fixture proves that multiple publishers on `/tf` are
  allowed when they own disjoint edges, preventing a publisher-count shortcut.
- [ ] A bounded direct-controller motion followed by an explicit zero keeps
  the same edge-to-owner set and preserves scan-time transformability.
- [ ] Launch cleanup leaves no Gazebo, bridge, controller, state-publisher, or
  entity-spawner process behind.
- [ ] Repository contracts, Xacro/URDF/SDF checks, build, package tests, and
  full verification pass in WSL2 with ROS 2 Jazzy and Gazebo Harmonic.
- [ ] Lesson 0008 contains the tests-first workflow, deterministic range
  derivation, GID ownership model, failure injection, troubleshooting,
  submission evidence, and reflection questions.
- [ ] The learner record contains only real commands, outputs, commit
  identities, PR/CI links, and review findings before its status changes from
  Pending.
- [ ] A reviewed PR passes required hosted CI and is rebase-merged to `main`;
  only then is annotated tag `course/0008-solution` created.

## Risks and rollback

- A visually present obstacle may lack collision geometry, producing no range
  return. Static checks inspect collision geometry and the integration test
  verifies the analytic front-face beam.
- A cached Fuel model can make a developer machine pass and clean CI hang or
  fail. The world rejects external URIs and is exercised from its installed
  package-share path.
- A rendering sensor can work in the GUI but fail in headless CI when the
  Sensors system or rendering flag is missing. The headless integration gate
  is authoritative; a screenshot is supplemental only.
- A 360-sample inclusive angle interval need not place a sample exactly at
  zero. The test selects `argmin(abs(angle_i))` and computes the ray-plane
  distance from the message geometry; it does not hard-code index 180 or
  exactly `1.65`.
- A bridge configured bidirectionally can create an unintended ROS-to-Gazebo
  control seam. The YAML allowlist fixes direction, types, QoS, and topics.
- A relay can make `/odom` appear correct while leaving two liveness and
  ownership boundaries. Static launch checks and graph endpoint checks require
  the direct controller remap.
- Counting `/tf` publishers can both false-fail a healthy graph and miss a
  duplicate edge hidden behind a reused node name. Edge/GID correlation is
  tested with both valid disjoint-writer and invalid same-name-writer
  fixtures.
- DDS discovery can expose a message before its graph endpoint record. The
  runtime audit waits with a bound for GID correlation; it does not silently
  label an unknown endpoint.
- `/tf_static` may be published before a volatile test subscriber starts. The
  audit uses transient-local-compatible QoS and fails if expected static edges
  never arrive.
- The branch can be reverted or abandoned and recreated from immutable
  `course/0008-start`. Published start and solution tags are never
  force-updated.

## Design impact

- Stable Interfaces changed: product `/scan` and `/odom` become available;
  `/diff_drive_controller/odom` stops being a published product endpoint.
- TF or motion ownership changed: no semantic TF owner changes from the
  Lesson 0007 target. This Work Item proves ownership by edge and endpoint GID
  and closes the graph-wide evidence gap.
- Bridge ownership changed: `/scan` joins `/clock` as the only allowed
  Gazebo-to-ROS bridge.
- Runtime assets changed: the product simulation uses a packaged non-empty
  world and one deterministic 2D LiDAR.
- ADR required: no new ADR. This implements
  [ADR-0002](../adr/0002-migrate-to-gz-ros2-control.md) and the accepted
  [TF/mode contract](../architecture/tf-and-operating-modes.md) without
  introducing a new architectural choice.

## Test plan

- Unit:
  - edge/GID aggregation accepts disjoint edge owners;
  - duplicate same-edge GIDs fail even with identical node names;
  - missing graph correlation, wrong expected owner, leading-slash frames, and
    `map -> odom` fail.
- Static:
  - parse packaged world, Xacro, expanded URDF/SDF, bridge YAML, launch,
    package metadata, CMake install rules, and course catalog;
  - enforce exact world, LiDAR, bridge, odometry-remap, and process-boundary
    contracts;
  - exercise one valid fixture and focused negative fixtures before applying
    assertions to the repository.
- Contract:
  - exact topic names, types, directions, QoS, frames, geometry, timestamps,
    direct remap, and prohibited endpoints;
  - expected TF edge-to-owner table across `/tf` and `/tf_static`.
- Integration:
  - launch the installed world headlessly;
  - observe advancing `/clock`, repeated `/scan`, direct `/odom`, expected TF
    edges, endpoint GIDs, and clean process shutdown;
  - compare the nearest-to-zero beam with the analytic box intersection;
  - resolve TF at three scan stamps and compare matched odometry/TF pose.
- Fault injection:
  - run the edge/GID audit against a second same-edge writer that deliberately
    reuses the expected node name;
  - publish bounded motion, then explicit zero, and prove that owner sets and
    scan-time transforms remain valid.
- Manual:
  - inspect the installed world and bridge configuration;
  - record Gazebo and ROS scan metadata, `/odom` endpoint information, a
    human-readable edge/GID/owner table, and a headless run summary;
  - use RViz only as supplemental visualization.
- Full gate: `bash scripts/verify.sh`.

## Documentation

- `docs/work-items/0009-lidar-world-tf-ownership.md`
- `docs/architecture/overview.md` if current implementation claims change
- `course/catalog.toml`
- `course/lessons/0008-lidar-world-tf-ownership.md`
- `course/records/0008-lidar-world-tf-ownership.md`
- `course/reference/tf-frame-contract.md`
- `CHANGELOG.md`

## Verification evidence

### Immutable start checkpoint

Verified locally and against `origin` on 2026-07-31:

```text
Annotated tag:
  course/0008-start
Tag object:
  982ec062889b5a2ab92c391967b9084d15e52b60
Peeled target:
  f99210d8830cd2cd16eb801ffe0de10422cf4584
Remote peeled target:
  f99210d8830cd2cd16eb801ffe0de10422cf4584
```

### Tests-first RED evidence

Verified in WSL2 on 2026-07-31 before any Lesson 0008 production change:

```text
Command:
env PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s tests -p 'test_*.py' -v

Exit status:
1

Summary:
Ran 80 tests in 10.505s
FAILED (failures=1)

Only failing assertion:
Simulation contract failed: simulation launch must load the packaged
non-empty test world; found built-in empty.sdf
```

The synthetic valid Lesson 0008 fixture and all 40 focused positive/negative
simulation-contract cases passed, including reachable-action, Xacro,
world-system, external-URI, inline-ground, LiDAR, bridge, package/CMake,
headless-rendering, direct-odom-remap, relay, and extra-TF-publisher cases.
The other 38 repository tests passed. This is a valid RED because only the
repository product assertion is red and its first diagnostic identifies the
missing product implementation rather than a syntax, import, fixture, or
discovery failure.

The test-only C++ GID auditor was then compiled in the package and exercised
before any Lesson 0008 production graph change:

```text
Focused launch test:
test_test_tf_ownership_conflict.py ... Passed

Focused CTest result:
1/1 passed; 100% tests passed, 0 failed

Enforcement evidence:
/tf_static tf_audit_parent -> tf_audit_child has 2 publisher GID(s);
expected 1

Sentinel evidence:
the same edge had two graph-correlated endpoint GIDs and passed only after
the full 5.000-second observation window

Disjoint-writer evidence:
two additional edges each had one graph-correlated owner while /tf_static
had four publisher endpoints in total

Dynamic-topic evidence:
a 20 Hz `/tf` writer passed only with expected topic `/tf` and owner FQN
`/dynamic_tf_owner`; a second auditor expecting `/wrong_dynamic_tf_owner`
failed and logged the actual graph endpoint FQN
```

The normal auditor returned `1` for the duplicate edge, the conflict sentinel
returned `0` only after observing both GIDs, and the disjoint-edge auditor
returned `0`. The dynamic auditor also completed the full observation window,
while the wrong-owner auditor failed immediately. This proves the gate is
based on expected topic, semantic edge, endpoint GID, and absolute owner FQN,
not node-name or total-topic-publisher shortcuts.

### Local implementation evidence

Pending. Record only output from the final implementation state, including:

- installed world and bridge asset paths;
- Gazebo and ROS scan contracts;
- analytic beam angle, expected range, observed range, and tolerance;
- three scan timestamps and successful timestamped transforms;
- `/odom` and old-topic publisher endpoint counts;
- matched odometry/TF pose error;
- edge, topic, publisher GID, and expected graph owner table before and after
  bounded motion;
- full verification summary and post-run process audit.

### Remote review and completion evidence

Pending. Do not fill commit identities, PR, CI, merge, review, or solution-tag
fields until those events exist and are queryable.
