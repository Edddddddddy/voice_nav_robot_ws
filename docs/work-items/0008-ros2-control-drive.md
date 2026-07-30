# VN-0008: Migrate the simulation drive path to ros2_control

**Status:** Done

**GitHub Issue:**
[#5](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/5)

**GitHub PR:**
[#6](https://github.com/Edddddddddy/voice_nav_robot_ws/pull/6)

**Branch:** `feat/vn-0008-l0007-ros2-control-drive`

## Goal

Replace the historical native Gazebo DiffDrive product path with the smallest
working ROS 2 control slice:

```text
geometry_msgs/msg/TwistStamped
  -> diff_drive_controller
  -> gz_ros2_control
  -> Gazebo wheel joints
```

The slice must be reproducible from `course/0007-start`, launch without manual
controller-manager commands, drive forward and rotate in the expected
directions, and return to zero command. It establishes the control boundary
used by the remaining v0.2 lessons without claiming that the complete
MotionGate safety chain is already implemented.

For the pinned ROS 2 Jazzy controller, `~/cmd_vel` natively consumes
`geometry_msgs/msg/TwistStamped`. There is no supported
`enable_stamped_cmd_vel` parameter to add. This Work Item corrects the target
documentation wherever that stale parameter name appears.

## Non-goals

- Adding LiDAR, a non-empty test world, SLAM, AMCL, Nav2, or saved maps.
- Implementing `motion_gate_node`, the 250 ms Runtime authority lease,
  candidate freshness, writer binding, or managed Gazebo safe-pause.
- Completing the process-death latency gates for MissionRuntime or MotionGate.
- Performing the cross-graph and publisher-GID audit needed to prove unique TF
  ownership under every launch composition. This lesson proves the controller's
  basic odometry and `odom → base_footprint` output; the deeper ownership audit
  belongs to the following Work Item.
- Bridging velocity, joint state, odometry, or TF through `ros_gz_bridge`.
- Changing the robot geometry, Mission Interfaces, or voice/Agent behavior.
- Rewriting the historical Lesson 0005 checkpoint. It remains evidence of why
  the native Gazebo DiffDrive teaching slice was superseded.

## Acceptance criteria

- [x] Annotated tag `course/0007-start` points to the reviewed v0.1 solution
  commit and is present on the remote without rewriting another tag.
- [x] Contract tests fail before implementation when the native DiffDrive
  plugin is still the product drive path.
- [x] `voice_nav_sim` declares all runtime and test dependencies required by
  `gz_ros2_control`, controller manager, the joint-state broadcaster, the
  differential-drive controller, launch, and the clock bridge.
- [x] The expanded robot description contains a
  `gz_ros2_control/GazeboSimSystem` with velocity command interfaces and
  position/velocity state interfaces for exactly
  `left_wheel_joint` and `right_wheel_joint`.
- [x] The native `gz::sim::systems::DiffDrive` plugin is absent from the
  product model.
- [x] The Gazebo ros2_control plugin receives its controller YAML through a
  launch-resolved package-share path; source and launch files contain no
  machine-specific absolute path.
- [x] Controller configuration contains one `joint_state_broadcaster` and one
  `diff_drive_controller`, uses the existing 0.40 m separation and 0.035 m
  radius, and sets the consumer `cmd_vel_timeout` to 0.35 s.
- [x] The differential-drive controller receives
  `geometry_msgs/msg/TwistStamped`; no unsupported
  `enable_stamped_cmd_vel` parameter is present.
- [x] Launch startup is dependency-ordered rather than sleep-based: Gazebo and
  robot description are available before entity creation, then the
  joint-state broadcaster becomes active before the drive controller.
- [x] `ros2 control list_controllers` reports both controllers active and
  `ros2 control list_hardware_interfaces` reports the intended wheel command
  and state interfaces.
- [x] A bounded stamped forward command increases x, and a bounded positive
  angular command produces a positive yaw change.
- [x] Motion changes wheel joint state and controller odometry on the
  controller-native `/diff_drive_controller/odom` endpoint; the odometry
  message and a basic TF query report frames `odom` and `base_footprint`.
- [x] Documentation labels `/diff_drive_controller/odom` as an intentional
  intermediate checkpoint. It does not claim that the target product-level
  `/odom` remap or cross-graph publisher uniqueness is already complete.
- [x] The deterministic fault-injection test destroys a non-zero command
  publisher and observes controller command output return to zero on the first
  control update after its age crosses 0.35 s while simulation time advances.
  The measured bound includes no more than one configured control period and
  the record distinguishes command zero from physical stationarity.
- [x] The ROS–Gazebo bridge configuration contains only `/clock` in this
  lesson; velocity, joint state, odometry, and TF remain in ROS 2.
- [x] Repository, Xacro/URDF/SDF, build, package-test, and full verification
  gates pass in WSL2 with ROS 2 Jazzy and Gazebo Harmonic.
- [x] Lesson 0007 contains the tests-first workflow, failure injection,
  troubleshooting path, submission evidence, and reflection questions.
- [x] The learner record contains real commands, results, commit identities,
  PR/CI links, and review findings before its status changes from Pending.
- [x] A reviewed PR passes the required hosted CI and is rebase-merged to
  `main`; only then is annotated tag `course/0007-solution` created.

## Risks and rollback

- A wheel joint can expose the wrong command or state Interface while the
  model still expands successfully. Contract tests inspect both joint names
  and Interface names; runtime inspection confirms controller claims.
- Incorrect wheel order, separation, radius, or joint axis can make rotation
  direction or odometry wrong. Geometry values remain derived from the
  existing Xacro contract and manual movement checks exercise both signs.
- Controller spawners can race the embedded controller manager. Launch
  sequencing uses process-exit/event dependencies and bounded startup
  failures, not arbitrary sleeps.
- A stale parameter copied from another ROS distribution can be silently
  misleading or rejected at configure time. Tests and documentation pin the
  Jazzy `TwistStamped` contract and reject `enable_stamped_cmd_vel`.
- Reapplying acceleration limits inside `diff_drive_controller` would delay
  the consumer-deadman zero after the 0.35 s staleness decision. This
  controller keeps hard velocity bounds but leaves acceleration shaping to the
  target upstream `nav2_velocity_smoother`; a contract test rejects accidental
  double limiting.
- The 0.35 s timeout uses advancing controller/simulation time and does not
  prove immediate physical stationarity or safe arbitrary Gazebo pause.
  Evidence records command and pose/odometry observations separately.
- The branch can be reverted or abandoned and the learner can recreate the
  old state from immutable `course/0007-start`. Published course and release
  tags are never force-updated.

## Design impact

- Stable Interfaces changed: the product velocity input becomes
  `diff_drive_controller/~/cmd_vel` with
  `geometry_msgs/msg/TwistStamped`; the historical Gazebo Transport command
  endpoint is removed from the product path.
- TF or motion ownership changed: wheel command ownership moves from native
  Gazebo DiffDrive to `diff_drive_controller` and `gz_ros2_control`.
  Controller odometry/TF are introduced here on the controller-native
  `/diff_drive_controller/odom` endpoint; the product-level `/odom` remap and
  unique graph ownership are closed by the following Work Item.
- Bridge ownership changed: no velocity, joint-state, odometry, or TF bridge
  is introduced.
- ADR required: no new ADR. This implements accepted
  [ADR-0002](../adr/0002-migrate-to-gz-ros2-control.md) and keeps
  [ADR-0001](../adr/0001-use-native-gazebo-diff-drive.md) as superseded
  historical context.

## Test plan

- Unit: none; this slice has configuration and adapter behavior rather than a
  new deterministic core.
- Static: parse Xacro, generated URDF/SDF, controller YAML, package metadata,
  launch source, and course catalog; reject the native DiffDrive plugin,
  unsupported stamped-velocity parameter, and machine-specific paths.
- Contract: assert exact wheel joint/interface sets, controller geometry,
  `cmd_vel_timeout=0.35`, command message type, controller startup order, and
  bridge allowlist.
- Integration: launch headless Gazebo, wait for both controllers to become
  active, inspect hardware interfaces, publish bounded stamped commands, and
  shut down with a zero command.
- Fault injection: terminate a live command publisher and observe the
  controller's limited command return to zero on the first update after
  command age crosses 0.35 s, using simulation timestamps and a one-control-
  period measurement tolerance.
- Manual: record before/after Gazebo pose for forward and positive-yaw
  commands; capture controller and hardware-interface listings, odometry frame
  IDs from `/diff_drive_controller/odom`, and a basic
  `odom → base_footprint` TF query.
- Full gate: `bash scripts/verify.sh`.

## Documentation

- `docs/work-items/0008-ros2-control-drive.md`
- `docs/architecture/safety-and-motion-contract.md`
- `course/catalog.toml`
- `course/lessons/0007-migrate-to-ros2-control-drive.md`
- `course/records/0007-ros2-control-drive.md`
- `CHANGELOG.md`

## Verification evidence

### Immutable start checkpoint

- Created and pushed annotated tag `course/0007-start` before opening the
  implementation branch.
- Both the tag and `v0.1.0` peel to the reviewed v0.1 solution commit
  `d76a560af5c8e4a5f2c050dec39b591e6b1d781e`.

### Tests-first red evidence

Environment: WSL2 `Ubuntu-24.04`, ROS 2 Jazzy, 2026-07-31.

```text
Command:
  python3 -m unittest discover -s tests -p 'test_*.py' -v
Exit status: 1
Tests: 29
Expected failure:
  test_repository_control_contract_passes
  Control contract failed: native Gazebo DiffDrive plugin must not remain
  in the product model
Other new contract fixtures: passed
```

This is the intended product-contract failure against the unmodified v0.1
model. The checker executed, its valid fixture passed, and its invalid plugin,
joint, Interface, timeout, frame, geometry, and unsupported-parameter fixtures
all passed. The red state is not caused by a syntax, discovery, fixture-path,
or missing-ROS error.

### Resolved local dependency versions

The development environment resolved these Ubuntu/ROS packages before the
green implementation:

```text
gz_ros2_control: 1.2.19
ros2_controllers / diff_drive_controller / joint_state_broadcaster: 4.40.1
controller_manager: 4.45.2
```

These local apt versions are environment evidence, not a substitute for
declaring package dependencies and verifying a clean CI resolution.

The green implementation evidence follows. The local implementation commit is
`ed621353c6ab4c3544a29b5763106a63539833d9`; its public rebase identity is
`66d771f302fdcc9ae160e706bcaa54797582734f`.

### Local implementation and integration evidence

Environment: WSL2 `Ubuntu-24.04`, ROS 2 Jazzy, Gazebo Sim 8.11.0,
2026-07-31.

```text
Canonical full-workspace command:
  bash scripts/verify.sh
Repository contracts:
  38 tests, all passed
Build result:
  6 packages finished
Package result:
  32 tests, 0 errors, 0 failures, 1 skipped
Final marker:
  VoiceNav Robot verification passed.

Controllers:
  diff_drive_controller active
  joint_state_broadcaster active
Claimed command Interfaces:
  left_wheel_joint/velocity
  right_wheel_joint/velocity
Command type:
  geometry_msgs/msg/TwistStamped

Hard-limit output:
  linear.x=0.400, angular.z=1.200
Gazebo ground-truth forward x:
  0.077 -> 0.222 m; delta=+0.145 m
Gazebo ground-truth yaw:
  0.249 -> 0.734 rad; delta=+0.485 rad
Consumer deadman simulation stamps:
  8.564 -> 8.919 s; delta=0.355 s
Allowed upper bound:
  0.35 + 0.010 control period + 0.002 simulation-step epsilon = 0.362 s
Physical-stationarity observation:
  |linear.x| < 0.02 m/s and |angular.z| < 0.02 rad/s
```

The test destroys the dedicated non-zero publisher endpoint before waiting for
the timeout. It queries Gazebo model pose independently rather than treating
controller-computed odometry as physical ground truth. All launch-owned
processes exited cleanly, and a post-run process audit found no simulator or
ROS graph residue.

### Remote review and completion evidence

- [Issue #5](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/5)
  tracked the Work Item and closed when the implementation merged.
- [PR #6](https://github.com/Edddddddddy/voice_nav_robot_ws/pull/6) contains
  only the VN-0008 implementation, course, architecture, and test scope.
- The required
  [`required / ubuntu-24.04 / ros-jazzy`](https://github.com/Edddddddddy/voice_nav_robot_ws/actions/runs/30565921460/job/90950343524)
  check passed in 5 minutes 6 seconds on feature head
  `bfac7a98fcfe322bec24ecf25a7536eeb81c479a`.
- The
  [independent review record](https://github.com/Edddddddddy/voice_nav_robot_ws/pull/6#issuecomment-5134178763)
  reports no remaining P0/P1 finding and explicitly bounds deferred Lesson
  0008–0010 work.
- PR #6 was rebase-merged at `2026-07-30T17:32:07Z`; the public
  implementation tip is `68bb49051680e1cd5cd138982b70bd4c89c5c920`.
- The remote feature branch was automatically deleted after merge.

GitHub's rebase merge produced these exact public identities:

| Local feature identity | Public rebase identity | Subject |
| --- | --- | --- |
| `a7feee41ee9330676a09ebc50c4461a9af76a90e` | `b4ebb81a96933f7495e82df96096563673283cd7` | `test(sim): define Lesson 0007 control contract` |
| `ed621353c6ab4c3544a29b5763106a63539833d9` | `66d771f302fdcc9ae160e706bcaa54797582734f` | `feat(sim): migrate drive path to ros2 control` |
| `742c82956a67a84a20d22b982bee14759712bdfc` | `4dfae5f97bae84fb462d230d5cd99aeffab6e6bc` | `docs(course): record Lesson 0007 implementation evidence` |
| `bfac7a98fcfe322bec24ecf25a7536eeb81c479a` | `68bb49051680e1cd5cd138982b70bd4c89c5c920` | `docs(work-item): record Lesson 0007 review identity` |

Only after that reviewed public merge, annotated tag
`course/0007-solution` was created. Tag object
`a4e75e8205f1e59c516e28d5ef8f7e02c30aaaad` peels to the public reviewed
solution `68bb49051680e1cd5cd138982b70bd4c89c5c920`; neither the start nor solution
tag was rewritten.

The closure diff, including the completed course status and exact remote
identity map, then passed the full local gate:

```text
Command:
  bash scripts/verify.sh
Repository contracts:
  38 tests, all passed
Build result:
  6 packages finished
Package result:
  32 tests, 0 errors, 0 failures, 1 skipped
Final marker:
  VoiceNav Robot verification passed.
Post-run process/source audit:
  no Gazebo/ROS process residue; no source __pycache__ or *.pyc
```
