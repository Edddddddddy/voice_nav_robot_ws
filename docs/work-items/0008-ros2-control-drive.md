# VN-0008: Migrate the simulation drive path to ros2_control

**Status:** In Progress

**GitHub Issue:**
[#5](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/5)

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
- [ ] `voice_nav_sim` declares all runtime and test dependencies required by
  `gz_ros2_control`, controller manager, the joint-state broadcaster, the
  differential-drive controller, launch, and the clock bridge.
- [ ] The expanded robot description contains a
  `gz_ros2_control/GazeboSimSystem` with velocity command interfaces and
  position/velocity state interfaces for exactly
  `left_wheel_joint` and `right_wheel_joint`.
- [ ] The native `gz::sim::systems::DiffDrive` plugin is absent from the
  product model.
- [ ] The Gazebo ros2_control plugin receives its controller YAML through a
  launch-resolved package-share path; source and launch files contain no
  machine-specific absolute path.
- [ ] Controller configuration contains one `joint_state_broadcaster` and one
  `diff_drive_controller`, uses the existing 0.40 m separation and 0.035 m
  radius, and sets the consumer `cmd_vel_timeout` to 0.35 s.
- [ ] The differential-drive controller receives
  `geometry_msgs/msg/TwistStamped`; no unsupported
  `enable_stamped_cmd_vel` parameter is present.
- [ ] Launch startup is dependency-ordered rather than sleep-based: Gazebo and
  robot description are available before entity creation, then the
  joint-state broadcaster becomes active before the drive controller.
- [ ] `ros2 control list_controllers` reports both controllers active and
  `ros2 control list_hardware_interfaces` reports the intended wheel command
  and state interfaces.
- [ ] A bounded stamped forward command increases x, and a bounded positive
  angular command produces a positive yaw change.
- [ ] Motion changes wheel joint state and controller odometry on the
  controller-native `/diff_drive_controller/odom` endpoint; the odometry
  message and a basic TF query report frames `odom` and `base_footprint`.
- [ ] Documentation labels `/diff_drive_controller/odom` as an intentional
  intermediate checkpoint. It does not claim that the target product-level
  `/odom` remap or cross-graph publisher uniqueness is already complete.
- [ ] The manual fault-injection exercise terminates a non-zero command
  publisher and observes controller command output return to zero on the first
  control update after its age crosses 0.35 s while simulation time advances.
  The measured bound includes no more than one configured control period and
  the record distinguishes command zero from physical stationarity.
- [ ] The ROS–Gazebo bridge configuration contains only `/clock` in this
  lesson; velocity, joint state, odometry, and TF remain in ROS 2.
- [ ] Repository, Xacro/URDF/SDF, build, package-test, and full verification
  gates pass in WSL2 with ROS 2 Jazzy and Gazebo Harmonic.
- [ ] Lesson 0007 contains the tests-first workflow, failure injection,
  troubleshooting path, submission evidence, and reflection questions.
- [ ] The learner record contains real commands, results, commit identities,
  PR/CI links, and review findings before its status changes from Pending.
- [ ] A reviewed PR passes the required hosted CI and is rebase-merged to
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

Implementation, green-test, integration, PR, hosted-CI, public merge, and
solution-tag evidence remain pending.
