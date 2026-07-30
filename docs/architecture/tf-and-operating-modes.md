# TF and operating modes

**Status:** Target v1.0 contract

VoiceNav Robot uses separate Mapping and Navigation Mode launch compositions.
The modes never overlap and do not switch online in v1.0.

## Frame tree and exact names

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── left_wheel
            ├── right_wheel
            ├── caster_link
            └── laser_link
```

The wheel frame names are exactly `left_wheel` and `right_wheel`. Joint names
are separate configuration identifiers and must not be substituted for frame
names.

## Unique TF ownership

| Transform | Mapping Mode owner | Navigation Mode owner |
| --- | --- | --- |
| `map → odom` | slam_toolbox | AMCL |
| `odom → base_footprint` | diff_drive_controller | diff_drive_controller |
| `base_footprint → base_link` | robot_state_publisher | robot_state_publisher |
| robot internal frames | robot_state_publisher | robot_state_publisher |

No composition may contain two publishers for one dynamic transform. An owner
change removes the prior publisher and updates TF contract tests in the same
Work Item.

`LaserScan.header.frame_id` is exactly `laser_link`. Sensor placement is owned
by Xacro and robot_state_publisher, not a duplicate static-transform process.

## Common target control and sensor stack

Both modes run:

- Gazebo Harmonic with the hand-written differential-drive model;
- `gz_ros2_control`;
- `joint_state_broadcaster`;
- `diff_drive_controller`, which owns odometry and
  `odom → base_footprint`;
- robot_state_publisher;
- `ros_gz_bridge` for `/clock` and `/scan` only;
- `mission_runtime_node` and independent `motion_gate_node`;
- nav2_velocity_smoother and nav2_collision_monitor upstream of Motion Gate.

Commands, joint state, odometry, and TF do not cross `ros_gz_bridge`.

## Mode matrix

| Runtime | Mapping Mode | Navigation Mode |
| --- | --- | --- |
| common simulation/control/sensor stack | on | on |
| slam_toolbox | on | off |
| map saver / pose-graph serializer | on | off |
| map server | off | on |
| AMCL | off | on |
| Nav2 planner/controller/behavior/lifecycle | off | on |
| Runtime configuration | `mode=mapping` | `mode=navigation` |

Mode is immutable for the process lifetime. A transition means:

1. request Operational Stop and observe locked zero;
2. stop the current composition;
3. verify its `map → odom` owner is gone;
4. start the other composition with an explicit saved-map selection where
   required;
5. read the new `/mission/state` Runtime instance and admission epoch.

## Mapping Mode

```text
Gazebo LiDAR ── /scan ──► slam_toolbox ──► map → odom
diff_drive_controller ──► odom + odom → base_footprint
semantic move/rotate Mission ──► fixed target motion chain
SAVE_MAP ──► logical occupancy map + pose graph artifacts
```

Allowed Mission steps are move-distance, rotate-angle, and save-map. A logical
map ID resolves below a configured map root. The caller cannot provide a path.
A completed logical map is published transactionally from the caller's view.

## Navigation Mode

```text
saved map ──► map server
/scan + initial pose ──► AMCL ──► map → odom
Named Place ──► Mission Runtime ──► Nav2 NavigateToPose
Nav2 velocity ──► nav2_velocity_smoother
              ──► nav2_collision_monitor ──► Motion Gate
```

Allowed Mission steps are move-distance, rotate-angle, and navigate-to-place.
Mission Runtime resolves a Named Place to a `PoseStamped` in frame `map`. The
Agent never receives or constructs raw map coordinates.

## Clock and data rules

- Physics, TF, SLAM, AMCL, and Nav2 use `use_sim_time=true`.
- Motion leases, controller liveness, cancellation bounds, audio liveness, and
  process supervision use a steady monotonic clock.
- Pausing simulation cannot preserve or renew a stale motion lease.
- diff_drive_controller odometry and `odom → base_footprint` use one pose and
  timestamp source.
- robot_state_publisher is the sole owner of internal robot frames.
- `/scan` must be transformable from `laser_link` at its message timestamp.
- `/clock` and `/scan` bridge directions, types, names, and QoS are explicit
  contract-test inputs.

## Acceptance checks

Each mode proves:

- exactly one `map → odom` publisher when that transform exists;
- exactly one `odom → base_footprint` publisher and that owner is
  diff_drive_controller;
- the connected chain
  `map → odom → base_footprint → base_link → laser_link`;
- exact `left_wheel` and `right_wheel` frames;
- LaserScan transformability at message timestamps;
- no mixed wall/simulation timestamps in navigation data;
- valid TF through bounded motion and after Operational Stop;
- no command, joint-state, odometry, or TF bridge beyond `/clock` and `/scan`.

Mapping acceptance saves and reloads a logical map. Navigation acceptance loads
that artifact, accepts an initial pose, localizes, and reaches a Named Place.
