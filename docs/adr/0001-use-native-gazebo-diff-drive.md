---
status: superseded by ADR-0002
---

# Use Gazebo native DiffDrive behind a ROS adapter

VoiceNav Robot uses Gazebo Sim native DiffDrive for its simulation-only mobile
base and will translate its model-scoped Gazebo Transport topics through a
`ros_gz_bridge` adapter. This keeps the first implementation small and follows
the Nav2 Gazebo odometry path while preventing Gazebo names from leaking into
Mission, Agent, or audio Interfaces.

This decision describes the completed early learning baseline. It was
superseded for the product target by
[ADR-0002](0002-migrate-to-gz-ros2-control.md); historical lessons and evidence
remain valid.

## Considered options

- Gazebo native DiffDrive plus `ros_gz_bridge`;
- `gz_ros2_control` plus `diff_drive_controller`.

## Consequences

The original consequence was that Native DiffDrive has velocity and
acceleration limits but no command timeout, so every manual test had to send
zero velocity. The project later chose the standard-controller path even
without adding hardware because independent gate supervision, consumer timeout,
Nav2 integration, odometry/TF ownership, and a minimal bridge became product
requirements; ADR-0002 records that superseding trade-off.
