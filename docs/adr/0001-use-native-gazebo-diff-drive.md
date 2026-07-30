---
status: accepted
---

# Use Gazebo native DiffDrive behind a ROS adapter

VoiceNav Robot uses Gazebo Sim native DiffDrive for its simulation-only mobile
base and will translate its model-scoped Gazebo Transport topics through a
`ros_gz_bridge` adapter. This keeps the first implementation small and follows
the Nav2 Gazebo odometry path while preventing Gazebo names from leaking into
Mission, Agent, or audio Interfaces.

## Considered options

- Gazebo native DiffDrive plus `ros_gz_bridge`;
- `gz_ros2_control` plus `diff_drive_controller`.

## Consequences

Native DiffDrive has velocity and acceleration limits but no command timeout,
so every manual test must send zero velocity and the ROS motion output must gain
a configured watchdog before Nav2 or voice control is connected. Migration to
`gz_ros2_control` is reconsidered if the project adds real hardware, multiple
controllers, controller lifecycle requirements, or requires the base
controller itself to own the timeout.
