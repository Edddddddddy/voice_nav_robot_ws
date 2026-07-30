---
status: accepted
---

# Migrate the product control path to gz_ros2_control

The native Gazebo DiffDrive baseline made the first physics lessons small, but
it holds the last command, lacks a consumer timeout, and would require
commands, odometry, and TF to cross a Gazebo bridge. Starting from the VN-0007
target baseline and shipping in v0.2, VoiceNav Robot uses `gz_ros2_control`,
`diff_drive_controller`, a 0.35 s controller consumer timeout, and an
independent Motion Gate with a 250 ms Runtime-renewed authority lease.
Candidate velocity never renews that lease. `ros_gz_bridge` remains only for
`/clock` and `/scan`.

## Considered options

- retain native DiffDrive and add only an upstream watchdog;
- use native DiffDrive throughout v1 and bridge its control/odometry topics;
- migrate to gz_ros2_control and standard ros2_control controllers.

## Consequences

Native-DiffDrive lessons remain historical evidence but are not the product
path. `diff_drive_controller` becomes the sole owner of odometry and
`odom → base_footprint`; joint state and control no longer use
`ros_gz_bridge`. The migration adds controller-manager configuration, but makes
timeout ownership, Nav2 integration, controller lifecycle, and tests explicit.
