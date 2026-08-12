---
status: accepted
---

# 将产品控制路径迁移至 gz_ros2_control

原生 Gazebo DiffDrive 基线使早期物理切片较小，但它保留最后一条命令、没有消费者超时，并会使命令、
odometry 和 TF 穿过 Gazebo bridge。自 VN-0007 目标基线起并在 v0.2 交付中，VoiceNav Robot 使用
`gz_ros2_control`、`diff_drive_controller`、`0.35 s` controller consumer timeout，以及一个由 Runtime
续约 `250 ms` authority lease 的独立 Motion Gate。candidate velocity 永远不能续约该 lease；
`ros_gz_bridge` 仅保留 `/clock` 与 `/scan`。

## 考虑过的方案

- 保留原生 DiffDrive，仅增加上游 watchdog；
- 在整个 v1 中使用原生 DiffDrive 并 bridge 控制/odometry topic；
- 迁移到 `gz_ros2_control` 和标准 ros2_control controller。

## 后果

native-DiffDrive 路径保留为历史证据，但不再是产品路径。`diff_drive_controller` 成为 odometry 和
`odom → base_footprint` 的唯一 owner；joint state 与 control 不再使用 `ros_gz_bridge`。迁移增加了
controller-manager 配置，却使 timeout ownership、Nav2 集成、controller lifecycle 和测试明确可见。
