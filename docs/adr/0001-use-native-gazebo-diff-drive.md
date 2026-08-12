---
status: superseded by ADR-0002
---

# 通过 ROS Adapter 使用 Gazebo 原生 DiffDrive

VoiceNav Robot 的仿真移动底盘曾使用 Gazebo Sim 原生 DiffDrive，并经由 `ros_gz_bridge` Adapter
转换 model-scoped Gazebo Transport topic。这个选择使首个实现较小，沿用 Nav2 的 Gazebo odometry
路径，同时避免 Gazebo 名称泄漏到 Mission、Agent 或 audio Interface。

本决策描述已经完成的早期仿真基线。它已由
[ADR-0002](0002-migrate-to-gz-ros2-control.md) 对产品目标 supersede；历史决策及其证据仍然有效。

## 考虑过的方案

- Gazebo 原生 DiffDrive 加 `ros_gz_bridge`；
- `gz_ros2_control` 加 `diff_drive_controller`。

## 后果

原生 DiffDrive 有速度和加速度限制，但没有命令超时。因此每项手工测试都必须发送零速度；在接入
Nav2 或语音控制前，ROS 运动输出必须获得已配置的 watchdog。若项目加入真实硬件、多个 controller、
controller lifecycle 需求，或要求 base controller 自己拥有 timeout，则重新考虑迁移到
`gz_ros2_control`。

**已 supersede：**尽管项目仍只用于仿真，上述最后一个触发条件后来成为产品需求。
[ADR-0002](0002-migrate-to-gz-ros2-control.md) 记录后续决策；本文件保留原有后果作为历史决策记录。
