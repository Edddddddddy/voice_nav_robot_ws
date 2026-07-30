# Lesson 0005 学习记录：在 Gazebo 中驱动差速机器人

状态：Completed

学习者手写 Gazebo native DiffDrive 配置，从两个单侧轮偏移推导完整轮距，降低固定支撑球的横向摩擦，并保持 `odom → base_footprint` frame 契约。

## 验收证据

- Xacro、URDF、SDF、依赖、构建和测试全部通过。
- 直行与正角速度左转均在仿真中得到验证。
- 每次运动测试都显式发送零 Twist 并确认 pose 不再变化。
- 能解释 `wheel_separation = 2 × wheel_y`，以及发布进程退出不代表机器人停车。

该课程也暴露了产品架构必须修复的缺口：native DiffDrive 会保持最后一条速度命令。Lesson 0007 起迁移到带消费端 timeout 的 `diff_drive_controller`。
