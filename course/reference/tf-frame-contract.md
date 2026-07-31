# TF Frame Contract

VoiceNav Robot reference

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

## Frame 的职责

- `map`：长期地图坐标；Mapping 时由 `slam_toolbox`、Navigation 时由 AMCL 负责。
- `odom`：局部连续里程计坐标，允许长期漂移。
- `base_footprint`：机器人在地面的二维投影，适合 2D 导航。
- `base_link`：机器人实体主体坐标。
- `laser_link`：LiDAR 安装坐标，后续 LaserScan 的 `frame_id` 必须与它一致。

当前 Xacro 的关键几何为：圆柱底盘半径 `0.20 m`、高度 `0.18 m`；轮半径 `0.035 m`；轮心 y 为 `±0.20 m`。`base_link` 比地面高 `0.035 m`，`laser_link` 相对 `base_link` 为 `[0.10, 0.00, 0.16]`，因此：

```text
base_footprint → laser_link = [0.100, 0.000, 0.195]
```

## 当前课程 checkpoint

- Lesson 0003 只有 `robot_state_publisher` 发布的机器人内部静态/关节
  frame；当时不存在 `map` 或 `odom`。
- Lesson 0005 的原生 Gazebo DiffDrive 在 Gazebo Transport 中生成
  odometry/TF 数据，但仓库尚未建立产品标准 ROS TF 链，也不能把它写成
  `diff_drive_controller` 已经在运行。
- Lesson 0007 已迁移到 `gz_ros2_control`。`diff_drive_controller` 发布
  controller-native odometry 和 `odom → base_footprint`，而
  `robot_state_publisher` 发布机器人内部 frame；当时产品级 `/odom`
  direct remap、LiDAR 和跨 graph 唯一所有权审计仍未完成。

## 如何证明一条 TF edge 只有一个 owner

`ros2 topic info /tf --verbose` 的 publisher 数量不是 TF owner 数量。
多个合法 node 会把不同 edge 复用到 `/tf`，一个 publisher 也可在一条
`TFMessage` 中发送多条 edge。node name 同样不是 endpoint identity：
两个独立进程可以使用相同 node name。

Lesson 0008 的验收按以下关系审计 `/tf` 和 `/tf_static`：

```text
(expected_topic, parent_frame, child_frame)
  → received MessageInfo.publisher_gid
  → graph topic endpoint GID
  → expected fully qualified node owner
```

每条 observed edge 的 GID set 必须恰好有一个元素，并能关联到预期 graph
endpoint；edge 出现在错误的 `/tf` 或 `/tf_static` 上同样必须失败。
owner 使用绝对 fully qualified node name，不能用会跨 namespace 误匹配的
短名。相同 node 在 `/tf` 与 `/tf_static` 上拥有不同 publisher GID 是
正常现象；唯一性约束作用于语义 edge，而不是整个 topic。`/tf_static`
订阅必须兼容 transient-local QoS，才能在测试晚启动时收到已有 fixed
transforms。VIOLATION 可以立即失败，但成功必须等完整观测窗口结束，避免
漏掉稍晚出现的第二 writer。

详细实践见
[Lesson 0008](../lessons/0008-lidar-world-tf-ownership.md)。

## v1.0 目标唯一发布者

| Transform | Mapping | Navigation |
| --- | --- | --- |
| `map → odom` | `slam_toolbox` | AMCL |
| `odom → base_footprint` | `diff_drive_controller` | `diff_drive_controller` |
| 机器人内部 frame | `robot_state_publisher` | `robot_state_publisher` |

这张表从 Lesson 0007 的 `gz_ros2_control` 迁移开始逐步实现，不描述当前
0001–0006 checkpoint。目标状态中 `slam_toolbox` 与 AMCL 不能同时启动；
`ros_gz_bridge` 是 transport adapter，不是 TF 语义 owner，也不桥接
`/tf`。

资料：[ROS 2 Jazzy：Using URDF with robot_state_publisher](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)。
