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

## 唯一发布者

| Transform | Mapping | Navigation |
| --- | --- | --- |
| `map → odom` | `slam_toolbox` | AMCL |
| `odom → base_footprint` | `diff_drive_controller` | `diff_drive_controller` |
| 机器人内部 frame | `robot_state_publisher` | `robot_state_publisher` |

`slam_toolbox` 与 AMCL 不能同时启动。`ros_gz_bridge` 是 transport adapter，不是 TF 语义 owner，也不桥接 `/tf`。静态模型阶段暂时没有 `map`、`odom`；TF 树从 `base_footprint` 开始，不要伪造未来的动态变换。

资料：[ROS 2 Jazzy：Using URDF with robot_state_publisher](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)。
