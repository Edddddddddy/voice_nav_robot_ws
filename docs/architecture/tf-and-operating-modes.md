# TF 与运行模式

**状态：**目标 v1.0 契约。

VoiceNav Robot 使用分别启动的 Mapping Mode 与 Navigation Mode。两个模式永不重叠，v1.0 不在线切换。

## Frame tree 与精确名称

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

wheel frame 名称严格为 `left_wheel` 与 `right_wheel`。joint name 是独立 configuration identifier，不能替代
frame name。

当前 model 使用圆柱 chassis：radius `0.20 m`、height `0.18 m`；wheel radius `0.035 m`、wheel width
`0.025 m`、wheel center `y = ±0.20 m`；caster radius `0.045 m`；LiDAR cylinder radius `0.04 m`、height
`0.05 m`。`base_link` 距 ground `0.035 m`，LiDAR 相对 pose 为 `[0.10, 0.00, 0.16]`，因此
`base_footprint → laser_link = [0.100, 0.000, 0.195]`。

wheel radius 为 `r`、wheel-center distance 为 `L = 0.40 m` 时，controller semantic 为：

```text
v_left  = v - ωL/2       v_right = v + ωL/2
v       = (v_right + v_left)/2
ω       = (v_right - v_left)/L
```

odometry 连续但可能 drift，不是 global map localization。每个 physical link 按需具有 visual、collision 与
inertial semantic，且拥有正 mass 和物理有效的 principal inertia。`base_footprint` 仅为 logical frame，
不得伪造 collision 或 inertia。

## 唯一 TF ownership

| 变换 | 建图模式所有者 | 导航模式所有者 |
| --- | --- | --- |
| `map → odom` | `slam_toolbox` | AMCL |
| `odom → base_footprint` | `diff_drive_controller` | `diff_drive_controller` |
| `base_footprint → base_link` | `robot_state_publisher` | `robot_state_publisher` |
| robot internal frame | `robot_state_publisher` | `robot_state_publisher` |

任一 composition 不能为同一 dynamic transform 包含两个 publisher。owner 改变时必须移除旧 publisher，并在同一
Issue 更新 TF contract test。

`LaserScan.header.frame_id` 严格为 `laser_link`。sensor placement 归 Xacro 与 `robot_state_publisher`，不归
duplicate static-transform process。

## 共同目标控制与传感器栈

两种模式均运行：

- 使用手写 differential-drive model 的 Gazebo Harmonic；
- `gz_ros2_control`；
- `joint_state_broadcaster`；
- 拥有 odometry 与 `odom → base_footprint` 的 `diff_drive_controller`；
- `robot_state_publisher`；
- 仅 bridge `/clock` 与 `/scan` 的 `ros_gz_bridge`；
- `mission_runtime_node` 与独立 `motion_gate_node`；
- 位于 Motion Gate 上游的 `nav2_velocity_smoother` 与 `nav2_collision_monitor`。

command、joint state、odometry 与 TF 不穿过 `ros_gz_bridge`。

## 模式矩阵

| 运行时 | 建图模式 | 导航模式 |
| --- | --- | --- |
| common simulation/control/sensor stack | on | on |
| `slam_toolbox` | on | off |
| map saver / pose-graph serializer | on | off |
| map server | off | on |
| AMCL | off | on |
| Nav2 planner/controller/behavior/lifecycle | off | on |
| 运行时配置 | `mode=mapping` | `mode=navigation` |

mode 在 process lifetime 内不可变。transition 意味着：

1. 请求 Operational Stop 并观察 locked zero；
2. 停止当前 composition；
3. 验证其 `map → odom` owner 已消失；
4. 在需要时使用明确 saved-map selection 启动另一 composition；
5. 读取新的 `/mission/state` Runtime instance 与 admission epoch。

## Mapping 模式

```text
Gazebo LiDAR ── /scan ──> slam_toolbox ──> map → odom
diff_drive_controller ──> odom + odom → base_footprint
semantic move/rotate Mission ──> fixed target motion chain
SAVE_MAP ──> logical occupancy map + pose graph artifact
```

允许的 Mission step 为 move-distance、rotate-angle 与 save-map。logical map ID 在 configured map root 下解析；
caller 不能提供 path。已完成 logical map 从 caller 视角 transactionally publish。

## Navigation 模式

```text
saved map ──> map server
/scan + initial pose ──> AMCL ──> map → odom
Named Place ──> Mission Runtime ──> Nav2 NavigateToPose
Nav2 velocity ──> nav2_velocity_smoother
              └──> nav2_collision_monitor ──> Motion Gate
```

允许的 Mission step 为 move-distance、rotate-angle 与 navigate-to-place。Mission Runtime 将 Named Place 解析为
frame `map` 内的 `PoseStamped`；Agent 永不接收或构造 raw map coordinate。

## Clock 与数据规则

- Physics、TF、SLAM、AMCL 与 Nav2 使用 `use_sim_time=true`。
- motion lease、controller liveness、cancellation bound、audio liveness 与 process supervision 使用 steady
  monotonic clock。
- pause simulation 不能保留或续约 stale motion lease。
- `diff_drive_controller` odometry 与 `odom → base_footprint` 使用同一 pose 与 timestamp source。
- `robot_state_publisher` 是 internal robot frame 的唯一 owner。
- `/scan` 必须可在其 message timestamp 从 `laser_link` transform。
- `/clock` 与 `/scan` bridge direction、type、name 和 QoS 是明确 contract-test input。

## 验收检查

每种 mode 均证明：

- transform 存在时，恰有一个 `map → odom` publisher；
- 恰有一个 `odom → base_footprint` publisher，且 owner 为 `diff_drive_controller`；
- 存在连接链 `map → odom → base_footprint → base_link → laser_link`；
- 精确的 `left_wheel` 与 `right_wheel` frame；
- LaserScan 在 message timestamp 可 transform；
- navigation data 无 mixed wall/simulation timestamp；
- 有界 motion 期间及 Operational Stop 后 TF 有效；
- 除 `/clock` 与 `/scan` 外无 command、joint-state、odometry 或 TF bridge。

Mapping acceptance 保存并重载 logical map。Navigation acceptance 加载该 artifact、接受 initial pose、完成
localization 并到达 Named Place。
