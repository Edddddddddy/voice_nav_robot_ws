# 差速驱动与里程计契约

VoiceNav Robot reference

## 坐标与符号

- x 向前，y 向左，z 向上。
- 线速度 `v > 0` 表示前进。
- 角速度 `ω > 0` 表示从上方看逆时针，即左转。
- `r` 是轮半径，`L` 是左右轮中心线的完整距离。
- 当前模型固定为 `r = 0.035 m`、`wheel_y = 0.20 m`，所以 `L = 2 × wheel_y = 0.40 m`。

## 从底盘速度得到轮速

```text
左轮线速度：v_left  = v - ωL/2
右轮线速度：v_right = v + ωL/2

左轮角速度：ω_left  = v_left  / r
右轮角速度：ω_right = v_right / r
```

直行时两轮同向同速；正角速度原地旋转时，左轮向后、右轮向前。

## 从轮速得到里程计速度

```text
v = (v_right + v_left) / 2
ω = (v_right - v_left) / L
```

控制器对这些速度积分，得到连续的 `odom → base_footprint` 位姿。轮径误差会影响直线距离与转角；轮距误差主要影响转角。里程计可连续但会积累漂移，它不是全局地图定位。

## 教学链与产品链

Lesson 0005 有意采用最小的 Gazebo Transport 链，便于独立验证几何与物理：

```text
gz.msgs.Twist → Gazebo native DiffDrive → wheel joints
                                      └→ Gazebo odometry
```

它没有 command timeout，只能作为教学 checkpoint。产品基线从 Lesson 0007 起使用：

```text
Nav2 / 相对运动候选速度
→ nav2_velocity_smoother
→ nav2_collision_monitor
→ 独立 MotionGate
→ diff_drive_controller
→ gz_ros2_control
→ Gazebo wheel joints
```

Mission、Agent 和 Voice 不知道 Gazebo topic，也不能直接控制轮关节。`nav2_velocity_smoother` 只调速，Collision Monitor 只做防碰撞保护，独立 MotionGate 是最终速度裁决者，`diff_drive_controller.cmd_vel_timeout` 是消费端第二道 deadman。

Lesson 0008 已验证到 LiDAR、controller、odom 与 TF 所有权。Lesson 0009 /
VN-0010 正在用 test authority/candidate harness 实现独立 MotionGate 的正常
lease 与 deadline；它尚不是已完成能力。进程 kill、consumer crash-stop 和
Gazebo pause/resume 留给 Lesson 0010。MissionRuntime、smoother、Collision
Monitor 的完整串接属于后续纵向切片。

## Topic 与 frame 所有权

- 最终 ROS 速度消费者：`diff_drive_controller`。
- MotionGate 是 `/diff_drive_controller/cmd_vel` 的唯一 publisher；使用
  `rclcpp::SystemDefaultsQoS()`，由运行时检查证明与 controller subscriber
  实际兼容，不把 introspection 的 `UNKNOWN` policy 硬断言成固定值。
- 动态 frame `odom → base_footprint`：只由 `diff_drive_controller` 发布。
- 机器人内部 frame：由 URDF 与 `robot_state_publisher` 发布。
- Mapping 的 `map → odom`：只由 `slam_toolbox` 发布。
- Navigation 的 `map → odom`：只由 AMCL 发布。
- `ros_gz_bridge` 只桥接 `/clock` 和 `/scan`，不桥速度、odom、joint state 或 TF。

## 停止与限速契约

- 所有速度必须经过可信 YAML 的配置上限。有限 `linear.x`/`angular.z`
  超限时 CLAMP；NaN、Inf 或 unsupported axis 非零时 retire 当前 lease
  并选择零。
- 运动测试在正常路径和异常清理路径都必须请求零速度。
- MotionGate 默认关闭。PREPARE 由 Gate 生成 lease ID 和 per-lease candidate
  topic；topic 位于 `/voice_nav_internal/motion_gate/candidate/lease_` 前缀。
  OPEN、RENEW、INHIBIT 引用当前 lease。四种操作共享 Gate-wide CAS
  `control_seq`，旧 instance/lease/sequence 的 INHIBIT 不能关闭新 lease。
- authority lease 为 250 ms steady time；candidate freshness 为独立的
  150 ms steady deadline，candidate 使用
  `BEST_EFFORT + VOLATILE + KEEP_LAST(1)`，不能续 authority。任一 deadline
  失效后 Gate 每 20 ms wall time 持续发布零。
- OPEN 在 Gate 本进程 graph 中要求恰好一个 writer，销毁 provisional
  reader/queue 后重建 `VOLATILE + KEEP_LAST(1)` reader，并绑定 Gate 本地
  观察到的完整 16-byte endpoint GID。control request 不传 caller
  `Publisher::get_gid()`；locked RMW 自检不能关联 graph GID 与
  `MessageInfo.publisher_gid` 时 fail closed。
- control、candidate、timer 和 publication 经过同一 serial barrier；
  current tuple 的 INHIBIT、expiry 或 invalid retirement 发布零后，旧的
  queued non-zero 不得再越过 barrier。
- `diff_drive_controller.cmd_vel_timeout` 覆盖 MotionGate 进程崩溃后最后一条命令被保持的风险。
- 上层任务超时不能代替底层速度 lease。
- Stop 确认只表示 Gate 已禁止运动并发布零速度，不表示机器人已经物理停稳。
- 本项目的“停止”是高优先级 operational stop，不宣称为经过功能安全认证的急停系统。

`InternalMotionGateControl` 与 `InternalMotionGateState` 只在
`voice_nav_mission` 内生成；状态名是
`INHIBITED`/`PREPARED`/`ARMED`/`FAULTED`。package-private 只表示产品不承诺
外部兼容，并不提供 DDS 身份认证或授权。节点 FQN 固定为
`/motion_gate_node`，private endpoint 固定为
`/motion_gate/internal/control` 与 `/motion_gate/internal/state`。这些名称、
candidate prefix 与 `/diff_drive_controller/cmd_vel` 都是代码常量，不是
YAML 参数或 product launch remap；参数 YAML 根固定为 `motion_gate_node`。

资料：[Gazebo Sim 8 DiffDrive API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)、[gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)、[diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)。
