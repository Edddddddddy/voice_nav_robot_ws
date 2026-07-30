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

## Topic 与 frame 所有权

- 最终 ROS 速度消费者：`diff_drive_controller`。
- 动态 frame `odom → base_footprint`：只由 `diff_drive_controller` 发布。
- 机器人内部 frame：由 URDF 与 `robot_state_publisher` 发布。
- Mapping 的 `map → odom`：只由 `slam_toolbox` 发布。
- Navigation 的 `map → odom`：只由 AMCL 发布。
- `ros_gz_bridge` 只桥接 `/clock` 和 `/scan`，不桥速度、odom、joint state 或 TF。

## 停止与限速契约

- 所有速度必须经过可信 YAML 的配置上限，并拒绝非有限数值和非法轴。
- 运动测试在正常路径和异常清理路径都必须请求零速度。
- MotionGate 默认关闭，以 steady clock 管理只由 MissionRuntime 续期的短
  authority lease；候选速度只受独立新鲜度检查，不能续 authority。
  任一 deadline 失效后持续发布零。
- `diff_drive_controller.cmd_vel_timeout` 覆盖 MotionGate 进程崩溃后最后一条命令被保持的风险。
- 上层任务超时不能代替底层速度 lease。
- Stop 确认只表示 Gate 已禁止运动并发布零速度，不表示机器人已经物理停稳。
- 本项目的“停止”是高优先级 operational stop，不宣称为经过功能安全认证的急停系统。

资料：[Gazebo Sim 8 DiffDrive API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)、[gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)、[diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)。
