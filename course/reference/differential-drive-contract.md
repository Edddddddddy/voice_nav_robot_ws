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

Lesson 0008 已验证 LiDAR、controller、odom 与 TF 所有权。Lesson 0009 /
VN-0010 的 test authority/candidate harness 与独立 MotionGate 已通过
exact-head 本地门禁、required CI 和 rebase merge，并发布不可变
`course/0009-solution`。进程 kill、consumer crash-stop 和 Gazebo
pause/resume 仍留给 Lesson 0010。
MissionRuntime、smoother、Collision Monitor 的完整串接属于后续纵向切片。

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
  IDL 对 request/Gate/lease ID 的 transport bound 是 36 字符，但运行时只
  接受 exact-32 lowercase-hex request/Gate ID；PREPARE lease 为空，其余操作
  的 lease 必须 exact-32。
- authority lease 为 250 ms steady time；candidate freshness 为独立的
  150 ms steady deadline，candidate 使用
  `BEST_EFFORT + VOLATILE + KEEP_LAST(1)`，不能续 authority。任一 deadline
  失效后 Gate 每 20 ms wall time 持续发布零。
- canonical Gate 启动要求 `use_sim_time=true`，并拒绝运行期修改。每次最终
  publication 都重新检查该参数和 active ROS clock；任一不变量丢失都会锁存
  fault，只发布 zero + zero stamp。ROS time 只用于最终 `TwistStamped`、
  odom、TF 和传感器时间戳，不驱动 authority、freshness 或其他 deadline。
- OPEN 先完成纯 Core request/state/CAS/lease/deadline 校验；拒绝路径不能
  访问 graph。随后使用 discard reader A、discard reader B 与第一个
  accepting reader C，在三次 graph snapshot 中要求同一个唯一 writer，
  并绑定 Gate 本地观察到的完整 16-byte endpoint GID。任一 snapshot 的
  writer 改变都 fail closed；control request 不传 caller
  `Publisher::get_gid()`。
- 当前 GID 关联严格锁定 `rmw_fastrtps_cpp`：product launch 显式选择它，
  Gate 在其他 RMW 下拒绝启动，manifest 声明 runtime dependency。FastDDS
  self-test 不能关联 graph GID 与 `MessageInfo.publisher_gid` 时 fail closed。
- control、candidate、timer 和 publication 经过同一 serial barrier；
  current tuple 的 INHIBIT、expiry 或 invalid retirement 发布零后，旧的
  queued non-zero 不得再越过 barrier。
- `diff_drive_controller.cmd_vel_timeout` 被配置为 MotionGate 进程崩溃后的
  消费端第二道 deadman；其 process-kill 实测属于 Lesson 0010，不能用配置
  存在代替证据。
- Lesson 0010 的轮端证据必须订阅 ros2_control
  `/controller_manager/introspection_data/full`，分别检查左右轮
  `command_interface.<joint>/velocity` 与 `state_interface.<joint>/velocity`。
  订阅 QoS 必须兼容其 `BEST_EFFORT + TRANSIENT_LOCAL + KEEP_LAST(1)` publisher，
  并在故障前先建立完整、有限、严格递增、左右 command 非零的 baseline。
  command-interface 值表示下一次同步 hardware write 将消费的值，不是 Gazebo
  已执行回执；这个有损 topic 既不能证明 exact first write，也不能证明中间没有
  command regression。A/B 都必须使用默认关闭的 test-only lossless
  hardware-write ledger。测试 Adapter 继承公开
  `GazeboSimSystemInterface`，原样委托给 pluginlib-loaded upstream
  `gz_ros2_control/GazeboSimSystem`，再在实际 write seam 逐次计入调用、generation、
  simulation stamp、上游返回值和左右轮值；ledger 必须有单调 `write_seq`、atomic
  ARM/SEAL fences、容量证明、overflow/overwrite fail-closed 与分页
  checksum/连续性验证。只有 generation、stamp、返回值和轮速位模式完全相同的
  连续调用才能折叠成带 sequence range/count 的 segment；paused 时合法重复写不
  无限占槽，但每次调用仍被计数。公开 hardware Interface 不含 Gazebo iteration，
  真实 iteration 由 World Statistics 独立证明并通过 fence 区间关联；introspection
  仍作为 mandatory corroboration。
  Gate process death 后，旁路 observer 看到的最后 input 不能代表 controller
  callback 真正接受的最后 input。MotionGate 会每 20 ms 重发当前 tuple，所以不能
  假设每条安全 command 都唯一。Gate-kill attempt 必须使用 generation 内此前未
  使用的 final marker；parent-owned Gate event journal 的 INTENT/COMMITTED output
  lane 证明它只 COMMIT 一次，匹配 non-zero `/cmd_vel_out` 在下一次重发前 ACK，
  随后 exact SIGKILL。若死亡前出现第二条 publish，本 generation 失败并重试；
  publisher 消失和 100 ms quiet 只作 cleanup evidence。
  `/cmd_vel_out` 是 controller 底盘命令，`/joint_states` 是状态，`/odom` 是
  物理静止代理；不能把任何一层冒充另一层。该 process-kill 验收目前仍为
  VN-0011A planned contract，尚未倒写成 Lesson 0009 已完成能力。
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
`motion_gate_core` 是 package-internal STATIC target，不安装或 export；唯一
安装的运行目标是 `motion_gate_node`。Core 拥有状态与 selected command，
其 typed surface 还包含只读 `selected_command()`；Adapter-only
`force_fault()` 只把 graph/reader/clock/publication failure 锁存到 Core。
Node Adapter 拥有 reader lifecycle、实际 publication 与 zero-published
acknowledgement。

资料：[Gazebo Sim 8 DiffDrive API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)、[gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)、[diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)。
