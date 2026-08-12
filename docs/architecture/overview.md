# 架构概览

本文件区分已验证的当前行为与已批准的 v1.0 目标。目标 Module 不表示已经实现。

## v0.1 基础中的当前实现

在本次文档迁移前已验证：

- active bounded Mission V1 `MissionStep.msg`、`ExecuteMission.action`、`MissionState.msg` 与
  `StopMission.srv`，具有 generated type contract test；
- 手写 physical differential-drive Xacro；
- static `robot_state_publisher` launch 与 internal TF；
- Gazebo native DiffDrive motion、configured limit 与 odometry；
- audio、Agent、Mission、bringup 的 package skeleton；
- 已完成的 package maintainer/description metadata；
- 统一 local verification command。

native Gazebo DiffDrive 是历史学习行为，不是产品目标。[ADR-0001](../adr/0001-use-native-gazebo-diff-drive.md)
已被 [ADR-0002](../adr/0002-migrate-to-gz-ros2-control.md) supersede。v0.1 checkpoint 时，目标 ros2_control
stack、2D LiDAR bridge、SLAM、Nav2、Mission Runtime、Motion Gate 和 local voice pipeline 均不是当前 claim。

## 当前 v0.2 仿真与 MotionGate 切片

仓库、静态检查和无头 Gazebo 门禁已验证：

- 产品模型使用 `gz_ros2_control/GazeboSimSystem`；
- Jazzy `diff_drive_controller` 拥有两个车轮速度 command；
- 其原生输入是 `geometry_msgs/msg/TwistStamped`；
- `joint_state_broadcaster` 发布车轮状态；
- controller 直接发布产品 `/odom` 与 `odom → base_footprint`；
- `cmd_vel_timeout=0.35` 提供消费者侧 deadman；
- 已安装的自包含测试世界具有固定解析障碍物；
- 一个 `360°` 的单层 GPU LiDAR 发布坐标系 `laser_link`；
- `/clock` 与 `/scan` 是本切片唯一 ROS–Gazebo bridge；
- 在有界观察窗口内覆盖 scan 时间的 TF、匹配的 odometry/TF pose，以及每条 TF 边的 topic、publisher
  GID 与完全限定 owner。

SLAM、完整物理 Nav2 运动链、Agent 与语音仍是目标能力。Mission Runtime 控制面、生产 RelativeMotion Adapter
与包内调节模块是下述当前切片。相对运动控制以基于 odometry 闭环的 MOVE/ROTATE 模块交付；无头物理
raw-stamp-age 与 TF 验收由 Issue #72 独立追踪。尚无 `map → odom` owner。controller timeout 已配置，
却不能当作 Gate 死亡或物理停止完成；进程死亡验收是独立目标切片。

## 当前独立 MotionGate 切片

仓库在 exact-head 本地验证、独立审查、required CI 与 rebase 合并后，公开交付独立验证的
MotionGate 垂直切片：

- 纯粹的包内静态 `MotionGateCore`，提供强类型
  `prepare`/`open`/`renew`/`inhibit`/`accept_candidate`/`tick`/`snapshot`/`selected_command` 方法；
  仅 Adapter 使用的 `force_fault` 入口、`PrepareAdmissionProvider`/`OpenBindingProvider` seam 与一个
  `motion_gate_node` ROS Adapter；Core 库/header 不安装或导出；
- 包内的 `InternalMotionGateControl` 与 `InternalMotionGateState` 类型，只含
  `PREPARE`/`OPEN`/`RENEW`/`INHIBIT`，只暴露在 `/motion_gate/internal/control` 与
  `/motion_gate/internal/state`；IDL 最多 `36` 字符，而 request/Gate identity 与每个非 PREPARE 租约 identity
  在语义上严格为 `32` 个小写十六进制字符；PREPARE 携带空租约字段；
- Gate 生成的 lease ID/candidate topic、一个全局 compare-and-swap `control_seq`，以及
  `INHIBITED`/`PREPARED`/`ARMED`/`FAULTED` state；
- Gate 本地 `16-byte` publisher-GID binding，OPEN barrier 使用丢弃 reader A/B、首个可接受 reader C、
  三个同 writer graph snapshot，随后才通过最终发布 barrier；
- `250 ms` 权限、`150 ms` candidate 新鲜度与 `20 ms` steady/wall 输出周期；
- 有限的支持轴 clamp；对非有限/不支持轴输入以 fail-closed 退役；唯一最终 command owner；
- 最终 publisher 使用 `rclcpp::SystemDefaultsQoS()`，以 runtime proof 验证与 controller subscriber 的兼容性；
  node FQN 为 `/motion_gate_node`，candidate topic 使用
  `/voice_nav_internal/motion_gate/candidate/lease_` 前缀；
- 严格 `rmw_fastrtps_cpp` runtime lock，由产品 bringup 选择并在 Gate startup 强制执行；`use_sim_time` 在
  startup 后不可变，publication barrier 还要求 active ROS-time override，否则以 fail-closed 进入 zero/stamp-zero；
  具有纯 Core、无 Gazebo/无 `/clock` Node 与无头 Gazebo 产品验收层。

私有 seam 缩小产品 surface，但不是 DDS access control。当前测试使用 authority/candidate harness，不声称 Mission
Runtime、速度平滑器、Collision Monitor、process-kill crash-stop 或 Gazebo pause/resume 完成；crash-stop/pause recovery
是独立目标验收切片。

## 当前 Issue #35 调节模块切片

由 Runtime 拥有的 `MotionConditioningPipeline` 是 `voice_nav_mission` 的包内模块。生产 Adapter 在
`/motion_conditioning_container` component container 中以 composition 固定 Nav2 `1.3.12` 的
`nav2_collision_monitor::CollisionMonitor` 与 `nav2_velocity_smoother::VelocitySmoother`，驱动其 lifecycle，且只在
有界 graph、health、controller、clock、lease 与 zero-proof 检查后将 candidate writer 交给 MotionGate。可信
bringup 记录将 MotionGate PREPARE 限为 `6000 ms`；调节模块将 component RPC 限为 `2000 ms`，并将
PREPARE 到 OPEN 的 handover 限为 `4000 ms`。

当前内部切片固定 component FQN 为 `/collision_monitor` 与 `/velocity_smoother`；raw/smoothed traffic 使用
`/voice_nav_internal/motion/raw` 与 `/voice_nav_internal/motion/smoothed`，Collision Monitor state 使用
`/voice_nav_internal/motion/collision_state`，每个 MotionGate lease 在
`/voice_nav_internal/motion_gate/candidate/lease_` 下获得 Gate 生成的 topic。Collision Monitor candidate writer
固定为 `RELIABLE + VOLATILE + KEEP_LAST(1)`；MotionGate candidate reader 是
`BEST_EFFORT + VOLATILE + KEEP_LAST(1)`。最终 controller publisher 保持 `rclcpp::SystemDefaultsQoS()` 与
writer/GID proof。它们均为私有 runtime seam，不 export 为公共 ROS IDL。

Issue #64 现在提供生产 pure-control 与 ROS-integration RelativeMotion 执行切片。其 Adapter 复用 #35
模块，并保持 handover 顺序 `OPEN -> Collision Monitor -> Velocity Smoother -> producer`。无头物理
raw-stamp-age/TF 验收有意由 Issue #72 独立追踪；该仓库切片不声称已通过 physical gate。

## 目标 v1.0 拓扑

```text
麦克风 / 扬声器
        │
        └── voice_node
              └── 已识别普通话 / 可取消语音
                    └── agent_node
                          └── ExecuteMission.action
                                └── mission_runtime_node ──> /mission/state snapshot
                                      ├── Nav2 速度 ───────────────────┐
                                      └── 相对运动速度 ────────────────┼── nav2_velocity_smoother
                                                                         └── nav2_collision_monitor
                                                                               └── motion_gate_node
                                                                                     └── diff_drive_controller
                                                                                           └── gz_ros2_control / Gazebo

运行停止 ── StopMission.srv ──> mission_runtime_node
```

自编写进程集合严格为：

```text
voice_node
agent_node
mission_runtime_node
motion_gate_node
```

上游 ROS/Gazebo 进程仍独立受管。

## 深层模块与公共接口

### 语音模块

接口产生已完成 Voice Turn 并接受可取消语音。实现隐藏 WSL 音频设备、播放 reference、
WebRTC 音频处理、唤醒词、VAD、本地 ASR、本地 TTS、缓冲与 barge-in。原始 `10 ms` PCM 保留在进程内。

### Agent 模块

接口将已识别文本转为 Mission、澄清、拒绝或回复。实现隐藏确定性规则、
受约束的本地 LLM 回退、轮次关联与过期结果拒绝。它从不发布速度，也不 import Nav2/Gazebo
类型。

### Mission 控制模块

`voice_nav_mission` 拥有两个进程：

- `mission_runtime_node`：公共 `ExecuteMission.action`、transient-local `/mission/state`、公共
  `StopMission.srv`、完整计划 Guard、单槽准入、终态意图线性化、工作流、来源选择、
  Nav2、相对运动与地图保存；
- `motion_gate_node`：包内 control seam、由 Runtime 续约的 `250 ms` 权限租约、每租约 candidate
  binding/freshness、lock、limit、zero 输出与唯一最终速度发布。Gate 生成 lease ID/topic；Runtime 用
  Gate instance、当前 lease 与全局 compare-and-swap sequence 驱动四项 lock operation。

包内的 `motion_gate_core` 静态 target 是 Adapter 后的深层模块，拥有 state、identity validation、deadline、
binding decision、clamp、retire 和 selected command。Node Adapter 拥有 ROS graph query、reader A/B/C lifecycle、实际
最终/state 发布、发布 sequence counter 与 `zero_published` acknowledgement。在该 MotionGate 切片内，
只有 `motion_gate_node` 被 install；Mission package 当前 Runtime 控制面 target 见下一节。

调用方只学习一个执行 operation、一个停止 operation 与一个状态 snapshot。内部复杂性保留在一个
package 内，独立进程仍使最终 watchdog 独立于编排。

## 当前 Mission Runtime 控制面切片

仓库当前安装 `mission_runtime_node` 作为 Mission 控制面进程，提供有界公共 surface
`/mission/execute`、`/mission/stop` 与 transient-local `/mission/state`，并有包内 Core、MotionGate Adapter
和脚本化 seam。admission、identity/epoch fencing、STOP 线性化、state/feedback/result 投影与 fail-closed
Gate 协调在本切片实现。

生产 `RelativeMotionPort` 基于深层、ROS-free 的 `RelativeMotionController` 与 ROS Adapter。MOVE 将 odometry 投影
到带符号的初始航向轴；ROTATE 比较解包 yaw。二者强制 steady-clock deadline、progress/stall policy、
source freshness、lease/generation fencing 及终态 result 前的 zero/stationarity。Adapter 复用 Issue #35 私有
conditioning/authority handover 模块，不增加 conditioned-scan relay、writer handover 或公共 ROS IDL。start/teardown
transaction 是异步且可观察取消的：由 Node 拥有的 Runtime event queue 优先 STOP/Cancel control，Adapter 围栏
generation 并开始 Gate inhibit/zero，而不持有 Node mutex。stationarity 从真实 steady-clock `zero_proven_at` 开始，使用
绝对 `zero_proven_at + 1200 ms` deadline。本 Task 证明 pure-control 与 ROS-integration 层；无头物理
raw-stamp-age/TF 验收仍由 Issue #72 追踪。

shutdown 立即关闭 command、timer、scan/clock、conditioning-health、collision 与 renew ingress，仅保留 Adapter odom
subscription 做 post-zero proof。保留的 callback 在 Gate zero 后仅观察，不能 renew、plan、publish 或推进 Runtime
counter。Gate zero 保持可信的 `250 ms` stop barrier；Node 级 shutdown deadline 是 `250 ms + 1200 ms` 联合 budget；
odom ingress 在 stationarity success/failure 后 drain。

### 仿真模块

`voice_nav_sim` 拥有 Xacro、Gazebo asset、`gz_ros2_control`、controller configuration 与 sensor bridge。目标
`ros_gz_bridge` traffic 只有 `/clock` 与 `/scan`；原始 scan stamp/frame 保持不变，Gazebo model-scoped name 不泄漏。
物理 raw-age/TF gate 属于 Issue #72。

## Package 与进程边界

| 软件包 | 拥有行为 | 自编写进程 |
| --- | --- | --- |
| `voice_nav_interfaces` | 有界稳定 ROS IDL | 无 |
| `voice_nav_audio` | 本地音频/语音管线 | `voice_node` |
| `voice_nav_agent` | 规则、本地 LLM Adapter、对话策略 | `agent_node` |
| `voice_nav_mission` | Runtime、Gate 与内部依赖 Adapter | `mission_runtime_node`、`motion_gate_node` |
| `voice_nav_sim` | 机器人模型、Gazebo、ros2_control、`/clock`/`/scan` bridge | 无 |
| `voice_nav_bringup` | launch、parameter、地图与 Named Place | 无 |

不得创建 Guard、scheduler、Nav2 bridge、map saver 或顶层行为树进程；它们是 Mission Runtime 内部 seam。

## 依赖方向

```text
voice_nav_audio ────────> voice_nav_agent ────────┐
voice_nav_mission ────────────────────────────────┼──> voice_nav_interfaces
voice_nav_agent ────────> 仅 Mission Interface    │
voice_nav_mission ─────> 标准 ROS / Nav2 / 地图 Interface
voice_nav_sim ─────────> Gazebo + gz_ros2_control + ros_gz_bridge
voice_nav_bringup ─────> runtime 组合与配置
```

禁止方向：

```text
agent   ──X──> Nav2 / Gazebo / cmd_vel / controller command
audio   ──X──> Mission Implementation / Nav2 / Gazebo
mission ──X──> Gazebo Transport / ASR / LLM / TTS
sim     ──X──> Agent / voice / Mission policy
```

## 运行流

Mapping：

```text
/scan + diff_drive_controller odometry
        └──> slam_toolbox ──> 逻辑地图保存
```

Navigation：

```text
已保存地图 ──> map server + AMCL
Named Place ──> Nav2 ──> 固定安全运动链
```

Mapping 与 Navigation 分别启动，`slam_toolbox` 和 AMCL 永不同时拥有 `map → odom`。

## 内部 seam 与 Adapter

Mission Runtime 对 Nav2、相对运动、candidate velocity、地图保存、Gate control 与 steady time 具有私有
Interface。每项均有生产 Adapter 与脚本化内存替身；同一模式适用于本地 ASR、LLM 与 TTS engine。
Guard 规则、FSM、Named Place value 与音频 callback wiring 保持普通实现，不是投机的 plug-in Interface。

## 稳定接口 surface

兼容性敏感行为包括：ROS name、type、有界 field、QoS、node name、parameter、frame；unit、limit、time
domain、admission、ordering、cancellation、error；TF/最终速度 owner；configuration、map、Named Place、
model-manifest schema。变化需要 contract test 与 changelog 处理。

详细目标契约：

- [Mission Runtime 接口](mission-runtime-interface.md)
- [安全与运动](safety-and-motion-contract.md)
- [TF 与运行模式](tf-and-operating-modes.md)
- [Voice 与 Agent](voice-and-agent.md)
- [ADR-0001：已被替代的原生 DiffDrive](../adr/0001-use-native-gazebo-diff-drive.md)
- [ADR-0002：迁移到 ros2_control](../adr/0002-migrate-to-gz-ros2-control.md)
- [ADR-0003：一个深层 Mission Runtime](../adr/0003-use-one-deep-mission-runtime.md)
- [ADR-0004：分离 Mapping 与 Navigation](../adr/0004-separate-mapping-and-navigation-modes.md)

发布兼容性遵循[发布策略](../process/release-policy.md)。
