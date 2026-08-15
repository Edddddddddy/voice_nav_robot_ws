# 系统总览

VoiceNav 的设计重点不是把更多模型接入 ROS，而是把不可信意图收敛成少量、可验证、可停止的操作。
系统把语音、Agent、Mission 编排和最终运动权限分开，使每一层都只有一个清晰责任。

<span class="vn-badge vn-badge--verified">控制纵向切片已验证</span>
<span class="vn-badge vn-badge--target">Mapping / Navigation 持续交付</span>

## 四个自编写运行进程

产品目标严格限制为四个自编写 runtime process：

| 进程 | 对外职责 | 隐藏的复杂性 |
| --- | --- | --- |
| `voice_node` | 发布完成的 `VoiceTurn`，执行可取消 `Speak` | 设备 I/O、AEC、KWS、VAD、ASR、TTS、barge-in |
| `agent_node` | 把 Voice Turn 变成 Mission、澄清、停止或回复 | 中文规则、本地 Response Provider、过期结果隔离 |
| `mission_runtime_node` | Mission 准入、执行、状态与 Operational Stop | 全计划校验、单执行槽、代际围栏、终态线性化 |
| `motion_gate_node` | 唯一最终速度发布与 fail-closed 权限控制 | 租约、writer 绑定、新鲜度、clamp、持续零输出 |

当前 `main` 已安装并验证 Mission Runtime 与 MotionGate；Agent Core/ROS Adapter 以及 Voice 输入边界已有实现和测试。
真实本地声学模型、完整 Mapping/Nav 闭环仍必须看路线图状态，不能由拓扑图推断为完成。

## 从意图到车轮

```text
麦克风 / scripted Voice input
       │
       ▼
  voice_node ── VoiceTurn ──▶ agent_node
                                   │
                                   ▼
                         ExecuteMission.action
                                   │
                                   ▼
                        mission_runtime_node
                           │               │
                     RelativeMotion      Nav2
                           └───────┬───────┘
                                   ▼
                      nav2_velocity_smoother
                                   ▼
                      nav2_collision_monitor
                                   ▼
                         motion_gate_node
                                   ▼
                      diff_drive_controller
                                   ▼
                        gz_ros2_control / Gazebo
```

链上只有 `motion_gate_node` 可以向 `/diff_drive_controller/cmd_vel` 发布最终命令。LLM 即使参与规划，也只能提出
受限的 Mission proposal；它不能给出 wheel speed、Twist、raw pose、文件路径、controller 参数或 timeout。

## 深层模块，而不是散落节点

Mission Runtime 对调用者只暴露一个执行 Action、一个 Stop Service 和一个状态快照。Guard、单槽准入、
source/runtime fencing、相对运动、Nav2 Adapter 与 Map Store 都留在 package 内部。

这让调用者只需理解：

- 当前 Runtime 身份与准入 epoch；
- 这一 Mission 是否被接受、执行到哪一步、以什么结构化结果结束；
- Stop 是否已经让 Gate inhibited 并发布零速度。

相对地，Gate 的 PREPARE / OPEN / RENEW / INHIBIT 是 `voice_nav_mission` 包内 seam，不是 Agent 或第三方应用的公共 API。

## 两种运行模式

Mapping 与 Navigation 是两个独立 process composition，而不是在线切换的一个 launch：

=== "Mapping Mode"

    `slam_toolbox` 拥有 `map → odom`，支持有界 MOVE / ROTATE，并以逻辑 Map ID 保存地图包。

=== "Navigation Mode"

    map server + AMCL + Nav2 运行，AMCL 是 `map → odom` 唯一 owner；Named Place 在可信 Runtime Adapter 内解析。

当前公开 `main` 尚未把完整 Mapping → Map Package → Navigation 闭环列为已交付能力。模式设计是已批准目标；
实际可用性以[当前状态与路线图](../roadmap.md)为准。

## 时钟与数据规则

- `/clock`、TF、sensor、SLAM、AMCL 与 Nav2 使用 simulation time；
- 权限租约、cancel bound、process supervision 与 liveness 使用 steady monotonic time；
- `/clock` 与 `/scan` 是唯一 ROS–Gazebo bridge 流量；
- `diff_drive_controller` 同时拥有 `/odom` 与 `odom → base_footprint`；
- `robot_state_publisher` 是机器人内部 frame 的唯一 owner。

页面事实依据：[`docs/architecture/overview.md`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/architecture/overview.md)、
[ADR-0003](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/adr/0003-use-one-deep-mission-runtime.md)、
[ADR-0004](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/adr/0004-separate-mapping-and-navigation-modes.md)。
