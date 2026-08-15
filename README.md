# VoiceNav Robot 语音导航机器人

VoiceNav Robot 是一个仅在仿真中运行的 ROS 2 系统：它将本地中文语音输入转为有边界、强类型的
Mission，并通过受保护的 Gazebo 差速驱动链执行。已批准的目标包含本地 KWS、AEC、ASR、确定性
规则、本地 LLM、TTS、建图和导航。

支持基线为 Windows 11、WSL2 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic。真实机器人、机械臂、
摄像头、云服务和功能安全紧急停止均不在支持范围内。

## 当前状态

仓库当前验证了 v0.2 仿真、MotionGate 与 Mission Runtime 控制面切片：

- 六个 ROS package，职责和依赖边界明确；
- 手写差速 Xacro，包含碰撞体、惯性与稳定生成；
- `gz_ros2_control`、Jazzy `diff_drive_controller` 和 `0.35 s` 消费者超时；
- 自包含 Gazebo 世界，其中 `laser_link` 安装一个 360 度二维 LiDAR；
- `/clock` 与 `/scan` 是唯一的 ROS/Gazebo bridge 流量；
- controller 直接产出产品 `/odom`，并对每条 TF 边检查 owner；
- 独立、fail-closed 的 MotionGate，具有 `250 ms` Runtime authority lease、`150 ms`
  candidate freshness deadline、writer binding 和唯一最终速度权限；
- 包内的 Mission Runtime Core、ROS Adapter 与生产 RelativeMotion Adapter，提供有界准入、STOP 围栏、
  强类型状态/反馈/结果与脚本化行为替身；MotionConditioningPipeline 在受控容器中协调
  `nav2_velocity_smoother`、`nav2_collision_monitor` 与 MotionGate。

已配置的 controller timeout 是消费者侧 deadman，本身并不能单独证明实体静止。当前 main 已验证
MotionGate、Mission Runtime、RelativeMotion、运动调节组件以及 crash-stop 的受限仿真验收；真实机器人、
生产硬件安全功能和未列入契约的恢复场景仍不在本仓库声明范围内。当前/目标边界见
[架构概览](docs/architecture/overview.md)，批准的验收流程见
[v1.0 产品规格](docs/product/v1.0-product-spec.md)。

## 目标控制路径

```text
麦克风与扬声器
  -> 本地 AEC / 唤醒词 / VAD / ASR
  -> STOP 优先的 Agent + 确定性规则 + 本地 LLM 回退
  -> 通过准入围栏验证的 Mission
  -> Nav2 或相对运动候选速度
  -> 速度平滑器 -> 碰撞监控器 -> MotionGate
  -> diff_drive_controller -> gz_ros2_control -> Gazebo
  -> Mapping Mode 中的 slam_toolbox，或 Navigation Mode 中的 AMCL + Nav2
  -> 本地 TTS
```

Mapping 与 Navigation 分别启动：Mapping Mode 中由 `slam_toolbox` 拥有 `map → odom`，
Navigation Mode 中由 AMCL 拥有它，两者绝不同时成为 owner。MotionGate 是唯一最终速度发布者；
Agent、Nav2 和 Gazebo bridge 均不能绕过它。

## 仓库结构

| 路径 | 职责 |
| --- | --- |
| `src/voice_nav_interfaces` | 稳定 ROS message、service 和 action |
| `src/voice_nav_sim` | Xacro、Gazebo 与仿真 Adapter |
| `src/voice_nav_mission` | Mission Runtime、MotionGate 与 Adapter |
| `src/voice_nav_agent` | 中文规则、本地 LLM Adapter 与对话策略 |
| `src/voice_nav_audio` | 音频 I/O、AEC、KWS、VAD、ASR 与 TTS |
| `src/voice_nav_bringup` | 模式 launch、配置与 composition |
| `docs/product` | 产品范围、验收与术语 |
| `docs/architecture` | 系统、接口、安全、TF 与模式契约 |
| `docs/process` | 质量、测试、变更、发布与复发控制 |
| `docs/adr` | 重要架构决策与替代关系 |
| `docs/agents` | GitHub 工作索引 |

## 构建与验证

### Scripted VoiceNav 仿真 Demo

完成普通安装并 source 工作区后，可运行固定的、仅仿真两轮演示：

```bash
ros2 launch voice_nav_bringup scripted_voice_demo.launch.py headless:=true

# Scripted STOP：实际 VoicePipeline → Agent → Runtime → MotionGate/controller
ros2 launch voice_nav_bringup scripted_voice_demo.launch.py headless:=true scenario:=stop
```

两个命令都只用于 Gazebo 仿真。`move` 为默认两轮澄清后前进场景；`stop` 会先观测真实
controller 非零输出，再由 scripted STOP 走既有 VoiceTurn、Agent 与 `StopMission` 链路停止运动。
它们不使用真实模型、声卡或云服务，也不是功能安全急停。

该入口只使用 deterministic scripted recognizer、loopback provider、fake TTS 与 manual
full-duplex device，明确不加载模型、不访问云、不打开物理声卡，也不能用于物理机器人。它会在
确认首轮澄清播放完成后提交唯一的 `MOVE_DISTANCE +0.5 m`，输出机器可读 summary，并有界退出。

开发中运行聚焦的仓库检查：

```bash
python3 tests/test_repository_contract.py
python3 scripts/check_repository.py --root .
```

完成最终变更后，在本地 WSL 2 Jazzy worktree 对 exact PR HEAD 运行适用的产品检查。产品变更通常
以完整本地质量门禁结束：

```bash
bash scripts/verify.sh
```

入口会自动解析普通 `.git` 目录、相对 worktree pointer 和 Windows 绝对 `gitdir:` pointer。不要预设
`GIT_DIR` 或 `GIT_WORK_TREE`，也不要让命令指向另一 checkout。

运行独立诊断前先记录门禁的真实退出状态；后续成功命令不得覆盖失败结果。完整节奏和证据归属见
[变更生命周期](docs/process/change-lifecycle.md)。

GitHub required check 有意保持为快速治理门禁：它检查 workflow、仓库契约和提交摘要，不安装 ROS、
不构建 workspace、也不运行 Gazebo。其成功不构成产品证据。合并前应将本地 exact-HEAD 的
命令、结果和 artifact 路径写入 Task 唯一的 canonical evidence comment；其归属与交接见
[AGENTS.md](AGENTS.md)。

不得提交生成的 `build/`、`install/`、`log/` 树、模型权重、地图、录音或运行时证据。支持的
MotionGate 实现固定为 `rmw_fastrtps_cpp`；canonical launch 选择它，节点拒绝其他 RMW。这是实现支持
边界，并非对所有 DDS 实现的可移植性声明。

## 开发流程

GitHub Issue 是需求、决策、验收、依赖和状态的规范记录。交付路径为：

```text
GitHub Issue -> 独立分支 -> 测试优先实现
  -> 按需更新文档 / ADR / changelog -> 本地验证
  -> 可审查里程碑 push -> Draft PR -> 审查 -> 治理 CI
  -> rebase 合并
```

本地提交可以保持小而可回滚。仅在本地验证完成的可审查里程碑 push，避免每个中间提交都触发冗余 CI。
编辑前阅读[贡献指南](CONTRIBUTING.md)、[变更生命周期](docs/process/change-lifecycle.md)及相关产品和
架构契约；角色与权限只以 [AGENTS.md](AGENTS.md) 为准。

## 安全、许可与报告

“Stop” 指仿真中的高优先级运行停止：它请求零速度并抑制 MotionGate，不证明实体静止，也不是认证
紧急停止。报告和部署边界见 [SECURITY.md](SECURITY.md)。

项目采用 Apache License 2.0。第三方来源与限制记录在
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
