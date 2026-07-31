# VoiceNav Robot

VoiceNav Robot 是一套从零手写的 ROS 2 纯仿真学习项目：让普通话语音经过
本地 KWS、AEC、ASR、规则与 LLM，转换为受约束的强类型 Mission，再由安全
运动链驱动 Gazebo 差速机器人完成建图与导航，最后使用本地 TTS 回复。

运行基线固定为 WSL2 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic。不包含
真机、机械臂、相机、云服务或经过功能安全认证的急停系统。

## 当前状态

Lesson 0001–0007 已完成；Lesson 0008 的 reference implementation 已完成
本地门禁与独立本地评审；draft
[PR #9](https://github.com/Edddddddddy/voice_nav_robot_ws/pull/9)
正在运行远端 CI 与评审。当前 `v0.2` 切片包括：

- 六个职责明确的 ROS package；
- 第一版 `MissionStep` 和 `ExecuteMission` 教学接口；
- 手写差速机器人 Xacro、TF、碰撞、惯量和稳定落地；
- `gz_ros2_control`、`diff_drive_controller` 与消费端 timeout；
- 自包含测试 world、360° 2D LiDAR、`/clock`/`/scan` 单向 bridge；
- controller 直接发布产品 `/odom`，以及按 edge、publisher GID 和完整
  node FQN 验证的 TF 唯一所有权；
- Work Item、质量门禁、变更记录和课程记录。

原生 Gazebo DiffDrive 仅保留为历史教学 checkpoint。下一切片将加入独立
MotionGate 和 crash-stop，不把当前 controller timeout 误称为完整安全链。

实现状态与目标设计必须区分阅读：

- [课程目录](course/README.md) 说明已经完成的学习 checkpoint；
- [v1.0 产品规范](docs/product/v1.0-product-spec.md) 定义最终验收；
- [架构总览](docs/architecture/overview.md) 区分 current 与 target；
- [发布策略](docs/process/release-policy.md) 给出逐版本交付边界。

## 目标链路

```text
单麦克风 + 外放
  → 本地 AEC / 唤醒词“小智” / VAD / ASR
  → STOP 快路径或规则优先的 Agent + 本地 LLM fallback
  → 强类型 Mission、整计划验证、单执行槽和 admission fencing
  → Nav2 / 相对运动候选速度
  → velocity smoother → collision monitor → MotionGate
  → diff_drive_controller → Gazebo
  → slam_toolbox 建图，或 AMCL + Nav2 导航
  → 本地 TTS
```

Mapping 与 Navigation 是两个独立启动模式，分别由 `slam_toolbox` 和 AMCL
拥有 `map → odom`。它们不能同时运行。MotionGate 是唯一最终速度发布者；
LLM、Agent、Nav2 和 Gazebo bridge 都不能绕过它。

## 仓库结构

| 路径 | 职责 |
| --- | --- |
| `src/voice_nav_interfaces` | 稳定的 ROS msg、srv、action |
| `src/voice_nav_sim` | Xacro、Gazebo 与仿真适配 |
| `src/voice_nav_mission` | Mission Core、运行时与 MotionGate |
| `src/voice_nav_agent` | 中文规则、本地 LLM 适配与对话策略 |
| `src/voice_nav_audio` | Audio I/O、AEC、KWS、VAD、ASR、TTS |
| `src/voice_nav_bringup` | 模式化 launch、配置与组合根 |
| `docs/product` | 产品范围、成功标准与术语 |
| `docs/architecture` | 系统、Interface、运动安全、TF 与模式 |
| `docs/process` | 质量、测试、变更和发布流程 |
| `docs/adr` | 有后果的架构决策及 superseded 关系 |
| `docs/work-items` | 版本化变更契约和验收证据 |
| `course/lessons` | 可复现课程 |
| `course/records` | 学习者提交与复盘 |
| `course/reference` | ROS、Gazebo、SLAM、Nav2 与语音参考 |

六个 ROS package 是部署和依赖边界，不为目录整齐继续拆分浅包。运行时的
深 Module 可以在同一 package 内拥有多个进程或私有 target。

## 构建与验证

在 WSL 中运行完整质量门禁：

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
bash scripts/verify.sh
```

只验证某个 package 及其依赖：

```bash
bash scripts/verify.sh voice_nav_sim
```

门禁会检查仓库与课程契约、Markdown 本地链接、ROS package 版本一致性、
声明依赖、Xacro/URDF/SDF 契约、构建以及全部测试。`build/`、`install/`、
`log/`、模型权重、地图、录音和运行证据不得提交。

## 开发与学习

工程变更遵循：

```text
Work Item → GitHub Issue → branch
→ tests-first implementation → docs / ADR / changelog
→ local verify → PR → CI → rebase merge → tag / release
```

`main` 始终保存参考 solution。Lesson 0007 起，每课使用
`course/NNNN-start` 与 `course/NNNN-solution` annotated tag；学习者从
start tag 创建独立 `learn/NNNN` 分支或 worktree，完成后与 solution tag
比较，不复制第二份源码。

开始修改前阅读 [贡献流程](CONTRIBUTING.md)、[变更生命周期](docs/process/change-lifecycle.md)
和相应 [Work Item](docs/work-items/)。

## 安全、许可证与报告

项目中的“停止”是仿真项目的高优先级 operational stop，不代表机器人已经
物理停稳，也不宣称满足功能安全标准。安全边界与漏洞报告方式见
[SECURITY.md](SECURITY.md)。

项目使用 Apache License 2.0。第三方来源和限制记录在
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
