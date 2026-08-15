# 六个 Package

工作区按产品责任分成六个 ROS package。公共类型、仿真、运行控制、Agent、音频与启动编排各自拥有一个主要变化理由。

## 依赖方向

```text
voice_nav_interfaces
        ▲          ▲
        │          │
voice_nav_audio  voice_nav_agent
        │          │
        └────┬─────┘
             ▼
     voice_nav_mission
             ▲
             │
voice_nav_sim ── voice_nav_bringup
```

这是概念依赖图，不替代 `package.xml` 的精确 build/exec/test dependency。

## voice_nav_interfaces

<span class="vn-badge vn-badge--verified">公共接口</span>

拥有 `MissionStep`、`MissionState`、`VoiceTurn`、`ExecuteMission`、`Speak` 与 `StopMission` 的 ROS IDL。
包中不放 Runtime、Agent 或测试用 endpoint。

## voice_nav_sim

<span class="vn-badge vn-badge--verified">仿真基线</span>

拥有手写差速 Xacro、Gazebo world、LiDAR、`gz_ros2_control`、controller 与 `/clock`、`/scan` bridge 配置。
controller 直接发布 `/odom` 与 `odom → base_footprint`。

## voice_nav_mission

<span class="vn-badge vn-badge--verified">控制面与运动权限</span>

拥有 `mission_runtime_node`、`motion_gate_node`、ROS-free Core、RelativeMotion、conditioning lifecycle 与包内 Gate seam。
公共 Mission surface 依赖 `voice_nav_interfaces`；InternalMotionGate 类型不属于公共产品 API。

## voice_nav_agent

<span class="vn-badge vn-badge--verified">中文决策与编排</span>

Pure Python `AgentCore` 负责确定性中文解析、澄清、STOP-first、planning token 与语义校验；`agent_node` 负责 ROS Adapter、
单 Mission slot、Speak/Stop clients 与 loopback Response Provider。

## voice_nav_audio

<span class="vn-badge vn-badge--target">真实音频持续交付</span>

目标拥有 full-duplex audio、AEC、KWS、VAD、ASR、TTS 与 barge-in。当前源码已经形成 audio engine、speech input Core、
closed recognizer event seam 和测试 fixture，但真实受许可模型/设备里程碑不能由 package 存在推断为完成。

## voice_nav_bringup

<span class="vn-badge vn-badge--verified">启动与可信配置</span>

组合 simulation、conditioning container、MotionGate 与 Runtime；集中保存 controller、Gate、Runtime 等受信任 YAML。
未来 Mapping/Navigation 也由各自独立 launch 组合，而不是在线切换 mode owner。

## 为什么这样拆分？

- Interface 不依赖实现，contract consumer 可以独立构建；
- Agent 不 import Nav2/Gazebo 类型，LLM 不能绕过 Mission trust boundary；
- MotionGate 与 Runtime 同 package 但不同进程，既缩小 public seam，又保留独立 failure domain；
- audio callback 的实时边界留在进程内，不把 10 ms PCM 放上 DDS；
- bringup 统一拥有 launch composition 与受信任策略，而不是让 caller 传任意路径或阈值。

页面事实依据：六个 [`package.xml`](https://github.com/Edddddddddy/voice_nav_robot_ws/tree/main/src) 与
[`README.md` 仓库结构](https://github.com/Edddddddddy/voice_nav_robot_ws#仓库结构)。
