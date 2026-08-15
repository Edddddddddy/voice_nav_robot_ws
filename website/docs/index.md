---
hide:
  - toc
---

<section class="vn-hero">
  <div class="vn-hero__eyebrow">VOICE · MISSION · MOTION</div>
  <h1>让一句中文指令，变成<br><span>有边界的机器人任务</span></h1>
  <p class="vn-hero__lead">
    VoiceNav Robot 是一个面向学习与验证的 ROS 2 仿真项目：它把本地中文语音意图收敛为强类型 Mission，
    再通过独立、失效关闭的运动权限链在 Gazebo 中执行。
  </p>
  <div class="vn-hero__actions">
    <a class="md-button md-button--primary" href="#project-now">了解当前能力</a>
    <a class="md-button" href="https://github.com/Edddddddddy/voice_nav_robot_ws">查看 GitHub</a>
  </div>
  <div class="vn-hero__meta" aria-label="项目技术基线">
    <span>ROS 2 Jazzy</span><span>Gazebo Harmonic</span><span>WSL2 · Ubuntu 24.04</span>
  </div>
</section>

<section class="vn-signal" aria-label="项目状态">
  <div>
    <strong>pre-1.0</strong>
    <span>当前版本边界</span>
  </div>
  <div>
    <strong>6</strong>
    <span>职责明确的 ROS package</span>
  </div>
  <div>
    <strong>≤ 3</strong>
    <span>每个 Mission 的语义步骤</span>
  </div>
  <div>
    <strong>250 ms</strong>
    <span>MotionGate 权限租约</span>
  </div>
</section>

## Project now { #project-now }

当前 `main` 已交付一条可验证的仿真控制纵向切片。它刻意把“能提出运动”和“能向控制器发布最终速度”分开：
语音、规则、LLM、Runtime 和 Nav2 都不能绕过 MotionGate。

<div class="grid cards vn-capability-grid" markdown>

-   :material-message-processing-outline:{ .lg .middle } **语义先于速度**

    ---

    `VoiceTurn` 进入确定性 Agent Core，经同一语义校验器形成一至三个有界 `MissionStep`。

-   :material-shield-check-outline:{ .lg .middle } **唯一最终速度出口**

    ---

    独立 `motion_gate_node` 校验 Runtime 权限、候选新鲜度和 writer 身份，失效时持续选择零速度。

-   :material-robot-industrial-outline:{ .lg .middle } **可复现仿真基线**

    ---

    手写差速模型、二维 LiDAR、`gz_ros2_control` 与 Jazzy `diff_drive_controller` 组成支持路径。

-   :material-test-tube:{ .lg .middle } **分层验证**

    ---

    Pure Core、接口契约、ROS integration 与无头 Gazebo 测试分别证明不同层级的行为。

</div>

!!! success "当前已验证的核心切片"
    仓库已经包含有界 Mission/Voice 公共接口、Mission Runtime 控制面、MotionGate、基于 odometry 的相对移动，
    以及确定性中文 Agent 与受限 loopback Response Provider 的实现与测试。

!!! info "已批准、仍在持续交付的目标"
    完整本地 KWS/VAD/ASR/TTS、Mapping → Map Package → AMCL/Nav2、Named Place 与真实音频里程碑属于 v1.0 目标，
    不能因为设计文档存在就视为已完成。

## 一条清晰的控制路径

```text
中文 VoiceTurn
  → STOP 优先的 Agent + 确定性规则 + 受限本地 LLM 回退
  → 强类型 ExecuteMission
  → Mission Runtime 准入、代际围栏与单执行槽
  → 相对运动 / Nav2 候选速度
  → Velocity Smoother → Collision Monitor → MotionGate
  → diff_drive_controller → gz_ros2_control → Gazebo
```

MotionGate 是唯一最终速度发布者；Runtime 与 Gate 位于不同进程，因此 Runtime 停滞时，Gate 仍可按自己的
steady clock 让权限租约过期。控制器侧另有 `0.35 s` 消费者超时，构成第二层 deadman。

## 项目边界同样重要

VoiceNav 当前只支持 Windows 11 + WSL2 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic 的仿真环境。
真实机器人、机械臂、摄像头、云服务和功能安全认证均不在支持范围内。这里的 **Operational Stop** 是仿真运行控制，
不是硬件急停，也不应被描述为认证安全功能。

<div class="vn-next" markdown>

### 从源码理解项目，而不是只看演示

接下来的文档会把六个 package、公共 ROS 接口、Mission 生命周期、运动安全链、本地语音与 Agent 边界、
构建测试以及里程碑状态逐页拆开，并明确标注哪些已经验证、哪些仍是目标。

[浏览源码 :octicons-arrow-right-24:](https://github.com/Edddddddddy/voice_nav_robot_ws){ .md-button }

</div>
