# 当前状态与路线图

VoiceNav 的产品规格会同时描述“当前实现”和“已批准目标”。这两者必须明确区分：目标文档、Issue 或源码骨架存在，
都不能自动变成已交付能力。

## 当前 `main` 的公开能力

<span class="vn-badge vn-badge--verified">已验证切片</span>

- `gz_ros2_control` + Jazzy `diff_drive_controller` 的差速仿真基线；
- 360° 二维 LiDAR，ROS–Gazebo bridge 仅 `/clock` 与 `/scan`；
- 独立、fail-closed MotionGate：250 ms authority、150 ms candidate freshness、唯一 final writer；
- Mission Runtime 控制面：全计划校验、单执行槽、STOP/cancel/timeout 终态收敛；
- odometry closed-loop 的 MOVE / ROTATE RelativeMotion；
- bounded Voice / Mission 公共 ROS 类型；
- 确定性中文 Agent Core、clarification、stale-result fencing 与 loopback Response Provider Adapter；
- 分层仓库契约、Core、ROS integration 与 headless Gazebo 验证入口。

!!! warning "不要过度解读"
    当前能力不代表完整 v1.0：真实本地语音模型、完整 Mapping/Nav、地图包与 release hardening 仍有独立 Issue 和验收。

## Walking-skeleton 里程碑

| 里程碑 | 能力边界 | 站点解释 |
| --- | --- | --- |
| v0.2 | 运动基线、TF、LiDAR、MotionGate、deadman | 当前核心纵向切片 |
| v0.3 | Mapping、原子地图包、AMCL、Named Place、Nav2 | 持续交付 |
| v0.4 | Mission validator/FSM 与 movement/nav/map Adapter | 多项切片已提前形成，正式版本待校准 |
| v0.5 | 本地中文 Agent、Qwen/llama.cpp fallback、clarification | Core/Adapter 已形成，真实 corpus/provisioning 另验收 |
| v0.6 | PortAudio、APM、KWS/VAD/ASR、TTS | 目标 |
| v0.7 | AEC、barge-in、Voice STOP、端到端模式 flow | 目标 |
| v1.0 | 故障恢复、soak、license/model inventory、release evidence | 目标 |

里程碑是产品能力顺序，不等于当前 package metadata 或 release tag。正式发布还要求 clean-checkout reproduction、
文档/接口一致、完整验收、license/model inventory 与不可变 tag。

## 已批准但未完成的完整 flow

1. Mapping Mode 构建地图；
2. 用逻辑 Map ID 原子保存 occupancy map 与 posegraph；
3. 关闭 Mapping，启动 Navigation；
4. AMCL 成为唯一 `map → odom` owner；
5. 中文 Voice Turn 形成 `NAVIGATE_TO` Mission；
6. Named Place 在 Runtime 内解析为可信 pose；
7. Nav2 candidate 经 smoother、Collision Monitor、MotionGate 到 controller；
8. cancel、timeout、STOP、Runtime/Gate failure 均有证据。

## 明确非目标

- 真实机器人 hardware driver、功能安全急停或安全认证；
- camera、IMU/EKF、视觉感知、机械臂、MoveIt 2；
- 在线切换 Mapping / Navigation 或自主探索；
- 云端 ASR/LLM/TTS、账号与长期对话记忆；
- 多机器人、公网 ROS 运行或生产级 Web 控制台。

页面事实依据：[`README.md`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/README.md)、
[`v1.0 产品规格`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/product/v1.0-product-spec.md)、
[`发布策略`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/process/release-policy.md)。
