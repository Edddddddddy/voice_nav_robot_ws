# VoiceNav ROS 2 审查清单

- 验收：每条 Issue 标准有直接证据，没有隐藏范围。
- 运动：所有权 fail-closed，命令有界，终态为零；timeout、cancel、stale、crash 与恢复符合契约。
- 时间：lease/deadline 使用 steady time；仿真、TF、sensor 使用 ROS time；暂停/恢复不能复活旧命令。
- ROS graph：类型、名称、QoS、生命周期、参数、单位、错误与取消保持兼容或已明确批准。
- TF/模式：每个 transform 只有一个 owner；SLAM 与 localization 不冲突。
- 并发：callback 所有权、generation/epoch、队列与 shutdown 顺序拒绝竞态和晚副作用。
- Gazebo/进程：精确子进程所有权、有界 teardown、不误杀无关进程、不泄漏 launch 进程。
- Voice/LLM：不可信文本不能绕过 typed Mission 或 MotionGate；STOP 优先级和幂等性正确。
- 依赖：manifest、lock、许可证、checksum 与离线行为随依赖或模型变更更新。
- 测试：优先公共行为而非实现形状；本地 exact HEAD 产品证据与远端治理证据分开记录。
- 交付：独立回滚，文档描述当前行为，Git 不包含构建产物、凭据、私有音频/地图、日志或模型权重。
