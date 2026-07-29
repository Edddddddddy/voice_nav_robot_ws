# Mission: VoiceNav Robot

## Why
亲手实现一个精简、完整、可解释的 ROS 2 语音导航机器人，用它掌握从机器人建模、SLAM、Nav2 到本地语音交互和安全任务执行的完整工程闭环，而不是只会运行现成示例。

## Success looks like
- 能独立写出并解释最小差速机器人、TF、里程计和 2D LiDAR 模型
- 能在 Gazebo 中完成建图、保存地图、AMCL 定位和 Nav2 导航
- 能用普通话和唤醒词“小智”发起本地语音命令，并正确处理 TTS 回声
- 能通过强类型 Mission 执行多步动作、门禁、取消和急停
- 能为 Core 编写 Fake 驱动测试，并解释各 ROS 包的依赖方向

## Constraints
- 只做仿真，运行在 WSL2 Ubuntu 24.04
- ROS 2 Jazzy + Gazebo Harmonic
- 单麦克风和外放，全部 ASR、LLM、TTS 在本地运行
- 采用两阶段流程：先建图，再加载地图导航
- 由学习者亲手编写功能代码；教学按小步任务、运行验证、代码复盘推进

## Out of scope
- 真实机器人硬件和硬件驱动
- 机械臂、MoveIt 2、抓取与视觉
- 相机、IMU/EKF、动态探索和复杂多机器人系统
- 云端 ASR、LLM、TTS
- Web 控制台、账号系统和长期对话记忆
