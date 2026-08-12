# VoiceNav Robot 术语表

这是 v1.0 的 canonical 产品语言。

## 术语

**Voice Turn**：
从 Wake Word 到响应的一次交互，包含一条最终 utterance 及其决策或回复。
_避免使用_：Conversation、request。

**Mission**：
一个经过验证、有顺序、包含一至三个语义机器人动作且只有一个终态结果的序列。
_避免使用_：Script、job、raw command list。

**Mission Step**：
一个语义动作：移动距离、旋转角度、导航到 Named Place 或保存地图。
_避免使用_：Twist、controller command。

**Named Place**：
仅在可信 navigation Implementation 内解析为 map pose 的、面向人的稳定标识符。
_避免使用_：Target string、raw pose。

**Mapping Mode**：
独立启动的模式；`slam_toolbox` 拥有 `map → odom`、构建地图，并可保存地图；不提供 global localization
navigation。
_避免使用_：SLAM session、online switch state。

**Navigation Mode**：
独立启动的模式；加载保存的地图，AMCL 拥有 `map → odom`，Nav2 接受 Named Place Mission；不提供在线
地图构建。
_避免使用_：AMCL mode、online switch state。

**Motion Gate**：
由 `motion_gate_node` 实现的独立最终速度权限；它执行 Runtime-bound `250 ms` authority lease、当前
candidate data-plane binding、candidate freshness、限制、inhibition 与 zero output。raw velocity sample
永远不能续约 authority。
_避免使用_：Mission scheduler、LLM controller、Gazebo controller。

**Operational Stop（运行停止）**：
高优先级、幂等的请求；它在常规 Mission cleanup 完成前让 Motion Gate lock 并发布零速度。其 ROS type
严格为 `StopMission.srv`。它是仿真运行停止，不是功能安全紧急停止或认证安全功能。
_避免使用_：Emergency stop、e-stop、Safety Stop、pause、`OperationalStop.srv`。

**Wake Word**：
开启普通 Voice Turn 的本地短语“**小智**”。
_避免使用_：Activation command。

**Source Instance**：
一个 Agent 或 stop producer 的生命周期，用于标识 source sequence，避免其跨 restart replay。
_避免使用_：Session、user。

**Runtime Instance**：
一个 `mission_runtime_node` 的生命周期，从 `/mission/state` 复制到 Goal，以 fence 旧进程产生的工作。
_避免使用_：Node name、Mission ID。

**Admission Epoch**：
从 `/mission/state` 复制的可变 Runtime ownership generation；为较早 generation 创建的 Goal 已过期。
_避免使用_：Timestamp、Action goal UUID。
