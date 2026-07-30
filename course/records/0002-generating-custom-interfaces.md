# Lesson 0002 学习记录：生成并验证自定义 Interface

状态：Completed

学习者手写了 `MissionStep.msg` 和 `ExecuteMission.action`，接入 rosidl 并用 ROS CLI 检查生成结果。首次构建暴露 `package.xml` 缺少 `rosidl_interface_packages` group membership，修正后依赖、构建与测试全部通过。

## 验收证据

- `ros2 interface show` 展示了四种 MissionStep 常量和 Action 的 Goal、Result、Feedback 三段。
- `rosdep check`：`All system dependencies have been satisfied`。
- Interface 测试全部通过。
- 能解释速度、容差与超时必须来自可信配置，LLM 只能表达受限语义动作。

后续课程把 Agent/LLM 视为不可信规划来源，所有 Mission 都必须通过本地语义校验。
