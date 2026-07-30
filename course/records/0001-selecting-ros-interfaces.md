# Lesson 0001 学习记录：按交互语义选择 ROS Interface

状态：Completed

学习者正确区分了 Topic、Service 和 Action：Topic 用于持续的单向数据流，Service 用于短请求/响应，Action 用于具有 feedback、result、cancel 的长任务。

## 验收证据

- `colcon build`：6 个 package 全部完成，摘要为 `6 packages finished [23.5s]`。
- `colcon list`：列出 `voice_nav_agent`、`voice_nav_audio`、`voice_nav_bringup`、`voice_nav_interfaces`、`voice_nav_mission`、`voice_nav_sim`。
- 能解释 Mission 为什么选择 Action，而不是 Topic 或 Service。

后续课程可以假设学习者已掌握工作区、package 与基本 Interface 选型。
