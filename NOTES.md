# Teaching Notes

- 用户希望通过重新手写精简版本锻炼动手能力。
- 默认由用户先实现；教师提供任务、接口约束、验收命令和代码复盘。
- 每次只推进一个可运行的垂直切片，避免一次生成完整项目。
- 发现问题时优先解释证据和原因，再让用户修改。
- 项目名称：VoiceNav Robot。
- 工作区目录：voice_nav_robot_ws。
- ROS package 前缀：voice_nav_。
- Lesson 0003 已完成：静态 Xacro、TF、RViz 与 `voice_nav_sim` 的 12 项测试全部通过。
- Lesson 0004 已完成：物理属性、URDF-to-SDF 校验、Gazebo 生成与稳定落地全部通过。
- Lesson 0005 已完成：Gazebo DiffDrive、轮关节物理运动、Gazebo odometry 与显式停车通过验收。
- 用户要求后续遵循完整但不过度的企业开发流程：Work Item、短分支、原子提交、文档、统一质量门禁、评审证据、CI 和版本发布。
- Lesson 0006 当前建立工程基线；ROS bridge 顺延到 Lesson 0007。
- 仿真驱动采用 Gazebo 原生 DiffDrive，再由 `ros_gz_bridge` 适配 ROS；只做仿真的当前阶段不引入 `gz_ros2_control`。
- Gazebo Sim 8 DiffDrive 没有 command timeout；所有测试必须显式发零速度，进入语音/Nav2 前必须在 ROS 运动出口加入可配置 watchdog。
- 当前 Jazzy 的 `robot_state_publisher` 未发布 `/robot_description` topic；统一仿真 launch 完成前，继续由 Xacro 生成临时 URDF，再用 `ros_gz_sim create --file` 生成模型。
- 当前工程基线分支：`chore/0006-engineering-baseline`。远程托管和可见性尚未选择，不得自行创建或 push。
- 各 package 仍有临时 maintainer/description 元数据；首次远程发布前需要用户提供可公开的维护者名称与邮箱并完成清理。
