# VoiceNav Robot Resources

## Knowledge

- [ROS 2 Jazzy：开发 ROS 2 package](https://docs.ros.org/en/jazzy/How-To-Guides/Developing-a-ROS-2-Package.html)
  官方 package 创建、CMake、Python entry point 和安装规则。创建或调整 package 时使用。
- [ROS 2 Jazzy：Topic、Service 与 Action](https://docs.ros.org/en/jazzy/How-To-Guides/Topics-Services-Actions.html)
  官方 Interface 选型说明。设计传感器数据、急停和长任务执行协议时使用。
- [colcon Quick Start](https://colcon.readthedocs.io/en/main/user/quick-start.html)
  官方工作区构建和 overlay 使用说明。处理构建、选择 package 和环境加载问题时使用。
- [Navigation2 文档](https://docs.nav2.org/)
  Nav2 的概念、配置、行为树和调试入口。进入导航阶段后作为主要资料。
- [slam_toolbox 文档](https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/)
  建图、地图保存和 pose graph 序列化资料。进入建图阶段后使用。
- [Gazebo Harmonic 文档](https://gazebosim.org/docs/harmonic/getstarted/)
  Gazebo world、模型、传感器和仿真运行资料。编写最小机器人模型时使用。
- [ROS 2 Jazzy：URDF 与 robot_state_publisher](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)
  官方机器人模型、JointState、TF 发布和 RViz 显示流程。建立静态机器人模型时使用。
- [ROS 2 Jazzy：xacro](https://docs.ros.org/en/jazzy/p/xacro/)
  官方 xacro 包文档。需要用属性和宏减少 URDF 重复时使用。
- [ROS 2：为 URDF 添加物理与碰撞属性](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html)
  官方 visual、collision、inertial 和 joint dynamics 入门。把静态模型变成物理仿真模型时使用。
- [Gazebo Harmonic：从 ROS 2 启动 Gazebo](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/)
  `ros_gz_sim` 的 server、GUI 和 launch 接入方式。建立仿真启动链时使用。
- [Gazebo Harmonic：从 ROS 2 生成模型](https://gazebosim.org/docs/harmonic/ros2_spawn_model/)
  向运行中的 world 创建 URDF/SDF 实体。验证 `/robot_description` 到 Gazebo 的模型链路时使用。
- [Gazebo Sim 8：DiffDrive system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
  原生差速驱动插件的关节、几何、topic、frame、里程计频率与速度限制参数。建立最小仿真底盘驱动时使用。
- [Nav2：在 Gazebo 中设置里程计](https://docs.nav2.org/setup_guides/odom/setup_odom_gz.html)
  Nav2 官方的 Gazebo DiffDrive、JointStatePublisher 与 ROS–Gazebo bridge 组合。接入 ROS 里程计与 TF 时使用。
- [Nav2：设置坐标变换](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html)
  `map → odom → base_link` 与传感器 frame 的职责。接入 SLAM、定位和导航前核对唯一发布者。
- [ros_gz_bridge：YAML 配置](https://docs.ros.org/en/ros2_packages/jazzy/api/ros_gz_bridge/index.html)
  ROS 与 Gazebo topic 的类型、方向、重命名及 QoS 配置。Lesson 0007 建立仿真 adapter 时使用。
- [ROS REP-2004：Package Quality Categories](https://reps.openrobotics.org/rep-2004/)
  ROS package 的版本、变更控制、文档、测试、依赖、平台与安全质量框架。建立和审查工程治理策略时使用。
- [Git：gitignore](https://git-scm.com/docs/gitignore)
  ignore pattern 与 tracked/untracked 文件语义。维护源码仓库边界时使用。
- [Git：git rm](https://git-scm.com/docs/git-rm)
  从工作区和/或 index 移除路径的准确语义。清理已被跟踪的生成物时使用。
- [Conventional Commits 1.0.0](https://www.conventionalcommits.org/en/v1.0.0/)
  为提交信息增加人类和机器可读的变更语义。提交、变更日志和自动版本工具使用。
- [Semantic Versioning 2.0.0](https://semver.org/spec/v2.0.0.html)
  `MAJOR.MINOR.PATCH` 与公开 Interface 兼容规则。制定里程碑和发布策略时使用。

## Wisdom (Communities)

- [ROS Discourse](https://discourse.ros.org/)
  ROS 设计、发行版和生态讨论。遇到架构取舍或版本兼容问题时使用。
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)
  可复现的机器人问题与工程解答。适合搜索 TF、Nav2、SLAM 和控制问题。
