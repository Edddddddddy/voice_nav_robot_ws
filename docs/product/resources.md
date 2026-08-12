# 工程资源

实现决策优先采用第一方文档；社区来源可辅助诊断，但不能覆盖目标契约。

## 平台与 ROS 2

- [开发 ROS 2 package](https://docs.ros.org/en/jazzy/How-To-Guides/Developing-a-ROS-2-Package.html)
- [ROS 2 Topic、Service 与 Action](https://docs.ros.org/en/jazzy/How-To-Guides/Topics-Services-Actions.html)
- [colcon 快速入门](https://colcon.readthedocs.io/en/main/user/quick-start.html)
- [使用 robot_state_publisher 的 URDF](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)
- [xacro](https://docs.ros.org/en/jazzy/p/xacro/)
- [URDF 物理属性](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-the-URDF-Model.html)

## Gazebo、控制、TF、SLAM 与导航

- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/)
- [从 ROS 2 启动 Gazebo](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/)
- [生成 Gazebo 模型](https://gazebosim.org/docs/harmonic/ros2_spawn_model/)
- [gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [ros_gz_bridge](https://docs.ros.org/en/ros2_packages/jazzy/api/ros_gz_bridge/index.html)
- [Nav2](https://docs.nav2.org/)
- [Nav2 velocity smoother](https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html)
- [Nav2 collision monitor](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html)
- [Nav2 transform setup](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html)
- [slam_toolbox](https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/)

历史 native-DiffDrive 实现使用
[Gazebo Sim 8 DiffDrive system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)。
它是历史证据，不是 v1.0 控制目标。

## 本地音频与模型

- [PortAudio callback 约束](https://portaudio.com/docs/v19-doxydocs/writing_a_callback.html)
- [webrtc-audio-processing](https://gitlab.freedesktop.org/pulseaudio/webrtc-audio-processing)
- [sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx)
- [sherpa-onnx KWS 模型目录](https://k2-fsa.github.io/sherpa/onnx/kws/pretrained_models/index.html)
- [Piper](https://github.com/rhasspy/piper)
- [llama.cpp](https://github.com/ggml-org/llama.cpp)
- [llama.cpp server schema 输出](https://github.com/ggml-org/llama.cpp/blob/master/tools/server/README.md)
- [官方 Qwen3-0.6B-GGUF](https://huggingface.co/Qwen/Qwen3-0.6B-GGUF)

运行时版本和模型只可通过具有精确 SHA-256 与许可证元数据的 pinned manifest 接受；仓库链接本身不是 pin。

## 工程治理

- [REP-2004](https://reps.openrobotics.org/rep-2004/)
- [gitignore](https://git-scm.com/docs/gitignore)
- [git rm](https://git-scm.com/docs/git-rm)
- [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)
- [Semantic Versioning](https://semver.org/spec/v2.0.0.html)

## 社区诊断

- [ROS Discourse](https://discourse.ros.org/)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)
