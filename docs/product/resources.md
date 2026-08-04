# Engineering resources

Primary documentation is preferred for implementation decisions. Community
sources help diagnosis but do not override the target contracts.

## Platform and ROS 2

- [Developing a ROS 2 package](https://docs.ros.org/en/jazzy/How-To-Guides/Developing-a-ROS-2-Package.html)
- [ROS 2 Topics, Services, and Actions](https://docs.ros.org/en/jazzy/How-To-Guides/Topics-Services-Actions.html)
- [colcon quick start](https://colcon.readthedocs.io/en/main/user/quick-start.html)
- [URDF with robot_state_publisher](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)
- [xacro](https://docs.ros.org/en/jazzy/p/xacro/)
- [URDF physical properties](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-the-URDF-Model.html)

## Gazebo, control, TF, SLAM, and navigation

- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/)
- [Launch Gazebo from ROS 2](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/)
- [Spawn a Gazebo model](https://gazebosim.org/docs/harmonic/ros2_spawn_model/)
- [gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [ros_gz_bridge](https://docs.ros.org/en/ros2_packages/jazzy/api/ros_gz_bridge/index.html)
- [Nav2](https://docs.nav2.org/)
- [Nav2 velocity smoother](https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html)
- [Nav2 collision monitor](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html)
- [Nav2 transform setup](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html)
- [slam_toolbox](https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/)

The historical native-DiffDrive implementation used the
[Gazebo Sim 8 DiffDrive system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html).
It is historical evidence, not the v1.0 control target.

## Local audio and models

- [PortAudio callback constraints](https://portaudio.com/docs/v19-doxydocs/writing_a_callback.html)
- [webrtc-audio-processing](https://gitlab.freedesktop.org/pulseaudio/webrtc-audio-processing)
- [sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx)
- [sherpa-onnx KWS model catalog](https://k2-fsa.github.io/sherpa/onnx/kws/pretrained_models/index.html)
- [Piper](https://github.com/rhasspy/piper)
- [llama.cpp](https://github.com/ggml-org/llama.cpp)
- [llama.cpp server schema output](https://github.com/ggml-org/llama.cpp/blob/master/tools/server/README.md)
- [Official Qwen3-0.6B-GGUF](https://huggingface.co/Qwen/Qwen3-0.6B-GGUF)

Runtime versions and models are accepted only through pinned manifests with
exact SHA-256 and license metadata; a repository link alone is not a pin.

## Engineering governance

- [REP-2004](https://reps.openrobotics.org/rep-2004/)
- [gitignore](https://git-scm.com/docs/gitignore)
- [git rm](https://git-scm.com/docs/git-rm)
- [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)
- [Semantic Versioning](https://semver.org/spec/v2.0.0.html)

## Community diagnosis

- [ROS Discourse](https://discourse.ros.org/)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)
