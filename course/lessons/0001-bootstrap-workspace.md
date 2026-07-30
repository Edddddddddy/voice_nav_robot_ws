# 建立 VoiceNav Robot 工作区

Lesson 0001 · 目标时间 15–25 分钟

**本课唯一成果：** 亲手创建六个空 ROS package，并让整个工作区第一次构建成功。

## 先回忆

在执行命令前，先用一句话回答：为什么长时间运行且需要反馈、取消的 Mission 应该使用 Action，而不是 Topic 或 Service？把答案和最终终端输出一起发给教师。

## 1. 进入 WSL 并确认环境

```
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
ros2 --help >/dev/null && echo "ros2 cli: OK"
colcon --help >/dev/null && echo "colcon: OK"
```

预期看到 `jazzy`、`ros2 cli: OK` 和 `colcon: OK`。

## 2. 建立 workspace

```
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
mkdir -p src
cd src
```

## 3. 创建六个 package

```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 \
  voice_nav_interfaces

ros2 pkg create --build-type ament_cmake --license Apache-2.0 \
  voice_nav_sim

ros2 pkg create --build-type ament_cmake --license Apache-2.0 \
  voice_nav_audio

ros2 pkg create --build-type ament_python --license Apache-2.0 \
  voice_nav_agent

ros2 pkg create --build-type ament_cmake --license Apache-2.0 \
  voice_nav_mission

ros2 pkg create --build-type ament_cmake --license Apache-2.0 \
  voice_nav_bringup
```

## 4. 第一次构建

```
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon list
```

**验收：** 构建必须成功，`colcon list` 必须列出六个 `voice_nav_*` package。不要提前添加节点或自定义消息。

## 你需要提交给教师

1. “为什么 Mission 使用 Action”的一句话回答。
1. `colcon build` 最后 20 行输出。
1. `colcon list` 完整输出。
1. 如果失败，保留第一条报错，不要只发最后一行。

## 主要资料

阅读官方 [Developing a ROS 2 package](https://docs.ros.org/en/jazzy/How-To-Guides/Developing-a-ROS-2-Package.html) 中的“Creating a package”部分即可。遇到任何不理解的命令，先停下来问教师。
