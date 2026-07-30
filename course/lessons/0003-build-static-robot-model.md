# 手写最小机器人模型与 TF

Lesson 0003 · 目标时间 45–70 分钟

**本课唯一成果：** 在 RViz 中显示你亲手写的差速机器人，并验证 `base_footprint → laser_link` 的 TF。

> 历史保真说明：正文迁移原 HTML 作业契约，不按后来的实现反向改写。
> 学习者最终提交与原始几何草案的差异单独记录在
> [Lesson 0003 验收实现勘误](../reference/lesson-0003-model-errata.md)；
> 当前 `main` 对应勘误中的已验收实现。

## 本课不做什么

暂不加入 Gazebo、碰撞、惯性、ros2_control、LiDAR 插件和轮子驱动。先把几何结构与 frame 语义做对。

## 1. 固定模型契约

```
base_footprint
└── base_link
    ├── left_wheel_link
    ├── right_wheel_link
    ├── caster_link
    └── laser_link
```

模型尺寸统一使用米和弧度：

- 底盘盒：长 0.40、宽 0.28、高 0.12。
- 轮子：半径 0.075、宽 0.04，左右中心的 y 为 ±0.16。
- `base_link` 相对地面高度为 0.075。
- 底盘盒中心相对 `base_link` 的 z 为 0.06。
- 万向轮使用半径 0.02 的球体，x 为 -0.15。
- LiDAR 相对 `base_link`：x=0.10、y=0、z=0.16。

## 2. 手写 Xacro

创建：

```
src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro
```

要求：

- 根元素使用 `robot`，声明 xacro XML namespace。
- 尺寸先定义为 `xacro:property`，几何和 joint origin 引用属性。
- `base_footprint → base_link` 使用 fixed joint。
- 左右轮使用 continuous joint，joint axis 为 y 轴。
- URDF 圆柱默认沿 z 轴，轮子 visual 需要旋转到 y 轴。
- caster 和 laser 使用 fixed joint。
- 每个可见 link 至少有 visual、geometry 和 material。
- 不要复制现有 TurtleBot 模型，也不要加入 Gazebo 标签。

## 3. 先做离线校验

```
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
source /opt/ros/jazzy/setup.bash

mkdir -p /tmp/voice_nav_robot

xacro src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro \
  > /tmp/voice_nav_robot/model.urdf

check_urdf /tmp/voice_nav_robot/model.urdf
```

预期为 6 个 link、5 个 joint，并以 `base_footprint` 为根。这一步失败时不要继续写 launch。

## 4. 编写显示 launch

创建：

```
src/voice_nav_sim/launch/display.launch.py
```

launch 只启动：

- `robot_state_publisher`，其 `robot_description` 来自运行 xacro 的 Command substitution。
- `joint_state_publisher`，为两个 continuous wheel joint 发布零位置。

不要在 Python 中硬编码绝对路径；使用 `FindPackageShare` 和 `PathJoinSubstitution`。暂时在另一个终端手动运行 `rviz2`，不要把 RViz 塞进 launch。

## 5. 安装运行资产

修改 `voice_nav_sim/CMakeLists.txt`，把 `urdf` 和 `launch` 安装到 package share。

修改 `voice_nav_sim/package.xml`，声明运行时依赖：

```
xacro
robot_state_publisher
joint_state_publisher
launch
launch_ros
```

## 6. 构建与运行

```
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
source /opt/ros/jazzy/setup.bash

rosdep check --from-paths src/voice_nav_sim --ignore-src

colcon build --packages-select voice_nav_sim \
  --symlink-install --event-handlers console_direct+

source install/setup.bash

colcon test --packages-select voice_nav_sim \
  --event-handlers console_direct+

colcon test-result --verbose

ros2 launch voice_nav_sim display.launch.py
```

新开 WSL 终端：

```
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

rviz2
```

在 RViz 中把 Fixed Frame 设为 `base_footprint`，添加 RobotModel 和 TF display。

## 7. TF 验收

```
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

**验收：** RViz 模型结构合理、轮子接地；TF 能从 `base_footprint` 连到 `laser_link`；该变换能由模型尺寸正确推导；全部 package 测试无失败。

## 提交给教师

1. Xacro、display launch、CMakeLists 和 package.xml 的完整内容。
1. `check_urdf` 完整输出。
1. `rosdep check` 和构建摘要。
1. `colcon test-result --verbose` 输出。
1. `tf2_echo base_footprint laser_link` 的一帧输出。
1. 一张 RViz 截图。
1. 回答：为什么 2D 导航仍要区分 `base_footprint` 和 `base_link`？

## 主要资料

阅读 [ROS 2 Jazzy：Using URDF with robot_state_publisher](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html) 和 [xacro 官方文档](https://docs.ros.org/en/jazzy/p/xacro/)。项目自己的 frame 约定见 [TF Frame Contract](../reference/tf-frame-contract.md)。遇到坐标轴、joint origin 或 launch substitution 不清楚时，先停下来问教师。
