# 让机器人稳定落地 Gazebo

Lesson 0004 · 目标时间 45–70 分钟

**本课唯一成果：** 为现有 Xacro 补齐碰撞、质量与惯量，将它生成的 URDF 放入 Gazebo Harmonic，并验证它落地后保持稳定。

## 本课不做什么

不加入 `ros2_control`、差速驱动、`cmd_vel`、里程计、Gazebo/ROS bridge、LiDAR 传感器、SLAM 或 Nav2。Gazebo 中的运动也暂时不会反馈到 ROS TF。

## 1. 先理解物理模型的三部分

- `visual` 决定外观。
- `collision` 决定接触边界。
- `inertial` 决定质量、质心与转动响应。

三者不能互相替代。先阅读 [URDF 物理属性速查](../reference/urdf-physical-properties.md)，再开始修改。

## 2. 给物理 link 补齐质量和惯量

修改 `src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro`。先新增以下质量参数，不要把数字散落到每个 link：

```
base_mass   = 5.0
wheel_mass  = 0.25
caster_mass = 0.20
laser_mass  = 0.10
base_com_x  = -0.02
```

为 `base_link`、两个 wheel link、`caster_link` 和 `laser_link` 各写一个 `inertial`。它与 `visual`、`collision` 同级，基本结构为：

```
<inertial>
  <origin xyz="质心位置"/>
  <mass value="${质量属性}"/>
  <inertia
    ixx="${公式}"
    ixy="0"
    ixz="0"
    iyy="${公式}"
    iyz="0"
    izz="${公式}"/>
</inertial>
```

按当前模型逐项完成：

- `base_link`：z 轴圆柱公式；质心为 `${base_com_x} 0 ${base_height / 2}`。
- 左右轮：圆柱主轴沿 y，因此 `Iyy` 是轴向惯量；把惯量写进现有 `drive_wheel` 宏，只写一次。
- `caster_link`：实心球公式；质心与球体中心一致， 即本 link 的 z=`caster_radius`。
- `laser_link`：z 轴圆柱公式；质心 z=`laser_height / 2`。
- `base_footprint` 保持空 link。它只是导航 frame，不是物理实体。

`base_com_x=-0.02` 是有意的：驱动轮接触线位于 x=0， caster 位于后方。让主要质量略偏后，可使质心投影落在三点支撑区域内。

## 3. 补齐碰撞与关节约束

- 为 `laser_link` 添加与 visual 同尺寸、同 origin 的 `collision`。其余实体已有 collision，保留即可。
- 在两个 continuous wheel joint 中加入 `limit`：effort=`5.0`、velocity=`20.0`。
- 在两个 wheel joint 中加入 `dynamics`：damping=`0.1`、friction=`0.0`。

 continuous joint 不写 lower/upper；它无限旋转，但仍需限制最大力矩和速度。

## 4. 声明仿真运行依赖

在 `src/voice_nav_sim/package.xml` 中新增：

```
<exec_depend>ros_gz_sim</exec_depend>
```

本课不新写总 launch。仍以 Xacro 为唯一模型源，先显式生成临时 URDF，再交给 Gazebo；这样解析失败和物理失败可以分开定位。

当前 Jazzy 环境中，`robot_state_publisher` 使用 `robot_description` 参数，但并不发布同名 topic。实测 `ros2 topic info /robot_description --verbose` 的 publisher count 为 0，因此不要用 `create --topic`。后续编写统一仿真 launch 时，再让同一次 Xacro 求值同时供给 state publisher 和 Gazebo spawn。

## 5. 离线校验、构建与测试

```
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
source /opt/ros/jazzy/setup.bash

mkdir -p /tmp/voice_nav_robot
xacro src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro \
  > /tmp/voice_nav_robot/model.urdf
check_urdf /tmp/voice_nav_robot/model.urdf
gz sdf -p /tmp/voice_nav_robot/model.urdf \
  > /tmp/voice_nav_robot/model.sdf

rosdep check --from-paths src/voice_nav_sim --ignore-src

colcon build --packages-select voice_nav_sim \
  --symlink-install --event-handlers console_direct+

source install/setup.bash

colcon test --packages-select voice_nav_sim \
  --event-handlers console_direct+
colcon test-result --verbose
```

`gz sdf -p` 应安静退出并返回 0。 任何质量或惯量出现 0、负数、`nan`，或者违反主惯量三角不等式时， 都不要继续启动 Gazebo。

## 6. 用两个终端运行最小仿真链

**终端 1：启动带地面的 Gazebo 世界**

```
source /opt/ros/jazzy/setup.bash

ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:="-r empty.sdf"
```

本机 Gazebo Sim 8 的 `empty.sdf` 已核对：world 名称为 `empty`，并且包含 `ground_plane`。

**终端 2：重新生成 URDF，然后创建实体**

```
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
source /opt/ros/jazzy/setup.bash

mkdir -p /tmp/voice_nav_robot
xacro src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro \
  > /tmp/voice_nav_robot/model.urdf

ros2 run ros_gz_sim create \
  --world empty \
  --file /tmp/voice_nav_robot/model.urdf \
  --name voice_nav_robot \
  --z 0.03
```

`--z 0.03` 让机器人从离地 3 cm 处落下，用实际接触检验 collision 和 inertial，而不是让它刚好“贴”在地面上。

`Entity creation successful` 只表示创建请求被接收， 不代表模型一定通过了物理解析。紧接着运行：

```
gz model --list
```

 列表中必须同时出现 `ground_plane` 和 `voice_nav_robot`；否则回到 Gazebo 启动终端阅读解析错误。

## 7. 验收稳定状态

等待至少 5 秒。机器人可以短暂下落和轻微晃动，但随后必须静止：不翻倒、不持续弹跳、不穿地、不突然飞走。

在终端 2 连续检查两次，中间间隔约 5 秒：

```
gz model -m voice_nav_robot -p
```

 两次输出的 z、roll、pitch 应基本不再变化，且不能出现 `nan`。不要求 model pose 的 z 精确等于 0。

**验收：** Xacro 与 URDF 解析成功；所有测试通过；实体能从生成的 URDF 创建；落地 5 秒后姿态稳定；visual、collision、inertial 的职责能够口头解释。

## 故障定位顺序

1. 创建超时：先确认 Gazebo 已启动，world 名确实是 `empty`。
1. create 报成功但列表无模型：查看 Gazebo 终端；通常是物理 link 缺少有效 inertial。
1. 提示重名：不要重复生成；关闭并重新启动 Gazebo 后再试。
1. 穿过地面：检查接触实体是否有 collision，以及 collision origin。
1. 飞走或疯狂抖动：检查质量、惯量、碰撞体重叠和数值数量级。
1. 向前翻倒：先检查质心与支撑区域，不要靠盲目增大 damping 掩盖。

## 提交给教师

1. 更新后的 Xacro 和 package.xml 完整内容。
1. `check_urdf` 完整输出。
1. `rosdep check`、构建摘要和 `colcon test-result --verbose`。
1. `create` 命令的完整输出。
1. 间隔约 5 秒的两份 `gz model -m voice_nav_robot -p` 输出。
1. 一张能看见地面和完整机器人的 Gazebo 截图。
1. 回答三个问题：visual、collision、inertial 各负责什么；为什么不给 `base_footprint` 加惯性；为什么底盘质心要略微移到轮轴后方。

## 主要资料

阅读 [ROS 2：URDF physical and collision properties](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html)、[Gazebo Harmonic：从 ROS 2 启动 Gazebo](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/) 和 [Gazebo Harmonic：从 ROS 2 生成模型](https://gazebosim.org/docs/harmonic/ros2_spawn_model/)。
