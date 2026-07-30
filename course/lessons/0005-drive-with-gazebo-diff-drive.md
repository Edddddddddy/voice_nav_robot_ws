# 让差速机器人第一次受控运动

Lesson 0005 · 目标时间 50–75 分钟

**本课唯一成果：** 给机器人加入 Gazebo 原生 DiffDrive system，通过 Gazebo Transport 命令它直行和原地左转，并用里程计与 model pose 证明它能运动、能停住。

> 课程演进说明：本课保留原生 DiffDrive 作为理解物理驱动与 Gazebo Transport 的最小纵向切片。产品基线将在 Lesson 0007 起迁移到 `gz_ros2_control` 与 `diff_drive_controller`，以获得速度消费端的 `cmd_vel_timeout`。本课配置不是最终运行架构。

## 为什么先不接 ROS

```
gz.msgs.Twist
      │
      ▼
Gazebo DiffDrive ──► 左右轮关节 ──► 物理运动
      │
      └──────────────────────────► Gazebo Odometry
```

本课只验证图中的一条仿真链。若方向、轮距、摩擦或限速出错，问题一定在机器人模型或 Gazebo 驱动层，不会被 bridge 和 ROS topic 干扰。

本课不加入 `ros2_control`、ROS–Gazebo bridge、JointState bridge、LiDAR、SLAM、Nav2 或总 launch。先阅读 [差速驱动与里程计契约](../reference/differential-drive-contract.md)。

## 1. 先算清楚两个几何参数

当前 Xacro 已给出：

```
wheel_radius = 0.035 m
wheel_y      = 0.20 m
```

`wheel_y` 是单个轮心相对车体中心线的 y 坐标， 而 DiffDrive 需要左右轮接触线之间的完整距离，所以：

```
wheel_separation = 2 × wheel_y = 0.40 m
```

 不要把轮宽加进这个值，也不要填 `0.20`。 轮距错误时，直行可能看似正常，但所有旋转里程计都会按错误比例计算。

## 2. 降低后置支撑球的横向摩擦

 修改 `src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro`。 在 `caster_link` 已定义之后添加：

```
<gazebo reference="caster_link">
  <mu1>0.001</mu1>
  <mu2>0.001</mu2>
</gazebo>
```

 当前 caster 是固定球，不会像真实万向轮那样绕竖轴转向。 较小摩擦让它承担支撑作用，同时减少原地旋转时的横向拖拽。 这不是驱动轮的摩擦设置，不要把左右轮也改成低摩擦。

## 3. 加入 Gazebo DiffDrive system

 在 Xacro 的 `</robot>` 之前加入一个模型级插件。 使用已有属性表达几何关系，不重复硬编码轮径和轮距：

```
<gazebo>
  <plugin
    filename="gz-sim-diff-drive-system"
    name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>${2 * wheel_y}</wheel_separation>
    <wheel_radius>${wheel_radius}</wheel_radius>

    <odom_publish_frequency>50</odom_publish_frequency>
    <frame_id>odom</frame_id>
    <child_frame_id>base_footprint</child_frame_id>

    <min_linear_velocity>-0.20</min_linear_velocity>
    <max_linear_velocity>0.40</max_linear_velocity>
    <min_angular_velocity>-1.20</min_angular_velocity>
    <max_angular_velocity>1.20</max_angular_velocity>

    <min_linear_acceleration>-0.50</min_linear_acceleration>
    <max_linear_acceleration>0.50</max_linear_acceleration>
    <min_angular_acceleration>-1.50</min_angular_acceleration>
    <max_angular_acceleration>1.50</max_angular_acceleration>
  </plugin>
</gazebo>
```

 暂时不要写 `<topic>`、`<odom_topic>` 和 `<tf_topic>`。省略后，Gazebo 会按运行时 model 名 自动生成有作用域的默认 topic：

- `/model/voice_nav_robot/cmd_vel`
- `/model/voice_nav_robot/odometry`
- `/model/voice_nav_robot/tf`
- `/model/voice_nav_robot/enable`

但 frame 名不会采用模型作用域，所以显式写成 `odom → base_footprint`，与项目的 [TF contract](../reference/tf-frame-contract.md) 对齐。

## 4. 离线确认插件没有在转换中丢失

```
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
source /opt/ros/jazzy/setup.bash

mkdir -p /tmp/voice_nav_robot
xacro src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro \
  > /tmp/voice_nav_robot/model.urdf
check_urdf /tmp/voice_nav_robot/model.urdf

gz sdf -p /tmp/voice_nav_robot/model.urdf \
  > /tmp/voice_nav_robot/model.sdf

grep -n "gz-sim-diff-drive-system" /tmp/voice_nav_robot/model.sdf
grep -n -A 6 "caster_link" /tmp/voice_nav_robot/model.sdf | head -n 20

rosdep check --from-paths src/voice_nav_sim --ignore-src

colcon build --packages-select voice_nav_sim \
  --symlink-install --event-handlers console_direct+
source install/setup.bash

colcon test --packages-select voice_nav_sim \
  --event-handlers console_direct+
colcon test-result --verbose
```

 第一条 `grep` 必须找到插件。第二条用于人工确认 caster 的 collision surface 中保留了约 `0.001` 的摩擦。 不要仅以 `check_urdf` 成功作为插件存在的证据： 它主要检查 URDF 结构，不负责证明 Gazebo 扩展已正确进入 SDF。

## 5. 在全新 world 中生成机器人

 关闭 Lesson 0004 遗留的 Gazebo 实例，再打开两个终端。 不要同时运行桌面展示用的 `display.launch.py`。

**终端 1：启动仿真**

```
source /opt/ros/jazzy/setup.bash

ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:="-r empty.sdf"
```

**终端 2：生成并创建实体**

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

等待机器人稳定落地后检查：

```
gz topic -l | grep "/model/voice_nav_robot"

gz topic -i \
  -t /model/voice_nav_robot/cmd_vel

gz model -m voice_nav_robot -p
```

 必须看见上一节列出的四个 model-scoped topic； `cmd_vel` 的消息类型必须是 `gz.msgs.Twist`。 如果 topic 不存在，先看终端 1 是否报告插件加载失败，不要直接去改关节。

## 6. 先观察 Gazebo 里程计

**终端 3：**

```
gz topic -e -n 1 \
  -t /model/voice_nav_robot/odometry
```

 先确认 header 中的 frame 是 `odom`，child frame 是 `base_footprint`。`-n 1` 表示收到一条消息后自动退出； 此时尚未发送运动命令。

## 7. 直行两秒，然后显式停车

**安全规则：** Gazebo Sim 8 的 DiffDrive 不提供 command timeout。 它会保留最后一条非零命令，所以关闭发布命令的终端不等于停车。 每个运动测试必须以零 Twist 结束。

 在终端 2 先记录起点，再把下面整段一次性粘贴执行：

```
gz model -m voice_nav_robot -p

gz topic -t /model/voice_nav_robot/cmd_vel \
  -m gz.msgs.Twist \
  -p "linear: {x: 0.15}, angular: {z: 0.0}"

sleep 2

gz topic -t /model/voice_nav_robot/cmd_vel \
  -m gz.msgs.Twist \
  -p "linear: {x: 0.0}, angular: {z: 0.0}"

sleep 1
gz model -m voice_nav_robot -p
sleep 3
gz model -m voice_nav_robot -p
```

结果应满足：

- x 明显增大，通常约为 0.2–0.4 m；
- y 和 yaw 只发生小幅漂移；
- 最后两份 pose 基本相同，证明零命令后已经停住。

## 8. 原地左转两秒，然后显式停车

再次先记录起始 pose，并把整段一次性执行：

```
gz model -m voice_nav_robot -p

gz topic -t /model/voice_nav_robot/cmd_vel \
  -m gz.msgs.Twist \
  -p "linear: {x: 0.0}, angular: {z: 0.6}"

sleep 2

gz topic -t /model/voice_nav_robot/cmd_vel \
  -m gz.msgs.Twist \
  -p "linear: {x: 0.0}, angular: {z: 0.0}"

sleep 1
gz model -m voice_nav_robot -p
sleep 3
gz model -m voice_nav_robot -p
```

结果应满足：

- yaw 为正并有明显变化，表示从上方看逆时针左转；
- x、y 的变化远小于直行测试；
- 最后两份 pose 再次基本相同。

 如方向相反，先核对 `left_joint` / `right_joint` 名称和轮关节 axis，不要通过交换语义或随意填负轮径掩盖问题。

## 仿真 operational stop 备用命令

 如果运动命令后无法正常发出零 Twist，可禁用本模型的 DiffDrive：

```
gz topic -t /model/voice_nav_robot/enable \
  -m gz.msgs.Boolean \
  -p "data: false"
```

继续实验前需用相同命令发送 `data: true`。这个 Gazebo 开关只是调试兜底，不是经过功能安全认证的急停，也不是项目最终的 `StopMission` 协议。

## 架构演进：为什么本课先不用 ros2_control

本课选择 Gazebo 原生 DiffDrive，是为了用最少组件验证轮距、轮径、关节方向、摩擦和里程计。这个选择只服务于教学顺序，不等于产品架构已经定型。

本课暴露了一个关键缺口：原生 DiffDrive 没有 command timeout，会保留最后一条非零命令。Lesson 0007 起迁移到 `gz_ros2_control` + `diff_drive_controller`，并在独立 MotionGate 之外增加消费端 deadman；这正是从教学原型演进到可故障注入架构的理由。

**验收：** DiffDrive 插件与 caster surface 正确进入 SDF； 四个 model-scoped topic 存在；直行方向正确；正角速度产生左转； 两个测试都由显式零 Twist 停住；现有构建与测试仍全部通过。

## 提交给教师

1. Xacro 中新增的 caster Gazebo 块和 DiffDrive 插件完整内容。
1. `check_urdf`、两条 `grep` 和测试摘要。
1. `gz topic -l | grep "/model/voice_nav_robot"` 输出。
1. 一小段 odometry 输出，需看见两个 frame 名。
1. 直行测试的三份 pose：运动前、停车后、再等 3 秒。
1. 旋转测试的三份 pose：运动前、停车后、再等 3 秒。
1. 一张运动后的 Gazebo 截图。
1. 回答三个问题：为什么轮距是 `2 * wheel_y`；为什么本课保留 model-scoped Gazebo topic；为什么命令发布进程退出不代表机器人会停车。

## 主要资料

 阅读 [Gazebo Sim 8 DiffDrive API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)、 [Nav2：Setting Up Odometry — Gazebo](https://docs.nav2.org/setup_guides/odom/setup_odom_gz.html) 和 [Nav2：Setting Up Transformations](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html)。
