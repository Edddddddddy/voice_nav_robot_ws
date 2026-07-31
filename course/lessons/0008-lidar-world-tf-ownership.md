# 接入 360° LiDAR 并证明 TF 唯一所有权

Lesson 0008 · 目标时间 150–210 分钟

**本课唯一成果：** 在包内非空 world 中运行一个确定性的 360° 2D
LiDAR，把 Gazebo `/scan` 单向桥接为 ROS `LaserScan`，把 controller
odometry 直接 remap 为产品 `/odom`，并用“TF edge + publisher GID”证明
每条坐标变换只有一个预期 owner。

```text
voice_nav_test_world.sdf
  -> fixed collision box
  -> gpu_lidar on laser_link
  -> Gazebo /scan
  -> GZ_TO_ROS bridge
  -> ROS /scan
  -> timestamped odom -> laser_link lookup

diff_drive_controller ~/odom
  -> direct remap
  -> /odom

/tf + /tf_static
  -> (parent, child, publisher_gid)
  -> graph endpoint owner
```

本课不加入 SLAM、AMCL、Nav2、地图保存、Named Place、MotionGate、
Mission、Agent 或语音。图中不会出现 `map -> odom`。Lesson 0009 才实现
独立 MotionGate；Lesson 0010 再完成进程故障和 managed-pause 的停止链。

先阅读：

- [VN-0009 Work Item](../../docs/work-items/0009-lidar-world-tf-ownership.md)
- [TF 与运行模式契约](../../docs/architecture/tf-and-operating-modes.md)
- [TF Frame Contract](../reference/tf-frame-contract.md)

## 0. 从不可变 start checkpoint 开始

教师已从完成 Lesson 0007 的 reviewed `main` 创建 annotated tag
`course/0008-start`。先核对 tag object 和 peeled commit：

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws

git fetch origin --tags
git show --no-patch --decorate course/0008-start
git rev-parse course/0008-start
git rev-parse "course/0008-start^{}"
git ls-remote origin \
  refs/tags/course/0008-start \
  "refs/tags/course/0008-start^{}"
```

教师基线应为：

```text
tag object:
982ec062889b5a2ab92c391967b9084d15e52b60

peeled target:
f99210d8830cd2cd16eb801ffe0de10422cf4584
```

不要直接在 `main` 上练习。创建独立 worktree：

```bash
git worktree add \
  ../voice_nav_robot_lesson_0008 \
  -b learn/0008 \
  course/0008-start

cd ../voice_nav_robot_lesson_0008
git status --short
```

最后一条命令必须无输出。`course/0008-solution` 只有在实现通过 PR、
required hosted CI 并合并到 `main` 后才会创建；不能预先猜测它的 commit。

## 1. 先画清数据 owner，而不是先堆插件

本课有三种不同的“所有权”，不能混为一谈：

| 数据 | 语义 owner | transport / adapter |
| --- | --- | --- |
| 激光测距 | Gazebo `gpu_lidar` | `ros_gz_bridge` 只转换消息 |
| `/odom` 和 `odom -> base_footprint` | `diff_drive_controller` | 直接 ROS remap，无 relay |
| 内部机器人 frame | `robot_state_publisher` | 无 TF bridge |

`ros_gz_bridge` 不是 TF owner，也不会发布 `/tf` 或 `/tf_static`。LiDAR
sensor 生成测距数据，但 `base_link -> laser_link` 的几何仍由 Xacro 和
`robot_state_publisher` 唯一发布。

目标 TF 树在本课只有 `odom` 以下部分：

```text
odom
└── base_footprint
    └── base_link
        ├── left_wheel
        ├── right_wheel
        ├── caster_link
        └── laser_link
```

`map -> odom` 不存在不是缺陷，而是本课边界。SLAM 和 AMCL 将在不同运行
模式中分别成为这条 edge 的 owner。

## 2. 冻结一个可计算的 world 与 LiDAR 契约

“RViz 看起来有激光线”不是可回归的验收。先冻结解析几何。

### World 几何

包内 world 固定为：

```text
src/voice_nav_sim/worlds/voice_nav_test_world.sdf
```

它至少包含：

- 直接写在 SDF 中的 ground collision；
- 一个静态 box collision，center 为 `(2.0, 0.0, 0.5)` m；
- box size 为 `(0.5, 1.0, 1.0)` m；
- Gazebo Physics、UserCommands、SceneBroadcaster 和 Sensors systems；
- Sensors system 的 `render_engine` 为 `ogre2`。

禁止使用 Fuel、HTTP(S)、只在本机缓存中存在的 `model://` 资产，或把系统
`empty.sdf` 当产品 world。world 必须由 CMake 安装，再由 package share
解析，不能从源码绝对路径启动。

box 的前表面为：

```text
x_front = 2.0 - 0.5 / 2 = 1.75 m
```

机器人初始 `laser_link.x = 0.10 m`，所以零角附近的解析距离约为：

```text
dx = 1.75 - 0.10 = 1.65 m
```

### LiDAR 几何

本课固定一个 single-layer 360° `gpu_lidar`：

| 字段 | 值 |
| --- | --- |
| parent/frame | `laser_link` |
| Gazebo topic | `/scan` |
| update rate | `10 Hz` |
| horizontal samples | `360` |
| horizontal resolution | `1` |
| min/max angle | `-pi / +pi` |
| vertical samples | `1` |
| vertical min/max angle | `0 / 0` |
| range min/max | `0.05 / 8.0 m` |
| range resolution | `0.01 m` |
| noise | none |

360 个 sample 覆盖包含两端的 `[-pi, +pi]` 时，不保证数组中恰好有
`0.0 rad`。测试必须从消息的 `angle_min` 和 `angle_increment` 计算：

```text
theta_i = angle_min + i * angle_increment
i* = argmin_i(abs(theta_i))
r_expected = (1.75 - laser_world_x) / cos(theta_i*)
```

然后验证交点的 y 落在 box 正面宽度内，并用 range resolution 加少量数值
容差比较 `ranges[i*]`。不要硬编码“第 180 个 beam 必须等于 1.65”。

## 3. Tests first：先让产品契约正确失败

先查看 Lesson 0007 的真实基线：

```bash
rg -n \
  "empty.sdf|clock_bridge|/diff_drive_controller/odom|gpu_lidar|/scan" \
  src/voice_nav_sim scripts tests

python3 -m unittest discover -s tests -p "test_*.py" -v
```

先增加 checker、valid fixture 和负向 fixtures，再把同一个 checker 用于
repository product。静态/纯函数契约至少覆盖：

### World 负向 fixtures

- world 没有任何固定 obstacle；
- obstacle 只有 visual，没有 collision；
- obstacle pose 或 size 错误；
- world 使用 Fuel、HTTP(S) 或不受控远端 URI；
- 缺少 Physics、UserCommands、SceneBroadcaster 或 Sensors system；
- `worlds/` 没有被安装。

### LiDAR 负向 fixtures

- LiDAR 缺失或重复；
- parent 或 `gz_frame_id` 不是 `laser_link`；
- topic 不是绝对 `/scan`；
- samples、角度、update rate、range 或 vertical layer 不匹配；
- 添加了 noise；
- headless launch 没有启用 Gazebo headless rendering。

### Bridge 负向 fixtures

- allowlist 多出第三个 topic；
- `/scan` 是 `BIDIRECTIONAL` 或 `ROS_TO_GZ`；
- ROS/Gazebo message type 错误；
- `/scan` 没有 sensor-data QoS；
- `/clock` 没有 clock QoS；
- 覆盖为 wall-time timestamp；
- bridge 速度、joint state、odom、`/tf` 或 `/tf_static`。

### Odometry 与 TF 负向 fixtures

- controller 没有直接 remap `~/odom:=/odom`；
- 新增 odom relay/republisher；
- 仍保留 `/diff_drive_controller/odom` publisher；
- 同一 `(parent, child)` edge 出现两个 publisher GID；
- 两个 GID 故意复用相同 node name；
- 出现 `map -> odom`；
- frame 带前导 `/`。

还要有一个**有效的多 publisher fixture**：两个 GID 分别发布不同 edge
应通过。否则测试可能错误地实现为“`/tf` 只能有一个 publisher”。

运行红灯：

```bash
python3 -m unittest discover -s tests -p "test_*.py" -v
git diff --check
```

有效 RED 应满足：

1. checker 本身能执行；
1. synthetic valid fixture 通过；
1. 每个负向 fixture 因预期原因被拒绝；
1. 只有 repository product assertion 因仍缺 world/LiDAR/remap 而失败。

语法错误、fixture 路径错误、导入 ROS 失败、测试没有被发现，或把所有新测试
都 skip，不算 tests-first 证据。

先阅读完整 staged test diff，再提交真实红灯：

```bash
git add \
  scripts \
  tests \
  src/voice_nav_sim/test
git diff --cached
git commit -m "test(sim): define lidar and tf ownership contract"
```

不要为了让历史“好看”而在实现后伪造 red commit。

## 4. 编写可离线加载的 packaged world

新增 `worlds/voice_nav_test_world.sdf`。核心结构应类似：

```xml
<sdf version="1.10">
  <world name="voice_nav_test_world">
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics"/>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands"/>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster"/>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Directly authored ground collision and fixed box model. -->
  </world>
</sdf>
```

ground 和 box 都直接定义 primitive visual/collision，不依赖远端 include。
box 必须是 static，collision 与 visual 可共享几何，但 contract 只把
collision 当传感器/物理事实。

在 `CMakeLists.txt` 安装 `worlds/`。launch 用
`FindPackageShare('voice_nav_sim')` 与 `PathJoinSubstitution` 获取安装空间
路径，把 world 文件直接传给 Gazebo。同步把 spawn 命令的 world 名从
`empty` 改为 `voice_nav_test_world`。

`headless:=true` 时除了 server-only 还必须传
`--headless-rendering`，否则 GPU LiDAR 在无窗口 CI 中可能没有渲染输出。
不要通过 sleep 猜 sensor ready；integration test 应有有界等待。

聚焦检查：

```bash
source /opt/ros/jazzy/setup.bash

gz sdf -k \
  src/voice_nav_sim/worlds/voice_nav_test_world.sdf

rg -n \
  "fuel|https?://|model://|Sensors|test_obstacle|collision" \
  src/voice_nav_sim/worlds/voice_nav_test_world.sdf
```

若使用 `model://` 只是因为本机已有 Ground Plane cache，删除它并直接写
plane；网络断开后的 CI 不会继承你的 cache。

## 5. 在现有 laser_link 上挂一个 sensor

机器人模型已经有 `laser_link` 和固定 joint。不要再创建第二个
`laser_frame`，也不要增加 `static_transform_publisher`。

在 Xacro 的 `laser_link` Gazebo 扩展中增加类似配置：

```xml
<gazebo reference="laser_link">
  <sensor name="lidar" type="gpu_lidar">
    <always_on>true</always_on>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <topic>/scan</topic>
    <gz_frame_id>laser_link</gz_frame_id>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.141592653589793</min_angle>
          <max_angle>3.141592653589793</max_angle>
        </horizontal>
        <vertical>
          <samples>1</samples>
          <resolution>1</resolution>
          <min_angle>0</min_angle>
          <max_angle>0</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.05</min>
        <max>8.0</max>
        <resolution>0.01</resolution>
      </range>
    </lidar>
  </sensor>
</gazebo>
```

关键点：

- 新 Gazebo 使用 `<lidar>`，不要机械复制 Gazebo Classic plugin；
- `<gz_frame_id>` 让 Gazebo scan 本身报告 `laser_link`；
- sensor pose 为 link-local zero，安装几何仍来自现有 fixed joint；
- 本课明确无 noise，不添加 gaussian 配置；
- `/scan` 是受控绝对 topic，避免 model-scoped 名逃逸到产品接口。

检查展开后的模型，而不只检查 Xacro 文本：

```bash
xacro \
  src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro \
  controllers_file:=/tmp/controller-placeholder.yaml \
  > /tmp/voice_nav_robot.urdf

check_urdf /tmp/voice_nav_robot.urdf

gz sdf -p /tmp/voice_nav_robot.urdf \
  > /tmp/voice_nav_robot.sdf

rg -n \
  "gpu_lidar|gz_frame_id|<topic>/scan|<samples>360" \
  /tmp/voice_nav_robot.sdf
```

若 Xacro 中存在、转换后的 SDF 中消失，说明 tag 层级或 SDF 版本不被接受；
不要只放宽静态测试。

## 6. 用 bridge.yaml 明确方向、类型和 QoS

新增 `src/voice_nav_sim/config/bridge.yaml`，把 bridge 从一条 CLI 字符串
升级为可审查 allowlist：

```yaml
- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
  qos_profile: CLOCK

- ros_topic_name: /scan
  gz_topic_name: /scan
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS
  qos_profile: SENSOR_DATA
```

launch 从 package share 解析 `bridge.yaml`，通过 `config_file` 参数交给
一个 `parameter_bridge` process。不要同时保留旧 clock CLI bridge，
否则 `/clock` 会有重复 publisher。

显式禁止：

```text
/cmd_vel
/joint_states
/odom
/tf
/tf_static
```

桥接方向必须是 `GZ_TO_ROS`。`BIDIRECTIONAL` 虽然“也能收到 scan”，却把
ROS 写回 Gazebo 的非预期入口一起暴露出来。不要开启
`override_timestamps_with_wall_time`；TF、odom 与 scan 必须共享仿真时间。

`package.xml` 应直接声明 bridge 配置和测试实际用到的 ROS dependencies，
而不是依赖桌面安装的传递依赖。运行：

```bash
rosdep check --from-paths src/voice_nav_sim --ignore-src
```

## 7. 把 controller odometry 直接命名为 /odom

Lesson 0007 的 controller-native checkpoint 是：

```text
/diff_drive_controller/odom
```

本课把 controller 的 private `~/odom` 在创建 controller node 时直接
remap：

```python
arguments=[
    'diff_drive_controller',
    # existing controller-manager bounds ...
    '--controller-ros-args',
    '--ros-args --remap ~/odom:=/odom',
]
```

这是 controller endpoint 的名字解析，不是新增 relay。不要写：

```text
/diff_drive_controller/odom -> relay node -> /odom
```

relay 会增加一个故障、时间戳和 QoS 边界，而且旧 endpoint 仍存在。目标
证据必须是：

```text
/odom:
  one publisher endpoint
  node = /diff_drive_controller

/diff_drive_controller/odom:
  zero publisher endpoints
```

`ros2 topic list` 受 discovery cache 影响，只看“topic 名是否出现”不够。
integration test 用 `get_publishers_info_by_topic()` 检查 endpoint，并核对
GID/node identity。

手工诊断：

```bash
ros2 topic info --verbose /odom
ros2 topic info --verbose /diff_drive_controller/odom
ros2 topic echo --once /odom
```

`/odom` 的 `header.frame_id` 必须为 `odom`，`child_frame_id` 必须为
`base_footprint`。

## 8. 正确证明 TF edge 唯一所有权

下面两个结论都不充分：

```text
“/tf 有两个 publisher，所以重复了”
“publisher node name 是 robot_state_publisher，所以没问题”
```

原因：

- 多个合法 owner 会把不同 edge 复用到同一个 `/tf` topic；
- 一个 publisher 可在一条 `TFMessage` 中发送多个 edge；
- 两个独立 endpoint 可以故意使用同一个 node name；
- 同一个 node 在 `/tf` 与 `/tf_static` 上有不同 publisher GID。

### Audit 数据模型

订阅 `/tf` 和 `/tf_static` 时，让 callback 同时接收 message info：

```text
TFMessage.transforms
MessageInfo.publisher_gid
```

对每个 transform 记录：

```text
key   = (header.frame_id, child_frame_id)
value = (topic, publisher_gid)
```

同时从 graph 读取两个 topic 的 publisher endpoint info：

```text
(topic, endpoint_gid)
  -> node_namespace
  -> node_name
  -> topic type
  -> QoS
```

等待 discovery 完成后，把 observed message GID 与 graph endpoint GID
关联。未知 GID 不能被忽略；它可能只是 discovery race，也可能是已经退出的
writer。测试应有 bounded retry，超时后明确失败。

### 本课预期 edge

| Topic | Edge | Expected owner |
| --- | --- | --- |
| `/tf` | `odom -> base_footprint` | `/diff_drive_controller` |
| `/tf` | `base_link -> left_wheel` | `/robot_state_publisher` |
| `/tf` | `base_link -> right_wheel` | `/robot_state_publisher` |
| `/tf_static` | `base_footprint -> base_link` | `/robot_state_publisher` |
| `/tf_static` | `base_link -> caster_link` | `/robot_state_publisher` |
| `/tf_static` | `base_link -> laser_link` | `/robot_state_publisher` |

`/tf_static` 订阅必须兼容 transient-local QoS，否则 test node 晚启动时会
漏掉已经发布的 fixed edges。每个 edge 的 GID set 必须恰好有一个元素，
且 graph endpoint 必须映射到预期的绝对 fully qualified owner。edge
出现在错误 topic 也必须失败。VIOLATION 可以立即结束测试；成功必须等
完整的有界观测窗口结束，不能在短暂稳定后提前退出。

还必须显式断言：

```text
map -> odom is absent
```

不要使用 `ros2 run tf2_tools view_frames` 的 PDF 代替 GID 审计。它适合人类
观察树结构，但不能证明 edge 的 endpoint identity。

## 9. 用同一时间戳验证 scan、odom 与 TF

“现在能 `tf2_echo`”不能证明历史 scan 可变换。SLAM 处理的是每条 scan 的
采样时刻，不是你查看终端的 wall time。

headless integration test 至少等待三条 scan，并对每条执行：

```text
lookup_transform(
  target_frame="odom",
  source_frame="laser_link",
  time=scan.header.stamp
)
```

三条 `scan.header.stamp` 必须严格递增，且能够在 TF buffer 中解析完整链：

```text
odom
  -> base_footprint
  -> base_link
  -> laser_link
```

不要用 `Time()` / latest transform 替代 scan stamp。latest 能通过而历史
查询失败，会在后续 SLAM 中表现为 Message Filter 丢包。

对 `/odom` 消息，在它自己的 `header.stamp` 查询
`odom -> base_footprint`，比较：

- x、y translation；
- quaternion 或展开后的 yaw。

两者来自同一个 controller pose/timestamp source，误差只允许数值级容差。
不要把 Gazebo ground-truth pose拿来替代这项一致性检查；ground truth 与
controller odometry 是不同证据。

## 10. 用解析 beam 证明你收到的不是空 scan

收到 360 个 range 并不保证 sensor 看见世界。integration test 在机器人
运动前：

1. 读取 scan 的 `angle_min`、`angle_increment` 和 `ranges`；
1. 计算 `i* = argmin(abs(theta_i))`；
1. 在 scan timestamp 查询 `odom -> laser_link` 或等价 world 初始几何；
1. 计算到 box x=1.75 前平面的 ray distance；
1. 验证交点 y 位于 `[-0.5, +0.5]`；
1. 比较 observed range 与 analytic range。

初始位置下期望值约为 `1.65 m`，但报告必须包含实际 `theta_i*` 和计算值。
建议容差由 `0.01 m` range resolution、角离散和少量仿真数值误差组成，
而不是宽到可以把 ground 或侧面误认为目标。

至少同时断言：

- `len(ranges) == 360`；
- `header.frame_id == "laser_link"`；
- reported angle/range limits 与配置一致；
- 目标 beam 是有限数，落在 `[range_min, range_max]`；
- 多帧时间戳推进。

其他方向没有障碍时出现 `+inf` 是合法 LaserScan 语义，不要要求所有 360
个 range 都有限。

## 11. 运行 headless 产品图

先完成聚焦构建与静态检查：

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_lesson_0008
source /opt/ros/jazzy/setup.bash

python3 -m unittest discover -s tests -p "test_*.py" -v
rosdep check --from-paths src/voice_nav_sim --ignore-src
bash scripts/verify.sh voice_nav_sim
```

然后启动：

```bash
source install/setup.bash
ros2 launch voice_nav_sim simulation.launch.py headless:=true
```

headless 模式应真正使用 Gazebo `--headless-rendering`。另一终端检查：

```bash
source /opt/ros/jazzy/setup.bash
source \
  /mnt/c/Users/lcy/code/ros2/voice_nav_robot_lesson_0008/install/setup.bash

gz topic -i -t /scan
ros2 topic type /scan
ros2 topic info --verbose /scan
ros2 topic echo --once /scan

ros2 topic info --verbose /odom
ros2 topic info --verbose /diff_drive_controller/odom

ros2 topic info --verbose /tf
ros2 topic info --verbose /tf_static
```

预期：

- `/clock` 推进；
- `/scan` 是 `sensor_msgs/msg/LaserScan`；
- `/scan` publisher 使用 sensor-data QoS；
- frame、360 beams、角度、range 和 stamp 符合契约；
- `/odom` 只有 controller endpoint；
- 旧 odom name 没有 publisher endpoint。

`ros2 topic info --verbose /tf` 只是 endpoint 诊断；最终唯一所有权证据仍由
自动 edge/GID audit 产生。

## 12. 故障注入：专门击穿错误的唯一性算法

### 同名双 writer

构造 synthetic graph：

```text
GID_A -> node /robot_state_publisher
GID_B -> node /robot_state_publisher

GID_A publishes base_link -> laser_link
GID_B publishes base_link -> laser_link
```

两个 endpoint 的 node name 故意相同。正确 audit 必须报告该 edge 有两个
GID 并失败；按 node name 去重的错误实现会假通过。

### 合法的不同 edge writers

再构造：

```text
GID_A publishes odom -> base_footprint
GID_B publishes base_link -> laser_link
```

它应通过唯一性聚合。若测试因为 `/tf` 有两个 publisher 而失败，说明算法
错误地按 topic 计数。

### 运行中 owner 稳定性

在 headless integration 中记录初始 edge/GID owner set，向
`/diff_drive_controller/cmd_vel` 发布一段受限 `TwistStamped`，然后显式
发布零。等待 odom 和 scan 继续更新后再次审计：

- owner set 与运动前完全相同；
- scan timestamp 仍能查询 TF；
- `/odom` 仍只有 controller endpoint；
- 旧 odom endpoint 仍为零；
- cleanup 路径一定发零并终止所有 launch-owned process。

本课的运动只用于证明 graph 不随状态变化，不重复 Lesson 0007 的 consumer
timeout，也不宣称 MotionGate 或 crash-stop 已完成。

## 13. 常见故障按层定位

### Gazebo 没有 /scan

按顺序检查：

1. installed world 是否真的是本课 world；
1. world 是否加载 Sensors system；
1. headless server 是否有 `--headless-rendering`；
1. 展开后的 SDF 是否保留 `gpu_lidar`；
1. Gazebo 日志是否有 Ogre2/rendering 错误。

不要先改 bridge；Gazebo `/scan` 不存在时，bridge 无数据可转。

### Gazebo 有 /scan，ROS 没有

检查 `bridge.yaml` 的 topic、type、direction、QoS 和安装路径。确认 launch
只启动一个 parameter bridge，没有同时保留旧 clock-only bridge。

### ROS scan frame 错误

检查 sensor 的 `<gz_frame_id>laser_link</gz_frame_id>`。不要用第二个
static transform 或全局 frame override 掩盖模型语义错误。

### latest TF 成功，scan-time TF 失败

检查所有 node 的 `use_sim_time`、bridge 是否覆盖 wall time、TF buffer
cache 和订阅启动顺序。必须查询 `scan.header.stamp`，不能把测试改成 latest。

### /odom 正确，但旧 topic 仍有 publisher

大概率使用了 relay，或 remap 参数交给了 spawner 自己而不是 controller
node。检查 `--controller-ros-args` 的完整字符串：

```text
--ros-args --remap ~/odom:=/odom
```

### TF audit 报 unknown GID

先用 bounded discovery retry 等待 graph endpoint；仍然未知时保留 GID、
topic 和采样时间并失败。不要把 unknown writer 归到“看起来最像”的 node。

## 14. 完整门禁、自审与提交

关闭所有非零 publisher，确认已发送零，然后退出 launch：

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_lesson_0008

git diff --check
bash scripts/verify.sh
git status --short
git diff
```

检查没有遗留进程：

```bash
pgrep -af \
  "gz sim|controller_manager|parameter_bridge|robot_state_publisher|ros_gz_sim.*create" \
  || true
```

按变更原因显式 staging，不使用 `git add .`：

```bash
git add \
  src/voice_nav_sim/worlds \
  src/voice_nav_sim/config/bridge.yaml \
  src/voice_nav_sim/urdf \
  src/voice_nav_sim/launch \
  src/voice_nav_sim/CMakeLists.txt \
  src/voice_nav_sim/package.xml
git diff --cached
git commit -m "feat(sim): add lidar world and product graph"

git add \
  CHANGELOG.md \
  course \
  docs
git diff --cached
git commit -m "docs(course): teach lidar and tf ownership"
```

tests-first commit 应已在实现前产生。每次 commit 前都完整阅读 staged diff；
不要把 generated world cache、logs、screenshots、build/install/log 或
`__pycache__` 提交。

教师只有在 required CI 通过、review conversation 解决、PR rebase-merge
到 `main` 后才创建 annotated `course/0008-solution`。学习分支 commit
不能冒充 public reviewed solution。

## 验收

- package share 中存在自包含非空 world；
- world 含固定 collision box 与所有 required Gazebo systems；
- headless rendering 下 Gazebo 和 ROS 都持续发布 `/scan`；
- ROS scan 的 frame、360° geometry、range、QoS 和仿真时间正确；
- nearest-to-zero beam 与 box front-face 解析距离一致；
- bridge allowlist 精确为 `/clock` 和 `/scan`，均为 GZ-to-ROS；
- controller 直接发布产品 `/odom`，旧 endpoint 无 publisher；
- 三条 scan 都能在自己的 timestamp 查询 `odom -> laser_link`；
- matched `/odom` 与 TF pose 一致；
- 每条 observed TF edge 只有一个 GID，并关联到预期 owner；
- 同名双 writer fixture 被拒绝，合法 disjoint writers 被接受；
- 不存在 `map -> odom`；
- bounded motion + zero 前后 owner set 不变；
- 聚焦、完整和 clean-process gates 全部通过；
- Work Item、学习记录、commit、PR、CI 和 review 形成可查询证据链。

## 提交给教师

1. `course/0008-start` 的 tag object、peeled target 和 remote 输出。
1. tests-first RED 的命令、退出状态、失败 assertion 和总测试数，并说明
   valid/negative fixtures 为什么证明 checker 自身有效。
1. installed `voice_nav_test_world.sdf` 路径、required systems、ground 与
   obstacle collision 片段。
1. 展开后 SDF 中完整的 `gpu_lidar` 片段。
1. 完整 `bridge.yaml`，并解释为何 `/scan` 不能是 bidirectional。
1. `/scan` type、verbose endpoint QoS、frame、geometry 和至少三条递增
   timestamp。
1. nearest-to-zero beam 的 index、angle、解析 range、实际 range 与容差。
1. `/odom` 和旧 odom topic 的 verbose publisher endpoint 输出。
1. matched timestamp 的 odom pose、TF pose 和误差。
1. 运行时 edge/topic/GID/owner 表，以及不存在 `map -> odom` 的断言。
1. 同名双 writer 与合法 disjoint-writer fixtures 的测试结果。
1. bounded motion + explicit zero 前后的 owner set 与 scan-time TF 结果。
1. `bash scripts/verify.sh` 摘要和无遗留进程检查。
1. `git log --oneline --decorate` 与干净的 `git status --short`。
1. GitHub Issue、PR、required CI 和 review conversation 链接。
1. 用自己的话回答复盘问题。

## 复盘问题

1. 为什么“`/tf` 有两个 publisher”不能直接证明 TF 冲突？
1. 为什么 node name 相同仍不能把两个 TF writer 当作同一 owner？
1. 为什么 `/tf_static` 的测试订阅需要 transient-local-compatible QoS？
1. 为什么 scan 必须在 `scan.header.stamp` 查询 TF，而不是查询 latest？
1. 为什么 direct remap 比 odom relay 更符合一个 owner 的架构？
1. 为什么 360 samples 的 `[-pi,+pi]` 不能硬编码 index 180 就是零角？
1. 如果 Gazebo 有 `/scan` 而 ROS 没有，你会按什么顺序定位故障层？
1. 本课如何证明 world 不依赖开发机网络或 Fuel cache？

## 主要资料

- [Gazebo Harmonic Sensors 教程](https://gazebosim.org/docs/harmonic/sensors/)
- [ros_gz_bridge Jazzy 配置](https://docs.ros.org/en/jazzy/p/ros_gz_bridge/)
- [ros2_control Jazzy controller manager 迁移说明](https://control.ros.org/jazzy/doc/ros2_control/doc/migration.html)
- [ROS 2 Jazzy tf2 时间教程](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp.html)

遇到 sensor、bridge、timestamp 或 TF ownership 错误时，先保留精确日志和
GID evidence，再只修改一个故障层。不要同时改 world、sensor、bridge 和
TF 测试来追逐绿灯。
