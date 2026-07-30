# 把差速驱动迁移到 ros2_control

Lesson 0007 · 目标时间 90–120 分钟

**本课唯一成果：** 用 `gz_ros2_control` 和
`diff_drive_controller` 替换产品模型中的 Gazebo 原生 DiffDrive，
通过 ROS 2 的 `TwistStamped` 命令完成一次受控直行和一次原地左转。

这是 v0.2 的第一条最小纵向切片：

```text
TwistStamped
  -> diff_drive_controller
  -> gz_ros2_control
  -> left/right wheel joint
  -> Gazebo physics
```

本课不加入 LiDAR、SLAM、Nav2、MissionRuntime 或 MotionGate，也不宣称
已经完成整个停止安全链。本课验收 controller 的基础 odom/TF 输出和
0.35 s consumer timeout；Lesson 0008 加入 world、LiDAR，并完成跨
launch graph / publisher GID 的 TF 唯一所有权深度审计；Lesson 0009
实现独立 MotionGate；Lesson 0010 再完成 Runtime、Gate 与 controller
的 crash-stop 和 managed-pause 验收。

先阅读 [ADR-0002](../../docs/adr/0002-migrate-to-gz-ros2-control.md)、
[运动安全契约](../../docs/architecture/safety-and-motion-contract.md) 和
[TF/运行模式契约](../../docs/architecture/tf-and-operating-modes.md)。

## 0. 从不可变 start checkpoint 开始

教师已经从 v0.1 的已发布 solution 创建 annotated tag
`course/0007-start`。不要直接在 `main` 上练习，也不要复制一份源码到
课程目录。推荐使用独立 worktree：

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws

git fetch origin --tags
git show --no-patch --decorate course/0007-start

git worktree add \
  ../voice_nav_robot_lesson_0007 \
  -b learn/0007 \
  course/0007-start

cd ../voice_nav_robot_lesson_0007
git status --short
```

`git status --short` 必须无输出。完成后会把自己的结果与
`course/0007-solution` 比较；solution tag 只有在实现通过 PR 和 hosted
CI、合并 `main` 后才创建，不能预先猜测其 commit。

## 1. 先解释为什么迁移

Lesson 0005 的 Gazebo DiffDrive 是正确的历史教学切片，但它有两个产品
缺口：

1. 命令、里程计和 TF 都停留在 Gazebo Transport 领域，需要额外 bridge；
1. 命令发布者消失后，原生插件仍可能保留最后一条非零命令。

迁移后，wheel joint 由标准 ros2_control hardware Interface 暴露；
`diff_drive_controller` 负责差速运动学、限速、里程计和消费端
`cmd_vel_timeout`；后续 MotionGate 可以只面对一个稳定的 ROS 2 控制入口。

这不是为了“换一个更高级的插件”，而是为了明确故障边界：

```text
上游发布者活着吗？
  └─ 由后续 MotionGate 的 steady-clock lease 处理

最终速度消费者还收到新命令吗？
  └─ 由 diff_drive_controller.cmd_vel_timeout 处理
```

两层 deadman 解决不同进程的故障。本课只建立第二层所在的控制链。

## 2. 冻结 Jazzy 的真实命令契约

VoiceNav Robot 固定使用 ROS 2 Jazzy。该版本
`diff_drive_controller` 的 `~/cmd_vel` **原生订阅**
`geometry_msgs/msg/TwistStamped`，不是 `Twist`。

因此：

- 全项目速度链使用 `TwistStamped`；
- 不添加 `enable_stamped_cmd_vel`；
- 不用来自其他发行版教程的参数名掩盖版本差异；
- 命令 topic 由 controller 名形成，本课固定为
  `/diff_drive_controller/cmd_vel`。

Jazzy 中不存在本项目可依赖的 `enable_stamped_cmd_vel` 参数。若当前
[运动安全契约](../../docs/architecture/safety-and-motion-contract.md)
仍写着“显式设置该参数”，把它作为本课的文档勘误，与实现一起修改并用
contract test 防止回归。

可对照官方
[Jazzy diff_drive_controller 文档](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
和
[Jazzy 控制器源码](https://github.com/ros-controls/ros2_controllers/blob/jazzy/diff_drive_controller/src/diff_drive_controller.cpp)，
不要用 Rolling 文档替代本项目固定版本。

## 3. Tests first：先写会失败的契约

先阅读当前模型并记录旧基线：

```bash
rg -n \
  "DiffDrive|ros2_control|GazeboSimSystem|cmd_vel_timeout" \
  src/voice_nav_sim

python3 -m unittest discover -s tests -p "test_*.py" -v
```

为本课增加静态/契约测试，至少覆盖：

- 展开后的机器人描述包含且只包含两个驱动轮 joint；
- 每个驱动轮具有 velocity command Interface，以及 position、velocity
  state Interface；
- hardware plugin 是 `gz_ros2_control/GazeboSimSystem`；
- Gazebo system plugin 是
  `gz_ros2_control::GazeboSimROS2ControlPlugin`；
- 原生 `gz::sim::systems::DiffDrive` 不再存在；
- controller YAML 的轮名、轮距 `0.40` m、轮径 `0.035` m 与 Xacro
  契约一致；
- `cmd_vel_timeout` 为 `0.35` s；
- 配置不包含 `enable_stamped_cmd_vel`；
- launch 不含 `sleep` 或机器绝对路径，并按依赖顺序启动 controllers；
- bridge allowlist 在本课只含 `/clock`。

先只提交测试，不改实现：

```bash
python3 -m unittest discover -s tests -p "test_*.py" -v
git diff --check
```

新测试必须因为仍存在原生 DiffDrive、缺少 ros2_control 和 controller
配置而失败。把失败原因记录在学习记录中；语法错误、找不到 fixture 或
测试根本未执行不算有效的红灯。

## 4. 声明依赖，而不是依赖本机偶然安装

在 `src/voice_nav_sim/package.xml` 中声明运行所需依赖，至少覆盖：

- `gz_ros2_control`；
- `controller_manager`；
- `joint_state_broadcaster`；
- `diff_drive_controller`；
- `ros_gz_sim`、`ros_gz_bridge`；
- `robot_state_publisher`、`xacro`、launch。

然后让 rosdep 解析工作区声明：

```bash
source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y

rosdep check --from-paths src --ignore-src
```

不要把 `sudo apt install ...` 当作仓库依赖记录；CI 只会看
`package.xml`。也不要把 `/opt/ros/jazzy` 下的文件复制进仓库。

## 5. 把两个 wheel joint 暴露给 gz_ros2_control

修改 `src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro`：

1. 删除 Lesson 0005 添加的模型级原生 DiffDrive plugin；
1. 增加名为 `GazeboSimSystem`、类型为 `system` 的
   `<ros2_control>`；
1. hardware plugin 使用 `gz_ros2_control/GazeboSimSystem`；
1. 只为 `left_wheel_joint` 和 `right_wheel_joint` 声明 Interface；
1. 每个 wheel joint 暴露一个 `velocity` command Interface，以及
   `position`、`velocity` state Interface；
1. 加入 Gazebo ros2_control system plugin，并让其读取 controller
   YAML。

不要给 fixed caster 或 laser joint 添加 command Interface。不要在
Xacro 中复制 wheel separation 和 radius；这些几何量已经由现有 property
定义，controller 配置必须由测试证明与它们一致。

controller YAML 的路径不能写成开发机绝对路径。推荐让 Xacro 接受
`controllers_file` 参数，launch 使用 `FindPackageShare` 与
`PathJoinSubstitution` 解析安装空间中的配置文件，再把结果交给 Xacro。
这样全新 CI checkout 在首次 build 前仍能展开源模型。

实现后只跑最短反馈：

```bash
xacro \
  src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro \
  controllers_file:=/tmp/controller-placeholder.yaml \
  > /tmp/voice_nav_robot.urdf

check_urdf /tmp/voice_nav_robot.urdf
gz sdf -p /tmp/voice_nav_robot.urdf \
  > /tmp/voice_nav_robot.sdf

rg -n \
  "GazeboSimSystem|gz_ros2_control|DiffDrive" \
  /tmp/voice_nav_robot.urdf \
  /tmp/voice_nav_robot.sdf
```

预期能找到 ros2_control hardware/system plugin，找不到原生
`gz::sim::systems::DiffDrive`。

## 6. 用可信 YAML 配置 controllers

新增 `src/voice_nav_sim/config/controllers.yaml`，并让 CMake
安装 `config/`。YAML 包含：

- controller manager 固定更新频率；
- `joint_state_broadcaster`；
- `diff_drive_controller`；
- 精确左右 wheel joint 名；
- `wheel_separation: 0.40`；
- `wheel_radius: 0.035`；
- `position_feedback: true`、`open_loop: false`；
- `odom_frame_id: odom`、`base_frame_id: base_footprint`；
- `enable_odom_tf: true`；
- `cmd_vel_timeout: 0.35`；
- 可信线速度、角速度和加速度限制。

这里的速度、加速度和 timeout 是受版本控制的可信策略，不由语音、LLM
或 Mission payload 指定。不要为了通过测试加入未在 Jazzy 参数契约中的
`enable_stamped_cmd_vel`。

本课必须证明 controller 产生基本 odom 和 `odom → base_footprint` TF，
并在仿真时间推进时执行 0.35 s consumer timeout。Lesson 0008 再通过
跨 launch graph 和 publisher GID 审计，证明任何受支持 composition 中
都没有第二个 TF owner，并把 controller 原生
`/diff_drive_controller/odom` 明确 remap 为产品级 `/odom`。

因此 `/diff_drive_controller/odom` 是本课**有意保留的中间
checkpoint**。不要把 target 架构中的 `/odom` 写成本课已经完成的事实。

## 7. 编写一个可重复启动的仿真 launch

新增 `src/voice_nav_sim/launch/simulation.launch.py`，一次启动：

1. `robot_state_publisher`，使用 `use_sim_time=true`；
1. Gazebo Harmonic；
1. `/clock` 的 ROS–Gazebo bridge；
1. 从同一份 `robot_description` 生成机器人；
1. `joint_state_broadcaster` spawner；
1. `diff_drive_controller` spawner。

启动顺序必须由 launch event 建模：

```text
spawn entity 完成
  -> joint_state_broadcaster spawner 完成
  -> diff_drive_controller spawner
```

不要用 `sleep 3` 猜 controller manager 何时可用。不要 bridge
`cmd_vel`、`joint_states`、`odom` 或 `/tf`；这些数据都已在 ROS 2
control 域内。`ros_gz_bridge` 在本课只负责 `/clock`。

launch 需要允许 headless CI 运行，也可提供 GUI 参数供手工观察，但
RViz 仍不是本课 launch 的必要组成。

## 8. 让红灯变绿

先运行聚焦检查：

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_lesson_0007
source /opt/ros/jazzy/setup.bash

python3 -m unittest discover -s tests -p "test_*.py" -v
rosdep check --from-paths src/voice_nav_sim --ignore-src

bash scripts/verify.sh voice_nav_sim
```

有效绿灯必须同时证明：

- 新 contract tests 的断言真实执行；
- Xacro、URDF 与 SDF 转换通过；
- package 成功 build；
- `voice_nav_sim` tests 为零失败。

若测试只因为被 skip 而“通过”，先修复环境或测试发现逻辑，不要继续。

## 9. 运行并检查控制链

**终端 1：启动仿真**

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_lesson_0007
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch voice_nav_sim simulation.launch.py
```

**终端 2：检查 controllers 和 Interfaces**

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/lcy/code/ros2/voice_nav_robot_lesson_0007/install/setup.bash

ros2 control list_controllers
ros2 control list_hardware_interfaces

ros2 topic type /diff_drive_controller/cmd_vel
ros2 topic info --verbose /diff_drive_controller/cmd_vel

ros2 topic echo --once /joint_states
ros2 topic echo --once /diff_drive_controller/odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

预期：

- `joint_state_broadcaster` 是 `active`；
- `diff_drive_controller` 是 `active`；
- 两个 wheel velocity command Interface 已 claimed；
- wheel position/velocity state Interfaces 可用；
- command topic 类型精确为
  `geometry_msgs/msg/TwistStamped`；
- 运动后 joint state 和 odometry 都会变化；
- `/diff_drive_controller/odom` 的 header/child frame 与 TF 语义为
  `odom → base_footprint`；
- 此时没有宣称产品级 `/odom` 已经存在。

若 controller 是 `unconfigured` 或 `inactive`，先查终端 1 的
controller-manager 错误与 YAML 作用域。若 hardware Interface 不存在，
查 `<ros2_control>` joint 名，不要先改 controller 名掩盖问题。

## 10. 发送有界 TwistStamped 并显式归零

先记录起点：

```bash
gz model -m voice_nav_robot -p
```

直行命令持续约两秒：

```bash
ros2 topic pub -r 10 \
  /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.15}, angular: {z: 0.0}}}"
```

约两秒后按 `Ctrl+C` 结束 publisher，并立即显式发布零：

```bash
ros2 topic pub --once \
  /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"

sleep 1
gz model -m voice_nav_robot -p
```

x 应明显增加。随后发送正角速度约两秒，再显式归零：

```bash
ros2 topic pub -r 10 \
  /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}, angular: {z: 0.6}}}"
```

结束 publisher 后：

```bash
ros2 topic pub --once \
  /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"

sleep 1
gz model -m voice_nav_robot -p
```

yaw 应为正向变化。若方向相反，检查 left/right joint 与 wheel axis；
不要交换语义名称或填负轮径。

## 11. 故障注入：命令发布者突然消失

本课要测量 controller 的 0.35 s consumer timeout。Lesson 0008 留下的
是跨 composition / publisher GID 的 TF 唯一所有权深度审计，不是把
timeout 继续延期。

打开 controller 的 limited-command 输出，并持续发送非零命令：

```bash
ros2 topic echo /diff_drive_controller/cmd_vel_out
```

在另一终端运行非零 publisher，机器人运动后用 `Ctrl+C` 模拟发布者消失，
**不要先发送零**。观察：

- `cmd_vel_out` 在最后一条非零命令年龄越过 0.35 s 后的第一个 control
  update 回到零；
- Gazebo pose 不再持续增长；
- publisher 退出不再像 Lesson 0005 一样无限保留非零 command。

随后仍显式发送一次零 TwistStamped 清理实验，并记录输出。这里看到
command 为零不代表机器人已在同一时刻物理静止；质量门禁会把“命令归零”
和“里程计进入静止容差”作为两个不同证据。

使用 `/clock` 或 controller 消息时间记录最后一条非零 sample 和第一条零
sample。允许的测量容差不超过一个已配置 control period；如果仿真暂停，
本次证据无效，因为 controller time 没有推进。

若 `cmd_vel_out` 不存在，检查是否启用了 controller 的 limited velocity
publication。若命令一直不归零，核对 YAML 的 `cmd_vel_timeout` 作用域和
仿真 `/clock` 是否在推进。

## 12. 完整门禁、评审与提交

关闭所有非零 publisher，并确认机器人停止后退出 Gazebo。然后运行：

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_lesson_0007

git diff --check
bash scripts/verify.sh
git status --short
git diff
```

按修改理由拆分提交；不要使用 `git add .`：

```bash
git add tests
git diff --cached
git commit -m "test(sim): define ros2 control drive contract"

git add src/voice_nav_sim
git diff --cached
git commit -m "feat(sim): migrate drive path to ros2 control"

git add \
  CHANGELOG.md \
  course \
  docs
git diff --cached
git commit -m "docs(course): teach ros2 control drive migration"
```

如果 tests-first 红灯测试已经先提交，保留真实的红→绿历史，不要为了
“好看”伪造顺序。提交 PR 前将学习记录中的 Pending 证据替换为真实输出，
关联 Work Item 与 GitHub Issue，并执行完整 staged-diff 自审。

教师只有在 required CI 通过、review conversation 解决、PR
rebase-merge 到 `main` 后才创建 annotated
`course/0007-solution`。课程 tag 不是个人练习分支的别名。

## 验收

- 产品模型不再加载 native Gazebo DiffDrive；
- 两个 wheel joint 由 `gz_ros2_control` 暴露；
- joint-state broadcaster 与 diff-drive controller 都 active；
- command topic 类型是 Jazzy 原生 `TwistStamped`；
- 直行和正向旋转符合几何语义；
- consumer timeout 的定性故障注入回到零 command；
- joint state、controller-native `/diff_drive_controller/odom` 与基础
  `odom → base_footprint` TF 输出有效；
- 0.35 s consumer timeout 在仿真时间上通过带一个 control-period
  容差的测量；
- bridge 未承载速度、joint state、odom 或 TF；
- 聚焦门禁和完整门禁均通过；
- Work Item、学习记录、提交、PR 和 CI 形成可审查证据链。

## 提交给教师

1. `git show --no-patch --decorate course/0007-start`。
1. tests-first 红灯命令、失败断言名称和退出状态。
1. `<ros2_control>` 与 Gazebo ros2_control plugin 的完整片段。
1. controller YAML 中 wheel、geometry、limit 和 timeout 配置。
1. `ros2 control list_controllers` 与
   `ros2 control list_hardware_interfaces` 输出。
1. `/diff_drive_controller/cmd_vel` 的类型和 verbose topic 信息。
1. 直行前后与旋转前后的 Gazebo pose。
1. joint state、`/diff_drive_controller/odom` frame 与
   `tf2_echo odom base_footprint` 证据，并解释它为何仍是中间
   checkpoint。
1. publisher 突然退出后，以仿真/controller 时间测量的
   `cmd_vel_out` 归零证据。
1. `bash scripts/verify.sh` 的最终摘要。
1. `git log --oneline --decorate` 与干净的 `git status --short`。
1. GitHub Issue、PR、required CI 和 review conversation 链接。
1. 回答四个问题：
   - 为什么 Jazzy 不应配置 `enable_stamped_cmd_vel`？
   - 为什么 wheel joint 同时需要 command 和 state Interfaces？
   - 为什么 controller spawner 不能用固定 sleep 排序？
   - 为什么 command 归零与机器人物理停稳必须分开证明？

## 主要资料

- [gz_ros2_control Jazzy 文档](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [diff_drive_controller Jazzy 文档](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [controller_manager spawner 文档](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html)
- [ROS 2 launch event handlers](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)

遇到 controller manager、hardware Interface 或 Gazebo plugin
错误时，先保留完整日志并定位故障层，不要同时改 joint、YAML 和 launch。
