# Lesson 0007 学习记录：迁移到 ros2_control 差速驱动

状态：Pending

本文件是未完成的验收证据模板，不是完成声明。只有真实实现通过本地完整
门禁、PR review 和 required hosted CI，并合并到 `main` 后，才能把状态
改为 `Completed`。不得预填测试成功、commit SHA、PR 链接或
`course/0007-solution` 指向。

## 变更身份

- Work Item：`VN-0008`
- GitHub Issue：
  [#5](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/5)
- 开发分支：`feat/vn-0008-l0007-ros2-control-drive`
- 学习分支：`learn/0007`
- Start tag：`course/0007-start`
- Solution tag：TBD（仅在 reviewed merge 后创建）
- PR：TBD
- Required CI：TBD
- Public merge commit：TBD

## Tests-first 证据

- [x] 记录首次红灯命令、退出状态和失败测试名称。
- [x] 证明失败来自缺少 ros2_control 产品实现，而不是测试语法、
  fixture 路径或环境错误。
- [ ] 记录实现后的同组绿灯命令、退出状态和测试数量。

```text
Red command:
python3 -m unittest discover -s tests -p 'test_*.py' -v
Exit status:
1
Expected contract failure:
test_repository_control_contract_passes
Control contract failed: native Gazebo DiffDrive plugin must not remain in the
product model

29 tests ran. The valid fixture and all new negative fixtures passed; only the
repository-level product assertion failed, so this is a valid red state.

Green command:
Exit status:
Result summary:
```

## 静态与依赖证据

- [ ] `rosdep check --from-paths src --ignore-src` 通过。
- [ ] Xacro 展开、`check_urdf` 和 URDF→SDF 转换通过。
- [ ] SDF 中存在 gz_ros2_control system plugin。
- [ ] 产品模型中不存在 native Gazebo DiffDrive plugin。
- [ ] controller 配置不包含 `enable_stamped_cmd_vel`。
- [ ] bridge allowlist 在本课只含 `/clock`。

```text
Commands:

Exit status:

Relevant output:
```

## 运行控制链证据

- [ ] `joint_state_broadcaster` 为 `active`。
- [ ] `diff_drive_controller` 为 `active`。
- [ ] 两个 wheel velocity command Interfaces 已 claimed。
- [ ] wheel position/velocity state Interfaces 可用。
- [ ] `/diff_drive_controller/cmd_vel` 类型为
  `geometry_msgs/msg/TwistStamped`。
- [ ] 运动会改变 wheel joint state 和 controller odometry。
- [ ] controller-native `/diff_drive_controller/odom` 与基础 TF 查询报告
  `odom → base_footprint`。
- [ ] 证据明确说明产品级 `/odom` remap 和跨图 publisher 唯一性仍由
  Lesson 0008 完成，没有把 target 写成 current。

```text
ros2 control list_controllers:

ros2 control list_hardware_interfaces:

ros2 topic type/info:

Joint state:

/diff_drive_controller/odom frames/sample:

tf2_echo odom base_footprint:
```

## 运动与故障注入证据

- [ ] 记录直行前后 pose，x 按预期增加。
- [ ] 记录正角速度旋转前后 pose，yaw 按预期增加。
- [ ] 在不先发布零的情况下终止非零 publisher。
- [ ] 使用仿真/controller 时间记录最后一条非零 command 与第一条零
  command；年龄越过 0.35 s 后不超过一个 control period 归零。
- [ ] 单独记录机器人进入物理静止的观察，不把 command-zero
  时间冒充物理停车时间。
- [ ] 实验结束时显式发送零并安全清理进程。

```text
Forward pose before:
Forward pose after:

Rotate pose before:
Rotate pose after:

Publisher termination time:
Last non-zero command simulation time:
First zero command simulation time:
Configured control period:
Observed zero-command time:
Physical stationarity observation:
```

## 本地完整门禁

- [ ] `git diff --check` 通过。
- [ ] `bash scripts/verify.sh` 通过。
- [ ] repository tests、build、package tests 均为零失败。
- [ ] 提交前阅读完整 staged diff。

```text
Verification date/environment:
Command:
Exit status:
Repository test summary:
Build summary:
ROS test summary:
Final marker:
```

## 评审与远端证据

- [ ] Work Item 已关联 GitHub Issue。
- [ ] PR diff 只包含 VN-0008 范围。
- [ ] required hosted CI 通过。
- [ ] review conversation 已解决。
- [ ] PR 以 rebase 方式合并。
- [ ] annotated `course/0007-solution` 指向 public reviewed solution。

```text
Issue:
PR:
Required CI:
Review:
Public merge identity:
Solution tag target:
```

## 复盘

完成后用自己的话回答，不复制课程原文：

1. 为什么 Jazzy 的控制入口不配置 `enable_stamped_cmd_vel`？
1. command Interface 与 state Interface 分别服务什么职责？
1. 为什么 launch event 比固定 sleep 更能表达启动依赖？
1. 为什么 consumer timeout、MotionGate lease 和物理停车是三个不同
   层次的问题？
1. 本次红→绿过程中最早暴露真实错误的是哪一层测试？为什么？

```text
Learner reflection:
TBD
```
