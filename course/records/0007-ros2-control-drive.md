# Lesson 0007 学习记录：迁移到 ros2_control 差速驱动

状态：Completed

参考 solution 已通过本地完整门禁、独立 review、required hosted CI，
并由 PR #6 rebase 合并到 `main`。本记录只填写已经发生且可查询的证据；
学习者仍从 start tag 创建自己的 `learn/0007` 分支并独立完成复盘。

## 变更身份

- Work Item：`VN-0008`
- GitHub Issue：
  [#5](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/5)
- 开发分支：`feat/vn-0008-l0007-ros2-control-drive`
- 学习分支：`learn/0007`
- Start tag：`course/0007-start`
- Solution tag：`course/0007-solution`
- 红灯契约 commit：
  `a7feee41ee9330676a09ebc50c4461a9af76a90e`
- 绿色实现 commit：
  `ed621353c6ab4c3544a29b5763106a63539833d9`
- PR：
  [#6](https://github.com/Edddddddddy/voice_nav_robot_ws/pull/6)
- Required CI：
  [`required / ubuntu-24.04 / ros-jazzy`](https://github.com/Edddddddddy/voice_nav_robot_ws/actions/runs/30565921460/job/90950343524)
- Public merge commit：
  `68bb49051680e1cd5cd138982b70bd4c89c5c920`

## Tests-first 证据

- [x] 记录首次红灯命令、退出状态和失败测试名称。
- [x] 证明失败来自缺少 ros2_control 产品实现，而不是测试语法、
  fixture 路径或环境错误。
- [x] 记录实现后的同组绿灯命令、退出状态和测试数量。

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

Final same-group green command (including review-strengthening tests):
PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s tests -p 'test_*.py' -v
Exit status:
0
Result summary:
38 tests ran; all passed.
```

## 静态与依赖证据

- [x] `rosdep check --from-paths src --ignore-src` 通过。
- [x] Xacro 展开、`check_urdf` 和 URDF→SDF 转换通过。
- [x] SDF 中存在 gz_ros2_control system plugin。
- [x] 产品模型中不存在 native Gazebo DiffDrive plugin。
- [x] controller 配置不包含 `enable_stamped_cmd_vel`。
- [x] bridge allowlist 在本课只含 `/clock`。

```text
Commands:
rosdep check --from-paths src --ignore-src
python3 scripts/check_control_contract.py ...
xacro ... controllers_file:=... > /tmp/voice-nav-model.urdf
check_urdf /tmp/voice-nav-model.urdf
gz sdf -p /tmp/voice-nav-model.urdf > /tmp/voice-nav-model.sdf
python3 scripts/check_sdf_contract.py /tmp/voice-nav-model.sdf

Exit status:
0

Relevant output:
All system dependencies have been satisfied
Control contract passed.
Successfully Parsed XML; root link: base_footprint
SDF contract passed.
```

## 运行控制链证据

- [x] `joint_state_broadcaster` 为 `active`。
- [x] `diff_drive_controller` 为 `active`。
- [x] 两个 wheel velocity command Interfaces 已 claimed。
- [x] wheel position/velocity state Interfaces 可用。
- [x] `/diff_drive_controller/cmd_vel` 类型为
  `geometry_msgs/msg/TwistStamped`。
- [x] 运动会改变 wheel joint state 和 controller odometry。
- [x] controller-native `/diff_drive_controller/odom` 与基础 TF 查询报告
  `odom → base_footprint`。
- [x] 证据明确说明产品级 `/odom` remap 和跨图 publisher 唯一性仍由
  Lesson 0008 完成，没有把 target 写成 current。

```text
ros2 control list_controllers:
diff_drive_controller   diff_drive_controller/DiffDriveController      active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster  active

ros2 control list_hardware_interfaces:
command interfaces
  diff_drive_controller/angular/velocity [available] [unclaimed]
  diff_drive_controller/linear/velocity [available] [unclaimed]
  left_wheel_joint/velocity [available] [claimed]
  right_wheel_joint/velocity [available] [claimed]
state interfaces
  left_wheel_joint/position
  left_wheel_joint/velocity
  right_wheel_joint/position
  right_wheel_joint/velocity

ros2 topic type/info:
/diff_drive_controller/cmd_vel:
geometry_msgs/msg/TwistStamped
Publisher count: 0
Subscription count: 1
Subscriber node: /diff_drive_controller
QoS reliability: BEST_EFFORT

Joint state:
Both wheel positions changed by more than 0.1 rad.

/diff_drive_controller/odom frames/sample:
header.frame_id: odom
child_frame_id: base_footprint
forward_odometry_delta_m: 0.146

tf2_echo odom base_footprint:
The launch test observed an odom -> base_footprint transform.
This is the controller-native checkpoint. Product `/odom` remapping and the
cross-graph publisher-GID uniqueness audit remain Lesson 0008 work.
```

## 运动与故障注入证据

- [x] 记录直行前后 pose，x 按预期增加。
- [x] 记录正角速度旋转前后 pose，yaw 按预期增加。
- [x] 在不先发布零的情况下销毁非零 publisher endpoint。
- [x] 使用仿真/controller 时间记录最后一条非零 command 与第一条零
  command；年龄越过 0.35 s 后不超过一个 control period 归零。
- [x] 单独记录机器人进入物理静止的观察，不把 command-zero
  时间冒充物理停车时间。
- [x] 实验结束时显式发送零并安全清理进程。

```text
Forward pose before:
x=0.077 m from `gz model -m voice_nav_robot -p`
Forward pose after:
x=0.222 m; Gazebo ground-truth delta=+0.145 m

Rotate pose before:
yaw=0.249 rad from `gz model -m voice_nav_robot -p`
Rotate pose after:
yaw=0.734 rad; Gazebo ground-truth delta=+0.485 rad

Controller cross-check:
forward_odometry_delta_m=0.146
positive_yaw_delta_rad=0.486

Hard-limit injection:
Input linear.x=2.0, angular.z=2.0
cmd_vel_out linear.x=0.400, angular.z=1.200

Publisher termination time:
The dedicated fault-injection publisher endpoint was destroyed immediately
after its final non-zero sample; no zero was sent first.
Last non-zero command simulation time:
8.564 s
First zero command simulation time:
8.919 s
Configured control period:
0.010 s
Observed zero-command time:
0.355 s; required upper bound is 0.35 + 0.010 + 0.002 = 0.362 s.
Physical stationarity observation:
After command zero, controller odometry reported
|linear.x| < 0.02 m/s and |angular.z| < 0.02 rad/s.
```

## 本地完整门禁

- [x] `git diff --check` 通过。
- [x] `bash scripts/verify.sh` 通过。
- [x] repository tests、build、package tests 均为零失败。
- [x] 提交前阅读完整 staged diff。

```text
Verification date/environment:
2026-07-31; WSL2 Ubuntu 24.04; ROS 2 Jazzy; Gazebo Sim 8.11.0
Command:
bash scripts/verify.sh
Exit status:
0
Repository test summary:
38 tests, all passed
Build summary:
6 packages finished
ROS test summary:
32 tests, 0 errors, 0 failures, 1 skipped
Final marker:
VoiceNav Robot verification passed.
```

## 评审与远端证据

- [x] Work Item 已关联 GitHub Issue。
- [x] PR diff 只包含 VN-0008 范围。
- [x] required hosted CI 通过。
- [x] review conversation 已解决。
- [x] PR 以 rebase 方式合并。
- [x] annotated `course/0007-solution` 指向 public reviewed solution。

```text
Issue: https://github.com/Edddddddddy/voice_nav_robot_ws/issues/5
PR: https://github.com/Edddddddddy/voice_nav_robot_ws/pull/6
Required CI:
https://github.com/Edddddddddy/voice_nav_robot_ws/actions/runs/30565921460/job/90950343524
Result: success in 5m06s on bfac7a98fcfe322bec24ecf25a7536eeb81c479a
Review:
https://github.com/Edddddddddy/voice_nav_robot_ws/pull/6#issuecomment-5134178763
Independent review found no remaining P0/P1; no unresolved review conversation.
Public merge identity:
68bb49051680e1cd5cd138982b70bd4c89c5c920
Merge method/time: rebase; 2026-07-30T17:32:07Z
Solution tag object:
a4e75e8205f1e59c516e28d5ef8f7e02c30aaaad
Solution tag peeled target:
68bb49051680e1cd5cd138982b70bd4c89c5c920
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
