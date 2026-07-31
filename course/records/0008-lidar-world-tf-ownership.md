# Lesson 0008 学习记录：LiDAR、world 与 TF 唯一所有权

状态：Pending

本记录只填写已经发生且可复查的证据。当前仅 immutable start checkpoint
已经验证；tests-first、实现、PR、CI、review、merge 与 solution tag 均保持
Pending。不要把课程中的期望输出复制成实验结果。

## 变更身份

- Work Item：`VN-0009`
- GitHub Issue：
  [#8](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/8)
- 开发分支：`feat/vn-0009-l0008-lidar-tf-ownership`
- 学习分支：`learn/0008`
- Start tag：`course/0008-start`
- Start tag object：
  `982ec062889b5a2ab92c391967b9084d15e52b60`
- Start tag peeled target：
  `f99210d8830cd2cd16eb801ffe0de10422cf4584`
- Tests-first contract commit：TBD
- Green implementation commit：TBD
- Documentation/evidence commit：TBD
- GitHub PR：TBD
- Required CI：TBD
- Public merge identity：TBD
- Solution tag：TBD，且只能在 reviewed public merge 后创建

## Immutable start checkpoint

- [x] annotated start tag 存在于本地。
- [x] remote tag object 与本地一致。
- [x] local/remote peeled target 都是 reviewed Lesson 0007 closure。
- [ ] 学习者从 start tag 创建独立 `learn/0008` worktree。
- [ ] 开始修改前 `git status --short` 无输出。

```text
Commands already verified by the teacher:
git show-ref --tags course/0008-start
git rev-parse "course/0008-start^{}"
git ls-remote origin \
  refs/tags/course/0008-start \
  "refs/tags/course/0008-start^{}"

Tag object:
982ec062889b5a2ab92c391967b9084d15e52b60

Local peeled target:
f99210d8830cd2cd16eb801ffe0de10422cf4584

Remote peeled target:
f99210d8830cd2cd16eb801ffe0de10422cf4584
```

## Tests-first RED 证据

- [x] 记录首次 RED 的完整命令与退出状态。
- [x] 记录总测试数和唯一 repository product assertion。
- [x] valid fixture 已通过。
- [x] world、LiDAR、bridge、odom-remap 和额外 TF publisher 的负向
  fixtures 均因
  预期原因通过。
- [x] 失败来自缺少产品实现，而不是 syntax/import/fixture/discovery 错误。
- [ ] 实现修改发生在 tests-first commit 之后。

```text
Command:
env PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s tests -p 'test_*.py' -v

Exit status:
1

Total tests:
80

Expected repository assertion:
Simulation contract failed: simulation launch must load the packaged
non-empty test world; found built-in empty.sdf

Valid-fixture result:
test_synthetic_valid_contract_passes ... ok

Negative-fixture result:
All 40 focused positive/negative simulation-contract fixtures passed.

Why this is a valid RED:
79 tests passed and the only failure was the repository product assertion.
Its first diagnostic identifies the still-loaded built-in empty.sdf. The
synthetic valid fixture and every focused contract fixture executed
successfully, so the failure is not a parser, import, test-discovery, or
fixture defect. No Lesson 0008 production source had been changed.
```

## Packaged world 证据

- [ ] `voice_nav_test_world.sdf` 从 package share 加载。
- [ ] installed world 与 source contract 一致。
- [ ] world 不含 Fuel、HTTP(S) 或依赖本机 cache 的 URI。
- [ ] Physics、UserCommands、SceneBroadcaster、Sensors systems 全部存在。
- [ ] headless path 使用 `--headless-rendering`。
- [ ] ground 与固定 obstacle 都有 collision。
- [ ] obstacle center 为 `(2.0, 0.0, 0.5)` m。
- [ ] obstacle size 为 `(0.5, 1.0, 1.0)` m。

```text
Installed world path:
TBD

World validation command and exit status:
TBD

Required systems:
TBD

Obstacle collision pose/size:
TBD

Network-dependency audit:
TBD
```

## LiDAR 与 bridge 证据

- [ ] Gazebo `/scan` 持续发布。
- [ ] ROS `/scan` 类型为 `sensor_msgs/msg/LaserScan`。
- [ ] `header.frame_id` 精确为 `laser_link`。
- [ ] update rate、360 samples、360° angle、single layer 与 range contract
  一致。
- [ ] sensor 没有 noise。
- [ ] ROS scan stamps 使用仿真时间且至少三条严格递增。
- [ ] `/scan` publisher 使用 sensor-data QoS。
- [ ] bridge allowlist 只有 `/clock` 与 `/scan`。
- [ ] 两项 direction 均为 `GZ_TO_ROS`。
- [ ] bridge 没有 wall-time timestamp override。
- [ ] bridge 不承载 cmd、joint state、odom、`/tf` 或 `/tf_static`。

```text
Gazebo /scan topic/type:
TBD

ROS /scan type:
TBD

ROS /scan endpoint QoS:
TBD

Frame and scan geometry:
TBD

Three increasing scan stamps:
TBD

Complete bridge.yaml:
TBD
```

## 解析 beam 证据

- [ ] 从消息几何计算 `theta_i`，没有硬编码 center index。
- [ ] 选择 `i* = argmin(abs(theta_i))`。
- [ ] 根据 box front face 与实际 beam angle 计算 expected range。
- [ ] ray/plane 交点位于 box 正面 y 范围内。
- [ ] observed range 在由 range resolution 与仿真数值误差组成的窄容差内。

```text
Laser world x:
TBD

Box front-face x:
1.75 m

Selected index:
TBD

Selected beam angle:
TBD

Expected analytic range:
TBD

Observed range:
TBD

Allowed tolerance:
TBD

Absolute error:
TBD
```

## Product /odom 证据

- [ ] controller 使用 `--controller-ros-args` 直接 remap
  `~/odom:=/odom`。
- [ ] launch 中不存在 odom relay/republisher。
- [ ] `/odom` 有且只有一个 publisher endpoint。
- [ ] endpoint GID 映射到 `diff_drive_controller`。
- [ ] `/diff_drive_controller/odom` 有零个 publisher endpoint。
- [ ] `/odom` frame 为 `odom`，child frame 为 `base_footprint`。

```text
Controller remap fragment:
TBD

/odom verbose publisher endpoint:
TBD

/diff_drive_controller/odom publisher endpoint count:
TBD

/odom frame sample:
TBD
```

## Scan-time TF 与 matched pose 证据

- [ ] 至少三条 scan 都在自己的 `header.stamp` 成功查询
  `odom -> laser_link`。
- [ ] 没有用 latest transform 替代 timestamped lookup。
- [ ] `/odom` pose 与 `odom -> base_footprint` TF 在同一 odom stamp 比较。
- [ ] translation 和 yaw 误差在数值容差内。

```text
Scan stamp 1 / transform result:
TBD

Scan stamp 2 / transform result:
TBD

Scan stamp 3 / transform result:
TBD

Matched odom stamp:
TBD

Odometry pose:
TBD

TF pose:
TBD

Translation error:
TBD

Yaw error:
TBD
```

## TF edge / publisher GID ownership 证据

- [ ] `/tf` 与 `/tf_static` callback 都记录
  `MessageInfo.publisher_gid`。
- [ ] graph endpoint GID 与 observed message GID 成功关联。
- [ ] 每个 expected edge 恰好有一个 publisher GID。
- [ ] 每个 GID 映射到预期 node owner。
- [ ] frame 名精确且无前导 `/`。
- [ ] `map -> odom` 不存在。
- [ ] `/tf_static` 使用 transient-local-compatible 订阅 QoS。

```text
Runtime ownership table:

topic | parent -> child | publisher GID | graph owner | result
TBD

Unknown/unmapped GIDs:
TBD

map -> odom:
TBD
```

## 唯一性算法故障注入

- [x] 同名 node、不同 GID、相同 edge 的 fixture 被拒绝。
- [x] 不同 GID、不同 edge 的 fixture 被接受。
- [x] 测试失败信息包含 conflicting edge 与 GIDs，而不是只报 publisher
  count。

```text
Same-name duplicate-edge fixture:
Normal auditor exited 1:
tf_audit_parent -> tf_audit_child has 2 publisher GID(s); expected 1

Conflict sentinel exited 0 only after both publisher GIDs mapped back to
graph endpoints whose absolute FQN was /duplicate_tf_owner, and only after
the full 5.000-second observation window.

Disjoint-edge valid fixture:
Disjoint auditor exited 0 with:
tf_disjoint_parent_one -> tf_disjoint_child_one owners=1
tf_disjoint_parent_two -> tf_disjoint_child_two owners=1

The same /tf_static graph contained four publisher endpoints in total.

Dynamic-topic and owner mapping:
A periodic /tf writer mapped to /dynamic_tf_owner and passed its full
observation window. A second auditor expecting /wrong_dynamic_tf_owner exited
1 and logged topic=/tf endpoints={/dynamic_tf_owner}.

Why node-name deduplication would be wrong:
The two conflicting endpoints deliberately reused duplicate_tf_owner but had
different publisher GIDs and different transform values. Treating node name
as endpoint identity would collapse a real two-writer conflict into one.
```

## Bounded motion 后的稳定性

- [ ] 运动前记录完整 edge/GID owner set。
- [ ] 只发送可信限速范围内的 `TwistStamped`。
- [ ] 实验结束显式发送零。
- [ ] 运动后 owner set 不变。
- [ ] 运动后 scan-time transforms 仍成功。
- [ ] `/odom` 与旧 topic endpoint count 不变。
- [ ] launch cleanup 后没有 Gazebo/ROS process residue。

```text
Bounded command:
TBD

Owner set before:
TBD

Owner set after:
TBD

Post-motion scan-time TF:
TBD

Explicit zero:
TBD

Post-run process audit:
TBD
```

## 本地完整门禁

- [ ] `git diff --check` 通过。
- [ ] repository tests 通过且新断言真实执行。
- [ ] Xacro、URDF/SDF、world 与 bridge contracts 通过。
- [ ] 六个 package build 通过。
- [ ] package tests 零 error / failure。
- [ ] headless integration 没有被 skip。
- [ ] `bash scripts/verify.sh` 输出最终成功 marker。
- [ ] source tree 没有 `__pycache__` 或 `*.pyc`。
- [ ] 提交前逐项阅读完整 staged diff。

```text
Verification date/environment:
TBD

Command:
bash scripts/verify.sh

Exit status:
TBD

Repository test summary:
TBD

Build summary:
TBD

ROS/package test summary:
TBD

Integration metrics:
TBD

Final marker:
TBD
```

## 评审与远端证据

- [ ] Work Item 已关联 GitHub Issue。
- [ ] PR diff 只包含 VN-0009 范围。
- [ ] required hosted CI 通过。
- [ ] independent review 已完成。
- [ ] review conversations 全部解决。
- [ ] PR 以 rebase 方式合并。
- [ ] record 写入 local-to-public rebase identity map。
- [ ] annotated `course/0008-solution` 指向 public reviewed solution。
- [ ] start/solution tags 均未被重写。

```text
Issue:
https://github.com/Edddddddddy/voice_nav_robot_ws/issues/8

PR:
TBD

Required CI:
TBD

Independent review:
TBD

Merge method/time:
TBD

Public identity map:
TBD

Solution tag object and peeled target:
TBD
```

## 复盘

完成后用自己的话回答，不复制课件原文：

1. 为什么“`/tf` 有两个 publisher”不能直接证明 TF 冲突？
1. 为什么相同 node name 的两个 publisher 仍要按 GID 区分？
1. 为什么 `/tf_static` evidence 需要 transient-local-compatible QoS？
1. 为什么 scan-time transform 比 latest transform 更接近 SLAM 的真实
   消费方式？
1. direct remap 与 relay 在 owner、QoS、timestamp 和故障边界上有什么
   区别？
1. 360 samples 覆盖 `[-pi,+pi]` 时，如何找离零角最近的真实 beam？
1. 哪个负向 fixture 最早暴露了真实设计错误？为什么？
1. 如果把 obstacle collision 删除但保留 visual，哪些证据会失败？

```text
Learner reflection:
TBD
```
