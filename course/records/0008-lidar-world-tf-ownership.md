# Lesson 0008 学习记录：LiDAR、world 与 TF 唯一所有权

状态：Pending

本记录只填写已经发生且可复查的证据。教师侧 reference implementation
已经完成 tests-first、本地实现与完整本地门禁；学习者复现、PR、CI、review、
merge 与 solution tag 仍保持 Pending。下列数值来自 2026-07-31 的最终本地
运行，不是课程中的期望输出。

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
- Tests-first contract commit：`40608da`
- Green implementation commit：`b246340`
- Review-fix commit：`0f5fdfa`
- Documentation/evidence commit：`bc9e636`
- GitHub PR：
  [#9](https://github.com/Edddddddddy/voice_nav_robot_ws/pull/9)
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
- [x] 实现修改发生在 tests-first commit 之后。

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

- [x] `voice_nav_test_world.sdf` 从 package share 加载。
- [x] installed world 与 source contract 一致。
- [x] world 不含 Fuel、HTTP(S) 或依赖本机 cache 的 URI。
- [x] Physics、UserCommands、SceneBroadcaster、Sensors systems 全部存在。
- [x] headless path 使用 `--headless-rendering`。
- [x] ground 与固定 obstacle 都有 collision。
- [x] obstacle center 为 `(2.0, 0.0, 0.5)` m。
- [x] obstacle size 为 `(0.5, 1.0, 1.0)` m。

```text
Installed world path:
install/voice_nav_sim/share/voice_nav_sim/worlds/voice_nav_test_world.sdf

World validation command and exit status:
python3 scripts/check_simulation_contract.py \
  --launch src/voice_nav_sim/launch/simulation.launch.py \
  --world src/voice_nav_sim/worlds/voice_nav_test_world.sdf \
  --robot-description \
    src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro \
  --bridge src/voice_nav_sim/config/bridge.yaml \
  --package src/voice_nav_sim/package.xml \
  --cmake src/voice_nav_sim/CMakeLists.txt
exit status: 0
gz sdf -k src/voice_nav_sim/worlds/voice_nav_test_world.sdf
exit status: 0

Required systems:
Physics, UserCommands, SceneBroadcaster, Sensors(render_engine=ogre2)

Obstacle collision pose/size:
center=(2.0, 0.0, 0.5) m; size=(0.5, 1.0, 1.0) m

Network-dependency audit:
PASS: no Fuel, HTTP(S), or machine-cache resource URI
```

## LiDAR 与 bridge 证据

- [x] Gazebo `/scan` 持续发布。
- [x] ROS `/scan` 类型为 `sensor_msgs/msg/LaserScan`。
- [x] `header.frame_id` 精确为 `laser_link`。
- [x] update rate、360 samples、360° angle、single layer 与 range contract
  一致。
- [x] sensor 没有 noise。
- [x] ROS scan stamps 使用仿真时间且至少三条严格递增。
- [x] `/scan` publisher 使用 sensor-data QoS。
- [x] bridge allowlist 只有 `/clock` 与 `/scan`。
- [x] 两项 direction 均为 `GZ_TO_ROS`。
- [x] bridge 没有 wall-time timestamp override。
- [x] bridge 不承载 cmd、joint state、odom、`/tf` 或 `/tf_static`。

```text
Gazebo /scan topic/type:
/scan / gz.msgs.LaserScan

ROS /scan type:
sensor_msgs/msg/LaserScan; exactly one publisher: /simulation_bridge

ROS /scan endpoint QoS:
BEST_EFFORT + VOLATILE (SENSOR_DATA)

Frame and scan geometry:
laser_link; 10 Hz; 360 beams; [-pi,+pi]; one vertical layer;
range=[0.05,8.0] m; resolution=0.01 m; no noise

Three increasing scan stamps:
4.2 s, 4.3 s, 4.4 s

Complete bridge.yaml:
/clock: gz.msgs.Clock -> rosgraph_msgs/msg/Clock, GZ_TO_ROS, CLOCK
/scan: gz.msgs.LaserScan -> sensor_msgs/msg/LaserScan,
       GZ_TO_ROS, SENSOR_DATA
```

## 解析 beam 证据

- [x] 从消息几何计算 `theta_i`，没有硬编码 center index。
- [x] 选择 `i* = argmin(abs(theta_i))`。
- [x] 根据 box front face 与实际 beam angle 计算 expected range。
- [x] ray/plane 交点位于 box 正面 y 范围内。
- [x] observed range 在由 range resolution 与仿真数值误差组成的窄容差内。

```text
Laser world x:
0.100 m

Box front-face x:
1.75 m

Selected index:
180

Selected beam angle:
0.008751 rad

Expected analytic range:
1.650 m

Observed range:
1.650 m

Allowed tolerance:
0.020 m

Absolute error:
0.000 m at the recorded precision
```

## Product /odom 证据

- [x] controller 使用 `--controller-ros-args` 直接 remap
  `~/odom:=/odom`。
- [x] launch 中不存在 odom relay/republisher。
- [x] `/odom` 有且只有一个 publisher endpoint。
- [x] endpoint GID 映射到 `diff_drive_controller`。
- [x] `/diff_drive_controller/odom` 有零个 publisher endpoint。
- [x] `/odom` frame 为 `odom`，child frame 为 `base_footprint`。

```text
Controller remap fragment:
--controller-ros-args '--ros-args --remap ~/odom:=/odom'

/odom verbose publisher endpoint:
count=1; owner=/diff_drive_controller; type=nav_msgs/msg/Odometry

/diff_drive_controller/odom publisher endpoint count:
0

/odom frame sample:
header.frame_id=odom; child_frame_id=base_footprint
```

## Scan-time TF 与 matched pose 证据

- [x] 至少三条 scan 都在自己的 `header.stamp` 成功查询
  `odom -> laser_link`。
- [x] 没有用 latest transform 替代 timestamped lookup。
- [x] `/odom` pose 与 `odom -> base_footprint` TF 在同一 odom stamp 比较。
- [x] translation 和 yaw 误差在数值容差内。

```text
Scan stamp 1 / transform result:
4.2 s / odom -> laser_link exact-time lookup PASS

Scan stamp 2 / transform result:
4.3 s / odom -> laser_link exact-time lookup PASS

Scan stamp 3 / transform result:
4.4 s / odom -> laser_link exact-time lookup PASS

Matched odom stamp:
4.529 s

Odometry pose:
x=0.000 m, y=-0.000 m, yaw=-0.000 rad

TF pose:
x=0.000 m, y=-0.000 m, yaw=-0.000 rad

Translation error:
<= 1e-5 m

Yaw error:
<= 1e-5 rad
```

## TF edge / publisher GID ownership 证据

- [x] `/tf` 与 `/tf_static` callback 都记录
  `MessageInfo.publisher_gid`。
- [x] graph endpoint GID 与 observed message GID 成功关联。
- [x] 每个 expected edge 恰好有一个 publisher GID。
- [x] 每个 GID 映射到预期 node owner。
- [x] frame 名精确且无前导 `/`。
- [x] `map -> odom` 不存在。
- [x] `/tf_static` 使用 transient-local-compatible 订阅 QoS。

```text
Runtime ownership table:

topic | parent -> child | publisher GID | graph owner | result
/tf_static | base_footprint -> base_link | run-local | /robot_state_publisher | PASS
/tf_static | base_link -> caster_link | run-local | /robot_state_publisher | PASS
/tf_static | base_link -> laser_link | run-local | /robot_state_publisher | PASS
/tf | base_link -> left_wheel | run-local | /robot_state_publisher | PASS
/tf | base_link -> right_wheel | run-local | /robot_state_publisher | PASS
/tf | odom -> base_footprint | run-local | /diff_drive_controller | PASS

The auditor observed the complete 35.000-second window and required a final
3.000-second stable interval. Publisher GIDs are DDS run-local identities and
are intentionally not copied into this durable record.

Unknown/unmapped GIDs:
0

map -> odom:
absent; reject-undeclared audit PASS
```

The review-fix extracted endpoint-identity decisions into a pure evaluator.
Its eight deterministic GTests cover absent graph data, both RMW unknown
placeholders, an expected owner, expected plus unresolved data, wrong plus
unresolved data, a fully resolved wrong owner, and the critical case where an
expected owner must not hide a second resolved wrong owner. The integration
test also lower-bounds its post-motion odometry/TF lookup by the final
odometry stamp and validates odometry, legacy odometry, and scan endpoints
from one decisive graph snapshot.

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
1 and logged:
GID <run-local> on /tf maps to {/dynamic_tf_owner};
expected /wrong_dynamic_tf_owner.

Why node-name deduplication would be wrong:
The two conflicting endpoints deliberately reused duplicate_tf_owner but had
different publisher GIDs and different transform values. Treating node name
as endpoint identity would collapse a real two-writer conflict into one.
```

## Bounded motion 后的稳定性

- [x] 运动前记录完整 edge/GID owner set。
- [x] 只发送可信限速范围内的 `TwistStamped`。
- [x] 实验结束显式发送零。
- [x] 运动后 owner set 不变。
- [x] 运动后 scan-time transforms 仍成功。
- [x] `/odom` 与旧 topic endpoint count 不变。
- [x] launch cleanup 后没有 Gazebo/ROS process residue。

```text
Bounded command:
linear.x=0.12 m/s for 0.80 s, then explicit zero for 0.30 s;
measured travel=0.096 m

Owner set before:
six expected TF edges; /odom owner=/diff_drive_controller;
legacy odom publishers=0

Owner set after:
unchanged

Post-motion scan-time TF:
5.5 s, 5.6 s, 5.7 s; all exact-time odom -> laser_link lookups PASS

Explicit zero:
published and final odometry linear/angular velocity entered zero tolerance

Post-run process audit:
printf 'PROCESS_RESIDUE_AUDIT_BEGIN\n'
if pgrep -af '[g]z sim|[g]z-sim|[g]zserver|[r]os2 launch .*voice_nav_sim|[l]aunch_testing\.launch_test.*voice_nav_sim|[p]arameter_bridge|[r]obot_state_publisher|[t]f_ownership_auditor|[c]ontroller_manager|[r]os_gz_sim.*create|[s]pawner.*(joint_state_broadcaster|diff_drive_controller)'
then
  printf 'FAIL: launch-owned process residue detected\n'
  exit 1
fi
printf 'PASS: no matching launch-owned process remains\n'
printf 'PROCESS_RESIDUE_AUDIT_END\n'

Output appended to /tmp/vn0009-final-verify.log:
PROCESS_RESIDUE_AUDIT_BEGIN
PASS: no matching launch-owned process remains
PROCESS_RESIDUE_AUDIT_END
```

## 本地完整门禁

- [x] `git diff --check` 通过。
- [x] repository tests 通过且新断言真实执行。
- [x] Xacro、URDF/SDF、world 与 bridge contracts 通过。
- [x] 六个 package build 通过。
- [x] package tests 零 error / failure。
- [x] headless integration 没有被 skip。
- [x] `bash scripts/verify.sh` 输出最终成功 marker。
- [x] Git index 未跟踪 `__pycache__` 或 `*.pyc`，安装规则也排除它们。
- [ ] 提交前逐项阅读完整 staged diff。

```text
Verification date/environment:
2026-07-31; WSL2 Ubuntu 24.04; ROS 2 Jazzy; Gazebo Harmonic

Command:
bash scripts/verify.sh

Exit status:
0

Repository test summary:
80 tests; 80 passed

Build summary:
Summary: 6 packages finished

ROS/package test summary:
Summary: 55 tests, 0 errors, 0 failures, 4 skipped

Integration metrics:
/scan owner/QoS, exact-time TF, analytic beam, direct odom, full-window GID
ownership and bounded-motion assertions all executed and passed

Final marker:
VoiceNav Robot verification passed.
```

## 评审与远端证据

- [x] Work Item 已关联 GitHub Issue。
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
https://github.com/Edddddddddy/voice_nav_robot_ws/pull/9

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
