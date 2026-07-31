# Lesson 0009 学习记录：独立 MotionGate

状态：Pending（教师参考实现）

学习者复现状态：Pending

本记录只填写已经发生且可查询的事实。当前只有 Work Item、GitHub Issue 和
immutable start tag 已建立；tests-first、实现、运行结果、PR、CI、review、
merge 与 solution tag 均保持 Pending。不要把课件中的期望值复制成输出。

## 变更身份

- Work Item：`VN-0010`
- GitHub Issue：
  [#11](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/11)
- 教师开发分支：`feat/vn-0010-l0009-motion-gate`
- 学习分支：`learn/0009`
- Start tag：`course/0009-start`
- Start tag object：
  `79150d9cac31b2ff28b75a9893afdd99b0870642`
- Start tag peeled target：
  `53c0a937ecc8c1d842c72f8542f19af661d620cf`
- Tests-first commit：TBD
- Green implementation commit：TBD
- Documentation/evidence commit：TBD
- GitHub PR：TBD
- Required exact-head CI：TBD
- Public merge identity：TBD
- Solution tag object/peeled target：TBD

## Immutable start checkpoint

- [x] annotated start tag 存在于本地。
- [x] remote tag object 与本地一致。
- [x] local/remote peeled target 都是 reviewed Lesson 0008 closure。
- [ ] 学习者从 start tag 创建独立 `learn/0009` worktree。
- [ ] 开始修改前 `git status --short` 无输出。

```text
Commands and output:
TBD by learner
```

## Tests-first RED 证据

- [ ] valid Core/IDL/config/launch fixture 先通过。
- [ ] repository product assertion 单独失败。
- [ ] 失败原因是缺少 MotionGate product behavior，不是
  syntax/import/CMake/fixture/discovery 错误。
- [ ] global CAS、250/150/20 ms、CLAMP/retire、OPEN queue barrier、
  publication serial barrier 与 16-byte GID 均有负向 fixture。
- [ ] 实现修改发生在 tests-first commit 之后。

```text
Command:
TBD

Exit status:
TBD

Test count:
TBD

Decisive RED:
TBD
```

## Private Interface 证据

- [ ] `InternalMotionGateControl.srv` 位于 `voice_nav_mission`。
- [ ] `InternalMotionGateState.msg` 位于 `voice_nav_mission`。
- [ ] `voice_nav_interfaces` 没有新增 Gate private type。
- [ ] node FQN 精确为 `/motion_gate_node`。
- [ ] private endpoint 精确为 `/motion_gate/internal/control` 与
  `/motion_gate/internal/state`。
- [ ] operation 精确为 PREPARE、OPEN、RENEW、INHIBIT。
- [ ] control request 使用 Gate instance 与 expected global
  `control_seq`。
- [ ] Gate 生成 lease ID 与 candidate topic；caller 不能传任意 topic。
- [ ] control request 不包含 caller `writer_gid`。
- [ ] State 的 `bound_writer_gid` 恰好 16 bytes，且只作为本次运行诊断。
- [ ] 字符串、数组与诊断字段有界。

```text
ros2 interface show voice_nav_mission/srv/InternalMotionGateControl:
TBD

ros2 interface show voice_nav_mission/msg/InternalMotionGateState:
TBD
```

## MotionGateCore manual-clock 证据

- [ ] Core 无 ROS I/O、ROS time、sleep 或 graph access。
- [ ] 初始状态 inhibited 且 selected output zero。
- [ ] PREPARE/OPEN/RENEW/INHIBIT 只通过 typed event Interface。
- [ ] 状态名称精确为 INHIBITED、PREPARED、ARMED、FAULTED。
- [ ] global CAS mismatch 不会打开、续期或关闭 authority。
- [ ] stale instance/lease/control_seq 的旧 INHIBIT 不会关闭新 lease。
- [ ] current tuple 的 INHIBIT 先发布零，再返回 acknowledgement。
- [ ] Gate restart 使旧 instance/lease/control_seq 失效。
- [ ] retired/expired lease 永不复活。

```text
Manual-clock transition table:

event | steady time | expected seq | resulting seq | state | selected output
TBD

Authority boundary:
T + 249 ms:
T + 250 ms:

Candidate boundary:
U + 149 ms:
U + 150 ms:
```

## Trusted YAML 与限速证据

- [ ] YAML 顶层参数根精确为 `motion_gate_node`。
- [ ] node/control/state/candidate-prefix/final-command 名称不是 YAML 参数。
- [ ] authority lease 精确为 `250 ms`。
- [ ] candidate freshness 精确为 `150 ms`。
- [ ] output period 精确为 `20 ms`。
- [ ] writer graph deadline 有界且初值为 `1 s`。
- [ ] Gate linear clamp 为 `[-0.20, 0.40] m/s`。
- [ ] Gate angular clamp 为 `[-1.20, 1.20] rad/s`。
- [ ] Gate clamp 不宽于 controller limit。
- [ ] controller `cmd_vel_timeout` 仍为 `0.35 s`，且 acceleration/deceleration
  limiter 未启用。

```text
Resolved trusted configuration:
TBD

Cross-config checker:
TBD
```

## CLAMP 与 invalid retirement 证据

- [ ] 有限 `linear.x > 0.40` 输出 `0.40`。
- [ ] 有限 `linear.x < -0.20` 输出 `-0.20`。
- [ ] 有限 `angular.z > 1.20` 输出 `1.20`。
- [ ] 有限 `angular.z < -1.20` 输出 `-1.20`。
- [ ] NaN/Inf retire 当前 lease 并选择零。
- [ ] `linear.y`、`linear.z`、`angular.x`、`angular.y` 任一非零 retire。
- [ ] retirement 后同一 lease 的合法 sample/RENEW 不能恢复 motion。

```text
case | input | output/state | result
TBD
```

## Per-lease topic、OPEN barrier 与 GID 证据

- [ ] PREPARE 返回 Gate 生成且每个 lease 不同、位于
  `/voice_nav_internal/motion_gate/candidate/lease_` 前缀下的 bounded topic。
- [ ] PREPARE 可创建 provisional reader 发现 graph，但所有 candidate 均
  discard。
- [ ] OPEN 前 queued sample 不会在 OPEN 后产生 non-zero。
- [ ] OPEN 前销毁 provisional reader/queue，再创建
  `VOLATILE + KEEP_LAST(1)` reader。
- [ ] OPEN 只在 Gate 本进程 graph 快照恰好发现一个 publisher endpoint
  时成功。
- [ ] zero/two writers 均保持 inhibited。
- [ ] Gate-local graph endpoint GID 与 Gate-local
  `MessageInfo.publisher_gid` 比较完整 16 bytes。
- [ ] request 没有 caller `Publisher::get_gid()`；没有跨进程 GID 断言。
- [ ] unbound second writer、Gate-local GID mismatch、old topic 与
  post-INHIBIT sample 被拒绝。
- [ ] old writer 未从 graph 消失时不能打开下一 lease。
- [ ] locked `rmw_fastrtps_cpp` 自检证明 Gate-local graph endpoint GID 与
  Gate-local `MessageInfo` GID 可关联；不能关联时 fail closed。

```text
lease | candidate topic | bound 16-byte GID | graph owner | result
TBD

OPEN reader-queue fault injection:
TBD

Old writer/topic fault injection:
TBD
```

GID 是一次运行内、同一 Gate context 的 endpoint identity，不是稳定配置值，
不写入 YAML，也不是跨进程 capability。package-private IDL 和 topic 名称只缩小
接口面，不构成 DDS 安全边界。

## Publication serial barrier 证据

- [ ] service、candidate 与 timer callback 不直接 publish。
- [ ] Core decision 与 final/state publication 通过一个 serial path。
- [ ] INHIBIT/expiry/invalid 的 zero 在 acknowledgement 前 publish。
- [ ] barrier 后没有更早的 queued non-zero 再次发布。
- [ ] inhibited 时每 20 ms 持续发布零。

```text
Injected ordering:
TBD

Observed publication sequence:
TBD
```

## Product bringup 与最终 owner 证据

- [ ] canonical launch 来自 `voice_nav_bringup`。
- [ ] lower-level simulator、controller 与 Gate 由同一 launch composition
  管理。
- [ ] product launch 不启动 test authority/candidate fixture。
- [ ] product launch 不包含 direct controller command bypass 或
  `twist_mux`。
- [ ] product launch 不 remap Gate 的固定 control/state/candidate/final
  endpoint。
- [ ] Gate 默认 inhibited/zero。
- [ ] `/diff_drive_controller/cmd_vel` 只有一个 publisher endpoint。
- [ ] publisher GID 映射到精确 FQN `/motion_gate_node`，并在观察窗口
  保持稳定。
- [ ] Gate final publisher 使用 `rclcpp::SystemDefaultsQoS()`。
- [ ] runtime checker 证明 Gate final publisher 与 controller subscriber
  实际兼容；introspection 为 `UNKNOWN` 的 reliability/history/depth 不被
  硬断言成固定值。
- [ ] bridge 仍只承载 `/clock` 与 `/scan`。

```text
Canonical launch command:
TBD

Final endpoint evidence:
topic | type | QoS | publisher GID | owner FQN | count
TBD

Initial Gate state:
TBD
```

## Headless bounded motion 证据

- [ ] test harness 读取 Gate instance/control_seq。
- [ ] PREPARE 取得 Gate 生成 lease/topic。
- [ ] 单 writer OPEN 成功，并由 Gate 记录本地观察到的 bound GID。
- [ ] RENEW 与 fresh candidate 产生受限 forward motion。
- [ ] `INHIBIT` acknowledgement 只在 Gate zero publication 后返回。
- [ ] controller limited output 随后归零。
- [ ] `/odom` 进入 stationarity tolerance 并保持至少 `200 ms`。
- [ ] Gate zero、controller zero 与 physical stationarity 分别计时。

```text
Bounded command:
TBD

Odometry before/after:
TBD

Gate zero time:
TBD

Controller zero time:
TBD

Physical stationarity window:
TBD
```

## Authority 与 freshness expiry 证据

- [ ] candidate 持续发布时停止 RENEW，Gate 仍 retire。
- [ ] last accepted renewal 到 first Gate zero 不超过 `300 ms` steady time。
- [ ] RENEW 持续时停止 candidate，Gate 仍 retire。
- [ ] last accepted candidate 到 first Gate zero 不超过 `200 ms` steady
  time。
- [ ] 两项测试使用不同的新 lease，未复用已 retired lease。

```text
Authority-expiry run:
last accepted renewal:
first Gate zero:
steady delta:
result:
TBD

Candidate-expiry run:
last accepted candidate:
first Gate zero:
steady delta:
result:
TBD
```

这些证据只证明正常运行 Gate 的独立 deadline，不写成 Runtime/candidate
process crash。

## Lesson 0010 deferred crash-stop audit

- [ ] 本记录没有声称 authority process kill 已完成。
- [ ] 本记录没有声称 candidate process kill 已完成。
- [ ] 本记录没有声称 MotionGate kill/0.35 s consumer timeout 已完成。
- [ ] 本记录没有声称 managed pause/token/first-resume zero 已完成。
- [ ] 本记录没有声称 unmanaged pause 原地恢复安全。

```text
Deferred Work Item:
Lesson 0010 / VN-0011
```

## 本地完整门禁

- [ ] `git diff --check` 通过。
- [ ] repository tests 与新断言通过。
- [ ] private IDL 生成通过。
- [ ] MotionGate Core/node/config/launch contracts 通过。
- [ ] 六个 package build 通过。
- [ ] package tests 零 error/failure。
- [ ] headless MotionGate integration 没有被 skip。
- [ ] `bash scripts/verify.sh` 输出最终成功 marker。
- [ ] guarded process-residue audit 无匹配进程。
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

Headless metrics:
TBD

Final marker:
TBD

Process residue audit:
TBD
```

## 评审与远端证据

- [x] Work Item 已关联 GitHub Issue。
- [ ] PR diff 只包含 VN-0010 范围。
- [ ] required hosted CI 在 exact head 通过。
- [ ] independent review 完成且发现项均有 tests-first 修复证据。
- [ ] review conversations 全部解决。
- [ ] PR 以 rebase 方式合并。
- [ ] record 写入 local-to-public identity map。
- [ ] annotated `course/0009-solution` 指向 public reviewed solution。
- [ ] start/solution tags 均未被重写。

```text
Issue:
https://github.com/Edddddddddy/voice_nav_robot_ws/issues/11

PR:
TBD

Required exact-head CI:
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

1. 为什么 candidate 持续发布不能续 authority？
1. 为什么 Gate 生成 lease/topic，而不是信任 caller 提供路径？
1. global CAS `control_seq` 解决了什么晚到控制问题？
1. 为什么 per-lease topic 之外仍需 OPEN reader-queue barrier？
1. 为什么 writer identity 必须比较完整 16-byte GID？
1. 有限超限为何 CLAMP，NaN/Inf/unsupported axis 为何 retire？
1. publication serial barrier 如何保证 zero 后没有旧 non-zero？
1. Gate zero、controller zero 与 physical stationarity 有什么区别？
1. 为什么 Lesson 0009 尚未完成 crash-stop？

```text
Learner reflection:
TBD
```
