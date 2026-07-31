# Lesson 0009 学习记录：独立 MotionGate

状态：Historical full gate GREEN / current-head focused checks GREEN /
exact-head full gate and final review pending
（教师参考实现）

学习者复现状态：Pending

本记录只填写已经发生且可查询的事实。PR 和 pre-remediation head 的 hosted CI
已经发生；`8e022580` 的完整本地门禁已经通过，当前 `517339a` 的 focused checks
已经通过。当前 head 的完整门禁、final reviewed head 的 exact-head hosted CI、
merge 与 solution tag 尚未发生，因此仍保持 Pending。

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
- Work Item/course contract commit：
  `1d5abfbcb454bc41c22de7e9ec98b4b64f2b6323`
- Tests-first commit：
  `4e1318b1f4ba5e3a0e176bc051ce3890eb55035e`
- GREEN implementation commit：
  `8e80dcc`
- Documentation/evidence commit：
  `3cdd815433c15ff91043827c568c185bc39fff51`
- Baseline PR-ready head：
  `c8b9eefed5729a9a2ab2a60a9d8302e697beeb70`
- CI-readiness tests-first commits：
  `b82cb736d358f1ce6374373efabc692a3426bd83`、
  `3ce547f90b6bed000512cda81c99db3233d145f1`、
  `e86a07ddae17f65c0cd040fbdda3e05420346bbc`
- CI-readiness implementation commit：
  `e984433c80d9f9a2afa81011c8d606ccf8a3c79e`
- Harness-teardown contract commit：
  `bf1f6ac9650222cecbdbd2e5777caf9f00748cca`
- Cross-package/review RED commit：
  `a5fb71e92560a0d9e7f5f5bd5ceb3794a8b1e5fd`
- Reviewed readiness-convergence code head：
  `d5392d30efcc975cd280f8af6e1d7a433184d0ff`
- Scoped-result tests-first/implementation commits：
  `caf5cd923cdd27f06e7ac7b7f8c1fbdfe25495f0`、
  `c5d88c24b8e2361bf1403e314fb04e7fd604629f`
- Build-boundary tests-first/implementation commits：
  `9d968ae236325a19deb0236749ea715f4c05c42f`、
  `8e022580a7add59a9c5d5a95973182322a0641c0`
- Clean-count verification code head：
  `8e022580a7add59a9c5d5a95973182322a0641c0`
- Scoped-evidence hardening head under review：
  `517339a3d313910a937fef973a9bdd635b457fc8`
- GitHub PR：
  [#12](https://github.com/Edddddddddy/voice_nav_robot_ws/pull/12)（Ready）
- Remote PR head（remediation push 前）：
  `c8b9eefed5729a9a2ab2a60a9d8302e697beeb70`
- Required exact-head CI on final reviewed head：Pending
- Public merge identity：Pending
- Solution tag object/peeled target：Pending

## Immutable start checkpoint

- [x] annotated start tag 存在于本地。
- [x] remote tag object 与本地一致。
- [x] local/remote peeled target 都是 reviewed Lesson 0008 closure。
- [ ] 学习者从 start tag 创建独立 `learn/0009` worktree。
- [ ] 学习者开始修改前 `git status --short` 无输出。

```text
Learner commands/output:
TBD by learner
```

## Tests-first RED 证据

- [x] valid Core/IDL/config/launch fixture 先通过。
- [x] repository product assertion 单独失败。
- [x] 失败原因是缺少 MotionGate product behavior，不是
  syntax/import/CMake/fixture/discovery 错误。
- [x] global CAS、250/150/20 ms、CLAMP/retire、OPEN queue barrier、
  publication serial barrier 与 16-byte GID 均有负向 fixture。
- [x] 实现修改发生在 tests-first commit 之后。

```text
Command:
python3 -m unittest discover -s tests -p "test_motion_gate_contract.py" -v

Exit status:
1 (expected RED)

Test count:
20 run; 19 passed; 1 expected repository-product failure

Decisive RED:
MotionGate contract failed: missing Lesson 0009 MotionGate artifact:
src/voice_nav_mission/srv/InternalMotionGateControl.srv
```

GitHub evidence：

- Gate-local GID architecture correction：
  https://github.com/Edddddddddy/voice_nav_robot_ws/issues/11#issuecomment-5140686616
- immutable tag/commit/RED evidence：
  https://github.com/Edddddddddy/voice_nav_robot_ws/issues/11#issuecomment-5140897655

## Private Interface 证据

- [x] `InternalMotionGateControl.srv` 与 `InternalMotionGateState.msg` 位于
  `voice_nav_mission`。
- [x] `voice_nav_interfaces` 没有新增 Gate private type。
- [x] node FQN 精确为 `/motion_gate_node`。
- [x] private endpoint 精确为 `/motion_gate/internal/control` 与
  `/motion_gate/internal/state`。
- [x] operation 精确为 PREPARE、OPEN、RENEW、INHIBIT。
- [x] request 使用 Gate instance 与 expected global `control_seq`，不携带
  caller `writer_gid` 或任意 candidate topic。
- [x] Gate 生成 lease ID 与 candidate topic。
- [x] request/Gate ID 语义为 exact-32 lowercase hex；PREPARE lease 为空，
  其余操作 lease 为 exact-32 lowercase hex。
- [x] State 的 `bound_writer_gid` 恰好 16 bytes；字符串和诊断字段有界。

```text
ros2 interface show voice_nav_mission/srv/InternalMotionGateControl:
request = operation + string<=36 request_id/gate_instance_id
          + expected_control_seq + string<=36 lease_id
response = bounded status/snapshot, string<=128 candidate_topic,
           uint8[16] bound_writer_gid, string<=160 detail

ros2 interface show voice_nav_mission/msg/InternalMotionGateState:
bounded state snapshot with string<=36 IDs, string<=128 topic,
uint8[16] bound_writer_gid, typed reason, publication sequences,
and string<=160 detail
```

## MotionGateCore manual-clock 证据

- [x] Core 是 package-internal `STATIC` target，无 ROS I/O、ROS time、
  sleep、filesystem 或 graph access。
- [x] header/library 不安装、不导出；唯一新增安装目标是
  `motion_gate_node`。
- [x] 初始状态是 INHIBITED，`selected_command()` 为 zero。
- [x] 真实 typed surface 为 `prepare/open/renew/inhibit`、
  `accept_candidate/tick/snapshot/selected_command`；`force_fault` 仅是
  Node Adapter 的 fail-closed ingress。
- [x] global CAS mismatch 不会打开、续期或关闭 authority。
- [x] stale instance/lease/control_seq 的旧 INHIBIT 不会关闭新 lease。
- [x] retired/expired lease 永不复活，Gate restart 使旧 tuple 失效。
- [x] 41 个 Core 用例覆盖 exact boundary、idempotence/collision、
  identity、clamp、invalid retirement 和 selected-zero。

```text
Manual-clock boundary:
authority: T+249 ms remains live; T+250 ms retires and selects zero
candidate: U+149 ms remains fresh; U+150 ms retires and selects zero

Current-tuple INHIBIT:
Core selects zero and retires the lease; Node publication barrier publishes
that zero before returning the acknowledgement.
```

## Trusted YAML、时间与限速证据

- [x] YAML 顶层参数根精确为 `motion_gate_node`。
- [x] endpoint/name constants 不可由 YAML 或 launch remap 改写。
- [x] authority/freshness/output 精确为 `250/150/20 ms`。
- [x] prepare 与 writer graph timeout 初值均为 `1000 ms`。
- [x] Gate linear clamp 为 `[-0.20, 0.40] m/s`。
- [x] Gate angular clamp 为 `[-1.20, 1.20] rad/s`。
- [x] Gate clamp 不宽于 controller limit。
- [x] controller `cmd_vel_timeout=0.35 s`，未启用 acceleration/deceleration
  limiter。
- [x] steady clock 驱动 lease/freshness/output；ROS time 只写最终 stamp。
- [x] `use_sim_time` 启动必须为 true，运行时修改被拒绝；发布时若参数或
  active ROS clock 不满足，Gate 锁存 `CONFIGURATION_INVALID`，选择 zero，
  stamp 置零。

```text
Resolved trusted configuration:
authority_lease_ms=250
candidate_freshness_ms=150
output_frequency_hz=50.0
prepare_timeout_ms=1000
writer_graph_timeout_ms=1000
linear_x=[-0.20, 0.40]
angular_z=[-1.20, 1.20]
controller_cmd_vel_timeout=0.35

Cross-config checker:
MotionGate contract passed
```

## CLAMP 与 invalid retirement 证据

- [x] 有限 `linear.x` 正/负超限分别 clamp 为 `0.40/-0.20`。
- [x] 有限 `angular.z` 正/负超限分别 clamp 为 `1.20/-1.20`。
- [x] NaN、Inf 或任一 unsupported axis 非零会 retire 当前 lease 并选择
  zero。
- [x] retirement 后同一 lease 的合法 sample/RENEW 不能恢复 motion。

```text
case                               | output/state
finite supported-axis over-limit   | exact trusted clamp / ARMED
NaN or Inf                         | zero / retired
linear.y/z or angular.x/y non-zero | zero / retired
sample or RENEW after retirement   | rejected / zero
```

## Per-lease topic、OPEN barrier 与 GID 证据

- [x] PREPARE 返回 Gate-generated unique lease/topic：
  `/voice_nav_internal/motion_gate/candidate/lease_<32-lowercase-hex>`。
- [x] PREPARE admission 先确认 retired writer 已退出，再创建 discard-only
  reader A。
- [x] OPEN 在访问 graph 前完成纯 Core 校验；拒绝路径不改 reader。
- [x] snapshot #1 后销毁 A；discard-only reader B 的 snapshot #2 必须保持
  同一完整 GID。
- [x] Core 以 zero 进入 ARMED 后销毁 B；accepting reader C 的 snapshot #3
  仍须保持同一 GID 和 healthy controller。
- [x] zero/two/changing writers 均 fail closed。
- [x] Gate-local graph GID 与 Gate-local `MessageInfo.publisher_gid` 比较完整
  16 bytes。
- [x] pre-OPEN、old-topic、post-INHIBIT、unbound-second-writer 与 mismatched
  GID sample 均不能产生 non-zero。
- [x] canonical launch 和 Gate 都锁定 `rmw_fastrtps_cpp`；不能证明 GID 关联
  时拒绝启动或 fail closed。

```text
Successful product observation:
gate_instance = b81a3caca08247de959ba5ca78f25e5d
lease         = b81a3caca08247de959ba5ca78f25e5c
topic         = /voice_nav_internal/motion_gate/candidate/
                lease_b81a3caca08247de959ba5ca78f25e5c
writer_gid    = 010f0c2655f4d6fa0000000000001a03
result        = one unique bound writer; OPEN applied

Fault injection:
queued pre-OPEN data, snapshot writer replacement, zero/two writers,
old topic, post-INHIBIT data and mismatched MessageInfo GID all remain zero.
```

GID 是一次运行内、同一 Gate context 的 endpoint identity，不是稳定配置值，
不写入 YAML，也不是跨进程 capability。package-private IDL 和 topic 名称只缩小
接口面，不构成 DDS 安全边界。

## Publication serial barrier 证据

- [x] service、candidate 与 timer callback 不直接向最终 endpoint publish。
- [x] Core decision、final/state publication 通过单线程 executor、
  mutually-exclusive callback group 与一个统一 serial path。
- [x] INHIBIT/expiry/invalid 的 zero 在 acknowledgement 前 publish。
- [x] barrier 后没有更早 queued non-zero 再次发布。
- [x] inhibited 时每 20 ms 持续发布 zero。

```text
Injected ordering:
moving command -> current INHIBIT/expiry/invalid -> queued old candidate

Observed invariant:
first safety decision selects and publishes zero; acknowledgement follows;
old queued callback cannot publish non-zero through the serial barrier.
```

## Product bringup 与最终 owner 证据

- [x] canonical launch 来自 `voice_nav_bringup/product_sim.launch.py`。
- [x] lower-level simulator、controller 与 Gate 由同一 composition 管理。
- [x] product launch 不启动 test authority/candidate fixture，不含
  `twist_mux` 或 final-command bypass，不 remap Gate 固定 endpoint。
- [x] Gate 默认 inhibited/zero。
- [x] `/diff_drive_controller/cmd_vel` 只有一个 publisher endpoint。
- [x] publisher FQN 为 `/motion_gate_node`，GID 在观察窗口保持稳定。
- [x] Gate final publisher 使用 `rclcpp::SystemDefaultsQoS()`，并验证与
  controller subscriber 的实际兼容性。
- [x] bridge 仍只承载 `/clock` 与 `/scan`。

```text
Canonical launch:
ros2 launch voice_nav_bringup product_sim.launch.py headless:=true

Final endpoint:
topic=/diff_drive_controller/cmd_vel
type=geometry_msgs/msg/TwistStamped
publisher_count=1
owner=/motion_gate_node
publisher_gid=010f0c2691f43e800000000000001503
active_rmw=rmw_fastrtps_cpp
initial_state=INHIBITED/zero
```

## 四层本地证据

### Static prerequisite

```text
Commands:
python3 scripts/check_motion_gate_contract.py --root .
python3 -m unittest discover -s tests -p "test_motion_gate_contract.py" -v

Environment:
WSL2 Ubuntu 24.04; repository source only

Exit status:
0

Count/skips/elapsed:
contract checker passed; 33 unittest cases, 0 skipped, 8.382 s

Decisive assertion:
private bounded IDL, internal STATIC/non-installed Core, exact endpoints,
trusted config, three-snapshot source binding, FastDDS lock and sole final
publisher all satisfy the repository contract.
```

### Layer 1 — Core

```text
Command:
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ctest --test-dir build/voice_nav_mission \
  --output-on-failure -R '^motion_gate_core_test$'

Environment:
ROS 2 Jazzy; pure manual-steady-clock GTest; no ROS graph or Gazebo

Exit status:
0

Count/skips/elapsed:
1 CTest target; 41 inner GTest cases; 0 skipped; 0.24 s CTest wall time

Decisive assertion:
exact 249/250 ms and 149/150 ms boundaries, global CAS, restart,
idempotence/collision, exact-32 identities, clamp/retirement and permanent
selected-zero behavior all pass.
```

### Layer 2 — Node without Gazebo

```text
Command:
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ctest --test-dir build/voice_nav_mission \
  --output-on-failure -R '^test_test_motion_gate_node.py$'

Environment:
rmw_fastrtps_cpp; ROS_DOMAIN_ID=91;
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; no Gazebo; no /clock

Exit status:
0

Count/skips/elapsed:
1 CTest target; 2 inner launch testcases; 0 skipped; 7.46 s CTest wall time

Decisive assertion:
private service/state, frozen ROS time with steady deadlines, A/B/C readers,
three same-GID snapshots, use_sim_time mutation rejection, stale requests,
zero-before-ack and clean node shutdown all pass.
```

### Layer 3 — headless product

```text
Command:
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ctest --test-dir build/voice_nav_bringup \
  --output-on-failure -R '^test_test_motion_gate_product.py$'

Environment:
rmw_fastrtps_cpp; ROS_DOMAIN_ID=92;
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;
GZ_PARTITION=voice_nav_l0009_product_test; headless Gazebo Harmonic

Exit status:
0

Count/skips/elapsed:
1 CTest target; 2 inner launch testcases; 0 skipped; 18.09 s CTest wall time

Decisive metrics:
bounded distance=0.097500 m
Gate/controller clamp=(0.400 m/s, 1.200 rad/s)
authority-expiry conservative measurement-start->Gate-zero upper bound
  =254.560 ms; controller after Gate=7.338 ms;
  stationarity after Gate=97.453 ms; hold=200.248 ms
candidate-expiry conservative measurement-start->Gate-zero upper bound
  =162.782 ms; controller after Gate=7.808 ms;
  stationarity after Gate=98.059 ms; hold=200.073 ms
INHIBIT acknowledgement=10.185 ms; Gate zero=0.920 ms;
  controller zero=9.048 ms; stationarity after Gate=118.259 ms;
  hold=219.798 ms
final owner/GID remained stable and all launch-managed processes exited cleanly
```

这里的两个 expiry `gate_zero_ms` 从测试测量窗口开始前计时，是保守上界，
不是精确的“last accepted event → first zero”采样。它们分别满足本课
`<=300 ms` 与 `<=200 ms` 的门限。

### Clean-prefix install boundary

```text
Command:
bash scripts/check_clean_motion_gate_install.sh

Environment:
fresh /tmp build/install/log bases; source /opt/ros/jazzy/setup.bash

Exit status:
0

Count/skips/elapsed:
1 package built; 1 Core CTest target passed; 0 skipped

Decisive marker:
Clean MotionGate install audit passed: Core private, node installed.

Install result:
motion_gate_core header/library/CMake export absent;
lib/voice_nav_mission/motion_gate_node present.
```

## Lesson 0010 deferred crash-stop audit

- [x] 本记录没有声称 authority process kill 已完成。
- [x] 本记录没有声称 candidate process kill 已完成。
- [x] 本记录没有声称 MotionGate kill/0.35 s consumer timeout 已完成。
- [x] 本记录没有声称 managed pause/token/first-resume zero 已完成。
- [x] 本记录没有声称 unmanaged pause 原地恢复安全。

```text
Deferred Work Item:
Lesson 0010 / VN-0011
```

## 本地完整门禁

- [x] `git diff --check` 与 repository static tests 通过。
- [x] private IDL、Core/node/config/launch contracts 通过。
- [x] 六个 package build 通过。
- [x] package tests 零 error/failure。
- [x] headless MotionGate integration 没有被 skip。
- [x] clean-prefix install boundary 通过。
- [x] `bash scripts/verify.sh` 输出最终成功 marker。
- [x] guarded residue audit 相对 baseline 无新增匹配进程。
- [x] 文档提交前逐项阅读完整 staged diff。

```text
Verification date/environment:
2026-07-31; WSL2 Ubuntu 24.04; ROS 2 Jazzy; Gazebo Harmonic;
rmw_fastrtps_cpp

Command:
bash scripts/verify.sh

Exit status:
0

Repository static:
121 tests passed

Build summary:
6 packages finished [27.6 s]

ROS/package test summary:
6 packages finished [3 min 8 s]
210 tests, 0 errors, 0 failures, 12 skipped

Clean install:
1 package built; Core test passed; private Core absent; node present

Final marker:
VoiceNav Robot verification passed.

Process baseline before full verification:
3631225 gz sim server
3631226 gz sim gui

Process audit after full verification:
3631225 gz sim server
3631226 gz sim gui

Delta:
no new matching process; user-owned PIDs were not signaled or modified
```

第一次全量运行的产品行为断言和指标全部通过，但一个 launch-managed Gazebo
进程在 teardown 的 SIGINT/SIGTERM 后没有按期退出，严格 exit-code 断言将其
SIGKILL 并令该次门禁失败。诊断没有发现 OOM、segfault 或残留；随后同一产品
测试连续 `--repeat until-fail:10` 全部通过，两个 `voice_nav_sim` Gazebo 测试
后紧接产品测试也通过，第二次完整门禁通过。没有放宽 exit-code 或清理断言；
该低频 WSL/Gazebo teardown flake 被保留为透明证据。

## CI readiness remediation 证据

### Hosted pre-remediation history

- [x] attempt 1 与 attempt 2 的 SHA 相同。
- [x] attempt 2 只是 rerun，没有代码修改。
- [x] 记录把 rerun GREEN 解释为 flake evidence，而不是 remediation evidence。

```text
Run:
https://github.com/Edddddddddy/voice_nav_robot_ws/actions/runs/30625857309

Head SHA:
c8b9eefed5729a9a2ab2a60a9d8302e697beeb70

Attempt 1:
2026-07-31T11:06:25Z; conclusion=failure
simulation: /controller_manager/list_controllers response timeout
product: candidate topic has no writer
aggregate: 136 tests, 0 errors, 4 failed records, 8 skipped
(4 records represented 2 failing launch cases)

Attempt 2:
2026-07-31T11:17:16Z; conclusion=success
same workflow + same SHA + no code change
aggregate: 136 tests, 0 errors, 0 failures, 8 skipped

Interpretation:
same-head rerun demonstrated readiness flakiness only
```

### Tests-first、修复与复审

- [x] 初始 readiness contract RED：7 failed、1 passed。
- [x] deep convergence helper 在实现前产生 6 errors。
- [x] simulation runtime isolation contract 在实现前失败。
- [x] 只重试精确的
  `REJECTED/WRITER_UNAVAILABLE/candidate topic has no writer`。
- [x] PREPARE 之前启动 1 秒 absolute steady deadline；重试期间 Gate 始终
  inhibited，request ID 每次更新。
- [x] `/controller_manager/list_controllers` response 单独使用 15 秒预算；
  通用 service 等待仍为 5 秒。
- [x] 第一次实现后的全量运行保留为失败证据：内部 6/6 在 0.33 秒通过，
  但外层 CTest teardown wrapper 在 5.01 秒 timeout；raw workspace
  aggregate 为 221 result records、0 errors、1 failure、12 skipped。后续
  审查证明其中混入 74 条 stale records，因此不把 221 当作本轮干净计数。
- [x] wrapper 预算改为 30 秒后 focused convergence 7/7 通过；未修改产品
  authority/candidate/controller deadline。
- [x] 复审先以测试暴露 mission/sim ROS domain 都为 91，以及固定 12 次尝试
  可能先于 absolute deadline 终止；随后分别改为 91/92/93，并删除正常路径
  的 attempt cap。
- [x] `d5392d30` readiness-convergence code head 的独立 code review 与
  safety review 均无剩余 finding。
- [x] evidence review 发现 `build/voice_nav_mission.stale-l0009` 使
  `d5392d30` 的 raw `222/0/0/12` 多计 74 records/4 skips；当时真实六包
  结果是 `148/0/0/8`。
- [x] 新契约先证明 scoped reporter、empty-result、path traversal、symlink、
  stale/renamed build directory、空 allowlist、排序诊断和三阶段边界均会失败。
- [x] 修复后 verify 从 `src` 获取完整包集合，并在 build 前、build 后且
  clear/test 前、test 后且最终 report 前共三次 fail closed；只清除和聚合
  本次精确包集合，不自动删除污染目录。

```text
Commit responsibility:
b82cb736  initial readiness RED and pure convergence fixture
3ce547f9  package-owned convergence policy tests
e86a07dd  isolated simulation runtime RED
e984433c  bounded fail-closed readiness implementation
bf1f6ac9  CTest teardown wrapper budget and regression guard
a5fb71e9  cross-package domain/deadline review RED
d5392d30  domain isolation and absolute-deadline correction
caf5cd92  scoped result aggregation RED
c5d88c24  exact-package clear/report implementation
9d968ae2  fail-closed build-boundary RED
8e022580  build-boundary and path-safety implementation

Production timing/configuration unchanged:
publish_rate_hz=50; authority_timeout_ms=250; candidate_timeout_ms=150;
prepare_timeout_ms=1000; diff_drive_controller cmd_vel_timeout=0.35 s
```

### `8e022580..517339a` scoped-evidence hardening

除 `f50aa3a` 只补充已实现边界的威胁模型外，每一项行为变化均先有失败契约，
再有最小实现：

```text
72abf22 / 27ff84a  path-escape rejection and private evidence sandbox
74a05b8 / 4958b7c run repository contracts only after ROS is available
ec13bb3 / 527a0b5 third build-boundary check after package tests
a1bd5b5 / bf0c1fe preserve colcon package-discovery failure
10ede88 + 5a16670 / 516c846 accept only colcon-compatible package names
afce577 / 19de986 source ROS setup with nounset disabled locally
f8c5a88 / 73fd263 reject missing CTest TAG/XML evidence
9abd0c7 / 4bbd92d reject malformed or unconsumed CTest evidence
108ddfa / ff44a87 anchor scoped result deletion to directory descriptors
34d9005 / 953a14a anchor result snapshots to opened source descriptors
761730a / c2954ca make snapshot descriptor ownership leak-free
5286b36 / b1bd863 close the complete directory manifest during staging
0b385e1 / fe7b844 carry package/file identities from parse into clear
4f1cab0 / cc908fb require every mandatory xUnit file to be consumed
f50aa3a             document the quiescent-build-tree threat model
7c2f9f3 / 93d968e  reject hidden symlink evidence with an explicit allowlist
e4c10c0 / 061d88c  bind allowed ament paths to exact targets and manifests
3dc4fa3 / 0b77d4a  reject symlinked source roots and normalize XML errors
6e04af2 / 517339a  resolve pivoted targets with strict kernel semantics
```

Current-head focused evidence recorded before the exact-head full gate:

```text
Head:
517339a3d313910a937fef973a9bdd635b457fc8

tests.test_scoped_test_results: 41 passed
repository static suite: 168 passed
read-only scoped package report: 148 tests, 0 errors, 0 failures, 8 skipped
Python compile check: passed
git diff --check: passed
scripts/verify.sh on this head: Pending
```

Evidence mutation threat model：

- 选定的 build tree 与锁定 source-package layout 在 clear/report 期间必须保持静止。
- 锚定目录 descriptor 将普通路径替换限制在已选 package 内，并检测常规并发变化；
  symlink 和 path traversal 逃逸继续被拒绝。
- `src` 与 `src/<package>` 必须是直接、非 symlink 目录。目录 symlink 默认
  fail closed；只放行锁定 ament 布局中两个明确的 path/expected-target 对。
  ament_python 源 manifest 还必须具有 `package` 根节点和匹配的 package name；
  根 `package.xml` 是唯一允许跳过的 XML symlink。
- 允许目标使用 `resolve(strict=True)` 的真实解析结果比较，避免中间 symlink 与
  `..` 的内核语义被词法 `abspath` 错误折叠。
- 工具不宣称抵御恶意同 UID 进程在最终 identity check 与按名称 unlink 之间的竞态，
  也不宣称识别 hardlink 或 bind mount 对 lexical ownership 的伪造。
- 这是显式接受的企业 CI 剩余风险，不应把该工具表述为对抗性文件系统安全边界。

### Scoped clean-count canonical verification

```text
Verified code head:
8e022580a7add59a9c5d5a95973182322a0641c0

Command:
bash scripts/verify.sh

Exit status:
0

Repository static:
140 tests passed

Build summary:
6 packages finished [32.8 s]

ROS/package test summary:
6 packages finished [3 min 39 s]
148 scoped result records, 0 errors, 0 failures, 8 skipped

Clean install:
1 package built; Core CTest passed
Clean MotionGate install audit passed: Core private, node installed.

Final marker:
VoiceNav Robot verification passed.

Selected product metrics (milliseconds unless stated):
bounded motion distance=0.097500 m; clamp linear=0.400, angular=1.200
authority expiry: Gate zero=259.730; controller after=3.830;
  stationary=104.071; hold=219.930
candidate expiry: Gate zero=163.004; controller after=5.814;
  stationary=95.828; hold=200.005
INHIBIT: ack=10.199; Gate zero=1.152; controller zero=3.896;
  stationary=112.921; hold=219.899
final command owner remained stable
```

Evidence contamination handling:

```text
Detected stale directory:
build/voice_nav_mission.stale-l0009

Raw d5392d30 aggregate:
222 records, 0 errors, 0 failures, 12 skipped

Decomposition:
148 current-package records / 8 skipped
+ 74 stale records / 4 skipped

Action:
directory was not deleted; after absolute-path validation it was moved to
C:\Users\lcy\AppData\Local\Temp\voice_nav_mission.stale-l0009-20260731

Regression behavior:
verify rejects any unexpected top-level build directory or direct symlink;
the scoped reporter rejects missing results and cannot traverse package paths
```

## 评审与远端证据

- [x] Work Item 已关联 GitHub Issue。
- [x] baseline 三次独立只读审查完成；P0/P1 均为零。
- [x] P2 clean-install 残留问题已以 fresh-prefix audit、规范 overlay 重建和
  regression contract 关闭。
- [x] readiness-convergence code head 的 code/safety re-review 完成且无
  finding。
- [x] evidence-review P2 已由 scoped reporting、三阶段 build-boundary guard 与
  干净 148-result 全量门禁关闭。
- [x] scoped-evidence implementation head `517339a` 的 code/safety re-review 完成。
- [ ] final documentation head 的 evidence re-review 完成。
- [x] PR diff 只包含 VN-0010 范围。
- [ ] required hosted CI 在 exact head 通过。
- [ ] review conversations 全部解决。
- [ ] PR 以 rebase 方式合并。
- [ ] record 写入 local-to-public identity map。
- [ ] annotated `course/0009-solution` 指向 public reviewed solution。
- [ ] start/solution tags 均未被重写。

```text
Issue:
https://github.com/Edddddddddy/voice_nav_robot_ws/issues/11

Independent review:
baseline code review: P0=0, P1=0; stale incremental install P2 closed
baseline safety review: P0=0, P1=0, P2=0; availability-only P3 recorded
baseline evidence review: P0=0, P1=0, P2=0; local closure YES
remediation code review on d5392d30: no findings; prior domain-collision P2
  and early-attempt-cap P3 closed
remediation safety review on d5392d30: P0=0, P1=0, P2=0, P3=0
evidence review: P2 stale result aggregation found; historical full-gate
  evidence on 8e022580 closes the original contamination finding
scoped-evidence code review on 517339a: no P0-P3 findings
scoped-evidence safety review on 517339a: no P0-P3 findings
remediation evidence review on final documentation head: Pending

PR:
https://github.com/Edddddddddy/voice_nav_robot_ws/pull/12
state=OPEN; draft=false; base=main; head=feat/vn-0010-l0009-motion-gate
creation head=fcf9af874df93438519e3d7472e5b3237fdd21f0
remote head before remediation push=c8b9eefed5729a9a2ab2a60a9d8302e697beeb70
local readiness-reviewed head=d5392d30efcc975cd280f8af6e1d7a433184d0ff
local clean-count verified head=8e022580a7add59a9c5d5a95973182322a0641c0
local scoped-evidence hardening head=517339a3d313910a937fef973a9bdd635b457fc8
remediation pushed=false

Hosted CI history on pre-remediation head:
run 30625857309 attempt 1: failure, head=c8b9eefed5729a9a2ab2a60a9d8302e697beeb70
run 30625857309 attempt 2: success rerun, same head, no code change

Required exact-head CI:
Pending on the final documentation/review head

Merge method/time:
Pending

Public identity map:
Pending

Solution tag object and peeled target:
Pending
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
