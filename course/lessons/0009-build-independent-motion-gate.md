# Lesson 0009：手写独立 MotionGate

本课只完成一个纵向切片：把最终速度裁决从测试代码移入独立
`motion_gate_node`，用 steady-clock authority/freshness、Gate 生成的
per-lease topic、16-byte publisher GID、可信限速和串行发布屏障，让产品
bringup 默认关闭且 fail-closed。

```text
test-only authority/candidate harness
  -> InternalMotionGateControl
  -> per-lease TwistStamped + bound GID
  -> motion_gate_node
  -> /diff_drive_controller/cmd_vel
  -> diff_drive_controller
  -> Gazebo
```

本课不实现 MissionRuntime、StopMission、Nav2、velocity smoother、
Collision Monitor 或进程崩溃验收。Lesson 0010 才 kill authority/Gate
进程并完成 controller crash-stop 与 managed pause。正常 lease 过期不是
Runtime crash；YAML 中已有 `cmd_vel_timeout` 也不是 Gate-death 证据。

先阅读：

- [VN-0010 Work Item](../../docs/work-items/0010-independent-motion-gate.md)
- [Safety and motion contract](../../docs/architecture/safety-and-motion-contract.md)
- [差速驱动与里程计契约](../reference/differential-drive-contract.md)
- [Testing strategy](../../docs/process/testing-strategy.md)

## 0. 从不可变 start checkpoint 开始

教师已从 Lesson 0008 reviewed closure 创建 annotated tag
`course/0009-start`：

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws

git fetch origin --tags
git show --no-patch --decorate course/0009-start
git rev-parse course/0009-start
git rev-parse "course/0009-start^{}"
git ls-remote origin \
  refs/tags/course/0009-start \
  "refs/tags/course/0009-start^{}"
```

当前不可变身份应为：

```text
tag object:
79150d9cac31b2ff28b75a9893afdd99b0870642

peeled target:
53c0a937ecc8c1d842c72f8542f19af661d620cf
```

从 tag 创建独立 worktree，不在 `main` 上练习：

```bash
git worktree add \
  ../voice_nav_robot_lesson_0009 \
  -b learn/0009 \
  course/0009-start

cd ../voice_nav_robot_lesson_0009
git status --short
```

最后一条命令必须无输出。`course/0009-solution` 只能在实现通过 reviewed
PR、exact-head hosted CI 并 rebase merge 后创建；现在不能猜它的 hash。

## 1. 先区分四种“停”

不要用一个 timeout 解释所有故障：

| 层 | 时钟 | 检测什么 | 本课是否完成 |
| --- | --- | --- | --- |
| Candidate freshness | Gate steady clock，`150 ms` | 当前 writer 是否仍产生候选速度 | 是 |
| Runtime authority | Gate steady clock，`250 ms` | 编排者是否仍明确授权 | 是；本课用 test harness |
| Consumer timeout | controller/simulation time，`0.35 s` | MotionGate 进程是否仍发布 | 只保留配置；kill 验收在 Lesson 0010 |
| Physical stationarity | odometry / simulation | 机器人是否真正静止 | 本课分开观察，不冒充 command zero |

Candidate 即使持续发布，也不能续 authority。反过来，authority 即使持续
RENEW，也不能让旧 candidate 保鲜。MotionGate 活着且 inhibited 时每
`20 ms` 持续发布零；MotionGate 自身死亡后没有这个 wall timer，因此必须由
Lesson 0010 单独证明 controller timeout。

## 2. 冻结 owner、Topic、QoS 与两种时钟

本课的产品 owner 表：

| Interface | Owner / writer | QoS |
| --- | --- | --- |
| `/motion_gate/internal/control` | trusted internal client → Gate service | service request/response |
| `/motion_gate/internal/state` | `/motion_gate_node` | RELIABLE, TRANSIENT_LOCAL, KEEP_LAST(1) |
| `/voice_nav_internal/motion_gate/candidate/lease_<id>` | 当前唯一 candidate writer | BEST_EFFORT, VOLATILE, KEEP_LAST(1) |
| `/diff_drive_controller/cmd_vel` | **仅 `/motion_gate_node`** | `rclcpp::SystemDefaultsQoS()`，匹配 controller |
| `/diff_drive_controller/cmd_vel_out` | `diff_drive_controller` | controller evidence |

速度数据继续使用 `geometry_msgs/msg/TwistStamped`。不要给 controller 命令
添加 Mission metadata，也不要加入 `twist_mux`。

两种时钟有不同职责：

- `std::chrono::steady_clock`：authority、freshness、writer
  disappearance/bind、cleanup 与测试 deadline；
- ROS simulation time：输出 `TwistStamped.header.stamp`、odom、TF 与
  Gazebo 物理。

Candidate 自带 stamp 不能证明 steady freshness。Gate 在通过 state、topic、
OPEN barrier 和 GID 检查后，记录本机 steady receipt time；最终输出由 Gate
重写 ROS stamp。

最终 publisher 不把 RELIABLE、history 或 depth 写死成课程事实。Jazzy
`diff_drive_controller` 使用 `rclcpp::SystemDefaultsQoS()`；Gate 跟随同一
profile，并用 actual endpoint compatibility checker 验证连通。若 DDS
introspection 把某项 policy 报成 `UNKNOWN`，checker 不能伪造一个固定值。

## 3. Tests first：先写能正确失败的契约

先运行当前基线并确认 `voice_nav_mission` 仍是空骨架：

```bash
rg -n \
  "motion_gate|InternalMotionGate|candidate_freshness|control_seq" \
  src scripts tests course docs

python3 -m unittest discover -s tests -p "test_*.py" -v
colcon list
```

先增加 checker、valid fixture 和负向 fixtures，再引用 repository product。
RED 必须证明“产品还没有实现”，不能来自 import、XML、CMake 或 fixture
错误。

静态/contract fixtures 至少覆盖：

- private IDL 被误放入 `voice_nav_interfaces`；
- operation 拼成 `ACTIVATE`、`CLOSE` 或其他未锁定名字；
- `control_seq` 不是 Gate-wide CAS；
- caller 可以提供 lease ID 或任意 candidate topic；
- GID 少于/多于 16 bytes；
- topic durability 变成 transient local，或 candidate queue 深度大于 1；
- final output 多出第二个 product publisher；
- Gate limit 比 controller 更宽；
- authority/freshness/output period 不是 `250/150/20 ms`；
- 使用 ROS time 计算 lease/freshness；
- bringup 启动 test harness、`twist_mux` 或第五个 resident process；
- Gate exit 立即关闭/respawn simulator，从而掩盖 Lesson 0010 consumer
  timeout。

先提交 RED：

```bash
git add \
  tests \
  docs/work-items/0010-independent-motion-gate.md \
  course/lessons/0009-build-independent-motion-gate.md \
  course/records/0009-independent-motion-gate.md \
  course/catalog.toml
git diff --cached
git commit -m "test(mission): define MotionGate authority contract"
```

## 4. 用 manual clock 写一个深的 MotionGateCore

Core 不创建 ROS Node，不读取 YAML，不调用 `now()`，也不 sleep。构造时接收
已经验证的 policy；每个 typed event 显式携带 steady time，返回下一状态与
需要执行的 effects。

建议把 Core 的测试面压缩成一个事件入口：

```text
Decision handle(Event event, SteadyTime now)
```

`Event` 可以是 C++ variant，但不要把 variant 暴露成公共 ROS Interface。
事件至少表达：

```text
Prepare(expected_control_seq)
Open(expected_control_seq, lease_id)
Renew(expected_control_seq, lease_id)
Inhibit(expected_control_seq, lease_id)
Candidate(lease_id, writer_gid, Twist)
Tick
```

Gate 才生成 lease ID 与 topic；测试 fixture 不能把任意路径塞进 Core。
这里的 candidate `writer_gid` 只代表 Node 在 **Gate 本进程** 从
graph/MessageInfo 观察后交给 Core 的 opaque identity，绝不是 control
request 从 caller 进程传来的 `Publisher::get_gid()`。

Core 不直接 publish。`Decision` 至少能告诉 Node Adapter：

- control response code 与新 `control_seq`；
- 是否创建/销毁 candidate reader；
- 是否等待/绑定 writer；
- 当前 selected command；
- 是否必须先 publish zero；
- 是否更新 state snapshot；
- lease 是否已永久 retired。

这条 Interface 必须让以下行为只在一个地方实现一次：

- 初始 inhibited；
- global CAS；
- PREPARE/OPEN/RENEW/INHIBIT 顺序；
- 250/150 ms deadline；
- finite supported-axis CLAMP；
- invalid candidate retirement；
- old lease 不可复活；
- publication serial ordering。

### 精确的 manual-clock 边界

不要用 `sleep_for` 猜时间。测试直接推进 manual clock：

```text
last accepted RENEW at T
T + 249 ms: authority live
T + 250 ms: lease retired, selected output zero

last accepted candidate at U
U + 149 ms: candidate fresh
U + 150 ms: lease retired, selected output zero
```

再证明：

- 过期之后发送同一个或更大 `control_seq` 的 RENEW 也不能复活旧 lease；
- candidate 持续到达但没有 RENEW，仍在 250 ms retire；
- RENEW 持续到达但 candidate 停止，仍在 150 ms retire；
- Gate restart 生成新 instance，旧 instance/lease/control_seq 全部无效；
- 旧 lease 的晚到 INHIBIT 不能关闭随后创建并 ARMED 的新 lease；
  current tuple 的 INHIBIT 必须先发布零再确认。

## 5. 定义 package-private typed seam

在 `voice_nav_mission` 内生成：

```text
srv/InternalMotionGateControl.srv
msg/InternalMotionGateState.msg
```

它们不进入 `voice_nav_interfaces`。所有字符串与 sequence 有界；Gate/lease
ID 使用固定长度表示；bound writer GID 恰好 16 bytes。

Control request 只允许以下 operation：

```text
PREPARE
OPEN
RENEW
INHIBIT
```

请求包含 Gate instance 与 expected global `control_seq`。除 PREPARE 外，
请求还引用 Gate 已返回的当前 lease；caller 永远不生成 lease 或 topic。
Request 中没有 `writer_gid`。跨进程传递 caller 的
`Publisher::get_gid()` 再比较，不属于本协议。

State 是最后一个 snapshot，不是 event log。至少记录：

- Gate instance、当前 global `control_seq`；
- state：INHIBITED、PREPARED、ARMED 或 FAULTED；
- Gate 生成的 lease/topic；
- 16-byte bound GID 是否存在；
- authority/freshness/writer validity；
- output sequence、`output_zero` 与 typed reason。

Package-private 表示不承诺外部兼容，不表示其他 DDS participant 无法调用。
本项目当前信任本机仿真组合，不把 topic name 当成安全凭据。

四个 operation 都校验 Gate instance 与 expected global `control_seq`；
OPEN、RENEW、INHIBIT 还必须匹配当前 lease。晚到的旧 INHIBIT 不能关闭较新的
lease，而是返回 typed mismatch 且不改变状态。未来 public `StopMission`
之所以仍无条件产生安全效果，是因为 Runtime 会先在线性化点接受 STOP，再用
当时的 current Gate tuple 发起 INHIBIT；不要把 public STOP 语义错误下沉为
“任意 private INHIBIT 都关 Gate”。

## 6. 实现 PREPARE / OPEN reader-queue barrier

完整的 Gate-side handover 顺序是：

```text
INHIBIT and publish zero
  -> retire old lease
  -> destroy old candidate reader
  -> confirm old writer GID disappeared
  -> PREPARE generates new lease/topic and a provisional discarding reader
  -> create exactly one test candidate writer
  -> Gate-local OPEN graph snapshot finds exactly one endpoint
  -> destroy the provisional reader and its queue
  -> recreate a VOLATILE depth-1 reader
  -> bind the Gate-observed endpoint's 16-byte GID
  -> cross the serialized OPEN barrier
  -> compare only Gate-local MessageInfo GIDs with that exact GID
```

PREPARED 状态收到的样本全部 discard。OPEN 不能只把一个 bool 改成 true：
reader queue 中可能已经有旧样本。Node 必须在 OPEN 串行点销毁并重建
volatile depth-1 reader，再完成 Core OPEN decision 与 publication；
OPEN 之前旧 reader 观察或排队的 sample 永远不能在 OPEN 后变成 non-zero。

锁定 Fast DDS/RMW 的 self-test 要证明：Gate 自己
`get_publishers_info_by_topic()` 观察的唯一 endpoint GID，与 Gate 自己收到
的 `MessageInfo.publisher_gid` 对同一 writer 能关联。若不一致或无法建立，
fail closed。State 可暴露固定 `uint8[16] bound_writer_gid` 作为 run-local
诊断；caller 的 GID 不参与协议。

故障注入：

- OPEN 时 zero writer；
- OPEN 时 two writers；
- graph endpoint GID 与 `MessageInfo.publisher_gid` 不一致；
- INHIBIT 后继续向旧 topic 发 non-zero；
- old writer 不退出就 PREPARE 新 lease；
- OPEN 后第二个未绑定 writer 在相同 topic 发 sample；
- OPEN 之前排队 non-zero，OPEN 后不再发任何新样本。

所有情况都必须保持最终 zero，而不是“挑一个看起来正确的 writer”。

## 7. 锁定 CLAMP 与 retirement 语义

可信 YAML 初值：

```yaml
motion_gate_node:
  ros__parameters:
    use_sim_time: true
    output_frequency_hz: 50.0
    authority_lease_ms: 250
    candidate_freshness_ms: 150
    prepare_timeout_ms: 1000
    writer_graph_timeout_ms: 1000
    candidate_qos_depth: 1
    expected_candidate_writer_fqn: /collision_monitor
    request_cache_size: 64
    linear_x_min: -0.20
    linear_x_max: 0.40
    angular_z_min: -1.20
    angular_z_max: 1.20
```

YAML 根名必须是 `motion_gate_node`。node FQN、control/state endpoint、
candidate prefix 与 final command endpoint 都是代码常量，不得做成 YAML
参数，也不得在 product launch 中 remap。

合法有限值的行为：

```text
linear.x = 2.0  -> publish 0.40
linear.x = -2.0 -> publish -0.20
angular.z = 2.0 -> publish 1.20
angular.z = -2.0 -> publish -1.20
```

这是 **CLAMP**，不是 reject。controller 保留相同的第二道 velocity limit。

下列输入不是 clamp，而是 retire 当前 lease：

- 任一 Twist 数值 NaN 或 Inf；
- `linear.y`、`linear.z`、`angular.x` 或 `angular.y` 非零；
- topic、lease 或 writer GID 不匹配；
- authority 或 candidate deadline 失效。

Retire 后先零，后续合法 sample/RENEW 也不能恢复；必须重新 PREPARE。

## 8. 用一个 publication serial barrier 消除晚到 non-zero

不要让 service callback、candidate callback 和 timer callback 分别直接
publish。它们都把 event 交给同一串行执行路径：

```text
observe event
  -> Core decision
  -> update selected command/state
  -> publish required final command
  -> publish state snapshot
  -> complete control response
```

INHIBIT、expiry 或 invalid retirement 的 response 只有在 transition-after
zero 已交给 final publisher 后返回。这个确认只表示 Gate 已禁止并 publish
zero，不表示 controller 已处理，也不表示 odometry 已静止。

测试专门制造：

```text
candidate callback decides non-zero
INHIBIT arrives
old non-zero publication is delayed
```

正确实现必须让 zero 成为 barrier 后的第一个/持续输出，旧 decision 不得在
zero 之后发布。

## 9. 建立 canonical product bringup

配置属于 `voice_nav_bringup`：

```text
src/voice_nav_bringup/config/motion_gate.yaml
src/voice_nav_bringup/launch/product_sim.launch.py
```

`product_sim.launch.py` include
`voice_nav_sim/launch/simulation.launch.py`，再启动 `motion_gate_node`；Gate
直接发布到固定 `/diff_drive_controller/cmd_vel`，launch 不 remap 这个
endpoint。普通产品启动不包含 authority/candidate fixture，因此机器人默认
静止。

运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch voice_nav_bringup product_sim.launch.py headless:=true
```

另一个终端只读检查：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic info -v /diff_drive_controller/cmd_vel
ros2 topic echo --once /motion_gate/internal/state
ros2 control list_controllers
```

Final command 必须只有一个 publisher endpoint，并映射到
`/motion_gate_node`。状态必须是 inhibited/zero。不要因为 Gate exit 立即
shutdown 或 respawn simulator；Lesson 0010 需要 controller/simulation
继续推进来测第二道 deadman。

## 10. Headless 最小纵向切片

authority/candidate harness 只能是 test fixture。它执行：

1. 读取 transient-local Gate state，取得 instance 与 `control_seq`；
2. PREPARE，取得 Gate 生成的 lease/topic；
3. 在返回 topic 创建唯一 writer；
4. OPEN，让 Gate 在自身 graph snapshot 绑定唯一 endpoint 的完整 GID；
5. 定期 RENEW，并发布有限或可 clamp 的 candidate；
6. 观察 Gate final output、controller limited output、odom；
7. INHIBIT；
8. 分开记录 Gate zero、controller zero、odom stationarity；
9. 销毁 writer，确认 old GID 从 graph 消失。

有效 forward candidate 应产生正 odometry delta。证据不是固定某一次精确位姿，
而是配置范围内的有界运动、随后 zero，以及在容差内保持至少 `200 ms` 的
stationarity。

### Authority expiry

保持 candidate writer 继续发送合法 non-zero，只停止 RENEW：

```text
last accepted renewal -> first Gate zero
required: <= 300 ms steady time
```

这证明 candidate 不能续 authority，但仍不是“kill Runtime”。

### Candidate freshness expiry

保持 RENEW，只停止 candidate：

```text
last accepted candidate -> first Gate zero
required: <= 200 ms steady time
```

### Clamp 与 invalid

每个 retirement case 使用新 lease：

- 四个上下限 clamp 值；
- NaN；
- Inf；
- 每个 unsupported axis 非零；
- OPEN 后未绑定的第二 writer；
- two writers；
- pre-OPEN queued sample；
- post-INHIBIT old-topic sample。

不要在同一个已 retire lease 上继续测试并误以为后续 case 被单独执行。

## 11. 记录三种不同的 zero

Headless evidence 至少有三列：

| 事实 | 来源 | 含义 |
| --- | --- | --- |
| Gate final output zero | `/diff_drive_controller/cmd_vel` sample/Gate state | Gate 已选择并发布零 |
| Controller output zero | `/diff_drive_controller/cmd_vel_out` | controller 已消费/限制为零 |
| Physical stationarity | `/odom` 连续窗口 | 模拟机器人已经静止 |

不要用一个时间戳替代另外两个。GID 也是 run-local 证据，记录完整 16 bytes
和 graph owner，但不要把它写入配置。

## 12. Lesson 0010 的 crash-stop 边界

到本课完成时，可以声明：

- Gate 与 controller 分进程；
- Gate 正常运行时 authority/freshness 独立 fail closed；
- per-lease topic/GID 和两个 barrier 拒绝晚到 candidate；
- controller timeout 仍配置为 `0.35 s`。

还不能声明：

- kill authority process 后的实际 expiry；
- kill candidate process 后的实际 freshness；
- kill Gate 后 controller 在 advancing simulation time 内归零；
- Gate crash 后物理停稳；
- moving safe-pause、zero-proof token、paused Gate death；
- first resumed wheel command zero；
- zero proof 失败后的 full restart；
- 无 token 的 GUI/Transport pause 可以原地恢复。

这些全部进入 Lesson 0010 / VN-0011。那一课使用独立 test authority process，
不是提前实现 MissionRuntime，也不新增第五个产品进程。

## 13. 常见故障按层定位

### Gate 一直 INHIBITED

依次检查 Gate instance、expected `control_seq`、PREPARE 返回的 lease/topic、
writer endpoint 数量、完整 GID、OPEN barrier 和 candidate freshness。不要
通过放宽 GID 检查制造绿灯。

### OPEN 偶发接受旧 command

这是 reader-queue barrier 缺失。仅换新 topic 不足以证明 OPEN 前已经进入
DDS/Executor queue 的 sample 被 discard。

### INHIBIT 后偶发再出现 non-zero

这是 publication serial barrier 缺失。检查是否有 callback 绕过 Core/serial
path 直接调用 publisher。

### 超限值导致 retire

重新阅读契约：有限的 `linear.x`/`angular.z` 超限应 CLAMP；只有非有限值、
unsupported non-zero axis 或身份/时效错误才 retire。

### lease 跟着 `/clock` pause

说明错误使用了 ROS clock。Core unit test 必须只推进 manual steady clock；
Node Adapter 使用 steady clock 计算 deadline，仅输出 stamp 使用 ROS time。

### 出现两个 final publisher

使用 `ros2 topic info -v` 查完整 endpoint/GID/FQN 和 actual QoS
compatibility。最终 endpoint 使用 SystemDefaults；introspection 为
`UNKNOWN` 的 policy 不能被硬断言成 RELIABLE 或某个 depth。产品 bringup
禁止 test fixture 直发 controller；历史低层 simulation tests 不等于产品
composition。

## 14. 完整门禁、自审与提交

先确保当前 lease 已 INHIBIT、final/controller output 为零、odom 已静止，再
退出 launch。

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_lesson_0009

git diff --check
bash scripts/verify.sh
git status --short
git diff
```

用避免自匹配的完整命令检查残留：

```bash
if pgrep -af \
  '[g]z sim|[g]z-sim|[g]zserver|[r]os2 launch .*voice_nav|[m]otion_gate_node|[c]ontroller_manager|[p]arameter_bridge|[r]obot_state_publisher'
then
  printf 'FAIL: launch-owned process residue detected\n'
  exit 1
fi
printf 'PASS: no matching launch-owned process remains\n'
```

按变更原因显式 staging，不使用 `git add .`：

```bash
git add \
  src/voice_nav_mission \
  src/voice_nav_bringup \
  scripts \
  tests
git diff --cached
git commit -m "feat(mission): add independent MotionGate authority"

git add \
  README.md \
  CHANGELOG.md \
  course \
  docs
git diff --cached
git commit -m "docs(course): teach independent MotionGate"
```

每次提交前完整阅读 staged diff。不要提交 build/install/log、bags、运行日志、
GID 配置、截图、`__pycache__` 或 `*.pyc`。

## 验收

- `MotionGateCore` 通过 manual-clock 与 event-table tests；
- private operation 精确为 PREPARE/OPEN/RENEW/INHIBIT；
- Gate 生成 lease/topic，global CAS `control_seq` 防 stale control；
- control request 不携 GID；Gate-local graph/MessageInfo 自测后绑定
  16-byte GID；
- candidate reader 通过 OPEN destroy/recreate queue barrier；
- finite supported-axis over-limit 被 clamp；
- NaN/Inf/unsupported non-zero axis retire；
- authority/freshness 分别按 250/150 ms steady deadline 失效；
- publication serial barrier 保证 zero 后无旧 non-zero；
- product bringup 默认 inhibited，Gate 是唯一 final publisher；
- headless 有 bounded motion、两种 expiry、command/controller/physical zero；
- full gate 与 process residue audit 通过；
- 文档没有声称 Lesson 0010 crash-stop/pause 已完成。

## 提交给教师

提交真实、可查询的内容：

1. `git status --short` 与 `git log --oneline --decorate -12`；
2. start tag object/peeled target；
3. tests-first RED 与 GREEN commit；
4. `ros2 interface show` 两个 private type；
5. trusted YAML、private topic QoS 与 final SystemDefaults compatibility；
6. PREPARE/OPEN/RENEW/INHIBIT state/`control_seq` 表；
7. per-lease topic、Gate-local 16-byte bound GID 与 graph owner；
8. unique final publisher evidence；
9. clamp、invalid retirement、pre-OPEN/old-writer injection；
10. authority/freshness last-event→Gate-zero steady latency；
11. Gate zero、controller zero、odom stationary 三个时间；
12. complete verify/test/build summary、final marker、residue audit；
13. Issue/PR/CI/review/rebase/tag 只在真实发生后填写。

## 复盘问题

1. 为什么 candidate 持续到达不能续 authority？
1. 为什么 Gate 生成 topic 比接受 caller 任意 topic 更容易封闭 Interface？
1. global CAS `control_seq` 防住了什么晚到 control request？
1. 为什么换 topic 后仍需要 OPEN reader-queue barrier？
1. 为什么 GID 必须比较完整 16 bytes，而不能只看 node name？
1. CLAMP 与 retire 分别适合哪些输入？为什么有限超限不属于 NaN/Inf？
1. publication serial barrier 如何阻止 zero 后的晚到 non-zero？
1. 为什么 Gate zero、controller zero、physical stationarity 是三种证据？
1. 为什么本课不能宣称 MotionGate-death crash-stop 已完成？

## 主要资料

- [ROS 2 Jazzy executors](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html)
- [ROS 2 Jazzy QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [diff_drive_controller Jazzy user documentation](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [ADR-0002: migrate to gz_ros2_control](../../docs/adr/0002-migrate-to-gz-ros2-control.md)
- [ADR-0003: one deep Mission Runtime](../../docs/adr/0003-use-one-deep-mission-runtime.md)
