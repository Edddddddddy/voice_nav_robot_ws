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

教师分支当前是 local-GREEN implementation，完整本地门禁与独立证据评审已经
通过。required CI、rebase merge 与 `course/0009-solution` 未闭环前，本课
不得标记 completed。

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

`use_sim_time` 是产品安全不变量，不是可热切换的普通参数。Gate 启动时必须
看到 `true`，随后用 on-set parameter callback 拒绝任何运行期修改；被拒的
`false` 不能改变当前值，也不能在运动中制造 zero pulse。每次最终发布还要在
同一 serial barrier 内再次确认参数仍为 `true` 且 ROS clock 已激活。任一
检查失败都通过 Adapter-only `force_fault()` 锁存
`ConfigurationInvalid`，把 command 替换为 zero，并把 stamp 置零；绝不能
把 system-time-stamped non-zero 送给 simulation-time controller。没有
`/clock` publisher 时 ROS time 可以冻结在零，但 steady deadline 和 20 ms
输出仍要前进，输出 stamp 也保持零。

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
- 启动允许 `use_sim_time=false`、运行期可修改它，或最终 publication 不再
  检查 active ROS clock；
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
已经验证的 policy；每个会改变时间语义的方法显式接收 steady time。真实
Interface 不是另写一套通用 `handle(Event)` 状态机，而是以下 typed surface：

```text
prepare(request, now, PrepareAdmissionProvider)
open(request, now, OpenBindingProvider)
renew(request, now)
inhibit(request, now)
accept_candidate(candidate, now)
tick(now)
snapshot()
selected_command()
force_fault(reason, detail)  # Adapter-only fault ingress
```

`selected_command()` 是只读诊断/测试视图；`force_fault()` 只供 Node Adapter
把 graph、reader、clock 或 publication failure 锁存为 fail-closed fault，
不是第五种 control operation。`PrepareAdmissionProvider` 与
`OpenBindingProvider` 是 Core implementation
内部使用的两个 seam：production Adapter 提供 bounded graph fact，测试提供
确定性 fake。它们不进入 ROS Interface，也不允许 Core 自己访问 graph。

在 CMake 中把 Core 建成 package-internal `STATIC` target
`motion_gate_core`。不要安装或 export 该 library/header；唯一安装的运行目标是
`motion_gate_node`。这让 Core 保持同 package 内的深 Module，而不是制造新的
公共 package 或外部 Interface。

Gate 才生成 lease ID 与 topic；测试 fixture 不能把任意路径塞进 Core。
这里的 candidate `writer_gid` 只代表 Node 在 **Gate 本进程** 从
graph/MessageInfo 观察后交给 Core 的 opaque identity，绝不是 control
request 从 caller 进程传来的 `Publisher::get_gid()`。

Core 拥有 state、global CAS、lease/topic 生成、两个 deadline、GID binding
decision、clamp/retirement 和 selected command。它返回 typed
`ControlResult`、`CandidateResult`、`Command` 与 `Snapshot`，但不直接 publish，
也不拥有 reader lifecycle 或“zero 已发布”事实。

Node Adapter 拥有 graph discovery、reader A/B/C、ROS endpoints、实际
final/state publication、`output_publish_seq`、`zero_publish_seq` 和 response
里的 `zero_published`。Adapter 根据 Core 前后 snapshot 协调 side effect；
不要把 ROS side effect 伪装成第二套 Core event/effect 状态机。

Core Interface 必须让以下行为只在一个地方实现一次：

- 初始 inhibited；
- global CAS；
- PREPARE/OPEN/RENEW/INHIBIT 顺序；
- 250/150 ms deadline；
- finite supported-axis CLAMP；
- invalid candidate retirement；
- old lease 不可复活。

publication serial ordering 属于 Node Adapter 的单一发布路径，但所有选择
依据仍来自 Core；回调不得绕过二者直接向 final endpoint publish。

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

它们不进入 `voice_nav_interfaces`。所有字符串与 sequence 有界；
`request_id`、`gate_instance_id` 与 `lease_id` 的 IDL transport bound 是
`string<=36`，但 Core 的语义校验更窄：request/Gate ID 必须恰好是 **32 个
lowercase hexadecimal characters**；PREPARE 的 lease 必须为空，
OPEN/RENEW/INHIBIT 的 lease 必须 exact-32。`uuid.uuid4().hex` 是合法 fixture；
大写、带连字符 UUID、31/33 字符和非 hex 字符必须被拒绝。bound writer GID
恰好 16 bytes。

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
- 当前存在时的 Gate-generated lease/topic；retire 后二者与 bound GID 清空，
  原因保留在 bounded `reason`/`detail`；
- 16-byte bound GID 是否存在；
- authority/freshness/writer validity；
- `zero_selected`、`output_publish_seq`、`zero_publish_seq` 与 typed reason。

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
  -> PREPARE admission provider confirms retired writer is absent
  -> Core PREPARE generates new lease/topic
  -> create discard-only reader A
  -> create exactly one test candidate writer
  -> Core OPEN first validates request/idempotence/state/CAS/lease/deadline;
     rejection must not query graph or mutate a reader
  -> graph snapshot #1 finds one endpoint and checks controller health
  -> destroy reader A and its queue; create discard-only reader B
  -> graph snapshot #2 must find the same complete 16-byte GID
  -> Core atomically enters ARMED with selected output still zero
  -> destroy reader B and its queue; create accepting reader C
  -> graph snapshot #3 must find the same GID and healthy controller
  -> only then complete OPEN; mismatch faults and selects zero
  -> reader C compares Gate-local MessageInfo GIDs with that exact GID
```

PREPARED 状态收到的样本全部 discard。A 与 B 永远不接受 command；C 是第一
个 accepting `VOLATILE + KEEP_LAST(1)` reader，并且只能在 Core 已经以 zero
进入 ARMED 后创建。两次 discard-reader destruction 清空旧 queue，三次 graph
snapshot 则证明 barrier 前后仍是同一个唯一 writer。OPEN 之前观察或排队的
sample 永远不能在 OPEN 后变成 non-zero。

锁定 Fast DDS/RMW 的 self-test 要证明：Gate 自己
`get_publishers_info_by_topic()` 观察的唯一 endpoint GID，与 Gate 自己收到
的 `MessageInfo.publisher_gid` 对同一 writer 能关联。若不一致或无法建立，
fail closed。State 可暴露固定 `uint8[16] bound_writer_gid` 作为 run-local
诊断；caller 的 GID 不参与协议。

这里的“锁定”是可执行约束，不只是测试备注：

- `product_sim.launch.py` 必须在任何 ROS process 前设置
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`；
- `motion_gate_node` 在构造时读取 active RMW，非
  `rmw_fastrtps_cpp` 必须拒绝启动；
- `voice_nav_mission` 与 `voice_nav_bringup` 的 manifest 必须声明
  `rmw_fastrtps_cpp` runtime dependency；
- Node/product launch tests 必须断言 active RMW。

故障注入：

- OPEN 时 zero writer；
- OPEN 时 two writers；
- graph endpoint GID 与 `MessageInfo.publisher_gid` 不一致；
- INHIBIT 后继续向旧 topic 发 non-zero；
- old writer 不退出就 PREPARE 新 lease；
- OPEN 后第二个未绑定 writer 在相同 topic 发 sample；
- snapshot #1/#2 之间 writer 改变；
- accepting reader C 创建后、snapshot #3 前 writer 改变；
- OPEN 之前排队 non-zero，OPEN 后不再发任何新样本；
- invalid/stale/collision OPEN 尝试诱发 graph access。

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
静止。launch 的第一个 action 设置
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`；Gate 本身也检查 active RMW，不能只
依赖调用者碰巧设置了正确环境。两个相关 package 都声明该 runtime dependency。

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
3. 确认 discard reader A 已建立，再在返回 topic 创建唯一 writer；
4. OPEN 先通过 Core 纯校验，再让 Gate 依次完成 snapshot #1、A→B、
   snapshot #2、Core ARMED-zero、B→C、snapshot #3；三次都必须绑定同一个
   唯一 endpoint 的完整 GID；
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

# 在任何本课验收启动前记录 baseline。已有的用户 Gazebo 进程属于
# baseline，不得为了制造“无输出”而终止。
residue_before="$(mktemp)"
residue_after="$(mktemp)"
pattern='[g]z sim|[g]z-sim|[g]zserver|[r]os2 launch .*voice_nav|[m]otion_gate_node|[c]ontroller_manager|[p]arameter_bridge|[r]obot_state_publisher'
pgrep -f "${pattern}" | sort -n > "${residue_before}" || true

git diff --check
python3 scripts/check_motion_gate_contract.py --root .
python3 -m unittest discover -s tests -p "test_motion_gate_contract.py" -v
bash scripts/verify.sh

source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Layer 1: pure Core + manual clock
ctest --test-dir build/voice_nav_mission \
  --output-on-failure -R '^motion_gate_core_test$'

# Layer 2: FastDDS, no Gazebo, no /clock
ctest --test-dir build/voice_nav_mission \
  --output-on-failure -R '^test_test_motion_gate_node.py$'

# Layer 3: FastDDS + headless Gazebo product composition
ctest --test-dir build/voice_nav_bringup \
  --output-on-failure -R '^test_test_motion_gate_product.py$'

colcon test-result --verbose
git status --short
git diff

# 三层测试和完整门禁完成后，只检查本课新引入并残留的 PID。
# product launch 的 assertExitCodes 仍负责证明所有 launch-managed
# process 正常退出。
pgrep -f "${pattern}" | sort -n > "${residue_after}" || true
if comm -13 "${residue_before}" "${residue_after}" | grep -q .; then
  printf 'FAIL: newly introduced process residue detected\n'
  comm -13 "${residue_before}" "${residue_after}"
  rm -f -- "${residue_before}" "${residue_after}"
  exit 1
fi
printf 'PASS: no newly introduced process remains; baseline preserved\n'
rm -f -- "${residue_before}" "${residue_after}"
```

三层验收不能合并成一条模糊的“integration passed”：

1. Core GTest 只跨 Core Interface，用 manual clock，不启动 ROS；
2. Node launch test 无 Gazebo、无 `/clock`，用 FastDDS、ROS domain 91、
   localhost discovery、60 秒 timeout 与 serial execution；
3. product launch test 用 FastDDS、ROS domain 92、localhost discovery、独立
   Gazebo partition、180 秒 timeout 与 serial execution。

repository-static checker 是三层之前的 prerequisite，不是第四个 runtime
layer。每条命令都分别记录 command、environment/active RMW、exit status、
test/skip count、elapsed time 与 decisive assertion；未执行时保持 `TBD`，不得
把 tests-first RED 的旧计数改写成后续 local-GREEN。

接口和手动产品检查使用真实命令，输出同样只在执行后粘贴：

```bash
ros2 interface show voice_nav_mission/srv/InternalMotionGateControl
ros2 interface show voice_nav_mission/msg/InternalMotionGateState
ros2 launch voice_nav_bringup product_sim.launch.py headless:=true
```

上面的 baseline/delta 命令必须真正包住本课验收，不能在测试结束后连续
采样两次并把“无差异”误记为 PASS。

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

- `motion_gate_core` 是不安装、不 export 的内部 STATIC target，唯一安装的
  runtime target 是 `motion_gate_node`；
- `MotionGateCore` 通过 manual-clock tests，使用真实 typed methods 与
  `PrepareAdmissionProvider`/`OpenBindingProvider`，没有第二套
  `handle(Event)` 状态机；
- private operation 精确为 PREPARE/OPEN/RENEW/INHIBIT；
- request/Gate ID 以及非 PREPARE lease 在 `string<=36` 内仍必须恰好为 32
  lowercase hex；PREPARE lease 必须为空；
- Gate 生成 lease/topic，global CAS `control_seq` 防 stale control；
- control request 不携 GID；Gate-local graph/MessageInfo 自测后绑定
  16-byte GID；
- OPEN 先纯 Core 校验，再跨 reader A/B/C 与三个 same-writer graph snapshot；
- product launch 选择 FastDDS，Gate 对其他 RMW 拒绝启动；
- `use_sim_time=true` 在启动和每次 publication 都受检查；运行期修改被拒，
  clock invariant 丢失只允许 zero + zero stamp；
- finite supported-axis over-limit 被 clamp；
- NaN/Inf/unsupported non-zero axis retire；
- authority/freshness 分别按 250/150 ms steady deadline 失效；
- publication serial barrier 保证 zero 后无旧 non-zero；
- product bringup 默认 inhibited，Gate 是唯一 final publisher；
- headless 有 bounded motion、两种 expiry、command/controller/physical zero；
- Core、无 Gazebo/`/clock` Node、headless product 三层验收以及 full gate、
  process residue audit 通过；
- 文档没有声称 Lesson 0010 crash-stop/pause 已完成。

## 提交给教师

提交真实、可查询的内容：

1. `git status --short` 与 `git log --oneline --decorate -12`；
2. start tag object/peeled target；
3. tests-first RED 与 GREEN commit；
4. 两条真实 `ros2 interface show` 命令及两个 private type 输出；
5. trusted YAML、private topic QoS 与 final SystemDefaults compatibility；
6. PREPARE/OPEN/RENEW/INHIBIT state/`control_seq` 表；
7. exact-32 identity 正反例，以及 per-lease topic、Gate-local 16-byte bound
   GID 与 graph owner；
8. unique final publisher evidence；
9. reader A/B/C、snapshot #1/#2/#3、clamp、invalid retirement、
   pre-OPEN/old-writer injection；
10. authority/freshness last-event→Gate-zero steady latency；
11. Gate zero、controller zero、odom stationary 三个时间；
12. static prerequisite 与三层 acceptance 的逐条命令/exit/count/elapsed/
    decisive assertion，以及 complete verify/build marker、residue audit；
13. Issue/PR/CI/review/rebase/tag 只在真实发生后填写。

## 复盘问题

1. 为什么 candidate 持续到达不能续 authority？
1. 为什么 Gate 生成 topic 比接受 caller 任意 topic 更容易封闭 Interface？
1. global CAS `control_seq` 防住了什么晚到 control request？
1. 为什么换 topic 后仍需要 OPEN reader-queue barrier？
1. 为什么 OPEN 需要 discard reader A/B、accepting reader C 和三个 graph
   snapshot，而不是一次 discover 后直接 ARMED？
1. 为什么 GID 必须比较完整 16 bytes，而不能只看 node name？
1. 为什么 IDL `string<=36` 不等于运行时可以接受任意 36 字符 ID？
1. 为什么当前 GID 关联实现必须锁定 FastDDS，并在错误 RMW 下拒绝启动？
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
