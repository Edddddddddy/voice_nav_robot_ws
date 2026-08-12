# 安全与运动契约

**状态：**目标 v1.0 契约。

VoiceNav Robot 为支持的仿真环境提供确定性、fail-closed 的 **Operational Stop**。它不声称硬件 emergency stop
或功能安全认证。

## 信任与 ownership

| 来源或进程 | 可提出运动 | 可发布最终控制器速度 |
| --- | --- | --- |
| KWS、ASR、local LLM | 可以，不可信 intent | 不可以 |
| deterministic Agent rule | 可以，semantic step | 不可以 |
| `mission_runtime_node` | 可以，whole-plan admission 后 | 不可以 |
| Nav2 或 relative-motion executor | 可以，仅 active generation | 不可以 |
| `nav2_velocity_smoother` | condition active candidate | 不可以 |
| `nav2_collision_monitor` | filter conditioned candidate | 不可以 |
| `motion_gate_node` | enforce active lease 与 limit | **可以，唯一 publisher** |
| `diff_drive_controller` | consume final command | 不可以 |

自编写 runtime process 严格为：

```text
voice_node
agent_node
mission_runtime_node
motion_gate_node
```

`mission_runtime_node` 与 `motion_gate_node` 是独立 process。独立 failure domain 使 Runtime crash、stall 或失去
executor progress 时，Gate 仍可 expiry steady-clock lease。

## 固定目标链

```text
Nav2 or relative-motion candidate
  -> MissionRuntime generation filter
  -> nav2_velocity_smoother
  -> nav2_collision_monitor
  -> independent motion_gate_node
  -> diff_drive_controller
  -> gz_ros2_control / Gazebo wheel joints
```

完整链使用 `geometry_msgs/msg/TwistStamped`。固定的 Jazzy `diff_drive_controller` Interface 天生订阅该 type，
没有 `enable_stamped_cmd_vel` switch。configuration 与 contract test 拒绝该虚构 parameter 以及已废弃的 demo-only
`use_stamped_vel` key。

没有 `twist_mux`：single Mission execution slot 已决定 active source，第二套 priority/ownership model 会含糊。
velocity smoother 只 condition acceleration 和 velocity；Collision Monitor 是 protective collision-avoidance layer，
不是认证 emergency-stop system，二者均不能替代 MotionGate。

`diff_drive_controller` enforce 最终 hard linear/angular velocity bound，但 acceleration/deceleration limit parameter 保持
unset。若在这里再加 limiter，`cmd_vel_timeout` expiry 后将进入 ramp，而不是在第一个 controller update 选择 zero。
`nav2_velocity_smoother` 在上游拥有 normal acceleration shaping；command-zero latency 与 physical stationarity 是独立
measurement。`ros_gz_bridge` 在目标产品路径只 bridge `/clock` 与 `/scan`；velocity command、joint state、odometry 与
TF 均留在 ROS 2 control。

## MotionGate 契约

- MotionGate start 时 inhibited，restart 后不能恢复更早 lease。
- 它是 controller final-command endpoint 的唯一 publisher。
- Runtime 在 smoother 前按 Runtime instance、admission epoch、Mission generation 与 step generation filter child
  callback；这些 identity 不在 `TwistStamped` 中，MotionGate 不会假装从 candidate message 恢复或验证它们。
- MotionGate 在 `PREPARE` 时生成 opaque authority `lease_id` 与 per-lease candidate topic；caller 不得提供任意
  ID 或 path。Runtime 通过 private control seam open/renew。authority lease 是 **Gate steady clock 上 `250 ms`**；
  velocity candidate 永不 renew。
- IDL 为 transport 将 `request_id`、`gate_instance_id`、`lease_id` bound 为 36 char。Core 要求 request 与 Gate
  identity 严格为 32 个 lowercase hexadecimal char；`PREPARE` 的 lease field 必须 empty，`OPEN`/`RENEW` 必须携带
  精确 32 lowercase-hex current lease。`PREPARED`/`ARMED` 状态的 `INHIBIT` 同样必须携带 current lease；Gate 已经
  `INHIBITED` 时，zero reassert 必须使用 empty lease。uppercase、hyphenated UUID、short value 与 non-hex text 都
  invalid。
- 每个 control operation 使用一个 Gate-wide compare-and-swap `control_seq`，并精确匹配 current Gate instance；
  `OPEN`、`RENEW` 与带 lease 的 `INHIBIT` 还必须匹配 current lease。generation-bound teardown 只能在授予该
  generation 的 Gate identity 内重建 stale `control_seq`/lease，不能跨 identity 取得 replacement Gate 的 lease，
  也不能把 replacement Gate 的 zero 当作旧 generation 的完成证明。stale request 没有任何 state effect，包括与更晚
  lease race 的 old-lease `INHIBIT`。expired/revoked lease 不得 resurrect；再次取得 authority 必须重新执行完整
  handover protocol。
- candidate freshness 是独立 steady-clock deadline。non-zero output 同时要求 live Runtime authority 和从 current
  bound data plane 收到 fresh candidate。freshness 在 **`150 ms` Gate steady time** 后 expiry。新 lease 的 first
  candidate 到达前，successful activation RENEW 只重启此有界 first-sample window；任一 candidate accepted 后，
  RENEW 永不延长 candidate freshness。candidate sample 永不延长 authority lease。
- finite `linear.x`/`angular.z` 超 trusted YAML bound 时 clamp。NaN、Inf、non-zero unsupported axis、stale input，
  或来自 unbound writer/topic generation 的 sample，均 retire lease 并选择 zero。
- authority 或 candidate expiry inhibit motion，并每 **`20 ms` wall time** continuous publish zero，不等待 Runtime、
  ROS time、Nav2 或 Gazebo time。
- matching current `INHIBIT` 在 acknowledgement control request 前选择并 publish zero。
- `diff_drive_controller.cmd_vel_timeout` 是 **`0.35 s`**，是 MotionGate 自身死亡后的 consumer-side second deadman。
- Runtime crash、cancel、timeout、dependency loss、invalid generation 与 Gate fault 均 fail closed。
- 无法建立 Gate health 或 zero-output state 时，生成 `SAFETY_FAULT`；Runtime faulted 时不准入新 Mission。
- Runtime 仅在 active step、Gate、`/clock`、odometry 与必要 dependency 均按 steady-clock liveness fresh 时 renew
  authority。任一 prerequisite loss 都停止 renewal 并终结旧 lease；dependency 恢复不会 reopen 它。

WSL 不是 real-time environment。`250 ms` 与 `0.35 s` 是支持环境下的测试 budget，不是 hard real-time guarantee。

### 包内 control/state seam

private ROS type 位于 `voice_nav_mission`，而非 `voice_nav_interfaces`：

```text
voice_nav_mission/srv/InternalMotionGateControl
voice_nav_mission/msg/InternalMotionGateState
```

`motion_gate_core` 是 package-internal static build target，header 与 library 不 install/export；
`motion_gate_node` 是该 MotionGate submodule 唯一 install 的 runtime target。同一 package 还 install 当前 Mission
control-plane target `mission_runtime_node`；其 public endpoint 与 unavailable production-motion boundary 由
[Mission Runtime 接口](mission-runtime-interface.md)定义。

Core Interface 是 typed `prepare`/`open`/`renew`/`inhibit`/`accept_candidate`/`tick`/`snapshot` surface 与 read-only
`selected_command`。Adapter-only `force_fault` 将 graph、reader、clock 或 publication failure latch 入 fail-closed state，
它不是第五项 control operation。`PrepareAdmissionProvider` 与 `OpenBindingProvider` 是 internal seam，允许 ROS
Adapter 提供 bounded graph fact，而不将 ROS graph API 移入 Core contract。

节点 FQN 为 `/motion_gate_node`；私有绝对 endpoint 是 `/motion_gate/internal/control` 与
`/motion_gate/internal/state`。PREPARE 返回位于
`/voice_nav_internal/motion_gate/candidate/lease_` 下的有界 topic。这些名称以及最终的
`/diff_drive_controller/cmd_vel` endpoint 都是代码常量，不是 YAML parameter 或产品 launch remap。可信参数 YAML
使用精确根 `motion_gate_node`。

仅有 `PREPARE`、`OPEN`、`RENEW` 与 `INHIBIT` 四种 operation。`PREPARE` 匹配当前 Gate instance 与预期的全局
`control_seq`；其余 operation 还匹配当前 lease。每个被接受的 operation 都推进唯一的 Gate-wide sequence。过期的
instance、lease 或 sequence 返回有界的强类型 mismatch，而不改变 Gate state。公共 `StopMission` 在 Mission 边界仍
无条件具有安全效果：Runtime 先线性化 STOP，再 inhibit **当前** Gate tuple；任意私有、过期的 `INHIBIT` 不是 STOP。
request 与 Gate-instance identity，以及每个非 PREPARE lease identity，虽有 36 字符的 IDL transport bound，仍具有上文
所定义的精确 32 字符小写十六进制语义。PREPARE 不得携带 lease ID。

`InternalMotionGateControl` 不包含 writer GID。`OPEN` 时，MotionGate 使用自身的 graph context 要求精确一个
publisher endpoint，并记录该 endpoint 完整的 16-byte GID。candidate callback 只与同一 Gate context 中观测到的
`MessageInfo.publisher_gid` 比较。锁定 Fast-DDS 的 self-test 证明两种 Gate-local 表示关联；失败或 mismatch 保持
Gate inhibited。这是严格的受支持 runtime 限制：canonical 产品 bringup 设置
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，`motion_gate_node` 在 startup 拒绝其他 RMW，且两个 runtime package 均将
`rmw_fastrtps_cpp` 声明为 execution dependency。调用方的 `Publisher::get_gid()` 不跨进程传输或比较。

同一 fail-closed 规则适用于 command clock。产品 startup 要求 `use_sim_time=true`；Node 拒绝任何运行时修改该
parameter 的尝试。每次最终发布前，串行 barrier 独立要求该 parameter 保持 true，且
`get_clock()->ros_time_is_active()` 为真。任一 invariant 丢失都会 latch `ConfigurationInvalid`，将 selected command
替换为 zero 并发出 zero ROS stamp，从而使 system-time-stamped 的 non-zero command 无法绕过 controller 的
simulation-time consumer timeout。

state snapshot 使用 `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`，报告 Gate instance、全局 sequence、
`INHIBITED`/`PREPARED`/`ARMED`/`FAULTED` state、当前 lease 与 topic、validity flag、output sequence/zero state、
有界 reason，以及仅用于本运行诊断的可选固定 16-byte bound GID。包内 type 与不显眼的 topic 名称缩小受支持的
Interface surface；它们不是 DDS authentication 或 authorization。

candidate input 使用 `BEST_EFFORT + VOLATILE + KEEP_LAST(1)`。向 `/diff_drive_controller/cmd_vel` 的最终 Gate
publisher 使用 `rclcpp::SystemDefaultsQoS()`，以匹配锁定 controller 的 subscriber。Runtime graph check 证明实际
endpoint compatibility 与唯一 ownership；内省得到的 reliability、history 或 depth 如为 `UNKNOWN`，不得被硬编码成
虚假的 assertion。

control、candidate、expiry、state 与 output decision 都跨越同一 publication serial barrier。callback 从不直接发布。
一旦 current-lease `INHIBIT`、expiry 或 invalid-input retirement 经该 barrier 发布 zero，较早排队的 non-zero decision
不得随后发布。Core 拥有 selected command 与 state decision，而非 publication acknowledgement。Node Adapter 拥有
实际 final/state publication、`output_publish_seq`、`zero_publish_seq` 与 response 中的 `zero_published` 事实。

### Authority 与 candidate handover barrier

`TwistStamped` 在完整链中有意保持 velocity type；向 controller command 添加 Mission metadata 会耦合 motion
conditioner 与 Mission internal。该选择需要显式 barrier，防止 smoother、Collision Monitor 或 DDS 缓冲的 old command
被误认为新 admitted step command。

每次 initial arm、source change、step change、expired lease、cancel recovery、STOP recovery 或 Runtime restart 均执行：

```text
revoke old authority, inhibit Gate, and select/publish zero
  -> stop old producer and cancel child operation
  -> fully unload/destroy old smoother and Collision Monitor instances
  -> destroy Gate's old candidate subscription
  -> confirm old output writer GID disappeared from ROS graph
  -> PREPARE admission confirms retired writer absent
  -> Core PREPARE generates new lease ID and per-lease candidate topic
  -> create discard-only reader A
  -> create/configure new Collision Monitor and smoother downstream-to-upstream
  -> OPEN pure Core validation (reject: no graph query/reader mutation)
  -> graph snapshot #1: one writer and healthy final controller
  -> destroy reader A/queue; create discard-only reader B
  -> graph snapshot #2: same unique writer GID
  -> Core atomically enters ARMED with selected output still zero
  -> destroy reader B/queue; create accepting reader C
  -> graph snapshot #3: same writer GID and healthy controller
  -> only then complete OPEN with a 250 ms Runtime authority lease
  -> RENEW while zero; activate Collision Monitor; RENEW;
     activate Velocity Smoother; RENEW once more under admitted generation
  -> start new producer last
```

reader A/B 始终 discard-only。reader C 是第一个 accepting reader，仅在 Core atomically enter `ARMED` 且选择 zero 后
创建。两次 discard-reader destruction 是 queue barrier；三次 graph snapshot 证明 unique writer 未改变。因此 pre-OPEN
sample 永不能变成有效的 post-OPEN non-zero command。Gate callback 只接受 handover 中绑定的 writer 和新 per-lease
channel；old sample 保留 old channel 或 Gate-local writer identity，即使 DDS 在新 lease open 后投递也 invalid。

lifecycle deactivate/cleanup/configure cycle 不充分：固定的 Nav2 1.3.12
[Velocity Smoother cleanup](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_velocity_smoother/src/velocity_smoother.cpp#L199-L206)
不会清除所有 cached command state。支持实现完全 unload/recreate 两个 component 与 Gate reader。quiet window 可作为
diagnostic 记录，但不是 isolation proof。failed unload、graph disappearance、new-writer binding、activation 或
acknowledgement 都保持 Gate inhibited 并变为 `SAFETY_FAULT`。

barrier 在连续 Mission step 间也必需，即使两 step 使用同一 producer。limit、timeout 与 queue bound 来自 trusted
configuration，并以故意 delayed old command 验证。当前实现交付 normal-running Core、private seam、Gate-local binding、
barrier、final ownership 与 deadline expiry 的 in-process authority/candidate harness；它不声称完整 Runtime/smoother/
Collision Monitor integration。process-kill crash-stop、consumer-deadman proof 与 managed/unmanaged Gazebo pause
behavior 是独立 target acceptance slice。

### Gazebo managed safe-pause 与 resume

controller timeout 本身不能证明 pause 后“无 replay”：Jazzy `diff_drive_controller`
[从 controller time 与 Twist stamp 计算 command age](https://github.com/ros-controls/ros2_controllers/blob/jazzy/diff_drive_controller/src/diff_drive_controller.cpp#L94-L116)。simulation time 停止后，`0.35 s` timeout 不推进。

支持的 product/test path 在 simulation update 仍推进时做两阶段 safe-pause transaction：

1. reject new Mission、停止 renew Runtime authority、inhibit MotionGate，并收到 zero-output acknowledgement；
2. 观察 `diff_drive_controller` limited command output 和 wheel command/velocity state，在配置数量的完整 control
   period 内为 zero；
3. Gate 在 proof 前 failure 时，让 consumer timeout 或 controller deactivation 仍在 update loop 推进时完成，
   但仍必须直接观测 configured control period 的 zero wheel command；inactive/released interface 不是 zero proof；
4. 仅在 zero proof 后 pause Gazebo，并记录含 world iteration 与 Gate/controller instance state 的 opaque safe-pause
   token。

这是 `voice_nav_bringup`/test-harness operational transaction，不是 public Mission pause endpoint，也不是第五个
self-written resident process。若在 bounded pause deadline 前不能观测 zero，不 mint token；harness 从 known zero state
terminate/restart simulation/control，而非冻结未证明 command。

managed resume 要求该 token，并验证记录的 controller state 未变。因此无论 controller remain active 还是曾
deactivate，first resumed `PreUpdate` 前 wheel command 已证明 zero。deactivated controller 只能在 inhibited Gate 与
zero path healthy 后再次 activate；first resumed wheel command 显式 test 为 zero。

在该 barrier 前的 direct GUI/Gazebo Transport pause 没有 safe-pause token。managed resume 拒绝原地 unpause；支持的
recovery 是从 known inactive、zero-command state 完整 restart simulation/control。原因是 paused `gz_ros2_control`
不能在第一次 resumed write 前处理 controller switch 或消费新 buffer zero。项目不将任意外部 pause/resume 视为
functional-safety mechanism。

## StopMission ownership 与顺序

`mission_runtime_node` 拥有 public `StopMission.srv` endpoint，因为 STOP、cancel、success、timeout 与 downstream
completion 必须通过一个 serial linearization point。MotionGate 仅向 Runtime 暴露 internal control seam；它不是第二个
public product-control API。

新的 Stop request 永远有 safety effect，即使 source sequence 已旧。`request_id` 使 retry 幂等，不赋予忽略 stop
的权限：

```text
deduplicate request_id
  -> first terminal-intent linearization
  -> rotate admission_epoch for a new request
  -> inhibit MotionGate and select/publish zero
  -> cancel active downstream executor
  -> commit active Mission's STOPPED result when STOP won
  -> return StopMission response
```

response 仅在 MotionGate 报告 inhibited 且已 published zero 后发送。`motion_inhibited=true` **不**表示 simulation
inertia 已带来 physical rest。重复相同 `request_id` 返回 current response，不再 rotate epoch。后续 STOP 即使另一个
terminal intent 已赢得 active Goal，仍 rotate global epoch 和 inhibit motion，但不改该 Goal 历史 Result。Operational
Stop 不是 pause；只有携带最新 Runtime instance/admission epoch 的新计划、经 complete validation 后，才能取得新 lease。

## Cancel、STOP 与 completion race

所有 terminal intent 采用 first-terminal-intent-wins：

- **Cancel 赢**：Gate zero、downstream cancel，再 outer/inner `CANCELED`；后续 STOP 仍 rotate epoch 并保持 motion
  inhibited。
- **STOP 赢**：rotate epoch、Gate zero、downstream cancel，再 outer `ABORTED` 与 inner `STOPPED`；后续 cancel 不得
  改写 Result。
- **Success 赢**：publish final zero 并 commit `SUCCEEDED`；后续 STOP 只变 current global admission，不变 history。
- **Timeout/failure 赢**：Gate zero、cancel child，commit 严格一个 structured failure Result。

late Nav2、relative-motion、map、timer callback 按 Runtime instance、admission epoch、Mission generation 与 step generation
discard；late velocity sample 由上述 candidate handover barrier isolation。已进入 production `on_accepted` 且获得
`GoalHandle`/`CallbackLease` 的 Goal 在 graceful shutdown 获得 exactly one terminal result。没有 handle 的 provisional goal
在 bounded handoff window 内 revoke，不能 fabricated Result；context/process closing 后，transport 不声称 exactly-once。

## 相对运动

- `RelativeMotionController` 是 production `RelativeMotionPort` 后的 deep ROS-free Module；其 ROS Adapter 观察
  odometry/source-health，不拥有 final velocity writer。
- MOVE 将 odometry displacement project 到 signed initial-heading axis；ROTATE 在比较 signed angular displacement 前
  unwrap yaw。
- 两者使用 trusted YAML speed、acceleration、tolerance、stall threshold 与 policy-computed deadline；临近 target
  slowdown、completion 前 publish zero，并 commit 一个 first-terminal Result。
- Relative-motion sample 受 Runtime/admission/Mission/step generation 与 active Gate lease fencing；late odometry、timer
  或 downstream callback 不能 publish command 或改写 terminal Result。
- step deadline、stall window、lease expiry 与 cancel grace 用 steady clock。ROS time 只 timestamp simulation-time data，
  包括最终 `TwistStamped`、odometry、TF 和 sensor message；永不驱动 deadline。MotionGate 在 process lifetime 锁定
  `use_sim_time=true`；若 invariant 或 active ROS clock loss，则 fault closed，只 emit zero command with zero stamp。
- dependency steady liveness 为 `200 ms`。在 simulation 中，Collision Monitor raw ROS-time source-age limit 固定
  `300 ms`；即使 callback 继续到达，old raw measurement 仍 fail closed。保留 original scan measurement stamp/frame，
  Collision Monitor 直接消费 `/scan`，consumer 使用 `SENSOR_DATA + KEEP_LAST(1)`；没有 conditioned-scan relay
  restamp 或 mask sensor backlog。headless raw-age/TF physical acceptance 由 Issue #72 追踪。
- Runtime child callback 经由带 reserved control capacity 和 generation-tagged event 的 Node-owned typed queue serialise。
  STOP/Cancel 先 fence generation、启动 async teardown；若 ROS service 不能 enqueue/await response，使用 serialized
  state snapshot。
- normal queue saturation 仅 reject normal event 并记录一个 QueueFault；reserved STOP/Cancel lane 保持可用。若 queue
  admission 或 Runtime worker fail，Adapter 独立 emergency inhibit/zero path 仍执行且幂等。
- stationarity 只从真实 steady-clock Gate `zero_proven_at` 后收到的 odometry 测量；deadline 绝对为
  `zero_proven_at + 1200 ms`，不因 cleanup 延长。

### RelativeMotion 生产 seam

- Runtime event queue 有独立 normal/control capacity。normal lane full 记录 bounded queue fault；control lane full
  触发独立 EmergencyFence，推进 admission epoch、inhibit/zero Gate，并阻止 late event reopen old generation。
- cancellation 在每个 controller、writer、lifecycle、component boundary 后 fence，且在 `OPEN` 前再次 fence。因此
  cancelled start 即使 downstream late return，也不能 publish producer command 或进入 `OPEN`。
- start-drain timeout cleanup 由 object-held async continuation 拥有，不依赖 destruction；producer stop、component cleanup、
  generation reclaim 与 terminal publication 每项至多一次。
- health/teardown 保持 frozen typed failure-code taxonomy：source-only odom/scan/clock liveness loss 为
  `DEPENDENCY_UNAVAILABLE`；RelativeMotion step deadline 为 `TIMEOUT`；stall/collision/其他 motion execution failure 为
  `EXECUTION_FAILED`；Gate/controller/container/component/candidate-writer/zero-proof/handover/stationarity failure 为
  `SAFETY_FAULT`。只有 teardown 不能 prove Gate inhibited+zero 时，original business failure 才升级为
  `SAFETY_FAULT`；证明 zero 不改写 infrastructure safety fault；residual safety fault 对后续 admission latch。
- Node shutdown 停止 ingress、drain 已接受的 internal completion event，等待 production `on_accepted` 保存的
  GoalHandle/CallbackLease 取得一个 graceful-shutdown terminal，再关闭 queue 并销毁 Runtime state。provisional/no-handle
  ticket 在 fixed bound revoke，永不 fabricated Result。ROS context/process closing 后不声称 distributed exactly-once。
  terminal record bound 为最近八个 generation。
- Action admission 由 Node-owned gate linearise，该 gate 共享 on-goal/on-accepted handoff、AdmitEvent dispatch、start
  permit 与 quiesce。generation-bound permit 在 quiesce 后 invalid，因此已在 queue 的 event 也不能 start Core、
  PREPARE、OPEN 或 producer。provisional revoked ticket 是 bounded shutdown state，不承诺保留 late transport handoff。
  只有已进入 production `on_accepted` 且有 GoalHandle/CallbackLease 的 callback 参与 graceful second drain；
  MotionGate inhibited+zero 仍是独立 safety guarantee。
- production Node 使用 package-private RuntimeExecutionPlane，一起拥有 RuntimeCore 与 NodeCompletionMailbox。transaction、
  start failure 与 emergency relay rejection 都经该 plane 收敛为一个 structured Goal terminal；mailbox shutdown 幂等，
  并在同步 state 已构建后 join reaper。
- reentrant RelativeMotion ROS callback 使用 shared lifetime ingress、weak Impl/producer capture 与 in-flight guard。
  shutdown 在 reset subscription、raw timer、producer 前禁用新 ingress，再等待 queued/active callback 结束后释放 state。
  production seam 在真实 MultiThreadedExecutor 上测试 odom、scan、clock、raw timer 和 command-supplier callback 的
  barrier。

## 失败行为

| 失败 | 必需行为 |
| --- | --- |
| invalid、stale 或 oversized Mission | execution side effect 前 reject |
| busy 时第二个 Mission | 返回 `BUSY`；无 queue/implicit preemption |
| whole-plan validation 时 dependency unavailable | earlier step 启动前 reject |
| candidate stale 或 authority lease expired | Gate inhibit、latch old lease closed、continuous publish zero |
| Runtime 消失 | Gate lease independent expiry |
| MotionGate 在 simulation 推进时消失 | controller 无 fresh command，在推进 simulation 的 `0.35 s` 内 timeout |
| MotionGate 在 managed safe-pause 后消失 | token 发行前已直接证明 wheel command zero；token-checked resume 保持此状态 |
| Gazebo safe-pause/managed resume | first resumed wheel command zero；不 replay stale non-zero |
| 没有 safe-pause token 的 direct external pause | 拒绝 in-place resume；从 known zero state restart simulation/control |
| Nav2 abort 或 step deadline | Gate zero、cancel child、fail step、skip remainder |
| map save partial failure | 不 publish completed logical map directory |
| cancel 后 late callback | 按 epoch/generation discard；经 inhibited handover barrier discard velocity |
| Gate health 或 zero proof unavailable | 报告 `SAFETY_FAULT` 并保持 fail-closed |

终态代码刻意不由稍后 zero proof 推断：

| 类型化原因 | 终态代码 |
| --- | --- |
| 仅 odom/scan/clock source liveness | `DEPENDENCY_UNAVAILABLE` |
| 仅 RelativeMotion step deadline | `TIMEOUT` |
| stall、collision 或 motion execution failure | `EXECUTION_FAILED` |
| Gate/controller/container/component/writer/zero/handover/stationarity | `SAFETY_FAULT` |

## 验证义务

- 当前累计验证保留 pure-Core manual-clock GTest、deterministic conditioning/ROS-integration check、没有 Gazebo 与
  `/clock` 的 Fast-DDS-locked Node launch test，以及既有 MotionGate/perception headless product layer。Issue #64 不
  声称 headless physical RelativeMotion acceptance；raw-age/TF evidence 属于 Issue #72。repository-static contract 是
  prerequisite，不能替代任何 layer。
- historical fixed-domain evidence 不是当前 acceptance evidence。当前 launch layer 使用官方
  `run_test_isolated.py`，清除 inherited `ROS_DOMAIN_ID` 与 `DISABLE_ROS_ISOLATION`，并分配 process-isolated ROS
  domain with localhost discovery；该规则不会 retroactively 为旧 tag 提供 evidence。
- Node layer timeout `60 s` 且 serial execution；product layer 另有 unique Gazebo partition、`180 s` timeout 与
  serial execution。
- manual-clock test 无 sleep 地证明 lease、cancel-grace、timeout 与 callback-fencing behavior；OPEN test 证明 pure
  validation 先于 graph access，reader A/B/C 成功前严格跨三次 same-writer graph snapshot。
- Runtime-death test 持续 inject valid-looking candidate，证明不能 renew independent authority lease；Stop test assert
  `EPOCH -> INHIBIT/ZERO -> CANCEL -> RESPONSE` 与 idempotent `request_id`；Runtime test 覆盖
  cancel/STOP/success race 与 exactly-one Result。
- process-death test 分别 kill Runtime 与 MotionGate，证明两层 deadman；managed safe-pause/resume test 证明 old
  non-zero command 不能在契约边界内恢复运动。
- handover test 在 full pipeline recreation 前、中、后注入 old-writer candidate，证明 channel/GID binding 全部 reject；
  pause test 运动中请求 safe-pause，证明 controller/wheel command 在 `/clock` 停止前到达 zero，pause 中 kill MotionGate，
  并 assert first resumed wheel command zero。
- pause test 还在 zero proof 前 kill MotionGate；interface release/controller inactivity 永不在无 observed zero 时 mint
  token，bounded proof failure 选择 full restart。unmanaged-pause test 证明 missing/mismatched safe-pause token 拒绝
  in-place resume 并选择 full-restart recovery。
- odometry test 区分 command-zero latency 和 physical stationarity；RelativeMotion test 覆盖 signed projection、跨
  `+/-pi` yaw unwrap、有界 command limit、progress monotonicity、stall/deadline edge 和 manual steady clock 下的
  zero-proof stationarity fencing。
- 任一 Gate 或 controller 仍可能保留 authorized non-zero command 时，test 不得退出。
