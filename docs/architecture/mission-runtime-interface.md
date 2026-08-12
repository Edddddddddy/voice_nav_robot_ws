# Mission Runtime 接口

**状态：**活跃的 pre-1.0 Mission V1 公共接口。Issue #34 在该稳定公共接口后实现 Mission Runtime
控制面；Issue #64 增加用于 pure-control 与 ROS-integration 验收的生产 odometry-closed-loop
RelativeMotion Adapter。无头物理 raw-stamp-age 与 TF 验收有意由 Issue #72 独立追踪。

Mission Runtime 是深层模块，拥有两项状态改变操作和一项只读状态投影：

```text
/mission/execute  voice_nav_interfaces/action/ExecuteMission
/mission/stop     voice_nav_interfaces/srv/StopMission
/mission/state    voice_nav_interfaces/msg/MissionState
```

Action cancel 属于 execution。没有 public queue、validate、execute-step、pause、resume、raw-pose 或 raw-velocity
operation。

## `MissionStep.msg`

```text
uint8 MOVE_DISTANCE=1
uint8 ROTATE_ANGLE=2
uint8 NAVIGATE_TO=3
uint8 SAVE_MAP=4

uint8 kind
float32 distance_m
float32 angle_rad
string<=64 target_id
```

message 是在 trust boundary 强制执行的 closed discriminated union：

| 类型 | 必需载荷 | 每个未使用字段 |
| --- | --- | --- |
| `MOVE_DISTANCE` | finite、non-zero `distance_m` | zero 或 empty |
| `ROTATE_ANGLE` | finite、non-zero `angle_rad` | zero 或 empty |
| `NAVIGATE_TO` | 已知 Named Place `target_id` | zero 或 empty |
| `SAVE_MAP` | 有效 logical Map ID `target_id` | zero 或 empty |

unknown kind、NaN、infinity、unused payload、out-of-policy value 和 invalid ID 均被 reject。Map ID 永不作为
path。velocity、acceleration、tolerance、deadline、retry 与 controller parameter 只来自 trusted YAML。

## `MissionState.msg`

```text
uint8 MAPPING=1
uint8 NAVIGATION=2

uint8 UNAVAILABLE=0
uint8 AVAILABLE=1
uint8 BUSY=2
uint8 FAULTED=3

uint8 GATE_INHIBITED=0
uint8 GATE_ARMED=1
uint8 GATE_FAULTED=2

string<=36 runtime_instance_id
uint64 admission_epoch
uint8 operating_mode
uint8 availability
uint8 gate_state
uint32 active_step
uint32 supported_step_mask
uint8 max_steps
string<=64[<=32] named_place_ids
```

没有 active Mission step 时，`active_step` 是 `UINT32_MAX`（`4294967295`）。Runtime contract test 必须保留该
sentinel，不得引入第二个 out-of-band state field。

QoS 为 `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`，使迟到的 Agent 在 planning 前得到 latest state。每次
`mission_runtime_node` start 都改变 `runtime_instance_id`；新的 STOP、admission-policy change 或 Named Place change
都会 rotate `admission_epoch`。

Agent 在 deterministic 或 LLM planning 前 snapshot Runtime ID 和 epoch，并必须将同一 snapshot 随 resulting Goal
发送；slow LLM 返回后 refresh token 会破坏 stale-plan fencing，因此禁止。

## `ExecuteMission.action`

```text
# Goal（目标）
string<=36 source_instance_id
uint64 source_seq
string<=36 runtime_instance_id
uint64 admission_epoch
MissionStep[<=3] steps
---
# Result（结果）
uint16 SUCCEEDED=0
uint16 INVALID_PLAN=10
uint16 BUSY=11
uint16 MODE_MISMATCH=12
uint16 UNKNOWN_TARGET=13
uint16 STALE_REQUEST=14
uint16 UNSUPPORTED_STEP=15
uint16 DEPENDENCY_UNAVAILABLE=20
uint16 EXECUTION_FAILED=21
uint16 TIMEOUT=22
uint16 CANCELED=30
uint16 STOPPED=31
uint16 SAFETY_FAULT=32
uint16 INTERNAL_ERROR=99

uint16 code
int32 failed_step
string<=160 detail
---
# Feedback（反馈）
uint8 VALIDATING=1
uint8 EXECUTING=2
uint8 SAFE_STOPPING=3

uint8 phase
uint32 step_index
float32 progress
```

`source_instance_id` 在 producer process restart 时改变，`source_seq` 在 instance 内严格递增。session 与 Voice Turn
ID 保留在 `agent_node`，不泄漏入 Mission domain。

Feedback 是 advisory：`step_index` 从不递减，`progress` 是 `[0, 1]` 内 non-decreasing best estimate，而非 deadline
promise。caller 按 Result code 分支，绝不按 diagnostic text。`failed_step=-1` 表示没有 step 开始。

每个 wire-valid Goal 都在 ROS Action transport layer 接受。invalid、stale、busy、unsupported 或 wrong-mode 等
business rejection 以 `ABORTED` 和 structured Result 完成。transport rejection 只保留给 shutdown 或不可用的
Action server。

## `StopMission.srv`

```text
# Request（请求）
string<=36 request_id
string<=36 source_instance_id
uint64 source_seq
string<=160 reason
---
# Response（响应）
uint16 APPLIED=0
uint16 DUPLICATE=1
uint16 SAFETY_FAULT=2

uint16 code
string<=36 runtime_instance_id
uint64 admission_epoch
bool motion_inhibited
string<=160 detail
```

行为称为 **Operational Stop**，ROS type 仍严格为 `StopMission.srv`。`mission_runtime_node` 服务
`/mission/stop`，因此 STOP、cancel、success、timeout 与 dependency completion 经由同一 terminal-intent
linearization point。Runtime 通过 package-private seam 同步控制独立 Gate。

production Gate Adapter 给每项 PREPARE、OPEN、RENEW、INHIBIT logical operation 一个共享的 **`250 ms`
steady-clock overall convergence deadline**。每次 service discovery 或 response wait 使用“剩余 overall time”和受信任
**`100 ms` single-attempt budget** 中较小者；每次 response 后再次检查 overall deadline。timeout 以同一 request
ID 和 payload retry。只有明确的 `STALE_GATE`、`STALE_SEQUENCE` 或 `STALE_LEASE` response 可以 rebuild authority
tuple；operation kind 与其他 immutable logical payload 仍绑定该 request ID。

新 request 无条件 rotate epoch、inhibit Gate、publish zero 并 cancel downstream operation。以同一 `request_id`
retry 返回 cached logical outcome，不再重复 rotate state。stale source metadata 不能阻止 STOP 生效。Service 仅在 Gate
inhibited 且已 published zero 后返回；`motion_inhibited=true` 不声称仿真质量已实体静止，stationarity 由 odometry
独立证明。

## 准入与执行不变量

- plan 含一至三个 step，且在第一个 motion 或 map-write side effect 前 complete validation。
- atomic validation 不是 rollback：后续 execution failure 不撤销已完成 physical motion。
- 严格一个 Mission 拥有 execution slot；没有 hidden queue，第二个 Goal 返回 `BUSY`。
- Runtime/source identity 与 epoch check 在全部 dependency call 前完成。
- Mapping 接受 move、rotate、save-map；Navigation 接受 move、rotate、navigate-to。
- Named Place、limit、mode、Gate health 与 downstream readiness 是一个 immutable admission snapshot。
- step 严格按序执行；第一个 failure 跳过余下 step。
- typed dependency callback 绑定 Runtime、epoch、Mission 与 step generation，不能推进新 Mission。raw
  `TwistStamped` 没有这些 identity；其 isolation 依赖重新创建的 per-lease data plane 与 writer-GID binding。
- 只有 Runtime private control heartbeat 续约 MotionGate authority；任何 dependency callback 或 velocity sample
  都不能 renew、reopen 或 resurrect lease。
- timeout、cancel、STOP、dependency loss、exception 与 success 均经过一个 serial terminal-intent linearization point。
- 已进入 production `on_accepted` 且获得 GoalHandle/CallbackLease 的每个 Goal，在 graceful shutdown 获得一个
  terminal Result。private Action Adapter 保持该 accepted handoff，直到 Core admission 与有界 shutdown drain 恰好
  交付一次。
- Action admission submission、queued dispatch 与 worker start permit 共用 Node-owned generation gate。quiesce
  原子关闭该 gate；queued admission 返回一个 structured safety result，不进入 PREPARE 或 OPEN，且在 Core 与
  RelativeMotion side effect 前立即重新检查 permit。
- provisional response timeout 创建有界 revoked ticket：在固定 deadline withdraw；若尚未取得
  GoalHandle/CallbackLease，不 fabricated Result。只有已进入 production `on_accepted` 的 callback 属于 graceful
  shutdown 的 late case；ROS context 或 process 开始关闭后，transport 不声称 distributed exactly-once delivery。
- immutable RelativeMotion completion record 传给 Node-owned RuntimeExecutionPlane。delivery callback 与 Goal/Core
  state 不在 Adapter transaction thread 执行；rejected record 由可独立 join 的 Node mailbox reaper reclaim。

## 终态顺序与 race

first terminal intent 决定历史 Result：

- cancel 先到：Gate zero、downstream cancel、再 `CANCELED`；后续 STOP 仍 rotate global epoch，但不改写 Result；
- STOP 先到：epoch rotation、Gate zero、downstream cancel、再 `ABORTED/STOPPED`；后续 cancel 不得改写；
- natural success 先到：`SUCCEEDED`；后续 STOP 改变当前 global authority，不改 completed history。

共同 safe-stop sequence：

```text
选择 terminal intent
  -> capture original child token，并使其 generation 失效
  -> inhibit MotionGate 并 publish zero
  -> cancel 或 abandon 已捕获的 downstream operation
  -> 等待有界 acknowledgement 或 cleanup grace
  -> commit 严格一个 Result 与匹配的 Service outcome
```

若无法证明 Gate inhibited，则返回 `SAFETY_FAULT` 并保持 Runtime unavailable。若 active STOP 因受信任 counter
耗尽而无法推进 `admission_epoch`，Runtime 保持 `FAULTED`，不声称 epoch rotation 成功，但仍完成有界 Gate-zero 与
child-cancel transaction，并交付一个 typed `SAFETY_FAULT` Result。

Mission deadline、cancel grace、Gate lease、STOP barrier 与 liveness 使用 steady clock。ROS time 仅用于 TF、sensor
data、SLAM、Nav2 与 simulation；pause 或 rewind `/clock` 不能保留旧 lease。

## 受信任 Runtime 策略

`src/voice_nav_bringup/config/mission_runtime.yaml` 是 Runtime control-plane slice 的单一已审计策略记录。参数在
startup 后只读，Node 拒绝任何与下表固定值不同的 override；它们不是 public ROS IDL 的新增内容。

| 参数 | 固定值 |
| --- | ---: |
| `mission_deadline_ms` | `30000` |
| `gate_discovery_deadline_ms` | `2000` |
| `control_response_deadline_ms` | `100` |
| `stop_barrier_ms` | `250` |
| `cancel_grace_ms` | `250` |
| `source_cache_size` / `stop_cache_size` | `64` / `64` |
| `max_steps` | `3` |
| `move_distance_min_m` / `move_distance_max_m` | `0.05` / `2.0` |
| `rotate_angle_min_rad` / `rotate_angle_max_rad` | `0.05` / `6.283185` |
| `stationarity_deadline_ms` | `1200` |

MOVE 与 ROTATE union validator 使用同一 policy value，不维护第二份 range definition。Gate discovery 在有界
steady-clock window 内持续 event-driven observation；missed startup window 使 Runtime 保持 `UNAVAILABLE` 与
fail-closed，直到观察到 healthy Gate snapshot。

production RelativeMotion Adapter 保持 public Core non-blocking：start transaction 与 teardown 在 bounded worker path
执行，STOP/Cancel 先 fence generation，并在不持有 Node mutex 时启动 #35 Gate inhibit/zero path。Runtime callback
进入 Node-owned typed queue，control event 有优先级。queue 物理预留八个 control slot 和 `120` 个 normal slot；
normal saturation 记录一个 QueueFault，不关闭 STOP/Cancel。Adapter 提供幂等 emergency inhibit/zero seam，不依赖
queue admission 或 Runtime worker。cached serialized state snapshot 用于 service timeout response；shutdown 显式 drain
ingress、Adapter transaction 与 completion callback，再释放 queue、worker 和 Core。#35 conditioning Module 保留其
`2000 ms` component RPC bound、`4000 ms` PREPARE-to-OPEN handover deadline 及
`OPEN -> Collision Monitor -> Velocity Smoother -> producer` order。reentrant odom/scan/clock callback 与 raw producer
timer 使用 shared lifetime ingress、weak owner capture 和 in-flight drain，才释放 Adapter state。

shutdown 第一阶段关闭 command、raw-timer、scan/clock、conditioning-health、collision 与 renew ingress，只保留
odom ingress 做 post-zero stationarity proof；shutdown 后该 callback 仅观测，不能创建 plan、feedback 或 raw output。
Gate zero 保持受信任 `250 ms` stop barrier，Node shutdown coordinator 使用 `250 ms + 1200 ms` joint deadline。
stationarity 从真实 steady-clock `zero_proven_at` 开始，且必须在该 absolute deadline 前证明连续 `200 ms` freshness；
proof 到达 terminal outcome 后关闭并 drain odom subscription。

## 运动与地图语义

`MOVE_DISTANCE` 在初始 heading 的 signed odometry projection 上闭环；`ROTATE_ANGLE` 在 closed-loop control 前
unwrap yaw。两者均采用 trusted slowdown、tolerance、stall 与 deadline policy。

RelativeMotion terminal code 按 cause 冻结：仅 source odom/scan/clock liveness loss 为
`DEPENDENCY_UNAVAILABLE`；RelativeMotion step deadline 为 `TIMEOUT`；stall、collision 与 execution failure 为
`EXECUTION_FAILED`；Gate、controller、container、component、candidate-writer、zero-proof、handover 与
stationarity failure 为 `SAFETY_FAULT`。只有 teardown 不能证明 Gate inhibited+zero 时，原 business failure 才提升为
`SAFETY_FAULT`；后续 zero proof 不得改写 infrastructure safety fault。

`NAVIGATE_TO` 在可信 navigation Adapter 内解析 Named Place。`SAVE_MAP` 将 occupancy YAML、image 与 posegraph
写入 temporary directory，验证 complete set 后 atomic rename。默认 reject overwrite，caller text 永不成为 path。

## 内部 seam 与 fake

private Interface 保持在 `voice_nav_mission`：

```text
RelativeMotionPort
NavigationPort
MapStorePort
MotionAuthorityPort
MotionObserverPort
SteadyClockPort
```

production Adapter 包装 odometry control、Nav2、atomic map storage、独立 MotionGate、odometry observation 与
`std::chrono::steady_clock`。scripted fake 注入 success、abort、timeout、process restart、Gate loss、partial map、
delayed cancel 与 late result。Guard、FSM 与 Named Place policy 均为 ordinary private implementation；v1 不引入
`pluginlib`、generic workflow DSL 或 Mission-level Behavior Tree。

## 当前到目标的迁移

pre-1.0 migration 已在 `voice_nav_interfaces` 完成：四种 public type 全部 bounded，runtime/source fencing 明确，
generated C++/Python contract consumer 验证 public surface。产品声称 Mission execution 前，Runtime producer 与
consumer 必须采用该 Interface。v1.0 后，breaking DDS change 创建 V2 type/endpoint 加 temporary V1 Adapter；
`api_version` field 不能使不兼容 DDS type 兼容。
