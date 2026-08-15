# Mission Runtime

Mission 是一次有开端、结果和终态的用户意图，不是无限对话。`mission_runtime_node` 把每个 Mission 当作一个完整计划：
先验证全部步骤，再允许第一个 motion 或 map-write side effect。

<span class="vn-badge vn-badge--verified">Runtime 控制面已验证</span>

## 一个深层公共接口

```text
/mission/execute  voice_nav_interfaces/action/ExecuteMission
/mission/stop     voice_nav_interfaces/srv/StopMission
/mission/state    voice_nav_interfaces/msg/MissionState
```

没有 public validate、queue、pause、resume、raw pose 或 raw velocity endpoint。调用者不需要编排 Runtime 内部的
RelativeMotion、Nav2、Map Store 或 Gate operation。

## Mission Step 是闭合联合类型

| Step | 有效载荷 | 典型模式 |
| --- | --- | --- |
| `MOVE_DISTANCE` | finite、non-zero `distance_m` | Mapping / Navigation |
| `ROTATE_ANGLE` | finite、non-zero `angle_rad` | Mapping / Navigation |
| `NAVIGATE_TO` | 已知 Named Place `target_id` | Navigation |
| `SAVE_MAP` | 合法逻辑 Map ID `target_id` | Mapping |

所有未使用字段必须为 zero 或 empty。NaN、infinity、unknown kind、超策略范围、未知 Place 和把 path 塞进 Map ID
都会在 side effect 前被拒绝。每个计划含一至三个步骤，按顺序执行；第一个失败会跳过剩余步骤。

## 准入围栏

Agent 在规划开始时固定以下 token，并把它原样带入 Goal：

```text
source_instance_id + source_seq
runtime_instance_id + admission_epoch
operating_mode + supported_step_mask
max_steps + named_place_ids
```

慢 LLM 返回后不能用新的 Runtime snapshot“刷新旧计划”。如果 Runtime restart、Stop 或策略变化已经改变身份/epoch，
旧 proposal 会以 `STALE_REQUEST` 或相应结构化结果失败，而不是被悄悄适配。

## 单执行槽

Runtime 同时最多允许一个 active Mission：

- 第二个 Goal 返回 `BUSY`；
- 不进入隐藏队列；
- 不静默抢占当前任务；
- cancel、STOP、timeout、dependency loss 与 success 共享一个终态线性化点。

Feedback 只提供 `VALIDATING` / `EXECUTING` / `SAFE_STOPPING`、step index 与单调 progress。
结果分支必须依赖 code，而不能解析诊断文本。

## Stop 的安全顺序

新的 Stop request 即使 source sequence 已旧，仍具有安全效果。`request_id` 只让重试幂等，不赋予忽略停止的权限。

```text
选择 first terminal intent
  → 推进 admission epoch（新 Stop）
  → inhibit MotionGate 并证明 zero published
  → cancel / abandon downstream operation
  → 提交严格一个 Mission Result
  → 返回 StopMission Response
```

若无法证明 Gate inhibited + zero，Runtime 返回 `SAFETY_FAULT` 并保持 unavailable。`motion_inhibited=true` 也只表示
运动权限已关闭，不等同于 Gazebo 中的实体已经静止；stationarity 由 odometry 独立证明。

## 可信策略

`src/voice_nav_bringup/config/mission_runtime.yaml` 是当前控制面的审计策略入口。关键固定值包括：

| 策略 | 值 |
| --- | ---: |
| Mission deadline | 30 s |
| Stop barrier | 250 ms |
| Cancel grace | 250 ms |
| Stationarity deadline | 1200 ms |
| MOVE 范围 | 0.05–2.0 m（带符号） |
| ROTATE 范围 | 0.05–6.283185 rad（带符号） |
| 最大步骤数 | 3 |

页面事实依据：[`Mission Runtime 接口`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/architecture/mission-runtime-interface.md)、
[`mission_runtime_core.hpp`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/src/voice_nav_mission/include/voice_nav_mission/mission_runtime_core.hpp)。
