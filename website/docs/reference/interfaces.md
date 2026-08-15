# 公共 ROS 接口

VoiceNav 刻意保持较小、强类型、有界的公共 ROS surface。稳定接口使用 bounded string/sequence 与 structured code，
调用者不应依赖诊断文本或包内 topic。

## 接口一览

| Endpoint | Type | 责任 |
| --- | --- | --- |
| `/voice/turn` | `VoiceTurn.msg` | 完成的中文 COMMAND / STOP Turn |
| `/voice/speak` | `Speak.action` | 可取消、可 barge-in 的语音播放 |
| `/mission/execute` | `ExecuteMission.action` | 执行一至三个 typed Step |
| `/mission/stop` | `StopMission.srv` | 幂等 Operational Stop |
| `/mission/state` | `MissionState.msg` | Runtime 最新 retained snapshot |

## MissionStep

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

这是 closed discriminated union。每种 kind 只能使用对应载荷，其他字段必须 zero/empty。

## ExecuteMission

Goal 同时携带 Agent source identity 与规划时冻结的 Runtime identity：

```text
string<=36 source_instance_id
uint64 source_seq
string<=36 runtime_instance_id
uint64 admission_epoch
MissionStep[<=3] steps
```

常用 Result code：

| Code | 值 | 语义 |
| --- | ---: | --- |
| `SUCCEEDED` | 0 | 所有 step 成功 |
| `INVALID_PLAN` | 10 | union/范围/计划非法 |
| `BUSY` | 11 | 单执行槽已占用 |
| `STALE_REQUEST` | 14 | source/runtime fencing 失败 |
| `DEPENDENCY_UNAVAILABLE` | 20 | 必需依赖不健康 |
| `TIMEOUT` | 22 | Step deadline 到达 |
| `CANCELED` / `STOPPED` | 30 / 31 | cancel 或 Operational Stop 赢得终态 |
| `SAFETY_FAULT` | 32 | Gate/zero/handover/stationarity 证明失败 |

## MissionState

状态快照包含 `runtime_instance_id`、`admission_epoch`、mode、availability、Gate state、active step、capability mask、
`max_steps` 与 Named Place IDs。没有 active step 时，`active_step=4294967295`。

QoS 是 `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`。Agent 在规划前需要 latest state，但 retained snapshot 不是永不过期的
运动权限；Runtime identity 与 epoch 仍必须在 Goal 中匹配。

## VoiceTurn 与 Speak

`VoiceTurn` 使用 `RELIABLE + VOLATILE + KEEP_LAST(1)`，因为它是即时工作，而非给迟到进程保存的 authority。

```text
VoiceTurn:
  voice_instance_id + voice_seq
  session_id + turn_id
  kind(COMMAND|STOP) + text + confidence + during_playback
```

`Speak` Goal 绑定 source/session/turn，带 `NORMAL|URGENT` priority、bounded text 与 `allow_barge_in`。
每个 accepted Goal 最终恰有一个 `COMPLETED|CANCELED|BARGED_IN|FAILED` Result。取消 Speak 不等于取消 Mission。

## StopMission

Request 以 `request_id` 幂等，Response 返回当前 Runtime identity/epoch、结构化 code 与 `motion_inhibited`。
`APPLIED` / `DUPLICATE` 都不应被解释为物理静止；实体 stationarity 是另一项 observation。

页面事实依据：[`voice_nav_interfaces`](https://github.com/Edddddddddy/voice_nav_robot_ws/tree/main/src/voice_nav_interfaces)、
[`mission-runtime-interface.md`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/architecture/mission-runtime-interface.md)、
[`voice-interface.md`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/architecture/voice-interface.md)。
