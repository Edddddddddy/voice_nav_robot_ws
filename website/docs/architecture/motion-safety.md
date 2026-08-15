# 运动安全链

VoiceNav 的安全目标是：在支持的仿真环境中，用确定性、失效关闭的机制控制运动权限。它不是硬件 emergency stop，
也不声称功能安全认证。

<span class="vn-badge vn-badge--verified">MotionGate 已验证</span>
<span class="vn-badge vn-badge--boundary">非认证安全功能</span>

## 谁能发布最终速度？

| 来源 | 可提出运动 | 可发布最终控制器速度 |
| --- | :---: | :---: |
| KWS / ASR / Local LLM | ✓ | — |
| Deterministic Agent | ✓（语义 Step） | — |
| Mission Runtime | ✓（完成准入后） | — |
| RelativeMotion / Nav2 | ✓（active generation） | — |
| Velocity Smoother | 调节候选 | — |
| Collision Monitor | 过滤候选 | — |
| **MotionGate** | 校验租约与候选 | **唯一可以** |
| diff_drive_controller | 消费命令 | — |

## 两层 deadman

MotionGate 和 controller 各自承担不同失效边界：

1. **Runtime authority lease：250 ms**<br>
   只有 Runtime control heartbeat 能续约；速度 candidate 不能续租。Runtime crash 或依赖失活会停止续约。
2. **Controller consumer timeout：0.35 s**<br>
   当 MotionGate 进程本身消失且仿真时间继续推进时，controller 因收不到 fresh command 选择 zero。

candidate freshness 另有 `150 ms` steady-clock deadline。authority live 与 candidate fresh 必须同时成立，
non-zero 才可能被选择。

!!! note "三个时间不是同一件事"
    `250 ms` 权限租约、`150 ms` candidate 新鲜度和 `0.35 s` controller timeout 分别覆盖 Runtime、数据平面和 Gate
    进程死亡。WSL 不是实时系统，这些值是支持环境中的测试 budget，不是 hard real-time guarantee。

## 四项包内 Gate Operation

```text
PREPARE → OPEN → RENEW → INHIBIT
```

- `PREPARE` 由 Gate 生成新的 lease ID 与 per-lease candidate topic；
- `OPEN` 绑定唯一 candidate writer，并在零输出状态下进入 ARMED；
- `RENEW` 只延长 Runtime authority，不延长已见 candidate 的 freshness；
- `INHIBIT` 在 acknowledgement 前先选择并发布 zero。

每项 operation 都匹配 Gate instance、全局 compare-and-swap sequence；除 PREPARE 外还要匹配 current lease。
expired/revoked lease 不能 resurrect。

## Writer handover barrier

`TwistStamped` 不携带 Mission identity，所以系统不假装能从消息里恢复 generation。每次 step/source/restart 都会：

```text
撤销旧权限并发布 zero
  → 停止旧 producer
  → 销毁旧 smoother / collision monitor / reader
  → 证明旧 writer 从 graph 消失
  → Gate 生成新 lease 与 channel
  → 三次观察同一个唯一新 writer
  → 在 zero selected 下完成 OPEN
  → 最后才启动新 producer
```

这样，旧 DDS queue 中延迟到达的命令仍属于旧 channel 或旧 GID，不能在新 lease 中变成有效 non-zero。

## 失效行为

| 失败 | 必需结果 |
| --- | --- |
| NaN / Inf / 非支持轴 | retire lease，选择 zero |
| authority 或 candidate 过期 | inhibit，持续发布 zero |
| Runtime 消失 | Gate 独立让 250 ms lease 过期 |
| Gate 消失 | controller 在推进中的 0.35 s 后 timeout |
| Gate/zero proof 不可用 | Runtime `SAFETY_FAULT`，拒绝新 Mission |
| cancel/STOP 后 late callback | 按 epoch/generation 丢弃 |

页面事实依据：[`安全与运动契约`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/architecture/safety-and-motion-contract.md)、
[`motion_gate_core.hpp`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/src/voice_nav_mission/include/voice_nav_mission/motion_gate_core.hpp)、
[ADR-0002](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/adr/0002-migrate-to-gz-ros2-control.md)。
