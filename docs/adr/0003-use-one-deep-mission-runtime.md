---
status: accepted
---

# 使用一个深层 Mission Runtime 与独立 Motion Gate

VoiceNav Robot 通过 `ExecuteMission.action` 暴露 Mission 执行，通过 `StopMission.srv` 暴露
Operational Stop，并通过一个 transient-local state snapshot 提供观测。Guard、single-slot admission、
source/runtime fencing、workflow、Nav2、relative motion 与 map saving 均位于 `mission_runtime_node`
之后；最终速度权限位于同 package 的独立 `motion_gate_node` 进程。

## 考虑过的方案

- 分离 Guard、scheduler、per-step executor、Nav2 bridge 和 map-saver node；
- 向 Agent 暴露直接的 Nav2 和 velocity 操作；
- 保持较小的 public Interface、内部 Adapter 和独立最终 gate。

## 后果

调用者只需理解两项 mutation operation，而非分布式编排。测试替换内部 Nav2、velocity、map、gate 和
clock Adapter。独立 Gate 进程在 Runtime 停滞时仍能让 lease expiry 生效；package locality 则避免其私有
control seam 变为 Agent contract。
