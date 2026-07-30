# ROS Interface 选型速查

VoiceNav Robot reference

## 当前 checkpoint 与目标状态

Lesson 0001–0006 的当前源码只有临时的 `MissionStep.msg` 和
`ExecuteMission.action`，尚未实现 `/mission/state`、`StopMission.srv`、
`VoiceTurn.msg` 或 `Speak.action`。下表用这些名字说明最终接口选型；它们
是 v1.0 目标契约，不是当前运行端点。

| Interface | 交互语义 | 适合 | 本项目示例 |
| --- | --- | --- | --- |
| Topic | 异步、单向、连续数据流 | 状态和传感器流 | `/scan`、`/odom`、`/mission/state` |
| Service | 短请求、短响应、立即确认 | 查询或快速控制 | `StopMission` 的“运动门已关闭”确认 |
| Action | 长任务、feedback、result、cancel | 有生命周期的操作 | 执行 Mission、TTS 播放、Nav2 导航 |

## 快速判断

- 它是不是持续产生数据？优先 Topic。
- 调用方是否需要立即得到一个短响应？优先 Service。
- 它是否需要几秒或几分钟，并且可能被取消？优先 Action。

`StopMission` 是 operational stop，不是认证急停。其 Service 响应只承诺 MotionGate 已禁止运动并请求零速度；物理停稳需要独立观察 odometry。

资料：[ROS 2 Jazzy 官方 Interface 对比](https://docs.ros.org/en/jazzy/How-To-Guides/Topics-Services-Actions.html)。
