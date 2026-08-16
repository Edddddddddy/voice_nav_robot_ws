# Voice Interface 契约

**状态：**pre-1.0 稳定公共 ROS 接口。

本文件冻结 Voice 对外暴露的两个 public type。#164 提供一个仅用于本地锁定 SenseVoice WAV 的已安装
`voice_node` composition root，负责真实 provider、`SpeechInputNode` 与 `VoiceTurn`；Agent、audio capture、
KWS、真实 TTS 与 runtime endpoint 仍不属于这个最小 voice_node 实现。

## 公共 endpoint

Voice 新增 public type 仅有：

```text
/voice/turn   voice_nav_interfaces/msg/VoiceTurn
/voice/speak  voice_nav_interfaces/action/Speak
```

`/voice/turn` 使用 `RELIABLE + VOLATILE + KEEP_LAST(1)`。它承载即时工作的最终 Mandarin Turn，不为迟到
process 保留 authority 或历史工作。`/voice/speak` 是标准 ROS Action endpoint；Action 内部 transport 不会另行
冻结为自定义 Topic QoS。

## `VoiceTurn.msg`

field 和 constant 必须保持以下 order、type 与 upper bound：

```text
uint8 COMMAND=1
uint8 STOP=2

string<=36 voice_instance_id
uint64 voice_seq
string<=36 session_id
string<=36 turn_id
uint8 kind
string<=512 text
float32 confidence
bool during_playback
```

Voice 只发布已完成 endpoint 的最终 Mandarin transcript。`COMMAND` 进入普通 Agent processing；`STOP` 表示
Operational Stop request，并以 `turn_id` 作为幂等 request identity。

## `Speak.action`

Goal、Result、Feedback 三段保持以下 order、type 与 upper bound：

```text
# Goal（目标）
uint8 NORMAL=1
uint8 URGENT=2

string<=36 source_instance_id
uint64 source_seq
string<=36 session_id
string<=36 turn_id
uint8 priority
string<=512 text
bool allow_barge_in
---
# Result（结果）
uint16 COMPLETED=0
uint16 CANCELED=1
uint16 BARGED_IN=2
uint16 FAILED=10

uint16 code
string<=160 detail
---
# Feedback（反馈）
builtin_interfaces/Duration played
```

每个 accepted Speak Goal 最终获得且只获得一个 Result。Action cancel、更新的 PlaybackScope 或 barge-in
不得暗示 Mission cancel；Speak cancel 只作用于对应 speech playback Goal。

## 类型边界

Voice public Interface 不新增 partial ASR、VAD、KWS、PCM、audio frame、device state、model token、临时 text
topic 或 test publisher/server。Voice type 使用 bounded string；`Speak.Feedback.played` 使用正式
`builtin_interfaces/Duration`，因此 `builtin_interfaces` 同时是生成期与运行期 dependency。

install space 只包含生成的 public Interface。C++/Python contract test 与 helper 只存在于 build/test context，
不安装为产品 node 或 public type。

## 契约验证

`voice_interface_cpp_contract` 与 `voice_interface_python_contract` 在 generated type 上验证 public behavior：

- C++ 与 Python 均验证全部 constant 的 name、type 和 value；
- `VoiceTurn`、`Speak.Goal`、`Speak.Result` 与 `Speak.Feedback` 均验证完整有序的
  `(field_name, field_type)` sequence；
- 验证每个 string upper bound 及 Feedback 的 `Duration` nested type；
- C++ 使用 generated type introspection，Python 使用 generated public field-type mapping；不扫描 IDL/AST，
  也不比较 full-file fingerprint。

定向验证命令：

```bash
colcon build --packages-select voice_nav_interfaces --symlink-install --event-handlers console_direct+
colcon test --packages-select voice_nav_interfaces --event-handlers console_direct+
colcon test-result --verbose
ros2 interface show voice_nav_interfaces/msg/VoiceTurn
ros2 interface show voice_nav_interfaces/action/Speak
```
