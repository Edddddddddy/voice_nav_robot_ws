# Voice Interface 契约

**状态：** pre-1.0 稳定公共 ROS Interface

本文件冻结 Voice 对外暴露的两个公共类型。该 Task 只生成和验证契约，
不实现 `voice_node`、`agent_node`、音频采集、ASR、TTS 或运行时 endpoint。

## 公共 endpoint

Voice 新增的公共类型只有：

```text
/voice/turn   voice_nav_interfaces/msg/VoiceTurn
/voice/speak  voice_nav_interfaces/action/Speak
```

`/voice/turn` 使用 `RELIABLE + VOLATILE + KEEP_LAST(1)`。它承载即时工作的
最终 Mandarin Turn，不为迟到进程保留权限或历史工作。

`/voice/speak` 是标准 ROS Action endpoint。Action 的内部 transport 不作为
自定义 Topic QoS 另行冻结。

## `VoiceTurn.msg`

字段和常量必须保持以下顺序、类型与上界：

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

Voice 只发布已经 endpoint 的最终 Mandarin transcript。`COMMAND` 进入普通
Agent 处理；`STOP` 表示 Operational Stop 请求，并使用 `turn_id` 作为
幂等请求身份。

## `Speak.action`

Goal、Result、Feedback 三段必须保持以下顺序、类型和上界：

```text
# Goal
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
# Result
uint16 COMPLETED=0
uint16 CANCELED=1
uint16 BARGED_IN=2
uint16 FAILED=10

uint16 code
string<=160 detail
---
# Feedback
builtin_interfaces/Duration played
```

每个已接受的 Speak Goal 必须最终获得且只获得一个 Result。Action Cancel、
更新的 PlaybackScope 或获准 barge-in 不得暗示取消 Mission；Speak cancel
只作用于对应语音播放 Goal。

## 类型边界

Voice 公共 Interface 不增加 partial ASR、VAD、KWS、PCM、音频帧、设备状态、
模型 token、临时文本 Topic 或测试 publisher/server。Voice 类型使用有界
字符串；`Speak.Feedback.played` 使用正式的
`builtin_interfaces/Duration`，因此 `builtin_interfaces` 同时是生成时和
运行时依赖。

安装空间只包含生成的公共 Interface；C++/Python contract test 及其 helper
只存在于构建/测试上下文，不安装为产品节点或公共类型。

## 契约验证

`voice_interface_cpp_contract` 和 `voice_interface_python_contract` 通过
生成后的类型验证公共行为：

- C++ 与 Python 都验证全部常量的名称、类型和值；
- `VoiceTurn` 以及 `Speak.Goal`、`Speak.Result`、`Speak.Feedback` 都验证
  完整有序的 `(field_name, field_type)` 序列；
- 验证每个字符串上界，并验证 Feedback 的 `Duration` 嵌套类型；
- C++ 使用生成类型的 introspection，Python 使用生成类型公开的字段类型
  映射；不扫描 IDL/AST，也不比较完整文件指纹。

定向验证命令为：

```bash
colcon build --packages-select voice_nav_interfaces --symlink-install --event-handlers console_direct+
colcon test --packages-select voice_nav_interfaces --event-handlers console_direct+
colcon test-result --verbose
ros2 interface show voice_nav_interfaces/msg/VoiceTurn
ros2 interface show voice_nav_interfaces/action/Speak
```
