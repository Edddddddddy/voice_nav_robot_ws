# Voice 与 Agent

Voice 模块负责把音频收敛成最终中文 Turn；Agent 负责把 Turn 收敛成 typed Mission、澄清、停止或回复。
两者都没有最终运动权限。

<span class="vn-badge vn-badge--verified">Agent Core / ROS Adapter 已实现</span>
<span class="vn-badge vn-badge--target">真实本地声学模型仍属里程碑</span>

## Voice 的进程边界

Voice 对 ROS 只公开两个正式类型：

```text
/voice/turn   voice_nav_interfaces/msg/VoiceTurn
/voice/speak  voice_nav_interfaces/action/Speak
```

partial ASR、VAD、KWS decision、PCM、device state 和 model token 都保留在 audio process 内。
当前源码中的 `SpeechInputCore` 只接受清理后的 `16 kHz / mono / 160-sample` frame，通过 package-private recognizer
Adapter 接收闭合事件，并只发布完成的 `VoiceTurn`。

完整目标音频边界使用单个 `48 kHz / mono` PortAudio full-duplex stream 与 `10 ms / 480 samples` DSP frame。
AEC reference 必须来自 callback 实际写入设备的最终 PCM，而不是原始 TTS buffer。

## Agent 的决策顺序

每个 COMMAND Turn 都按固定顺序处理：

```text
1. STOP classification
2. deterministic Mandarin rules
3. 缺少参数时的 bounded clarification
4. loopback local Response Provider / LLM fallback
5. closed schema validation
6. local semantic validation
7. typed ExecuteMission submission
```

确定性 rule 与 Response Provider proposal 必须经过同一个 `SemanticValidator`。Validator 只接受闭合 MissionStep union，
并检查 Runtime mode、capability mask、Named Place、数值范围和 planning token。

## Agent Core 的闭合结果

| Decision | 含义 | 运动副作用 |
| --- | --- | --- |
| `MISSION` | 已通过语义校验的 typed Mission | 交由 Runtime 再次准入 |
| `CANCEL` | 取消 Agent 当前 Mission handle | 不直接发速度 |
| `STOP` | 用 Voice identity 调用 Operational Stop | 走 Runtime Stop seam |
| `CLARIFY` | 有界追问距离、角度、Place 或 Map ID | 无 Mission |
| `REPLY` | 安全回复或拒绝 | 无 Mission |
| `LLM_NEEDED` | 进入受限 loopback provider | proposal 仍需校验 |
| `IGNORE` | bad envelope 或 replay | 无副作用 |

`AgentCore` 是无 ROS I/O、无 HTTP、可注入 steady clock 的 pure Python 模块。`agent_node` 才拥有 ROS subscription、
Action client、Stop client、Speak client 与 loopback provider Adapter。

## STOP 优先且不依赖 LLM

`VoiceTurn.kind=STOP` 或固定停止短语会绕过普通 planning。STOP request ID 直接复用 `turn_id`，source identity 复用
Voice instance/sequence；Agent retry 与 Voice fast path 因 Runtime 幂等性可以安全收敛。

停止语义仍是仿真 Operational Stop。“紧急停止”作为识别短语，不代表系统获得硬件 emergency-stop 等级。

## 过期结果隔离

Agent 只保留 capacity-one pending provider slot，并采用 latest-turn-wins：

- 新 Turn 递增 local generation；
- 旧 provider response、旧 Voice instance/sequence、旧 Runtime ID/epoch 全部丢弃；
- planning token 在开始时冻结，慢结果不能 refresh；
- provider 不可用时，确定性规则与 STOP 仍可工作。

页面事实依据：[`voice-and-agent.md`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/architecture/voice-and-agent.md)、
[`core.py`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/src/voice_nav_agent/voice_nav_agent/core.py)、
[`agent_node.py`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/src/voice_nav_agent/voice_nav_agent/agent_node.py)、
[`speech_input_core.hpp`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/src/voice_nav_audio/src/speech_input_core.hpp)。
