# Voice 与 Agent 契约

**状态：**目标 v1.0 契约。

`voice_node` 负责 full-duplex device、real-time audio boundary、AEC、KWS、VAD、ASR、TTS、playback 与
barge-in。`agent_node` 负责 deterministic command rule、clarification、受约束 local-LLM fallback 与 Mission
submission。两个 process 均没有最终运动权限。

## Issue #46 Agent Core 边界

`voice_nav_agent` 的 Core 是无 ROS I/O、无 HTTP、可注入 steady clock 的 pure Python Module。其 behavior seam
为一次 `handle_turn(VoiceTurn, MissionState-or-none)` 和共享 `SemanticValidator`；normalizer、closed rule parser、
clarification table 与 Voice sequence fencing 都保留在 Core 内。Core 输出 closed
`MISSION`、`CANCEL`、`STOP`、`CLARIFY`、`REPLY`、`LLM_NEEDED` 或 `IGNORE` Decision；未来 LLM proposal 与 rule
proposal 必须经过相同 Validator。

非 STOP 的 Mission/LLM planning token 在 turn 开始时固定 Agent source identity、Voice turn identity、generation、
Runtime ID/epoch/mode/capability/Named Place snapshot，planning 期间不 refresh。STOP 遵循 D-046-003B：request ID
等于 `turn_id`，source instance/sequence 直接复用 Voice Turn 的 `voice_instance_id`/`voice_seq`，reason 固定为
`voice_stop`。

`SemanticValidator.validate()` 必须同时接收 proposal 与产生它的原始 immutable planning token；缺失或 context 不匹配
的 proposal 不得进入 Mission。Voice fencing 的 retired-instance capacity 耗尽后 latch fail-closed，直到建立新的
Core/Voice lifetime；latch 期间 COMMAND 一律 reject，STOP 仍走 D-046-003B fast path。

Core 在 STOP 扫描前保留 `，,；;。！!?、` 与 `然后`、`再` 的 clause 边界；单个句末 `。！!?` 只能结束最后一个
非空 clause，重复或内部空 clause 都 reject。称呼与批准 request word 最多消耗一个 approval boundary，
不能吞掉后续重复 separator。完成 collection 后再 classify：只有单独 unknown expression 进入 LLM-needed；
unknown 与 rule 或缺参混合时 deterministic reject。含缺参 clause 的 Turn 在创建 clarification pending 前，先以同一
planning token 和 `SemanticValidator` 验证全部完整 sibling step；失败直接 `REPLY`，不保留该 Turn pending。
rotate angle 使用 Runtime 冻结的 `float32 6.283185F` 作为 wire upper bound；中文 `±360°` 显式映射为对应 binary32
value，外部 proposal 的 wire representation 超出上限则 reject。pending 的创建与覆盖经由 Core private commit seam，
它仅在当前未撤销 token 下重新验证全部完整 sibling 后写回。clarification answer 若 unit 或 result 仍不完整可再次
`CLARIFY`；完整但 range、wire range、numeric 或 Place/Map ID 非法时直接 `REPLY` 并终止 pending，后续裸回答不得
续接。重新验证使用当前 snapshot，因此 capability、Named Place、`max_steps` 或 mode 变化会在再次 clarification 前
deterministic reject。

## 公共 ROS surface

Voice 仅暴露：

```text
/voice/turn   voice_nav_interfaces/msg/VoiceTurn
/voice/speak  voice_nav_interfaces/action/Speak
```

partial ASR、VAD、KWS decision、`10 ms` frame、PCM、device state 与 model-specific token 均为 `voice_node` private
detail。精确 public field、constant、QoS 和 type boundary 见[Voice Interface 契约](voice-interface.md)。

`voice_instance_id` 在每次 Voice start 时变化，`voice_seq` 在 instance 内严格递增；`session_id` 聚合普通
follow-up turn，`turn_id` 标识一条 accepted utterance。STOP turn 以其 `turn_id` 作为幂等
`StopMission.request_id`，因此 Voice 与 Agent 不会创造两个 identity。仅最终 endpointed Mandarin transcript 会发布，
QoS 为 `RELIABLE + VOLATILE + KEEP_LAST(1)`：Voice Turn 是 live work，不是给迟到 process 的 retained authority。

ROS Action cancel、新 PlaybackScope 或 accepted barge-in 通过有界 fade-to-silence drain queued speech。每个 accepted
Speak Goal 获得一个 Result；cancel speech 永不意味着 cancel Mission。

## 交互 scope

Voice 维护三种不同 lifetime：

- **PlaybackScope**：一个 active Speak synthesis/playback generation；
- **TurnScope**：wake、capture、endpointing、ASR 与 resulting turn generation；
- **MissionScope**：由 Mission Runtime 拥有，而非 Voice。

playback 中普通“**小智**” wake 会 cancel 较早的 PlaybackScope 与 TurnScope，再打开新的 TurnScope；它**不会**
cancel 正在运行的 Mission。playback 期间只有 wake word 与固定 STOP phrase 可 interrupt；任意 VAD energy 不足，
因为 speaker echo 会制造 false barge-in。

## 固定 STOP 快路径

固定 phrase：

```text
小智停止
紧急停止
```

接受任一 phrase 时，Voice：

1. 创建 STOP Voice Turn 并固定 `turn_id`；
2. 以 `turn_id` 为 `request_id` 直接调用 `/mission/stop`，不等待 Agent 或 LLM；
3. 发布同一 turn，`kind=STOP`；
4. 将 response 或 timeout 留作 turn-local evidence。

Agent 接收 STOP turn，以同一 `request_id` retry `StopMission` 并生成 spoken reply。Runtime idempotency 使丢失的
Service response 与该有意 retry 安全。该 phrase 是仿真 Operational Stop，不声称认证 emergency-stop recognition。

## Audio ownership 与 real-time boundary

`voice_node` 严格打开一个 **48 kHz、mono、PortAudio full-duplex stream**：

```text
device capture -> callback -> bounded capture SPSC -> DSP worker
device render  <- callback <- bounded playback SPSC <- TTS worker
                           \-> bounded exact-render-reference SPSC
```

每个 SPSC ring 都是 fixed-capacity、preallocated。overflow 和 underflow 增加 lock-free counter，并选择有文档的
bounded fallback，例如丢弃最旧 capture frame 或 render silence；queue 不能增长。

real-time callback 不得：

- allocate/free；
- 获取 blocking lock、wait 或执行 file/network I/O；
- log 或调用 ROS；
- 执行 DSP、model inference 或 dynamic reconfiguration；
- 跨 callback boundary throw。

它只复制 bounded sample、应用已准备的 constant-time output state、更新 lock-free index/counter 并返回。

## 精确 AEC reference 与 DSP 顺序

render reference 是 PortAudio callback **实际写入 device 的 PCM**，在该 callback 中完成每项 application-side
resample、mix、gain、fade、saturation 与 short-buffer truncation 决策后复制。pre-resample TTS PCM、稍后才 fade 的
queued buffer、text 或 audio file 都不是有效 reference。

DSP thread 按以下顺序消费同步的 `10 ms / 480-sample` frame：

```text
1. exact final render-reference frame
2. measured render/capture delay update
3. 48 kHz captured frame through WebRTC APM capture processing
4. cleaned capture resampled to 16 kHz
5. KWS, VAD/endpointing, and streaming ASR
```

device discontinuity、xrun、unrecoverable ring loss 或 stream restart 都 rotate audio generation 并 reset delay/AEC state。
late TTS PCM 不得进入新 PlaybackScope 或成为其 reference。

## 锁定 DSP dependency 与本地模型

Implementation 使用 WebRTC AudioProcessing **2.1** 与一个 compatible Abseil revision，作为一组已审查 dependency。
其 lock manifest 记录精确 upstream revision/version、URL、SHA-256、license、patch、build option 与支持 compiler。
二者在忽略的 `.deps/` 下构建；build 不得静默使用 floating system Abseil 或 Ubuntu 旧
`webrtc-audio-processing` `0.3.1` package。compatible Abseil revision 在 v0.6 dependency Issue 中从
AudioProcessing 2.1 upstream build metadata 选择并验证；v0.1 architecture document 不会杜撰未验证 tag。

每个 model 有审查过的 manifest，记录 immutable source revision、URL、SHA-256、file size、license、sample rate、
runtime version 与固定 Mandarin acceptance corpus。weight 是本地 artifact，永不 commit。默认 policy：

- KWS：`sherpa-onnx-kws-zipformer-zh-en-3M`；
- ASR：先用 `14M Chinese Streaming Zipformer`；若未通过 fixed-corpus gate，自动选 locked
  `2025-06-30 int8` model；
- TTS：`vits-piper-zh_CN-chaowen-medium`；
- LLM：官方 `Qwen3-0.6B-GGUF` `Q8_0`。

model selection 是可复现 policy，不是 online “latest” lookup。runtime 不会静默 download 或 upgrade model。
`llama-server` 是由固定 llama.cpp commit 构建的独立 dependency process：只 bind loopback，加载 locked GGUF，并使用
bounded context、output、concurrency 与 request deadline。v1.0 acceptance 没有 cloud fallback。

## Agent 决策顺序

每个 `COMMAND` turn，Agent 先分配 source sequence，并从 `/mission/state` snapshot `runtime_instance_id` 与
`admission_epoch`。整个 planning attempt 中该 snapshot immutable：

```text
1. STOP classification
2. deterministic Mandarin rules
3. clarification for missing or ambiguous required data
4. local LLM fallback
5. JSON Schema validation
6. local semantic validation
7. typed ExecuteMission submission
```

slow LLM 返回后，Agent 不得以较新 epoch refresh old plan。stale result 用原 snapshot 提交后被 reject，或在本地
discard。

rule 覆盖 closed move、rotate、save-map、Named Place、cancel 与 common dialogue vocabulary。clarification 处理缺失
distance、angle、logical Map ID 或 Named Place。LLM output 只是一项 Agent-internal JSON value，必须通过 closed JSON
Schema 与同一 local semantic policy，才能构造 ROS `MissionStep`。ROS Mission Interface 永不变为 dynamic JSON catalog。

任意 LLM output 都不能提供 Twist、wheel speed、path、raw pose、file path、controller parameter、speed、
acceleration、tolerance 或 timeout。即使 `llama-server` 不可用，deterministic rule 与 fixed STOP 仍可用。

## Queue 与 stale-result 策略

Agent 有一个 pending LLM slot，采用 **latest-turn-wins**：

- 新完成 turn 替换 pending turn；
- active inference 在支持时 receive cancellation，且始终 rotate local generation；
- 具有 stale voice instance、sequence、turn ID、Runtime instance、admission epoch 或 generation 的 output 被 discard；
- STOP 绕过 LLM queue；
- inference slow 或 unavailable 时 queue 仍保持 bounded。

Speak 使用相同 session/turn correlation；新的 PlaybackScope 阻止 older generation 的未播放或 late PCM 到达 device。

## 验证义务

- callback inspection 与 stress test 覆盖 allocation、blocking、logging、ROS call、inference、xrun、overflow、
  underflow 与 silence fallback；
- DSP fixture 验证精确 `480-sample` framing、render-reference ordering、`40–250 ms` delay、drift、reset behavior
  与连续 16 kHz output；
- playback test 证明普通 VAD 不能 interrupt，而 wake 与固定 STOP 可以；
- decision test 证明 STOP 与 deterministic rule 从不调用 LLM；
- Agent test 证明 planning-time epoch snapshot、capacity one、latest-wins、schema reject、semantic reject、timeout
  与 late-result isolation；
- manifest test 在 dependency 或 model load 前验证每个 checksum 与 license；
- real analog audio 与 locked-model metric 仍是[测试策略](../process/testing-strategy.md)规定的 v0.7/v1.0 release evidence。
