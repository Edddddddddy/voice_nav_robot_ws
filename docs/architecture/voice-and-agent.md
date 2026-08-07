# Voice and Agent contract

**Status:** Target v1.0 contract

`voice_node` owns the full-duplex device, real-time audio boundary, AEC, KWS,
VAD, ASR, TTS, playback, and barge-in. `agent_node` owns deterministic command
rules, clarification, constrained local-LLM fallback, and Mission submission.
Neither process has final motion authority.

## Issue #46 Agent Core boundary

`voice_nav_agent` 的 Core 是无 ROS I/O、无 HTTP、可注入 steady clock 的纯
Python Module。它的行为 seam 是一次 `handle_turn(VoiceTurn,
MissionState-or-none)` 和一个共享的 `SemanticValidator`；Normalizer、封闭
规则解析、澄清表与 Voice sequence fencing 都留在 Core 内部。Core 输出封闭的
`MISSION`、`CANCEL`、`STOP`、`CLARIFY`、`REPLY`、`LLM_NEEDED` 或 `IGNORE`
Decision，未来 LLM proposal 与规则 proposal 必须经过同一 Validator。

非 STOP 的 Mission/LLM planning token 在 turn 开始时固定 Agent source
identity、Voice turn identity、generation 与 Runtime ID/epoch/mode/capability/
Named Place 快照；不会在规划过程中刷新。STOP 遵循 D-046-003B：request ID
等于 `turn_id`，source instance/sequence 直接复用 Voice Turn 的
`voice_instance_id`/`voice_seq`，reason 固定为 `voice_stop`。

## Public ROS surface

Voice exposes only:

```text
/voice/turn   voice_nav_interfaces/msg/VoiceTurn
/voice/speak  voice_nav_interfaces/action/Speak
```

Partial ASR, VAD, KWS decisions, 10 ms frames, PCM, device state, and
model-specific tokens remain private to `voice_node`.

### `VoiceTurn.msg`

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

`voice_instance_id` changes whenever Voice starts. `voice_seq` strictly
increases within that instance. `session_id` groups ordinary follow-up turns;
`turn_id` identifies one accepted utterance. A STOP turn uses its `turn_id` as
the idempotent `StopMission.request_id`, so Voice and Agent can make the same
request without inventing two identities.

Only a final endpointed Mandarin transcript is published. QoS is
`RELIABLE + VOLATILE + KEEP_LAST(1)`: a Voice Turn is live work, not retained
authority for a process that joins later.

### `Speak.action`

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

ROS Action cancel, a newer PlaybackScope, or an accepted barge-in drains
queued speech through a bounded fade to silence. Every accepted Speak Goal
receives one Result. Canceling speech never implies Mission cancel.

## Interaction scopes

Voice maintains three distinct lifetimes:

- **PlaybackScope**: one active Speak synthesis/playback generation;
- **TurnScope**: wake, capture, endpointing, ASR, and the resulting turn
  generation;
- **MissionScope**: owned by Mission Runtime, not by Voice.

An ordinary “小智” wake during playback cancels the older PlaybackScope and
TurnScope, then opens a new TurnScope. It does **not** cancel the running
Mission. During playback, only the wake word and the fixed STOP phrases are
allowed to interrupt; arbitrary VAD energy is not sufficient because speaker
echo would create false barge-in.

## Fixed STOP fast path

The fixed phrases are:

```text
小智停止
紧急停止
```

When either phrase is accepted, Voice:

1. creates the STOP Voice Turn and fixes its `turn_id`;
2. calls `/mission/stop` directly using `turn_id` as `request_id`, without
   waiting for Agent or LLM;
3. publishes the same turn as `kind=STOP`;
4. retains the response or timeout as turn-local evidence.

Agent receives the STOP turn, retries `StopMission` with the same
`request_id`, and produces the spoken reply. Runtime idempotency makes a lost
Service response or this intentional retry safe. The phrase is an operational
simulation stop, not a claim of certified emergency-stop recognition.

## Audio ownership and real-time boundary

`voice_node` opens exactly one **48 kHz, mono, PortAudio full-duplex stream**:

```text
device capture -> callback -> bounded capture SPSC -> DSP worker
device render  <- callback <- bounded playback SPSC <- TTS worker
                           \-> bounded exact-render-reference SPSC
```

Every SPSC ring is fixed-capacity and preallocated. Overflow and underflow
increment lock-free counters and choose a documented bounded fallback, such as
dropping the oldest capture frame or rendering silence. Queues cannot grow.

The real-time callback must not:

- allocate or free;
- acquire a blocking lock, wait, or perform file/network I/O;
- log or call ROS;
- run DSP, model inference, or dynamic reconfiguration;
- throw across the callback boundary.

It only copies bounded samples, applies already-prepared constant-time output
state, updates lock-free indices/counters, and returns.

## Exact AEC reference and DSP order

The render reference is the PCM that the PortAudio callback **actually writes
to the device**, copied in that callback after every application-side
resample, mix, gain, fade, saturation, and short-buffer truncation decision.
Pre-resample TTS PCM, a queued buffer before a later fade, text, or an audio
file is not a valid reference.

The DSP thread consumes synchronized 10 ms / 480-sample frames in this order:

```text
1. exact final render-reference frame
2. measured render/capture delay update
3. 48 kHz captured frame through WebRTC APM capture processing
4. cleaned capture resampled to 16 kHz
5. KWS, VAD/endpointing, and streaming ASR
```

Device discontinuity, xrun, unrecoverable ring loss, or stream restart rotates
the audio generation and resets delay/AEC state. No late TTS PCM may enter a
new PlaybackScope or become its reference.

## Locked DSP dependency

The implementation uses WebRTC AudioProcessing **2.1** and one compatible
Abseil revision as a single reviewed dependency set. Its lock manifest records
the exact upstream revision/version, URL, SHA-256, license, patches, build
options, and supported compiler. Both are built below ignored `.deps/`; the
build may not silently use a floating system Abseil or Ubuntu's old
`webrtc-audio-processing` 0.3.1 package.

The concrete compatible Abseil revision is selected and verified in the v0.6
dependency Issue from AudioProcessing 2.1's upstream build metadata; the v0.1
architecture document does not fabricate an unverified tag.

## Locked local models

Each model has a reviewed manifest with immutable source revision, URL,
SHA-256, file sizes, license, sample rate, runtime version, and fixed Mandarin
acceptance corpus. Weights are local artifacts and are never committed.

The default model policy is:

- KWS: `sherpa-onnx-kws-zipformer-zh-en-3M`;
- ASR: the 14M Chinese Streaming Zipformer first; if it misses the fixed-corpus
  gate, automatically select the locked 2025-06-30 int8 model;
- TTS: `vits-piper-zh_CN-chaowen-medium`;
- LLM: official `Qwen3-0.6B-GGUF` `Q8_0`.

Model selection is reproducible policy, not an online “latest” lookup. No
model is silently downloaded or upgraded at runtime.

`llama-server` is an independent dependency process built from a fixed
llama.cpp commit. It binds only to loopback, loads the locked GGUF, and uses
bounded context, output, concurrency, and request deadlines. There is no cloud
fallback in v1.0 acceptance.

## Agent decision order

For every `COMMAND` turn, Agent first allocates its source sequence and
snapshots `runtime_instance_id` plus `admission_epoch` from `/mission/state`.
That snapshot is immutable for the entire planning attempt:

```text
1. STOP classification
2. deterministic Mandarin rules
3. clarification for missing or ambiguous required data
4. local LLM fallback
5. JSON Schema validation
6. local semantic validation
7. typed ExecuteMission submission
```

Agent must not refresh an old plan with a newer epoch after a slow LLM
returns. A stale result is submitted with its original snapshot and rejected,
or discarded locally.

Rules cover the closed move, rotate, save-map, Named Place, cancel, and common
dialogue vocabulary. Clarification handles a missing distance, angle, logical
Map ID, or Named Place. LLM output is only an Agent-internal JSON value. It
must pass a closed JSON Schema and the same local semantic policy before
construction of ROS `MissionStep` values. The ROS Mission Interface never
becomes a dynamic JSON catalog.

Arbitrary LLM output cannot provide Twist, wheel speed, path, raw pose, file
path, controller parameter, speed, acceleration, tolerance, or timeout.
Deterministic rules and fixed STOP remain available when `llama-server` is
down.

## Queue and stale-result policy

Agent has one pending LLM slot and uses **latest-turn-wins**:

- a newer completed turn replaces the pending turn;
- active inference receives cancellation where supported and always rotates
  its local generation;
- output with stale voice instance, sequence, turn ID, Runtime instance,
  admission epoch, or generation is discarded;
- STOP bypasses the LLM queue;
- the queue remains bounded while inference is slow or unavailable.

Speak uses the same session/turn correlation. A newer PlaybackScope prevents
unplayed or late PCM from an older generation from reaching the device.

## Verification obligations

- Callback inspection and stress tests cover allocation, blocking, logging,
  ROS calls, inference, xrun, overflow, underflow, and silence fallback.
- DSP fixtures verify exact 480-sample framing, render-reference ordering,
  40–250 ms delay, drift, reset behavior, and continuous 16 kHz output.
- Playback tests prove ordinary VAD cannot interrupt, while wake and fixed STOP
  can.
- Decision tests prove STOP and deterministic rules never call the LLM.
- Agent tests prove planning-time epoch snapshot, capacity one, latest-wins,
  schema rejection, semantic rejection, timeout, and late-result isolation.
- Manifest tests verify every checksum and license before a dependency or
  model loads.
- Real analog audio and locked-model metrics remain v0.7/v1.0 release evidence
  as specified by the testing strategy.
