# Mission Runtime Interface

**Status:** Approved v1.0 target; the v0.1 source IDL is still provisional.

Mission Runtime is a deep Module with two mutation operations and one read-only
state projection:

```text
/mission/execute  voice_nav_interfaces/action/ExecuteMission
/mission/stop     voice_nav_interfaces/srv/StopMission
/mission/state    voice_nav_interfaces/msg/MissionState
```

Action cancel belongs to execution. There is no public queue, validate,
execute-step, pause, resume, raw-pose, or raw-velocity operation.

## `MissionStep.msg`

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

The message is a closed discriminated union enforced at the trust boundary:

| Kind | Required payload | Every unused field |
| --- | --- | --- |
| `MOVE_DISTANCE` | finite, non-zero `distance_m` | zero or empty |
| `ROTATE_ANGLE` | finite, non-zero `angle_rad` | zero or empty |
| `NAVIGATE_TO` | known Named Place `target_id` | zero or empty |
| `SAVE_MAP` | valid logical Map ID in `target_id` | zero or empty |

Unknown kinds, NaN, infinity, unused payload, out-of-policy values, and invalid
IDs are rejected. A Map ID is never a path. Velocity, acceleration, tolerance,
deadline, retry, and controller parameters come only from trusted YAML.

## `MissionState.msg`

```text
uint8 MAPPING=1
uint8 NAVIGATION=2

uint8 UNAVAILABLE=0
uint8 AVAILABLE=1
uint8 BUSY=2
uint8 FAULTED=3

uint8 GATE_INHIBITED=0
uint8 GATE_ARMED=1
uint8 GATE_FAULTED=2

string<=36 runtime_instance_id
uint64 admission_epoch
uint8 operating_mode
uint8 availability
uint8 gate_state
uint32 active_step
uint32 supported_step_mask
uint8 max_steps
string<=64[<=32] named_place_ids
```

QoS is `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`. A late Agent receives the
latest state before planning. `runtime_instance_id` changes on every
`mission_runtime_node` start. A new STOP, admission-policy change, or Named
Place change rotates `admission_epoch`.

The Agent snapshots Runtime ID and epoch before starting deterministic or LLM
planning. It must send that same snapshot with the resulting Goal; refreshing
the token after a slow LLM returns would defeat stale-plan fencing.

## `ExecuteMission.action`

```text
# Goal
string<=36 source_instance_id
uint64 source_seq
string<=36 runtime_instance_id
uint64 admission_epoch
MissionStep[<=3] steps
---
# Result
uint16 SUCCEEDED=0
uint16 INVALID_PLAN=10
uint16 BUSY=11
uint16 MODE_MISMATCH=12
uint16 UNKNOWN_TARGET=13
uint16 STALE_REQUEST=14
uint16 UNSUPPORTED_STEP=15
uint16 DEPENDENCY_UNAVAILABLE=20
uint16 EXECUTION_FAILED=21
uint16 TIMEOUT=22
uint16 CANCELED=30
uint16 STOPPED=31
uint16 SAFETY_FAULT=32
uint16 INTERNAL_ERROR=99

uint16 code
int32 failed_step
string<=160 detail
---
# Feedback
uint8 VALIDATING=1
uint8 EXECUTING=2
uint8 SAFE_STOPPING=3

uint8 phase
uint32 step_index
float32 progress
```

`source_instance_id` changes when the producer process restarts and
`source_seq` strictly increases within it. Session and Voice Turn IDs remain in
`agent_node`; they do not leak into the Mission domain.

Feedback is advisory. `step_index` never decreases and `progress` is a
non-decreasing best estimate in `[0, 1]`, not a deadline promise. Callers branch
on Result code, never diagnostic text. `failed_step=-1` means no step began.

Every wire-valid Goal is accepted at the ROS Action transport layer. Business
rejections such as invalid, stale, busy, unsupported, or wrong-mode plans
finish as `ABORTED` with their structured Result. Transport rejection is
reserved for shutdown or an unusable Action server.

## `StopMission.srv`

```text
# Request
string<=36 request_id
string<=36 source_instance_id
uint64 source_seq
string<=160 reason
---
# Response
uint16 APPLIED=0
uint16 DUPLICATE=1
uint16 SAFETY_FAULT=2

uint16 code
string<=36 runtime_instance_id
uint64 admission_epoch
bool motion_inhibited
string<=160 detail
```

The behavior is called **Operational Stop**; the ROS type remains exactly
`StopMission.srv`. `mission_runtime_node` serves `/mission/stop` so STOP,
cancel, success, timeout, and dependency completion pass through the same
terminal-intent linearization point. Runtime synchronously controls the
separate Gate through a package-private seam.

A new request unconditionally rotates the epoch, inhibits the Gate, publishes
zero, and cancels the downstream operation. A retry with the same `request_id`
returns the cached logical outcome and does not rotate state again. Stale
source metadata cannot prevent STOP from taking effect.

The Service returns only after the Gate is inhibited and zero has been
published. `motion_inhibited=true` does not claim that simulated mass has
physically stopped; odometry proves stationarity separately.

## Admission and execution invariants

- A plan contains one to three steps and is completely validated before its
  first motion or map-write side effect.
- Atomic validation is not rollback: completed physical motion is not undone
  after a later execution failure.
- Exactly one Mission owns the execution slot. There is no hidden queue and a
  second Goal returns `BUSY`.
- Runtime/source identity and epoch checks precede all dependency calls.
- Mapping accepts move, rotate, and save-map. Navigation accepts move, rotate,
  and navigate-to.
- Named Places, limits, mode, Gate health, and downstream readiness are one
  immutable admission snapshot.
- Steps execute strictly in order; the first failure skips the remainder.
- Typed dependency callbacks are bound to Runtime, epoch, Mission, and step
  generation and cannot advance a newer Mission. Raw `TwistStamped` samples do
  not carry those identities; their isolation uses a recreated per-lease data
  plane and writer-GID binding.
- Only Runtime's private control heartbeat renews MotionGate authority. No
  dependency callback or velocity sample can renew, reopen, or resurrect a
  lease.
- Timeout, cancel, STOP, dependency loss, exception, and success all pass
  through one serial terminal-intent linearization point.
- Every Goal produces exactly one Result.

## Terminal ordering and races

First terminal intent wins the historical Result:

- cancel first: Gate zero, downstream cancel, then `CANCELED`; a later STOP
  still rotates the global epoch but does not rewrite that Result;
- STOP first: epoch rotation, Gate zero, downstream cancel, then
  `ABORTED/STOPPED`; a later cancel cannot rewrite it;
- natural success first: `SUCCEEDED`; a later STOP changes current global
  authority but not completed history.

The common safe-stop sequence is:

```text
select terminal intent
→ inhibit MotionGate and publish zero
→ invalidate child generation
→ cancel or abandon downstream operation
→ wait for bounded acknowledgement or cleanup grace
→ commit exactly one Result
```

Failure to prove an inhibited Gate returns `SAFETY_FAULT` and keeps Runtime
unavailable.

Mission deadlines, cancel grace, Gate lease, STOP barrier, and liveness use a
steady clock. ROS time is used only for TF, sensor data, SLAM, Nav2, and
simulation. Pausing or rewinding `/clock` cannot preserve an old lease.

## Motion and map semantics

`MOVE_DISTANCE` closes a feedback loop on signed odometry projection along the
initial heading. `ROTATE_ANGLE` unwraps yaw before closed-loop control. Both use
trusted slowdown, tolerance, stall, and deadline policies.

`NAVIGATE_TO` resolves a Named Place inside the trusted navigation Adapter.
`SAVE_MAP` writes occupancy YAML, image, and posegraph into a temporary
directory, verifies the complete set, and atomically renames the directory.
Overwrite is rejected by default and caller text never becomes a path.

## Internal seams and fakes

Private Interfaces remain inside `voice_nav_mission`:

```text
RelativeMotionPort
NavigationPort
MapStorePort
MotionAuthorityPort
MotionObserverPort
SteadyClockPort
```

Production Adapters wrap odometry control, Nav2, atomic map storage, the
independent MotionGate, odometry observation, and `std::chrono::steady_clock`.
Scripted fakes inject success, abort, timeout, process restart, Gate loss,
partial map, delayed cancel, and late result. Guard, FSM, and Named Place
policy remain ordinary private implementation; v1 does not add `pluginlib`, a
generic workflow DSL, or a Mission-level Behavior Tree.

## Current-to-target migration

At v0.1 the source IDL is intentionally provisional: it still has session/turn
fields, unbounded strings/steps, no Runtime fencing, no STOP Service, and no
state snapshot. The v0.4 Issue updates IDL, all producers/consumers,
contract tests, documentation, and changelog together. After v1.0, a breaking
DDS change creates V2 types/endpoints plus a temporary V1 Adapter; an
`api_version` field cannot make incompatible DDS types compatible.
