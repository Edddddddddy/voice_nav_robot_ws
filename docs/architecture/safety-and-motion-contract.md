# Safety and motion contract

**Status:** Target v1.0 contract

VoiceNav Robot provides a deterministic, fail-closed **Operational Stop** for
the supported simulation environment. It does not claim a hardware emergency
stop or functional-safety certification.

## Trust and ownership

| Source or process | May propose motion | May publish final controller velocity |
| --- | --- | --- |
| KWS, ASR, local LLM | yes, untrusted intent | no |
| deterministic Agent rules | yes, semantic steps | no |
| `mission_runtime_node` | yes, after whole-plan admission | no |
| Nav2 or relative-motion executor | yes, for the active generation | no |
| `nav2_velocity_smoother` | conditions the active candidate | no |
| `nav2_collision_monitor` | filters the conditioned candidate | no |
| `motion_gate_node` | enforces the active lease and limits | **yes, sole publisher** |
| `diff_drive_controller` | consumes the final command | no |

The self-written runtime processes are exactly:

```text
voice_node
agent_node
mission_runtime_node
motion_gate_node
```

`mission_runtime_node` and `motion_gate_node` are separate processes. The
separate failure domains let the Gate expire its steady-clock lease if Runtime
crashes, stalls, or loses executor progress.

## Fixed target chain

```text
Nav2 or relative-motion candidate
  -> MissionRuntime generation filter
  -> nav2_velocity_smoother
  -> nav2_collision_monitor
  -> independent motion_gate_node
  -> diff_drive_controller
  -> gz_ros2_control / Gazebo wheel joints
```

The complete chain uses `geometry_msgs/msg/TwistStamped`. The pinned Jazzy
`diff_drive_controller` Interface intrinsically subscribes to that type; it
has no `enable_stamped_cmd_vel` switch. Configuration and contract tests reject
that fictitious parameter and the obsolete demo-only `use_stamped_vel` key.

There is no `twist_mux`. The single Mission execution slot already determines
the active source, so a second priority and ownership model would be
ambiguous. The velocity smoother only conditions acceleration and velocity.
Collision Monitor is a protective collision-avoidance layer, not a certified
emergency-stop system. Neither replaces MotionGate.

`diff_drive_controller` enforces the final hard linear/angular velocity bounds,
but its acceleration and deceleration limit parameters remain unset. Applying
a second acceleration limiter there would make `cmd_vel_timeout` begin a ramp
instead of selecting zero on the first controller update after expiry.
`nav2_velocity_smoother` owns normal acceleration shaping upstream; command
zero latency and physical stationarity remain separate measurements.

`ros_gz_bridge` bridges only `/clock` and `/scan` in the target product path.
Velocity commands, joint state, odometry, and TF remain in ROS 2 control.

## MotionGate contract

- MotionGate starts inhibited and cannot restore an earlier lease after a
  restart.
- It is the only publisher to the controller's final command endpoint.
- Runtime filters child callbacks by Runtime instance, admission epoch,
  Mission generation, and step generation **before** the smoother. These
  identities are not present in `TwistStamped`, so MotionGate does not pretend
  to recover or validate them from candidate messages.
- MotionGate generates the opaque authority `lease_id` and per-lease candidate
  topic during `PREPARE`; callers cannot supply arbitrary IDs or paths.
  Runtime opens and renews that lease through the private control seam. The
  authority lease is **250 ms on MotionGate's steady clock**; velocity
  candidates never renew it.
- Every control operation uses one Gate-wide compare-and-swap `control_seq`.
  `OPEN`, `RENEW`, and `INHIBIT` also match the current Gate instance and
  lease. A stale request has no state effect, including a late old-lease
  `INHIBIT` racing a newer lease. An expired or revoked lease cannot be
  resurrected; obtaining authority again requires the complete handover
  protocol below.
- Candidate freshness is a second, independent steady-clock deadline. A
  non-zero output requires both live Runtime authority and a fresh candidate
  from the currently bound data plane. Freshness expires at **150 ms** of
  MotionGate steady time.
- Finite `linear.x` and `angular.z` values outside trusted YAML bounds are
  clamped to those bounds. NaN, Inf, a non-zero unsupported axis, stale input,
  or a sample from an unbound writer/topic generation retires the lease and
  selects zero.
- Authority or candidate expiry inhibits motion and continuously publishes
  zero every **20 ms wall time** without waiting for Runtime, ROS time, Nav2,
  or Gazebo time.
- A matching current `INHIBIT` selects and publishes zero before it
  acknowledges the control request.
- `diff_drive_controller.cmd_vel_timeout` is **0.35 s**. It is the
  consumer-side second deadman if MotionGate itself dies.
- Runtime crash, cancel, timeout, dependency loss, invalid generation, and
  Gate fault all fail closed.
- A Gate health or zero-output state that cannot be established produces
  `SAFETY_FAULT`; Runtime admits no new Mission while faulted.
- Runtime renews authority only while the active step, Gate, `/clock`, odometry,
  and required dependencies are fresh according to steady-clock liveness
  checks. Losing any prerequisite stops renewal and terminates the old lease;
  resuming a dependency never reopens it.

WSL is not a real-time environment. The 250 ms and 0.35 s values are tested
budgets for this supported environment, not hard real-time guarantees.

### Package-private control and state seam

The private ROS types live in `voice_nav_mission`, not
`voice_nav_interfaces`:

```text
voice_nav_mission/srv/InternalMotionGateControl
voice_nav_mission/msg/InternalMotionGateState
```

The node FQN is `/motion_gate_node`; the private absolute endpoints are
`/motion_gate/internal/control` and `/motion_gate/internal/state`. PREPARE
returns a bounded topic below
`/voice_nav_internal/motion_gate/candidate/lease_`. These names and the final
`/diff_drive_controller/cmd_vel` endpoint are code constants, not YAML
parameters or product-launch remaps. Trusted parameter YAML uses the exact root
`motion_gate_node`.

The only operations are `PREPARE`, `OPEN`, `RENEW`, and `INHIBIT`.
`PREPARE` matches the current Gate instance and expected global
`control_seq`; the other operations additionally match the current lease.
Each accepted operation advances the single Gate-wide sequence. A stale
instance, lease, or sequence returns a bounded typed mismatch without changing
Gate state. Public `StopMission` remains unconditionally safety-effective at
the Mission boundary because Runtime first linearizes STOP and then inhibits
the **current** Gate tuple; an arbitrary private stale `INHIBIT` is not STOP.

`InternalMotionGateControl` contains no writer GID. At `OPEN`, MotionGate uses
its own graph context to require exactly one publisher endpoint and records
that endpoint's complete 16-byte GID. Candidate callbacks compare it only with
the `MessageInfo.publisher_gid` observed in the same Gate context. A
locked-Fast-DDS self-test proves that those two Gate-local representations
correlate; failure or mismatch keeps the Gate inhibited. A caller's
`Publisher::get_gid()` is neither transported nor compared across processes.

The state snapshot uses
`RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)` and reports the Gate instance,
global sequence, `INHIBITED`/`PREPARED`/`ARMED`/`FAULTED` state, current lease
and topic, validity flags, output sequence/zero state, bounded reason, and an
optional fixed 16-byte bound GID for run-local diagnosis only. Package-private
types and obscure topic names reduce the supported Interface surface; they
are not DDS authentication or authorization.

Candidate input uses `BEST_EFFORT + VOLATILE + KEEP_LAST(1)`. The final Gate
publisher to `/diff_drive_controller/cmd_vel` uses
`rclcpp::SystemDefaultsQoS()` to match the pinned controller's subscriber.
Runtime graph checks prove actual endpoint compatibility and unique ownership;
an introspected reliability, history, or depth reported as `UNKNOWN` is not
hard-coded into a false assertion.

Control, candidate, expiry, state, and output decisions cross one publication
serial barrier. Callbacks never publish directly. Once current-lease
`INHIBIT`, expiry, or invalid-input retirement publishes zero through that
barrier, an earlier queued non-zero decision cannot publish afterward.

### Authority and candidate handover barrier

`TwistStamped` deliberately remains the velocity type across the complete
chain; adding Mission metadata to a controller command would couple the motion
conditioners to Mission internals. That choice requires an explicit barrier so
an old command buffered by the smoother, Collision Monitor, or DDS cannot be
mistaken for a command from a newly admitted step.

Every initial arm, source change, step change, expired lease, cancel recovery,
STOP recovery, or Runtime restart follows this internal protocol:

```text
revoke old authority, inhibit Gate, and select/publish zero
  -> stop the old producer and cancel its child operation
  -> fully unload/destroy the old smoother and Collision Monitor instances
  -> destroy Gate's old candidate subscription
  -> confirm the old output writer GID has disappeared from the ROS graph
  -> PREPARE; Gate generates a new lease ID and per-lease candidate topic
  -> Gate may create a provisional reader that discards every sample
  -> create/configure new Collision Monitor and smoother downstream-to-upstream
  -> require exactly one writer in the Gate-local graph snapshot
  -> OPEN serial point destroys the provisional reader and its queued samples
  -> recreate the VOLATILE KEEP_LAST(1) reader and bind the Gate-observed GID
  -> activate Collision Monitor, then smoother, while Gate remains inhibited
  -> complete OPEN and enter ARMED with a 250 ms Runtime authority lease
  -> start the new producer last
```

At the OPEN serial point, destroying and recreating the reader is a queue
barrier: samples received or queued by the provisional reader can never become
valid after the state reaches `ARMED`. Gate callbacks accept only the writer
bound during this handover on the new per-lease channel. An old sample retains
its old channel or Gate-local writer identity and remains invalid even if DDS
delivers it after the new lease opens.

A lifecycle deactivate/cleanup/configure cycle is not sufficient: the pinned
Nav2 1.3.12
[Velocity Smoother cleanup](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_velocity_smoother/src/velocity_smoother.cpp#L199-L206)
does not clear all cached command state. The supported implementation fully
unloads and recreates both components and the Gate reader. A quiet window may
be recorded for diagnostics, but it is not the isolation proof.

The Gate records the opaque lease ID for control-request idempotence, but the
ID is not carried in `TwistStamped`. While inhibited or waiting for handover,
every candidate is discarded. A failed unload, graph disappearance, new-writer
binding, activation, or acknowledgement keeps the Gate inhibited and becomes
`SAFETY_FAULT`.

The barrier is required between consecutive Mission steps too, even when both
steps use the same producer. Limits, timeouts, and queue bounds come from
trusted configuration and are verified with deliberately delayed old commands.

Lesson 0009 implements and verifies the normal-running Core, private seam,
Gate-local binding, barriers, final ownership, and deadline expiry with a test
authority/candidate harness. It does not claim the complete Runtime/smoother/
Collision Monitor integration. Process-kill crash-stop and managed/unmanaged
Gazebo pause behavior are reserved for Lesson 0010 / VN-0011.

### Gazebo managed safe-pause and resume

The controller timeout alone cannot prove “no replay” across a pause:
Jazzy `diff_drive_controller`
[computes command age from controller time and the Twist stamp](https://github.com/ros-controls/ros2_controllers/blob/jazzy/diff_drive_controller/src/diff_drive_controller.cpp#L94-L116).
When simulation time is stopped, its 0.35 s timeout does not advance.

The supported product and test path therefore uses a two-phase safe-pause
transaction while simulation updates are still advancing:

1. reject new Missions, stop renewing Runtime authority, inhibit MotionGate,
   and receive its zero-output acknowledgement;
2. observe `diff_drive_controller`'s limited command output and the wheel
   command/velocity state at zero for a configured number of complete control
   periods;
3. if Gate fails before that proof, let the consumer timeout or deactivate the
   controller while the update loop is still advancing, but still require
   direct observation of zero wheel command for the configured control
   periods; inactive or released interfaces alone are not zero proof;
4. only after the zero proof pause Gazebo and record an opaque safe-pause token
   containing the world iteration and Gate/controller instance state.

This is an operational `voice_nav_bringup`/test-harness transaction, not a
public Mission pause endpoint and not a fifth self-written resident process.
If zero cannot be observed by the bounded pause deadline, no token is minted
and the harness terminates/restarts simulation and control from a known zero
state instead of freezing an unproved command.

Managed resume requires that token and verifies the recorded controller state
has not changed. The wheel command is therefore already proven zero before the
first resumed `PreUpdate`, whether the controller remains active or was also
deactivated. A deactivated controller is activated later only after an
inhibited Gate and zero path are healthy. The first resumed wheel command is
explicitly tested as zero.

A direct GUI or Gazebo Transport pause taken before this barrier has no
safe-pause token. Managed resume refuses to unpause it in place; the supported
recovery is a full simulation/control restart from a known inactive,
zero-command state. This is required because paused `gz_ros2_control` cannot
process a controller switch or consume a newly buffered zero before the first
resumed write. The project does not claim arbitrary external pause/resume as a
functional-safety mechanism.

## StopMission ownership and ordering

`mission_runtime_node` owns the public `StopMission.srv` endpoint because
STOP, cancel, success, timeout, and downstream completion must pass through
one serial linearization point. MotionGate exposes only an internal control
seam to Runtime; it is not a second public product-control API.

A new Stop request always has safety effect, even when its source sequence is
old. `request_id` makes retries idempotent; it does not grant permission to
ignore a stop.

```text
deduplicate request_id
  -> first terminal-intent linearization
  -> rotate admission_epoch for a new request
  -> inhibit MotionGate and select/publish zero
  -> cancel the active downstream executor
  -> commit the active Mission's STOPPED result when STOP won
  -> return StopMission response
```

The response is sent only after MotionGate reports inhibited and has published
zero. `motion_inhibited=true` does **not** claim that simulated inertia has
already brought the robot physically to rest.

Repeating the same `request_id` returns the current response without rotating
the epoch again. A later STOP still rotates the global epoch and inhibits
motion even if another terminal intent already won for the active Goal; it
does not rewrite that Goal's historical result.

Operational Stop is not pause. Only a newly planned and fully validated
Mission carrying the latest Runtime instance and admission epoch can obtain a
new lease.

## Cancel, STOP, and completion races

All terminal intents use first-terminal-intent-wins:

- **Cancel wins:** Gate zero, downstream cancel, then outer `CANCELED` and
  inner `CANCELED`. A later STOP still rotates the epoch and keeps motion
  inhibited.
- **STOP wins:** rotate epoch, Gate zero, downstream cancel, then outer
  `ABORTED` and inner `STOPPED`. A later cancel cannot rewrite the result.
- **Success wins:** publish final zero and commit `SUCCEEDED`. A later STOP
  changes current global admission only, not history.
- **Timeout or failure wins:** Gate zero, cancel downstream, and commit its one
  structured failure result.

Late Nav2, relative-motion, map, and timer callbacks are discarded by Runtime
instance, admission epoch, Mission generation, and step generation. Late
velocity samples are isolated by the candidate handover barrier described
above. Every Goal receives exactly one terminal result.

## Relative motion

- MOVE projects odometry displacement onto the signed initial-heading axis.
- ROTATE unwraps yaw before comparing signed angular displacement.
- Both use trusted YAML for speed, acceleration, tolerance, stall thresholds,
  and a policy-computed deadline.
- Both slow down near the target and publish zero before completing.
- Step deadlines, stall windows, lease expiry, and cancel grace use a steady
  clock. ROS time remains for odometry and TF timestamps only.

## Failure behavior

| Failure | Required behavior |
| --- | --- |
| Invalid, stale, or oversized Mission | reject before any execution side effect |
| Second Mission while busy | return `BUSY`; no queue or implicit preemption |
| Dependency unavailable during whole-plan validation | reject before an earlier step starts |
| Candidate stale or authority lease expired | Gate inhibits, latches the old lease closed, and continuously publishes zero |
| Runtime disappears | Gate lease expires independently |
| MotionGate disappears while simulation advances | controller consumes no fresh command and times out within 0.35 s of advancing simulation |
| MotionGate disappears after a managed safe-pause | the wheel command was directly proven zero before the token was issued; token-checked resume preserves it |
| Gazebo safe-pause and managed resume | first resumed wheel command is zero; stale non-zero command is never replayed |
| Direct external pause without a safe-pause token | in-place resume is refused; restart simulation/control from a known zero state |
| Nav2 abort or step deadline | Gate zero, cancel child, fail step, skip remainder |
| Map save partial failure | publish no completed logical map directory |
| Late callback after cancel | discard callback by epoch/generation; discard velocity through the inhibited handover barrier |
| Gate health or zero proof unavailable | report `SAFETY_FAULT` and remain fail-closed |

## Verification obligations

- Manual-clock tests prove lease, cancel-grace, timeout, and callback-fencing
  behavior without sleeping.
- Runtime-death tests keep injecting valid-looking candidates and prove they
  cannot renew the independent authority lease.
- Stop tests assert `EPOCH -> INHIBIT/ZERO -> CANCEL -> RESPONSE`, plus
  idempotent `request_id` behavior.
- Runtime tests cover cancel/STOP/success races and exactly-one Result.
- Process-death tests kill Runtime and MotionGate separately to prove both
  deadman layers.
- Managed safe-pause/resume tests prove that an old non-zero command cannot
  resume motion within the documented boundary.
- Handover tests inject old-writer candidates before, during, and after full
  pipeline recreation and prove that channel/GID binding rejects all of them.
- Pause tests request safe-pause while moving, prove the controller and wheel
  command reach zero before `/clock` stops, kill MotionGate while paused, and
  assert the first resumed wheel command is zero.
- Pause tests also kill MotionGate before zero proof; interface release or
  controller inactivity never mints a token without observed zero, and a
  failed bounded proof selects full restart.
- Unmanaged-pause tests prove that a missing or mismatched safe-pause token
  refuses in-place resume and selects the full-restart recovery path.
- Odometry tests distinguish command-zero latency from physical stationarity.
- No test exits while either Gate or controller can retain an authorized
  non-zero command.
