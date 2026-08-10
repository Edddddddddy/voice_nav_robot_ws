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
- IDL bounds `request_id`, `gate_instance_id`, and `lease_id` at 36 characters
  for transport. The Core requires request and Gate identities to be exactly
  32 lowercase hexadecimal characters; PREPARE must carry an empty lease field,
  while OPEN/RENEW/INHIBIT require an exact 32-lowercase-hex lease. Uppercase,
  hyphenated UUID text, short values, and non-hexadecimal text are invalid.
- Every control operation uses one Gate-wide compare-and-swap `control_seq`.
  `OPEN`, `RENEW`, and `INHIBIT` also match the current Gate instance and
  lease. A stale request has no state effect, including a late old-lease
  `INHIBIT` racing a newer lease. An expired or revoked lease cannot be
  resurrected; obtaining authority again requires the complete handover
  protocol below.
- Candidate freshness is a second, independent steady-clock deadline. A
  non-zero output requires both live Runtime authority and a fresh candidate
  from the currently bound data plane. Freshness expires at **150 ms** of
  MotionGate steady time. Before the first candidate of a newly opened lease,
  a successful activation RENEW restarts only this bounded first-sample
  window; once any candidate is accepted, RENEW never extends candidate
  freshness. Candidate samples never extend the independent authority lease.
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

`motion_gate_core` is a package-internal static build target. Its header and
library are neither installed nor exported; `motion_gate_node` is the only
installed runtime target owned by this MotionGate submodule. The same package
also installs the current Mission control-plane target `mission_runtime_node`,
whose public endpoints and unavailable production motion boundary are defined
in `mission-runtime-interface.md`. The Core Interface is the typed
`prepare`/`open`/`renew`/`inhibit`/`accept_candidate`/`tick`/`snapshot`
surface plus the read-only `selected_command`. Adapter-only `force_fault`
latches graph, reader, clock, or publication failures into fail-closed state;
it is not a fifth control operation.
`PrepareAdmissionProvider` and `OpenBindingProvider` are internal seams that
let the ROS Adapter supply bounded graph facts without moving ROS graph access
into the Core.

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
Request and Gate-instance identities, plus every non-PREPARE lease identity,
have the exact 32-character lowercase hexadecimal semantic described above
even though their IDL fields have a 36-character transport bound. PREPARE must
not carry a lease ID.

`InternalMotionGateControl` contains no writer GID. At `OPEN`, MotionGate uses
its own graph context to require exactly one publisher endpoint and records
that endpoint's complete 16-byte GID. Candidate callbacks compare it only with
the `MessageInfo.publisher_gid` observed in the same Gate context. A
locked-Fast-DDS self-test proves that those two Gate-local representations
correlate; failure or mismatch keeps the Gate inhibited. This is a strict
supported-runtime constraint: canonical product bringup sets
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `motion_gate_node` rejects every other
RMW at startup, and both runtime packages declare `rmw_fastrtps_cpp` as an
execution dependency. A caller's `Publisher::get_gid()` is neither transported
nor compared across processes.

The same fail-closed rule applies to the command clock. Product startup
requires `use_sim_time=true`; the Node rejects any runtime attempt to change
that parameter. Immediately before every final publication, the serialized
barrier independently requires both the parameter to remain true and
`get_clock()->ros_time_is_active()`. Losing either invariant latches
`ConfigurationInvalid`, replaces the selected command with zero, and emits a
zero ROS stamp, so a system-time-stamped non-zero command cannot defeat the
controller's simulation-time consumer timeout.

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
The Core owns the selected command and state decision, not publication
acknowledgement. The Node Adapter owns actual final/state publication,
`output_publish_seq`, `zero_publish_seq`, and the response's `zero_published`
fact.

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
  -> PREPARE admission confirms the retired writer is absent
  -> Core PREPARE generates a new lease ID and per-lease candidate topic
  -> create discard-only reader A
  -> create/configure new Collision Monitor and smoother downstream-to-upstream
  -> OPEN first performs pure Core request/state/CAS/lease/deadline validation;
     a rejected request performs no graph query or reader mutation
  -> graph snapshot #1 requires one writer and healthy final controller
  -> destroy reader A and its queue; create discard-only reader B
  -> graph snapshot #2 requires the same unique writer GID
  -> Core atomically enters ARMED with selected output still zero
  -> destroy reader B and its queue; create accepting reader C
  -> graph snapshot #3 requires the same writer GID and healthy controller;
     mismatch faults the Gate and selects zero
  -> only now complete OPEN with a 250 ms Runtime authority lease
  -> RENEW while still selecting zero; activate Collision Monitor; RENEW;
     activate the smoother; then RENEW once more under the admitted generation
  -> start the new producer last
```

Readers A and B are always discard-only. Reader C is the first accepting
reader and is created only after Core has atomically entered `ARMED` with zero
selected. The two discard-reader destructions are queue barriers; the three
graph snapshots prove that the unique writer did not change across them. A
pre-OPEN sample can therefore never become a valid post-OPEN non-zero command.
Gate callbacks accept only the writer bound during this handover on the new
per-lease channel. An old sample retains its old channel or Gate-local writer
identity and remains invalid even if DDS delivers it after the new lease opens.

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

The current implementation delivers the normal-running Core, private seam,
Gate-local binding, barriers, final ownership, and deadline expiry with an
in-process test authority/candidate harness. It does not claim the complete
Runtime/smoother/Collision Monitor integration. Process-kill crash-stop,
consumer-deadman proof, and managed/unmanaged Gazebo pause behavior remain
separate target acceptance slices.

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
above. Every Goal that has entered production `on_accepted` and acquired a
`GoalHandle`/`CallbackLease` receives exactly one terminal result during
graceful shutdown. A provisional goal without a handle is revoked within its
bounded handoff window without fabricating a Result; after context or process
closure the transport does not claim exactly-once delivery.

## Relative motion

- `RelativeMotionController` is the deep ROS-free Module behind the production
  `RelativeMotionPort`; its ROS Adapter observes odometry and source-health
  signals without taking ownership of the final velocity writer.
- MOVE projects odometry displacement onto the signed initial-heading axis.
- ROTATE unwraps yaw before comparing signed angular displacement.
- Both use trusted YAML for speed, acceleration, tolerance, stall thresholds,
  and a policy-computed deadline.
- Both slow down near the target, publish zero before completing, and commit
  exactly one first-terminal result.
- Relative-motion samples are fenced by Runtime / admission / Mission / step
  generations and the active Gate lease; late odometry, timer, or downstream
  callbacks cannot publish a command or rewrite a terminal result.
- Step deadlines, stall windows, lease expiry, and cancel grace use a steady
  clock. ROS time is used only to stamp simulation-time data, including the
  final `TwistStamped`, odometry, TF, and sensor messages; it never drives a
  deadline. MotionGate locks `use_sim_time=true` for the process lifetime. If
  that invariant or the active ROS clock is lost, it faults closed and emits
  only zero commands with a zero stamp.
- Dependency steady liveness is 200 ms. In simulation, Collision Monitor's
  raw ROS-time source-age limit is independently fixed at 300 ms; an old raw
  measurement remains fail-closed even when callbacks continue arriving.
  Original scan measurement stamps and frames are retained, Collision Monitor
  consumes direct `/scan`, and the consumer uses `SENSOR_DATA` with
  `KEEP_LAST(1)`; no conditioned-scan relay restamps or masks sensor backlog.
  Headless raw-age and TF physical acceptance is tracked by Issue #72.
- Runtime child callbacks are serialized through a Node-owned typed queue with
  reserved control capacity and generation-tagged events. STOP/Cancel fences
  the generation first, starts asynchronous teardown, and uses a serialized
  state snapshot if the ROS service cannot enqueue or await its response.
- Normal queue saturation rejects only the normal event and records one
  QueueFault; the reserved STOP/Cancel lane remains usable. If queue admission
  or the Runtime worker fails, the Adapter's independent emergency inhibit/
  zero path still runs and remains idempotent.
- Stationarity is measured only from odometry received at or after the actual
  steady-clock Gate `zero_proven_at`; its deadline is absolute at
  `zero_proven_at + 1200 ms`, with no cleanup-time extension.

### RelativeMotion production seams

- The Runtime event queue has separate normal and control capacity. A full
  normal lane records a bounded queue fault; a full control lane raises an
  independent EmergencyFence that advances the admission epoch, inhibits and
  zeros the Gate, and prevents a later event from reopening the old generation.
- Cancellation is fenced after every controller, writer, lifecycle, and
  component boundary and again immediately before `OPEN`. A cancelled start
  therefore cannot publish a producer command or enter `OPEN`, even when the
  downstream call returns late.
- Start-drain timeout cleanup is owned by an object-held asynchronous
  continuation. It is not dependent on destruction, and producer stop,
  component cleanup, generation reclaim, and terminal publication are each
  performed at most once.
- Runtime health and teardown keep the frozen failure-code taxonomy typed:
  source-only odom/scan/clock liveness loss is
  `DEPENDENCY_UNAVAILABLE`; a RelativeMotion step deadline is `TIMEOUT`; and
  stall, collision, or other motion execution failure is `EXECUTION_FAILED`.
  Gate, controller, container, component, candidate-writer, zero-proof,
  handover, and stationarity failures are `SAFETY_FAULT`. An original
  business failure is upgraded to `SAFETY_FAULT` only when teardown cannot
  prove Gate inhibited+zero; proving zero does not rewrite an infrastructure
  safety fault. The residual safety fault remains latched against later
  admission.
- Node shutdown stops ingress, drains accepted internal completion events, and
  waits for the saved GoalHandle/CallbackLease from production `on_accepted`
  to receive its one graceful-shutdown terminal before closing the queue and
  destroying Runtime state. A provisional/no-handle ticket is revoked at its
  fixed bound and never receives a fabricated Result. Once the ROS context or
  process is closing, transport delivery is not claimed to be distributed
  exactly-once. Terminal records are bounded to eight recent generations.
- Action admission is linearized by one Node-owned gate shared by the
  on-goal/on-accepted handoff, AdmitEvent dispatch, start permits, and
  quiesce. A generation-bound permit is invalid after quiesce, so an event
  already in the queue cannot start Core, PREPARE, OPEN, or the producer.
  Provisional revoked tickets are bounded shutdown state, not a promise to
  retain a late transport handoff. Only an already-entered production
  `on_accepted` callback with a GoalHandle/CallbackLease participates in the
  graceful second drain; MotionGate inhibited+zero remains the independent
  safety guarantee.
- The production Node uses a package-private RuntimeExecutionPlane that owns
  RuntimeCore and the NodeCompletionMailbox together. Transaction, start
  failure, and emergency relay rejection all converge through this plane to a
  single structured Goal terminal; mailbox shutdown is idempotent and joins
  its reaper after all synchronization state has been constructed.
- Reentrant RelativeMotion ROS callbacks use a shared lifetime ingress with a
  weak Impl/producer capture and an in-flight guard. Shutdown disables new
  ingress before resetting subscriptions, the raw timer, and producer, then
  waits for queued and active callbacks before releasing the state. The
  production seam tests this barrier on a real MultiThreadedExecutor for
  odom, scan, clock, raw timer, and command-supplier callbacks.

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

The corresponding terminal codes are deliberately not inferred from the
presence of a later zero proof:

| Typed cause | Terminal code |
| --- | --- |
| Odom/scan/clock source liveness only | `DEPENDENCY_UNAVAILABLE` |
| RelativeMotion step deadline only | `TIMEOUT` |
| Stall, collision, or motion execution failure | `EXECUTION_FAILED` |
| Gate/controller/container/component/writer/zero/handover/stationarity | `SAFETY_FAULT` |

## Verification obligations

- Current cumulative verification retains the pure-Core manual-clock GTest, the
  deterministic conditioning/ROS-integration checks, a Fast-DDS-locked Node
  launch test with neither Gazebo nor `/clock`, and the existing MotionGate /
  perception headless product layer. Issue #64 does not claim a headless
  physical RelativeMotion acceptance; raw-age and TF evidence belongs to
  Issue #72. Repository-static contract checks are a prerequisite, not a
  substitute for any layer.
- Historical fixed-domain evidence is not current acceptance evidence. Current
  launch layers use the official
  `run_test_isolated.py` runner, clear inherited `ROS_DOMAIN_ID` and
  `DISABLE_ROS_ISOLATION`, and allocate a process-isolated ROS domain with
  localhost discovery. This current rule is not retroactive evidence for the
  old tag.
- The Node layer has a 60-second timeout and serial execution; the product
  layer additionally has
  a unique Gazebo partition, a 180-second timeout, and serial execution.
- Manual-clock tests prove lease, cancel-grace, timeout, and callback-fencing
  behavior without sleeping.
- OPEN tests prove pure validation precedes graph access and that readers A/B/C
  cross exactly three same-writer graph snapshots before success.
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
- RelativeMotion tests cover signed projection, yaw unwrap across `+/-pi`,
  bounded command limits, progress monotonicity, stall/deadline edges, and
  zero-proof stationarity fencing with a manual steady clock.
- No test exits while either Gate or controller can retain an authorized
  non-zero command.
