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

`motion_gate_core` is a package-internal static build target. Its header and
library are neither installed nor exported; only `motion_gate_node` is an
installed runtime target. The Core Interface is the typed
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
acknowledgement. `MotionGateProcessRuntime` owns the serialized final-command
transaction, its attempt/success/zero counters, journal retirement, direct
safety-zero fallback, and one-shot terminal-cause consumption. The Node Adapter
owns the ROS publisher and state publisher only: it supplies the simulation
stamp and transport callback, then maps the Runtime result and counters into
the ROS State and control response.

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
  -> activate Collision Monitor, then smoother under the admitted generation
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

Lesson 0009 publicly delivers the normal-running Core, private seam,
Gate-local binding, barriers, final ownership, and deadline expiry with an
in-process test authority/candidate harness. It does not claim the complete
Runtime/smoother/Collision Monitor integration. Process-kill crash-stop,
consumer-deadman proof, and Managed Safe Pause / Unmanaged Pause behavior are
reserved for Lesson 0010 / VN-0011.

### Gazebo Managed Safe Pause and resume

**Status boundary:** the following is the accepted VN-0011B target contract,
not current Lesson 0009 behavior. VN-0011A first proves process crash-stop;
VN-0011B later implements and verifies the coordinator and token policy.

The controller timeout alone cannot prove “no replay” across a pause:
Jazzy `diff_drive_controller`
[computes command age from controller time and the Twist stamp](https://github.com/ros-controls/ros2_controllers/blob/jazzy/diff_drive_controller/src/diff_drive_controller.cpp#L94-L116).
When simulation time is stopped, its 0.35 s timeout does not advance.

The accepted package-private coordinator/test-Adapter path therefore uses a
two-phase Managed Safe Pause transaction while simulation updates are still
advancing. VN-0011B proves this protocol but does not yet expose a user-facing
product pause function:

1. reject new test admissions, stop renewing the current test authority,
   inhibit MotionGate, and receive its zero-output acknowledgement; a future
   Runtime/supervisor integration owns the equivalent product admission step;
2. observe `diff_drive_controller`'s limited command output and the wheel
   command/velocity state at zero for a configured number of complete control
   periods;
3. if Gate fails before that proof, let the active consumer timeout while the
   update loop is still advancing and still require direct observation of zero
   wheel command for the configured control periods; controller deactivation,
   inactivity, or released interfaces cannot mint a token and select
   `RESTART_REQUIRED` instead;
4. only after the zero proof pause Gazebo and record an opaque Safe-Pause Token
   containing the world iteration and Gate/controller instance state.

Wheel command is observed through ros2_control's diagnostic
`/controller_manager/introspection_data/full` stream, not inferred from
`/joint_states`. The required finite fields are
`command_interface.left_wheel_joint/velocity`,
`command_interface.right_wheel_joint/velocity`,
`state_interface.left_wheel_joint/velocity`, and
`state_interface.right_wheel_joint/velocity`. Controller body output, wheel
command, wheel state, and odometry remain separate evidence surfaces.
The subscriber uses `BEST_EFFORT + TRANSIENT_LOCAL + KEEP_LAST(1)`, waits for
discovery, and arms a fault only after complete finite, strictly increasing
samples corroborate that both wheel commands are non-zero. Delivery is lossy,
so this topic cannot prove an exact first write or the absence of an
intermediate non-zero regression. A default-off test-only lossless ledger at
the actual hardware-write seam accounts for each invocation, test generation,
simulation stamp, delegated return result, and both wheel-command bit patterns.
The test Adapter inherits the public
`gz_ros2_control::GazeboSimSystemInterface`, delegates every lifecycle and I/O
call to a pluginlib-loaded upstream `gz_ros2_control/GazeboSimSystem`, and
records `JointVelocityCmd` only after the upstream `write()` returns. Direct
inheritance from the concrete upstream class is not an extension seam.
The crash harness expands the canonical product Xacro, requires exactly one
upstream hardware plugin, and transforms only that XML block to the Adapter
while injecting the parent-owned shared-memory identity. Canonical Xacro,
product launch, and product YAML contain neither the Adapter nor journal
parameters. The Adapter's added post-delegate journal operations are
preallocated and allocation-free; no such claim is made about upstream
`write()` itself.

The public Interface does not receive Gazebo `UpdateInfo.iterations`; World
Statistics owns real iteration evidence independently. ARM/SEAL fences and
exact-step observations correlate that world evidence with the ledger's
`sim_stamp` and non-wrapping `write_seq` without fabricating an iteration value.
Crash-stop and pause tests use the ledger to prove exact wheel-write
transitions and no regression, while introspection remains mandatory
independent corroboration. Only consecutive invocations with identical
generation, simulation stamp, delegated return result, and exact wheel-command
bits may extend one active accumulator whose sequence range and invocation
count agree. A tuple change or `SEAL` finalizes it; only finalized segments and
snapshot pages are immutable. Segment capacity is proven before arming from a
bounded write-invocation/command-transition budget. Overflow/overwrite, an
unaccounted invocation, or a non-zero write in a zero-required interval latches
failure. Sealed segments are retained, and bounded immutable pages must have
valid checksums plus contiguous generation/`write_seq` ranges; simulation
stamp is nondecreasing and may repeat while paused. A reliable topic or
overwrite-on-full ring is not lossless evidence.

VN-0011A separately uses a parent-owned Gate event journal for applied
control/terminal transitions and crash-resilient output INTENT/COMMITTED
records. MotionGate normally republishes the selected tuple every 20 ms, so a
value match alone is ambiguous. The Gate-kill attempt creates a previously
unseen final marker, requires exactly one committed Gate publish and a matching
non-zero controller-output ACK, then dispatches exact SIGKILL before the next
periodic publish. A second publish or any later output record invalidates the
generation. Only that single committed input can define the consumer timeout
origin. For
authority/candidate death, the same journal includes every accepted
intervening RENEW; terminal retirement advances exactly once from its final
committed predecessor. One Core-owned wrapper samples its same-host monotonic
transition-linearization fence immediately before the bounded state mutation;
the bound zero output records its pre-publish `INTENT` timestamp. Both must be
no earlier than exact `ProcessExited`, while later `COMMITTED` times prove
completion only. Scattering records through Node callbacks or comparing only
post-hoc commit times is not causal proof. DDS receipt order cannot provide
that proof either. Wheel states and
odometry must remain stationary for a shared final window of at least 0.20 s
simulation time.

The test-only Node seam is disabled unless both read-only parameters
`test_gate_event_journal_name` and `test_gate_event_journal_descriptor` are
non-empty; partial or malformed configuration fails before any MotionGate
product publisher, service, subscription, or timer is created. The `rclcpp`
Node base may already own framework parameter, time-source, or logging entities.
The versioned descriptor supplies UID, generation, capacity, and the complete
nonce out of band, so the attacher never derives expected identity from
unpublished shared memory. One package-private
`MotionGateProcessRuntime` object inside the existing `motion_gate_node`
process constructs the Attached Journal before the Core and destroys the Core
before the Attached Journal. It also owns the single final-output transaction,
non-wrapping attempt/success counters, and the one-shot terminal-cause binding.
Journal or DDS evidence failure may invalidate the test generation, but it
cannot prevent a direct safety-zero attempt.

This is an operational `voice_nav_bringup` package-private coordinator and
test-Adapter transaction, not a public Mission pause endpoint and not a fifth
self-written resident process. The test Adapter is the current caller and
receives `RESTART_REQUIRED`; future product integration belongs to a lifecycle
supervisor.

If zero cannot be observed by the bounded pause deadline, no token is minted
and the test Adapter structurally shuts down the old simulation/control
generation instead of freezing an unproved command. Automatic replacement
launch is not part of VN-0011B.

Managed resume requires that token and verifies partition, world, exact Gazebo
process identity, paused iteration/time, fixed step/controller period,
controller generation/state/update stamp, Gate instance/control
sequence/final-publisher identity, and zero-proof stamp/sequence plus
lossless-oracle generation/sealed fence. The original Gate
may remain present and inhibited with the same identities, or it may be proven
to have exited after token creation while the final-command topic has zero
publishers. Any replacement Gate, changed Gate tuple, oracle generation/fence,
or new/different final publisher invalidates the token and returns
`RESTART_REQUIRED`.

VN-0011B mints a token only while `diff_drive_controller` remains ACTIVE; its
generation/state/update stamp are token-bound. A deactivated, inactive, or
replaced controller cannot satisfy the bounded update probe and returns
`RESTART_REQUIRED`; activation recovery belongs to a future supervisor.
Because ros2_control introspection is asynchronous BEST_EFFORT, it cannot
alone prove that the first resumed write was zero. Without enabling
continuous run, VN-0011B sends only exact
`{pause: true, multi_step: 1}` requests. Omitting `pause`, sending false, or
duplicating a request is a fault: Gazebo processes the pause field before the
step field, so the protobuf default would otherwise enable continuous run.
The positive response means queued intent only. World Statistics must confirm
exactly one iteration of progress and re-paused state after every request.

The fixed simulation step is smaller than the 100 Hz controller period. The
coordinator steps one iteration at a time until, within
`ceil(control_period / step_size) + 1` requests, a new same-stamp
`/cmd_vel_out` and complete introspection sample prove a controller update and
zero controller/wheel commands. A non-zero or missing observation fails before
the updated command can be written. It then performs one additional exact
single-step transaction. The lossless journal must prove that every write in
the armed probe stayed zero and that the first write after the controller
update wrote both post-update zeros. Only then may the coordinator send
`pause:false` for continuous execution. Iteration may repeat during paused
runner loops; `write_seq` orders those writes, while each controlled request
must cause exactly one `N -> N+1` transition.

A direct GUI or Gazebo Transport pause taken before this barrier has no
Safe-Pause Token. Managed resume refuses to unpause it in place and returns
`RESTART_REQUIRED`; the test Adapter terminates the old simulation/control
generation without claiming that it reached inactive/zero before shutdown. A
future supervisor may start a replacement from a known inactive, zero-command
state. This is required because paused `gz_ros2_control` cannot
process a controller switch or consume a newly buffered zero before the first
resumed write. The project does not claim arbitrary external pause/resume as a
functional-safety mechanism. This refusal is only a project Adapter policy; it
does not prevent a local operator from directly sending Gazebo Transport
`pause:false`. VN-0011B may return `RESTART_REQUIRED` and structurally shut
down the old generation. Automatically launching a replacement generation
requires a later supervisor and is not part of this contract.

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
  clock. ROS time is used only to stamp simulation-time data, including the
  final `TwistStamped`, odometry, TF, and sensor messages; it never drives a
  deadline. MotionGate locks `use_sim_time=true` for the process lifetime. If
  that invariant or the active ROS clock is lost, it faults closed and emits
  only zero commands with a zero stamp.

## Failure behavior

| Failure | Required behavior |
| --- | --- |
| Invalid, stale, or oversized Mission | reject before any execution side effect |
| Second Mission while busy | return `BUSY`; no queue or implicit preemption |
| Dependency unavailable during whole-plan validation | reject before an earlier step starts |
| Candidate stale or authority lease expired | Gate inhibits, latches the old lease closed, and continuously publishes zero |
| Runtime disappears | Gate lease expires independently |
| MotionGate disappears while simulation advances | a marker new to the generation is COMMITTED exactly once and ACKed by non-zero controller output before the next 20 ms Gate publish; exact kill leaves it as the final source record, otherwise the generation retries. The controller selects zero on the first update where that one input is older than 0.35 s simulation time |
| MotionGate disappears after a Managed Safe Pause | exact original-Gate exit plus zero final publishers remains admissible; any replacement identity invalidates the token |
| Managed Safe Pause and resume | exact single-step/re-pause transactions reach and prove the next controller update zero; one additional step losslessly writes that post-update zero before continuous `pause:false` |
| Unmanaged Pause without a Safe-Pause Token | in-place resume is refused; return `RESTART_REQUIRED` and shut down the old generation |
| Nav2 abort or step deadline | Gate zero, cancel child, fail step, skip remainder |
| Map save partial failure | publish no completed logical map directory |
| Late callback after cancel | discard callback by epoch/generation; discard velocity through the inhibited handover barrier |
| Gate health or zero proof unavailable | report `SAFETY_FAULT` and remain fail-closed |

## Verification obligations

- Current cumulative verification retains the three executable layers
  established by Lesson 0009: pure-Core manual-clock GTest; a Fast-DDS-locked
  Node launch test with neither Gazebo nor `/clock`; and a Fast-DDS-locked
  headless Gazebo product launch test. Repository-static contract checks are a
  prerequisite, not a substitute for any layer.
- The immutable `course/0009-solution` used fixed ROS domains 91/92. After the
  post-tag [VN-0010-C2](../work-items/0010-corrective-gazebo-teardown.md)
  correction, both launch layers use the official
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
- Managed Safe Pause/resume tests prove that an old non-zero command cannot
  resume motion within the documented boundary.
- Handover tests inject old-writer candidates before, during, and after full
  pipeline recreation and prove that channel/GID binding rejects all of them.
- Pause tests request Managed Safe Pause while moving, prove the controller and
  wheel command reach zero before `/clock` stops, kill MotionGate while paused,
  send only exact `{pause: true, multi_step: 1}` requests until the next
  controller update is observed zero, then perform one additional step and
  require every armed write plus the post-update write to be zero before
  continuous unpause. BEST_EFFORT introspection alone, a retained zero,
  omitted/false pause, duplicate request, or incomplete write interval cannot
  satisfy this assertion.
- Pause tests also kill MotionGate before zero proof; interface release or
  controller inactivity never mints a token without observed zero, and a
  failed bounded proof selects `RESTART_REQUIRED` and old-generation shutdown.
- Unmanaged Pause tests prove that a missing or mismatched Safe-Pause Token
  refuses in-place resume and selects `RESTART_REQUIRED` without claiming an
  automatic replacement launch.
- Odometry tests distinguish command-zero latency from physical stationarity.
- No test exits while either Gate or controller can retain an authorized
  non-zero command.
