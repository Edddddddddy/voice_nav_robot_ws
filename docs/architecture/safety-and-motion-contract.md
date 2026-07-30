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

The complete chain uses `geometry_msgs/msg/TwistStamped`. Jazzy configuration
sets `enable_stamped_cmd_vel=true` explicitly.

There is no `twist_mux`. The single Mission execution slot already determines
the active source, so a second priority and ownership model would be
ambiguous. The velocity smoother only conditions acceleration and velocity.
Collision Monitor is a protective collision-avoidance layer, not a certified
emergency-stop system. Neither replaces MotionGate.

`ros_gz_bridge` bridges only `/clock` and `/scan` in the target product path.
Velocity commands, joint state, odometry, and TF remain in ROS 2 control.

## MotionGate contract

- MotionGate starts inhibited and cannot restore an earlier lease after a
  restart.
- It is the only publisher to the controller's final command endpoint.
- A candidate is accepted only for the active Runtime instance, admission
  epoch, Mission generation, and step generation.
- It rejects NaN, Inf, stale input, non-zero unsupported axes, and values
  outside trusted YAML limits.
- A valid candidate renews a **250 ms steady-clock lease**.
- Lease expiry inhibits motion and continuously publishes zero without
  waiting for Runtime, ROS time, Nav2, or Gazebo time.
- Closing or revoking a lease selects zero before it acknowledges the control
  request.
- `diff_drive_controller.cmd_vel_timeout` is **0.35 s**. It is the
  consumer-side second deadman if MotionGate itself dies.
- Runtime crash, cancel, timeout, dependency loss, invalid generation, and
  Gate fault all fail closed.
- A Gate health or zero-output state that cannot be established produces
  `SAFETY_FAULT`; Runtime admits no new Mission while faulted.

WSL is not a real-time environment. The 250 ms and 0.35 s values are tested
budgets for this supported environment, not hard real-time guarantees.

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
instance, admission epoch, Mission generation, and step generation. Every Goal
receives exactly one terminal result.

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
| Candidate stale or lease expired | Gate inhibits and continuously publishes zero |
| Runtime disappears | Gate lease expires independently |
| MotionGate disappears | controller consumes no fresh command and times out within 0.35 s of advancing simulation |
| Gazebo pause and resume | stale non-zero command is never replayed |
| Nav2 abort or step deadline | Gate zero, cancel child, fail step, skip remainder |
| Map save partial failure | publish no completed logical map directory |
| Late callback after cancel | discard by epoch and generation |
| Gate health or zero proof unavailable | report `SAFETY_FAULT` and remain fail-closed |

## Verification obligations

- Manual-clock tests prove lease, cancel-grace, timeout, and callback-fencing
  behavior without sleeping.
- Stop tests assert `EPOCH -> INHIBIT/ZERO -> CANCEL -> RESPONSE`, plus
  idempotent `request_id` behavior.
- Runtime tests cover cancel/STOP/success races and exactly-one Result.
- Process-death tests kill Runtime and MotionGate separately to prove both
  deadman layers.
- Pause/resume tests prove that an old non-zero command cannot resume motion.
- Odometry tests distinguish command-zero latency from physical stationarity.
- No test exits while either Gate or controller can retain an authorized
  non-zero command.
