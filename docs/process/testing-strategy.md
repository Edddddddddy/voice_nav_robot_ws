# Testing strategy

Tests follow the deepest stable Interface. A behavior test should retain its value when the Implementation behind that Interface is refactored.

## Test layers

| Layer | Purpose | Examples |
| --- | --- | --- |
| Static | Reject malformed source and metadata | XML, YAML, Python, CMake, license |
| Unit | Exercise deterministic behavior and state | Mission Validator/FSM, Agent rules, audio buffers |
| Contract | Protect externally visible semantics | ROS IDL, topic type, QoS, TF owner, units, limits |
| Integration | Verify connected ROS Modules | launch, Action cancel, bridge directions, lifecycle |
| Headless simulation | Verify physics and bounded flows | drive, stop, odom, scan, map, Named Place |
| Model fixture | Verify locked local models and offline audio | KWS, ASR, TTS, LLM, AEC fixtures |
| Manual release gate | Validate the supported WSL audio path | real single microphone, speaker, AEC, barge-in |

## Developer and CI loops

During implementation:

```bash
bash scripts/verify.sh <changed-package>
```

Before review or merge:

```bash
bash scripts/verify.sh
```

The full gate starts from declared dependencies, validates repository and
robot-model contracts, builds all packages, runs all tests, and reports a
zero-error `colcon test-result`. Repository contracts run through
`scripts/run_repository_tests.py`; discovery uses the real non-package
`tests/` layout and any skipped contract makes the gate fail.

Critical launch tests use Jazzy's official `run_test_isolated.py`. Their
generated CTest contract clears inherited `ROS_DOMAIN_ID` and
`DISABLE_ROS_ISOLATION`, retains `RUN_SERIAL`, and permits only the reviewed
result-neutral properties. Source CMake is not final evidence: after configure,
`scripts/check_generated_launch_tests.py` inspects
`ctest --show-only=json-v1` for the exact runner, source target, environment,
timeout, working directory, labels, and result semantics. The reporter then
requires the matching critical xUnit testcase structure; a skip is accepted
only for the exact package-local cppcheck artifact/class allowlist.

Run a release gate as the terminal command whose exit status is consumed.
Process snapshots and other diagnostics run as separate commands afterward;
a trailing successful `ps`, `grep`, or cleanup command must never replace a
failed CTest status.

PR CI uses deterministic in-memory fakes as soon as their Module exists and
adds bounded headless Gazebo tests with the v0.2 simulation milestones. At
v0.1 the hosted gate covers repository metadata, static robot-model
validation, package build, and package tests; it does not claim to launch
Gazebo. Nightly validation uses the locked model set after the model fixtures
are introduced. Real-audio metrics are manual hardware Release Gates for
`v0.7` and `v1.0`; they are not weakened into CI simulations.

## Adapter and time strategy

Internal seams have production and deterministic in-memory Adapters:

| Seam | Production Adapter | Deterministic test Adapter |
| --- | --- | --- |
| Navigation | Nav2 `NavigateToPose` | scripted goal/result/cancel fake |
| Relative motion | odom feedback and candidate Twist | scripted motion fake |
| Motion authority | independent MotionGate | event recorder with lease expiry |
| Map saving | slam_toolbox/map saver | in-memory map registry |
| Clock | steady monotonic clock | manual clock |
| ASR/TTS/LLM | locked local runtimes | scripted text/audio/result fakes |

Mission behavior tests cross the Mission Module Interface and replace downstream Adapters. They do not assert private FSM states. Adapter contract tests prove that production Adapters map upstream ROS behavior to the same internal semantics.

- Physics, TF, SLAM, AMCL, and Nav2 use simulation time where appropriate.
- Mission timeouts, cancel grace, command leases, audio liveness, and cleanup deadlines use a steady monotonic clock.
- Unit tests advance a manual clock instead of sleeping.
- Fakes inject timeout, abort, partial map, delayed cancel, late success, and dependency loss.
- Random seeds, worlds, initial poses, model versions, and resource limits are fixed in acceptance evidence.

## Coverage gates

- Mission Core and Agent: at least 90% line coverage and at least 80% branch coverage.
- Audio code that does not require real hardware: at least 80% line coverage.

Coverage is a release gate for the relevant milestone. It supplements behavior assertions and does not replace them.

## Mission completion criteria

Mission unit and contract tests cover:

- invalid combinations of discriminant and payload fields;
- NaN and Inf rejection;
- Mapping/Navigation Mode policy;
- atomic whole-plan validation for a three-step Mission before any motion side effect;
- single execution-slot `BUSY` behavior;
- source ordering, `runtime_instance_id`, and `admission_epoch`;
- Runtime restart invalidating an old request;
- Cancel, STOP, natural success, and timeout races through one terminal linearization point;
- late Nav2, relative-motion, map, and Agent callbacks;
- exactly one Result and non-decreasing best-estimate feedback;
- steady-clock timeout behavior while ROS time is paused or changed.

The test suite also proves that a rejected plan starts no downstream Adapter and that late results cannot reopen the MotionGate or advance the next step.

## MotionGate and stopping completion criteria

Every automated motion test uses configured limits, a steady-clock deadline, zero output in success and cleanup paths, odometry-based stationarity checks, and bounded process cleanup. `Ctrl+C`, publisher exit, Action Result, or a single zero publication is not proof of stopping.

### Gazebo launch-test lifecycle

Tests that own a Gazebo server use a lifecycle oracle separate from their
product assertions. At module import, each test process overwrites inherited
state with a scope/PID/128-bit-random non-empty `GZ_PARTITION`; CMake does not
provide a reusable fixed partition. Cleanup first selects zero or inhibits
MotionGate, sends `stop: true` to `/server_control` with the same environment
snapshot that was validated, requires a positive `gz.msgs.Boolean`
acknowledgement, and then waits for the launch-managed `gazebo` process itself
to exit. An ACK is request acceptance, not process completion. A post-shutdown
test finally applies an unfiltered `assertExitCodes(proc_info)` to every
launch-managed process.

The product launch still defaults to shutting down when Gazebo exits. Tests
disable only that immediate event handler while their failure-safe cleanup
performs the structured stop and process join. The cleanup ladder is must-run:
zero/inhibit, structured stop, and ROS fixture destruction are independent
LIFO `unittest` cleanups, so one exception cannot short-circuit the next.
Cleanup phases that own multiple resources use an exhaustive aggregator and
raise the collected errors only after every step was attempted. A typed
`TimeoutExpired` from the isolated idempotent stop request is retried once in a
fresh CLI process; all other CLI/ACK errors fail immediately, and two timeouts
still fail. Static mutation tests reject fixed partitions, fixed sleeps,
global process killing, shell execution, forced-exit allowlists, ACK-only
cleanup, rebound or unreachable oracles, disabled critical test modules,
wrong RPC environments, cleanup list mutation, and cleanup registration that
can be skipped after an active assertion failure.

Gazebo ground-truth movement evidence is separate from ROS odometry. A pure
test-support module queries the exact isolated world's pose topic with a
10-second deadline and one read-only retry. It accepts at most four adjacent
complete JSON documents because `gz topic --num 1` can race with a high-rate
publisher and emit a small burst; every document must contain one valid model
pose, and the newest is used. Wrong partition, malformed/extra output,
duplicate/missing model, zero/non-finite quaternion, and non-finite pose all
fail. Query failure remains an active-test failure, not a teardown diagnosis.

This fixture contract proves deterministic test teardown. It does not prove
the internal cause of a slow signal-only Gazebo shutdown, ordinary user
`Ctrl+C` behavior, MotionGate crash-stop, controller deadman, or managed
pause/resume semantics. See
[VN-0010-C2](../work-items/0010-corrective-gazebo-teardown.md) and
[PIT-0012](../../course/reference/engineering-pitfalls.md#pit-0012-no-residual-gazebo-process-is-not-a-clean-gazebo-exit).

The source/AST guards are cooperative correctness controls for ordinary
reviewed changes. They do not claim to sandbox a malicious same-UID process or
deliberate Python dynamic metaprogramming that rewrites files or imported
objects at runtime.

### Lesson 0009 normal-running Gate slice

VN-0010 / Lesson 0009 proves the independent Gate without claiming process
death or pause recovery:

- Manual-clock Core tables cover the exact 250 ms authority and 150 ms
  candidate-freshness boundaries; a 20 ms wall output tick continuously
  selects zero while inhibited.
- Every `PREPARE`/`OPEN`/`RENEW`/`INHIBIT` request uses the Gate instance and
  one global compare-and-swap `control_seq`; operations after `PREPARE` also
  match the Gate-generated current lease. A late old-lease `INHIBIT` cannot
  stop a newer lease, while a matching current `INHIBIT` publishes zero before
  acknowledgement.
- Finite `linear.x`/`angular.z` values are clamped to trusted YAML limits.
  NaN, Inf, or a non-zero unsupported axis retires the current lease and
  selects zero.
- `/motion_gate_node` serves `/motion_gate/internal/control` and
  `/motion_gate/internal/state`. PREPARE returns a Gate-generated per-lease
  topic below `/voice_nav_internal/motion_gate/candidate/lease_`. OPEN first
  validates in Core without graph access, then requires the same unique
  publisher GID across graph snapshot #1 with discard reader A, snapshot #2
  after recreating discard reader B, and snapshot #3 after creating the first
  accepting `VOLATILE + KEEP_LAST(1)` reader C. Any change faults closed.
- Contract tests require trusted YAML root `motion_gate_node` and prove all
  node/control/state/candidate-prefix/final-command names are code constants,
  absent from YAML parameters and product remaps.
- A locked `rmw_fastrtps_cpp` self-test correlates the Gate-local graph GID
  with Gate-local `MessageInfo.publisher_gid`; the control request never
  carries caller `Publisher::get_gid()`.
- Candidate QoS is `BEST_EFFORT + VOLATILE + KEEP_LAST(1)`. The sole final
  publisher uses `rclcpp::SystemDefaultsQoS()` and a runtime checker proves
  actual compatibility with the controller subscriber; policies introspected
  as `UNKNOWN` are not asserted as fixed reliability/history/depth.
- A serial publication barrier proves that no earlier queued non-zero command
  can publish after current-lease INHIBIT, expiry, or invalid-input zero.
- A runtime parameter test rejects changing `use_sim_time` while moving.
  Publication also requires both the parameter value and
  `ros_time_is_active()`; loss of either invariant faults closed, publishes
  zero, and never emits a system-time-stamped non-zero command.
- Headless Gazebo evidence separately records Gate zero, controller output
  zero, and odometry stationarity after bounded motion and after each normal
  deadline expiry.

Package-private IDL is an encapsulation boundary, not DDS security. Lesson 0009
uses a test authority/candidate harness. It does not count an authority,
candidate, or MotionGate process kill, managed pause token, first-resume zero,
or unmanaged-pause recovery as completed.

### Lesson 0010 crash-stop and pause slice

Lesson 0010 / VN-0011 supplies the process-death and Gazebo-time evidence:

- killing the authority while valid-looking candidates continue still expires
  the independent Gate lease;
- killing the candidate producer still expires candidate freshness;
- killing MotionGate causes `diff_drive_controller.cmd_vel_timeout` to select
  zero on the first control update after 0.35 seconds of advancing simulation
  time;
- managed pause proves Gate, controller, and wheel zero before minting a token,
  then proves the first resumed wheel command is zero;
- unmanaged GUI/Transport pause has no token and requires a full
  simulation/control restart rather than in-place resume.

The later Mission and voice milestones inherit these v1.0 quantitative
acceptance criteria:

- From a `StopMission` request to the final zero-velocity output:
  - P95 ≤ 100 ms;
  - P99 ≤ 200 ms;
  - maximum ≤ 300 ms.
- From maximum configured speed, STOP causes odometry to enter the stationary tolerance within 1.2 seconds and remain there for 200 ms.
- Killing MissionRuntime causes the independent MotionGate lease to expire and automatically select zero velocity.
- Killing MotionGate causes `diff_drive_controller.cmd_vel_timeout` to select
  zero on the first control update after 0.35 seconds of advancing simulation
  time. The configured 100 Hz period gives the measurement one 10 ms
  scheduling tolerance; physical stationarity is a separate assertion.
- Candidate samples never renew Runtime authority; a test continues feeding
  valid-looking smoother output after killing Runtime and still observes Gate
  inhibition.
- Every step handover recreates the candidate data plane. Tests inject samples
  from the old topic generation and an unbound Gate-local writer after the new
  lease opens and prove they are rejected.
- Managed Gazebo safe-pause first proves Gate output, controller output, and
  wheel command are zero while simulation still advances, then pauses and
  records a token. After MotionGate dies during that pause, the first resumed
  wheel command is still asserted to be zero.
- A fault-injection case kills MotionGate before zero proof. Controller
  inactivity or released command interfaces are insufficient: the harness
  issues a token only after directly observing zero wheel command for the
  configured periods; otherwise it selects full restart.
- A direct GUI/Transport pause has no safe-pause token; in-place resume is
  refused and the tested recovery is a full simulation/control restart from
  an inactive zero-command state.

These command-inhibition thresholds do not claim that the system is a functionally certified emergency stop.

## Mapping completion criteria

- A TF ownership check proves that each required transform has one semantic owner.
- Saving produces a complete atomic map directory containing occupancy YAML, image, and posegraph.
- The saved map package can be loaded again.
- A partial failure exposes no half-written map package.
- Map ID handling rejects path traversal.

## Navigation completion criteria

- All three predefined Named Places succeed without collision.
- Each final pose has position error ≤ 0.25 m and yaw error ≤ 0.25 rad.
- Successful, failed, canceled, and timed-out navigation paths return to zero velocity.
- A launch that tries to start SLAM and AMCL together fails instead of creating two `map → odom` owners.

The mode checks also prove:

- Mapping uses slam_toolbox for `map → odom`;
- Navigation uses AMCL for `map → odom`;
- both modes use `diff_drive_controller` for `odom → base_footprint`;
- robot-internal frames belong to `robot_state_publisher`;
- `ros_gz_bridge` bridges only `/clock` and `/scan`.

## Agent completion criteria

The fixed Mandarin corpus covers:

- deterministic rules;
- clarification;
- local LLM fallback;
- schema-valid but semantically invalid output;
- LLM timeout;
- a late LLM response after a newer turn or STOP.

For every case, arbitrary LLM output is unable to publish velocity, provide a path, or override trusted speed, acceleration, tolerance, timeout, map-path, or admission policy.

## Offline voice and audio completion criteria

Deterministic offline fixtures cover:

- far-end only audio;
- near-end only audio;
- double-talk;
- acoustic delay from 40 ms through 250 ms;
- clock drift of ±100 ppm;
- PortAudio xrun;
- ring-buffer overflow;
- late TTS PCM;
- fixed STOP preemption.

The fixtures also verify the 48 kHz mono full-duplex callback boundary, 10 ms/480-sample DSP framing, render-reference ordering, 16 kHz KWS/ASR input, and stale playback/turn result isolation. The real-time callback performs no allocation, blocking, logging, ROS calls, or model inference.

## Real single-microphone and speaker completion criteria

Acceptance uses the supported motherboard analog microphone input and speaker output, with Windows audio enhancements and spatial audio disabled.

- Far-end-only ERLE median ≥ 6 dB, excluding the first 2 seconds of convergence.
- Wake-word recall:
  - quiet environment ≥ 95%;
  - during playback ≥ 90%.
- Double-talk command semantic success rate ≥ 85%.
- A two-hour TTS-only run produces zero erroneous Missions.
- Fixed STOP recall ≥ 95%.
- From the end of the STOP phrase to MotionGate zero velocity, P95 ≤ 500 ms.
- A 30-minute soak has no unhandled overflow, no uncontrolled playback, and no obvious memory growth.

## Evidence and current gaps

Automated evidence is a command, exit status, concise test-result summary, and the relevant coverage or latency report. Manual evidence may add a screenshot, pose sample, TF graph, sanitized audio clip, or model manifest, but cannot replace an automatable assertion.

Evidence belongs in the Work Item or course record. Generated logs and private artifacts do not enter Git.

At the v0.1 foundation audit, the unified gate covered repository metadata, model expansion, URDF/SDF semantics, build, and package tests. It did not yet contain `gz_ros2_control`, LiDAR, MotionGate, Mission Runtime, SLAM, Nav2, Agent, or voice behavior tests. Each gap is closed by the release and lessons assigned in the approved roadmap.
