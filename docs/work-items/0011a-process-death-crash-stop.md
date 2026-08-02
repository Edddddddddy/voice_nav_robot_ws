# VN-0011A: Prove independent process crash-stop

**Status:** In Progress

**Parent:** [VN-0011](0011-crash-stop-and-safe-pause.md) / GitHub
[#20](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/20)

**GitHub Issue:**
[#21](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/21)

**Branch:** `feat/vn-0011a-l0010-crash-stop`

**Capability state:** In progress. The exact-action/exact-exit CrashLedger, Gate
output transaction, Core-owned transition matrix, one-shot transition
capability, POSIX attachment/lifetime proof, and Runtime-owned final publisher
composition are complete. The current `voice_nav_mission` package gate is
18/18 GREEN. Real pluginlib discovery now constructs the public-Interface test
Adapter and its pinned upstream plugin. Its complete Jazzy 1.2.19 forwarding
surface and post-delegate `JointVelocityCmd` observation are behavior-tested;
the focused Adapter target is 10/10 GREEN. The pure crash robot-description
transformer now replaces only the unique canonical upstream plugin, injects
only the validated journal name/nonce, and rejects structural or ownership
ambiguity. `voice_nav_sim` is 13/13 CTest GREEN (69 scoped xUnit tests, 6
skipped). The preallocated pure-C++ hardware-write ledger core now has
contiguous non-wrapping sequence accounting, count-preserving segmentation,
immutable CRC64 snapshot pages, and sticky generation/finite/stamp/capacity/
zero-window faults; its focused behavior suite is 14/14 GREEN. Shared-memory
ARM/SEAL control, retained dual banks and ACK, Adapter fail-closed integration,
evidence policy and composition, repository GREEN, product crash runs, the
final local gate, PR, and CI remain open.

## Immutable base

```text
course/0010-start tag object:
92a054c3eaae6e4dd0e8500aa712e866e8a71e33

peeled public target:
f75a9c48f610306a1cf3ec83d0e5e99474220ad6
```

## Goal

Prove three independent process-death paths without changing product motion
semantics:

```text
authority process dies -> 250 ms Gate authority deadman -> zero
candidate process dies -> 150 ms Gate freshness deadman -> zero
MotionGate dies       -> 0.35 s controller consumer deadman -> zero
```

The authority renewer and candidate publisher are separate, test-only OS
processes. Candidate FQN remains exactly `/collision_monitor`. They exercise
the real package-private Gate protocol but are not product nodes.

## Stable Interface and architecture impact

- Public `voice_nav_interfaces`: unchanged.
- Package-private MotionGate IDL and Core semantics: unchanged.
- Product resident processes: unchanged; no respawn or Gate-exit shutdown is
  added.
- Controller limits, 100 Hz update rate, stamped command mode, and
  `cmd_vel_timeout=0.35`: unchanged.
- New code is test support, fault-injection orchestration, pure evidence
  analysis, default-off crash-resilient Gate-event and hardware-write
  journals, and the minimum launch composition seam needed to retain an exact
  MotionGate ProcessAction.
- The real `motion_gate_node` contains the Gate-event-journal test seam. Its two
  read-only string parameters are exactly `test_gate_event_journal_name` and
  `test_gate_event_journal_descriptor`, both defaulting to empty. Both empty
  disables the seam; exactly one non-empty value fails startup. The crash
  harness gives only that exact launch action a parent-owned POSIX shared-memory
  name plus the complete versioned descriptor defined below. MotionGate opens
  but never creates or unlinks the object and validates ABI, size, UID, nonce,
  capacity, and single-writer claim. Normal product composition passes neither
  parameter; static contracts reject either key in product launch or YAML.
  This is test instrumentation, not a ROS Interface or security boundary.
- Only the crash/pause test model selects a journaled Adapter that inherits the
  public `gz_ros2_control::GazeboSimSystemInterface`. The Adapter owns a
  `pluginlib` loader and an upstream interface instance created from
  `gz_ros2_control/GazeboSimSystem`; it delegates lifecycle, interface export,
  mode switching, `read()`, `write()`, and `initSim()` without changing their
  arguments or return values. After the delegated `write()` returns, the
  test-only instrumentation reads the actual left/right
  `JointVelocityCmd` components from the saved ECM joint entities. Product
  URDF continues selecting the upstream plugin directly. Static, compile, and
  parity tests reject direct subclassing, selecting the Adapter outside the
  owned test model, or changing delegated base behavior. Member lifetime keeps
  the plugin loader alive until after the upstream instance is destroyed.
- The crash harness expands the canonical product Xacro, parses it as XML,
  requires exactly one upstream hardware plugin, replaces only that block with
  the test Adapter, and injects the shared-memory identity there. A second or
  missing plugin, any other XML difference, or any Adapter/journal reference in
  canonical Xacro, product launch, or product YAML fails the static parity
  contract.
- The public hardware Interface receives ROS simulation `time` and `period`
  but not Gazebo `UpdateInfo.iterations`. Its lossless ledger therefore records
  `sim_stamp`, non-wrapping `write_seq`, exact command bits, and the delegated
  return result. World Statistics records real iteration independently; the
  ARM/SEAL interval and exact-step protocol correlate the two evidence streams
  without inventing an iteration field at the hardware seam.
- The package that owns the introspection subscriber declares a direct
  `pal_statistics_msgs` test dependency; it does not rely on a transitive
  controller-manager dependency.
- The Adapter test target directly discovers and declares
  `gz_sim_vendor`, `gz-sim8`, `gz_ros2_control`, `hardware_interface`,
  `pluginlib`, `rclcpp`, and `rclcpp_lifecycle`; its plugin XML names
  `gz_ros2_control::GazeboSimSystemInterface` as the base class. It does not
  rely on the upstream package's incomplete exported-target discovery order or
  transitive manifest dependencies.
- No new ADR is required for A; it executes the dual-deadman decision already
  recorded by [ADR-0002](../adr/0002-migrate-to-gz-ros2-control.md).

## Crash acceptance matrix

| Case | Fault linearization | Live counter-evidence | Required terminal evidence |
| --- | --- | --- | --- |
| Authority | exact authority `ProcessExited` with `-SIGKILL`, recorded on the observer monotonic clock | the fault-arming barrier proves `authority_live=true`, candidate fresh, and every command surface non-zero | Gate reason `AUTHORITY_EXPIRED`; journaled terminal transition and bound-zero output pre-call fences do not precede ProcessExited, and later commits prove completion; the matching state is observed no later than 300 ms steady time afterwards |
| Candidate | exact candidate `ProcessExited` with `-SIGKILL`, recorded on the observer monotonic clock | the fault-arming barrier proves `candidate_fresh=true`, authority live, and every command surface non-zero | Gate reason `CANDIDATE_EXPIRED`; journaled intervening RENEWs lead to one terminal predecessor-plus-one retirement whose transition/output fences follow ProcessExited; the matching state is observed no later than 200 ms steady time afterwards |
| MotionGate | exact Gate `ProcessExited` with `-SIGKILL` | the fault-arming barrier proves Gate authority/candidate validity and every command surface non-zero; Gazebo, controller manager, and controller stay live; test publishes no zero | a previously unseen final marker is COMMITTED exactly once by the Gate journal, ACKed by non-zero controller output, and followed immediately by exact Gate SIGKILL; a second periodic publish before death invalidates the attempt and requires a fresh generation. After exit there is no later INTENT/COMMITTED output record. First controller-output zero satisfies `0.35 s < delta_sim <= 0.36 s + step_epsilon` from that one input stamp; the lossless hardware-write journal separately proves both wheel commands reach zero and never regress; downstream stationarity also passes |

Each case starts a new Gate lease generation. The Gate case runs last because
the product does not respawn MotionGate.

### Fault-arming barrier

Every SIGKILL is armed by one bounded observer barrier, not by an earlier
sample that happens to be non-zero. The steady-clock part spans no more than
two 20 ms Gate output periods and must see, for the same generation:

- a Gate state with both `authority_live=true` and `candidate_fresh=true`;
- a recent non-zero final Gate input; the MotionGate-death case additionally
  requires a marker not previously selected in that generation; and
- no intervening zero, invalid lease/freshness state, publisher replacement,
  or generation change.

At signal dispatch, the final Gate-state receipt is at most one 20 ms output
period old. Simulation evidence has a separate precondition: `/clock` is
strictly advancing and the latest non-zero controller output, complete
introspection sample, and lossless write record are each no more than 30 ms of
simulation time older than the final Gate-input stamp. They are not forced
into the <=40 ms steady window, so a low real-time factor does not change the
contract. The exact matching `ProcessExited` observer receipt is timestamped
with Linux `CLOCK_MONOTONIC`. The Gate event journal must prove that the
terminal transition linearization fence and the bound zero output's pre-call
`INTENT` timestamp occurred no earlier than that timestamp; the later
`COMMITTED` records prove completion, not causal order. DDS state/command
receipt order alone is not a causal ordering proof. If zero or invalidation
wins before that event, the
generation is not crash evidence and the case must retry from a fresh
generation or fail its bounded outer wall-time budget. Delaying SIGKILL,
reusing a stale baseline, or substituting cross-topic receipt order is a
required failing mutation.

### Gate-zero linearization

The authority/candidate latency endpoint is not a boolean chosen after the
fact. Arming stores the Gate instance, non-maximal `control_seq`, lease,
`output_publish_seq`, `zero_publish_seq`, and Gate-event-journal fence. Every
successful same-generation control operation records its before/after
`control_seq` in that journal, including RENEW operations accepted while the
signal or process-exit event is in flight. Expiry retires and clears the lease.
The journal must contain exactly one expected terminal retirement whose
predecessor is the last committed control transition, whose non-wrapping
`after_control_seq == predecessor_after_control_seq + 1`, and whose monotonic
`transition_linearization_ns` sampled immediately before its bounded mutation
is no earlier than the exact producer `ProcessExited` observer time. A later
commit timestamp cannot substitute for that fence. After that proof, the
latency endpoint is the observer steady receipt of the first matching state
with:

- the same `gate_instance_id`;
- `control_seq` equal to the journaled terminal `after_control_seq`;
- `state=INHIBITED` and an empty terminal `lease_id`;
- the expected `AUTHORITY_EXPIRED` or `CANDIDATE_EXPIRED` reason;
- `motion_inhibited=true` and `zero_selected=true`;
- `output_publish_seq` greater than the armed baseline; and
- `zero_publish_seq == output_publish_seq` and greater than the armed
  `zero_publish_seq`.

MotionGate publishes this state only after the serialized final-command
publish succeeds. The event journal binds the terminal transition to that
zero-publication `COMMITTED` record and `output_publish_seq`. The transition
linearization fence and the output record's pre-publish `INTENT` timestamp use
the same monotonic clock and must be ordered after `ProcessExited`; their later
commit timestamps only prove successful completion. The test separately
requires a final-command zero receipt and no subsequent non-zero,
but cross-topic receipt order is neither substituted for the journal ordering
nor for the steady latency endpoint. A stale zero sequence or internal reason
without a newly published zero cannot complete the case.

## Evidence surfaces

The test must preserve these as distinct observations:

| Surface | Meaning | Clock |
| --- | --- | --- |
| `/motion_gate/internal/state` | pre-fault live/fresh snapshot, Gate decision, and typed expiry reason | steady observer receipt latency; ROS stamp is not the deadline clock |
| `/diff_drive_controller/cmd_vel` | MotionGate's periodic controller inputs; the final crash marker is new to the generation and must occur exactly once before Gate death | strictly increasing simulation stamp |
| default-off Gate event journal | applied control/terminal transitions plus crash-resilient two-phase INTENT/COMMITTED records for attempted final publishes; final records survive exact SIGKILL | same-host monotonic transition-linearization and output-intent times, later commit time, and journal sequence; output records also carry simulation stamp |
| `/diff_drive_controller/cmd_vel_out` | controller-limited body command; a marker match ACKs that the journaled input affected an actual controller update | strictly increasing simulation stamp |
| `/controller_manager/introspection_data/full` (`pal_statistics_msgs/msg/Statistics`) | command-interface values that the next synchronous hardware write will consume, plus matching state-interface values; not a Gazebo execution acknowledgement | strictly increasing simulation stamp |
| default-off test hardware-write ledger | every delegated left/right command write, its test generation, simulation stamp, exact value bits, and upstream return result; identical consecutive calls may use a count-preserving segment | contiguous `write_seq`; simulation stamp is nondecreasing and may repeat while paused; World Statistics owns iteration evidence |
| `/joint_states` | measured wheel velocity state, never a command oracle | strictly increasing simulation stamp |
| `/odom` | physical stationarity proxy | strictly increasing simulation stamp |

The introspection sample must contain all four finite fields:

```text
command_interface.left_wheel_joint/velocity
command_interface.right_wheel_joint/velocity
state_interface.left_wheel_joint/velocity
state_interface.right_wheel_joint/velocity
```

The subscription explicitly uses the publisher-compatible
`BEST_EFFORT + TRANSIENT_LOCAL + KEEP_LAST(1)` QoS. Before every fault, it must
discover the publisher and observe a complete finite, strictly increasing
sequence in which both wheel commands are non-zero; a retained or initial zero
cannot arm the fault. Because that stream is lossy, it remains mandatory
corroboration but cannot prove that an intermediate non-zero write did not
occur. The default-off test ledger therefore accounts for every actual hardware
write after arming. It proves the first both-wheel zero write and that neither
wheel command regresses through the end of one shared final window in which
both wheel states and odometry remain stationary for at least 0.20 s of
advancing simulation.

Define command-zero linearization as the later of the first controller-output
zero and the first losslessly recorded hardware write where both wheel
commands are zero. The first shared wheel-state/odom stationary sample must be
no later than 1.2 s of simulation time after that linearization and the
stationary window then holds for at least 0.20 s. A single zero sample,
controller inactivity, interface release, lossy introspection alone, or
`/joint_states` alone is insufficient. See the
[Jazzy ros2_control introspection contract](https://control.ros.org/jazzy/doc/ros2_control/doc/introspection.html).

The candidate helper generates bounded finite `(linear.x, angular.z)` marker
tuples inside the unchanged trusted speed limits, with spacing greater than the
declared comparison tolerance. MotionGate intentionally republishes its
selected tuple every 20 ms, so repeated values are normal outside the decisive
window. The Gate-kill attempt starts only on the first `COMMITTED` publish of a
marker never used earlier in that generation. A matching non-zero
`/cmd_vel_out` must ACK it, and the harness must dispatch exact SIGKILL before
the next 20 ms Gate publish. If another output record appears before actual
death, or the marker appeared earlier, the attempt is ambiguous and retries
with a fresh generation.

The Gate's default-off, parent-owned event journal has one non-wrapping global
`journal_seq`, a generation, checksum, and Linux `CLOCK_MONOTONIC` timestamp
for every record. Its control lane records applied control transitions and
automatic terminal retirement with before/after `control_seq`, reason, and
lease identity. One Core-owned transition wrapper writes `INTENT`, samples the
explicit linearization time immediately before the bounded non-blocking state
mutation, captures its after-image, and then marks the slot `COMMITTED`; Node
callbacks never duplicate that protocol. Its output lane atomically appends an
`INTENT` with a same-host monotonic pre-call timestamp before each final
command publish containing non-wrapping attempt and intended-success
sequences, the exact pre-publish Core before-image, header stamp, marker, and
checksum. Only after the DDS publish call succeeds does that same slot become
`COMMITTED`; the post-publisher commit path is `noexcept`. Journal writes use
release stores, and the parent reads them with acquire ordering only after
`ProcessExited`. The bounded shared-memory journal survives exact SIGKILL.

Control-transition `event_code` values are stable ABI facts:

| Code | Transition |
| --- | --- |
| `1` | `PREPARE` |
| `2` | `OPEN` |
| `3` | `RENEW` |
| `4` | explicit `INHIBIT` |
| `5` | automatic lease retirement; `reason` identifies expiry or candidate failure |
| `6` | `FAULT`, including sequence exhaustion |

The Core is non-copyable and non-movable, but that object trait is not the
journal ownership proof. Each journal generation permanently issues at most
one move-only transition capability; `MotionGateCore` consumes it, its
transition entry point is private to the Core, and a second claim is rejected
even after the first Core is destroyed. The explicit journal-bound Core
constructor rejects an empty capability; only the separate three-argument
constructor selects no-journal mode. Every fenced mutation is compile-time
constrained to a bounded `noexcept` callback. Journal reservation failure has
two deliberately different policies: admission/extension transitions
(`PREPARE`, `OPEN`, and `RENEW`) do not mutate, while safety-terminal
transitions (`INHIBIT`, automatic retirement, and `FAULT`) still inhibit/fault
and select zero. The latter invalidates the evidence generation through the
latched journal fault, but evidence capture is never allowed to veto a safety
mutation.

The lifetime contract is thread-confined: production constructs the Attached
Journal before the Core and destroys the Core/capability before unmapping the
Journal. Sequential reverse destruction is also fail-safe because destroying
the Journal first detaches the capability; later admission fails before
mutation, while a forced safety fault still selects zero. Concurrent Journal /
Binding destruction is not claimed safe and is outside the single-threaded
Node composition.

The parent supplies the complete expected owner UID, non-zero generation,
non-zero 128-bit nonce, capacity, and exact region size. The attacher validates
its configuration, then opens an existing object only, checks the same fd is a
regular object owned by the expected UID with exact mode `0600`, link count
one, and exact size, and maps it. It reads no ordinary Header field before
`GateEventJournal` acquire-loads `READY`; only after the complete ABI,
identity, capacity, checksum, and empty-slot validation does it atomically
claim `writer_pid`. It never creates, truncates, unlinks, or clears the parent
object. Same-process tests are Layer 1. The permanent Layer-2 guard uses a
Python parent that creates the object directly through libc, initializes the
ABI, and release-publishes `READY`; a separately executed C++ child attaches,
claims its real PID, waits while the parent unlinks the name, and then commits
through the surviving mapping. The parent acquire-loads `COMMITTED` and checks
the complete record with an independent CRC64 implementation after child exit.

The Node descriptor grammar is exactly:

```text
v1:<owner_uid_decimal>:<generation_decimal>:<capacity_decimal>:<nonce_32_lower_hex>
```

It contains no whitespace, signs, empty or extra fields. Decimal fields are
canonical: only zero may begin with `0`. Owner UID must equal `geteuid()`;
generation is non-zero; capacity is `1..16384`; nonce is exactly 32 lowercase
hex digits and is not all zero. The name remains
`/voice_nav_gate_<32-lower-hex>`; that suffix is an independent opaque locator
and is not required to equal the descriptor nonce. The parser consumes only
these out-of-band parameters and never reads ordinary mapped Header fields
before the Journal's `READY` acquire. A malformed descriptor, wrong object, or
partial configuration fails before any MotionGate product publisher, service,
subscription, or timer is created. The `rclcpp::Node` base may already own
framework parameter, time-source, or logging entities; those are not part of
the product endpoint claim.

Each successful terminal transition exposes its committed `journal_seq` in
the Core snapshot as `output_cause_transition_journal_seq`; Prepared/Armed
states and unjournaled safety fallback expose zero. The first successful
COMMITTED final-zero output binds and consumes that exact cause while the
journal lane remains valid. A DDS failure retires that lane; the resulting
Core fault establishes the post-failure terminal cause, and the first
successful direct zero consumes it locally without claiming a COMMITTED cause
binding. Later direct zeros use cause zero. A constructor-time
`ConfigurationInvalid` state is explicitly an initial state rather than a
transition, consumes no slot, and has cause zero.

Output `event_code=1` means `FINAL_COMMAND_PUBLISH` within record kind
`OUTPUT_ATTEMPT`; `flags=0` in ABI v1. `output_attempt_seq` starts at one and is
consumed by every planned journal transaction, including a reservation or DDS
failure. `intended_output_seq` is the next successful Node publish sequence and
may be reused after a failed DDS call; successful publish and zero sequences
never wrap. ROS signed seconds are widened to `int64_t` and then stored modulo
2^64, nanoseconds are zero-extended, and command doubles use their exact IEEE
754 bits. Reason, state/control sequences, lease words, Gate-instance words,
and terminal cause all come from one post-fail-close, pre-publish Snapshot.

Journal failure never vetoes stopping. A pre-publisher journal failure forbids
the selected non-zero output, faults the Core, retires the journal lane, and
attempts one unjournaled zero. A DDS exception leaves the slot at `INTENT`,
faults the Core, retires the journal lane and that evidence generation, and
also attempts one direct zero. A sequence boundary faults without wrap and
continues direct zero attempts. Every successful DDS publish advances the
successful Node output counters, including an unjournaled safety-zero fallback.
While the journal lane is usable, its `noexcept` commit occurs before those
counters advance; an unjournaled success invalidates crash evidence but does
not erase the fact that the Node published a command.

Gate journal ABI v1 is a little-endian C layout with a 128-byte header and
256-byte fixed slots. All three checksums use CRC64-ECMA-182, polynomial
`0x42F0E1EBA9EA3693`, init/xorout zero, non-reflected processing, and feed each
`uint64_t` as eight least-significant-first bytes. Coverage and order are
normative:

- header: `magic`, `abi_version`, `header_bytes`, `slot_bytes`,
  `region_bytes`, `capacity`, `owner_uid`, `generation`, `nonce_hi`,
  `nonce_lo`, `reserved`; it excludes `init_state`, `claimed_slots`,
  `overflow_latched`, `writer_pid`, and `header_checksum`;
- output INTENT: `record_kind`, `journal_seq`, `generation`,
  `intent_monotonic_ns`, `event_code`, `reason`, `before_state_seq`,
  `before_control_seq`, `output_attempt_seq`, `intended_output_seq`, ROS stamp
  seconds/nanoseconds, linear/angular value bits, before-lease words,
  Gate-instance words, `cause_transition_journal_seq`, and `flags`; it excludes
  phase, transition/commit time, both checksum fields, all after-image fields,
  and reserved words;
- COMMITTED: `intent_checksum`, then the complete INTENT sequence above, then
  `transition_linearization_ns`, `commit_monotonic_ns`, after state/control
  sequences, and after-lease words; it excludes phase, `commit_checksum`, and
  reserved words.

The version-controlled oracle combines fixed externally calculated fixture
values with an include/exclude mutation matrix; comparing the production helper
only with itself is not acceptance evidence.

After `ProcessExited`, the final slot must be that single non-zero `COMMITTED`
record. A trailing `INTENT`, overflow, wrap, corrupt checksum, gap, a second
publish of the final marker, or any later output record invalidates the
generation. Because the marker was never used earlier, a matching non-zero
`/cmd_vel_out` observed before the second Gate period is the controller-update
ACK for this one publish. Its header stamp is therefore the exact timeout
origin. A marker seen only by a side input observer, one already used in the
generation, or an ACK arriving after a repeated Gate publish is insufficient.

After MotionGate exits, the observer still waits for its input publisher count
to reach zero and a 100 ms steady quiet barrier. That barrier is cleanup and
late-traffic evidence only; it is never the timeout origin.

### Lossless hardware-write ledger protocol

“Lossless” is executable, not an adjective:

- the write seam owns a monotonically increasing, non-wrapping
  `uint64 write_seq`; every invocation obtains exactly one sequence value and
  checks both finite wheel commands against the currently armed predicate;
- one active accumulator contains generation, first/last `write_seq`,
  invocation count, simulation stamp, delegated return result, and exact
  wheel-command bit patterns. Only a consecutive invocation with the
  identical tuple may atomically extend its last sequence/count. A tuple
  change or `SEAL` finalizes the segment; only then is it immutable. Its count
  must equal
  `last_seq - first_seq + 1`, so repeated paused writes remain individually
  accounted without consuming one slot each;
- `ARM` and `SEAL` create atomic sequence fences at that same seam; the test
  analyzes the closed inclusive interval and never infers boundaries from
  receipt time;
- preallocated segment capacity is proven before arming from the bounded
  write-invocation/command-transition budget; valid identical-stamp repeats
  fold into the current segment. Any value/stamp/result change that cannot
  create a segment, overflow, overwrite, wrap, unaccounted invocation, or non-zero
  write in a zero-required interval latches an oracle fault and invalidates the
  case;
- finalized fenced segments are read through immutable bounded snapshot pages;
  snapshots expose only finalized data. Page checksum, generation/fence
  identity, segment counts, and contiguous `write_seq` ranges make loss,
  duplication, reordering, stale pages, and partial snapshots fail closed;
  simulation stamp is nondecreasing and may repeat while paused. The ledger
  does not claim Gazebo iteration; in VN-0011A World Statistics independently
  proves continuous progress, while VN-0011B correlates ARM/SEAL with one
  acknowledged, World-Statistics-confirmed exact `N -> N+1` step; and
- a sealed interval is retained until the test explicitly acknowledges it;
  later writes use separate storage/fences and cannot mutate its pages;
  neither DDS BEST_EFFORT nor an overwrite-on-full ring is an admissible
  implementation of the proof channel.

The added journal instrumentation on both paths is preallocated and uses
bounded non-blocking atomic stores only; it performs no allocation, filesystem
I/O, logging, ROS calls, or transport publication in the Gate publication
critical section or after the delegated hardware `write()`. This does not
claim that pinned upstream `GazeboSimSystem::write()` is allocation-free.
Snapshot pagination runs outside those instrumentation paths.

## Exact crash ledger

- The test keeps the exact `ExecuteProcess`/`Node` action returned by launch.
- It sends `SignalProcess(SIGKILL, matches_action(exact_action))`; no process
  name, global PID lookup, shell, `pkill`, or substring matcher is allowed.
- The matching `ProcessExited` event is the death linearization point. Sending
  the signal is only intent.
- Predeclared killed actions must exit exactly `-SIGKILL`.
- Every other launch-managed action must exit zero after deterministic
  teardown. There is no `[0, -9]` allowlist and no Gazebo exception.

## Static and mutation contracts

Tests must reject at least:

- authority and candidate collapsed into one process;
- candidate with a FQN other than `/collision_monitor`;
- wrong kill target, SIGINT/SIGTERM, process-name broadcast, or global kill;
- signal-send time or producer publish time used instead of exact
  `ProcessExited` observer time for Gate latency;
- a terminal state that reuses the retired lease, has the wrong Gate instance,
  does not equal the journaled terminal sequence, whose terminal transition is
  not exactly one non-wrapping step after the last committed intervening
  control transition, substitutes a post-hoc commit time for the transition
  linearization fence, or reuses an armed zero/output sequence;
- a test-supplied zero after MotionGate death;
- Gate respawn or Gate exit shutting down Gazebo;
- broad forced-exit allowlists or unclassified launch actions;
- `cmd_vel_timeout` other than 0.35 s, controller update rate other than
  100 Hz, or controller evidence measured with wall time;
- the last periodic `/cmd_vel_out` non-zero stamp used as timeout origin
  instead of the last non-zero Gate input header stamp;
- a side observer's last input, publisher disappearance, or quiet period used
  as controller-acceptance evidence instead of the final crash-resilient
  exactly-once final-marker COMMIT plus its pre-repeat controller-output ACK;
- authority/candidate latency measured with ROS simulation time;
- omitted wheel command introspection, wheel state, or odometry hold evidence;
- default/reliable introspection QoS, no complete pre-fault four-field sample,
  a retained-zero baseline, or non-increasing simulation stamps;
- a concrete `GazeboSimSystem` subclass, non-delegating/copy-pasted upstream
  behavior, a test transformer that replaces zero or multiple hardware blocks,
  or any Adapter/journal reference in product Xacro, launch, or YAML;
- lossy introspection used to claim exact first-wheel-zero or no-regression,
  an omitted/incomplete lossless hardware-write ledger, a missing/duplicate
  final marker, a trailing/absent Gate-output-journal commit, stale arming sample, a Gate
  steady barrier longer than 40 ms, a Gate state older than 20 ms at signal
  dispatch, a simulation surface older than
  30 ms simulation time, stalled simulation, delayed SIGKILL, a Gate journal
  that omits an intervening accepted RENEW, a terminal transition fence or
  zero-output pre-call `INTENT` timestamped before `ProcessExited`, or a
  journal protocol scattered across Node callbacks; DDS receipt order alone
  must also fail;
- a ledger without monotonic sequence/fences, bounded capacity proof,
  overflow/overwrite fault, immutable paged snapshot, checksum, or contiguous
  sequence/generation validation, or one that fabricates hardware-record
  iteration or requires simulation stamp to increase for every paused write;
- an unqualified allocation-free claim over delegated upstream `write()`
  instead of only the added journal instrumentation;
- skipped active/post-shutdown cases, early returns, or non-deterministic
  cleanup.

## Isolation and cleanup

The crash test is separate from Lesson 0009's normal-running product test. It
uses the repository's process-scoped ROS domain lease, a runtime-unique Gazebo
partition, serial CTest execution, the structured Gazebo stop protocol, and
failure-safe LIFO teardown. It must never signal or otherwise mutate the
pre-existing user-owned Gazebo process.

## Non-goals

- SafePauseCoordinator, Gazebo pause/unpause, tokens, or restart policy; those
  belong to VN-0011B.
- A production MissionRuntime process or a claim that Runtime has been killed.
- Public IDL, a new package, product nodes, changed Gate behavior, or new
  trusted parameters.
- Functional-safety certification or hardware emergency-stop semantics.

## Risks and rollback

- Dynamic helper processes can escape normal launch accounting. The launch
  action itself remains the identity and the exhaustive ledger owns every
  exit.
- Introspection is diagnostic evidence, not a control path. Failure to observe
  the complete finite field set fails the test; it never changes motion.
- Simulation stamps can repeat or arrive out of order. Pure evidence analysis
  rejects non-increasing samples instead of silently sorting them.
- Reverting A removes only the new test/support/composition seam and leaves the
  Lesson 0009 product baseline unchanged.

## Test plan

- Unit: crash ledger exact-action/exact-exit semantics, Gate terminal-state
  linearization, source INTENT/COMMITTED recovery, hardware-journal
  fences/gaps/repeated simulation stamps plus World-Statistics correlation,
  marker ACK correlation, and
  simulation-window evidence analysis.
- Cross-process contract: parent-owned POSIX object, release/acquire `READY`,
  exact child PID claim, parent-only unlink, post-unlink child commit,
  post-exit parent validation through the existing mapping, independent CRC64,
  and idempotent cleanup with no `/dev/shm` residue.
- Contract: product launch topology, helper separation, clocks, controller
  parameters, evidence surfaces, generated test inventory, and mutation
  resistance.
- Integration: one isolated headless launch, in order: authority, candidate,
  then MotionGate SIGKILL.
- Repetition: bounded fresh launches after GREEN, followed by canonical
  `bash scripts/verify.sh` on the exact final local/pushed HEAD.

## Documentation

- [VN-0011 umbrella](0011-crash-stop-and-safe-pause.md)
- [Lesson 0010](../../course/lessons/0010-prove-crash-stop-and-safe-pause.md)
- [Lesson 0010 record](../../course/records/0010-crash-stop-and-safe-pause.md)
- [Motion safety contract](../architecture/safety-and-motion-contract.md)
- [Testing strategy](../process/testing-strategy.md)
- [Differential-drive reference](../../course/reference/differential-drive-contract.md)

## Verification evidence

- `2e052e1` is the isolated Layer-2 RED. In
  `/tmp/vn-cross-red-20260802-001`, the exact registered CTest executed and
  failed 1/1 because the required attach-probe executable did not exist. Shell
  wrapper failures observed before that invocation are not product RED.
- `8dd79a3` is the Layer-2 GREEN. The exact cross-process CTest passed five
  consecutive executions. It proved that `writer_pid` equals the real child
  PID, parent unlink followed by `ENOENT`, a child COMMITTED record after
  unlink, complete parent validation after child exit, and independent
  header/INTENT/COMMITTED CRC64 checks. A post-test
  `/dev/shm/voice_nav_gate_*` inventory was empty.
- `63a5ee2`, `825c662`, and review correction `0034c9d` close the Core journal
  transition matrix. The 21-test GTest suite proves all four automatic
  retirement reasons, full-journal admission-versus-safety policy, a live
  candidate at 249 ms followed by authority expiry at 250 ms, preservation of
  a selected non-zero command when an unrecordable RENEW is rejected, one
  COMMITTED provider-fault record with no replay duplication, and a repeated
  `force_fault()` that changes neither Snapshot nor overflow state.
- With both `/opt/ros/jazzy/setup.bash` and the workspace overlay sourced, the
  complete `voice_nav_mission` CTest gate passed 16/16, including launch, C ABI,
  cross-process, static-analysis, formatting, and XML/Python checks.
- `fb42e9d` and `011beb6` preserve the real-Node output-journal and static
  Runtime-ownership REDs. `c58b6fb` delegates final publication through the
  package-private Runtime transaction and removes the Node-owned publication
  mutex, counters, selected-command argument, and duplicate DDS-failure
  fallback. On that exact implementation tree, the repository MotionGate
  contract passed 56/56, the focused Runtime/cross-process/two-Node-launch set
  passed 4/4, the package built, and all 18 `voice_nav_mission` CTests passed.
  The valid journal launch observed an independently checksummed COMMITTED
  zero-output record from the exact child PID; both partial configurations
  remained unclaimed and exited 1 as designed.

These are component and Layer-2 contract facts, not observed product crash
evidence. The repository topology contract remains deliberately RED until Node
composition and the Gazebo Adapter exist. Exact final local/pushed HEAD, PR,
CI, rebase/public tree, and Issue closure identities remain pending under the
delivery identity policy.
