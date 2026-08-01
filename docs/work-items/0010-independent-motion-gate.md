# VN-0010: Add fail-closed MotionGate authority

**Status:** Done

**GitHub Issue:**
[#11](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/11)

**Branch:** `feat/vn-0010-l0009-motion-gate`

**Implementation state:** reviewed feature head
`956fde31841a8c2ded6b37e905a8352afd10cdce` passed the exact-head local gate
and required GitHub Actions run `30651076176`. PR #12 was rebase-merged on
2026-08-01T04:22:41Z as public tip
`c2d7631b57001a6e3e360f9bbdb558de6b7a85ed`; its tree is identical to the
reviewed feature tree. Immutable annotated tag `course/0009-solution` has tag
object `f31c559d30a5876f036991bacf357b00ea1ea128` and peels to that public tip.
The authoritative 58-commit local-to-public map is in the
[Lesson 0009 record](../../course/records/0009-independent-motion-gate.md).

## Goal

Deliver the smallest independent final-velocity authority that can be verified
without Mission Runtime:

```text
test-only authority and candidate harness
  -> package-private typed lease control
  -> one per-lease TwistStamped topic and one bound writer GID
  -> motion_gate_node
  -> /diff_drive_controller/cmd_vel
  -> diff_drive_controller
  -> gz_ros2_control / Gazebo
```

The canonical product bringup starts MotionGate inhibited. A valid lease and a
fresh candidate can produce bounded differential-drive motion. `INHIBIT`, an
expired 250 ms authority lease, a stale 150 ms candidate, a non-finite value,
an unsupported non-zero axis, or a broken topic/GID invariant selects zero
without depending on ROS time. MotionGate is the only publisher to the final
controller command endpoint in the product composition.

VN-0010 is Lesson 0009. Lesson 0010 will use a separate Work Item for
process-death crash-stop and managed Gazebo pause/resume evidence.

## Stable and private contracts

### Public product Interface

This Work Item changes no type in `voice_nav_interfaces` and adds no public
Mission operation. `StopMission`, Mission Runtime, Nav2, relative-motion
execution, mapping, and voice remain later slices.

MotionGate's ROS control seam is package-private and unsupported outside
`voice_nav_mission`:

```text
voice_nav_mission/srv/InternalMotionGateControl
voice_nav_mission/msg/InternalMotionGateState
```

Installing these types so two processes can communicate does not make them a
stable public product Interface. Their naming and documentation are an
ownership rule, not authentication: a local DDS participant is inside the
trusted simulation environment unless a later security Work Item says
otherwise.

### Deep MotionGate Core

`MotionGateCore` is an in-process deep Module built as the package-internal
static target `motion_gate_core`. Its header and library are not installed or
exported; only the `motion_gate_node` executable is installed. Its typed
control/data surface is `prepare`, `open`, `renew`, `inhibit`,
`accept_candidate`, `tick`, `snapshot`, and the read-only
`selected_command`. `force_fault` is the Adapter-only fault-ingress method
used when a ROS graph, clock, publication, or reader invariant fails.
`PrepareAdmissionProvider` and `OpenBindingProvider` are internal seams through
which the Adapter supplies bounded external facts. The implementation owns:

- the state machine and one global compare-and-swap `control_seq`;
- generation of the opaque lease ID and per-lease topic;
- authority and candidate deadlines;
- one 16-byte writer GID binding;
- finite-value checks, supported-axis checks, and configured clamps;
- retirement of an invalid or expired lease;
- the selected final command and pure state/result decisions.

The Core has no ROS graph calls, publisher, subscription, executor, ROS time,
sleep, or filesystem access. Production supplies steady time; unit tests
supply a manual clock. Callers and tests cross the same Core Interface rather
than reaching into private state.

`motion_gate_node` is the ROS Adapter. It owns graph discovery,
`MessageInfo.publisher_gid`, the dynamically recreated candidate reader,
private control/state endpoints, the final publisher, and one serialized
event/publication path. It also owns actual final/state publication,
`output_publish_seq`, `zero_publish_seq`, and the response's `zero_published`
fact; those are not Core state. Its exact FQN is `/motion_gate_node`; its private
absolute endpoints are `/motion_gate/internal/control` and
`/motion_gate/internal/state`. `InternalMotionGateControl` never transports a
writer GID: a GID obtained from another process is not a portable authority
token.

### Locked control protocol

The only control operations are:

```text
PREPARE -> OPEN -> RENEW* -> INHIBIT
```

Every request identifies the current Gate instance and supplies the caller's
expected global `control_seq`. An accepted transition advances the one
Gate-wide sequence and returns its new value. `OPEN`, `RENEW`, and `INHIBIT`
also identify the current lease. A stale instance, lease, or compare-and-swap
cannot change Gate state: in particular, a late `INHIBIT` for an old lease
must not stop a newer lease. A valid `INHIBIT` for the current tuple selects
zero before returning. The later public `StopMission` path remains
unconditionally safety-effective because Runtime first linearizes STOP, then
reads and inhibits the **current** Gate tuple.

The IDL transport bound for `request_id`, `gate_instance_id`, and `lease_id`
is 36 characters. Their runtime semantic is narrower: request and Gate
identities must be exactly 32 lowercase hexadecimal characters; PREPARE must
carry an empty lease, while OPEN/RENEW/INHIBIT require an exact
32-lowercase-hex lease. Uppercase, hyphenated, short, or non-hexadecimal
identities are rejected before state mutation.

Protocol semantics:

1. `PREPARE` is accepted only while inhibited. Its injected admission provider
   first waits for the retired writer to disappear. The Gate then generates a
   new opaque lease ID and a bounded topic below
   `/voice_nav_internal/motion_gate/candidate/lease_<gate-generated-id>`,
   creates discard-only reader A, and remains inhibited.
2. The caller creates exactly one candidate writer on the returned topic.
3. Core `open` first validates idempotence, identity, operation, state, CAS,
   lease, and prepare deadline. Any rejected request returns before the binding
   provider runs, so it cannot query the graph or mutate a reader.
4. Graph snapshot #1 requires exactly one publisher endpoint and a healthy
   final controller. The Adapter records the complete Gate-observed 16-byte
   GID; the request contains no writer GID.
5. The Adapter destroys reader A and its queue, creates discard-only reader B,
   then takes graph snapshot #2. The same unique writer GID must remain.
6. Only after those two snapshots does Core atomically enter `ARMED`, with the
   selected output still zero. The Adapter then destroys B and creates reader
   C, the first accepting `VOLATILE + KEEP_LAST(1)` reader.
7. Graph snapshot #3 must still contain the same unique writer and a healthy
   final controller. A change faults the Gate and selects zero; success is not
   returned before this check. Reader C accepts only samples whose Gate-local
   `MessageInfo.publisher_gid` equals the bound graph endpoint GID.
8. `RENEW` advances only the 250 ms authority deadline. It cannot change the
   topic, writer, limits, or candidate freshness.
9. `INHIBIT` retires the current lease permanently, selects and publishes zero
   through the serial publication barrier, destroys the old reader, and only
   then acknowledges inhibited state.

An expired, retired, or Gate-restart lease cannot be reopened. Another motion
attempt starts again at `PREPARE` and receives a different Gate-generated
lease and topic. Before a new lease opens, the old writer GID must disappear
from the old topic's graph; a bounded failure leaves the Gate inhibited.

### Candidate and output contract

The candidate data plane is
`geometry_msgs/msg/TwistStamped`,
`BEST_EFFORT + VOLATILE + KEEP_LAST(1)`. Candidate samples never renew
authority. MotionGate records candidate freshness from steady receipt time
after topic, state, OPEN barrier, and writer-GID validation; the untrusted
candidate header stamp is not a safety clock.

Only `linear.x` and `angular.z` may be non-zero:

- finite values on those two axes are clamped to trusted YAML bounds;
- a finite over-limit value is not a fault and is never forwarded unclamped;
- NaN, Inf, or any unsupported non-zero axis retires the lease and selects
  zero;
- a writer mismatch, ambiguous writer set, stale candidate, or stale authority
  also retires the lease and selects zero.

The final publisher uses `rclcpp::SystemDefaultsQoS()` to match the pinned
Jazzy controller subscription and publishes directly to the fixed
`/diff_drive_controller/cmd_vel` endpoint. Product bringup does not remap it.
Runtime verification proves actual endpoint compatibility; it does not
hard-code reliability/history/depth assertions when DDS introspection reports
those policies as `UNKNOWN`. MotionGate rewrites the outgoing ROS stamp for
the controller, while all Gate deadlines use steady time.

One serial publication barrier orders control transitions, candidate
selection, timer expiry, state publication, and final velocity publication.
No callback publishes directly. Once a retirement or `INHIBIT` zero crosses
that barrier, an earlier queued non-zero decision cannot publish afterward.
While inhibited, a 20 ms wall timer continuously publishes zero.

The private state snapshot uses
`RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)` and includes:

- Gate instance and global `control_seq`;
- `INHIBITED`, `PREPARED`, `ARMED`, or `FAULTED` state;
- current lease ID and derived candidate topic while one exists; retirement
  clears the lease, topic, and bound GID, while bounded `reason`/`detail`
  preserve the diagnostic;
- whether authority, candidate freshness, and writer binding are valid;
- the bound 16-byte GID when present, as run-local diagnostics only;
- output sequence, zero flag, and a bounded typed reason.

A locked Fast DDS/RMW self-test proves that the GID returned by the Gate's
graph query and the GID received in Gate-local `MessageInfo` correlate for the
same publisher. If this invariant is unavailable or mismatched,
OPEN/candidate handling fails closed. Neither the caller nor its
`Publisher::get_gid()` result participates in that comparison.

This implementation supports only `rmw_fastrtps_cpp`: canonical product
bringup sets `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `motion_gate_node` rejects
another RMW during construction, and both `voice_nav_mission` and
`voice_nav_bringup` declare the runtime dependency. This lock is part of the
Interface contract for Lesson 0009, not an assertion that GID correlation is
portable to every DDS implementation.

### Trusted configuration

The first product configuration is fixed in trusted YAML under the exact ROS
parameter root `motion_gate_node`. Node, control, state, candidate-prefix, and
final-command names are code constants; they are not YAML parameters and
product launch does not remap them.

| Parameter | Value |
| --- | --- |
| ROS time source | startup requires `use_sim_time=true`; runtime changes are rejected; every publication also requires the parameter to remain true and `ros_time_is_active()`, otherwise the Gate faults closed and publishes zero with stamp zero |
| Authority lease | `250 ms` steady time |
| Candidate freshness | `150 ms` steady time |
| Output period | `20 ms` wall time |
| Prepared handover timeout | `1000 ms` steady time |
| Writer graph timeout | `1000 ms` steady time |
| Candidate QoS depth | `1` |
| Expected candidate writer FQN | `/collision_monitor` |
| Idempotence cache bound | `64` requests |
| Linear x clamp | `[-0.20, 0.40] m/s` |
| Angular z clamp | `[-1.20, 1.20] rad/s` |
| Controller consumer timeout | existing `0.35 s` simulation/controller time |

Contract tests compare the Gate clamps with the controller limits. The Gate
may never be configured wider. The controller keeps its second hard clamp and
consumer timeout; its acceleration/deceleration limiters remain unset.

## Non-goals

- `mission_runtime_node`, public `StopMission`, Mission validation, cancel, or
  terminal-intent races.
- Nav2, relative-motion execution, velocity smoother, Collision Monitor, SLAM,
  AMCL, or map storage.
- `twist_mux`, a fifth resident watchdog process, a generic workflow engine,
  or a new ROS package.
- Authenticating arbitrary local DDS callers. Package-private is not SROS.
- Treating zero publication as proof of physical stationarity or calling this
  a certified emergency-stop system.
- Killing the Runtime/authority process or candidate process.
- Killing MotionGate and measuring the controller's 0.35 s consumer timeout.
- Managed safe-pause tokens, Gate death during pause, first-resume zero proof,
  or unmanaged-pause restart.

The last four process/pause cases are the explicit Lesson 0010 / VN-0011
crash-stop boundary. This Work Item must not describe normal lease expiry as a
Runtime process crash, nor a configured controller timeout as completed
Gate-death evidence.

## Acceptance criteria

- [x] Annotated `course/0009-start` exists locally and remotely and peels to
  reviewed Lesson 0008 closure
  `53c0a937ecc8c1d842c72f8542f19af661d620cf`.
- [x] Tests-first RED executes valid fixtures and fails only because the
  repository does not yet contain MotionGate product behavior.
- [x] `MotionGateCore` starts inhibited and is tested with a manual steady
  clock without sleeps.
- [x] `motion_gate_core` is an internal STATIC target; neither its library nor
  header is installed/exported, and only `motion_gate_node` is installed.
- [x] Core tests and callers use the real typed methods and the two provider
  seams; there is no parallel `handle(Event)` state machine.
- [x] Core tests cover global CAS success/mismatch, Gate restart, generated
  lease/topic uniqueness, PREPARE/OPEN/RENEW/INHIBIT ordering, permanent
  retirement, deadlines, candidate validation, and selected-zero behavior.
- [x] Node Adapter tests cover the A/B/C OPEN reader-queue barrier and the
  zero-before-ack publication serial barrier; those ROS side effects are not
  attributed to the pure Core.
- [x] A stale `INHIBIT` with an old Gate instance, lease, or `control_seq`
  cannot inhibit a newer lease; a matching current `INHIBIT` publishes zero
  before acknowledgement.
- [x] Authority is live immediately before 250 ms and retired at the 250 ms
  boundary; a late RENEW cannot resurrect it.
- [x] Candidate freshness is independent: continuing candidates cannot renew
  authority, and continuing renewals cannot make a candidate older than
  150 ms valid.
- [x] Finite `linear.x`/`angular.z` values are clamped exactly to trusted
  limits. NaN, Inf, or a non-zero unsupported axis retires the lease and
  selects zero.
- [x] The candidate reader uses a Gate-generated per-lease topic. OPEN performs
  pure Core validation before graph access, crosses discard readers A/B and
  accepting reader C, observes the same unique 16-byte writer GID in exactly
  three graph snapshots, discards pre-OPEN queued samples, and rejects
  old-topic or unbound-GID samples.
- [x] The control request carries no writer GID. A locked-RMW self-test proves
  Gate-local graph-GID to `MessageInfo` correlation; mismatch fails closed.
- [x] The old writer disappears before another lease opens; failure remains
  inhibited and has a bounded typed diagnostic.
- [x] `INHIBIT` publishes zero through the serial barrier before the control
  response reports inhibited. Inhibited mode continues zero every 20 ms.
- [x] Private IDL is bounded, remains in `voice_nav_mission`, and is absent
  from `voice_nav_interfaces`.
- [x] The 36-character IDL bounds do not weaken the runtime rule: request and
  Gate-instance IDs are exactly 32 lowercase hexadecimal characters; PREPARE
  carries no lease, and the other operations require an exact-32 lease, with
  negative tests for case, hyphens, length, and alphabet.
- [x] Product bringup loads trusted YAML, includes the lower-level simulator,
  launches `motion_gate_node` inhibited, and contains no test authority
  process or direct final-command bypass.
- [x] Product bringup selects `rmw_fastrtps_cpp`; Gate startup rejects another
  RMW, manifests declare it, and both launch-test layers assert the active RMW.
- [x] `use_sim_time` cannot be changed after startup. A moving-node test
  rejects `false`, preserves ROS-time stamps without a zero pulse, and the
  publication path independently faults closed if either the parameter or
  `ros_time_is_active()` invariant is lost.
- [x] The final publisher uses `rclcpp::SystemDefaultsQoS()` and is proven
  compatible with the actual controller subscriber without treating
  introspection `UNKNOWN` values as fixed QoS facts.
- [x] In the product composition MotionGate is the sole publisher endpoint on
  `/diff_drive_controller/cmd_vel`; owner FQN is exactly
  `/motion_gate_node`, and its publisher GID remains stable for the observation
  window.
- [x] A headless valid lease produces bounded forward odometry. Separate
  evidence records Gate zero, controller limited-output zero, and odometry
  stationarity after `INHIBIT`.
- [x] With valid-looking candidates still arriving, stopping renewals causes
  Gate zero no later than 300 ms steady time after the last accepted renewal.
- [x] With renewals continuing, stopping candidates causes Gate zero no later
  than 200 ms steady time after the last accepted candidate.
- [x] Repository contracts, interface generation, build, package tests,
  headless integration, full verification, and guarded process-residue audit
  pass.
- [x] Acceptance is recorded as three executable layers: pure Core GTest,
  no-Gazebo/no-`/clock` Node launch test, and headless-Gazebo product launch
  test. Static repository guards are recorded separately as prerequisites.
- [x] Lesson 0009, its evidence record, architecture/current-status documents,
  and `CHANGELOG.md` match the implemented slice.
- [x] A reviewed PR passes required hosted CI and is rebase-merged before
  annotated `course/0009-solution` is created.

## CI-readiness remediation

The first hosted run exposed two eventually-consistent readiness assumptions
in the test harnesses; it did not expose a product MotionGate safety failure.
The history is intentionally preserved rather than replacing the failed run
with its rerun:

- CI run
  [30625857309, attempt 1](https://github.com/Edddddddddy/voice_nav_robot_ws/actions/runs/30625857309/attempts/1)
  tested pre-remediation head
  `c8b9eefed5729a9a2ab2a60a9d8302e697beeb70` and failed. The simulation
  harness timed out waiting for the `/controller_manager/list_controllers`
  response, and the product harness observed `candidate topic has no writer`.
  The aggregate was 136 tests, 0 errors, 4 failed records, and 8 skipped; the
  four records represented those two failing launch cases.
- [Attempt 2](https://github.com/Edddddddddy/voice_nav_robot_ws/actions/runs/30625857309/attempts/2)
  reran the same workflow and same SHA without a code change and passed with
  136 tests, 0 errors, 0 failures, and 8 skipped. This demonstrated a flake;
  it is not evidence that the later remediation worked.
- `b82cb736d358f1ce6374373efabc692a3426bd83`,
  `3ce547f90b6bed000512cda81c99db3233d145f1`, and
  `e86a07ddae17f65c0cd040fbdda3e05420346bbc` added tests-first contracts for
  bounded fail-closed OPEN convergence, exact retry classification, and
  isolated simulation runtime.
- `e984433c80d9f9a2afa81011c8d606ccf8a3c79e` implemented the harness-only
  readiness remediation: PREPARE starts a one-second absolute steady deadline;
  OPEN retries only the exact `REJECTED/WRITER_UNAVAILABLE/candidate topic has
  no writer` state while every intermediate snapshot remains inhibited; the
  simulation controller-list response gets its own 15-second budget.
- The first full local run after that implementation proved all six pure
  convergence cases in 0.33 seconds, but the outer five-second CTest wrapper
  expired during Python teardown at 5.01 seconds. Its raw workspace aggregate
  was 221 result records, 0 errors, 1 failure, and 12 skipped; later review
  proved that aggregate also included 74 records from a stale build directory.
  Commit
  `bf1f6ac9650222cecbdbd2e5777caf9f00748cca` raised only that wrapper budget
  to 30 seconds and added a regression guard; it did not widen product timing.
- Independent review then found a cross-package ROS-domain collision and an
  attempt-count cap that could stop before the promised absolute deadline.
  `a5fb71e92560a0d9e7f5f5bd5ceb3794a8b1e5fd` first exposed both gaps, and
  `d5392d30efcc975cd280f8af6e1d7a433184d0ff` assigned mission/product/sim
  domains 91/92/93 and made the positive-sleep absolute deadline the sole
  convergence terminator. Code and safety re-review reported no remaining
  finding on that code head.
- Evidence review then found that the otherwise successful `d5392d30` run's
  raw `222/0/0/12` aggregate was not a clean six-package count: default
  `colcon test-result` had recursively added 74 records and 4 skips from
  `build/voice_nav_mission.stale-l0009` to the current 148/0/0/8 results.
  The successful build, runtime assertions, product metrics, clean-install
  audit, and final marker remained valid; the 222 count did not.
- `caf5cd923cdd27f06e7ac7b7f8c1fbdfe25495f0` and
  `9d968ae236325a19deb0236749ea715f4c05c42f` first specified scoped reporting
  and fail-closed build-boundary behavior. `c5d88c24b8e2361bf1403e314fb04e7fd604629f`
  added exact-package result clearing/reporting, and
  `8e022580a7add59a9c5d5a95973182322a0641c0` completed the boundary: current
  package names come from `src`, unexpected top-level build directories and
  direct symlinks fail before build and before result collection, and no
  generated directory is silently deleted.

Review of the evidence boundary then drove a second tests-first hardening pass:

- `72abf22` / `27ff84a` reject path escape and stage each selected package in
  a private sandbox; `74a05b8` / `4958b7c` run repository contracts only
  after ROS is available in hosted CI.
- `ec13bb3` / `527a0b5` add the third build-boundary check after package tests;
  the three checks are now before build, after build and before clear/test,
  and after tests and before the final report.
- `a1bd5b5` / `bf0c1fe` propagate `colcon list` failure;
  `10ede88` + `5a16670` / `516c846` constrain evidence package names to the
  colcon-compatible grammar; `afce577` / `19de986` make ROS setup nounset-safe.
- `f8c5a88` / `73fd263` and `9abd0c7` / `4bbd92d` require every CTest TAG to
  reference an existing local XML document that the CTest adapter consumes.
- `108ddfa` / `ff44a87`, `34d9005` / `953a14a`, and
  `761730a` / `c2954ca` anchor deletion and snapshot reads to opened directory
  descriptors while preserving precise, leak-free descriptor ownership.
- `5286b36` / `b1bd863` close the complete result-directory manifest;
  `0b385e1` / `fe7b844` carry package/file identities from parsing into clear;
  `4f1cab0` / `cc908fb` require every mandatory xUnit file to be consumed.
- `f50aa3a4d78aa63fe6f0ff46133a0111aa7d804d` records the explicit mutation
  threat model rather than claiming a stronger security boundary.
- `7c2f9f3` first reproduces hidden failures behind a symlinked result
  directory and an over-broad nested `package.xml` exception. `93d968e` then
  rejects every directory symlink except the two explicit non-evidence paths
  emitted by the locked ament layouts, and limits the XML exception to the
  package-build-root `package.xml`.
- `e4c10c0` / `061d88c` bind those ament paths to their exact source-module or
  in-package rosidl target and validate the source manifest root and name.
  `3dc4fa3` / `0b77d4a` additionally require direct, non-symlinked `src` and
  source-package directories and normalize unknown XML encodings into the
  reporter's controlled failure boundary.
- `6e04af2` / `517339a` reproduce and close absolute and relative
  `pivot-symlink/../expected` bypasses by comparing strict, actually resolved
  targets instead of lexically normalized paths.

The historical canonical local verification of code head
`8e022580a7add59a9c5d5a95973182322a0641c0` passed 140 repository tests,
built all six packages in 32.8 seconds, passed 148 scoped ROS/package results
with 0 errors, 0 failures, and 8 skips in 3 minutes 39 seconds, passed the
fresh-prefix Core/node install audit, and printed
`VoiceNav Robot verification passed.` It remains historical evidence rather
than the final hosted gate.

Focused checks on the then-current hardening head
`517339a3d313910a937fef973a9bdd635b457fc8` passed 41 scoped-evidence tests,
all 168 repository-static tests,
a read-only `148/0/0/8` scoped package report,
Python compilation, and `git diff --check`. Documentation head
`ff06e6b393f2c8d974afbcc6ebd1c6766f8de2d7`, containing that unchanged code
head, then passed the full `scripts/verify.sh` gate in 346.5 seconds: 168
repository tests, six-package build in 25.7 seconds, six-package sequential
tests in 169.7 seconds, scoped `148/0/0/8`, clean Core/node install audit, and
the final `VoiceNav Robot verification passed.` marker.

The exact product evidence at `ff06e6b` recorded 0.100500 m bounded motion;
0.400 m/s and 1.200 rad/s matching Gate/controller clamps; authority and
candidate Gate-zero latencies of 256.784 ms and 156.780 ms; explicit INHIBIT
ack/Gate-zero/controller-zero latencies of 10.190/1.187/4.009 ms; and a stable
sole final command owner. The SDF extension and existing workspace-underlay
messages remained non-blocking diagnostics; no assertion or exit-code rule was
relaxed.

Final feature head `956fde31841a8c2ded6b37e905a8352afd10cdce`
then passed `scripts/verify.sh` in 328.6 seconds: all 168 repository contracts,
the six-package build, manifest-scoped `148/0/0/8`, the clean Core/node install
audit, and the final success marker. Its product evidence recorded 0.099000 m
bounded motion; Gate/controller clamps of 0.400 m/s and 1.200 rad/s; authority
and candidate Gate-zero latencies of 258.316 ms and 157.476 ms; explicit
INHIBIT ack/Gate-zero/controller-zero latencies of 10.110/0.700/4.996 ms; and
the stable sole `/motion_gate_node` final owner. Required hosted run
[`30651076176`](https://github.com/Edddddddddy/voice_nav_robot_ws/actions/runs/30651076176)
passed that exact SHA. Independent code, safety, and documentation reviews had
no remaining P0-P3 finding, and PR #12 had no unresolved review thread before
its rebase merge.

The evidence tool assumes the selected build tree and locked source-package
layout are quiescent during clear/report. `src` and `src/<package>` must be
direct, non-symlinked directories; the two ament exceptions are exact
path/strictly-resolved-target pairs, and the source manifest root/name must
match. Anchored directory descriptors contain ordinary path replacement to the
selected package and reject symlink/path-traversal escape, but the tool
does not claim to defeat a hostile same-UID process racing the final identity
check against name-based unlink, or hardlink/bind-mount provenance forgery.
That residual risk is explicitly accepted for the controlled enterprise CI
workspace; this is not an adversarial filesystem security boundary.

## Risks and rollback

- A shallow ROS callback implementation could duplicate lease rules and make
  races untestable. Keep all state transitions in the Core Interface and all
  side effects in one Node Adapter.
- DDS discovery is eventually consistent. PREPARE/OPEN and old-writer
  disappearance use bounded steady deadlines and remain inhibited on
  ambiguity; they never choose the first convenient writer.
- A delayed pre-OPEN sample could otherwise become the first non-zero output.
  The reader-queue and publication serial barriers are mandatory test
  surfaces.
- Candidate CLAMP semantics could drift from controller limits. Cross-YAML
  contracts fail if Gate limits are wider or the locked values differ.
- Product bringup must not shut down or automatically respawn the whole
  simulator immediately when Gate exits; Lesson 0010 needs advancing
  controller time to prove the second deadman.
- Rollback removes the product bringup and MotionGate executable while keeping
  the reviewed Lesson 0008 simulator/controller baseline. It does not rewrite
  course tags or public history.

## Design impact

- Stable Interfaces changed: none.
- Private Interfaces added:
  `InternalMotionGateControl.srv` and `InternalMotionGateState.msg`.
- Motion ownership changed: product final velocity moves from direct
  test-only controller publication to sole `motion_gate_node` publication.
- TF ownership changed: none.
- Packages changed: `voice_nav_mission` and `voice_nav_bringup`; existing
  `voice_nav_sim` controller configuration remains the lower layer.
- Build/installation changed: `motion_gate_core` is an internal STATIC target;
  its header/library are not installed or exported. `motion_gate_node` is the
  only new installed runtime target in `voice_nav_mission`.
- ADR required: no. This implements ADR-0002 and ADR-0003; a deviation from
  the separate Gate process or six-package topology requires a new ADR.

Reviewed implementation paths:

```text
src/voice_nav_mission/
  msg/InternalMotionGateState.msg
  srv/InternalMotionGateControl.srv
  include/voice_nav_mission/motion_gate_core.hpp
  src/motion_gate_core.cpp
  src/motion_gate_node.cpp
  test/...

src/voice_nav_bringup/
  config/motion_gate.yaml
  launch/product_sim.launch.py
  test/...
```

## Test plan

Repository-static prerequisites run before, but do not replace, the three
runtime acceptance layers:

```bash
python3 scripts/check_motion_gate_contract.py --root .
python3 -m unittest discover -s tests -p "test_motion_gate_contract.py" -v
bash scripts/verify.sh
```

The static contract covers bounded private IDL, exact 32-lowercase-hex
semantics, operation names, internal STATIC/non-installed Core, real typed
methods/providers, trusted Gate/controller limits, exact QoS/endpoints,
FastDDS dependencies and launch lock, three-snapshot source invariants, no
public-type leakage, no cross-process writer-GID field, no `twist_mux`, and no
second production final-command source.

After the build and workspace setup, record each acceptance layer separately:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ctest --test-dir build/voice_nav_mission \
  --output-on-failure -R '^motion_gate_core_test$'

ctest --test-dir build/voice_nav_mission \
  --output-on-failure -R '^test_test_motion_gate_node.py$'

ctest --test-dir build/voice_nav_bringup \
  --output-on-failure -R '^test_test_motion_gate_product.py$'

python3 scripts/report_test_results.py \
  --build-base build \
  --package voice_nav_agent \
  --package voice_nav_audio \
  --package voice_nav_bringup \
  --package voice_nav_interfaces \
  --package voice_nav_mission \
  --package voice_nav_sim
```

1. **Layer 1 — pure Core GTest**: manual-clock tables cover every state, CAS,
   exact 249/250 ms and 149/150 ms boundaries, idempotence/collision,
   exact-32 identity validation, clamp/invalid retirement, permanent lease
   retirement, and selected-zero behavior without ROS I/O or sleeps.
2. **Layer 2 — Node without Gazebo or `/clock`**: the 60-second serial launch
   test runs with `rmw_fastrtps_cpp`, ROS domain 91, and localhost discovery.
   It covers private service/state behavior, frozen ROS time with advancing
   steady deadlines, zero/one/two writers, A/B/C readers and three same-writer
   graph snapshots, Gate-local graph/MessageInfo GID correlation, pre-OPEN and
   stale samples, and zero-before-ack publication ordering.
3. **Layer 3 — headless product**: the 180-second serial launch test runs with
   `rmw_fastrtps_cpp`, ROS domain 92, localhost discovery, and a unique Gazebo
   partition. It covers canonical composition, sole final owner, bounded and
   clamped motion, authority/freshness expiry, separate Gate/controller zero
   observations, odometry stationarity, and clean launch-managed shutdown.

Optional RViz/Gazebo observation may supplement evidence but cannot replace an
automated assertion. Process-kill, consumer-deadman, and pause/resume tests are
Lesson 0010, not an omitted fourth Lesson 0009 layer.

## Documentation

Files created by this Work Item:

- `docs/work-items/0010-independent-motion-gate.md`
- `course/lessons/0009-build-independent-motion-gate.md`
- `course/records/0009-independent-motion-gate.md`

Files updated:

- `README.md`
- `CHANGELOG.md`
- `course/README.md`
- `course/catalog.toml`
- `docs/architecture/overview.md`
- `docs/architecture/safety-and-motion-contract.md`
- `docs/process/testing-strategy.md`
- `course/reference/differential-drive-contract.md`

No release-policy, public Mission Interface, product-specification, or ADR
change is required for the locked design.

## Verification evidence

Do not pre-fill results. After implementation, record:

- start tag object/peeled target and tests-first/green/review commit identities;
- full commands and exit statuses;
- unit/contract/headless test counts and non-skipped integration result;
- exact manual-clock transition table;
- private Interface definitions, topic QoS, and trusted YAML values;
- Gate state transitions and global `control_seq`;
- per-lease topic, bound 16-byte GID, graph owner, and old-writer
  disappearance evidence, noting that GIDs are run-local;
- last accepted renewal/candidate to first Gate-zero steady latency;
- distinct Gate-zero, controller-output-zero, and odometry-stationary times;
- final controller topic endpoint/GID/FQN owner set before and after motion;
- CLAMP output and invalid-axis/non-finite retirement diagnostics;
- six-package build, package test summary, final verification marker, and
  guarded residue audit;
- Issue, PR, exact-head hosted CI, independent review, rebase identity map, and
  start/solution tag objects after those events exist.

The evidence record must copy the exact commands from the Test plan rather
than paraphrasing them. Interface and manual product inspection use:

```bash
ros2 interface show voice_nav_mission/srv/InternalMotionGateControl
ros2 interface show voice_nav_mission/msg/InternalMotionGateState
ros2 launch voice_nav_bringup product_sim.launch.py headless:=true
```

For the static prerequisite and each of the three acceptance layers, preserve
an unfilled block until that command has actually run:

```text
Layer/command: TBD after execution
Environment and active RMW: TBD after execution
Exit status: TBD after execution
Test count and skipped count: TBD after execution
Elapsed time: TBD after execution
Decisive assertion or measured metric: TBD after execution
```

Historical tests-first RED evidence remains separate from later local GREEN
and cannot be overwritten with a later test count.

Generated logs, bags, build outputs, and screenshots are not committed.
