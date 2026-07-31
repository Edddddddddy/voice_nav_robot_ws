# VN-0010: Add fail-closed MotionGate authority

**Status:** In Progress

**GitHub Issue:**
[#11](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/11)

**Branch:** `feat/vn-0010-l0009-motion-gate`

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

`MotionGateCore` is an in-process deep Module. Its small typed Interface accepts
control, candidate, and timer events and returns decisions/effects. The
implementation owns:

- the state machine and one global compare-and-swap `control_seq`;
- generation of the opaque lease ID and per-lease topic;
- authority and candidate deadlines;
- one 16-byte writer GID binding;
- finite-value checks, supported-axis checks, and configured clamps;
- retirement of an invalid or expired lease;
- the selected final command and zero-publication acknowledgement.

The Core has no ROS graph calls, publisher, subscription, executor, ROS time,
sleep, or filesystem access. Production supplies steady time; unit tests
supply a manual clock. Callers and tests cross the same Core Interface rather
than reaching into private state.

`motion_gate_node` is the ROS Adapter. It owns graph discovery,
`MessageInfo.publisher_gid`, the dynamically recreated candidate reader,
private control/state endpoints, the final publisher, and one serialized
event/publication path. Its exact FQN is `/motion_gate_node`; its private
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

Protocol semantics:

1. `PREPARE` is accepted only while inhibited. The Gate generates a new opaque
   lease ID and a bounded topic below
   `/voice_nav_internal/motion_gate/candidate/lease_<gate-generated-id>`, may
   create only a provisional/discarding reader, and remains inhibited.
2. The caller creates exactly one candidate writer on the returned topic.
3. `OPEN` requires the prepared lease, the current `control_seq`, and exactly
   one discoverable writer. In its own RMW context, the Gate records that
   endpoint's complete 16-byte GID; the request contains no writer GID.
4. At the serialized OPEN point, the Gate destroys any prepared reader and
   creates a new `VOLATILE + KEEP_LAST(1)` reader. This OPEN reader-queue
   barrier discards the old reader and everything queued on it before OPEN.
   The new reader accepts only samples whose Gate-observed
   `MessageInfo.publisher_gid` equals the Gate-observed graph endpoint GID.
5. `RENEW` advances only the 250 ms authority deadline. It cannot change the
   topic, writer, limits, or candidate freshness.
6. `INHIBIT` retires the current lease permanently, selects and publishes zero
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
- current/retired lease ID and derived candidate topic;
- whether authority, candidate freshness, and writer binding are valid;
- the bound 16-byte GID when present, as run-local diagnostics only;
- output sequence, zero flag, and a bounded typed reason.

A locked Fast DDS/RMW self-test proves that the GID returned by the Gate's
graph query and the GID received in Gate-local `MessageInfo` correlate for the
same publisher. If this invariant is unavailable or mismatched,
OPEN/candidate handling fails closed. Neither the caller nor its
`Publisher::get_gid()` result participates in that comparison.

### Trusted configuration

The first product configuration is fixed in trusted YAML under the exact ROS
parameter root `motion_gate_node`. Node, control, state, candidate-prefix, and
final-command names are code constants; they are not YAML parameters and
product launch does not remap them.

| Parameter | Value |
| --- | --- |
| ROS time source | `use_sim_time=true`; final stamp is Gate ROS now |
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
- [ ] Tests-first RED executes valid fixtures and fails only because the
  repository does not yet contain MotionGate product behavior.
- [ ] `MotionGateCore` starts inhibited and is tested with a manual steady
  clock without sleeps.
- [ ] Core tests cover global CAS success/mismatch, Gate restart, generated
  lease/topic uniqueness, PREPARE/OPEN/RENEW/INHIBIT ordering, permanent
  retirement, the OPEN queue barrier, and the publication serial barrier.
- [ ] A stale `INHIBIT` with an old Gate instance, lease, or `control_seq`
  cannot inhibit a newer lease; a matching current `INHIBIT` publishes zero
  before acknowledgement.
- [ ] Authority is live immediately before 250 ms and retired at the 250 ms
  boundary; a late RENEW cannot resurrect it.
- [ ] Candidate freshness is independent: continuing candidates cannot renew
  authority, and continuing renewals cannot make a candidate older than
  150 ms valid.
- [ ] Finite `linear.x`/`angular.z` values are clamped exactly to trusted
  limits. NaN, Inf, or a non-zero unsupported axis retires the lease and
  selects zero.
- [ ] The candidate reader uses a Gate-generated per-lease topic, binds one
  complete 16-byte publisher GID observed by Gate, rejects zero/two/unknown
  writers, recreates a volatile depth-1 reader at OPEN, discards pre-OPEN
  queued samples, and rejects old-topic or unbound-GID samples.
- [ ] The control request carries no writer GID. A locked-RMW self-test proves
  Gate-local graph-GID to `MessageInfo` correlation; mismatch fails closed.
- [ ] The old writer disappears before another lease opens; failure remains
  inhibited and has a bounded typed diagnostic.
- [ ] `INHIBIT` publishes zero through the serial barrier before the control
  response reports inhibited. Inhibited mode continues zero every 20 ms.
- [ ] Private IDL is bounded, remains in `voice_nav_mission`, and is absent
  from `voice_nav_interfaces`.
- [ ] Product bringup loads trusted YAML, includes the lower-level simulator,
  launches `motion_gate_node` inhibited, and contains no test authority
  process or direct final-command bypass.
- [ ] The final publisher uses `rclcpp::SystemDefaultsQoS()` and is proven
  compatible with the actual controller subscriber without treating
  introspection `UNKNOWN` values as fixed QoS facts.
- [ ] In the product composition MotionGate is the sole publisher endpoint on
  `/diff_drive_controller/cmd_vel`; owner FQN is exactly
  `/motion_gate_node`, and its publisher GID remains stable for the observation
  window.
- [ ] A headless valid lease produces bounded forward odometry. Separate
  evidence records Gate zero, controller limited-output zero, and odometry
  stationarity after `INHIBIT`.
- [ ] With valid-looking candidates still arriving, stopping renewals causes
  Gate zero no later than 300 ms steady time after the last accepted renewal.
- [ ] With renewals continuing, stopping candidates causes Gate zero no later
  than 200 ms steady time after the last accepted candidate.
- [ ] Repository contracts, interface generation, build, package tests,
  headless integration, full verification, and guarded process-residue audit
  pass.
- [ ] Lesson 0009, its evidence record, architecture/current-status documents,
  and `CHANGELOG.md` match the implemented slice.
- [ ] A reviewed PR passes required hosted CI and is rebase-merged before
  annotated `course/0009-solution` is created.

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
- ADR required: no. This implements ADR-0002 and ADR-0003; a deviation from
  the separate Gate process or six-package topology requires a new ADR.

Expected implementation paths, recorded here without pre-creating source:

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

- Unit:
  - manual-clock Core table tests for every state, CAS, deadline, clamp,
    retirement, queue barrier, and serial publication outcome;
  - exact 249/250 ms and 149/150 ms boundary cases;
  - valid extrema, over-limit CLAMP, signed zero, NaN/Inf, and each unsupported
    axis;
  - randomized event sequences asserting that non-zero output implies ARMED,
    live authority, fresh bound-writer candidate, and finite clamped axes.
- Contract:
  - bounded private IDL and exact operation names;
  - exact private topic types/QoS plus runtime compatibility for the
    SystemDefaults final endpoint;
  - no type leakage into `voice_nav_interfaces`;
  - no writer-GID field in the control request;
  - trusted Gate/controller limit and timeout agreement;
  - installed launch/config/executable and product final-publisher ownership;
  - no `twist_mux`, direct product bypass, or fifth resident process.
- Integration without Gazebo:
  - private service/state behavior, global CAS, transient state snapshot;
  - zero/one/two writer cases and Gate-local graph/MessageInfo GID
    correlation;
  - pre-OPEN, old-topic, unbound Gate-local GID, and post-INHIBIT samples;
  - zero-before-ack and no queued non-zero after the serial barrier.
- Headless Gazebo:
  - default inhibited zero and unique final owner;
  - bounded forward command and odometry;
  - authority and freshness expiry while the other data/control path remains
    active;
  - clamp observation at the controller input/output;
  - command-zero and physical-stationarity timestamps;
  - bounded cleanup and no process residue.
- Manual:
  - optional RViz/Gazebo observation may supplement evidence but cannot replace
    an automated assertion.

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

Generated logs, bags, build outputs, and screenshots are not committed.
