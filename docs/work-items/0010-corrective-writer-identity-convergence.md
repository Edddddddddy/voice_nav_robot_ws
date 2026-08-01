# VN-0010-C1: Converge Gate-local writer identity safely

**Status:** In Progress — the original technical delivery merged through
combined PR #16; the bounded-diagnostic correction, exact-head closure CI,
and Issue #14 closure remain pending.

**GitHub Issue:**
[#14](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/14)

**Original implementation branch:** `fix/vn-0010-writer-identity-convergence`

**Closure branch:** `fix/vn-0010-c1-diagnostic-bounds`

## Problem statement

Documentation-only PR #13 exposed a low-probability MotionGate OPEN failure in
required GitHub Actions run `30684948558`. DDS matching had completed, but the
Gate-local ROS graph snapshot returned `WRITER_MISMATCH` while the fixed test
publisher was being handed over for a third lease. The same product code and
resolved dependencies passed run `30651076176`, the exact local gate, and ten
extra serial product repetitions.

The evidence proves that transport matching and Gate-local endpoint-identity
observation are not one atomic transaction. It does **not** prove which graph
field was incomplete, nor does it prove an `rmw_fastrtps_cpp` regression. This
Work Item is a corrective child of VN-0010 / Lesson 0009 and is deliberately
separate from documentation-only PR #13.

## Delivery deviation

The originally planned dedicated C1 hotfix PR did not occur. The complete
original 28-commit C1 writer-convergence stack was delivered as the lower
stack of combined PR #16, followed by C2 Gazebo-teardown work. PR #16 provides
combined integration and hosted-CI evidence; this record does not
retroactively relabel it as a dedicated C1-only PR.

A later C1-specific audit found that whole-message truncation could remove
mandatory diagnostic fields. That correction, its exact-head verification,
and Issue #14 governance closure are tracked by the subsequent C1 closure PR.

## Goal

Represent only a narrowly unresolved ROS node identity as a typed, bounded,
fail-closed convergence state. Keep the Gate `PREPARED`, selected/published
output zero, and provisionally pin the first eligible non-zero endpoint GID.
The same GID may become READY after its identity resolves; replacement or any
definitive policy violation must remain terminal.

## Non-goals

- Retrying arbitrary `WRITER_MISMATCH` results.
- Treating DDS matching as proof that graph metadata is complete.
- Relaxing the one-second absolute OPEN deadline, adding sleeps, or weakening
  MotionGate assertions.
- Supporting another RMW implementation or claiming a Fast DDS defect.
- Changing public `voice_nav_interfaces`, motion ownership, TF ownership, or
  Lesson 0010 crash-stop/pause scope.

## Safety and Interface contract

`WRITER_METADATA_PENDING=19` is added only to the package-private
`InternalMotionGateControl` and `InternalMotionGateState` protocols. A pending
response is legal only when all of these facts hold:

- one publisher endpoint is visible;
- endpoint kind, topic type, and compatible candidate QoS agree with policy;
- the endpoint GID is non-zero and is pinned to this PREPARE generation;
- only node-identity components remain unresolved: an empty node name or the
  exact Jazzy `_NODE_NAME_UNKNOWN_` / `_NODE_NAMESPACE_UNKNOWN_` sentinels;
- every node-name or namespace component that is already known agrees with
  policy;
- Core remains `PREPARED`, does not expose a bound GID, selects zero, and the
  Node publishes zero before responding.

Wrong non-empty type/FQN, any contradictory known identity component, wrong QoS, wrong
endpoint kind, zero GID, duplicate endpoints, disappearance or replacement
after pinning, and barrier-time writer changes are definitive mismatch. A
pinned-generation mismatch latches until the next successful PREPARE.

The package-private `WriterObservationSession` is compiled directly into
`motion_gate_node` and its GTest. It is neither installed nor exported and
does not contaminate the pure `motion_gate_core` static library with RMW or
rclcpp dependencies. The Node checks final-controller health before it starts
candidate GID pinning, then preserves the three existing Gate-local graph
snapshots and discard-reader barrier.

The OPEN convergence caller retries only:

1. the legacy exact `WRITER_UNAVAILABLE` / `candidate topic has no writer`
   observation; or
2. the new typed `WRITER_METADATA_PENDING` reason.

Every retry uses a fresh request ID, the original absolute steady-clock
deadline, and an unchanged `PREPARED`/zero response snapshot. Diagnostic text
is never a control discriminator for reason 19.

## Acceptance criteria

- [x] Behavior-level RED covers unresolved identity and provisional GID
  stability.
- [x] Package-private reason 19 exists in Core, service, and state types.
- [x] Observation classification separates READY, typed identity pending, and
  definitive mismatch.
- [x] Wrong kind/type/FQN/partial namespace/QoS, zero GID, duplicate writers,
  disappearance, replacement, and GID change remain fail-closed.
- [x] A contradictory provider result (`ready=true`, non-`NONE` reason) faults
  Core closed before GID binding or ARMED side effects.
- [x] Only typed metadata pending and the legacy exact no-writer observation
  are retried with fresh request IDs and the original deadline.
- [x] The absolute steady deadline is checked both before and immediately after
  every RPC; a late APPLIED response is a timeout, not success.
- [x] Unique endpoint diagnostics are bounded to 160 characters and record
  endpoint count, kind, type, node identity, QoS, GID, and steady elapsed time.
- [x] Focused Core, observation, convergence, and Node tests pass locally.
- [x] Product handover test passes at least 20 serial fresh-launch
  repetitions; this is retrospective evidence collected at unchanged C1
  runtime tree `04db928c` during C2 verification, not an exact-head C1-only
  gate.
- [ ] Canonical `scripts/verify.sh` passes on the exact final head.
- [ ] Independent safety/code review has no unresolved P0-P2 finding.
- [x] Combined PR #16 contains the complete original C1 stack, passes required
  hosted CI on closure head `66f6834`, and rebase-merges without C1 tree drift.
- [x] The deviation from the planned dedicated C1 PR is explicitly recorded;
  PR #16 is not represented as a C1-only PR.
- [x] No rebased replacement head for PR #13 was published before C1 technical
  delivery; PR #13 remains a separate public-ledger change.
- [ ] The C1 bounded-diagnostic correction passes exact-head local and hosted
  gates, merges, and Issue #14 closes as completed.

## Risks and rollback

- A too-broad pending classifier could admit the wrong writer. The classifier
  therefore permits only exact unresolved identity representations after all
  other fields, all known identity components, and a non-zero GID pass; it
  latches post-pin changes terminally.
- A session accidentally reused across leases could carry authority forward.
  Only an applied PREPARE resets the session and observation clock.
- A caller could misread diagnostic text. Control flow uses the typed reason;
  mutation and convergence tests keep mismatch terminal.
- Reverting this hotfix restores the previous fail-closed behavior but also
  restores the non-atomic graph-observation flake. No persistent data or
  public Interface migration is involved.

## Design impact

- Stable Interfaces changed: none.
- Package-private Interface changed: reason 19 added.
- TF or motion ownership changed: none; MotionGate remains the sole final
  velocity publisher.
- ADR required: no. This is a bounded corrective refinement of VN-0010's
  existing Gate-local GID decision, recorded here and in Lesson 0009.

## Tests-first commit trail

- `1f18030` / `f4e579e`: specify and implement provisional GID convergence.
- `ad29c2e` / `91d4d3c`: preserve typed pending through Core without binding.
- `b3d5b87` / `a339d40`: latch post-pin replacement for one generation.
- `c5e2aac` / `647d7e7`: add typed bounded OPEN convergence.
- `db7e2e9`: connect the tested observer to the Node and private IDL.
- `2696f2d` / `44b2d27`: reject contradictory partial namespaces.
- `23302d0`: cover definitive kind/type/FQN/QoS/GID/count failures.
- `f93b001` / `5225c39`: require and implement bounded field diagnostics.
- `f031b4d` / `eb78ba8`: classify exact Jazzy unknown identity sentinels while
  rejecting contradictory known components.
- `b184c2a`: govern the private observation seam with static mutation tests.
- `064efd7` / `7845667`: require Core to fault a contradictory READY binding.
- `9f62f55` / `beb865e`: enforce the absolute OPEN deadline after every RPC.
- `cf077a8`: lock both safety invariants with static mutation tests.

## Verification evidence

### Original C1 delivery identity

```text
Common baseline:
c2d7631b57001a6e3e360f9bbdb558de6b7a85ed

Original reviewed C1 head:
ccf9fbedc399f7b775a12c273663a8c55aa35cfc
tree=e4d2690d811f91e79c08208376aec10d90d97aef
commit count from baseline=28

Corresponding public C1 head inside rebase-merged PR #16:
cbb2d7b6e1df8c940b3dc193349bcd2b71aaed14
tree=e4d2690d811f91e79c08208376aec10d90d97aef
commit count from baseline=28

git range-diff c2d7631..ccf9fbe c2d7631..cbb2d7b
Result: 28/28 rows `=`; 0 changed, dropped, or added
```

Public head `f31e3b7` is the combined C1+C2 PR head, not the C1 public head.

### Retrospective product and combined-CI evidence

```text
Product execution head: 53f9568dabf44555782680c4a49c392409320ef8
Public equivalent: 7e737f5564bde6546b702d1b570190997b418e1a
Tree: 04db928cc031f62708ca6cc00ce6e8a46d477674

ctest --test-dir build/voice_nav_bringup --output-on-failure \
  --repeat until-fail:20 -R '^test_test_motion_gate_product.py$'
Result: 20/20 PASS; 19.58-28.41 s per launch; 507.21 s total

Required combined PR #16 CI:
run 30710167163, attempt 1, head 1ddd970, required job 6m20s
run 30710678583, attempt 1, closure head 66f6834, required job 5m49s
rebase-merged public head: f31e3b7
```

The 20-run result is retrospective stability evidence for the unchanged C1
MotionGate runtime. The PR #16 runs are combined C1+C2 integration evidence.
Neither substitutes for the new correction's exact-final-head local and
hosted gates.

### Pre-closure focused evidence

Evidence recorded before final static/product/full-gate closure:

```text
colcon build --packages-select voice_nav_mission voice_nav_bringup
Result: 2 packages finished

ctest --test-dir build/voice_nav_mission --output-on-failure \
  -R 'motion_gate_core_test|writer_observation_test|test_test_motion_gate_node.py'
Result: 3/3 CTest targets passed
Inner coverage at that pre-final head: 42 Core cases; 6 observation cases;
Node launch matrix passed

python3 -m unittest \
  src/voice_nav_bringup/test/test_motion_gate_open_convergence.py
Result: 8 tests passed

ros2 interface show voice_nav_mission/srv/InternalMotionGateControl \
  | grep WRITER_METADATA_PENDING
Result: uint16 WRITER_METADATA_PENDING=19
```

The bounded-diagnostic correction now has tests-first RED evidence, focused
10/10 WriterObservationSession GREEN evidence, and 41/41 static/mutation
contract evidence. Its final commit hashes, exact full verification, hosted
CI, independent rereview, PR identity, merge mapping, and Issue #14 closure
remain pending and must not be claimed early.
