# VN-0010-C1: Converge Gate-local writer identity safely

**Status:** In Progress

**GitHub Issue:**
[#14](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/14)

**Branch:** `fix/vn-0010-writer-identity-convergence`

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
- endpoint kind, topic type, compatible candidate QoS, and the already-known
  namespace agree with policy;
- the endpoint GID is non-zero and is pinned to this PREPARE generation;
- only the ROS node name remains unresolved;
- Core remains `PREPARED`, does not expose a bound GID, selects zero, and the
  Node publishes zero before responding.

Wrong non-empty type/FQN, contradictory partial namespace, wrong QoS, wrong
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
- [x] Only typed metadata pending and the legacy exact no-writer observation
  are retried with fresh request IDs and the original deadline.
- [x] Unique endpoint diagnostics are bounded to 160 characters and record
  endpoint count, kind, type, node identity, QoS, GID, and steady elapsed time.
- [x] Focused Core, observation, convergence, and Node tests pass locally.
- [ ] Product handover test passes at least 20 serial fresh-launch
  repetitions.
- [ ] Canonical `scripts/verify.sh` passes on the exact final head.
- [ ] Independent safety/code review has no unresolved P0-P2 finding.
- [ ] A dedicated hotfix PR passes required hosted CI and merges before PR #13
  is rebased.

## Risks and rollback

- A too-broad pending classifier could admit the wrong writer. The classifier
  therefore permits only missing node name after all other fields and a
  non-zero GID pass, and latches post-pin changes terminally.
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

## Verification evidence

Evidence recorded before final static/product/full-gate closure:

```text
colcon build --packages-select voice_nav_mission voice_nav_bringup
Result: 2 packages finished

ctest --test-dir build/voice_nav_mission --output-on-failure \
  -R 'motion_gate_core_test|writer_observation_test|test_test_motion_gate_node.py'
Result: 3/3 CTest targets passed
Inner coverage: 42 Core cases; 6 observation cases; Node launch matrix passed

python3 -m unittest \
  src/voice_nav_bringup/test/test_motion_gate_open_convergence.py
Result: 8 tests passed

ros2 interface show voice_nav_mission/srv/InternalMotionGateControl \
  | grep WRITER_METADATA_PENDING
Result: uint16 WRITER_METADATA_PENDING=19
```

Final exact-head hashes, static-contract counts, 20 fresh product runs, full
verification, hosted CI, review, PR, and merge evidence remain pending and
must not be claimed early.
