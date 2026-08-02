# VN-0011: Prove crash-stop and Managed Safe Pause

**Status:** In Progress

**GitHub Issue:**
[#20](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/20)

**Delivery type:** Umbrella tracker; it has no implementation branch or PR.
Its child slices use `Refs #20`, and this Issue remains open until the course
solution artifact is published.

## Immutable start checkpoint

Lesson 0010 starts from the cumulative public baseline after Lesson 0009 and
its C1/C2 corrective delivery:

```text
annotated tag: course/0010-start
tag object: 92a054c3eaae6e4dd0e8500aa712e866e8a71e33
peeled target: f75a9c48f610306a1cf3ec83d0e5e99474220ad6
```

The local and remote objects have been verified equal. No
`course/0010-solution` object exists yet, and this tree must not predict one.

## Goal

Complete Lesson 0010 with two independently reviewable outcomes:

1. prove that loss of the authority producer, candidate producer, or
   MotionGate process selects zero through the correct deadman without a test
   bypass; and
2. define and prove, through a package-private coordinator and test Adapter, a
   tokenized Managed Safe Pause transaction that proves zero before pausing and
   refuses unsafe in-place resume.

The result remains a simulation operational-stop mechanism. It is not a
functionally certified emergency stop.

## Delivery slices

| Slice | Issue / branch | State | Observable outcome |
| --- | --- | --- | --- |
| VN-0011A | [#21](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/21) / `feat/vn-0011a-l0010-crash-stop` | In progress | Three exact-process SIGKILL crash-stop cases |
| VN-0011B | Issue and branch not created | Planned | Package-private Managed Safe Pause protocol and `RESTART_REQUIRED` policy proof |

VN-0011A and VN-0011B use separate tests-first PRs. Completing A does not make
Lesson 0010 complete; the catalog remains `in_progress` until B, course
evidence, and the immutable solution checkpoint are complete.

## Aggregate acceptance matrix

| Owner | Scenario | Required evidence |
| --- | --- | --- |
| A | Authority SIGKILL | one <=40 ms steady arming barrier proves authority/candidate validity and a unique non-zero Gate marker, with the final Gate state <=20 ms old; simulation evidence is independently advancing/non-zero and <=30 ms old in simulation time; a parent-owned Gate event journal includes every intervening accepted control transition and proves the terminal retirement plus bound zero commit occur no earlier than exact authority `ProcessExited`; the received matching state clears the lease, matches the journaled predecessor-plus-one control sequence, reports `AUTHORITY_EXPIRED`, and arrives within 300 ms steady time afterwards |
| A | Candidate SIGKILL | the same split-clock arming and Gate-journal rules apply; continued authority RENEWs are journaled rather than forbidden; the terminal retirement is exactly one non-wrapping sequence step after the final accepted RENEW, occurs no earlier than exact candidate `ProcessExited`, and the matching state reports `CANDIDATE_EXPIRED` within 200 ms steady time afterwards |
| A | MotionGate SIGKILL | Gazebo and controller remain live; no test zero is injected; the Gate event journal's crash-resilient output lane proves the final Gate publish is an unambiguous non-zero COMMITTED marker with no later intent/commit, and matching non-zero controller output ACKs its acceptance; first controller zero occurs only when its stamp age is greater than 0.35 s and by the next 100 Hz update plus explicit step tolerance; publisher disappearance/quiet are cleanup only |
| A | Downstream stop | controller body command, both wheel command interfaces, both wheel states, and odometry are separate evidence; mandatory introspection corroborates the wheel values while a fenced, contiguous, overflow-failing lossless hardware-write ledger proves first both-wheel zero and no command regression; a shared 0.20 s wheel-state/odom stationary window begins within 1.2 s after the later controller/ledger zero linearization |
| A | Process accounting | only predeclared killed actions may exit `-SIGKILL`; every other launch-managed action exits zero; no broad allowlist, name broadcast, `pkill`, or Gazebo exception |
| B | Managed Safe Pause | Gate, controller, wheel command/state, and odometry zero are proven while simulation advances; World Statistics then confirms pause before a token is minted |
| B | Pause-time Gate loss | the original Gate may remain inhibited or be proven absent with no final-command publisher; any replacement Gate/publisher invalidates the token; exact `{pause: true, multi_step: 1}` requests step/re-pause until a bounded next controller update is observed zero, then one additional step losslessly writes that post-update zero before `pause:false` is allowed |
| B | Failed zero proof | no token is minted; the coordinator returns `RESTART_REQUIRED` and selects structured generation shutdown |
| B | Missing/stale/replayed token | managed resume refuses the request and never sends `pause:false` |

VN-0011B mints a token only for the same ACTIVE controller generation. Any
deactivation, inactivity, or replacement selects `RESTART_REQUIRED`; this
slice does not add an activation/reconstruction resume branch.

## Shared invariants

- Public ROS IDL, packages, MotionGate Core semantics, trusted speed limits,
  controller update rate, and `cmd_vel_timeout=0.35` do not change.
- The authority and candidate used in A are test stand-ins. A does not claim
  that the not-yet-implemented `mission_runtime_node` was killed.
- Lease/freshness and process-death latency use steady time. Controller, wheel,
  odometry, and pause evidence use strictly increasing simulation stamps. Wall
  time is only the outer bounded-test watchdog.
- The lossless write journal is mode-aware: VN-0011A observes naturally
  advancing continuous-run iterations; only VN-0011B's paused probe restricts
  iteration changes to acknowledged and World-Statistics-confirmed exact
  single-step transactions.
- Managed resume is a project policy boundary, not protection against a local
  user invoking Gazebo Transport directly.
- VN-0011B delivers a package-private coordinator and test Adapter protocol
  proof, not a user-facing product pause function. The test Adapter is the
  caller that receives `RESTART_REQUIRED`; a future lifecycle supervisor owns
  product integration.
- VN-0011B may return `RESTART_REQUIRED` and perform structured shutdown;
  automatic relaunch requires a future supervisor and is not implied here.

## Non-goals

- MissionRuntime, public `StopMission`, Nav2, SLAM, smoother, Collision Monitor,
  Agent, voice, a public pause service, or a user-facing product pause command.
- A new ROS package, fifth resident product process, CLI, persistent token
  store, or restart supervisor.
- Preventing a trusted local operator from using Gazebo Transport directly.
- Calling a pause acknowledgement alone a committed pause or treating released
  command interfaces alone as zero proof.

## Risks and rollback

- A weak fault harness could pass after killing the wrong process or injecting
  its own zero. Exact Launch ProcessActions, an exhaustive crash ledger, and
  mutation contracts make those paths fail closed.
- Mixed steady/simulation/wall clocks can produce plausible but false latency.
  Each measurement names its clock and linearization event.
- Pause can freeze an old non-zero controller command because controller time
  stops. The accepted design in
  [ADR-0005](../adr/0005-use-tokenized-managed-safe-pause.md) proves zero before
  pause and requires a token for managed resume.
- Either child slice can be reverted independently. Existing Lesson 0009
  normal-running tests and product behavior remain the rollback baseline.

## Closure conditions

- [ ] VN-0011A repository acceptance and Issue #21 delivery closure complete.
- [ ] VN-0011B repository acceptance and delivery closure complete.
- [ ] Lesson and record contain only evidence actually observed on their
  reviewed trees.
- [ ] Exact-final-head local gate, independent review, and required CI are
  recorded externally for each child.
- [ ] Both child PRs are rebase-merged and their public trees verified.
- [ ] Annotated `course/0010-solution` is published from reviewed public
  `main`, with its object and peeled target recorded in Issue #20.
- [ ] Issue #20 receives the final closure comment and is then closed.

Final local/pushed HEADs, their gate results, public commits/trees, CI runs,
and the future solution tag belong in PR/Issue evidence under
[the delivery identity policy](../process/change-lifecycle.md#不可自引用的交付身份),
not in a recursive ledger commit.
