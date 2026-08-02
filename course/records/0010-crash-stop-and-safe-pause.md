# Lesson 0010 证据记录：进程崩溃停止与托管安全暂停

**教师参考实现：** In progress

**学习者复现：** Pending

**Umbrella:** [VN-0011](../../docs/work-items/0011-crash-stop-and-safe-pause.md) /
[#20](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/20)

**Current slice:**
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md) /
[#21](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/21) /
`feat/vn-0011a-l0010-crash-stop`

## Immutable start identity

```text
course/0010-start
tag object: 92a054c3eaae6e4dd0e8500aa712e866e8a71e33
peeled target: f75a9c48f610306a1cf3ec83d0e5e99474220ad6
local/remote identity: verified equal
```

Current delivery identities:

```text
VN-0011A PR: Not yet created
VN-0011A required CI: Not yet captured
VN-0011A merge/public tree: Not yet captured
VN-0011B Issue/branch/PR: Not yet created
course/0010-solution: Does not exist yet
```

Future exact-final-head, gate, PR, CI, merge/public-tree, and solution-tag
identities are recorded externally after they exist. This target tree does not
require its own future identity.

## VN-0011A tests-first RED

### Cycle A1: exact-action CrashLedger

The first executable tracer bullet isolates launch-process exit accounting.
It deliberately supplies a loadable support module whose constructor raises a
named `NotImplementedError`, so collection/import succeeds and exactly one
test body reaches the missing behavior.

```text
Command:
python3 -m pytest -q src/voice_nav_sim/test/test_crash_evidence.py

Exit status: 1
Executed tests: 1
Expected failure:
CrashLedgerTest.test_exact_action_exit_accounting_is_closed_and_exhaustive
-> NotImplementedError: VN-0011A tests-first RED
```

This is a valid RED rather than a syntax, import, discovery, skip, or wrong-
environment failure. It requires exact action identity even for distinct
objects that compare equal, a closed `-SIGKILL`/zero exit set, rejection of
unknown/duplicate/wrong exits, and exhaustive completion.

Minimal GREEN:

```text
Command:
python3 -m pytest -q src/voice_nav_sim/test/test_crash_evidence.py

Exit status: 0
Executed tests: 1
Result: 1 passed in 0.20s
```

The first implementation attempt exposed a dynamic-loader/dataclass collection
failure; it was rejected as GREEN. [PIT-0034](../reference/engineering-pitfalls.md#pit-0034-dynamic-exec_module-may-not-satisfy-decorator-assumptions)
records the cause and the executable guard.

Package registration was then verified through the actual ament/CTest path:

```text
Command:
colcon build --packages-select voice_nav_sim --symlink-install
colcon test --packages-select voice_nav_sim
colcon test-result --test-result-base build/voice_nav_sim/test_results --verbose

Build: 1 package finished
CTest: 11/11 targets passed
Result: 33 tests, 0 errors, 0 failures, 3 existing approved skips
crash_evidence_test: 1 passed
```

The run used the repository's isolated ROS-domain and unique-partition launch
contracts; it did not target the user's separately running Gazebo process.

### Cycle A2: signal intent versus death observation

```text
Command:
python3 -m pytest -q src/voice_nav_sim/test/test_crash_evidence.py

Exit status: 1
Executed tests: 2
Result: 1 passed, 1 failed
Expected failure:
CrashLedgerTest.test_signal_intent_is_not_exit_and_event_time_is_retained
-> AttributeError: CrashLedger has no arm_sigkill
```

The test body ran and isolates the next missing behavior: signal intent must be
armed against one exact action but cannot satisfy completion; only a later
same-host monotonic process-exit observation may do so. It also rejects an
unarmed kill, inverted event time, arming a clean action, and an empty ledger.

- [ ] Pure valid fixtures pass.
- [ ] Negative/mutation fixtures fail for their expected reasons.
- [ ] Repository assertion alone fails because crash-stop artifacts are absent.
- [ ] RED commit identity is captured after the run.

```text
Command: Not yet captured
Exit status: Not yet captured
Executed tests: Not yet captured
Expected repository failure: Not yet captured
```

## VN-0011A observed crash evidence

| Case | Expected threshold | Observed result |
| --- | --- | --- |
| Authority SIGKILL | <=40 ms steady valid/Gate-marker barrier; Gate receipt <=20 ms; advancing non-zero simulation surfaces <=30 ms simulation age; exact `-SIGKILL`; Gate-journal terminal/zero commits are not earlier than ProcessExited; state has empty lease, journaled predecessor-plus-one `control_seq`, new/equal zero-output seq, `AUTHORITY_EXPIRED`; <=300 ms steady | Not yet captured |
| Candidate SIGKILL | same bounded arming/Gate-journal rules; exact `-SIGKILL`; authority RENEWs remain live and are journaled; terminal sequence follows the last committed predecessor; `CANDIDATE_EXPIRED`; event-to-state <=200 ms steady | Not yet captured |
| MotionGate SIGKILL | exact `-SIGKILL`; Gazebo/controller live; no injected zero; Gate event journal output lane ends in an unambiguous non-zero COMMITTED record with no later intent/publish and its marker is ACKed by non-zero controller output; first controller zero satisfies `0.35 s < delta_sim <= 0.36 s + epsilon`; graph quiet is cleanup only | Not yet captured |
| Wheel command/state | compatible BEST_EFFORT introspection remains mandatory corroboration; fenced lossless ledger accounts for each invocation through contiguous sequence-range/count segments, nondecreasing iteration, and no overflow/gap/nonzero violation, proving first both-wheel zero plus no regression | Not yet captured |
| Physical stationarity | after the later controller/lossless both-wheel-zero linearization, shared wheel-state/odom window begins <=1.2 s and holds >=0.20 s simulation time | Not yet captured |
| Exit ledger | killed actions `-SIGKILL`; all other launch-managed actions 0 | Not yet captured |

## VN-0011A mutation and repository gates

- [ ] Wrong process/signal is rejected.
- [ ] Collapsed helper process is rejected.
- [ ] Test-injected zero after Gate death is rejected.
- [ ] Broad exit allowlist or skipped case is rejected.
- [ ] Wrong timeout/update rate/clock is rejected.
- [ ] Missing wheel command/state/odom surface is rejected.
- [ ] Stale/delayed arming, terminal/zero journal commit before ProcessExited,
  DDS receipt ordering used as causal proof, omitted intervening RENEW, queued
  late Gate input, duplicate/unmatched markers, absent/trailing Gate-output commit,
  side-observer origin, lossy no-regression evidence, and incomplete write
  ledger are rejected.
- [ ] Focused product repetitions pass.
- [ ] Canonical local gate passes on the exact final local/pushed HEAD.
- [ ] Independent P0-P2 review and required CI pass.

```text
Observed results: Not yet captured
```

## VN-0011B Managed Safe Pause evidence

- [ ] Pure coordinator/state-machine tests pass.
- [ ] Zero proof completes before pause request and token creation.
- [ ] World Statistics confirms paused state and stopped iteration/time.
- [ ] Token binds partition/world/process/controller/Gate/publisher plus
  controller update/fixed step/period, lossless-oracle generation/sealed-fence,
  and zero-proof facts; it is opaque, single-use, and replay-resistant.
- [ ] Original Gate may remain inhibited or be proven absent with zero final
  publishers; any replacement identity invalidates the token.
- [ ] Without continuous run, exact `{pause:true,multi_step:1}` requests each
  advance once and re-pause; bounded steps encounter a new zero controller
  update, then one additional step losslessly writes its post-update zeros;
  omitted/false pause, duplicate request, non-zero/missing update, gaps, or
  wrong iteration are rejected before `pause:false`.
- [ ] Failed zero proof mints no token and returns `RESTART_REQUIRED`.
- [ ] Only the same ACTIVE controller generation may mint/resume a token;
  inactive, deactivated, or replaced controller returns `RESTART_REQUIRED`.
- [ ] Unmanaged/stale/mismatched token never calls `pause:false`.
- [ ] Structured old-generation shutdown is proven.

```text
Observed results: Not yet captured
```

## Course closure

- [ ] VN-0011A delivery complete.
- [ ] VN-0011B delivery complete.
- [ ] Lesson and record match implemented behavior.
- [ ] Catalog moves from `in_progress` to `completed` only after both slices.
- [ ] `course/0010-solution` is created from reviewed public `main` and its
  exact object/target are recorded in Issue #20.
- [ ] Issue #20 receives its closure comment and is closed.

## 学习者复盘

```text
Learner evidence: Pending
Learner reflection: Pending
```
