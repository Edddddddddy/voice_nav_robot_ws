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

```text
Minimal GREEN command:
python3 -m pytest -q src/voice_nav_sim/test/test_crash_evidence.py

Exit status: 0
Executed tests: 2
Result: 2 passed in 0.36s
```

### Architecture correction: use the public hardware extension seam

Before implementing the hardware oracle, a temporary compile probe tested the
planned direct subclass against the installed environment at ancestor HEAD
`f475147`:

```text
Environment:
Ubuntu 24.04 / ROS 2 Jazzy
gz_ros2_control 1.2.19
g++ 13.3.0

Probe:
class Probe final : public gz_ros2_control::GazeboSimSystem {};
int main() { Probe probe; }

Decisive compiler result:
std::default_delete<gz_ros2_control::GazeboSimSystemPrivate>
error: invalid application of sizeof to incomplete type
```

The installed `gz_system.hpp` forward-declares the private PImpl, stores it in
`std::unique_ptr`, and declares no out-of-line destructor. The installed plugin
XML instead names `GazeboSimSystemInterface` as the public base class. The
probe also established that CMake must discover `gz_sim_vendor` and `gz-sim8`
before consuming the exported `gz_ros2_control` targets. The temporary probe
files were removed after observation.

The accepted correction is a test-only Adapter that inherits the public
Interface, pluginlib-loads `gz_ros2_control/GazeboSimSystem`, delegates every
call unchanged, and journals actual `JointVelocityCmd` only after upstream
`write()` returns. That Interface has `time` and `period` but no
`UpdateInfo.iterations`; hardware records therefore use `sim_stamp +
write_seq`, while World Statistics independently owns real iteration and is
correlated through ARM/SEAL. The no-allocation claim applies only to added
journal instrumentation, not to pinned upstream `write()`.

An initial inline cross-shell compile command was rejected by Bash near `(`
because PowerShell consumed nested quoting. No repository or external state
changed; the diagnostic was rerun through the explicit CMake probe. This is a
recurrence of [PIT-0001](../reference/engineering-pitfalls.md#pit-0001-windows-to-wsl-quoting-is-a-two-shell-contract), not compiler evidence.

The first root-contract rerun used a fresh WSL shell without sourcing Jazzy.
All 288 tests were discovered, but the 13 tests that import Gazebo launch
support errored with `ModuleNotFoundError: launch`; this is environment failure,
not tests-first RED. Repeating the same runner after
`source /opt/ros/jazzy/setup.bash` executed all 288 tests with zero failure or
error. [PIT-0011](../reference/engineering-pitfalls.md#pit-0011-ctest-needs-the-ros-environment-not-only-a-build-directory)
now covers both CTest wrappers and direct ROS-aware repository support imports.

The static-RED verification then reproduced
[PIT-0001](../reference/engineering-pitfalls.md#pit-0001-windows-to-wsl-quoting-is-a-two-shell-contract)
three more times before any intended test body ran: a spaced pytest `-k`
expression arrived as only `-k not`, an Awk `$0` program was expanded by the
outer shell, and a `$(git rev-parse HEAD)` guard lost its quote boundary. None
was accepted as RED. The permanent command pattern now reads HEAD in a separate
native Git call, uses pytest's no-space `--deselect=<node-id>`, and performs
line inspection in PowerShell rather than nesting another language inside
`bash -lc`.

- [x] Pure valid fixtures pass.
- [x] Negative/mutation fixtures fail for their expected reasons.
- [x] Repository assertion alone fails because crash-stop artifacts are absent.
- [x] RED commit identity is captured after the run.

```text
RED commit: b04b23f78442f0691a53f82c9e3b8612edf80fa3

Focused command:
python3 -m pytest -q tests/test_crash_stop_contract.py \
  --deselect=tests/test_crash_stop_contract.py::\
CrashStopContractTest::test_repository_crash_stop_contract_passes

Focused result: 17 passed, 1 deselected

Canonical command on the exact RED commit:
source /opt/ros/jazzy/setup.bash
python3 scripts/run_repository_tests.py

Exit status: 1
Executed tests: 306
Expected repository failure: exactly one
test_repository_crash_stop_contract_passes
-> missing src/voice_nav_sim/test_support/
   journaled_gazebo_sim_system_adapter.hpp
```

The static checker is deliberately a topology contract. It rejects the
concrete PImpl subclass, missing upstream override-surface forwarding calls,
fabricated per-write iteration, omitted test generation, product seam leaks,
extra hardware plugins, XML changes outside the one Adapter replacement and
two journal parameters, wrong plugin type, missing CMake export/install/link,
and absent direct package dependencies. It does not claim that source-token
presence proves unchanged runtime forwarding. A separately registered C++
fake-upstream test and actual pluginlib load smoke remain mandatory before the
repository assertion may turn GREEN.

### Architecture correction: record the transition fence, not a later snapshot

A second seam review found that a timestamp written only after the Core had
mutated state could be later than `ProcessExited` even when the mutation was
earlier. That post-hoc commit would manufacture the desired causal order under
preemption. The accepted protocol therefore distinguishes:

- `transition_linearization_ns`, sampled immediately before the one bounded
  Core-owned mutation;
- output `INTENT` time, sampled before the DDS publish call;
- later `COMMITTED` time, which proves completion but not ordering.

All PREPARE/OPEN/RENEW/INHIBIT, automatic retirement, invalid-input retirement,
fault, and sequence-exhaustion paths must pass through one private Core
transition wrapper. `reconcile_adapter_transition()` remains subscription
cleanup, and Node callbacks do not independently journal Core transitions.
This correction is preserved as
[PIT-0039](../reference/engineering-pitfalls.md#pit-0039-post-hoc-commit-time-is-not-transition-linearization-time)
and
[PIT-0040](../reference/engineering-pitfalls.md#pit-0040-scattered-callback-journals-create-partial-evidence).

### Gate event journal output transaction and ABI evidence

The first Gate-journal tracer bullet was preserved as a real RED commit before
implementation:

```text
RED commit:
e78635fa373f8bdfc921cca436012f7fb3f8186a

Command:
source /opt/ros/jazzy/setup.bash
ctest --test-dir build/voice_nav_mission \
  -R gate_event_journal_test --output-on-failure

Observed: 1 test executed, 1 failed
Only failure: C++ exception "VN-0011A tests-first RED"
```

Commit `844a335` implemented the bounded output transaction. The single writer
atomically claims a never-reused slot, writes payload and CRC64, release-stores
`INTENT`, calls the supplied final publisher exactly once, then writes commit
time/CRC64 and release-stores `COMMITTED`. A crash after claim but before
INTENT leaves a claimed/FREE gap; a publisher exception or kill after INTENT
leaves a trailing INTENT. Both invalidate the evidence generation instead of
being repaired or overwritten.

An independent review reported P0=0 and P1=0. It confirmed that the payload
clear begins at offset 8 and never writes `phase` non-atomically, each reader
boundary is paired with release/acquire ordering, and publisher exceptions
cannot reach `commit_output`. It identified the production-checksum helper
self-comparison as a P2 oracle weakness. Commit `774d525` closed that weakness
with a real C11 ABI target, fixed externally calculated header/INTENT/COMMITTED
constants, and include/exclude mutation matrices for all 11/20/27 covered
fields. The checksum contract is recorded as
[PIT-0041](../reference/engineering-pitfalls.md#pit-0041-a-checksum-implementation-cannot-be-its-own-oracle).

```text
Final focused result on 774d525:
gate_event_journal_test .......... passed
gate_event_journal_c_abi_test .... passed

voice_nav_mission package gate:
13 CTest tests passed
123 tests, 0 errors, 0 failures, 12 skipped
```

Three more Windows-to-WSL command-boundary failures occurred while collecting
this evidence: a wildcard embedded in a Windows `rg` path was not expanded, a
quoted CTest regex containing `|` arrived as Bash pipelines, and a quoted
`stat -c` format containing spaces lost its argument boundary. These were
classified as further PIT-0001 occurrences and replaced with `rg -g`, separate
CTest calls, and `stat -c %Y`; none was accepted as code RED. One incremental
build also reported a dependency timestamp 23 ms in the future. Comparing the
WSL/file epochs and rerunning the same target produced a clean zero exit with
no warning, so the transient mounted-filesystem case is retained as
[PIT-0042](../reference/engineering-pitfalls.md#pit-0042-mounted-filesystem-clock-skew-needs-a-bounded-rerun).

This is not VN-0011A completion. The journal is still an uninstalled static
module with no MotionGate product link. Transition records, POSIX shared-memory
ownership, Node integration, the Gazebo hardware Adapter, and real crash
evidence remain open; the repository-level topology test therefore remains
deliberately RED.

## VN-0011A observed crash evidence

| Case | Expected threshold | Observed result |
| --- | --- | --- |
| Authority SIGKILL | <=40 ms steady valid/recent-nonzero-Gate-commit barrier; Gate receipt <=20 ms; advancing non-zero simulation surfaces <=30 ms simulation age; exact `-SIGKILL`; Gate-journal terminal transition-linearization and bound-zero pre-publish fences are not earlier than ProcessExited, while later commits prove completion; state has empty lease, journaled predecessor-plus-one `control_seq`, new/equal zero-output seq, `AUTHORITY_EXPIRED`; <=300 ms steady | Not yet captured |
| Candidate SIGKILL | same bounded arming/Gate-journal rules; exact `-SIGKILL`; authority RENEWs remain live and are journaled; terminal sequence follows the last committed predecessor; `CANDIDATE_EXPIRED`; event-to-state <=200 ms steady | Not yet captured |
| MotionGate SIGKILL | exact `-SIGKILL`; Gazebo/controller live; no injected zero; a marker new to the generation is COMMITTED exactly once, ACKed by non-zero controller output before the next 20 ms repeat, and remains the final Gate output record after death; any repeat retries the generation; first controller zero satisfies `0.35 s < delta_sim <= 0.36 s + epsilon`; graph quiet is cleanup only | Not yet captured |
| Wheel command/state | compatible BEST_EFFORT introspection remains mandatory corroboration; delegated hardware ledger accounts for each invocation through contiguous `write_seq` range/count segments, nondecreasing simulation stamp, and no overflow/gap/nonzero violation; World Statistics separately owns iteration; together they prove first both-wheel zero plus no regression | Not yet captured |
| Physical stationarity | after the later controller/lossless both-wheel-zero linearization, shared wheel-state/odom window begins <=1.2 s and holds >=0.20 s simulation time | Not yet captured |
| Exit ledger | killed actions `-SIGKILL`; all other launch-managed actions 0 | Not yet captured |

## VN-0011A mutation and repository gates

- [ ] Wrong process/signal is rejected.
- [ ] Collapsed helper process is rejected.
- [ ] Test-injected zero after Gate death is rejected.
- [ ] Broad exit allowlist or skipped case is rejected.
- [ ] Wrong timeout/update rate/clock is rejected.
- [ ] Missing wheel command/state/odom surface is rejected.
- [ ] Stale/delayed arming, terminal transition or bound-zero pre-call fence
  before ProcessExited, post-hoc commit time substituted for linearization,
  DDS receipt ordering used as causal proof, omitted intervening RENEW, queued
  late Gate input, reused/repeated/unmatched final marker, absent/trailing
  Gate-output commit,
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
