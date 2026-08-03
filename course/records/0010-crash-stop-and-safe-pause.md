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

At checkpoint `774d525`, this was not VN-0011A completion: the journal was
still an uninstalled static module, and POSIX ownership, Node composition, the
Gazebo hardware Adapter, and real crash evidence were all open. Later sections
record the completed ownership layers without changing the still-RED product
topology boundary.

### Core-owned transition integration

The Core integration was implemented as reviewable TDD microcycles:

```text
d689df5 RED  -> 22b4472 GREEN  PREPARE
dee5f9e RED  -> b486beb GREEN  OPEN
586d412 RED  -> 403b408 GREEN  RENEW
75e8e65 RED  -> 3618fdb GREEN  INHIBIT / shared retirement seam
417d8dc RED  -> bfe5702 GREEN  FAULT / sequence exhaustion
```

Each successful transition records an exact before-image, an INTENT and
linearization sample immediately before the Core-owned mutation, an exact
after-image, and a COMMITTED checksum. Stable event codes are `1..6` for
PREPARE, OPEN, RENEW, explicit INHIBIT, automatic retirement, and FAULT.
The first integration made `MotionGateCore` non-copyable and non-movable and
initially treated that object trait as the single-writer proof. A later P1
counterexample invalidated that assumption: two distinct Core objects could
still alias one raw journal pointer. The final proof is the one-shot capability
recorded below. Fence callbacks remain compile-time `noexcept`; strings are
prepared outside the fence and swapped inside it.

Independent review of the first PREPARE slice reported P0=0 and P1=0. Its two
P2 findings became executable corrections: commits `e471dd9`/`0887c35` lock
single ownership, while `345304b` locks the no-throw fence and proves a
reservation failure leaves PREPARE inhibited and unchanged.

A later safety review exposed a more important asymmetric failure policy. A
full journal should reject an unrecordable PREPARE, but it must never prevent
INHIBIT or FAULT. Commit `4ebac11` preserved the failing counterexample;
`7244541` introduced explicit `RejectMutation` and `ApplySafetyMutation`
policies. Terminal mutation still selects zero while journal overflow
invalidates the evidence generation. The reusable lesson is
[PIT-0043](../reference/engineering-pitfalls.md#pit-0043-evidence-failure-must-not-veto-a-safety-mutation).

One grouped CTest filter containing `()` and `|` again failed at the
PowerShell-to-Bash boundary before entering the test gate. Separate literal
CTest invocations succeeded, and the recurrence was appended to PIT-0001. It
was not counted as a product RED.

### Ownership, attachment, and terminal-cause corrections

The complete-Core review then found two real P1 defects that object-level
non-copyability had hidden. One journal could still be passed by raw pointer to
two independently constructed Cores, and an active lease at
`control_seq == UINT64_MAX` committed a FAULT but returned `Applied/None` on
the first INHIBIT. Both counterexamples were preserved before repair:

```text
2e9b87d RED  -> 236812c GREEN  INHIBIT returns the committed sequence fault
32e1049 RED  -> 41b44ba GREEN  one-shot transition capability and lifetime detach
```

The first review of `41b44ba` confirmed the sequence result and permanent
second-claim rejection, then found two remaining capability bypasses: its
transition method was public, and the journal-bound constructor accepted an
empty capability. Commit `d4c2f95` locked both failures; `aa6612d` made the
transition entry private to `MotionGateCore`, added a narrow low-level test
peer, separated explicit no-journal construction, and rejected null in the
journal-bound constructor. This recurrence is captured as
[PIT-0044](../reference/engineering-pitfalls.md#pit-0044-object-non-copyability-is-not-resource-exclusivity).

The review also showed that the Core discarded the terminal transition's
`journal_seq`. Commit `856cf0c` required terminal-cause propagation and
`4e6d0c1` exposed it in the snapshot, with zero for unjournaled fallback and
non-terminal states. Commit `7a8216f` proves the first zero-output intent can
bind that exact terminal sequence. Commit `7403899` locks both sequential
lifetime orders and explicitly classifies constructor-time configuration fault
as an unjournaled initial state. Concurrent Journal/Binding destruction is not
claimed; the Node composition is thread-confined and must destroy Core before
the Attached mapping.

The POSIX attacher followed its own TDD pair:

```text
6a5d289 RED  -> cb94da4 GREEN  parent-supplied identity/capacity attachment
```

The first implementation had read `header.generation` before the validator's
acquire of `READY` and had derived expectations from the object being checked.
The corrected API receives complete UID/generation/nonce/capacity from the
parent, validates exact fd metadata and size, then delegates the first mapped
payload read to `GateEventJournal`'s READY-acquire boundary. Wrong identity,
capacity, size, mode, and name fail before `writer_pid` claim. This is recorded
as [PIT-0045](../reference/engineering-pitfalls.md#pit-0045-acquire-must-precede-every-ordinary-shared-memory-read).

Independent Attached review reported P0=0, P1=0 and one P2 acceptance gap:
the seven same-process tests could not prove real publication and mapping
lifetime. That gap became a separate TDD slice:

```text
2e052e1 RED  -> 8dd79a3 GREEN  real parent/child journal attachment
```

The RED used the exact registered CTest in
`/tmp/vn-cross-red-20260802-001`; it executed and failed because the required
probe binary did not exist. The GREEN uses a direct-libc Python owner and an
executed C++ child. It proves the real child PID claim, release/acquire `READY`,
parent-only unlink followed by `ENOENT`, child COMMITTED publication through
the surviving mapping, post-exit parent validation with an independent
CRC64-ECMA oracle, and no residual `/dev/shm/voice_nav_gate_*` object. An
independent review reported P0=0, P1=0, and P2=0.

The transition-matrix review was then closed without changing production
semantics:

- `63a5ee2` covered event 5 for `PrepareExpired`, `AuthorityExpired`,
  `WriterMismatch`, and `InvalidCandidate`, plus full-journal and provider-fault
  branches.
- `825c662` made exact capacity/overflow, safety-state sequence advancement,
  preserved non-zero RENEW command, COMMITTED event 6 checksums/cause, and
  complete repeated-fault Snapshot invariants explicit.
- Review correction `0034c9d` proves the candidate is still selected at
  249 ms before authority expires at 250 ms, and proves a repeated
  `force_fault()` does not even latch a new overflow.

```text
Checkpoint 41b44ba package gate:
15 CTest tests passed, including launch_test and every lint target

Independent Attached temporary-build gate on cb94da4:
7 tests, 0 errors, 0 failures, 0 skipped

Current component gate after 0034c9d:
motion_gate_journal_test: 21 tests passed
cross-process exact CTest: 5 consecutive executions passed
voice_nav_mission package: 16/16 CTest tests passed
```

One full CTest run sourced `/opt/ros/jazzy/setup.bash` but omitted
`install/setup.bash`; only the launch test failed to import
`voice_nav_mission`. The exact rerun with both overlays passed all 15 tests, so
this was another [PIT-0011](../reference/engineering-pitfalls.md#pit-0011-ctest-needs-the-ros-environment-not-only-a-build-directory)
occurrence, not a code RED. Review tooling also reproduced PIT-0001 by turning
a quoted `|` filter into Bash pipelines. During the Attached loop, guessed
ament test names produced `No tests found`; `ctest -N` followed by literal
registered names corrected the evidence, reinforcing PIT-0016's rule that a
zero exit is not proof the intended test ran.

The Layer-2 loop also reproduced two coupled wrapper defects. PowerShell
consumed a Bash `tmpdir=$(mktemp ...)` expression, leaving an empty target that
degenerated to `/build` and `/log`; a later successful diagnostic then hid the
earlier failure behind exit zero. No product test ran and the repository did
not change, so these are PIT-0001 and PIT-0018 recurrences, not the TDD RED.
The valid RED used the explicit temporary directory above and made CTest the
terminal command. A later grouped regex again failed at `(` before build, and
enabling `set -u` before sourcing ROS failed on
`AMENT_TRACE_SETUP_FILES`; separate literal tests and source-before-strict-mode
restored the intended gate. Finally, the first incremental probe build emitted
sub-second clock-skew warnings; the focused rebuild was warning-free and all
five cross-process repetitions passed, closing the bounded PIT-0042 rerun.

### Runtime-owned Node final-output composition checkpoint

The final-output transaction was next composed into the real ROS Node through
two independent RED controls:

```text
fb42e9d  launch RED: no committed GateOutput event_code=1 appeared
011beb6  static RED: Node bypassed runtime_.publish_final_command
c58b6fb  GREEN: thin ROS time/DDS Adapter delegates to the Runtime
```

`MotionGateProcessRuntime` now selects the current Core command, owns the
serialized `INTENT -> DDS -> COMMITTED` transaction, the successful-output and
zero-output counters, terminal-cause consumption, sequence exhaustion, and the
single direct-zero fallback. `motion_gate_node` supplies only ROS time and the
DDS Publisher Adapter. Candidate and timer callbacks issue exactly one Runtime
transaction; state publication failure remains a distinct new fault and may
request one later Runtime transaction. State messages and control responses
map the Runtime-owned counters instead of maintaining parallel Node state.

The launch acceptance uses a parent-owned POSIX journal and scans every claimed
slot rather than assuming output is slot zero. It proves the launched Node PID,
generation, `COMMITTED` output kind/event, zero ROS stamp without `/clock`, zero
linear/angular bit patterns, and independent INTENT/COMMIT CRC64 values. Both
partial parameter configurations still exit 1 without claiming a slot, while
the valid process exits 0 after exact-action SIGINT and leaves the parent
mapping readable.

The first compile reached the intended Adapter and rejected
`rclcpp::Time::to_msg()` on Jazzy. The corrected explicit conversion to
`builtin_interfaces::msg::Time` is recorded as
[PIT-0053](../reference/engineering-pitfalls.md#pit-0053-ros-distribution-apis-must-be-proved-by-the-target-compiler).
One shared synthetic fixture also left an old mutation anchor behind; the full
contract-file rerun caught what the nine new focused cases missed, adding a
second recurrence to PIT-0050.

```text
Exact c58b6fb implementation gate:
motion-gate repository contract: 56/56 passed
focused Runtime/cross-process/two Node launch CTests: 4/4 passed
voice_nav_mission package build: passed
voice_nav_mission package CTest: 18/18 passed
```

This closes only the Node/Runtime output composition micro-slice. It does not
make the root crash-stop contract green: the Gazebo hardware Adapter, crash
policy/composition, three real SIGKILL cases, wheel ledger, physical-stop
evidence, repetitions, PR, and CI remain open.

### Gazebo hardware Adapter plugin-load checkpoint

The first real Adapter tracer used the installed Jazzy plugin registry rather
than a static token fixture:

```text
c9223d3  RED: exported Adapter lookup throws LibraryLoadException
f93b233  GREEN: outer Adapter and pinned standard upstream both construct
```

The RED target compiled and executed. Its only test failed because pluginlib
listed `gz_ros2_control/GazeboSimSystem` and the legacy alias, but not
`voice_nav_sim/JournaledGazeboSimSystemAdapter`. The GREEN adds a test-only
shared library and plugin XML, derives from the public
`GazeboSimSystemInterface`, and creates the fixed upstream
`gz_ros2_control/GazeboSimSystem` through a managed loader. The loader member
precedes the shared instance so reverse destruction unloads safely. Canonical
product Xacro still selects the standard upstream plugin directly.

This tracer deliberately implements only the pure `initSim`, `read`, and
`write` delegation needed for a loadable Interface instance. It does not yet
claim lifecycle/interface-export/mode-switch parity, post-delegate
`JointVelocityCmd` observation, or journaling; each remains a later behavior
RED rather than untested breadth.

```text
voice_nav_sim forced-configure build: passed
plugin-load CTest: 1/1 passed
cppcheck/lint_cmake/uncrustify/xmllint: 4/4 passed
warning-free incremental build: passed
voice_nav_sim package CTest: 12/12 passed
scoped xUnit: 54 tests, 0 errors, 0 failures, 6 skipped
user Gazebo PID 3631225: identical and live before/after package gate
root crash-stop checker: expected RED, now advances to missing crash_stop_policy.py
```

The first configure also proved
[PIT-0054](../reference/engineering-pitfalls.md#pit-0054-ament-cmake-fixes-the-link-signature-style-per-target):
ament had already selected the plain `target_link_libraries` signature, so a
later `PRIVATE` form was invalid. A grouped cross-shell CTest regex then
reproduced PIT-0001; separate literal test names produced the valid lint gate.
Neither wrapper/configuration failure was counted as the behavior RED.

Independent review reported P0=0, P1=0 and one explicit P2: selecting this
temporary Adapter in a test URDF before the next microcycle would bypass
upstream lifecycle, interface-export, and mode-switch behavior. Product and
test transformers therefore remain forbidden from selecting it until complete
parameter/return parity is behavior-tested.

### Complete Adapter forwarding and post-write observation checkpoint

The follow-up vertical cycles now cover the complete Jazzy 1.2.19 forwarding
surface: `initSim`, lifecycle transitions, interface export, mode switching,
`read`, and `write` preserve the exact arguments and upstream result. The test
constructor injects the upstream Interface, a package-private hardware-write
sink, and its generation; the default plugin still loads the pinned upstream
through the managed loader.

The first post-write RED required evidence from the real ECM wheel components,
not the exported command-interface cache. GREEN now calls upstream `write()`
first, then re-queries left and right `JointVelocityCmd`, copies each `double`
into its exact IEEE-754 bit pattern with `memcpy`, and records simulation stamp,
delegated result, generation, and the first contiguous `write_seq`. The fake
upstream creates both components lazily during `write()` and returns `ERROR`, so
the test proves ordering as well as result parity. A separate RED/GREEN clears
old ECM/entity bindings before a repeated failed `initSim()`; a later write
cannot create evidence from the stale world.

```text
focused Adapter behavior: 10/10 passed
warning-free focused build: passed
voice_nav_sim package CTest: 12/12 passed
scoped xUnit: 63 tests, 0 errors, 0 failures, 6 skipped
Adapter static contract: passed
protected Gazebo fingerprint before/after:
  3631225|1|34712103|gz sim server (identical)
root crash-stop checker: expected RED at missing crash_stop_policy.py
```

Independent review again found P0=0 and P1=0. Its remaining P2 is intentionally
the next ledger slice, not a completed claim: missing/null/duplicate wheel
entities, missing/empty command components, sequence exhaustion, and sink
rejection must latch an oracle fault without changing the upstream hardware
result. Product Xacro still selects only `gz_ros2_control/GazeboSimSystem`; at
this checkpoint the test transformer remained unimplemented and therefore
could not yet select the Adapter.

### Owned crash robot-description transformer checkpoint

The next tracer began with a registered pytest that could not load the missing
`crash_robot_description.py`. GREEN introduced one pure function over expanded
URDF text. It parses XML, requires exactly one canonical
`gz_ros2_control/GazeboSimSystem` plugin in one hardware block, replaces that
plugin with the owned Adapter, and appends only `journal_name` and
`journal_nonce`. The test removes those two additions, restores the original
plugin, and compares recursive element signatures, so unrelated tags,
attributes, text, or child order cannot change unnoticed.

Later RED/GREEN cycles reject a pre-existing owned parameter, an empty or
unsafe POSIX shared-memory name, and a nonce that is not exactly 32 lowercase,
nonzero hexadecimal digits. Missing, duplicate, unexpected, or multiply owned
hardware plugins are also characterized as fail-closed inputs. Canonical
product Xacro remains byte-for-byte unchanged and still selects only the
upstream plugin.

The first static run exposed a checker bug rather than a transformer defect:
the Windows-drive regex matched `p:/` inside the standard Apache `http://`
license URL. A synthetic-repository RED now requires URLs to pass while real
`C:/`, `C:\\`, `/home`, and `/mnt/c` paths still fail; a boundary-aware regex
made all 19 non-root crash-contract tests GREEN.

```text
focused transformer pytest: 4 tests passed
transformer static contract: passed
crash-contract sibling tests: 19 passed, 1 intentional root test deselected
voice_nav_sim package CTest: 13/13 passed
scoped xUnit: 69 tests, 0 errors, 0 failures, 6 skipped
protected Gazebo fingerprint before/after:
  3631225|1|34712103|gz sim server (identical)
root crash-stop checker: expected RED at missing crash_stop_policy.py
```

### Hardware-write ledger core checkpoint

The hardware observation types were first extracted from the Gazebo Adapter
into the independent `HardwareWriteSink` seam. A pure C++
`HardwareWriteLedger` Adapter now sits behind that same one-method Interface,
so its protocol tests do not load Gazebo, ROS, or pluginlib. Construction is
the only allocation point: append and SEAL use fixed preallocated segment
storage plus lock-free atomics; immutable snapshot copying and CRC calculation
run after the release-published seal.

Vertical RED/GREEN cycles prove one sealed write, identical-tuple folding,
tuple-change segmentation, bounded pagination with previous-page checksum
chaining, duplicate/gap/non-wrapping sequence handling, nondecreasing
simulation stamps, finite wheel values, capacity exhaustion, and a numeric
zero-required predicate that accepts either signed zero. CRC64-ECMA-182 is
checked by an independent implementation, a fixed external constant, and a
bit-flip rejection. Every attempted invocation is retained in the fence/count
metadata even when no valid segment can be created.

Independent review found that the first implementation could not SEAL an
interval whose first write was itself invalid. A dedicated RED then required
a zero-segment fault-only page. The first repair covered semantic faults but
not a first sequence gap; a second RED made attempted sequence/count evidence
precede sequence validation. Final review reported no remaining P0-P2 in this
core slice. It also required and verified a compile-time lock-free assertion
for the atomic sealed flag.

```text
focused hardware_write_ledger GTest: 14/14 passed
focused cppcheck + uncrustify: passed
voice_nav_sim package CTest: 14/14 passed
scoped xUnit: 93 tests, 0 errors, 0 failures, 10 skipped
independent final review: P0=0, P1=0, P2=0
protected Gazebo fingerprint:
  3631225|1|34712103|gz sim server (unchanged)
root crash-stop checker: still expected RED at missing crash_stop_policy.py
```

This checkpoint is not the complete cross-process oracle. Atomic shared-memory
ARM/SEAL requests, immutable dual-bank retention until ACK, fixed mapping ABI,
Adapter construction from the transformed journal identity, and fail-closed
handling of missing wheel entities/components, sink rejection, and Adapter
sequence exhaustion remain the next slices.

### Shared-memory ARM and deferred-SEAL checkpoint

The cross-process Module now owns a 192-byte control area and one request
envelope with explicit `IDLE -> WRITING -> READY -> READING -> IDLE`
ownership. Writer copies a fixed snapshot only after claiming READY. Immediate
requests release the envelope before their receipt; deferred SEAL retains
READING until the qualifying write has finished, preventing a late retry from
stranding READY after the final write. Response CRC binds the consumed request
checksum rather than mutable request bytes.

ARM, global non-wrapping sequence assignment, multi-write append/folding,
simulation-stamp regression, ticket wrap/replay, and single-Writer lifecycle
are executable. Deferred exact SEAL was added tests-first: stamps 100 and 150
remain inside ACTIVE, the stamp-200 BEGIN still has no receipt, and only its
FINISH appends sequence three, publishes `last_completed_write_seq`, fixes the
three-segment/two-page root CRC, release-publishes SEALED_OK, and then publishes
the fence-three receipt. A corrupted segment count now faults and remains
unsealed without traversing untrusted geometry.

Independent review found the pending-retry and untrusted-count P1 defects;
both received deterministic RED tests before correction. Its remaining P2 is
not claimed complete here: qualifying fault writes must still prove exact
attempted fence/count metadata, and the invalid SEAL/exact-skip/terminal
immutability matrix remains the next slice before ACK and page reads.

```text
owned-envelope ABI/behavior commits: 7faa91e..43bf6de
deferred-SEAL RED/refinement/review RED: d92bef8, 68c03a6, 9d958ad
deferred-SEAL GREEN: f6cd3ef
cross-process CTest: 5/5 consecutive executions passed
focused C ABI/concurrency/cross-process/core CTests: 4/4 passed
Writer TSAN CTest: 10/10 consecutive executions passed
cppcheck/flake8/lint_cmake/pep257/uncrustify: 5/5 passed
package/full-repository/Gazebo runtime gates: not run for this checkpoint
root crash-stop checker: still expected RED at missing crash_stop_policy.py
```

### Attempt-accounting and segment-integrity checkpoint

The next TDD cycles separated the bank's complete attempted interval from its
stored tuple evidence. A qualifying invocation-budget failure now advances the
bank fence/count before the deferred exact SEAL terminalizes. Exact-stamp skip
coverage proves that the qualifying write is retained before `SIM_STAMP` fixes
the bank as faulted. An invalid wheel observation consumes its sequence but
does not fabricate exact command bits; a later valid tuple can still occupy
the first segment, and a later transition that exhausts segment capacity adds
only the expected sticky faults.

Independent review then found one remaining clean-bank integrity hole: a
mutated segment count could discard recorded coverage while leaving attempted
metadata and the fault mask apparently clean. RED commit `1727a16` reproduced
the false `SEALED_OK`. GREEN commit `220fdec` validates every bounded stored
range, permits count gaps only after a sticky fault, and requires exact stored
versus attempted invocation equality for an otherwise clean bank. Review
confirmed P0=0, P1=0; protocol and PIT-0062 now make the raw timestamp-regression
segment exception explicit.

```text
attempt/fault coverage commits: 8a57388..cae61bb
clean-coverage RED/GREEN: 1727a16, 220fdec
focused C ABI/concurrency/cross-process/core CTests: 4/4 passed
cross-process CTest: 5/5 consecutive executions passed
Writer TSAN CTest: 10/10 consecutive executions passed
cppcheck/flake8/lint_cmake/pep257/uncrustify: 5/5 passed
PIT-0011 invocation-only failure: corrected by sourcing base + overlay
PIT-0042 clock-skew warning: identical target rebuild passed cleanly
package/full-repository/Gazebo runtime gates: not run for this checkpoint
root crash-stop checker: expected RED at missing crash_stop_policy.py
```

### Invalid-SEAL and terminal-immutability checkpoint

A terminal-bank tracer found that `process_seal()` correctly rejected an
invalid request but echoed the request's real bank/epoch in its receipt. RED
commit `ce4598a` proved the misleading identity while also snapshotting every
bank word and the full fixed segment capacity around both BEGIN and FINISH.
GREEN `ec693da` returns the canonical no-selection identity instead.

Follow-up coverage names the all-ones sentinel in ABI v1, rejects a corrupted
request checksum with an independently verified response CRC, mutates every
SEAL field against an ACTIVE bank, and then proves a correct next-ticket SEAL
still succeeds. Equivalent post-terminal requests leave both `SEALED_OK` and
`SEALED_FAULT` evidence byte-for-byte unchanged; only the global `PROTOCOL`
fault records the invalid control attempt. PIT-0063 captures the reusable
failure pattern.

```text
canonical INVALID RED/GREEN: ce4598a, ec693da
checksum/sentinel coverage: b0cf723
field matrix and both terminal states: 84f9bda
focused C ABI/concurrency/cross-process/core CTests: 4/4 passed
cross-process CTest: 5/5 consecutive executions passed
Writer TSAN CTest: 10/10 consecutive executions passed
cppcheck/flake8/lint_cmake/pep257/uncrustify: 5/5 passed
repository contract: passed
protected Gazebo: 3631225|1|34712103|gz sim server (unchanged)
package/full-repository/Gazebo runtime gates: not run for this checkpoint
root crash-stop checker: expected RED at missing crash_stop_policy.py
```

### Immutable Parent pages and exact-ACK checkpoint

Parent now exposes one deep `read_sealed_interval()` / `acknowledge()` Module
instead of raw mmap operations. It acquire-observes a terminal bank, validates
the immutable header and bounded bank geometry, copies only finalized
segments, checks tuple/range/predicate semantics, independently recomputes the
bank CRC, and constructs the complete local page CRC chain. It then rereads
the bank and segments and acquire-rechecks the terminal state before returning
a frozen snapshot. Reading has no shared-memory side effect.

The first RED/GREEN pair proves three segments split into two immutable pages,
followed by one exact ACK and duplicate rejection. The next pair retains both
banks, observes `NO_FREE_BANK`, releases only an exact snapshot, advances the
reused bank epoch, and rejects stale or altered identity/root/page snapshots.
Review found that an invocation-budget fault is valid attempted evidence even
when its count exceeds the budget; `02c0783` reproduced the unreadable
`SEALED_FAULT` and `39d1315` now requires `CAPACITY` rather than rejecting it.

Review also found a same-Parent ABA: two callers could validate one old
snapshot before either performed the state-only CAS. Deterministic RED
`c870a4b` paused the first caller after its full copy and showed the duplicate
winning the release. GREEN `1345750` claims the registered snapshot under a
Parent-local lock, marks that bank in flight through validation and its single
CAS, and rejects duplicate ACK/read attempts. Re-review reports P0=0/P1=0;
PIT-0064 and PIT-0065 preserve both patterns.

```text
immutable page RED/GREEN: 6279025, 58ec486
dual-bank/exact-ACK RED/GREEN: eb39405, 328cdc1
faulted-attempt reader RED/GREEN: 02c0783, 39d1315
duplicate-ACK ABA RED/GREEN: c870a4b, 1345750
focused C ABI/concurrency/cross-process/core CTests: 4/4 passed
cross-process CTest: 5/5 consecutive executions passed
Writer TSAN CTest: 10/10 consecutive executions passed
cppcheck/flake8/lint_cmake/pep257/uncrustify: 5/5 passed
static repository contract: passed
unsourced repository runner: reproduced PIT-0011 import failures
sourced repository runner: 319 tests, exactly 1 deliberate root RED at the
  missing src/voice_nav_sim/test_support/crash_stop_policy.py
protected Gazebo: not signaled, stopped, or reused
package/full-repository/Gazebo runtime gates: pending the crash-stop policy
```

### Real Gazebo Adapter runtime checkpoint

The previous checkpoints proved the ledger and Adapter separately. Commit
`9f4df62` adds the first vertical runtime proof through the default constructor:
the launch test expands the canonical product Xacro, transforms only the
hardware plugin, supplies the Parent-owned name/nonce, and starts a fresh
headless Gazebo partition. The ledger accepts only the exact launch-managed
Gazebo PID. After both controllers become ACTIVE, Parent ARMs, publishes a
bounded non-zero stamped drive command, posts inclusive SEAL, reads the full
immutable page chain, and ACKs it. GREEN requires a clean terminal bank and
global fault word, Writer-contiguous fences, `VALID + upstream OK`, finite
left/right command bits with at least one non-zero pair, and exact ACK.

The first runner RED was only a fixture import failure: launch-testing does not
place the source `test/` directory on `sys.path`. Loading the sibling module by
its file path reached the real behavior RED, `SEALED_FAULT/CAPACITY (0x10)`.
The assumption that evidence ran at the 100 Hz controller rate was wrong;
Gazebo called hardware `write()` at each 1 ms simulation step. The fixture now
uses a hard 8192-segment bound and only a 120 ms command window. PIT-0068 keeps
that distinction visible.

Commit `09bba8b` records the hardened contract. Independent reviews found that
keyword search could pass after assertions were deleted, reversed, skipped,
rebound, or made unreachable;
response fields and snapshot provenance could be forged; transformed XML
could miss the launched RobotStatePublisher; and CMake labels, variables,
Python overrides, skip flags, or later property writes could masquerade as the
effective runner/serialization configuration. The repaired checker binds the
top-level collected TestCase, unmodified `self` assertions, exact ordered
Parent flow, snapshot-page generator, wheel-bit decoder, and direct RSP launch
action. Its CMake parser mirrors keyword consumption and requires an unchanged
`BUILD_TESTING`, exact isolated runner, no bypass arguments, and one explicit
final serial property. PIT-0069 records why keyword presence is not executable
evidence.

```text
runtime test RED 1: sibling test-support module absent from runner sys.path
runtime test RED 2: SEALED_FAULT, bank/global fault 0x10 (CAPACITY)
runtime test GREEN: 3/3 fresh launches, 43.43 s total
focused Adapter/Writer/POSIX/transform/runtime CTests: 5/5 passed
static crash-stop contract: 65/65 passed
voice_nav_sim package gate: 19/19 CTests; 134 xUnit, 18 skipped
generated CTest: exact isolated runner; RUN_SERIAL=True; TIMEOUT=180
full scripts/verify.sh: passed
repository tests: 364 passed
all-package scoped xUnit: 398 total, 0 errors, 0 failures, 39 skipped
clean MotionGate install audit: passed
post-run /dev/shm/voice_nav_hardware_* inventory: empty
protected Gazebo: 3631225|1|34712103|gz sim server (unchanged)
```

This checkpoint proves real Adapter composition and lossless hardware-write
evidence. It does not satisfy any authority, candidate, or MotionGate SIGKILL
row below; those remain the next VN-0011A slice.

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
