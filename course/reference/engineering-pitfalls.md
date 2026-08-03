# VoiceNav Robot engineering pitfalls

This is the course's reusable diagnostic register. It records failure patterns,
not a chronological activity log. The governing capture and recurrence policy
is [Problem learning and recurrence control](../../docs/process/problem-learning.md).

## Quick routing

| ID | Symptom | First discriminator | Status |
| --- | --- | --- | --- |
| PIT-0001 | WSL command has a Bash syntax error or expands the wrong text | Did PowerShell parse quotes, `$`, or command substitution first? | Guarded |
| PIT-0002 | WSL prints localhost/NAT noise or starts slowly | What is the command exit code and decisive test output? | Guarded |
| PIT-0003 | Native Windows Git reports dubious ownership | Is Git running as the repository-owning WSL user? | Guarded |
| PIT-0004 | `rosidl_generate_interfaces` rejects an interface package | Is `rosidl_interface_packages` membership in `package.xml`? | Guarded |
| PIT-0005 | Gazebo rejects `base_footprint` inertia | Does the logical planar frame carry physical inertia? | Guarded |
| PIT-0006 | xacro reports `unknown macro name: xacro:material` | Was a `material` macro actually defined or included? | Guarded |
| PIT-0007 | DDS is matched but MotionGate reports writer identity mismatch | Is graph identity temporarily unresolved for the same non-zero GID? | Guarded |
| PIT-0008 | Local evidence was green, then the final change invalidated it | Were tests rerun on the exact final HEAD? | Guarded |
| PIT-0009 | Code/tests support a case that the lesson still forbids | Do prose, tests, and implementation describe the same closed set? | Guarded |
| PIT-0010 | A bounded RPC returns success after its total budget | Is the deadline checked again immediately after the RPC? | Guarded |
| PIT-0011 | A CTest cannot import ament or the built workspace package | Were ROS and the workspace overlay sourced in that shell? | Guarded |
| PIT-0012 | Product assertions pass, but Gazebo exits `-9` during CTest teardown | Did failure occur in the active test or the strict post-shutdown exit check? | Guarded |
| PIT-0013 | A focused runner test passes, but canonical discovery cannot import the real test tree | Does the fixture match the repository's package markers and import path? | Guarded |
| PIT-0014 | Concurrent launch tests collide despite a fixed `ROS_DOMAIN_ID` | Does generated CTest metadata invoke the official isolated runner without overriding its domain? | Guarded |
| PIT-0015 | One cleanup failure prevents later fixture destruction | Are teardown phases independent LIFO cleanups with exhaustive error aggregation? | Guarded |
| PIT-0016 | A green report did not execute the intended contract | Do source inventory, generated CTest, and critical xUnit name the same tests with no unapproved skip? | Guarded |
| PIT-0017 | Repeated WSL simulation fails while collecting Gazebo pose evidence | Is the server unhealthy, or did the bounded CLI query time out / emit a small JSON burst? | Guarded |
| PIT-0018 | CTest prints a failure but the surrounding command returns zero | Did a later diagnostic command replace the gate's shell exit status? | Guarded |
| PIT-0019 | Generated CTest passes with a wrong label, timeout, or working directory | Does the checker compare exact property values or only property names? | Guarded |
| PIT-0020 | A scaled quaternion produces the wrong RPY and misleading movement evidence | Is the finite valid quaternion normalized before unit-quaternion formulas? | Guarded |
| PIT-0021 | A bounded diagnostic loses mandatory fields when one value is long | Are variable fields compacted independently before composing the fixed field layout? | Guarded |
| PIT-0022 | Canonical verification rejects an xUnit file that changed during evidence collection | Did another reviewer or test process write the same shared build tree? | Guarded |
| PIT-0023 | Current lesson content no longer matches its frozen solution tag | Was later material marked as post-tag errata with a new cumulative checkpoint? | Guarded |
| PIT-0024 | Delivery closure creates another closure PR | Is the tree being asked to contain its own future tag or public identity? | Guarded |
| PIT-0025 | Controller deadman appears to fire in one update instead of 0.35 s | Was latency measured from a periodic output instead of the final input stamp used for command age? | Specified |
| PIT-0026 | A crash/pause test sees zero but may have dropped a non-zero wheel write | Is a lossy diagnostic stream being used to prove exact first-write or no-regression? | Specified |
| PIT-0027 | A process-kill test passes although the deadman had already expired | Were all live/non-zero facts captured in one bounded arming barrier immediately before SIGKILL? | Specified |
| PIT-0028 | A side observer's last input is treated as the controller's final accepted input | Does a crash-resilient final source commit also have a controller-output ACK? | Specified |
| PIT-0029 | A resume probe writes old zero but never crosses a controller update | Did exact pause+single-step transactions prove update zero and then write it? | Specified |
| PIT-0030 | A steady-time arming deadline flakes when simulation RTF falls | Were steady Gate freshness and simulation-sample freshness constrained on separate clocks? | Specified |
| PIT-0031 | A “lossless” write log silently overwrites or omits records | Are sequence fences, capacity, overflow, and snapshot continuity executable invariants? | Specified |
| PIT-0032 | A correct producer-death test fails while authority RENEWs continue | Is terminal `control_seq` compared with the final committed predecessor instead of the arming snapshot? | Specified |
| PIT-0033 | Delayed DDS delivery makes a pre-death zero look post-death | Do same-host transition/output pre-call fences occur after `ProcessExited`? | Specified |
| PIT-0034 | A dynamically loaded test-support module fails while decorating a dataclass | Does the loader register the module, or does the support type avoid that hidden dependency? | Guarded |
| PIT-0035 | A concrete plugin class looks inheritable but an external subclass does not compile | Is the documented plugin Interface the real extension seam? | Specified |
| PIT-0036 | A hardware journal claims a Gazebo iteration it cannot observe | Does the callback actually receive that field, or must World Statistics own it? | Specified |
| PIT-0037 | A value-matched controller ACK may correspond to a periodic repeat | Was the final marker committed exactly once and ACKed before the next Gate period? | Specified |
| PIT-0038 | “No allocation in the write seam” accidentally includes upstream code | Is the real-time claim scoped to added instrumentation only? | Specified |
| PIT-0039 | A late journal commit makes an earlier transition look post-crash | Is the recorded time the transition linearization fence or only a later snapshot? | Specified |
| PIT-0040 | Some Gate transitions have evidence while equivalent paths do not | Is journaling owned by one Core transition seam or scattered through Node callbacks? | Specified |
| PIT-0041 | A checksum test stays green after its implementation omits a field | Does an independent constant and an include/exclude mutation matrix define the oracle? | Guarded |
| PIT-0042 | An incremental WSL build warns that a dependency file is milliseconds in the future | Does the same target rebuild cleanly after comparing WSL time and file epoch? | Guarded |
| PIT-0043 | A full evidence journal prevents MotionGate from inhibiting or faulting | Is evidence failure policy different for admission and safety-terminal mutations? | Guarded |
| PIT-0044 | A non-copyable Core still shares one journal with another Core | Is ownership carried by a one-shot capability rather than an object trait? | Guarded |
| PIT-0045 | Shared-memory identity is read before the producer publishes `READY` | Did the consumer acquire `READY` before reading any ordinary payload field? | Guarded |
| PIT-0046 | A checksum covers a field, but every producer record still writes zero | Does a non-zero golden test prove the producer API accepts and serializes the semantic value? | Guarded |
| PIT-0047 | A newly declared CMake target reports “No rule to make target” | Was the cached package explicitly reconfigured before interpreting a target-level RED? | Guarded |
| PIT-0048 | A read-only parameter test passes although the parameter is undeclared | Did `describe_parameters` prove the exact name, string type, and read-only descriptor before testing mutation? | Guarded |
| PIT-0049 | A direct package install leaves a manifest in the repository root | Was installation run through `colcon`, followed by a clean-worktree check? | Guarded |
| PIT-0050 | A static checker rejects the correct implementation after a field refactor | Do both the synthetic fixture and real-repository positive controls use the current semantic token? | Guarded |
| PIT-0051 | One launch test intentionally starts both valid and invalid processes | Are exit codes asserted per exact launch action instead of globally? | Guarded |
| PIT-0052 | An evidence/DDS failure prevents the direct safety-zero fallback | Can fault recording itself throw before the zero publisher is called? | Guarded |
| PIT-0053 | Jazzy rejects `rclcpp::Time::to_msg()` while wiring a message stamp | Was an API from another ROS distribution assumed instead of compiling the target? | Guarded |
| PIT-0054 | CMake rejects a target after `ament_*` wiring with a plain/keyword signature conflict | Did a later `target_link_libraries(... PRIVATE ...)` mix styles with an ament macro's plain call? | Guarded |
| PIT-0055 | A PowerShell-to-WSL script pipe prepends a UTF-8 BOM or strips Bash variables | Are UTF-8 bytes transported as Base64 before Bash decodes them? | Guarded |
| PIT-0056 | A protected WSL process appears to change its start time | Is identity based on raw `/proc/<pid>/stat` start ticks instead of `ps lstart`? | Guarded |
| PIT-0057 | A drive-letter regex rejects an ordinary source URL | Does the path detector require a token boundary and test both URLs and real machine paths? | Guarded |
| PIT-0058 | Entry-time SEAL excludes the write being proved | Is SEAL deferred until the qualifying write is recorded? | Guarded |
| PIT-0059 | TSAN intermittently crashes before the test under WSL | Does only the TSAN executable run non-PIE with per-process ASLR disabled? | Guarded |
| PIT-0060 | A mailbox replay races or binds its response to changed request bytes | Does a pending request retain Writer ownership and bind the consumed snapshot? | Guarded |
| PIT-0061 | A fail-closed checksum path reads beyond fixed evidence storage | Was untrusted count geometry rejected before traversal? | Guarded |
| PIT-0062 | An invalid write becomes a fake segment or makes the next valid write look out of sequence | Are attempted metadata and recordable segment coverage validated separately? | Guarded |
| PIT-0063 | An INVALID receipt appears to select an existing terminal bank | Does rejection use the canonical invalid identity without touching bank evidence? | Guarded |
| PIT-0064 | A valid faulted interval cannot be read because attempts exceed its armed budget | Does CAPACITY explain the attempted count instead of making the bank structurally unreadable? | Guarded |
| PIT-0065 | Two Parent ACK callers can release a later bank epoch | Is the registered snapshot claimed before validation and excluded until its single CAS completes? | Guarded |

## PIT-0001: Windows-to-WSL quoting is a two-shell contract

**Symptom.** A valid-looking Bash command fails near `(`, a commit message is
split, `$variable`/command substitution executes before Bash receives it, or a
multi-line GitHub body fetched by PowerShell is expanded into many CLI flags.

**Cause.** PowerShell parses the outer command line before `bash -lc` parses the
inner script. Nested double quotes and `$()` therefore have two interpreters.

**Safe diagnostic and guardrail.** Prefer direct argument passing for one
program:

```powershell
wsl.exe -d Ubuntu-24.04 -- git -C /mnt/c/... status --short
```

Use `bash -lc '...'` only when shell features or ROS environment sourcing are
required, keep the inner program single-quoted at the PowerShell layer, and
avoid command substitution in the outer command. For serial CTest repetition,
prefer `ctest --repeat until-fail:20` over a shell loop. A quoting failure is
not evidence that any Git mutation occurred; inspect `git status` immediately.
Regex characters such as `()|` are shell metacharacters too; either quote them
inside one controlled Linux shell or run the small complete CTest set instead
of building a fragile cross-shell filter. PowerShell captures native command
output as a line array; before passing multi-line text to another native
command, join it explicitly to one string and quote the argument, or prefer the
tool's stdin/file option. If an edit command rejects the first body line as an
unknown flag, verify the remote object before retrying; do not assume a partial
mutation. For a ROS compiler-feasibility probe with transitive include/link
requirements, prefer a tiny explicit CMake target in an owned temporary source
directory over nested PowerShell/Bash command substitution that synthesizes
`-I` flags. Remove the probe after preserving the compiler result and inspect
`git status`. Do not embed pytest `-k` expressions containing spaces, Awk
programs containing `$0`, or `$(git ...)` identity guards inside a
PowerShell-to-`bash -lc` string. Read Git identity in a separate native command;
use pytest's no-space `--deselect=<node-id>` form and PowerShell-native text
inspection when either shell can consume the inner syntax. PowerShell does not
expand a Linux-style wildcard embedded in a path argument for a Windows-native
`rg`; pass the directory and use `rg -g '<pattern>'` instead. A quoted CTest
regex containing `|` and a quoted `stat -c` format containing spaces can lose
their inner quote boundary at the same shell crossing. Prefer separate CTest
invocations and no-space diagnostic formats such as `stat -c %Y`.

**Recurrence evidence.** During VN-0011A Core-journal integration, a grouped
CTest regex containing `()` and `|` again reached Bash without its intended
quote boundary and failed before any build or test ran. The replacement used
separate literal `ctest -R <name>` invocations. This recurrence confirms that
the safe template, not another layer of escaping, is the permanent process
guardrail.

During the Layer-2 journal slice, PowerShell also consumed a Bash
`tmpdir=$(mktemp ...)` expression before the Linux shell received it. The empty
value degraded later paths to `/build` and `/log`, which failed with permission
errors before any product test ran. The valid RED used the explicit path
`/tmp/vn-cross-red-20260802-001`. A later grouped CTest regex again reached
Bash as syntax near `(`; separate literal test names remained the repair. In
both cases `git status` confirmed that the wrapper failure had not mutated the
repository, and neither event was counted as product RED.

The Adapter GREEN lint attempt reproduced the grouped-regex failure a third
time: a cross-shell `ctest -R "a|b|c"` lost its quote boundary, ran one test,
then treated the remaining names as shell pipelines. Four separate literal
CTest invocations replaced it; all passed. This was wrapper evidence only and
did not alter the focused Adapter result.

## PIT-0002: WSL transport warnings are not the command result

**Symptom.** WSL emits a localhost/NAT warning, sometimes with garbled Windows
encoding, or cold startup exceeds a short timeout.

**Cause.** WSL startup/network diagnostics and the Linux child process have
separate outcomes. The warning may accompany either exit zero or a real child
failure.

**Safe diagnostic and guardrail.** Use a bounded but realistic startup timeout,
then judge the child exit code and decisive ROS/test output. Do not suppress all
stderr and do not classify a run as green from the warning text alone.

## PIT-0003: Repository ownership differs across Windows and WSL identities

**Symptom.** Native Windows Git reports `detected dubious ownership` even though
Git inside WSL works.

**Cause.** The sandboxed Windows process and the WSL user have different
security identities; Git correctly refuses an untrusted owner by default.

**Safe diagnostic and guardrail.** Run repository Git commands as the configured
WSL user. Do not add a broad global `safe.directory` exception from project
automation. Confirm branch, status, and recent commits after reconnecting.

## PIT-0004: A ROS interface package must declare its group

**Symptom.** Jazzy CMake reports that packages installing interfaces must
include `member_of_group`.

**Cause.** `rosidl_generate_interfaces()` is present, but `package.xml` omits:

```xml
<member_of_group>rosidl_interface_packages</member_of_group>
```

**Guardrail.** Keep the declaration beside the generator dependencies and run a
package-select build plus `ros2 interface show`. See
[Lesson 0002](../lessons/0002-define-first-interfaces.md).

## PIT-0005: `base_footprint` is a logical frame, not a physical body

**Symptom.** Gazebo model creation fails with `A link named base_footprint has
invalid inertia`.

**Cause.** A massless navigation frame was given an invalid or unnecessary
inertial model. Physical collision, visual, mass, and inertia belong on
`base_link`; `base_footprint` exists to anchor planar navigation TF.

**Guardrail.** Keep `base_footprint` free of visual/collision/inertial elements
and connect it to the physical base with the intended fixed transform. Validate
with `check_urdf`, spawn, and TF inspection. See
[URDF physical properties](urdf-physical-properties.md) and
[Lesson 0004](../lessons/0004-spawn-physical-robot-in-gazebo.md).

## PIT-0006: `xacro:material` is not an implicit built-in macro

**Symptom.** xacro reports `unknown macro name: xacro:material`.

**Cause.** The file invokes a project macro named `material`, but that macro was
not defined or included before expansion. A normal URDF `<material>` element is
different from a `<xacro:material>` macro invocation.

**Guardrail.** Either use standard URDF material syntax or define/include the
macro explicitly. Run xacro to a temporary URDF before launching
`robot_state_publisher`. See
[Lesson 0003](../lessons/0003-build-static-robot-model.md).

## PIT-0007: DDS matching and ROS graph identity are non-atomic

**Symptom.** The candidate writer and reader are DDS-matched, but a Gate-local
`get_publishers_info_by_topic()` snapshot temporarily reports unresolved node
identity and OPEN fails intermittently.

**Supported cause.** Transport matching and graph metadata convergence are
different observations, not one atomic transaction. The exact middleware field
arrival order is not assumed.

**Guardrail.** Only the exact typed `WRITER_METADATA_PENDING` state is retryable:
there must be one publisher, correct type/kind/QoS, a non-zero GID pinned to the
PREPARE generation, and every known name/namespace component must agree. Empty
node name or Jazzy's exact `_NODE_NAME_UNKNOWN_` and
`_NODE_NAMESPACE_UNKNOWN_` sentinels may be unresolved. Contradictory known
identity, replacement, disappearance after pinning, zero GID, or barrier change
remains terminal. See
[VN-0010-C1](../../docs/work-items/0010-corrective-writer-identity-convergence.md)
and [Lesson 0009](../lessons/0009-build-independent-motion-gate.md).

## PIT-0008: Acceptance evidence belongs to an exact commit

**Symptom.** A work item says "all tests passed," but product code, tests, or
configuration changed afterward.

**Cause.** Diagnostic evidence from an ancestor was copied forward as release
evidence.

**Guardrail.** Keep pre-final evidence labeled as such. After the final product
or contract change, rerun focused tests, repeated fresh-launch tests, clean
install audit, and canonical `scripts/verify.sh`. Record that exact local or
pushed HEAD and its result in the PR / Issue evidence, not in a new commit to
the tree it verifies. Hosted CI must test that same pushed head. See
[the change lifecycle](../../docs/process/change-lifecycle.md#不可自引用的交付身份).

## PIT-0009: Documentation is part of the closed-set contract

**Symptom.** Implementation and tests allow exact Jazzy UNKNOWN sentinels, but a
Work Item or lesson still says only node name may be unresolved.

**Cause.** Behavior evolved after the first documentation commit and prose was
not included in the same review matrix.

**Guardrail.** Independent review compares implementation, tests, static
contract, Work Item, and lesson. When a closed set changes, update all five in
one corrective slice and retain the old evidence as pre-final rather than
silently relabeling it.

**Recurrence evidence.** VN-0011A froze the test-only Node keys as
`test_gate_event_journal_name` and `test_gate_event_journal_descriptor` in the
Work Item, safety architecture, and implementation, while the crash-stop
product-isolation checker and its mutations still exercised an obsolete
`crash_journal_name` spelling. The closed-set review therefore continued at
the stale-guard boundary described in PIT-0050 instead of treating a rejection
of the unused spelling as product evidence.

The same review found two incompatible publication owners in one architecture
document: an earlier paragraph assigned successful-output counters to the Node
Adapter, while the later frozen crash contract assigned the transaction,
counters, and terminal-cause binding to `MotionGateProcessRuntime`. The deeper
Runtime boundary is authoritative: Node now supplies only ROS time/transport
and maps the returned facts. This contradiction was removed before coding the
transaction, so tests do not institutionalize two ownership models.

## PIT-0010: An absolute deadline surrounds the RPC

**Symptom.** A convergence helper checks the remaining budget before an RPC,
but accepts an APPLIED response that arrives after the absolute deadline.

**Cause.** Passing `remaining` to a transport is not itself proof that every
transport implementation returned within that budget. The caller omitted its
own post-RPC steady-clock check on the terminal path.

**Guardrail.** Update `last_response` and attempt count, then check the absolute
deadline immediately after every RPC and before either terminal return or
pending validation/backoff. A fake clock regression test advances past the
deadline inside `attempt()` and proves the late APPLIED result is rejected.

## PIT-0011: CTest needs the ROS environment, not only a build directory

**Symptom.** Every CTest target fails at `/opt/ros/.../run_test.py` with
`ModuleNotFoundError: ament_cmake_test`, or a direct repository Python suite
fails every ROS-aware support test with `ModuleNotFoundError: launch` while
unrelated pure tests pass. A package launch test can also fail with
`ModuleNotFoundError: voice_nav_mission` when the base ROS installation was
sourced but the newly built workspace overlay was not.

**Cause.** CTest or `scripts/run_repository_tests.py` was invoked from a fresh
WSL shell without sourcing ROS, or only the base installation was sourced when
the test imports the workspace's installed Python package. Generated ament
wrappers and support modules need both applicable Python paths; compiled
binaries and an existing build tree do not supply that environment.

**Guardrail.** Source the base installation and current overlay in the same
shell before CTest:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash  # when exercising the built overlay
ctest --test-dir build/voice_nav_mission --output-on-failure

# Root contracts that import installed ROS Python packages need at least base:
python3 scripts/run_repository_tests.py
```

When a diagnostic shell uses Bash strict mode, enable `set -e -o pipefail`,
source both setup files, and only then enable `set -u`. Jazzy setup scripts may
legitimately inspect an unset environment variable; enabling nounset first
turns environment initialization into a false product failure.

If all targets fail at the same import, fix the invocation before diagnosing
product code. Canonical `scripts/verify.sh` already sources both environments.

**Recurrence evidence.** VN-0011A once sourced only the base installation; the
package's launch test alone failed to import `voice_nav_mission`, while the
same 15-test gate passed after sourcing the overlay. The Layer-2 verification
also enabled `set -u` before sourcing ROS and stopped on the intentionally
unset `AMENT_TRACE_SETUP_FILES`. Sourcing first, then enabling strict mode,
allowed the complete 16/16 package gate to run. Neither invocation failure was
accepted as a code RED. The hardware-write ledger slice reproduced both
failure modes: its first diagnostic enabled nounset too early, and a later
direct CTest omitted the sourced environment. Both were classified by this
existing pitfall before product debugging continued.

## PIT-0012: No residual Gazebo process is not a clean Gazebo exit

**Symptom.** Headless product behavior and MotionGate assertions pass, but
launch escalates Gazebo teardown through SIGINT and SIGTERM to SIGKILL. The
strict post-shutdown `assertExitCodes(proc_info)` then reports exit `-9`.

**Supported cause and uncertainty.** The narrow supported cause is that the
launch-managed Gazebo process did not exit inside the cumulative graceful
shutdown window. A historical `voice_nav_sim` occurrence did not include
MotionGate, so the evidence does not attribute the failure to the Gate. The
current launcher already uses direct argv, not the old shell-wrapper pattern.
No evidence yet identifies which Gazebo, `gz_ros2_control`, controller, or
thread prevents timely exit. Finding no newly introduced residual process is
useful supporting evidence, but `-9` still violates the clean-exit contract.

**Safe diagnostic path.** First separate an active-test assertion failure from
the strict post-shutdown process failure. Record the exact head, repeat index,
exit code, and signal escalation. Reproduce at the smaller
`voice_nav_sim`/controller seam inside its claimed `GZ_PARTITION`, and compare
the before/after process set without stopping any pre-existing user process.
A successful rerun demonstrates low frequency; it is not remediation.

**Implemented guardrail.** Every Gazebo launch-test process overwrites inherited
state with a scope/PID/128-bit-random `GZ_PARTITION` before launch context
construction. A failure-safe cleanup always attempts zero/inhibit, then issues
direct-argv `/server_control` `stop: true` with the same checked environment
snapshot, requires a positive Boolean ACK, waits for the launch-managed Gazebo
process itself to exit, and destroys the ROS fixture even if an earlier cleanup
step fails. These are independent LIFO cleanups with exhaustive sub-resource
aggregation. Only a typed CLI timeout retries the same idempotent request once;
all other errors and two timeouts fail closed. ACK is not process completion.
The tests use the official isolated ROS runner, and the final global
`assertExitCodes(proc_info)` remains unchanged. Static mutation tests reject
fixed partitions, unreachable or rebound oracles, skipped critical modules,
fixed sleeps, unbounded timeout extensions, `-9` allowlists, shell execution,
and global process killing. The canonical repository runner also treats every
skipped contract as failure. See
[VN-0010-C2](../../docs/work-items/0010-corrective-gazebo-teardown.md).

**Status and scope.** The guard is complete: both 20-run fresh-launch gates,
the exact local canonical gate, two independent P0-P2 rereviews, and required
hosted CI on the final reviewed implementation/documentation head passed.
This is a test/process-lifecycle correction. It is not Lesson 0010
Runtime/Gate process-death, controller consumer-deadman, managed safe-pause, or
first-resume-zero evidence. See
[PR #16](https://github.com/Edddddddddy/voice_nav_robot_ws/pull/16) and
VN-0010-C2 for exact evidence; the PR merge remains a separate delivery state.

## PIT-0013: Test discovery fixtures must match the repository layout

**Symptom.** Unit tests for a custom test runner pass, but running that runner
from `scripts/` fails with `Start directory is not importable` or repository
modules such as `scripts.colcon_evidence` cannot be imported.

**Cause.** The focused fixture used an importable temporary package while the
real repository deliberately has no `tests/__init__.py`. In addition,
`unittest.defaultTestLoader` is a mutable singleton whose remembered
`top_level_dir` can leak between discoveries, and `python3 scripts/tool.py`
places `scripts/`, not the repository root, at the front of `sys.path`.

**Guardrail.** Exercise a temporary non-package `tests/` directory that imports
a helper from its repository root. The canonical runner inserts the resolved
repository root for the complete run, uses a fresh `unittest.TestLoader` per
discovery, and removes the transient test-directory path added by discovery.
Run that exact entry point, not only its `run_suite()` unit helper, before
claiming the gate is usable. The same runner fails closed if any discovered
contract is skipped.

**Recurrence evidence.** The Node-journal launch test initially imported a
sibling support module as though its source directory were guaranteed on
`sys.path`. CTest actually launched it with the package build directory as the
working directory, so collection failed before the Node ran. The test now
resolves the helper from `Path(__file__)` and loads that exact file explicitly;
the generated-metadata contract continues to verify the real build working
directory so a source-tree-only fixture cannot hide the dependency.

## PIT-0014: A fixed ROS domain is not concurrent test isolation

**Symptom.** A launch test passes alone but intermittently discovers another
test's nodes, services, or topics under concurrent CI, another worktree, or a
repeated local run. Giving every test the same apparently unused
`ROS_DOMAIN_ID` only changes which runs can collide.

**Cause.** A DDS domain ID is a shared discovery namespace, not an ownership
lease. A fixed value cannot be unique across independently scheduled
processes. ROS 2's `run_test_isolated.py` provides a cooperative isolation
boundary for test processes that use the generated runner contract; it is not
effective if CMake or inherited environment state replaces its selected
domain. Inspecting only the source `CMakeLists.txt` is insufficient because
macros and later test properties determine the final CTest command.

**Guardrail.** Register each critical launch test with the official isolated
runner, clear inherited `ROS_DOMAIN_ID` and `DISABLE_ROS_ISOLATION`, and keep
the expected local discovery and RMW environment explicit. After configure,
treat `ctest --show-only=json-v1` as the final evidence: verify the exact test
inventory, source-test path, runner path, environment modifications,
serialization, and a closed allowlist of CTest properties; keep the bounded
timeout in the matching source registration contract. This also
rejects result-semantic overrides such as `WILL_FAIL`,
`PASS_REGULAR_EXPRESSION`, and `SKIP_REGULAR_EXPRESSION` that source-level
checks could otherwise miss.

## PIT-0015: One composite cleanup is one failure boundary

**Symptom.** The active assertion fails, the first teardown operation raises,
and zero publication, structured Gazebo stop, thread join, or ROS fixture
destruction never runs. The opposite failure mode is a broad `except` that
continues teardown but hides the cleanup error from the test result.

**Cause.** `unittest` continues through separately registered cleanups in LIFO
order, but it cannot resume halfway through one composite callback after that
callback raises. A helper that performs several destructive phases in one
ordinary sequence therefore short-circuits the remaining phases; a helper
that swallows exceptions makes incomplete cleanup look successful.

**Guardrail.** Register independent cleanup phases as soon as their resources
can exist, in reverse of the required execution order: the LIFO run should
first publish zero or inhibit, then request and await structured server stop,
then destroy the ROS fixture. When one phase itself owns multiple independent
resources, use an exhaustive aggregator that attempts every callback, retains
each original exception, and raises one `ExceptionGroup` afterward. Failure to
annotate one exception must not skip later callbacks. Do not replace
`doCleanups()`, clear `_cleanups`, or use a composite wrapper to bypass these
independent failure boundaries. See
[VN-0010-C2](../../docs/work-items/0010-corrective-gazebo-teardown.md).

## PIT-0016: Green status is not proof that the intended tests ran

**Symptom.** The test command returns zero and the summary is green, but a new
contract was not discovered, a critical launch test was marked skipped, an
exit code was inverted or ignored by CTest, or an xUnit file describes fewer
tests than the source and generated registration require.

**Cause.** Discovery, CTest execution, and xUnit reporting are separate
inventories with different extension and skip mechanisms. Any one layer can
look internally consistent while omitting work from another layer. Examples
include a module-level `def test_*` ignored by a `unittest` runner, a
`load_tests` hook that replaces discovery, CTest pass/skip regex properties,
and an aggregate skipped count with no matching allowed testcase.

**Guardrail.** Close the evidence chain rather than trusting one summary:

1. Build a source inventory for every repository `test_*.py`, reject
   unsupported test shapes and collection hooks, and require discovered IDs
   and the executed count to match that inventory exactly.
2. Inspect generated CTest JSON after configure and require the exact launch
   tests, official command, isolation environment, and closed property set.
3. Validate each critical launch xUnit artifact structurally: all required
   testcase identities, named non-duplicate cases, counts consistent with the
   actual elements and suite aggregates, and zero failure, error, or skip
   elements.
4. Permit skips only through an exact allowlist of artifact path and testcase
   classname. The current exception is the package's own
   `test_results/<package>/cppcheck.xunit.xml` with classname
   `<package>.cppcheck`; a global skipped total is never an exemption.
   A generated Python lint test with a stale `pytest.mark.skip` is fixed and
   enabled in source; it is never added to this allowlist.
5. When a static contract requires an active C++ GTest, normalize first, then
   parse paired same-length logical views that mask comments plus ordinary,
   character, and raw string literals. For a dedicated contract source, make
   the whole-file rule explicit: conditional compilation is forbidden. Apply
   C++ translation phase-2 line-splice normalization before *all* lexical
   classification, not only before directive inspection; a splice can also
   form a comment or join
   `GTEST_SKIP`. Reject skip macros, every early return, and conditional or
   looping control flow in the required linear tests. Prove an ordered
   top-level sequence from boundary-value setup through the tested call,
   expected rejection state, and product-derived assertion. Unreachable or
   reordered statements, ternary decoys, scattered tokens, or a string
   containing a decoy `TEST(...)` are not execution evidence. Where an input
   or derived predicate could be rebound between those statements, remove the
   mutable seam: construct the boundary value directly in the product call
   and assert a direct pure expression of the product result.

**Checker threat boundary.** These guards prevent ordinary and accidental
source, CMake, discovery, and report regressions in a cooperative repository.
They are correctness checks, not a security sandbox. They do not claim to
resist a malicious same-UID process or deliberately hostile Python dynamic
metaprogramming that mutates files, imported objects, or runtime test IDs.

**Recurrence evidence.** During the VN-0011A Attached loop, guessed ament test
names returned `No tests found` with exit zero. `ctest -N` exposed the actual
registration, and a literal
`ctest -R '^gate_event_journal_cross_process_test$'` run supplied the real
evidence. The zero-test command was discarded rather than reported as a GREEN
gate.

## PIT-0017: A Gazebo query timeout is not a teardown diagnosis

**Symptom.** During repeated WSL launch tests, the general
`gz model -m voice_nav_robot -p` query occasionally reaches its five-second
timeout. The server may still be publishing the robot pose, and the later
structured shutdown may still complete cleanly.

**Supported diagnosis and boundary.** The observed failure is instability in
the generic model query path under repetition, not evidence by itself that the
launch-managed Gazebo process died or failed teardown. That command performs
more discovery and model-query work than the assertion needs. The exact
upstream transport cause remains unproven. Classify a timeout raised during
the active test separately from a post-shutdown process exit failure; only the
latter belongs to the PIT-0012 teardown oracle.

**Guardrail.** Read the narrowest authoritative stream directly from the
exact isolated `GZ_PARTITION`: query
`/world/voice_nav_test_world/pose/info` with `gz topic --echo`, `--num 1`, and
`--json-output`. Use a 10-second subprocess bound and at most one read-only
retry. In observed high-rate runs the CLI could return two complete JSON
documents despite `--num 1`, so the parser accepts no more than four adjacent
complete documents, validates every document, and uses the newest. Reject a
wrong partition, malformed or trailing data, missing/duplicate model, invalid
or non-finite quaternion, and non-finite XYZ/RPY. This removes the unneeded
general model-query seam without turning evidence collection into an unbounded
or permissive operation. Keep structured server stop, launch-managed process
completion, and strict exit codes as separate teardown evidence. See
[PIT-0012](#pit-0012-no-residual-gazebo-process-is-not-a-clean-gazebo-exit).

## PIT-0018: A trailing diagnostic can erase a failed gate status

**Symptom.** CTest clearly prints `The following tests FAILED`, but the outer
PowerShell/WSL command reports exit code zero. Automation could therefore
classify a real failure as success even though the test artifact is red.

**Cause and boundary.** A shell pipeline or command list normally returns the
status of its last command. Adding a successful `ps` process snapshot after
CTest can replace CTest's non-zero status. Attempting to preserve `$?` inside
nested PowerShell, `wsl.exe`, and `bash -lc` quoting adds a second independent
failure seam if the variable is expanded or lost in the outer shell. This is
an evidence-wrapper defect, not a CTest or product defect.

**Guardrail.** Make the gate the terminal command whose status the caller
consumes. Run read-only process snapshots in a separate invocation after the
gate returns, and record the two results independently. If a repository-owned
script must combine them, test that script with a deliberately failing child
and require the original non-zero status; do not improvise cross-shell `$?`
capture in an acceptance command. Canonical `scripts/verify.sh` and the repeat
commands follow this terminal-gate rule.

**Recurrence evidence.** The first Layer-2 RED attempt combined a broken
cross-shell temporary-directory expression with trailing diagnostics. The
permission failure appeared in output, but the successful final diagnostic
made the wrapper return zero. The corrected invocation used a fixed owned
`/tmp` path and ended with the exact CTest, which then returned the genuine 1/1
missing-probe RED. The wrapper incident remains evidence-tooling failure, not a
product or TDD result.

## PIT-0019: A property-name allowlist does not validate property values

**Symptom.** Generated CTest metadata contains all expected property names, so
the checker passes even though `LABELS`, `TIMEOUT`, or `WORKING_DIRECTORY` has
the wrong value. A changed label can escape a label-selected run, an inflated
timeout weakens a bounded test, and a different directory changes relative
resource resolution.

**Cause.** Comparing only the set of property names constrains vocabulary, not
semantics. A later CMake property assignment can replace a correct source
registration, which is why source text is not final generated evidence.

**Guardrail.** Validate `ctest --show-only=json-v1` against exact generated
values: the single `launch_test` label, the reviewed per-test timeout, and the
resolved `build/<package>` working directory, alongside the existing runner,
source, environment, serialization, and closed property set. Required mutation
tests independently replace `LABELS`, `TIMEOUT`, and `WORKING_DIRECTORY` and
must prove that every replacement is rejected.

**Recurrence evidence.** When the mission package gained a second launch test,
a fixture regex using `.*?` crossed a closing CMake parenthesis and associated
an unrelated cross-process filename with a later `set_tests_properties()`
call. Restricting the captured target list to `[^)]*?` restored the actual
single-call boundary. The exact generated inventory remains the authoritative
oracle; source regex is only a focused structural guard.

## PIT-0020: Unit-quaternion formulas require a unit quaternion

**Symptom.** A finite Gazebo quaternion has every component multiplied by the
same scale, but the derived roll, pitch, or yaw changes. Ground-truth movement
can then appear larger or differently directed and make a movement assertion
false-green.

**Cause.** Scaled non-zero quaternions represent the same rotation only after
normalization. The usual RPY equations contain constant terms derived for a
unit quaternion, so checking a finite valid norm and then applying those
equations to the unnormalized components is not scale-invariant.

**Guardrail.** Parse finite components, require a finite valid norm, divide all
four components by that norm, and only then compute RPY. Keep zero, too-small,
and non-finite norms fail-closed. A required regression supplies a deliberately
scaled quaternion for a known rotation and proves it yields the same RPY as
the equivalent unit quaternion.

## PIT-0021: A size bound is not a schema-completeness guarantee

**Symptom.** `detail.size() <= 160` passes, but a long wrong node name,
namespace, or topic type consumes the prefix and whole-message truncation
removes later `q=`, `g=`, or `ms=` fields. Rejection remains fail-closed, but
the diagnostic no longer satisfies the required observation record.

**Cause.** Resizing the complete concatenated string proves only an upper byte
bound. It makes fields near the tail conditional on earlier untrusted values.
Adding another terminal prefix and truncating again can also erase fields from
a previously complete record.

**Guardrail.** Reserve space for the fixed `n=1`, `k=`, `t=`, `id=`, `q=`,
`g=`, and `ms=` schema, then compact each variable field independently before
composition, using a stable digest for omitted bytes. Preserve the stored
terminal record instead of wrapping and truncating it again. Required unit and
static/mutation regressions use overlong node name, namespace, and type values
and assert both the 160-character bound and every mandatory field marker.

See
[VN-0010-C1](../../docs/work-items/0010-corrective-writer-identity-convergence.md).

## PIT-0022: Test-result evidence requires one shared-tree writer

**Symptom.** Canonical verification passes repository tests, dependency
checks, model validation, and build, then fails closed with `result path
changed after evidence collection` for an xUnit file. Rerunning immediately
without identifying the writer would hide whether the evidence boundary or a
product test failed.

**Confirmed cause and discriminator.** During the C1 closure gate, an
independent documentation reviewer ran `ctest` against the same
`build/voice_nav_mission` tree while `scripts/verify.sh` was collecting result
identities. The xUnit modification time fell inside that overlap, and the
reviewer confirmed the command. No product assertion failed; the evidence
snapshot correctly detected a concurrent writer.

**Guardrail.** Treat the workspace `build/**/test_results` tree as a
single-writer resource. From the start of canonical verification until its
terminal status returns, reviewers and parallel agents perform read-only
inspection only. A necessary concurrent test uses its own build, install, and
log bases; otherwise it waits. Before retrying this failure, identify and stop
the writer, prove the tree is quiescent, and rerun the complete gate. Never
weaken file-identity validation or delete the reported artifact to manufacture
a pass. The existing anchored snapshot is the automated fail-closed guard;
the process rule prevents avoidable collisions.

See
[the testing strategy](../../docs/process/testing-strategy.md#shared-test-result-ownership)
and
[VN-0010-C1](../../docs/work-items/0010-corrective-writer-identity-convergence.md).

## PIT-0023: An immutable course solution tag cannot absorb later errata

**Symptom.** A lesson continues to document post-release corrections, but its
published `course/NNNN-solution` tag still points to the earlier reviewed
snapshot. A learner follows the current lesson and then finds that the promised
solution comparison lacks the corrected files or behavior.

**Cause.** Course prose on `main` can evolve, while an annotated solution tag is
an immutable delivery identity. Treating the moving document and frozen tree as
if they always contained the same scope either makes the exercise misleading or
creates pressure to rewrite a published tag.

**Guardrail.** Never rewrite the old tag. Mark later material explicitly as
post-tag errata, link its corrective Work Item, and state which original steps
remain comparable with the frozen solution. Publish the next cumulative
corrected baseline under the next course start tag only after that tag actually
exists. Until then, use the exact correction commits recorded in the Work Item;
do not call moving `main` an immutable answer. Repository review must compare
the current lesson tree with its advertised solution tag whenever a correction
is appended after publication.

Lesson 0009 applies this rule to
[VN-0010-C1](../../docs/work-items/0010-corrective-writer-identity-convergence.md)
and
[VN-0010-C2](../../docs/work-items/0010-corrective-gazebo-teardown.md).

## PIT-0024: An artifact cannot contain its own future identity

**Symptom.** A course or release PR cannot satisfy its closure checklist until
the PR is merged and a tag is created, so another documentation-only PR is
opened to copy the resulting public commit and tag object back into the tree.
That second PR then has its own future rebase identity, creating a recursive
closure cycle.

**Cause.** A Git tree contributes to the hashes of the commit and annotated tag
that will identify it. The tree cannot already contain identities that exist
only after review, hosted CI, rebase merge, or tag publication. Treating those
future values as required in-tree evidence confuses implementation acceptance
with artifact publication.

**Guardrail.** Publish the course start tag from public `main` before creating
the feature branch and record that existing identity in the Work Item. Use
`Refs #NN` while exact-final-head or post-merge work remains. The reviewed tree
records only facts that already exist; after the final commit, record the
exact-head gate externally. After merge, verify the public tree, create the
immutable solution/release artifact, record its exact identities in the PR and
Issue closure comment, and then close the Issue. Never open a recursive ledger
PR only to make a target tree claim its own future identity. The durable rule
lives in [the change lifecycle](../../docs/process/change-lifecycle.md#不可自引用的交付身份),
the [Work Item template](../../docs/work-items/TEMPLATE.md), and the
[PR template](../../.github/PULL_REQUEST_TEMPLATE.md); PR
[#13](https://github.com/Edddddddddy/voice_nav_robot_ws/pull/13) is the bounded
incident that exposed the cycle.

## PIT-0025: Periodic output time is not command-age origin

**Symptom.** A proposed `diff_drive_controller` crash-stop metric measures from
the last non-zero `/cmd_vel_out` sample to the first zero sample and therefore
expects roughly 0.35 s, even though the topic is stamped and republished on
every controller update. In practice that delta is about one update period.

**Cause.** The pinned Jazzy controller computes age from the stored input
`TwistStamped.header.stamp`, but stamps `/cmd_vel_out` with the current update
time. Output-to-output delta therefore measures the zero transition period,
not how old the last input was.

**Diagnostic and planned guardrail.** Reserve one marker not previously used in
the generation for the final Gate-kill attempt. The parent-owned Gate event
journal must show exactly one COMMITTED publish of that marker, and matching
non-zero `/cmd_vel_out` must ACK it before MotionGate's next 20 ms periodic
publish. Exact SIGKILL follows immediately; a second publish invalidates and
retries the generation. Use that one input stamp as the start and the first
controller zero stamp as the timeout end. Wheel writes are a separate
lossless-ledger assertion. Mutations must fail if the origin is rebound to
periodic output time, an older/late ACK, a repeated marker, or a side
observer's last receipt. This is
specified in [VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md)
and [Lesson 0010](../lessons/0010-prove-crash-stop-and-safe-pause.md). The
[pinned controller source](https://github.com/ros-controls/ros2_controllers/blob/jazzy/diff_drive_controller/src/diff_drive_controller.cpp)
is the primary semantic reference. Change this entry to `Guarded` only after
the VN-0011A executable contract lands.

## PIT-0026: Lossy introspection cannot prove exact or continuous writes

**Symptom.** A pause-resume test observes a zero wheel command on
`/controller_manager/introspection_data/full` and claims either that the first
resumed hardware write was zero or that wheel commands never regressed during
a final window, although an earlier/intermediate non-zero sample could have
been dropped.

**Cause.** ros2_control's full introspection stream is asynchronous,
`BEST_EFFORT`, and `KEEP_LAST(1)`. It is mandatory sampled corroboration, but
delivery preserves neither an exact first-sample boundary nor every
intermediate write, so it cannot prove a universal no-regression claim.
Furthermore, a default reliable rclpy subscription is incompatible and may
receive nothing, while a retained initial zero can create a false positive if
the fault is not armed by a non-zero baseline.

**Diagnostic and planned guardrail.** VN-0011A uses compatible QoS, waits for
discovery, and requires complete finite, strictly increasing, non-zero
pre-fault samples, but a default-off lossless ledger at the actual hardware
write seam proves first both-wheel zero and no later regression. VN-0011B uses
the same journal across its controller-update and post-update-write probe;
`/joint_states` and odometry remain separate physical evidence. See
[Jazzy ros2_control introspection](https://control.ros.org/jazzy/doc/ros2_control/doc/introspection.html),
[VN-0011](../../docs/work-items/0011-crash-stop-and-safe-pause.md), and
[ADR-0005](../../docs/adr/0005-use-tokenized-managed-safe-pause.md). Change this
entry to `Guarded` only after the corresponding executable contracts land.

## PIT-0027: A stale baseline is not a fault-arming barrier

**Symptom.** A process-death test observes a live/non-zero sample, waits long
enough for a lease or freshness deadline to expire naturally, then sends
SIGKILL and credits the already-occurring zero to the crash path.

**Cause.** Independent booleans and command samples collected at unrelated
times do not establish one causal pre-fault state. Without a bounded barrier
and event ordering, `ProcessExited` need not precede the terminal zero being
measured.

**Diagnostic and planned guardrail.** VN-0011A uses a <=40 ms steady-clock
barrier for both Gate validity flags and a recent non-zero Gate commit; the
final Gate receipt is <=20 ms old at signal dispatch. Marker uniqueness is an
additional narrow requirement only for the MotionGate-death case. Independently, strictly
advancing non-zero simulation surfaces are <=30 ms old in simulation time.
No invalid/zero event intervenes. A parent-owned Gate event journal uses the
same host's monotonic clock to prove the terminal transition-linearization and
bound-zero pre-publish fences do not precede exact `ProcessExited`; later
commits prove completion, while DDS receipt order is not used as causal proof.
The matching state keeps the Gate instance but intentionally
clears the retired lease, matches the journaled terminal `control_seq`, and
advances `zero_publish_seq == output_publish_seq`; requiring the old lease or
accepting a stale zero are both errors. Stale-baseline, delayed-kill,
cross-topic-order, and zero-first mutations must fail. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0028: A side subscriber is not a consumer-acceptance oracle

**Symptom.** A MotionGate crash test starts the controller timeout from the
last input seen by an observer, even though the controller's separate reader
and callback may have consumed a different final sample.

**Cause.** Reliable DDS delivery and publisher-count convergence do not make
two independent readers advance or execute callbacks in lockstep. A 100 ms
quiet period can drain the observer while still saying nothing exact about the
controller callback's accepted command.

**Diagnostic and planned guardrail.** Emit one marker not previously used in
the generation and record every periodic source publish in a crash-resilient
INTENT/COMMITTED journal. Only its first COMMITTED record plus a matching
non-zero controller-output ACK observed before the next 20 ms Gate publish can
supply the timeout origin. Exact SIGKILL follows immediately. A trailing
intent, second publish, late ACK, limiter change, gap, or overflow invalidates
and retries the generation. Publisher disappearance and 100 ms quiet remain
cleanup only.
Mutations must make the side-observer shortcut fail. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0029: A single paused step may not cross a controller update

**Symptom.** A test advances one paused Gazebo iteration, observes an old zero
hardware write, and declares resume safe although the 100 Hz controller has
not yet consumed a buffered command.

**Cause.** Pinned `gz_ros2_control` writes in every `PreUpdate`, but performs
controller read/update in `PostUpdate` only when the control period is due.
With the default 1 ms simulation step and 10 ms controller period, one step
usually writes pre-pause state and does not test the post-update command.
Also, Gazebo processes the `pause` field before `multi_step`; protobuf's
default false can turn a request lacking `pause:true` into continuous unpause.

**Diagnostic and planned guardrail.** Keep continuous run disabled and send
only exact `{pause:true,multi_step:1}` requests. After every queued ACK, World
Statistics must prove one step and re-pause. Step within the computed bound
until same-stamp controller output and introspection prove a new zero update;
then step once more and require the entire lossless interval plus post-update
write to stay zero. Only then send continuous `pause:false`. Omitted/false
pause, duplicate request, non-zero/missing update, gap, or wrong ordering must
fail. See
[ADR-0005](../../docs/adr/0005-use-tokenized-managed-safe-pause.md).

## PIT-0030: A cross-clock arming window smuggles in real-time factor

**Symptom.** A correct crash test fails on a slow WSL/CI runner because it
requires a 50 Hz simulation-time controller publication to arrive inside a
40 ms wall/steady interval.

**Cause.** Gate lease/output deadlines use steady time, while controller,
wheel, and odometry samples advance in simulation time. Constraining both with
one steady deadline silently assumes a minimum Gazebo real-time factor.

**Diagnostic and planned guardrail.** Keep the <=40/20 ms bounds only on Gate
steady-clock state/output freshness. Separately require `/clock` to advance and
simulation surfaces to be non-zero, monotonic, and <=30 ms old in simulation
time; wall time is only the bounded outer watchdog. Mutate low RTF without
weakening either domain's own freshness rule. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0031: “Lossless” needs a completeness protocol

**Symptom.** A hardware-write ring or reliable topic reports an apparently
continuous zero window even though it overwrote, dropped, reordered, or mixed
records from another generation.

**Cause.** Transport labels do not prove a complete interval. Without sequence
fences and capacity/overflow semantics, absence of a record is indistinguishable
from absence of a write. Capacity based only on advancing iterations is also
invalid: a paused runner can invoke the write seam repeatedly without advancing
iteration.

**Diagnostic and planned guardrail.** The write seam owns a non-wrapping
`uint64 write_seq`; atomic ARM/SEAL fences close the analyzed interval. Every
invocation advances the sequence. Only consecutive calls with identical
generation, simulation stamp, delegated return result, and exact command bits
may fold into a segment; its first/last sequence and invocation count must
agree. Prove segment capacity from the bounded write-invocation/transition
budget before arming, latch overflow/overwrite, unaccounted calls, and
zero-window nonzero writes as failure, retain sealed segments until
acknowledgement, and validate immutable bounded pages by checksum plus
contiguous generation/sequence ranges. Simulation stamp may repeat during a
paused runner without consuming one slot per identical write. The hardware
Interface has no iteration field; World Statistics independently proves
continuous progress in A and exact `N -> N+1` single steps in B, correlated
through ARM/SEAL. DDS BEST_EFFORT and overwrite-on-full are forbidden for the
proof channel. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0032: Heartbeats make an arming-snapshot `control_seq + 1` false

**Symptom.** Candidate-death evidence rejects a correct terminal state because
its `control_seq` is greater than `armed_control_seq + 1`, or the test stops
authority RENEWs to make that assertion pass.

**Cause.** Every accepted RENEW advances the Gate-wide compare-and-swap
sequence. The terminal retirement advances it once more. RENEWs deliberately
continue during candidate loss, and a request already in flight may also be
accepted around process exit, so the arming snapshot is not the terminal
transition's immediate predecessor.

**Diagnostic and planned guardrail.** The parent-owned Gate event journal
records every applied same-generation control transition with its before/after
sequence. The terminal retirement must be exactly one non-wrapping step after
the final committed predecessor, and the received terminal state must match
that journal record. Missing an intervening RENEW, freezing authority traffic,
or comparing directly with the arming snapshot must fail. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0033: DDS receipt order is not cross-process event order

**Symptom.** The observer receives `ProcessExited` and then a zero/state sample,
so the test credits the zero to process death even though MotionGate published
it earlier and DDS delivered it late.

**Cause.** Reliable and transient-local QoS preserve a writer's stream but do
not establish causal order between a launch event and separate DDS topics.
Observer receipt timestamps can therefore invert the actual Gate commit order.

**Diagnostic and planned guardrail.** Timestamp exact `ProcessExited` receipt,
the Gate transition linearization fence, and the zero output's pre-publish
`INTENT` with Linux `CLOCK_MONOTONIC` on the same host. The crash-resilient Gate
event journal binds those fences to the retirement, resulting output sequence,
and later zero `COMMITTED` record. Both pre-operation fences must be no earlier
than the process-exit timestamp; a later commit alone cannot prove that order.
DDS state receipt remains the latency endpoint, not the ordering oracle.
Pre-death transition/output intent with post-death receipt and receipt-only
mutations must fail. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0034: Dynamic `exec_module` may not satisfy decorator assumptions

**Symptom.** A support file parses and is found, but pytest fails during
collection inside `dataclasses._is_type` with `sys.modules.get(...)=None`.

**Cause.** `importlib.util.module_from_spec()` plus
`spec.loader.exec_module(module)` does not itself insert the temporary module
into `sys.modules`. The `dataclass` decorator consults that registry while
resolving annotations, so a loader pattern that works for plain classes can
fail only after a decorator is added.

**Diagnostic and guardrail.** Either register the module under the exact spec
name before execution with failure-safe cleanup, or keep dynamically loaded
support primitives independent of registration-sensitive decorators. The
VN-0011A `CrashLedger` uses a small slotted internal record and its package
pytest loads the file through the same dynamic path used by launch support.
Collection failure is not accepted as tests-first RED or GREEN; the executable
test body must run. See
[Lesson 0010 evidence](../records/0010-crash-stop-and-safe-pause.md).

## PIT-0035: A concrete PImpl plugin class may be a false extension seam

**Symptom.** A test-only class derives from
`gz_ros2_control::GazeboSimSystem` because its methods are virtual, but a
minimal Jazzy compile fails in `std::default_delete` with an invalid `sizeof`
of incomplete `GazeboSimSystemPrivate`.

**Cause.** The installed concrete class owns a
`std::unique_ptr<GazeboSimSystemPrivate>`, forward-declares that PImpl in the
public header, and does not declare an out-of-line destructor. Instantiating an
external derived destructor therefore requires a private type that is still
incomplete. Virtual methods do not by themselves make a concrete PImpl class a
supported inheritance boundary. The installed plugin XML names
`GazeboSimSystemInterface` as the public base class.

**Diagnostic and planned guardrail.** Compile the smallest derived object
against the exact installed release before designing the Adapter. Implement
the test plugin by inheriting `GazeboSimSystemInterface`, owning a pluginlib
loader and upstream Interface instance, and delegating lifecycle, interface
export, mode switching, `read()`, `write()`, and `initSim()` unchanged. The
test package must directly discover `gz_sim_vendor` and `gz-sim8` before
consuming `gz_ros2_control`'s exported targets. A compile/static contract will
reject the concrete subclass and verify the plugin XML base type, while product
URDF continues selecting the upstream plugin directly. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md) and the
[Lesson 0010 occurrence](../records/0010-crash-stop-and-safe-pause.md#architecture-correction-use-the-public-hardware-extension-seam).

## PIT-0036: An evidence seam cannot record a field it never receives

**Symptom.** A hardware-write record claims an exact Gazebo iteration for each
`write()` call, even though the Adapter only receives ROS simulation `time` and
`period`.

**Cause.** `GazeboSimSystemInterface::write()` has no
`UpdateInfo.iterations` argument, and the ECM exposed at `initSim()` has no
world-iteration component. The real iteration is available to the outer Gazebo
System's `PreUpdate`, not to this public hardware plugin seam. Copying a nearby
World Statistics value into every write record would manufacture precision and
could misorder paused runner calls.

**Diagnostic and planned guardrail.** The hardware ledger records only facts it
owns: non-wrapping `write_seq`, generation, `sim_stamp`, delegated result, and
exact wheel-command bits. World Statistics independently records real
iteration, simulation time, and paused state. ARM/SEAL fences plus exact
single-step request/commit evidence correlate the streams. Static and mutation
tests must reject a per-write iteration field or an assertion that stamp must
advance on every paused write. See
[ADR-0005](../../docs/adr/0005-use-tokenized-managed-safe-pause.md).

## PIT-0037: Periodic source repeats make value-only ACKs ambiguous

**Symptom.** A non-zero `/cmd_vel_out` tuple matches the Gate journal's final
value, so the test assumes the controller accepted one exact Gate publish even
though MotionGate republishes the same selected tuple every 20 ms.

**Cause.** The pinned controller stamps `/cmd_vel_out` with its update time and
does not echo the input header stamp. A value reused by multiple Gate publishes
cannot identify which input the controller consumed. Publisher disappearance
and quiet time also do not repair that ambiguity.

**Diagnostic and planned guardrail.** Reserve a marker never previously used
in the generation. Arm only on its first journal COMMIT, require a matching
non-zero controller output, and dispatch exact Gate SIGKILL before the next
20 ms source publish. After `ProcessExited`, the journal must still end at that
one COMMITTED record. If a second publish, trailing intent, or late ACK occurs,
discard and retry the whole generation. Producer-death tests that do not use
the controller timeout need only a recent non-zero commit; they must not inherit
this narrow uniqueness rule. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0038: Instrumentation real-time claims do not cover delegated code

**Symptom.** A design says the “hardware-write seam performs no allocation,”
then treats any allocation inside pinned upstream
`GazeboSimSystem::write()` as a product defect or silently copies the upstream
implementation to make the claim true.

**Cause.** The test owns only its added journal operations. The delegated
implementation may create or assign Gazebo components and is outside the
instrumentation's real-time contract. An unqualified seam-level statement
expands ownership beyond what the project can enforce.

**Diagnostic and planned guardrail.** State the invariant narrowly: after the
delegated call, added journal work uses preallocated fixed storage, bounded
lock-free atomic operations, and no filesystem, logging, ROS, or transport
calls. Preserve upstream behavior through delegation rather than copying it.
Static wording tests and implementation review must reject broader claims.
See [the motion safety contract](../../docs/architecture/safety-and-motion-contract.md).

## PIT-0039: Post-hoc commit time is not transition linearization time

**Symptom.** Crash evidence compares `ProcessExited` with a timestamp written
after reading the new Gate snapshot and concludes that expiry happened after
the process died, even though the state mutation may have occurred earlier and
the executor was descheduled before the journal commit.

**Cause.** A post-transition snapshot and its durability commit bound when the
observer recorded the outcome, not when the state and `control_seq` changed.
Preemption between mutation and timestamping can manufacture the desired
causal order.

**Diagnostic and planned guardrail.** Give MotionGate Core one transition
wrapper. It appends `INTENT`, samples `CLOCK_MONOTONIC` immediately before a
bounded non-blocking mutation, records that sample as the explicit
`transition_linearization_ns`, captures the after-image, and only then marks
the slot `COMMITTED`. Crash analysis compares exact `ProcessExited` with the
linearization field; commit time remains a durability fact. A fake-clock unit
test must fail if an implementation substitutes the later commit time. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0040: Scattered callback journals create partial evidence

**Symptom.** PREPARE and service-driven INHIBIT are journaled, but automatic
expiry, invalid-candidate retirement, writer mismatch, or sequence-exhaustion
fault has no record or produces duplicate records.

**Cause.** Node callbacks are transport adapters, not the owner of Gate state
transitions. The same Core mutation can be reached from a service, timer,
candidate callback, or hidden helper. Adding before/after calls to each ROS
handler creates a shallow protocol whose completeness depends on every caller.

**Diagnostic and planned guardrail.** Journal transitions once at a private
Core-owned `apply_control_transition` seam covering PREPARE, OPEN, RENEW,
INHIBIT, automatic retirement, invalid input, fault, and sequence exhaustion.
The Node owns only the final DDS output seam. Coverage/mutation tests enumerate
every transition kind and reject both missing and duplicate journal sequence
values. Do not extend `reconcile_adapter_transition()` into a recorder; it runs
after mutation and only owns subscription cleanup. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0041: A checksum implementation cannot be its own oracle

**Symptom.** A checksum test writes a record, compares its stored checksum with
the same production helper, and passes even after a required field is removed
from both calls. Non-zero output proves neither the selected polynomial nor the
ABI coverage domain.

**Cause.** Producer and verifier share one implementation and therefore share
the same mistake. Size/offset assertions protect layout but do not protect the
algorithm, field order, byte order, or the intended mutable-field exclusions.

**Guardrail.** Specify the checksum independently in the Work Item. Lock at
least one externally calculated constant for each record phase, then mutate
every included field and every excluded field individually. Included mutations
must change the checksum; mutable phase/claim/commit fields excluded from that
phase must not. Compile the shared ABI header through a real C11 target as well
as C++. For Gate journal ABI v1 the fixed oracle is CRC64-ECMA-182, non-
reflected, init/xorout zero, feeding each `uint64_t` least-significant byte
first. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0042: Mounted-filesystem clock skew needs a bounded rerun

**Symptom.** GNU Make completes a target but warns that a dependency file has a
modification time a few milliseconds in the future and that the build may be
incomplete.

**Cause.** WSL and the Windows-mounted NTFS path can expose slightly different
sub-second timestamp observations. This is distinct from a compiler failure,
but the warning also prevents treating that one invocation as final evidence.

**Guardrail.** Keep the exact target and output. Compare the WSL epoch with the
reported file epoch using no-space commands, then rerun the same incremental
target once the alleged future timestamp has passed. Accept the build only if
the rerun exits zero without the warning and the focused tests still pass. If
the warning repeats, stop and inspect host/guest clock divergence and
concurrent writers; do not use `touch`, delete build metadata, or suppress the
warning to manufacture green evidence.

**Recurrence evidence.** The first incremental build of the Layer-2 attach
probe reported several dependency files less than one second in the future.
The same focused targets rebuilt without the warning after the boundary had
passed, the cross-process CTest then passed five consecutive executions, and
the complete package gate passed 16/16. Only the warning-free focused rerun and
tests were retained as acceptance evidence.

## PIT-0043: Evidence failure must not veto a safety mutation

**Symptom.** MotionGate has an active lease, but `INHIBIT`, automatic expiry,
or `force_fault()` throws when the crash-evidence journal is full, leaving the
Gate armed because the recorder could not reserve another slot.

**Cause.** One generic transaction policy treats both authority admission and
safety termination as if journal durability were the primary outcome. For an
admission transition, refusing an unrecordable mutation is fail-closed. For a
terminal transition, refusing to select zero is the unsafe outcome.

**Guardrail.** The Core owns two explicit policies at its single transition
seam. `PREPARE`, `OPEN`, and `RENEW` reject mutation when reservation fails.
`INHIBIT`, automatic retirement, and `FAULT` execute one bounded `noexcept`
safety mutation even if reservation fails; the journal latches overflow or
corruption so that the evidence generation is rejected. Unit tests fill the
journal immediately before both PREPARE and INHIBIT and require opposite state
outcomes. Evidence may fail closed as evidence, but it may never become a
dependency for stopping motion. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0044: Object non-copyability is not resource exclusivity

**Symptom.** `MotionGateCore` is non-copyable and non-movable, yet two
independently constructed Cores can receive the same journal pointer and both
commit a `before_control_seq=0 -> after_control_seq=1` transition. A
"journal-bound" constructor can also accept null and silently run without the
promised evidence path.

**Cause.** Type traits constrain copies of one state-machine object; they say
nothing about aliasing an external resource. A reusable raw pointer or public
transition writer is still a second ownership path, and nullable dependency
injection can turn a required mode into an optional one without an error.

**Guardrail.** Each journal generation permanently issues at most one
move-only transition capability. The Core consumes that capability, its
transition method is private to the Core, a second claim fails even after the
first Core is destroyed, and the journal-bound constructor rejects an empty
capability. The separate three-argument constructor is the only explicit
no-journal mode. The sequential lifetime contract detaches the capability if
the Journal is destroyed first; production composition constructs the
Attached Journal before the Core and destroys them in reverse. Do not claim
concurrent destruction safety without a synchronized lifetime state and TSAN
evidence. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0045: Acquire must precede every ordinary shared-memory read

**Symptom.** A shared-memory attacher derives the expected generation or
capacity from ordinary Header fields and only later constructs the validator
that acquire-loads `init_state == READY`. Same-process tests pass, but a real
consumer may read unpublished or stale payload while the parent is still
initializing it.

**Cause.** A release/acquire pair orders only operations that occur after the
consumer's acquire. Reading ordinary payload first cannot be repaired by a
later acquire, and deriving "expected" identity from the untrusted object also
turns comparison into self-validation.

**Guardrail.** The parent supplies the complete expected UID, generation,
128-bit nonce, capacity, and therefore exact byte size out of band. The
attacher validates only configuration and fd metadata before `mmap`, then
passes the mapping and parent expectations directly to `GateEventJournal`.
That constructor acquire-loads `READY` before reading any ordinary Header or
slot field and claims `writer_pid` only after every validation succeeds.
Wrong generation, nonce, capacity, mode, name, and size must fail before the
claim. Same-process tests protect ordering in code review; a real parent/child
probe is now the permanent Layer-2 guard: the parent directly initializes and
release-publishes `READY`, the separately executed child claims its real PID,
the parent unlinks the name, and the child still commits through its mapping;
after child exit the parent acquire-loads `COMMITTED` and validates all CRCs
with an independent implementation. Keep that test registered; do not regress
the contract back to same-process evidence. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0046: Checksum coverage is not producer population

**Symptom.** ABI v1 and its checksum include output `before_state_seq`,
`before_control_seq`, and before-lease words, yet before `5afbebc` every
produced output record contained zero there. The checksum suite remained green
because it proved that mutating a slot field changes the checksum, not that the
producer could supply that field.

**Cause.** Layout/checksum coverage, typed producer input, and serialization
mapping are three separate contracts. `GateOutputIntent` omitted the four
before-image fields, and `begin_output()` therefore had nothing to copy even
though the shared-memory slot already reserved and checksummed them.

**Guardrail.** Give every normative semantic field a typed producer input and
an explicit serialization assignment. Use a non-zero output fixture, assert
the complete stored before-image, and bind it to an independently calculated
CRC64 golden value. Keep C ABI offsets and cross-process zero fixtures as
separate compatibility checks. Commit `2a8f31b` preserved the compile-time RED;
`5afbebc` completed the mapping without changing the 256-byte ABI. See
[VN-0011A](../../docs/work-items/0011a-process-death-crash-stop.md).

## PIT-0047: A cached target build can stop before the intended RED

**Symptom.** After adding `motion_gate_process_runtime_test`, a targeted
`colcon build --cmake-target motion_gate_process_runtime_test` failed with
`No rule to make target`. It never compiled the test and therefore did not
prove the expected missing-header RED.

**Cause.** A target-only build can reuse an already-generated package build
tree whose CMake target graph predates the edited `CMakeLists.txt`. The failure
describes stale build metadata, not missing product behavior.

**Guardrail.** The first targeted build after adding or renaming a CMake target
must use `--cmake-force-configure`, or follow a successful package configure.
Accept a TDD RED only after output proves the intended source or assertion was
reached. Commit `5314866` was accepted only after the forced configure reached
the precise missing `motion_gate_process_runtime.hpp` include; `639e1f1` then
made that focused test green.

## PIT-0048: Mutation rejection does not prove a parameter is read-only

**Symptom.** A test sets a safety-relevant test parameter, sees an unsuccessful
result, and reports that the parameter is immutable. The Node may never have
declared that parameter at all; ROS rejects both cases.

**Cause.** The negative mutation path collapses two different contracts:
“declared and read-only” and “unknown name”. It therefore cannot prove the
parameter exists, has string type, or defaults to the disabled value.

**Guardrail.** First call `describe_parameters` and assert the exact ordered
names, `PARAMETER_STRING`, and `read_only=true`. Then call `get_parameters` and
assert both values are empty before testing mutation rejection. Commit
`12671d8` preserved the undeclared-parameter RED; `f0ed0f7` declared the two
default-off parameters and made the launch test green.

## PIT-0049: Direct symlink installation can leak a manifest into source

**Symptom.** Running `cmake --install build/voice_nav_mission` from the
repository root creates an untracked `symlink_install_manifest.txt` beside the
source tree even though all actual install links target `install/`.

**Cause.** The ament symlink-install override writes its auxiliary manifest
relative to the install command's working directory. A technically successful
install can therefore violate the clean-worktree boundary.

**Guardrail.** Prefer `colcon build --packages-select ... --symlink-install`
for package installation. If a direct CMake install is diagnostically
necessary, run it from the package build directory and immediately inspect
`git status --short`. The observed root manifest was verified as generated
install-path inventory and removed explicitly; it was never staged.

## PIT-0050: A guard can become stale while the invariant stays correct

**Symptom.** The real Core correctly renews with
`config_.authority_lease`, but `check_motion_gate_contract.py` fails because it
still searches for the older `authority_lease` spelling.

**Cause.** A source-token checker is coupled to implementation vocabulary as
well as behavior. Its synthetic “valid” fixture retained the old spelling, so
that positive control passed while the real-repository positive control failed.

**Guardrail.** After a guarded refactor, run both the synthetic valid fixture
and the checker against the actual repository, plus at least one mutation that
must still fail. Commit `b1a562a` aligned the token and fixture with
`config_.authority_lease`; the two positive controls and the
candidate-must-not-renew mutation then passed together.

**Recurrence evidence.** The VN-0011A product-isolation checker still searched
for the obsolete generic `crash_journal` marker after the governing contract had
frozen `test_gate_event_journal_name` and
`test_gate_event_journal_descriptor`. Its old mutation was green because it
injected a key the Node never declared, while complete frozen-key pairs in
both product launch and YAML were incorrectly accepted. Focused RED tests now
inject both exact keys and require both in the diagnostic; replacing the stale
marker with those two keys made all 17 non-topology crash-contract fixture
cases pass. The repository-root checker remains separately RED for the
not-yet-implemented Gazebo Adapter and was not counted as this correction's
failure.

**Second recurrence.** The final-output checker updated its synthetic Node to
the Runtime-owned Adapter and nine new focused tests passed, but one older
`use_sim_time` mutation still searched for a statement removed by that same
fixture refactor. Running the complete contract file, excluding only the
deliberately RED real-repository case, exposed the drift as 1 failure among 55
tests. The fixture was corrected and the full 55-test sibling set became the
handoff gate; a newly added focused subset alone is never enough evidence that
an edited shared fixture is valid.

## PIT-0051: Mixed-outcome launch tests need per-action exit assertions

**Symptom.** A launch test deliberately starts one valid process and malformed
siblings, correctly observes the siblings exit with code 1, but the suite still
fails during a global exit-code assertion.

**Cause.** `launch_testing.assertExitCodes(proc_info)` applies one allowable
set to every managed action. A process whose rejection is the behavior under
test is therefore indistinguishable from an unexpected process failure when
the assertion is global.

**Guardrail.** Keep a distinct launch action object for each process and call
`assertExitCodes(..., process=action, allowable_exit_codes=[...])` for every
expected outcome. Also bind stderr and PID assertions to that same action so an
unrelated process cannot satisfy the oracle. The Node-journal acceptance test
requires code 0 for the fully configured Gate and code 1 for each partial
configuration, both during the live test and again in post-shutdown evidence.

## PIT-0052: Fault recording cannot be a prerequisite for stopping

**Symptom.** A journal reservation or DDS publish fails, but the intended
direct zero is never attempted because execution exits while trying to record
the resulting Core fault.

**Cause.** Safety logic often treats `force_fault()` as an infallible state
assignment. Its detail path uses `std::string`, however, and can allocate or
otherwise throw before the bounded safety mutation. Placing it before the
fallback makes observability/state recording a hidden prerequisite for the
actual stopping attempt.

**Guardrail.** `MotionGateProcessRuntime` retires the output evidence lane
first, calls fault recording through a catch-all best-effort boundary, and then
attempts exactly one direct zero regardless of that outcome. A package-private
fault Adapter injects `std::bad_alloc`: the test proves the Core may remain
Armed with its old non-zero selection while Runtime still publishes zero and
permanently routes every later output through the direct-zero path. The
Adapter is a unit-test seam and canonical product composition must leave it
empty.

## PIT-0053: ROS distribution APIs must be proved by the target compiler

**Symptom.** The Runtime-owned output Adapter was logically correct, but the
Jazzy build failed because `rclcpp::Time` has no `to_msg()` member.

**Cause.** A familiar conversion API from another ROS type or distribution was
used from memory. Static source contracts could verify clock ownership and
field mapping, but they could not prove that the selected Jazzy C++ API exists.

**Guardrail.** Compile the smallest affected package immediately after wiring
a ROS Adapter. On Jazzy, explicitly convert with
`const builtin_interfaces::msg::Time stamp = get_clock()->now();`, then copy
its integer `sec` and `nanosec` fields; do not reconstruct the stamp through
floating-point seconds. The corrected package build, Node launch tests, and
format/static-analysis gate must all pass before the Adapter is committed.

## PIT-0054: Ament CMake fixes the link-signature style per target

**Symptom.** CMake configuration fails with `The plain signature for
target_link_libraries has already been used` after a new direct Gazebo library
is linked with `PRIVATE`.

**Cause.** On Jazzy, both `ament_target_dependencies()` and
`ament_add_gtest()` internally call the plain `target_link_libraries` form.
CMake forbids mixing that form with the keyword form (`PRIVATE`, `PUBLIC`, or
`INTERFACE`) for the same target.

**Guardrail.** Once an ament macro has wired a target, add direct imported
targets with the plain form too:

```cmake
ament_target_dependencies(target dependency)
target_link_libraries(target gz-sim8::gz-sim8)
```

Force package reconfiguration and require the real target to compile. Do not
drop direct dependencies or hide them transitively merely to avoid the style
conflict. VN-0011A first exposed this while linking both the test-only Gazebo
hardware Adapter and its GTest; using one plain form per target produced the
focused plugin-load GREEN.

## PIT-0055: A PowerShell-to-WSL script pipe can prepend a UTF-8 BOM

**Symptom.** A Bash script piped from a PowerShell here-string starts with
`bash: line 1: ﻿set: command not found`, while later build and test commands
may still run successfully.

**Cause.** The producer encoded the piped script with a UTF-8 byte-order mark.
Bash treated the mark as part of the first command name. In this case the
failed first command was `set -o pipefail`, so a later pipeline could also have
hidden its real exit status.

**Guardrail.** Do not pipe an implicitly encoded PowerShell here-string into
WSL Bash for evidence-producing commands. A direct multiline native argument
is also unsafe here: WSL/PowerShell re-quoting stripped shell variables from a
`bash -lc` script, producing redirects such as `> 2>&1` and replacing `$?`
before Bash owned it. Encode the UTF-8-no-BOM script bytes as Base64, pass only
that safe token across the Windows boundary, then decode into an inner Bash;
alternatively write an exact temporary script with `new UTF8Encoding(false)`.
Preserve and test the build or test process exit code independently. A green
tail is not evidence when shell setup or argument transport failed.

## PIT-0056: WSL wall-clock correction makes `ps lstart` an unstable identity

**Symptom.** A protected Gazebo process retained the same PID, PPID, and
command line across a package gate, but `ps -o lstart` moved forward by 21
seconds and falsely reported that the process identity changed.

**Cause.** Linux stores process start time as ticks since boot. `ps lstart`
converts those ticks into calendar time using the current boot/wall-clock
relationship. WSL clock correction can change that conversion while the
process itself remains unchanged.

**Guardrail.** Fingerprint a protected WSL process with PID, PPID, field 22
(`starttime`) from `/proc/<pid>/stat`, and `/proc/<pid>/cmdline`. Parse the stat
line after its final `)` because the parenthesized command name may contain
spaces. Compare the raw start ticks before and after the gate; use `lstart`
only for human display, never as the teardown-safety identity oracle.

## PIT-0057: A drive-letter regex can mistake a URL scheme for a path

**Symptom.** The real transformer passed its behavior tests but its static
contract rejected the standard Apache license URL as a machine-specific
Windows path. The decisive regex match was `p:/` inside `http://`.

**Cause.** The checker searched for any `[a-z]:/` substring without requiring
a token boundary before the drive letter. A URL scheme therefore contained a
syntactically matching suffix even though no machine path was present.

**Guardrail.** Machine-path checks must require a non-alphanumeric boundary
before a drive letter or Unix absolute-path marker. Test both sides of the
classifier: ordinary `http://` and `https://` source text must pass, while
quoted or commented `C:/`, `C:\\`, `/home/...`, and `/mnt/c/...` examples must
still fail. Never remove a license header to satisfy a faulty source checker.

## PIT-0058: Entry-time SEAL can exclude the write being proved

**Symptom.** A paused single-step test receives a valid SEAL receipt and an
exact World Statistics `N -> N+1`, but the sealed interval ends before the
post-controller-update hardware write. Proving that write would require a
second bookkeeping step before `pause:false`.

**Cause.** SEAL was linearized at the next hardware `write()` entry and closed
the old interval immediately. The invocation that triggered SEAL was therefore
outside `(arm_fence, seal_fence]`, even though Managed Safe Pause allows only
one final exact step to write and prove the updated zeros.

**Guardrail.** SEAL is deferred and inclusive. Its request carries
`not_before_sim_stamp`; the first qualifying invocation delegates upstream,
captures the actual command observation, appends and validates it, finalizes
the segment, then release-publishes the immutable bank and receipt. That
invocation owns `seal_fence` and remains inside the interval. VN-0011B also
requires an exact trigger stamp and independently proves the matching
World-Statistics single step; a skipped stamp, missing write, or extra step
fails closed.

## PIT-0059: WSL address randomization can make TSAN look like a product crash

**Symptom.** An unchanged ThreadSanitizer test first reports the intended data
race, then intermittently exits with `unexpected memory mapping` or a bare
segmentation fault before executing the assertion.

**Cause.** Under WSL2, a randomized executable mapping can collide with TSAN's
reserved shadow-address layout. This is a sanitizer runtime failure, not
evidence that the tested lifecycle still races.

**Guardrail.** Build the dedicated TSAN executable as non-PIE and invoke only
that test through `setarch <architecture> -R`; do not disable ASLR globally.
Keep `halt_on_error=1` and a distinct race exit code, then repeat the focused
test before accepting GREEN. The ordinary production target remains
unsanitized and retains normal platform policy.

## PIT-0060: Reusing a publication ticket does not republish mutable payload

**Symptom.** A same-ticket retry is sometimes accepted with the wrong payload,
faults only under concurrency, or produces a valid-looking response CRC that
depends on request bytes changed after Writer began consuming them.

**Cause.** Release-storing the same numeric ticket again does not establish a
new ownership handoff while ordinary request fields are still mutable. Writer
could copy those fields while Parent prepared or retried the slot. Binding the
response CRC to the live request checksum repeated the same race on the return
path.

**Guardrail.** Use the single owned-envelope state machine from
[ADR-0007](../../docs/adr/0007-own-hardware-ledger-request-publication.md).
Parent writes only while it owns WRITING; Writer reads only after claiming
READY and snapshots locally. Immediate requests release IDLE before their
receipt; deferred requests retain READING until terminal response. The response
stores and checksums the consumed request checksum. A cross-process regression
test deliberately
holds WRITING across a Writer `begin_write()` and proves there is no receipt,
bank activation, or protocol fault until Parent release-publishes READY.

## PIT-0061: Detecting corrupt geometry does not make later traversal safe

**Symptom.** Writer correctly latches a protocol fault for a segment count
larger than the fixed bank capacity, but then crashes, reads the adjacent bank,
or publishes a root checksum over bytes outside the declared evidence.

**Cause.** The structural validator reported the bad count and then reused the
normal terminalization path. That path iterated to the untrusted count while
calculating CRC, so the failure handler crossed the boundary it had just found
invalid.

**Guardrail.** Treat fixed-capacity geometry as a prerequisite for every loop,
not merely as a fault bit. Before page-count or CRC calculation, revalidate the
segment count and page limit against the mapped capacity. On failure, latch
PROTOCOL and publish neither a readable sealed bank nor a receipt. The
cross-process mutation test sets the count to capacity plus one and requires
the exact Writer child to remain alive with the bank ACTIVE and faulted.

## PIT-0062: Attempted writes and recordable segments are different evidence

**Symptom.** A missing wheel component consumes a legitimate write sequence
but is serialized as if exact wheel bits existed. Alternatively, omitting that
fake segment makes the next valid tuple trigger a derived `PROTOCOL` fault, or
a corrupted clean bank with fewer stored invocations is sealed `SEALED_OK`.

**Cause.** One set of first/last/count fields was made to describe both the
complete attempted interval and the subset for which an exact command tuple
can be recorded. Requiring the last segment to end at the latest attempt also
forbade legitimate gaps in a bank that was already faulted.

**Guardrail.** Bank metadata counts every contiguous included attempt.
Segments contain only otherwise recordable tuples and use strictly ordered,
non-overlapping ranges; folding requires adjacency and therefore never crosses
a gap. A sticky fault permits recorded counts to be smaller than attempted
counts, while a fault-free bank requires exact equality before it can remain
eligible for `SEALED_OK`. A relational simulation-stamp regression retains its
otherwise recordable offending tuple as a separate forensic segment and
latches `SIM_STAMP`. Cross-process tracers cover an observation-only gap,
later valid evidence, capacity exhaustion, and a clean segment-count mutation
that must become `PROTOCOL/SEALED_FAULT`.

## PIT-0063: A rejected control request must not echo a bank identity

**Symptom.** A checksummed but invalid SEAL against retained terminal evidence
returns `INVALID` while its receipt still names that real bank and epoch. The
bank bytes happen to remain unchanged, but the response looks as though Writer
selected or validated the terminal identity.

**Cause.** The rejection branch copied `bank_index` and `bank_epoch` from the
untrusted request into the response instead of publishing the protocol's
no-selection identity.

**Guardrail.** Every invalid control response uses the public all-ones
`INVALID_BANK_INDEX`, epoch zero, and the completed-write fence observed before
the new write sequence is assigned. It binds the exact consumed request and
response by CRC, latches global `PROTOCOL`, and never changes ACTIVE,
`SEALED_OK`, or `SEALED_FAULT` bank bytes. Cross-process coverage mutates every
SEAL field independently, corrupts the request checksum, and compares all bank
words plus the full fixed segment capacity before and after unarmed FINISH.

## PIT-0064: An armed budget limits success, not attempted evidence

**Symptom.** Writer correctly seals an invocation-budget overflow as
`SEALED_FAULT`, but Parent refuses to read or ACK it because
`invocation_count > invocation_budget`. Repeating this on both banks leaves no
FREE bank even though the retained forensic evidence is complete and
checksummed.

**Cause.** Reader treated the trusted admission budget as a structural bound
on terminal attempted metadata. The qualifying over-budget invocation must
still consume its global sequence and extend first/last/count; only its exact
tuple is omitted. Exceeding the budget is therefore the evidence represented
by `FAULT_CAPACITY`, not proof that the mapping is corrupt.

**Guardrail.** Require a non-zero attempted count and exact fence arithmetic.
Allow it to exceed the invocation budget only when `FAULT_CAPACITY` is sticky;
continue to bound every stored segment by the fixed segment capacity and root
CRC. The regression reads a `budget=1, count=2` terminal page, verifies its one
retained segment plus two-attempt page range, and ACKs the exact faulted bank.

## PIT-0065: Validate-then-claim permits a duplicate-ACK ABA

**Symptom.** Two Parent threads ACK the same immutable snapshot. One releases
the bank; Writer reuses and seals the same bank at a newer epoch; the delayed
caller then compares only the identical terminal-state value and releases the
new evidence.

**Cause.** Both callers checked the snapshot registry and copied the old epoch
before either removed it. The final CAS protected only `SEALED_OK` or
`SEALED_FAULT`, so state reuse formed an ABA window even though every identity
and checksum comparison had been correct earlier.

**Guardrail.** Under a Parent-local lock, remove the exact registered snapshot
and mark its bank ACK-in-flight before any final validation. A duplicate ACK or
same-bank read fails immediately until the claimant finishes its one strong
CAS; every return and exception clears the in-flight marker. Writer cannot
reuse the bank before that CAS changes it to FREE. A deterministic two-thread
test pauses the claimant after its complete copy and proves the duplicate
cannot release the state or consume the newer epoch.

## PIT-0066: A delegated exception still owns a hardware-write sequence

**Symptom.** Adapter tests pass for ordinary `OK` and `ERROR` returns, but an
exception from the delegated Gazebo write leaves Writer permanently
OUTSTANDING, or is recorded as though upstream returned `ERROR` normally.

**Cause.** The Adapter began a Writer-owned cycle before delegation but closed
it only on the return path. Fabricating `ERROR` in the catch path hid the
difference between a returned result and an absent result.

**Guardrail.** Every successful `begin_write()` has exactly one
`finish_write()`, including exception paths. Use a named out-of-range delegated
exception sentinel so Writer latches `PROTOCOL`, preserve any independent
wheel-observation fault, and rethrow the original exception. Unit tests must
prove ordering and exactly-once completion without letting the Adapter own or
invent sequence numbers.

## PIT-0067: Shared-memory identity validators can drift across layers

**Symptom.** The pure robot-description transformer accepts an identity that
the C++ Attached Adapter rejects at runtime, or the Adapter opens a stale
same-name object because one layer applies a weaker nonce rule.

**Cause.** Parent creation, XML transformation, and C++ discovery each grew
their own approximation of a POSIX name or nonce grammar. Component tests used
different examples, so every layer was locally GREEN while composition could
not attach safely.

**Guardrail.** Lock one strict external identity contract everywhere:
`/voice_nav_hardware_<16-lower-hex>` plus exactly 32 lowercase, non-zero nonce
hex digits. Mutation tests compare Python and C++ rejection boundaries;
cross-process discovery proves a wrong nonce cannot claim `writer_pid`; the
real Gazebo test proves the exact launch-managed PID claims the transformed
positive identity.

## PIT-0068: Gazebo hardware-write cadence is not controller update cadence

**Symptom.** A short real-Gazebo ARM-to-SEAL interval unexpectedly terminates
with `FAULT_CAPACITY` even though its budget comfortably covers the expected
100 Hz controller updates.

**Cause.** `gz_ros2_control` can invoke the hardware `write()` path on each
1 ms simulation step while the controller manager updates commands every
10 ms. Hardware evidence assigns a sequence to the former, so a budget sized
from the latter undercounts by roughly an order of magnitude and can undercount
further when simulation runs faster than wall time.

**Guardrail.** Size and time evidence intervals from the observed hardware
write cadence, keep the capture window minimal, and retain a hard protocol
capacity. The isolated runtime regression uses an 8192-segment bounded region,
a 120 ms continuously published command window, immediate inclusive SEAL, and
requires a fault-free terminal snapshot plus no `/dev/shm` residue after three
fresh launches.

## PIT-0069: Evidence keywords do not prove that evidence executed

**Symptom.** A static contract and CTest both report GREEN after the runtime
test is changed to return or skip before its proof, an assertion is reversed,
the transformed URDF is discarded, or `RUNNER` / `RUN_SERIAL` text is moved
into an unrelated CMake label.

**Cause.** Text-presence checks confuse vocabulary with semantics. Python
tokens can remain in unreachable or negative assertions, while CMake
multi-value keywords consume later tokens and a later property assignment can
override an earlier one.

**Guardrail.** Develop evidence checkers with false-green mutation tests. Parse
the runtime test AST, reject skip/xfail/expected-failure and early termination,
bind the ordered PID, ARM, SEAL, snapshot, and ACK data flow to positive
assertions, and trace the owned transform into the returned launch graph.
Parse `add_launch_test` using its actual one-value/multi-value keyword rules;
require the exact isolated runner, owned CTest target, and one unambiguous
truthy `RUN_SERIAL` assignment. Keep the real headless-Gazebo test in the
package gate because a static checker is a guardrail, not runtime evidence.

## PIT-0070: A launch-test change has several evidence inventories

**Symptom.** The new launch test itself passes, but the repository contract,
generated-CTest audit, or final scoped xUnit report rejects the workspace. A
renamed critical test can likewise leave CTest GREEN while the final evidence
gate reports a missing case.

**Cause.** CMake registration is only one layer. The repository also owns an
allowed target/count contract, an expected generated CTest inventory, and a
critical `(classname, testcase)` xUnit inventory. Adding or renaming a test
without updating each semantic consumer creates drift.

**Guardrail.** Treat launch-test registration as one atomic change across
CMake, CI-readiness expectations, generated metadata validation, scoped result
reporting, and their mutation/unit tests. Run the complete `scripts/verify.sh`,
not only `colcon test`: the final gate must prove target inventory, isolated
runner, timeout, serialization, required testcase identities, and clean-install
boundaries in one pass.

## PIT-0071: Ambient Flake8 plugins are not the ROS package lint contract

**Symptom.** A focused `python3 -m flake8` invocation reports import-order and
docstring failures that the package's registered ROS lint target never
reported; changing imports to satisfy that invocation can then make
`ament_flake8` reject the opposite order.

**Cause.** The generic executable loads whatever Flake8 plugins and defaults
are installed in the current Python environment. ROS 2 package gates use the
versioned `ament_flake8` and `ament_pep257` policies separately, so their rule
set and import-order interpretation are the authoritative repository contract.

**Guardrail.** Use `ament_flake8` for style/import checks and `ament_pep257`
for docstrings, then execute the registered CTest to prove discovery and the
package environment. A generic Flake8 run may be useful diagnostic input, but
do not rewrite otherwise valid code solely to alternate between two ambient
plugin policies. Preserve its raw output in `/tmp` and record which command is
the release gate.
