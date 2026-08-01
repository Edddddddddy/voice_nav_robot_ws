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
| PIT-0011 | Every ament CTest fails to import `ament_cmake_test` | Were ROS and the workspace overlay sourced in that shell? | Guarded |
| PIT-0012 | Product assertions pass, but Gazebo exits `-9` during CTest teardown | Did failure occur in the active test or the strict post-shutdown exit check? | Known (guard implemented; final evidence pending) |
| PIT-0013 | A focused runner test passes, but canonical discovery cannot import the real test tree | Does the fixture match the repository's package markers and import path? | Guarded |
| PIT-0014 | Concurrent launch tests collide despite a fixed `ROS_DOMAIN_ID` | Does generated CTest metadata invoke the official isolated runner without overriding its domain? | Guarded |
| PIT-0015 | One cleanup failure prevents later fixture destruction | Are teardown phases independent LIFO cleanups with exhaustive error aggregation? | Guarded |
| PIT-0016 | A green report did not execute the intended contract | Do source inventory, generated CTest, and critical xUnit name the same tests with no unapproved skip? | Guarded |
| PIT-0017 | Repeated WSL simulation fails while collecting Gazebo pose evidence | Is the server unhealthy, or did the bounded CLI query time out / emit a small JSON burst? | Guarded |
| PIT-0018 | CTest prints a failure but the surrounding command returns zero | Did a later diagnostic command replace the gate's shell exit status? | Guarded |

## PIT-0001: Windows-to-WSL quoting is a two-shell contract

**Symptom.** A valid-looking Bash command fails near `(`, a commit message is
split, or `$variable`/command substitution executes before Bash receives it.

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
of building a fragile cross-shell filter.

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
install audit, and canonical `scripts/verify.sh`; record the exact HEAD. Hosted
CI must test that same pushed head.

## PIT-0009: Documentation is part of the closed-set contract

**Symptom.** Implementation and tests allow exact Jazzy UNKNOWN sentinels, but a
Work Item or lesson still says only node name may be unresolved.

**Cause.** Behavior evolved after the first documentation commit and prose was
not included in the same review matrix.

**Guardrail.** Independent review compares implementation, tests, static
contract, Work Item, and lesson. When a closed set changes, update all five in
one corrective slice and retain the old evidence as pre-final rather than
silently relabeling it.

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
`ModuleNotFoundError: ament_cmake_test`, including otherwise unrelated GTests
and linters.

**Cause.** `ctest --test-dir build/<package>` was invoked from a fresh WSL shell
without sourcing ROS. The generated ament wrappers need ROS Python paths; the
existence of compiled test binaries does not supply that environment.

**Guardrail.** Source the base installation and current overlay in the same
shell before CTest:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ctest --test-dir build/voice_nav_mission --output-on-failure
```

If all targets fail at the same import, fix the invocation before diagnosing
product code. Canonical `scripts/verify.sh` already sources both environments.

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

**Scope.** This is a test/process-lifecycle correction. It is not Lesson 0010
Runtime/Gate process-death, controller consumer-deadman, managed safe-pause, or
first-resume-zero evidence. Keep this entry
`Known (guard implemented; final evidence pending)` until the unit,
integration, repeated, full-gate, review, and hosted-CI evidence passes.

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

**Checker threat boundary.** These guards prevent ordinary and accidental
source, CMake, discovery, and report regressions in a cooperative repository.
They are correctness checks, not a security sandbox. They do not claim to
resist a malicious same-UID process or deliberately hostile Python dynamic
metaprogramming that mutates files, imported objects, or runtime test IDs.

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
