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
