# Known operational pitfalls

This is the current grouped reference for recurring execution, interface,
simulation, evidence, and validation failures. Each `PIT-NNNN` heading is a
stable link target. The entries summarize the reusable rule; the owning Issue,
PR, test, or primary source carries incident-specific evidence.

## WSL execution and ownership

### PIT-0001: Windows-to-WSL quoting is a two-shell contract

PowerShell parses the outer command before Bash parses the inner command.
Prefer a short, explicitly quoted WSL script and inspect the final command
boundary before blaming Bash.

### PIT-0002: WSL transport warnings are not the command result

WSL startup or localhost/NAT diagnostics may be noisy or slow. Use the child
command exit status and decisive output as evidence, not the transport warning.

### PIT-0003: Repository ownership differs across Windows and WSL identities

“Dubious ownership” usually means Git is running as an identity different from
the repository owner. Run Git in the owning environment or use a narrowly
approved repository-local trust setting; do not broaden global trust blindly.

### PIT-0023: WSL Git cannot consume a Windows managed-worktree pointer

A Codex managed worktree may contain a `.git` file whose `gitdir:` value is a
Windows absolute path. WSL Git treats that value as a relative path and fails
before it can inspect the worktree. `scripts/verify.sh` sources
`scripts/prepare_git_context.sh` before its first Git subprocess; the helper
resolves ordinary directories and relative pointers, converts Windows
absolute pointers with `wslpath`, validates the target and `HEAD`, and exports
`GIT_DIR` plus `GIT_WORK_TREE` to child processes. Do not pre-set those
variables, edit `.git`, or guess another checkout. Missing, malformed,
unconvertible, and nonexistent targets must fail closed with a safe diagnostic.

### PIT-0024: A WSL transport hang needs bounded diagnosis and manual recovery

The WSL transport can recur in a state where
`wsl.exe -d Ubuntu-24.04 --exec /bin/true` hangs or Codex/Worker reports
`Bash/Service/0x8007274c`, even while `WslService`, `vmcompute`, and `hns` are
Running and `wsl --version` succeeds. Distinguish this from a ROS test by
running the `/bin/true` command as a seconds-scale red/green loop and recording
the exact `wsl.exe`/`wslhost.exe` PIDs and command lines.

Before recovery, confirm that no simulation, build, or unsaved WSL process must
be preserved. The permitted recovery is one official `wsl --shutdown`; it
terminates running processes in all WSL distributions, so it must not be
automated or run without that activity check. Do not unregister a distribution,
delete or move a VHD, reinstall Ubuntu, or add an automatic restart/polling
script. Verify recovery with repeated `/bin/true` executions and then
`source /opt/ros/jazzy/setup.bash && ros2 pkg prefix rclcpp`. If it still fails,
stop and escalate to Windows/WSL service diagnosis rather than hiding the
failure with a global kill loop.

## ROS interface and model semantics

### PIT-0004: A ROS interface package must declare its group

An interface-generating package must declare `rosidl_interface_packages` in
`package.xml` alongside its generator dependencies. Validate metadata and build
from a sourced Jazzy environment.

### PIT-0005: `base_footprint` is a logical frame, not a physical body

Do not add visual, collision, mass, or inertia to `base_footprint`. Physical
properties belong to the robot body links; the planar frame is a navigation
coordinate only.

### PIT-0006: `xacro:material` is not an implicit built-in macro

Use standard URDF material syntax or define/include the project macro before
invocation. A visually plausible model is not evidence that macro expansion is
valid.

### PIT-0007: DDS matching and ROS graph identity are non-atomic

A matched endpoint can have a temporarily unresolved node identity. Retry only
the typed pending state inside the original absolute deadline; wrong type, kind,
QoS, FQN, duplicate, zero, missing, or replaced GID remains fail-closed.

### PIT-0008: Acceptance evidence belongs to an exact commit

Evidence from an ancestor can become stale after code, tests, or documentation
change. Record the exact final HEAD and rerun final checks there.

### PIT-0009: Documentation is part of the closed-set contract

When a supported value set or sentinel changes, update implementation, tests,
checker, and documentation together. A green implementation with contradictory
prose is an incomplete contract.

### PIT-0020: Unit-quaternion formulas require a unit quaternion

A finite non-zero quaternion must be norm-checked and normalized before RPY
formulas. Reject zero, non-finite, or invalid norms rather than accepting a
plausible-looking pose.

### PIT-0021: A size bound is not a schema-completeness guarantee

Bound each variable field before composing a fixed diagnostic schema. Truncating
the final string can hide a missing field or make a long wrong field appear
valid.

## DDS, deadlines, and isolation

### PIT-0010: An absolute deadline surrounds the RPC

Passing a remaining duration to an RPC does not prove the response arrived in
budget. Update the response timestamp and check the absolute deadline again
immediately after every bounded call.

### PIT-0011: CTest needs the ROS environment, not only a build directory

Source the base ROS installation and the current overlay in the shell that runs
CTest. A configured build directory does not supply `ament` imports or runtime
environment.

### PIT-0014: A fixed ROS domain is not concurrent test isolation

A domain ID is a shared discovery namespace, not ownership. Use the official
process-isolated runner with a runtime-unique lease and clear inherited domain
overrides.

### PIT-0017: A Gazebo query timeout is not a teardown diagnosis

A bounded pose query can time out because the server or query stream is
unhealthy. Diagnose the authoritative query separately from the shutdown
oracle; do not turn a query retry into a teardown pass.

### PIT-0025: Publisher disappearance is not a zero proof

When a MotionGate process dies, its final publisher disappearing only proves
that the writer is gone. It does not prove what the controller selected or
that the robot stopped. Observe the controller output, wheel velocities, and
odometry under the bounded zero and stationarity contracts.

### PIT-0026: Controller `ACTIVE` is not a wheel-zero proof

Lifecycle state and claimed command interfaces may remain healthy while a
stale or non-zero command is still being consumed. Acceptance must observe
the controller-selected command and both wheel velocities; do not replace
that evidence with `ACTIVE` or interface ownership.

### PIT-0027: Wall time and simulation time answer different questions

Runtime lease expiry and test watchdogs use steady/wall time. Consumer
deadman latency, odometry stationarity, and no-Goal recovery windows use an
advancing simulation clock. If `/clock` stops or regresses, fail the measured
scenario instead of substituting wall time for a simulation-time claim.

### PIT-0028: Process names and PID scans are unsafe injection selectors

A process name or a PID discovered by scanning can refer to a user's ROS or
Gazebo resource, or to a reused PID. Bind injection to the launch-owned
`ProcessStarted` action, pidfd, `/proc` starttime/executable/cmdline, and a
unique ROS graph owner. Use only `pidfd_send_signal(SIGKILL)` and close the
same exact handle during cleanup.

## Evidence and test-result integrity

### PIT-0016: Green status is not proof that the intended tests ran

Close the evidence chain: source inventory, discovery, generated CTest metadata,
critical xUnit names, skip policy, and executed count must describe the same
tests. A summary alone is insufficient.

### PIT-0018: A trailing diagnostic can erase a failed gate status

Make the gate the terminal command whose exit status is consumed. Run process
snapshots, cleanup, and other diagnostics as separate commands so a later
success cannot mask a failed test.

### PIT-0019: A property-name allowlist does not validate property values

Inspect generated CTest metadata and compare exact runner, label, timeout,
working directory, environment, and result semantics. Checking only property
names validates vocabulary, not behavior.

### PIT-0022: Test-result evidence requires one shared-tree writer

Treat `build/**/test_results` as a single-writer resource from verification
startup through its terminal status. Reviewers may inspect copied evidence but
must not run another result-writing test or clear a changing file.

## Gazebo cleanup

### PIT-0012: No residual Gazebo process is not a clean Gazebo exit

An empty process list does not prove that every launch-managed process returned
the required exit code. Structured stop requires the validated partition,
positive acknowledgement, process exit, and unfiltered exit-code assertions.

### PIT-0013: Test discovery fixtures must match the repository layout

A temporary importable package can make a focused runner test pass while the
real non-package `tests/` tree fails. Exercise the same directory shape and
discovery entry point used by the repository gate.

### PIT-0015: One composite cleanup is one failure boundary

Register zero/inhibit, structured stop, and fixture destruction as independent
must-run LIFO cleanups. Aggregate errors only after every cleanup phase has
been attempted.

## Shell and schema guardrails

The repository checker validates text encoding, trailing whitespace, Markdown
links and fences, supported documentation layout, package versions, Issue/PR
forms, and the retired-path contract. It intentionally does not claim to
sandbox malicious same-UID processes or replace runtime safety tests.
