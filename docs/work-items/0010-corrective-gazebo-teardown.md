# VN-0010-C2: Make launch-managed Gazebo teardown deterministic

**Status:** In Progress

**GitHub Issue:**
[Edddddddddy/voice_nav_robot_ws#15](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/15)

**Branch:** `fix/vn-0010-gazebo-teardown`

## Problem statement

At exact head `ccf9fbe`, the MotionGate product test passed its first five
fresh launches. On the sixth launch every active product assertion still
passed, including writer binding, bounded motion, clamping, both automatic
expiry paths, explicit inhibit, stationarity, and final-writer ownership.
The test then failed only in its strict post-shutdown check: launch escalated
from SIGINT to SIGTERM and finally SIGKILL, so the launch-managed Gazebo
process returned `-9`.

The same class of teardown failure occurred historically in a
`voice_nav_sim` test without MotionGate. The supported fact is therefore
narrow: signal-driven Gazebo teardown sometimes exceeds launch's cumulative
grace window. The evidence does not identify a particular Gazebo,
`gz_ros2_control`, controller, Ruby, or middleware defect. It also does not
make “no new residual process” equivalent to a clean exit.

This corrective item is stacked on VN-0010-C1 so that writer-identity and
Gazebo-lifecycle causes remain independently reviewable. After C1 merges, C2
must be rebased onto `main` before its own merge.

## Goal

Give every launch test that starts Gazebo one explicit lifecycle contract:

1. preserve the test's normal zero/inhibit cleanup;
2. send `stop: true` to Gazebo's `/server_control` service inside the test's
   exact isolated `GZ_PARTITION`;
3. require a positive Boolean acknowledgement;
4. wait for the launch-managed process named `gazebo` to exit; and
5. retain an unfiltered post-shutdown `assertExitCodes(proc_info)` over every
   launch-managed process.

## Non-goals

- Changing MotionGate, Mission, public ROS Interfaces, TF ownership, or motion
  ownership.
- Claiming that the internal Gazebo/plugin blocking cause has been proven.
- Allowing exit `-9`/`137`, deleting the strict exit assertion, adding a fixed
  sleep, or merely extending signal timeouts.
- Sending a service request in the default partition or terminating any
  pre-existing user Gazebo process.
- Counting this as Lesson 0010 process-death, controller-deadman, managed
  pause, or first-resume-zero evidence.
- Treating server shutdown as a recoverable simulation pause.

## Lifecycle contract

- Product behavior keeps its existing default: an unexpected Gazebo exit
  shuts down the launch. Tests alone set a bounded launch argument that keeps
  the test runner alive while its registered cleanup stops and joins Gazebo.
- Cleanup is registered before the active test body can fail and remains
  effective when `setUp` or a behavior assertion fails.
- Cleanup ordering is zero/Gate inhibit, structured server stop, positive ACK,
  launch-managed process-exit barrier, then ROS fixture destruction.
- Each launch-test process replaces any inherited `GZ_PARTITION` with a
  scope, PID, and 128-bit random nonce before launch constructs its context.
  CMake retains `RUN_SERIAL` but owns no fixed Gazebo partition.
- The helper uses direct argv with `shell=False`, validates the claimed
  partition, and passes the same checked environment snapshot to the CLI so a
  concurrent environment mutation cannot redirect the stop request.
- RPC timeout, non-zero CLI exit, false/malformed ACK, an already-dead server,
  or failure to observe the real process exit all fail the test.
- A `TimeoutExpired` from the isolated, idempotent `stop: true` CLI is retried
  once in a fresh process. Non-zero exit, malformed ACK, wrong partition, two
  timeouts, and the process-exit barrier remain fail-closed.
- ACK means only that the stop request was accepted; it never substitutes for
  the process-exit barrier.
- The final post-shutdown assertion remains strict and global.

## Acceptance criteria

- [x] A deterministic unit RED covers wrong/missing partition, CLI failure,
  timeout, false/malformed ACK, positive ACK, and failure after ACK to observe
  process exit.
- [x] A strict `voice_nav_sim` post-shutdown oracle covers the historical seam
  without MotionGate.
- [x] All three Gazebo launch tests register failure-safe structured cleanup.
- [x] The launch test seam defaults to normal product behavior and is disabled
  only by the isolated tests.
- [x] Static mutation guards reject ACK-only cleanup, filtered exit codes,
  fixed sleep, process-kill shortcuts, `shell=True`, missing partition
  isolation, and cleanup absent from a failure path.
- [x] Every Gazebo test claims a process-unique partition; fixed CMake
  partitions and mismatched RPC environments are rejected.
- [x] The canonical repository runner executes the real non-package `tests/`
  layout and fails the gate if any contract test is skipped.
- [x] The `voice_nav_sim` control test passes 20 serial fresh launches.
- [x] The complete MotionGate product test passes 20 serial fresh launches.
- [ ] Exact-head package tests, clean-install audit, and
  `bash scripts/verify.sh` pass.
- [x] Before/after process evidence has no newly introduced residual process;
  this remains supporting evidence rather than the clean-exit oracle.
- [ ] Independent review has no unresolved P0-P2 finding.
- [ ] Required hosted CI passes on the final head.
- [ ] PIT-0012 is changed from `Known (guard planned)` to `Guarded` only after
  all preceding gates pass.

## Risks and rollback

- A stop request could target another server. A per-process scope/PID/random
  partition, the exact checked RPC environment, and the launch-managed process
  barrier bind both transport and ownership. This prevents accidental stale,
  concurrent CTest, and cross-worktree collisions; it does not claim to resist
  a malicious same-UID process that steals a live nonce.
- A positive RPC response could be mistaken for completion. The separate
  process barrier and global exit-code assertion prevent that collapse.
- Cleanup failure could hide the original assertion. unittest records cleanup
  errors separately and still runs remaining cleanups; evidence preserves
  active-test and post-shutdown phases.
- Reverting C2 restores the signal-only teardown flake. It does not migrate
  persistent data or change a stable ROS Interface.

## Design impact

- Stable Interfaces changed: the simulation/product launch configuration gains
  the backward-compatible `shutdown_on_gazebo_exit` argument, defaulting to
  `true`. Public ROS names, types, QoS, TF ownership, and motion semantics are
  unchanged.
- TF or motion ownership changed: none.
- Runtime product default changed: none.
- ADR required: no. This is a test-fixture lifecycle correction. Migrating the
  launcher to a different Gazebo server ownership API would require a separate
  architecture decision.

## Tests-first trail

- `841b7a7`: deterministic helper/static lifecycle RED and strict integration
  exit oracles.
- `a9dec46`: structured stop, default-on launch seam, and must-run cleanup
  GREEN.
- `91c6318` / `8096f0b`: unreachable-oracle RED/GREEN.
- `59136c8` / `fe92c6c`: process-unique Gazebo partition RED/GREEN.
- `d3b1dfa` / `23c9b8f`: skip, rebind, and unreachable-checker bypass
  RED/GREEN.
- `1d7653c`, `ebd2fe1` / `a651bca`: real non-package repository discovery and
  fail-on-skip runner correction.
- `5f65d65`: failure-safe independent cleanup, structured-stop process join,
  official per-process ROS isolation, and strict post-shutdown GREEN.
- `cc2733b`: generated CTest/xUnit/source inventory closes execution and skip
  evidence gaps.
- `281f409`: generated-result semantics, module-level test, cleanup-rebinding,
  and direct pose-topic evidence hardening.
- `46f7b71`: typed transient structured-stop timeout RED/GREEN retry.
- `09d213c` / `53f9568`: bounded, isolated ground-truth pose module and
  multi-document CLI burst RED/GREEN.
- Pending: exact full gate, final review, hosted CI, and merge.

## Verification evidence

Pre-remediation failure boundary:

```text
Exact head: ccf9fbe
Command: ctest --test-dir build/voice_nav_bringup --output-on-failure \
  --repeat until-fail:20 -R test_test_motion_gate_product.py
Fresh launches 1-5: PASS
Fresh launch 6 active MotionGate/product assertions: PASS
Fresh launch 6 post-shutdown: gazebo exit -9; strict assertExitCodes failed
```

Final documentation-head static-contract counts, full verification,
independent review, hosted CI, PR, and merge evidence remain pending and must
not be claimed early.

Repeated runtime evidence after the final implementation change:

```text
Implementation head: 53f9568
Command: ctest --test-dir build/voice_nav_sim --output-on-failure \
  --repeat until-fail:20 -R '^test_test_simulation_control.py$'
Result: 20/20 PASS; 31.84-43.81 s per launch; CTest exit 0
Total CTest time: 716.29 s

Command: ctest --test-dir build/voice_nav_bringup --output-on-failure \
  --repeat until-fail:20 -R '^test_test_motion_gate_product.py$'
Result: 20/20 PASS; 19.58-28.41 s per launch; CTest exit 0
Total CTest time: 507.21 s

Postcondition: the only matching Gazebo/launch/control process was the
pre-existing user-owned PID 3631225 (PPID 1); no test-owned process remained.
This process snapshot supports, but does not replace, positive ACK, process
join, and unfiltered assertExitCodes evidence.
```

During the repeat gate, three distinct client-side evidence failures were
preserved and corrected tests-first: a 5 s `gz model` timeout, a 7 s
structured-stop CLI timeout, and `gz topic --num 1` returning two complete
JSON documents. None was relabeled as a Gazebo clean-exit failure. The final
pose helper uses the exact isolated partition, two bounded 10 s read-only
attempts, at most four complete documents, the newest complete snapshot, a
unique model pose, a valid finite quaternion, and finite XYZ/RPY fields.

Pre-final repository-contract evidence:

```text
Head: a651bca
Command: source /opt/ros/jazzy/setup.bash &&
  python3 scripts/run_repository_tests.py
Result: 214 tests, 0 skipped, 0 errors, 0 failures
Classification: implementation evidence only; final exact-head gate pending
```
