# VN-0010-C2: Make launch-managed Gazebo teardown deterministic

**Status:** In Progress

**GitHub Issue:** TBD

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
- Product ordering is Gate inhibit, structured server stop, positive ACK,
  launch-managed process-exit barrier, then ROS fixture destruction.
- The helper uses direct argv with `shell=False`, inherits the CMake-owned
  partition, and refuses any missing or different partition before invoking
  the CLI.
- RPC timeout, non-zero CLI exit, false/malformed ACK, an already-dead server,
  or failure to observe the real process exit all fail the test.
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
- [ ] The `voice_nav_sim` control test passes 20 serial fresh launches.
- [ ] The complete MotionGate product test passes 20 serial fresh launches.
- [ ] Exact-head package tests, clean-install audit, and
  `bash scripts/verify.sh` pass.
- [ ] Before/after process evidence has no newly introduced residual process;
  this remains supporting evidence rather than the clean-exit oracle.
- [ ] Independent review has no unresolved P0-P2 finding.
- [ ] Required hosted CI passes on the final head.
- [ ] PIT-0012 is changed from `Known (guard planned)` to `Guarded` only after
  all preceding gates pass.

## Risks and rollback

- A stop request could target another server. Exact non-empty test partitions
  and the launch-managed process barrier bound both transport and ownership.
- A positive RPC response could be mistaken for completion. The separate
  process barrier and global exit-code assertion prevent that collapse.
- Cleanup failure could hide the original assertion. unittest records cleanup
  errors separately and still runs remaining cleanups; evidence preserves
  active-test and post-shutdown phases.
- Reverting C2 restores the signal-only teardown flake. It does not migrate
  persistent data or change a stable ROS Interface.

## Design impact

- Stable Interfaces changed: none.
- TF or motion ownership changed: none.
- Runtime product default changed: none.
- ADR required: no. This is a test-fixture lifecycle correction. Migrating the
  launcher to a different Gazebo server ownership API would require a separate
  architecture decision.

## Tests-first trail

- `841b7a7`: deterministic helper/static lifecycle RED and strict integration
  exit oracles.
- Pending commit: structured stop, default-on launch seam, cleanup ordering,
  control-checker refinement, and mutation guards GREEN.
- Pending: 20-run evidence, exact full gate, review, CI, and merge.

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

Final exact-head hashes, 20-run reports, static-contract counts, full
verification, independent review, hosted CI, PR, and merge evidence remain
pending and must not be claimed early.
