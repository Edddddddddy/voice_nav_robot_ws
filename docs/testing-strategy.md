# Testing Strategy

Testing follows the deepest stable Interface available. A test should keep its
value when an internal implementation is refactored.

## Test layers

| Layer | Purpose | Examples |
| --- | --- | --- |
| Static | Reject malformed source and metadata | XML, YAML, Python, CMake, license |
| Unit | Exercise pure behavior and state transitions | Mission guard, rule NLU, timeout FSM |
| Contract | Protect externally visible semantics | ROS IDL, topic type, QoS, TF owner, limits |
| Integration | Verify connected ROS Modules | launch, Action cancellation, bridge directions |
| Simulation smoke | Verify physics and bounded workflows | drive, stop, odom, scan, map, Nav2 goal |
| Milestone | Validate real local models and GUI | Mandarin ASR/TTS, AEC, SLAM map quality |

## Developer loops

During implementation:

```bash
bash scripts/verify.sh <changed-package>
```

Before review or merge:

```bash
bash scripts/verify.sh
```

The full gate must start from declared dependencies, validate the robot model,
build all packages, run all tests, and produce a zero-error
`colcon test-result`.

Package test execution is sequential in the unified gate. The workspace is
small, and stable ordering, readable evidence, and bounded resource use are more
valuable here than the few seconds saved by package-level test parallelism.

The canonical REP-149 package schema is vendored under `tools/schema/`, and the
gate exports an XML catalog that maps the public schema URL to that reviewed
copy. Package validation therefore remains schema-aware without depending on a
remote HTTP response.

## Simulation safety

Gazebo Sim 8 native DiffDrive holds its last command and has no command
timeout. Every automated motion test must:

1. use conservative configured limits;
2. bound the test duration;
3. send zero Twist in both success and cleanup paths;
4. disable or pause the simulated drive if normal cleanup fails;
5. assert that pose or odometry becomes stationary;
6. terminate all processes it started.

`Ctrl+C` and publisher exit are not accepted as proof of stopping.

## Determinism

- Tests use simulation time only where behavior genuinely depends on it.
- Watchdogs and cancellation deadlines use a monotonic clock seam.
- Unit tests inject a manual clock rather than sleeping.
- LLM, ASR, TTS, map saving, and Nav2 adapters use scripted fakes for failure
  and race scenarios.
- Random seeds, worlds, initial poses, and resource limits are fixed.

## Evidence

Automated evidence is the command, exit status, and test result. Manual evidence
may add a screenshot, pose sample, TF tree, rosbag excerpt, or audio sample, but
must not replace an automatable assertion.

Verification evidence belongs in the work item or learning record. Generated
logs and artifacts do not belong in Git.

## Current gaps

The unified gate already checks the Xacro-to-URDF-to-SDF contract as well as the
package linters. The repository does not yet have:

- a package-level headless Gazebo drive-and-stop smoke test;
- ROS bridge type/direction tests;
- Mission runtime behavior tests.

These gaps are added alongside the relevant implementation rather than hidden
behind an arbitrary coverage percentage.
