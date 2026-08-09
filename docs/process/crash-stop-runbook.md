# Crash-stop acceptance runbook

This runbook describes the Issue #36 launch acceptance for an independent
`mission_runtime_node` or `motion_gate_node` failure. It is a real headless
Gazebo/Fast DDS/controller/odometry test; mocks may cover deterministic unit
seams but may not replace the process being killed.

## Scope and cadence

The ordinary PR gate runs each scenario once, with a fresh launch, partition,
and process-isolated ROS domain. The first real failure stops the run and is
not retried. Five fresh runs per scenario are nightly/release hardening
evidence, not an ordinary PR gate. Do not use CTest `--retest-until-pass`.

The two exact scenarios are:

- `mission_runtime_crash_stop`: kill the launch-owned Runtime process while a
  valid non-zero MOVE is active; prove Gate lease expiry, zero output,
  stationarity, restart isolation, and a new Goal recovery.
- `motion_gate_consumer_deadman`: kill the launch-owned Gate process while it
  is the unique final-command publisher; prove the controller consumer
  timeout, stationarity, restart isolation, and a new Goal recovery.

The tests must not claim external Gazebo pause/resume safety, supervisor
respawn, a journal, or a change to a public ROS IDL.

## Directed local gate

Run from a sourced Jazzy environment. The directed build may prepare the
workspace, but the acceptance command is exactly the two CTest names below:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select voice_nav_mission voice_nav_sim voice_nav_bringup \
  --symlink-install --event-handlers console_direct+
colcon test --packages-select voice_nav_bringup --ctest-args \
  -R 'mission_runtime_crash_stop|motion_gate_consumer_deadman' \
  --stop-on-failure --event-handlers console_direct+
colcon test-result --verbose
```

Do not substitute `scripts/verify.sh` for this task's directed gate and do
not poll hosted CI. A test that fails before its SIGKILL injection is still a
failure of the launch acceptance; record whether the failure is harness,
environment, or product behavior and stop before the next scenario.

## Exact process injection

Each test retains the `ProcessStarted` action for its product node. Before
signalling, the harness must prove all of the following for that same action:

1. The captured PID is the process started by the action, and `os.pidfd_open`
   succeeds immediately after the start event.
2. `/proc/<pid>/stat` starttime, `/proc/<pid>/exe`, and cmdline still match the
   launch executable and exact node marker.
3. The expected ROS fully qualified node name has one graph owner and the
   owner reports the expected executable/start arguments.
4. The pidfd remains valid immediately before injection.

The only permitted signal is
`signal.pidfd_send_signal(pidfd, signal.SIGKILL)`. The harness must not use
`pkill -f`, `killall`, process-name matching, PID files, `os.kill`, or a scan
of user processes. The acknowledgement timestamp is captured from
`time.monotonic_ns()` immediately after the pidfd syscall succeeds.

Cleanup closes the exact launch-owned actions and uses the existing structured
Gazebo shutdown. It does not kill by name and does not touch resources from a
user ROS/Gazebo session. Post-shutdown exit assertions accept SIGKILL only for
the exact injected action; all other launch-managed processes must exit
normally.

## Observable acceptance

Before Runtime injection, observe for at least 200 ms with advancing `/clock`:
Gate `ARMED`, live authority, fresh candidate data, non-zero final command,
and non-zero `cmd_vel_out`. From the pidfd acknowledgement, Gate must become
inhibited and publish a newer proven zero within 350 ms steady/wall time.

Before Gate injection, prove that Gate is the unique final-command publisher
and save the final non-zero command stamp. After injection, measure the first
zero `cmd_vel_out` against the last non-zero final command in advancing
simulation time; the accepted interval is `(0.35 s, 0.36 s]`. Publisher
disappearance is not zero proof.

In both scenarios, physical stationarity is a separate proof: `/odom` must
reach `abs(linear.x) <= 0.01 m/s` and `abs(angular.z) <= 0.02 rad/s`, and both
wheel velocities must be zero, within 1.2 s simulation time and held for
200 ms. Each simulation wait has a 5 s wall watchdog so a stopped clock fails
closed.

After the exact process restart, verify a new Runtime or Gate identity, reject
old Runtime/epoch or Gate instance/lease/control-sequence/candidate-writer
traffic without state mutation, and observe an advancing 1.0 s window with no
Goal in which final command, controller output, wheel velocity, and odometry
remain zero. A fresh `MOVE +0.25 m` must then complete and return to zero.

## Evidence and failure handling

Record the exact repository HEAD, package/RMW versions, scenario, launch
partition/domain, process PID/starttime/executable/cmdline, instance IDs,
kill acknowledgement, zero and stationarity timestamps, maximum measured
latencies, cleanup exit codes, and the bounded failure diagnosis. Keep full
logs outside Git and retain only compact evidence in the Issue or PR.

If a real run exposes a Runtime/MotionGate product defect, persist the command,
observations, smallest decision needed, options, and recommendation in Issue
#36 immediately, mark the work blocked, and notify the Manager. Do not expand
this harness task into a Runtime or MotionGate redesign. Do not run the second
scenario after the first real failure and do not automatically retry the
failed scenario.
