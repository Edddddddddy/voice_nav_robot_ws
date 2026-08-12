# Crash-stop 验收手册

本手册描述 Issue #36 对独立 `mission_runtime_node` 或 `motion_gate_node` failure 的 launch 验收。它是
真实的 headless Gazebo/Fast DDS/controller/odometry test；mock 可覆盖确定性 unit seam，但不得替代被 kill
的 process。

## 范围与节奏

常规 PR gate 对每个 scenario 运行一次，使用全新 launch、partition 与 process-isolated ROS domain。首次真实
failure 即停止，不重试。每个 scenario 五次全新运行属于 nightly/release hardening evidence，不属于常规 PR
gate。不得使用 CTest `--retest-until-pass`。

两个精确 scenario 为：

- `mission_runtime_crash_stop`：有效非零 MOVE active 时 kill launch-owned Runtime process；证明 Gate lease
  expiry、zero output、stationarity、restart isolation 和新 Goal recovery。
- `motion_gate_consumer_deadman`：Gate 作为唯一 final-command publisher 时 kill launch-owned Gate process；
  证明 controller consumer timeout、stationarity、restart isolation 和新 Goal recovery。

测试不得声称外部 Gazebo pause/resume safety、supervisor respawn、journal 或 public ROS IDL 变化。

## 定向本地门禁

从已 source 的 Jazzy environment 运行。directed build 可准备 workspace，但验收命令严格为以下两个 CTest
name：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select voice_nav_mission voice_nav_sim voice_nav_bringup \
  --symlink-install --event-handlers console_direct+
colcon test --packages-select voice_nav_bringup --ctest-args \
  -R 'mission_runtime_crash_stop|motion_gate_consumer_deadman' \
  --stop-on-failure --event-handlers console_direct+
colcon test-result --verbose
```

不得用 `scripts/verify.sh` 代替本 Task 的定向门禁，也不得 poll hosted CI。若 test 在 SIGKILL injection 前失败，
仍属于 launch acceptance failure；记录其归类为 harness、environment 或 product behavior，并在下一 scenario 前
停止。

## 精确进程注入

每项 test 为 product node 保留 `ProcessStarted` action。signalling 前，harness 必须针对同一个 action 证明：

1. 捕获的 PID 确为 action 启动的 process，且 start event 后立即 `os.pidfd_open` 成功。
2. `/proc/<pid>/stat` starttime、`/proc/<pid>/exe` 与 cmdline 仍匹配 launch executable 和 exact node marker。
3. 预期 ROS fully qualified node name 具有一个 graph owner，且该 owner 报告预期 executable/start argument。
4. 注入前 pidfd 仍有效。

唯一允许的 signal 是 `signal.pidfd_send_signal(pidfd, signal.SIGKILL)`。harness 不得使用 `pkill -f`、
`killall`、process-name matching、PID file、`os.kill` 或扫描用户 process。acknowledgement timestamp 在 pidfd
syscall 成功后立即由 `time.monotonic_ns()` 捕获。

cleanup 关闭精确 launch-owned action，并使用既有 structured Gazebo shutdown；不按名称 kill，也不触碰用户
ROS/Gazebo session 的资源。post-shutdown exit assertion 仅允许被精确 injected action 因 SIGKILL 退出；所有
其他 launch-managed process 必须正常退出。

## 可观察验收

Runtime injection 前，在推进的 `/clock` 下至少观察 `200 ms`：Gate `ARMED`、live authority、fresh candidate
data、非零 final command 和非零 `cmd_vel_out`。从 pidfd acknowledgement 开始，Gate 必须在 `350 ms` steady/wall
time 内 inhibited 并发布更新的、已证明的 zero。

Gate injection 前，证明 Gate 是唯一 final-command publisher，并保存最后一个非零 final command stamp。injection
后，以推进的 simulation time，测量首个零 `cmd_vel_out` 相对最后非零 final command 的时差；可接受区间为
`(0.35 s, 0.36 s]`。publisher 消失不是 zero proof。

两个 scenario 中，physical stationarity 都是独立证明：`/odom` 必须在 `1.2 s` simulation time 内达到
`abs(linear.x) <= 0.01 m/s` 和 `abs(angular.z) <= 0.02 rad/s`，两个 wheel velocity 必须为零，并保持 `200 ms`。
每个 simulation wait 使用 `5 s` wall watchdog，因此 stopped clock fail closed。

精确 process restart 后，验证新的 Runtime 或 Gate identity，拒绝旧 Runtime/epoch 或 Gate instance/lease/
control-sequence/candidate-writer traffic，且不发生 state mutation。接着观察一个推进的 `1.0 s`、没有 Goal 的窗口，
其中 final command、controller output、wheel velocity 和 odometry 均保持零；随后新的 `MOVE +0.25 m` 必须完成并
回到零。

## 证据与失败处理

记录 exact repository HEAD、package/RMW version、scenario、launch partition/domain、process PID/starttime/
executable/cmdline、instance ID、kill acknowledgement、zero 与 stationarity timestamp、最大 measured latency、
cleanup exit code 与有界 failure diagnosis。完整 log 保留在 Git 外，Issue 或 PR 仅存紧凑证据。

若真实运行暴露 Runtime/MotionGate product defect，立即在 Issue #36 持久化 command、observation、最小所需
decision、option 与 recommendation，标记工作 blocked 并按 [AGENTS.md](../../AGENTS.md) 交接。不得将 harness Task 扩展为 Runtime 或
MotionGate redesign。首次真实 failure 后不得运行第二个 scenario，也不得自动 retry 失败 scenario。
