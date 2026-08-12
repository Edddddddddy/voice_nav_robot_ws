# 测试策略

测试遵循最深稳定 Interface 与可观察行为。即使该 Interface 之后的 Implementation 被重构，behavior test 仍应
保留价值。测试应覆盖最高稳定 public seam，而不是 private implementation detail。

## 测试层次

| 层次 | 目的 | 示例 |
| --- | --- | --- |
| Static | 拒绝 malformed source 与 metadata | XML、YAML、Python、CMake、license |
| Unit | 覆盖确定性 behavior 与 state | Mission Validator/FSM、Agent rule、audio buffer |
| Contract | 保护外部可见 semantic | ROS IDL、topic type、QoS、TF owner、unit、limit |
| Integration | 验证连接的 ROS Module | launch、Action cancel、bridge direction、lifecycle |
| Headless simulation | 验证 physics 与有界 flow | drive、stop、odom、scan、map、Named Place |
| Model fixture | 验证锁定本地 model 与 offline audio | KWS、ASR、TTS、LLM、AEC fixture |
| Manual release gate | 验证支持的 WSL audio path | 真实单麦克风、speaker、AEC、barge-in |

## 验证节奏

实施中按需运行聚焦 repository check：

```bash
python3 tests/test_repository_contract.py
python3 scripts/check_repository.py --root .
```

review 前、最终 change 后且在最终 PR HEAD 上，产品变更最多运行一次完整 repository gate：

```bash
bash scripts/verify.sh
```

消费并记录该 invocation 的真实 exit status。后续 diagnostic、snapshot、cleanup 或成功 shell command 都不得
替代失败 gate status。full log 保留在 Git 外；PR 记录 command、真实 exit status、简洁 test summary 与有界
manual evidence。

完整 gate 从声明 dependency 开始，验证 repository 和 robot-model contract、构建全部 package、运行全部 test，
并报告零 error 的 `colcon test-result`。repository contract 经由 `scripts/run_repository_tests.py` 运行；
discovery 使用真实 non-package `tests/` layout，任何 skipped contract 都使 gate 失败。

关键 launch test 使用 Jazzy 官方 `run_test_isolated.py`。其 generated CTest contract 清除继承的
`ROS_DOMAIN_ID` 与 `DISABLE_ROS_ISOLATION`、保留 `RUN_SERIAL`，且只允许经过审查且 result-neutral 的 property。
source CMake 不是最终证据：configure 后，`scripts/check_generated_launch_tests.py` 检查
`ctest --show-only=json-v1`，确认 exact runner、source target、environment、reviewed per-test timeout、
resolved package build working directory、单一 `launch_test` label 与 result semantic。required mutation test 分别
替换 `LABELS`、`TIMEOUT` 与 `WORKING_DIRECTORY`，每种替换都必须失败。reporter 随后要求匹配的 critical xUnit
testcase structure；skip 仅允许精确 package-local cppcheck artifact/class allowlist。不得以该 allowlist 新增
scaffolded Python lint skip，而应删除 skip 并使其通过。

完整 gate 是其 exit status 被消费的终端 verification command。process snapshot 和其他 diagnostic 在之后作为
独立 command 运行；尾部成功的 `ps`、`grep` 或 cleanup 永远不能替代失败 CTest status。

## 受限结构检查器

既有 safety、concurrency、test-result ownership 与 Gazebo lifecycle checker 持续有效。新增 AST、source-shape 或
full-file-fingerprint checker 仅在以下三项均成立时允许：

1. 父 Issue 或 Task Issue 已记录真实 recurring failure。
2. checker 保护能表达该 failure 的最窄稳定 public repository seam；global text ban 不能替代它。
3. 父 Issue 或 Task Issue 在实现前明确批准该 checker。

checker 必须检验可观察 repository contract，不能替代 stable Interface 的 behavioral test。change-volume 和
ten-commit stop rule 见[变更生命周期](change-lifecycle.md#停止并重新划分范围)。

### 共享 test-result ownership

workspace 的 `build/**/test_results` tree 同一时刻只能有一个 writer。canonical `scripts/verify.sh` 运行从启动到
terminal status 独占 operational window。在此期间，reviewer 与 parallel agent 可以检查 source、Git metadata 和
已复制 evidence，但不得运行 `ctest`、`colcon test`、另一 verify process 或任何改写共享 result tree 的 helper。
并发测试必须使用独立 build/install/log base，或等待 canonical gate 结束。

result reporter 有意 snapshot inode、size、mtime 与 ctime；若 writer 与 evidence collection overlap 则 fail closed。
出现该 diagnostic 时，识别 writer 并建立 quiescence 后才可做完整 retry；不得清除指定文件或放宽 identity check。
参见 [PIT-0022](known-pitfalls.md#pit-0022test-result-evidence-需要共享树单-writer)。

本地 WSL 上的 exact HEAD 产品验证在 Module 存在后尽快采用确定性 in-memory fake，并在 v0.2 simulation
milestone 后增加有界 headless Gazebo test。远端 PR CI 只运行 shellcheck、actionlint、治理契约和 Conventional
Commit；它不安装 ROS，也不运行 ROS/package build、package test、headless Gazebo、Voice 或 LLM 产品检查。
model fixture 引入后的 locked model set、real-audio metric 和 `v0.7`/`v1.0` manual hardware Release Gate 都是本地
产品验证或人工 release evidence，不会被削弱成远端 CI simulation。

## Adapter 与时间策略

内部 seam 具有 production 与确定性 in-memory Adapter：

| 接缝 | 生产适配器 | 确定性测试适配器 |
| --- | --- | --- |
| Navigation | Nav2 `NavigateToPose` | scripted goal/result/cancel fake |
| Relative motion | odom feedback 与 candidate Twist | scripted motion fake |
| Motion authority | 独立 MotionGate | 含 lease expiry 的 event recorder |
| Map saving | slam_toolbox/map saver | in-memory map registry |
| Clock | steady monotonic clock | manual clock |
| ASR/TTS/LLM | locked local runtime | scripted text/audio/result fake |

Mission behavior test 穿过 Mission Module Interface 并替换下游 Adapter，不断言 private FSM state。Adapter contract test
证明 production Adapter 将 upstream ROS behavior 映射为相同 internal semantic。

- Physics、TF、SLAM、AMCL 与 Nav2 在适用时使用 simulation time。
- Mission timeout、cancel grace、command lease、audio liveness 与 cleanup deadline 使用 steady monotonic clock。
- unit test 通过推进 manual clock，而不是 sleep。
- fake 注入 timeout、abort、partial map、delayed cancel、late success 与 dependency loss。
- random seed、world、initial pose、model version 与 resource limit 固定在 acceptance evidence 中。

## 覆盖率门禁

- Mission Core 与 Agent：至少 `90%` line coverage、至少 `80%` branch coverage。
- 不需要真实 hardware 的 audio code：至少 `80%` line coverage。

coverage 是相关 milestone 的 release gate。它补充 behavior assertion，不能替代它们。

## Mission 完成标准

Mission unit 与 contract test 覆盖：

- discriminant 与 payload field 的无效组合；
- NaN 与 Inf reject；
- Mapping/Navigation Mode policy；
- 任何 motion side effect 前，对三步 Mission 执行 atomic whole-plan validation；
- single execution-slot `BUSY` behavior；
- source ordering、`runtime_instance_id` 与 `admission_epoch`；
- Runtime restart 使旧 request 失效；
- Cancel、STOP、natural success 与 timeout race 经由同一 terminal linearization point；
- late Nav2、relative-motion、map 与 Agent callback；
- 严格一个 Result 与 non-decreasing best-estimate feedback；
- ROS time paused 或变化时的 steady-clock timeout behavior。

suite 还证明 rejected plan 不启动下游 Adapter，且 late result 不能重新打开 MotionGate 或推进下一 step。

## MotionGate 与停止完成标准

每个 automated motion test 都使用 configured limit、steady-clock deadline、success 与 cleanup path 的 zero output、
odometry-based stationarity check 和有界 process cleanup。`Ctrl+C`、publisher exit、Action Result 或单次 zero
publication 都不是 stopping proof。

### Gazebo launch-test 生命周期

拥有 Gazebo server 的 test 使用独立于 product assertion 的 lifecycle oracle。module import 时，每个 test process
用 scope/PID/128-bit-random non-empty `GZ_PARTITION` 覆盖 inherited state；CMake 不提供可复用的 fixed partition。
cleanup 先选择 zero 或 inhibit MotionGate，在已验证的相同 environment snapshot 中向 `/server_control` 发送
`stop: true`，要求正向 `gz.msgs.Boolean` acknowledgement，然后等待 launch-managed `gazebo` process 自身 exit。
ACK 只是 request acceptance，不是 process completion。post-shutdown test 最后对每个 launch-managed process 使用
无过滤的 `assertExitCodes(proc_info)`。

product launch 默认在 Gazebo exit 时 shutdown。test 仅禁用该立即 event handler，以便其 failure-safe cleanup 执行
structured stop 与 process join。cleanup ladder 必须运行：zero/inhibit、structured stop 与 ROS fixture destruction 是
独立 LIFO `unittest` cleanup，因此一个 exception 不得短路后续项。拥有多个 resource 的 cleanup phase 使用
exhaustive aggregator，并仅在每一步均已尝试后 raise collected error。isolated idempotent stop request 的 typed
`TimeoutExpired` 在 fresh CLI process 中 retry 一次；其他 CLI/ACK error 立即失败，两个 timeout 仍失败。static
mutation test 拒绝 fixed partition、fixed sleep、global process kill、shell execution、forced-exit allowlist、
ACK-only cleanup、rebound/unreachable oracle、disabled critical test module、错误 RPC environment、cleanup list
mutation 和可在 active assertion failure 后跳过的 cleanup registration。

Gazebo ground-truth movement evidence 与 ROS odometry 分离。pure test-support module 以 `10 s` deadline 和一次
read-only retry 查询精确 isolated world 的 pose topic。它最多接受四个相邻完整 JSON document，因为
`gz topic --num 1` 可与高频 publisher race 并输出小 burst；每个 document 都必须含一个 valid model pose，
使用最新者。wrong partition、malformed/extra output、duplicate/missing model、zero/non-finite quaternion 和
non-finite pose 都失败。finite valid-norm check 后，四个 quaternion component 均在 unit-quaternion RPY formula
运行前 normalize；scaled-quaternion regression 必须得到与等价 unit quaternion 相同 RPY。query failure 是 active-test
failure，不是 teardown diagnosis。

该 fixture contract 证明确定性 test teardown，不证明缓慢 signal-only Gazebo shutdown 的内部 cause、普通用户
`Ctrl+C` behavior、MotionGate crash-stop、controller deadman 或 managed pause/resume semantic。参见
[PIT-0012](known-pitfalls.md#pit-0012没有残留-gazebo-process-不等于-gazebo-clean-exit)。

source/AST guard 只是普通审查变更的 cooperative correctness control，不声称 sandbox 恶意 same-UID process 或
故意使用 Python dynamic metaprogramming 改写 file/imported object 的行为。

### 当前正常运行 Gate 切片

当前 Gate 切片证明正常独立运行，但不声称 process death 或 pause recovery：

- manual-clock Core table 覆盖精确 `250 ms` authority 和 `150 ms` candidate-freshness boundary；`20 ms` wall output
  tick 在 inhibited 时持续选择 zero。它还证明 activation RENEW 仅在 first accepted sample 前可重启有界
  first-candidate window，之后 RENEW 不能隐藏 stale producer，candidate sample 永远不能续约 Runtime authority。
- 每个 `PREPARE`/`OPEN`/`RENEW`/`INHIBIT` request 使用 Gate instance 与一个 global compare-and-swap `control_seq`；
  `PREPARE` 后的 operation 还匹配 Gate-generated current lease。late old-lease `INHIBIT` 不能停止新 lease；
  matching current `INHIBIT` 在 acknowledgement 前发布 zero。
- finite `linear.x`/`angular.z` 被 clamp 到 trusted YAML limit。NaN、Inf 或 non-zero unsupported axis 会 retire current
  lease 并选择 zero。
- `/motion_gate_node` 服务 `/motion_gate/internal/control` 与 `/motion_gate/internal/state`。PREPARE 返回
  `/voice_nav_internal/motion_gate/candidate/lease_` 下 Gate-generated per-lease topic。OPEN 先在 Core 中无 graph
  access 验证，再要求同一 unique publisher GID 依次出现在 discard reader A 的 graph snapshot #1、recreate
  discard reader B 后的 snapshot #2，以及创建第一个 accepting `VOLATILE + KEEP_LAST(1)` reader C 后的
  snapshot #3；任何变化都 fault closed。
- contract test 要求 trusted YAML root `motion_gate_node`，并证明全部 node/control/state/candidate-prefix/
  final-command name 为 code constant、不在 YAML parameter 或 product remap 中。
- locked `rmw_fastrtps_cpp` self-test 将 Gate-local graph GID 与 Gate-local `MessageInfo.publisher_gid` 关联；
  control request 永不携带 caller `Publisher::get_gid()`。
- candidate QoS 是 `BEST_EFFORT + VOLATILE + KEEP_LAST(1)`。唯一 final publisher 使用
  `rclcpp::SystemDefaultsQoS()`，runtime checker 证明与 controller subscriber 的实际 compatibility；被 introspect
  为 `UNKNOWN` 的 policy 不得断言为固定 reliability/history/depth。
- serial publication barrier 证明 current-lease INHIBIT、expiry 或 invalid-input zero 后，先前 queued non-zero
  command 不会 publish。
- runtime parameter test 拒绝移动中改变 `use_sim_time`。publication 同时要求 parameter value 与
  `ros_time_is_active()`；任一 invariant 丢失即 fault closed、publish zero，并且永不发 system-time-stamped
  non-zero command。
- headless Gazebo evidence 分别记录有界 motion 后及每个 normal deadline expiry 后的 Gate zero、controller output
  zero 与 odometry stationarity。

package-private IDL 是 encapsulation boundary，不是 DDS security。当前 test 使用 authority/candidate harness；
它们不将 authority、candidate 或 MotionGate process kill、managed pause token、first-resume zero 或 unmanaged-pause
recovery 视为已完成。

### Process-death 与 pause 验收切片

process-death 与 Gazebo-time acceptance 需要证明：

- authority 被 kill 而看似有效 candidate 持续时，独立 Gate lease 仍会 expiry；
- candidate producer 被 kill 时，candidate freshness 仍会 expiry；
- MotionGate 被 kill 时，`diff_drive_controller.cmd_vel_timeout` 在推进 simulation time 的 `0.35 s` 后第一次
  control update 选择 zero；
- managed pause 在 mint token 前证明 Gate、controller 与 wheel zero，并在 resume 后证明首个 wheel command 为 zero；
- unmanaged GUI/Transport pause 没有 token，需完整 simulation/control restart，不做 in-place resume。

后续 Mission 与 voice milestone 继承以下 v1.0 quantitative acceptance criterion：

- 从 `StopMission` request 到最终 zero-velocity output：P95 `<= 100 ms`、P99 `<= 200 ms`、maximum `<= 300 ms`；
- 从最大 configured speed 开始，STOP 使 odometry 在 `1.2 s` 内进入 stationary tolerance，并保持 `200 ms`；
- kill MissionRuntime 导致独立 MotionGate lease expiry，并自动选择 zero velocity；
- kill MotionGate 导致 `diff_drive_controller.cmd_vel_timeout` 在推进 simulation time 的 `0.35 s` 后第一次 control
  update 选择 zero；配置的 `100 Hz` period 给测量一项 `10 ms` scheduling tolerance，physical stationarity 为独立
  assertion；
- candidate sample 永不续约 Runtime authority；test 在 kill Runtime 后持续投送有效外观 smoother output，仍观测
  Gate inhibition；
- 每次 step handover 重新创建 candidate data plane。test 在新 lease open 后注入 old topic generation 和 unbound
  Gate-local writer sample，并证明它们被 reject；
- managed Gazebo safe-pause 先在 simulation 仍推进时证明 Gate output、controller output 与 wheel command 为 zero，
  再 pause 并记录 token；pause 中 MotionGate death 后，首个 resumed wheel command 仍必须为 zero；
- fault-injection 在 zero proof 前 kill MotionGate。controller inactivity 或 released command interface 不足：harness
  仅在直接观测到配置时段的 zero wheel command 后发 token，否则选择完整 restart；
- direct GUI/Transport pause 没有 safe-pause token；拒绝 in-place resume，测试 recovery 为从 inactive zero-command
  state 开始的完整 simulation/control restart。

这些 command-inhibition threshold 不声称系统是功能安全认证的 emergency stop。

### Issue #36 crash-stop 验收

crash-stop 切片是实际 headless Gazebo/Fast DDS/controller/odometry acceptance，不是被 kill process 的 mock
replacement。两个普通 PR scenario 各运行一次，均使用 fresh process-isolated launch；第一次真实 failure 停止该对
scenario 且不 retry。五次 fresh repetition 属于 nightly/release hardening，对普通 PR 非阻塞。

harness 为 Runtime 和 MotionGate 拥有 exact `ProcessStarted` action：立即打开 pidfd、记录 `/proc/<pid>/stat`
starttime、executable 与 cmdline、验证唯一 ROS graph owner，并且仅注入
`signal.pidfd_send_signal(pidfd, SIGKILL)`。禁止 process name、PID scan、`pkill -f` 和 broad cleanup。Runtime death
从 steady pidfd acknowledgement 到 Gate zero 测量；Gate death 从最后非零 final command 到推进 simulation time 中
首个 zero controller output 测量。publisher disappearance 和 controller `ACTIVE` 不是 zero 或 stationarity proof。

两个 scenario 独立证明 odometry 与 wheel stationarity、fresh Runtime/Gate identity、stale tuple 与 writer isolation、
`1.0 s` no-Goal zero window，以及新 Goal recovery。launch fixture 使用 unique Gazebo partition、structured shutdown、
有界 wall watchdog 和 unfiltered exit-code assertion。真实 product failure 持久化为 blocked Issue，不授权在
acceptance harness 中重构 Runtime/MotionGate。

## Mapping 完成标准

- TF ownership check 证明每条所需 transform 只有一个 semantic owner。
- 保存生成包含 occupancy YAML、image 与 posegraph 的完整 atomic map directory。
- saved map package 可再次加载。
- partial failure 不暴露 half-written map package。
- Map ID handling reject path traversal。

## Navigation 完成标准

- 三个预定义 Named Place 均不碰撞地成功。
- 每个 final pose position error `<= 0.25 m`，yaw error `<= 0.25 rad`。
- success、failure、cancel 和 timeout navigation path 均返回 zero velocity。
- 尝试同时启动 SLAM 和 AMCL 的 launch 必须失败，不能生成两个 `map → odom` owner。

模式检查还证明：Mapping 使用 `slam_toolbox` 拥有 `map → odom`；Navigation 使用 AMCL 拥有它；两种模式均由
`diff_drive_controller` 拥有 `odom → base_footprint`；robot-internal frame 归 `robot_state_publisher`；
`ros_gz_bridge` 只 bridge `/clock` 与 `/scan`。

## Agent 完成标准

固定中文语料覆盖 deterministic rule、clarification、local LLM fallback、schema-valid 但 semantic-invalid output、
LLM timeout，以及较新 turn 或 STOP 后 late LLM response。任意 LLM output 都不能 publish velocity、提供 path，
或覆盖 trusted speed、acceleration、tolerance、timeout、map-path 或 admission policy。

## Offline voice 与 audio 完成标准

确定性 offline fixture 覆盖 far-end-only audio、near-end-only audio、double-talk、`40 ms` 至 `250 ms` acoustic delay、
`±100 ppm` clock drift、PortAudio xrun、ring-buffer overflow、late TTS PCM 和 fixed STOP preemption。fixture 还验证
48 kHz mono full-duplex callback boundary、`10 ms/480-sample` DSP framing、render-reference ordering、16 kHz KWS/ASR
input 以及 stale playback/turn result isolation。real-time callback 不做 allocation、blocking、logging、ROS call 或
model inference。

## 真实单麦克风与 speaker 完成标准

验收使用支持的 motherboard analog microphone input 与 speaker output，关闭 Windows audio enhancement 与 spatial audio：

- 排除前 `2 s` convergence 后，far-end-only ERLE median `>= 6 dB`；
- wake-word recall：quiet environment `>= 95%`，playback 期间 `>= 90%`；
- double-talk command semantic success rate `>= 85%`；
- 两小时 TTS-only run 产生零错误 Mission；
- fixed STOP recall `>= 95%`；
- 从 STOP phrase 结束到 MotionGate zero velocity，P95 `<= 500 ms`；
- `30 min` soak 无未处理 overflow、无 uncontrolled playback，且无明显 memory growth。

## 证据与当前缺口

automated evidence 是 command、真实 exit status、简洁 test-result summary 和相关 coverage 或 latency report。
manual evidence 可附 screenshot、pose sample、TF graph、sanitized audio clip 或 model manifest，但不能替代可自动化
assertion。

Issue 维护需求、决策、验收、依赖和状态；PR 维护结果、final HEAD、验收映射、test summary、接口影响、回滚和
残余风险。不得重复 Issue 正文、粘贴完整 log 或维护 per-commit evidence diary；generated log 与 private artifact
不进入 Git。

v0.1 foundation audit 时，unified gate 覆盖 repository metadata、model expansion、URDF/SDF semantic、build 和
package test，但尚未包含 `gz_ros2_control`、LiDAR、MotionGate、Mission Runtime、SLAM、Nav2、Agent 或 voice
behavior test。每项缺口都由已批准路线图中的 capability milestone 收敛。
