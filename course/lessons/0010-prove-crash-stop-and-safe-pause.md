# 证明进程崩溃停止与托管安全暂停

Lesson 0010 · 教师参考实现：进行中 · 学习者复现：Pending

**本课唯一成果：** 你将用可杀死的独立进程和真实 Gazebo 控制链证明两层
deadman，并以 package-private coordinator + test Adapter 证明“先证明零、再
暂停、凭单次 token 恢复”的 Managed Safe Pause 协议。

```text
authority/candidate process loss
        -> MotionGate steady-time deadman
        -> final zero

MotionGate process loss
        -> diff_drive_controller simulation-time deadman
        -> wheel command/state zero
        -> odometry stationary

Managed Safe Pause
        -> zero proof while simulation advances
        -> observed paused world
        -> generation-bound one-shot token
        -> token-checked resume or RESTART_REQUIRED
```

本课不会加入 MissionRuntime、StopMission、SLAM、Nav2、语音、公共 pause API、
新 ROS 包或第五个产品常驻进程。“停止”仍是仿真项目的 operational stop，不是
功能安全认证急停。

先阅读：

- [VN-0011 umbrella](../../docs/work-items/0011-crash-stop-and-safe-pause.md)
- [VN-0011A crash-stop](../../docs/work-items/0011a-process-death-crash-stop.md)
- [Motion safety contract](../../docs/architecture/safety-and-motion-contract.md)
- [ADR-0005](../../docs/adr/0005-use-tokenized-managed-safe-pause.md)
- [故障学习与复发控制](../../docs/process/problem-learning.md)

## 0. 从不可变累计基线开始

本课从 Lesson 0009 及其 C1/C2 修正后的公开 `main` 开始：

```text
course/0010-start tag object:
92a054c3eaae6e4dd0e8500aa712e866e8a71e33

peeled target:
f75a9c48f610306a1cf3ec83d0e5e99474220ad6
```

先核对本地和远端身份：

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
git fetch origin --tags
git rev-parse course/0010-start
git rev-parse "course/0010-start^{}"
git ls-remote origin \
  refs/tags/course/0010-start \
  "refs/tags/course/0010-start^{}"
```

学习者不要直接改 `main`：

```bash
git worktree add \
  ../voice_nav_robot_lesson_0010 \
  -b learn/0010 \
  course/0010-start
```

`course/0010-solution` 尚未创建。它只能在 A/B 两个实现切片通过评审、CI 并
合并到公开 `main` 后发布，不能预写其 hash。

## 1. 先分清三种时钟和五种证据

| 事实 | 正确时钟 / 线性化点 |
| --- | --- |
| authority lease 与 candidate freshness | MotionGate steady clock；测试延迟从 observer 收到 exact `ProcessExited` 起算 |
| 进程真正死亡 | exact ProcessAction 的 `ProcessExited`，不是发信号调用 |
| controller timeout | 严格递增的 simulation stamp |
| wheel command/state 与 odometry 静止 | 严格递增的 simulation stamp |
| 测试卡死保护 | wall-time watchdog，仅用于有界退出 |
| pause commit | World Statistics 报告 paused，且 iteration/sim time 停止增长 |
| resume zero proof | 不开启 continuous run；exact 单步/重暂停先遇到并证明 controller update zero，再多一步 losslessly 写入该 zero |
| continuous resume commit | lossless zero proof 后才发送 `pause:false`，随后 iteration 持续增长 |

证据面不能互相替代：

- `/motion_gate/internal/state` 说明 Gate 决策和原因；
- `/diff_drive_controller/cmd_vel_out` 是 controller 限幅后的底盘命令；
- `/controller_manager/introspection_data/full` 中
  `command_interface.<wheel>/velocity` 才是 ros2_control 轮端命令；
- `/joint_states` 是轮端状态，不是命令；
- `/odom` 证明仿真机器人进入并保持静止容差。

introspection 订阅必须显式使用 publisher-compatible
`BEST_EFFORT + TRANSIENT_LOCAL + KEEP_LAST(1)`；故障前先观察完整四字段、有限、
严格递增且左右 command 连续非零的 armed baseline。否则 retained/initial zero
可能制造假阳性。其 command 值是下一次同步 hardware write 将消费的值，不是
Gazebo 已执行回执。

而且 introspection 是有损诊断流：它必须作为独立佐证保留，但不能证明中间没有
漏掉一条 non-zero write，也不能证明精确 first write。A/B 共用一个默认关闭的
test-only lossless hardware-write ledger。测试 Adapter 继承公开的
`GazeboSimSystemInterface`，通过 pluginlib 创建固定的上游
`gz_ros2_control/GazeboSimSystem`，原样委托 lifecycle、interface、`read()`、
`write()` 与 `initSim()`；只在上游 `write()` 返回后读取真实左右轮
`JointVelocityCmd`。不要直接继承具体 `GazeboSimSystem`：Jazzy 1.2.19 的头文件中
PImpl 类型未完成，外部派生类析构会编译失败。

不要复制整份机器人 Xacro。测试 transformer 应先展开 canonical product Xacro，
要求恰好一个 upstream hardware plugin，只替换这一 XML block 并注入共享内存身份；
其余结构必须等价。产品 Xacro、launch 与 YAML 中不得出现 Adapter 或 journal 参数。

ledger 在实际 `write()` seam 计入每次调用、test generation、simulation stamp、
上游返回值和左右轮命令位模式。“lossless”还必须落成协议：单调不回绕
`write_seq`、同一 write seam 的 atomic ARM/SEAL fence、arm 前 segment 容量证明、
overflow/overwrite/未计入调用/zero-window nonzero 故障锁存、sealed interval 保留，
以及分页 checksum 与 generation/`write_seq` 连续检查。只有 generation、stamp、
返回值与轮速位模式完全相同的连续调用才能折叠，segment 的 sequence range 和
count 必须一致。只把 topic 改为 reliable 不够。

公开 hardware Interface 没有 `UpdateInfo.iterations` 参数，不能伪造每条 write 的
Gazebo iteration。每次调用仍必须推进 `write_seq`，simulation stamp 在 pause 时可
重复；真实 iteration 由 World Statistics 独立记录。Slice A 用它证明 world 连续
前进；Slice B 用 ARM/SEAL 区间关联经 ACK、World-Statistics-confirmed 的精确
`N -> N+1` 单步。新增 journal instrumentation 必须预分配且无分配；这不等于宣称
固定上游 `write()` 自身无分配。

如果只看到一条 zero、controller inactive、interface released 或 odom 接近零，
都还没有完成完整的零证明。

## 2. 把工作拆成两个 tests-first 切片

### Slice A：独立进程 crash-stop

Slice A 使用三个 fault case，每个 case 都创建新的 Gate generation：

1. authority 与 candidate 是两个独立 OS 进程；
2. 只按手里持有的 exact launch action 发送 `SIGKILL`；
3. 每次 SIGKILL 前，<=40 ms steady barrier 只约束 authority/candidate 有效和
   recent non-zero Gate commit，发信号时最后 Gate receipt <=20 ms；只有 Gate-kill
   case 才另外要求 generation 内此前未使用且 exactly-once 的 final marker；simulation
   evidence 独立要求 `/clock` 前进，controller/introspection/lossless write 非零、
   严格递增且相对 Gate input 不旧于 30 ms simulation time；不能把低 RTF 混入
   steady deadline，期间也不能插入 zero/invalid；
4. authority 死亡时，candidate 继续；parent-owned Gate event journal 用同一主机
   `CLOCK_MONOTONIC` 证明 terminal transition 的显式 linearization fence 与绑定
   zero output 的 publish-call 前 INTENT 不早于 exact ProcessExited；更晚的 COMMIT
   只证明操作完成，不能倒推因果顺序。Core 的唯一 transition wrapper 记录
   signal/exit 在途期间接受的全部 control transition；禁止把 journal 调用散落到
   Node callbacks；
   terminal `control_seq` 必须是最后一个已提交前驱的非回绕 `+1`。随后收到的 state
   保持同一 Gate instance、清空 lease、匹配 journal sequence，zero/output publish
   seq 产生新且相等的 zero，并以 `AUTHORITY_EXPIRED` 在 300 ms steady time 内到达；
5. candidate 死亡时，authority 继续 renew；这些合法 RENEW 不能被测试禁止，而要
   全部进入 journal。terminal retirement 同样严格接在最后已提交 RENEW 之后，并以
   `CANDIDATE_EXPIRED` 在 exact ProcessExited 后 200 ms steady time 内归零；
6. Gate 最后被杀，Gazebo/controller 继续运行，测试不得补发 zero；candidate
   生成限幅内、间距大于比较容差且 generation 内此前未使用的 final tuple。Gate
   本来就会每 20 ms 重发当前 tuple，因此不能要求整个 crash window 的所有值都
   唯一。parent-owned crash-resilient journal 对每次 publish 先写 INTENT、成功后
   改为 COMMITTED；final marker 第一次 COMMIT 后必须先被 non-zero
   `/cmd_vel_out` ACK，再在下一次 20 ms publish 前发送 exact SIGKILL。若死亡前已
   出现第二条相同或任何后续 output record，本 generation 失败并以新 generation
   重试；SIGKILL 后 final record 必须仍是这条唯一 COMMITTED marker；
7. 第一条 controller output zero 只有在该 final committed input 年龄严格大于
   0.35 s 后出现；publisher count 归零和 100 ms quiet 只证明 cleanup，不能冒充
   controller callback acceptance；
   并在下一次 100 Hz update 加显式 step tolerance 内完成；lossless ledger
   独立证明 first both-wheel zero 和之后不回归；
8. 从较晚的 controller zero / lossless both-wheel-zero linearization 起，
   1.2 s 内开始共同 wheel-state/odom stationary window，并保持 0.20 s。

Crash ledger 必须穷尽：预先登记的被杀 action 恰好退出 `-SIGKILL`，其他所有
launch-managed action 必须退出 0。禁止 `[0, -9]` 宽 allowlist、`pkill`、名称
广播和 Gazebo 例外。

### Slice B：托管安全暂停

Slice B 在 `voice_nav_bringup` 内实现 package-private 深模块，并由 test Adapter
调用：

```text
RUNNING
  -> INHIBITING
  -> ZERO_PROVING
  -> PAUSE_REQUESTED
  -> PAUSED(token)
  -> RESUME_VALIDATING
  -> RUNNING

any proof/token failure -> RESTART_REQUIRED
```

必须先在 simulation time 仍前进时完成 Gate/controller/wheel/odom 零证明，
再请求 pause，并用 World Statistics 确认真正暂停，最后才生成 opaque、单次使用、
generation-bound token。缺失、过期、重放或 generation 不匹配的 token 必须拒绝
原地恢复，而且不能发送 `pause:false`。

token 至少绑定 partition、world、Gazebo process identity、paused
iteration/time、controller generation/state、Gate instance/control sequence、
final publisher identity、controller update stamp、fixed step/control period、
lossless-oracle generation/sealed fence 和
zero-proof stamp/sequence。原 Gate 若仍在，必须保持
同一 inhibited tuple；若在 token 后死亡，必须有 exact death evidence 且 final
command topic 没有 publisher。replacement Gate、不同 tuple 或新 publisher 都使
token 失效。由于 introspection 是异步 BEST_EFFORT，B 还要使用默认关闭的
test-only lossless hardware-write oracle。不发送 continuous `pause:false`；每次
WorldControl 都必须精确为 `{pause: true, multi_step: 1}`，ACK 只代表排队，World
Statistics 必须逐次证明恰好 `+1` 且重新 paused。以单步方式在
`ceil(control_period / step_size) + 1` 的界内遇到下一次 controller update；同一
sim stamp 的新 `/cmd_vel_out` 与完整 introspection 必须都为 zero。然后再单步
一次，lossless journal 必须证明整个 armed probe 以及 post-update write 全为 zero，
才允许连续运行。省略/false pause、重复 request、non-zero/missing update、缺号或
错误 iteration 都必须失败。

本切片只允许同一个 ACTIVE `diff_drive_controller` generation 获得并消费 token。
如果 zero proof 必须 deactivate controller，或 controller inactive/replaced，直接
返回 `RESTART_REQUIRED`；不要在本课偷偷增加 activation resume 分支。

该切片只交付协议和 test Adapter，不声称已经存在用户可调用的产品 pause
功能；test Adapter 接收 `RESTART_REQUIRED`。未来 lifecycle supervisor 才是
产品宿主。

“拒绝 unmanaged resume”只约束项目提供的 managed Adapter。它不是 Gazebo
Transport 的权限边界；本机用户仍可绕过项目直接操作 Gazebo。项目策略在这种
情况下选择旧 generation 结构化关闭并返回 `RESTART_REQUIRED`。自动拉起新
generation 需要未来 supervisor，本课不伪装成已经实现。

## 3. RED 必须证明测试自己是可信的

本课采用同一个循环：

```text
pure valid fixture passes
  + negative/mutation fixtures fail for exact reasons
  + repository product assertion alone is RED
  -> commit test contract
  -> implement minimum GREEN
  -> refactor behind the same Interface
```

Slice A 的文档 contract 已落盘，CrashLedger 的 exact identity/closed exit 与
signal-intent/ProcessExited 两个纯逻辑 RED/GREEN 微循环也已落盘并通过 ament/CTest
注册。simulation-window analyzer、repository-only RED、Gate/hardware journal 和
isolated launch 仍必须继续按 tests-first 补齐：

- 扩展 pure crash-ledger tests；
- pure simulation-window evidence tests；
- static/mutation repository contract；
- isolated headless product launch test；
- generated CTest/xUnit inventory and strict teardown checks。

在任何新测试存在之前，可以执行当前基线检查：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/run_repository_tests.py
bash scripts/verify.sh
```

新测试落盘后，记录精确命令、退出码、测试数量和唯一预期 RED。语法错误、导入
失败、测试未发现、全量 skip 或错误环境不算 tests-first 证据。

## 4. 故障注入矩阵

| 注入 | 必须仍然存活/推进 | 必须失败的错误实现 |
| --- | --- | --- |
| kill authority | candidate、Gate、controller、simulation | candidate 数据偷偷续租 authority |
| kill candidate | authority renew、Gate、controller、simulation | renew 偷偷刷新 candidate freshness |
| kill MotionGate | Gazebo、controller、simulation | test 发 zero、Gate respawn、Gate 退出带走 Gazebo |
| pause 前 Gate 故障 | simulation 继续到 bounded zero proof | inactive/released 被误当 zero |
| pause 后 Gate 故障 | paused generation 与 token | 恢复首个 wheel command 重放旧 non-zero |
| unmanaged/stale token | managed Adapter | 仍调用 `pause:false` 原地恢复 |

每次实际发现新的可复发原因，按
[Problem learning](../../docs/process/problem-learning.md) 处理：先保留 symptom、
命令、环境和 HEAD，再把 root cause 放入已有或新的 `PIT-NNNN`，并在最近所有权
边界增加自动 guardrail。不要为同一原因创建多个名字不同的坑记录。

## 5. 自动验收

Lesson 0010 完成时至少满足：

- authority/candidate 是可分别 kill 的 OS 进程；
- exact action、exact signal、exact exit 与 exhaustive ledger 全部通过；
- authority/candidate Gate-zero 分别不超过 300/200 ms steady time；
- 每次 fault 的 <=40 ms steady arming、<=20 ms Gate receipt 与 <=30 ms
  simulation-sample age 均成立；Gate journal 的 terminal transition-linearization
  fence 与 bound-zero pre-publish INTENT 不早于 `ProcessExited`，later COMMITTED
  只证明完成，不能拿它或 DDS 接收顺序替代因果证据；
- terminal state 必须同一 Gate instance、空 lease，`control_seq` 匹配 journal 中
  最后已提交 control predecessor 的非回绕 `+1`，并使用新
  `zero_publish_seq == output_publish_seq`；不能漏记合法 RENEW 或接受陈旧 zero；
- controller deadman 从 crash-resilient journal 中 generation 内此前未使用且
  exactly-once COMMITTED 的 Gate input 起算；它必须在下一次 20 ms 重发前由匹配
  non-zero `/cmd_vel_out` ACK 并触发 exact kill，满足严格
  `age > 0.35 s` 与 100 Hz update 窗口；
- introspection 仍是左右轮 command/state 的 mandatory corroboration；lossless
  write ledger 以 fences、无缺号、无 overflow/overwrite 的 closed interval 证明
  exact wheel transition 与 no-regression；odom 独立；
- 从较晚的 controller/lossless both-wheel command-zero sample 起，1.2 s 内进入共同的
  wheel-state/odom 静止容差并保持至少 0.20 s simulation time；
- Managed Safe Pause 在 token 前证明 zero 和真实 paused state；
- 不开启 continuous run，以 exact `{pause:true,multi_step:1}` 单步到下一次
  zero controller update，再多一步证明 post-update hardware write 与整个 probe
  都是 zero，之后才 continuous unpause；
- 无效 token 不 unpause，进入 `RESTART_REQUIRED`；
- 每个 Gazebo test 有独立 ROS domain、唯一 partition、串行执行和确定性 teardown；
- exact-final-head local gate、独立评审、required CI、public tree 与 solution tag
  按不自引用策略记录。

## 6. 提交给教师的材料

完成复现后提交：

1. `git log --oneline --decorate` 与 clean `git status --short`；
2. start tag object/peeled target 核对结果；
3. RED 中唯一预期失败及为什么测试框架本身可信；
4. 三个 crash case 的 exact action/exit、时钟和指标表；
5. Gate、controller、wheel command、wheel state、odom 五层证据；
6. Managed Safe Pause / Unmanaged Pause 的 token/generation/World Statistics
   证据；
7. canonical gate 摘要与 process-residue audit；
8. 本课中新建或更新的 PIT 条目，以及对应自动 guardrail；
9. 下面复盘问题的个人回答。

## 复盘

请用自己的话回答：

1. 为什么 candidate 继续发消息不能证明 authority 仍活着？
2. 为什么 `SIGKILL` 调用不是进程死亡的线性化点？
3. 为什么 Gate deadman 用 steady time，而 controller deadman 用 simulation time？
4. 为什么 `/cmd_vel_out`、wheel command、wheel state 和 odom 不能互相替代？
5. 为什么 controller timeout 条件是 `> 0.35 s`，不是 `>= 0.35 s`？
6. 为什么必须在 pause 前完成零证明？
7. token 需要绑定哪些 generation 事实，为什么只能消费一次？
8. 为什么 managed resume 不是 Gazebo Transport 安全边界？
9. 为什么 `RESTART_REQUIRED` 不等于“自动重启已经完成”？
10. 哪个 mutation 最能暴露一个看似通过、实际绕过安全链的测试？

```text
Learner reflection:
TBD
```
