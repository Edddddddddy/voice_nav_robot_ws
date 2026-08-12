# 已知运行陷阱

这是执行、接口、仿真、证据和验证中可复发 failure 的当前归组参考。每个 `PIT-NNNN` heading 是稳定 link
target。条目只概述可复用规则；incident-specific evidence 由 owning Issue、PR、test 或 primary source 保存。

## WSL 执行与 ownership

### PIT-0001：Windows 到 WSL 的 quoting 是两层 shell 契约

PowerShell 先解析外层命令，Bash 再解析内层命令。优先使用短且显式 quoting 的 WSL script，在归因于 Bash
前检查最终 command boundary。

### PIT-0002：WSL transport warning 不是命令结果

WSL startup 或 localhost/NAT diagnostic 可能嘈杂或缓慢。以 child command 的 exit status 和决定性 output
作为证据，而不是 transport warning。

### PIT-0003：Windows 与 WSL identity 的 repository ownership 不同

“Dubious ownership”通常表示 Git 运行 identity 与 repository owner 不同。在拥有环境中运行 Git，或使用狭窄、
已批准的 repository-local trust setting；不要盲目扩大 global trust。

### PIT-0023：WSL Git 不能直接使用 Windows managed-worktree pointer

Codex managed worktree 的 `.git` file 可能含 Windows absolute `gitdir:`。WSL Git 会将它视为 relative path，并在
检查 worktree 前失败。`scripts/verify.sh` 在第一个 Git subprocess 前 source
`scripts/prepare_git_context.sh`：helper 解析普通 directory 与 relative pointer，用 `wslpath` 转换 Windows
absolute pointer，验证 target 与 `HEAD`，并向 child process export `GIT_DIR` 与 `GIT_WORK_TREE`。不得预设这些
变量、编辑 `.git` 或猜测另一 checkout。missing、malformed、unconvertible 或 nonexistent target 必须 fail closed
并给出安全 diagnostic。

### PIT-0024：WSL transport hang 需要有界诊断和人工恢复

`wsl.exe -d Ubuntu-24.04 --exec /bin/true` 可能挂起，或任务运行器报告 `Bash/Service/0x8007274c`，即使
`WslService`、`vmcompute`、`hns` 正在 Running 且 `wsl --version` 成功。用秒级 `/bin/true` red/green loop 记录
精确 `wsl.exe`/`wslhost.exe` PID 和 command line，将它与 ROS test 区分。

恢复前确认没有必须保留的 simulation、build 或未保存 WSL process。允许的恢复是一次官方 `wsl --shutdown`；
它会终止所有 WSL distribution 中的 process，因此不得自动执行，也不得未经 activity check 执行。不得 unregister
distribution、删除/移动 VHD、重装 Ubuntu，或添加自动 restart/polling script。用重复 `/bin/true` 及
`source /opt/ros/jazzy/setup.bash && ros2 pkg prefix rclcpp` 验证恢复；若仍失败，停止并升级到 Windows/WSL
service diagnosis，而不是以 global kill loop 隐藏问题。

## ROS Interface 与模型语义

### PIT-0004：ROS interface package 必须声明其 group

interface-generating package 必须在 `package.xml` 中与 generator dependency 一起声明
`rosidl_interface_packages`；在 source 的 Jazzy environment 中验证 metadata 与 build。

### PIT-0005：`base_footprint` 是逻辑 frame，不是实体 body

不得为 `base_footprint` 增加 visual、collision、mass 或 inertia。物理属性归属 robot body link；该平面 frame
仅是 navigation coordinate。

### PIT-0006：`xacro:material` 不是隐式 built-in macro

使用标准 URDF material syntax，或调用前定义/包含项目 macro。视觉上合理的 model 不是 macro expansion 有效的
证据。

### PIT-0007：DDS matching 与 ROS graph identity 不是原子操作

matched endpoint 可能暂时没有 resolved node identity。仅在原 absolute deadline 内 retry typed pending state；
wrong type、kind、QoS、FQN、duplicate、zero、missing 或 replaced GID 必须 fail closed。

### PIT-0008：验收证据属于 exact commit

代码、test 或文档变化后，祖先证据可能已过期。记录 exact final HEAD，并在该 HEAD 重跑最终检查。

### PIT-0009：文档是 closed-set contract 的一部分

supported value set 或 sentinel 改变时，同步更新 implementation、test、checker 与文档。implementation green 而
prose 矛盾时，契约尚未完成。

### PIT-0020：unit-quaternion formula 需要 unit quaternion

finite non-zero quaternion 在使用 RPY formula 前必须 norm-check 并 normalize。拒绝 zero、non-finite 或 invalid
norm，而不是接受看似合理的 pose。

### PIT-0021：size bound 不保证 schema complete

在组合 fixed diagnostic schema 前限制每个 variable field。截断最终 string 可能隐藏 missing field，或让一个很长的
错误 field 看似有效。

## DDS、deadline 与 isolation

### PIT-0010：absolute deadline 包围 RPC

向 RPC 传递剩余 duration 不代表 response 在 budget 内到达。每个 bounded call 后立即更新 response timestamp，
并再次检查 absolute deadline。

### PIT-0011：CTest 需要 ROS environment，不只是 build directory

在运行 CTest 的同一个 shell source base ROS installation 与 current overlay。configured build directory 不提供
`ament` import 或 runtime environment。

### PIT-0014：固定 ROS domain 不是 concurrent test isolation

domain ID 是 shared discovery namespace，不是 ownership。使用 official process-isolated runner 与 runtime-unique
lease，并清除 inherited domain override。

### PIT-0017：Gazebo query timeout 不是 teardown diagnosis

bounded pose query 可因 server 或 query stream 不健康而 timeout。将 authoritative query 与 shutdown oracle 分别诊断；
不要将 query retry 变成 teardown pass。

### PIT-0025：publisher 消失不是 zero proof

MotionGate process 死亡时，final publisher 消失只证明 writer 已消失；不能证明 controller 选择了什么，也不能证明
robot 已停止。必须在有界 zero 和 stationarity contract 下观测 controller output、wheel velocity 与 odometry。

### PIT-0026：controller `ACTIVE` 不是 wheel-zero proof

lifecycle state 与 claimed command interface 即使健康，仍可能消费 stale 或非零 command。验收必须观测
controller-selected command 和两个 wheel velocity；不得用 `ACTIVE` 或 interface ownership 代替。

### PIT-0027：wall time 与 simulation time 回答不同问题

Runtime lease expiry 和 test watchdog 使用 steady/wall time；consumer deadman latency、odometry stationarity 和
no-Goal recovery window 使用推进的 simulation clock。若 `/clock` 停止或 regression，测量 scenario 必须失败，
不能用 wall time 替代 simulation-time claim。

### PIT-0028：process name 与 PID scan 是不安全的 injection selector

按名称或扫描 PID 可能定位到用户的 ROS/Gazebo resource，或遇到 reused PID。将 injection 绑定到 launch-owned
`ProcessStarted` action、pidfd、`/proc` starttime/executable/cmdline 和唯一 ROS graph owner。仅使用
`pidfd_send_signal(SIGKILL)`，并在 cleanup 关闭同一精确 handle。

## 证据与 test-result integrity

### PIT-0016：Green status 不证明预期 test 实际运行

闭合 evidence chain：source inventory、discovery、generated CTest metadata、critical xUnit name、skip policy 和
executed count 必须描述同一组 test。summary 本身不充分。

### PIT-0018：尾部 diagnostic 可抹去失败 gate status

让 gate 成为其 exit status 被消费的终端命令。process snapshot、cleanup 等 diagnostic 作为独立命令运行，避免
后续 success 掩盖失败 test。

### PIT-0019：property-name allowlist 不验证 property value

检查 generated CTest metadata，比较 exact runner、label、timeout、working directory、environment 与 result
semantic。仅检查 property name 只验证 vocabulary，而不验证 behavior。

### PIT-0022：test-result evidence 需要共享树单 writer

从 verification startup 至 terminal status，将 `build/**/test_results` 视为 single-writer resource。只读审查可检查
copy 的 evidence，但不得运行另一项 result-writing test，也不得清除变化中的文件。

## Gazebo 清理

### PIT-0012：没有残留 Gazebo process 不等于 Gazebo clean exit

空 process list 不证明每个 launch-managed process 都按要求 exit code 返回。structured stop 需要 validated partition、
positive acknowledgement、process exit 和 unfiltered exit-code assertion。

### PIT-0013：test discovery fixture 必须匹配 repository layout

临时 importable package 可能让 focused runner test 通过，而真实 non-package `tests/` tree 失败。应练习与 repository
gate 使用的相同 directory shape 和 discovery entry point。

### PIT-0015：一个 composite cleanup 是一个 failure boundary

将 zero/inhibit、structured stop 与 fixture destruction 注册为独立、必须运行的 LIFO cleanup。仅在每个 cleanup
phase 均已尝试后 aggregate error。

## Shell 与 schema guardrail

repository checker 验证 text encoding、trailing whitespace、Markdown link/fence、支持的 documentation layout、
package version、Issue/PR form 和 retired-path contract。它有意不声称 sandbox 恶意的 same-UID process，也不替代
runtime safety test。
