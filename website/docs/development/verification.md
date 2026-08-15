# 测试与贡献

VoiceNav 的远端 CI 只承担快速治理；ROS、Gazebo 和产品行为必须在支持的本地 WSL2 exact HEAD 上验证。
“GitHub check 变绿”不等于机器人产品切片已经通过。

## 一项变更的生命周期

```text
GitHub Issue
  → 独立 worktree / 短生命周期 branch
  → 聚焦 RED → 最小 GREEN → green 下重构
  → 本地验证与完整 diff 审查
  → Draft PR + fresh read-only Review
  → required 治理 CI
  → rebase merge
```

Issue 是需求、决策、验收、依赖与状态的权威。一个 Task 对应一个 independently reversible change，
不得把外部 PR 当成需求入口，也不得直接在共享 checkout 或 `main` 上开发。

## 分层测试

| 层级 | 证明什么 | 典型对象 |
| --- | --- | --- |
| Pure unit | parser、validator、state transition、deadline | AgentCore、RuntimeCore、MotionGateCore |
| Contract | name、type、QoS、parameter、TF owner | ROS IDL、YAML、launch、bridge |
| Integration | Node composition、lifecycle、process seam | Runtime/Gate/Agent/audio Adapter |
| Headless product | 真实 controller、Gazebo、motion、stationarity | supported WSL2 product slice |
| Manual milestone | 真实音频设备、锁定模型与用户可见 flow | v0.7 / v1.0 release evidence |

低层测试不能替代高层产品证据，高层 smoke 也不能取代 deterministic Core failure matrix。

## 开发期间的聚焦检查

```bash
python3 tests/test_repository_contract.py
python3 scripts/check_repository.py --root .
git diff --check
```

产品变更在最终 exact HEAD 通常只运行一次完整门禁：

```bash
bash scripts/verify.sh
```

真实失败后不能在无代码变化的同一 HEAD 反复重跑来“碰绿”。应先保存 exit status 与日志，定位因果，形成新 HEAD 后再跑。

## 远端 required check 的边界

`required / ubuntu-24.04 / ros-jazzy` 检查 shellcheck、actionlint、仓库治理与中文 Conventional Commit。
它不会安装 ROS、运行 rosdep、colcon、Gazebo、真实语音或 `scripts/verify.sh`。

## 运动测试的额外义务

- normal 与 cleanup path 都请求 zero；
- 直接观察 Gate/command zero 与 odometry stationarity；
- 使用 steady clock 验证 lease、cancel、timeout；
- 检查 TF edge 的唯一 fully-qualified owner；
- process-death 测试分别覆盖 Runtime、Gate 与 controller deadman；
- 任一 owner 仍可能保留 authorized non-zero command 时，测试不得退出。

## 文档与依赖

- README 只描述实际能力，目标行为必须带状态；
- Stable Interface 变更同步 producer、consumer、contract test、文档与 changelog；
- `package.xml` 是 ROS dependency 的 source of truth；
- Python dependency 首次引入要有审查过的 lock；
- model weights、录音、bag、私有地图、凭据和 raw evidence 不进入 Git。

页面事实依据：[`CONTRIBUTING.md`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/CONTRIBUTING.md)、
[`change-lifecycle.md`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/process/change-lifecycle.md)、
[`testing-strategy.md`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/docs/process/testing-strategy.md)。
