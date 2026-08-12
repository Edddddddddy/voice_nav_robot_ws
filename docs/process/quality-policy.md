# 质量策略

VoiceNav Robot 以 [REP-2004](https://reps.openrobotics.org/rep-2004/) 作为版本、变更控制、文档、测试、
依赖、平台支持和安全的检查清单。它不声称生产级或安全认证质量：这是一个 pre-1.0 仿真原型，目标是
获得可复现、可审查的行为。

最严格的仓库策略适用于全部六个 ROS package。

## 版本策略

- 项目发布遵循 Semantic Versioning。
- ROS package version 在 release boundary 保持同步。
- Stable Interface 包含 ROS 名称、type、field、QoS、TF ownership、configuration schema、unit、
  ordering、error、clock 和 cancellation。
- pre-1.0 兼容性可以变化，但破坏性变更必须在 changelog 中声明，并在一个 Issue 内更新所有
  producer、consumer、contract test 与文档。

## 变更控制策略

- 不得直接在 `main` 开发 feature。
- 每个变更须有决策完整 Issue、短生命周期 branch、可衡量验收、test plan、文档影响和审查过的 diff。
- commit 遵循 Conventional Commits。
- 最终 PR HEAD 的本地完整 `scripts/verify.sh` 门禁最多运行一次，且 merge 前 required hosted CI 必须
  通过；hosted result 不能替代本地产品验证证据。节奏和真实 exit-status 规则见
  [变更生命周期](change-lifecycle.md#验证节奏)。
- 已发布 release tag 不可变，release history 不重写。
- 远端写入、仓库可见性变化和 branch-protection 的权限边界见
  [AGENTS.md](../../AGENTS.md)；本策略不另行定义角色授权。

## 文档策略

- `README.md` 描述实际能力和 setup，而不是 target claim。
- [产品](../product/v1.0-product-spec.md) 与[架构](../architecture/overview.md)文档必须标记 target
  behavior。
- 每个 Stable Interface 都要说明 field、invariant、ordering、timeout、cancellation 与 error。
- ADR 只记录重要权衡；它们通过 supersede 而非重写变更。
- [产品术语表](../product/glossary.md) 是 canonical 项目语言。
- `CHANGELOG.md` 记录显著 behavior 与 Interface 变化。
- Issue comment 保存需求、决策、验收、依赖和状态；PR comment 保存结果、最终 HEAD、测试摘要、验收映射、
  接口影响、回滚和残余风险。新 structural checker 的审批门禁见
  [测试策略](testing-strategy.md#受限结构检查器)。

## 测试策略

- pure parsing、validation 和 state transition 需要 unit test。
- ROS name、type、QoS、parameter、unit 和 TF ownership 需要 contract test。
- node composition 与 lifecycle 需要 launch/integration test。
- Gazebo、SLAM 与 Nav2 需要有界 headless smoke test。
- Voice 与 LLM 的本地产品测试使用确定性 in-memory fake；大本地模型是本地 milestone test，
  而非远端 CI dependency。
- invalid input、busy admission、timeout、cancel、stale result、Operational Stop 和 command-lease expiry
  均需显式 test。
- 每项 motion test 在 normal 与 cleanup path 都请求零速度并观测 stationarity。
- 只有 executable Mission 和 Agent behavior 存在时才引入 coverage threshold；lint-only line 没有有意义的
  coverage。

## 依赖策略

- `package.xml` 加 rosdep 是 ROS dependency 的 source of truth。
- 直接 runtime dependency 必须直接声明。
- 不可避免的 source dependency 使用固定 commit 的 `.repos` file。
- Python dependency 首次引入时须有审查过的 lock file。
- 本地 model weight 永不进入 Git。model manifest 记录 URL、version、精确 SHA-256、license、预期 size
  和支持 runtime。
- 再分发 code、voice 或 weight 前审查 third-party license。
- 外部 backup bundle 是 recovery evidence，不是 runtime dependency，也不是仓库内容。

## 支持平台

```text
Windows 11 host
WSL2 Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
```

1.0 前仅承诺该 matrix。远端 `required / ubuntu-24.04 / ros-jazzy` CI 只运行快速治理检查：
shellcheck、actionlint、治理契约和 Conventional Commit；它不安装 ROS，也不运行 package build、package test、
headless Gazebo、Voice 或 LLM 产品检查。所有 ROS/package build/headless Gazebo/Voice/LLM 产品检查均属于
本地 WSL 上 exact HEAD 的证据。WSL GUI 与 analog audio 仍为有界手工 milestone check。

## 安全与隐私

- credential 和 `.env` file 永不进入版本控制。
- recording、bag、含私有布局的生成 map、model weight、含私有数据的 prompt 和 raw runtime evidence 均
  保持本地，除非明确脱敏。
- DDS 与 model server 默认仅本地访问。
- LLM output 不可信，不能发布 motion。
- motion 需要 typed schema validation、allowlist、configured limit、monotonic command lease 与
  MotionGate admission。
- Operational Stop 是仿真运行控制，不得描述为认证 emergency stop。
- SROS2 暂缓用于本地仿真；在 networked 或 multi-user deployment 前必须重新考虑。
