# 为 VoiceNav Robot 贡献

GitHub Issue 是需求、决策、验收标准、依赖和状态的唯一记录。每项实现变更只对应一个 owning Issue、一个隔离分支和一个 Draft PR。

一个 Goal 恰好绑定一个 decision-complete Issue 和一个 Draft PR；Task 合并或结束后 Goal 停止，绝不自动选择下一 Task，下一项工作只能由 Manager 新建 Goal。用户已持续授权已批准 Issue 范围内的普通 Git/GitHub/WSL/build/test 操作；仅破坏性、跨范围或平台强制交互需要再次请求用户。

## 变更流程

1. 在 GitHub Issue 写明目标、非目标、验收、风险、接口影响、依赖、回滚和验证要求。
2. 基于当前 `main` 创建隔离 worktree 和短生命周期分支。
3. 每次实现一个可观察行为：先记录聚焦 RED，再最小 GREEN，最后只在保持 GREEN 时重构。
4. 仅在适用时更新用户文档、`CHANGELOG.md` 和 ADR。
5. 开发中运行聚焦仓库检查并可小步本地提交；仅在可审查里程碑推送。推送前在本地 WSL 对 exact HEAD 执行一次适用的产品验证；远端 required CI 只验证治理，不代表产品验证通过。
6. Manager 创建 Draft PR，使用 `Closes #NN`，且只维护一份 canonical evidence summary：验收映射、最终 HEAD、检查摘要、接口影响、回滚和残余风险。原始日志留在 Git 外，以 artifact 路径引用，不逐提交粘贴日志或重复 Issue 正文。
7. 仅在独立审查、远端治理 CI 与本地产品证据齐备后执行 rebase merge；Review 修复可聚合后再推送，合并前 rebase 并保持单 Task 一提交的线性历史。

不得直接在 `main` 开发；不得使用共享 checkout；PR 不能替代其 owning Issue。

Worker/Reviewer 不直接写 PR comment；Reviewer 只交接 exact-HEAD P0–P3 简体中文证据，Manager 使用 `COMMENT` 写入 GitHub。

## 分支与提交

使用能表明 Issue 和目的的短生命周期分支，例如：

```text
feat/25-mission-admission
fix/31-stop-race
test/37-tf-ownership
docs/25-retire-legacy-workflow
```

提交必须符合 [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) 的 `type(scope): 中文摘要`，其中 `scope` 可省略。允许的 primary type 为 `feat`、`fix`、`test`、`docs`、`refactor`、`perf`、`build`、`ci`、`chore` 和 `revert`。每个提交只表达一个可审查的变更理由，避免把大范围格式化和行为改动混在一起。

PR 的完整门禁检查 `base..head` 中全部提交；main push 对 `before..sha` 检查本次提交范围，全零 `before` fail closed，因此不依赖 branch protection 提供该项覆盖。

## 远端治理 CI 与本地产品验证

`required / ubuntu-24.04 / ros-jazzy` 是稳定的 branch-protection required check，只运行 shellcheck、actionlint、治理合同和 Conventional Commit。它不运行 ROS、rosdep、colcon、Gazebo 或 `scripts/verify.sh`，通过不代表产品验证通过。

产品 PR 在推送可审查里程碑前必须在本地 WSL 对 exact HEAD 完成适用的 build、定向测试或一次完整验证。Manager 把命令、退出状态、结果和 artifact 路径写入唯一的 canonical evidence COMMENT；治理或纯文档 Task 可按范围只运行治理检查。

## 聚焦检查与完整门禁

开发中运行：

```bash
python3 -m unittest tests.test_repository_contract
python3 scripts/check_repository.py --root .
```

每个推送候选 exact HEAD 的本地产品完整门禁（最多一次）：

```bash
bash scripts/verify.sh
```

先记录该调用的真实退出状态；真实失败后不得在无变更的同一 HEAD 重跑，必须先有因果修复形成新 exact HEAD；后续诊断命令成功不得覆盖失败门禁的结论。远端治理 CI 不代表产品验证通过。完整规则见[变更生命周期](docs/process/change-lifecycle.md)。提交前检查将要提交的内容：

```bash
git status --short
git diff
git diff --cached
```

生成的 workspace、模型权重、录音、bag、凭据和运行时证据不得提交。混合 Windows/WSL 环境中优先使用显式 pathspec，避免未经审阅的 `git add .`。

## 完成定义

变更完成的前提是：关联 Issue 的验收已满足；相关包能从声明依赖构建；自动化测试覆盖新成功路径和重要失败路径；每个候选 exact HEAD 最多一次 `bash scripts/verify.sh` 并记录真实状态；Stable Interface 影响在 Issue 和文档中说明；必要时有 ADR；无生成数据、密钥、私有音频或模型权重；PR 的 canonical evidence summary 完整且作者审阅了完整 staged diff。

## 接口、架构和发布

- `voice_nav_interfaces` 不依赖项目业务包。
- `voice_nav_agent` 不发布车轮或最终速度命令；LLM 输出不可信，必须经过强类型 Mission gate。
- `voice_nav_audio` 不依赖 Nav2、SLAM 或 Gazebo；`voice_nav_mission` 不依赖 Gazebo；`voice_nav_sim` 只包含仿真适配器；`voice_nav_bringup` 只组合模块和配置。
- 每条动态 TF 边和最终运动输出只有一个 owner。
- ADR 只记录代价高、缺少上下文会令人意外、且存在真实权衡的决定。
- 发布代表连贯的能力里程碑，不代表单个分支或提交；遵循[发布策略](docs/process/release-policy.md)。

仓库当前只有一名维护者。CI 和已解决的对话仍是必须条件；PR 作者的批准不能替代独立审查。
