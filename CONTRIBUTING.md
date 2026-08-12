# 为 VoiceNav Robot 贡献

GitHub Issue 是需求、决策、验收、依赖与状态的唯一记录。一个实现变更只对应一个 owning Issue、
一个独立分支和一个 Draft PR。角色、权限、Goal/Task、交付及恢复顺序由
[AGENTS.md](AGENTS.md) 唯一规定；本页只提供贡献入口与本地验证参考。

## 变更入口

1. 在 GitHub Issue 写明目标、非目标、验收、风险、接口影响、依赖、回滚和验证要求。
2. 基于当前 `main` 创建独立 worktree 与短生命周期分支。
3. 对可观察行为先记录聚焦 RED，再做最小 GREEN；仅在 GREEN 时重构。
4. 适用时更新用户文档、`CHANGELOG.md` 和 ADR。版本号与发布节奏依照
   [发布策略](docs/process/release-policy.md)。
5. 开发中运行聚焦检查并可小步本地提交；只在可审查里程碑 push。

不得直接在 `main` 开发，不得使用共享 checkout，PR 也不能替代 owning Issue。

## 分支与提交

使用能说明 Issue 和目的的短生命周期分支，例如：

```text
feat/25-mission-admission
fix/31-stop-race
test/37-tf-ownership
docs/25-retire-legacy-workflow
```

提交遵循 [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)：
`type(scope): 中文摘要`，其中 `scope` 可省略。允许的 primary type 为 `feat`、`fix`、`test`、
`docs`、`refactor`、`perf`、`build`、`ci`、`chore` 和 `revert`。每个提交只表达一个可审查的
变更理由，避免将大范围格式化与行为变更混在一起。

PR 的完整门禁检查 `base..head` 中全部提交；main push 对 `before..sha` 检查本次提交范围。全零
`before` fail closed，因此不依赖 branch protection 提供该覆盖。

## 远端治理 CI 与本地产品验证

`required / ubuntu-24.04 / ros-jazzy` 是稳定的 branch-protection required check，只运行
shellcheck、actionlint、治理契约和 Conventional Commit。它不运行 ROS、rosdep、colcon、Gazebo 或
`scripts/verify.sh`，通过不代表产品验证通过。

产品 PR 在 push 可审查里程碑前，必须在本地 WSL 对 exact HEAD 完成适用的 build、定向测试或一次
完整验证。命令、退出状态、结果和 artifact 路径写入唯一的 canonical evidence COMMENT；其归属与
交接见 [AGENTS.md](AGENTS.md)。治理或纯文档 Task 可按范围只运行治理检查。

## 检查节奏

开发期间可运行：

```bash
python3 tests/test_repository_contract.py
python3 scripts/check_repository.py --root .
```

产品变更在推送前对 exact HEAD 最多运行一次适用的完整本地门禁：

```bash
bash scripts/verify.sh
```

先记录该调用的真实退出状态。真实失败后不得在无变更的同一 HEAD 重跑；必须先有因果修复形成新的
exact HEAD。后续诊断命令成功不得覆盖失败门禁结论。验证细节见
[变更生命周期](docs/process/change-lifecycle.md)。

提交前审阅明确 pathspec 的内容：

```bash
git status --short
git diff
git diff --cached
```

不得提交生成的 workspace、模型权重、录音、bag、凭据或运行时证据。Windows/WSL 混合环境中优先
使用明确 pathspec，避免未经审阅的 `git add .`。

## 完成定义

变更完成时：关联 Issue 的验收已满足；相关 package 能从声明依赖构建；自动化测试覆盖新成功路径和
重要失败路径；适用的本地产品证据已记录；Stable Interface 影响已在 Issue 与文档中说明，必要时已有
ADR；没有生成数据、密钥、私有音频或模型权重；PR 正文只链接唯一 canonical evidence COMMENT，
不复制完整日志或逐提交日记。

接口和模块边界见[架构概览](docs/architecture/overview.md)。安全报告和部署边界见
[SECURITY.md](SECURITY.md)。
