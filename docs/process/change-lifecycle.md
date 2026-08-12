# 变更生命周期

GitHub Issue 是需求、决策、验收、依赖和状态的规范来源。一个变更通过一个独立分支和一个与
Issue 关联的 pull request 交付。本文件仅作为验证节奏和证据记录参考；
[AGENTS.md](../../AGENTS.md) 是角色、权限、流程、交付和恢复的唯一权威，不重复或覆盖它的规则。

```text
GitHub Issue
   -> 独立的短生命周期分支
   -> 实现 + 聚焦测试 + 文档
   -> 本地验证
   -> 完整 diff 审查 / Draft PR / CI
   -> rebase 合并到 main
   -> 功能就绪时的发布里程碑
```

## 证据归属

Issue 负责需求、决策、验收标准、依赖和 workflow 状态；PR 负责可观察结果、验收映射、最终 HEAD、
聚焦与完整检查摘要、接口影响、回滚和残余风险。Goal/Task 绑定、角色权限与交接顺序见
[AGENTS.md](../../AGENTS.md)。

每个 Task 只维护一份规范证据 COMMENT；其归属、持久化和交接顺序以
[AGENTS.md](../../AGENTS.md) 为准。PR 正文仅链接该 COMMENT，不复制摘要、逐提交开发日志、完整原始日志或
Issue 正文。原始日志保留在 Git 外，只用 artifact 路径引用；唯一摘要只在 COMMENT 更新，并记录简要命令、
退出状态与结果。

## 各阶段证据

| 阶段 | 最小持久证据 |
| --- | --- |
| Issue | Goal、非目标、验收、风险、接口影响、依赖、回滚、验证 |
| 设计 | Stable Interface 文档；重要权衡对应 ADR |
| 实现 | 最深稳定 Interface 上的源码、配置与测试 |
| 验证 | exact command、真实退出状态、测试摘要和所需人工证据 |
| 审查 | 完整 staged diff、验收映射和已解决的 review finding |
| 发布 | 版本、changelog、不可变 tag 和关联验收证据 |

## 远端治理与本地产品验证

`required / ubuntu-24.04 / ros-jazzy` 只运行快速治理：shellcheck、actionlint、治理契约和
Conventional Commit。它不安装 ROS，也不运行 rosdep、colcon、Gazebo 或 `scripts/verify.sh`，因此通过
不代表产品验证通过。

产品 PR 在 push 可审查里程碑前，在本地 WSL 对 exact HEAD 运行适用的 build、定向测试或一次
`bash scripts/verify.sh`。将命令、真实退出状态、结果与 artifact 路径更新到唯一 canonical
evidence COMMENT；治理或纯文档 Task 可按范围只运行治理检查。开发可以小步本地提交，Review 修复聚合后
再 push；合并前 rebase 并保持单 Task 一提交的线性历史。

## 三种粒度

- **Issue**：一项可独立回滚、具有可观察验收的变更。
- **Commit**：一个连贯的变更理由与审查单元。
- **Release**：一组不可变的可交付能力，不是单个 commit 或 branch。

## 验证节奏

PR 的完整门禁对 `base..head` 的全部提交运行 Conventional Commit 检查，并要求冒号后的摘要包含简体中文。
main push 对 `before..sha` 检查本次提交范围；全零 `before` fail closed，因此不依赖 branch protection
提供该覆盖。

实施中按需运行聚焦仓库检查：

```text
python3 tests/test_repository_contract.py
python3 scripts/check_repository.py --root .
```

可按变更行为增加检查，但它们必须针对最窄稳定 Interface，不能替代仓库契约检查。

每个准备 push 的 exact HEAD 在本地最多执行一次适用的完整产品门禁。真实失败后不得在无变更的同一 HEAD
重跑，必须先有因果修复形成新的 exact HEAD。远端 required CI 不代表产品验证通过。

```text
bash scripts/verify.sh
```

该调用的 exit status 就是结果。运行独立诊断前先记录真实状态；后续成功命令不得覆盖或重新解释失败门禁。

## 停止并重新划分范围

以下是升级触发器而不是配额：

- 单个 Task diff 中新增 test/checker implementation 超过新增 product implementation 的三倍。只比较相关的
  新增 implementation，排除文档、生成输出和纯空白变更；不得重写既有 product test 以满足比例。
- 十个连贯 commit 完成后仍未满足任何验收标准。

任一触发器出现时，停止实现，在 Issue 记录证据，并先拆分或重塑工作。该规则只在本节解释；其他文档可
链接本节，但不得复制其解释。

## Git 安全边界

- `.gitignore` 仅影响未跟踪文件，不会移除已跟踪数据。
- `git rm --cached` 移除 index entry；不带 `--cached` 还会移除工作树文件。
- 在 Windows/WSL 混合 workspace 中使用明确 pathspec；不要以 destructive reset 或 clean 命令消除不确定性。
- 已发布的 tag 与 release history 永不重写。

## PR 前最终检查

```bash
git status --short
git diff
git diff --cached
```

参考 [REP-2004](https://reps.openrobotics.org/rep-2004/)、
[Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) 和
[Semantic Versioning](https://semver.org/spec/v2.0.0.html)。
