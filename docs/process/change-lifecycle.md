# 工程变更生命周期

VoiceNav Robot reference

```
Work Item
   ↓
短生命周期 Branch
   ↓
实现 + 自动测试 + 文档
   ↓
本地统一质量门禁
   ↓
Diff 自审 / PR / CI
   ↓
合并 main
   ↓
达到里程碑时才 Release
```

## 每层留下什么证据

| 阶段 | 最小产物 |
| --- | --- |
| Work Item | 目标、非目标、验收条件、风险、测试计划 |
| 设计 | Interface 文档；真正重大取舍才写 ADR |
| 实现 | 源码、配置和最靠近稳定 Interface 的测试 |
| 验证 | 统一命令、退出状态、测试摘要、必要的人工证据 |
| 评审 | 完整 staged diff、Definition of Done |
| 发布 | 版本、CHANGELOG、不可变 tag、发布验收 |

## 三个不要混淆的粒度

- **Work Item**：一个可验收变化，例如接入 ROS bridge。
- **Commit**：一个连贯的修改理由，可作为评审单位。
- **Release**：一组可交付能力的不可变版本，不等于一课。

## 什么时候写 ADR

以下三项必须同时成立：

1. 更换选择的成本明显；
1. 没有上下文时，未来维护者会疑惑；
1. 确实比较过有意义的替代方案。

普通参数、文件位置和易于替换的库选择不需要 ADR。它们应留在代码、配置、测试或 Interface 文档中。

## Git 安全边界

- `.gitignore` 只影响未跟踪文件，不会自动遗忘已跟踪文件。
- `git rm --cached` 只从索引移除；省略 `--cached` 会同时删除工作区文件。
- 混合工作区使用明确 pathspec 暂存，不盲目执行 `git add .`。
- 禁止使用 `reset --hard` 或 `clean -fdx` 处理不清楚的工作区。
- 发布后的 tag 与历史不重写。

## 不可自引用的交付身份

Git tree 不能包含由该 tree 在未来生成的最终本地或 pushed HEAD、该 HEAD 的
门禁结果、tag object、rebase 后 public commit 或最终 CI 身份。课程 start tag
必须在功能分支创建前从公开 `main` 发布，因此可安全写入 Work Item；solution
tag 必须在 reviewed PR 合并后从新的公开 `main` 发布，因此不能成为其目标
tree 内的强制字段。

课程或 Release Work Item 的 PR 默认使用 `Refs #NN`。树内只记录提交时已经
存在且可验证的事实；最终提交完成后，在 exact local/pushed HEAD 上运行门禁，
把该 HEAD、结果、merge、public tree、required CI 和最终 tag 的精确身份写入
PR 与 Issue closure comment，而不再提交 evidence-only 修改。树内 Work Item
可在 repository acceptance 完成时标记 `Done`，linked Issue 则继续保持打开并
承担 post-merge delivery ledger。发布不可变 artifact、核验本地/远端对象并在
Issue 留下 closure comment 后，再通过 closed-state event 关闭 Issue；不再创建
一个只为把未来身份抄回仓库的递归 ledger PR。

## 提交前最小检查

```
git status --short
git diff
bash scripts/verify.sh
git diff --cached
```

依据：[REP-2004](https://reps.openrobotics.org/rep-2004/)、[Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) 与 [Semantic Versioning](https://semver.org/spec/v2.0.0.html)。
