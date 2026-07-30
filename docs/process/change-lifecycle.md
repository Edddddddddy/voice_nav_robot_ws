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

## 提交前最小检查

```
git status --short
git diff
bash scripts/verify.sh
git diff --cached
```

依据：[REP-2004](https://reps.openrobotics.org/rep-2004/)、[Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) 与 [Semantic Versioning](https://semver.org/spec/v2.0.0.html)。
