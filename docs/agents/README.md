# GitHub 工作索引

[AGENTS.md](../../AGENTS.md) 是角色、权限、恢复和交付协议的唯一权威；
[CONTEXT.md](../../CONTEXT.md) 是产品词汇表。本页只索引 GitHub Issue/PR 记录和标签，
不重复交接协议。

## Issue 与 PR 索引

- GitHub Issue 记录需求、决策、验收标准、依赖和状态；PR 以 `Closes #NN` 链接其 Issue。
- Issue 与 PR 评论保存决策、阻塞原因、验证摘要和已持久化证据。local notes 与 Task YAML 可以帮助恢复，
  但不能覆盖 Issue、PR 或 ADR。
- worktree、分支、Goal/Task 绑定、角色权限和交接时序不在本页定义，统一见
  [AGENTS.md](../../AGENTS.md)。

## 标签

保留一个类型标签和至多一个流程状态标签：

| 标签 | 含义 |
| --- | --- |
| `type:prd` | 父产品需求文档 |
| `type:task` | 可独立回滚的实现 Task |
| `ready-for-agent` | 决策完整，可分配 |
| `in-progress` | 正在实施 |
| `blocked` | 缺少具名决策或依赖 |
| `review-needed` | Draft PR 已可独立审查 |
| `verified` | 验收证据和适用检查完整 |

状态变化与最小阻塞原因写入 Issue 评论。

## 外部贡献

外部 PR 不是需求或决策输入面。维护者先确定 canonical Issue，在其中记录范围与验收，再将
贡献链接到该 Issue。

## 简短示例

```text
Issue 链接: #NN
验收证据: command/result 或 immutable Git object
状态: ready-for-agent
```

```text
PR: Closes #NN
结果: acceptance criterion → file → evidence
```
