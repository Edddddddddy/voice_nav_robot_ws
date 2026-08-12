# 问题复发控制

VoiceNav Robot 将一次 failure 视为未完成，直到捕获有用诊断事实，并在可行时将其转为可执行 guardrail。
按主题归组的[已知陷阱参考](known-pitfalls.md)保留可复用规则；Issue 与 PR 保留具体变更和证据。

## 词汇

- **Occurrence**：具有精确命令、环境、输出和仓库 head 的一次已观察失败。
- **Pitfall**：可以在另一变更中复发的泛化失败模式。
- **Root cause**：证据支持的最窄因果陈述；未经证明的实现特定解释仍是假设。
- **Guardrail**：阻止或发现复发的自动化测试、静态契约、有界运行时检查或进程检查。

## 必经循环

```text
观察精确症状
  -> 保留命令 / 退出状态 / 环境 / HEAD
  -> 复现或界定不确定性
  -> 陈述最窄根因
  -> 添加最近的可执行护栏
  -> 更新 Issue、PR 证据和相关陷阱
  -> 在最终 exact HEAD 验证
```

1. 编辑前先捕获事实。将 warning 与决定性 failure 分开，并记录命令是否改变仓库或外部 state。
2. 在最小稳定 seam 复现：先 pure unit test，再 ROS launch test，边界需要时再运行完整 Gazebo product test。
3. 对 behavior defect 使用 tests-first correction；当它有助于审查时保留 RED/GREEN 因果链。
4. 在 ownership boundary 设置 guardrail：Core invariant 放在 Core test，ROS graph behavior 放在 Node test，
   composition behavior 放在 product test，repository shape 放在 repository contract。
5. 每次 product-code change 后替换临时证据。祖先 commit 上的 passing run 只能用于诊断，不是最终验收证据。

## 复发升级

| 信号 | 必需响应 |
| --- | --- |
| 第一次代码缺陷 | 回归测试加简短陷阱条目 |
| 同一模式再次出现 | 在共享 seam 增加自动化/静态护栏 |
| 模式穿透护栏 | 审查模块边界与测试保真度 |
| 架构权衡改变 | 创建或替代 ADR |

文档本身不是可复现 safety 或 release failure 的充分响应。命令执行问题应写入归组参考与更安全 command
template，而不是写入 product test。

## Pitfall 条目契约

`known-pitfalls.md` 的每条条目保持稳定 `PIT-NNNN` anchor、symptom、决定性 discriminator、已支持的 cause
和最短安全 diagnostic 或 guardrail。相同 cause 再次出现时更新既有 ID；仅当 cause 或 prevention boundary
实质不同才创建新 ID。不得移除稳定 anchor，除非迁移其 active link。

## 反模式

- 只根据最后一条 warning line 推断 cause；
- 以固定 sleep 隐藏 asynchronous race；
- 从一个 typed transient state 向所有 mismatch 扩大 retry；
- 为让一次 invocation 通过而修改 global Git 或 WSL configuration，却不了解 ownership；
- exact final commit 尚不存在就声称最终验证；
- 将 incident log 复制到多个文档，直到它们彼此矛盾。
