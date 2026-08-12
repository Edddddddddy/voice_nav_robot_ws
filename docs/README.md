# VoiceNav Robot 文档索引

此目录区分“已验证的当前行为”与“已批准的 v1.0 目标”。目标文档是规范性设计契约，
不表示当前源码已经实现其全部描述。

## 产品

- [v1.0 产品规格](product/v1.0-product-spec.md)
- [产品术语表](product/glossary.md)
- [工程资源](product/resources.md)

## 架构

- [架构概览](architecture/overview.md)
- [Mission Runtime 接口](architecture/mission-runtime-interface.md)
- [Voice Interface 契约](architecture/voice-interface.md)
- [安全与运动契约](architecture/safety-and-motion-contract.md)
- [TF 与运行模式](architecture/tf-and-operating-modes.md)
- [语音与 Agent 契约](architecture/voice-and-agent.md)

## 过程

- [变更生命周期](process/change-lifecycle.md)
- [质量策略](process/quality-policy.md)
- [测试策略](process/testing-strategy.md)
- [发布策略与路线图](process/release-policy.md)
- [问题复发控制](process/problem-learning.md)
- [已知运行陷阱](process/known-pitfalls.md)
- [Crash-stop 验收手册](process/crash-stop-runbook.md)
- [第三方 LLM 通知](process/third-party-llm-notices.md)

## 治理

- [仓库协作协议](../AGENTS.md)
- [架构决策记录](adr/)
- [GitHub 工作索引](agents/README.md)

ADR 是历史决策记录。不要为了让旧选择显得“当前”而重写它；若决策变化，应新增一份
superseding ADR。

## 状态词汇

- **当前**：仓库中已有可验证的行为。
- **目标 v1.0**：已批准但可能尚待实现的行为。
- **证据**：与 GitHub Issue、PR、CI run 或不可变 Git 对象关联的命令、结果或审查产物。
