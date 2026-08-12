---
name: voice-nav-requirements
description: 澄清 VoiceNav 产品或工程需求，将其整理为决策完整的 PRD 与可独立执行 Issue；不用于实现、PR 修复或代码审查。
---

# VoiceNav 需求

先读取仓库根目录 `AGENTS.md`。角色、权限、Goal/Task、交接状态机和 GitHub 写入规则
只由该文件定义；本 Skill 只保留需求工作步骤。

1. 读取 `CONTEXT.md`、适用 ADR、当前 Issue/PR、公开接口与已有证据；先发现事实，再提出
   会改变范围、接口、风险或验收的最小问题，并附推荐答案。
2. 在交付前明确目标、使用者、当前行为、范围、非目标、约束、接口、失败行为、依赖、
   测试接缝、迁移、回滚及可度量结果。
3. 将一个父 PRD 拆为可独立验收、独立回滚的纵向 Task。每个 Task 只交付一个可观察能力，
   不把产品行为、架构或阈值选择留给 Worker。
4. 输出简体中文的 PRD/Issue 草案，结构遵循
   [Issue 形状](references/issue-templates.md)。保留命令、接口名、协议字段和必要技术名称。
5. 不直接写 GitHub；把草案交给 Manager 持久化。后续决策也先形成完整中文证据，再由
   Manager 更新唯一 canonical evidence COMMENT。

不得实现代码、修改产品文件、分派 Worker、调用 Reviewer 或替 Manager 执行 GitHub 写入。
