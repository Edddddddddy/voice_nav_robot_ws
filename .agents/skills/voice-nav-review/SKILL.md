---
name: voice-nav-review
description: 对一个 VoiceNav Draft PR 的 exact HEAD 进行全新只读 P0-P3 审查并向 Manager 交接中文证据；不用于实现或修改分支。
---

# VoiceNav Reviewer

先读取仓库根目录 `AGENTS.md`。角色、权限、恢复和交接状态机只由该文件定义；本 Skill
只保留审查步骤。

1. 在全新只读上下文中审查一个 Draft PR。读取子 Issue、父 PRD、决策、canonical evidence、
   merge base 到 exact HEAD 的完整 diff、公开接口与适用 ADR。
2. 先验证范围和每条验收标准，再检查实现质量。覆盖变更的生产、测试、配置、launch 和文档。
3. 应用 [ROS 2 审查清单](references/ros2-review-checklist.md)。测试必须证明可观察行为，并能
   因目标回归而失败。
4. findings first：按 P0–P3 提供紧凑文件/行、触发场景、影响和修正方向。P0/P1 阻断合并；
   无 finding 时也明确写出 P0–P3=0、未验证项与残余风险。
5. 只运行验证 finding 或证据缺口所需的聚焦只读检查，不把远端治理 CI 当作产品证据。
6. 交接前重新读取远端 PR HEAD；若与 reviewed HEAD 不同，拒绝提交 stale Review。
7. 向 Manager 交接完整简体中文 Review，由 Manager 使用 COMMENT 持久化；等待其响应后才
   发送根协议要求的紧凑最终事件。

不得编辑、提交、push、直接写 PR Review/COMMENT、轮询、merge、tag 或 release。
