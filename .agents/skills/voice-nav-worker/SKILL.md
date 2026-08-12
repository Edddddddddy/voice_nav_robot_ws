---
name: voice-nav-worker
description: 在独立 worktree 中实现或修复一个决策完整的 VoiceNav Issue，完成本地验证与提交后向 Manager 交接；不用于需求澄清或只读审查。
---

# VoiceNav Worker

先读取仓库根目录 `AGENTS.md`。角色、权限、恢复、验证与交接状态机只由该文件定义；
本 Skill 只保留实施步骤。

1. 只处理一个已分配 Issue。读取父 PRD、决策、canonical evidence、适用接口和 ADR。
2. 确认当前 worktree 独立、clean、基于当前 `origin/main`，且分支只属于该 Task；保留所有
   无关改动。
3. 若需求、接口、阈值、依赖或范围决策缺失，不实施猜测性变更；形成包含最小决策、选项和
   建议的简体中文 `blocked` 交接。
4. 按可观察行为执行聚焦 RED、最小 GREEN、保持 GREEN 后重构。优先测试最高稳定公共接口；
   未经 Issue 授权不添加 AST/source-shape/full-file-fingerprint 检查。
5. 开发中运行聚焦检查。push 前在本地 WSL 对 exact HEAD 运行一次 Issue 所需的适用 build、
   定向测试或 `bash scripts/verify.sh`，保存完整日志到 Git 外 artifact，并如实记录失败。
   远端 required CI 只验证治理，不代表产品验证通过；不得以远端结果代替本地产品证据。
6. 复核最终 diff 与验收映射，整理为单一 Conventional Commit、clean worktree，并按根目录
   `AGENTS.md` 的“交付与证据”协议向 Manager 交付 exact HEAD、命令/结果、接口影响、
   回滚、残余风险和 artifact 路径。
7. 不直接 push、写 GitHub、merge、tag 或 release；等待 Manager 的持久化响应后，才发送
   根协议要求的紧凑最终事件。

不得使用共享 checkout、另一 Task 分支、无变更重跑来掩盖失败，或以 Skill 扩大 Issue 范围。
