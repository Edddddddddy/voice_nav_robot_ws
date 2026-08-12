# VoiceNav 仓库协作协议

`AGENTS.md` 是角色、权限、Goal/Task、交付状态和恢复顺序的**唯一权威**。
`CONTEXT.md` 只维护产品领域词汇；GitHub Issue 维护需求、决策、验收、依赖和状态；
ADR 维护产品与接口决策。其他流程文档只能链接本文件，不得复制或另立竞争性协议。

## 角色与责任

- **Manager** 维护父 PRD 与 Task 划分，最多同时管理两个互不依赖的会话；也是 GitHub
  写入的唯一传输负责人：负责
  Issue/PR 评论、标签、push、Draft PR、review、merge、tag 和 release。
- **Goal 与 Task**：一个 Goal 只绑定一个决策完整的 Issue 与一个 Draft PR，路径为
  `PRD -> Issue -> isolated Task -> Draft PR`。Manager 在分配前将 Goal 与验收写入
  Issue。Task 合并或终止后，该 Goal 停止；不得自动选择下一 Task，下一项必须由
  Manager 新建 Goal。
- **Worker** 只在一个基于当前 `origin/main` 的独立 worktree 中实现一个决策完整的
  Issue。Worker 使用 Luna/Max（max reasoning），遵循聚焦 RED、最小 GREEN、保持
  GREEN 后重构，并在本地提交。
- **Reviewer** 是全新、只读、针对 exact HEAD 的 PR 审查者，使用 Sol/xhigh。其依据
  Issue 契约和完整 diff 提交 P0–P3 简体中文证据；不得修改 Worker 分支或直接写 PR
  评论。Manager 使用 `COMMENT` 持久化审查证据。
- 所有新笔记、GitHub 评论、Issue/PR 正文、review、面向人的证据和交接均使用简体中文。
  命令、路径、标识符、协议字段、标准名和为精度必须保留的技术名称保持原样。

- 仓库级 Skill 的唯一版本化来源是根目录 `.agents/skills`；个人目录中的同名 Skill 不是项目
  权威。需求澄清使用 `$voice-nav-requirements`，实施使用 `$voice-nav-worker`，审查使用
  `$voice-nav-review`。Manager 分派角色 Task 时必须在正文显式写出对应 `$skill-name`，
  不得依赖隐式触发。
- 同一子任务最多选择一份 VoiceNav 角色 Skill；可按需组合 `tdd`、`diagnosing-bugs`、
  `resolving-merge-conflicts`、`token-economy` 等通用 Skill，但通用 Skill 不得扩大 Issue 范围或角色权限，也不能替代角色 Skill。
- 指令优先级为：系统/用户明确指令 > 仓库 `AGENTS.md` > 已批准 Issue/ADR > VoiceNav 角色 Skill > 通用 Skill 建议。任何 Skill 都不得绕过 Manager-only GitHub
  写入、隔离 worktree、本地产品验证、fresh exact-HEAD Review 或 P0/P1 合并阻断。
- 调用前确认 `SKILL.md` 可发现且可读；缺失时不得声称已使用，先记录并按本协议保守执行。
  若影响角色或安全验收则交接 `blocked`；更新后仍未发现新版本时重启或新建 Task。

## 权限与持续授权

- 只有 Manager 调用 GitHub 写 API 或传输分支、评论、审查、合并、标签和发布。Worker/
  Reviewer 不登录、不打开认证浏览器、不读取或复制 token，也不因普通 Git、GitHub、
  WSL/ROS/build/test 或聚焦检查向用户逐次请求许可。
- 用户已提供一次持续授权：在已批准的 Issue 范围内，可执行正常 Git/GitHub/WSL/build/
  test 操作。只有破坏性、跨范围，或平台强制交互时才请求用户决定。
- 发生 auth/403、shared-index 或命令边界失败时，精确保留 `cwd`、`command`、
  `timeout`、预期产物、本地 `HEAD`、结果和证据；Manager 执行该精确有界命令
  或已批准等价命令。这是传输限制，不是产品阻塞。
- 用户介入仅用于平台强制的交互确认，或无法排除机器级/破坏性影响的只读审计。

## 交付与证据

交付采用两阶段传输，不进行 polling：

1. 完成本地提交和证据后，Worker 或 Reviewer 发送完整简体中文
   `VOICE_NAV_HANDOFF: ready|blocked|reviewed`，包含 exact `issue`、`pr`、`thread`、
   `head`、`body`、`cwd`、命令/结果与 `local_artifacts`。被阻塞的交接必须写明尝试、
   最小缺失决策、选项和建议。
2. Manager 将完整证据持久化到 GitHub，并直接返回
   `VOICE_NAV_PERSISTED` 与规范 URL。
3. 仅在收到该响应后，才发送紧凑的
   `VOICE_NAV_EVENT: blocked|completed|reviewed`。不得伪造或复用 URL，也不得提前发送
   最终事件；仅最终的 `VOICE_NAV_EVENT` 使用紧凑 envelope。未解决的 P0/P1 审查发现
   阻止合并；最终的 `VOICE_NAV_EVENT: reviewed` 必须填写 `decision_needed`。仅当不需要任何
   决策或行动时，才使用 `none`。

```text
VOICE_NAV_HANDOFF: ready|blocked|reviewed
issue: #NN
pr: #NN|none
thread: <thread-id>
head: <exact-sha|none>
body: <完整简体中文证据>
cwd: <absolute path>
command: <exact command>
timeout: <milliseconds>
expected_artifact: <path or none>
results: <summary>
evidence: <URL or immutable Git object or none>
local_artifacts: <paths or none>

VOICE_NAV_EVENT: blocked|completed|reviewed
issue: #NN
pr: #NN|none
thread: <thread-id>
head: <exact-sha|none>
evidence: <canonical URL>
decision_needed: <none or required decision>
```

Worker 在本地提交并验证后交接；Manager 负责 push 及创建/更新 PR。开发中可小步本地
提交，只在完成本地验证的可审查里程碑 push。产品 Task 在 push 前须于本地 WSL 对 exact
HEAD 运行一次适用的 build、定向测试或 `bash scripts/verify.sh`，并记录真实结果。真实
失败后不得在无变更的同一 HEAD 重跑。远端 required CI 只验证治理，不代表产品验证通过。
Review 修复可聚合后再 push；合并前 rebase，保持单 Task 一提交的线性历史。交接必须保留
回滚、接口影响、残余风险和 exact evidence。

## 恢复顺序

1. 读取 `manager-state.yaml`/Task YAML，再读取已分配的 Issue/PR、父 PRD、依赖和已持久化
   证据。
2. 读取本文件、`CONTEXT.md`、`docs/agents/README.md` 和适用 ADR；后两者只是参考，不能
   与本协议竞争。
3. 确认仓库根目录、独立 worktree、分支、`HEAD` 与 `origin/main`，并保留无关变更。
4. 从已持久化记录重建验收映射。若需求、接口、阈值、依赖或范围决策缺失，不得实施猜测性
   变更；向 Manager 交接 `blocked` 证据。

## 禁止事项

- 禁止 subagents、thread/CI polling、循环监控、共享 checkout，或在另一 Task 分支工作。
- Worker/Reviewer 不得 merge、tag、release、force-push，也不得用实现猜测替代缺失的产品
  决策。未经 Issue 明确批准，不得添加 AST/source-shape/full-file-fingerprint 检查。
- 仓库流程 Task 未获 Issue 明确授权时，不得修改 ROS 接口或 Runtime 行为。
