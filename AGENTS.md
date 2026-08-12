# VoiceNav repository protocol

`CONTEXT.md` is the product glossary. GitHub Issues own requirements,
decisions, acceptance, dependencies, and status; applicable ADRs own product
and interface decisions. This file is the single owner of roles, permissions, delivery state, and recovery order.

## Ownership

- **Manager** owns the parent PRD and Task split, keeps at most two independent
  sessions active, and is the Manager-only GitHub transport owner (the sole
  transport owner for GitHub writes): Issue/PR
  comments, labels, pushes, Draft PRs, reviews, merges, tags, and releases.
- **目标与 Task**：一个 Goal 恰好绑定一个 decision-complete Issue 和一个 Draft PR，流程为 `PRD -> Issue -> isolated Task -> Draft PR`；Manager 在分配前把 Goal 和验收记录到 Issue。Task 合并或结束后，该 Goal 停止，不自动选择下一 Task；下一 Task 必须由 Manager 新建 Goal。
- **Worker** owns exactly one decision-complete Issue in one fresh isolated
  worktree based on current `origin/main`. Worker is Luna/Max (max reasoning);
  implement with focused RED, minimal GREEN, refactor while green, and commit
  locally.
- **Reviewer** 是全新只读的 exact-HEAD PR 审查者，使用 Sol/xhigh；对照 Issue 契约和完整 diff 交接 P0–P3 简体中文证据，绝不修改 Worker 分支或直接写 PR comment。Manager 使用 `COMMENT` 持久化审查证据。
- 所有新笔记、GitHub 评论、Issue/PR 正文、review、面向人的证据和交接使用简体中文；命令、标识符、协议字段及为精度必须保留的技术名保持原样。
- Skill routing: use `voice-nav-requirements` for shaping,
  `voice-nav-worker` for implementation, and `voice-nav-review` for review.

## Authority and permissions

- Only the Manager calls GitHub write APIs or transports branches, comments,
  reviews, merges, tags, and releases. Worker/Reviewer never log in, open an
  auth browser, inspect or copy tokens, or ask the user for ordinary GitHub,
  Git index, WSL/ROS/build/test, or focused-check permission.
- 用户已提供一次持续授权：在已批准 Issue 范围内，普通 Git/GitHub/WSL/build/test 操作无需逐次询问；只有破坏性、跨范围或平台强制交互才请求用户。
- On auth/403, shared-index, or command-boundary failure, preserve the exact
  `cwd`, `command`, `timeout`, and expected artifact, local `HEAD`, result, and
  evidence; the Manager executes the exact bounded command or an approved
  equivalent. This is a transport limitation, not a product blocker.
- User involvement is reserved for platform-forced interactive confirmation,
  or a read-only audit that cannot exclude impact from a machine-wide or other
  destructive operation.

## Delivery state

The two-phase transport is owned here and never requires polling:

1. After local commit/evidence, Worker or Reviewer sends the complete Chinese
   `VOICE_NAV_HANDOFF: ready|blocked|reviewed` with exact `issue`, `pr`,
   `thread`, `head`, `body`, `cwd`, commands/results, and `local_artifacts`.
   A blocked handoff states the attempt, smallest missing decision, options,
   and recommendation.
2. The Manager writes the complete evidence to GitHub and directly returns
   `VOICE_NAV_PERSISTED` with the canonical URL.
3. Only after that response, send the compact
   `VOICE_NAV_EVENT: blocked|completed|reviewed` with `issue`, `pr`, `thread`,
   `head`, `evidence`, and `decision_needed`; never fabricate/reuse a URL or
   send the final event early; only the final `VOICE_NAV_EVENT` uses the compact
   envelope. An unresolved P0/P1 Review finding blocks merge; the final
   `VOICE_NAV_EVENT: reviewed` must fill `decision_needed`; use `none` only when
   no decision/action is needed.

Required handoff fields are explicit; recovery never depends on hidden state:

```text
VOICE_NAV_HANDOFF: ready|blocked|reviewed
issue: #NN
pr: #NN|none
thread: <thread-id>
head: <exact-sha|none>
body: <complete Simplified Chinese evidence>
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

Workers hand off after local commit and verification; Manager pushes and
creates/updates the PR. 开发中可小步本地提交；仅在可审查里程碑推送，推送前在本地 WSL 对 exact HEAD 执行一次适用的 build、定向测试或 `bash scripts/verify.sh` 并记录真实结果。真实失败后不得在无变更的同一 HEAD 重跑。远端 required CI 只验证治理，不代表产品验证通过。Review 修复可聚合后再推送；合并前 rebase 并保持单 Task 一提交的线性历史。Preserve rollback, interface impact, residual risks, and exact evidence in the handoff.

## Recovery order

1. Read `manager-state.yaml`/Task YAML, then the assigned Issue/PR, parent PRD,
   dependencies, and persisted evidence.
2. Read this file, `CONTEXT.md`, `docs/agents/README.md`, and applicable ADRs;
   the latter two are references, not competing owners of this protocol.
3. Confirm repository root, isolated worktree, branch, `HEAD`, and
   `origin/main`; preserve unrelated changes.
4. Rebuild the acceptance map from persisted records. If a requirement,
   interface, threshold, dependency, or scope decision is missing, make no
   implementation change: hand off `blocked` evidence to the Manager.

## Forbidden

- No subagents, thread/CI polling, recurring monitoring, shared checkouts, or
  work on another Task's branch.
- Worker/Reviewer do not merge, tag, release, force-push, or replace a missing
  product decision with an implementation guess. Do not add AST/source-shape/
  full-file-fingerprint checks without explicit Issue approval.
- Do not change ROS interfaces or runtime behavior for a repository-workflow
  Task unless its Issue explicitly authorizes that scope.
