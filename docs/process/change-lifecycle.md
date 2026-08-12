# Change lifecycle

GitHub Issues are the canonical source for requirements, decisions,
acceptance, dependencies, and status. A change is delivered through one
isolated branch and one pull request linked to its Issue. The verification
cadence and evidence records here are reference material only.

[AGENTS.md](../../AGENTS.md) 是角色、权限、流程、交付和恢复的唯一权威。
本文件仅作为验证节奏和证据记录参考，不重述或覆盖 `AGENTS.md` 的规则。

```text
GitHub Issue
   -> isolated short-lived branch
   -> implementation + focused tests + documentation
   -> local verification
   -> complete diff review / Draft PR / CI
   -> rebase merge to main
   -> release milestone when the capability is ready
```

## Evidence ownership

Issue 负责需求、决策、验收标准、依赖和 workflow 状态。PR 负责可观察结果、验收映射、最终 HEAD、聚焦与完整检查摘要、接口影响、回滚和残余风险。

一个 Goal 只绑定一个 decision-complete Issue 和一个 Draft PR。Task 合并或结束即停止该 Goal，不自动续接下一 Task；只有 Manager 能为下一 Task 新建 Goal。Worker/Reviewer 只交接 exact-HEAD 简体中文证据，Manager 使用 `COMMENT` 持久化 GitHub 记录。用户对已批准 Issue 范围内普通 Git/GitHub/WSL/build/test 操作提供持续授权；破坏性、跨范围或平台强制交互除外。

每个 Task 只维护一份 Manager-owned canonical evidence COMMENT；PR 正文仅链接该 COMMENT，不复制摘要、逐提交开发日志、完整原始日志或 Issue 正文。原始日志留在 Git 外，仅以 artifact 路径引用；唯一摘要只在 COMMENT 更新，并记录简明命令、退出状态和结果。

## Evidence at each stage

| Stage | Minimum durable evidence |
| --- | --- |
| Issue | Goal, non-goals, acceptance, risk, interface impact, dependencies, rollback, verification |
| Design | Stable Interface documentation; ADR for a consequential trade-off |
| Implementation | Source, configuration, and tests at the deepest stable Interface |
| Verification | Exact command, true exit status, test summary, and required manual evidence in the PR |
| Review | Complete staged diff, acceptance mapping, and resolved review findings |
| Release | Version, changelog, immutable tag, and linked acceptance evidence |

## 远端治理与本地产品验证

`required / ubuntu-24.04 / ros-jazzy` 只运行快速治理：shellcheck、actionlint、治理合同与 Conventional Commit。它不安装 ROS、不运行 rosdep、colcon、Gazebo 或 `scripts/verify.sh`，因此通过不代表产品验证通过。

产品 PR 在推送可审查里程碑前，由 Worker 在本地 WSL 对 exact HEAD 运行适用的 build、定向测试或一次 `bash scripts/verify.sh`。Manager 将命令、真实退出状态、结果和 artifact 路径更新到唯一的 canonical evidence COMMENT；治理或纯文档 Task 可按范围只运行治理检查。开发可小步本地提交，Review 修复聚合后再推送，合并前 rebase 并保持单 Task 一提交的线性历史。

## Three distinct granularities

- **Issue**: one independently reversible change with observable acceptance.
- **Commit**: one coherent modification reason and review unit.
- **Release**: an immutable set of deliverable capabilities, not a single
  commit or branch.

## Verification cadence

PR 的完整门禁会对 `base..head` 的全部提交运行 Conventional Commit 检查，并要求冒号后的摘要包含简体中文。main push 对 `before..sha` 检查本次提交范围；全零 `before` fail closed，因此不依赖 branch protection 提供该项覆盖。

During implementation, run the focused repository checks as often as needed:

```text
python3 -m unittest tests.test_repository_contract
python3 scripts/check_repository.py --root .
```

Additional checks may be run when the changed behavior requires them, but they
must use the narrowest stable Interface and must not replace the repository
contract checks.

每个推送候选 exact HEAD 在本地最多执行一次适用的产品完整门禁；真实失败后不得在无变更的同一 HEAD 重跑，必须先有因果修复形成新 exact HEAD。远端 required CI 不代表产品验证通过。

```text
bash scripts/verify.sh
```

The exit status of that invocation is the result. Record the true status before
running any separate diagnostic command; a later successful command must not
overwrite or reinterpret a failed gate.

## Stop and re-scope

The following are escalation triggers, not quotas:

- In one Task diff, newly added test/checker implementation exceeds three
  times the newly added product implementation. Compare only the relevant
  added implementation, excluding documentation, generated output, and
  whitespace-only changes; do not rewrite existing product tests to satisfy
  the ratio.
- Ten coherent commits complete without satisfying any acceptance criterion.

When either trigger fires, stop implementation, record the evidence on the
Issue, and split or re-shape the work before continuing. This rule is defined
only here; other documents may link to this section but do not repeat its
interpretation.

## Git safety boundaries

- `.gitignore` affects only untracked files; it does not remove tracked data.
- `git rm --cached` removes an index entry but, without `--cached`, also removes
  the working-tree file.
- Use explicit pathspecs in a mixed Windows/WSL workspace. Do not use
  destructive reset or clean commands to resolve uncertainty.
- Published tags and release history are never rewritten.

## Final pre-PR checks

```bash
git status --short
git diff
git diff --cached
```

See [REP-2004](https://reps.openrobotics.org/rep-2004/),
[Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/), and
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).
