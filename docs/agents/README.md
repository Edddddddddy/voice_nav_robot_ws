# GitHub operations

The root [AGENTS.md](../../AGENTS.md) owns delivery roles, permissions,
recovery, and the two-phase evidence protocol. The product glossary is
[CONTEXT.md](../../CONTEXT.md). This file contains only GitHub Issue/PR
operations and short examples.

## Issue and PR records

- A GitHub Issue is the canonical record for requirements, decisions,
  acceptance criteria, dependencies, and status.
- One Task has one Issue, one managed worktree/branch, one Worker context, and
  one Draft PR. A PR maps acceptance to files and evidence and includes
  `Closes #NN`.
- Comments preserve decisions, blocked reasons, verification summaries, and
  evidence after transport. Local notes and Task YAML accelerate recovery but
  never override the Issue, PR, or ADR.

## Labels

Keep the type label and at most one workflow-state label:

| Label | Meaning |
| --- | --- |
| `type:prd` | Parent product requirements document |
| `type:task` | Independently reversible implementation Task |
| `ready-for-agent` | Decision-complete and eligible for assignment |
| `in-progress` | A Worker owns the Task |
| `blocked` | A named decision or dependency is missing |
| `review-needed` | Draft PR is ready for independent review |
| `verified` | Acceptance evidence and required checks are complete |

State changes and the smallest blocker are recorded in an Issue comment.

## External contributions

An external PR is not a requirements or decision intake surface. A maintainer
first identifies the canonical Issue, records scope and acceptance there, then
links the contribution to that Issue.

## Short examples

```text
Issue 链接: #NN
验收证据: command/result or immutable Git object
状态: ready-for-agent
```

```text
PR: Closes #NN
结果: acceptance criterion → file → evidence
```
