# Agent control plane

This directory records the repository-level delivery rules. The root
[`AGENTS.md`](../../AGENTS.md) defines role behavior and the root
[`CONTEXT.md`](../../CONTEXT.md) defines shared domain terms.

## GitHub Issue tracking

- A GitHub Issue is the canonical record for requirements, decisions,
  acceptance criteria, dependencies, and status.
- A PR is the delivery record for one Issue. Its description maps acceptance to
  files and evidence and includes `Closes #NN`.
- Issue comments and PR comments preserve decisions, verification summaries,
  blocked reasons, and event evidence. Direct messages only carry the compact
  event envelope.
- One Task has one fresh managed worktree, one branch, one Worker context, and
  one Draft PR. A Reviewer receives a separate read-only context.
- Local notes or task-state indexes may accelerate recovery, but they never
  override the canonical Issue, PR, or authoritative ADR.

## Standard labels

Keep the type label and at most one workflow-state label on an Issue:

| Label | Meaning |
| --- | --- |
| `type:prd` | Parent product requirements document |
| `type:task` | Independently reversible implementation Task |
| `ready-for-agent` | Decision-complete and eligible for a fresh Worker |
| `in-progress` | A Worker currently owns the Task |
| `blocked` | Work cannot continue until a named decision or dependency changes |
| `needs-review` | A Draft PR is ready for independent review |
| `verified` | Acceptance evidence and required checks are complete |

State changes are recorded in an Issue comment. A blocked Task names the
smallest decision needed to resume; it is not silently reinterpreted by a
Worker.

## External PR intake

External pull requests are not a requirements or decision intake surface. A
maintainer first creates or identifies the canonical GitHub Issue, records its
scope and acceptance criteria, and links any later contribution to that Issue.
The Issue remains authoritative even when a PR originates outside the
repository's managed Worker flow.

## Single-context layout

The repository has one shared context layer:

```text
AGENTS.md              role and protocol rules
CONTEXT.md             implementation-free domain glossary
docs/adr/              authoritative product and interface decisions
docs/agents/           delivery configuration and label rules
.github/               Issue and PR forms
```

Do not create competing root glossaries or per-Task protocol copies. A fresh
context reads the assigned Issue, parent PRD, linked evidence, this layout,
and relevant ADRs, then works only in its own managed worktree.
