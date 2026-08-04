# VoiceNav repository agent protocol

Read this file and the root [domain glossary](CONTEXT.md) before acting on a
repository Task. GitHub Issues are the canonical source for requirements,
decisions, acceptance, dependencies, and status. Existing
[architecture ADRs](docs/adr/) remain authoritative for product and interface
decisions.

## Roles

### Manager

- Owns the parent PRD and decomposes it into small, decision-complete Tasks.
- Assigns one fresh managed worktree, branch, Worker context, and Draft PR to
  each independently reversible Task.
- Keeps at most two independent implementation sessions active and coordinates
  them through persisted Issue/PR comments plus direct event messages.
- Resolves blocked decisions in the canonical Issue before asking a Worker to
  continue.

### Worker

- Owns exactly one assigned Task and does not broaden its allowed scope.
- Starts from the current `origin/main` in an isolated managed worktree; a
  shared checkout is never a Worker workspace.
- Implements one observable behavior at a time with focused RED, minimal GREEN,
  and refactoring while green. Tests exercise the highest stable public
  Interface.
- Posts durable evidence, commits intentionally, pushes the Task branch, and
  opens one Draft PR linked with `Closes #NN`.
- Runs the complete repository gate once, on the final PR HEAD. Development
  uses focused checks.

### Reviewer

- Is a fresh, on-demand, read-only context for one Draft PR.
- Compares the PR with the linked Issue acceptance criteria and records P0–P3
  findings on the PR.
- Does not modify the Worker branch, merge the PR, or replace missing product
  decisions with implementation guesses.

## Skill routing

- Requirements, PRD, or Task shaping: `voice-nav-requirements`.
- Implementation of one assigned Issue: `voice-nav-worker`.
- Read-only PR evaluation: `voice-nav-review`.
- Read the applicable skill instructions before taking the routed action.

## Event message

Persist the full evidence to the canonical Issue or PR first. Then send the
following compact envelope to the Manager; do not put logs or the Issue body in
the direct message:

```text
VOICE_NAV_EVENT: blocked|completed|reviewed
issue: #NN
pr: #NN|none
thread: <thread-id>
head: <sha|none>
evidence: <Issue-or-PR-comment-URL>
decision_needed: <required for blocked or reviewed P0/P1 blockers; otherwise none>
```

A `blocked` event must state what was attempted, the smallest unresolved
decision, available options, and a recommendation in the persisted comment.
A `reviewed` event with a P0/P1 finding that blocks merge must fill `decision_needed`; use `none` only when no decision/action is needed.
An implementation Worker sends `completed` only after its Draft PR and
verification evidence exist. A Reviewer sends `reviewed` after recording the
review result. No event asks another context to poll for progress.

## Context recovery

When a context is new, resumed, or compacted:

1. Read the assigned Issue and all comments, its parent PRD, linked dependency
   Issues, and any linked PR/decision evidence.
2. Read this file, `CONTEXT.md`, `docs/agents/`, and the relevant `docs/adr/`
   files. A local task-state index, when present, is secondary evidence.
3. Confirm the repository, isolated worktree, current branch, `HEAD`, and
   `origin/main` before editing. Preserve unrelated changes.
4. Rebuild the acceptance mapping from persisted records rather than hidden
   conversation history.
5. If a requirement, interface, threshold, dependency, or scope decision is
   missing, persist a blocked comment and send a `blocked` event before making
   implementation changes.

## Forbidden

- Do not use subagents or delegate work outside the assigned context.
- Do not use thread polling, recurring monitoring, or CI polling; use persisted
  events and direct messages.
- Do not work in a shared checkout or modify another Task's branch/worktree.
- Do not merge, tag, release, or force-push as a Worker or Reviewer.
- Do not treat an external PR as a requirements or decision intake surface.
- Do not add source-shape, AST, or full-file-fingerprint checks without explicit
  Issue approval.
- Do not change ROS interfaces or runtime behavior for a repository-workflow
  Task unless its Issue explicitly authorizes that scope.
