# VoiceNav domain glossary

This glossary gives the project one implementation-free meaning for the terms
used in requirements, delivery decisions, and review evidence.

## Terms

- **VoiceNav** — the product that helps a person express and complete a
  bounded walking or navigation intention with a robot.
- **Mission** — a user-visible intention with a beginning, an outcome, and a
  terminal result. A Mission is either completed, cancelled, or failed; it is
  not an unbounded conversation.
- **Place** — a named or otherwise recognizable destination in the robot's
  operating environment.
- **Stop** — an explicit request to end active movement and leave the system in
  a safe stationary state.
- **Issue** — the canonical GitHub record for a requirement, decision,
  dependency, acceptance criterion, or delivery status.
- **PRD** — a parent Product Requirements Document Issue that records the
  user value, boundaries, decisions, and acceptance shape for related tasks.
- **Task** — one independently reversible implementation change with explicit
  acceptance criteria and one owning delivery context.
- **Acceptance criterion** — an observable condition that determines whether a
  Task or PRD is complete.
- **Stable Interface** — a behavior or contract that other parts of the
  product, its operators, or its verification evidence may rely on.
- **Evidence** — a durable reference to the result of a decision, change, or
  verification, stored in an Issue, pull request, or immutable Git object.
- **Manager** — the role that decomposes approved PRDs, assigns decision-
  complete Tasks, and coordinates delivery through persisted events.
- **Worker** — the role that owns exactly one Task in one fresh isolated
  context and produces its Draft PR and evidence.
- **Reviewer** — the read-only role that evaluates one Draft PR against its
  Issue acceptance criteria and records findings.
- **Context** — the bounded set of requirements, decisions, interfaces, and
  evidence needed to act on one Task without relying on hidden conversation
  history.
- **Clarification（澄清）** — 为一个缺失或歧义语义参数发出的有界追问；它会过期，
  不是 Mission，也不是聊天记忆。
