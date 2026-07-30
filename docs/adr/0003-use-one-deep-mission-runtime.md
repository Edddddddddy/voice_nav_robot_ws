---
status: accepted
---

# Use one deep Mission Runtime with an independent Motion Gate

VoiceNav Robot exposes Mission execution through `ExecuteMission.action`,
Operational Stop through `StopMission.srv`, and observation through one
transient-local state snapshot. Guard, single-slot admission, source/runtime
fencing, workflows, Nav2, relative motion, and map saving stay behind
`mission_runtime_node`; final velocity authority stays in the separate
`motion_gate_node` process in the same package.

## Considered options

- separate Guard, scheduler, per-step executor, Nav2 bridge, and map-saver
  nodes;
- expose direct Nav2 and velocity operations to Agent;
- keep a small public Interface with internal Adapters and an independent final
  gate.

## Consequences

Callers learn two mutation operations instead of distributed orchestration.
Tests replace internal Nav2, velocity, map, gate, and clock Adapters. The
separate Gate process preserves lease expiry if Runtime stalls, while package
locality prevents its private control seam from becoming an Agent contract.
