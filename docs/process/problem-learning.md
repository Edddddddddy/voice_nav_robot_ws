# Problem recurrence control

VoiceNav Robot treats a failure as unfinished until the useful diagnostic fact
is captured and, where practical, converted into an executable guardrail. The
grouped [known-pitfalls reference](known-pitfalls.md) preserves reusable rules;
the Issue and PR preserve the specific change and evidence.

## Vocabulary

- **Occurrence**: one observed failure with an exact command, environment,
  output, and repository head.
- **Pitfall**: a generalized failure pattern that can recur in another change.
- **Root cause**: the narrowest causal statement supported by evidence. An
  unproven implementation-specific explanation remains a hypothesis.
- **Guardrail**: an automated test, static contract, bounded runtime check, or
  process check that prevents or detects recurrence.

## Required loop

```text
observe exact symptom
  -> preserve command / exit / environment / HEAD
  -> reproduce or bound the uncertainty
  -> state the narrow root cause
  -> add the nearest executable guardrail
  -> update the Issue, PR evidence, and relevant pitfall
  -> verify on the exact final HEAD
```

1. Capture facts before editing. Keep warnings separate from the decisive
   failure and record whether the command changed repository or external state.
2. Reproduce at the smallest stable seam: pure unit test, then ROS launch test,
   then full Gazebo product test when the boundary requires it.
3. Use tests-first correction for behavior defects and preserve the RED/GREEN
   causal chain when it makes review easier.
4. Put the guardrail at the ownership boundary: Core invariant in Core tests,
   ROS graph behavior in Node tests, composition behavior in product tests, and
   repository shape in repository contracts.
5. Replace provisional evidence after every product-code change. A passing run
   from an ancestor commit is diagnosis, not final acceptance evidence.

## Recurrence escalation

| Signal | Required response |
| --- | --- |
| First code defect | Regression test plus a concise pitfall entry |
| Same pattern appears again | Add an automated/static guardrail at the shared seam |
| Pattern survives a guardrail | Review module boundary and test fidelity |
| Architectural trade-off changes | Create or supersede an ADR |

Documentation alone is not an adequate response to a repeatable safety or
release failure. Command-execution problems belong in the grouped reference
and safer command templates rather than in product tests.

## Pitfall entry contract

Each entry in `known-pitfalls.md` keeps a stable `PIT-NNNN` anchor, a symptom,
the decisive discriminator, the supported cause, and the shortest safe
diagnostic or guardrail. Update an existing ID when the same cause recurs;
create a new ID only when the cause or prevention boundary is materially
different. Do not remove a stable anchor without migrating its active links.

## Anti-patterns

- inferring the cause from the last warning line alone;
- adding a fixed sleep to hide an asynchronous race;
- broadening retries from one typed transient state to every mismatch;
- changing global Git or WSL configuration to make one invocation pass without
  understanding ownership;
- claiming final verification before the exact final commit exists; and
- copying incident logs into multiple documents until they disagree.
