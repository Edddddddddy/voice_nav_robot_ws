# Problem learning and recurrence control

VoiceNav Robot treats a failure as unfinished work until the useful lesson is
captured and, where practical, converted into an executable guardrail. This
process complements a Work Item: the Work Item closes one change, while the
pitfall register preserves the reusable diagnostic pattern.

## Vocabulary

- **Occurrence**: one observed failure with an exact command, environment,
  output, and repository head.
- **Pitfall**: a generalized failure pattern that can recur in another lesson
  or change.
- **Root cause**: the narrowest causal statement supported by evidence. An
  unproven implementation-specific explanation remains a hypothesis.
- **Guardrail**: an automated test, static contract, bounded runtime check, or
  process check that prevents or detects recurrence.
- **Learning artifact**: the short troubleshooting entry that lets a learner
  recognize and diagnose the pattern without replaying the incident history.

## Required loop

```text
observe exact symptom
  -> preserve command / exit / environment / HEAD
  -> reproduce or bound the uncertainty
  -> state the narrow root cause
  -> add the nearest executable guardrail
  -> update Work Item and troubleshooting reference
  -> verify on the exact final HEAD
```

1. Capture facts before editing. Keep warnings separate from the decisive
   failure and record whether the command changed repository or external state.
2. Reproduce at the smallest stable seam. Prefer a pure unit test before a ROS
   launch test, and a launch test before a full Gazebo product test.
3. Use tests-first correction for behavior defects. Preserve the RED and GREEN
   commits when they make the causal chain reviewable.
4. Put the guardrail at the ownership boundary: Core invariant in Core tests,
   ROS graph behavior in Node tests, composition behavior in product tests, and
   repository shape in static contract tests.
5. Add or update one entry in
   [the engineering pitfall register](../../course/reference/engineering-pitfalls.md).
   Link the governing Work Item or lesson instead of copying its full history.
6. Replace provisional evidence after every product-code change. A passing run
   from an ancestor commit is useful diagnosis, not final acceptance evidence.

## Recurrence escalation

| Signal | Required response |
| --- | --- |
| First code defect | Regression test plus a concise pitfall entry |
| Same pattern appears again | Add an automated/static guardrail at the shared seam |
| Pattern survives a guardrail | Review the module boundary and test fidelity |
| Architectural trade-off changes | Create or supersede an ADR |

Documentation alone is not an adequate response to a repeatable safety or
release failure. Conversely, not every shell typo needs a product test; command
execution problems belong in the reference and in safer command templates.

## Pitfall entry contract

Every register entry carries:

- stable `PIT-NNNN` ID and status;
- symptom and decisive discriminator;
- supported cause, with uncertainty stated explicitly;
- shortest safe diagnostic path;
- permanent guardrail or the reason one is not yet automated;
- links to the lesson, Work Item, test, or primary source.

Update an existing ID when the same cause recurs. Create a new ID when the
cause or prevention boundary is materially different. Never delete a corrected
entry; mark it superseded and link the replacement.

## Anti-patterns

- inferring the cause from the last warning line alone;
- adding a fixed sleep to hide an asynchronous race;
- broadening retries from one typed transient state to every mismatch;
- changing global Git or WSL configuration to make one automation invocation
  pass without understanding ownership;
- claiming final verification before the exact final commit exists;
- copying incident logs into multiple documents until they disagree.

