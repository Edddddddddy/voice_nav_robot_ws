# Change lifecycle

GitHub Issues are the canonical source for requirements, decisions,
acceptance, dependencies, and status. A change is delivered through one
isolated branch and one pull request linked to its Issue. The verification
cadence, evidence ownership, and stop rules in this document are the single
repository contract; other documents link here instead of restating them.

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

The Issue owns the requirement, decisions, acceptance criteria, dependencies,
and workflow status. The PR owns the observable result, acceptance mapping,
final HEAD, focused and complete test summaries, interface impact, rollback,
and residual risks.

Do not create a per-commit development diary, paste complete logs into the
Issue and PR, or create a second local evidence ledger. Keep raw logs outside
Git and store concise command, exit-status, and result summaries in the PR.

## Evidence at each stage

| Stage | Minimum durable evidence |
| --- | --- |
| Issue | Goal, non-goals, acceptance, risk, interface impact, dependencies, rollback, verification |
| Design | Stable Interface documentation; ADR for a consequential trade-off |
| Implementation | Source, configuration, and tests at the deepest stable Interface |
| Verification | Exact command, true exit status, test summary, and required manual evidence in the PR |
| Review | Complete staged diff, acceptance mapping, and resolved review findings |
| Release | Version, changelog, immutable tag, and linked acceptance evidence |

## Three distinct granularities

- **Issue**: one independently reversible change with observable acceptance.
- **Commit**: one coherent modification reason and review unit.
- **Release**: an immutable set of deliverable capabilities, not a single
  commit or branch.

## Verification cadence

During implementation, run the focused repository checks as often as needed:

```text
python3 -m unittest tests.test_repository_contract
python3 scripts/check_repository.py --root .
```

Additional checks may be run when the changed behavior requires them, but they
must use the narrowest stable Interface and must not replace the repository
contract checks.

After the final change, run the complete repository gate exactly once on the
final PR HEAD:

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
