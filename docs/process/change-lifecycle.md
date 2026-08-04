# Change lifecycle

GitHub Issues are the canonical source for requirements, decisions,
acceptance, dependencies, and status. A change is delivered through one
isolated branch and one pull request linked to its Issue.

```text
GitHub Issue
   -> isolated short-lived branch
   -> implementation + focused tests + documentation
   -> local verification
   -> complete diff review / Draft PR / CI
   -> rebase merge to main
   -> release milestone when the capability is ready
```

## Evidence at each stage

| Stage | Minimum durable evidence |
| --- | --- |
| Issue | Goal, non-goals, acceptance, risk, interface impact, dependencies, rollback, verification |
| Design | Stable Interface documentation; ADR for a consequential trade-off |
| Implementation | Source, configuration, and tests at the deepest stable Interface |
| Verification | Exact command, exit status, test summary, and required manual evidence |
| Review | Complete staged diff, acceptance mapping, and resolved review findings |
| Release | Version, changelog, immutable tag, and linked acceptance evidence |

## Three distinct granularities

- **Issue**: one independently reversible change with observable acceptance.
- **Commit**: one coherent modification reason and review unit.
- **Release**: an immutable set of deliverable capabilities, not a single
  commit or branch.

## Git safety boundaries

- `.gitignore` affects only untracked files; it does not remove tracked data.
- `git rm --cached` removes an index entry but, without `--cached`, also removes
  the working-tree file.
- Use explicit pathspecs in a mixed Windows/WSL workspace. Do not use
  destructive reset or clean commands to resolve uncertainty.
- Published tags and release history are never rewritten.

## Minimum pre-PR checks

```bash
git status --short
git diff
bash scripts/verify.sh
git diff --cached
```

See [REP-2004](https://reps.openrobotics.org/rep-2004/),
[Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/), and
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).
