# Contributing to VoiceNav Robot

GitHub Issues are the canonical requirements, decisions, acceptance criteria,
dependencies, and status record. Each implementation change has one owning
Issue, one isolated branch, and one Draft PR.

## Change workflow

1. Shape or update the GitHub Issue with goal, non-goals, acceptance, risk,
   interface impact, dependencies, rollback, and verification requirements.
2. Start from the current `main` in an isolated worktree and create a
   short-lived branch.
3. Implement the smallest observable behavior. For behavior changes, record a
   focused RED test, make it GREEN, then refactor while it remains green.
4. Update user documentation, the changelog, and an ADR only when applicable.
5. Run the focused repository checks during development. After the final
   change, run the complete repository gate exactly once on the final PR HEAD
   and retain its true exit status.
6. Open a Draft PR with `Closes #NN`, record the result, acceptance mapping,
   final test summary, interface impact, rollback, and residual risks, then
   review the complete diff.
7. Rebase-merge only after independent review, required CI, and every
   Definition of Done item are satisfied.

Do not develop directly on `main`. Do not use a shared checkout for an
implementation task, and do not treat a PR as a replacement for its Issue.

## Branch names

Use a short-lived branch whose name identifies the Issue and intent, for example:

```text
feat/25-mission-admission
fix/31-stop-race
test/37-tf-ownership
docs/25-retire-legacy-workflow
```

## Commit messages

Use [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/):

```text
feat(sim): bridge Gazebo odometry into ROS
fix(mission): reject non-finite motion distance
test(sim): verify odom frame ownership
docs(adr): record the simulation drive adapter
chore(repo): stop tracking generated outputs
```

Allowed primary types are `feat`, `fix`, `test`, `docs`, `refactor`, `perf`,
`build`, `ci`, and `chore`. Each commit should represent one coherent reason
for change. Avoid broad formatting mixed with behavior changes.

## Focused repository checks

Run these checks during development:

```bash
python3 -m unittest tests.test_repository_contract
python3 scripts/check_repository.py --root .
```

The complete change-request gate is run exactly once after the final change,
on the final PR HEAD. Run it from that managed worktree; the verification
entry point automatically resolves the `.git` pointer and exports the Git
context to its child processes:

```bash
bash scripts/verify.sh
```

Record its actual exit status before running any separate diagnostics. A later
successful command must not overwrite a failed gate result. See the
[change lifecycle](docs/process/change-lifecycle.md) for the canonical
cadence, evidence ownership, and stop rules.

Review what will be committed with:

```bash
git status --short
git diff
git diff --cached
```

Generated workspaces, model weights, recordings, bags, credentials, and
runtime evidence must never be committed. Prefer explicit pathspecs over an
unreviewed `git add .`.

## Definition of Done

A change is done only when:

- its linked Issue acceptance criteria are satisfied;
- the relevant package builds from declared dependencies;
- automated tests cover the new success path and important failure paths;
- `bash scripts/verify.sh` was run exactly once on the final PR HEAD and its
  true exit status is recorded;
- ROS names, types, QoS, parameters, units, TF ownership, error behavior, and
  ordering constraints are documented when they form a Stable Interface;
- motion tests request zero velocity during normal and failure cleanup;
- user-visible behavior is recorded under `Unreleased` in `CHANGELOG.md`;
- architecture documentation distinguishes current implementation from the
  approved target architecture;
- a qualifying architectural trade-off has an ADR;
- the diff contains no generated data, secrets, private audio, or model weights;
- the PR records exact verification evidence without a per-commit development
  diary or duplicated Issue body; and
- the author has reviewed the complete staged diff.

## Architecture decision threshold

Write an ADR only when the choice is costly to reverse, surprising without its
context, and the result of a real trade-off. Ordinary implementation details
belong in code, tests, configuration, or the relevant Interface document.

## Interface and dependency rules

- `voice_nav_interfaces` depends on no project business package.
- `voice_nav_agent` never publishes wheel or final velocity commands.
- `voice_nav_audio` does not depend on Nav2, SLAM, or Gazebo.
- `voice_nav_mission` does not depend on Gazebo.
- `voice_nav_sim` contains simulation adapters, not domain behavior.
- `voice_nav_bringup` composes Modules and configuration but owns no business
  rules.
- Every dynamic TF edge and final motion output has exactly one owner.
- LLM output is untrusted and must pass a strongly typed Mission gate.

## Releases

Releases represent coherent capability milestones, not individual branches or
commits. They follow [the release policy](docs/process/release-policy.md).
The current archive tag is recovery evidence and is not a supported release.

The repository currently has one maintainer. CI and resolved conversations are
required; approval by the PR author never substitutes for independent review.
