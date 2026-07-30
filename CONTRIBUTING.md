# Contributing to VoiceNav Robot

This repository uses a lightweight enterprise workflow scaled for one learner
and one reviewer. The process exists to make every change understandable,
reproducible, and recoverable; it does not add ceremonies that do not improve
those properties.

## Change workflow

1. Create or update one file in `docs/work-items/` with the goal, non-goals,
   acceptance criteria, risk, test plan, and documentation impact.
2. Open the matching GitHub Issue and link the repository Work Item.
3. Create a short-lived branch from `main`.
4. Implement the smallest vertical slice that satisfies the work item, with
   the first failing test recorded before production behavior is added.
5. Update user documentation, the changelog, and an ADR only when applicable.
6. Run the local quality gate and record exact evidence in the Work Item.
7. Open a PR, review the complete diff, and let hosted CI pass.
8. Rebase-merge only after every Definition of Done item is satisfied.

Do not develop directly on `main`. A long-lived `develop` branch is not used;
small branches reduce integration delay and keep the main line authoritative.

## Branch names

Use one of these forms:

```text
feat/0007-ros-gz-motion-bridge
fix/0012-stop-watchdog
test/0015-mission-cancel-race
docs/0006-engineering-baseline
chore/0006-engineering-baseline
```

The number points to the related work item or lesson.

## Commit messages

Use [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/):

```text
feat(sim): bridge Gazebo odometry into ROS
fix(mission): reject non-finite motion distance
test(sim): verify odom frame ownership
docs(adr): record the simulation drive adapter
chore(repo): stop tracking colcon artifacts
```

Allowed primary types are `feat`, `fix`, `test`, `docs`, `refactor`, `perf`,
`build`, `ci`, and `chore`. Each commit should represent one coherent reason
for change. Do not mix broad formatting with behavior changes.

## Local quality gates

Fast package loop:

```bash
bash scripts/verify.sh voice_nav_sim
```

Full change-request gate:

```bash
bash scripts/verify.sh
```

Generated `build/`, `install/`, `log/`, model weights, recordings, bags,
credentials, and runtime evidence must never be committed.

Run the repository-only fast gate before ROS work:

```bash
python3 -m unittest discover -s tests -p "test_*.py" -v
```

Review what will be committed with:

```bash
git status --short
git diff
git diff --cached
```

Prefer explicit pathspecs such as `git add src/voice_nav_sim docs/adr/...`.
Avoid an unreviewed `git add .`, especially in a mixed Windows/WSL workspace.

## Definition of Done

A change is done only when:

- its work-item acceptance criteria are all satisfied;
- the relevant package builds from declared dependencies;
- automated tests cover the new success path and important failure paths;
- `bash scripts/verify.sh` passes;
- ROS names, types, QoS, parameters, units, TF ownership, error behavior, and
  ordering constraints are documented when they form part of an Interface;
- motion tests always request zero velocity during normal and failure cleanup;
- user-visible behavior is recorded under `Unreleased` in `CHANGELOG.md`;
- architecture documentation distinguishes current implementation from the
  accepted target architecture;
- a qualifying architectural trade-off has an ADR;
- the diff contains no generated data, secrets, private audio, or model weights;
- verification evidence is recorded in the work item or linked learning record;
- the author has reviewed the complete staged diff.

## Architecture decision threshold

Write an ADR only if the choice is costly to reverse, surprising without its
context, and the result of a real trade-off. An ordinary implementation detail
belongs in code, tests, or the relevant Interface documentation.

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

Individual lessons are changes, not releases. Lesson start/solution tags are
annotated teaching checkpoints, not supported product versions. Releases
happen at coherent capability milestones and follow
[docs/process/release-policy.md](docs/process/release-policy.md).

The repository currently has one maintainer. CI and resolved conversations are
required; approval by the PR author never substitutes for independent review.
An approval requirement is enabled only after a real second reviewer exists,
so branch protection cannot deadlock the repository.
