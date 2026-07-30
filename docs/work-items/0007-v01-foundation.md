# VN-0007: Establish the v0.1 foundation and v1.0 target contracts

**Status:** In Progress

## Goal

Complete the v0.1 foundation: preserve a verified recovery point, remove
generated colcon output from all reachable history, retain the accepted
simulation and engineering work, migrate product/course/process documentation,
add repository contracts and hosted CI, establish the course dual track,
protect `main`, and record the approved v1.0 contracts without claiming target
behavior is implemented.

The user explicitly authorized this complete plan on 2026-07-30. The Work Item
remains open until both its local and remote gates are completed.

## Non-goals

- Implementing the v0.2 ros2_control migration.
- Implementing Mission Runtime, Motion Gate, SLAM, Nav2, or voice behavior.
- Semantically rewriting completed lessons, ADR decisions, or earlier source
  commits beyond the approved removal of `build/`, `install/`, and `log/`.
- Committing local models, generated maps, bags, recordings, or build output.
- Changing repository visibility, adding collaborators, or weakening security
  settings outside the approved v0.1 governance scope.

## Acceptance criteria

### Local v0.1 foundation

- [x] The earlier clean-source, package-metadata, minimum robot, Gazebo motion,
  odometry, and engineering-baseline facts remain traceable to their original
  commits and records.
- [x] Documentation is organized under `docs/product`, `docs/architecture`,
  `docs/process`, `docs/adr`, and `docs/work-items`.
- [x] Flat top-level normative documents are removed rather than duplicated.
- [x] The product glossary and engineering resources are preserved under
  `docs/product`.
- [x] v1.0 product acceptance and scope are explicit.
- [x] The target process names and package ownership are explicit.
- [x] Mission fencing, bounded IDL, `StopMission.srv`, and transient-local
  `/mission/state` are specified.
- [x] The target motion chain, independent Motion Gate, 250 ms lease, and
  0.35 s controller timeout are specified.
- [x] TF owners, exact wheel frames, separate Mapping/Navigation modes, and the
  `/clock`/`/scan`-only bridge are specified.
- [x] Voice/Agent audio, model, decision-order, and latest-wins constraints are
  specified.
- [x] ADR-0001 is preserved and marked superseded; ADR-0002 through ADR-0004
  record the approved decisions.
- [x] VN-0006 retains Done status and concrete evidence.
- [x] A complete external recovery bundle and remote baseline are audited.
- [x] Version, package metadata, repository contracts, Issue/PR templates, and
  the required Ubuntu 24.04 / ROS 2 Jazzy CI workflow are present locally.
- [x] The course catalog and start/solution-tag workflow make `main` the one
  reference solution without copying source code.
- [x] Repository-wide links and root/course navigation point to the new tree.
- [x] Formatting/link checks and the unified repository gate pass after all
  parallel v0.1 migration work is integrated.
- [x] The final local diff is reviewed against the complete v0.1 Work Item,
  with generated files, private data, and target-as-current claims absent.
- [x] A second pre-rewrite external bundle captures the clean committed v0.1
  branch and all refs.
- [x] An isolated clone proves the narrow history rewrite removes every
  `build/`, `install/`, and `log/` path without changing the intended HEAD tree.
- [x] The same verified narrow rewrite is applied to the local refs.

### Remote gate

- [x] The user explicitly authorized the approved plan's remote write scope and
  `Edddddddddy/voice_nav_robot_ws` destination on 2026-07-30.
- [ ] Immediately before writing, remote `main`, branch set, and collaborators
  still match the audited baseline; otherwise work stops without overwriting.
- [ ] Rewritten `main` is updated once with an exact
  `--force-with-lease=<old-sha>` and no broad force.
- [ ] The Work Item has a GitHub Issue and the feature branch has a reviewed PR.
- [ ] The reviewed v0.1 foundation commit is present on the intended remote
  branch and is rebase-merged into remote `main`.
- [ ] Hosted CI runs the declared repository verification and passes.
- [ ] `main` requires PRs, the stable CI check, conversation resolution, and
  linear history; force-push and deletion are disabled.
- [ ] Remote review/merge evidence and the final remote commit ID are recorded
  here.
- [ ] The `v0.1.0` annotated tag and GitHub Release are created only after the
  release gate is satisfied.

## Recovery bundle

External bundle:

```text
Windows:
C:\Users\lcy\code\ros2\voice_nav_robot_ws_backups\pre-v01-20260730.bundle

WSL:
/mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_backups/pre-v01-20260730.bundle
```

Verified file evidence:

```text
size_bytes=987446
sha256=EF872D496C5C546766F92AC18C792156194129FFDFBBE027E575C282A668CA34
```

`Get-Item` reported 987446 bytes and
`Get-FileHash -Algorithm SHA256` produced the value above.

`git bundle verify` reported a complete history containing:

```text
bd93dffcd8936182821daad89761a752a0dd0447
  refs/heads/chore/0006-engineering-baseline
bc2264257aea246c61ec183f3736c663a86a65d1
  refs/heads/main
  refs/remotes/origin/main
bd93dffcd8936182821daad89761a752a0dd0447
  HEAD
```

The bundle is outside the repository, is not runtime input, and was not added
to Git.

## Pre-rewrite v0.1 bundle and rewrite proof

After the four reviewed v0.1 commits and a clean worktree, a second complete
bundle was created:

```text
Windows:
C:\Users\lcy\code\ros2\voice_nav_robot_ws_backups\pre-rewrite-v01-20260730.bundle

WSL:
/mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_backups/pre-rewrite-v01-20260730.bundle

size_bytes=1083754
sha256=341FBF83EA6251B46D9F5E5A6BD0213572D5BE6D5D0115876D6B14C1D815FB9D
```

`git bundle verify` reported complete history with five refs, including
pre-rewrite feature HEAD
`cccd1ada1f523760eb3831cb1fc2ecf112f67e53`.

The approved transformation was first run twice in disposable `/tmp` clones
from that bundle using Ubuntu's `git-filter-repo 2.38.0`:

```text
git filter-repo \
  --force \
  --replace-refs delete-no-add \
  --invert-paths \
  --path build \
  --path install \
  --path log
```

The second isolated proof produced:

```text
before_head=cccd1ada1f523760eb3831cb1fc2ecf112f67e53
after_head=e67dd93a717b000a7f49fbc7bb5222ab6d3325c6
before_tree=28e722b28b742dd55d91cbe140a6eff83c971b3e
after_tree=28e722b28b742dd55d91cbe140a6eff83c971b3e
reachable_generated_paths=0
replace_refs=0
git_fsck=PASS
```

The isolated clean checkout then passed the full repository, model, build, and
27-test WSL gate. Only after that proof was the identical transformation
applied locally. Local HEAD and tree matched the isolated values, all three
local branches were rewritten, `git fsck --full --strict` passed, and the
reachable generated-path count remained zero. `filter-repo` removed the
`origin` configuration as expected; it was restored to the audited SSH URL
without fetching old objects.

## Remote and local baseline audit

Read-only audit on 2026-07-30:

```text
origin:
git@github.com:Edddddddddy/voice_nav_robot_ws.git

remote HEAD:
refs/heads/main

origin/main:
bc2264257aea246c61ec183f3736c663a86a65d1

local branch:
chore/0007-v01-foundation

local HEAD before VN-0007 working-tree edits:
bd93dffcd8936182821daad89761a752a0dd0447
```

`git ls-remote origin refs/heads/*` showed only remote `main`. The local
foundation was six commits ahead of `origin/main`:

```text
c402fd0 chore(repo): establish clean workspace boundaries
46b6d2f feat(sim): add physical differential-drive robot
a9747d4 docs(course): record robot modeling and motion lessons
14b82e1 docs(repo): establish engineering governance baseline
dbaefad chore(repo): complete package metadata
bd93dff docs(work-item): close engineering baseline
```

No remote write was performed.

## Audit conclusions

- The external bundle is valid, complete, and contains both the local
  engineering baseline and remote-main baseline.
- Remote `main` alone could not recover the six local baseline commits at the
  audit time; the verified external bundle is the recovery point.
- Package metadata is complete at `dbaefad`; statements that metadata or remote
  selection remain undecided are stale.
- Lesson/VN-0006 engineering-baseline work is complete at `bd93dff`; current
  work is VN-0007, not an in-progress Lesson 0006.
- Native Gazebo DiffDrive remains valid historical lesson evidence but is
  superseded for the product by ADR-0002 and the v0.2 ros2_control migration.
- The canonical behavior term is operational stop (运行停止); its exact ROS
  type is `StopMission.srv`. It is not a functional-safety emergency stop.
- Target documents and current implementation are explicitly separated.
- The remote gate is open: the audit performed no push, merge, tag, hosted-CI,
  visibility, or branch-protection change.

## Risks and rollback

- The external backup path is machine-specific. Recovery instructions retain
  both Windows and WSL forms and use the verified byte size and full checksum.
- Broad document movement can break historical links. Repository-wide link
  checking and coordinated root/course updates are required before closure.
- Target IDL intentionally differs from current provisional source. It remains
  labeled target until one Work Item migrates every producer and consumer.
- Document changes can be reverted without affecting source or the external
  bundle.

## Design impact

- Stable Interfaces changed now: documentation only.
- Target stable Interfaces added: bounded Mission, Mission state snapshot,
  operational stop using `StopMission.srv`, Voice Turn, and Speak.
- TF or motion ownership changed now: none.
- Target ownership changed: diff_drive_controller owns odometry/TF;
  `motion_gate_node` solely publishes final velocity.
- ADR required: yes; ADR-0002, ADR-0003, and ADR-0004.

## Test plan

- Static: `git diff --check`.
- Documentation: verify every relative Markdown link inside `docs/`.
- Audit: re-run bundle checksum, `git bundle verify`, branch/ref comparison, and
  generated-directory tracked-file count read-only.
- Integration: run the unified repository gate only after all parallel
  migration edits are integrated.

## Documentation

- `docs/product/`
- `docs/architecture/`
- `docs/process/`
- `docs/adr/`
- `docs/work-items/`

## Verification evidence

Documentation-subtree checks completed on 2026-07-30:

```text
docs relative Markdown link check: PASS
git diff --check -- docs: PASS
git diff --check: PASS at documentation handoff
```

Integrated local gate completed in WSL Ubuntu 24.04 on 2026-07-30:

```text
Repository contract passed.
19 repository/SDF tests: OK
All system dependencies have been satisfied
robot name is: voice_nav_robot
SDF contract passed.
Summary: 6 packages finished [33.1s]
Summary: 27 tests, 0 errors, 0 failures, 1 skipped
VoiceNav Robot verification passed.
```

`git diff --check` also passed. The Work Item remains In Progress until the
final staged-diff review, history-rewrite proof, and every remote-gate item are
recorded.
