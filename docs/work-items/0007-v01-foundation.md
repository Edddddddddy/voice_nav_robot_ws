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
  Later implementation differences are recorded as separate errata rather than
  retroactively changing the historical assignment.
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
- [x] The target motion chain, independent Motion Gate, Runtime-renewed 250 ms
  authority lease, per-lease candidate data-plane binding, managed
  safe-pause/resume barrier, and 0.35 s controller timeout are specified.
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
- [ ] Rewritten `main` is updated once from the audited remote SHA to the
  rewritten VN-0006 baseline with this exact refspec and lease:
  `git push origin
  a9ac21fa54de9ce69dd19d5cc6eaf65de7251ffb:refs/heads/main
  --force-with-lease=refs/heads/main:bc2264257aea246c61ec183f3736c663a86a65d1`.
  No broad force is used.
- [ ] The Work Item has a GitHub Issue and the feature branch has a reviewed PR.
- [ ] The reviewed v0.1 foundation commit is present on the intended remote
  branch and is rebase-merged into remote `main`.
- [ ] Hosted CI runs the declared repository verification and passes.
- [ ] `main` requires PRs, the stable CI check, conversation resolution, and
  linear history; force-push and deletion are disabled.
- [ ] Remote review/merge evidence and the final remote commit ID are recorded
  here, including the local-post-filter to public-rebase identity mapping.
- [ ] Release eligibility and all v0.1 evidence are prepared so this Work Item
  can close before the separate annotated-tag and GitHub Release step.

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

### Commit identity map

The history cleanup necessarily changed every descendant commit identity. The
external bundles retain the pre-rewrite objects; the second column records the
exact local identities produced by the verified `git-filter-repo` run.

| Pre-rewrite identity | Local post-filter identity | Subject |
| --- | --- | --- |
| `bc2264257aea246c61ec183f3736c663a86a65d1` | `b895e174a2c3afd6ec42d99d0a905f32c6ee9100` | `Init` |
| `c402fd050397af89d3a6889c7a28d928b1f4d133` | `2855943fa4cad72c6b7c1a401796f3145bd3b893` | `chore(repo): establish clean workspace boundaries` |
| `46b6d2fad7187d54948dad3aca304d2d9c5902ec` | `e1dc36c79038d4a92b21db87dee4aec392a7c86c` | `feat(sim): add physical differential-drive robot` |
| `a9747d4c8f7c1506c04973ecb8671eb6d7d931f3` | `eef682f53225bf4cef49d48eef27b34cdeab6236` | `docs(course): record robot modeling and motion lessons` |
| `14b82e1f79f3eb09af8af94d8bc6586af5d6f79e` | `2cf50e79459571c861d1f8754415147ed48efcfb` | `docs(repo): establish engineering governance baseline` |
| `dbaefad4b0fe03e04e131353c81870697e07728e` | `08fd6e30277cd25b323abea3a6dbd9861f0a0068` | `chore(repo): complete package metadata` |
| `bd93dffcd8936182821daad89761a752a0dd0447` | `a9ac21fa54de9ce69dd19d5cc6eaf65de7251ffb` | `docs(work-item): close engineering baseline` |
| `fd8f55d1e1dd18d16c4159b08cf247a8520a3b58` | `38901c16ae47aa4f63c579af8d5fed4a8953077d` | `docs(repo): establish versioned product and course structure` |
| `2ba1aec12c345fca4d98fe74aaa5792c3c0d79d3` | `4c42d6fec4d55fee6fe2c56b8994709b18842d43` | `chore(release): prepare v0.1 metadata` |
| `28a895ec04f096f6d90459663d0eca27f63f4a00` | `f63feef5a25b7ee232eb636fd3611654a6463550` | `test(repo): enforce repository and model contracts` |
| `cccd1ada1f523760eb3831cb1fc2ecf112f67e53` | `e67dd93a717b000a7f49fbc7bb5222ab6d3325c6` | `ci(repo): add hosted quality gate and review templates` |

The post-rewrite evidence commit
`52193337f004ec320981bded32d070f072d78e80`
(`docs(work-item): record verified history rewrite`) was created only after the
transformation and therefore has no pre-rewrite counterpart.

The rows through `a9ac21f` are intended to become exact public identities in
the one-time baseline update. The VN-0007 feature commits from `38901c1`
onward are local review identities: GitHub's rebase merge creates new public
commit objects. After merge this Work Item records a second local-post-filter
→ public-rebase mapping rather than claiming those local SHAs are stable
public evidence.

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

local HEAD before VN-0007 working-tree edits (pre-rewrite identity):
bd93dffcd8936182821daad89761a752a0dd0447
```

`git ls-remote origin refs/heads/*` showed only remote `main`. The local
foundation was six commits ahead of `origin/main`; these are the audited
pre-rewrite identities preserved by the bundle:

```text
c402fd0 chore(repo): establish clean workspace boundaries
46b6d2f feat(sim): add physical differential-drive robot
a9747d4 docs(course): record robot modeling and motion lessons
14b82e1 docs(repo): establish engineering governance baseline
dbaefad chore(repo): complete package metadata
bd93dff docs(work-item): close engineering baseline
```

No remote write was performed.

After the verified rewrite, the corresponding reviewed VN-0006 baseline is
`a9ac21fa54de9ce69dd19d5cc6eaf65de7251ffb`. The one-time remote update targets
that commit, not the rewritten root, so the already reviewed baseline becomes
stable on `main` before the VN-0007 feature branch is pushed normally.

## Audit conclusions

- The external bundle is valid, complete, and contains both the local
  engineering baseline and remote-main baseline.
- Remote `main` alone could not recover the six local baseline commits at the
  audit time; the verified external bundle is the recovery point.
- Package metadata is complete at pre-rewrite `dbaefad` and public rewritten
  `08fd6e3`; statements that metadata or remote selection remain undecided are
  stale.
- Lesson/VN-0006 engineering-baseline work is complete at pre-rewrite
  `bd93dff` and public rewritten `a9ac21f`; current work is VN-0007, not an
  in-progress Lesson 0006.
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

`git diff --check` also passed.

Independent review on 2026-07-30 initially found five history/course/interface
P1 findings, four P2 findings, and then three deeper motion-safety P1 findings.
The resulting corrections include:

- exact force-with-lease and old→local-post-filter identity evidence;
- historical Lesson 0003 plus a separate accepted-implementation erratum;
- an explicitly non-replayable Lesson 0006 and real historical/current paths;
- current-versus-target CI, ROS Interface, and TF language;
- Runtime-only authority renewal, independent candidate freshness, per-lease
  data-plane/writer binding, and full conditioner recreation;
- a tokenized safe-pause performed only after observed controller/wheel zero,
  with full restart instead of in-place resume for an unmanaged pause or
  failed zero proof.

Two focused re-reviews reported no remaining P0/P1. The Work Item remains In
Progress until every remote-gate item is recorded.

Post-review-fix local gate completed in the same environment on 2026-07-30:

```text
Repository contract passed.
20 repository/SDF/CI-contract tests: OK
All system dependencies have been satisfied
robot name is: voice_nav_robot
SDF contract passed.
Build summary: 6 packages finished [13.7s]
Test summary: 6 packages finished [21.0s]
Summary: 27 tests, 0 errors, 0 failures, 1 skipped
VoiceNav Robot verification passed.
```

The first hosted run,
[30557679323](https://github.com/Edddddddddy/voice_nav_robot_ws/actions/runs/30557679323),
failed before workspace verification because `rosdep install` was given apt's
unsupported `--yes` spelling. A red regression test reproduced the workflow
contract error; the invocation now uses rosdep's supported `-y`. The 20-test
repository suite, pinned actionlint, and the full WSL gate above passed before
the fix was pushed for a second hosted run.
