# VN-0006: Establish an auditable engineering baseline

**Status:** Done

## Goal

Turn the current learning workspace into a clean, reviewable source repository
with explicit development, quality, testing, architecture, and release rules.

## Non-goals

- Hosted Git remote, branch protection, and provider-specific CI.
- Rewriting the existing root commit.
- Renormalizing every existing file in the same change.
- Adding new robot behavior.

## Acceptance criteria

- [x] Work continues on `chore/0006-engineering-baseline`.
- [x] Repository ignore, line-ending, and editor policies exist.
- [x] Architecture, domain language, contribution, testing, quality, release,
  and changelog documents exist.
- [x] Native Gazebo DiffDrive trade-off is recorded as an ADR.
- [x] One local verification command covers dependency, model, build, and test
  checks.
- [x] `build/`, `install/`, and `log/` are no longer tracked by Git.
- [x] The three local generated directories still exist after index cleanup.
- [x] Meaningful source, course, and governance changes are committed
  separately.
- [x] The full verification command passes.
- [x] A remote host and visibility are selected before push or hosted CI setup.

## Risks and rollback

- An incorrect `git rm` could delete local generated files. Use
  `git rm --cached` only and confirm the directories still exist.
- Line-ending renormalization creates a broad diff. It is explicitly deferred
  to its own reviewed change.
- Existing uncommitted work must be staged with explicit paths, not a blind
  `git add .`.
- All Git index changes can be unstaged without changing source files.

## Design impact

- Stable Interfaces changed: none.
- TF or motion ownership changed: none.
- ADR required: yes, for the already selected simulation drive adapter.

## Test plan

- Contract: Xacro expansion, URDF parse, SDF conversion, and DiffDrive
  parameter assertions.
- Integration: clean full colcon build and test.
- Manual: inspect Git tracked-file counts and staged diff.

## Documentation

- Root repository guidance and `docs/`.
- Lesson 0006 and Lesson 0005 learning record.

## Verification evidence

Completed and closed on 2026-07-30.

- Stable public closure commit after the approved v0.1 history cleanup:
  `a9ac21fa54de9ce69dd19d5cc6eaf65de7251ffb`
  (`docs(work-item): close engineering baseline`).
- Original pre-rewrite closure commit:
  `bd93dffcd8936182821daad89761a752a0dd0447`. It remains recoverable from the
  verified external bundles; VN-0007 records the complete old-to-new mapping.
- The closure review recorded the full `bash scripts/verify.sh` criterion as
  passed. Generated colcon logs were intentionally not committed.
- Read-only re-audit on 2026-07-30:
  - `git ls-files build install log` returned zero tracked files;
  - local `build/`, `install/`, and `log/` directories all still existed;
  - public `2855943` / pre-rewrite `c402fd0` established clean workspace
    boundaries;
  - public `e1dc36c` / pre-rewrite `46b6d2f` recorded the physical simulation
    implementation;
  - public `eef682f` / pre-rewrite `a9747d4` recorded course and learning
    evidence;
  - public `2cf50e7` / pre-rewrite `14b82e1` established governance;
  - public `08fd6e3` / pre-rewrite `dbaefad` completed package maintainer and
    description metadata;
  - public `a9ac21f` / pre-rewrite `bd93dff` closed VN-0006.
- The later addition of `origin` does not change the historical non-goal:
  hosted CI and branch protection were not part of VN-0006.
