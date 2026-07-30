# VN-0006: Establish an auditable engineering baseline

**Status:** In Progress

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
- [ ] `build/`, `install/`, and `log/` are no longer tracked by Git.
- [ ] The three local generated directories still exist after index cleanup.
- [ ] Meaningful source, course, and governance changes are committed
  separately.
- [ ] The full verification command passes.
- [ ] A remote host and visibility are selected before push or hosted CI setup.

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

Pending learner execution of Lesson 0006.
