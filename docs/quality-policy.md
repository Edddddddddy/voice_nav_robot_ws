# Quality Policy

VoiceNav Robot uses
[REP-2004](https://reps.openrobotics.org/rep-2004/) as a checklist for
versioning, change control, documentation, testing, dependencies, platform
support, and security. It does not currently claim production quality: this is
a pre-1.0 learning product with an explicit path toward a reproducible,
reviewable prototype.

The strictest repository policy applies to all six ROS packages.

## Version policy

- Project releases use Semantic Versioning.
- ROS package versions remain synchronized at release boundaries.
- The stable Interface includes ROS names, types, QoS, TF ownership,
  configuration schemas, units, ordering, errors, and cancellation behavior.
- Pre-1.0 compatibility can change, but breaking changes must still be called
  out in the changelog and updated across every producer, consumer, and test in
  one change.

## Change-control policy

- No direct feature development on `main`.
- Every change has a work item, short-lived branch, acceptance criteria, test
  plan, documentation impact, and reviewed diff.
- Commits follow Conventional Commits.
- Main-line changes must pass the same `scripts/verify.sh` gate locally and in
  hosted CI once a remote provider is selected.
- Published release tags are immutable; history is not rewritten after a
  release.

## Documentation policy

- `README.md` describes actual capabilities and setup.
- `docs/architecture.md` separates current implementation from target design.
- Every stable Interface has usage and behavioral documentation.
- ADRs record only consequential architectural trade-offs.
- `CONTEXT.md` is the canonical project-specific language.
- `CHANGELOG.md` records notable behavior and Interface changes.
- Learning records count as verification evidence only after their acceptance
  commands and explanations have been reviewed.

## Test policy

- Pure parsing, validation, and state transitions receive unit tests.
- ROS names, types, QoS, parameters, and TF ownership receive contract tests.
- Node composition and lifecycle receive launch/integration tests.
- Gazebo, SLAM, and Nav2 receive bounded headless smoke tests.
- Voice and LLM CI uses deterministic fakes; large local models are milestone
  tests, not ordinary CI dependencies.
- Safety-critical transitions such as invalid input, timeout, cancel, stale
  result, busy admission, and Safety Stop require explicit tests.
- Motion tests send zero velocity in normal and cleanup paths.
- A coverage threshold will be introduced when executable Mission and Agent
  code exists; lint-only lines are not treated as meaningful coverage.

## Dependency policy

- `package.xml` plus rosdep is the source of truth for ROS dependencies.
- Direct runtime dependencies are declared directly, not relied on
  transitively.
- Source dependencies, when unavoidable, use a `.repos` file pinned to a
  commit.
- Python dependencies will use a reviewed lock file when first introduced.
- Local model weights are never stored in Git. A future model manifest must
  record URL, version, SHA-256, license, expected size, and supported runtime.
- Third-party licenses are reviewed before code, voices, or weights are
  redistributed.

## Platform policy

The supported development and runtime platform is intentionally narrow:

```text
Windows 11 host
WSL2 Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
```

Only this matrix is promised before 1.0. CI should reproduce Ubuntu 24.04 and
ROS 2 Jazzy; GUI behavior remains a manual WSL milestone check.

## Security and privacy policy

- Credentials and `.env` files never enter version control.
- Recordings, bags, generated maps containing private layouts, and runtime
  evidence remain local unless explicitly sanitized.
- DDS and future model servers default to local-only access.
- LLM output is untrusted and cannot directly publish motion.
- Motion requires schema validation, an allowlist, configured limits,
  watchdog, and Motion Gate admission.
- SROS2 is deferred because the current product is local simulation only; this
  decision must be revisited before networked or multi-user deployment.
