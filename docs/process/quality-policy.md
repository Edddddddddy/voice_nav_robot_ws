# Quality policy

VoiceNav Robot uses
[REP-2004](https://reps.openrobotics.org/rep-2004/) as a checklist for
versioning, change control, documentation, testing, dependencies, platform
support, and security. It does not claim production or safety-certified
quality: this is a pre-1.0 learning product with a path to a reproducible,
reviewable simulation prototype.

The strictest repository policy applies to all six ROS packages.

## Version policy

- Project releases follow Semantic Versioning.
- ROS package versions remain synchronized at release boundaries.
- The stable Interface includes ROS names, types, fields, QoS, TF ownership,
  configuration schemas, units, ordering, errors, clocks, and cancellation.
- Pre-1.0 compatibility may change, but a breaking change is called out in the
  changelog and updates every producer, consumer, contract test, and document
  in one Work Item.

## Change-control policy

- No direct feature development on `main`.
- Every change has a Work Item, short-lived branch, measurable acceptance,
  test plan, documentation impact, and reviewed diff.
- Commits follow Conventional Commits.
- The local `scripts/verify.sh` gate and the required hosted CI check must both
  pass before merge; remote hosting alone is not CI evidence.
- Published release tags are immutable and release history is not rewritten.
- Remote writes, repository visibility changes, and branch-protection changes
  require explicit user authority.

## Documentation policy

- `README.md` describes actual capability and setup rather than target claims.
- [Product](../product/v1.0-product-spec.md) and
  [architecture](../architecture/overview.md) documents label target behavior.
- Every stable Interface has field, invariant, ordering, timeout, cancellation,
  and error documentation.
- ADRs record only consequential trade-offs and are superseded, not rewritten.
- [The product glossary](../product/glossary.md) is the canonical
  project-specific language.
- `CHANGELOG.md` records notable behavior and Interface changes.
- Completed Work Items retain their evidence and are not silently changed back
  to proposals.
- Learning records count as evidence only after their commands and explanations
  have been reviewed.

## Test policy

- Pure parsing, validation, and state transitions receive unit tests.
- ROS names, types, QoS, parameters, units, and TF ownership receive contract
  tests.
- Node composition and lifecycle receive launch/integration tests.
- Gazebo, SLAM, and Nav2 receive bounded headless smoke tests.
- Voice and LLM CI uses deterministic fakes; large local models are milestone
  tests, not ordinary CI dependencies.
- Invalid input, busy admission, timeout, cancel, stale result, Operational
  Stop, and command-lease expiry require explicit tests.
- Every motion test requests zero in normal and cleanup paths and observes
  stationarity.
- A coverage threshold is introduced only when executable Mission and Agent
  behavior exists; lint-only lines are not meaningful coverage.

## Dependency policy

- `package.xml` plus rosdep is the source of truth for ROS dependencies.
- Direct runtime dependencies are declared directly.
- Unavoidable source dependencies use a `.repos` file pinned to a commit.
- Python dependencies gain a reviewed lock file when first introduced.
- Local model weights never enter Git. A model manifest records URL, version,
  exact SHA-256, license, expected size, and supported runtime.
- Third-party licenses are reviewed before code, voices, or weights are
  redistributed.
- A local external backup bundle is recovery evidence, not a runtime
  dependency and not repository content.

## Supported platform

```text
Windows 11 host
WSL2 Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
```

Only this matrix is promised before 1.0. Hosted CI reproduces Ubuntu 24.04 and
ROS 2 Jazzy for deterministic and headless checks. WSL GUI and analog audio
remain bounded manual milestone checks.

## Security and privacy

- Credentials and `.env` files never enter version control.
- Recordings, bags, generated maps containing private layouts, model weights,
  prompts with private data, and raw runtime evidence remain local unless
  explicitly sanitized.
- DDS and model servers default to local-only access.
- LLM output is untrusted and cannot publish motion.
- Motion requires typed schema validation, an allowlist, configured limits,
  monotonic command lease, and Motion Gate admission.
- Operational Stop is an operational simulation control and must not be
  presented as a certified emergency stop.
- SROS2 is deferred for local simulation and must be reconsidered before
  networked or multi-user deployment.
