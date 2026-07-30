# Release Policy

VoiceNav Robot uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
Before 1.0, compatibility changes remain possible but are always explicit.

## Planned milestones

| Version | Capability |
| --- | --- |
| `v0.1.0` | Differential-drive simulation, odometry, TF, and 2D LiDAR |
| `v0.2.0` | SLAM mapping and map saving |
| `v0.3.0` | AMCL localization and Nav2 navigation |
| `v0.4.0` | Mission workflow, Motion Gate, cancel, timeout, and Safety Stop |
| `v0.5.0` | Local Mandarin voice loop with AEC, ASR, LLM, and TTS |
| `v1.0.0` | Full documented acceptance flow with reproducible setup |

Lessons and merged work items do not automatically create releases.

## Version meaning

- `MAJOR`: an incompatible stable Interface change after 1.0.
- `MINOR`: a backward-compatible capability or pre-1.0 milestone.
- `PATCH`: a backward-compatible correction that changes no intended
  Interface.

All project ROS packages use the same version at a release boundary. Package
versions are not bumped for every commit.

## Release gate

A release requires:

- all work-item acceptance criteria for the milestone are complete;
- the full quality gate passes from a clean checkout;
- supported environment and dependency installation are documented;
- package metadata and versions are consistent;
- `CHANGELOG.md` moves relevant `Unreleased` entries into a dated version;
- architecture and Interface documentation match the released behavior;
- manual milestone checks are recorded;
- no unresolved critical safety, data-loss, or license issue remains;
- an annotated Git tag is created from `main`.

Release tags and their contents are immutable. A problem found after release is
fixed in a new version.

## Distribution

Before 1.0, distribution is source plus a tagged repository release. Bloom and
ROS apt packaging are out of scope unless the project later needs public binary
distribution.
