# Changelog

All notable user-visible and Interface changes to VoiceNav Robot are recorded
here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and releases follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Strongly typed Mission step and execution Action interfaces.
- Hand-written differential-drive Xacro model and internal TF tree.
- Gazebo collision, inertia, stable spawning, native DiffDrive, velocity
  limits, and odometry.
- Incremental lessons, engineering references, and learning records.
- Repository governance, architecture, quality, testing, and release policies.

### Security

- Ignore rules prevent newly created generated data, local credentials,
  recordings, bags, and model weights from being added. Removal of the
  already-tracked colcon outputs is tracked by work item VN-0006.
