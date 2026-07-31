# Changelog

All notable user-visible and Interface changes to VoiceNav Robot are recorded
here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and releases follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Added the in-progress Lesson 0008 Work Item, course contract, and pending
  evidence template for a packaged LiDAR world, direct product `/odom`, and
  per-edge TF publisher-GID ownership verification.
- Added the tests-first Lesson 0007 contract, Work Item, and pending evidence
  record for migrating the simulation drive path to `gz_ros2_control` and
  Jazzy's native `TwistStamped` `diff_drive_controller` input.
- Added a headless Gazebo integration gate that proves controller activation,
  wheel Interface claims, forward/positive-yaw odometry, TF output, publisher
  loss, and the 0.35 s consumer deadman.

### Changed

- Replaced the product model's native Gazebo DiffDrive plugin with
  `gz_ros2_control`, a Jazzy `diff_drive_controller`, and an event-ordered
  launch owned directly by ROS 2 launch.
- Restricted the Lesson 0007 Gazebo bridge to `/clock` and documented normal
  acceleration shaping as an upstream velocity-smoother responsibility.

### Fixed

- Prevented WSL launch tests from orphaning a shell-owned Gazebo process and
  accidentally reusing an old controller graph.

## [0.1.0] - 2026-07-30

### Added

- Strongly typed Mission step and execution Action interfaces.
- Hand-written differential-drive Xacro model and internal TF tree.
- Gazebo collision, inertia, stable spawning, native DiffDrive, velocity
  limits, and odometry.
- Versioned Markdown course catalog, Lessons 0001–0006, engineering references,
  and evidence-backed learning records.
- Repository contract tests for course ordering, path confinement, local links,
  documentation layout, and synchronized ROS package versions.
- GitHub Issue and PR templates, pinned Ubuntu 24.04 / ROS 2 Jazzy CI, and
  monthly GitHub Actions dependency updates.
- Product, architecture, Mission, motion, TF/mode, quality, testing, release,
  security, ADR, and Work Item documentation for the accepted v1.0 plan.

### Changed

- Reorganized documentation into `docs/product`, `docs/architecture`,
  `docs/process`, `docs/adr`, `docs/work-items`, and `course`.
- Synchronized all ROS package metadata with the repository `0.1.0` version.
- Defined native Gazebo DiffDrive as a historical teaching checkpoint; the
  product path now migrates to `gz_ros2_control`, `diff_drive_controller`, an
  independent MotionGate, and consumer-side command timeout.
- Extended the canonical verification command to run repository contracts and
  reject tracked colcon outputs before building.

### Fixed

- Preserved the original Lesson 0003 box-robot assignment and added a separate
  erratum for the accepted cylinder implementation, wheel links, dimensions,
  and measured LiDAR transform.
- Closed the Lesson 0006 evidence gap using the learner's recorded commits and
  verification output.

### Security

- Ignore rules exclude generated data, local dependencies, credentials,
  recordings, bags, model weights, and runtime evidence.
- The quality gate rejects tracked `build/`, `install/`, and `log/` paths.
- Operational stop terminology no longer implies a certified emergency-stop
  or physical-stop guarantee.
