# VoiceNav product context

This file is limited to product vocabulary and architecture. Delivery roles,
permissions, recovery, and handoff state live in [AGENTS.md](AGENTS.md).

## Product vocabulary

- **VoiceNav** helps a person express and complete a bounded walking or
  navigation intention with a robot.
- **Mission** is a user-visible intention with a beginning, outcome, and
  terminal result: completed, cancelled, or failed; it is not an unbounded
  conversation.
- **Place** is a named or otherwise recognizable destination in the operating
  environment.
- **Stop** explicitly ends active movement and leaves the system stationary.

## Product architecture

- One deep Mission Runtime owns Mission execution behind stable actions,
  services, and observation; an independent Motion Gate retains final velocity
  authority ([ADR-0003](docs/adr/0003-use-one-deep-mission-runtime.md)).
- The product control path uses `gz_ros2_control` and
  `diff_drive_controller`; the historical native-DiffDrive baseline is in
  [ADR-0002](docs/adr/0002-migrate-to-gz-ros2-control.md) and
  [ADR-0001](docs/adr/0001-use-native-gazebo-diff-drive.md).
- Mapping and Navigation launch as separate modes so `map → odom` has one
  owner at a time ([ADR-0004](docs/adr/0004-separate-mapping-and-navigation-modes.md)).

For GitHub Issue/PR operations, see
[docs/agents/README.md](docs/agents/README.md).
