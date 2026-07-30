# VoiceNav Robot

VoiceNav Robot is a simulation-only ROS 2 learning product that is being
implemented from first principles. Its end-to-end goal is:

```text
Mandarin voice
  → wake word / AEC / ASR
  → rules + local LLM
  → strongly typed Mission
  → motion gate and workflow
  → SLAM or Nav2
  → Gazebo differential-drive robot
  → local TTS
```

The project targets WSL2 Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic.
Hardware, cloud speech services, cameras, IMU/EKF, and robot arms are out of
scope.

## Current status

Implemented:

- six ROS 2 package skeletons;
- strongly typed `MissionStep` and `ExecuteMission` interfaces;
- a hand-written differential-drive Xacro model and TF tree;
- physical collision and inertia suitable for Gazebo;
- Gazebo native DiffDrive motion and odometry.

Planned next:

- ROS–Gazebo bridge for command, odometry, TF, joint state, and simulation time;
- 2D LiDAR;
- two-stage SLAM and map-based Nav2 navigation;
- Mission runtime, motion gate, cancellation, and safety stop;
- local Mandarin wake-word, ASR, LLM, AEC, and TTS.

## Repository layout

| Path | Responsibility |
| --- | --- |
| `src/voice_nav_interfaces` | Stable ROS messages, services, and actions |
| `src/voice_nav_sim` | Robot description and Gazebo-only adapters |
| `src/voice_nav_mission` | Trusted Mission validation and execution |
| `src/voice_nav_agent` | Rules, local LLM adapter, and dialogue policy |
| `src/voice_nav_audio` | Audio I/O, AEC, KWS, VAD, ASR, and TTS |
| `src/voice_nav_bringup` | Composition, configuration, and operating modes |
| `docs/` | Architecture, decisions, work items, quality, and release policy |
| `lessons/` | Incremental implementation lessons |
| `reference/` | Compact engineering reference material |
| `learning-records/` | Verified learning outcomes |

See [Architecture](docs/architecture.md) for current and target dependency
directions.

## Build and verify

Run commands inside WSL:

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
bash scripts/verify.sh
```

For a shorter loop that builds dependencies and tests selected packages:

```bash
bash scripts/verify.sh voice_nav_sim
```

The script checks declared dependencies, expands and validates the robot model,
builds the workspace, runs tests, and reports all test results.

## Development workflow

All changes use a short-lived branch and a documented work item:

```text
work item → branch → implementation + tests + docs
          → local quality gate → review → merge → optional release
```

Read [CONTRIBUTING.md](CONTRIBUTING.md) before changing the repository. The
project uses Conventional Commits, a Definition of Done, SemVer releases, and
ADRs only for consequential architectural trade-offs.

## Documentation index

- [Mission](MISSION.md)
- [Domain language](CONTEXT.md)
- [Architecture](docs/architecture.md)
- [Quality policy](docs/quality-policy.md)
- [Testing strategy](docs/testing-strategy.md)
- [Release policy](docs/release-policy.md)
- [Architecture decisions](docs/adr/)
- [Work items](docs/work-items/)
- [Changelog](CHANGELOG.md)
- [Third-party notices](THIRD_PARTY_NOTICES.md)

## License

Apache License 2.0. See [LICENSE](LICENSE).
