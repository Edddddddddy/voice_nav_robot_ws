# Architecture overview

This document separates verified current behavior from the approved v1.0
target. Target Modules are not presented as implemented.

## Current implementation at the v0.1 foundation

Verified before this documentation migration:

- provisional `MissionStep.msg` and `ExecuteMission.action`;
- a hand-written physical differential-drive Xacro;
- static robot-state-publisher launch and internal TF;
- Gazebo native DiffDrive motion, configured limits, and odometry;
- package skeletons for audio, Agent, Mission, and bringup;
- completed package maintainer and description metadata;
- a unified local verification command.

Native Gazebo DiffDrive is historical learning behavior, not the product target.
[ADR-0001](../adr/0001-use-native-gazebo-diff-drive.md) is superseded by
[ADR-0002](../adr/0002-migrate-to-gz-ros2-control.md).

The target ros2_control stack, 2D LiDAR bridge, SLAM, Nav2, Mission Runtime,
Motion Gate, and local voice pipeline are not current claims.

## Target v1.0 topology

```text
microphone / speakers
        │
        ▼
    voice_node
        │ recognized Mandarin / cancelable speech
        ▼
    agent_node
        │ ExecuteMission.action
        ▼
mission_runtime_node ◄──── /mission/state snapshot
        │
        ├── Nav2 velocity ─────────┐
        └── relative velocity ─────┴─► nav2_velocity_smoother
                                             │
                                      nav2_collision_monitor
                                             │
                                      motion_gate_node
                                             │
                                  diff_drive_controller
                                             │
                                  gz_ros2_control / Gazebo

Operational Stop ── StopMission.srv ──► mission_runtime_node
```

The self-written process set is exactly:

```text
voice_node
agent_node
mission_runtime_node
motion_gate_node
```

Upstream ROS/Gazebo processes remain independently managed.

## Deep Modules and public Interfaces

### Voice Module

Its Interface produces completed Voice Turns and accepts cancelable speech.
Its Implementation hides the WSL audio device, playback reference, WebRTC
audio processing, Wake Word, VAD, local ASR, local TTS, buffering, and barge-in.
Raw 10 ms PCM remains in-process.

### Agent Module

Its Interface turns recognized text into a Mission, clarification, rejection,
or reply. Its Implementation hides deterministic rules, constrained local LLM
fallback, turn correlation, and stale-result rejection. It never publishes
velocity and does not import Nav2 or Gazebo types.

### Mission control Module

`voice_nav_mission` owns two processes:

- `mission_runtime_node`: public `ExecuteMission.action`, transient-local
  `/mission/state`, public `StopMission.srv`, whole-plan Guard, single-slot
  admission, terminal-intent linearization, workflows, source selection, Nav2,
  relative motion, and map saving;
- `motion_gate_node`: package-private control seam, final authorization, 250 ms
  lease, lock, limits, zero output, and sole final velocity publication.

The caller learns one execution operation, one stop operation, and one state
snapshot. Internal complexity remains local to one package while separate
processes keep the final watchdog independent from orchestration.

### Simulation Module

`voice_nav_sim` owns Xacro, Gazebo assets, `gz_ros2_control`, controller
configuration, and the sensor bridge. Target `ros_gz_bridge` traffic is only
`/clock` and `/scan`; Gazebo model-scoped names do not escape.

## Package and process boundaries

| Package | Owned behavior | Self-written process |
| --- | --- | --- |
| `voice_nav_interfaces` | bounded stable ROS IDL | none |
| `voice_nav_audio` | local audio and speech pipeline | `voice_node` |
| `voice_nav_agent` | rules, local LLM Adapter, dialogue policy | `agent_node` |
| `voice_nav_mission` | Runtime, Gate, and internal dependency Adapters | `mission_runtime_node`, `motion_gate_node` |
| `voice_nav_sim` | robot model, Gazebo, ros2_control, `/clock` and `/scan` bridge | none |
| `voice_nav_bringup` | launches, parameters, maps, and Named Places | none |

Do not create Guard, scheduler, Nav2 bridge, map saver, or top-level behavior
tree processes. These are internal seams of Mission Runtime.

## Dependency direction

```text
voice_nav_audio ─────┐
voice_nav_agent ─────┼──► voice_nav_interfaces
voice_nav_mission ───┘

voice_nav_agent ──► Mission Interface only
voice_nav_mission ──► standard ROS / Nav2 / map Interfaces
voice_nav_sim ──► Gazebo + gz_ros2_control + ros_gz_bridge
voice_nav_bringup ──► runtime composition and configuration
```

Forbidden directions:

```text
agent   ─X─► Nav2 / Gazebo / cmd_vel / controller commands
audio   ─X─► Mission Implementation / Nav2 / Gazebo
mission ─X─► Gazebo Transport / ASR / LLM / TTS
sim     ─X─► Agent / voice / Mission policy
```

## Operating flows

Mapping:

```text
/scan + diff_drive_controller odometry
        ─► slam_toolbox ─► logical map save
```

Navigation:

```text
saved map ─► map server + AMCL
Named Place ─► Nav2 ─► fixed safety motion chain
```

Mapping and Navigation are separately launched. slam_toolbox and AMCL never
own `map → odom` simultaneously.

## Internal seams and Adapters

Mission Runtime has private Interfaces for Nav2, relative motion, candidate
velocity, map saving, Gate control, and steady time. Each has a production
Adapter and a scripted in-memory fake. The same pattern applies to local ASR,
LLM, and TTS engines.

Guard rules, FSM, Named Place values, and audio callback wiring remain ordinary
Implementation. They are not speculative plug-in Interfaces.

## Stable Interface surface

Compatibility-sensitive behavior includes:

- ROS names, types, bounded fields, QoS, node names, parameters, and frames;
- units, limits, time domains, admission, ordering, cancellation, and errors;
- TF and final-velocity ownership;
- configuration, map, Named Place, and model-manifest schemas.

Changes require contract tests and changelog treatment. Detailed target
contracts:

- [Mission Runtime Interface](mission-runtime-interface.md)
- [Safety and motion](safety-and-motion-contract.md)
- [TF and operating modes](tf-and-operating-modes.md)
- [Voice and Agent](voice-and-agent.md)
- [ADR-0001: superseded native DiffDrive](../adr/0001-use-native-gazebo-diff-drive.md)
- [ADR-0002: migrate to ros2_control](../adr/0002-migrate-to-gz-ros2-control.md)
- [ADR-0003: one deep Mission Runtime](../adr/0003-use-one-deep-mission-runtime.md)
- [ADR-0004: separate mapping and navigation modes](../adr/0004-separate-mapping-and-navigation-modes.md)

Release compatibility follows the
[release policy](../process/release-policy.md).
