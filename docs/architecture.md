# Architecture

This document distinguishes the system that exists today from the target
architecture. Planned Modules are not described as if they were implemented.

## Current implementation

The repository currently contains:

- generated ROS Interface code from `MissionStep.msg` and
  `ExecuteMission.action`;
- a static robot-state-publisher launch for model and TF inspection;
- a physical differential-drive Xacro model;
- Gazebo native DiffDrive motion, limits, and odometry;
- skeleton packages for audio, Agent, Mission, and bringup.

ROS–Gazebo bridges, LiDAR, SLAM, Nav2, Mission execution, and voice processing
are not implemented yet.

## Target dependency direction

```text
voice_nav_audio ─────┐
voice_nav_agent ─────┼──► voice_nav_interfaces
voice_nav_mission ───┘

voice_nav_agent ──► Mission Interface only
voice_nav_mission ──► standard ROS / Nav2 Interfaces
voice_nav_sim ──► Gazebo and ROS–Gazebo adapters
voice_nav_bringup ──► runtime composition and configuration
```

Forbidden dependency directions:

```text
agent   ─X─► Gazebo / Nav2 / wheel commands / final cmd_vel
audio   ─X─► Mission implementation / Nav2 / Gazebo
mission ─X─► Gazebo / ASR / LLM / TTS
sim     ─X─► Agent / voice policy / Mission business rules
```

## Deep Modules and seams

### Voice Module

Interface:

- starts a Voice Turn after the Wake Word;
- produces recognized text and structured audio events;
- accepts speech requests and cancellation.

Implementation hides audio devices, AEC, KWS, VAD, ASR, TTS, and barge-in.

### Agent Module

Interface:

- converts recognized text plus current capabilities into a Mission,
  clarification, or rejection;
- maps Mission events to Mandarin replies.

Implementation hides deterministic rules, local LLM prompting, late-result
rejection, and dialogue policy. It never acquires motion authority.

### Mission Runtime Module

Interface:

- `ExecuteMission.action` for bounded, observable, cancelable work;
- a separate Safety Stop fast lane;
- capability and event observation.

Implementation hides whole-plan validation, the single execution slot,
ordering, timeouts, cancellation, motion gating, Nav2 adaptation, map saving,
and stable error normalization. Tests and callers cross the same Interface.

### Simulation adapter

The seam between ROS motion contracts and Gazebo Transport is implemented only
inside `voice_nav_sim`. Gazebo topic names, message types, and model-scoped
paths must not escape into Agent or Mission code.

## Operating modes

The product has two separately launched modes:

```text
Mapping Mode:
  Gazebo → odom / scan / TF → slam_toolbox → map save

Navigation Mode:
  saved map → AMCL + Nav2 → gated velocity → Gazebo
```

The two modes do not switch dynamically. `slam_toolbox` and AMCL never own the
same transform at the same time.

## TF ownership

| Transform | Owner |
| --- | --- |
| `map → odom` in Mapping Mode | `slam_toolbox` |
| `map → odom` in Navigation Mode | AMCL |
| `odom → base_footprint` | one odometry publisher |
| robot internal frames | `robot_state_publisher` |

An adapter change that moves `odom → base_footprint` ownership must remove the
old publisher in the same change.

## Motion authority

The target command path is:

```text
Mission relative motion ─┐
Nav2 velocity ────────────┼─► Motion Gate ─► watchdog / limits ─► simulator
Safety Stop ──────────────┘
```

Only the Motion Gate may own the final velocity output. The LLM emits semantic
Mission Steps, never velocity. Every loss-of-command or failure path must
converge on zero velocity.

## Stable Interfaces

The following are compatibility-sensitive even when they are not language
types:

- ROS message, service, and Action fields and semantics;
- topic, service, Action, node, parameter, and frame names;
- QoS profiles and clock behavior;
- units, limits, timeout ownership, ordering, and cancellation rules;
- result and error-code meanings;
- supported configuration file schemas.

Changing one requires tests and changelog updates; a breaking change also
requires the version treatment described in the release policy.
