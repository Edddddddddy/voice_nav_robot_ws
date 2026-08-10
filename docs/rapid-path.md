# Rapid local end-to-end path

This branch contains a deliberately fast local demonstration path.  It is for
getting the full simulation, mapping/navigation, Mandarin command, and speech
chain running before the production safety path is integrated.  It does not
make a MotionGate, collision-monitor, recovery, or real-robot safety claim.

## Components

```text
Gazebo house + LiDAR
  -> SLAM Toolbox (mapping) or AMCL + Nav2 (navigation)
  -> rapid mission bridge (up to three ordered places)
  -> Agent deterministic Mandarin parser
  -> Vosk microphone transcript -> wake phrase gate
  -> Piper speech output
```

The rapid navigation launch publishes a short burst of `/initialpose` samples
from a ROS node after Gazebo starts, so AMCL can acquire its starting pose
without a hand-written CLI YAML message.

## Start

From WSL Ubuntu 24.04 with ROS 2 Jazzy and the repository dependencies
installed:

```bash
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_rapid
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Build a static map from the fixed house world.
ros2 launch voice_nav_bringup mapping_sim.launch.py headless:=true

# In a second run, load the supplied house map and start the whole rapid path.
ros2 launch voice_nav_bringup navigation_sim.launch.py headless:=true
```

The navigation launch starts Gazebo, AMCL, Nav2, `rapid_mission_bridge`, the
existing Agent, and the rapid voice endpoint.  Named places are `home`,
`study`, and `kitchen`; try a Mandarin command such as `去厨房` (the local
parser maps it to `kitchen`).  The voice endpoint uses Vosk when its model and
Python path are available, and still accepts terminal text when standard input
is attached.

The rapid wake phrase is `小智`. For example, say or type `小智，去厨房，然后去书房`.
The resulting Mission executes its two Nav2 goals in order. This is a simple
Vosk-transcript gate, not the dedicated low-latency KWS planned for #56.

## Local speech assets

This local workspace uses untracked assets under `.deps/voice/`:

- `vosk-model-small-cn-0.22` for offline Mandarin ASR;
- Piper's `zh_CN-huayan-medium` voice model for local TTS.

`navigation_sim.launch.py` supplies the paths used by this checkout.  For a
different clone, pass the corresponding parameters to `rapid_voice_node`, or
change those four local launch values.  Missing speech assets do not prevent
simulation and navigation from starting; the endpoint logs speech to the
console instead.

## Lightweight smoke checks

Use the small checks while iterating; the complete repository gate is
intentionally deferred for this rapid branch:

```bash
python3 -m py_compile src/voice_nav_agent/voice_nav_agent/rapid_*.py
colcon build --packages-select voice_nav_agent voice_nav_bringup --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF
ros2 launch voice_nav_bringup navigation_sim.launch.py --show-args
```
