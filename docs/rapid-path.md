# Rapid local end-to-end path

This branch contains a deliberately fast local demonstration path.  It is for
getting the full simulation, mapping/navigation, Mandarin command, and speech
chain running before the production safety path is integrated.  It does not
make a MotionGate, collision-monitor, recovery, or real-robot safety claim.

## Components

```text
Gazebo house + LiDAR
  -> SLAM Toolbox (mapping) or AMCL + Nav2 (navigation)
  -> rapid mission bridge (motion/map save or up to three ordered places)
  -> Agent deterministic parser + loopback Qwen fallback
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

# Run voice-driven motion and save a map from the fixed house world.
ros2 launch voice_nav_bringup mapping_sim.launch.py headless:=true

# In a second run, load the supplied house map and start the whole rapid path.
ros2 launch voice_nav_bringup navigation_sim.launch.py headless:=true
```

The navigation launch starts Gazebo, AMCL, Nav2, `rapid_mission_bridge`, the
existing Agent, and the rapid voice endpoint.  Named places are `home`,
`study`, and `kitchen`; try a Mandarin command such as `去厨房` (the local
parser maps it to `kitchen`). Their poses are loaded from the installed
`house_demo_places.yaml`, validated once at startup, and published in the
MissionState snapshot. The voice endpoint uses Vosk when its model and Python
path are available, and still accepts terminal text when standard input is
attached.

For predictable WSL performance, this launch waits for Gazebo/odometry before
starting Nav2, uses Regulated Pure Pursuit instead of the heavier default MPPI
controller, and allows two seconds for Behavior Tree action acknowledgement.
The rapid-only velocity relay forwards stamped `/cmd_vel_nav` output directly
to the simulated base; it deliberately bypasses the product safety chain.

To navigate with a package saved in Mapping Mode, bind the whole package
instead of passing occupancy and Named Place files separately. Launch verifies
the manifest, all five SHA-256 entries, `map_id`, relative occupancy image, and
Named Place schema before constructing AMCL/Nav2 actions:

```bash
ros2 launch voice_nav_bringup navigation_sim.launch.py \
  map_package:=/tmp/voice_nav_rapid_maps/house_new map_id:=house_new
```

The rapid wake phrase is `小智`. For example, say or type `小智，去厨房，然后去书房`.

The navigation launch now starts the provisioned loopback Qwen server by
default.  It reads `VOICE_NAV_LLM_BUNDLE` when set, otherwise it uses the
bundle created by `scripts/llm/provision.sh --root
/home/ubuntu/.local/share/voice-nav-llm`.  Set `llm_enabled:=false` to run the
deterministic rules without the model.
The resulting Mission executes its two Nav2 goals in order. This is a simple
Vosk-transcript gate, not the dedicated low-latency KWS planned for #56.

The mapping launch starts the same Agent/voice/model stack in Mapping mode.
It accepts MOVE, ROTATE, and SAVE_MAP Missions. A command such as
`保存地图为 house_new` calls SLAM Toolbox's SaveMap and SerializePoseGraph
services and writes `map.yaml`, `map.pgm`, `map.posegraph`, and `map.data`
plus `named_places.yaml` and a SHA-256 `manifest.yaml` under
`/tmp/voice_nav_rapid_maps/house_new/`. The six files are first assembled in a
same-root staging directory and exposed by one rename. Pass
`map_output_root:=...` to change that trusted root. Mapping and navigation
rapid launches explicitly disable the production MotionGate chain because
they publish directly to the simulated controller; `product_sim.launch.py`
remains safe-by-default with the chain enabled.

## Local speech assets

This local workspace uses untracked assets under `.deps/voice/`:

- `vosk-model-small-cn-0.22` for offline Mandarin ASR;
- Piper's `zh_CN-huayan-medium` voice model for local TTS.

The rapid stack starts `voice_nav_audio/audio_engine_node`. Its PortAudio
callback moves 48 kHz full-duplex samples through fixed SPSC rings; the worker
performs a small 48→16 kHz conversion and writes private PCM frames to a FIFO
consumed by Vosk. When the voice node is started alone without a FIFO, Vosk
still falls back to WSLg `parec` or PyAudio. Transcript wake matching accepts
`小 智` and the common Vosk `小志`/`晓智` forms. Piper output is converted to
48 kHz mono in its worker and written through a second private FIFO into the
AudioEngine playback ring.
The rapid Speak server now keeps a real PlaybackScope: it reports elapsed
feedback, waits for AudioEngine playback (or standalone `paplay` fallback) to
finish, and returns canceled/barged-in results instead of claiming completion
when playback merely starts. An accepted wake can interrupt an allowed scope,
while `小智停止` always interrupts playback, calls `/mission/stop` directly with
the Voice Turn ID, and then publishes the same STOP turn for the Agent's
idempotent retry.

This makes the C++ capture worker the real rapid ASR source and the same
full-duplex callback the real TTS render sink, without exposing PCM as a ROS
interface. Every callback render sample is also copied to the existing exact
reference ring. Locked WebRTC AEC and sherpa-onnx KWS/VAD/ASR/TTS remain
production gaps; rapid mode still uses Vosk wake/transcription and Piper.

Set `VOICE_NAV_VOICE_ROOT` for a different clone or speech asset root. Missing
speech assets do not prevent
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
bash scripts/rapid-navigation-smoke.sh
bash scripts/rapid-mapping-smoke.sh
bash scripts/rapid-voice-smoke.sh
bash scripts/rapid-audio-smoke.sh
bash scripts/rapid-speech-input-smoke.sh
```
