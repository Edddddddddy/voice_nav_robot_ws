#!/usr/bin/env bash
set -e

workspace=${1:-/mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_rapid}
log=${VOICE_NAV_SMOKE_LOG:-/tmp/voice-nav-speech-input-smoke.log}

cd "$workspace"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
: >"$log"

status=0
timeout --signal=INT --kill-after=5s 18s \
  ros2 launch voice_nav_bringup rapid_agent_stack.launch.py \
  mode:=mapping llm_enabled:=false >"$log" 2>&1 || status=$?
test "$status" -eq 0 -o "$status" -eq 124
grep -q '"audio_source": "audio_engine_fifo"' "$log"
grep -Eq 'fifo_frames=[1-9][0-9]*' "$log"
echo 'SPEECH_INPUT_FIFO=PASS source=audio_engine_fifo'
grep -E 'audio_source|full-duplex AudioEngine started|audio blocks=' "$log" | \
  tail -n 5
