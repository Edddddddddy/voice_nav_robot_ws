#!/usr/bin/env bash
set -e

workspace=${1:-/mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_rapid}
log=${VOICE_NAV_SMOKE_LOG:-/tmp/voice-nav-speech-input-smoke.log}

cd "$workspace"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
: >"$log"

setsid ros2 launch voice_nav_bringup rapid_agent_stack.launch.py \
  mode:=mapping llm_enabled:=false >"$log" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 150); do
  if grep -q '"audio_source": "audio_engine_fifo"' "$log" && \
      ros2 action info /voice/speak 2>/dev/null | \
        grep -q 'Action servers: 1'; then
    break
  fi
  sleep 0.1
done
grep -q '"audio_source": "audio_engine_fifo"' "$log"

ros2 action send_goal --feedback /voice/speak \
  voice_nav_interfaces/action/Speak \
  '{source_instance_id: smoke, source_seq: 1, session_id: smoke, turn_id: playback, priority: 1, text: "音频引擎播放测试。", allow_barge_in: true}' \
  >/tmp/voice-nav-engine-playback-action.log 2>&1
grep -q 'code: 0' /tmp/voice-nav-engine-playback-action.log
grep -q 'detail: AudioEngine playback completed' \
  /tmp/voice-nav-engine-playback-action.log

for _ in $(seq 1 30); do
  grep -Eq 'playback_frames=[1-9][0-9]*' "$log" && break
  sleep 0.1
done
grep -Eq 'fifo_frames=[1-9][0-9]*' "$log"
grep -Eq 'playback_frames=[1-9][0-9]*' "$log"
echo 'SPEECH_FULL_DUPLEX=PASS source=audio_engine_fifo'
grep -E 'audio_source|full-duplex AudioEngine started|audio blocks=' "$log" | \
  tail -n 6
grep -E 'Feedback:|code:|detail:' \
  /tmp/voice-nav-engine-playback-action.log | tail -n 5
