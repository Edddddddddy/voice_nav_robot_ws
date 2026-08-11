#!/usr/bin/env bash

workspace=${1:-/mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_rapid}
log=${VOICE_NAV_SMOKE_LOG:-/tmp/voice-nav-navigation-smoke.log}

cd "$workspace"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
: > "$log"

setsid ros2 launch voice_nav_bringup navigation_sim.launch.py \
  headless:=true llm_enabled:=false >"$log" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  sleep 3
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 50); do
  if test "$(grep -c 'Managed nodes are active' "$log")" -ge 2 && \
      grep -q 'Rapid demo bypass enabled' "$log"; then
    break
  fi
  sleep 1
done

ros2 topic pub --once --qos-durability transient_local \
  /voice/turn voice_nav_interfaces/msg/VoiceTurn \
  '{voice_instance_id: probe, voice_seq: 1, session_id: probe-session, turn_id: probe-turn, kind: 1, text: "去kitchen", confidence: 1.0, during_playback: false}' \
  >/tmp/voice-nav-navigation-publish.log 2>&1

outcome=TIMEOUT
for _ in $(seq 1 55); do
  if grep -q 'SPEAK: 任务已完成' "$log"; then
    outcome=PASS
    break
  fi
  if grep -q 'SPEAK: 任务执行失败' "$log"; then
    outcome=FAIL
    break
  fi
  sleep 1
done

echo "NAVIGATION_E2E=$outcome"
grep -E 'Rapid demo bypass|First Nav2 velocity|Begin navigating|Forwarded rapid|Goal succeeded|Failed to make progress|SPEAK:|Received a goal|Passing new path|Goal failed' \
  "$log" | tail -n 40
test "$outcome" = PASS
