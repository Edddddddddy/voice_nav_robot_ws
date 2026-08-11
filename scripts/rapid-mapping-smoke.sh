#!/usr/bin/env bash
set -e

workspace=${1:-/mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_rapid}
map_id=${VOICE_NAV_SMOKE_MAP_ID:-rapid_smoke_$RANDOM}
map_root=${VOICE_NAV_SMOKE_MAP_ROOT:-/tmp/voice_nav_rapid_maps}
log=${VOICE_NAV_SMOKE_LOG:-/tmp/voice-nav-mapping-smoke.log}

cd "$workspace"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
: >"$log"

setsid ros2 launch voice_nav_bringup mapping_sim.launch.py \
  headless:=true llm_enabled:=false map_output_root:="$map_root" \
  >"$log" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  sleep 3
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 60); do
  if ros2 service type /slam_toolbox/save_map 2>/dev/null | grep -q SaveMap && \
      ros2 service type /slam_toolbox/serialize_map 2>/dev/null | \
        grep -q SerializePoseGraph; then
    break
  fi
  sleep 0.5
done

timeout 30 ros2 topic echo --once /map nav_msgs/msg/OccupancyGrid \
  >/dev/null 2>&1

ros2 topic pub --once /voice/turn voice_nav_interfaces/msg/VoiceTurn \
  "{voice_instance_id: probe, voice_seq: 1, session_id: probe-session, turn_id: probe-turn, kind: 1, text: '保存地图为 $map_id', confidence: 1.0, during_playback: false}" \
  >/tmp/voice-nav-mapping-publish.log 2>&1

for _ in $(seq 1 60); do
  grep -q 'SPEAK: 任务已完成' "$log" && break
  sleep 0.5
done
grep -q 'SPEAK: 任务已完成' "$log"

package=$map_root/$map_id
for file in map.yaml map.pgm map.posegraph map.data manifest.yaml named_places.yaml; do
  test -s "$package/$file"
done
grep -q 'image: map.pgm' "$package/map.yaml"
echo "MAPPING_E2E=PASS package=$package"
grep -E 'Saved rapid Map Package|SPEAK:' "$log" | tail -n 4
