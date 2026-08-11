#!/usr/bin/env bash
set -e

workspace=${1:-/mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_rapid}
voice_root=${VOICE_NAV_VOICE_ROOT:-$workspace/.deps/voice}
run_dir=$(mktemp -d /tmp/voice-nav-voice-smoke.XXXXXX)
input_fifo=$run_dir/input
voice_log=$run_dir/voice.log
action_log=$run_dir/action.log
completion_log=$run_dir/completion.log

cd "$workspace"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkfifo "$input_fifo"
exec 3<>"$input_fifo"

setsid ros2 run voice_nav_agent rapid_mission_bridge --ros-args \
  -p mode:=mapping >"$run_dir/mission.log" 2>&1 &
mission_pid=$!
setsid ros2 run voice_nav_agent rapid_voice_node --ros-args \
  -p piper_path:="$voice_root/bin/piper" \
  -p piper_model:="$voice_root/models/zh_CN-huayan-medium.onnx" \
  <"$input_fifo" >"$voice_log" 2>&1 &
voice_pid=$!

cleanup() {
  kill -INT -- "-$voice_pid" "-$mission_pid" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-$voice_pid" "-$mission_pid" 2>/dev/null || true
  exec 3>&-
}
trap cleanup EXIT

for _ in $(seq 1 30); do
  if ros2 action info /voice/speak 2>/dev/null | grep -q 'Action servers: 1' && \
      ros2 service type /mission/stop 2>/dev/null | grep -q StopMission; then
    break
  fi
  sleep 0.2
done

ros2 action send_goal --feedback /voice/speak \
  voice_nav_interfaces/action/Speak \
  '{source_instance_id: smoke, source_seq: 1, session_id: smoke, turn_id: speech, priority: 1, text: "正在执行一段可以被新的唤醒词打断的本地语音播放测试，请稍候。", allow_barge_in: true}' \
  >"$action_log" 2>&1 &
action_pid=$!

for _ in $(seq 1 30); do
  grep -q 'SPEAK:' "$voice_log" && break
  sleep 0.1
done
printf '小智停止\n' >&3
wait "$action_pid" || true

for _ in $(seq 1 30); do
  grep -q 'Direct voice STOP result code=' "$voice_log" && break
  sleep 0.1
done

grep -q 'code: 2' "$action_log"
grep -q 'Direct voice STOP result code=0 inhibited=True' "$voice_log"

ros2 action send_goal --feedback /voice/speak \
  voice_nav_interfaces/action/Speak \
  '{source_instance_id: smoke, source_seq: 2, session_id: smoke, turn_id: complete, priority: 1, text: "播放完成。", allow_barge_in: true}' \
  >"$completion_log" 2>&1
grep -q 'code: 0' "$completion_log"
grep -q 'detail: playback completed' "$completion_log"
grep -q 'Feedback:' "$completion_log"
echo 'VOICE_BARGE_STOP=PASS'
grep -E 'SPEAK:|Accepted voice command|Direct voice STOP result' "$voice_log"
grep -E 'code:|detail:' "$action_log" | tail -n 4
grep -E 'Feedback:|code:|detail:' "$completion_log" | tail -n 5
