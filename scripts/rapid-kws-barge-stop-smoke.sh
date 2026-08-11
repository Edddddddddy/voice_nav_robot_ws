#!/usr/bin/env bash
set -eo pipefail

workspace=${1:-/mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_rapid}
voice_root=${VOICE_NAV_VOICE_ROOT:-$workspace/.deps/voice}
run_dir=$(mktemp -d /tmp/voice-nav-kws-barge-stop.XXXXXX)
capture_fifo=$run_dir/capture.pcm
log=$run_dir/stack.log

cd "$workspace"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

synthesize_fixture() {
  local text=$1 name=$2
  printf '%s' "$text" | "$voice_root/bin/python" -u \
    src/voice_nav_agent/voice_nav_agent/sherpa_tts_worker.py \
    "$voice_root/models/vits-piper-zh_CN-chaowen-medium-int8" \
    "$run_dir/$name.wav"
  PYTHONWARNINGS=ignore::DeprecationWarning python3 - \
    "$run_dir/$name.wav" "$run_dir/$name.pcm" <<'PY'
import audioop
from pathlib import Path
import sys
import wave

with wave.open(sys.argv[1], 'rb') as source:
    pcm = source.readframes(source.getnframes())
    pcm, _ = audioop.ratecv(
        pcm, source.getsampwidth(), source.getnchannels(),
        source.getframerate(), 16000, None
    )
Path(sys.argv[2]).write_bytes(pcm + bytes(32000))
PY
}

synthesize_fixture '小智' wake
synthesize_fixture '紧急停止' stop

setsid ros2 launch voice_nav_bringup rapid_agent_stack.launch.py \
  mode:=mapping llm_enabled:=false audio_enabled:=false \
  use_sim_time:=false capture_fifo:="$capture_fifo" \
  >"$log" 2>&1 &
launch_pid=$!

cleanup() {
  exec 4>&- 2>/dev/null || true
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 300); do
  if grep -q '"backend": "sherpa"' "$log" && \
      ros2 action info /voice/speak 2>/dev/null | \
        grep -q 'Action servers: 1' && \
      ros2 service type /mission/stop 2>/dev/null | \
        grep -q StopMission; then
    break
  fi
  sleep 0.1
done
grep -q '"backend": "sherpa"' "$log"
grep -q 'Rapid TTS backend=chaowen_int8' "$log"
exec 4>"$capture_fifo"

before=$(timeout 5 ros2 topic echo --once /mission/state | \
  awk '/admission_epoch:/ {print $2; exit}')

ros2 action send_goal --feedback /voice/speak \
  voice_nav_interfaces/action/Speak \
  '{source_instance_id: smoke, source_seq: 1, session_id: smoke, turn_id: wake, priority: 1, text: "这是一段允许被普通唤醒词打断的长语音。", allow_barge_in: true}' \
  >"$run_dir/wake-action.log" 2>&1 &
wake_action_pid=$!
for _ in $(seq 1 100); do
  grep -q 'SPEAK: 这是一段' "$log" && break
  sleep 0.05
done
dd if="$run_dir/wake.pcm" status=none >&4
wait "$wake_action_pid"
grep -q 'code: 2' "$run_dir/wake-action.log"

ros2 action send_goal --feedback /voice/speak \
  voice_nav_interfaces/action/Speak \
  '{source_instance_id: smoke, source_seq: 2, session_id: smoke, turn_id: stop, priority: 1, text: "这是一段不允许被普通唤醒词打断但必须响应紧急停止的长语音。", allow_barge_in: false}' \
  >"$run_dir/stop-action.log" 2>&1 &
stop_action_pid=$!
for _ in $(seq 1 100); do
  grep -q 'SPEAK: 这是一段不允许' "$log" && break
  sleep 0.05
done
dd if="$run_dir/stop.pcm" status=none >&4
wait "$stop_action_pid"

for _ in $(seq 1 100); do
  if grep -q 'Direct voice STOP result code=0 inhibited=True' "$log" && \
      grep -q 'SPEAK: 已停止。' "$log"; then
    break
  fi
  sleep 0.05
done
grep -q 'code: 2' "$run_dir/stop-action.log"
grep -q 'Direct voice STOP result code=0 inhibited=True' "$log"
grep -q 'Accepted voice command: 紧急停止' "$log"
grep -q 'SPEAK: 已停止。' "$log"

after=$(timeout 5 ros2 topic echo --once /mission/state | \
  awk '/admission_epoch:/ {print $2; exit}')
test "$after" -eq "$((before + 1))"

echo "KWS_BARGE_STOP=PASS epoch=$before->$after log=$log"
grep -E '"wake"|SPEAK:|Accepted voice command|Direct voice STOP' \
  "$log" | tail -n 12
grep -E 'code:|detail:' "$run_dir/wake-action.log" | tail -n 4
grep -E 'code:|detail:' "$run_dir/stop-action.log" | tail -n 4
