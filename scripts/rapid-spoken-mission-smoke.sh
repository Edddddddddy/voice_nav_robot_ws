#!/usr/bin/env bash
set -eo pipefail

workspace=${1:-/mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_rapid}
voice_root=${VOICE_NAV_VOICE_ROOT:-$workspace/.deps/voice}
run_dir=$(mktemp -d /tmp/voice-nav-spoken-mission.XXXXXX)
capture_fifo=$run_dir/capture.pcm
wav=$run_dir/command.wav
log=$run_dir/stack.log

cd "$workspace"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

printf '小智小智前进一米\n' | "$voice_root/bin/piper" \
  --model "$voice_root/models/zh_CN-huayan-medium.onnx" \
  --output_file "$wav"

setsid ros2 launch voice_nav_bringup rapid_agent_stack.launch.py \
  mode:=mapping llm_enabled:=false audio_enabled:=false \
  use_sim_time:=false capture_fifo:="$capture_fifo" >"$log" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 200); do
  if grep -q '"backend": "sherpa"' "$log" && \
      ros2 action info /mission/execute 2>/dev/null | \
        grep -q 'Action servers: 1'; then
    break
  fi
  sleep 0.1
done
grep -q '"backend": "sherpa"' "$log"

PYTHONWARNINGS=ignore::DeprecationWarning \
  python3 - "$wav" "$capture_fifo" <<'PY'
import audioop
from pathlib import Path
import sys
import wave

with wave.open(sys.argv[1], 'rb') as source:
    pcm = source.readframes(source.getnframes())
    if source.getnchannels() != 1:
        raise SystemExit('Piper fixture must be mono')
    pcm, _ = audioop.ratecv(
        pcm, source.getsampwidth(), 1, source.getframerate(), 16000, None
    )
with Path(sys.argv[2]).open('wb', buffering=0) as sink:
    sink.write(pcm)
    sink.write(bytes(32000))
PY

for _ in $(seq 1 300); do
  grep -q 'SPEAK: 任务已完成。' "$log" && break
  sleep 0.1
done

grep -q 'Accepted voice command: 前进一米' "$log"
grep -q 'Rapid relative motion cycles=' "$log"
grep -q 'SPEAK: 任务已完成。' "$log"
echo "SPOKEN_MISSION_E2E=PASS log=$log"
grep -E 'wake|Accepted voice command|Mission feedback|relative motion|SPEAK:' \
  "$log" | tail -n 12
