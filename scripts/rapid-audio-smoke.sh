#!/usr/bin/env bash
set -e

workspace=${1:-/mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_rapid}
run_dir=$(mktemp -d /tmp/voice-nav-audio-smoke.XXXXXX)
fifo=$run_dir/capture.pcm
pcm=$run_dir/sample.pcm
log=$run_dir/audio.log

cd "$workspace"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkfifo "$fifo"

dd if="$fifo" of="$pcm" bs=320 count=100 status=none &
reader_pid=$!
setsid ros2 run voice_nav_audio audio_engine_node --ros-args \
  -p capture_fifo:="$fifo" >"$log" 2>&1 &
audio_pid=$!

cleanup() {
  kill -INT -- "-$audio_pid" 2>/dev/null || true
  sleep 1
  kill -TERM -- "-$audio_pid" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 150); do
  kill -0 "$reader_pid" 2>/dev/null || break
  sleep 0.1
done
if kill -0 "$reader_pid" 2>/dev/null; then
  echo 'AudioEngine did not deliver 100 PCM frames' >&2
  exit 1
fi
wait "$reader_pid"
sleep 2.2
test "$(wc -c <"$pcm")" -eq 32000
grep -Eq 'fifo_frames=[1-9][0-9]*' "$log"
echo "AUDIO_FIFO=PASS bytes=32000 artifact=$pcm"
grep -E 'full-duplex AudioEngine started|audio blocks=' "$log" | tail -n 3
