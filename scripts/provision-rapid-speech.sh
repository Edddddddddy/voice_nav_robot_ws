#!/usr/bin/env bash
set -euo pipefail

workspace=${1:-/mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws_rapid}
root=${VOICE_NAV_VOICE_ROOT:-$workspace/.deps/voice}
models=$root/models
scratch=$(mktemp -d)
trap 'rm -rf "$scratch"' EXIT

download() {
  local url=$1 output=$2 digest=$3
  curl -L --fail --retry 3 -o "$output" "$url"
  printf '%s  %s\n' "$digest" "$output" | sha256sum --check --status
}

mkdir -p "$models"
"$root/bin/pip" install \
  click==8.1.8 pypinyin==0.53.0 sentencepiece==0.2.0 \
  sherpa-onnx==1.13.4

kws=sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20
if [[ ! -d "$models/$kws" ]]; then
  download \
    "https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/$kws.tar.bz2" \
    "$scratch/kws.tar.bz2" \
    68447f4fbc67e70eee3a93961f36e81e98f47aef73ce7e7ca00885c6cd3616a6
  tar xjf "$scratch/kws.tar.bz2" -C "$models"
fi

vad=$models/silero_vad.int8.onnx
if [[ ! -f "$vad" ]]; then
  download \
    https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/silero_vad.int8.onnx \
    "$vad" \
    c36d490aff5ab924ca6c7aeec4d8f6bd3d22db6fa17611b9c5b17eae58ac3a20
fi

asr=sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30
if [[ ! -d "$models/$asr" ]]; then
  download \
    "https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/$asr.tar.bz2" \
    "$scratch/asr.tar.bz2" \
    5a2832047ea1f97dd0dc595b816c230c4bafad65cfc0341fa57517cadc50afd0
  tar xjf "$scratch/asr.tar.bz2" -C "$models"
fi

tts=vits-piper-zh_CN-chaowen-medium-int8
if [[ ! -d "$models/$tts" ]]; then
  download \
    "https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/$tts.tar.bz2" \
    "$scratch/tts.tar.bz2" \
    f5f7c8628427fbb259ea4b7ec1a9a822a0c04e3f267071f0abfa0610371d9e0c
  tar xjf "$scratch/tts.tar.bz2" -C "$models"
fi

"$root/bin/sherpa-onnx-cli" text2token \
  --tokens "$models/$kws/tokens.txt" \
  --tokens-type phone+ppinyin \
  --lexicon "$models/$kws/en.phone" \
  "$workspace/src/voice_nav_agent/config/rapid_keywords_raw.txt" \
  "$models/rapid_keywords.txt"

echo "RAPID_SPEECH_READY=$root"
