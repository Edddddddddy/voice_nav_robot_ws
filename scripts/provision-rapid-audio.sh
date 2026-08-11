#!/usr/bin/env bash
set -euo pipefail

mode=${1:---check}
packages=(portaudio19-dev libwebrtc-audio-processing-dev pkg-config)

if [[ "$mode" == "--install" ]]; then
  sudo apt-get update
  sudo apt-get install -y "${packages[@]}"
elif [[ "$mode" != "--check" ]]; then
  echo "usage: $0 [--check|--install]" >&2
  exit 2
fi

for package in "${packages[@]}"; do
  if ! dpkg-query -W -f='${db:Status-Abbrev}' "$package" 2>/dev/null | \
      grep -q '^ii '; then
    echo "missing package: $package" >&2
    echo "run: bash scripts/provision-rapid-audio.sh --install" >&2
    exit 1
  fi
done

portaudio_version=$(pkg-config --modversion portaudio-2.0)
webrtc_version=$(pkg-config --modversion webrtc-audio-processing)
pkg-config --cflags --libs portaudio-2.0 webrtc-audio-processing >/dev/null

echo "RAPID_AUDIO_READY portaudio=$portaudio_version webrtc_apm=$webrtc_version"
