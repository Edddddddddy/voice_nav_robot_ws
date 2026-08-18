#!/usr/bin/env bash
set -euo pipefail

usage() {
  printf 'Usage: %s mapping|navigation\n' "$0" >&2
}

if (($# != 1)); then
  usage
  exit 2
fi

case "$1" in
  mapping|navigation) mode="$1" ;;
  *)
    usage
    exit 2
    ;;
esac

exec ros2 run voice_nav_bringup voice_nav_app \
  --mode "$mode" \
  --display headless \
  --input vad-auto
