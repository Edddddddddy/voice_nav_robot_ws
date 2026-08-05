#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(git rev-parse --show-toplevel)"
temporary_root="$(mktemp -d /tmp/voice-nav-l0009-clean.XXXXXX)"

cleanup() {
  local resolved_root
  resolved_root="$(realpath "${temporary_root}")"
  case "${resolved_root}" in
    /tmp/voice-nav-l0009-clean.*)
      rm -rf -- "${resolved_root}"
      ;;
    *)
      printf 'Refusing to remove unexpected temporary path: %s\n' \
        "${resolved_root}" >&2
      return 1
      ;;
  esac
}
trap cleanup EXIT

resolved_root="$(realpath "${temporary_root}")"
case "${resolved_root}" in
  /tmp/voice-nav-l0009-clean.*) ;;
  *)
    printf 'Unexpected temporary path: %s\n' "${resolved_root}" >&2
    exit 1
    ;;
esac

# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
set -u

colcon --log-base "${resolved_root}/log" build \
  --base-paths "${workspace_root}/src" \
  --packages-select voice_nav_mission \
  --build-base "${resolved_root}/build" \
  --install-base "${resolved_root}/install" \
  --event-handlers console_direct+

ctest --test-dir "${resolved_root}/build/voice_nav_mission" \
  --output-on-failure \
  -R '^(motion_gate_core_test|writer_observation_test)$'

leaked_core="$(
  find "${resolved_root}/install" \
    \( -type f -o -type l \) \
    \( \
      -name 'motion_gate_core.hpp' -o \
      -name 'writer_observation.hpp' -o \
      -name 'writer_observation.cpp' -o \
      -name 'libmotion_gate_core.a' -o \
      -name 'libmotion_gate_core.so' \
    \) \
    -print -quit
)"
if [[ -n "${leaked_core}" ]]; then
  printf 'Internal MotionGate Core leaked into clean install: %s\n' \
    "${leaked_core}" >&2
  exit 1
fi

leaked_export="$(
  grep -RIl \
    --include='*.cmake' \
    -- 'motion_gate_core' \
    "${resolved_root}/install" \
    2>/dev/null \
    | head -n 1 \
    || true
)"
if [[ -n "${leaked_export}" ]]; then
  printf 'Internal MotionGate Core leaked through CMake export metadata: %s\n' \
    "${leaked_export}" >&2
  exit 1
fi

node_path="$(
  find "${resolved_root}/install" \
    -type f \
    -path '*/lib/voice_nav_mission/motion_gate_node' \
    -print -quit
)"
if [[ -z "${node_path}" || ! -x "${node_path}" ]]; then
  printf 'motion_gate_node is missing from clean install\n' >&2
  exit 1
fi

printf '%s\n' \
  'Clean MotionGate install audit passed: internals private, node installed.'
