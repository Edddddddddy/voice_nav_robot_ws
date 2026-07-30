#!/usr/bin/env bash

set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd -- "${script_dir}/.." && pwd)"
ros_distro="${ROS_DISTRO:-jazzy}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS setup not found: ${ros_setup}" >&2
  exit 2
fi

# ROS setup scripts are not guaranteed to be nounset-safe.
set +u
source "${ros_setup}"
set -u

cd "${workspace_root}"

export XML_CATALOG_FILES="${workspace_root}/tools/schema/catalog.xml"

echo "[1/4] Checking declared dependencies"
rosdep check --from-paths src --ignore-src

echo "[2/4] Validating the robot model contract"
robot_xacro="src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro"
robot_urdf="$(mktemp /tmp/voice-nav-model.XXXXXX.urdf)"
robot_sdf="$(mktemp /tmp/voice-nav-model.XXXXXX.sdf)"

cleanup() {
  rm -f -- "${robot_urdf}" "${robot_sdf}"
}
trap cleanup EXIT

xacro "${robot_xacro}" > "${robot_urdf}"
check_urdf "${robot_urdf}"
gz sdf -p "${robot_urdf}" > "${robot_sdf}"

require_sdf_text() {
  local expected="$1"
  if ! grep -Fq -- "${expected}" "${robot_sdf}"; then
    echo "Missing SDF contract text: ${expected}" >&2
    exit 3
  fi
}

require_sdf_text "filename='gz-sim-diff-drive-system'"
require_sdf_text "<wheel_separation>0.4</wheel_separation>"
require_sdf_text "<wheel_radius>0.035</wheel_radius>"
require_sdf_text "<frame_id>odom</frame_id>"
require_sdf_text "<child_frame_id>base_footprint</child_frame_id>"
require_sdf_text "<mu>0.001</mu>"
require_sdf_text "<mu2>0.001</mu2>"

package_args=("$@")

echo "[3/4] Building"
if (( ${#package_args[@]} > 0 )); then
  colcon build \
    --packages-up-to "${package_args[@]}" \
    --symlink-install \
    --event-handlers console_direct+
else
  colcon build \
    --symlink-install \
    --event-handlers console_direct+
fi

set +u
source install/setup.bash
set -u

echo "[4/4] Testing"
if (( ${#package_args[@]} > 0 )); then
  colcon test \
    --packages-select "${package_args[@]}" \
    --executor sequential \
    --event-handlers console_direct+
else
  colcon test \
    --executor sequential \
    --event-handlers console_direct+
fi

colcon test-result --verbose
echo "VoiceNav Robot verification passed."
