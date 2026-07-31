#!/usr/bin/env bash

set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd -- "${script_dir}/.." && pwd)"
ros_distro="jazzy"
if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${ros_distro}" ]]; then
  echo "VoiceNav Robot requires ROS_DISTRO=jazzy; found ${ROS_DISTRO}" >&2
  exit 2
fi
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
export PYTHONDONTWRITEBYTECODE=1

echo "[1/5] Checking repository and course contracts"
python3 scripts/check_repository.py
python3 scripts/check_motion_gate_contract.py --root .
python3 -m unittest discover -s tests -p "test_*.py" -v

tracked_generated="$(
  git ls-files -- "build/**" "install/**" "log/**"
)"
if [[ -n "${tracked_generated}" ]]; then
  echo "Generated workspace outputs are tracked:" >&2
  echo "${tracked_generated}" >&2
  exit 4
fi

git diff --check
git diff --cached --check

echo "[2/5] Checking declared dependencies"
rosdep check --from-paths src --ignore-src

echo "[3/5] Validating the robot model contract"
robot_xacro="src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro"
controllers_yaml="src/voice_nav_sim/config/controllers.yaml"
bridge_yaml="src/voice_nav_sim/config/bridge.yaml"
simulation_world="src/voice_nav_sim/worlds/voice_nav_test_world.sdf"
robot_urdf="$(mktemp /tmp/voice-nav-model.XXXXXX.urdf)"
robot_sdf="$(mktemp /tmp/voice-nav-model.XXXXXX.sdf)"

cleanup() {
  rm -f -- "${robot_urdf}" "${robot_sdf}"
}
trap cleanup EXIT

python3 scripts/check_control_contract.py \
  --robot-description "${robot_xacro}" \
  --controllers "${controllers_yaml}" \
  --package src/voice_nav_sim/package.xml \
  --cmake src/voice_nav_sim/CMakeLists.txt \
  --launch src/voice_nav_sim/launch/simulation.launch.py

python3 scripts/check_simulation_contract.py \
  --launch src/voice_nav_sim/launch/simulation.launch.py \
  --world "${simulation_world}" \
  --robot-description "${robot_xacro}" \
  --bridge "${bridge_yaml}" \
  --package src/voice_nav_sim/package.xml \
  --cmake src/voice_nav_sim/CMakeLists.txt

xacro \
  "${robot_xacro}" \
  "controllers_file:=$(realpath "${controllers_yaml}")" \
  > "${robot_urdf}"
check_urdf "${robot_urdf}"
gz sdf -p "${robot_urdf}" > "${robot_sdf}"
python3 scripts/check_sdf_contract.py "${robot_sdf}"
gz sdf -k "${simulation_world}"

package_args=("$@")

echo "[4/5] Building"
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

echo "[5/5] Testing"
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
