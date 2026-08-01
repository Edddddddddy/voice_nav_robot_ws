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

echo "[1/6] Checking repository and course contracts"
python3 scripts/check_repository.py
python3 scripts/check_motion_gate_contract.py --root .
python3 scripts/check_gazebo_teardown_contract.py --root .
python3 scripts/run_repository_tests.py

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

workspace_package_output=""
if ! workspace_package_output="$(
  colcon list --base-paths src --names-only
)"; then
  echo "Failed to discover ROS packages under src" >&2
  exit 5
fi

workspace_packages=()
if [[ -n "${workspace_package_output}" ]]; then
  mapfile -t workspace_packages <<< "${workspace_package_output}"
fi
if (( ${#workspace_packages[@]} == 0 )); then
  echo "No ROS packages discovered under src" >&2
  exit 5
fi

build_boundary_args=("--build-base" "build")
for package_name in "${workspace_packages[@]}"; do
  build_boundary_args+=("--package" "${package_name}")
done
python3 scripts/check_colcon_build_boundary.py "${build_boundary_args[@]}"

echo "[2/6] Checking declared dependencies"
rosdep check --from-paths src --ignore-src

echo "[3/6] Validating the robot model contract"
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
if (( ${#package_args[@]} > 0 )); then
  test_packages=("${package_args[@]}")
else
  test_packages=("${workspace_packages[@]}")
fi

if (( ${#test_packages[@]} == 0 )); then
  echo "No ROS packages selected for verification" >&2
  exit 5
fi

test_result_args=("--build-base" "build")
for package_name in "${test_packages[@]}"; do
  test_result_args+=("--package" "${package_name}")
done

echo "[4/6] Building"
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

echo "[5/6] Testing"
python3 scripts/check_colcon_build_boundary.py "${build_boundary_args[@]}"
python3 scripts/check_generated_launch_tests.py "${test_result_args[@]}"
python3 scripts/report_test_results.py "${test_result_args[@]}" --clear
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

python3 scripts/check_colcon_build_boundary.py "${build_boundary_args[@]}"
python3 scripts/report_test_results.py "${test_result_args[@]}"
echo "[6/6] Auditing the clean MotionGate install boundary"
bash scripts/check_clean_motion_gate_install.sh
echo "VoiceNav Robot verification passed."
