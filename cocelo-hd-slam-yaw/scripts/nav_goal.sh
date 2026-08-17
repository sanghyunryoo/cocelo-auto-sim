#!/usr/bin/env bash
set -Eeuo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 X Y [YAW_DEG]" >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
LOCAL_PREFIX="${PROJECT_DIR}/.deps/opt/ros/${ROS_DISTRO_NAME}"

set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
[[ -f "${PROJECT_DIR}/install/setup.bash" ]] && source "${PROJECT_DIR}/install/setup.bash"
set -u
export AMENT_PREFIX_PATH="${LOCAL_PREFIX}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${LOCAL_PREFIX}/lib:${LOCAL_PREFIX}/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
for python_path in "${LOCAL_PREFIX}"/lib/python*/site-packages "${LOCAL_PREFIX}"/lib/python*/dist-packages; do
  [[ -d "${python_path}" ]] || continue
  export PYTHONPATH="${python_path}${PYTHONPATH:+:${PYTHONPATH}}"
done
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"

read -r qz qw < <(/usr/bin/python3 - "${3:-}" <<'PY'
import math
import sys
yaw = math.radians(float(sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else 0.0))
print(math.sin(yaw * 0.5), math.cos(yaw * 0.5))
PY
)

action_output="$(
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2, z: 0.0}, orientation: {z: ${qz}, w: ${qw}}}}}"
)"
printf '%s\n' "${action_output}"

if [[ "${action_output}" != *"Goal finished with status: SUCCEEDED"* ]]; then
  exit 1
fi
