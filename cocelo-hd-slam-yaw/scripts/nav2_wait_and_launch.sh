#!/usr/bin/env bash
set -Eeuo pipefail

PARAMS_FILE="${1:?Nav2 parameter file is required}"
USE_SIM_TIME="${2:-false}"
ODOM_TOPIC="${3:-/lio/odom}"
WAIT_TIMEOUT="${4:-120}"

echo "autonomy-nav2: waiting for ${ODOM_TOPIC} before lifecycle activation"
if ! timeout "${WAIT_TIMEOUT}" ros2 topic echo --once "${ODOM_TOPIC}" >/dev/null 2>&1; then
  echo "error: no LIO odometry received on ${ODOM_TOPIC} within ${WAIT_TIMEOUT}s" >&2
  exit 1
fi

echo "autonomy-nav2: odometry ready; starting Nav2"
exec ros2 launch autonomy_light nav2_live.launch.py \
  "params_file:=${PARAMS_FILE}" "use_sim_time:=${USE_SIM_TIME}"
