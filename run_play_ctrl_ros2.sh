#!/usr/bin/env bash

set -Eeuo pipefail
export ROS_DOMAIN_ID=88
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_LOG_DIR="${RUNTIME_LOG_DIR:-$PROJECT_ROOT/log/runtime}"
mkdir -p "$RUNTIME_LOG_DIR"
RUNTIME_RUN_ID="$(date '+%Y%m%d_%H%M%S')_$$"
RUNTIME_CONSOLE_LOG="${RUNTIME_CONSOLE_LOG:-$RUNTIME_LOG_DIR/stack_${RUNTIME_RUN_ID}.log}"
SLAM_DIAGNOSTICS_CSV="${SLAM_DIAGNOSTICS_CSV:-$RUNTIME_LOG_DIR/slam_${RUNTIME_RUN_ID}.csv}"
exec > >(tee -a "$RUNTIME_CONSOLE_LOG") 2>&1
echo "[run_play_ctrl_ros2] runtime log: $RUNTIME_CONSOLE_LOG"
echo "[run_play_ctrl_ros2] SLAM diagnostics: $SLAM_DIAGNOSTICS_CSV"
USER_HOME="$(getent passwd "$(id -u)" | cut -d: -f6)"
CONDA_ROOT_DEFAULT="${CONDA_EXE:+${CONDA_EXE%/bin/conda}}"
CONDA_ROOT_DEFAULT="${CONDA_ROOT_DEFAULT:-$USER_HOME/miniconda3}"
CONDA_SH="${CONDA_SH:-$CONDA_ROOT_DEFAULT/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-env_isaaclab}"
ISAACLAB_ROOT="${ISAACLAB_ROOT:-$USER_HOME/IsaacLab}"
ISAAC_SIM_SETUP="${ISAAC_SIM_SETUP:-$ISAACLAB_ROOT/_isaac_sim/setup_conda_env.sh}"
ROS_DISTRO_DEFAULT="humble"
if [[ ! -f "/opt/ros/$ROS_DISTRO_DEFAULT/setup.bash" ]]; then
    for ros_setup_candidate in /opt/ros/*/setup.bash; do
        [[ -f "$ros_setup_candidate" ]] || continue
        ROS_DISTRO_DEFAULT="$(basename "$(dirname "$ros_setup_candidate")")"
        break
    done
fi
ROS_DISTRO="${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/$ROS_DISTRO/setup.bash}"
ROS_WS_ROOT="${ROS_WS_ROOT:-$PROJECT_ROOT/core_ws}"
AUTO_BUILD_CORE_MSGS="${AUTO_BUILD_CORE_MSGS:-1}"
AUTO_INSTALL_COLCON="${AUTO_INSTALL_COLCON:-1}"
AUTO_INSTALL_ROS_BUILD_DEPS="${AUTO_INSTALL_ROS_BUILD_DEPS:-1}"
AUTO_INSTALL_PYNPUT="${AUTO_INSTALL_PYNPUT:-1}"
AUTO_INSTALL_ONNXRUNTIME="${AUTO_INSTALL_ONNXRUNTIME:-1}"
ENABLE_SLAM="${ENABLE_SLAM:-1}"
ENABLE_NAV2="${ENABLE_NAV2:-$ENABLE_SLAM}"
AUTO_BUILD_SLAM="${AUTO_BUILD_SLAM:-1}"
AUTO_INSTALL_NAV2="${AUTO_INSTALL_NAV2:-1}"
SLAM_ROOT="${SLAM_ROOT:-$PROJECT_ROOT/cocelo-hd-slam-yaw}"
SLAM_CONFIG="${SLAM_CONFIG:-$SLAM_ROOT/config/autonomy_light.yaml}"
SLAM_PATH_TOPIC="${SLAM_PATH_TOPIC:-/path_slam}"
ROBOT_NAMESPACE="${ROBOT_NAMESPACE:-/f4}"
ROBOT_FRAME_PREFIX="${ROBOT_FRAME_PREFIX:-f4/}"
ROBOT_BASE_FRAME="${ROBOT_FRAME_PREFIX}base_link"
ROBOT_MAP_FRAME="${ROBOT_MAP_FRAME:-map}"
ROBOT_ODOM_FRAME="${ROBOT_ODOM_FRAME:-odom}"
ROBOT_WORLD_FRAME="${ROBOT_WORLD_FRAME:-$([[ "$ENABLE_SLAM" == "1" ]] && printf map || printf world)}"
ROBOT_PATH_GT_TOPIC="${ROBOT_PATH_GT_TOPIC:-/path_gt}"
ROBOT_GT_ODOM_TOPIC="${ROBOT_GT_ODOM_TOPIC:-/gt/lidar_odom}"
ROBOT_GT_CHILD_FRAME="${ROBOT_GT_CHILD_FRAME:-${ROBOT_FRAME_PREFIX}lidar_link}"
ROS2_PUBLISH_ROOT_TF="${ROS2_PUBLISH_ROOT_TF:-$([[ "$ENABLE_SLAM" == "1" ]] && printf 0 || printf 1)}"
ROS2_PATH_GT_RELATIVE_TO_INITIAL="${ROS2_PATH_GT_RELATIVE_TO_INITIAL:-$([[ "$ENABLE_SLAM" == "1" ]] && printf 1 || printf 0)}"
ROBOT_URDF="${ROBOT_URDF:-$PROJECT_ROOT/urdf/urdf/2WL_V3_Assem_v2.urdf}"
ROBOT_RVIZ_CONFIG="${ROBOT_RVIZ_CONFIG:-$PROJECT_ROOT/rviz/flamingo_ros2.rviz}"
RUN_RVIZ2="${RUN_RVIZ2:-1}"
KILL_STALE_ROBOT_STATE_PUBLISHERS="${KILL_STALE_ROBOT_STATE_PUBLISHERS:-1}"
ENABLE_ROS2_FRONT_CAMERA="${ENABLE_ROS2_FRONT_CAMERA:-1}"
ENABLE_ROS2_ADAS_CAMERA="${ENABLE_ROS2_ADAS_CAMERA:-1}"
ENABLE_ROS2_IMU="${ENABLE_ROS2_IMU:-1}"
ENABLE_ROS2_LIDAR="${ENABLE_ROS2_LIDAR:-1}"
ENABLE_ROS2_LIDAR_IMU="${ENABLE_ROS2_LIDAR_IMU:-1}"
ENABLE_ROS2_HEIGHT_MAP="${ENABLE_ROS2_HEIGHT_MAP:-0}"
ROS2_CAMERA_RATE="${ROS2_CAMERA_RATE:-30}"
ROS2_CAMERA_WIDTH="${ROS2_CAMERA_WIDTH:-320}"
ROS2_CAMERA_HEIGHT="${ROS2_CAMERA_HEIGHT:-240}"
ROS2_IMU_RATE="${ROS2_IMU_RATE:-100}"
ROS2_LIDAR_RATE="${ROS2_LIDAR_RATE:-10}"
ROS2_NAV_CMD_VEL_TOPIC="${ROS2_NAV_CMD_VEL_TOPIC:-/nav2/cmd_vel}"
ROS2_COMMAND_USER_PUBLISH_KEYBOARD="${ROS2_COMMAND_USER_PUBLISH_KEYBOARD:-$([[ "$ENABLE_NAV2" == "1" ]] && printf 0 || printf 1)}"
POLICY_ONNX_PATH="${POLICY_ONNX_PATH:-$PROJECT_ROOT/weights/example_policy.onnx}"
HW_SHOULDER_KP="${HW_SHOULDER_KP:-35.0}"
HW_SHOULDER_KD="${HW_SHOULDER_KD:-0.45}"
HW_WHEEL_KP="${HW_WHEEL_KP:-0.0}"
HW_WHEEL_KD="${HW_WHEEL_KD:-0.3}"
ACTION_SHOULDER_SCALE="${ACTION_SHOULDER_SCALE:-0.25}"
ACTION_WHEEL_SCALE="${ACTION_WHEEL_SCALE:-40.0}"
OBS_JOINT_POS_SCALE="${OBS_JOINT_POS_SCALE:-1.0}"
OBS_JOINT_VEL_SCALE="${OBS_JOINT_VEL_SCALE:-0.15}"
OBS_BASE_ANG_VEL_SCALE="${OBS_BASE_ANG_VEL_SCALE:-0.25}"
OBS_PROJECTED_GRAVITY_SCALE="${OBS_PROJECTED_GRAVITY_SCALE:-1.0}"
OBS_CMD_VEL_X_SCALE="${OBS_CMD_VEL_X_SCALE:-2.0}"
OBS_CMD_VEL_Y_SCALE="${OBS_CMD_VEL_Y_SCALE:-0.0}"
OBS_CMD_VEL_YAW_SCALE="${OBS_CMD_VEL_YAW_SCALE:-0.25}"

source_if_exists() {
    [[ -f "$1" ]] || return 0
    source "$1"
}

build_ros_pythonpath() {
    local paths=()
    local path
    for path in \
        "$ISAAC_ROS_RCLPY" \
        "$ROS_WS_INSTALL_BASE"/*/local/lib/python*/dist-packages \
        "$ROS_WS_INSTALL_BASE"/*/local/lib/python*/site-packages \
        "$ROS_WS_INSTALL_BASE"/*/lib/python*/dist-packages \
        "$ROS_WS_INSTALL_BASE"/*/lib/python*/site-packages; do
        [[ -d "$path" ]] && paths+=("$path")
    done

    IFS=:
    printf "%s" "${paths[*]}"
    unset IFS
}

require_file() {
    if [[ ! -f "$1" ]]; then
        echo "[run_play_ctrl_ros2] missing: $1" >&2
        exit 1
    fi
}

ensure_colcon() {
    if command -v colcon >/dev/null 2>&1; then
        return 0
    fi

    if [[ "$AUTO_INSTALL_COLCON" == "1" ]] && [[ "$(id -u)" == "0" ]] && command -v apt-get >/dev/null 2>&1; then
        echo "[run_play_ctrl_ros2] colcon not found; installing python3-colcon-common-extensions"
        apt-get update
        apt-get install -y python3-colcon-common-extensions
    fi

    if ! command -v colcon >/dev/null 2>&1; then
        echo "[run_play_ctrl_ros2] missing colcon. Install python3-colcon-common-extensions or set AUTO_BUILD_CORE_MSGS=0." >&2
        exit 1
    fi
}

ensure_ros_build_python() {
    local ros_pythonpath
    ros_pythonpath="$(build_ros_pythonpath)"

    if env \
        -u PYTHONHOME \
        PYTHONPATH="${ros_pythonpath}${PYTHONPATH:+:$PYTHONPATH}" \
        "$RUNTIME_PYTHON" - <<'PY' >/dev/null 2>&1
import catkin_pkg
import em
import lark
import numpy
import rosidl_adapter
PY
    then
        return 0
    fi

    if [[ "$AUTO_INSTALL_ROS_BUILD_DEPS" == "1" ]]; then
        echo "[run_play_ctrl_ros2] installing ROS 2 build dependencies into conda env '$CONDA_ENV'"
        "$RUNTIME_PYTHON" -m pip install "empy==3.3.4" catkin_pkg lark
    fi

    if ! env \
        -u PYTHONHOME \
        PYTHONPATH="${ros_pythonpath}${PYTHONPATH:+:$PYTHONPATH}" \
        "$RUNTIME_PYTHON" - <<'PY' >/dev/null 2>&1
import catkin_pkg
import em
import lark
import numpy
import rosidl_adapter
PY
    then
        echo "[run_play_ctrl_ros2] $RUNTIME_PYTHON cannot import the ROS 2 interface build dependencies." >&2
        echo "[run_play_ctrl_ros2] Install empy==3.3.4, catkin_pkg, and lark in '$CONDA_ENV'." >&2
        exit 1
    fi
}

core_msg_import_ok() {
    local ros_pythonpath
    ros_pythonpath="$(build_ros_pythonpath)"
    LD_LIBRARY_PATH="$ROS_WS_INSTALL_BASE/core/lib:$ISAAC_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
        PYTHONPATH="${ros_pythonpath}${PYTHONPATH:+:$PYTHONPATH}" \
        "$RUNTIME_PYTHON" - <<'PY' >/dev/null 2>&1
import rclpy
from core.msg import CommandUser, EventUser
_ = CommandUser()
_ = EventUser()
PY
}

ensure_core_msgs() {
    if core_msg_import_ok; then
        return 0
    fi

    if [[ "$AUTO_BUILD_CORE_MSGS" != "1" ]]; then
        echo "[run_play_ctrl_ros2] core/msg/CommandUser is not available and AUTO_BUILD_CORE_MSGS=0." >&2
        exit 1
    fi

    if [[ ! -f "$ROS_WS_ROOT/src/core/package.xml" ]]; then
        echo "[run_play_ctrl_ros2] missing bundled ROS 2 package: $ROS_WS_ROOT/src/core/package.xml" >&2
        exit 1
    fi

    echo "[run_play_ctrl_ros2] building bundled ROS 2 core/msg package in $ROS_WS_ROOT"
    ensure_colcon
    ensure_ros_build_python

    (
        cd "$ROS_WS_ROOT"

        # Build the custom interfaces for Isaac Sim's Python ABI. System ROS
        # Jazzy uses Python 3.12 while Isaac Sim 5.1 embeds Python 3.11.
        unset PYTHONHOME
        export PYTHONPATH="$(build_ros_pythonpath)${PYTHONPATH:+:$PYTHONPATH}"
        export LD_LIBRARY_PATH="$ISAAC_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
        hash -r

        colcon --log-base "$ROS_WS_ROOT/log/$ROS_RUNTIME_ID" build \
            --base-paths "$ROS_WS_ROOT/src" \
            --build-base "$ROS_WS_ROOT/build/$ROS_RUNTIME_ID" \
            --install-base "$ROS_WS_INSTALL_BASE" \
            --packages-select core \
            --symlink-install \
            --cmake-clean-cache \
            --cmake-args \
                "-DPython3_EXECUTABLE=$RUNTIME_PYTHON" \
                "-DPYTHON_EXECUTABLE=$RUNTIME_PYTHON"
    )

    if [[ -f "$ROS_WS_SETUP" ]]; then
        # shellcheck disable=SC1090
        source "$ROS_WS_SETUP"
    fi

    if ! core_msg_import_ok; then
        echo "[run_play_ctrl_ros2] failed to import core.msg.CommandUser after building for $RUNTIME_PYTHON" >&2
        exit 1
    fi
}

ensure_pynput() {
    if python - <<'PY' >/dev/null 2>&1
import pynput
PY
    then
        return 0
    fi

    if [[ "$AUTO_INSTALL_PYNPUT" != "1" ]]; then
        echo "[run_play_ctrl_ros2] pynput is not installed; Isaac-window keyboard teleop will not work." >&2
        return 0
    fi

    echo "[run_play_ctrl_ros2] installing pynput into conda env '$CONDA_ENV' for Isaac-window keyboard teleop"
    python -m pip install pynput
}

ensure_onnxruntime() {
    if [[ -z "$POLICY_ONNX_PATH" ]]; then
        return 0
    fi
    if python - <<'PY' >/dev/null 2>&1
import onnxruntime
PY
    then
        return 0
    fi

    if [[ "$AUTO_INSTALL_ONNXRUNTIME" != "1" ]]; then
        echo "[run_play_ctrl_ros2] onnxruntime is not installed and AUTO_INSTALL_ONNXRUNTIME=0." >&2
        exit 1
    fi

    echo "[run_play_ctrl_ros2] installing onnxruntime into conda env '$CONDA_ENV' for ONNX policy inference"
    python -m pip install onnxruntime
}

ensure_slam_workspace() {
    if [[ "$ENABLE_SLAM" != "1" ]]; then
        return 0
    fi
    require_file "$SLAM_ROOT/launch.sh"
    require_file "$SLAM_CONFIG"

    local slam_executable="$SLAM_ROOT/install/super_lio/lib/super_lio/super_lio_node"
    local wall_executable="$SLAM_ROOT/install/autonomy_light/lib/autonomy_light/front_wall_angle_estimator"
    local occupancy_executable="$SLAM_ROOT/install/autonomy_light/lib/autonomy_light/live_occupancy_mapper"
    local occupancy_source="$SLAM_ROOT/src/live_occupancy_mapper.cpp"
    local slam_build_required=0
    if [[ ! -x "$slam_executable" || ! -x "$wall_executable" ]]; then
        slam_build_required=1
    fi
    if [[ "$ENABLE_NAV2" == "1" ]] && \
        { [[ ! -x "$occupancy_executable" ]] || [[ "$occupancy_source" -nt "$occupancy_executable" ]]; }; then
        slam_build_required=1
    fi
    if [[ "$slam_build_required" == "1" ]]; then
        if [[ "$AUTO_BUILD_SLAM" != "1" ]]; then
            echo "[run_play_ctrl_ros2] SLAM is not built and AUTO_BUILD_SLAM=0: $SLAM_ROOT" >&2
            exit 1
        fi

        echo "[run_play_ctrl_ros2] building cocelo-hd-slam-yaw simulation stack"
        "$SLAM_ROOT/build.sh" --sim --skip-apt --ros-distro "$ROS_DISTRO"
        if [[ ! -x "$slam_executable" || ! -x "$wall_executable" ]] || \
            { [[ "$ENABLE_NAV2" == "1" ]] && [[ ! -x "$occupancy_executable" ]]; }; then
            echo "[run_play_ctrl_ros2] SLAM build completed without required executables" >&2
            exit 1
        fi
    fi

    if [[ "$ENABLE_NAV2" == "1" ]]; then
        require_file "$SLAM_ROOT/scripts/setup_nav2.sh"
        if [[ "$AUTO_INSTALL_NAV2" != "1" ]] && \
            [[ ! -x "/opt/ros/$ROS_DISTRO/lib/nav2_bt_navigator/bt_navigator" ]] && \
            [[ ! -x "$SLAM_ROOT/.deps/opt/ros/$ROS_DISTRO/lib/nav2_bt_navigator/bt_navigator" ]]; then
            echo "[run_play_ctrl_ros2] Nav2 is unavailable and AUTO_INSTALL_NAV2=0." >&2
            exit 1
        fi
        if [[ "$AUTO_INSTALL_NAV2" == "1" ]]; then
            "$SLAM_ROOT/scripts/setup_nav2.sh" --ros-distro "$ROS_DISTRO"
        fi
    fi
}

require_file "$CONDA_SH"
require_file "$ISAAC_SIM_SETUP"
require_file "$ROS_SETUP"
require_file "$ROBOT_URDF"

set +u
source "$ROS_SETUP"
SYSTEM_ROS_PATH="$PATH"
SYSTEM_ROS_PYTHONPATH="${PYTHONPATH:-}"
SYSTEM_ROS_LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
SYSTEM_ROS_AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}"
SYSTEM_ROS_CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}"
REQUESTS_HELP=0
for arg in "$@"; do
    if [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
        REQUESTS_HELP=1
    fi
done
if [[ "$REQUESTS_HELP" != "1" ]]; then
    ensure_slam_workspace
fi
source "$CONDA_SH"
conda activate "$CONDA_ENV"
source "$ISAAC_SIM_SETUP"
RUNTIME_PYTHON="$(command -v python)"
PYTHON_ABI_TAG="$($RUNTIME_PYTHON -c 'import sys; print(f"py{sys.version_info.major}{sys.version_info.minor}")')"
ROS_RUNTIME_ID="$ROS_DISTRO-$PYTHON_ABI_TAG"
ROS_WS_SETUP="${ROS_WS_SETUP:-$ROS_WS_ROOT/install/$ROS_RUNTIME_ID/setup.bash}"
ROS_WS_INSTALL_BASE="$(dirname "$ROS_WS_SETUP")"
ISAAC_SIM_ROOT="${ISAACSIM_PATH:-$(readlink -f "$ISAACLAB_ROOT/_isaac_sim")}"
ISAAC_ROS_BRIDGE="${ISAAC_ROS_BRIDGE:-$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge}"
ISAAC_ROS_RCLPY="$ISAAC_ROS_BRIDGE/$ROS_DISTRO/rclpy"
ISAAC_ROS_LIB="$ISAAC_ROS_BRIDGE/$ROS_DISTRO/lib"

require_file "$ISAAC_ROS_RCLPY/rclpy/_rclpy_pybind11.cpython-${PYTHON_ABI_TAG#py}-x86_64-linux-gnu.so"
if [[ ! -d "$ISAAC_ROS_LIB" ]]; then
    echo "[run_play_ctrl_ros2] missing Isaac ROS 2 libraries: $ISAAC_ROS_LIB" >&2
    exit 1
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONPATH="$ISAAC_ROS_RCLPY${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$ISAAC_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ensure_core_msgs
source_if_exists "$ROS_WS_SETUP"
ensure_pynput
ensure_onnxruntime
set -u

cd "$PROJECT_ROOT"

ROS_PYTHONPATH="$(build_ros_pythonpath)"
export PYTHONPATH="$PROJECT_ROOT${ROS_PYTHONPATH:+:$ROS_PYTHONPATH}${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$ROS_WS_INSTALL_BASE/core/lib:$ISAAC_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

START_ROS_HELPERS=1
HEADLESS=0
PLAY_CTRL_PID=""
ROBOT_STATE_PUBLISHER_PID=""
SLAM_LAUNCH_PID=""
SLAM_PROCESS_GROUP_ID=""
RVIZ2_PID=""
CLEANUP_DONE=0
for arg in "$@"; do
    if [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
        START_ROS_HELPERS=0
    fi
    if [[ "$arg" == "--headless" ]]; then
        HEADLESS=1
    fi
done
if [[ "$START_ROS_HELPERS" == "1" && "$ENABLE_SLAM" == "1" ]] && \
    [[ "$ENABLE_ROS2_LIDAR" != "1" || "$ENABLE_ROS2_LIDAR_IMU" != "1" ]]; then
    echo "[run_play_ctrl_ros2] ENABLE_SLAM=1 requires ENABLE_ROS2_LIDAR=1 and ENABLE_ROS2_LIDAR_IMU=1." >&2
    exit 2
fi
echo "[run_play_ctrl_ros2] starting lightweight ROS 2 play control"
echo "[run_play_ctrl_ros2] extra args are passed through after defaults"
if [[ "$HEADLESS" == "1" ]]; then
    echo "[run_play_ctrl_ros2] headless mode enabled; Isaac GUI disabled, rviz2 remains enabled"
fi
TELEOP_USE_STDIN="${TELEOP_USE_STDIN:-$HEADLESS}"

kill_pids() {
    local name="$1"
    shift
    local pids=("$@")

    if [[ "${#pids[@]}" -eq 0 ]]; then
        return 0
    fi

    echo "[run_play_ctrl_ros2] stopping $name process(es): ${pids[*]}"
    kill "${pids[@]}" 2>/dev/null || true
    sleep 1.0

    local alive_pids=()
    local pid
    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            alive_pids+=("$pid")
        fi
    done

    if [[ "${#alive_pids[@]}" -gt 0 ]]; then
        kill -9 "${alive_pids[@]}" 2>/dev/null || true
    fi
}

kill_stale_ros_helpers() {
    local stale_rsp_pids=()
    local stale_slam_pids=()
    local stale_slam_pgids=()
    local stale_rviz_pids=()
    local stale_play_pids=()

    mapfile -t stale_rsp_pids < <(
        ps -eo pid=,cmd= |
            awk '/[r]obot_state_publisher/ && /--params-file \/tmp\/flamingo_rsp_/ { print $1 }'
    )
    mapfile -t stale_rviz_pids < <(
        ps -eo pid=,cmd= |
            awk -v cfg="$ROBOT_RVIZ_CONFIG" '$0 ~ /[r]viz2/ && index($0, cfg) { print $1 }'
    )
    mapfile -t stale_slam_pids < <(
        ps -eo pid=,cmd= |
            awk -v launcher="$SLAM_ROOT/launch.sh" 'index($0, launcher) && $0 !~ /awk/ { print $1 }'
    )
    mapfile -t stale_slam_pgids < <(
        ps -eo pgid=,cmd= |
            awk -v launcher="$SLAM_ROOT/launch.sh" 'index($0, launcher) && $0 !~ /awk/ { print $1 }' |
            sort -u
    )
    mapfile -t stale_play_pids < <(
        ps -eo pid=,cmd= |
            awk '/[p]ython/ && /scripts\/co_rl\/play_ctrl.py/ { print $1 }'
    )

    kill_pids "stale play_ctrl.py" "${stale_play_pids[@]}"
    kill_pids "stale cocelo SLAM launcher" "${stale_slam_pids[@]}"
    local stale_slam_pgid
    for stale_slam_pgid in "${stale_slam_pgids[@]}"; do
        terminate_process_group "$stale_slam_pgid" "stale cocelo SLAM"
    done
    kill_pids "stale robot_state_publisher" "${stale_rsp_pids[@]}"
    kill_pids "stale rviz2" "${stale_rviz_pids[@]}"
}

terminate_process_tree() {
    local pid="$1"
    local name="$2"

    if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
        return 0
    fi

    local child_pids=()
    mapfile -t child_pids < <(pgrep -P "$pid" 2>/dev/null || true)

    if [[ "${#child_pids[@]}" -gt 0 ]]; then
        kill "${child_pids[@]}" 2>/dev/null || true
    fi
    kill "$pid" 2>/dev/null || true

    local deadline=$((SECONDS + 5))
    while kill -0 "$pid" 2>/dev/null && [[ "$SECONDS" -lt "$deadline" ]]; do
        sleep 0.2
    done

    if [[ "${#child_pids[@]}" -gt 0 ]]; then
        local alive_child_pids=()
        local child_pid
        for child_pid in "${child_pids[@]}"; do
            if kill -0 "$child_pid" 2>/dev/null; then
                alive_child_pids+=("$child_pid")
            fi
        done
        if [[ "${#alive_child_pids[@]}" -gt 0 ]]; then
            kill -9 "${alive_child_pids[@]}" 2>/dev/null || true
        fi
    fi
    if kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
    fi

    wait "$pid" 2>/dev/null || true
    echo "[run_play_ctrl_ros2] stopped $name"
}

terminate_supervised_process() {
    local pid="$1"
    local name="$2"

    if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
        return 0
    fi

    # The SLAM launcher supervises several ROS processes and knows their
    # dependency shutdown order. Signal only its parent first so it can clean
    # up without child-side rclpy/rclcpp shutdown races.
    kill -TERM "$pid" 2>/dev/null || true
    local deadline=$((SECONDS + 20))
    while kill -0 "$pid" 2>/dev/null && [[ "$SECONDS" -lt "$deadline" ]]; do
        sleep 0.2
    done
    if kill -0 "$pid" 2>/dev/null; then
        pkill -TERM -P "$pid" 2>/dev/null || true
        kill -KILL "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
    echo "[run_play_ctrl_ros2] stopped $name"
}

terminate_process_group() {
    local pgid="$1"
    local name="$2"
    local own_pgid
    own_pgid="$(ps -o pgid= -p "$$" | tr -d ' ')"
    if [[ ! "$pgid" =~ ^[0-9]+$ ]] || [[ "$pgid" == "$own_pgid" ]]; then
        return 0
    fi
    if ! ps -eo pgid= | awk -v target="$pgid" '$1 == target { found=1 } END { exit !found }'; then
        return 0
    fi
    kill -TERM -- "-$pgid" 2>/dev/null || true
    local deadline=$((SECONDS + 5))
    while ps -eo pgid= | awk -v target="$pgid" '$1 == target { found=1 } END { exit !found }' \
        && [[ "$SECONDS" -lt "$deadline" ]]; do
        sleep 0.2
    done
    if ps -eo pgid= | awk -v target="$pgid" '$1 == target { found=1 } END { exit !found }'; then
        kill -KILL -- "-$pgid" 2>/dev/null || true
    fi
    echo "[run_play_ctrl_ros2] stopped residual $name process group"
}

cleanup() {
    if [[ "$CLEANUP_DONE" == "1" ]]; then
        return 0
    fi
    CLEANUP_DONE=1

    trap - EXIT INT TERM

    terminate_process_tree "$PLAY_CTRL_PID" "play_ctrl.py"
    terminate_supervised_process "$SLAM_LAUNCH_PID" "cocelo SLAM"
    terminate_process_group "$SLAM_PROCESS_GROUP_ID" "cocelo SLAM"
    terminate_process_tree "$ROBOT_STATE_PUBLISHER_PID" "robot_state_publisher"
    terminate_process_tree "$RVIZ2_PID" "rviz2"

    if [[ "$KILL_STALE_ROBOT_STATE_PUBLISHERS" == "1" ]]; then
        kill_stale_ros_helpers
    fi

    rm -f "${RVIZ_URDF:-}" "${RSP_PARAMS:-}"
    if [[ -t 0 ]]; then
        stty sane 2>/dev/null || true
    fi
}

handle_signal() {
    local signal_name="$1"
    echo "[run_play_ctrl_ros2] received $signal_name; shutting down"
    cleanup
    exit 130
}
trap cleanup EXIT
trap 'handle_signal INT' INT
trap 'handle_signal TERM' TERM

if [[ "$START_ROS_HELPERS" == "1" ]] && command -v ros2 >/dev/null 2>&1; then
    if [[ "$KILL_STALE_ROBOT_STATE_PUBLISHERS" == "1" ]]; then
        kill_stale_ros_helpers
    fi

    RVIZ_URDF="$(mktemp /tmp/flamingo_rviz_XXXXXX.urdf)"
    python3 - "$ROBOT_URDF" "$RVIZ_URDF" "$PROJECT_ROOT/urdf" "$ROBOT_FRAME_PREFIX" <<'PY'
import sys
import xml.etree.ElementTree as ET

source_path, output_path, description_root, frame_prefix = sys.argv[1:5]
tree = ET.parse(source_path)
root = tree.getroot()

link_names = set()
for link in root.findall("link"):
    name = link.get("name")
    if name:
        link_names.add(name)
        if not name.startswith(frame_prefix):
            link.set("name", f"{frame_prefix}{name}")

for joint in root.findall("joint"):
    for tag_name in ("parent", "child"):
        tag = joint.find(tag_name)
        if tag is None:
            continue
        link_name = tag.get("link")
        if link_name in link_names and not link_name.startswith(frame_prefix):
            tag.set("link", f"{frame_prefix}{link_name}")

mesh_prefix = "package://2WL_V3_Assem_v3"
file_prefix = f"file://{description_root}"
for mesh in root.findall(".//mesh"):
    filename = mesh.get("filename")
    if filename and filename.startswith(mesh_prefix):
        mesh.set("filename", filename.replace(mesh_prefix, file_prefix, 1))

tree.write(output_path, encoding="utf-8", xml_declaration=True)
PY
    RSP_PARAMS="$(mktemp /tmp/flamingo_rsp_XXXXXX.yaml)"
    {
        printf "/robot_state_publisher:\n"
        printf "  ros__parameters:\n"
        printf "    use_sim_time: true\n"
        printf "    robot_description: |\n"
        sed "s/^/      /" "$RVIZ_URDF"
    } > "$RSP_PARAMS"

    env \
        PATH="$SYSTEM_ROS_PATH" \
        PYTHONPATH="$SYSTEM_ROS_PYTHONPATH" \
        LD_LIBRARY_PATH="$SYSTEM_ROS_LD_LIBRARY_PATH" \
        AMENT_PREFIX_PATH="$SYSTEM_ROS_AMENT_PREFIX_PATH" \
        CMAKE_PREFIX_PATH="$SYSTEM_ROS_CMAKE_PREFIX_PATH" \
        ros2 run robot_state_publisher robot_state_publisher \
        --ros-args \
        -r "joint_states:=${ROBOT_NAMESPACE}/joint_states" \
        --params-file "$RSP_PARAMS" &
    ROBOT_STATE_PUBLISHER_PID=$!
    echo "[run_play_ctrl_ros2] robot_state_publisher: robot_description + TF from ${ROBOT_NAMESPACE}/joint_states"

    if [[ "$ENABLE_SLAM" == "1" ]]; then
        SLAM_LOCAL_ROS_PREFIX="$SLAM_ROOT/.deps/opt/ros/$ROS_DISTRO"
        SLAM_LOCAL_SYSROOT="$SLAM_ROOT/.deps/system/usr"
        SLAM_AMENT_PREFIX_PATH="$SYSTEM_ROS_AMENT_PREFIX_PATH"
        SLAM_CMAKE_PREFIX_PATH="$SYSTEM_ROS_CMAKE_PREFIX_PATH"
        SLAM_LD_LIBRARY_PATH="$SYSTEM_ROS_LD_LIBRARY_PATH"
        SLAM_PYTHONPATH="$SYSTEM_ROS_PYTHONPATH"
        if [[ -d "$SLAM_LOCAL_ROS_PREFIX" ]]; then
            SLAM_AMENT_PREFIX_PATH="$SLAM_LOCAL_ROS_PREFIX${SLAM_AMENT_PREFIX_PATH:+:$SLAM_AMENT_PREFIX_PATH}"
            SLAM_CMAKE_PREFIX_PATH="$SLAM_LOCAL_ROS_PREFIX${SLAM_CMAKE_PREFIX_PATH:+:$SLAM_CMAKE_PREFIX_PATH}"
            SLAM_LD_LIBRARY_PATH="$SLAM_LOCAL_ROS_PREFIX/lib:$SLAM_LOCAL_ROS_PREFIX/lib/x86_64-linux-gnu${SLAM_LD_LIBRARY_PATH:+:$SLAM_LD_LIBRARY_PATH}"
            for slam_python_path in "$SLAM_LOCAL_ROS_PREFIX"/lib/python*/site-packages "$SLAM_LOCAL_ROS_PREFIX"/lib/python*/dist-packages; do
                [[ -d "$slam_python_path" ]] || continue
                SLAM_PYTHONPATH="$slam_python_path${SLAM_PYTHONPATH:+:$SLAM_PYTHONPATH}"
            done
        fi
        if [[ -d "$SLAM_LOCAL_SYSROOT" ]]; then
            SLAM_CMAKE_PREFIX_PATH="$SLAM_LOCAL_SYSROOT${SLAM_CMAKE_PREFIX_PATH:+:$SLAM_CMAKE_PREFIX_PATH}"
            SLAM_LD_LIBRARY_PATH="$SLAM_LOCAL_SYSROOT/lib/x86_64-linux-gnu${SLAM_LD_LIBRARY_PATH:+:$SLAM_LD_LIBRARY_PATH}"
        fi
        SLAM_LAUNCH_COMMAND=("$SLAM_ROOT/launch.sh")
        if command -v setsid >/dev/null 2>&1; then
            SLAM_LAUNCH_COMMAND=(setsid "$SLAM_ROOT/launch.sh")
        fi
        env \
            PATH="$SYSTEM_ROS_PATH" \
            PYTHONPATH="$SLAM_PYTHONPATH" \
            LD_LIBRARY_PATH="$SLAM_LD_LIBRARY_PATH" \
            AMENT_PREFIX_PATH="$SLAM_AMENT_PREFIX_PATH" \
            CMAKE_PREFIX_PATH="$SLAM_CMAKE_PREFIX_PATH" \
            "${SLAM_LAUNCH_COMMAND[@]}" \
                --sim \
                --config "$SLAM_CONFIG" \
                --raw-lidar-topic "${ROBOT_NAMESPACE}/lidar/points" \
                --raw-imu-topic "${ROBOT_NAMESPACE}/lidar/imu" \
                --sim-topic-prefix "$ROBOT_NAMESPACE" \
                --no-lidar-static-tf \
                "$([[ "$ENABLE_NAV2" == "1" ]] && printf '%s' --nav2 || printf '%s' --no-nav2)" \
                --ros-distro "$ROS_DISTRO" &
        SLAM_LAUNCH_PID=$!
        SLAM_PROCESS_GROUP_ID="$(ps -o pgid= -p "$SLAM_LAUNCH_PID" | tr -d ' ')"
        echo "[run_play_ctrl_ros2] SLAM started: map -> odom -> f4/slam_imu_link -> $ROBOT_BASE_FRAME, path: $SLAM_PATH_TOPIC"
        sleep 1
        if ! kill -0 "$SLAM_LAUNCH_PID" 2>/dev/null; then
            wait "$SLAM_LAUNCH_PID" || true
            echo "[run_play_ctrl_ros2] cocelo SLAM launcher exited during startup" >&2
            exit 1
        fi
    fi

    if [[ "$RUN_RVIZ2" == "1" ]] && command -v rviz2 >/dev/null 2>&1; then
        RVIZ_ARGS=()
        [[ -f "$ROBOT_RVIZ_CONFIG" ]] && RVIZ_ARGS=(-d "$ROBOT_RVIZ_CONFIG")
        RVIZ_PYTHONPATH="$SYSTEM_ROS_PYTHONPATH"
        RVIZ_LD_LIBRARY_PATH="$SYSTEM_ROS_LD_LIBRARY_PATH"
        RVIZ_AMENT_PREFIX_PATH="$SYSTEM_ROS_AMENT_PREFIX_PATH"
        RVIZ_CMAKE_PREFIX_PATH="$SYSTEM_ROS_CMAKE_PREFIX_PATH"
        if [[ "$ENABLE_SLAM" == "1" ]]; then
            RVIZ_PYTHONPATH="$SLAM_PYTHONPATH"
            RVIZ_LD_LIBRARY_PATH="$SLAM_LD_LIBRARY_PATH"
            RVIZ_AMENT_PREFIX_PATH="$SLAM_AMENT_PREFIX_PATH"
            RVIZ_CMAKE_PREFIX_PATH="$SLAM_CMAKE_PREFIX_PATH"
        fi
        env \
            PATH="$SYSTEM_ROS_PATH" \
            PYTHONPATH="$RVIZ_PYTHONPATH" \
            LD_LIBRARY_PATH="$RVIZ_LD_LIBRARY_PATH" \
            AMENT_PREFIX_PATH="$RVIZ_AMENT_PREFIX_PATH" \
            CMAKE_PREFIX_PATH="$RVIZ_CMAKE_PREFIX_PATH" \
            rviz2 "${RVIZ_ARGS[@]}" &
        RVIZ2_PID=$!
        echo "[run_play_ctrl_ros2] rviz2 started. Fixed Frame: '$ROBOT_WORLD_FRAME', RobotModel description topic: '/robot_description'"
    fi
elif [[ "$START_ROS_HELPERS" == "1" ]]; then
    echo "[run_play_ctrl_ros2] ros2 command not found; skipping robot_state_publisher/rviz2" >&2
fi

PLAY_CTRL_STDIN="/dev/null"
if [[ "$TELEOP_USE_STDIN" == "1" ]] && [[ -r /dev/tty ]]; then
    PLAY_CTRL_STDIN="/dev/tty"
fi

set +e
python scripts/co_rl/play_ctrl.py \
    --task Isaac-Velocity-Flat-Flamingo-Light-Play-v1-ppo \
    --algo ppo \
    --rendering_mode performance \
    --teleop True \
    --teleop_use_stdin "$TELEOP_USE_STDIN" \
    --policy_onnx_path "$POLICY_ONNX_PATH" \
    --hw_shoulder_kp "$HW_SHOULDER_KP" \
    --hw_shoulder_kd "$HW_SHOULDER_KD" \
    --hw_wheel_kp "$HW_WHEEL_KP" \
    --hw_wheel_kd "$HW_WHEEL_KD" \
    --action_shoulder_scale "$ACTION_SHOULDER_SCALE" \
    --action_wheel_scale "$ACTION_WHEEL_SCALE" \
    --obs_joint_pos_scale "$OBS_JOINT_POS_SCALE" \
    --obs_joint_vel_scale "$OBS_JOINT_VEL_SCALE" \
    --obs_base_ang_vel_scale "$OBS_BASE_ANG_VEL_SCALE" \
    --obs_projected_gravity_scale "$OBS_PROJECTED_GRAVITY_SCALE" \
    --obs_cmd_vel_x_scale "$OBS_CMD_VEL_X_SCALE" \
    --obs_cmd_vel_y_scale "$OBS_CMD_VEL_Y_SCALE" \
    --obs_cmd_vel_yaw_scale "$OBS_CMD_VEL_YAW_SCALE" \
    --num_envs 1 \
    --enable_cameras \
    --num_policy_stacks 2 \
    --num_critic_stacks 2 \
    --enable_ros2_robot_state True \
    --ros2_joint_states_topic "${ROBOT_NAMESPACE}/joint_states" \
    --ros2_world_frame_id "$ROBOT_WORLD_FRAME" \
    --ros2_base_frame_id "$ROBOT_BASE_FRAME" \
    --ros2_path_gt_topic "$ROBOT_PATH_GT_TOPIC" \
    --ros2_gt_odom_topic "$ROBOT_GT_ODOM_TOPIC" \
    --ros2_gt_child_frame_id "$ROBOT_GT_CHILD_FRAME" \
    --ros2_slam_diagnostics_csv "$SLAM_DIAGNOSTICS_CSV" \
    --ros2_publish_root_tf "$ROS2_PUBLISH_ROOT_TF" \
    --ros2_path_gt_relative_to_initial "$ROS2_PATH_GT_RELATIVE_TO_INITIAL" \
    --enable_ros2_depth "$ENABLE_ROS2_FRONT_CAMERA" \
    --ros2_front_rgb_topic "${ROBOT_NAMESPACE}/front_camera/rgb/image_raw" \
    --ros2_depth_topic "${ROBOT_NAMESPACE}/front_camera/depth/image_rect_raw" \
    --ros2_camera_info_topic "${ROBOT_NAMESPACE}/front_camera/depth/camera_info" \
    --ros2_depth_frame_id "${ROBOT_FRAME_PREFIX}F_camera_link" \
    --ros2_camera_rate "$ROS2_CAMERA_RATE" \
    --ros2_camera_width "$ROS2_CAMERA_WIDTH" \
    --ros2_camera_height "$ROS2_CAMERA_HEIGHT" \
    --enable_ros2_auto_camera "$ENABLE_ROS2_ADAS_CAMERA" \
    --ros2_auto_depth_topic "${ROBOT_NAMESPACE}/adas_camera/depth/image_rect_raw" \
    --ros2_auto_depth_camera_info_topic "${ROBOT_NAMESPACE}/adas_camera/depth/camera_info" \
    --ros2_auto_depth_frame_id "${ROBOT_FRAME_PREFIX}A_camera_link" \
    --ros2_auto_rgb_topic "${ROBOT_NAMESPACE}/adas_camera/rgb/image_raw" \
    --ros2_auto_rgb_camera_info_topic "${ROBOT_NAMESPACE}/adas_camera/rgb/camera_info" \
    --ros2_auto_rgb_frame_id "${ROBOT_FRAME_PREFIX}A_camera_link" \
    --enable_ros2_stereo False \
    --enable_ros2_imu "$ENABLE_ROS2_IMU" \
    --ros2_imu_topic "${ROBOT_NAMESPACE}/imu" \
    --ros2_imu_frame_id "$ROBOT_BASE_FRAME" \
    --ros2_imu_rate "$ROS2_IMU_RATE" \
    --enable_ros2_height_map "$ENABLE_ROS2_HEIGHT_MAP" \
    --enable_ros2_mid360 "$ENABLE_ROS2_LIDAR" \
    --enable_ros2_mid360_rtx False \
    --ros2_mid360_lidar_topic "${ROBOT_NAMESPACE}/lidar/points" \
    --ros2_mid360_frame_id "${ROBOT_FRAME_PREFIX}lidar_link" \
    --ros2_mid360_sensor lidar \
    --ros2_mid360_rate "$ROS2_LIDAR_RATE" \
    --enable_ros2_mid360_imu "$ENABLE_ROS2_LIDAR_IMU" \
    --ros2_mid360_imu_topic "${ROBOT_NAMESPACE}/lidar/imu" \
    --ros2_mid360_imu_sensor lidar_imu \
    --enable_ros2_front_camera_imu False \
    --enable_ros2_command_user True \
    --ros2_command_user_topic "/control_command/user_odom" \
    --ros2_command_user_publish_keyboard "$ROS2_COMMAND_USER_PUBLISH_KEYBOARD" \
    --ros2_nav_cmd_vel_topic "$ROS2_NAV_CMD_VEL_TOPIC" \
    "$@" < "$PLAY_CTRL_STDIN" &
PLAY_CTRL_PID=$!
wait "$PLAY_CTRL_PID"
PLAY_CTRL_STATUS=$?
PLAY_CTRL_PID=""
set -e
exit "$PLAY_CTRL_STATUS"
