#!/usr/bin/env bash

set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_SH="${CONDA_SH:-/root/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-env_isaaclab}"
ISAACLAB_ROOT="${ISAACLAB_ROOT:-/root/IsaacLab}"
ISAAC_SIM_SETUP="${ISAAC_SIM_SETUP:-$ISAACLAB_ROOT/_isaac_sim/setup_conda_env.sh}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/$ROS_DISTRO/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-/root/ros2_ws/install/setup.bash}"
ROS_WS_ROOT="${ROS_WS_ROOT:-$(dirname "$(dirname "$ROS_WS_SETUP")")}"
ROS_PYTHON="${ROS_PYTHON:-/usr/bin/python3}"
AUTO_BUILD_CORE_MSGS="${AUTO_BUILD_CORE_MSGS:-1}"
AUTO_INSTALL_COLCON="${AUTO_INSTALL_COLCON:-1}"
AUTO_INSTALL_PYNPUT="${AUTO_INSTALL_PYNPUT:-1}"
AUTO_INSTALL_ONNXRUNTIME="${AUTO_INSTALL_ONNXRUNTIME:-1}"
ROBOT_NAMESPACE="${ROBOT_NAMESPACE:-/f4}"
ROBOT_FRAME_PREFIX="${ROBOT_FRAME_PREFIX:-f4/}"
ROBOT_BASE_FRAME="${ROBOT_FRAME_PREFIX}base_link"
ROBOT_WORLD_FRAME="${ROBOT_WORLD_FRAME:-world}"
ROBOT_PATH_GT_TOPIC="${ROBOT_PATH_GT_TOPIC:-/path_gt}"
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
ROS2_LIDAR_RATE="${ROS2_LIDAR_RATE:-5}"
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
        /opt/ros/"$ROS_DISTRO"/local/lib/python*/dist-packages \
        /opt/ros/"$ROS_DISTRO"/lib/python*/dist-packages \
        "$ROS_WS_ROOT"/install/*/local/lib/python*/dist-packages \
        "$ROS_WS_ROOT"/install/*/lib/python*/site-packages; do
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

write_core_msg_package() {
    local pkg_dir="$ROS_WS_ROOT/src/core"
    mkdir -p "$pkg_dir/msg"

    cat > "$pkg_dir/msg/EventUser.msg" <<'EOF'
bool estop
bool wake
bool sleep
bool rough_drive_toggle
EOF

    cat > "$pkg_dir/msg/CommandUser.msg" <<'EOF'
nav_msgs/Odometry odom
EventUser event
EOF

    cat > "$pkg_dir/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.8)
project(core)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EventUser.msg"
  "msg/CommandUser.msg"
  DEPENDENCIES nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
EOF

    cat > "$pkg_dir/package.xml" <<'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>core</name>
  <version>0.0.0</version>
  <description>Custom command user messages.</description>
  <maintainer email="root@localhost.localdomain">root</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
}

core_msg_import_ok() {
    local ros_pythonpath
    ros_pythonpath="$(build_ros_pythonpath)"
    PYTHONPATH="${ros_pythonpath}${PYTHONPATH:+:$PYTHONPATH}" "$ROS_PYTHON" - <<'PY' >/dev/null 2>&1
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

    echo "[run_play_ctrl_ros2] preparing ROS 2 core/msg CommandUser package in $ROS_WS_ROOT"
    write_core_msg_package
    ensure_colcon

    (
        cd "$ROS_WS_ROOT"
        colcon build --base-paths "$ROS_WS_ROOT/src" --packages-select core --symlink-install
    )

    if [[ -f "$ROS_WS_SETUP" ]]; then
        # shellcheck disable=SC1090
        source "$ROS_WS_SETUP"
    fi

    if ! core_msg_import_ok; then
        echo "[run_play_ctrl_ros2] failed to import core.msg.CommandUser after building $ROS_WS_ROOT/src/core with $ROS_PYTHON" >&2
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

require_file "$CONDA_SH"
require_file "$ISAAC_SIM_SETUP"
require_file "$ROS_SETUP"
require_file "$ROS_PYTHON"
require_file "$ROBOT_URDF"

set +u
source "$ROS_SETUP"
source_if_exists "$ROS_WS_SETUP"
ensure_core_msgs
source "$CONDA_SH"
conda activate "$CONDA_ENV"
source "$ISAAC_SIM_SETUP"
ensure_pynput
ensure_onnxruntime
source "$ROS_SETUP"
source_if_exists "$ROS_WS_SETUP"
set -u

cd "$PROJECT_ROOT"

ROS_PYTHONPATH="$(build_ros_pythonpath)"
export PYTHONPATH="$PROJECT_ROOT${ROS_PYTHONPATH:+:$ROS_PYTHONPATH}${PYTHONPATH:+:$PYTHONPATH}"

START_ROS_HELPERS=1
HEADLESS=0
PLAY_CTRL_PID=""
ROBOT_STATE_PUBLISHER_PID=""
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
    mapfile -t stale_play_pids < <(
        ps -eo pid=,cmd= |
            awk '/[p]ython/ && /scripts\/co_rl\/play_ctrl.py/ { print $1 }'
    )

    kill_pids "stale play_ctrl.py" "${stale_play_pids[@]}"
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

cleanup() {
    if [[ "$CLEANUP_DONE" == "1" ]]; then
        return 0
    fi
    CLEANUP_DONE=1

    trap - EXIT INT TERM

    terminate_process_tree "$PLAY_CTRL_PID" "play_ctrl.py"
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

    ros2 run robot_state_publisher robot_state_publisher \
        --ros-args \
        -r "joint_states:=${ROBOT_NAMESPACE}/joint_states" \
        --params-file "$RSP_PARAMS" &
    ROBOT_STATE_PUBLISHER_PID=$!
    echo "[run_play_ctrl_ros2] robot_state_publisher: robot_description + TF from ${ROBOT_NAMESPACE}/joint_states"

    if [[ "$RUN_RVIZ2" == "1" ]] && command -v rviz2 >/dev/null 2>&1; then
        RVIZ_ARGS=()
        [[ -f "$ROBOT_RVIZ_CONFIG" ]] && RVIZ_ARGS=(-d "$ROBOT_RVIZ_CONFIG")
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
    --ros2_command_user_publish_keyboard True \
    "$@" < "$PLAY_CTRL_STDIN"
PLAY_CTRL_STATUS=$?
exit "$PLAY_CTRL_STATUS"
