#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./launch.sh [--real|--sim] [--no-drivers] [options]

  --real                  Start Livox and Super-LIO (default).
  --sim                   Use Livox CustomMsg simulation input; do not start Livox.
  --no-drivers            Use external LIO only when startup INIT is disabled.
  --config FILE           Override autonomy_light.yaml.
  --raw-lidar-topic TOPIC Override the Super-LIO LiDAR input topic.
  --raw-imu-topic TOPIC   Override the Super-LIO IMU input topic.
  --sim-topic-prefix PFX  Simulation prefix (default: /f4).
  --no-static-tf          Do not publish imu -> base_link -> lidar_link.
  --no-lidar-static-tf    Keep imu -> base_link, but let robot_state_publisher own base_link -> lidar_link.
  --nav2                  Start live-cloud Nav2 navigation (default).
  --no-nav2               Run SLAM without Nav2.
  --nav2-config FILE      Override nav2_live.yaml.
  --command-user-topic T  CommandUser output topic (default: /control_command/user_odom).
  --vis-rate HZ           Set status-table output frequency (default: 2.0 Hz).
  --ros-distro NAME       ROS distribution (default: $ROS_DISTRO or humble).
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_CONFIG_DIR="${SCRIPT_DIR}/config"
# In a colcon install, launch.sh lives in <prefix>/lib/autonomy_light while
# the installed config directory is <prefix>/share/autonomy_light/config.
if [[ ! -f "${PACKAGE_CONFIG_DIR}/super_lio_mid360.yaml" ]]; then
  INSTALLED_CONFIG_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)/share/autonomy_light/config"
  if [[ -f "${INSTALLED_CONFIG_DIR}/super_lio_mid360.yaml" ]]; then
    PACKAGE_CONFIG_DIR="${INSTALLED_CONFIG_DIR}"
  fi
fi
MODE="real"
NO_DRIVERS="false"
NO_STATIC_TF="false"
NO_LIDAR_STATIC_TF="false"
ENABLE_NAV2="${AUTONOMY_LIGHT_ENABLE_NAV2:-true}"
VIS_RATE="2.0"
CONFIG_FILE="${AUTONOMY_LIGHT_CONFIG:-${PACKAGE_CONFIG_DIR}/autonomy_light.yaml}"
NAV2_CONFIG_FILE="${AUTONOMY_LIGHT_NAV2_CONFIG:-${PACKAGE_CONFIG_DIR}/nav2_live.yaml}"
COMMAND_USER_TOPIC="${AUTONOMY_LIGHT_COMMAND_USER_TOPIC:-/control_command/user_odom}"
RAW_LIDAR_TOPIC=""
RAW_IMU_TOPIC=""
SIM_TOPIC_PREFIX="${AUTONOMY_LIGHT_SIM_TOPIC_PREFIX:-/f4}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --real) MODE="real"; shift ;;
    --sim) MODE="sim"; shift ;;
    --no-drivers) NO_DRIVERS="true"; shift ;;
    --no-static-tf) NO_STATIC_TF="true"; shift ;;
    --no-lidar-static-tf) NO_LIDAR_STATIC_TF="true"; shift ;;
    --nav2) ENABLE_NAV2="true"; shift ;;
    --no-nav2) ENABLE_NAV2="false"; shift ;;
    --nav2-config) NAV2_CONFIG_FILE="${2:?--nav2-config requires a file}"; shift 2 ;;
    --nav2-config=*) NAV2_CONFIG_FILE="${1#*=}"; shift ;;
    --command-user-topic) COMMAND_USER_TOPIC="${2:?--command-user-topic requires a topic}"; shift 2 ;;
    --command-user-topic=*) COMMAND_USER_TOPIC="${1#*=}"; shift ;;
    --vis-rate) VIS_RATE="${2:?--vis-rate requires a positive frequency}"; shift 2 ;;
    --vis-rate=*) VIS_RATE="${1#*=}"; shift ;;
    --config) CONFIG_FILE="${2:?--config requires a file}"; shift 2 ;;
    --config=*) CONFIG_FILE="${1#*=}"; shift ;;
    --raw-lidar-topic) RAW_LIDAR_TOPIC="${2:?--raw-lidar-topic requires a topic}"; shift 2 ;;
    --raw-lidar-topic=*) RAW_LIDAR_TOPIC="${1#*=}"; shift ;;
    --raw-imu-topic) RAW_IMU_TOPIC="${2:?--raw-imu-topic requires a topic}"; shift 2 ;;
    --raw-imu-topic=*) RAW_IMU_TOPIC="${1#*=}"; shift ;;
    --sim-topic-prefix) SIM_TOPIC_PREFIX="${2:?--sim-topic-prefix requires a prefix}"; shift 2 ;;
    --sim-topic-prefix=*) SIM_TOPIC_PREFIX="${1#*=}"; shift ;;
    --ros-distro) ROS_DISTRO_NAME="${2:?--ros-distro requires a name}"; shift 2 ;;
    --ros-distro=*) ROS_DISTRO_NAME="${1#*=}"; shift ;;
    *) echo "error: unsupported option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -f "${CONFIG_FILE}" ]] || { echo "error: config not found: ${CONFIG_FILE}" >&2; exit 1; }
if [[ "${ENABLE_NAV2}" == "true" ]]; then
  [[ -f "${NAV2_CONFIG_FILE}" ]] || { echo "error: Nav2 config not found: ${NAV2_CONFIG_FILE}" >&2; exit 1; }
fi

ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
[[ -f "${ROS_SETUP}" ]] || { echo "error: ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }
set +u
# shellcheck source=/dev/null
source "${ROS_SETUP}"
LOCAL_ROS_PREFIX="${SCRIPT_DIR}/.deps/opt/ros/${ROS_DISTRO_NAME}"
LOCAL_SYSROOT="${SCRIPT_DIR}/.deps/system/usr"
if [[ -d "${LOCAL_ROS_PREFIX}" ]]; then
  export CMAKE_PREFIX_PATH="${LOCAL_ROS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export AMENT_PREFIX_PATH="${LOCAL_ROS_PREFIX}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${LOCAL_ROS_PREFIX}/lib:${LOCAL_ROS_PREFIX}/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  for python_path in "${LOCAL_ROS_PREFIX}"/lib/python*/site-packages "${LOCAL_ROS_PREFIX}"/lib/python*/dist-packages; do
    [[ -d "${python_path}" ]] || continue
    export PYTHONPATH="${python_path}${PYTHONPATH:+:${PYTHONPATH}}"
  done
fi
if [[ -d "${LOCAL_SYSROOT}" ]]; then
  export CMAKE_PREFIX_PATH="${LOCAL_SYSROOT}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${LOCAL_SYSROOT}/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
[[ -f "${SCRIPT_DIR}/install/setup.bash" ]] && source "${SCRIPT_DIR}/install/setup.bash"
set -u
command -v ros2 >/dev/null || { echo "error: ros2 is unavailable" >&2; exit 1; }

RUNTIME_DIR="$(mktemp -d "/tmp/autonomy_light_$(id -u)_XXXXXX")"
LIO_CONFIG="${RUNTIME_DIR}/super_lio.yaml"
WALL_INIT_CONFIG="${RUNTIME_DIR}/front_wall_angle_initializer.yaml"
WALL_RUNTIME_CONFIG="${RUNTIME_DIR}/front_wall_angle_runtime.yaml"
LIVOX_CONFIG="${RUNTIME_DIR}/livox_driver.json"
STATIC_TF="${RUNTIME_DIR}/static_tf.txt"
RUNTIME_INFO="${RUNTIME_DIR}/runtime.env"
NAV2_RUNTIME_CONFIG="${RUNTIME_DIR}/nav2_live.yaml"

/usr/bin/python3 - "${CONFIG_FILE}" "${PACKAGE_CONFIG_DIR}/super_lio_mid360.yaml" \
  "${LIO_CONFIG}" "${WALL_INIT_CONFIG}" "${WALL_RUNTIME_CONFIG}" "${LIVOX_CONFIG}" "${STATIC_TF}" "${RUNTIME_INFO}" \
  "${NAV2_CONFIG_FILE}" "${NAV2_RUNTIME_CONFIG}" \
  "${MODE}" "${RAW_LIDAR_TOPIC}" "${RAW_IMU_TOPIC}" "${SIM_TOPIC_PREFIX}" "${COMMAND_USER_TOPIC}" <<'PY'
import ipaddress
import json
import math
import os
import sys
import yaml

(source_path, lio_default, lio_target, wall_init_target, wall_runtime_target,
 livox_target, tf_target, runtime_target, nav2_source, nav2_target, mode,
 lidar_override, imu_override, sim_prefix, command_user_topic) = sys.argv[1:16]

def read_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}

def params(document, node):
    return document.setdefault(node, {}).setdefault("ros__parameters", {})

def vector(values, name):
    if not isinstance(values, list) or len(values) != 3:
        raise SystemExit(f"error: {name} must contain exactly three numeric values")
    return [float(value) for value in values]

def required_text(value, name):
    result = str(value if value is not None else "").strip()
    if not result:
        raise SystemExit(f"error: {name} must not be empty")
    return result

def ipv4(value, name):
    text = required_text(value, name)
    try:
        return str(ipaddress.IPv4Address(text))
    except ipaddress.AddressValueError as error:
        raise SystemExit(f"error: {name} must be an IPv4 address: {error}")

def positive_float(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise SystemExit(f"error: {name} must be numeric: {error}")
    if not math.isfinite(result) or result <= 0.0:
        raise SystemExit(f"error: {name} must be a finite value greater than zero")
    return result

def integer(value, name):
    try:
        return int(value)
    except (TypeError, ValueError) as error:
        raise SystemExit(f"error: {name} must be an integer: {error}")

def port_map(value, name):
    if not isinstance(value, dict):
        raise SystemExit(f"error: {name} must be a mapping")
    names = ("cmd_data", "push_msg", "point_data", "imu_data", "log_data")
    result = {}
    for port_name in names:
        port = integer(value.get(port_name), f"{name}.{port_name}")
        if not 1 <= port <= 65535:
            raise SystemExit(f"error: {name}.{port_name} must be in 1..65535")
        result[port_name] = port
    return result

def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)

def multiply(first, second):
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)

def inverse(quaternion):
    x, y, z, w = quaternion
    return (-x, -y, -z, w)

def rotate(quaternion, point):
    qpoint = (point[0], point[1], point[2], 0.0)
    result = multiply(multiply(quaternion, qpoint), inverse(quaternion))
    return result[:3]

def matrix_from_quaternion(x, y, z, w):
    return [
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
        2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
        2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y),
    ]

root = params(read_yaml(source_path), "autonomy_light")
simulation = root.get("simulation", {}) if mode == "sim" else {}
if not isinstance(simulation, dict):
    raise SystemExit("error: autonomy_light.simulation must be a mapping")
livox = root.get("livox")
if not isinstance(livox, dict):
    raise SystemExit("error: livox must be a mapping in autonomy_light.yaml")
livox_model = required_text(livox.get("model"), "livox.model").lower().replace("-", "")
if livox_model not in ("mid360", "mid360s"):
    raise SystemExit("error: livox.model must be mid360 or mid360s")
livox_interface = required_text(livox.get("interface"), "livox.interface")
livox_host_ip = ipv4(livox.get("host_ip"), "livox.host_ip")
livox_lidar_ip = ipv4(livox.get("lidar_ip"), "livox.lidar_ip")
livox_lidar_topic = required_text(
    livox.get("lidar_topic", root.get("raw_lidar_topic", "/livox/lidar")), "livox.lidar_topic")
livox_imu_topic = required_text(
    livox.get("imu_topic", root.get("raw_imu_topic", "/livox/imu")), "livox.imu_topic")
livox_frame_id = required_text(livox.get("frame_id", "livox_frame"), "livox.frame_id")
livox_publish_freq = positive_float(livox.get("publish_freq", 50.0), "livox.publish_freq")
livox_pcl_data_type = integer(livox.get("pcl_data_type", 1), "livox.pcl_data_type")
livox_pattern_mode = integer(livox.get("pattern_mode", 0), "livox.pattern_mode")
livox_xfer_format = integer(livox.get("xfer_format", 1), "livox.xfer_format")
livox_multi_topic = integer(livox.get("multi_topic", 0), "livox.multi_topic")
livox_data_src = integer(livox.get("data_src", 0), "livox.data_src")
livox_output_data_type = integer(livox.get("output_data_type", 0), "livox.output_data_type")
livox_cmdline_bd_code = required_text(
    livox.get("cmdline_input_bd_code", "livox0000000001"), "livox.cmdline_input_bd_code")
livox_lidar_ports = port_map(livox.get("lidar_ports"), "livox.lidar_ports")
livox_host_ports = port_map(livox.get("host_ports"), "livox.host_ports")
lidar_topic = lidar_override or livox_lidar_topic
imu_topic = imu_override or livox_imu_topic
if mode == "sim":
    prefix = sim_prefix.rstrip("/")
    lidar_topic = lidar_override or simulation.get("lidar_topic", f"{prefix}/lidar/points")
    imu_topic = imu_override or simulation.get("imu_topic", f"{prefix}/lidar/imu")

base_to_lidar = vector(simulation.get("target_to_lidar_xyz", root.get("target_to_lidar_xyz")), "target_to_lidar_xyz")
base_to_lidar_q = quaternion_from_rpy(*vector(simulation.get("target_to_lidar_rpy", root.get("target_to_lidar_rpy")), "target_to_lidar_rpy"))
imu_from_lidar = vector(simulation.get("imu_from_lidar_xyz", root.get("imu_from_lidar_xyz")), "imu_from_lidar_xyz")
imu_from_lidar_q = quaternion_from_rpy(*vector(simulation.get("imu_from_lidar_rpy", root.get("imu_from_lidar_rpy")), "imu_from_lidar_rpy"))

# T_base_imu = T_base_lidar * inverse(T_imu_lidar); publish imu -> base_link.
lidar_from_imu_q = inverse(imu_from_lidar_q)
lidar_from_imu_t = rotate(lidar_from_imu_q, [-value for value in imu_from_lidar])
base_from_imu_q = multiply(base_to_lidar_q, lidar_from_imu_q)
base_from_imu_t = [base_to_lidar[index] + rotate(base_to_lidar_q, lidar_from_imu_t)[index] for index in range(3)]
imu_to_base_q = inverse(base_from_imu_q)
imu_to_base_t = rotate(imu_to_base_q, [-value for value in base_from_imu_t])

lio_source = root.get("super_lio_config_file") or lio_default
if not os.path.isfile(lio_source):
    raise SystemExit(f"error: Super-LIO config not found: {lio_source}")
lio = read_yaml(lio_source)
lio_params = params(lio, "/**")
lio_params["lio.ros.lidar_topic"] = lidar_topic
lio_params["lio.ros.imu_topic"] = imu_topic
# Both MID360 variants publish the same Livox CustomMsg. Network sockets are
# owned by Driver2, but preserve the selected transport identity in Super-LIO
# as inspectable runtime parameters so both nodes use one YAML source of truth.
lio_params["lio.sensor.lidar_type"] = 1
lio_params["lio.ros.livox_model"] = livox_model
lio_params["lio.ros.livox_interface"] = livox_interface
lio_params["lio.ros.livox_host_ip"] = livox_host_ip
lio_params["lio.ros.livox_lidar_ip"] = livox_lidar_ip
lio_params["lio.ros.global_frame"] = simulation.get("map_frame", root.get("map_frame", "map"))
lio_params["lio.ros.odom_frame"] = simulation.get("odom_frame", root.get("odom_frame", "odom"))
lio_params["lio.ros.imu_frame"] = simulation.get("imu_frame", root.get("imu_frame", "imu"))
lio_params["lio.extrinsic.lidar_imu"] = imu_from_lidar + matrix_from_quaternion(*imu_from_lidar_q)
lio_params["lio.sensor.lidar_type"] = int(simulation.get("lidar_type", 3 if mode == "sim" else 1))
lio_params["lio.sensor.gravity_norm"] = float(
    simulation.get("gravity_norm", lio_params.get("lio.sensor.gravity_norm", 9.81)))
for key, default in (
    ("imu_init_samples", 100),
    ("imu_init_acc_norm_tolerance", 0.75),
    ("imu_init_acc_std_threshold", 0.50),
    ("imu_init_gyro_threshold", 0.05),
    ("imu_init_gyro_std_threshold", 0.03),
):
    value = simulation.get(key, default)
    lio_params[f"lio.sensor.{key}"] = int(value) if key == "imu_init_samples" else float(value)
lio_params["lio.kf.max_scan_correction_translation"] = float(
    simulation.get("max_scan_correction_translation", 0.0))
lio_params["lio.kf.max_scan_correction_rotation_deg"] = float(
    simulation.get("max_scan_correction_rotation_deg", 0.0))
lio_params["lio.slam.enable"] = bool(simulation.get("slam_enable", root.get("slam_enable", True)))
lio_params["lio.slam.max_correction_translation"] = float(
    simulation.get("max_correction_translation", 0.0))
lio_params["lio.slam.max_correction_rotation_deg"] = float(
    simulation.get("max_correction_rotation_deg", 0.0))
lio_params["use_sim_time"] = mode == "sim"

wall = read_yaml(source_path)
wall_params = params(wall, "front_wall_angle_estimator")
wall_params["base_frame"] = simulation.get("target_frame", root.get("target_frame", "base_link"))
wall_params["use_sim_time"] = mode == "sim"
startup_alignment_required = bool(
    simulation.get("require_initial_alignment", wall_params.get("require_initial_alignment", True))
)
wall_init_params = dict(wall_params)
wall_init_params["require_initial_alignment"] = startup_alignment_required
wall_runtime_params = dict(wall_params)
# The temporary SLAM instance already verified startup alignment. The final
# SLAM instance must begin publishing immediately after its clean restart.
wall_runtime_params["require_initial_alignment"] = False

# Preserve every validated Nav2 tuning value and rewrite only deployment
# identities. Simulation uses f4/* frames while the physical stack uses the
# unprefixed calibrated frames from autonomy_light.yaml.
nav2 = read_yaml(nav2_source)
nav2_map_frame = simulation.get("map_frame", root.get("map_frame", "map"))
nav2_odom_frame = simulation.get("odom_frame", root.get("odom_frame", "odom"))
nav2_base_frame = simulation.get("target_frame", root.get("target_frame", "base_link"))
nav2_lidar_frame = simulation.get("lidar_frame", root.get("lidar_frame", "lidar_link"))
params(nav2, "live_occupancy_mapper")["map_frame"] = nav2_map_frame
params(nav2, "live_occupancy_mapper")["sensor_frame"] = nav2_lidar_frame
params(nav2, "nav2_command_user_bridge")["output_command_topic"] = required_text(
    command_user_topic, "command_user_topic")
params(nav2, "nav2_command_user_bridge")["odom_frame"] = nav2_odom_frame
params(nav2, "nav2_command_user_bridge")["child_frame"] = nav2_base_frame
params(nav2["local_costmap"], "local_costmap")["global_frame"] = nav2_odom_frame
params(nav2["local_costmap"], "local_costmap")["robot_base_frame"] = nav2_base_frame
params(nav2["global_costmap"], "global_costmap")["global_frame"] = nav2_map_frame
params(nav2["global_costmap"], "global_costmap")["robot_base_frame"] = nav2_base_frame
params(nav2, "behavior_server")["local_frame"] = nav2_odom_frame
params(nav2, "behavior_server")["global_frame"] = nav2_map_frame
params(nav2, "behavior_server")["robot_base_frame"] = nav2_base_frame
params(nav2, "bt_navigator")["global_frame"] = nav2_map_frame
params(nav2, "bt_navigator")["robot_base_frame"] = nav2_base_frame
params(nav2, "bt_navigator")["odom_topic"] = "/lio/odom"

lidar_ports = {f"{name}_port": port for name, port in livox_lidar_ports.items()}
host_ports = {f"{name}_port": port for name, port in livox_host_ports.items()}
if livox_model == "mid360":
    livox_driver_config = {
        "lidar_summary_info": {"lidar_type": 8},
        "MID360": {
            "lidar_net_info": lidar_ports,
            "host_net_info": {
                "cmd_data_ip": livox_host_ip,
                "cmd_data_port": host_ports["cmd_data_port"],
                "push_msg_ip": livox_host_ip,
                "push_msg_port": host_ports["push_msg_port"],
                "point_data_ip": livox_host_ip,
                "point_data_port": host_ports["point_data_port"],
                "imu_data_ip": livox_host_ip,
                "imu_data_port": host_ports["imu_data_port"],
                "log_data_ip": "",
                "log_data_port": host_ports["log_data_port"],
            },
        },
    }
else:
    livox_driver_config = {
        "lidar_summary_info": {"lidar_type": 8},
        "Mid360s": {
            "lidar_net_info": lidar_ports,
            "host_net_info": [{"host_ip": livox_host_ip, **host_ports}],
        },
    }
livox_driver_config["lidar_configs"] = [{
    "ip": livox_lidar_ip,
    "pcl_data_type": livox_pcl_data_type,
    "pattern_mode": livox_pattern_mode,
    # The calibrated LiDAR->IMU transform is owned by Super-LIO config above.
    "extrinsic_parameter": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                            # Livox Driver2 parses translations with GetInt().
                            "x": 0, "y": 0, "z": 0},
}]

with open(lio_target, "w", encoding="utf-8") as stream:
    yaml.safe_dump(lio, stream, default_flow_style=False, sort_keys=False)
with open(wall_init_target, "w", encoding="utf-8") as stream:
    yaml.safe_dump({"front_wall_angle_estimator": {"ros__parameters": wall_init_params}}, stream,
                   default_flow_style=False, sort_keys=False)
with open(wall_runtime_target, "w", encoding="utf-8") as stream:
    yaml.safe_dump({"front_wall_angle_estimator": {"ros__parameters": wall_runtime_params}}, stream,
                   default_flow_style=False, sort_keys=False)
with open(nav2_target, "w", encoding="utf-8") as stream:
    yaml.safe_dump(nav2, stream, default_flow_style=False, sort_keys=False)
with open(livox_target, "w", encoding="utf-8") as stream:
    json.dump(livox_driver_config, stream, indent=2)
    stream.write("\n")
with open(tf_target, "w", encoding="utf-8") as stream:
    stream.write(" ".join(str(value) for value in (*imu_to_base_t, *imu_to_base_q)) + "\n")
    stream.write(" ".join(str(value) for value in (*base_to_lidar, *base_to_lidar_q)) + "\n")
with open(runtime_target, "w", encoding="utf-8") as stream:
    stream.write(f"{int(root.get('ros_domain_id', 0))}\n")
    stream.write(f"{simulation.get('imu_frame', root.get('imu_frame', 'imu'))}\n")
    stream.write(f"{simulation.get('target_frame', root.get('target_frame', 'base_link'))}\n")
    stream.write(f"{simulation.get('lidar_frame', root.get('lidar_frame', 'lidar_link'))}\n")
    stream.write(f"{wall_params.get('input_topic', '/lio/cloud_world')}\n")
    stream.write(f"{wall_params.get('output_topic', '/autonomy_light/front_wall_angle')}\n")
    stream.write(f"{int(startup_alignment_required)}\n")
    stream.write(f"{livox_model}\n")
    stream.write(f"{livox_interface}\n")
    stream.write(f"{livox_host_ip}\n")
    stream.write(f"{livox_lidar_ip}\n")
    stream.write(f"{livox_frame_id}\n")
    stream.write(f"{livox_publish_freq}\n")
    stream.write(f"{livox_xfer_format}\n")
    stream.write(f"{livox_multi_topic}\n")
    stream.write(f"{livox_data_src}\n")
    stream.write(f"{livox_output_data_type}\n")
    stream.write(f"{livox_cmdline_bd_code}\n")
    stream.write(f"{lidar_topic}\n")
    stream.write(f"{imu_topic}\n")
PY

mapfile -t RUNTIME_VALUES < "${RUNTIME_INFO}"
ROS_DOMAIN_ID="${RUNTIME_VALUES[0]}"
IMU_FRAME="${RUNTIME_VALUES[1]}"
BASE_FRAME="${RUNTIME_VALUES[2]}"
LIDAR_FRAME="${RUNTIME_VALUES[3]}"
WALL_INPUT_TOPIC="${RUNTIME_VALUES[4]}"
WALL_OUTPUT_TOPIC="${RUNTIME_VALUES[5]}"
STARTUP_ALIGNMENT_REQUIRED="${RUNTIME_VALUES[6]}"
LIVOX_MODEL="${RUNTIME_VALUES[7]}"
LIVOX_INTERFACE="${RUNTIME_VALUES[8]}"
LIVOX_HOST_IP="${RUNTIME_VALUES[9]}"
LIVOX_LIDAR_IP="${RUNTIME_VALUES[10]}"
LIVOX_FRAME_ID="${RUNTIME_VALUES[11]}"
LIVOX_PUBLISH_FREQ="${RUNTIME_VALUES[12]}"
LIVOX_XFER_FORMAT="${RUNTIME_VALUES[13]}"
LIVOX_MULTI_TOPIC="${RUNTIME_VALUES[14]}"
LIVOX_DATA_SRC="${RUNTIME_VALUES[15]}"
LIVOX_OUTPUT_DATA_TYPE="${RUNTIME_VALUES[16]}"
LIVOX_CMDLINE_BD_CODE="${RUNTIME_VALUES[17]}"
SUPER_LIO_LIDAR_TOPIC="${RUNTIME_VALUES[18]}"
SUPER_LIO_IMU_TOPIC="${RUNTIME_VALUES[19]}"
export ROS_DOMAIN_ID
read -r IMU_BASE_X IMU_BASE_Y IMU_BASE_Z IMU_BASE_QX IMU_BASE_QY IMU_BASE_QZ IMU_BASE_QW < "${STATIC_TF}"
read -r BASE_LIDAR_X BASE_LIDAR_Y BASE_LIDAR_Z BASE_LIDAR_QX BASE_LIDAR_QY BASE_LIDAR_QZ BASE_LIDAR_QW < <(sed -n '2p' "${STATIC_TF}")

declare -a PIDS=()
declare -A PID_LABELS=()
start() {
  local label="$1"
  shift
  echo "autonomy-slam: starting ${label}"
  "$@" &
  local pid="$!"
  PIDS+=("${pid}")
  PID_LABELS["${pid}"]="${label}"
}

stop_process() {
  local pid="$1"
  local label="${PID_LABELS[${pid}]:-process ${pid}}"

  # Async children of a non-interactive shell can inherit SIGINT=ignored.
  # Use TERM here, so both the normal Ctrl-C cleanup and the INIT handoff
  # reliably stop the actual ROS executable rather than only its wrapper.
  if ! kill -0 "${pid}" 2>/dev/null; then
    wait "${pid}" 2>/dev/null || true
    return 0
  fi
  echo "autonomy-slam: stopping ${label}"
  pkill -TERM -P "${pid}" 2>/dev/null || true
  kill -TERM "${pid}" 2>/dev/null || true
  for _ in {1..20}; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      return 0
    fi
    sleep 0.2
  done
  echo "autonomy-slam: ${label} did not stop after 4 s; forcing shutdown" >&2
  pkill -KILL -P "${pid}" 2>/dev/null || true
  kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  trap - EXIT INT TERM
  # Stop dependants before their providers: monitor -> estimator -> LIO -> driver.
  local index
  for ((index = ${#PIDS[@]} - 1; index >= 0; --index)); do
    stop_process "${PIDS[${index}]}"
  done
  find "${RUNTIME_DIR}" -depth -delete 2>/dev/null || true
}

handle_signal() {
  local status="$1"
  cleanup
  exit "${status}"
}

trap cleanup EXIT
trap 'handle_signal 130' INT
trap 'handle_signal 143' TERM

verify_running() {
  local pid="$1"
  local label="${PID_LABELS[${pid}]:-process ${pid}}"
  sleep 2
  if kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi

  local status=1
  if wait "${pid}"; then
    status=0
  else
    status="$?"
  fi
  echo "error: ${label} exited during startup (exit status ${status})." >&2
  echo "error: startup handoff aborted; inspect the ROS error immediately above." >&2
  exit "${status}"
}

echo "autonomy-slam: mode=${MODE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} config=${CONFIG_FILE}"
echo "autonomy-slam: Nav2=$([[ "${ENABLE_NAV2}" == "true" ]] && echo enabled || echo disabled)"
echo "front-wall angle: ${WALL_INPUT_TOPIC} -> ${WALL_OUTPUT_TOPIC}"
echo "Livox: model=${LIVOX_MODEL} interface=${LIVOX_INTERFACE} host=${LIVOX_HOST_IP} lidar=${LIVOX_LIDAR_IP}"
if [[ "${MODE}" == "sim" ]]; then
  echo "Super-LIO: PointCloud2=${SUPER_LIO_LIDAR_TOPIC} IMU=${SUPER_LIO_IMU_TOPIC}"
else
  echo "Super-LIO: Livox CustomMsg=${SUPER_LIO_LIDAR_TOPIC} IMU=${SUPER_LIO_IMU_TOPIC} model=${LIVOX_MODEL}"
fi

if [[ "${NO_DRIVERS}" == "true" && "${STARTUP_ALIGNMENT_REQUIRED}" == "1" ]]; then
  echo "error: --no-drivers cannot guarantee a pre-SLAM yaw reset." >&2
  echo "error: let launch.sh own Super-LIO so it can restart it after INIT succeeds." >&2
  exit 2
fi

if [[ "${MODE}" == "real" && "${NO_DRIVERS}" != "true" ]]; then
  command -v ip >/dev/null || { echo "error: 'ip' is required to validate livox.interface" >&2; exit 1; }
  [[ -d "/sys/class/net/${LIVOX_INTERFACE}" ]] || {
    echo "error: livox.interface '${LIVOX_INTERFACE}' does not exist" >&2
    exit 1
  }
  if ! ip -o -4 addr show dev "${LIVOX_INTERFACE}" | \
      awk -v expected="${LIVOX_HOST_IP}" '{split($4, address, "/"); if (address[1] == expected) found = 1} END {exit !found}'; then
    echo "error: livox.host_ip '${LIVOX_HOST_IP}' is not assigned to '${LIVOX_INTERFACE}'" >&2
    echo "error: configure the NIC first, then run launch.sh again; the launcher does not change network IPs." >&2
    exit 1
  fi
  start "Livox driver (${LIVOX_MODEL}: ${LIVOX_LIDAR_IP})" \
    ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
      -r __node:=livox_lidar_publisher \
      -p "xfer_format:=${LIVOX_XFER_FORMAT}" \
      -p "multi_topic:=${LIVOX_MULTI_TOPIC}" \
      -p "data_src:=${LIVOX_DATA_SRC}" \
      -p "publish_freq:=${LIVOX_PUBLISH_FREQ}" \
      -p "output_data_type:=${LIVOX_OUTPUT_DATA_TYPE}" \
      -p "frame_id:=${LIVOX_FRAME_ID}" \
      -p "user_config_path:=${LIVOX_CONFIG}" \
      -p "cmdline_input_bd_code:=${LIVOX_CMDLINE_BD_CODE}"
fi
if [[ "${NO_STATIC_TF}" != "true" ]]; then
  start "static imu -> base_link TF" ros2 run tf2_ros static_transform_publisher \
    --x "${IMU_BASE_X}" --y "${IMU_BASE_Y}" --z "${IMU_BASE_Z}" \
    --qx "${IMU_BASE_QX}" --qy "${IMU_BASE_QY}" --qz "${IMU_BASE_QZ}" --qw "${IMU_BASE_QW}" \
    --frame-id "${IMU_FRAME}" --child-frame-id "${BASE_FRAME}"
  if [[ "${NO_LIDAR_STATIC_TF}" != "true" ]]; then
    start "static base_link -> lidar_link TF" ros2 run tf2_ros static_transform_publisher \
      --x "${BASE_LIDAR_X}" --y "${BASE_LIDAR_Y}" --z "${BASE_LIDAR_Z}" \
      --qx "${BASE_LIDAR_QX}" --qy "${BASE_LIDAR_QY}" --qz "${BASE_LIDAR_QZ}" --qw "${BASE_LIDAR_QW}" \
      --frame-id "${BASE_FRAME}" --child-frame-id "${LIDAR_FRAME}"
  fi
fi

VIS_SCRIPT="${SCRIPT_DIR}/scripts/monitor_wall_state.py"
# launch.sh can be run from the installed package as well as the source tree.
[[ -f "${VIS_SCRIPT}" ]] || VIS_SCRIPT="${SCRIPT_DIR}/monitor_wall_state.py"
[[ -f "${VIS_SCRIPT}" ]] || {
  echo "error: wall-state monitor is missing: ${VIS_SCRIPT}" >&2
  exit 1
}

if [[ "${STARTUP_ALIGNMENT_REQUIRED}" == "1" ]]; then
  # The first LIO instance exists only to judge physical square-on alignment.
  # Its map is intentionally discarded. Once INIT accepts, Super-LIO is stopped
  # and started again so the operational map/yaw begins from that aligned pose.
  start "temporary Super-LIO for startup alignment" ros2 run super_lio super_lio_node \
    --ros-args --params-file "${LIO_CONFIG}"
  TEMP_LIO_PID="${PIDS[$((${#PIDS[@]} - 1))]}"
  start "startup wall alignment estimator" ros2 run autonomy_light front_wall_angle_estimator \
    --ros-args --params-file "${WALL_INIT_CONFIG}"
  TEMP_WALL_PID="${PIDS[$((${#PIDS[@]} - 1))]}"
  echo "autonomy-slam: waiting for startup wall alignment before starting operational SLAM"
  /usr/bin/python3 "${VIS_SCRIPT}" --init-only --exit-on-init --rate "${VIS_RATE}" \
    --wall-topic "${WALL_OUTPUT_TOPIC}" --odom-topic /lio/odom
  echo "autonomy-slam: startup alignment accepted; restarting Super-LIO with a fresh map/yaw"
  stop_process "${TEMP_WALL_PID}"
  stop_process "${TEMP_LIO_PID}"
  # Let DDS release the temporary node endpoints before reusing the same
  # node names for the operational instance.
  sleep 1
  start "Super-LIO" ros2 run super_lio super_lio_node --ros-args --params-file "${LIO_CONFIG}" \
    -r /lio/path:=/path_slam
  OPERATIONAL_LIO_PID="${PIDS[$((${#PIDS[@]} - 1))]}"
  verify_running "${OPERATIONAL_LIO_PID}"
elif [[ "${NO_DRIVERS}" != "true" ]]; then
  start "Super-LIO" ros2 run super_lio super_lio_node --ros-args --params-file "${LIO_CONFIG}" \
    -r /lio/path:=/path_slam
  OPERATIONAL_LIO_PID="${PIDS[$((${#PIDS[@]} - 1))]}"
  verify_running "${OPERATIONAL_LIO_PID}"
fi

if [[ "${ENABLE_NAV2}" == "true" ]]; then
  NAV2_WAIT_SCRIPT="${SCRIPT_DIR}/scripts/nav2_wait_and_launch.sh"
  [[ -f "${NAV2_WAIT_SCRIPT}" ]] || NAV2_WAIT_SCRIPT="${SCRIPT_DIR}/nav2_wait_and_launch.sh"
  [[ -f "${NAV2_WAIT_SCRIPT}" ]] || {
    echo "error: Nav2 startup supervisor is missing: ${NAV2_WAIT_SCRIPT}" >&2
    exit 1
  }
  USE_SIM_TIME="false"
  [[ "${MODE}" == "sim" ]] && USE_SIM_TIME="true"
  start "Nav2 navigation" "${NAV2_WAIT_SCRIPT}" "${NAV2_RUNTIME_CONFIG}" "${USE_SIM_TIME}" /lio/odom 120
  NAV2_PID="${PIDS[$((${#PIDS[@]} - 1))]}"
  verify_running "${NAV2_PID}"
fi

start "front-wall angle estimator" ros2 run autonomy_light front_wall_angle_estimator \
  --ros-args --params-file "${WALL_RUNTIME_CONFIG}"
WALL_ANGLE_PID="${PIDS[$((${#PIDS[@]} - 1))]}"
verify_running "${WALL_ANGLE_PID}"
start "wall state monitor (${VIS_RATE} Hz)" /usr/bin/python3 "${VIS_SCRIPT}" \
  --rate "${VIS_RATE}" --wall-topic "${WALL_OUTPUT_TOPIC}" --odom-topic /lio/odom

declare -a CRITICAL_PIDS=("${WALL_ANGLE_PID}")
if [[ -n "${OPERATIONAL_LIO_PID:-}" ]]; then
  CRITICAL_PIDS+=("${OPERATIONAL_LIO_PID}")
fi
if [[ -n "${NAV2_PID:-}" ]]; then
  CRITICAL_PIDS+=("${NAV2_PID}")
fi
if wait -n "${CRITICAL_PIDS[@]}"; then
  EXIT_STATUS=0
else
  EXIT_STATUS="$?"
fi

EXITED_LABEL="a critical operational process"
for pid in "${CRITICAL_PIDS[@]}"; do
  if ! kill -0 "${pid}" 2>/dev/null; then
    EXITED_LABEL="${PID_LABELS[${pid}]:-process ${pid}}"
    break
  fi
done
echo "error: ${EXITED_LABEL} stopped unexpectedly (exit status ${EXIT_STATUS})." >&2
exit "${EXIT_STATUS}"
