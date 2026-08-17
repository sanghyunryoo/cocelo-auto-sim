#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  build.sh [options]

Build the Livox driver, Super-LIO, and front-wall angle estimator.

Options:
  --skip-apt          Do not install Ubuntu/ROS dependencies.
  --skip-sdk          Do not build/install Livox-SDK2.
  --clean             Remove this workspace's build/install/log first.
  --setup-only        Install runtime/build dependencies only; do not build.
  --sim               Build a simulation-only stack without the physical Livox SDK node.
  --packages PKGS     Packages to build. Default: core livox_ros_driver2 super_lio autonomy_light.
  --ros-distro NAME   ROS distro. Default: ROS_DISTRO or humble.
  -h, --help          Show this help.
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
SKIP_APT="false"
SKIP_SDK="false"
CLEAN="false"
SETUP_ONLY="false"
SIM_ONLY="false"
PACKAGES=(core livox_ros_driver2 super_lio autonomy_light)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --skip-apt) SKIP_APT="true"; shift ;;
    --skip-sdk) SKIP_SDK="true"; shift ;;
    --clean) CLEAN="true"; shift ;;
    --setup-only) SETUP_ONLY="true"; shift ;;
    --sim) SIM_ONLY="true"; SKIP_SDK="true"; shift ;;
    --packages)
      shift
      PACKAGES=()
      while [[ $# -gt 0 && "$1" != --* ]]; do PACKAGES+=("$1"); shift; done
      [[ "${#PACKAGES[@]}" -gt 0 ]] || { echo "error: --packages requires at least one package name" >&2; exit 2; }
      ;;
    --ros-distro) ROS_DISTRO_NAME="${2:?--ros-distro requires a name}"; shift 2 ;;
    --ros-distro=*) ROS_DISTRO_NAME="${1#--ros-distro=}"; shift ;;
    *) echo "error: unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

LOCAL_ROS_PREFIX="${WORKSPACE_DIR}/.deps/opt/ros/${ROS_DISTRO_NAME}"
LOCAL_SYSROOT="${WORKSPACE_DIR}/.deps/system"
if [[ "${SIM_ONLY}" == "true" && ! -f "/opt/ros/${ROS_DISTRO_NAME}/lib/cmake/GTSAM/GTSAMConfig.cmake" && ! -f "${LOCAL_ROS_PREFIX}/lib/cmake/GTSAM/GTSAMConfig.cmake" ]]; then
  command -v apt-get >/dev/null || { echo "error: apt-get is required to fetch the local GTSAM runtime" >&2; exit 1; }
  command -v dpkg-deb >/dev/null || { echo "error: dpkg-deb is required to unpack the local GTSAM runtime" >&2; exit 1; }
  mkdir -p "${WORKSPACE_DIR}/.deps/cache" "${WORKSPACE_DIR}/.deps"
  (
    cd "${WORKSPACE_DIR}/.deps/cache"
    apt-get download "ros-${ROS_DISTRO_NAME}-gtsam"
    GTSAM_DEB="$(find . -maxdepth 1 -name "ros-${ROS_DISTRO_NAME}-gtsam_*.deb" -print -quit)"
    [[ -n "${GTSAM_DEB}" ]] || { echo "error: failed to download ros-${ROS_DISTRO_NAME}-gtsam" >&2; exit 1; }
    dpkg-deb -x "${GTSAM_DEB}" "${WORKSPACE_DIR}/.deps"
  )
fi

if [[ "${SIM_ONLY}" == "true" && ! -f "/usr/lib/x86_64-linux-gnu/cmake/glog/glog-config.cmake" && ! -f "${LOCAL_SYSROOT}/usr/lib/x86_64-linux-gnu/cmake/glog/glog-config.cmake" ]]; then
  command -v apt-get >/dev/null || { echo "error: apt-get is required to fetch local glog development files" >&2; exit 1; }
  command -v dpkg-deb >/dev/null || { echo "error: dpkg-deb is required to unpack local glog development files" >&2; exit 1; }
  mkdir -p "${WORKSPACE_DIR}/.deps/cache" "${LOCAL_SYSROOT}"
  (
    cd "${WORKSPACE_DIR}/.deps/cache"
    for package in libgoogle-glog-dev libgoogle-glog0v6t64 libgflags-dev libgflags2.2 libunwind-dev; do
      apt-get download "${package}"
      deb_file="$(find . -maxdepth 1 -name "${package}_*.deb" -print -quit)"
      [[ -n "${deb_file}" ]] || { echo "error: failed to download ${package}" >&2; exit 1; }
      dpkg-deb -x "${deb_file}" "${LOCAL_SYSROOT}"
    done
  )
fi

if [[ -d "${LOCAL_ROS_PREFIX}" ]]; then
  export CMAKE_PREFIX_PATH="${LOCAL_ROS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export AMENT_PREFIX_PATH="${LOCAL_ROS_PREFIX}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${LOCAL_ROS_PREFIX}/lib:${LOCAL_ROS_PREFIX}/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
if [[ -d "${LOCAL_SYSROOT}/usr" ]]; then
  export CMAKE_PREFIX_PATH="${LOCAL_SYSROOT}/usr${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${LOCAL_SYSROOT}/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export PKG_CONFIG_PATH="${LOCAL_SYSROOT}/usr/lib/x86_64-linux-gnu/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
fi

SETUP_ARGS=()
[[ "${SKIP_APT}" == "true" ]] && SETUP_ARGS+=(--skip-apt)
[[ "${SKIP_SDK}" == "true" ]] && SETUP_ARGS+=(--skip-sdk)

if [[ "${CLEAN}" == "true" ]]; then
  echo "Cleaning build/install/log."
  rm -rf "${WORKSPACE_DIR}/build" "${WORKSPACE_DIR}/install" "${WORKSPACE_DIR}/log"
fi

echo "Preparing vendored Livox driver"
ROS_DISTRO="${ROS_DISTRO_NAME}" "${SCRIPT_DIR}/scripts/setup_livox_driver.sh" "${SETUP_ARGS[@]}"

if [[ "${SETUP_ONLY}" == "true" ]]; then
  echo "Setup complete."
  exit 0
fi

ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
[[ -f "${ROS_SETUP}" ]] || { echo "error: ${ROS_SETUP} not found" >&2; exit 1; }
set +u
# shellcheck source=/dev/null
source "${ROS_SETUP}"
set -u

if [[ -x /usr/bin/python3 ]]; then
  BUILD_PYTHON="/usr/bin/python3"
else
  BUILD_PYTHON="$(command -v python3)"
fi
cd "${WORKSPACE_DIR}"
echo "Building packages: ${PACKAGES[*]}"
colcon build --packages-up-to "${PACKAGES[@]}" \
  --base-paths \
    "${WORKSPACE_DIR}" \
    "${WORKSPACE_DIR}/interfaces" \
    "${WORKSPACE_DIR}/third_party/livox_ros_driver2" \
    "${WORKSPACE_DIR}/third_party/super_lio_ros2" \
  --cmake-args \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS="${ROS_DISTRO_NAME}" \
    -DBUILD_LIVOX_DRIVER="$([[ "${SIM_ONLY}" == "true" ]] && echo OFF || echo ON)" \
    -DCMAKE_MODULE_PATH="${LOCAL_SYSROOT}/usr/share/glog/cmake" \
    -DPYTHON_EXECUTABLE="${BUILD_PYTHON}" \
    -DPython3_EXECUTABLE="${BUILD_PYTHON}"

cat <<EOF

Build finished.

Next:
  source ${ROS_SETUP}
  source ${WORKSPACE_DIR}/install/setup.bash
  ${SCRIPT_DIR}/launch.sh --real
EOF
