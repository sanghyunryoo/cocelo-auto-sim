#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/package_deb.sh [options]

Build a source-free runtime .deb in dist/.

Options:
  --version VERSION       Debian upstream version. Default: package.xml version.
  --revision REVISION     Debian revision. Default: 1.
  --output-dir DIR        Output directory. Default: dist.
  --ros-distro NAME       ROS distribution. Default: ROS_DISTRO or humble.
  --skip-build            Package the existing install tree without rebuilding.
  --no-strip              Keep debug symbols in packaged ELF files.
  -h, --help              Show this help.

The package contains autonomy_light, livox_ros_driver2, super_lio, basic, and
the Livox-SDK2 shared library. The target must already provide the same Ubuntu
and ROS 2 distribution used to build the package.
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
INSTALL_ROOT="${REPO_DIR}/install"
PACKAGE_NAME="cocelo-hd-slam-yaw"
PACKAGE_ROOT="/opt/cocelo/${PACKAGE_NAME}"
CONFIG_DIR="/etc/cocelo/${PACKAGE_NAME}"
USER_GUIDE_PDF="${REPO_DIR}/dist/${PACKAGE_NAME}_user-guide_ko.pdf"
VERSION=""
REVISION="7"
OUTPUT_DIR="${REPO_DIR}/dist"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
SKIP_BUILD="false"
DO_STRIP="true"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --version) VERSION="${2:?--version requires a value}"; shift 2 ;;
    --version=*) VERSION="${1#--version=}"; shift ;;
    --revision) REVISION="${2:?--revision requires a value}"; shift 2 ;;
    --revision=*) REVISION="${1#--revision=}"; shift ;;
    --output-dir) OUTPUT_DIR="${2:?--output-dir requires a directory}"; shift 2 ;;
    --output-dir=*) OUTPUT_DIR="${1#--output-dir=}"; shift ;;
    --ros-distro) ROS_DISTRO_NAME="${2:?--ros-distro requires a name}"; shift 2 ;;
    --ros-distro=*) ROS_DISTRO_NAME="${1#--ros-distro=}"; shift ;;
    --skip-build) SKIP_BUILD="true"; shift ;;
    --no-strip) DO_STRIP="false"; shift ;;
    *) echo "error: unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

read_package_version() {
  /usr/bin/python3 - "${REPO_DIR}/package.xml" <<'PY'
import sys
import xml.etree.ElementTree as ET

print(ET.parse(sys.argv[1]).getroot().findtext("version", "0.0.0"))
PY
}

detect_architecture() {
  local architecture
  architecture="$(dpkg --print-architecture)"
  case "${architecture}" in
    amd64|arm64) printf '%s\n' "${architecture}" ;;
    *)
      echo "error: unsupported Debian architecture: ${architecture}" >&2
      echo "supported architectures: amd64, arm64" >&2
      exit 1
      ;;
  esac
}

read_ubuntu_version() {
  local version
  version="$(awk -F= '$1 == "VERSION_ID" {gsub(/"/, "", $2); print $2; exit}' /etc/os-release)"
  [[ -n "${version}" ]] || { echo "error: cannot read VERSION_ID from /etc/os-release" >&2; exit 1; }
  printf '%s\n' "${version}"
}

copy_install_tree() {
  local destination="$1"
  mkdir -p "${destination}"
  (
    cd "${INSTALL_ROOT}"
    find . \
      \( -type d -name __pycache__ \) -prune -o \
      \( -type f -name '*.pyc' -o -type f -name '*.pyo' \) -prune -o \
      \( -type f -o -type l \) -print0 | \
      tar --create --null --dereference --files-from=-
  ) | tar --directory="${destination}" --extract
}

copy_file_if_present() {
  local source="$1"
  local destination="$2"
  [[ -f "${source}" ]] && cp -aL "${source}" "${destination}"
}

[[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]] || {
  echo "error: ROS setup not found: /opt/ros/${ROS_DISTRO_NAME}/setup.bash" >&2
  exit 1
}
command -v dpkg-deb >/dev/null || { echo "error: dpkg-deb is required" >&2; exit 1; }
command -v dpkg >/dev/null || { echo "error: dpkg is required" >&2; exit 1; }
command -v tar >/dev/null || { echo "error: tar is required" >&2; exit 1; }

if [[ -z "${VERSION}" ]]; then
  VERSION="$(read_package_version)"
fi
[[ "${VERSION}" =~ ^[0-9A-Za-z][0-9A-Za-z.+:~\-]*$ ]] || {
  echo "error: invalid Debian upstream version: ${VERSION}" >&2
  exit 2
}
[[ "${REVISION}" =~ ^[0-9A-Za-z][0-9A-Za-z.+~]*$ ]] || {
  echo "error: invalid Debian revision: ${REVISION}" >&2
  exit 2
}

ARCHITECTURE="$(detect_architecture)"
UBUNTU_VERSION="$(read_ubuntu_version)"
DEBIAN_VERSION="${VERSION}-${REVISION}+${ROS_DISTRO_NAME}${UBUNTU_VERSION}"

if [[ "${SKIP_BUILD}" != "true" ]]; then
  # Packaging rebuilds application artifacts but must not mutate apt packages.
  "${REPO_DIR}/build.sh" --skip-apt --ros-distro "${ROS_DISTRO_NAME}" \
    --packages livox_ros_driver2 super_lio autonomy_light
fi

for prefix in basic livox_ros_driver2 super_lio autonomy_light; do
  [[ -d "${INSTALL_ROOT}/${prefix}" ]] || {
    echo "error: install tree is missing ${INSTALL_ROOT}/${prefix}" >&2
    echo "hint: omit --skip-build or run build.sh first." >&2
    exit 1
  }
done
[[ -f "${INSTALL_ROOT}/autonomy_light/lib/autonomy_light/launch.sh" ]] || {
  echo "error: installed launch.sh is missing; rebuild before packaging." >&2
  exit 1
}

"${SCRIPT_DIR}/generate_user_guide_pdf.sh" --output "${USER_GUIDE_PDF}"

LIVOX_SDK_LIBRARY=""
for candidate in \
  /usr/local/lib/liblivox_lidar_sdk_shared.so \
  /usr/local/lib/liblivox_lidar_sdk_shared.so.0
do
  if [[ -f "${candidate}" ]]; then
    LIVOX_SDK_LIBRARY="${candidate}"
    break
  fi
done
[[ -n "${LIVOX_SDK_LIBRARY}" ]] || {
  echo "error: Livox-SDK2 library not found under /usr/local/lib" >&2
  echo "hint: run build.sh without --skip-sdk before packaging." >&2
  exit 1
}

STAGE_ROOT="$(mktemp -d "/tmp/${PACKAGE_NAME}.XXXXXX")"
cleanup() {
  rm -rf "${STAGE_ROOT}"
}
trap cleanup EXIT

mkdir -p \
  "${STAGE_ROOT}/DEBIAN" \
  "${STAGE_ROOT}${PACKAGE_ROOT}/install" \
  "${STAGE_ROOT}${PACKAGE_ROOT}/lib" \
  "${STAGE_ROOT}${CONFIG_DIR}" \
  "${STAGE_ROOT}/usr/bin" \
  "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}"

copy_install_tree "${STAGE_ROOT}${PACKAGE_ROOT}/install"
cp -aL "${REPO_DIR}/config/autonomy_light.yaml" \
  "${STAGE_ROOT}${CONFIG_DIR}/autonomy_light.yaml"
chmod 0644 "${STAGE_ROOT}${CONFIG_DIR}/autonomy_light.yaml"
cp -aL "${LIVOX_SDK_LIBRARY}" "${STAGE_ROOT}${PACKAGE_ROOT}/lib/"
copy_file_if_present "${REPO_DIR}/README.md" "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/README.md"
if [[ -d "${REPO_DIR}/docs" ]]; then
  cp -aL "${REPO_DIR}/docs/." "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/"
fi
cp -aL "${USER_GUIDE_PDF}" \
  "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/user_guide_ko.pdf"

# The colcon-generated setup.bash may retain the build machine's chained
# workspaces. Supply a relocatable setup that sources only target ROS + this
# packaged install tree.
cat > "${STAGE_ROOT}${PACKAGE_ROOT}/setup.bash" <<'EOF'
#!/usr/bin/env bash

_cocelo_package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_cocelo_ros_distro="@ROS_DISTRO@"
if [[ ! -f "/opt/ros/${_cocelo_ros_distro}/setup.bash" ]]; then
  echo "error: /opt/ros/${_cocelo_ros_distro}/setup.bash not found" >&2
  return 1 2>/dev/null || exit 1
fi
_cocelo_restore_nounset="false"
if [[ "$-" == *u* ]]; then
  _cocelo_restore_nounset="true"
  set +u
fi
source "/opt/ros/${_cocelo_ros_distro}/setup.bash"
source "${_cocelo_package_root}/install/local_setup.bash"
if [[ "${_cocelo_restore_nounset}" == "true" ]]; then
  set -u
fi
unset _cocelo_package_root _cocelo_ros_distro _cocelo_restore_nounset
EOF
sed -i "s/@ROS_DISTRO@/${ROS_DISTRO_NAME}/g" "${STAGE_ROOT}${PACKAGE_ROOT}/setup.bash"
chmod 0755 "${STAGE_ROOT}${PACKAGE_ROOT}/setup.bash"

cat > "${STAGE_ROOT}/usr/bin/autonomy-slam" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

PACKAGE_ROOT="/opt/cocelo/cocelo-hd-slam-yaw"
export AUTONOMY_LIGHT_CONFIG="${AUTONOMY_LIGHT_CONFIG:-/etc/cocelo/cocelo-hd-slam-yaw/autonomy_light.yaml}"
export LD_LIBRARY_PATH="${PACKAGE_ROOT}/lib:${LD_LIBRARY_PATH:-}"
source "${PACKAGE_ROOT}/setup.bash"
exec "${PACKAGE_ROOT}/install/autonomy_light/lib/autonomy_light/launch.sh" "$@"
EOF
chmod 0755 "${STAGE_ROOT}/usr/bin/autonomy-slam"

cat > "${STAGE_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${DEBIAN_VERSION}
Section: robotics
Priority: optional
Architecture: ${ARCHITECTURE}
Maintainer: autonomy_light maintainer <todo@example.com>
Depends: bash, iproute2, python3, python3-yaml
Description: Livox and Super-LIO front-wall angle runtime
 Source-free runtime bundle for autonomy_light, Livox ROS Driver2, and
 Super-LIO. ROS 2 ${ROS_DISTRO_NAME} must already be installed under
 /opt/ros/${ROS_DISTRO_NAME} on the target system.
EOF

cat > "${STAGE_ROOT}/DEBIAN/conffiles" <<EOF
${CONFIG_DIR}/autonomy_light.yaml
EOF

cat > "${STAGE_ROOT}/DEBIAN/postinst" <<'EOF'
#!/usr/bin/env bash
set -e
echo "cocelo-hd-slam-yaw installed."
echo "Edit LiDAR model/IP settings: /etc/cocelo/cocelo-hd-slam-yaw/autonomy_light.yaml"
echo "Run: autonomy-slam --real"
EOF
chmod 0755 "${STAGE_ROOT}/DEBIAN/postinst"

cat > "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/runtime_assumptions.txt" <<EOF
Runtime assumptions
===================

- Architecture: ${ARCHITECTURE}
- Ubuntu version: ${UBUNTU_VERSION}
- ROS 2 ${ROS_DISTRO_NAME} exists at /opt/ros/${ROS_DISTRO_NAME}
- The LiDAR NIC/IP is configured before running autonomy-slam

The editable configuration is ${CONFIG_DIR}/autonomy_light.yaml.
EOF

if [[ "${DO_STRIP}" == "true" ]] && command -v strip >/dev/null && command -v file >/dev/null; then
  while IFS= read -r binary; do
    if file "${binary}" | grep -q 'ELF'; then
      strip --strip-unneeded "${binary}" 2>/dev/null || true
    fi
  done < <(find "${STAGE_ROOT}${PACKAGE_ROOT}" -type f)
fi

find "${STAGE_ROOT}" -type d -exec chmod 0755 {} +
find "${STAGE_ROOT}${PACKAGE_ROOT}/install" -type f \
  \( -path '*/lib/autonomy_light/*' -o -path '*/lib/livox_ros_driver2/*' -o -path '*/lib/super_lio/*' \) \
  -exec chmod 0755 {} +

mkdir -p "${OUTPUT_DIR}"
DEB_PATH="${OUTPUT_DIR}/${PACKAGE_NAME}_${DEBIAN_VERSION}_${ARCHITECTURE}.deb"
dpkg-deb --build --root-owner-group "${STAGE_ROOT}" "${DEB_PATH}"

echo
echo "Created: ${DEB_PATH}"
echo "Install: sudo apt install ./$(basename "${DEB_PATH}")"
echo "Run: autonomy-slam --real"
