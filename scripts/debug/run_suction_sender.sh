#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROOT_DIR="/home/primarymage/WorkFile/esp_ws"
WS_DIR="${ROOT_DIR}/ros2_suction_ws"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../common_ros_domain.sh"
source_repo_ros_domain_env "${REPO_DIR}"

SERVICE_NAME="${1:-set_suction}"
ROS_DOMAIN_ID_VALUE="${2:-${ROS_DOMAIN_ID:-66}}"
RMW_IMPL="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Missing /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "Missing ${WS_DIR}/install/setup.bash. Build the workspace first." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export RMW_IMPLEMENTATION="${RMW_IMPL}"

echo "Sender host started."
echo "  service_name=${SERVICE_NAME}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "Input y to enable suction, n to disable, q to quit."

exec ros2 run suction_serial_bridge suction_keyboard_client \
  --ros-args \
  -p service_name:="${SERVICE_NAME}"
