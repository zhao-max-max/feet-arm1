#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/scripts/common_ros_domain.sh"

write_repo_ros_domain_env "${SCRIPT_DIR}" "65"

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  source_repo_ros_domain_env "${SCRIPT_DIR}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (current shell + project default)."
else
  echo "Project default ROS_DOMAIN_ID switched to 65."
  echo "Saved to ${SCRIPT_DIR}/.ros_domain_id.env"
  echo "Current shell unchanged. To switch this terminal too, run:"
  echo "  source \"${SCRIPT_DIR}/set_ros_domain_65.sh\""
fi
