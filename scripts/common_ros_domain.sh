#!/usr/bin/env bash

repo_ros_domain_env_file() {
  local repo_root="$1"
  if [[ -n "${ROS_DOMAIN_ENV_FILE:-}" ]]; then
    printf '%s\n' "${ROS_DOMAIN_ENV_FILE}"
  else
    printf '%s/.ros_domain_id.env\n' "${repo_root}"
  fi
}

source_repo_ros_domain_env() {
  local repo_root="$1"
  local env_file
  env_file="$(repo_ros_domain_env_file "${repo_root}")"
  [[ -f "${env_file}" ]] || return 0

  set +u
  # shellcheck disable=SC1090
  source "${env_file}"
  set -u
}

write_repo_ros_domain_env() {
  local repo_root="$1"
  local domain_id="$2"
  local env_file
  env_file="$(repo_ros_domain_env_file "${repo_root}")"
  printf 'export ROS_DOMAIN_ID=%s\n' "${domain_id}" > "${env_file}"
}
