#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$SCRIPT_DIR}"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"
DRIVER_PARAMS_FILE="${DRIVER_PARAMS_FILE:-$WS_DIR/src/dm_motor_sdk_ros/config/dm_motor_robot_driver.yaml}"
CONTROL_PARAMS_FILE="${CONTROL_PARAMS_FILE:-$WS_DIR/src/arm2_task/config/params.yaml}"
READY_TIMEOUT="${READY_TIMEOUT:-15}"
AUTO_BUILD="${AUTO_BUILD:-false}"
ROS_DOMAIN_ENV_FILE="${ROS_DOMAIN_ENV_FILE:-$WS_DIR/.ros_domain_id.env}"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/scripts/common_ros_domain.sh"

DRIVER_PID=""
CONTROL_PID=""
UI_PID=""
CLEANUP_RUNNING=0
LOG_DIR=""

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

启动这条调试链路：
  1. ros2 launch dm_motor_sdk_ros dm_motor_robot_driver.launch.py
  2. ros2 launch arm2_task control_node.launch.py
  3. ros2 launch debug_tool joint_slider_ui.launch.py

Options:
  --build                 启动前先编译相关包
  --driver-params <file>  指定驱动参数文件
  --control-params <file> 指定 control_node 参数文件
  --ready-timeout <sec>   等待 /robot_driver/ready 超时秒数（默认 ${READY_TIMEOUT}）
  -h, --help              显示帮助

环境变量：
  WS_DIR                  工作区根目录
  DRIVER_PARAMS_FILE      驱动参数文件
  CONTROL_PARAMS_FILE     control_node 参数文件
  READY_TIMEOUT           等待 /robot_driver/ready 的超时秒数
  AUTO_BUILD=true         等同于 --build
USAGE
}

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

source_repo_ros_domain_env "$WS_DIR"

launch_in_group() {
  local __var="$1"
  local logfile="$2"
  shift 2
  setsid bash -c '"$@" 2>&1 | tee "$0"' "$logfile" "$@" &
  local pid=$!
  printf -v "$__var" '%s' "$pid"
}

# Recursively collect all descendant PIDs (depth-first, leaves first).
collect_descendants() {
  local pid="$1"
  local children
  children=$(pgrep -P "$pid" 2>/dev/null || true)
  for child in $children; do
    collect_descendants "$child"
  done
  echo "$pid"
}

stop_process_group() {
  local pid="$1"
  local name="$2"
  [[ -n "$pid" ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0

  # Collect the entire process subtree BEFORE sending any signal,
  # so we don't lose track of children after the launch parent exits.
  local tree
  tree=$(collect_descendants "$pid")

  echo "[run_arm_slider_ui] stopping $name (pids: $tree)..."

  # Send SIGINT to every process in the tree so each node runs its own
  # graceful shutdown (e.g. dm_motor_disable in the driver destructor).
  for p in $tree; do
    kill -INT "$p" 2>/dev/null || true
  done

  # Wait up to 3 s for everything to exit cleanly.
  for _ in $(seq 1 30); do
    local any_alive=0
    for p in $tree; do
      kill -0 "$p" 2>/dev/null && { any_alive=1; break; }
    done
    (( any_alive )) || { wait "$pid" 2>/dev/null || true; return 0; }
    sleep 0.1
  done

  # Anything still alive after graceful window gets SIGKILL.
  echo "[run_arm_slider_ui] WARN: $name did not stop cleanly; sending SIGKILL"
  for p in $tree; do
    kill -KILL "$p" 2>/dev/null || true
  done
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  (( CLEANUP_RUNNING )) && return
  CLEANUP_RUNNING=1
  echo "[run_arm_slider_ui] shutting down..."
  stop_process_group "$UI_PID" "joint_slider_ui"
  stop_process_group "$CONTROL_PID" "control_node"
  stop_process_group "$DRIVER_PID" "dm_motor driver"
  echo "[run_arm_slider_ui] done."
}

wait_for_ready() {
  local timeout_sec="$1"
  local deadline=$((SECONDS + timeout_sec))
  echo "[run_arm_slider_ui] waiting for /robot_driver/ready (timeout ${timeout_sec}s)..."
  while (( SECONDS < deadline )); do
    if [[ -n "$DRIVER_PID" ]] && ! kill -0 "$DRIVER_PID" 2>/dev/null; then
      echo "[run_arm_slider_ui] ERROR: driver exited before becoming ready." >&2
      return 1
    fi
    if ros2 topic echo /robot_driver/ready std_msgs/msg/Bool \
      --once --qos-reliability reliable --qos-durability transient_local \
      2>/dev/null | grep -q "data: true"; then
      if [[ -n "$DRIVER_PID" ]] && ! kill -0 "$DRIVER_PID" 2>/dev/null; then
        echo "[run_arm_slider_ui] ERROR: received stale /robot_driver/ready after driver exit." >&2
        return 1
      fi
      echo "[run_arm_slider_ui] /robot_driver/ready received."
      return 0
    fi
    sleep 0.3
  done
  echo "[run_arm_slider_ui] ERROR: timed out waiting for /robot_driver/ready." >&2
  return 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) AUTO_BUILD=true; shift ;;
    --driver-params) DRIVER_PARAMS_FILE="$2"; shift 2 ;;
    --control-params) CONTROL_PARAMS_FILE="$2"; shift 2 ;;
    --ready-timeout) READY_TIMEOUT="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "[run_arm_slider_ui] ERROR: unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

[[ -f "$ROS_SETUP" ]] || {
  echo "[run_arm_slider_ui] ERROR: ROS setup not found: $ROS_SETUP" >&2
  exit 1
}
[[ -f "$DRIVER_PARAMS_FILE" ]] || {
  echo "[run_arm_slider_ui] ERROR: driver params not found: $DRIVER_PARAMS_FILE" >&2
  exit 1
}
[[ -f "$CONTROL_PARAMS_FILE" ]] || {
  echo "[run_arm_slider_ui] ERROR: control params not found: $CONTROL_PARAMS_FILE" >&2
  exit 1
}

source_setup "$ROS_SETUP"
if [[ "$AUTO_BUILD" == "true" ]]; then
  echo "[run_arm_slider_ui] building packages..."
  (
    cd "$WS_DIR"
    colcon build \
      --packages-select robot_msgs dm_motor_sdk_ros arm2_task debug_tool \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  )
fi

[[ -f "$WS_SETUP" ]] || {
  echo "[run_arm_slider_ui] ERROR: install/setup.bash not found; build the workspace first." >&2
  exit 1
}
source_setup "$WS_SETUP"
trap cleanup EXIT INT TERM
cd "$SCRIPT_DIR"
LOG_DIR="$WS_DIR/logs/slider_ui_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║     run_arm_slider_ui.sh — 调试滑条链路      ║"
echo "╚══════════════════════════════════════════════╝"
echo "  workspace      : $WS_DIR"
echo "  driver params  : $DRIVER_PARAMS_FILE"
echo "  control params : $CONTROL_PARAMS_FILE"
echo "  ready timeout  : ${READY_TIMEOUT}s"
echo "  ros_domain_id  : ${ROS_DOMAIN_ID:-<unset>}"
echo ""

echo "[run_arm_slider_ui] launching dm_motor_sdk_ros driver..."
launch_in_group DRIVER_PID "$LOG_DIR/driver.log" \
  ros2 launch dm_motor_sdk_ros dm_motor_robot_driver.launch.py \
    params_path:="$DRIVER_PARAMS_FILE"

wait_for_ready "$READY_TIMEOUT"

echo "[run_arm_slider_ui] launching control_node..."
launch_in_group CONTROL_PID "$LOG_DIR/control_node.log" \
  ros2 launch arm2_task control_node.launch.py \
    params_path:="$CONTROL_PARAMS_FILE"

# 给 control_node 一点时间创建 action / service，UI 自身也会持续等待。
sleep 1

echo "[run_arm_slider_ui] launching joint_slider_ui..."
launch_in_group UI_PID "$LOG_DIR/joint_slider_ui.log" \
  ros2 launch debug_tool joint_slider_ui.launch.py

echo ""
echo "[run_arm_slider_ui] all nodes started. Press Ctrl+C to stop everything."
echo "  driver       pid: $DRIVER_PID"
echo "  control_node pid: $CONTROL_PID"
echo "  slider_ui    pid: $UI_PID"
echo "  logs         dir: $LOG_DIR"
echo ""

wait -n "$DRIVER_PID" "$CONTROL_PID" "$UI_PID"
