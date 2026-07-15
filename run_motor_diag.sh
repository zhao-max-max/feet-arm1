#!/usr/bin/env bash
# 电机诊断脚本 — 单独启动驱动，全量保存日志
# 用法: bash run_motor_diag.sh [--params <yaml>]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$SCRIPT_DIR}"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"
DRIVER_PARAMS_FILE="${DRIVER_PARAMS_FILE:-$WS_DIR/src/dm_motor_sdk_ros/config/dm_motor_robot_driver.yaml}"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/scripts/common_ros_domain.sh"
source_repo_ros_domain_env "$WS_DIR"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --params) DRIVER_PARAMS_FILE="$2"; shift 2 ;;
    *) echo "未知参数: $1" >&2; exit 1 ;;
  esac
done

[[ -f "$ROS_SETUP" ]]          || { echo "ERROR: $ROS_SETUP 不存在" >&2; exit 1; }
[[ -f "$WS_SETUP" ]]           || { echo "ERROR: $WS_SETUP 不存在，先 colcon build" >&2; exit 1; }
[[ -f "$DRIVER_PARAMS_FILE" ]] || { echo "ERROR: 参数文件不存在: $DRIVER_PARAMS_FILE" >&2; exit 1; }

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WS_SETUP"
set -u

LOG_DIR="$WS_DIR/logs/motor_diag_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

DRIVER_PID=""
JOINT_PID=""
TEMP_PID=""
CLEANUP_RUNNING=0

cleanup() {
  (( CLEANUP_RUNNING )) && return
  CLEANUP_RUNNING=1
  echo ""
  echo "[motor_diag] 停止中..."
  for pid in "$DRIVER_PID" "$JOINT_PID" "$TEMP_PID"; do
    [[ -n "$pid" ]] && kill -INT "$pid" 2>/dev/null || true
  done
  for pid in "$DRIVER_PID" "$JOINT_PID" "$TEMP_PID"; do
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
  done
  echo "[motor_diag] 日志已保存到:"
  echo "  驱动日志   : $LOG_DIR/driver.log"
  echo "  关节状态   : $LOG_DIR/joint_state.log"
  echo "  温度       : $LOG_DIR/temperature.log"
}
trap cleanup EXIT INT TERM

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║       run_motor_diag.sh — 电机诊断       ║"
echo "╚══════════════════════════════════════════╝"
echo "  params  : $DRIVER_PARAMS_FILE"
echo "  log dir : $LOG_DIR"
echo "  domain  : ${ROS_DOMAIN_ID:-<unset>}"
echo ""

# 启动驱动，DEBUG 级别，完整日志写文件同时打屏
echo "[motor_diag] 启动驱动节点（DEBUG 日志）..."
setsid bash -c '
  export RCUTILS_LOGGING_MIN_SEVERITY=DEBUG
  ros2 launch dm_motor_sdk_ros dm_motor_robot_driver.launch.py \
    params_path:="$0" \
    2>&1 | tee "$1"
' "$DRIVER_PARAMS_FILE" "$LOG_DIR/driver.log" &
DRIVER_PID=$!

# 等驱动就绪再挂 topic 录制（最多等 20s）
echo "[motor_diag] 等待 /robot_driver/ready ..."
READY=0
for _ in $(seq 1 40); do
  if ros2 topic echo /robot_driver/ready std_msgs/msg/Bool \
       --once --qos-reliability reliable --qos-durability transient_local \
       2>/dev/null | grep -q "data: true"; then
    if kill -0 "$DRIVER_PID" 2>/dev/null; then
      READY=1; break
    fi
  fi
  sleep 0.5
done

if (( READY )); then
  echo "[motor_diag] 驱动就绪，开始录制关节状态和温度..."

  # 关节状态（含 q/dq/tau_est/valid，每帧打时间戳）
  setsid bash -c '
    ros2 topic echo --full-length /arm2/_lowState/joint \
      2>&1 | while IFS= read -r line; do
        printf "[%s] %s\n" "$(date +%T.%3N)" "$line"
      done | tee "$0"
  ' "$LOG_DIR/joint_state.log" &
  JOINT_PID=$!

  # 温度（1Hz，每帧打时间戳）
  setsid bash -c '
    ros2 topic echo /arm2/_lowState/temperature \
      2>&1 | while IFS= read -r line; do
        printf "[%s] %s\n" "$(date +%T.%3N)" "$line"
      done | tee "$0"
  ' "$LOG_DIR/temperature.log" &
  TEMP_PID=$!
else
  echo "[motor_diag] WARN: 驱动未在 20s 内就绪，跳过 topic 录制（驱动日志仍在记录）"
fi

echo ""
echo "[motor_diag] 运行中，按 Ctrl+C 停止并查看日志。"
echo ""

wait "$DRIVER_PID" 2>/dev/null || true
