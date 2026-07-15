#!/usr/bin/env bash
# kill_arm.sh — 强制停止所有机械臂相关进程，并确保电机失能
#
# 用法：bash kill_arm.sh
#
# 逻辑：
#   1. 先发 SIGINT 给 dm_motor_robot_driver_node（触发析构 → 电机失能）
#   2. 等待最多 3s 让驱动优雅退出
#   3. 再 SIGTERM 所有其余相关进程
#   4. 等待最多 3s
#   5. SIGKILL 任何还活着的

set -uo pipefail

PATTERNS=(
  "dm_motor_robot_driver_node"
  "arm2_task"
  "task_node"
  "control_node"
  "debug_tool_node"
  "suction_serial_bridge"
  "suction_service"
  "[r]un_arm"
)

find_pids() {
  local pattern="$1"
  pgrep -f "$pattern" 2>/dev/null || true
}

all_arm_pids() {
  local pids=()
  for p in "${PATTERNS[@]}"; do
    while IFS= read -r pid; do
      [[ -n "$pid" ]] && pids+=("$pid")
    done < <(find_pids "$p")
  done
  # deduplicate
  printf '%s\n' "${pids[@]}" | sort -un
}

send_sig() {
  local sig="$1"; shift
  for pid in "$@"; do
    kill "-$sig" "$pid" 2>/dev/null || true
  done
}

wait_all_dead() {
  local timeout_ds="$1"; shift  # in deciseconds
  local pids=("$@")
  for _ in $(seq 1 "$timeout_ds"); do
    local any=0
    for pid in "${pids[@]}"; do
      kill -0 "$pid" 2>/dev/null && { any=1; break; }
    done
    (( any )) || return 0
    sleep 0.1
  done
  return 1
}

echo "[kill_arm] 扫描机械臂相关进程..."

# Step 1: 找驱动进程，先单独 SIGINT（触发电机失能）
driver_pids=()
while IFS= read -r pid; do
  [[ -n "$pid" ]] && driver_pids+=("$pid")
done < <(find_pids "dm_motor_robot_driver_node")

if (( ${#driver_pids[@]} > 0 )); then
  echo "[kill_arm] 发送 SIGINT 给驱动进程 (pids: ${driver_pids[*]}) — 等待电机失能..."
  send_sig INT "${driver_pids[@]}"
  if wait_all_dead 30 "${driver_pids[@]}"; then
    echo "[kill_arm] 驱动已优雅退出（电机已失能）。"
  else
    echo "[kill_arm] WARN: 驱动未在 3s 内退出，将强制杀死。"
  fi
fi

# Step 2: 收集所有剩余进程并 SIGTERM
mapfile -t remaining < <(all_arm_pids)

if (( ${#remaining[@]} == 0 )); then
  echo "[kill_arm] 没有找到残留进程。"
  exit 0
fi

echo "[kill_arm] 发送 SIGTERM 给剩余进程 (pids: ${remaining[*]})..."
send_sig TERM "${remaining[@]}"

if wait_all_dead 30 "${remaining[@]}"; then
  echo "[kill_arm] 所有进程已停止。"
  exit 0
fi

# Step 3: 还有残留 → SIGKILL
still_alive=()
for pid in "${remaining[@]}"; do
  kill -0 "$pid" 2>/dev/null && still_alive+=("$pid")
done

if (( ${#still_alive[@]} > 0 )); then
  echo "[kill_arm] WARN: 以下进程拒绝退出，强制 SIGKILL: ${still_alive[*]}"
  send_sig KILL "${still_alive[@]}"
  sleep 0.3
fi

# 最终状态
mapfile -t final < <(all_arm_pids)
if (( ${#final[@]} == 0 )); then
  echo "[kill_arm] 完成，所有进程已清理。"
else
  echo "[kill_arm] ERROR: 以下进程仍然存在: ${final[*]}" >&2
  ps -p "$(IFS=,; echo "${final[*]}")" 2>/dev/null || true
  exit 1
fi
