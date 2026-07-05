#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$SCRIPT_DIR}"
ROBOCON_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ARM_CAMERA_WS="${ARM_CAMERA_WS:-${ROBOCON_DIR}/camera/arm_camera}"
HEAD_CAMERA_WS="${HEAD_CAMERA_WS:-${ROBOCON_DIR}/camera/head_camera}"
SESSION_NAME="${SESSION_NAME:-nav_full}"
ATTACH="${ATTACH:-true}"
ARM_CAMERA_ENABLED="${ARM_CAMERA_ENABLED:-true}"
HEAD_CAMERA_ENABLED="${HEAD_CAMERA_ENABLED:-false}"
ARM_CAMERA_HEADLESS="${ARM_CAMERA_HEADLESS:-true}"
ROS_DOMAIN_ENV_FILE="${ROS_DOMAIN_ENV_FILE:-$WS_DIR/.ros_domain_id.env}"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/scripts/common_ros_domain.sh"
source_repo_ros_domain_env "$WS_DIR"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options] [-- run_arm_nav args...]

Create a tmux session for the navigation arm stack.

Windows:
  1. arm_nav      -> bash run_arm_nav.sh
  2. arm_camera   -> bash camera/arm_camera/run_neweyes.sh --headless   (default on)
  3. head_camera  -> bash camera/head_camera/run_neweyes.sh             (default off)

Options:
  --session <name>        tmux session name (default: $SESSION_NAME)
  --detach                Create the session without attaching
  --attach                Attach after creation (default)
  --no-arm-camera         Do not launch arm_camera
  --with-head-camera      Launch head_camera in a separate tmux window
  --arm-camera-gui        Launch arm_camera without --headless
  --arm-camera-ws <dir>   arm_camera workspace path
  --head-camera-ws <dir>  head_camera workspace path
  -h, --help              Show this help

All unrecognized arguments are forwarded to run_arm_nav.sh.

Examples:
  bash run_nav_full_tmux.sh
  bash run_nav_full_tmux.sh --sim
  bash run_nav_full_tmux.sh --detach --build
  bash run_nav_full_tmux.sh --with-head-camera --ready-timeout 30
USAGE
}

shell_join() {
  local args=()
  local arg
  for arg in "$@"; do
    args+=("$(printf '%q' "$arg")")
  done
  local IFS=' '
  printf '%s' "${args[*]}"
}

make_window_cmd() {
  local workdir="$1"
  shift
  printf 'cd %s && exec %s' "$(printf '%q' "$workdir")" "$(shell_join "$@")"
}

require_option_arg() {
  local opt="$1"
  local next_arg="${2-}"
  [[ -n "$next_arg" ]] || {
    echo "[run_nav_full_tmux] ERROR: $opt requires an argument." >&2
    exit 1
  }
}

ARM_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --session)
      require_option_arg "$1" "${2-}"
      SESSION_NAME="$2"
      shift 2
      ;;
    --detach)
      ATTACH=false
      shift
      ;;
    --attach)
      ATTACH=true
      shift
      ;;
    --no-arm-camera)
      ARM_CAMERA_ENABLED=false
      shift
      ;;
    --with-head-camera)
      HEAD_CAMERA_ENABLED=true
      shift
      ;;
    --arm-camera-gui)
      ARM_CAMERA_HEADLESS=false
      shift
      ;;
    --arm-camera-ws)
      require_option_arg "$1" "${2-}"
      ARM_CAMERA_WS="$2"
      shift 2
      ;;
    --head-camera-ws)
      require_option_arg "$1" "${2-}"
      HEAD_CAMERA_WS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      ARM_ARGS+=("$@")
      break
      ;;
    *)
      ARM_ARGS+=("$1")
      shift
      ;;
  esac
done

command -v tmux >/dev/null 2>&1 || {
  echo "[run_nav_full_tmux] ERROR: tmux not found. Install tmux first." >&2
  exit 1
}

[[ -x "$WS_DIR/run_arm_nav.sh" ]] || {
  echo "[run_nav_full_tmux] ERROR: run_arm_nav.sh not found: $WS_DIR/run_arm_nav.sh" >&2
  exit 1
}

if [[ "$ARM_CAMERA_ENABLED" == "true" ]]; then
  [[ -f "$ARM_CAMERA_WS/run_neweyes.sh" ]] || {
    echo "[run_nav_full_tmux] ERROR: arm_camera launcher not found: $ARM_CAMERA_WS/run_neweyes.sh" >&2
    exit 1
  }
fi

if [[ "$HEAD_CAMERA_ENABLED" == "true" ]]; then
  [[ -f "$HEAD_CAMERA_WS/run_neweyes.sh" ]] || {
    echo "[run_nav_full_tmux] ERROR: head_camera launcher not found: $HEAD_CAMERA_WS/run_neweyes.sh" >&2
    exit 1
  }
  if [[ -z "${DISPLAY:-}" ]] && [[ -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo "[run_nav_full_tmux] WARN: head_camera is enabled, but no graphical display is set." >&2
    echo "[run_nav_full_tmux]       Its current launcher still opens an OpenCV window and may fail." >&2
  fi
fi

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  echo "[run_nav_full_tmux] session '$SESSION_NAME' already exists."
  if [[ "$ATTACH" == "true" ]]; then
    exec tmux attach-session -t "$SESSION_NAME"
  fi
  exit 0
fi

arm_cmd=(bash "$WS_DIR/run_arm_nav.sh" "${ARM_ARGS[@]}")
arm_window_cmd="$(make_window_cmd "$WS_DIR" "${arm_cmd[@]}")"

camera_env=("PYTHONUNBUFFERED=1")
if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
  camera_env+=("ROS_DOMAIN_ID=${ROS_DOMAIN_ID}")
fi

arm_camera_cmd=(env "${camera_env[@]}" bash "$ARM_CAMERA_WS/run_neweyes.sh")
if [[ "$ARM_CAMERA_HEADLESS" == "true" ]]; then
  arm_camera_cmd+=(--headless)
fi
arm_camera_window_cmd="$(make_window_cmd "$ARM_CAMERA_WS" "${arm_camera_cmd[@]}")"

head_camera_cmd=(env "${camera_env[@]}" bash "$HEAD_CAMERA_WS/run_neweyes.sh")
head_camera_window_cmd="$(make_window_cmd "$HEAD_CAMERA_WS" "${head_camera_cmd[@]}")"

tmux new-session -d -s "$SESSION_NAME" -n arm_nav "$arm_window_cmd"
tmux setw -t "$SESSION_NAME:arm_nav" remain-on-exit on
tmux select-pane -t "$SESSION_NAME:arm_nav.0" -T arm_nav

if [[ "$ARM_CAMERA_ENABLED" == "true" ]]; then
  tmux new-window -t "$SESSION_NAME" -n arm_camera "$arm_camera_window_cmd"
  tmux setw -t "$SESSION_NAME:arm_camera" remain-on-exit on
  tmux select-pane -t "$SESSION_NAME:arm_camera.0" -T arm_camera
fi

if [[ "$HEAD_CAMERA_ENABLED" == "true" ]]; then
  tmux new-window -t "$SESSION_NAME" -n head_camera "$head_camera_window_cmd"
  tmux setw -t "$SESSION_NAME:head_camera" remain-on-exit on
  tmux select-pane -t "$SESSION_NAME:head_camera.0" -T head_camera
fi

tmux select-window -t "$SESSION_NAME:arm_nav"

echo "[run_nav_full_tmux] created session '$SESSION_NAME'"
echo "[run_nav_full_tmux]   arm_nav     -> $WS_DIR/run_arm_nav.sh ${ARM_ARGS[*]:-}"
if [[ "$ARM_CAMERA_ENABLED" == "true" ]]; then
  arm_camera_desc="$ARM_CAMERA_WS/run_neweyes.sh"
  if [[ "$ARM_CAMERA_HEADLESS" == "true" ]]; then
    arm_camera_desc+=" --headless"
  fi
  echo "[run_nav_full_tmux]   arm_camera  -> $arm_camera_desc"
fi
if [[ "$HEAD_CAMERA_ENABLED" == "true" ]]; then
  echo "[run_nav_full_tmux]   head_camera -> $HEAD_CAMERA_WS/run_neweyes.sh"
fi
if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
  echo "[run_nav_full_tmux]   ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

if [[ "$ATTACH" == "true" ]]; then
  exec tmux attach-session -t "$SESSION_NAME"
fi

echo "[run_nav_full_tmux] attach with: tmux attach-session -t $SESSION_NAME"
