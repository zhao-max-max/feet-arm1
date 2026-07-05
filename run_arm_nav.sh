#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_IN_XTERM="${TASK_IN_XTERM:-false}"
export TASK_IN_XTERM
exec bash "$SCRIPT_DIR/run_arm.sh" --nav --debug-tool "$@"
