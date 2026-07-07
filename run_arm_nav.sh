#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RUN_ARM_SHUTDOWN_SUCTION_OFF="${RUN_ARM_SHUTDOWN_SUCTION_OFF:-true}"
exec bash "$SCRIPT_DIR/run_arm.sh" --nav --debug-tool "$@"
