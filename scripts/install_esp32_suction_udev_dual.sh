#!/usr/bin/env bash
# install_esp32_suction_udev_dual.sh
#
# 一键为两块 ESP32 吸盘控制板安装 udev 规则：
#   /dev/esp32_suction_c3   — 机械臂末端吸盘（现有）
#   /dev/esp32_dog_suction  — 狗背吸盘（新增）
#
# 用法：
#   sudo ./scripts/install_esp32_suction_udev_dual.sh
#
# 也可以直接指定设备路径跳过交互：
#   sudo ./scripts/install_esp32_suction_udev_dual.sh --arm /dev/ttyACM0 --dog /dev/ttyACM1

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_SCRIPT="${SCRIPT_DIR}/install_esp32_suction_udev.sh"

ARM_ALIAS="esp32_suction_c3"
DOG_ALIAS="esp32_dog_suction"
ARM_DEVICE=""
DOG_DEVICE=""

usage() {
  cat <<EOF
Usage:
  sudo ./scripts/install_esp32_suction_udev_dual.sh [OPTIONS]

Options:
  --arm <device>   Skip detection, use this path for arm suction ESP32
  --dog <device>   Skip detection, use this path for dog suction ESP32
  -h, --help       Show this help

Examples:
  sudo ./scripts/install_esp32_suction_udev_dual.sh
  sudo ./scripts/install_esp32_suction_udev_dual.sh --arm /dev/ttyACM0 --dog /dev/ttyACM1
EOF
}

wait_for_new_device() {
  # Returns the device path that appeared after the prompt.
  # Polls /dev/ttyACM* and /dev/ttyUSB* for a new entry.
  local before_file after_file new_dev
  before_file="$(mktemp)"
  find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) | sort > "${before_file}"

  echo ""
  echo ">>> $1"
  echo "    Press ENTER after plugging in the device..."
  read -r

  # Wait up to 5 seconds for the device to appear
  local i=0
  while [[ $i -lt 25 ]]; do
    after_file="$(mktemp)"
    find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) | sort > "${after_file}"
    new_dev="$(comm -13 "${before_file}" "${after_file}" | head -n1)"
    rm -f "${after_file}"
    if [[ -n "${new_dev}" ]]; then
      rm -f "${before_file}"
      echo "    Detected new device: ${new_dev}"
      echo "${new_dev}"
      return 0
    fi
    sleep 0.2
    (( i++ )) || true
  done

  rm -f "${before_file}"
  echo "No new serial device detected within 5 seconds." >&2
  exit 1
}

# ── Parse args ────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --arm)   ARM_DEVICE="$2"; shift 2 ;;
    --dog)   DOG_DEVICE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run this script with sudo." >&2
  exit 1
fi

if [[ ! -x "${INSTALL_SCRIPT}" ]]; then
  echo "Cannot find or execute: ${INSTALL_SCRIPT}" >&2
  exit 1
fi

# ── Step 1: ARM suction ESP32 ─────────────────────────────────────────────────
if [[ -z "${ARM_DEVICE}" ]]; then
  echo "=== Step 1/2: ARM suction ESP32 (will be /dev/${ARM_ALIAS}) ==="
  echo "    Make sure ONLY the arm suction ESP32 is plugged in."
  echo "    (Unplug the dog suction ESP32 if it is connected.)"
  ARM_DEVICE="$(wait_for_new_device "Plug in the ARM suction ESP32 now")"
fi

echo ""
echo "Installing udev rule for ARM suction: ${ARM_DEVICE} → /dev/${ARM_ALIAS}"
bash "${INSTALL_SCRIPT}" "${ARM_ALIAS}" "${ARM_DEVICE}"

# ── Step 2: DOG suction ESP32 ─────────────────────────────────────────────────
if [[ -z "${DOG_DEVICE}" ]]; then
  echo ""
  echo "=== Step 2/2: DOG suction ESP32 (will be /dev/${DOG_ALIAS}) ==="
  DOG_DEVICE="$(wait_for_new_device "Now plug in the DOG suction ESP32")"
fi

echo ""
echo "Installing udev rule for DOG suction: ${DOG_DEVICE} → /dev/${DOG_ALIAS}"
bash "${INSTALL_SCRIPT}" "${DOG_ALIAS}" "${DOG_DEVICE}"

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "============================================"
echo "Done. Both udev rules installed:"
echo ""
printf "  %-30s → /dev/%s\n" "${ARM_DEVICE}" "${ARM_ALIAS}"
printf "  %-30s → /dev/%s\n" "${DOG_DEVICE}" "${DOG_ALIAS}"
echo ""
echo "Symlink status:"
for alias in "${ARM_ALIAS}" "${DOG_ALIAS}"; do
  if [[ -e "/dev/${alias}" ]]; then
    ls -l "/dev/${alias}"
  else
    echo "  /dev/${alias} — will appear after replug if not visible"
  fi
done
echo ""
echo "Rule files:"
ls -l "/etc/udev/rules.d/99-${ARM_ALIAS}.rules" \
       "/etc/udev/rules.d/99-${DOG_ALIAS}.rules" 2>/dev/null || true
echo "============================================"
