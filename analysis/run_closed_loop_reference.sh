#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${1:-/dev/cu.usbserial-A10N20X1}"
SECONDS="${2:-30}"
BAUD="${3:-115200}"

echo "Uploading closed-loop firmware to ${PORT} ..."
(
  cd "${ROOT_DIR}/firmware"
  pio run -e nano_new -t upload --upload-port "${PORT}"
)

echo
echo "Place the ball at the physical centre and make sure the beam is ready."
echo "Press Enter to start capture. Opening the serial port may reset the Nano."
read -r

(
  cd "${ROOT_DIR}"
  ./.venv/bin/python analysis/capture_serial.py --port "${PORT}" --baud "${BAUD}" --seconds "${SECONDS}"
)
