#!/bin/bash
set -euo pipefail

SERVICE_FILE="/etc/systemd/system/robot-vision-qrcode.service"

if systemctl is-enabled --quiet robot-vision-qrcode.service 2>/dev/null; then
  sudo systemctl disable --now robot-vision-qrcode.service || true
fi

sudo rm -f "$SERVICE_FILE"
sudo systemctl daemon-reload

echo "✅ robot-vision-qrcode odstraněn"
