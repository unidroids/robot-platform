#!/bin/bash
set -euo pipefail
SERVICE_FILE="/etc/systemd/system/robot-oow-bridge.service"

if systemctl is-enabled --quiet robot-oow-bridge.service; then
  sudo systemctl disable --now robot-oow-bridge.service || true
fi

sudo rm -f "$SERVICE_FILE"
sudo systemctl daemon-reload

echo "✅ robot-oow-bridge odstraněn"
