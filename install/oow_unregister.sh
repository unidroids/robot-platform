#!/bin/bash
set -euo pipefail
SERVICE_FILE="/etc/systemd/system/robot-oow.service"

if systemctl is-enabled --quiet robot-oow.service; then
  sudo systemctl disable --now robot-oow.service || true
fi

sudo rm -f "$SERVICE_FILE"
sudo systemctl daemon-reload

echo "✅ robot-oow odstraněn"
