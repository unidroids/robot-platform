#!/bin/bash
set -euo pipefail
SERVICE_FILE="/etc/systemd/system/robot-pilot-manual.service"

if systemctl is-enabled --quiet robot-pilot-manual.service; then
  sudo systemctl disable --now robot-pilot-manual.service || true
fi

sudo rm -f "$SERVICE_FILE"
sudo systemctl daemon-reload

echo "✅ robot-pilot-manual odstraněn"
