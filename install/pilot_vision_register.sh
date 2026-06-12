#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-pilot-vision.service"
LOG_DIR="/data/logs/pilot_vision"
LOG_FILE="$LOG_DIR/pilot_vision.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠  Creating systemd service robot-pilot-vision"

sudo tee "$SERVICE_PATH" > /dev/null <<'EOF'
[Unit]
Description=Robot Pilot Vision for Robotour
Wants=network-online.target
After=network-online.target

ConditionPathExists=/opt/projects/robotour/pilot_vision/main.py

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/pilot_vision

# před spuštěním ukonči libovolný proces, který drží port 9102
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9102/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1

ExecStart=/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/pilot_vision/main.py

StandardOutput=append:/data/logs/pilot_vision/pilot_vision.log
StandardError=append:/data/logs/pilot_vision/pilot_vision.log

Restart=always
RestartSec=3
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-pilot-vision.service

echo "✅ Service robot-pilot-vision is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
