#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-gps.service"
LOG_DIR="/robot/data/logs/gps"
LOG_FILE="$LOG_DIR/gps.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠  Creating systemd service robot-gps"

sudo tee "$SERVICE_PATH" > /dev/null <<'EOF'
[Unit]
Description=Robot GPS server for Robotour (UM980)
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/gps

# před spuštěním ukonči libovolný proces, který drží port 9004
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9004/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1
ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/gps/main.py

StandardOutput=append:/robot/data/logs/gps/gps.log
StandardError=append:/robot/data/logs/gps/gps.log

Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-gps.service

echo "✅ Service robot-gps is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
