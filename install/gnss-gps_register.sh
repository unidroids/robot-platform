#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-gnss-gps.service"
LOG_DIR="/robot/data/logs/gnss-gps"
LOG_FILE="$LOG_DIR/gnss-gps.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠  Creating systemd service robot-gnss-gps"

sudo tee "$SERVICE_PATH" > /dev/null <<'EOF'
[Unit]
Description=Robot GNSS-GPS server for Robotour (UM980)
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/gnss-gps

# před spuštěním ukonči libovolný proces, který drží port 9004
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9004/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1
ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/gnss-gps/main.py

StandardOutput=append:/robot/data/logs/gnss-gps/gnss-gps.log
StandardError=append:/robot/data/logs/gnss-gps/gnss-gps.log

Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-gnss-gps.service

echo "✅ Service robot-gnss-gps is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
