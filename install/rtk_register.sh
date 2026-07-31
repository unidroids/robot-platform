#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-rtk.service"
LOG_DIR="/robot/data/logs/rtk"
LOG_FILE="$LOG_DIR/rtk.log"
FULL_LOG_FILE="$LOG_DIR/rtk_full.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"
sudo touch "$FULL_LOG_FILE"
sudo chmod 664 "$FULL_LOG_FILE"
sudo chown -R user:user "$LOG_DIR"

echo "🛠  Creating systemd service robot-rtk"

sudo tee "$SERVICE_PATH" > /dev/null <<'EOF'
[Unit]
Description=Robot RTK service for Robotour
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/rtk

# před spuštěním ukonči libovolný proces, který drží port 9015
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9015/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1
ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/rtk/main.py

StandardOutput=append:/robot/data/logs/rtk/rtk.log
StandardError=append:/robot/data/logs/rtk/rtk.log

Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-rtk.service

echo "✅ Service robot-rtk is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
