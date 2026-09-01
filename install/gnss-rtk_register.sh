#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-gnss-rtk.service"
LOG_DIR="/robot/data/logs/gnss-rtk"
LOG_FILE="$LOG_DIR/gnss-rtk.log"
FULL_LOG_FILE="$LOG_DIR/gnss-rtk_full.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"
sudo touch "$FULL_LOG_FILE"
sudo chmod 664 "$FULL_LOG_FILE"
sudo chown -R user:user "$LOG_DIR"

echo "🛠  Creating systemd service robot-gnss-rtk"

sudo tee "$SERVICE_PATH" > /dev/null <<'EOF'
[Unit]
Description=Robot GNSS-RTK service for Robotour
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/gnss-rtk

# před spuštěním ukonči libovolný proces, který drží port 9015
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9015/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1
ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/gnss-rtk/main.py

StandardOutput=append:/robot/data/logs/gnss-rtk/gnss-rtk.log
StandardError=append:/robot/data/logs/gnss-rtk/gnss-rtk.log

Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-gnss-rtk.service

echo "✅ Service robot-gnss-rtk is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
