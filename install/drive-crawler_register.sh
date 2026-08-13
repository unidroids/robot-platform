#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-drive-crawler.service"
LOG_DIR="/robot/data/logs/drive-crawler"
LOG_FILE="$LOG_DIR/drive-crawler.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠  Creating systemd service robot-drive-crawler"

sudo tee "$SERVICE_PATH" > /dev/null <<EOF
[Unit]
Description=Robot Drive Crawler server for Robotour
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/drive-crawler

# před spuštěním ukonči libovolný proces, který drží port 9003
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9003/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1
ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/drive-crawler/main.py

StandardOutput=append:$LOG_FILE
StandardError=append:$LOG_FILE

Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-drive-crawler.service

echo "✅ Service robot-drive-crawler is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
