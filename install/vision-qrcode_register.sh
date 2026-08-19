#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-vision-qrcode.service"
LOG_DIR="/data/logs/vision-qrcode"
LOG_FILE="$LOG_DIR/vision-qrcode.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo chown user:user "$LOG_DIR" || true
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"
sudo chown user:user "$LOG_FILE" || true

echo "🛠  Creating systemd service robot-vision-qrcode"

sudo tee "$SERVICE_PATH" > /dev/null <<'EOF'
[Unit]
Description=Robot Vision QR Code for Robotour (TCP 9201)
Wants=network-online.target
After=network-online.target

ConditionPathExists=/opt/projects/robotour/vision-qrcode/main.py

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/vision-qrcode

# Před spuštěním ukonči libovolný proces, který drží port 9201
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9201/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1

ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/vision-qrcode/main.py

StandardOutput=append:/data/logs/vision-qrcode/vision-qrcode.log
StandardError=append:/data/logs/vision-qrcode/vision-qrcode.log

Restart=always
RestartSec=3
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-vision-qrcode.service

echo "✅ Service robot-vision-qrcode is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
